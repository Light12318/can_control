#include "can_control/usb_can_v2.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>
#include <vector>
#include <array>

static class OmniWheelController *g_omni_instance = nullptr;

class OmniWheelController : public rclcpp::Node
{
private:
    const double wheel_radius = 0.075;  // d=15cm
    const double wheel_distance = 0.23; // r=23cm
    
    const double motion_matrix[4][3] = {
        {-1.0, 1.0, wheel_distance},   // 轮子1：左上45度
        {-1.0, -1.0, wheel_distance},  // 轮子2：右上45度
        {1.0, -1.0, wheel_distance},   // 轮子3：右下45度
        {1.0, 1.0, wheel_distance}     // 轮子4：左下45度
    };
    
    usb_can_v2 can_device;
    std::mutex can_mutex;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_pub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::array<double, 4> target_wheel_speeds{0, 0, 0, 0};  // 目标轮速 (rad/s)
    std::array<double, 4> actual_wheel_speeds{0, 0, 0, 0};  // 实际轮速 (rad/s)
    std::array<double, 4> wheel_rpm{0, 0, 0, 0};            // 实际转速 (rpm)
    
    struct PIDController {
        double kp = 0.5;   
        double ki = 0.01;   
        double kd = 0.05;   
        double integral = 0;
        double prev_error = 0;
        double output_limit = 3000;  
    } pid_controllers[4];
    
    geometry_msgs::msg::Twist current_velocity;
    std::mutex imu_mutex;
    
    const double gear_ratio = 19.0;

    // 功率模型和映射参数
    const double CT_coeff = 1.996e-6;     // CT (力矩电流系数)  (units consistent with RPM)
    const double k1 = 0.0;                // ω^2 项系数（可调）
    const double k2 = 0.0;                // I^2 项系数（可调）
    const double K_rpm_to_count = 10.0;   // 初始从 rpm -> count 的映射（可调，作为初值）
    const double count_to_amp = 20.0 / 16384.0;
    const double amp_to_count = 16384.0 / 20.0;
    const double motor_voltage = 24.0;    // V
    const double total_power_limit_w = 60.0; // 总功率上限（W）

public:
    OmniWheelController() : Node("can_control"), can_device()
    {
        g_omni_instance = this;

        // 检查 CAN 设备初始化状态，初始化失败则记录并抛出异常以终止节点
        if (!can_device.ok()) {
            RCLCPP_FATAL(this->get_logger(), "USB CAN init failed, code=%d", can_device.error_code());
            throw std::runtime_error("usb_can_v2 initialization failed");
        }

        try {
             can_device.listen(0x201, &OmniWheelController::canRxStatic);
             can_device.listen(0x202, &OmniWheelController::canRxStatic);
             can_device.listen(0x203, &OmniWheelController::canRxStatic);
             can_device.listen(0x204, &OmniWheelController::canRxStatic);
             RCLCPP_INFO(this->get_logger(), "CAN device listeners registered");
         } catch (const std::exception& e) {
             RCLCPP_ERROR(this->get_logger(), "Failed to register CAN listeners: %s", e.what());
         }
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OmniWheelController::cmdVelCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&OmniWheelController::imuCallback, this, std::placeholders::_1));

        wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "wheel_speeds", 10);
        
        // 创建控制定时器 (100Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&OmniWheelController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Omni Wheel Controller initialized");
    }

    ~OmniWheelController() override
    {
        g_omni_instance = nullptr;
    }

    static void canRxStatic(uint16_t id, uint8_t* data, int length)
    {
        if (g_omni_instance) {
            g_omni_instance->canCallback(id, data, length);
        }
    }
    
private:
    // 运动学解算：从线速度和角速度计算轮子速度
    void calculateWheelSpeeds(double vx, double vy, double omega)
    {
        for (int i = 0; i < 4; i++) {
            // 轮子线速度 = M * [vx, vy, omega]^T
            double wheel_linear_speed = 
                motion_matrix[i][0] * vx + 
                motion_matrix[i][1] * vy + 
                motion_matrix[i][2] * omega;
            
            // 转换为轮子角速度 (rad/s)
            target_wheel_speeds[i] = wheel_linear_speed / wheel_radius;
        }
    }
    
    void canCallback(uint16_t id, uint8_t* data, int length)
    {
        (void)length;
        std::lock_guard<std::mutex> lock(can_mutex);
        
        int wheel_index = static_cast<int>(id) - 0x201;
        if (wheel_index >= 0 && wheel_index < 4) {
            // 合并data[2]和data[3]得到16位转速值
            int16_t raw_rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
            
            // 存储原始rpm
            wheel_rpm[wheel_index] = static_cast<double>(raw_rpm);
            
            // 转换为轮子角速度 (rad/s)
            // rpm -> rad/s: (rpm * 2π / 60) / 减速比
            actual_wheel_speeds[wheel_index] = 
                (wheel_rpm[wheel_index] * 2.0 * M_PI / 60.0) / gear_ratio;
        }
    }
    
    void publishWheelSpeeds()
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        for (int i = 0; i < 4; i++) {
            msg.data.push_back(static_cast<float>(actual_wheel_speeds[i]));
            msg.data.push_back(static_cast<float>(wheel_rpm[i]));
        }
        wheel_speed_pub_->publish(msg);

        // 打印目标/实际轮速（rad/s）和电机 rpm
        RCLCPP_INFO(this->get_logger(),
            "Wheel target (rad/s): [%.2f, %.2f, %.2f, %.2f]  actual (rad/s): [%.2f, %.2f, %.2f, %.2f]  motor rpm: [%.1f, %.1f, %.1f, %.1f]",
            target_wheel_speeds[0], target_wheel_speeds[1], target_wheel_speeds[2], target_wheel_speeds[3],
            actual_wheel_speeds[0], actual_wheel_speeds[1], actual_wheel_speeds[2], actual_wheel_speeds[3],
            wheel_rpm[0], wheel_rpm[1], wheel_rpm[2], wheel_rpm[3]
        );
    }
    
    // PID控制函数
    double pidControl(double target, double actual, PIDController& pid, double dt)
    {
        double error = target - actual;
        pid.integral += error * dt;
        double derivative = (error - pid.prev_error) / dt;
        double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
        output = std::max(-pid.output_limit, std::min(pid.output_limit, output));
        pid.prev_error = error;
        
        return output;
    }
    
    // 发送CAN控制指令（使用功率再分配 + 模型反算 Icmd）
    void sendControlCommands()
    {
        std::lock_guard<std::mutex> lock(can_mutex);
        unsigned char data[8] = {0};

        // Step1: 计算 PID 输出并得到初始 Icmd & Pcmd（signed）
        std::array<double,4> initial_counts{};
        std::array<double,4> initial_amps{};
        std::array<double,4> Pcmd{};
        std::array<double,4> omega_rpm{}; // 以 RPM 为单位（使用电机转速）
        double sumPcmd = 0.0;

        for (int i = 0; i < 4; ++i) {
            // 目标转速 (rad/s -> motor rpm)
            double target_rpm = target_wheel_speeds[i] * (60.0 / (2.0 * M_PI)) * gear_ratio;

            double dt = 0.01; // 10 ms
            double control_rpm = pidControl(target_rpm, wheel_rpm[i], pid_controllers[i], dt);

            // 初始映射：控制 rpm -> count -> amp
            double cnt = K_rpm_to_count * control_rpm;
            cnt = std::max(-16384.0, std::min(16384.0, cnt));
            double Icmd = cnt * count_to_amp; // A

            initial_counts[i] = cnt;
            initial_amps[i] = Icmd;

            // 电机转速 ω (RPM)，使用接收的 motor rpm（wheel_rpm 存储 raw motor rpm）
            double w_rpm = wheel_rpm[i];
            omega_rpm[i] = w_rpm;

            // 计算 Pcmd（模型）： P = CT * I * ω + k1*ω^2 + k2*I^2
            double P = CT_coeff * Icmd * w_rpm + k1 * w_rpm * w_rpm + k2 * Icmd * Icmd;
            Pcmd[i] = P;
            sumPcmd += P;
        }

        // Step2: 计算缩放系数 k = Pmax / ΣPcmd （按照给出公式）
        double scale_k = 1.0;
        if (std::abs(sumPcmd) > 1e-9) {
            double k = total_power_limit_w / sumPcmd;
            // 规则：若 k > 1 不缩放（即 scale = 1），否则 scale = k
            if (k > 1.0) scale_k = 1.0;
            else scale_k = k;
        } else {
            scale_k = 1.0;
        }

        // Step3: 对正功率按 scale_k 缩放，负功率不缩放；通过模型反解 Icmd'
        for (int i = 0; i < 4; ++i) {
            double P_des = Pcmd[i];
            if (Pcmd[i] > 0.0) {
                P_des = Pcmd[i] * scale_k; // 缩放正功率
            } else {
                P_des = Pcmd[i]; // 负功率保持原样
            }

            double w = omega_rpm[i]; // RPM
            double chosen_I = initial_amps[i]; // fallback to initial

            // Solve k2*I^2 + (CT*ω)*I + (k1*ω^2 - P_des) = 0
            if (std::abs(k2) < 1e-12) {
                // 退化为线性： CT*ω*I = P_des - k1*ω^2
                double denom = CT_coeff * w;
                if (std::abs(denom) > 1e-12) {
                    chosen_I = (P_des - k1 * w * w) / denom;
                } else {
                    // ω == 0 (或极小),无法通过模型求解，保留初始
                    chosen_I = initial_amps[i];
                }
            } else {
                double a = k2;
                double b = CT_coeff * w;
                double c = k1 * w * w - P_des;
                double disc = b * b - 4.0 * a * c;
                if (disc >= 0.0) {
                    double sqrtD = std::sqrt(disc);
                    double I1 = (-b + sqrtD) / (2.0 * a);
                    double I2 = (-b - sqrtD) / (2.0 * a);
                    // 选择与初始 Icmd 极性一致的根，若初始为零则选择幅值靠近的根
                    if (std::abs(initial_amps[i]) < 1e-6) {
                        // 选择幅度更小的解
                        chosen_I = (std::abs(I1) < std::abs(I2)) ? I1 : I2;
                    } else {
                        if ((initial_amps[i] >= 0 && I1 >= 0) || (initial_amps[i] < 0 && I1 < 0)) {
                            chosen_I = I1;
                        } else if ((initial_amps[i] >= 0 && I2 >= 0) || (initial_amps[i] < 0 && I2 < 0)) {
                            chosen_I = I2;
                        } else {
                            // 若都不符合符号，选择与初始幅值更接近的解
                            chosen_I = (std::abs(I1 - initial_amps[i]) < std::abs(I2 - initial_amps[i])) ? I1 : I2;
                        }
                    }
                } else {
                    // 判别式小于0，退化为使用线性近似或初始
                    double denom = CT_coeff * w;
                    if (std::abs(denom) > 1e-12) {
                        chosen_I = (P_des - k1 * w * w) / denom;
                    } else {
                        chosen_I = initial_amps[i];
                    }
                }
            }

            // 将 chosen_I 转为 count 并限幅
            double out_cnt = chosen_I * amp_to_count;
            out_cnt = std::max(-16384.0, std::min(16384.0, out_cnt));
            int16_t final_cnt = static_cast<int16_t>(std::lround(out_cnt));

            data[i * 2]     = static_cast<unsigned char>((final_cnt >> 8) & 0xFF);
            data[i * 2 + 1] = static_cast<unsigned char>(final_cnt & 0xFF);
        }

        try {
            can_device.transmit(0x200, data, 8);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send CAN command: %s", e.what());
        }
    }
    
    void controlLoop()
    {
        publishWheelSpeeds();
        sendControlCommands();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        calculateWheelSpeeds(msg->linear.x, msg->linear.y, msg->angular.z);
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            current_velocity.linear.x = msg->linear.x;
            current_velocity.linear.y = msg->linear.y;
            current_velocity.angular.z = msg->angular.z;
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        (void)msg;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniWheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}