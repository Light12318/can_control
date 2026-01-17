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

// 全局实例指针，用于将 C 回调转发到类成员
static class OmniWheelController *g_omni_instance = nullptr;

class OmniWheelController : public rclcpp::Node
{
private:
    // 运动学参数
    const double wheel_radius = 0.075;  // 轮子半径15cm，直径15cm
    const double wheel_distance = 0.23; // 轮子到中心距离23cm
    
    // 运动学矩阵（麦克纳姆轮）
    const double motion_matrix[4][3] = {
        {1.0, -1.0, -wheel_distance},   // 轮子1：左上45度
        {-1.0, -1.0, -wheel_distance},  // 轮子2：右上45度
        {-1.0, 1.0, -wheel_distance},   // 轮子3：右下45度
        {1.0, 1.0, -wheel_distance}     // 轮子4：左下45度
    };
    
    // CAN相关：直接作为成员对象（不可拷贝/不可赋值）
    usb_can_v2 can_device;
    std::mutex can_mutex;
    
    // 订阅器和发布器
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // 控制变量
    std::array<double, 4> target_wheel_speeds{0, 0, 0, 0};  // 目标轮速 (rad/s)
    std::array<double, 4> actual_wheel_speeds{0, 0, 0, 0};  // 实际轮速 (rad/s)
    std::array<double, 4> wheel_rpm{0, 0, 0, 0};            // 实际转速 (rpm)
    
    // PID控制器参数
    struct PIDController {
        double kp = 0.5;    // 比例系数
        double ki = 0.01;   // 积分系数
        double kd = 0.05;   // 微分系数
        double integral = 0;
        double prev_error = 0;
        double output_limit = 3000;  // 输出限幅(rpm)
    } pid_controllers[4];
    
    // IMU数据
    geometry_msgs::msg::Twist current_velocity;
    std::mutex imu_mutex;
    
    // 减速比
    const double gear_ratio = 19.0;

public:
    OmniWheelController() : Node("can_control"), can_device()
    {
        // 设置全局实例指针（供静态回调转发）
        g_omni_instance = this;

        // 初始化并注册 CAN 回调（listen 需要函数指针）
        try {
            can_device.listen(0x201, &OmniWheelController::canRxStatic);
            can_device.listen(0x202, &OmniWheelController::canRxStatic);
            can_device.listen(0x203, &OmniWheelController::canRxStatic);
            can_device.listen(0x204, &OmniWheelController::canRxStatic);
            RCLCPP_INFO(this->get_logger(), "CAN device listeners registered");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to register CAN listeners: %s", e.what());
        }
        
        // 创建订阅器
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OmniWheelController::cmdVelCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&OmniWheelController::imuCallback, this, std::placeholders::_1));
        
        // 创建发布器
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

    // 静态回调，符合 usb_can_v2::listen 的函数指针签名
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
    
    // CAN接收回调（成员）
    void canCallback(uint16_t id, uint8_t* data, int length)
    {
        (void)length;
        std::lock_guard<std::mutex> lock(can_mutex);
        
        // 提取轮子编号 (0x201-0x204对应轮子1-4)
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
    
    // 发布轮速信息
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
        
        // 积分项
        pid.integral += error * dt;
        
        // 微分项
        double derivative = (error - pid.prev_error) / dt;
        
        // 计算输出
        double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
        
        // 限幅
        output = std::max(-pid.output_limit, std::min(pid.output_limit, output));
        
        // 更新上一次误差
        pid.prev_error = error;
        
        return output;
    }
    
    // 发送CAN控制指令
    void sendControlCommands()
    {
        std::lock_guard<std::mutex> lock(can_mutex);
        
        unsigned char data[8] = {0};  // 8个字节对应4个轮子
        
        for (int i = 0; i < 4; i++) {
            // 计算目标转速 (rad/s -> rpm)
            double target_rpm = target_wheel_speeds[i] * (60.0 / (2.0 * M_PI)) * gear_ratio;
            
            // PID控制
            double dt = 0.01;  // 10ms控制周期
            double control_rpm = pidControl(target_rpm, wheel_rpm[i], pid_controllers[i], dt);
            
            // 转换为16位整数
            int16_t rpm_int = static_cast<int16_t>(std::round(control_rpm));
            
            // 填充数据 (每个轮子2个字节，高位在前)
            data[i * 2] = static_cast<unsigned char>((rpm_int >> 8) & 0xFF);     // 高8位
            data[i * 2 + 1] = static_cast<unsigned char>(rpm_int & 0xFF);        // 低8位
        }
        
        // 发送CAN报文 (ID: 0x200)
        try {
            can_device.transmit(0x200, data, 8);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to send CAN command: %s", e.what());
        }
    }
    
    // 控制主循环
    void controlLoop()
    {
        // 发布当前轮速
        publishWheelSpeeds();
        
        // 发送控制指令
        sendControlCommands();
    }
    
    // cmd_vel回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 运动学解算
        calculateWheelSpeeds(msg->linear.x, msg->linear.y, msg->angular.z);
        
        // 更新当前速度
        {
            std::lock_guard<std::mutex> lock(imu_mutex);
            current_velocity.linear.x = msg->linear.x;
            current_velocity.linear.y = msg->linear.y;
            current_velocity.angular.z = msg->angular.z;
        }
    }
    
    // IMU回调
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        (void)msg;
        // 暂时不使用 IMU 数据
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