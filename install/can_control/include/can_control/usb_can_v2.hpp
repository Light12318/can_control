#ifndef __USB_CAN_V2_HPP__
#define __USB_CAN_V2_HPP__

#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstring>
#include <cassert>
#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <queue>

#define USB_CAN_OK 0
#define USB_CAN_LIBUSB_FAIL -1
#define USB_CAN_OPEN_FAIL -2
#define USB_CAN_INVALID_LENGTH -3
#define USB_CAN_INVALID_ID -4
#define USB_CAN_NULL_PTR -5



class usb_can_v2
{
private:
    libusb_context *ctx;
    char usb_can_error;
    libusb_device_handle *handle;
    libusb_transfer* tx_transfer;
    libusb_transfer* rx_transfer;
    std::thread event_thread;
    std::atomic<bool> thread_running_flag{false};
    
    static std::mutex single_lock;
    static int instance_cnt;

    static std::mutex tx_lock;
    static std::atomic<int> tx_cnt;
    static void tx_callback(libusb_transfer*){
        tx_cnt--;
        tx_lock.unlock();
    }
    

    std::unordered_map<uint16_t, void (*)(uint16_t, uint8_t*, int)> rx_callback_map;
    uint8_t rx_buffer[64];
    static void rx_callback(libusb_transfer* transfer){
        usb_can_v2* this_ptr = static_cast<usb_can_v2*>(transfer->user_data);
        
        static std::queue<uint8_t> rx_queue = {};
        for (int i = 0; i < transfer->actual_length; i++)
        {
            rx_queue.push(transfer->buffer[i]);
        }

        static int rx_decode_mode = 0;// 0: start; 1:pass head, decode id high 8bit; 2: decode id low 8bit
        static int head_cnt = 0;
        static int length = 0;
        static uint16_t id = 0;
        static uint8_t data[8] = {};
        static int data_idx = 0;
        while (rx_queue.empty() == false)
        {
            uint8_t byte = rx_queue.front();
            rx_queue.pop();
            if(rx_decode_mode == 0)
            {
                if(byte == 0xFF)
                {
                    head_cnt ++;
                }
                else if(head_cnt >= 9 && byte>= 0x0F)
                {
                    length = (byte >> 4);
                    if(length > 8)
                    {
                        length = 8;
                        std::cout << "Invalid length, resize as 8" << std::endl;
                    }
                    head_cnt = 0;
                    rx_decode_mode = 1;
                }
                else
                {
                    head_cnt = 0;
                }
            }
            else if(rx_decode_mode == 1)
            {
                if(byte <= 0x07)
                {
                    id = byte << 8;
                    rx_decode_mode = 2;
                }
                else
                {
                    rx_decode_mode = 0;
                }
            }
            else if(rx_decode_mode == 2)
            {
                id = id | byte;
                rx_decode_mode = 3;
            }
            else if(rx_decode_mode == 3)
            {
                data[data_idx] = byte;
                data_idx++;
                if (data_idx == length)
                {
                    auto callback = this_ptr->rx_callback_map.find(id);
                    if(callback != this_ptr->rx_callback_map.end()){
                        callback->second(id, data, length);
                    }
                    rx_decode_mode = 0;
                    data_idx = 0;
                    id = 0;
                    length = 0;
                }                
            }
        }
        
        libusb_submit_transfer(transfer);
    }

public:
    usb_can_v2();
    ~usb_can_v2();
    int transmit(uint16_t id, uint8_t* data, int length);
    int listen(uint16_t id, void (*callback)(uint16_t id, uint8_t* data, int length));
};

#endif
