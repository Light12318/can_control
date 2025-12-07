#include "can_control/usb_can_v2.hpp" 

int usb_can_v2::instance_cnt = 0;
std::mutex usb_can_v2::single_lock;

std::mutex usb_can_v2::tx_lock;

std::atomic<int> usb_can_v2::tx_cnt = 0;

usb_can_v2::usb_can_v2()
{   
    ctx = nullptr;
    usb_can_error = USB_CAN_OK;
    handle = nullptr;
    std::lock_guard<std::mutex> lock(single_lock);
    instance_cnt ++;
    assert(instance_cnt == 1 && "Only one usb_can instance is accept");

    // Initialize libusb API
    int ret = 0; 
    ret = libusb_init(&ctx);
    if (ret != LIBUSB_SUCCESS)
    {
        usb_can_error = USB_CAN_LIBUSB_FAIL;
        std::cout << "Failed to initialize libusb. Error code:" << ret << std::endl;
        return;
    }

    if (ret != LIBUSB_SUCCESS)
    {
        usb_can_error = USB_CAN_LIBUSB_FAIL;
        std::cout << "Failed to initialize libusb. Error code:" << ret << std::endl;
        return;
    }

    // Open device
    handle = libusb_open_device_with_vid_pid(ctx, 0x1a86, 0x55d3);
    if (handle == NULL)
    {
        usb_can_error = USB_CAN_OPEN_FAIL;
        std::cout << "Failed to open device." << std::endl;
        return;
    }
    
    ret = libusb_detach_kernel_driver(handle, 0);
    if (ret != LIBUSB_SUCCESS)
    {
        usb_can_error = USB_CAN_OPEN_FAIL;
        std::cout << "Failed to detach kernel driver. Error code:" << ret << std::endl;
        return;
    }
    
    ret = libusb_claim_interface(handle, 0);
    if (ret != LIBUSB_SUCCESS)
    {
        usb_can_error = USB_CAN_OPEN_FAIL;
        std::cout << "Failed to claim interface 0. Error code:" << ret << std::endl;
        return;
    }
    ret = libusb_claim_interface(handle, 1);
    if (ret != LIBUSB_SUCCESS)
    {
        usb_can_error = USB_CAN_OPEN_FAIL;
        std::cout << "Failed to claim interface 1. Error code:" << ret << std::endl;
        return;
    }
    
    struct line_coding {
        uint32_t dwDTERate;
        uint8_t  bCharFormat;
        uint8_t  bParityType;
        uint8_t  bDataBits;
    };
    struct line_coding lc = {
        .dwDTERate = 4000000,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8
    };


    ret = libusb_control_transfer(
        handle,
        0x21,
        0x20,
        0x0000,
        0,
        (unsigned char*)&lc,
        sizeof(lc),
        1000
    );

    if (ret != sizeof(lc))
    {
        std::cout << "Failed to set parameter. Error code:" << ret << std::endl;
    }
    
    tx_transfer = libusb_alloc_transfer(0);
    rx_transfer = libusb_alloc_transfer(0);

    libusb_fill_bulk_transfer(rx_transfer, handle, 0x82, rx_buffer, sizeof(rx_buffer), rx_callback, this, 1000);
    libusb_submit_transfer(rx_transfer);

    thread_running_flag = true;
    event_thread = std::thread([this]{while (thread_running_flag)
    {
        int ret = libusb_handle_events(ctx);
        if (ret == LIBUSB_ERROR_INTERRUPTED) {
            break;
        }
        else if (ret == LIBUSB_TRANSFER_CANCELLED)
        {
            libusb_free_transfer(rx_transfer);
        }
        else if (ret != LIBUSB_SUCCESS)
        {
            std::cout << "Event handle fail. Error code:" << ret << std::endl;
        }
    }
    });

}

usb_can_v2::~usb_can_v2()
{
    int ret = LIBUSB_SUCCESS;
    if (handle != NULL)
    {
        while (tx_cnt);
        libusb_cancel_transfer(tx_transfer);
        ret = libusb_cancel_transfer(rx_transfer);
        if (ret != LIBUSB_SUCCESS)
        {
            std::cout << "Failed to cancel transfer. Error code:" << ret << std::endl;
        }
        
        thread_running_flag = false;
        libusb_interrupt_event_handler(ctx);
        event_thread.join();
        libusb_free_transfer(tx_transfer);

        libusb_release_interface(handle, 0);
        libusb_release_interface(handle, 1);
        ret = libusb_attach_kernel_driver(handle, 0);
        libusb_close(handle);
    }
    if (ret != LIBUSB_SUCCESS)
    {
        std::cout << "Failed to attach kernel driver. Error code:" << ret << std::endl;
    }
    libusb_exit(ctx);
}

int usb_can_v2::transmit(uint16_t id, uint8_t* data, int length)
{
    if(length > 8) return USB_CAN_INVALID_LENGTH;
    if(id > 0x7FF) return USB_CAN_INVALID_ID;
    uint8_t buffer[10] ={};
    buffer[0] = id >> 8;
    buffer[1] = id & 0xFF;
    memcpy(buffer + 2, data, length);
    int actual_length = 0;
    if(handle != NULL)
    {
        tx_lock.lock();
        tx_cnt++;
        libusb_fill_bulk_transfer(tx_transfer, handle, 0x02, buffer, length + 2, tx_callback, NULL, 0);
        libusb_submit_transfer(tx_transfer);
    }
    return actual_length;
}

int usb_can_v2::listen(uint16_t id, void (*callback)(uint16_t id, uint8_t* data, int length))
{    
    if(id > 0x7FF) return USB_CAN_INVALID_ID;
    if(callback == NULL) return USB_CAN_NULL_PTR;
    rx_callback_map.insert_or_assign(id, callback);
    return USB_CAN_OK;
}