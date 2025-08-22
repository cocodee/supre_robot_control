// include/gripper_bus.h
#ifndef GRIPPER_BUS_H
#define GRIPPER_BUS_H

#include <modbus/modbus.h>
#include <string>

class GripperBus {
public:
    /**
     * @brief 构造函数
     * @param device 串口设备名, e.g., "/dev/ttyUSB0" on Linux, "COM3" on Windows
     * @param baud 波特率, 默认为 115200
     * @param parity 校验位 'N', 'E', 'O'
     * @param data_bit 数据位 7 or 8
     * @param stop_bit 停止位 1 or 2
     */
    GripperBus(const std::string& device, int baud = 115200, char parity = 'N', int data_bit = 8, int stop_bit = 1);

    ~GripperBus();

    // 禁用拷贝构造和赋值
    GripperBus(const GripperBus&) = delete;
    GripperBus& operator=(const GripperBus&) = delete;

    /**
     * @brief 连接到总线
     * @return true 连接成功, false 连接失败
     */
    bool connect();

    /**
     * @brief 断开总线连接
     */
    void disconnect();

    /**
     * @brief 检查总线是否连接
     * @return true 已连接, false 未连接
     */
    bool isConnected() const;

    /**
     * @brief 获取 libmodbus 上下文指针
     * @return modbus_t* 指针
     */
    modbus_t* getModbusContext();

private:
    modbus_t* ctx_;
    std::string device_;
    int baud_;
    char parity_;
    int data_bit_;
    int stop_bit_;
    bool is_connected_;
};

#endif // GRIPPER_BUS_H