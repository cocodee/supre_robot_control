// MisumiGripper.h
#ifndef MISUMI_GRIPPER_H
#define MISUMI_GRIPPER_H

#include <string>
#include <vector>
#include <cstdint>
#include <modbus/modbus.h>
// 前向声明，避免在头文件中包含 libmodbus 的头文件

// 定义寄存器地址，提高可读性
namespace GripperRegisters {
    // 控制寄存器 (Holding Registers)
    constexpr int INIT = 0x0FA0;
    constexpr int CONTROL_MODE = 0x0FA1;
    constexpr int TARGET_POS = 0x0FA2;
    constexpr int TARGET_SPEED = 0x0FA3;
    constexpr int TARGET_TORQUE = 0x0FA4;
    constexpr int TRIGGER_ACTION = 0x0FA5;
    // --- 新增：预设参数寄存器起始地址 ---
    constexpr int PRESET_POS_1 = 0x0FA6;
    constexpr int PRESET_SPEED_1 = 0x0FA7;
    constexpr int PRESET_TORQUE_1 = 0x0FA8;
    // ... 其他预设点可以通过偏移量计算

    // --- 新增：停止控制寄存器 ---
    constexpr int STOP_CONTROL = 0x0FBE; // 文档 6.2.10
    // 状态寄存器 (Input Registers)
    constexpr int INIT_STATUS = 0x1194;
    constexpr int FAULT_STATUS = 0x1195;
    constexpr int GRIP_STATUS = 0x1196;
    constexpr int CURRENT_POS = 0x1197;
    constexpr int CURRENT_SPEED = 0x1198;
    constexpr int CURRENT_TORQUE = 0x1199;
}

// 夹爪状态结构体，用于返回完整的状态信息
struct GripperStatus {
    bool is_enabled = false;
    uint16_t fault_code = 0;
    uint16_t grip_state = 0; // 0:默认, 1:运行中, 2:到位, 3:夹到, 4:掉落
    double position_mm = 0.0;
    int speed_percent = 0;
    int torque_percent = 0;
};


class MisumiGripper {
public:
    /**
     * @brief 构造函数
     * @param device 串口设备路径 (例如 "/dev/ttyUSB0" on Linux, "COM3" on Windows)
     * @param slave_id 夹爪的 Modbus 从站 ID (1-247)
     * @param baud_rate 波特率，默认为 115200
     * @param parity 校验位 ('N', 'E', 'O')，默认为 'N'
     * @param data_bit 数据位 (5-8)，默认为 8
     * @param stop_bit 停止位 (1-2)，默认为 1
     */
    MisumiGripper(const std::string& device, int slave_id, 
                  int baud_rate = 115200, char parity = 'N', int data_bit = 8, int stop_bit = 1);

    /**
     * @brief 析构函数，自动关闭连接并释放资源
     */
    ~MisumiGripper();

    // 禁止拷贝和赋值
    MisumiGripper(const MisumiGripper&) = delete;
    MisumiGripper& operator=(const MisumiGripper&) = delete;

    /**
     * @brief 连接到夹爪
     * @return true 如果连接成功, false 如果失败
     */
    bool connect();

    /**
     * @brief 断开与夹爪的连接
     */
    void disconnect();

    /**
     * @brief 检查是否已连接
     */
    bool isConnected() const;

    /**
     * @brief 获取最近一次操作的错误信息
     * @return 错误信息字符串
     */
    std::string getLastError() const;

    /**
     * @brief 使能夹爪 (执行搜索行程)
     * @return true 如果成功发送指令, false 如果失败
     */
    bool enable();

    /**
     * @brief 去使能夹爪
     * @return true 如果成功发送指令, false 如果失败
     */
    bool disable();

    /**
     * @brief 让夹爪运动到指定位置 (动态参数模式)
     * @param position_mm 目标位置 (单位: 毫米)
     * @param speed_percent 速度百分比 (1-100)
     * @param torque_percent 力矩百分比 (1-100)
     * @return true 如果成功发送指令, false 如果失败
     */
    bool moveTo(double position_mm, int speed_percent, int torque_percent);
    
    /**
     * @brief 夹爪闭合 (使用预设的全力全速关闭指令)
     * @return true 如果成功发送指令, false 如果失败
     */
    bool grip();

    /**
     * @brief 夹爪张开 (使用预设的全力全速打开指令)
     * @return true 如果成功发送指令, false 如果失败
     */
    bool open();
    
    /**
     * @brief 读取夹爪的完整状态
     * @param status 用于接收状态信息的结构体引用
     * @return true 如果读取成功, false 如果失败
     */
    bool readStatus(GripperStatus& status);

    /**
     * @brief 停止夹爪当前的运动。
     * @return true 如果成功发送指令, false 如果失败。
     */
    bool stop();

    /**
     * @brief 配置一个预设参数点。
     * @param preset_number 预设点编号 (1-8)。
     * @param position_mm 目标位置 (单位: 毫米)。
     * @param speed_percent 速度百分比 (1-100)。
     * @param torque_percent 力矩百分比 (1-100)。
     * @return true 如果配置成功, false 如果失败。
     */
    bool setPreset(int preset_number, double position_mm, int speed_percent, int torque_percent);

    /**
     * @brief 执行一个预设的动作。
     * @param preset_number 要执行的预设点编号 (1-8)。
     * @return true 如果成功发送指令, false 如果失败。
     */
    bool executePreset(int preset_number);
private:
    // 底层 Modbus 读写函数的封装
    bool writeRegister(int addr, uint16_t value);
    bool writeRegisters(int start_addr, const std::vector<uint16_t>& values);
    bool readRegisters(int start_addr, int num, uint16_t* dest);
    
    modbus_t* m_ctx = nullptr; // libmodbus 上下文指针
    std::string m_device;
    int m_slave_id;
    int m_baud_rate;
    char m_parity;
    int m_data_bit;
    int m_stop_bit;
    bool m_is_connected = false;
    std::string m_last_error;
};

#endif // MISUMI_GRIPPER_H