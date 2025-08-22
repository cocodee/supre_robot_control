// include/gripper.h
#ifndef GRIPPER_H
#define GRIPPER_H

#include "GripperBus.hpp"
#include <cstdint>
#include <string>

// 根据文档 P15，gOBJ 位定义
enum class ObjectDetectionStatus : uint8_t {
    MOVING = 0,             // 0x0: 手指正向指定位置移动
    INNER_GRIP_DETECTED = 1,// 0x1: 内撑模式接触到物体
    OUTER_GRIP_DETECTED = 2,// 0x2: 外夹模式接触到物体
    NO_OBJECT_DETECTED = 3, // 0x3: 到达指定位置，未检测到物体
    UNKNOWN = 0xFF
};

// 根据文档 P15，gSTA 位定义
enum class ActivationStatus : uint8_t {
    RESETTING = 0,         // 0x0: 处于复位或巡检状态
    ACTIVATING = 1,        // 0x1: 正在激活
    RESERVED = 2,          // 0x2: 未使用
    ACTIVATION_COMPLETE = 3// 0x3: 激活完成
};

// 夹爪状态反馈结构体
struct GripperStatus {
    bool enabled;                     // gACT: 使能状态
    bool is_moving;                   // gGTO: 动作状态
    ActivationStatus activation_status; // gSTA: 激活状态
    ObjectDetectionStatus object_status; // gOBJ: 目标检测状态
    uint8_t position;                 // 当前位置 (0-255, 0=打开, 255=闭合)
    uint8_t speed;                    // 当前速度
    uint8_t force_current;            // 当前力/电流
    uint8_t bus_voltage;              // 母线电压
    uint8_t temperature;              // 环境温度
    
    void print() const;
};

class JodellGripper {
public:
    JodellGripper(GripperBus& bus, int slave_id);

    /**
     * @brief 使能夹爪 (激活)
     *        首次上电或重启后必须执行。夹爪会执行初始化动作。
     * @return true 操作成功, false 操作失败
     */
    bool enable();

    /**
     * @brief 禁用夹爪 (清除使能状态)
     * @return true 操作成功, false 操作失败
     */
    bool disable();

    /**
     * @brief 控制夹爪移动到指定位置
     *        这是有参模式的控制方法。
     * @param pos 目标位置 (0-255, 0=完全打开, 255=完全闭合)
     * @param speed 运行速度 (1-255)
     * @param force 夹持力 (1-255)
     * @return true 操作成功, false 操作失败
     */
    bool move(uint8_t pos, uint8_t speed, uint8_t force);

    /**
     * @brief 获取夹爪的当前全部状态
     * @param status 用于存储状态的结构体引用
     * @return true 读取成功, false 读取失败
     */
    bool getStatus(GripperStatus& status);

    /**
     * @brief 等待夹爪运动停止
     * @param timeout_ms 超时时间 (毫秒)
     * @return true 运动在超时前停止, false 超时
     */
    bool waitMotionComplete(int timeout_ms = 5000);

private:
    GripperBus& bus_;
    int slave_id_;

    // 寄存器地址定义
    static const uint16_t REG_CONTROL = 0x03E8;
    static const uint16_t REG_POSITION_SET = 0x03E9;
    static const uint16_t REG_SPEED_FORCE_SET = 0x03EA;
    
    static const uint16_t REG_STATUS = 0x07D0;
    static const uint16_t REG_POS_STATUS = 0x07D1;
    static const uint16_t REG_SPEED_FORCE_STATUS = 0x07D2;
    static const uint16_t REG_VOLTAGE_TEMP_STATUS = 0x07D3;
};

#endif // GRIPPER_H