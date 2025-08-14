#ifndef MISUMI_GRIPPER_BUS_HPP
#define MISUMI_GRIPPER_BUS_HPP

#include <string>
#include <vector>
#include <modbus/modbus.h>

// 这个类现在只管理物理总线连接
class MisumiGripperBus {
public:
    MisumiGripperBus(const std::string& device, int baud_rate, char parity = 'N', int data_bit = 8, int stop_bit = 1);
    ~MisumiGripperBus();

    bool connect();
    void disconnect();
    bool isConnected() const;
    std::string getLastError() const;
    
    // 允许朋友类 MisumiGripper 访问私有的 m_ctx
    friend class MisumiGripper;

private:
    // Helper methods that now need the slave_id
    bool writeRegister(int slave_id, int addr, uint16_t value);
    bool writeRegisters(int slave_id, int start_addr, const std::vector<uint16_t>& values);
    bool readRegisters(int slave_id, int start_addr, int num, uint16_t* dest);

    modbus_t* m_ctx;
    std::string m_device;
    int m_baud_rate;
    char m_parity;
    int m_data_bit;
    int m_stop_bit;
    
    bool m_is_connected = false;
    std::string m_last_error;
};

#endif // MISUMI_GRIPPER_BUS_HPP