#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QCoreApplication>
#include <QTime>
#include <QList>

#define  BRODCAST_ID  0xfe

struct set_arm_data{
   unsigned char id;
   float position_angle;
   int time_ms;
   bool torque_enable;
};

class RobotArm : public QObject
{
    Q_OBJECT
public:
    explicit RobotArm(QObject *parent = 0);
    ~RobotArm();

    bool openServoPort(QString port_name);
    //发送移动角度和移动时间给舵机
    //param1:舵机id，param2：舵机执行角度，param3：舵机执行时间，param4：舵机执行模式：false立即执行，true等待同步命令模式
    bool sendServoMoveAngleAndTime(unsigned char id, float angle_du, unsigned short time_ms, bool wait_flag = false);
    //开启或关闭舵机力矩
    bool sendServoLoadOrUnload(unsigned char id, bool load_flag);

    //等待模式时发送开时信号
    bool sendStartCmd(unsigned char id);
    //立即停止舵机运动
    bool sendStopCmd(unsigned char id);

    //读取舵机力矩是否处于开启状态,
    bool getServoLoadState(unsigned char id, bool& load_state, int timeout_ms);
    //读取舵机角度
    bool getServoPresentAngle(unsigned char id, float& angle, int timeout_ms);
    //读取舵机电源电压
    bool getServoPowerVol(unsigned char id, float& power_vol, int timeout_ms);
    //读取舵机错误指示LED设置值
    bool getServoErrCode(unsigned char id, unsigned char& err_code, int timeout_ms);

    void closePort();
    void delayMs(int ms);

private:
    bool openPort(QString port_name, int baud_rate);

    unsigned char getCheckSum(const QByteArray& data_qba, int start_idx);

    bool sendServoDataPacket(unsigned char id, unsigned char cmd, const QList<unsigned char>& param_list);


signals:

public slots:
    void readPortDataSlot();

private:
    QSerialPort* servo_port;
    bool rec_data_finish;
    QByteArray rec_data;
};

#endif // RobotArm_H
