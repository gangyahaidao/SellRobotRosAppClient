#include "robotarm.h"
#include <QDebug>

RobotArm::RobotArm(QObject *parent) : QObject(parent)
{
    servo_port = 0;
}

RobotArm::~RobotArm()
{
    closePort();
}

bool RobotArm::openServoPort(QString port_name)
{
    //bool open_flag = openPort("/dev/arm_servo", 115200);
    bool open_flag = openPort(port_name, 115200);
    if(open_flag){
        qDebug() << "open arm servo control port sucess!";
        connect(servo_port, SIGNAL(readyRead()), this, SLOT(readPortDataSlot()));
    }else{
        qDebug() << "open arm servo control port fail!";
    }

    return open_flag;
}

bool RobotArm::sendServoMoveAngleAndTime(unsigned char id, float angle_du, unsigned short time_ms, bool wait_flag)
{
    //角度:0-240度对应0-1000,时间最大为30000

    unsigned char cmd;
    if(wait_flag)
        cmd = 0x07;
    else
        cmd = 0x01;

    QList<unsigned char> param_list;
    unsigned short angle_val = angle_du / 0.24;
    unsigned char angle_low = angle_val;
    unsigned char angle_high = angle_val >> 8;
    param_list.push_back(angle_low);
    param_list.push_back(angle_high);
    unsigned char time_low = time_ms;
    unsigned char time_high = time_ms >> 8;
    param_list.push_back(time_low);
    param_list.push_back(time_high);

    bool flag = sendServoDataPacket(id, cmd, param_list);

    return flag;
}

bool RobotArm::sendServoLoadOrUnload(unsigned char id, bool load_flag)
{
    unsigned char cmd = 0x1f;

    QList<unsigned char> param_list;
    unsigned char load_param;
    if(load_flag)
        load_param = 1;
    else
        load_param = 0;
    param_list.push_back(load_param);
    bool flag = sendServoDataPacket(id, cmd, param_list);

    return flag;
}

bool RobotArm::sendStartCmd(unsigned char id)
{
    unsigned char cmd = 0x0b;

    QList<unsigned char> param_list;

    bool flag = sendServoDataPacket(id, cmd, param_list);

    return flag;
}

bool RobotArm::sendStopCmd(unsigned char id)
{
    unsigned char cmd = 0x0c;

    QList<unsigned char> param_list;

    bool flag = sendServoDataPacket(id, cmd, param_list);

    return flag;
}

bool RobotArm::getServoLoadState(unsigned char id, bool &load_state, int timeout_ms)
{
    bool flag = false;
    unsigned char read_cmd = 0x20;
    QList<unsigned char> param_list;
    bool send_flag = sendServoDataPacket(id, read_cmd, param_list);
    if(!send_flag){
        qDebug() << "send servo data packet fail";
        return false;
    }

    rec_data.clear();
    rec_data_finish = false;

    QTime t = QTime::currentTime().addMSecs(timeout_ms);
    while(!rec_data_finish){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if(QTime::currentTime() > t){
            break;
        }
    }

    if(rec_data_finish && rec_data.size() >= 4){
        flag = true;
        unsigned char id = rec_data.at(0);
        unsigned char data_len = rec_data.at(1);
        unsigned char cmd = rec_data.at(2);
        load_state = rec_data.at(3);
    }

    return flag;
}

bool RobotArm::getServoPresentAngle(unsigned char id, float &angle, int timeout_ms)
{
    bool flag = false;
    unsigned char read_cmd = 0x1c;
    QList<unsigned char> param_list;
    bool send_flag = sendServoDataPacket(id, read_cmd, param_list);
    if(!send_flag){
        qDebug() << "send servo data packet fail";
        return false;
    }
    rec_data.clear();
    rec_data_finish = false;

    QTime t = QTime::currentTime().addMSecs(timeout_ms);
    while(!rec_data_finish){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if(QTime::currentTime() > t){
            break;
        }
    }
    if(rec_data_finish && rec_data.size() >= 5){
        unsigned char id = rec_data.at(0);
        unsigned char data_len = rec_data.at(1);
        unsigned char cmd = rec_data.at(2);
        if(data_len == 5){
            unsigned char low_ch = rec_data.at(3);
            unsigned char high_ch = rec_data.at(4);
            unsigned short val = high_ch << 8;
            val |= low_ch;
            flag = true;
            angle = (signed short)val * 0.24;
            //qDebug("rec data, id:%x, cmd:%x, val:%x", id, cmd, val);
        }else{
            qDebug() << "rec data finish but data_len err != 5!";
        }
    }

    return flag;
}

bool RobotArm::getServoErrCode(unsigned char id, unsigned char &err_code, int timeout_ms)
{
    bool flag = false;
    unsigned char read_cmd = 0x24;
    QList<unsigned char> param_list;
    bool send_flag = sendServoDataPacket(id, read_cmd, param_list);
    return true;
    if(!send_flag){
        qDebug() << "send servo data packet fail";
        return false;
    }
    rec_data.clear();
    rec_data_finish = false;

    QTime t = QTime::currentTime().addMSecs(timeout_ms);
    while(!rec_data_finish){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if(QTime::currentTime() > t){
            break;
        }
    }
    if(rec_data_finish){
        unsigned char id = rec_data.at(0);
        unsigned char data_len = rec_data.at(1);
        unsigned char cmd = rec_data.at(2);
        if(data_len == 4){
            err_code = rec_data.at(3);
            //qDebug("rec data, id:%x, cmd:%x, err:%x", id, cmd, err_code);
        }
    }

    return flag;
}

void RobotArm::delayMs(int ms)
{
    QTime delay_time = QTime::currentTime().addMSecs(ms);
    while(QTime::currentTime() < delay_time)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
}

bool RobotArm::openPort(QString port_name, int baud_rate)
{
    servo_port = new QSerialPort();
    servo_port->setPortName(port_name);
    servo_port->setBaudRate(baud_rate);
    servo_port->setDataBits(QSerialPort::Data8);
    servo_port->setStopBits(QSerialPort::OneStop);
    servo_port->setParity(QSerialPort::NoParity);
    servo_port->setFlowControl(QSerialPort::NoFlowControl);

    bool open_flag = servo_port->open(QIODevice::ReadWrite);

    return open_flag;
}

void RobotArm::closePort()
{
    if(servo_port != NULL){
        servo_port->close();
    }
}

//start_idx > 0, 从start_idx开始算到最后一个
//start_dix < 0，从头开始算到len+start_idx
unsigned char RobotArm::getCheckSum(const QByteArray &data_qba, int start_idx)
{
    unsigned char chk_sum = 0;
    int end_idx = data_qba.size();
    if(start_idx < 0){
        end_idx += start_idx;
        start_idx = 0;
    }
    for(int i = start_idx; i < end_idx; i++){
        chk_sum += data_qba.at(i);
    }
    chk_sum = ~chk_sum;
    return chk_sum;
}

bool RobotArm::sendServoDataPacket(unsigned char id, unsigned char cmd, const QList<unsigned char> &param_list)
{
    if(servo_port == 0){
        return false;
    }

    QByteArray packet_qba;

    packet_qba.append(0x55);
    packet_qba.append(0x55);
    packet_qba.append(id);
    int param_len = param_list.size();
    packet_qba.append(param_len + 3);
    packet_qba.append(cmd);
    for(int i = 0; i < param_len; i++){
        packet_qba.append(param_list.at(i));
    }
    unsigned char chk_sum = getCheckSum(packet_qba, 2);
    packet_qba.append(chk_sum);
    //qDebug() << "send servo data packet:" << packet_qba.toHex();//
    int wr_len = servo_port->write(packet_qba);
    if(wr_len > 0)
        return true;
    else
        return false;
}

void RobotArm::readPortDataSlot()
{
    //返回的数据包格式：0x55 0x55 id length cmd param_1 ... param_n check_sum

    static QByteArray data_qba;
    static int rec_start_count = 0;

    QByteArray qba = servo_port->readAll();
    for(int i = 0; i < qba.size(); i++){
        unsigned char ch = qba.at(i);
        if(rec_start_count == 0 && ch == 0x55){
            rec_start_count++;
        }else if(rec_start_count == 1){
            if(ch == 0x55){
                rec_start_count++;
            }else{
                rec_start_count = 0;
            }
        }else if(rec_start_count >= 2){
            rec_start_count++;
            data_qba.append(ch);
            if(rec_start_count > 4){
                int data_len = data_qba.at(1);
                if(rec_start_count - 3 == data_len){
                    //接收字节数达到，重置接收参数
                    rec_start_count = 0;
                    //检查检验和
                    unsigned char chk_sum = getCheckSum(data_qba, -1);
                    if(chk_sum == (unsigned char)data_qba.at( data_qba.size() - 1 ) ){
                        rec_data.append(data_qba);
                        rec_data_finish = true;
                        /*if(data_qba.size() == 6){
                            unsigned char id = data_qba.at(0);
                            unsigned char cmd = data_qba.at(2);
                            unsigned char low_ch = data_qba.at(3);
                            unsigned char high_ch = data_qba.at(4);
                            unsigned short val = high_ch << 8;
                            val |= low_ch;
                            qDebug("id:%x, cmd:%x, val:%x", id, cmd, val);
                        }*/

                    }else{
                        qDebug() << "rec data check sum err!";
                    }
                    data_qba.clear();
                }
            }
        }
    }

    //qDebug() << "rec data:" << qba.toHex();
}
