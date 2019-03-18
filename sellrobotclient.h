#ifndef SELLROBOTCLIENT_H
#define SELLROBOTCLIENT_H

#include <QObject>
#include <QtNetwork/QHostInfo>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QCoreApplication>
#include <QtCore/QJsonValue>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonParseError>
#include <QTime>
#include <QTimer>
#include <QList>
#include <QDebug>

#include "robotpose.h"
#include "movebase.h"
#include "robotarm.h"
#include "robotconfig.h"

#define     MOVEBASE_STATE_IDLE                 0
#define     MOVEBASE_STATE_RUN                  1
#define     MOVEBASE_STATE_NAV_CHARGE_POS       10
#define     MOVEBASE_STATE_DOCK_CHARGE          11
#define     MOVEBASE_STATE_CHARGING             12
#define     MOVEBASE_STATE_DOCK_FAIL            13
#define     MOVEBASE_STATE_NOT_DET_VOL          14
#define     MOVEBASE_STATE_CHARGE_FAIL          15


static const quint16 REGISTER_SELLROBOT_CMD = 0x9000;   //注册
static const quint16 SEND_HEART_BEAT_SELLROBOT_CMD = 0x9007; //发送给服务器心跳
static const quint16 REC_HEART_BEAT_SELLROBOT_CMD = 0x9008;  //接收服务器发送的心跳
static const quint16 SEND_POS_CMD = 0x9001;             //发送当前位置，机器人发送
static const quint16 ARRIVE_DES_POS_CMD = 0x9002;       //到达目标点，机器人发送
static const quint16 MOVE_TO_DES_POS_CMD = 0x9003;      //前往某个坐标点，服务器发送
static const quint16 STOP_MOVE_CMD = 0x9004;            //停止移动或继续移动
static const quint16 GET_MAP_POS_CMD = 0x9005;          //获取当前位置坐标，服务器发送
static const quint16 SEND_MAP_POS_CMD = 0x9006;            //发送当前位置坐标，机器人发送
static const quint16 START_DOCK_CHARGE_CMD = 0x9009;        //开始对接充电坐，服务器发送
static const quint16 SEND_MOVEBASE_STATE_CMD = 0x9010;      //发送当前机器人状态，机器人发送


#define      SELL_ROBOT_ID           "3"


class SellRobotClient : public QObject
{
    Q_OBJECT
public:
    explicit SellRobotClient(MoveBase* move_base_ptr, QObject* parent = nullptr);
    SellRobotClient();

    void init(MoveBase* move_base_ptr);

    bool connectToServer();

    void startRealsenseCam();

    void test();

private:
    //从qstring和qbytearray解析json
    bool parseJsonFromQString(QJsonObject& json_obj, QString& json_qs);
    bool parseJsonFromQByteArray(QJsonObject& json_obj, const QByteArray& json_qba);
    int parseJsonFromQByteArray(QJsonObject& json_obj, QJsonArray& json_arr, const QByteArray& json_qba);
    //通讯转义和校验
    char calculateCheckCode(QByteArray& data_qba, int st_idx, int end_idx);
    void replaceSymbolInSendData(QByteArray& data_qba, int st_idx, int end_idx);
    bool replaceSymbolInRecData(QByteArray& data_qba, int st_idx, int end_idx);
    //发送命令
    bool sendCmdToServer(const quint16 cmd_code, const QString data_str);
    //通讯和命令处理
    void processSocketData(QList<QByteArray>& qba_list);
    void processServerCmd(const quint16 cmd_code, const QByteArray& data_qba);

    bool sendPosMsgToServer();      //发送当前位置给服务器

    bool sendArriveOnePointMsgToServer(); //发送到达一个里程点位置给服务器
    //发送到达消息给服务器
    bool sendArriveMsgToServer(QString pos_name);

    //按接收到的路线信息移动
    bool moveToDesPos(const QByteArray& data_qba);
    //暂停或继续运行
    bool pauseOrContinueMove(const QByteArray& data_qba);
    //根据坐标名称获取坐标地址
    bool getPosByPosName(RobotPose& pos, const QString pos_name);
    //计算两点距离
    float calTwoPointDis(RobotPose& point1, RobotPose& point2);

    //发送注册信息给服务器
    bool sendRegisterMsg();
    //发送心跳信息给服务器
    bool sendHeartBeatMsg();
    //发送全局坐标给服务器
    bool sendMapPosToServer();

    //发送底盘状态给服务器
    bool sendMovebaseStateCodeToServer(int state_code);

    //开始对接充电坐
    bool startDockChargeStation(const QByteArray& data_qba);

    void dockChargeStation();

    void moveToChargePosAndDock(RobotPose& charge_pos, int wait_time_sec);

    //退出充电坐
    bool unDockChargeStation();

signals:
    void startHeadRotateSignal(QString angle);

    void startActionSignal(QString act_id);

    void startRealsenseCameraSignal();

public slots:
    void getMoveBaseRunStateSlot(QVariant state_var);   //底盘运行状态


private slots:
    //socket状态
    void getTcpSocketErrSlot(QAbstractSocket::SocketError err);
    void getSocketStateChangeSlot(QAbstractSocket::SocketState socketState);
    void getTcpSocketDisconnectSlot();
    //socket数据处理
    void getTcpSocketDataSlot();
    //发送心跳和监控socket状态的定时器处理
    void heartBeatTimerSlot();

    void sendPosTimerSlot();

    void checkChargeStateTimerSlot();

private:
    QList<QByteArray> rec_qba_list; //收到的未经过转义的原始消息队列

    QTcpSocket* robot_socket_p;
    QString server_ip;
    int server_port;
    QTimer* heartBeat_timer_p;       //发送心跳状态和监控socket连接状态

    bool register_robot_flag;      //注册标志，注册成功true，失败false
    bool heart_beat_flag;          //心跳标志，收到心跳信息将此标志设为true
    bool socket_connect_flag;      //后台连接标志，连线true， 断线false

    MoveBase* move_base_p;
    RobotArm* robot_arm_p;
    RobotConfig robot_config;


    int curr_patral_id;
    bool get_movebase_state_flag;
    moveBase_state_stru move_base_state;

    QStringList pos_list_qs;
    QList<RobotPose> des_pos_list;

    bool is_moving_flag;

    QTimer* send_pos_timer_p;

    QTimer* check_charge_state_timer_p;
    bool arrive_one_pos_flag;
    RobotPose charge_pos;       //充电坐标

    //底盘运行状态码，
    //0：待机，1：运行
    //10：前往充电点，11：充电对接，12：正在充电，13：充电失败
    int movebase_state_code;
    bool dock_proc_finish_flag;
};

#endif // SELLROBOTCLIENT_H
