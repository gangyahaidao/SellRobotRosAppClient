#ifndef MOVEBASE_H
#define MOVEBASE_H

#include <QObject>
#include <QtCore/QJsonValue>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonParseError>
#include <QCoreApplication>
#include <QString>
#include <QByteArray>
#include <QDebug>
#include <QtNetwork/QTcpSocket>
#include <QTimer>
#include <QTime>
//#include <QVector3D>
//#include <QVector4D>
#include <QList>
#include <qmath.h>
#include <QRegExp>


#include "robotpose.h"
#include "myvector4d.h"


struct moveBase_state_stru {
  int run_state;
  int err_code;
  int curr_des_idx;
  int curr_arrive_idx;
  float bat_vol;
  RobotPose robot_map_pose;
  RobotPose robot_odom_pose;
};

struct startMove_param_stru {
    QList<RobotPose> des_pose_list;
    int run_mode;
    int rot_desAngle_mode;
    int arrive_desPose_mode;
    int start_idx;
};

Q_DECLARE_METATYPE(moveBase_state_stru)
Q_DECLARE_METATYPE(startMove_param_stru)

class MoveBase : public QObject
{
    Q_OBJECT
public:
    explicit MoveBase(QObject *parent = 0);


    void initMoveBase();

    bool startMove(QList<RobotPose>& des_pose_list, int run_mode, int rot_desAngle_mode, int arrive_desPose_mode, int start_idx = 0);
    bool startMoveAndWaitConfirm(QList<RobotPose>& des_pose_list, int run_mode, int rot_desAngle_mode, int arrive_desPose_mode, int start_idx = 0, int timeout_ms = 500);

    bool stopMove();
    bool stopMoveAndWaitConfirm(int timeout_ms = 500);

    bool continueMove();
    bool continueMoveAndWaitConfirm(int timeout_ms = 500);

    //底盘导航程序中move_dis<=0.5,rot_angle<=6.3;
    //一次控制只有一个参数有效，另一个必须为0,当两个都不为0时，只有move_dis有效
    bool controlMove(float move_dis, float rot_angle);
    bool controlMoveAndWaitConfirm(float move_dis, float rot_angle, int timeout_ms);
    //获取底盘状态
    bool getMoveBaseState();
    bool getMoveBaseStateAndWaitReturn(moveBase_state_stru& move_base_sta, int timeout_ms = 1000);
    //发送深度数据给底盘导航程序
    bool sendDepthPoints(QList<RobotPose>& depth_point_list);

    void delayMs(int ms);
    //发送速度信息给底盘
    bool sendMoveSpeedMsg(float v_lx, float v_th);

    //发送对接充电坐信号给底盘
    bool startDockChargeStation();

    //控制继电器
    bool ctrlRelay(bool open_flag);

private:
    bool subscribeRosTopic(QString topic_name, QString msg_type = "", QString id = "");
    bool unsubscribeRosTopic(QString topic_name, QString id = "");
    bool advertiseRosTopic(QString topic_name, QString msg_type, QString id = "");
    bool unadvertiseRosTopic(QString topic_name, QString id = "");
    bool pubJsonMsgToRosTopic(QString topic_name, QJsonObject json_msg, QString id = "");
    void connectTcpSocket();


    float quatAngleToYawAngle(float x, float y, float z, float w);
    MyVector4D yawAngleToQuatAngle(float yaw_angle);

signals:
    void recMoveBaseRunStateSignal(int run_state);
    void sendMoveBaseRunStateSignal(QVariant state_var);

public slots:
    void recDepthPointSlot(QList<RobotPose> blc_point_list);
    void timer1Slot();

    void continueMoveSlot();
    void startMoveSlot(QVariant param_var);
    void stopMoveSlot();

private slots:
    void socketReadyReadData();
    void tcpSocketErr(QAbstractSocket::SocketError err);

public:
    moveBase_state_stru move_base_state;
    bool get_state_flag;

private:
    QTcpSocket* moveBase_client;

    bool socket_close_flag;

    QTimer* timer_test;
};

#endif // MOVEBASE_H
