#include "movebase.h"
#include <QTime>

MoveBase::MoveBase(QObject *parent) : QObject(parent)
{
    moveBase_client = NULL;
    socket_close_flag = false;

    //timer_test = new QTimer(this);
    //connect(timer_test, &QTimer::timeout, this, &MoveBase::timer1Slot);
    //timer_test->start(1000);
}

bool MoveBase::subscribeRosTopic(QString topic_name, QString msg_type, QString id)
{
    if(moveBase_client == NULL){
        return false;
    }

    bool sub_flag = false;
    QJsonObject obj_sub;
    if(msg_type != "")
        obj_sub.insert("type", msg_type);
    obj_sub.insert("topic", topic_name);
    obj_sub.insert("op", "subscribe");   //订阅节点
    if(id != "")
        obj_sub.insert("id", id);
    QJsonDocument json_doc(obj_sub);
    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);
    moveBase_client->write(json_qba);
    bool wr_flag = moveBase_client->waitForBytesWritten(200);
    if(wr_flag)
        sub_flag = true;

    return sub_flag;
}

bool MoveBase::unsubscribeRosTopic(QString topic_name, QString id)
{   
    if(moveBase_client == NULL){
        return false;
    }

    bool unsub_flag = false;
    QJsonObject obj_unsub;
    obj_unsub.insert("topic", topic_name);
    obj_unsub.insert("op", "unsubscribe");
    if(id != "")
        obj_unsub.insert("id", id);
    QJsonDocument json_doc(obj_unsub);
    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);
    moveBase_client->write(json_qba);
    bool wr_flag = moveBase_client->waitForBytesWritten(200);
    if(wr_flag)
        unsub_flag = true;

    return unsub_flag;
}

bool MoveBase::advertiseRosTopic(QString topic_name, QString msg_type, QString id)
{
    if(moveBase_client == NULL){
        return false;
    }

    bool flag = false;
    QJsonObject obj;
    obj.insert("type", msg_type);
    if(id != "")
        obj.insert("id", id);
    obj.insert("topic", topic_name);
    obj.insert("op", "advertise");
    QJsonDocument json_doc(obj);
    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);
    moveBase_client->write(json_qba);
    bool wr_flag = moveBase_client->waitForBytesWritten(200);
    if(wr_flag)
        flag = true;

    return flag;
}

bool MoveBase::unadvertiseRosTopic(QString topic_name, QString id)
{
    if(moveBase_client == NULL){
        return false;
    }

    bool flag = false;
    QJsonObject obj;
    obj.insert("topic", topic_name);
    if(id != "")
        obj.insert("id", id);
    obj.insert("op", "unadvertise");
    QJsonDocument json_doc(obj);
    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);
    moveBase_client->write(json_qba);
    bool wr_flag = moveBase_client->waitForBytesWritten(200);
    if(wr_flag)
        flag = true;

    return flag;
}

bool MoveBase::pubJsonMsgToRosTopic(QString topic_name, QJsonObject json_msg, QString id)
{
    if(moveBase_client == NULL){
        return false;
    }

    bool flag = false;
    QJsonObject obj;
    obj.insert("topic", topic_name);
    if(id != "")
        obj.insert("id", id);
    obj.insert("msg", json_msg);
    obj.insert("op", "publish");
    QJsonDocument json_doc(obj);
    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);
    moveBase_client->write(json_qba);
    bool wr_flag = moveBase_client->waitForBytesWritten(200);
    if(wr_flag)
        flag = true;


    return flag;
}

void MoveBase::initMoveBase()
{
    connectTcpSocket();
    //发布导航命令主题
    //start_code: 0, stop | 1, start | 2, continue
    //run_mode: 0, 路线只运行一次 | 1, 重复运行，到达最后一点后往第一点再运行 | 2, 重复运行，到达最后一点后往倒数第二点倒运行
    //rotate_desAngle_mode: 0, 到达目标点后不转动 | 1, 到达目标点后转动到目标方向
    //arrive_desPose_mode: 0, 连续行动 | 1, 到达一个路线点后暂停，等待继续运行信号
    //des_pose_arr: 目标点集合
    bool state_flag = advertiseRosTopic("robot_nav/nav_cmd", "robot_nav_pkg/NavCmd");
    if(state_flag)
        qDebug() << "advertise nav_cmd topic sucess";
    else
        qDebug() << "advertise nav_cmd topic fail";
    //订阅导航状态主题
    //run_state: 0, stop | 1, running | 2, wait continue signal | 3, arrive des pose and stop
    // 4, running but path block | 5, not find path, stop | 6, search path fail  | 7, outside cancel run, stop
    // 10,imp sensor act  11,sonar act  12,laser a1 act  13,laser a2 act  14,depth camera act,
    //15,control move sucess  16,contorl move fail 17,新的路径是绕路
    //17, odom change too big, 18, map change too big, 19:导航激光雷达检测到前，左右有障碍包围, 20:未检测到障碍包围
    //err_code: 0, cancel nav | 1, arrive point | -1, 未找到路径 | -2, odom坐标变动太大，退出导航 | -3, map坐标变动太大，退出导航
    //curr_des_idx: 当前目标点序号
    state_flag = subscribeRosTopic("robot_nav/nav_state", "robot_nav_pkg/NavState");
    if(state_flag)
        qDebug() << "subscribe nav_state topic sucess";
    else
        qDebug() << "subscribe nav_state topic fail";

    //发布深度数据主题
    state_flag = advertiseRosTopic("/depth_scan", "robot_nav_pkg/PointArray");
    if(state_flag)
        qDebug() << "advertise depth_scan topic sucess";
    else
        qDebug() << "advertise depth_scan topic fail";

    //发布遥控移动的cmd_vel主题
    state_flag = advertiseRosTopic("/turtle1/cmd_vel", "geometry_msgs/Twist");
    if(state_flag)
        qDebug() << "advertise turtle1/cmd_vel topic sucess";
    else
        qDebug() << "advertise turtle1/cmd_vel topic fail";

    state_flag = advertiseRosTopic("/relay_control", "std_msgs/Int32");
    if(state_flag)
        qDebug() << "advertise /relay_control topic sucess";
    else
        qDebug() << "advertise /relay_control topic fail";

}

void MoveBase::connectTcpSocket()
{
    moveBase_client = new QTcpSocket(this);
    moveBase_client->connectToHost("127.0.0.1", 9090, QTcpSocket::ReadWrite);
    bool connect_flag = moveBase_client->waitForConnected(2000);
    if(connect_flag){
      qDebug() << "movebase socket connect sucess!";
      connect(moveBase_client, SIGNAL(readyRead()), this, SLOT(socketReadyReadData()));
      connect(moveBase_client, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(tcpSocketErr(QAbstractSocket::SocketError)));
    }else{
      qDebug() << "movebase socket connect fail!";
      moveBase_client = NULL;
    }

}

bool MoveBase::startMove(QList<RobotPose> &des_pose_list, int run_mode, int rot_desAngle_mode, int arrive_desPose_mode, int start_idx)
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 1);
    nav_cmd_msg.insert("run_mode", run_mode);
    nav_cmd_msg.insert("rotate_desAngle_mode", rot_desAngle_mode);
    nav_cmd_msg.insert("arrive_desPose_mode", arrive_desPose_mode);
    nav_cmd_msg.insert("poseArray_start_idx", start_idx);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    for(int i = 0; i < des_pose_list.size(); i++){
        RobotPose des_pose = des_pose_list.at(i);
        point_msg.insert("x", des_pose.x);
        point_msg.insert("y", des_pose.y);
        point_msg.insert("z", des_pose.z);
        MyVector4D quatAngle = yawAngleToQuatAngle(des_pose.z);
        quat_msg.insert("x", quatAngle.x);
        quat_msg.insert("y", quatAngle.y);
        quat_msg.insert("z", quatAngle.z);
        quat_msg.insert("w", quatAngle.w);
        pose_msg.insert("position", point_msg);
        pose_msg.insert("orientation", quat_msg);
        poses_msg.append(pose_msg);
    }
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", 0);
    nav_cmd_msg.insert("rotate_angle", 0);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::startMoveAndWaitConfirm(QList<RobotPose> &des_pose_list, int run_mode, int rot_desAngle_mode, int arrive_desPose_mode, int start_idx, int timeout_ms)
{
    bool flag;

    get_state_flag = false;
    flag = startMove(des_pose_list, run_mode, rot_desAngle_mode, arrive_desPose_mode, start_idx);

    if(flag){
        QTime t = QTime::currentTime().addMSecs(timeout_ms);
        while(!get_state_flag){
            delayMs(50);
            if(QTime::currentTime() > t){
               flag = false;
               break;
            }
        }
        if(get_state_flag){
            if(move_base_state.run_state == 1){
               flag = true;
            }else{
                flag = false;
            }
        }
    }

    return flag;
}

bool MoveBase::stopMove()
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 0);
    nav_cmd_msg.insert("run_mode", 0);
    nav_cmd_msg.insert("rotate_desAngle_mode", 0);
    nav_cmd_msg.insert("arrive_desPose_mode", 0);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    point_msg.insert("x", 0);
    point_msg.insert("y", 0);
    point_msg.insert("z", 0);
    quat_msg.insert("x", 0);
    quat_msg.insert("y", 0);
    quat_msg.insert("z", 0);
    quat_msg.insert("w", 0);
    pose_msg.insert("position", point_msg);
    pose_msg.insert("orientation", quat_msg);
    poses_msg.append(pose_msg);
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", 0);
    nav_cmd_msg.insert("rotate_angle", 0);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::stopMoveAndWaitConfirm(int timeout_ms)
{
    bool flag;

    get_state_flag = false;
    flag = stopMove();

    if(flag){
        QTime t = QTime::currentTime().addMSecs(timeout_ms);
        while(!get_state_flag){
            delayMs(50);
            if(QTime::currentTime() > t){
               flag = false;
               break;
            }
        }
        if(get_state_flag){
            if(move_base_state.run_state == 7){
               flag = true;
            }else{
                flag = false;
            }
        }
    }

    return flag;
}

bool MoveBase::continueMove()
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 2);
    nav_cmd_msg.insert("run_mode", 0);
    nav_cmd_msg.insert("rotate_desAngle_mode", 0);
    nav_cmd_msg.insert("arrive_desPose_mode", 0);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    point_msg.insert("x", 0);
    point_msg.insert("y", 0);
    point_msg.insert("z", 0);
    quat_msg.insert("x", 0);
    quat_msg.insert("y", 0);
    quat_msg.insert("z", 0);
    quat_msg.insert("w", 0);
    pose_msg.insert("position", point_msg);
    pose_msg.insert("orientation", quat_msg);
    poses_msg.append(pose_msg);
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", 0);
    nav_cmd_msg.insert("rotate_angle", 0);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::continueMoveAndWaitConfirm(int timeout_ms)
{
    bool flag;

    get_state_flag = false;
    flag = continueMove();

    if(flag){
        QTime t = QTime::currentTime().addMSecs(timeout_ms);
        while(!get_state_flag){
            delayMs(50);
            if(QTime::currentTime() > t){
               flag = false;
               break;
            }
        }
        if(get_state_flag){
            if(move_base_state.run_state == 1){
               flag = true;
            }else{
                flag = false;
            }
        }
    }

    return flag;
}

bool MoveBase::controlMove(float move_dis, float rot_angle)
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 3);
    nav_cmd_msg.insert("run_mode", 0);
    nav_cmd_msg.insert("rotate_desAngle_mode", 0);
    nav_cmd_msg.insert("arrive_desPose_mode", 0);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    point_msg.insert("x", 0);
    point_msg.insert("y", 0);
    point_msg.insert("z", 0);
    quat_msg.insert("x", 0);
    quat_msg.insert("y", 0);
    quat_msg.insert("z", 0);
    quat_msg.insert("w", 0);
    pose_msg.insert("position", point_msg);
    pose_msg.insert("orientation", quat_msg);
    poses_msg.append(pose_msg);
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", move_dis);
    nav_cmd_msg.insert("rotate_angle", rot_angle);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::controlMoveAndWaitConfirm(float move_dis, float rot_angle, int timeout_ms)
{
    bool flag;

    get_state_flag = false;
    flag = controlMove(move_dis, rot_angle);

    if(flag){
        QTime t = QTime::currentTime().addMSecs(timeout_ms);
        while(!get_state_flag){
            delayMs(50);
            if(QTime::currentTime() > t){
               flag = false;
               break;
            }
        }
        if(get_state_flag){
            if(move_base_state.run_state == 15){
               flag = true;
            }else{
                flag = false;
            }
        }
    }

    return flag;
}

bool MoveBase::getMoveBaseState()
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 4);
    nav_cmd_msg.insert("run_mode", 0);
    nav_cmd_msg.insert("rotate_desAngle_mode", 0);
    nav_cmd_msg.insert("arrive_desPose_mode", 0);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    point_msg.insert("x", 0);
    point_msg.insert("y", 0);
    point_msg.insert("z", 0);
    quat_msg.insert("x", 0);
    quat_msg.insert("y", 0);
    quat_msg.insert("z", 0);
    quat_msg.insert("w", 0);
    pose_msg.insert("position", point_msg);
    pose_msg.insert("orientation", quat_msg);
    poses_msg.append(pose_msg);
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", 0);
    nav_cmd_msg.insert("rotate_angle", 0);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::getMoveBaseStateAndWaitReturn(moveBase_state_stru &move_base_sta, int timeout_ms)
{
    bool flag;

    get_state_flag = false;
    flag = getMoveBaseState();

    if(flag){
        QTime t = QTime::currentTime().addMSecs(timeout_ms);
        while(!get_state_flag){
            delayMs(50);
            if(QTime::currentTime() > t){
               flag = false;
               break;
            }
        }
        if(get_state_flag){
            move_base_sta = move_base_state;
            flag = true;
        }
    }

    return flag;
}


bool MoveBase::sendDepthPoints(QList<RobotPose> &depth_point_list)
{
    if(socket_close_flag){
        return false;
    }

    QJsonObject depth_point_msg, depth_points_msg;
    QJsonArray depth_points_arr;
    RobotPose point;
    for(int i = 0; i < depth_point_list.size(); i++){
        point = depth_point_list.at(i);
        depth_point_msg.insert("x", point.x);
        depth_point_msg.insert("y", point.y);
        depth_point_msg.insert("z", point.z);
        depth_points_arr.append(depth_point_msg);
    }
    depth_points_msg.insert("pointArray", QJsonValue(depth_points_arr));

    bool flag = pubJsonMsgToRosTopic("/depth_scan", depth_points_msg);

    return flag;
}

float MoveBase::quatAngleToYawAngle(float qx, float qy, float qz, float qw)
{
    float yaw_z = atan2( 2*(qw*qz + qx* qy), 1 - 2*(qy*qy + qz*qz) );
    return yaw_z;
}

MyVector4D MoveBase::yawAngleToQuatAngle(float yaw_angle)
{
    float roll_angle = 0, pitch_angle = 0;
    MyVector4D quatAngle;
    float qx = sin(roll_angle / 2) * cos(pitch_angle / 2) * cos(yaw_angle / 2) - cos(roll_angle / 2) * sin(pitch_angle / 2) * sin(yaw_angle / 2);
    float qy = cos(roll_angle / 2) * sin(pitch_angle / 2) * cos(yaw_angle / 2) + sin(roll_angle / 2) * cos(pitch_angle / 2) * sin(yaw_angle / 2);
    float qz = cos(roll_angle / 2) * cos(pitch_angle / 2) * sin(yaw_angle / 2) - sin(roll_angle / 2) * sin(pitch_angle / 2) * cos(yaw_angle / 2);
    float qw = cos(roll_angle / 2) * cos(pitch_angle / 2) * cos(yaw_angle / 2) + sin(roll_angle / 2) * sin(pitch_angle / 2) * sin(yaw_angle / 2);
    quatAngle.x = qx;
    quatAngle.y = qy;
    quatAngle.z = qz;
    quatAngle.w = qw;
    return quatAngle;
}

void MoveBase::delayMs(int ms)
{
    QTime delay_time = QTime::currentTime().addMSecs(ms);
    while(QTime::currentTime() < delay_time)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
}

//vlx m/s, v_th rad/s
bool MoveBase::sendMoveSpeedMsg(float v_lx, float v_th)
{
    QJsonObject linear_obj, angular_obj, twist_obj;

    linear_obj.insert("x", v_lx);
    linear_obj.insert("y", 0);
    linear_obj.insert("z", 0);

    angular_obj.insert("x", 0);
    angular_obj.insert("y", 0);
    angular_obj.insert("z", v_th);

    twist_obj.insert("linear", linear_obj);
    twist_obj.insert("angular", angular_obj);

    bool pub_flag = pubJsonMsgToRosTopic("/turtle1/cmd_vel", twist_obj);

    return pub_flag;
}

bool MoveBase::startDockChargeStation()
{
    QJsonObject nav_cmd_msg;
    nav_cmd_msg.insert("start_code", 5);
    nav_cmd_msg.insert("run_mode", 0);
    nav_cmd_msg.insert("rotate_desAngle_mode", 0);
    nav_cmd_msg.insert("arrive_desPose_mode", 0);
    QJsonObject pose_arr_msg, pose_msg, point_msg, quat_msg;
    QJsonArray poses_msg;
    point_msg.insert("x", 0);
    point_msg.insert("y", 0);
    point_msg.insert("z", 0);
    quat_msg.insert("x", 0);
    quat_msg.insert("y", 0);
    quat_msg.insert("z", 0);
    quat_msg.insert("w", 0);
    pose_msg.insert("position", point_msg);
    pose_msg.insert("orientation", quat_msg);
    poses_msg.append(pose_msg);
    pose_arr_msg.insert("poses", QJsonValue(poses_msg));
    nav_cmd_msg.insert("des_poseArr", pose_arr_msg);

    nav_cmd_msg.insert("move_dis", 0);
    nav_cmd_msg.insert("rotate_angle", 0);

    bool flag = pubJsonMsgToRosTopic("robot_nav/nav_cmd", nav_cmd_msg);

    return flag;
}

bool MoveBase::ctrlRelay(bool open_flag)
{
    QJsonObject ctrl_relay_obj;
    int ctrl_code = 0;
    if(open_flag){
       ctrl_code = 1;
    }
    ctrl_relay_obj.insert("data", ctrl_code);

    bool pub_flag = pubJsonMsgToRosTopic("/relay_control", ctrl_relay_obj);

    return pub_flag;
}


void MoveBase::recDepthPointSlot(QList<RobotPose> blc_point_list)
{
    sendDepthPoints(blc_point_list);
    //qDebug("move base rec depth data, size:%d", blc_point_list.size());
}

void MoveBase::timer1Slot()
{
    qDebug() << "emit sig...";
    //emit recMoveBaseRunStateSignal(nav_run_state);
}

void MoveBase::continueMoveSlot()
{
    continueMove();
}

void MoveBase::startMoveSlot(QVariant param_var)
{
    startMove_param_stru param_stru = param_var.value<startMove_param_stru>();
    startMove(param_stru.des_pose_list, param_stru.run_mode, param_stru.rot_desAngle_mode, param_stru.arrive_desPose_mode, param_stru.start_idx);
}

void MoveBase::stopMoveSlot()
{
    stopMove();
}


void MoveBase::socketReadyReadData()
{
    //qDebug() << "rec data...";
    QByteArray qba = moveBase_client->readAll();
    QString json_qs = QString(qba);
    //qDebug() << "json str:" << json_qs;
#if 0
    //使用正则表达式从json字符串中提取信息
    //json str: "{\"topic\": \"robot_nav/nav_state\",
    //\"msg\": {\"robot_odom_pose\": {\"y\": 1.955293100763811e-06, \"x\": 4.148233711021021e-05, \"z\": 0.02442999929189682},
    //\"robot_map_pose\": {\"y\": 0.016255684196949005, \"x\": 0.05200754106044769, \"z\": 0.00254823244176805},
    //\"bat_vol\": 216341.203125, \"run_state\": 0, \"err_code\": 0, \"curr_des_idx\": 0}, \"op\": \"publish\"}"
    //QRegExp regexp( "run_state.:\\s*(\\d+)" );
    QRegExp regexp( "run_state.:\\s*(\\d+)" );
    int pos = 0;
    //从data_qs中从pos位置开始查找匹配的字符串，返回匹配到的第一个字符位置
    QStringList qs_list;
    QString run_state_qs;
    while( (pos = regexp.indexIn(json_qs, pos)) != -1){
        run_state_qs = regexp.cap(1);
        qs_list << run_state_qs;
        pos++;
    }
    for(int i = 0; i < qs_list.length(); i++){
        //qDebug() << "run state:" << qs_list.at(i);
        run_state_qs = qs_list.at(i);
        bool ok;
        //nav_run_state = run_state_qs.toInt(&ok, 10);
        if(ok){
            //emit recMoveBaseRunStateSignal(nav_run_state);
        }else{
            qDebug() << "tranf run_state_qs to int num fail";
        }
    }

    return;

#endif

//*
    QJsonParseError json_err;
    QJsonDocument json_doc = QJsonDocument::fromJson(qba, &json_err);
    QJsonObject json_obj;
    QJsonValue json_value;
    if(!json_doc.isNull() && (json_err.error == QJsonParseError::NoError)){        
        if(json_doc.isObject()){
            json_obj = json_doc.object();
            if(json_obj.contains("topic")){
                json_value = json_obj.value("topic");
                if(json_value.isString()){
                    QString topic_qs = json_value.toString();
                    //qDebug() << "topic:" << topic_qs;
                }
            }
            if(json_obj.contains("msg")){
                json_value = json_obj.value("msg");
                if(json_value.isObject()){
                    json_obj = json_value.toObject();
                }else{
                    qDebug() << "json not contain msg, return";
                    return;
                }
            }

            if(json_obj.contains("run_state")){
                json_value = json_obj.value("run_state");
                if(json_value.isDouble()){
                    int nav_run_state = json_value.toDouble();
                    //emit recMoveBaseRunStateSignal(nav_run_state);
                    move_base_state.run_state = nav_run_state;
                    //qDebug("get nav run state:%d", nav_run_state);
                }
            }

            if(json_obj.contains("err_code")){
                json_value = json_obj.value("err_code");
                if(json_value.isDouble()){
                    int nav_err_code = json_value.toDouble();
                    move_base_state.err_code = nav_err_code;
                    //qDebug("get nav err code:%d", nav_err_code);
                }
            }

            if(json_obj.contains("curr_des_idx")){
                json_value = json_obj.value("curr_des_idx");
                if(json_value.isDouble()){
                    int nav_curr_idx = json_value.toDouble();
                    move_base_state.curr_des_idx = nav_curr_idx;
                    //qDebug("get nav curr des idx:%d", nav_curr_idx);
                }
            }

            if(json_obj.contains("curr_arrive_idx")){
                json_value = json_obj.value("curr_arrive_idx");
                if(json_value.isDouble()){
                    int curr_arr_idx = json_value.toDouble();
                    move_base_state.curr_arrive_idx = curr_arr_idx;
                    //qDebug("get nav curr arrive idx:%d", curr_arr_idx);
                }
            }

            if(json_obj.contains("bat_vol")){
                json_value = json_obj.value("bat_vol");
                if(json_value.isDouble()){
                    float bat_vol = json_value.toDouble();
                    move_base_state.bat_vol = bat_vol;
                    //qDebug("bat_vol:%f", bat_vol);
                }
            }

            if(json_obj.contains("robot_odom_pose")){
                json_value = json_obj.value("robot_odom_pose");
                if(json_value.isObject()){
                    QJsonObject obj_tmp = json_value.toObject();
                    float odom_pose_x, odom_pose_y, odom_pose_angle;
                    if(obj_tmp.contains("x")){
                        json_value = obj_tmp.value("x");
                        odom_pose_x = json_value.toDouble();
                    }
                    if(obj_tmp.contains("y")){
                        json_value = obj_tmp.value("y");
                        odom_pose_y = json_value.toDouble();
                    }
                    if(obj_tmp.contains("z")){
                        json_value = obj_tmp.value("z");
                        odom_pose_angle = json_value.toDouble();
                    }
                    move_base_state.robot_odom_pose.x = odom_pose_x;
                    move_base_state.robot_odom_pose.y = odom_pose_y;
                    move_base_state.robot_odom_pose.z = odom_pose_angle;
                    //qDebug("robot odom pose, x:%f, y:%f, angle:%f", odom_pose_x, odom_pose_y, odom_pose_angle);
                }
            }

            if(json_obj.contains("robot_map_pose")){
                json_value = json_obj.value("robot_map_pose");
                if(json_value.isObject()){
                    QJsonObject obj_tmp = json_value.toObject();
                    float map_pose_x, map_pose_y, map_pose_angle;
                    if(obj_tmp.contains("x")){
                        json_value = obj_tmp.value("x");
                        map_pose_x = json_value.toDouble();
                    }
                    if(obj_tmp.contains("y")){
                        json_value = obj_tmp.value("y");
                        map_pose_y = json_value.toDouble();
                    }
                    if(obj_tmp.contains("z")){
                        json_value = obj_tmp.value("z");
                        map_pose_angle = json_value.toDouble();
                    }
                    move_base_state.robot_map_pose.x = map_pose_x;
                    move_base_state.robot_map_pose.y = (map_pose_y);
                    move_base_state.robot_map_pose.z = (map_pose_angle);
                    //qDebug("robot map pose, x:%f, y:%f, angle:%f", map_pose_x, map_pose_y, map_pose_angle);
                }
            }

            QVariant state_var;
            state_var.setValue(move_base_state);
            get_state_flag = true;
            emit sendMoveBaseRunStateSignal(state_var);

        }
    }else{
        qDebug() << "parse json err:" << json_err.errorString();
    }

//*/
}

void MoveBase::tcpSocketErr(QAbstractSocket::SocketError err)
{
    if(err == QAbstractSocket::RemoteHostClosedError){
        socket_close_flag = true;
    }
    qDebug() << "tcpSocketErr:" << moveBase_client->errorString();
}

