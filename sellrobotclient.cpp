#include "sellrobotclient.h"

SellRobotClient::SellRobotClient(MoveBase* move_base_ptr, QObject* parent) : QObject(parent)
{
    move_base_p = move_base_ptr;

    //心跳检测和连接状态监控定时器初始化，信号连接
    heartBeat_timer_p = new QTimer(this);
    connect(heartBeat_timer_p, &QTimer::timeout, this, &SellRobotClient::heartBeatTimerSlot);

    send_pos_timer_p = new QTimer(this);
    connect(send_pos_timer_p, SIGNAL(timeout()), this, SLOT(sendPosTimerSlot()));

    bool rd_flag = robot_config.readLocalConfig();

    if(rd_flag){
        server_ip = robot_config.server_ip;
        server_port = robot_config.server_port;
    }else{
        qDebug() << "sell robot client rd local config fail!";
    }

    robot_socket_p = NULL;

    is_moving_flag = false;
}

SellRobotClient::SellRobotClient()
{
    move_base_p = 0;
}

void SellRobotClient::init(MoveBase *move_base_ptr)
{
    move_base_p = move_base_ptr;

    //心跳检测和连接状态监控定时器初始化，信号连接
    heartBeat_timer_p = new QTimer(this);
    connect(heartBeat_timer_p, &QTimer::timeout, this, &SellRobotClient::heartBeatTimerSlot);

    send_pos_timer_p = new QTimer(this);
    connect(send_pos_timer_p, SIGNAL(timeout()), this, SLOT(sendPosTimerSlot()));

    check_charge_state_timer_p = new QTimer(this);
    connect(check_charge_state_timer_p, SIGNAL(timeout()), this, SLOT(checkChargeStateTimerSlot()));

    bool rd_flag = robot_config.readLocalConfig();

    if(rd_flag){
        server_ip = robot_config.server_ip;
        server_port = robot_config.server_port;
    }else{
        qDebug() << "sell robot client rd local config fail!";
    }

    robot_socket_p = NULL;

    is_moving_flag = false;

    movebase_state_code = MOVEBASE_STATE_IDLE;
    dock_proc_finish_flag = true;
}

bool SellRobotClient::connectToServer()
{
    if(server_ip.isEmpty() || server_port == 0){
        qDebug() << "connect server but server ip or server port is empty, return";
        return false;
    }

    robot_socket_p = new QTcpSocket(this);

    robot_socket_p->connectToHost(server_ip, server_port);

    connect(robot_socket_p, &QTcpSocket::readyRead, this, &SellRobotClient::getTcpSocketDataSlot);
    connect(robot_socket_p, &QTcpSocket::disconnected, this, &SellRobotClient::getTcpSocketDisconnectSlot);
    connect(robot_socket_p, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(getTcpSocketErrSlot(QAbstractSocket::SocketError)));
    connect(robot_socket_p, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(getSocketStateChangeSlot(QAbstractSocket::SocketState)));

    bool conn_flag = robot_socket_p->waitForConnected(2000);

    if(conn_flag){
        qDebug() << "conn server sucess, server_ip:" << server_ip << ",port:" << server_port;

        sendRegisterMsg();  //发送注册消息

        //连接注册成功后启动监控定时器
        if(!heartBeat_timer_p->isActive()){
            heartBeat_timer_p->start(1000);
        }

    }else{
        qDebug() << "conn server fail, server_ip:" << server_ip << ",port:" << server_port;
    }

    return conn_flag;
}

void SellRobotClient::startRealsenseCam()
{
    qDebug() << "start realsense camera!";
    emit startRealsenseCameraSignal();
}

void SellRobotClient::test()
{
    move_base_p->delayMs(3000);
    charge_pos.x = 0;
    charge_pos.y = 0;
    movebase_state_code = MOVEBASE_STATE_DOCK_CHARGE;
    check_charge_state_timer_p->start(100);
    qDebug() << "start dock timer!";

//    pos_list_qs.push_back("实验室");
//    pos_list_qs.push_back("商会");
//    pos_list_qs.push_back("会议室");
//    //pos_list_qs.push_back("公司前台");

//    des_pos_list.clear();
//    for(int i = 0; i < pos_list_qs.size(); i++){
//        //qDebug() << pos_list_qs.at(i);
//        RobotPose pos;
//        bool get_flag = getPosByPosName(pos, pos_list_qs.at(i));
//        if(!get_flag){
//            qWarning() << "not find pos by pos name, return;";
//            return;
//        }
//        des_pos_list.push_back(pos);
//    }

//    for(int i = 0; i < des_pos_list.size(); i++){
//        qDebug() << "i:" << i << ", pos x:" << des_pos_list.at(i).x << ", y:" << des_pos_list.at(i).y;
//    }

//    //开始沿路线移动
//    curr_patral_id = 0;
//    move_base_p->startMove(des_pos_list, 0, 0, 0, 0);

//    is_moving_flag = true;
//    send_pos_timer_p->start(1500);  //开始移动后开启发送当前位置定时器

}

bool SellRobotClient::parseJsonFromQString(QJsonObject &json_obj, QString &json_qs)
{
    QJsonParseError json_err;
    QByteArray data_qba = json_qs.toUtf8();
    QJsonDocument json_doc = QJsonDocument::fromJson(data_qba, &json_err);
    if(json_err.error != QJsonParseError::NoError || json_doc.isNull()){
        qWarning() << "parse Json fail, return";
        return false;
    }

    if(!json_doc.isObject()){
        qWarning() << "json doc is not object, return";
        return false;
    }

    json_obj = json_doc.object();

    return true;
}

bool SellRobotClient::parseJsonFromQByteArray(QJsonObject &json_obj, const QByteArray &json_qba)
{
    QJsonParseError json_err;
    QJsonDocument json_doc = QJsonDocument::fromJson(json_qba, &json_err);
    if(json_err.error != QJsonParseError::NoError || json_doc.isNull()){
        qWarning() << "parse Json fail, return";
        return false;
    }

    if(!json_doc.isObject()){
        qWarning() << "json doc is not object, return";
        return false;
    }

    json_obj = json_doc.object();

    return true;
}

int SellRobotClient::parseJsonFromQByteArray(QJsonObject &json_obj, QJsonArray &json_arr, const QByteArray &json_qba)
{
    int rt_code = 0;
    QJsonParseError json_err;
    QJsonDocument json_doc = QJsonDocument::fromJson(json_qba, &json_err);
    if(json_err.error != QJsonParseError::NoError || json_doc.isNull()){
        qWarning() << "parse Json fail, return";
        return rt_code;
    }

    if(json_doc.isObject()){
       json_obj = json_doc.object();
       rt_code = 1;
    }else if(json_doc.isArray()){
       json_arr = json_doc.array();
       rt_code = 2;
    }else{
       qWarning() << "parse json but jsondoc not obj or arr!";
    }

    return rt_code;
}

char SellRobotClient::calculateCheckCode(QByteArray &data_qba, int st_idx, int end_idx)
{
    //计算校验码，范围：[开始序号st_idx, 结束序号end_idx]
    char chk_byte = data_qba.at(st_idx);
    for(int i = st_idx+1; i <= end_idx; i++){
        chk_byte ^= data_qba.at(i);
    }
    return chk_byte;
}

void SellRobotClient::replaceSymbolInSendData(QByteArray &data_qba, int st_idx, int end_idx)
{
    for(int i = st_idx; i <= end_idx; i++){
        char ch = data_qba[i];
        if(ch == 0x7e){
            data_qba[i] = 0x7d;
            i++;
            data_qba.insert(i, 0x02);
            end_idx++;
        }else if(ch == 0x7d){
            data_qba[i] = 0x7d;
            i++;
            data_qba.insert(i, 0x01);
            end_idx++;
        }
    }
}

bool SellRobotClient::replaceSymbolInRecData(QByteArray &data_qba, int st_idx, int end_idx)
{
    bool replace_success_flag = true;
    for(int i = st_idx; i < end_idx; i++){
        char ch = data_qba[i];
        if(ch == 0x7d){
            if(i+1 > end_idx){
                replace_success_flag = false;
                break;
            }else{
                char ch_next = data_qba[i+1];
                if(ch_next == 0x01){			//0x7d0x01替换为0x7d
                    data_qba.remove(i+1, 1);
                    end_idx--;
                }else if(ch_next == 0x02){		//0x7d0x02替换为0x7e
                    data_qba[i] = 0x7e;
                    data_qba.remove(i+1, 1);
                    end_idx--;
                }else{
                    replace_success_flag = false;
                    break;
                }
            }
        }
    }

    return replace_success_flag;
}

bool SellRobotClient::sendCmdToServer(const quint16 cmd_code, const QString data_str)
{
    char high_byte, low_byte, check_byte;
    QByteArray send_qba;
    //开始标志位
    send_qba.append(0x7e);

    //消息头：命令码+加密方式+消息体长度
    low_byte = cmd_code;
    high_byte = cmd_code >> 8;
    send_qba.append(low_byte);			//命令码，低字节在前，高字节在后
    send_qba.append(high_byte);
    send_qba.append(char(0x00));
    QByteArray data_qba = data_str.toUtf8();		//消息体转换成utf8
    quint16 data_len = data_qba.size();
    low_byte = data_len;
    high_byte = data_len >> 8;
    send_qba.append(low_byte);					//消息体长度共2字节，低字节在前
    send_qba.append(high_byte);

    //消息体
    send_qba.append(data_qba);

    //校验码：消息头+消息体校验
    check_byte = calculateCheckCode(send_qba, 1, send_qba.size() - 1);
    send_qba.append(check_byte);

    //结束标志
    send_qba.append(0x7e);

    //替换消息头和消息体中的特殊字符0x7d, 0x7e
    replaceSymbolInSendData(send_qba, 1, send_qba.size() - 2);

    //发送封装好的命令给服务器
    int wr_len = 0;
    if(robot_socket_p != NULL)
        wr_len = robot_socket_p->write(send_qba);

    if(wr_len > 0)
        return true;
    else
        return false;
}

void SellRobotClient::processSocketData(QList<QByteArray> &qba_list)
{
    while(qba_list.size() > 0){

        QByteArray qba = qba_list.front();
        /*qDebug() << "show all data arr...";
        for(int i = 0; i < qba.size(); i++){
            qDebug("i:%d, data:%d", i, qba.at(i));
        }*/

        replaceSymbolInRecData(qba, 0, qba.size() - 1);

        //qDebug() << "socket data after replace QbyteArray data:" << qba.toHex();
        quint16 cmd_code = 0x00;
        unsigned char low_byte = qba.at(0);
        unsigned char high_byte = qba.at(1);
        cmd_code |= high_byte;
        cmd_code = cmd_code << 8;
        cmd_code |= low_byte;
        qDebug("cmd_code:%x", cmd_code);

        //char encrypt_code = qba.at(2);

        quint16 data_len = 0x00;
        low_byte = qba.at(3);
        high_byte = qba.at(4);
        data_len |= high_byte;
        data_len = data_len << 8;
        data_len |= low_byte;
        //qDebug() << "data_len:" << data_len << ", qba size:" << qba.size();
        if(data_len != qba.size() - 6){
            qDebug() << "rec msg data len err";
            qba_list.pop_front();
            continue;
        }

        unsigned char chk_code = qba.at(qba.size() - 1);
        unsigned char cal_chk_code = calculateCheckCode(qba, 0, qba.size()-2);
        //qDebug("rec_chk_code:%x", chk_code);
        //qDebug("cal_chk_code:%x", cal_chk_code);

        if(chk_code != cal_chk_code){	//检查校验码
            qDebug() << "proc msg chk code err";
            qba_list.pop_front();
            continue;
        }

        QByteArray data_qba = qba.mid(5, qba.size() - 1 - 5); //提取消息体，去掉消息头和检验码

        QString data_str = QString::fromUtf8(data_qba);
        qDebug() << "msg data json:" << data_str;

        processServerCmd(cmd_code, data_qba);

        //从列表中清除已处理过的消息
        qba_list.pop_front();
    }
}

void SellRobotClient::processServerCmd(const quint16 cmd_code, const QByteArray &data_qba)
{
    switch (cmd_code) {

    case REGISTER_SELLROBOT_CMD:    //注册确认

        break;

    case REC_HEART_BEAT_SELLROBOT_CMD:    //心跳命令
        heart_beat_flag = true;
        break;

    case MOVE_TO_DES_POS_CMD:    //移动到目标点命令
        moveToDesPos(data_qba);
        break;

    case STOP_MOVE_CMD:
        //qDebug() << "rec stop move";
        //move_base_p->stopMove();
        pauseOrContinueMove(data_qba);
        break;

    case GET_MAP_POS_CMD:
        sendMapPosToServer();
        break;

    case START_DOCK_CHARGE_CMD:
        startDockChargeStation(data_qba);
        break;
    default:
        break;
    }
}

bool SellRobotClient::sendPosMsgToServer()
{   
    int speed = 300; //mm/s
    QString start_pos_name;
    QString end_pos_name;
    int percent = 0;

    //判断是否到达终点
    //int path_size = des_pos_list.size();
    if(move_base_state.curr_des_idx <= -1 && move_base_state.run_state == 3 && is_moving_flag){
        is_moving_flag = false;
        send_pos_timer_p->stop();

        qDebug() << "arrive final pos!";
        if(pos_list_qs.size() > 0){
            sendArriveMsgToServer(pos_list_qs.at(pos_list_qs.size() - 1));
        }else{
            qWarning() << "des pos list size <= 0, not send final arrive msg";
        }

        des_pos_list.clear();
        pos_list_qs.clear();

        return true;
    }

    int path_size = des_pos_list.size();
    if(move_base_state.curr_des_idx >= path_size || move_base_state.curr_des_idx < 0){
        qDebug("curr patral idx over path list size");
        return false;
    }

    int last_des_idx = move_base_state.curr_des_idx - 1;
    if(last_des_idx < 0){
        start_pos_name = pos_list_qs.at(0);
        if(pos_list_qs.size() > 1){
            end_pos_name = pos_list_qs.at(1);
        }else{
            end_pos_name = "";
            qDebug() << "pos list size < 2";
            return false;
        }
        percent = 0;    //还没到路线起点，百分比为0
        qDebug() << "first pos";
    }else{
        start_pos_name = pos_list_qs.at(last_des_idx);
        end_pos_name = pos_list_qs.at(last_des_idx+1);
        RobotPose start_pos = des_pos_list.at(last_des_idx);
        RobotPose end_pos = des_pos_list.at(last_des_idx+1);
        float dis_start = calTwoPointDis(move_base_state.robot_map_pose, start_pos);
        float dis_end = calTwoPointDis(move_base_state.robot_map_pose, end_pos);

        percent = ( dis_start / (dis_start + dis_end) ) * 100;

        qDebug() << "after pos";
    }

    qDebug() << "start_pos:" << start_pos_name << ", end_pos:" << end_pos_name << ", percent:" << percent;


    QJsonObject json_data
    {
        {"carOneSpeed", speed},
        {"carOneStartPosName", start_pos_name},
        {"carOneEndPosName", end_pos_name},
        {"carOnePosPercent", percent}
    };

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    //qDebug() << "send pos msg to server json data:" << json_qba;

    //bool send_flag = true;
    bool send_flag = sendCmdToServer(SEND_POS_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::sendArriveOnePointMsgToServer()
{
    int speed = 300;
    QString start_pos_name, end_pos_name;
    int percent = 100;

    int arrive_des_idx = move_base_state.curr_des_idx - 1;
    if(arrive_des_idx > 0){

        start_pos_name = pos_list_qs.at(arrive_des_idx - 1);
        end_pos_name = pos_list_qs.at(arrive_des_idx);

    }else{
        //判断是否到达路径终点
        if(move_base_state.curr_des_idx == - 1 && move_base_state.run_state == 3){
            is_moving_flag = false;
            send_pos_timer_p->stop();

            qDebug() << "sendArriveOnePointMsgToServer arrive final pos!";
            if(pos_list_qs.size() > 0){
                sendArriveMsgToServer(pos_list_qs.at(pos_list_qs.size() - 1));
            }else{
                qWarning() << "des pos list size <= 0, not send final arrive msg";
            }

            des_pos_list.clear();
            pos_list_qs.clear();

            return true;
        }else{
            qDebug() << "sendArriveOnePointMsgToServer arrive path idx <= 0, not send msg";
            return false;
        }
    }

    qDebug() << "start_pos:" << start_pos_name << ", end_pos:" << end_pos_name << ", percent:" << percent;


    QJsonObject json_data
    {
        {"carOneSpeed", speed},
        {"carOneStartPosName", start_pos_name},
        {"carOneEndPosName", end_pos_name},
        {"carOnePosPercent", percent}
    };

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    //qDebug() << "send pos msg to server json data:" << json_qba;

    //bool send_flag = true;
    bool send_flag = sendCmdToServer(SEND_POS_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::sendArriveMsgToServer(QString pos_name)
{
    QString arr_pos_name = pos_name;
    QJsonObject json_data
    {
        {"taskFinished", true},
        {"reachedPosName", arr_pos_name}
    };

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    qDebug() << "send arrive msg to server json data:" << json_qba;

    bool send_flag = sendCmdToServer(ARRIVE_DES_POS_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::moveToDesPos(const QByteArray &data_qba)
{
    qDebug() << "rec server msg move to des pos json str:" << data_qba;

    if(is_moving_flag)
        move_base_p->stopMove();

    QJsonObject obj;

    bool parse_flag = parseJsonFromQByteArray(obj, data_qba);

    if(!parse_flag){
        qWarning() << "parse json fail, return!";
        return false;
    }

    if(!dock_proc_finish_flag){
        //对接充电坐过程未完成，忽略命令
        qDebug() << "dock charge not finish, cancel move cmd!";
        return false;
    }

    pos_list_qs.clear();
    des_pos_list.clear();

    if(obj.contains("carOneGoalPosName")){
        QJsonValue jval = obj.value("carOneGoalPosName");
        if(jval.isArray()){
            QJsonArray jarr = jval.toArray();
            for(int i = 0; i < jarr.size(); i++){
                QJsonValue jval = jarr.at(i);
                if(jval.isObject()){
                    QJsonObject jobj = jval.toObject();
                    if(jobj.contains("posName")){
                        QString pos_name = jobj.value("posName").toString();
                        pos_list_qs.push_back(pos_name);
                    }
                    RobotPose pos;
                    if(jobj.contains("X")){
                       pos.x = jobj.value("X").toDouble();
                    }
                    if(jobj.contains("Y")){
                       pos.y = jobj.value("Y").toDouble();
                    }
                    if(jobj.contains("Z")){
                       pos.z = jobj.value("Z").toDouble();
                    }
                    des_pos_list.push_back(pos);
                }
            }
        }
    }



    if(pos_list_qs.size() <= 0 || des_pos_list.size() <= 0){
        qWarning() << "moveToDesPos but parse pose list size <= 0, return false";
        return false;
    }

    if(pos_list_qs.size() != des_pos_list.size()){
        qWarning() << "moveToDesPos but name list size != pose list size, return false";
        return false;
    }


    curr_patral_id = 0;


    //当前如果处于充电状态，先退出充电坐
    if(movebase_state_code == MOVEBASE_STATE_CHARGING){
        movebase_state_code = MOVEBASE_STATE_RUN;

        //unDockChargeStation();

        check_charge_state_timer_p->start(50);

        qDebug() << "robot now is in charging, start undock charge station!";

        return true;
    }


//    for(int i = 0; i < pos_list_qs.size(); i++){
//        RobotPose pos = des_pos_list.at(i);
//        qDebug() << "name:" << pos_list_qs.at(i) << ",pos:" << pos.x << "," << pos.y << "," << pos.z;
//    }



    //开始沿路线移动

    move_base_p->startMove(des_pos_list, 0, 0, 0, 0);

    is_moving_flag = true;
    if(des_pos_list.size() > 1){
        send_pos_timer_p->start(1500);  //开始移动后开启发送当前位置定时器
    }

    return true;
}

bool SellRobotClient::pauseOrContinueMove(const QByteArray &data_qba)
{
    QJsonObject obj;

    bool parse_flag = parseJsonFromQByteArray(obj, data_qba);
    if(!parse_flag){
        qWarning() << "parse json fail, return!";
        return false;
    }

    bool stop_flag;
    parse_flag = false;
    if(obj.contains("stopMove")){
        QJsonValue jval = obj.value("stopMove");
        if(jval.isBool()){
            stop_flag = jval.toBool();
            parse_flag = true;
        }
    }

    if(parse_flag){
        if(stop_flag){
            //暂停运行
            is_moving_flag = false;
            move_base_p->stopMove();
            send_pos_timer_p->stop();
            qDebug() << "pause move!";
        }else{
            //继续运行
            if(is_moving_flag){
                qWarning() << "continue move fail because now is in moving state";
                return false;
            }

            if(des_pos_list.size() > 0 && curr_patral_id < des_pos_list.size() && curr_patral_id >= 0){
                qDebug() << "continue move!move des patral id:" << curr_patral_id;
                move_base_p->startMove(des_pos_list, 0, 0, 0, curr_patral_id);
                if(des_pos_list.size() > 1){
                    send_pos_timer_p->start(1500);
                }
                is_moving_flag = true;
            }else{
                qWarning() << "continue move but des_pose list is empty or curr_patral_id over list size!";

            }
        }
    }else{
        qWarning() << "rec stop cmd but parse data fail";
    }
}

bool SellRobotClient::getPosByPosName(RobotPose &pos, const QString pos_name)
{
    bool get_flag = true;

//    if(pos_name == "实验室"){
//        pos = RobotPose(7.8, 3.3, 0);
//    }else if(pos_name == "商会"){
//        pos = RobotPose(7.5, 9.4, 0);
//    }else if(pos_name == "会议室"){
//        pos = RobotPose(7, 16.3, 0);
//    }else if(pos_name == "财务室"){
//        pos = RobotPose(0.64, 18.4, 0);
//    }else if(pos_name == "蔡总办公室"){
//        pos = RobotPose(1.8, 20.2, 0);
//    }else if(pos_name == "庄总办公室"){
//        pos = RobotPose(4.3, 19.4, 0);
//    }else if(pos_name == "研发部办公区"){
//        pos = RobotPose(18, 20, 0);
//    }else if(pos_name == "小蔡总办公室"){
//        pos = RobotPose(27, 20.6, 0);
//    }else if(pos_name == "公司前台"){
//        pos = RobotPose(29.1, 20.5, 0);
//    }else if(pos_name == "数据中心"){
//        pos = RobotPose(32.7, 17.4, 0);
//    }else if(pos_name == "工程部"){
//        pos = RobotPose(31.3, -4, 0);
//    }else if(pos_name == "商管部"){
//        pos = RobotPose(31.7, -7.8, 0);
//    }else if(pos_name == "其他公司办公区域"){
//        pos = RobotPose(22.5, -9, 0);
//    }else if(pos_name == "电梯通道"){
//        pos = RobotPose(20.7, 10.6, 0);
//    }else if(pos_name == "男厕所"){
//        pos = RobotPose(8.8, 16.3, 0);
//    }else if(pos_name == "女厕所"){
//        pos = RobotPose(31.3, -2, 0);
//    }else{
//        get_flag = false;
//    }

    if(pos_name == "擎谱武道馆"){
        pos = RobotPose(0, 0, 0);
    }else if(pos_name == "木木茶"){
        pos = RobotPose(13.2, -1.5, 0);
    }else if(pos_name == "小派的任意门"){
        pos = RobotPose(22.7, -1.0, 0);
    }else{
        get_flag = false;
    }

    return get_flag;
}

float SellRobotClient::calTwoPointDis(RobotPose &point1, RobotPose &point2)
{
    float dis = std::sqrt( (point1.x - point2.x) * (point1.x - point2.x) + ( point1.y - point2.y ) * ( point1.y - point2.y ) );
    return dis;
}

bool SellRobotClient::sendRegisterMsg()
{
    QString reg_code_qs = SELL_ROBOT_ID;
    QJsonObject json_data
    {
        {"registerCode", reg_code_qs}
    };

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    //qDebug() << "send register msg to server json data:" << json_qba;

    bool send_flag = sendCmdToServer(REGISTER_SELLROBOT_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::sendHeartBeatMsg()
{
    QString heart_code_qs = SELL_ROBOT_ID;
    QJsonObject json_data
    {
        {"machineId", heart_code_qs}
    };

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    bool send_flag = sendCmdToServer(SEND_HEART_BEAT_SELLROBOT_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::sendMapPosToServer()
{
    get_movebase_state_flag = false;
    move_base_p->getMoveBaseState();

    int out_tms = 200;
    QTime curr_t = QTime::currentTime().addMSecs(out_tms);
    while(!get_movebase_state_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    if(!get_movebase_state_flag){
        qWarning() << "send map pos to server but get movebase state fail!";
        return false;
    }

    float x = move_base_state.robot_map_pose.x;
    float y = move_base_state.robot_map_pose.y;
    float z = move_base_state.robot_map_pose.z;


    QJsonObject json_data;

    json_data.insert("X", x);
    json_data.insert("Y", y);
    json_data.insert("Z", z);

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    bool send_flag = sendCmdToServer(SEND_MAP_POS_CMD, json_qba);

    return send_flag;

}

bool SellRobotClient::sendMovebaseStateCodeToServer(int state_code)
{
    QJsonObject json_data;
    json_data.insert("MovebaseStateCode", state_code);

    QJsonDocument json_doc(json_data);

    QByteArray json_qba = json_doc.toJson(QJsonDocument::Compact);

    bool send_flag = sendCmdToServer(SEND_MOVEBASE_STATE_CMD, json_qba);

    return send_flag;
}

bool SellRobotClient::startDockChargeStation(const QByteArray &data_qba)
{
    QJsonObject obj;

    bool parse_flag = parseJsonFromQByteArray(obj, data_qba);

    if(!parse_flag){
        qWarning() << "parse json fail, return!";
        return false;
    }

    if(obj.contains("X")){
        QJsonValue jval = obj.value("X");
        charge_pos.x = jval.toDouble();
    }
    if(obj.contains("Y")){
        QJsonValue jval = obj.value("Y");
        charge_pos.y = jval.toDouble();
    }
    if(obj.contains("Z")){
        QJsonValue jval = obj.value("Z");
        charge_pos.z = jval.toDouble();
    }

    qDebug() << "rec charge pos, x:" << charge_pos.x << ", y:" << charge_pos.y;

    movebase_state_code = MOVEBASE_STATE_DOCK_CHARGE;
    dock_proc_finish_flag = false;
    check_charge_state_timer_p->start(100);
}

void SellRobotClient::dockChargeStation()
{
    /*
    QList<RobotPose> des_pos_list;
    des_pos_list.push_back(charge_pos);

    movebase_state_code = MOVEBASE_STATE_NAV_CHARGE_POS;   //前往充电点
    sendMovebaseStateCodeToServer(movebase_state_code);

    arrive_one_pos_flag = false;
    move_base_p->startMove(des_pos_list, 0, 0, 0, 0);

    int out_tms = wait_time_sec * 1000;
    QTime curr_t = QTime::currentTime().addMSecs(out_tms);
    while(!arrive_one_pos_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    if(!arrive_one_pos_flag){
        //超时未到达充电点
        movebase_state_code = MOVEBASE_STATE_CHARGE_FAIL;
        sendMovebaseStateCodeToServer(movebase_state_code);
        return;
    }
    */


    movebase_state_code = MOVEBASE_STATE_DOCK_CHARGE;   //开始对接
    //sendMovebaseStateCodeToServer(movebase_state_code);
    get_movebase_state_flag = false;
    move_base_p->startDockChargeStation();

    //等待对接结果
    QTime curr_t = QTime::currentTime().addMSecs(60000);
    while(!get_movebase_state_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    bool charge_dock_flag = false;
    bool det_vol_flag = true;
    switch (move_base_state.run_state) {
    case 30:
        //初始搜索未找到充电站
        qDebug() << "first search not find charge station";
        break;
    case 31:
        //充电对接转动过程中检测到碰撞触发
        qDebug() << "rot and det imp sensor act,stop dock";
        break;
    case 32:
        //充电对接前进过程中检测到碰撞触发
        qDebug() << "move and det imp sensor act, stop dock";
        break;
    case 33:
        //直线前进到充电坐距离超过限制距离
        qDebug() << "move dis too big, stop dock";
        break;
    case 34:
        //后退对接失败
        qDebug() << "move back dock fail";
        break;
    case 35:
        //对接完成
        qDebug() << "dock finish, det charge vol";
        charge_dock_flag = true;
        break;
    case 36:
        //
        qDebug() << "dock finish, not det charge vol";
        charge_dock_flag = true;
        //det_vol_flag = false;
    default:
        break;
    }

    if(!charge_dock_flag){
        //对接失败，导航到充电点，重新开始对接流程
        movebase_state_code = MOVEBASE_STATE_DOCK_FAIL;
        //sendMovebaseStateCodeToServer(movebase_state_code);
        return;
    }

    //对接成功，检测充电电压接入状态，未检测到充电电压接入，导航到充电点，重新开始对接流程
    if(det_vol_flag){
        //检测到充电电压接入，正在充电，充电对接完成，等待充电完成
        movebase_state_code = MOVEBASE_STATE_CHARGING;
    }else{
        movebase_state_code = MOVEBASE_STATE_NOT_DET_VOL;
    }

    //sendMovebaseStateCodeToServer(movebase_state_code);

    return;
}

void SellRobotClient::moveToChargePosAndDock(RobotPose &charge_pos, int wait_time_sec)
{

    QList<RobotPose> des_pos_list;
    des_pos_list.push_back(charge_pos);

    movebase_state_code = MOVEBASE_STATE_NAV_CHARGE_POS;   //前往充电点
    //sendMovebaseStateCodeToServer(movebase_state_code);

    arrive_one_pos_flag = false;
    move_base_p->startMove(des_pos_list, 0, 0, 0, 0);

    int out_tms = wait_time_sec * 1000;
    QTime curr_t = QTime::currentTime().addMSecs(out_tms);
    while(!arrive_one_pos_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    if(!arrive_one_pos_flag){
        //超时未到达充电点
        movebase_state_code = MOVEBASE_STATE_CHARGE_FAIL;
        //sendMovebaseStateCodeToServer(movebase_state_code);
        return;
    }

    movebase_state_code = MOVEBASE_STATE_DOCK_CHARGE;   //开始对接
    //sendMovebaseStateCodeToServer(movebase_state_code);
    get_movebase_state_flag = false;
    move_base_p->startDockChargeStation();

    //等待对接结果
    curr_t = QTime::currentTime().addMSecs(60000);
    while(!get_movebase_state_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    bool charge_dock_flag = false;
    bool det_vol_flag = true;
    switch (move_base_state.run_state) {
    case 30:
        //初始搜索未找到充电站
        qDebug() << "first search not find charge station";
        break;
    case 31:
        //充电对接转动过程中检测到碰撞触发
        qDebug() << "rot and det imp sensor act,stop dock";
        break;
    case 32:
        //充电对接前进过程中检测到碰撞触发
        qDebug() << "move and det imp sensor act, stop dock";
        break;
    case 33:
        //直线前进到充电坐距离超过限制距离
        qDebug() << "move dis too big, stop dock";
        break;
    case 34:
        //后退对接失败
        qDebug() << "move back dock fail";
        break;
    case 35:
        //对接完成
        qDebug() << "dock finish, det charge vol";
        charge_dock_flag = true;
        break;
    case 36:
        //
        qDebug() << "dock finish, not det charge vol";
        charge_dock_flag = true;
        //det_vol_flag = false;
    default:
        break;
    }

    if(!charge_dock_flag){
        //对接失败，导航到充电点，重新开始对接流程
        movebase_state_code = MOVEBASE_STATE_DOCK_FAIL;
        //sendMovebaseStateCodeToServer(movebase_state_code);
        return;
    }

    //对接成功，检测充电电压接入状态，未检测到充电电压接入，导航到充电点，重新开始对接流程
    if(det_vol_flag){
        //检测到充电电压接入，正在充电，充电对接完成，等待充电完成
        movebase_state_code = MOVEBASE_STATE_CHARGING;
    }else{
        movebase_state_code = MOVEBASE_STATE_NOT_DET_VOL;
    }

    return;
}

bool SellRobotClient::unDockChargeStation()
{
   bool move_flag = move_base_p->controlMoveAndWaitConfirm(0.2, 0, 3);
   return move_flag;
}


void SellRobotClient::getMoveBaseRunStateSlot(QVariant state_var)
{

    move_base_state = state_var.value<moveBase_state_stru>();
    get_movebase_state_flag = true;
    qDebug("run_state:%d, curr_idx:%d, bat_vol:%f, map pose x:%f, y:%f, z:%f", move_base_state.run_state, move_base_state.curr_des_idx, move_base_state.bat_vol, move_base_state.robot_map_pose.x, move_base_state.robot_map_pose.y, move_base_state.robot_map_pose.z);
    //return;
    switch (move_base_state.run_state) {
    case 0:
        //停止状态

        break;
    case 1:
        //正在运行状态

        break;
    case 2:
        //到达巡逻点，如果巡逻模式为到达等待模式，则等待继续信号再继续巡逻
        curr_patral_id = move_base_state.curr_des_idx;  //保存当前巡逻目标点序号，暂停巡逻后从当前点开始巡逻
        break;
    case 3:
        //到达目标点
        arrive_one_pos_flag = true;
        curr_patral_id = move_base_state.curr_des_idx;
        sendArriveOnePointMsgToServer();
        break;
    case 4:
        //前方通过空间太小

        break;
    case 5:
        //多次搜索路径失败，取消当前目标点

        break;
    case 6:
        //指定的目标点搜索路径失败

        break;
    case 7:
        //外部取消运行

        break;
    case 10:
        //碰撞传感器触发

        break;
    case 11:
        //超声波传感器检测到障碍

        break;
    case 12:
        //底部避障激光雷达检测到障碍

        break;
    case 13:
        //导航激光雷达检测到障碍

        break;
    case 14:
        //深度摄像头检测到障碍

        break;
    case 15:
        //控制移动成功

        break;
    case 16:
        //控制移动失败

        break;
    case 17:
        //检测到里程坐标值变化超过最大值
        qDebug() << "odom pose change over limit val, stop move!";
        break;
    case 18:
        //检测到地图坐标变化值超过最大值
        qDebug() << "map pose change over limit val, stop move!";
        break;
    default:
        break;
    }
}

void SellRobotClient::getTcpSocketErrSlot(QAbstractSocket::SocketError err)
{
    qDebug() <<  "tcp socket err:" << robot_socket_p->errorString();
    if(err == QAbstractSocket::HostNotFoundError){
        //qDebug() << "tcp socket err: HostNotFoundError!";
        //isTcpSocketConnect_flag = false;
    }else if(err == QAbstractSocket::RemoteHostClosedError){
        //qDebug() << "tcp socket err: RemoteHostClosedError, reConnectTimes:" << reConnect_times;
        //isTcpSocketConnect_flag = false;
    }else if(err == QAbstractSocket::NetworkError){
        //qDebug() << "tcp socket err: NetworkError!";
        //isTcpSocketConnect_flag = false;
    }
}

void SellRobotClient::getSocketStateChangeSlot(QAbstractSocket::SocketState socketState)
{
    //QAbstractSocket::ConnectingState, QAbstractSocket::ClosingState

    if(socketState == QAbstractSocket::UnconnectedState){
        qCritical() << "tcp socket unconnect!";

        socket_connect_flag = false;
        //register_robot_flag = false;

    }else if(socketState == QAbstractSocket::ConnectedState){

        qDebug() << "tcp socket has connect!";
        socket_connect_flag = true;

    }
}

void SellRobotClient::getTcpSocketDisconnectSlot()
{

}

void SellRobotClient::getTcpSocketDataSlot()
{
    static QByteArray rec_qba;

    while(robot_socket_p->bytesAvailable() > 0){
        rec_qba.append(robot_socket_p->readAll());
    }

    //qDebug("rec_data size:%d", rec_qba.size());
    //从接收到的数据中按消息标志头和尾提取出一个原始消息，保存到消息队列中
    int st_pos = -1 ,end_pos = -1;
    for(int i = 0; i < rec_qba.size(); i++){
        if(rec_qba.at(i) == '~'){
            if(st_pos == -1){
                st_pos = i;
            }else{
                end_pos = i;
                int len = end_pos - st_pos - 1;			//两个标志位之间的数据字节数
                QByteArray data_qba = rec_qba.mid(st_pos+1, len);
                rec_qba_list << data_qba;
                rec_qba.remove(0, end_pos+1);			//清除已提取过数据的部分
                i = -1;
                st_pos = -1;
            }
        }
    }

    if(rec_qba_list.size() > 0){
        //emit recSocketDataSignal();
        processSocketData(rec_qba_list);
    }
}

void SellRobotClient::heartBeatTimerSlot()
{
    static int check_heartBeat_count, reconnect_times;

    if(!heart_beat_flag){
        check_heartBeat_count++;
        if(check_heartBeat_count >= 3){
            if(is_moving_flag){
                is_moving_flag = false;
                move_base_p->stopMove();
                send_pos_timer_p->stop();
                qDebug() << "det server disconnect, pause move!";
            }
            qDebug() << "re connect server!";
            check_heartBeat_count = 0;
            reconnect_times++;
            heartBeat_timer_p->stop();
            if(reconnect_times <= 3){
                bool conn_flag = connectToServer();
            }else{
                //如果连续重连超过3次还未能成功连接，则变为10秒连接一次
                if(reconnect_times % 10 == 0){
                    bool conn_flag = connectToServer();
                }
            }
            heartBeat_timer_p->start(1000);
        }
    }

    heart_beat_flag = false;
    bool send_flag = sendHeartBeatMsg();

//    if(!send_flag || !socket_connect_flag){
//        //心跳发送失败，与服务器连接断开
//        qDebug() << "re connect server!";
//        reconnect_times++;
//        heartBeat_timer_p->stop();
//        if(reconnect_times <= 3){
//            bool conn_flag = connectToServer();
//        }else{
//            //如果连续重连超过3次还未能成功连接，则变为10秒连接一次
//             if(reconnect_times % 10 == 0){
//                 bool conn_flag = connectToServer();
//             }
//        }
//        heartBeat_timer_p->start(1000);
//    }

}

void SellRobotClient::sendPosTimerSlot()
{
    //qDebug() << "send get movebase state msg...";
    get_movebase_state_flag = false;
    move_base_p->getMoveBaseState();

    int out_tms = 500;
    QTime curr_t = QTime::currentTime().addMSecs(out_tms);
    while(!get_movebase_state_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    if(!get_movebase_state_flag){
        qDebug() << "send pos slot get movebase state fail";
        return;
    }else{
        //qDebug() << "send pos slot get movebase state sucess!";
    }

    sendPosMsgToServer();

}

void SellRobotClient::checkChargeStateTimerSlot()
{
    //static int get_movebase_state_fail_count;
    //static int not_det_input_charge_vol_count;
    //float charge_finish_vol = 28.5;

    if(movebase_state_code == MOVEBASE_STATE_DOCK_CHARGE){
        //开始对接
        check_charge_state_timer_p->stop();
        sendMovebaseStateCodeToServer(movebase_state_code);
        qDebug() << "start dock!";
        dockChargeStation();
        //对接失败
        if(movebase_state_code != MOVEBASE_STATE_CHARGING){
            int dock_count = 0;
            //重新移动到充电点，再次对接，再次对接失败超过5次，则整个充电失败
            while(dock_count < 5){
                dock_count++;
                qDebug() << "try rec dock!";
                moveToChargePosAndDock(charge_pos, 30);
                if(movebase_state_code == MOVEBASE_STATE_CHARGING){
                    break;
                }
            }
        }

        dock_proc_finish_flag = true;

        sendMovebaseStateCodeToServer(movebase_state_code);
        if(movebase_state_code != MOVEBASE_STATE_CHARGING){
            //当前不是处于充电状态
            qDebug() << "dock and charge fail!";
            return;
        }else{
            //对接成功，处于充电状态
            qDebug() << "dock and charge,robot in charging state!";
            move_base_p->ctrlRelay(true);
            return;
        }
    }else if(movebase_state_code == MOVEBASE_STATE_RUN){

        check_charge_state_timer_p->stop();

        move_base_p->ctrlRelay(false);

        move_base_p->delayMs(2000);

        unDockChargeStation();

        //开始沿路线移动

        move_base_p->startMove(des_pos_list, 0, 0, 0, 0);

        is_moving_flag = true;
        if(des_pos_list.size() > 1){
            send_pos_timer_p->start(1500);  //开始移动后开启发送当前位置定时器
        }

    }



    /*

    if(movebase_state_code != MOVEBASE_STATE_CHARGING){
        //当前不是处于充电状态
        qDebug() << "not in charging state,return!";
        return;
    }

    //获取底盘状态
    get_movebase_state_flag = false;
    move_base_p->getMoveBaseState();

    int out_tms = 300;
    QTime curr_t = QTime::currentTime().addMSecs(out_tms);
    while(!get_movebase_state_flag){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(QTime::currentTime() > curr_t){
            break;
        }
    }

    if(!get_movebase_state_flag){
        //获取底盘状态失败
        get_movebase_state_fail_count++;
        qDebug() << "get_movebase state fail, count:" << get_movebase_state_fail_count;
    }else{
        get_movebase_state_fail_count = 0;

        //检测充电电压接入状态,未检测到充电接入，则后退0.2压紧接触电极，多次未检测到充电电压，则充电失败
        //todo

        //检测充电是否完成
        if(move_base_state.bat_vol > charge_finish_vol){
            //movebase_state_code = MOVEBASE_STATE_IDLE;
        }
    }

    if(get_movebase_state_fail_count > 5){
        qDebug() << "get movebase state fail count > 5";
    }

    */
}
