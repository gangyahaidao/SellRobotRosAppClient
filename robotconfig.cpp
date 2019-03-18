#include "robotconfig.h"

QString program_config_path = "./config/act_config.txt";    //动作脚本路径
QString local_config_path = "./config/local_config.txt";    //本地配置路径
QString patral_config_path = "./config/patral_config.txt";  //巡逻配置路径


RobotConfig::RobotConfig(QObject *parent) : QObject(parent)
{

}

void RobotConfig::test()
{
    bool rd_flag = readPatralConfig();

    if(rd_flag){
        qDebug("patral intervel:%d, arrive_mode:%d, patral_mode:%d, rot_des_angle_mode:%d", patral_config.patral_interval, patral_config.arrive_mode, patral_config.patral_mode, patral_config.rot_des_angle_code);
        qDebug() << "run_act_type:" << patral_config.run_act_type;
        for(int i = 0; i < patral_config.path_list.size(); i++){
            QString act_type = patral_config.path_list.at(i).act_type;
            RobotPose pt = patral_config.path_list.at(i).path_point;
            qDebug() << "act_type:" << act_type;
            qDebug("x:%f, y:%f, z:%f", pt.x, pt.y, pt.z);
        }
    }
}

bool RobotConfig::readProgramConfig()
{
    QJsonObject json_obj;
    bool rd_flag = readJsonFile(program_config_path, json_obj);
    if(rd_flag){
        program_config_list.clear();
        bool tr_flag = tranfJsonObjToProgramConfigList(json_obj, program_config_list);
        if(tr_flag){
            return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}

bool RobotConfig::readLocalConfig()
{
    QJsonObject json_obj;

    bool rd_flag = readJsonFile(local_config_path, json_obj);

    if(!rd_flag){
        qDebug() << "read local config file fail!";
        return false;
    }

    server_ip.clear();
    servo_port.clear();
    xfei_port.clear();
    server_port = 0;
    client_port = 0;

    if(json_obj.contains("server_ip")){
        QJsonValue jval = json_obj.value("server_ip");
        if(jval.isString()){
            server_ip = jval.toString();
        }
    }

    if(json_obj.contains("server_port")){
        QJsonValue jval = json_obj.value("server_port");
        if(jval.isDouble()){
            server_port = jval.toDouble();
        }
    }

    if(json_obj.contains("servo_port")){
        QJsonValue jval = json_obj.value("servo_port");
        if(jval.isString()){
            servo_port = jval.toString();
        }
    }

    if(json_obj.contains("client_port")){
        QJsonValue jval = json_obj.value("client_port");
        if(jval.isDouble()){
            client_port = jval.toDouble();
        }
    }

    if(json_obj.contains("xfei_port")){
        QJsonValue jval = json_obj.value("xfei_port");
        if(jval.isString()){
            xfei_port = jval.toString();
        }
    }

    if(server_port != 0 && client_port != 0 && !server_ip.isEmpty() && !servo_port.isEmpty() && !xfei_port.isEmpty()){
        return true;
    }else{
        qDebug() << "parse local config file err!";
        return false;
    }
}

bool RobotConfig::readPatralConfig()
{
    QJsonObject json_obj;

    bool rd_flag = readJsonFile(patral_config_path, json_obj);

    if(!rd_flag){
        qDebug() << "read patral config file fail!";
        return false;
    }

    //清除路线信息
    patral_config.path_list.clear();

    if(json_obj.contains("patral_interval")){
        QJsonValue jval = json_obj.value("patral_interval");
        if(jval.isDouble()){
            patral_config.patral_interval = jval.toDouble();
        }
    }

    if(json_obj.contains("run_act_type")){
        QJsonValue jval = json_obj.value("run_act_type");
        if(jval.isString()){
             patral_config.run_act_type = jval.toString();
        }
    }

    if(json_obj.contains("stop_act_type")){
        QJsonValue jval = json_obj.value("stop_act_type");
        if(jval.isString()){
             patral_config.stop_act_type = jval.toString();
        }
    }

    if(json_obj.contains("block_act_type")){
        QJsonValue jval = json_obj.value("block_act_type");
        if(jval.isString()){
             patral_config.block_act_type = jval.toString();
        }
    }

    if(json_obj.contains("patral_mode")){
        QJsonValue jval = json_obj.value("patral_mode");
        if(jval.isDouble()){
            patral_config.patral_mode = jval.toDouble();
        }
    }

    if(json_obj.contains("arrive_mode")){
        QJsonValue jval = json_obj.value("arrive_mode");
        if(jval.isDouble()){
            patral_config.arrive_mode = jval.toDouble();
        }
    }

    if(json_obj.contains("rot_des_angle_code")){
        QJsonValue jval = json_obj.value("rot_des_angle_code");
        if(jval.isDouble()){
            patral_config.rot_des_angle_code = jval.toDouble();
        }
    }

    if(json_obj.contains("path_list")){
        QJsonValue jval = json_obj.value("path_list");
        if(jval.isArray()){
            QJsonArray jarr = jval.toArray();
            for(int i = 0; i < jarr.size(); i++){
                QJsonValue jval = jarr.at(i);
                if(jval.isObject()){
                    struct patral_point_stru patral_point;
                    QJsonObject jobj = jval.toObject();
                    if(jobj.contains("act_type")){
                        QJsonValue jval = jobj.value("act_type");
                        if(jval.isString()){
                            patral_point.act_type = jval.toString();
                        }
                    }
                    if(jobj.contains("path_point")){
                        QJsonValue jval = jobj.value("path_point");
                        if(jval.isObject()){
                            QJsonObject jobj = jval.toObject();
                            if(jobj.contains("x")){
                                QJsonValue jval = jobj.value("x");
                                patral_point.path_point.x = (jval.toDouble());
                            }
                            if(jobj.contains("y")){
                                QJsonValue jval = jobj.value("y");
                                patral_point.path_point.y = (jval.toDouble());
                            }
                            if(jobj.contains("z")){
                                QJsonValue jval = jobj.value("z");
                                patral_point.path_point.z = (jval.toDouble());
                            }
                        }
                    }

                    patral_config.path_list.push_back(patral_point);
                }
            }
        }
    }

    return true;

}

bool RobotConfig::saveJsonObjAsFile(QString file_path, QJsonObject &json_obj)
{
    if(json_obj.empty()){
        qDebug() << "json obj is empty!";
        return false;
    }

    QJsonDocument json_doc(json_obj);

    QByteArray qba = json_doc.toJson();

    QFile json_file(file_path);

    if(!json_file.open(QIODevice::WriteOnly)){
        qDebug() << "open file to write file!";
        return false;
    }

    json_file.write(qba);

    json_file.close();

    return true;
}

bool RobotConfig::readJsonFile(QString file_path, QJsonObject &json_obj)
{
    QFile json_file(file_path);

    if(!json_file.open(QIODevice::ReadOnly)){
        qDebug() << "open json file to read fail!";
        return false;
    }

    QByteArray qba = json_file.readAll();

    QJsonParseError json_err;

    QJsonDocument json_doc = QJsonDocument::fromJson(qba, &json_err);

    if(json_err.error != QJsonParseError::NoError || json_doc.isNull()){
        qDebug() << "json txt to json obj tran fail!";
        return false;
    }

    if(!json_doc.isObject()){
       qDebug() << "read json file but json doc is not object!";
       return false;
    }

    json_obj = json_doc.object();

    return true;

}

bool RobotConfig::tranfProgramConfigListToJsonObj(QList<program_config_stru> &program_config_list, QJsonObject &json_obj)
{
    bool flag = false;

    QJsonArray program_config_json_arr;

    for(int i = 0; i < program_config_list.size(); i++){

        program_config_stru program_config = program_config_list.at(i);

        //单个表演配置对象,由语音，表情，动作等配置对象组成
        QJsonObject program_config_json_obj;
        program_config_json_obj.insert("program_name", program_config.program_name);
        program_config_json_obj.insert("program_id", program_config.program_id);
        program_config_json_obj.insert("program_duration_ms", program_config.program_duration_ms);

        //将语音参数配置列表转换为json数组
        QJsonArray voice_json_arr;
        for(int j = 0; j < program_config.voice_config_list.size(); j++){
            voice_param_stru voice_param = program_config.voice_config_list.at(j);
            QJsonObject voice_json_obj;
            voice_json_obj.insert("act_time_ms", voice_param.act_time_ms);
            voice_json_obj.insert("voice_name", voice_param.voice_name);
            voice_json_obj.insert("tts_text", voice_param.tts_text);
            voice_json_obj.insert("voice_path", voice_param.voice_path);
            voice_json_arr.push_back(voice_json_obj);
        }
        program_config_json_obj.insert("voice_config_list", voice_json_arr);

        //表情
        QJsonArray face_json_arr;
        for(int j = 0; j < program_config.face_config_list.size(); j++){
            face_param_stru face_param = program_config.face_config_list.at(j);
            QJsonObject face_json_obj;
            face_json_obj.insert("act_time_ms", face_param.act_time_ms);
            face_json_obj.insert("face_name", face_param.face_name);
            face_json_obj.insert("face_id", face_param.face_id);
            face_json_arr.push_back(face_json_obj);
        }
        program_config_json_obj.insert("face_config_list", face_json_arr);

        //动作
        QJsonArray servo_config_arr;
        for(int j = 0; j < program_config.servo_config_list.size(); j++){
            servo_config_stru servo_config = program_config.servo_config_list.at(j);

            //多个舵机参数组成的动作参数列表
            QJsonArray servo_config_json_arr;
            for(int k = 0; k < servo_config.servo_param_list.size(); k++){
                servo_param_stru servo_param = servo_config.servo_param_list.at(k);
                //单个舵机参数
                QJsonObject servo_param_json_obj;
                servo_param_json_obj.insert("servo_id", servo_param.servo_id);
                servo_param_json_obj.insert("servo_angle", servo_param.servo_angle);
                servo_param_json_obj.insert("servo_time", servo_param.servo_time);
                servo_config_json_arr.push_back(servo_param_json_obj);
            }

            //动作参数对象
            QJsonObject servo_config_obj;
            servo_config_obj.insert("act_time_ms", servo_config.act_time_ms);
            servo_config_obj.insert("act_name", servo_config.act_name);
            servo_config_obj.insert("servo_param_list", servo_config_json_arr);
            servo_config_arr.push_back(servo_config_obj);
        }


        program_config_json_obj.insert("servo_config_list", servo_config_arr);

        //时间点列表
        QJsonArray act_time_ms_arr;
        for(int j = 0; j < program_config.act_time_ms_list.size(); j++){
            act_time_ms_arr.push_back(program_config.act_time_ms_list.at(j));
        }
        program_config_json_obj.insert("act_time_ms_list", act_time_ms_arr);

        //底盘动作
        QJsonArray movebase_config_arr;
        for(int j = 0; j < program_config.movebase_config_list.size(); j++){
            movebase_config_stru movebase_config = program_config.movebase_config_list.at(j);
            QJsonObject movebase_json_obj;
            movebase_json_obj.insert("act_time_ms", movebase_config.act_time_ms);
            movebase_json_obj.insert("movebase_config_name", movebase_config.movebase_config_name);
            QJsonObject movebase_param_json_obj;
            movebase_param_json_obj.insert("dis", movebase_config.movebase_param.dis);
            movebase_param_json_obj.insert("angle", movebase_config.movebase_param.angle);
            movebase_json_obj.insert("movebase_param", movebase_param_json_obj);
            movebase_config_arr.push_back(movebase_json_obj);
        }

        program_config_json_obj.insert("movebase_config_list", movebase_config_arr);

        //多个表演配置对象组成表演配置列表
        program_config_json_arr.push_back(program_config_json_obj);

        flag = true;
    }

    json_obj.insert("program_config_list", program_config_json_arr);

    return flag;
}


bool RobotConfig::tranfJsonObjToProgramConfigList(QJsonObject &json_obj, QList<program_config_stru> &program_config_list)
{
    if(!json_obj.contains("program_config_list")){
        qDebug() << "json obj not contain program config list key";
        return false;
    }

    QJsonValue jval = json_obj.value("program_config_list");

    if(!jval.isArray()){
        qDebug() << "json value is not array!";
        return false;
    }

    //多个表演配置组成的数组
    QJsonArray jarr = jval.toArray();

    for(int i = 0; i < jarr.size(); i++){
        //单个表演配置
        QJsonValue jval_arr = jarr.at(i);
        if(!jval_arr.isObject()){
            continue;
        }

        QJsonObject program_config_json_obj = jval_arr.toObject();
        program_config_stru program_config;

        if(!program_config_json_obj.contains("program_name")){
            continue;
        }
        QJsonValue jval_t = program_config_json_obj.value("program_name");
        program_config.program_name = jval_t.toString();

        if(!program_config_json_obj.contains("program_id")){
            continue;
        }
        program_config.program_id = program_config_json_obj.value("program_id").toString();

        if(!program_config_json_obj.contains("program_duration_ms")){
            continue;
        }
        program_config.program_duration_ms = program_config_json_obj.value("program_duration_ms").toDouble();

        //解析时间点列表
        if(!program_config_json_obj.contains("act_time_ms_list")){
            continue;
        }
        jval_t = program_config_json_obj.value("act_time_ms_list");
        if(!jval_t.isArray()){
            continue;
        }
        QJsonArray jarr_t = jval_t.toArray();
        for(int j = 0; j < jarr_t.size(); j++){
            jval_t = jarr_t.at(j);
            program_config.act_time_ms_list.push_back(jval_t.toDouble());
        }

        //解析voice_config_list
        if(!program_config_json_obj.contains("voice_config_list")){
            continue;
        }
        jval_t = program_config_json_obj.value("voice_config_list");
        if(!jval_t.isArray()){
            continue;
        }
        jarr_t = jval_t.toArray();
        for(int j = 0; j < jarr_t.size(); j++){
            jval_t = jarr_t.at(j);
            QJsonObject jobj_t = jval_t.toObject();
            voice_param_stru voice_param;
            voice_param.act_time_ms = jobj_t.value("act_time_ms").toDouble();
            voice_param.voice_name = jobj_t.value("voice_name").toString();
            voice_param.voice_path = jobj_t.value("voice_path").toString();
            voice_param.tts_text = jobj_t.value("tts_text").toString();
            program_config.voice_config_list.push_back(voice_param);
        }

        //解析face_config_list
        if(!program_config_json_obj.contains("face_config_list")){
            continue;
        }
        jval_t = program_config_json_obj.value("face_config_list");
        if(!jval_t.isArray()){
            continue;
        }
        jarr_t = jval_t.toArray();
        for(int j = 0; j < jarr_t.size(); j++){
            jval_t = jarr_t.at(j);
            QJsonObject jobj_t = jval_t.toObject();
            face_param_stru face_param;
            face_param.act_time_ms = jobj_t.value("act_time_ms").toDouble();
            face_param.face_name = jobj_t.value("face_name").toString();
            face_param.face_id = jobj_t.value("face_id").toString();
            program_config.face_config_list.push_back(face_param);
        }


        //解析movebase_config_list
        if(!program_config_json_obj.contains("movebase_config_list")){
            continue;
        }
        jval_t = program_config_json_obj.value("movebase_config_list");
        if(!jval_t.isArray()){
            continue;
        }
        jarr_t = jval_t.toArray();
        for(int j = 0; j < jarr_t.size(); j++){
            jval_t = jarr_t.at(j);
            QJsonObject jobj_t = jval_t.toObject();
            movebase_config_stru movebase_config;
            movebase_config.act_time_ms = jobj_t.value("act_time_ms").toDouble();
            movebase_config.movebase_config_name = jobj_t.value("movebase_config_name").toString();
            QJsonObject jobj_tt = jobj_t.value("movebase_param").toObject();
            movebase_config.movebase_param.dis = jobj_tt.value("dis").toDouble();
            movebase_config.movebase_param.angle = jobj_tt.value("angle").toDouble();
            program_config.movebase_config_list.push_back(movebase_config);
        }

        //解析servo_config_list
        if(!program_config_json_obj.contains("servo_config_list")){
            continue;
        }
        jval_t = program_config_json_obj.value("servo_config_list");
        if(!jval_t.isArray()){
            continue;
        }
        jarr_t = jval_t.toArray();
        for(int j = 0; j < jarr_t.size(); j++){
            jval_t = jarr_t.at(j);
            QJsonObject jobj_t = jval_t.toObject();
            servo_config_stru servo_config;
            servo_config.act_name = jobj_t.value("act_name").toString();
            servo_config.act_time_ms = jobj_t.value("act_time_ms").toDouble();
            QJsonValue jval_tt = jobj_t.value("servo_param_list");
            if(!jval_tt.isArray()){
                continue;
            }
            QJsonArray jarr_tt = jval_tt.toArray();
            for(int k = 0; k < jarr_tt.size(); k++){
                jval_tt = jarr_tt.at(k);
                QJsonObject jobj_t = jval_tt.toObject();
                servo_param_stru servo_param;
                servo_param.servo_id = jobj_t.value("servo_id").toDouble();
                servo_param.servo_angle = jobj_t.value("servo_angle").toDouble();
                servo_param.servo_time = jobj_t.value("servo_time").toDouble();
                servo_config.servo_param_list.push_back(servo_param);
            }
            program_config.servo_config_list.push_back(servo_config);
        }
        //保存到表演配置列表中
        program_config_list.push_back(program_config);

    }

    return true;
}


int RobotConfig::getRandVal(int max_val)
{
    qsrand(QTime::currentTime().msecsTo(QTime(0, 0, 0, 0)));
    int rand_val = qrand() % max_val;
    return rand_val;
}

