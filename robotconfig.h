#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <QObject>
#include <QFile>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonValue>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonParseError>
#include <QtCore/QJsonDocument>
#include <QtGlobal>
#include <QTime>
#include <QDateTime>
//#include <QVector3D>
#include <QDebug>


#include "robotpose.h"

//广告和问候语
struct qstring_rate_stru
{
    QString data_qs;
    int rate;
    int choose_val;
};
//问答
struct quesion_answer_stru
{
    QString quesion;
    QStringList keyword_list;
    QString answer;
};

//节目配置信息

//表情配置参数
struct face_param_stru
{
    int act_time_ms;           //表情触发时间
    QString face_name;         //表情名称
    QString face_id;          //表情id
};

//单个舵机参数
struct servo_param_stru
{
    int servo_id;
    int servo_angle;    //du
    int servo_time;     //ms
};

//多个舵机参数组成的动作参数
struct servo_config_stru
{
    int act_time_ms;        //动作触发时间点
    QString act_name;       //动作名称
    QList<servo_param_stru> servo_param_list;
};

//底盘移动参数
struct movebase_param_stru
{
    float dis;             //正前进，负后退，单位米
    float angle;          //正左转，负右转，单位度
};

//底盘动作配置参数
struct movebase_config_stru
{
    int act_time_ms;
    QString movebase_config_name;
    movebase_param_stru movebase_param;
};

//单个声音参数
struct voice_param_stru
{
    int act_time_ms;
    QString voice_name;         //语音名称
    QString tts_text;           //语音合成文本
    QString voice_path;         //如果内容为"tts",说明没有声音路径，语音为实时合成

};

//单个节目配置参数
struct program_config_stru
{
    QString program_name;        //节目名称
    QString program_id;         //节目id
    int program_duration_ms;   //节目总时长
    QList<int> act_time_ms_list;                  //节目时间点列表
    QList<voice_param_stru> voice_config_list;   //多个声音参数列表
    QList<face_param_stru> face_config_list;     //多个表情参数列表
    QList<servo_config_stru> servo_config_list;     //多个动作参数列表
    QList<movebase_config_stru> movebase_config_list;   //多个底盘动作参数列表
};


//巡逻配置
struct patral_point_stru
{
    QString  act_type;
    RobotPose path_point;
};

struct patral_config_stru
{
    int patral_interval;            //巡逻间隔，单位秒
    QString run_act_type;           //运行过程中执行的动作类型
    QString stop_act_type;          //停止的过程中执行的动作类型
    QString block_act_type;         //检测到障碍时执行的动作类型
    int patral_mode;                //巡逻模式：1、列表路线只跑一次，2、到达列表最后一点，下一目标点为列表起点，3、到达最后一点，下一目标点为倒数第二点
    int arrive_mode;                //到达模式：到达一个巡逻点后继续下一点，或到达一点后等待开始信号
    int rot_des_angle_code;         //到达目标点后，是否转动到目标角度
    //QList<QVector3D> path_list;     //巡逻路线点
    QList<patral_point_stru> path_list;     //巡逻路线配置
};



class RobotConfig : public QObject
{
    Q_OBJECT
public:
    explicit RobotConfig(QObject *parent = 0);

    void test();

    int getRandVal(int max_val);

    //读取配置文件
    bool readProgramConfig();

    bool readLocalConfig();

    bool readPatralConfig();

    //将json对象保存为文件
    bool saveJsonObjAsFile(QString file_path, QJsonObject& json_obj);

    //读取json文件，转换到json对象中
    bool readJsonFile(QString file_path, QJsonObject& json_obj);

    //将表演配置数据列表转换为json对象
    bool tranfProgramConfigListToJsonObj(QList<program_config_stru>& program_config_list, QJsonObject& json_obj);

    //将json对象转换为表演配置数据列表
    bool tranfJsonObjToProgramConfigList(QJsonObject& json_obj, QList<program_config_stru>& program_config_list);

private:



signals:

public slots:


public:

    QString server_ip;  //远程服务器ip
    int server_port;    //远程服务器端口

    int client_port;    //本地服务器监听端口号

    QString servo_port; //舵机串口名

    QString xfei_port; //语音模块串口名

    //表演节目配置信息
    QList<program_config_stru> program_config_list;

    //巡逻配置信息
    patral_config_stru patral_config;


};

#endif // ROBOTCONFIG_H
