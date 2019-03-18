#ifndef ROBOTACTION_H
#define ROBOTACTION_H

#include <QObject>
#include <QTimer>
#include <QTime>
#include <QDebug>

#include "robotarm.h"
#include "robotconfig.h"
//#include "robotface.h"
//#include "robotvoice.h"

#define    HEAD_SERVO_ID            5
#define    HEAD_SERVO_MAX_ANLGE     200
#define    HEAD_SERVO_MIN_ANLGE     40
#define    SELL_ROBOT_SERVO_NUM     5

class RobotAction : public QObject
{
    Q_OBJECT
public:
    explicit RobotAction(QObject *parent = 0);

    ~RobotAction();

    bool init();

    bool startAction(QString act_id);

    bool startActionLoop(QString act_id);

    void stopAction();

    bool startHeadRotation(QString angle);

    void delayMs(int time_ms);

    void test();


private:
    bool findProgramConfig(QString act_id, program_config_stru& program_config);

signals:
    void actionFinishSignal(QString act_id);

    void headRotationFinishSignal();

    void actionErrSignal();

private slots:
    void actionPlayTimerSlot();


public slots:
    void startActionSlot(QString act_id);

    void startActionLoopSlot(QString act_id);

    void stopActionSlot();

    void startHeadRotationSlot(QString angle);


private:

    RobotConfig robot_config;
    RobotArm robot_arm;

    QTimer action_play_timer;       //动作播放定时器
    int play_time_ms;               //动作定时器当前计时时间
    int curr_face_play_id;
    int curr_voice_play_id;
    int curr_servo_play_id;
    int curr_movebase_ctrl_id;
    program_config_stru program_config;     //动作脚本参数
    bool action_loop_flag;

};


#endif // ROBOTACTION_H
