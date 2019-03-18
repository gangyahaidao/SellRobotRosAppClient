#include "robotaction.h"

RobotAction::RobotAction(QObject *parent) : QObject(parent)
{

}

RobotAction::~RobotAction()
{

}

bool RobotAction::init()
{
    bool flag1 = robot_config.readProgramConfig();
    if(!flag1){
        qDebug() << "read program config fail";
    }

    bool rd_local_config = robot_config.readLocalConfig();
    if(!rd_local_config){
        qDebug() << "read local config fail!";
        return false;
    }

    bool flag3 = robot_arm.openServoPort(robot_config.servo_port);
    if(!flag3){
        qDebug() << "open servo port fail!";
    }

    connect(&action_play_timer, SIGNAL(timeout()), this, SLOT(actionPlayTimerSlot()));

    if(flag1 && flag3){
        return true;
    }else{
        return false;
    }
}

void RobotAction::test()
{
    QString tts = "你好，这是一个语音合成的测试例子！";

}

bool RobotAction::startAction(QString act_id)
{

    bool find_flag = findProgramConfig(act_id, program_config);

    if(!find_flag){
        emit actionErrSignal();
        return false;
    }

    action_loop_flag = false;

    play_time_ms = -100;
    curr_face_play_id = 0;
    curr_servo_play_id = 0;
    curr_voice_play_id = 0;
    curr_movebase_ctrl_id = 0;
    actionPlayTimerSlot();
    action_play_timer.start(100);

    return true;
}

bool RobotAction::startActionLoop(QString act_id)
{
    bool find_flag = findProgramConfig(act_id, program_config);

    if(!find_flag){
        emit actionErrSignal();
        return false;
    }

    action_loop_flag = true;

    play_time_ms = -100;
    curr_face_play_id = 0;
    curr_servo_play_id = 0;
    curr_voice_play_id = 0;
    curr_movebase_ctrl_id = 0;
    actionPlayTimerSlot();
    action_play_timer.start(100);

    return true;
}

void RobotAction::stopAction()
{
    action_loop_flag = false;

    action_play_timer.stop();

    //动作回到原点
    //todo
}

bool RobotAction::startHeadRotation(QString angle)
{
    float rot_angle, curr_angle;

    bool ok;
    rot_angle = angle.toDouble(&ok);

    if(!ok){
        //emit actionErrSignal();
        return false;
    }

    if(rot_angle > HEAD_SERVO_MAX_ANLGE){
        rot_angle = HEAD_SERVO_MAX_ANLGE;
    }
    if(rot_angle < HEAD_SERVO_MIN_ANLGE){
        rot_angle = HEAD_SERVO_MIN_ANLGE;
    }

    bool get_flag = robot_arm.getServoPresentAngle(HEAD_SERVO_ID, curr_angle, 200);

    if(!get_flag){
        curr_angle = 120;
    }

    float delta_angle = rot_angle - curr_angle;

    int servo_time_ms = std::abs(delta_angle) / (HEAD_SERVO_MAX_ANLGE - HEAD_SERVO_MIN_ANLGE) * 3000 + 300;

    robot_arm.sendServoMoveAngleAndTime(HEAD_SERVO_ID, rot_angle, servo_time_ms);

    delayMs(servo_time_ms);

    emit headRotationFinishSignal();

    return true;
}

void RobotAction::delayMs(int time_ms)
{
    QTime delay_time = QTime::currentTime().addMSecs(time_ms);
    while(QTime::currentTime() < delay_time){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }
}

bool RobotAction::findProgramConfig(QString act_id, program_config_stru &program_config)
{
    bool find_flag = false;

    for(int i = 0; i < robot_config.program_config_list.size(); i++){
        program_config = robot_config.program_config_list.at(i);
        if(program_config.program_id == act_id){
            find_flag = true;
            break;
        }
    }

    return find_flag;
}

void RobotAction::actionPlayTimerSlot()
{
    play_time_ms += 100;

    if(play_time_ms >= program_config.program_duration_ms){
        if(action_loop_flag){
            play_time_ms = -100;
            curr_face_play_id = 0;
            curr_servo_play_id = 0;
            curr_voice_play_id = 0;
            curr_movebase_ctrl_id = 0;
            return;
        }else{
            action_play_timer.stop();
            emit actionFinishSignal(program_config.program_id);
        }
        //robot_face_p->showFace("DAI_JI_FACE_ID");
        for(int i = 1; i <= SELL_ROBOT_SERVO_NUM; i++){
            robot_arm.sendServoLoadOrUnload(i, false);
        }
    }

    //播放表情
    //

    //播放声音
    //

    //播放动作
    if(curr_servo_play_id < program_config.servo_config_list.size()){
        servo_config_stru servo_config = program_config.servo_config_list.at(curr_servo_play_id);
        if(play_time_ms >= servo_config.act_time_ms){
            curr_servo_play_id++;

            for(int j = 0; j < servo_config.servo_param_list.size(); j++){
                servo_param_stru servo_param = servo_config.servo_param_list.at(j);
                robot_arm.sendServoMoveAngleAndTime(servo_param.servo_id, servo_param.servo_angle, servo_param.servo_time, true);
            }

            robot_arm.sendStartCmd(BRODCAST_ID);
        }
    }

    //执行底盘动作
    //

}

void RobotAction::startActionSlot(QString act_id)
{
    startAction(act_id);
}

void RobotAction::startActionLoopSlot(QString act_id)
{
    qDebug() << "start action loop!";

    startActionLoop(act_id);
}

void RobotAction::stopActionSlot()
{
    qDebug() << "stop action!";

    stopAction();
}


void RobotAction::startHeadRotationSlot(QString angle)
{
    startHeadRotation(angle);
}


