#include <QCoreApplication>
#include <QThread>

#include "movebase.h"
#include "sellrobotclient.h"
#include "robotaction.h"
#include "robotconfig.h"
//#include "realsensecamera.h"
#include "realsensemulticam.h"

MoveBase move_base;
SellRobotClient sell_robot_client;
RobotAction robot_action;
//RealSenseCamera rls_cam;
RealsenseMultiCam rls_mul_cam;


void initSignalSlot()
{
   QObject::connect(&move_base, SIGNAL(sendMoveBaseRunStateSignal(QVariant)), &sell_robot_client, SLOT(getMoveBaseRunStateSlot(QVariant)));

   QObject::connect(&sell_robot_client, SIGNAL(startHeadRotateSignal(QString)), &robot_action, SLOT(startHeadRotationSlot(QString)));
   QObject::connect(&sell_robot_client, SIGNAL(startActionSignal(QString)), &robot_action, SLOT(startActionSlot(QString)));

   //QObject::connect(&sell_robot_client, SIGNAL(startRealsenseCameraSignal()), &rls_cam, SLOT(startRealsenseCameraSlot()));
   QObject::connect(&sell_robot_client, SIGNAL(startRealsenseCameraSignal()), &rls_mul_cam, SLOT(startRealsenseCameraSlot()));


}


void initSoft()
{
    move_base.initMoveBase();

    sell_robot_client.init(&move_base);

    //robot_action.init();

}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    qDebug() << "sell robot client app start!";



    initSoft();

    initSignalSlot();

    QThread* cam_thread_p = new QThread;
    rls_mul_cam.moveToThread(cam_thread_p);
    cam_thread_p->start();

    sell_robot_client.connectToServer();

    sell_robot_client.startRealsenseCam();

    //sell_robot_client.test();


    return a.exec();
}
