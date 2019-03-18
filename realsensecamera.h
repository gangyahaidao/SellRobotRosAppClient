#ifndef REALSENSECAMERA_H
#define REALSENSECAMERA_H

#include <QObject>
//#include <qvector3d.h>
#include <QList>

#include "robotpose.h"

#undef foreach

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering

#include <opencv2/core.hpp>

//SR300:640x480  D435:1280x720,640x360
#define DEPTH_WIDTH     640        //640
#define DEPTH_HEIGHT    480        //360 480
#define COLOR_WIDTH     1280
#define COLOR_HEIGHT    720
#define DEPTH_FPS       30
#define COLOR_FPS       30

#define DET_BLOCK_DIS   0.8

const int o_col = DEPTH_WIDTH / 2;  //画面中心列 = 画面宽度/2
const int o_row = DEPTH_HEIGHT / 2; //画面中心行 = 画面高度/2

class RealSenseCamera : public QObject
{
    Q_OBJECT
public:
    QList<RobotPose> blc_point_list;

    explicit RealSenseCamera(QObject *parent = 0);

    void showCamera();
    bool getNearBlockFromDepthData(rs2::depth_frame& depth_frame, QList<RobotPose>& block_point_list);
    bool tranfDepthPointToWorldXYZ(RobotPose& world_xyz, int depth_row, int depth_col, float depth_dis);
    void filterDepthPoint(QList<RobotPose>& blc_point_list);
signals:
    void sendDepthPointSignal(QList<RobotPose> block_point_list);

public slots:
    void startRealsenseCameraSlot();
    void stopRealsenseCameraSlot();

private:
    bool camera_start_flag;

};

#endif // REALSENSECAMERA_H
