#include "realsensecamera.h"
#include <QDebug>
#include <QTime>
#include <QMetaType>

RealSenseCamera::RealSenseCamera(QObject *parent) : QObject(parent)
{
        qRegisterMetaType<QList<RobotPose> >("QList<RobotPose>");
}

void RealSenseCamera::showCamera()
{
    // Create a simple OpenGL window for rendering:
    //window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    //texture depth_image, color_image;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_DEPTH, 0, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 0, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);
    cfg.enable_stream(RS2_STREAM_COLOR, 0, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, COLOR_FPS);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    pipe.start(cfg);
    //pipe.start();

    int count = 0;

    while(/*app &&*/ camera_start_flag) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        //rs2::frame depth = color_map(data.get_depth_frame()); // Find and colorize the depth data
        //rs2::frame depth = apply_filter(color_map);
        rs2::frame color = data.get_color_frame();            // Find the color data

        count++;
        if(count > 2){
            count = 0;
            rs2::depth_frame depth_frame = data.get_depth_frame();

            bool get_flag = getNearBlockFromDepthData(depth_frame, blc_point_list);
            filterDepthPoint(blc_point_list);
            if(get_flag){
                //*
                RobotPose point;
                //qDebug() << "........";
                for(int i = 0; i < blc_point_list.size(); i += 1){
                    point = blc_point_list.at(i);
                    qDebug("col:%d, blc point x:%f, y:%f, z:%f", i, point.x, point.y, point.z);
                }
                //*/
            }else{
                //qDebug() << "not det near blc!";
            }

            if(blc_point_list.size() > 5)
                emit sendDepthPointSignal(blc_point_list);
        }


        // Render depth on to the first half of the screen and color on to the second
        //depth_image.render(depth, { 0,               0, app.width() / 2, app.height() });
        //color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
    }
}


int det_depth_height = 400;     //摄像头下面被挡住，要去掉下面的数据

bool RealSenseCamera::getNearBlockFromDepthData(rs2::depth_frame &depth_frame, QList<RobotPose> &block_point_list)
{
    //QList<RobotPose> min_depthpoint_list;
    cv::Mat depth_mat(depth_frame.get_height(), depth_frame.get_width(), CV_16U, (uchar*)depth_frame.get_data());
    block_point_list.clear();
    RobotPose depthpoint_in_worldxyz;
    bool find_near_block = false;
    for(int col = 0; col < DEPTH_WIDTH; col += 2){ //隔一列扫描
        int min_row;
        float min_dis = DET_BLOCK_DIS + 0.1;
        //选出这一列中深度值最小的点

        for(int row = 0; row < det_depth_height; row++){
            //D435一个单位代表1mm,SR300一个单位代表0.125mm
            //float depth_dis = (float)depth_mat.at<unsigned short>(row, col) / 1000.0;
            float depth_dis = depth_mat.at<unsigned short>(row, col) * 0.125 / 1000.0;
            //判断深度数据是否有效
            if(depth_dis < 0.1 || depth_dis > DET_BLOCK_DIS)
                continue;
            if(depth_dis < min_dis){
                min_dis = depth_dis;
                min_row = row;
            }
        }

        if(min_dis > DET_BLOCK_DIS){
            //这一列未找到有效深度数据
            depthpoint_in_worldxyz.x = 0;
            depthpoint_in_worldxyz.y = 0;
            depthpoint_in_worldxyz.z = 0;
            block_point_list.push_back(depthpoint_in_worldxyz);
        }else{
            find_near_block = true;
            tranfDepthPointToWorldXYZ(depthpoint_in_worldxyz, min_row, col, depth_frame.get_distance(col, min_row));
            block_point_list.push_back(depthpoint_in_worldxyz);
        }

    }

    return find_near_block;
}

//D435 深度fov：水平85.2,垂直58
//世界坐标系定义：摄像头最右边为y轴原点，摄像头上下中心为z轴原点，从摄像头原点向前为x，向左为y，向上为z，单位为米
//摄像头画面中左边为世界坐标系y轴正方向
//SR300 深度FOV：水平71.5,垂直55
//世界坐标系定义：从摄像头后面往前看，摄像头中心往右偏0.04米为原点，向左为y正向，向上为z正向，单位米
bool RealSenseCamera::tranfDepthPointToWorldXYZ(RobotPose &world_xyz, int depth_row, int depth_col, float depth_dis)
{
    world_xyz.x = (depth_dis);
    //D435
    //float y = (float)(o_col - depth_col) / o_col * 0.91955 * depth_dis;
    //float z = (float)(o_row - depth_row) / o_row * 0.55431 * depth_dis;
    //SR300
    float y = (float)(o_col - depth_col) / o_col * 0.72 * depth_dis;
    float z = (float)(o_row - depth_row) / o_col * 0.52 * depth_dis;
    world_xyz.y = (y);
    world_xyz.z = (z);

    return true;
}

void RealSenseCamera::filterDepthPoint(QList<RobotPose> &blc_point_list)
{
    RobotPose depth_point;
    QList<RobotPose>::iterator it;
    int data_count = 0;
    it = blc_point_list.begin();
    //迭代器删除该位置的数据后返回下一个位置的数据位置
    while(it != blc_point_list.end()){
        depth_point = *it;
        if(depth_point.x < 0.1){
            //深度值=0,无效数据
            if(data_count > 0 && data_count < 5){
                //有效数据个数小于5列，可能是干扰，删除这几列数据
                it = blc_point_list.erase(it - data_count, it);
            }else{
                //前面的数据个数大于判断值，视为有效数据，只删除当前为0的数据
                it = blc_point_list.erase(it);
            }
            data_count = 0;
        }else{
            //有效数据，迭代器指针加1指向下一个数据
            data_count++;
            it++;
        }
    }
}

void RealSenseCamera::startRealsenseCameraSlot()
{
    camera_start_flag = true;
    showCamera();
}

void RealSenseCamera::stopRealsenseCameraSlot()
{
    camera_start_flag = false;
    qDebug() << "rec stop camera signal...";
}
