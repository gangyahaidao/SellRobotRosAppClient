#include "realsensemulticam.h"

RealsenseMultiCam::RealsenseMultiCam(QObject *parent) : QObject(parent)
{

}

int RealsenseMultiCam::startCam() try
{

#ifdef SHOW_CAMERA
    window app(1280, 960, "CPP Multi-Camera Example");
#endif

    device_container connected_devices;

    rs2::context ctx;    // Create librealsense context for managing devices

    // Register callback for tracking which devices are currently connected
//    ctx.set_devices_changed_callback([&](rs2::event_information& info)
//    {
//        connected_devices.remove_devices(info);
//        for (auto&& dev : info.get_new_devices())
//        {
//            connected_devices.enable_device(dev);
//        }
//    });

    // Initial population of the device list
    for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices
    {
        //connected_devices.enable_device(dev);
        const char* cam_info = dev.get_info(RS2_CAMERA_INFO_NAME);
        qDebug() << "cam info name:" << QString(cam_info);
        if(cam_info == sr300_camera_name){
            connected_devices.enable_device(dev);
        }
    }

    int cout = 0;
#ifdef SHOW_CAMERA
    while (app && camera_run_flag) // Application still alive?
#else
    while(camera_run_flag)
#endif
    {
        //处理深度数据
        rs2::frameset frameset;
        bool get_flag = connected_devices.poll_frames(frameset);

        if(get_flag){
            rs2::depth_frame depth_frame = frameset.get_depth_frame();
            rs2::video_frame color_frame = frameset.get_color_frame();

            int depth_width = depth_frame.get_width();
            int depth_height = depth_frame.get_height();
            int color_width = color_frame.get_width();
            int color_height = color_frame.get_height();

            cout++;

            if(cout > 1){
                cout = 0;
                sendDepthData(depth_frame);
                //qDebug("depth wid:%d, hei:%d, color wid:%d, hei:%d", depth_width, depth_height, color_width, color_height);
            }


        }

        //QCoreApplication::processEvents(QEventLoop::AllEvents, 50);

        usleep(30000);

        //显示画面处理
#ifdef SHOW_CAMERA
        auto total_number_of_streams = connected_devices.stream_count();
        if (total_number_of_streams == 0)
        {
            draw_text(int(std::max(0.f, (app.width() / 2) - no_camera_message.length() * 3)),
                      int(app.height() / 2), no_camera_message.c_str());
            continue;
        }
//        if (connected_devices.device_count() == 1)
//        {
//            draw_text(0, 10, "Please connect another camera");
//        }
        int cols = int(std::ceil(std::sqrt(total_number_of_streams)));
        int rows = int(std::ceil(total_number_of_streams / static_cast<float>(cols)));

        float view_width = (app.width() / cols);
        float view_height = (app.height() / rows);

        connected_devices.render_textures(cols, rows, view_width, view_height);
#endif
    }

    return EXIT_SUCCESS;
}catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


bool RealsenseMultiCam::getNearBlockFromDepthData(rs2::depth_frame &depth_frame, QList<RobotPose> &block_point_list)
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

        for(int row = 0; row < DEPTH_HEIGHT; row++){
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
bool RealsenseMultiCam::tranfDepthPointToWorldXYZ(RobotPose &world_xyz, int depth_row, int depth_col, float depth_dis)
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

void RealsenseMultiCam::filterDepthPoint(QList<RobotPose> &blc_point_list)
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

bool RealsenseMultiCam::sendDepthData(rs2::depth_frame &depth_frame)
{
    bool send_flag = false;
    QList<RobotPose> blc_point_list;

    bool get_flag = getNearBlockFromDepthData(depth_frame, blc_point_list);
    if(get_flag){
        filterDepthPoint(blc_point_list);

        if(blc_point_list.size() > 5){
            emit sendDepthPointSignal(blc_point_list);
            send_flag = true;
        }

        RobotPose point;
        for(int i = 0; i < blc_point_list.size(); i += 1){
            point = blc_point_list.at(i);
            //qDebug("col:%d, blc point x:%f, y:%f, z:%f", i, point.x, point.y, point.z);
        }
    }

    return send_flag;
}

void RealsenseMultiCam::startRealsenseCameraSlot()
{
    camera_run_flag = true;
    startCam();
}

void RealsenseMultiCam::stopRealsenseCameraSlot()
{
    camera_run_flag = false;
}


