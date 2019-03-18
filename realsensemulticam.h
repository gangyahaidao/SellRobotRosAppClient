#ifndef REALSENSEMULTICAM_H
#define REALSENSEMULTICAM_H

#include <QObject>
#include <QCoreApplication>
#include <QList>
#include <QDebug>

#include <unistd.h>

#undef foreach

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "example.hpp"              // Include short list of convenience functions for rendering

#include <string>
#include <map>
#include <algorithm>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil

#include <opencv2/core.hpp>

#include "robotpose.h"

const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

const std::string sr300_camera_name = "Intel RealSense SR300";
const std::string d435_camera_name = "Intel RealSense 435";

//SR300:640x480  D435:1280x720,640x360
#define DEPTH_WIDTH     640        //640
#define DEPTH_HEIGHT    480        //360 480
#define COLOR_WIDTH     1280
#define COLOR_HEIGHT    720
#define DEPTH_FPS       10
#define COLOR_FPS       10

#define DET_BLOCK_DIS   0.8

const int o_col = DEPTH_WIDTH / 2;  //画面中心列 = 画面宽度/2
const int o_row = DEPTH_HEIGHT / 2; //画面中心行 = 画面高度/2

//#define SHOW_CAMERA


class device_container
{
    // Helper struct per pipeline
    struct view_port
    {
        std::map<int, rs2::frame> frames_per_stream;
        rs2::colorizer colorize_frame;
        texture tex;
        rs2::pipeline pipe;
        rs2::pipeline_profile profile;
    };

public:

    void enable_device(rs2::device dev)
    {
        std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::lock_guard<std::mutex> lock(_mutex);

        if (_devices.find(serial_number) != _devices.end())
        {
            return; //already in
        }

        // Ignoring platform cameras (webcams, etc..)
        if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
        {
            return;
        }
        // Create a pipeline from the given device
        rs2::pipeline p;
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_DEPTH, 0, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, DEPTH_FPS);
        cfg.enable_stream(RS2_STREAM_COLOR, 0, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, COLOR_FPS);

        cfg.enable_device(serial_number);
        // Start the pipeline with the configuration
        rs2::pipeline_profile profile = p.start(cfg);
        // Hold it internally
        _devices.emplace(serial_number, view_port{ {},{},{}, p, profile });

    }

    void remove_devices(const rs2::event_information& info)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over the list of devices and check if it was disconnected
        auto itr = _devices.begin();
        while(itr != _devices.end())
        {
            if (info.was_removed(itr->second.profile.get_device()))
            {
                itr = _devices.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }

    size_t device_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _devices.size();
    }

    int stream_count()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int count = 0;
        for (auto&& sn_to_dev : _devices)
        {
            for (auto&& stream : sn_to_dev.second.frames_per_stream)
            {
                if (stream.second)
                {
                    count++;
                }
            }
        }
        return count;
    }

    void poll_frames()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        // Go over all device
        for (auto&& view : _devices)
        {
            // Ask each pipeline if there are new frames available
            rs2::frameset frameset;
            if (view.second.pipe.poll_for_frames(&frameset))
            {
                for (int i = 0; i < frameset.size(); i++)
                {
                    rs2::frame new_frame = frameset[i];
                    int stream_id = new_frame.get_profile().unique_id();
                    view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(new_frame); //update view port with the new stream
                }
            }
        }
    }

    bool poll_frames(rs2::frameset& frameset)
    {
        std::lock_guard<std::mutex> lock(_mutex);

        bool get_flag = false;

        // Go over all device
        for (auto&& view : _devices)
        {
            // Ask each pipeline if there are new frames available

            if (view.second.pipe.poll_for_frames(&frameset))
            {
                get_flag = true;
                for (int i = 0; i < frameset.size(); i++)
                {
                    rs2::frame new_frame = frameset[i];
                    int stream_id = new_frame.get_profile().unique_id();
                    view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(new_frame); //update view port with the new stream
                }
            }
        }

        return get_flag;
    }

    void render_textures(int cols, int rows, float view_width, float view_height)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        int stream_no = 0;
        for (auto&& view : _devices)
        {
            // For each device get its frames
            for (auto&& id_to_frame : view.second.frames_per_stream)
            {
                // If the frame is available
                if (id_to_frame.second)
                {
                    view.second.tex.upload(id_to_frame.second);
                }
                rect frame_location{ view_width * (stream_no % cols), view_height * (stream_no / cols), view_width, view_height };
                if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
                {
                    rect adjuested = frame_location.adjust_ratio({ static_cast<float>(vid_frame.get_width())
                                                                 , static_cast<float>(vid_frame.get_height()) });
                    view.second.tex.show(adjuested);
                    stream_no++;
                }
            }
        }
    }
private:
    std::mutex _mutex;
    std::map<std::string, view_port> _devices;
};

class RealsenseMultiCam : public QObject
{
    Q_OBJECT
public:
    explicit RealsenseMultiCam(QObject *parent = 0);

    int startCam();

    bool getNearBlockFromDepthData(rs2::depth_frame& depth_frame, QList<RobotPose>& block_point_list);
    bool tranfDepthPointToWorldXYZ(RobotPose& world_xyz, int depth_row, int depth_col, float depth_dis);
    void filterDepthPoint(QList<RobotPose>& blc_point_list);

    bool sendDepthData(rs2::depth_frame& depth_frame);

signals:
    void sendDepthPointSignal(QList<RobotPose> block_point_list);

public slots:
    void startRealsenseCameraSlot();
    void stopRealsenseCameraSlot();

private:
    bool camera_run_flag;
};

#endif // REALSENSEMULTICAM_H
