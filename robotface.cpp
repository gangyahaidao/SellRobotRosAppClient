#include "robotface.h"

RobotFace::RobotFace(QObject *parent) : QObject(parent)
{
    face_label_p = 0;
}

RobotFace::~RobotFace()
{

}

void RobotFace::initFaceLabel(QLabel *_face_label_p)
{
   if(_face_label_p == 0)
       return;
   face_label_p = _face_label_p;
   face_movie.setFileName(DAI_JI_PATH);
   face_label_p->setMovie(&face_movie);
   face_movie.start();

}


bool RobotFace::showFace(QString face_id)
{
    if(face_label_p == 0)
        return false;

    QString face_gif_path;
    if(face_id == "DAI_JI_FACE_ID"){
        face_gif_path = DAI_JI_PATH;
    }else if(face_id == "SHUO_HUA_FACE_ID"){
        face_gif_path = SHUO_HUA_PATH;
    }else if(face_id == "WEI_XIAO_FACE_ID"){
        face_gif_path = WEI_XIAO_PATH;
    }else if(face_id == "AI_XING_FACE_ID"){
        face_gif_path = AI_XING_PATH;
    }else if(face_id == "DAI_MA_FACE_ID"){
        face_gif_path = DAI_MA_PATH;
    }else if(face_id == "KU_FACE_ID"){
        face_gif_path = KU_PATH;
    }else if(face_id == "HAN_FACE_ID"){
        face_gif_path = HAN_PATH;
    }else if(face_id == "HAI_XIU_FACE_ID"){
        face_gif_path = HAI_XIU_PATH;
    }else if(face_id == "WEI_QU_FACE_ID"){
        face_gif_path = WEI_QU_PATH;
    }else if(face_id == "XING_XING_FACE_ID"){
        face_gif_path = XING_XING_PATH;
    }else if(face_id == "XIU_XI_FACE_ID"){
        face_gif_path = XIU_XI_PATH;
    }else{
        return false;
    }

    face_movie.stop();
    face_movie.setFileName(face_gif_path);
    face_movie.start();

    return true;
}

void RobotFace::showFaceSlot(QString face_id)
{
    showFace(face_id);
}

