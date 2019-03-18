#ifndef ROBOTFACE_H
#define ROBOTFACE_H

#include <QObject>
#include <QMovie>
#include <QLabel>
#include <QDebug>

#define     DAI_JI_PATH         "face_gif/daiji.gif"
#define     SHUO_HUA_PATH       "face_gif/shuohua.gif"
#define     WEI_XIAO_PATH       "face_gif/weixiao.gif"
#define     AI_XING_PATH        "face_gif/aixing.gif"
#define     DAI_MA_PATH         "face_gif/daima.gif"
#define     KU_PATH             "face_gif/ku.gif"
#define     HAN_PATH            "face_gif/han.gif"
#define     HAI_XIU_PATH        "face_gif/haixiu.gif"
#define     WEI_QU_PATH         "face_gif/weiqu.gif"
#define     XING_XING_PATH      "face_gif/xingxing.gif"
#define     XIU_XI_PATH         "face_gif/xiuxi.gif"

class RobotFace : public QObject
{
    Q_OBJECT
public:
    explicit RobotFace(QObject *parent = 0);
    ~RobotFace();

    void initFaceLabel(QLabel* _face_label_p);

    bool showFace(QString face_id);


signals:

public slots:
    void showFaceSlot(QString face_id);

private:
    QMovie face_movie;
    QLabel* face_label_p;
};

#endif // ROBOTFACE_H
