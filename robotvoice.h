#ifndef ROBOTVOICE_H
#define ROBOTVOICE_H

#include <QObject>
#include <QTime>
#include <QCoreApplication>
#include <QByteArray>
#include <QtMultimedia>
#include <QList>
#include <QFile>
#include <QBuffer>
#include <QTimer>
//#include <QSound>
#include <QRegExp>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>



class RobotVoice : public QObject
{
    Q_OBJECT
public:
    explicit RobotVoice(QObject *parent = 0);
    explicit RobotVoice(QString _port_name, QObject *parent = 0);
    ~RobotVoice();

    //初始化语音模块
    bool initRobotVoice(QString port_name);

    //开始在线语音合成
    bool startLongTextToSpeech(const char* text);
    //停止在线语音合成
    bool stopTextToSpeech();

    //等待语音播放结束
    bool waitAudioOutPutComplete(int max_wait_second);

    //开始离线语音合成
    bool startTextToSpeechLocal(const char* text);
    bool startTextToSpeechLocal(QString text_qs);

    //开始语音识别
    bool startListen();
    //停止语音识别
    bool stopListen();

    //语音唤醒设置
    void enableVoiceWakeUp();
    void disAbleVoiceWakeUp();
    bool getVoiceWakeUpState();

    //当设置为允许发送语音唤醒信号，延时会被语音唤醒打断，返回true表示延时正常完成，返回false表示延时被打断
    bool delayMs(unsigned int ms);

    //增强某个方向语音接收
    bool enhanceXfeiRecorderVoiceBeam(int beam_id);

    //重置讯飞语音硬件模块
    bool resetXfeiRecorderWakeUp();

private:
    bool openXfeiRecorderComPort(QString port_name, int baud_rate);

    void initRecorder();
    void initAudioOutput();

    bool init_xfei();

    void startRecording();
    void stopRecording();

    bool uploadUserWord(QString word_file);

    void splitLongText(QStringList& txt_list, const char* txt, int split_n = 30);
    void startAudioOutput(QByteArray& voice_data);

    void writeAudioData(QByteArray& recorder_data_qba);

signals:
    //语音识别完成后，通过此信号发送识别结果文本
    void getSpeakTextSignal(QString spk_qs);
    //检测到唤醒词，通过此信号发送唤醒声音方位
    void voiceWakeUpDetSignal(QString angle_qs);

    //void writeAudioDataSignal();

    //文字转语音音频播放完成信号
    void audioPlayFinishSignal();

public slots:
    //开始本地TTS
    void startTextToSpeechLocalSlot(QString spk_qs);
    //语音识别
    void startListenSlot();
    void stopListenSlot();
    //设置是否开启语音唤醒
    void setVoiceWakeUpStateSlot(bool enable_flag);
    void enableVoiceWakeUpSlot();
    void disAbleVoiceWakeUpSlot();
    //增强某个通道语音
    void enhanceXfeiRecorderVoiceBeamSlot(int beam_id);

private slots:
    //读取录音数据
    void readAudioInputDataSlot();
    //音频输入或输出状态变化
    void audioInputStateChangeSlot(QAudio::State new_state);
    void audioOutputStateChangeSlot(QAudio::State new_state);
    //在线文字转语音定时器执行函数
    void getSpeechTimerSlot();
    //离线文字转语音定时器执行函数
    void getSpeechTimerLocalSlot();
    //读取语音模块板串口数据
    void readXfeiRecorderSerialPortDataSlot();



private:
    QAudioInput* audio_input;
    QAudioOutput* audio_output;
    //录音输入数据
    QBuffer recorder_buffer;
    //QByteArray recorder_data_qba;   // 声音数据量（字节/秒）= (采样频率（Hz）× 采样位数（bit） × 声道数)/ 8
    qint64 last_pos;
    QAudioDeviceInfo audioInputDev_info;
    QAudioDeviceInfo audioOutputDev_info;
    QFile audio_file;

    QTimer* get_speech_timer;         //在线语音合成定时器
    QTimer* get_speech_local_timer;  //离线语音合成定时器
    QByteArray speech_data_qba;     //获取到的文字转语音数据
    QByteArray audio_out_data;      //播放的语音数据
    QBuffer speech_audio_buf;
    bool audioOut_active_flag; //当前是否在输出声音
    bool get_speech_flag;       //文字转语音结束标志
    bool get_speech_active_flag;  //当前文字转语音在运行标志
    bool get_rlst_flag;     //语音识别结束标志
    bool audio_read_flag;   //为真则没有在读取录音数据，可以从录音缓冲区读取数据，为假，则录音缓冲区正在写入数据，不能读取
    bool first_audio_flag;
    char speech_rlst[2048];

    bool get_speech_err_flag;   //文字转语音出错标志
    bool tts_finish_flag;       //文字转语音完成标志
    QList<QByteArray> voice_data_list;  //文字转语音分段声音数据
    QStringList tts_list;       //长文字切割后分段内容
    int tts_list_idx;           //分段文字id
    int voice_data_idx;         //分段声音id


    QSerialPort* xfei_serial_port;  //串口对象
    QString port_name;              //串口名

    const char* session_id_isr;  //讯飞服务器交互的对话id
    const char* session_id_tts;

    int count_time;

    QTime c_time;  //计时器，用于计算执行时间或检测语音识别时录音超时

    bool voice_wakeUp_flag;         //是否发送语音唤醒信号
    bool delay_interupt_flag;       //延时过程中如果有唤醒，则退出延时

};

#endif // ROBOTVOICE_H
