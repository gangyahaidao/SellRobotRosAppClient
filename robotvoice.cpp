#include "robotvoice.h"
#include <QDebug>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "qtts.h"

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char            riff[4];                // = "RIFF"
    int				size_8;                 // = FileSize - 8
    char            wave[4];                // = "WAVE"
    char            fmt[4];                 // = "fmt "
    int				fmt_size;				// = 下一个结构体的大小 : 16

    short int       format_tag;             // = PCM : 1
    short int       channels;               // = 通道数 : 1
    int				samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
    int				avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
    short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
    short int       bits_per_sample;        // = 量化比特数: 8 | 16

    char            data[4];                // = "data";
    int				data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
{
    { 'R', 'I', 'F', 'F' },
    0,
    {'W', 'A', 'V', 'E'},
    {'f', 'm', 't', ' '},
    16,
    1,
    1,
    16000,
    32000,
    2,
    16,
    {'d', 'a', 't', 'a'},
    0
};

/*wave_pcm_hdr wav_hdr = default_wav_hdr;
wav_hdr.data_size = speech_buf.size();
wav_hdr.size_8 = wav_hdr.data_size + (sizeof(wav_hdr) - 8);
char wav_head[100];
memcpy(wav_head, &wav_hdr, sizeof(wav_hdr));
speech_buf.prepend(wav_head, sizeof(wav_hdr));

QFile wav_file;
wav_file.setFileName("test.wav");
wav_file.open(QIODevice::WriteOnly);
wav_file.write(speech_buf.data(), speech_buf.size());
wav_file.close();
*/


RobotVoice::RobotVoice(QObject *parent) : QObject(parent)
{
    get_rlst_flag = true;
    get_speech_active_flag = false;
    audioOut_active_flag = false;

    voice_wakeUp_flag = true;

    //read_NavState_timer = new QTimer(this);
    //connect(read_NavState_timer, SIGNAL(timeout()), this, SLOT(readNavStateSlot()));

}

RobotVoice::RobotVoice(QString _port_name, QObject *parent) : QObject(parent)
{
    port_name = _port_name;

    get_rlst_flag = true;
    get_speech_active_flag = false;
    audioOut_active_flag = false;

    voice_wakeUp_flag = true;
}

RobotVoice::~RobotVoice()
{
    //xfei_serial_port->close();
}

bool RobotVoice::openXfeiRecorderComPort(QString port_name, int baud_rate)
{
    /*QList<QSerialPortInfo> portinfo_list = QSerialPortInfo::availablePorts();
    for(int i = 0; i < portinfo_list.size(); i++){
        QSerialPortInfo portinfo = portinfo_list.at(i);
        qDebug() << "port name:" << portinfo.portName() << "productIdentifier:" << portinfo.productIdentifier() << "vendorIdentifier" << portinfo.vendorIdentifier();

    }*/
    xfei_serial_port = new QSerialPort();
    xfei_serial_port->setPortName(port_name);
    xfei_serial_port->setBaudRate(baud_rate);
    xfei_serial_port->setDataBits(QSerialPort::Data8);
    xfei_serial_port->setStopBits(QSerialPort::OneStop);
    xfei_serial_port->setParity(QSerialPort::NoParity);
    xfei_serial_port->setFlowControl(QSerialPort::NoFlowControl);

    bool open_flag = xfei_serial_port->open(QIODevice::ReadWrite);

    if(open_flag){
        //qDebug() << "open xfei comm port sucess!";
    }else{
        qCritical() << "语音识别模块串口打开失败!";
    }

    connect(xfei_serial_port, SIGNAL(readyRead()), this, SLOT(readXfeiRecorderSerialPortDataSlot()));

    return true;
}

bool RobotVoice::resetXfeiRecorderWakeUp()
{
    const char* cmd = "RESET\n";
    qint64 wr_len = xfei_serial_port->write(cmd);
    if(wr_len > 0){
        return true;
    }else{
        return false;
    }
}

bool RobotVoice::enhanceXfeiRecorderVoiceBeam(int beam_id)
{
    //BEAM0 对应唤醒角度：330 - 30
    //BEAM1 对应唤醒角度：30 - 90
    //...
    //BEAM5 对应唤醒角度：270 - 330
    const char* cmd_0 = "BEAM 0\n";
    const char* cmd_1 = "BEAM 1\n";
    const char* cmd_2 = "BEAM 2\n";
    const char* cmd_3 = "BEAM 3\n";
    const char* cmd_4 = "BEAM 4\n";
    const char* cmd_5 = "BEAM 5\n";
    qint64 wr_len = 0;
    const char* cmd;
    if(beam_id == 0){
        cmd = cmd_0;
    }else if(beam_id == 1){
        cmd = cmd_1;
    }else if(beam_id == 2){
        cmd = cmd_2;
    }else if(beam_id == 3){
        cmd = cmd_3;
    }else if(beam_id == 4){
        cmd = cmd_4;
    }else if(beam_id == 5){
        cmd = cmd_5;
    }
    wr_len = xfei_serial_port->write(cmd);
    if(wr_len > 0){
        return true;
    }else{
        return false;
    }
}

bool RobotVoice::initRobotVoice(QString port_name)
{
    get_speech_timer = new QTimer(this);
    get_speech_local_timer = new QTimer(this);
    connect(get_speech_timer, SIGNAL(timeout()), this, SLOT(getSpeechTimerSlot()));
    connect(get_speech_local_timer, &QTimer::timeout, this, &RobotVoice::getSpeechTimerLocalSlot);

    initRecorder();
    initAudioOutput();

    bool flag1 = init_xfei();

//    if(port_name.isEmpty()){
//        port_name = "/dev/xfei_recorder";
//    }


    bool flag2 = openXfeiRecorderComPort(port_name, 115200);


    if(flag1 && flag2)
        return true;
    else
        return false;
}

void RobotVoice::initRecorder()
{
    QAudioFormat audioFormat;
    audioFormat.setSampleRate(16000);
    audioFormat.setChannelCount(1);
    audioFormat.setSampleSize(16);
    audioFormat.setCodec("audio/pcm");
    audioFormat.setByteOrder(QAudioFormat::LittleEndian);
    audioFormat.setSampleType(QAudioFormat::SignedInt);

    /*QList<QAudioDeviceInfo> dev_list = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    for(int i = 0; i < dev_list.size(); i++){
         QAudioDeviceInfo devinfo = dev_list.at(i);
         qDebug() << devinfo.deviceName();
    }*/

    audioInputDev_info = QAudioDeviceInfo::defaultInputDevice();
    if(!audioInputDev_info.isFormatSupported(audioFormat)){
        qDebug() << "default audio device not suport audio format!";
        audioFormat = audioInputDev_info.nearestFormat(audioFormat);
    }

    //audio_file.setFileName("test.raw");
    //audio_file.open(QIODevice::ReadOnly);

    //recorder_buffer.setBuffer(&audio_buf);
    recorder_buffer.open(QBuffer::ReadWrite);

    audio_input = new QAudioInput(audioInputDev_info, audioFormat, this);

    audio_input->setVolume(0.01);

    connect(&recorder_buffer, SIGNAL(readyRead()), this, SLOT(readAudioInputDataSlot()));
    //connect(audio_input, SIGNAL(stateChanged(QAudio::State)), this, SLOT(audioInputStateChangeSlot(QAudio::State)));

}

void RobotVoice::initAudioOutput()
{
    QAudioFormat audioFormat;
    audioFormat.setSampleRate(16000);
    audioFormat.setChannelCount(1);
    audioFormat.setSampleSize(16);
    audioFormat.setCodec("audio/pcm");
    audioFormat.setByteOrder(QAudioFormat::LittleEndian);
    audioFormat.setSampleType(QAudioFormat::SignedInt);

    audioOutputDev_info = QAudioDeviceInfo::defaultOutputDevice();
    if(!audioOutputDev_info.isFormatSupported(audioFormat)){
        qWarning() << "默认音频输出设备不支持当前设置的音频格式!";
        audioFormat = audioOutputDev_info.nearestFormat(audioFormat);
    }

    audio_output = new QAudioOutput(QAudioDeviceInfo::defaultOutputDevice(), audioFormat, this);

    connect(audio_output, SIGNAL(stateChanged(QAudio::State)), this, SLOT(audioOutputStateChangeSlot(QAudio::State)));
}

void RobotVoice::startRecording()
{
    if(!audio_input){
        //qDebug() << "audio_input is null, return!";
        return;
    }

    //qDebug() << "start audio input...";

    last_pos = 0;
    recorder_buffer.reset();
    audio_input->start(&recorder_buffer);

}

void RobotVoice::stopRecording()
{
    if(!audio_input){
        //qDebug() << "audio_input is null, return!";
        return;
    }

    audio_input->stop();

}

bool RobotVoice::init_xfei()
{
    const char* login_params = "appid = 5aaa3def, work_dir = .";

    int ret = MSP_SUCCESS;
    ret = MSPLogin(NULL, NULL, login_params);
    if(MSP_SUCCESS != ret){
        qWarning("xfei msp login failed, err code:%d", ret);
        MSPLogout();
        return false;
    }

    return true;
}

bool RobotVoice::startListen()
{
    if(!get_rlst_flag){
        //检查当前是否正在识别
        stopListen();
    }

    const char* session_begin_params = "sub = iat, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8";

    int err_code = MSP_SUCCESS;
    session_id_isr = NULL;

    session_id_isr = QISRSessionBegin(NULL, session_begin_params, &err_code); //听写不需要语法，第一个参数为NULL
    if(MSP_SUCCESS != err_code){
        qWarning("QISRSessionBegin failed! error code:%d", err_code);
        return false;
    }

    //检查当前是否在合成语音
    if(get_speech_active_flag){
        stopTextToSpeech();
        if(audio_output->state() != QAudio::StoppedState){
            audio_output->stop();
        }
    }

    //清除录音数据，识别结果数组清0
    first_audio_flag = true;
    get_rlst_flag = false;
    memset(speech_rlst, 0, sizeof(speech_rlst));

    startRecording();

    //检测超时计时器开始
    c_time.start();

    return true;
}

bool RobotVoice::stopListen()
{
    get_rlst_flag = true;
    stopRecording();

    if(session_id_isr == NULL)
        return true;

    int ret = QISRSessionEnd(session_id_isr, "normal end");

    if(MSP_SUCCESS != ret)
        return false;
    else
        return true;

}

void RobotVoice::enableVoiceWakeUp()
{
    voice_wakeUp_flag = true;
}

void RobotVoice::disAbleVoiceWakeUp()
{
    voice_wakeUp_flag = false;
}

bool RobotVoice::getVoiceWakeUpState()
{
    return voice_wakeUp_flag;
}


/*
bool RobotVoice::startTextToSpeech(const char *text)
{
    const char* session_begin_params = "engine_type = cloud,voice_name=xiaomeng, text_encoding = UTF8, sample_rate = 16000, speed = 60, volume = 70, pitch = 60, rdn = 2";
    int ret;

    if(!get_speech_flag){
        stopTextToSpeech();
    }

    session_id_tts = QTTSSessionBegin(session_begin_params, &ret);
    if(MSP_SUCCESS != ret){
        qDebug("text to speech session begin fail, err code:%d", ret);
        return false;
    }
    unsigned int txt_len = strlen(text);
    //qDebug("txt len:%d", txt_len);
    ret = QTTSTextPut(session_id_tts, text, txt_len, NULL);
    if(MSP_SUCCESS != ret){
        qDebug("text to speech text put fail, err code:%d", ret);
        return false;
    }
    speech_data_qba.clear();
    get_speech_flag = false;
    count_time = 0;
    get_speech_timer->start(50);

    return true;
}
*/

bool RobotVoice::startLongTextToSpeech(const char *text)
{
    //文字比较长的情况下先对文字进行分段，在语音播报的同时对下一段文字进行转换，减少等待时间
    const char* session_begin_params = "engine_type = cloud,voice_name=xiaomeng, text_encoding = UTF8, sample_rate = 16000, speed = 60, volume = 70, pitch = 60, rdn = 2";
    int ret;

    if(get_speech_active_flag){
        stopTextToSpeech();
    }

    session_id_tts = QTTSSessionBegin(session_begin_params, &ret);
    if(MSP_SUCCESS != ret){
        qWarning("text to speech session begin fail, err code:%d", ret);
        return false;
    }

    tts_list.clear();
    splitLongText(tts_list, text, 30);

    voice_data_list.clear();
    speech_data_qba.clear();
    get_speech_flag = false;
    get_speech_err_flag = false;
    tts_finish_flag = false;
    //audioOut_active_flag = false;
    get_speech_active_flag = true;
    tts_list_idx = 0;
    voice_data_idx = -1;
    count_time = 0;

    QString qs = tts_list.at(tts_list_idx);
    QByteArray qba = qs.toUtf8();
    ret = QTTSTextPut(session_id_tts, qba.data(), qba.length(), NULL);
    if(MSP_SUCCESS != ret){
        qWarning("text to speech text put fail, err code:%d", ret);
        return false;
    }

    get_speech_timer->start(50);

    c_time.start();

    return true;

}

//按照标点符号对长文字进行分段
void RobotVoice::splitLongText(QStringList& txt_list, const char *txt, int split_n)
{
    QString qs = QString(txt);
    if(qs.length() <= split_n){
        txt_list << qs;
        return;
    }
    QStringList cutqs_list;
    QRegExp regexp("[，；。！？：,;.!?:]");
    int pos = 0;
    int start_pos = 0, cut_len;
    while( (pos = regexp.indexIn(qs, pos)) != -1){
       cut_len = pos - start_pos;
       QString qs_cut = qs.mid(start_pos, cut_len+1);
       cutqs_list << qs_cut;
       pos++;
       start_pos = pos;
    }
    //qDebug("string leng:%d, pos+1:%d", qs.length(), start_pos);
    if(start_pos < qs.length()){
        cut_len = qs.length() - start_pos;
        QString qs_cut = qs.mid(start_pos, cut_len);
        cutqs_list << qs_cut;
    }

    QString qs_t;
    for(int i = 0; i < cutqs_list.size(); i++){
        qs_t.append( cutqs_list.at(i) );
        if(qs_t.length() > split_n){
            txt_list << qs_t;
            qs_t.clear();
        }
    }
    if(qs_t.length() > 0){
        txt_list << qs_t;
    }

    /*if(cutqs_list.size() > 0){
        qDebug() << "list has data";
        for(int i = 0; i < cutqs_list.size(); i++){
            qDebug() << i << ":" << cutqs_list.at(i);
        }
    }else{
        qDebug() << "list size is 0";
    }*/

}

void RobotVoice::startAudioOutput(QByteArray& voice_data)
{
    if(audioOut_active_flag){
        audio_output->stop();
    }
    speech_audio_buf.close();
    speech_audio_buf.setData(voice_data);
    speech_audio_buf.open(QBuffer::ReadWrite);
    //qDebug("speec aduio buf size:%lld", speech_audio_buf.size());

    audio_output->start(&speech_audio_buf);
}

void RobotVoice::writeAudioData(QByteArray &recorder_data_qba)
{
    if(session_id_isr == NULL){
        stopListen();
        return;
    }

    if(recorder_data_qba.size() == 0)
        return;

    int ret = MSP_SUCCESS, ep_status = 0, rslt_status = 0, audio_status;
    int audio_len = recorder_data_qba.size();
    //qDebug("wr audio len:%d", audio_len);
    if(first_audio_flag){
        audio_status = MSP_AUDIO_SAMPLE_FIRST;
        first_audio_flag = false;
    }else{
        audio_status = MSP_AUDIO_SAMPLE_CONTINUE;
    }

    ret = QISRAudioWrite(session_id_isr, recorder_data_qba.data(), audio_len, audio_status, &ep_status, &rslt_status);
    if(MSP_SUCCESS != ret){
        qWarning("audio write failed, err code:%d", ret);
    }

    while( MSP_REC_STATUS_SUCCESS == rslt_status){
        //已有部分识别结果
        ret = MSP_SUCCESS;
        const char* rec_rlst = QISRGetResult(session_id_isr, &rslt_status, 0, &ret);
        if( MSP_SUCCESS != ret){
            break;
        }
        if(rec_rlst != NULL){
            strcat(speech_rlst, rec_rlst);
            //qDebug("get rlst:%s", speech_rlst);
            continue;
        }
    }

    if(MSP_EP_AFTER_SPEECH == ep_status){
        //检测到音频后端点，停止录音
        //qDebug() << "stop listen after det audio finish";
        stopListen();
        emit getSpeakTextSignal(QString(speech_rlst));
    }
}

void RobotVoice::startTextToSpeechLocalSlot(QString spk_qs)
{
    startTextToSpeechLocal(spk_qs);
}

void RobotVoice::startListenSlot()
{
    startListen();
}

void RobotVoice::stopListenSlot()
{
    stopListen();
}

void RobotVoice::setVoiceWakeUpStateSlot(bool enable_flag)
{
    if(enable_flag){
        enableVoiceWakeUp();
    }else{
        disAbleVoiceWakeUp();
    }
}

void RobotVoice::enableVoiceWakeUpSlot()
{
    enableVoiceWakeUp();
}

void RobotVoice::disAbleVoiceWakeUpSlot()
{
    disAbleVoiceWakeUp();
}

void RobotVoice::enhanceXfeiRecorderVoiceBeamSlot(int beam_id)
{
    enhanceXfeiRecorderVoiceBeam(beam_id);
}

bool RobotVoice::stopTextToSpeech()
{
    get_speech_active_flag = false;
    if(get_speech_timer->isActive())
        get_speech_timer->stop();
    if(get_speech_local_timer->isActive())
        get_speech_local_timer->stop();
    if(session_id_tts != NULL){
        QTTSSessionEnd(session_id_tts, "AudioGetEnd");
    }
    session_id_tts = NULL;
    return true;
}

bool RobotVoice::waitAudioOutPutComplete(int max_wait_second)
{
    //audioOut_active_flag = true;
    int count_time = 0;
    bool start_flag = false;
    while(1){
      if(!start_flag && audioOut_active_flag){
         //qDebug() << "audio play start...";
         start_flag = true;
      }
      if(start_flag && !audioOut_active_flag)
          break;
      delayMs(100);
      count_time++;
      if(count_time / 10 > max_wait_second){
          //qDebug() << "wait audio play complete time out!";
          break;
      }
    }
    if(audioOut_active_flag)
        return false;
    else
        return true;
}

//离线语音合成
bool RobotVoice::startTextToSpeechLocal(const char *text)
{
    const char* session_begin_params =
            "engine_type = local,voice_name=nannan, text_encoding = UTF8, tts_res_path = fo|res/tts/nannan.jet;fo|res/tts/common.jet, sample_rate = 16000, speed = 70, volume = 80, pitch = 70, rdn = 2";
    int ret;

    if(get_speech_active_flag){
        //qDebug() << "start tts stop tts first";
        stopTextToSpeech();
    }

    session_id_tts = QTTSSessionBegin(session_begin_params, &ret);
    if(MSP_SUCCESS != ret){
        qWarning("text to speech local session begin fail, err code:%d", ret);
        return false;
    }

    speech_data_qba.clear();
    get_speech_active_flag = true;

    QString qs = QString(text);
    QByteArray qba = qs.toUtf8();
    ret = QTTSTextPut(session_id_tts, qba.data(), qba.length(), NULL);
    if(MSP_SUCCESS != ret){
        qWarning("text to speech local text put fail, err code:%d", ret);
        return false;
    }

    get_speech_local_timer->start(50);

    return true;

}

bool RobotVoice::startTextToSpeechLocal(QString text_qs)
{
    QByteArray qba = text_qs.toLocal8Bit();
    bool flag = startTextToSpeechLocal(qba.data());
    return flag;
}



bool RobotVoice::uploadUserWord(QString word_file)
{
    int ret;
    QFile file;
    file.setFileName(word_file); //"userwords.txt"
    if(file.open(QIODevice::ReadOnly | QIODevice::Text)){
        //qDebug() << "open user word file fail, return";
        return false;
    }

    QByteArray qba = file.readAll();
    char* userwords = qba.data();
    int len = qba.size();
    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret);
    if(MSP_SUCCESS != ret){
        //qDebug("upload userword fail, err code:%d", ret);
        return false;
    }

    return true;
}

//延时，当延时被语音唤醒打断时，返回false，延时正常完成，返回true
bool RobotVoice::delayMs(unsigned int ms)
{
    delay_interupt_flag = false;
    QTime delay_time = QTime::currentTime().addMSecs(ms);
    while(QTime::currentTime() < delay_time){
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        if(delay_interupt_flag)
            break;
    }
    return !delay_interupt_flag;
}

void RobotVoice::readAudioInputDataSlot()
{
    int listen_time_ms = c_time.elapsed();
    //qDebug("audio input start time:%d ms", listen_time_ms);
    if(listen_time_ms > 5000){
        //qDebug() << "listen time out, stop recorder!";
        stopListen();
        QString empty_qs;
        emit getSpeakTextSignal(empty_qs);
        return;
    }

    //qDebug("data size:%lld, pos:%lld", recorder_buffer.size(), recorder_buffer.pos());
    qint64 curr_pos = recorder_buffer.pos();
    qint64 data_len = curr_pos - last_pos;
    //qDebug("cal data_len:%lld", data_len);

    QByteArray qba;
    recorder_buffer.seek(last_pos);
    qba = recorder_buffer.read(data_len);
    //qDebug("rd data size:%d, after rd pos:%lld", qba.size(), recorder_buffer.pos());

    //recorder_data_qba.append(qba);
    writeAudioData(qba);

    last_pos += qba.size();
}

void RobotVoice::audioInputStateChangeSlot(QAudio::State new_state)
{
    Q_UNUSED(new_state);
    /*switch (new_state) {
            case QAudio::StoppedState:
                qDebug() << "change to stop state";
                if (audio_input->error() != QAudio::NoError) {
                    // Error handling
                    qDebug() << "err det";
                } else {
                    // Finished recording
                    qDebug() << "no err";
                }
                break;

            case QAudio::ActiveState:
                // Started recording - read from IO device
                qDebug() << "change to active state";
                break;
            case QAudio::IdleState:
                qDebug() << "change to idle state";
                break;

            default:
                // ... other cases as appropriate
                break;
    }*/
}

void RobotVoice::audioOutputStateChangeSlot(QAudio::State new_state)
{
    switch (new_state) {
            case QAudio::StoppedState:
                //qDebug() << "audioOutput change to stop state";
                if (audio_input->error() != QAudio::NoError) {
                    // Error handling
                    qDebug() << "audioOutput err det";
                } else {
                    //qDebug() << "audioOutput no err";
                }
                audioOut_active_flag = false;
                break;

            case QAudio::ActiveState:
                //qDebug() << "audioOutput change to active state";
                audioOut_active_flag = true;
                break;
            case QAudio::IdleState:
                //qDebug() << "audioOutput change to idle state";
                audioOut_active_flag = false;
                emit audioPlayFinishSignal();
                break;

            default:
                // ... other cases as appropriate
                break;
    }
}

/*
void RobotVoice::recorderTimerSlot()
{
    count_time++;
    if(count_time > 120){
        //50×120ms超时退出
        qDebug("record time out, return");
        get_rlst_flag = true;
        stopListen();
        //emit getSpeakTextSignal(QString(speech_rlst));
        //speechProcess(speech_rlst);
        return;
    }

    if(session_id_isr == NULL){
        stopListen();
        return;
    }

    if(!audio_read_flag)
        return;

    if(recorder_data_qba.size() == 0)
        return;

    int ret = MSP_SUCCESS, ep_status = 0, rslt_status = 0, audio_status;
    int audio_len = recorder_data_qba.size();
    qDebug("wr audio len:%d", audio_len);
    if(first_audio_flag){
        audio_status = MSP_AUDIO_SAMPLE_FIRST;
        first_audio_flag = false;
    }else{
        audio_status = MSP_AUDIO_SAMPLE_CONTINUE;
    }

    ret = QISRAudioWrite(session_id_isr, recorder_data_qba.data(), audio_len, audio_status, &ep_status, &rslt_status);
    if(MSP_SUCCESS != ret){
        qDebug("audio write failed, err code:%d", ret);
    }

    recorder_data_qba.clear();

    while( MSP_REC_STATUS_SUCCESS == rslt_status){
        //已有部分识别结果
        ret = MSP_SUCCESS;
        const char* rec_rlst = QISRGetResult(session_id_isr, &rslt_status, 0, &ret);
        if( MSP_SUCCESS != ret){
            break;
        }
        if(rec_rlst != NULL){
            strcat(speech_rlst, rec_rlst);
            qDebug("get rlst:%s", speech_rlst);
            continue;
        }
    }

    if(MSP_EP_AFTER_SPEECH == ep_status){
        //检测到音频后端点，停止录音
        qDebug() << "stop listen after det audio finish";
        get_rlst_flag = true;
        stopListen();
        emit getSpeakTextSignal(QString(speech_rlst));
    }
}
*/

void RobotVoice::getSpeechTimerSlot()
{
    static int count;
    int ret;
    count++;
    if(get_speech_err_flag){
        //转换出现错误，停止转换
        stopTextToSpeech();
        return;
    }

    if(get_speech_flag){
        //一次转换完成,开始下一段转换

        voice_data_list.append(speech_data_qba);

        get_speech_flag = false;
        speech_data_qba.clear();
        tts_list_idx++;
        if(tts_list_idx < tts_list.length()){
            //qDebug("start id:%d txt_list put", tts_list_idx);
            const char* session_begin_params = "engine_type = cloud,voice_name=xiaomeng, text_encoding = UTF8, sample_rate = 16000, speed = 60, volume = 70, pitch = 60, rdn = 2";

            if(session_id_tts != NULL){
                QTTSSessionEnd(session_id_tts, "AudioGetEnd");
            }
            session_id_tts = QTTSSessionBegin(session_begin_params, &ret);
            if(MSP_SUCCESS != ret){
                qWarning("text to speech session begin fail, err code:%d", ret);
                stopTextToSpeech();
                return;
            }
            QString qs = tts_list.at(tts_list_idx);
            QByteArray qba = qs.toUtf8();
            ret = QTTSTextPut(session_id_tts, qba.data(), qba.length(), NULL);
            if(MSP_SUCCESS != ret){
                qWarning("text to speech text put fail, err code:%d", ret);
                stopTextToSpeech();
                return;
            }
        }else{
            //所有分段文字都转换完成
            int tf_ms = c_time.restart();
            //qDebug() << "all txt put,tts time:" << tf_ms << "ms" << ", count num:" << count;
            count = 0;
            tts_finish_flag = true;
        }

    }


    if(!audioOut_active_flag && voice_data_idx + 1 < voice_data_list.length()){
        //当前声音播放完成，声音列表中有新的数据，继续播放新的数据
        //qDebug() << "play audio...";
        audio_out_data.clear();
        for(int i = voice_data_idx + 1; i < voice_data_list.length(); i++){
            audio_out_data.append(voice_data_list.at(i));
        }
        voice_data_idx = voice_data_list.length() - 1;
        startAudioOutput(audio_out_data);

        if(tts_finish_flag){
            stopTextToSpeech();
        }
    }

    int synth_status;
    unsigned int audio_len;
    while(!tts_finish_flag){
        const void* data = QTTSAudioGet(session_id_tts, &audio_len, &synth_status, &ret);
        if(MSP_SUCCESS != ret){
            //stopTextToSpeech();
            get_speech_err_flag = true;
            qWarning("text to speech get audio fail, err code:%d", ret);
            break;
        }
        if(NULL == data){
            //qDebug() << "get audio data is null!";
            break;
        }
        speech_data_qba.append((const char*)data, audio_len);
        if(MSP_TTS_FLAG_DATA_END == synth_status){
            get_speech_flag = true;
            break;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }

    /*int ret, synth_status;
    unsigned int audio_len;
    //bool tts_err_flag = false;
    while(1){
        const void* data = QTTSAudioGet(session_id_tts, &audio_len, &synth_status, &ret);
        if(MSP_SUCCESS != ret){
            stopTextToSpeech();
            qDebug("text to speech get audio fail, err code:%d", ret);
            break;
        }
        if(NULL == data){
            //qDebug() << "get audio data is null!";
            break;
        }
        speech_data_qba.append((const char*)data, audio_len);
        if(MSP_TTS_FLAG_DATA_END == synth_status){
            get_speech_flag = true;
            break;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }

    //qDebug("speech data size:%d", speech_data_qba.size());

    if(get_speech_flag && synth_status == MSP_TTS_FLAG_DATA_END){
        qDebug() << "get speech sucess";
        stopTextToSpeech();
        speech_audio_buf.close();
        speech_audio_buf.setData(speech_data_qba);
        speech_audio_buf.open(QBuffer::ReadWrite);
        //qDebug("speec aduio buf size:%lld", speech_audio_buf.size());

        audio_output->start(&speech_audio_buf);
    }*/

}

void RobotVoice::getSpeechTimerLocalSlot()
{
    int ret, synth_status;
    unsigned int audio_len;
    //qDebug() << "enter timer slot..";
    while(1){
        const void* data = QTTSAudioGet(session_id_tts, &audio_len, &synth_status, &ret);
        if(MSP_SUCCESS != ret){
            stopTextToSpeech();
            qWarning("text to speech get audio fail, err code:%d", ret);
            break;
        }
        if(NULL == data){
            //qDebug() << "get audio data is null!";
            break;
        }
        //qDebug("add speech data:%d", audio_len);
        speech_data_qba.append((const char*)data, audio_len);
        if(MSP_TTS_FLAG_DATA_END == synth_status){
            qDebug() << "get speech sucess";
            stopTextToSpeech();
            speech_audio_buf.close();
            speech_audio_buf.setData(speech_data_qba);
            speech_audio_buf.open(QBuffer::ReadWrite);
            //qDebug("speec aduio buf size:%lld", speech_audio_buf.size());

            audio_output->start(&speech_audio_buf);
            break;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    }

    //qDebug("speech data size:%d", speech_data_qba.size());

}


void RobotVoice::readXfeiRecorderSerialPortDataSlot()
{
    static QByteArray serialport_data_qba;
    serialport_data_qba.append(xfei_serial_port->readAll());
    QString data_qs = QString(serialport_data_qba);
    //qDebug() << "rec data:" << data_qs;
    //使用正则表达式从串口接收到的字符串提取唤醒角度
    //WAKE UP!angle:75 score:1720  key_word: xiao3
    //命令:VER\n   RESET\n ...
    QRegExp regexp( "angle:(\\d+)\\s*score:(\\d+)\\s+" );
    int pos = 0;
    //从data_qs中从pos位置开始查找匹配的字符串，返回匹配到的第一个字符位置
    if( (pos = regexp.indexIn(data_qs, pos)) != -1){
        serialport_data_qba.clear();
        QString angle_qs = regexp.cap(1);
        QString score_qs = regexp.cap(2);
        //qDebug() << "robot voice wake up angle:" << angle_qs << ", score:" << score_qs;
        if(voice_wakeUp_flag){
            delay_interupt_flag = true;
            emit voiceWakeUpDetSignal(angle_qs);
        }
    }
}

