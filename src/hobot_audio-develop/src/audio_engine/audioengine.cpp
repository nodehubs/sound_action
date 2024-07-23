// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "audio_engine/audioengine.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace audio {

    // VoIP数据回调函数
    void VoipDataCallback(const void*, const HrscCallbackData* data) {
        if (!data) return;  // 如果数据为空，则直接返回

        // 打印接收到的音频数据信息：角度、评分、数据大小
        RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
            "recv hrsc sdk callback audio, angle:%f, score:%f, data size:%d",
            data->angle, data->score, data->audio_buffer.size);

        // 调用注册的回调函数处理音频数据
        if (AudioEngine::Instance()->GetAudioDataCb()) {
            AudioEngine::Instance()->GetAudioDataCb()(
                reinterpret_cast<char*>(data->audio_buffer.audio_data),  // 将音频数据转换为char指针
                data->audio_buffer.size);  // 音频数据大小
        }

        // if (AudioEngine::Instance()->GetAudioSmartDataCb()) {
        //   AudioEngine::Instance()->GetAudioSmartDataCb()(data->angle);
        // }
    }

    // 唤醒数据回调函数
    void WakeupDataCallback(const void*, const HrscCallbackData* data,
        const int keyword_index) {
        if (!data) return;  // 如果数据为空，则直接返回

        // 打印接收到的唤醒数据信息：数据大小和关键词索引
        std::cout << "recv hrsc sdk wakeup data , size is " << data->audio_buffer.size
            << ", key index:" << keyword_index << std::endl;
    }

    // 事件回调函数
    void EventCallback(const void*, HrscEventType event) {
        static int wkp_count = 0;  // 静态计数器，记录事件触发次数

        // 如果事件是正常唤醒或单次唤醒
        if (event == kHrscEventWkpNormal || event == kHrscEventWkpOneshot) {
            std::cout << "recv hrsc sdk event wakeup success, wkp count is "
                << ++wkp_count << std::endl;  // 打印唤醒成功的事件和计数

            // 调用注册的事件回调函数处理事件
            if (AudioEngine::Instance()->GetAudioEventCb()) {
                AudioEngine::Instance()->GetAudioEventCb()(event);  // 将事件类型传递给回调函数
            }
        }
    }

    // 命令数据回调函数
    void CmdDataCallback(const void*, const char* cmd) {
        if (!cmd) return;  // 如果命令为空，则直接返回

        // 打印接收到的命令数据信息
        std::cout << "recv hrsc sdk command data: " << cmd << std::endl;    ////

        // 调用注册的命令数据回调函数处理命令数据
        if (AudioEngine::Instance()->GetAudioCmdDataCb()) {
            AudioEngine::Instance()->GetAudioCmdDataCb()(cmd);  // 将命令数据传递给回调函数
        }
    }

    // 方向角数据回调函数
    void DoaCallback(const void*, int doa) {
        // 打印接收到的方向角数据信息
        std::cout << "recv hrsc sdk doa data: " << doa << std::endl;  ////

        // 调用注册的智能音频数据回调函数处理方向角数据
        if (AudioEngine::Instance()->GetAudioSmartDataCb()) {
            AudioEngine::Instance()->GetAudioSmartDataCb()(doa);  // 将方向角数据传递给回调函数
        }
    }

    // ASR识别结果回调函数
    void AsrCallback(const void*, const char* asr) {
        // 打印ASR识别结果信息
        std::cout << "asr is: " << asr << std::endl;

        // 调用注册的ASR事件回调函数处理ASR识别结果
        if (AudioEngine::Instance()->GetASREventCb()) {
            AudioEngine::Instance()->GetASREventCb()(asr);  // 将ASR识别结果传递给回调函数
        }
    }
    
// AudioEngine类的构造函数
AudioEngine::AudioEngine() {}

// AudioEngine类的析构函数
AudioEngine::~AudioEngine() {
  if (adapter_buffer_) delete[] adapter_buffer_;
}

// 初始化音频引擎
int AudioEngine::Init(AudioDataFunc audio_cb, AudioSmartDataFunc audio_smart_cb,
                      AudioCmdDataFunc cmd_cb, AudioEventFunc event_cb,
                      AudioASRDataFunc asr_cb,
                      const int mic_chn, const std::string config_path,
                      const int voip_mode, const int mic_type,
                      const int asr_output_mode, const int asr_output_channel) {
  if (init_) {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
                "has already initialized.");  // 如果已经初始化过，则发出警告
    return 0;
  }

  // 设置音频引擎的参数
  mic_chn_num_ = mic_chn;         // 麦克风通道数
  voip_mode_ = voip_mode;         // VoIP模式
  mic_type_ = mic_type;           // 麦克风类型
  asr_mode_ = asr_output_mode;    // ASR输出模式
  asr_channel_ = asr_output_channel;  // ASR输出通道数
  sdk_file_path_ = config_path + "/hrsc";  // SDK配置文件路径

  // 如果SDK输入通道数大于麦克风通道数，则设置为3通道
  if (sdkin_chn_num_ > mic_chn_num_) {
    sdkin_chn_num_ = 3;
  }

  // 初始化音频SDK
  int ret = InitSDK();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "init audio sdk fail");
    return ret;
  }

  // 将回调函数注册到音频引擎中
  audio_cb_ = audio_cb;
  audio_smart_cb_ = audio_smart_cb;
  audio_cmd_cb_ = cmd_cb;
  audio_event_cb_ = event_cb;
  audio_asr_cb_ = asr_cb;
  init_ = true;  // 标记音频引擎已初始化成功
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "init hrsc sdk success!");
  return 0;
}

// 反初始化音频引擎
int AudioEngine::DeInit() { return 0; }

// 启动音频引擎
int AudioEngine::Start() {
    if (!init_) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "engine not init.");  // 如果未初始化，则发出错误信息
        return -1;
    }

    if (start_) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
            "engine has already started.");  // 如果已经启动，则发出错误信息
        return 0;
    }

    start_ = true;  // 标记音频引擎已经启动
    if (save_file_) {
        audio_inconvert_file_.open(
            "./audio_sdk.pcm", std::ios::app | std::ios::out | std::ios::binary);  // 打开音频文件用于保存数据
    }

    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine start success");
    return 0;
}

// 重置音频引擎
int AudioEngine::Reset() { return 0; }

// 停止音频引擎
int AudioEngine::Stop() {
    if (!start_) return 0;  // 如果未启动，则直接返回

    DeInitSDK();  // 反初始化音频SDK
    if (audio_inconvert_file_.is_open()) audio_inconvert_file_.close();  // 关闭音频文件
    return 0;  // 返回0，表示成功停止
}

// 初始化音频SDK
int AudioEngine::InitSDK() {
    input_cfg_.audio_channels = sdkin_chn_num_;  // 设置SDK输入的音频通道数
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "audioengine sdk in chn:%d",
        sdkin_chn_num_);  // 打印SDK输入通道数
    input_cfg_.sample_rate = 16000;  // 设置输入采样率
    input_cfg_.audio_format = kHrscAudioFormatPcm16Bit;  // 设置输入音频格式为16位PCM

    output_cfg_.audio_channels = 1;  // 设置输出的音频通道数
    output_cfg_.sample_rate = 16000;  // 设置输出采样率
    output_cfg_.audio_format = kHrscAudioFormatPcm16Bit;  // 设置输出音频格式为16位PCM
    memset(&effect_cfg_, 0, sizeof(HrscEffectConfig));  // 清空效果配置结构体的内存
    effect_cfg_.input_cfg = input_cfg_;  // 设置效果配置的输入配置
    effect_cfg_.output_cfg = output_cfg_;  // 设置效果配置的输出配置
    effect_cfg_.priv = &effect_cfg_;  // 设置效果配置的私有数据指针
    effect_cfg_.asr_timeout = 5000;  // 设置ASR超时时间为5000ms
    effect_cfg_.cfg_file_path = sdk_file_path_.c_str();  // SDK配置文件路径

    // 如果不是VoIP模式
    if (!voip_mode_) {
        std::string cus_path = sdk_file_path_ + "/cmd_word.json";  // 自定义命令词路径
        RCLCPP_INFO(rclcpp::get_logger("audio_capture"),
            "hrsc sdk file path:%s, cmd file:%s", sdk_file_path_.c_str(),
            cus_path.c_str());  // 打印SDK文件路径和命令文件路径
        if (cus_path.empty()) {
            effect_cfg_.custom_wakeup_word = nullptr;  // 如果路径为空，自定义唤醒词为空
        }
        else {
            std::fstream stream(cus_path, std::ios::in);  // 打开命令文件流
            if (!stream.is_open()) {
                effect_cfg_.custom_wakeup_word = nullptr;  // 如果文件未打开，自定义唤醒词为空
            }
            else {
                // 获取文件长度
                stream.seekg(0, stream.end);
                int length = stream.tellg();
                stream.seekg(0, stream.beg);
                char* buffer = new char[length + 1];  // 创建缓冲区
                stream.read(buffer, length);  // 读取文件内容
                buffer[length] = '\0';
                effect_cfg_.custom_wakeup_word = buffer;  // 设置自定义唤醒词
                stream.close();  // 关闭文件流
                // delete[] buffer;  // 注意：这里不应删除buffer，因为custom_wakeup_word还在使用该缓冲区
                std::cout << "hrsc sdk wakeup word is:" << std::endl
                    << effect_cfg_.custom_wakeup_word << std::endl;  // 打印自定义唤醒词
            }
        }
    }
    else {
        effect_cfg_.custom_wakeup_word = nullptr;  // VoIP模式下，自定义唤醒词为空
    }

    effect_cfg_.vad_timeout = 5000;  // 设置VAD超时时间为5000ms
    effect_cfg_.ref_ch_index = 6;  // 参考信号通道索引
    effect_cfg_.target_score = 0;  // 设置目标评分为0
    effect_cfg_.support_command_word = 0;  // 不支持命令词
    effect_cfg_.wakeup_prefix = 200;  // 唤醒前缀时间200ms
    effect_cfg_.wakeup_suffix = 200;  // 唤醒后缀时间200ms
    effect_cfg_.is_use_linear_mic_flag = mic_type_;  // 是否使用线性麦克风标志

    effect_cfg_.asr_output_mode = asr_mode_;  // 设置ASR输出模式
    effect_cfg_.asr_output_channel = asr_channel_;  // 设置ASR输出通道数

    // 设置各回调函数
    effect_cfg_.HrscVoipDataCallback = VoipDataCallback;
    effect_cfg_.HrscWakeupDataCallback = WakeupDataCallback;
    effect_cfg_.HrscEventCallback = EventCallback;
    effect_cfg_.HrscCmdCallback = CmdDataCallback;
    effect_cfg_.HrscDoaCallbadk = DoaCallback;
    effect_cfg_.HrscAsrCallback = AsrCallback;

    // 初始化SDK并获取SDK句柄
    sdk_handle_ = HrscInit(&effect_cfg_);
    if (sdk_handle_ == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc init error!!!");  // 打印错误信息
        return -1;  // 返回错误码
    }
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc init success!");  // 打印成功信息
    if (HRSC_CODE_SUCCESS != HrscStart(sdk_handle_)) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_capture"), "hrsc start error!!!");  // 打印错误信息
        return -1;  // 返回错误码
    }

    int value = 0;
    if (voip_mode_) {
        RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
            "audioengine sdk init on voip mode");  // 打印VoIP模式初始化信息
        value = 1;
    }
    HrscParamData data;
    data.value = &value;
    data.param_type = kHrscParasTypeVoipDataSwitch;  // 设置VoIP数据开关参数类型
    HrscSetParam(sdk_handle_, &data);  // 设置参数
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "hrsc start success!");  // 打印成功信息
    return 0;  // 返回0，表示成功
}

// 输入音频数据
int AudioEngine::InputData(char* data, int len, bool end) {
    if (!init_ || !start_) {
        RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
            "engine not init or start.");  // 如果未初始化或未启动，则打印错误信息
        return -1;  // 返回错误码
    }
    RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
        "audioengine send audio data to sdk");  // 打印调试信息
    if (end) {
        RCLCPP_DEBUG(rclcpp::get_logger("audio_capture"),
            "audioengine recv the last audio frame");  // 如果是最后一帧，打印调试信息
    }
    int size = len / mic_chn_num_ * sdkin_chn_num_;  // 计算音频数据大小
    if (audio_size_ != size) {
        if (adapter_buffer_) delete[] adapter_buffer_;  // 释放之前的缓冲区
        audio_size_ = size;
        adapter_buffer_ = new char[size];  // 创建新的缓冲区
        memset(adapter_buffer_, 0, size);  // 清空缓冲区
    }

    // 根据麦克风通道数和SDK输入通道数进行数据转换
    if (mic_chn_num_ == sdkin_chn_num_) {
        memcpy(adapter_buffer_, data, size);  // 如果通道数相同，直接复制数据
    }
    else if (mic_chn_num_ < sdkin_chn_num_) {
        // 2通道数据扩展为3通道数据
        char* dst_ptr = adapter_buffer_;
        char* src_ptr = data;
        int frame_count = len / (mic_chn_num_ * 2);  // 计算帧数
        int index = 0;
        while (index++ < frame_count) {
            memcpy(dst_ptr, src_ptr, mic_chn_num_ * 2);  // 复制数据
            dst_ptr += sdkin_chn_num_ * 2;
            src_ptr += mic_chn_num_ * 2;
        }
    }
    else {
        // 8通道数据->6通道数据，剔除5、6通道
        char* dst_ptr = adapter_buffer_;
        char* src_ptr = data;
        int frame_count = len / (mic_chn_num_ * 2);  // 计算帧数
        int index = 0;
        while (index++ < frame_count) {
            memcpy(dst_ptr, src_ptr, 4 * 2);  // 复制前4个通道的数据
            dst_ptr += 4 * 2;
            src_ptr += 6 * 2;
            memcpy(dst_ptr, src_ptr, 2 * 2);  // 复制最后2个通道的数据
            dst_ptr += 2 * 2;
            src_ptr += 2 * 2;
        }
    }

    // 创建音频缓冲区
    HrscAudioBuffer hrsc_buffer;
    hrsc_buffer.audio_data = adapter_buffer_;
    hrsc_buffer.size = audio_size_;
    HrscProcess(sdk_handle_, &hrsc_buffer);  // 处理音频数据
    if (save_file_ && audio_inconvert_file_.is_open()) {
        audio_inconvert_file_.write(adapter_buffer_, audio_size_);  // 如果需要保存文件，则写入音频数据
    }
    return 0;  // 返回0，表示成功
}

// 反初始化音频SDK
void AudioEngine::DeInitSDK() {
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "before stop audio sdk!");  // 打印停止SDK前的警告信息
    HrscStop(sdk_handle_);  // 停止SDK
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"), "stop audio sdk success!");  // 打印停止成功信息
    HrscRelease(&sdk_handle_);  // 释放SDK句柄
    RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
        "destory audio sdk success!");  // 打印销毁成功信息
}

}  // namespace audio
}  // namespace hobot
