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

#include "audio_capture/hb_audio_capture.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "audio_engine/audioengine.h"

namespace hobot {
namespace audio {
    // 构造函数：初始化音频捕获节点，声明并获取参数
HBAudioCapture::HBAudioCapture(const std::string &node_name,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  this->declare_parameter<std::string>("config_path", config_path_);
  this->declare_parameter<std::string>("audio_pub_topic_name",
                                       audio_pub_topic_name_);
  this->declare_parameter<std::string>("asr_pub_topic_name",
                                       asr_pub_topic_name_);

  this->get_parameter<std::string>("config_path", config_path_);
  this->get_parameter<std::string>("audio_pub_topic_name",
                                   audio_pub_topic_name_);
  this->get_parameter<std::string>("asr_pub_topic_name",
                                   asr_pub_topic_name_);

  std::string tros_distro
      = std::string(std::getenv("TROS_DISTRO")? std::getenv("TROS_DISTRO") : "");
  audio_sdk_path_ = "/opt/tros/" + tros_distro + "/lib/hobot_audio";
  
  std::stringstream ss;
  ss << "Parameter:"
     << "\n config_path: " << config_path_
     << "\n audio_pub_topic_name: " << audio_pub_topic_name_
     << "\n asr_pub_topic_name: " << asr_pub_topic_name_
     << "\n audio_sdk_path: " << audio_sdk_path_;
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "%s", ss.str().c_str());
}
// 析构函数：反初始化音频捕获
HBAudioCapture::~HBAudioCapture() { DeInit(); }
// 初始化音频捕获设备和音频引擎
int HBAudioCapture::Init() {
  std::string file = config_path_ + "/audio_config.json";
  ParseConfig(file);
  if (micphone_enable_ != 1) {
    RCLCPP_WARN(rclcpp::get_logger("hobot_audio"),
                "mic disable, do not capture audio!!!");
    return 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("hobot_audio"), "init to capture audio");
  micphone_device_ = alsa_device_allocate();
  if (!micphone_device_) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_audio"), "open mic device fail");
    return -1;
  }

  /* init micphone device*/
  micphone_device_->name = const_cast<char *>(micphone_name_.c_str());
  micphone_device_->format = SND_PCM_FORMAT_S16;
  micphone_device_->direct = SND_PCM_STREAM_CAPTURE;
  micphone_device_->rate = micphone_rate_;
  micphone_device_->channels = micphone_chn_;
  micphone_device_->buffer_time = micphone_buffer_time_;
  micphone_device_->nperiods = micphone_nperiods_;
  micphone_device_->period_size = micphone_period_size_;
  int ret = alsa_device_init(micphone_device_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_audio"),
                 "alsa device init fail, ret=%d", ret);
    return -1;
  }

  RCLCPP_WARN_STREAM(rclcpp::get_logger("hobot_audio"),
    "audio_sdk_path_ is [" << audio_sdk_path_ << "]");

  AudioEngine::Instance()->Init(
      std::bind(&HBAudioCapture::AudioDataFunc, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&HBAudioCapture::AudioSmartDataFunc, this,
                std::placeholders::_1),
      std::bind(&HBAudioCapture::AudioCmdDataFunc, this, std::placeholders::_1),
      std::bind(&HBAudioCapture::AudioEventFunc, this, std::placeholders::_1),
      std::bind(&HBAudioCapture::AudioASRDataFunc, this, std::placeholders::_1),
      micphone_chn_, audio_sdk_path_, voip_mode_, mic_type_,
      asr_output_mode_, asr_output_channel_);

  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "init success");
  // system("rm ./*.pcm -rf");
  if (save_audio_) {
    audio_infile_.open("./audio_in.pcm",
                       std::ios::app | std::ios::out | std::ios::binary);
    audio_sdk_.open("./audio_voip.pcm",
                    std::ios::app | std::ios::out | std::ios::binary);
  }

  msg_publisher_ = this->create_publisher<audio_msg::msg::SmartAudioData>(
      audio_pub_topic_name_, 10);
  
  if (asr_output_mode_ == 1 || asr_output_mode_ == 2) {
    asr_msg_publisher_ = this->create_publisher<std_msgs::msg::String>(asr_pub_topic_name_, 10);
  }
  is_init_ = true;
  return 0;
}
// 反初始化音频捕获设备和音频引擎
int HBAudioCapture::DeInit() {
  RCLCPP_INFO(rclcpp::get_logger("hobot_audio"), "deinit");
  if (!is_init_) return 0;
  if (!micphone_device_) return -1;
  if (micphone_device_) {
    alsa_device_deinit(micphone_device_);
    alsa_device_free(micphone_device_);
    micphone_device_ = nullptr;
  }
  AudioEngine::Instance()->Stop();
  AudioEngine::Instance()->DeInit();
  if (audio_infile_.is_open()) {
    audio_infile_.close();
  }
  if (audio_sdk_.is_open()) {
    audio_sdk_.close();
  }
  return 0;
}
// 运行音频捕获
int HBAudioCapture::Run() {
  if (!is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_audio"), "HBAudioCapture not init.");
    return -1;
  }

  AudioEngine::Instance()->Start();
  rclcpp::executors::SingleThreadedExecutor exec;
  auto capture_task = std::make_shared<std::thread>(
      std::bind(&HBAudioCapture::MicphoneGetThread, this));
  exec.spin();
  if (capture_task && capture_task->joinable()) {
    capture_task.reset();
  }
  exec.spin();
  return 0;
}
// 麦克风获取音频数据的线程
int HBAudioCapture::MicphoneGetThread() {
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "start to capture audio");
  if (!micphone_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_audio"), "micphone device is null");
    return -1;
  }

  int ret = -1;
  snd_pcm_sframes_t frames;
  frames = micphone_device_->period_size;
  int size = snd_pcm_frames_to_bytes(micphone_device_->handle, frames);
  char *buffer = new char[size];
  while (rclcpp::ok()) {
    // auto start_time = std::chrono::high_resolution_clock::now();
    ret = alsa_device_read(micphone_device_, buffer, frames);
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto cost_time = std::chrono::duration_cast<std::chrono::microseconds>(
    //     end_time - start_time).count();
    if (ret <= 0) continue;
    RCLCPP_DEBUG(rclcpp::get_logger("hobot_audio"), "capture audio size:%d",
                 size);
    audio_num_++;
    // time_stamp_ =
    //     std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now().time_since_epoch())
    //         .count();
    AudioEngine::Instance()->InputData(buffer, size, false);
    if (save_audio_ && audio_infile_.is_open()) {
      audio_infile_.write(buffer, size);
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "stop capture audio");
  delete[] buffer;
  return 0;
}
// 音频数据处理函数：发布音频数据消息
void HBAudioCapture::AudioDataFunc(char *buffer, int size) {
  RCLCPP_DEBUG(rclcpp::get_logger("hobot_audio"), "pub audio data, size:%d", size);
  audio_msg::msg::SmartAudioData::UniquePtr frame(
      new audio_msg::msg::SmartAudioData());
  frame->frame_type.value = frame->frame_type.SMART_AUDIO_TYPE_VOIP;
  frame->data.resize(size);
  memcpy(&frame->data[0], buffer, size);
  if (save_audio_ && audio_sdk_.is_open()) {
   audio_sdk_.write(buffer,size);
  }
  msg_publisher_->publish(std::move(frame));
}
// 音频智能数据处理函数（占位函数）
void HBAudioCapture::AudioSmartDataFunc(float theta) {
  audio_msg::msg::SmartAudioData::UniquePtr frame(new audio_msg::msg::SmartAudioData());
  frame->frame_type.value = frame->frame_type.SMART_AUDIO_TYPE_DOA;
  frame->doa_theta = theta;
  msg_publisher_->publish(std::move(frame));
}
// 音频命令数据处理函数（占位函数）
void HBAudioCapture::AudioCmdDataFunc(const char *cmd_word) {
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "recv cmd word:%s", cmd_word);
  audio_msg::msg::SmartAudioData::UniquePtr frame(new audio_msg::msg::SmartAudioData());
  frame->frame_type.value = frame->frame_type.SMART_AUDIO_TYPE_CMD_WORD;
  frame->cmd_word = cmd_word;
  msg_publisher_->publish(std::move(frame));
}
// 音频事件处理函数（占位函数）
void HBAudioCapture::AudioEventFunc(int event) {
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "recv event:%d", event);
  audio_msg::msg::SmartAudioData::UniquePtr frame(new audio_msg::msg::SmartAudioData());
  frame->frame_type.value = frame->frame_type.SMART_AUDIO_TYPE_EVENT;
  switch (event) {
    case kHrscEventWkpNormal:
      frame->event_type.value = frame->event_type.EVENT_WKPNORMAL;
      break;
    case kHrscEventWkpOneshot:
      frame->event_type.value = frame->event_type.EVENT_WKPONESHOT;
      break;
    case kHrscEventWaitAsrTimeout:
      frame->event_type.value = frame->event_type.EVENT_WAITASRTIMEOUT;
      break;
    case kHrscEventVadBegin:
      frame->event_type.value = frame->event_type.EVENT_VADBEGIN;
      break;
    case kHrscEventVadMid:
      frame->event_type.value = frame->event_type.EVENT_VADMID;
      break;
    case kHrscEventVadEnd:
      frame->event_type.value = frame->event_type.EVENT_VADEND;
      break;
    default:
      break;
  }
  msg_publisher_->publish(std::move(frame));
}
// 语音识别数据处理函数：发布ASR（自动语音识别）数据消息
void HBAudioCapture::AudioASRDataFunc(const char *asr) {
  RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "asr result:%s", asr);
  if (asr_output_mode_ == 1 || asr_output_mode_ == 2) {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = asr;
    asr_msg_publisher_->publish(std::move(message));
  }
}
// 配置解析函数：从配置文件中读取配置参数
int HBAudioCapture::ParseConfig(std::string config_file) {
  if (config_file.empty()) return -1;
  RCLCPP_INFO(rclcpp::get_logger("hobot_audio"), "hobot audio config file:%s",
              config_file.c_str());
  std::ifstream ifs(config_file);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_audio"), "open config file:%s fail",
                 config_file.c_str());
    return -1;
  }

  auto parse_line = [](std::string value, int &result) {
    size_t nEndPos = value.find(":");
    if (nEndPos == std::string::npos) return;
    value = value.substr(nEndPos + 1);
    result = atoi(value.c_str());
  };

  auto parse_line_string = [](const std::string &json, std::string &result) {
    size_t colonPos = json.find(":");
    if (colonPos == std::string::npos)
      return;

    size_t valueStart = json.find_first_not_of(" \t\n\r", colonPos + 1);
    if (valueStart == std::string::npos)
      return;

    if (json[valueStart] == '\"') {
      size_t valueContentStart = valueStart + 1;
      size_t valueContentEnd = json.find_first_of("\"", valueContentStart);
      if (valueContentEnd == std::string::npos)
        return;

      result =
          json.substr(valueContentStart, valueContentEnd - valueContentStart);
    } else {
      size_t valueEnd = json.find_first_of(",}\n\r", valueStart);
      if (valueEnd == std::string::npos)
        return;

      result = json.substr(valueStart, valueEnd - valueStart);
    }
  };

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.find("\"micphone_enable\"") != std::string::npos) {
      parse_line(line, micphone_enable_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_enable: %d",
                  micphone_enable_);
    }
    if (line.find("\"micphone_name\"") != std::string::npos) {
      parse_line_string(line, micphone_name_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_name: %s",
                  micphone_name_.c_str());
    }
    if (line.find("\"micphone_rate\"") != std::string::npos) {
      parse_line(line, micphone_rate_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_rate: %d",
                  micphone_rate_);
    }
    if (line.find("\"micphone_buffer_time\"") != std::string::npos) {
      parse_line(line, micphone_buffer_time_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_buffer_time: %d",
                  micphone_buffer_time_);
    }
    if (line.find("\"micphone_chn\"") != std::string::npos) {
      parse_line(line, micphone_chn_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_chn: %d",
                  micphone_chn_);
    }
    if (line.find("\"micphone_nperiods\"") != std::string::npos) {
      parse_line(line, micphone_nperiods_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_nperiods: %d",
                  micphone_nperiods_);
    }
    if (line.find("\"micphone_period_size\"") != std::string::npos) {
      parse_line(line, micphone_period_size_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "micphone_period_size: %d",
                  micphone_period_size_);
    }
    if (line.find("\"voip_mode\"") != std::string::npos) {
      parse_line(line, voip_mode_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "voip_mode: %d",
                  voip_mode_);
    }
    if (line.find("\"mic_type\"") != std::string::npos) {
      parse_line(line, mic_type_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "mic_type: %d",
                  mic_type_);
    }
    if (line.find("\"asr_mode\"") != std::string::npos) {
      parse_line(line, asr_output_mode_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "asr_mode: %d",
                  asr_output_mode_);
    }
    if (line.find("\"asr_channel\"") != std::string::npos) {
      parse_line(line, asr_output_channel_);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "asr_channel: %d",
                  asr_output_channel_);
    }
    if (line.find("\"save_audio\"") != std::string::npos) {
      int save = 0;
      parse_line(line, save);
      RCLCPP_WARN(rclcpp::get_logger("hobot_audio"), "save_audio: %d",
                  save);
      save_audio_ = save;
    }
  }
  ifs.close();
  return 0;
}

}  // namespace audio
}  // namespace hobot
