
#ifndef CTI_CHASSIS_COMMUNICATE_INTERFACE_H
#define CTI_CHASSIS_COMMUNICATE_INTERFACE_H
#include <queue>
#include <deque>
#include <string>
#include <mutex>
#include <shared_mutex>
#include "serial_cmd.hpp"

// class serial_frame_type;
// class serial_frame_cnt_type;

class ChassisCmncBase {
 public:
  virtual ~ChassisCmncBase(){};

  virtual void getDatas(unsigned char &cmd, unsigned int &length,
                        unsigned char *data, bool &update_flag) = 0;
  virtual void sendDatas() = 0;

  virtual int queueSendDatas(unsigned char cmd, unsigned int length, void *data,
                             const bool &set_front) = 0;

  //升级的特殊接口，后期建议移除
  virtual void send_serial_update_flow(const unsigned char *src,
                                       unsigned int cnt) = 0;

  const bool setDevice(const std::string &device_type_,
                       const std::string &device_name_,
                       const int &device_port_ = 0,
                       const std::string &dest_name_ = "null",
                       const int &dest_port_ = 0) {
    this->device_name = device_name_;
    this->device_type = device_type_;
    this->device_port = device_port_;
    this->dest_name = dest_name_;
    this->dest_port = dest_port_;
    return true;
  }

  const std::string getDeviceType() { return this->device_type; }

  const std::string getDeviceName() { return this->device_name; }

  const int getDevicePort() { return this->device_port; }

  const std::string getDestName() { return this->dest_name; }

  const int getDestPort() { return this->dest_port; }

  const int getQueueSize() { return this->recv_frame_queue_.size(); }

 protected:
  std::queue<serial_frame_type> recv_frame_queue_;
  std::deque<serial_frame_cnt_type> send_frame_deque_;

  std::mutex mutex_queue_;

 private:
  std::string device_name;
  std::string device_type;
  std::string dest_name;
  int device_port;
  int dest_port;
};

#endif