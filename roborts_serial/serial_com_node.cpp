/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "serial_com_node.h"

#define LOG_WARNING std::cout << "WARNING: "
#define LOG_FATAL std::cout << "FATAL: "

namespace roborts_serial {
SerialComNode::SerialComNode(std::string module_name)
    : fd_(0), is_open_(false), stop_receive_(true), stop_send_(true) {
  SerialPortConfig serial_port_config;

  std::string file_name = ros::package::getPath("roborts_serial") + "/config/serial_com_config.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &serial_port_config);
  if (!read_state) {
    ROS_ERROR("Cannot open %s", file_name.c_str());
  }

  // CHECK(serial_port_config.has_serial_port()) << "Port not set.";
  // CHECK(serial_port_config.has_serial_boudrate()) << "Baudrate not set.";
  length_beam_ = serial_port_config.link_beam();
  length_column_ = serial_port_config.link_column();
  baudrate_ = serial_port_config.serial_boudrate();
  port_ = serial_port_config.serial_port();
  // uwb_position_.header.frame_id = "uwb";
  // uwb_position_.header.seq = 0;
  if (!Initialization())
    ROS_ERROR("Initialization error.");
  is_open_ = true;
  stop_receive_ = false;
  stop_send_ = false;
  is_sim_ = serial_port_config.is_simulator();
  is_debug_ = serial_port_config.is_debug();
  pack_length_ = 0;
  total_length_ = 0;
  free_length_ = UART_BUFF_SIZE;
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 30);
  // tracking_ctrl_pub_ = nh_.advertise<messages::TrackingCtrl>("tracking_ctrl", 30);
  gim_pub_ = nh_.advertise<roborts_msgs::GimbalAngle>("gimbal", 30);

  if (is_debug_) {
    fp_ = fopen("debug_com.txt", "w+");
  }
}

bool SerialComNode::Initialization() {
  return SerialInitialization(port_, baudrate_, 0, 8, 1, 'E');
}

bool SerialComNode::SerialInitialization(std::string port,
                                         int baudrate,
                                         int flow_control,
                                         int data_bits,
                                         int stop_bits,
                                         int parity) {
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) 
    ROS_ERROR("Serial port open failed!");
  if (tcgetattr(fd_, &termios_options_) != 0)
    ROS_ERROR("1st Time Get serial attributes error!");
  termios_options_original_ = termios_options_;
  ConfigBaudrate(baudrate);
  termios_options_.c_cflag |= CLOCAL;
  termios_options_.c_cflag |= CREAD;
  termios_options_.c_cflag &= ~CSIZE;
  switch (flow_control) {
    case 0 :termios_options_.c_cflag &= ~CRTSCTS;
      break;
    case 1 :termios_options_.c_cflag |= CRTSCTS;
      break;
    case 2 :termios_options_.c_cflag |= IXON | IXOFF | IXANY;
      break;
    default: termios_options_.c_cflag &= ~CRTSCTS;
      break;
  }
  switch (parity) {
    case 'n':
    case 'N':termios_options_.c_cflag &= ~(PARENB | PARODD);
      termios_options_.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':termios_options_.c_cflag |= (PARODD | PARENB);
      termios_options_.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':termios_options_.c_cflag |= PARENB;
      termios_options_.c_cflag &= ~PARODD;
      termios_options_.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_cflag &= ~CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported parity";
      return false;
  }
  switch (stop_bits) {
    case 1: termios_options_.c_cflag &= ~CSTOPB;
      break;
    case 2: termios_options_.c_cflag |= CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported stop bits";
      return false;
  }
  switch (data_bits) {
    case 5 :termios_options_.c_cflag |= CS5;
      break;
    case 6 :termios_options_.c_cflag |= CS6;
      break;
    case 7 :termios_options_.c_cflag |= CS7;
      break;
    case 8 :termios_options_.c_cflag |= CS8;
      break;
    default: LOG_FATAL << "Unsupported data size";
      return false;
  }
  termios_options_.c_iflag = IGNBRK;
  termios_options_.c_iflag &= ~(IXON | IXOFF | IXANY);
  termios_options_.c_lflag = 0;
  termios_options_.c_oflag = 0;
  termios_options_.c_cc[VTIME] = 1;
  termios_options_.c_cc[VMIN] = 60;
  if (tcsetattr(fd_, TCSANOW, &termios_options_) != 0)
    ROS_ERROR("Set serial attributes error!");
  ROS_INFO("Com config success");
  int mcs = 0;
  ioctl(fd_, TIOCMGET, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(fd_, TIOCMSET, &mcs);
  return true;
}

bool SerialComNode::ConfigBaudrate(int baudrate) {
  int i;
  int speed_arr[] = {B921600, B576000, B460800, B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {921600, 576000, 460800, 230400, 115200, 19200, 9600, 4800, 2400, 1200, 300};
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (baudrate == name_arr[i]) {
      cfsetispeed(&termios_options_, speed_arr[i]); 
      cfsetospeed(&termios_options_, speed_arr[i]);
      return true;
    }
  }
  ROS_ERROR("Baudrate set error.");
  return false;
}

void SerialComNode::Run() {
  receive_loop_thread_ = new std::thread(boost::bind(&SerialComNode::ReceiveLoop, this));
  if (is_debug_) {
    keyboard_in_ = new std::thread(boost::bind(&SerialComNode::ListenClick, this));
  }
  // sub_cmd_sho_ = nh_.subscribe("shoot_ctrl", 1, &SerialComNode::ShootControlCallback, this);
  sub_cmd_gim_ = nh_.subscribe("cmd_gimbal_angle", 1, &SerialComNode::DetectionCallback, this);
  // sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &SerialComNode::ChassisControlCallback, this);
  send_loop_thread_ = new std::thread(boost::bind(&SerialComNode::SendPack, this));
  ros::spin();
}

void SerialComNode::ListenClick() {
  static struct termios oldt, newt;
  while (is_sim_ && ros::ok()) {
    usleep(1000);
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    key_ = getchar();
    if (key_ != 10) {
      valid_key_ = key_;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
}

void SerialComNode::ReceiveLoop() {
  index_ = 0;
  while (is_open_ && !stop_receive_ && ros::ok()) {
    read_buff_index_ = 0;
    read_len_ = ReceiveData(fd_, UART_BUFF_SIZE);
    if (read_len_ > 0) {
      while (read_len_--) {
        byte_ = rx_buf_[read_buff_index_++];
        switch (unpack_step_e_) {
          case STEP_HEADER_SOF: {
            if (byte_ == DN_REG_ID || byte_ == UP_REG_ID) {
              protocol_packet_[index_++] = byte_;
              unpack_step_e_ = STEP_LENGTH_LOW;
            } else {
              index_ = 0;
            }
          }
            break;
          case STEP_LENGTH_LOW: {
            data_length_ = byte_;
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_LENGTH_HIGH;
          }
            break;
          case STEP_LENGTH_HIGH: {
            data_length_ |= (byte_ << 8);
            protocol_packet_[index_++] = byte_;
            if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN)) {
              unpack_step_e_ = STEP_FRAME_SEQ;
            } else {
              LOG_WARNING << "Data length too big";
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
            }
          }
            break;
          case STEP_FRAME_SEQ: {
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_HEADER_CRC8;
          }
            break;
          case STEP_HEADER_CRC8: {
            protocol_packet_[index_++] = byte_;
            bool crc8_result = VerifyCrcOctCheckSum(protocol_packet_, HEADER_LEN);
            if (!crc8_result) {
              LOG_WARNING << "CRC 8 error";
            }
            if ((index_ == HEADER_LEN) && crc8_result) {
              if (index_ < HEADER_LEN) {
                LOG_WARNING << "CRC 8 index less.";
              }
              unpack_step_e_ = STEP_DATA_CRC16;
            } else {
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
            }
          }
            break;
          case STEP_DATA_CRC16: {
            if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              protocol_packet_[index_++] = byte_;
            } else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              LOG_WARNING << "Index Beyond";
            }
            if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
              if (VerifyCrcHexCheckSum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                DataHandle();
              } else {
                LOG_WARNING << "CRC16 error";
              }
            }
          }
            break;
          default: {
            LOG_WARNING << "Unpack not well";
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
            break;
        }
      }
    }
  }
}

int SerialComNode::ReceiveData(int fd, int data_length) {
  int received_length, selected;
  fd_set fs_read;
  struct timeval time;
  static int is_init = 0;
  if (is_init == 0) {
    is_init++;
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);
  }
  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);
  time.tv_sec = 0;
  time.tv_usec = 0;
  selected = select(fd + 1, &fs_read, NULL, NULL, &time);
  if (selected > 0) {
    received_length = read(fd, rx_buf_, data_length);
    int count = received_length;
    int p = 0;
    if (is_debug_) {
      fprintf(fp_, "%d,", received_length);
      while (count--) {
        fprintf(fp_, "%x,", rx_buf_[p++]);
      }
      fprintf(fp_, "\n");
//    fwrite(rx_buf_, sizeof(uint8_t), received_length, fp_);
      fflush(fp_);
    }
  } else if (selected == 0) {
    received_length = 0;
  } else {
    received_length = 0;
    ROS_ERROR("Select function error");
  }
  return received_length;
}

void SerialComNode::DataHandle() {
  ros::Time current_time = ros::Time::now();
  geometry_msgs::Quaternion q;
  std::lock_guard<std::mutex> guard(mutex_receive_);
  auto *p_header = (FrameHeader *) protocol_packet_;
  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
  uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;

  if (is_debug_)
    std::cout << "Received frame ID = " << cmd_id << std::endl;

  switch (cmd_id) {
    
    default:
      break;
  }
}


void SerialComNode::RemoteCtrlHandle() {
  // Handle tracking ctrl
#if 0
  {
    messages::TrackingCtrl ctrl;
    if (rc_info_data_.Keyboard.Bit.Z)
      ctrl.action = messages::TrackingCtrl::RESTART;
    else if (rc_info_data_.Keyboard.Bit.C)
      ctrl.action = messages::TrackingCtrl::START;
    else if (rc_info_data_.Keyboard.Bit.X)
      ctrl.action = messages::TrackingCtrl::PAUSE;
    else
      ctrl.action = 0;
    ctrl.offset_x = 0; //rc_info_data_.Mouse.x;
    ctrl.offset_y = 0; //rc_info_data_.Mouse.y;
    tracking_ctrl_pub_.publish(ctrl);
  }
#endif

  // Handle rune detection ctrl
#if 0
  {
    messages::RuneDetectionGoal goal;
    if (rc_info_data_.Keyboard.Bit.V) {
      goal.command = 1;
      goal.detector_id = 0;
      rune_ac_.sendGoal(goal);
    } else if (rc_info_data_.Keyboard.Bit.B) {
      goal.command = 1;
      goal.detector_id = 1;
      rune_ac_.sendGoal(goal);
    } else if (rc_info_data_.Keyboard.Bit.G) {
      goal.command = 3;
      rune_ac_.sendGoal(goal);
    }
  }


#endif

} 


void SerialComNode::DetectionCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
  DetectionResult detection_result;
  uint8_t pack[PACK_MAX_SIZE];
  detection_result.x = msg->yaw_angle;
  detection_result.y = msg->pitch_angle;
  detection_result.dist = 0;
  detection_result.valid = 1;

  if (is_debug_) {
    std::cout << "Sending Detection Result CtrlMsg { x: " << detection_result.x << " y: " << detection_result.y << " }" << std::endl;
  }

  int length = sizeof(DetectionResult), pack_length = length + HEADER_LEN + CMD_LEN + CRC_LEN + 1;
  SendDataHandle(DETECTION_RESULT_ID, (uint8_t *)&detection_result, pack, length);
  if (is_debug_) {
    printf("\n %d[ ", pack_length); for (int i = 0; i < pack_length; i++) printf("%x ", pack[i]); printf("] \n");
  }
  if (pack_length_ <= free_length_)
  {
    memcpy(tx_buf_ + total_length_, pack, pack_length);
    free_length_ -= pack_length;
    total_length_ += pack_length;
  }
  else
  {
    LOG_WARNING << "Overflow in Sending Detection Result";
  }
  
}

void SerialComNode::GimbalControlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg) {
  static int count = 0, time_ms = 0, compress = 0;
  static double frequency = 0;
  static struct timeval time_last, time_current;
  gettimeofday(&time_current, nullptr);
  if (count == 0) {
    count++;
  } else {
    time_ms = (time_current.tv_sec - time_last.tv_sec) * 1000 + (time_current.tv_usec - time_last.tv_usec) / 1000;
    frequency = 1000.0 / time_ms;
  }
  time_last = time_current;
  std::unique_lock<std::mutex> lock(mutex_pack_);
  uint8_t pack[PACK_MAX_SIZE];
  GimbalControl gimbal_control_data;
  //TODO(krik): choose the mode by command

  // if (msg->chassis_relax) {
  gimbal_control_data.ctrl_mode = GIMBAL_FOLLOW_CHASSIS;
  // } else {
  //   gimbal_control_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
  // }

  // SetChassisStatus(msg->chassis_relax, msg->enemy_yaw);

  if (is_debug_) {
    if (valid_key_ == 'h') {
      gimbal_control_data.ctrl_mode = GIMBAL_POSITION_MODE;
    } else if (valid_key_ == 'i') {
      gimbal_control_data.ctrl_mode = GIMBAL_SHOOT_BUFF;
    } else if (valid_key_ == 'j') {
      gimbal_control_data.ctrl_mode = GIMBAL_PATROL_MODE;
    } else if (valid_key_ == 'k') {
      gimbal_control_data.ctrl_mode = GIMBAL_FOLLOW_CHASSIS;
    } else if (valid_key_ == 'l') {
      gimbal_control_data.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
    } else if (valid_key_ == 'm') {
      gimbal_control_data.ctrl_mode = GIMBAL_INIT;
    } else if (valid_key_ == 'n') {
      gimbal_control_data.ctrl_mode = GIMBAL_RELAX;
    }
  }
  gimbal_control_data.pit_ref = (msg->pitch_angle) * 180 / M_PI;
  gimbal_control_data.yaw_ref = (msg->yaw_angle) * 180 / M_PI;
  gimbal_control_data.visual_valid = 1;

  if (is_debug_) {
    // std::cout << "Current Gimbal Data1 { pitch " << gim_angle_.pitch << " | yaw " << gim_angle_.yaw_angle << " }" << std::endl;
    std::cout << "Sending Gimbal CtrlMsg { ctrl_mode " << (int)gimbal_control_data.ctrl_mode << " | pitch " << gimbal_control_data.pit_ref << " | yaw " << gimbal_control_data.yaw_ref << " }" << std::endl;
    // Pretend to have feedback
    // gim_angle_.pitch = (msg->enemy_pitch + gim_angle_.pitch);
    // gim_angle_.yaw = (msg->enemy_yaw + gim_angle_.yaw);
  }

  int length = sizeof(GimbalControl), total_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
  SendDataHandle(GIMBAL_CTRL_ID, (uint8_t *) &gimbal_control_data, pack, length);
  if (is_debug_) {
    printf("\n %d[ ", total_length); for (int i = 0; i < total_length; i++) printf("%x ", pack[i]); printf("] \n");
  }
  if (total_length <= free_length_) {
    memcpy(tx_buf_ + total_length_, pack, total_length);
    free_length_ -= total_length;
    total_length_ += total_length;
  } else {  
    ROS_ERROR("Overflow in Gimbal CB");
  }
}

void SerialComNode::SendDataHandle(uint16_t cmd_id,
                                   uint8_t *topack_data,
                                   uint8_t *packed_data,
                                   uint16_t len
) {
  FrameHeader *p_header = (FrameHeader *) packed_data;
  p_header->sof = UP_REG_ID;
  p_header->data_length = len;
  memcpy(packed_data + HEADER_LEN, (uint8_t *) &cmd_id, CMD_LEN);
  AppendCrcOctCheckSum(packed_data, HEADER_LEN);
  memcpy(packed_data + HEADER_LEN + CMD_LEN, topack_data, len);
  AppendCrcHexCheckSum(packed_data, HEADER_LEN + CMD_LEN + CRC_LEN + len);
}

void SerialComNode::SendPack() {
  while (is_open_ && !stop_send_ && ros::ok()) {
    if (total_length_ > 0) {
      mutex_send_.lock();
      SendData(total_length_);
      total_length_ = 0;
      free_length_ = UART_BUFF_SIZE;
      mutex_send_.unlock();
    } else {
      usleep(100);
    }
  }
}

int SerialComNode::SendData(int data_len) {
  int length = 0;
  length = write(fd_, tx_buf_, data_len);
  lseek(fd_, 0, SEEK_CUR);
  std::cout << "Com sending: " << length << std::endl;
  if (length == data_len) {
    return length;
  } else {
    ROS_ERROR("Serial write error");
    return -1;
  }
}

SerialComNode::~SerialComNode() {
  if (receive_loop_thread_ != nullptr) {
    stop_receive_ = true;
    receive_loop_thread_->join();
    delete receive_loop_thread_;
  }
  if (send_loop_thread_ != nullptr) {
    stop_send_ = true;
    send_loop_thread_->join();
    delete send_loop_thread_;
  }
  tcsetattr(fd_, TCSANOW, &termios_options_original_);
  close(fd_);
  is_open_ = false;
  if (is_debug_) {
    fclose(fp_);
  }
}

void SerialComNode::Stop() {
  stop_send_ = true;
  stop_receive_ = true;
}

void SerialComNode::Resume() {
  stop_send_ = false;
  stop_receive_ = false;
}

uint8_t SerialComNode::GetCrcOctCheckSum(uint8_t *message, uint32_t length, uint8_t crc) {
  uint8_t index;
  while (length--) {
    index = crc ^ (*message++);
    crc = kCrcOctTable[index];
  }
  return (crc);
}

bool SerialComNode::VerifyCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t expected = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Verify CRC8 false";
    return false;
  }
  expected = GetCrcOctCheckSum(message, length - 1, kCrc8);
  return (expected == message[length - 1]);
}

void SerialComNode::AppendCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t crc = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Append CRC8 NULL";
    return;
  };
  crc = GetCrcOctCheckSum(message, length - 1, kCrc8);
  message[length - 1] = crc;
}

uint16_t SerialComNode::GetCrcHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc) {
  uint8_t data;
  if (message == NULL) {
    return 0xFFFF;
  }
  while (length--) {
    data = *message++;
    (crc) = ((uint16_t) (crc) >> 8) ^ kCrcTable[((uint16_t) (crc) ^ (uint16_t) (data)) & 0x00ff];
  }
  return crc;
}

bool SerialComNode::VerifyCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t expected = 0;
  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Verify CRC16 bad";
    return false;
  }
  expected = GetCrcHexCheckSum(message, length - 2, kCrc);
  return ((expected & 0xff) == message[length - 2] && ((expected >> 8) & 0xff) == message[length - 1]);
}

void SerialComNode::AppendCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t crc = 0;
  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Append CRC 16 NULL";
    return;
  }
  crc = GetCrcHexCheckSum(message, length - 2, kCrc);
  message[length - 2] = (uint8_t) (crc & 0x00ff);
  message[length - 1] = (uint8_t) ((crc >> 8) & 0x00ff);
}

}

int main(int argc, char **argv) {
  // signal(SIGINT, SignalHandler);
  // signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "serial_com_node", ros::init_options::NoSigintHandler);
  roborts_serial::SerialComNode serial_com_node("serial_com_node");
  ros::AsyncSpinner async_spinner(1);
  serial_com_node.Run();
  async_spinner.start();
  ros::waitForShutdown();
  serial_com_node.Stop();
  return 0;
}


