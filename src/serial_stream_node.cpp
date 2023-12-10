#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <libserial/SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <map>

using namespace LibSerial;
using namespace std;

// ボーレートを SerialStreamBuf::BaudRateEnum に変換する関数
BaudRate getBaudRateEnum(int baud_rate) {
  static const map<int, BaudRate> baud_rate_map = {
    {300, LibSerial::BaudRate::BAUD_300},
    {600, LibSerial::BaudRate::BAUD_600},
    {1200, LibSerial::BaudRate::BAUD_1200},
    {2400, LibSerial::BaudRate::BAUD_2400},
    {4800, LibSerial::BaudRate::BAUD_4800},
    {9600, LibSerial::BaudRate::BAUD_9600},
    {19200, LibSerial::BaudRate::BAUD_19200},
    {38400, LibSerial::BaudRate::BAUD_38400},
    {57600, LibSerial::BaudRate::BAUD_57600},
    {115200, LibSerial::BaudRate::BAUD_115200}, 
    {230400, LibSerial::BaudRate::BAUD_230400}
  };

  auto it = baud_rate_map.find(baud_rate);
  if (it != baud_rate_map.end()) {
    return it->second;
  } else {
    throw std::runtime_error("Unsupported baud rate");
  }
}

int main(int argc, char **argv) {
  // ROSの初期化
  ros::init(argc, argv, "serial_to_ros");
  ros::NodeHandle nh("~");

  // パブリッシャーの設定
  ros::Publisher pub = nh.advertise<std_msgs::String>("serial_data", 1000);
  ros::Publisher data0_pub = nh.advertise<std_msgs::Float64>("data0", 1000);
  ros::Publisher data1_pub = nh.advertise<std_msgs::Float64>("data1", 1000);
  ros::Rate loop_rate(1000);

  // シリアルポートとボーレートのパラメータ取得
  std::string serial_port;
  int baud_rate;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nh.param<int>("baud_rate", baud_rate, 9600);

  SerialStream serial_stream;
  try
  {
    // Open the Serial Port at the desired hardware port.
    serial_stream.Open(serial_port) ;
  }
  catch (const OpenFailed&)
  {
    std::cerr << "The serial port: " << serial_port << " did not open correctly." << std::endl;
    return EXIT_FAILURE ;
  }

  serial_stream.SetBaudRate(getBaudRateEnum(baud_rate));
  serial_stream.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);

  std::string data0_format, data1_format;
  nh.param<std::string>("data0_format", data0_format, "data0: %lf");
  nh.param<std::string>("data1_format", data1_format, "data1: %lf");
  std::string format = data0_format + ", " + data1_format;

  while (ros::ok()) {
    std::string line;
    std::getline(serial_stream, line);

    // 受信データの解析
    double data0, data1;
    /*
    string format is following
      "data0: 1.23, data1: 2.30"
    */
    if (sscanf(line.c_str(), format.c_str(), &data0, &data1) == 2) {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "data0: " << data0 << ", data1: " << data1;
      msg.data = ss.str();

      std_msgs::Float64 data0_msg, data1_msg;
      data0_msg.data = data0;
      data1_msg.data = data1;

      // ROSトピックのパブリッシュ
      pub.publish(msg);
      data0_pub.publish(data0_msg);
      data1_pub.publish(data1_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
