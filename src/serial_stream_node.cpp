#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <libserial/SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <map>
#include <vector>
#include <regex>
#include <sstream>

using namespace LibSerial;
using namespace std;

/**
 * シリアル通信からROSトピックへのブリッジクラス
 */
class SerialToRosBridge
{
private:
  ros::NodeHandle nh_;

  // 設定パラメータ
  std::string serial_port_;
  int baud_rate_;
  int data_count_;

  // データフォーマットと変数名
  vector<string> data_formats_;
  vector<string> var_names_;

  // パブリッシャー
  ros::Publisher raw_data_pub_;
  vector<ros::Publisher> data_pubs_;
  
  // サブスクライバー（制御コマンド用）
  ros::Subscriber speed_sub_;

public:
  SerialStream serial_stream_;
  /**
   * コンストラクタ
   * @param nh ROSノードハンドル
   */
  SerialToRosBridge(ros::NodeHandle nh) : nh_(nh)
  {
    readParameters();
    setupPublishers();
    setupSubscribers();
    setupSerialPort();
  }
  
  /**
   * デストラクタ
   */
  ~SerialToRosBridge()
  {
    serial_stream_.Close();
  }
  
  /**
   * メインループを実行
   */
  void run()
  {
    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
      processSerialData();

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  /**
   * ROSパラメータを読み込む
   */
  void readParameters()
  {
    // シリアルポートとボーレートのパラメータ取得
    nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    nh_.param<int>("baud_rate", baud_rate_, 9600);

    // データのフォーマット数を取得
    nh_.param<int>("data_count", data_count_, 3); // デフォルトは3

    ROS_INFO("Configuring for %d data fields", data_count_);

    // 各データのフォーマットを取得
    for (int i = 0; i < data_count_; i++)
    {
      string param_name = "data" + to_string(i) + "_format";
      string default_format = "data" + to_string(i) + ": %lf";
      string format;

      nh_.param<string>(param_name, format, default_format);
      data_formats_.push_back(format);

      // 変数名を抽出
      string var_name = extractVariableName(format);
      var_names_.push_back(var_name);

      ROS_INFO("Field %d: format='%s', var_name='%s'",
               i, format.c_str(), var_name.c_str());
    }
  }

  /**
   * パブリッシャーを設定
   */
  void setupPublishers()
  {
    raw_data_pub_ = nh_.advertise<std_msgs::String>("serial_data", 1000);

    for (int i = 0; i < data_count_; i++)
    {
      ros::Publisher data_pub = nh_.advertise<std_msgs::Float64>("data" + to_string(i), 1000);
      data_pubs_.push_back(data_pub);
    }
  }

  /**
   * サブスクライバーを設定
   */
  void setupSubscribers()
  {
    // スピード制御用のサブスクライバーを設定
    speed_sub_ = nh_.subscribe("motor_speed", 10, &SerialToRosBridge::speedCallback, this);
    
    ROS_INFO("Subscribed to motor_speed topic");
  }
  
  /**
   * スピード制御コールバック関数
   * @param msg 速度コマンドメッセージ
   */
  void speedCallback(const std_msgs::Float64::ConstPtr& msg)
  {
    int speed = static_cast<int>(msg->data);
    
    // 値の範囲を0-100に制限
    speed = std::max(0, std::min(100, speed));
    
    // コマンド文字列を作成
    std::stringstream cmd;
    cmd << "speed: " << speed << "\r\n";
    
    ROS_INFO("Sending speed command: %s", cmd.str().c_str());
    
    // シリアルポートにコマンドを送信
    try {
      serial_stream_ << cmd.str();
      serial_stream_.flush();
    } catch (const std::exception &e) {
      ROS_ERROR("Error sending speed command: %s", e.what());
    }
  }

  /**
   * シリアルポートを設定
   */
  void setupSerialPort()
  {
    try
    {
      // シリアルポートを開く
      serial_stream_.Open(serial_port_);

      // シリアルポートのパラメータを設定
      serial_stream_.SetBaudRate(getBaudRateEnum(baud_rate_));
      serial_stream_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

      ROS_INFO("Successfully opened serial port: %s with baud rate: %d",
               serial_port_.c_str(), baud_rate_);
    }
    catch (const OpenFailed &)
    {
      ROS_ERROR("Failed to open serial port: %s", serial_port_.c_str());
      throw std::runtime_error("Serial port did not open correctly");
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Error setting up serial port: %s", e.what());
      throw;
    }
  }

  /**
   * シリアルデータを処理してROSトピックとして発行
   */
  void processSerialData()
  {
    std::string line;

    try
    {
      // シリアルポートから1行読み込み
      std::getline(serial_stream_, line);

      if (line.empty())
      {
        return;
      }

      ROS_DEBUG("Received: %s", line.c_str());

      // データ解析
      map<string, double> data_values = parseSerialData(line);

      // 全ての変数が見つかったかチェック
      if (isDataComplete(data_values))
      {
        publishData(data_values);
      }
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Error processing serial data: %s", e.what());
    }
  }

  /**
   * シリアルデータを解析して変数と値のマップを作成
   * @param line シリアルポートから読み込んだ行
   * @return 変数名と値のマップ
   */
  map<string, double> parseSerialData(const string &line)
  {
    map<string, double> data_values;

    for (const string &var_name : var_names_)
    {
      size_t pos = line.find(var_name + ":");
      if (pos != string::npos)
      {
        // 変数名の後の値を探す
        size_t value_start = pos + var_name.length() + 1; // ":" の次

        // 空白をスキップ
        while (value_start < line.length() && isspace(line[value_start]))
        {
          value_start++;
        }

        // 値の終端を探す（次のカンマかスペースか行末）
        size_t value_end = line.find_first_of(" ,", value_start);
        if (value_end == string::npos)
        {
          value_end = line.length();
        }

        string value_str = line.substr(value_start, value_end - value_start);
        double value = stod(value_str);
        data_values[var_name] = value;

        ROS_DEBUG("Parsed: %s = %f", var_name.c_str(), value);
      }
    }

    return data_values;
  }

  /**
   * すべてのデータが揃っているかチェック
   * @param data_values 変数名と値のマップ
   * @return すべての変数が見つかった場合はtrue
   */
  bool isDataComplete(const map<string, double> &data_values)
  {
    for (const string &var_name : var_names_)
    {
      if (data_values.find(var_name) == data_values.end())
      {
        ROS_DEBUG("Missing variable: %s", var_name.c_str());
        return false;
      }
    }
    return true;
  }

  /**
   * データをROSトピックとして発行
   * @param data_values 変数名と値のマップ
   */
  void publishData(const map<string, double> &data_values)
  {
    // メッセージの作成
    std_msgs::String msg;
    std::stringstream ss;
    vector<double> values;

    for (size_t i = 0; i < var_names_.size(); i++)
    {
      if (i > 0)
        ss << ", ";
      double value = data_values.at(var_names_[i]);
      ss << var_names_[i] << ": " << value;
      values.push_back(value);
    }

    msg.data = ss.str();

    // 全データを含むメッセージをパブリッシュ
    raw_data_pub_.publish(msg);

    // 各値を個別にパブリッシュ
    for (size_t i = 0; i < values.size(); i++)
    {
      std_msgs::Float64 data_msg;
      data_msg.data = values[i];
      data_pubs_[i].publish(data_msg);
    }

    ROS_INFO("Published: %s", msg.data.c_str());
  }

  /**
   * フォーマット文字列から変数名を抽出する関数
   * @param format フォーマット文字列 (例: "name: %lf")
   * @return 変数名 (例: "name")
   */
  string extractVariableName(const string &format)
  {
    // "name: %lf" から "name" を抽出
    size_t colonPos = format.find(':');
    if (colonPos != string::npos)
    {
      return format.substr(0, colonPos);
    }
    return format; // コロンがなければそのまま返す
  }

  /**
   * ボーレートを SerialStreamBuf::BaudRateEnum に変換する関数
   * @param baud_rate ボーレート (例: 9600)
   * @return 対応するLibSerial::BaudRate列挙値
   */
  BaudRate getBaudRateEnum(int baud_rate)
  {
    static const map<int, BaudRate> baud_rate_map = {
        {300, BaudRate::BAUD_300},
        {600, BaudRate::BAUD_600},
        {1200, BaudRate::BAUD_1200},
        {2400, BaudRate::BAUD_2400},
        {4800, BaudRate::BAUD_4800},
        {9600, BaudRate::BAUD_9600},
        {19200, BaudRate::BAUD_19200},
        {38400, BaudRate::BAUD_38400},
        {57600, BaudRate::BAUD_57600},
        {115200, BaudRate::BAUD_115200},
        {230400, BaudRate::BAUD_230400}};

    auto it = baud_rate_map.find(baud_rate);
    if (it != baud_rate_map.end())
    {
      return it->second;
    }
    else
    {
      throw std::runtime_error("Unsupported baud rate: " + to_string(baud_rate));
    }
  }
};

/**
 * メイン関数
 */
int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, "serial_to_ros");
    ros::NodeHandle nh("~");

    SerialToRosBridge bridge(nh);
    bridge.run();

    return EXIT_SUCCESS;
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("Fatal error: %s", e.what());
    return EXIT_FAILURE;
  }
}