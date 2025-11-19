#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialStream.h>
#include <chrono>
#include <memory>
#include <iostream>
#include <unistd.h>
#include <map>
#include <vector>
#include <regex>
#include <sstream>
#include <string>

using namespace LibSerial;
using namespace std::chrono_literals;
using std::map;
using std::string;
using std::vector;

/**
 * シリアル通信からROSトピックへのブリッジクラス
 */
class SerialToRosBridge : public rclcpp::Node
{
private:
  // 設定パラメータ
  string serial_port_;
  int baud_rate_;
  int data_count_;

  // データフォーマットと変数名
  vector<string> data_formats_;
  vector<string> var_names_;

  // パブリッシャー
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_data_pub_;
  vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> data_pubs_;

public:
  SerialStream serial_stream_;
  /**
   * コンストラクタ
   * @param nh ROSノードハンドル
   */
  SerialToRosBridge() : rclcpp::Node("serial_to_ros")
  {
    readParameters();
    setupPublishers();
    setupSerialPort();
  }
  
  /**
   * デストラクタ
   */
  ~SerialToRosBridge()
  {
    if (serial_stream_.IsOpen())
    {
      serial_stream_.Close();
    }
  }
  
  /**
   * メインループを実行
   */
  void run()
  {
    rclcpp::Rate loop_rate(1000.0);

    while (rclcpp::ok())
    {
      processSerialData();

      rclcpp::spin_some(this->shared_from_this());
      loop_rate.sleep();
    }

    serial_stream_.Close();
  }

private:
  /**
   * ROSパラメータを読み込む
   */
  void readParameters()
  {
    // シリアルポートとボーレートのパラメータ取得
    serial_port_ = this->declare_parameter<string>("serial_port", "/dev/ttyUSB0");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 9600);

    // データのフォーマット数を取得
    data_count_ = this->declare_parameter<int>("data_count", 3); // デフォルトは3
    data_formats_.clear();
    var_names_.clear();

    RCLCPP_INFO(this->get_logger(), "Configuring for %d data fields", data_count_);

    // 各データのフォーマットを取得
    for (int i = 0; i < data_count_; i++)
    {
      string param_name = "data" + std::to_string(i) + "_format";
      string default_format = "data" + std::to_string(i) + ": %lf";
      string format = this->declare_parameter<string>(param_name, default_format);
      data_formats_.push_back(format);

      // 変数名を抽出
      string var_name = extractVariableName(format);
      var_names_.push_back(var_name);

      RCLCPP_INFO(this->get_logger(), "Field %d: format='%s', var_name='%s'",
                  i, format.c_str(), var_name.c_str());
    }
  }

  /**
   * パブリッシャーを設定
   */
  void setupPublishers()
  {
    raw_data_pub_ = this->create_publisher<std_msgs::msg::String>("~/serial_data", rclcpp::QoS(10));

    for (int i = 0; i < data_count_; i++)
    {
      string topic_name = "~/data" + std::to_string(i);
      auto data_pub = this->create_publisher<std_msgs::msg::Float64>(topic_name, rclcpp::QoS(10));
      data_pubs_.push_back(data_pub);
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

      RCLCPP_INFO(this->get_logger(), "Successfully opened serial port: %s with baud rate: %d",
                  serial_port_.c_str(), baud_rate_);
    }
    catch (const OpenFailed &)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      throw std::runtime_error("Serial port did not open correctly");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error setting up serial port: %s", e.what());
      throw;
    }
  }

  /**
   * シリアルデータを処理してROSトピックとして発行
   */
  void processSerialData()
  {
    if (!serial_stream_.IsOpen())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Serial port is not open; skipping read attempt.");
      return;
    }

    std::string line;

    try
    {
      // シリアルポートから1行読み込み
      std::getline(serial_stream_, line);

      if (line.empty())
      {
        return;
      }

      RCLCPP_DEBUG(this->get_logger(), "Received: %s", line.c_str());

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
      RCLCPP_ERROR(this->get_logger(), "Error processing serial data: %s", e.what());
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
        double value = std::stod(value_str);
        data_values[var_name] = value;

        RCLCPP_DEBUG(this->get_logger(), "Parsed: %s = %f", var_name.c_str(), value);
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
        RCLCPP_DEBUG(this->get_logger(), "Missing variable: %s", var_name.c_str());
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
    std_msgs::msg::String msg;
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
    raw_data_pub_->publish(msg);

    // 各値を個別にパブリッシュ
    for (size_t i = 0; i < values.size(); i++)
    {
      std_msgs::msg::Float64 data_msg;
      data_msg.data = values[i];
      data_pubs_[i]->publish(data_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
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
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate));
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
    rclcpp::init(argc, argv);
    auto bridge = std::make_shared<SerialToRosBridge>();
    bridge->run();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("serial_to_ros"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}
