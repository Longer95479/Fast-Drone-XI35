#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

class ParamReader {
 public:
  // 单例
  static ParamReader& getInstance() {
    static ParamReader instance;
    return instance;
  }

  // 初始化YAML配置文件
  bool initialize(const std::string& config_file) {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      config_ = YAML::LoadFile(config_file);
      is_initialized_ = true;
      std::cout << "Successfully loaded config file: " << config_file
                << std::endl;
      return true;
    } catch (const YAML::Exception& e) {
      std::cerr << "Failed to load config file: " << config_file
                << ", error: " << e.what() << std::endl;
      is_initialized_ = false;
      return false;
    }
  }

  // 通用模板get
  template <typename T>
  T get(const std::string& key, const T& default_value = T{}) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) {
      std::cerr << "ParamReader not initialized, using default value for key: "
                << key << std::endl;
      return default_value;
    }

    try {
      YAML::Node node = getNodeByKey(key);
      if (node) {
        return node.as<T>();
      } else {
        std::cerr << "Key not found: " << key << ", using default value"
                  << std::endl;
        return default_value;
      }
    } catch (const YAML::Exception& e) {
      std::cerr << "Error parsing key: " << key << ", error: " << e.what()
                << ", using default value" << std::endl;
      return default_value;
    }
  }

  std::string getString(const std::string& key,
                        const std::string& default_value = "") {
    return get<std::string>(key, default_value);
  }

  int getInt(const std::string& key, int default_value = 0) {
    return get<int>(key, default_value);
  }

  double getDouble(const std::string& key, double default_value = 0.0) {
    return get<double>(key, default_value);
  }

  bool getBool(const std::string& key, bool default_value = false) {
    return get<bool>(key, default_value);
  }

  // 获取动态向量
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> getVector(
      const std::string& key,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& default_value =
          Eigen::Matrix<Scalar, Eigen::Dynamic, 1>()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) {
      std::cerr << "ParamReader not initialized, using default value for key: "
                << key << std::endl;
      return default_value;
    }

    try {
      YAML::Node node = getNodeByKey(key);
      if (node && node.IsSequence()) {
        std::vector<Scalar> vec;
        for (size_t i = 0; i < node.size(); ++i) {
          vec.push_back(node[i].as<Scalar>());
        }
        return Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(vec.data(),
                                                                    vec.size());
      } else {
        std::cerr << "Key not found or not a sequence: " << key
                  << ", using default value" << std::endl;
        return default_value;
      }
    } catch (const YAML::Exception& e) {
      std::cerr << "Error parsing vector key: " << key
                << ", error: " << e.what() << ", using default value"
                << std::endl;
      return default_value;
    }
  }

  // 获取动态矩阵
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> getMatrix(
      const std::string& key,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>&
          default_value =
              Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) {
      std::cerr << "ParamReader not initialized, using default value for key: "
                << key << std::endl;
      return default_value;
    }

    try {
      YAML::Node node = getNodeByKey(key);
      if (node && node.IsSequence()) {
        size_t rows = node.size();
        if (rows == 0) return default_value;

        if (!node[0].IsSequence()) {
          size_t cols = node.size();
          Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> result(1, cols);
          for (size_t j = 0; j < cols; ++j) {
            result(0, j) = node[j].as<Scalar>();
          }
          return result;
        }

        size_t cols = node[0].size();
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> result(rows,
                                                                     cols);

        for (size_t i = 0; i < rows; ++i) {
          if (!node[i].IsSequence() || node[i].size() != cols) {
            std::cerr << "Invalid matrix format for key: " << key
                      << ", using default value" << std::endl;
            return default_value;
          }
          for (size_t j = 0; j < cols; ++j) {
            result(i, j) = node[i][j].as<Scalar>();
          }
        }
        return result;
      } else {
        std::cerr << "Key not found or not a sequence: " << key
                  << ", using default value" << std::endl;
        return default_value;
      }
    } catch (const YAML::Exception& e) {
      std::cerr << "Error parsing matrix key: " << key
                << ", error: " << e.what() << ", using default value"
                << std::endl;
      return default_value;
    }
  }

  // 获取固定大小向量
  template <typename Scalar, int Size>
  Eigen::Matrix<Scalar, Size, 1> getFixedVector(
      const std::string& key,
      const Eigen::Matrix<Scalar, Size, 1>& default_value =
          Eigen::Matrix<Scalar, Size, 1>()) {
    auto dynamic_vec = getVector<Scalar>(
        key, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(Size));

    if (dynamic_vec.size() == Size) {
      return dynamic_vec;
    } else {
      std::cerr << "Vector size mismatch for key: " << key
                << ", expected: " << Size << ", got: " << dynamic_vec.size()
                << ", using default value" << std::endl;
      return default_value;
    }
  }

  // 获取固定大小矩阵
  template <typename Scalar, int Rows, int Cols>
  Eigen::Matrix<Scalar, Rows, Cols> getFixedMatrix(
      const std::string& key,
      const Eigen::Matrix<Scalar, Rows, Cols>& default_value =
          Eigen::Matrix<Scalar, Rows, Cols>()) {
    auto dynamic_mat = getMatrix<Scalar>(
        key, Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(Rows,
                                                                         Cols));

    if (dynamic_mat.rows() == Rows && dynamic_mat.cols() == Cols) {
      Eigen::Matrix<Scalar, Rows, Cols> result;
      result = dynamic_mat;
      return result;
    } else {
      std::cerr << "Matrix size mismatch for key: " << key
                << ", expected: " << Rows << "x" << Cols
                << ", got: " << dynamic_mat.rows() << "x" << dynamic_mat.cols()
                << ", using default value" << std::endl;
      return default_value;
    }
  }

  void printAllParameters() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_initialized_) {
      std::cerr << "ParamReader not initialized, cannot print parameters"
                << std::endl;
      return;
    }

    std::cout << "\n========== All Parameters ==========" << std::endl;
    printNode("", config_);
    std::cout << "====================================\n" << std::endl;
  }

  bool isInitialized() const { return is_initialized_; }

 private:
  YAML::Node config_;
  bool is_initialized_ = false;
  std::mutex mutex_;

  ParamReader() = default;
  ~ParamReader() = default;

  ParamReader(const ParamReader&) = delete;
  ParamReader& operator=(const ParamReader&) = delete;

  // 根据键名获取节点（支持嵌套键，如 "parent.child"）
  YAML::Node getNodeByKey(const std::string& key) {
    YAML::Node node = config_;
    size_t start = 0;
    size_t end = key.find('.');

    while (end != std::string::npos) {
      std::string part = key.substr(start, end - start);
      node = node[part];
      if (!node) return node;
      start = end + 1;
      end = key.find('.', start);
    }

    std::string last_part = key.substr(start);
    return node[last_part];
  }
  // 递归打印YAML节点
  void printNode(const std::string& prefix, const YAML::Node& node,
                 int indent = 0) {
    const std::string indent_str(indent * 2, ' ');
    switch (node.Type()) {
      case YAML::NodeType::Null:
        std::cout << indent_str << prefix << ": null" << std::endl;
        break;
      case YAML::NodeType::Scalar:
        std::cout << indent_str << prefix << ": " << node.Scalar() << std::endl;
        break;
      case YAML::NodeType::Sequence:
        if (prefix.empty()) {
          for (size_t i = 0; i < node.size(); ++i) {
            printNode("[" + std::to_string(i) + "]", node[i], indent);
          }
        } else {
          std::cout << indent_str << prefix << ":" << std::endl;
          bool is_matrix = node.size() > 0 && node[0].IsSequence();
          if (is_matrix) {
            for (size_t i = 0; i < node.size(); ++i) {
              std::ostringstream row_stream;
              row_stream << indent_str << "  - [";
              for (size_t j = 0; j < node[i].size(); ++j) {
                row_stream << node[i][j].as<std::string>();
                if (j < node[i].size() - 1) {
                  row_stream << ", ";
                }
              }
              row_stream << "]";
              std::cout << row_stream.str() << std::endl;
            }
          } else {
            std::ostringstream seq_stream;
            seq_stream << "[";
            for (size_t i = 0; i < node.size(); ++i) {
              seq_stream << node[i].as<std::string>();
              if (i < node.size() - 1) {
                seq_stream << ", ";
              }
            }
            seq_stream << "]";
            std::cout << indent_str << "  " << seq_stream.str() << std::endl;
          }
        }
        break;
      case YAML::NodeType::Map:
        if (prefix.empty()) {
          for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            printNode(key, it->second, indent);
          }
        } else {
          std::cout << indent_str << prefix << ":" << std::endl;
          for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
            std::string key = it->first.as<std::string>();
            printNode(key, it->second, indent + 1);
          }
        }
        break;
      case YAML::NodeType::Undefined:
        std::cout << indent_str << prefix << ": undefined" << std::endl;
        break;
    }
  }
};