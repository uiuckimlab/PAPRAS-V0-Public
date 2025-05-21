// Copyright 2024 ROBOTIS CO., LTD.
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
//
// Authors: Hye-Jong KIM, Sungho Woo
// -----------------------------------------------------------------------------
// Modified by Kinetic Intelligent Machine LAB (KIMLAB), UIUC
// Description: Adapted for PAPRAS system
// Authors: Obin Kwon, Sankalp Yamsani
// -----------------------------------------------------------------------------


#include "papras_hw/dynamixel/dynamixel_info.hpp"
#include <string>
#include <utility>
#include <vector>

namespace papras_hardware_interface
{

void DynamixelInfo::SetDxlModelFolderPath(const char * path)
{
  dxl_model_file_dir = std::string(path);
}

void DynamixelInfo::InitDxlModelInfo()
{
  std::string model_file = dxl_model_file_dir + "/dynamixel.model";
  std::ifstream open_file(model_file.c_str());
  if (open_file.is_open() != 1) {
    fprintf(stderr, "[ERROR] CANNOT FIND DXL MODEL LIST FILE.\n%s\n", model_file.c_str());
    exit(-1);
  }
  std::string line;
  getline(open_file, line);

  //fprintf(stderr, "Dynamixel Information File List.\n");
  while (!open_file.eof()) {
    uint16_t model_number;
    std::string file_name;
    open_file >> model_number >> file_name;
    if (open_file.good()) {
      //fprintf(stderr, "num: %d, name: %s\n", model_number, file_name.c_str());
      dxl_model_list_.insert(std::make_pair(model_number, file_name));
    }
  }
  open_file.close();
}

void DynamixelInfo::ReadDxlModelFile(uint8_t id, uint16_t model_num)
{
  std::string path = dxl_model_file_dir + "/";

  auto it = dxl_model_list_.find(model_num);
  if (it != dxl_model_list_.end()) {
    path += it->second;
  } else {
    fprintf(stderr, "[ERROR] CANNOT FIND THE DXL MODEL FROM FILE LIST.\n");
    return;
  }
 
  std::ifstream open_file(path);
  if (open_file.is_open() != 1) {
    fprintf(stderr, "[ERROR] CANNOT FIND DXL [%s] MODEL FILE.\n", path.c_str());
    exit(-1);
  }

  DxlInfo temp_dxl_info;
  std::string line;

  temp_dxl_info.model_num = model_num;

  while (!open_file.eof() ) {
    getline(open_file, line);
    if (strcmp(line.c_str(), "[control table]") == 0) {
      break;
    }

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of("\t"));

    // print strs
    // fprintf(stderr, "Control Item printing strs\n");
    // for (size_t i = 0; i < strs.size(); i++) {
    //   fprintf(stderr, "str[%d] %s ", i, strs.at(i).c_str());
    // }

    if (strs.at(0) == "value_of_zero_radian_position") {
      temp_dxl_info.value_of_zero_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
    } else if (strs.at(0) == "value_of_max_radian_position") {
      temp_dxl_info.value_of_max_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
    } else if (strs.at(0) == "value_of_min_radian_position") {
      temp_dxl_info.value_of_min_radian_position = static_cast<int32_t>(stoi(strs.at(1)));
    } else if (strs.at(0) == "min_radian") {
      temp_dxl_info.min_radian = static_cast<double>(stod(strs.at(1)));
    } else if (strs.at(0) == "max_radian") {
      temp_dxl_info.max_radian = static_cast<double>(stod(strs.at(1)));
    } else if (strs.at(0) == "torque_constant") {
      temp_dxl_info.torque_constant = static_cast<double>(stod(strs.at(1)));
    } else if (strs.at(0) == "unit_scale") { // [PAPRAS] added for calculating proper position range
      temp_dxl_info.unit_scale = static_cast<double>(stod(strs.at(1)));
    }
  }

  getline(open_file, line);
  while (!open_file.eof() ) {
    getline(open_file, line);
    if (!open_file.good()) {
      break;
    }

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of("\t"));

    ControlItem temp;
    temp.address = static_cast<uint16_t>(stoi(strs.at(0)));
    temp.size = static_cast<uint8_t>(stoi(strs.at(1)));
    temp.item_name = strs.at(2);
    temp_dxl_info.item.push_back(temp);
  }

  dxl_info_[id] = temp_dxl_info;
  open_file.close();
}

bool DynamixelInfo::GetDxlControlItem(
  uint8_t id, std::string item_name, uint16_t & addr,
  uint8_t & size)
{
  for (size_t i = 0; i < dxl_info_[id].item.size(); i++) {
    if (strcmp(item_name.c_str(), dxl_info_[id].item.at(i).item_name.c_str()) == 0) {
      addr = dxl_info_[id].item.at(i).address;
      size = dxl_info_[id].item.at(i).size;
      return true;
    }
  }
  return false;
}

bool DynamixelInfo::CheckDxlControlItem(uint8_t id, std::string item_name)
{
  for (size_t i = 0; i < dxl_info_[id].item.size(); i++) {
    if (strcmp(item_name.c_str(), dxl_info_[id].item.at(i).item_name.c_str()) == 0) {
      return true;
    }
  }
  return false;
}

bool DynamixelInfo::GetDxlTypeInfo(
  uint8_t id,
  int32_t & value_of_zero_radian_position,
  int32_t & value_of_max_radian_position,
  int32_t & value_of_min_radian_position,
  double & min_radian,
  double & max_radian,
  double & torque_constant)
{
  value_of_zero_radian_position = dxl_info_[id].value_of_zero_radian_position;
  value_of_max_radian_position = dxl_info_[id].value_of_max_radian_position;
  value_of_min_radian_position = dxl_info_[id].value_of_min_radian_position;
  min_radian = dxl_info_[id].min_radian;
  max_radian = dxl_info_[id].max_radian;
  torque_constant = dxl_info_[id].torque_constant;
  return true;
}

int32_t DynamixelInfo::ConvertRadianToValue(uint8_t id, double radian)
{
  if (radian > 0) {
    return static_cast<int32_t>(radian *
           (dxl_info_[id].value_of_max_radian_position -
           dxl_info_[id].value_of_zero_radian_position) / dxl_info_[id].max_radian) +
           dxl_info_[id].value_of_zero_radian_position;
  } else if (radian < 0) {
    return static_cast<int32_t>(radian *
           (dxl_info_[id].value_of_min_radian_position -
           dxl_info_[id].value_of_zero_radian_position) / dxl_info_[id].min_radian) +
           dxl_info_[id].value_of_zero_radian_position;
  } else {
    return dxl_info_[id].value_of_zero_radian_position;
  }
}

double DynamixelInfo::ConvertValueToRadian(uint8_t id, int32_t value)
{
  double val = 0.0;

  if (value > dxl_info_[id].value_of_zero_radian_position) {
    val = static_cast<double>(value - dxl_info_[id].value_of_zero_radian_position) * dxl_info_[id].max_radian /
    static_cast<double>(dxl_info_[id].value_of_max_radian_position - dxl_info_[id].value_of_zero_radian_position);
  } else if (value < dxl_info_[id].value_of_zero_radian_position) {
    val = static_cast<double>(value - dxl_info_[id].value_of_zero_radian_position) * dxl_info_[id].min_radian /
    static_cast<double>(dxl_info_[id].value_of_min_radian_position - dxl_info_[id].value_of_zero_radian_position);
  }
  return val;
}


double DynamixelInfo::CustomValueToRadian(uint8_t id, int32_t value)
{
  double val = 0.0;

  if (value > dxl_info_[id].value_of_zero_radian_position) {
    val = static_cast<double>(value - dxl_info_[id].value_of_zero_radian_position) * dxl_info_[id].max_radian /
    static_cast<double>(dxl_info_[id].value_of_max_radian_position - dxl_info_[id].value_of_zero_radian_position);
  } else if (value < dxl_info_[id].value_of_zero_radian_position) {
    val = static_cast<double>(value - dxl_info_[id].value_of_zero_radian_position) * dxl_info_[id].min_radian /
    static_cast<double>(dxl_info_[id].value_of_min_radian_position - dxl_info_[id].value_of_zero_radian_position);
  }
  if (val > M_PI){
    val -= 2 * M_PI;
  } else if (val < -M_PI) {
    val += 2 * M_PI;
  }
  return val;
}
}  // namespace dynamixel_hardware_interface
