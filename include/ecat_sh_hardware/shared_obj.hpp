

#ifndef SHARED_OBJ_HPP_
#define SHARED_OBJ_HPP_

#include <cstdint>
namespace shared_obj_info {

constexpr auto SHARED_MEMORY_SEG_NAME = "/SH_ETHERCAT_DATA";
constexpr auto ETHERCAT_DATA_SEM_NAME = "/SH_ETHERCAT_DATA_SEM";

struct EthercatDataObject {
  // Rx Objects
  uint16_t control_word;
  uint8_t operation_mode;
  int32_t target_position;
  int32_t target_velocity;

  // Tx Objects:

  uint16_t status_word;
  int32_t current_position;
  int32_t current_velocity;

  EthercatDataObject() :
    control_word(0x0),
    operation_mode(0x0),
    target_position(0x0),
    target_velocity(0x0),
    status_word(0x0),
    current_position(0x0),
    current_velocity(0x0)
  {

  }

  EthercatDataObject(EthercatDataObject const&) = default;
  
  EthercatDataObject& operator=(EthercatDataObject other)
  {
    this->control_word = other.control_word;
    this->operation_mode = other.operation_mode;
    this->target_position = other.target_position;
    this->target_velocity = other.target_velocity;
    this->status_word = other.status_word;
    this->current_position = other.current_position;
    this->current_velocity = other.current_velocity;
    return *this;
  }
};

} // namespace shared_obj_info

#endif // SHARED_OBJ_HPP_
