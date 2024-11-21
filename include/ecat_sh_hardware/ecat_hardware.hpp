#ifndef ECAT_HARDWARE_HPP_
#define ECAT_HARDWARE_HPP_

#include "ecrt.h"
#include "ecat_sh_hardware/utils.hpp"

#include "ipc_handlers/shm_handler.hpp"

#include <sched.h>
#include <optional>
#include <stdint.h>
#include <string.h>
#include <errno.h>

template <typename T>
std::optional<T> readFromSlave(uint8_t *domainDataPtr, uint object_offset, uint bit_position = 0) {
  // auto data = m_DomainProcessDataPtr +
  // *m_SlaveOffsets->getData(value_to_read_name);

  auto data = domainDataPtr + object_offset;

  if constexpr (std::is_same_v<uint8_t, T>) {
    return EC_READ_U8(data);
  } else if constexpr (std::is_same_v<uint16_t, T>) {
    return EC_READ_U16(data);
  } else if constexpr (std::is_same_v<uint32_t, T>) {
    return EC_READ_U32(data);
  } else if constexpr (std::is_same_v<uint64_t, T>) {
    return EC_READ_U64(data);
  } else if constexpr (std::is_same_v<int8_t, T>) {
    return EC_READ_S8(data);
  } else if constexpr (std::is_same_v<int16_t, T>) {
    return EC_READ_S16(data);
  } else if constexpr (std::is_same_v<int32_t, T>) {
    return EC_READ_S32(data);
  } else if constexpr (std::is_same_v<int64_t, T>) {
    return EC_READ_S64(data);
  }
  else if constexpr (std::is_same_v<bool, T>){
    return EC_READ_BIT(data, bit_position);
  }

  return std::nullopt;
}

template <typename T>
void writeToSlave(uint8_t *domainDataPtr, uint object_offset,
                  const T &new_val, uint bit_position = 0) {

  auto data = domainDataPtr + object_offset;
  /*
Control statement for determining which type of value is used with the
template function. Uses the appropriate macro function from the ecrt.h to
write according to the template argument.
*/
  if constexpr (std::is_same_v<uint8_t, T>) {
    EC_WRITE_U8(data, new_val);
  } else if constexpr (std::is_same_v<uint16_t, T>) {
    EC_WRITE_U16(data, new_val);
  } else if constexpr (std::is_same_v<uint32_t, T>) {
    EC_WRITE_U32(data, new_val);
  } else if constexpr (std::is_same_v<uint64_t, T>) {
    EC_WRITE_U64(data, new_val);
  } else if constexpr (std::is_same_v<int8_t, T>) {
    EC_WRITE_S8(data, new_val);
  } else if constexpr (std::is_same_v<int16_t, T>) {
    EC_WRITE_S16(data, new_val);
  } else if constexpr (std::is_same_v<int32_t, T>) {
    EC_WRITE_S32(data, new_val);
  } else if constexpr (std::is_same_v<int64_t, T>) {
    EC_WRITE_S64(data, new_val);
  }
  else if constexpr (std::is_same_v<bool, T>){
    EC_WRITE_BIT(data, bit_position, new_val);
  }
}

#endif // ECAT_HARDWARE_HPP_
