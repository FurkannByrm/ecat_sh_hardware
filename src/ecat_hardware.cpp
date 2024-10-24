
#include "ecat_sh_hardware/ecat_hardware.hpp"
#include "ecat_sh_hardware/shared_memory_handler.hpp"
#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecrt.h"

#include <unordered_map>
#include <signal.h>
#include <iostream>

constexpr auto SLAVE_START_POSITION = 0;
constexpr auto SLAVE_ALIAS = 0;
constexpr auto SLAVE_VENDOR_ID = 0x000000fb;
constexpr auto SLAVE_PRODUCT_ID = 0x65520000;
struct
{
  uint control_word;
  uint operation_mode;
  uint target_position;
  uint target_velocity;
  uint status_word;
  uint current_position;
  uint current_velocity;
} LeftMotorEthercatDataOffsets;

struct
{
  uint control_word;
  uint operation_mode;
  uint target_position;
  uint target_velocity;
  uint status_word;
  uint current_position;
  uint current_velocity;
} RightMotorEthercatDataOffsets;
/*
const ec_pdo_entry_reg_t lifterDomainRegistries[] = {
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6040, 0x00, &EthercatDataOffsets.control_word},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6060, 0x00, &EthercatDataOffsets.operation_mode},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x607a, 0x00, &EthercatDataOffsets.target_position},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x60ff, 0x0, &EthercatDataOffsets.target_velocity},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6041, 0x0, &EthercatDataOffsets.status_word},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x6064, 0x0, &EthercatDataOffsets.current_position},
    {SLAVE_START_POSITION, SLAVE_ALIAS, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID,
     0x606C, 0x0, &EthercatDataOffsets.current_velocity},
    {}};*/

const ec_pdo_entry_reg_t domainRegistries[] = {
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6040, 0x00,
    &RightMotorEthercatDataOffsets.control_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6060, 0x00,
    &RightMotorEthercatDataOffsets.operation_mode },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x607a, 0x00,
    &RightMotorEthercatDataOffsets.target_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x60ff, 0x00,
    &RightMotorEthercatDataOffsets.target_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6041, 0x00,
    &RightMotorEthercatDataOffsets.status_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6064, 0x00,
    &RightMotorEthercatDataOffsets.current_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x606C, 0x00,
    &RightMotorEthercatDataOffsets.current_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6040, 0x00,
    &LeftMotorEthercatDataOffsets.control_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6060, 0x00,
    &LeftMotorEthercatDataOffsets.operation_mode },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x607a, 0x00,
    &LeftMotorEthercatDataOffsets.target_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x60ff, 0x00,
    &LeftMotorEthercatDataOffsets.target_velocity },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6041, 0x00,
    &LeftMotorEthercatDataOffsets.status_word },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x6064, 0x00,
    &LeftMotorEthercatDataOffsets.current_position },
  { SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID, 0x606C, 0x00,
    &LeftMotorEthercatDataOffsets.current_velocity },
  {}
};

ec_pdo_entry_info_t right_motor_pdo_entries[] = { { 0x6040, 0x00, 16 },
                                                  { 0x6060, 0x00, 8 },
                                                  { 0x607a, 0x00, 32 },
                                                  { 0x60ff, 0x00, 32 },
                                                  { 0x6041, 0x00, 16 },
                                                  { 0x6064, 0x00, 32 },
                                                  { 0x606c, 0x00, 32 } };

ec_pdo_info_t right_motor_pdo_info[] = { { 0x1603, 4, right_motor_pdo_entries + 0 },
                                         { 0x1A03, 3, right_motor_pdo_entries + 4 } };

ec_sync_info_t right_motor_slave_syncs[] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                             { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                             { 2, EC_DIR_OUTPUT, 1, right_motor_pdo_info + 0, EC_WD_ENABLE },
                                             { 3, EC_DIR_INPUT, 1, right_motor_pdo_info + 1, EC_WD_DISABLE },
                                             { 0xff } };

ec_pdo_entry_info_t left_motor_pdo_entries[] = { { 0x6040, 0x00, 16 },
                                                 { 0x6060, 0x00, 8 },
                                                 { 0x607a, 0x00, 32 },
                                                 { 0x60ff, 0x00, 32 },
                                                 { 0x6041, 0x00, 16 },
                                                 { 0x6064, 0x00, 32 },
                                                 { 0x606c, 0x00, 32 } };

ec_pdo_info_t left_motor_pdo_info[] = { { 0x1603, 4, left_motor_pdo_entries + 0 },
                                        { 0x1A03, 3, left_motor_pdo_entries + 4 } };

ec_sync_info_t left_motor_slave_syncs[] = { { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
                                            { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
                                            { 2, EC_DIR_OUTPUT, 1, left_motor_pdo_info + 0, EC_WD_ENABLE },
                                            { 3, EC_DIR_INPUT, 1, left_motor_pdo_info + 1, EC_WD_DISABLE },
                                            { 0xff } };

bool runHardwareLoop = true;

void sigInHandler(int signal)
{
  runHardwareLoop = false;
}

int main(int argc, char** argv)
{
  ec_master_t* masterPtr = nullptr;
  ec_domain_t* domainPtr = nullptr;
  ec_slave_config_t* slaveConfigPtr = nullptr;
  uint8_t* domainProcessData = nullptr;

  masterPtr = ecrt_request_master(0);
  if (masterPtr == nullptr)
  {
    return 1;
  }

  domainPtr = ecrt_master_create_domain(masterPtr);
  if (domainPtr == nullptr)
  {
    return 1;
  }

  // Configure first slave:
  slaveConfigPtr =
      ecrt_master_slave_config(masterPtr, SLAVE_ALIAS, SLAVE_START_POSITION, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID);
  if (slaveConfigPtr == nullptr)
  {
    return 1;
  }

  int configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, right_motor_slave_syncs);
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure right motor driver PDOs." << std::endl;
    return 1;
  }

  ecrt_slave_config_dc(slaveConfigPtr, 0x0300, 2000000, 1000000, 2000000, 0);

  // Configure second slave:

  slaveConfigPtr =
      ecrt_master_slave_config(masterPtr, SLAVE_ALIAS, SLAVE_START_POSITION + 1, SLAVE_VENDOR_ID, SLAVE_PRODUCT_ID);
  configurePdosRes = ecrt_slave_config_pdos(slaveConfigPtr, EC_END, left_motor_slave_syncs);
  if (configurePdosRes != 0)
  {
    std::cout << "Could not configure left motor driver PDOs." << std::endl;
    return 1;
  }

  int registerDomainEntriesRes = ecrt_domain_reg_pdo_entry_list(domainPtr, domainRegistries);

  if (registerDomainEntriesRes != 0)
  {
    std::cout << "Could not register PDO list." << std::endl;
    return 1;
  }

  ecrt_slave_config_dc(slaveConfigPtr, 0x0300, 2000000, 1000000, 2000000, 0);

  // Initiliaze shared memory:

  SharedMemoryHandler shMemHandler;

  if (shMemHandler.init() != ecat_sh_hardware::Error::NoError)
  {
    std::cout << "Could not initialize shared memory handler" << std::endl;
    return 1;
  }

  // Activate EtherCAT:

  int activateMasterRes = ecrt_master_activate(masterPtr);
  if (activateMasterRes != 0)
  {
    std::cout << "Could not activate master" << std::endl;
    return 1;
  }

  if (!(domainProcessData = ecrt_domain_data(domainPtr)))
  {
    std::cout << "Could not create domain data" << std::endl;
    return 1;
  }

  // Connect shutdown signal to SIGINT:
  signal(SIGINT, sigInHandler);

  ecat_sh_hardware::DistributedClockHelper distributedClockHelper;

  clock_gettime(CLOCK_MONOTONIC, &distributedClockHelper.wakeupTime);

  shared_obj_info::EthercatDataObject rightWheelData;
  shared_obj_info::EthercatDataObject leftWheelData;

  auto periodNs = ecat_sh_hardware::NANOSEC_PER_SEC / 500;
  distributedClockHelper.cycleTime = { 0, periodNs };
  distributedClockHelper.referenceClockCounter = 0;
/* 
  struct sched_param processParam = {};
  processParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
  if(sched_setscheduler(0, SCHED_FIFO, &processParam) != 0)
  {
    std::cout << "Could not change process priority" << std::endl;
    return 1;
  } */

  while (runHardwareLoop)
  {
    // DC sync
    distributedClockHelper.wakeupTime =
        ecat_sh_hardware::addTimespec(distributedClockHelper.wakeupTime, distributedClockHelper.cycleTime);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &distributedClockHelper.wakeupTime, NULL);

    ecrt_master_application_time(masterPtr, ecat_sh_hardware::timespecToNanoSec(distributedClockHelper.wakeupTime));

    ecrt_master_receive(masterPtr);
    ecrt_domain_process(domainPtr);

    /**
     * Read || Write Logic
     *
     */
    auto rightMotorSW = readFromSlave<uint16_t>(domainProcessData, RightMotorEthercatDataOffsets.status_word);
    if (rightMotorSW)
    {
      rightWheelData.status_word = rightMotorSW.value();
    }
    auto rightMotorCurrentPosition =
        readFromSlave<int32_t>(domainProcessData, RightMotorEthercatDataOffsets.current_position);
    if (rightMotorCurrentPosition)
    {
      rightWheelData.current_position = rightMotorCurrentPosition.value();
    }
    auto rightMotorCurrentVelocity =
        readFromSlave<int32_t>(domainProcessData, RightMotorEthercatDataOffsets.current_position);
    if (rightMotorCurrentPosition)
    {
      rightWheelData.current_position = rightMotorCurrentVelocity.value();
    }

    auto leftMotorSW = readFromSlave<uint16_t>(domainProcessData, LeftMotorEthercatDataOffsets.status_word);
    if (leftMotorSW)
    {
      leftWheelData.status_word = leftMotorSW.value();
    }
    auto leftMotorCurrentPosition =
        readFromSlave<int32_t>(domainProcessData, LeftMotorEthercatDataOffsets.current_position);
    if (leftMotorCurrentPosition)
    {
      leftWheelData.current_position = leftMotorCurrentPosition.value();
    }
    auto leftMotorCurrentVelocity =
        readFromSlave<int32_t>(domainProcessData, LeftMotorEthercatDataOffsets.current_position);
    if (leftMotorCurrentVelocity)
    {
      leftWheelData.current_position = leftMotorCurrentVelocity.value();
    }

    // If we get a lock on the semaphore, read/write from/to EtherCAT:
    if (shMemHandler.tryLockSem() == 0)
    {
      auto receivedDataObj = shMemHandler.getEcDataObject();
      if (receivedDataObj)
      {
        std::vector objs = receivedDataObj.value();

        if (!objs.empty())
        {
          // objs[0] := right wheel

          objs[0].status_word = rightWheelData.status_word;
          objs[0].current_position = rightWheelData.current_position;
          objs[0].current_velocity = rightWheelData.current_velocity;

          rightWheelData.control_word = objs[0].control_word;
          rightWheelData.operation_mode = objs[0].operation_mode;
          rightWheelData.target_position = objs[0].target_position;
          rightWheelData.target_velocity = objs[0].target_velocity;

          // objs[1] := left wheel
          objs[1].status_word = leftWheelData.status_word;
          objs[1].current_position = leftWheelData.current_position;
          objs[1].current_velocity = leftWheelData.current_velocity;

          leftWheelData.control_word = objs[1].control_word;
          leftWheelData.operation_mode = objs[1].operation_mode;
          leftWheelData.target_position = objs[1].target_position;
          leftWheelData.target_velocity = objs[1].target_velocity;
        }
      }
    }

    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.control_word, rightWheelData.control_word);
    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.operation_mode, rightWheelData.operation_mode);
    writeToSlave(domainProcessData, RightMotorEthercatDataOffsets.target_velocity, rightWheelData.target_velocity);

    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.control_word, rightWheelData.control_word);
    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.operation_mode, rightWheelData.operation_mode);
    writeToSlave(domainProcessData, LeftMotorEthercatDataOffsets.target_velocity, rightWheelData.target_velocity);

    clock_gettime(CLOCK_MONOTONIC, &distributedClockHelper.currentTime);
    if (distributedClockHelper.referenceClockCounter)
    {
      distributedClockHelper.referenceClockCounter -= 1;
      ecrt_master_sync_reference_clock_to(masterPtr,
                                          ecat_sh_hardware::timespecToNanoSec(distributedClockHelper.currentTime));
    }
    else
    {
      distributedClockHelper.referenceClockCounter = 1;
    }

    ecrt_master_sync_slave_clocks(masterPtr);

    ecrt_domain_queue(domainPtr);
    ecrt_master_send(masterPtr);
  }

  ecrt_release_master(masterPtr);

  return 0;
}
