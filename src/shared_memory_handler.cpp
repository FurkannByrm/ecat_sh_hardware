
#include "ecat_sh_hardware/shared_memory_handler.hpp"
#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecat_sh_hardware/utils.hpp"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>

SharedMemoryHandler::SharedMemoryHandler() {}

SharedMemoryHandler::~SharedMemoryHandler() {

  sem_close(m_EcatDataSem.get());
  sem_unlink(shared_obj_info::ETHERCAT_DATA_SEM_NAME);

  munmap(m_SharedObjectAddressPtr, m_SharedMemorySize);
  close(m_SharedMemoryFd);
  shm_unlink(shared_obj_info::SHARED_MEMORY_SEG_NAME);
}

ecat_sh_hardware::Error SharedMemoryHandler::init() {

  m_SharedMemoryFd = shm_open(shared_obj_info::SHARED_MEMORY_SEG_NAME,
                              O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);

  if (m_SharedMemoryFd == -1) {
    return ecat_sh_hardware::Error::shmOpenError;
  }

  int extentMemoryRes = ftruncate(m_SharedMemoryFd, m_SharedMemorySize);
  if (extentMemoryRes == -1) {
    return ecat_sh_hardware::Error::ftruncateError;
  }

  m_SharedObjectAddressPtr = (shared_obj_info::EthercatDataObject *)mmap(
      NULL, m_SharedMemorySize, PROT_READ | PROT_WRITE, MAP_SHARED,
      m_SharedMemoryFd, 0);
  if (m_SharedObjectAddressPtr == MAP_FAILED) {
    return ecat_sh_hardware::Error::mmapError;
  }

  sem_t *semPtr = sem_open(shared_obj_info::ETHERCAT_DATA_SEM_NAME, O_CREAT);

  if (semPtr == SEM_FAILED) {
    return ecat_sh_hardware::Error::semOpenError;
  }

  m_EcatDataSem = std::make_unique<sem_t>(*semPtr);

  return ecat_sh_hardware::Error::NoError;
}

std::optional<std::vector<shared_obj_info::EthercatDataObject>>
SharedMemoryHandler::getEcDataObject() {

  std::vector<shared_obj_info::EthercatDataObject> objects;

  objects.resize(m_SharedMemorySize);

  for (std::vector<shared_obj_info::EthercatDataObject>::iterator objIt =
           objects.begin();
       objIt != objects.end(); objIt++) {
    shared_obj_info::EthercatDataObject tempObj =
        m_SharedObjectAddressPtr[std::distance(objects.begin(), objIt)];
    *objIt = tempObj;
  }

  return objects;
}

ecat_sh_hardware::Error SharedMemoryHandler::sendEcDataObject(
    const std::vector<shared_obj_info::EthercatDataObject> &objects) {

  for (std::vector<shared_obj_info::EthercatDataObject>::const_iterator objIt =
           objects.cbegin();
       objIt != objects.cend(); objIt++) {
    m_SharedObjectAddressPtr[std::distance(objects.begin(), objIt)] = *objIt;
  }

  return ecat_sh_hardware::Error::NoError;
}
