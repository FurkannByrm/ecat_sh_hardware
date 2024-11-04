
#include "ecat_sh_hardware/shared_memory_handler.hpp"
#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecat_sh_hardware/utils.hpp"
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <iostream>

SharedMemoryHandler::SharedMemoryHandler() {}

SharedMemoryHandler::~SharedMemoryHandler() {

  sem_unlink(shared_obj_info::ETHERCAT_DATA_SEM_NAME);
  sem_close(m_EcatDataSem.get());
  
  munmap(m_SharedObjectAddressPtr, m_SharedMemorySize);
  close(m_SharedMemoryFd);
  shm_unlink(shared_obj_info::SHARED_MEMORY_SEG_NAME);
}

ecat_sh_hardware::Error SharedMemoryHandler::init(ShMode shmode) {

  m_SharedMemorySize = 2 * sizeof(shared_obj_info::EthercatDataObject);

  int shmFlag = O_RDWR; 
  int semFlag = 0;
  if(shmode == ShMode::Creator)
  {
    
    shmFlag = shmFlag | O_CREAT | O_EXCL;
    semFlag = O_CREAT;
  }
  else if(shmode == ShMode::User)
  {
    shmFlag = shmFlag;
  }

  m_SharedMemoryFd = shm_open(shared_obj_info::SHARED_MEMORY_SEG_NAME,
                              shmFlag, S_IRWXU);

  if (m_SharedMemoryFd == -1) {
    //std::cout << "Could not open shared memory file descriptor." << std::endl;
    printf("%s\n", strerror(errno));
    return ecat_sh_hardware::Error::shmOpenError;
  }
  
  if(shmode == ShMode::Creator){
  int extentMemoryRes = ftruncate(m_SharedMemoryFd, m_SharedMemorySize);
  if (extentMemoryRes == -1) {
    std::cout << "Could not truncate memory." << std::endl;
    return ecat_sh_hardware::Error::ftruncateError;
  }
  }

  m_SharedObjectAddressPtr = (shared_obj_info::EthercatDataObject *)mmap(
      NULL, m_SharedMemorySize, PROT_READ | PROT_WRITE, MAP_SHARED,
      m_SharedMemoryFd, 0);
  if (m_SharedObjectAddressPtr == MAP_FAILED) {
    std::cout << "Could not map memory" << std::endl;
    return ecat_sh_hardware::Error::mmapError;
  }

  sem_t *semPtr = sem_open(shared_obj_info::ETHERCAT_DATA_SEM_NAME, semFlag, S_IRUSR | S_IWUSR);

  if (semPtr == SEM_FAILED) {
    std::cout << "Could not create semaphore" << std::endl;
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
