#ifndef SHARED_MEMORY_HANDLER_HPP_
#define SHARED_MEMORY_HANDLER_HPP_

#include "ecat_sh_hardware/shared_obj.hpp"
#include "ecat_sh_hardware/utils.hpp"

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <optional>
#include <memory>
#include <vector>

enum class ShMode
{
  Creator,
  User
};

class SharedMemoryHandler {

public:
  SharedMemoryHandler();

  SharedMemoryHandler(const SharedMemoryHandler &other) = delete;

  SharedMemoryHandler operator=(const SharedMemoryHandler &other) = delete;

  SharedMemoryHandler(SharedMemoryHandler &&other) = delete;

  SharedMemoryHandler &operator=(SharedMemoryHandler &&other) = delete;

  ~SharedMemoryHandler();

  ecat_sh_hardware::Error init(ShMode shmode = ShMode::Creator);

  std::optional<std::vector<shared_obj_info::EthercatDataObject>> getEcDataObject();

  ecat_sh_hardware::Error sendEcDataObject(
      const std::vector<shared_obj_info::EthercatDataObject> &objects);

  inline int tryLockSem()
  {
    return sem_trywait(m_EcatDataSem.get());
  }
  
  inline bool lockSem() {
    int gotLock = sem_wait(m_EcatDataSem.get());
    //printf("%s\n", strerror(errno));
    return (gotLock == 0 ? true : false);
  }

  inline bool freeSem() {
    int couldFree = sem_post(m_EcatDataSem.get());
    //printf("%s\n", strerror(errno));
    return (couldFree == 0 ? true : false);
  }

private:
  int m_SharedMemoryFd;

  std::size_t m_SharedMemorySize;

  shared_obj_info::EthercatDataObject *m_SharedObjectAddressPtr;

  std::unique_ptr<sem_t> m_EcatDataSem;
};

#endif // SHARED_MEMORY_HANDLER_HPP_
