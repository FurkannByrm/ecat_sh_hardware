
#include "ecat_sh_hardware/io_tcp_server.hpp"

namespace ecat_sh_hardware {

using namespace std::chrono_literals;

void io_tcp_server_func(
  std::atomic<bool> &run_server,
  std::condition_variable &cv,
  std::shared_ptr<IoCommandQueue> queue,
  uint16_t server_port,
  uint16_t max_buffer_size
)
{

  int serverFd = 0;
  int socketFd = 0;

  sockaddr_in serverAddr;
  sockaddr_in clientAddr;
  socklen_t addresLen = sizeof(clientAddr);
  
  serverFd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (serverFd == -1)
  {
    std::cout << "Failed to create socket" << std::endl;
    return;
  }

  memset((sockaddr*)&serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_addr.s_addr = (htonl)INADDR_ANY;
  serverAddr.sin_port = htons(server_port);

  if(bind(serverFd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
  {
    std::cout << "Failed to bind socket" << std::endl;
    return;
  }

  if(listen(serverFd, 10) < 0)
  {
    std::cout << "Failed to listen on socket" << std::endl;
    return;
  }

  std::shared_ptr<IoCommandQueue> cmdQueue = queue;

  bool connected = false;
  while(run_server.load())
  {
    std::this_thread::sleep_for(20ms);
    if(!connected)
    socketFd = accept(serverFd, (sockaddr*)&clientAddr, &addresLen);
    if(socketFd == -1)
    {
      connected = false;
      continue;
    }

    if(socketFd < 0)
    {
      std::cout << "Failed to accept connection" << std::endl;
      connected = false;
      return;
    }
    else{
      connected = true;
    }

    if(connected){
    char messageBuffer[1024] = {0};
    int bytesRead = read(socketFd, messageBuffer, 1024);

    // Error cases
    if(bytesRead == -1)
    {
      //if(!connected)
      //  close(socketFd);
      continue;
    }
    else if(bytesRead == 0)
    {
      //if(!connected)
      //  close(socketFd);
      continue;
    }
    //std::cout << bytesRead << std::endl;
    // Deserialize message
    std::string reqStr = std::string(messageBuffer);
    
    std::cout << reqStr << "\n";
    auto reqOpt = IoRequest::fromStr(reqStr);
    if(!reqOpt.has_value())
    {
     
    }
    const auto request = reqOpt.value();
    
    for(const auto& [key, type, value] : request.requests)
    {
      std::lock_guard<std::mutex> lock(cmdQueue->commandQueueMutex);
      queue->commandQueue.push(IoCommandQueue::Command{type, key, value.value()});
    }

    std::unique_lock lk(queue->commandQueueMutex);
    cv.wait(lk, [&queue]{return queue->commandQueue.empty();});
    }
    
    //
    //std::vector<IoCommandQueue::Response> responses;
    //{
    //  std::lock_guard<std::mutex> lock(cmdQueue->commandQueueMutex);
    //  while(!queue->responseQueue.empty())
    //  {
    //    responses.push_back(queue->responseQueue.front());
    //    queue->responseQueue.pop();
    //  }
    //}
//
    //if(!responses.empty())
    //{
    //  IoResponse resp;
    //  resp.timestamp = std::chrono::system_clock::now();
//
    //  for(const auto& response : responses)
    //  {
    //    resp.responses.push_back(std::make_tuple(response.index, response.value));
    //  }
    //  std::string respStr = IoResponse::toJsonStr(resp);
    //  size_t writenBytes = write(socketFd, respStr.c_str(), sizeof(char) * respStr.size());
    //  if(writenBytes < 0)
    //  {
    //    std::cout << "Failed to write response" << std::endl;
    //  }
    //  
    //}
    
    if(!connected)
    close(socketFd);

  }

  close(serverFd);

}

}