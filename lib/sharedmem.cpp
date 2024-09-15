#include "sharedmem.hxx"
#include <iostream>
#include <string>
#include <cstring>

namespace SimITL{

  SharedMem::~SharedMem(){
    destroy();
  }

  bool SharedMem::createMemoryMap(size_t bufferSize, const char * identifier) {
#ifdef _WIN32
    // Open the shared memory object
    mHandle = CreateFileMappingA(
      INVALID_HANDLE_VALUE,
      NULL,
      PAGE_READWRITE,
      0,
      sizeof(SharedBuffer) + bufferSize,
      (std::string("Local\\") + std::string(identifier)).c_str()
    );

    if (mHandle == NULL) {
      std::cerr << "Could not create file mapping object." << std::endl;
      return false;
    }

    // Map the shared memory into the address space of the process
    mSharedBuffer = (SharedBuffer*)MapViewOfFile(
      mHandle,
      FILE_MAP_ALL_ACCESS,
      0,
      0,
      sizeof(SharedBuffer) + bufferSize
    );

    if (mSharedBuffer == NULL) {
      std::cerr << "Could not map view of file." << std::endl;
      CloseHandle(mHandle);
      return false;
    }
#endif 
#ifdef __linux__
    // Open the shared memory object
    int mHandle = shm_open( (std::string("/") + std::string(identifier)).c_str(), O_CREAT | O_RDWR, 0666);
    if (mHandle == -1) {
      std::cerr << "Failed to create shared memory object." << std::endl;
      return false;
    }

    // Configure the size of the shared memory object
    if(ftruncate(mHandle, sizeof(SharedBuffer) + bufferSize) != 0){
      std::cerr << "Failed to resize shared memory." << std::endl;
      close(mHandle);
      return false;
    }

    // Map the shared memory into the process address space
    mSharedBuffer = (SharedBuffer*)mmap(
      NULL,
      sizeof(SharedBuffer) + bufferSize,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      mHandle,
      0
    );

    if (mSharedBuffer == MAP_FAILED) {
      std::cerr << "Failed to map shared memory." << std::endl;
      close(mHandle);
      return false;
    }
#endif

     // Initialize buffer
    mSharedBuffer->head.store(0);
    mSharedBuffer->tail.store(0);
    mSharedBuffer->size = bufferSize;
    // set buffer memory to 0
    memset(BUF_DATA(mSharedBuffer), 0, sizeof(bufferSize)); 

    return true;
  }

  bool SharedMem::openMemoryMap(size_t bufferSize, const char * identifier) {
#ifdef _WIN32
    // Open the shared memory object
    mHandle = OpenFileMappingA(
      FILE_MAP_ALL_ACCESS,
      FALSE,
      (std::string("Local\\") + std::string(identifier)).c_str()
    );

    if (mHandle == NULL) {
      std::cerr << "Could not open file mapping object." << std::endl;
      return false;
    }

    // Map the shared memory into the address space of the process
    mSharedBuffer = (SharedBuffer*)MapViewOfFile(
      mHandle,
      FILE_MAP_ALL_ACCESS,
      0,
      0,
      sizeof(SharedBuffer) + bufferSize
    );

    if (mSharedBuffer == NULL) {
      std::cerr << "Could not map view of file." << std::endl;
      CloseHandle(mHandle);
      return false;
    }
#endif 
#ifdef __linux__
    // Open the shared memory object
    int mHandle = shm_open( (std::string("/") + std::string(identifier)).c_str(), O_RDWR, 0666);
    if (mHandle == -1) {
      std::cerr << "Failed to open shared memory object." << std::endl;
      return false;
    }

    // Map the shared memory into the process address space
    mSharedBuffer = (SharedBuffer*)mmap(
      NULL,
      sizeof(SharedBuffer) + bufferSize,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      mHandle,
      0
    );

    if (mSharedBuffer == MAP_FAILED) {
      std::cerr << "Failed to map shared memory." << std::endl;
      close(mHandle);
      return false;
    }
#endif

    return true;
  }

  bool SharedMem::destroy(){
    // Unmap the shared memory
#ifdef _WIN32
    UnmapViewOfFile(mSharedBuffer);
    CloseHandle(mHandle);
#endif 
#ifdef __linux__
    munmap(mSharedBuffer, sizeof(SharedBuffer));
    close(mHandle);
#endif
    return true;
  }

  int SharedMem::read(char * dest, size_t len){
    int tail = mSharedBuffer->tail.load();
    int head = mSharedBuffer->head.load();
    int bufferSize = mSharedBuffer->size;
    char * bufferArray = BUF_DATA(mSharedBuffer);

    //std::cout << "SimITLMemRead: head: " << head << " tail: " << tail << std::endl;

    if (tail == head) {
      return 0;
    }

    int messageLength = *(int*)&bufferArray[tail];
    tail += sizeof(int);

    if(tail + messageLength >= bufferSize){
      //std::cout << "SimITLMemRead: wrap-around at: " << tail << " with: " << messageLength << std::endl;
      tail = 0;
      messageLength = *(int*)&bufferArray[tail];
      tail += sizeof(int);
    }

    if (messageLength > len) {
      //std::cout << "SimITLMemRead: messageLength > len" << std::endl;
      return 0;
    }

    memcpy(dest, &bufferArray[tail], messageLength);
    tail += messageLength;

    mSharedBuffer->tail.store(tail % bufferSize);
    return messageLength;
  }

  int SharedMem::write(const char * src, size_t len){
    int head = mSharedBuffer->head.load();
    int tail = mSharedBuffer->tail.load();
    int bufferSize = mSharedBuffer->size;
    char * bufferArray = BUF_DATA(mSharedBuffer);

    //std::cout << "SimITLMemWrite: head: " << head << " tail: " << tail << std::endl;

    int availableSpace = 0;
    if (head >= tail) {
      availableSpace = bufferSize - head + tail - 1;
    } else {
      availableSpace = tail - head - 1;
    }

    if (availableSpace < len + sizeof(int)) {
      //std::cout << "SimITLMemWrite: availableSpace < len + sizeof(int)" << std::endl;
      return 0;
    }

    // send int is a placeholder indicator for the read function in case the last message fits perfectly
    // there is still an int which can be used as indicator...
    if (head + len + sizeof(int) + sizeof(int) >= bufferSize) {
      // insert a number bigger than the rest of the buffer to indicate a wrap around
      *(int*)&bufferArray[head] = bufferSize;
      //std::cout << "SimITLMemWrite: inserting wrap around at: " << head << std::endl;
      head = 0;
    }

    *(int*)&bufferArray[head] = len;
    head += sizeof(int);
    memcpy(&bufferArray[head], src, len);
    head += len;
    mSharedBuffer->head.store(head % bufferSize);

    return len;
  }

}