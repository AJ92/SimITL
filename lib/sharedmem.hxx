#ifndef SIMITL_SHAREDMEM_H
#define SIMITL_SHAREDMEM_H

#ifdef _WIN32
  #include <windows.h>
#endif

#ifdef __linux__
  #include <fcntl.h>
  #include <sys/mman.h>
  #include <unistd.h>
#endif

#include <atomic>
#include <iostream>

#define BUF_DATA(buf) (char*) (&((buf)[1]))

namespace SimITL{
  struct SharedBuffer {
    std::atomic<int> head;
    std::atomic<int> tail;
    int size = 0;
    // char[bufferSize] as requested by create function
  };

  class SharedMem {
    public:
      SharedMem() = default;
      ~SharedMem();
      bool createMemoryMap(size_t bufferSize, const char * identifier);
      bool openMemoryMap(size_t bufferSize, const char * identifier);
      bool destroy();
      int read(char * dest, size_t len);
      int write(const char * src, size_t len);
    private:
      SharedBuffer* mSharedBuffer = nullptr;
#ifdef _WIN32
      HANDLE mHandle = 0;
#endif 
#ifdef __linux__
      int mHandle = 0;
#endif
  };
}

extern "C" {
  void * SimITLMemCreate(int bufferSize, const char * identifier){
    SimITL::SharedMem * sharedMem = new SimITL::SharedMem();
    if(sharedMem->createMemoryMap(static_cast<size_t>(bufferSize), identifier)){
      return static_cast<void *>(sharedMem);
    }
    delete(sharedMem);
    return nullptr;
  }

  void * SimITLMemOpen(int bufferSize, const char * identifier){
    SimITL::SharedMem * sharedMem = new SimITL::SharedMem();
    if(sharedMem->openMemoryMap(static_cast<size_t>(bufferSize), identifier)){
      return static_cast<void *>(sharedMem);
    }
    delete(sharedMem);
    return nullptr;
  }

  void SimITLMemDestroy(void * instance){
    SimITL::SharedMem * sharedMem = static_cast<SimITL::SharedMem*>(instance);
    if(sharedMem){
      delete(sharedMem);
    }
    sharedMem = nullptr;
  }

  int SimITLMemRead(void * instance, void * dest, int bytes){
    SimITL::SharedMem * sharedMem = static_cast<SimITL::SharedMem*>(instance);
    char * destinationArray = static_cast<char *>(dest);
    if(sharedMem){
      return sharedMem->read(destinationArray, bytes);
    }
    return 0;
  }

  int SimITLMemWrite(void * instance, void * src, int bytes){
    SimITL::SharedMem * sharedMem = static_cast<SimITL::SharedMem*>(instance);
    const char * sourceArray = static_cast<char *>(src);
    if(sharedMem){
      return sharedMem->write(sourceArray, bytes);
    }
    return 0;
  }
}

#endif