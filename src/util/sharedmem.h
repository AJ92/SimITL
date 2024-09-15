#ifndef SHARED_MEM_INTERFACE
#define SHARED_MEM_INTERFACE

// uses the shared mem game client interface from ../lib

extern "C" {
  void * SimITLMemCreate(int bufferSize, const char * identifier);

  void * SimITLMemOpen(int bufferSize, const char * identifier);

  void SimITLMemDestroy(void * instance);

  int SimITLMemRead(void * instance, void * dest, int bytes);

  int SimITLMemWrite(void * instance, void * src, int bytes);
}

#endif