#ifndef SHARED_MEM_INTERFACE
#define SHARED_MEM_INTERFACE

// c only header

extern "C" {
  void * SimITLMemCreate(int bufferSize, const char * identifier);

  void * SimITLMemOpen(int bufferSize, const char * identifier);

  void SimITLMemDestroy(void * instance);

  int SimITLMemRead(void * instance, void * dest, int bytes);

  int SimITLMemWrite(void * instance, void * src, int bytes);
}

#endif