#ifndef SIMITL_SHAREDMEM_C_INTERFACE_H
#define SIMITL_SHAREDMEM_C_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
  bool SimITLOpen();
  void SimITLDestroy();
  bool SimITLRead();
  bool SimITLWrite();
#ifdef __cplusplus
}
#endif
#endif