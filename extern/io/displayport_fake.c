#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "displayport_fake.h"
#include "drivers/display.h"
#include "common/utils.h"
#include "pg/vcd.h"

uint8_t osdScreen[VIDEO_LINES][CHARS_PER_LINE];

// osd:
static uint8_t osdBackBuffer[VIDEO_LINES][CHARS_PER_LINE];
static displayPort_t fakeDisplayPort;

extern unsigned int resumeRefreshAt;
static int grab(displayPort_t *displayPort) {
    UNUSED(displayPort);

#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif
    return 0;
}

static int release(displayPort_t *displayPort) {
    UNUSED(displayPort);
    return 0;
}

static int clearScreen(displayPort_t *displayPort) {
    UNUSED(displayPort);
    printf("clearScreen: %i\n", displayPort);
    memset(osdBackBuffer, 0, 16 * 30);
    return 0;
}

static int drawScreen(displayPort_t *displayPort) {
    UNUSED(displayPort);
    printf("drawScreen: %i\n", displayPort);
    memcpy(osdScreen, osdBackBuffer, 16 * 30);
    return 0;
}

static int screenSize(const displayPort_t *displayPort) {
    UNUSED(displayPort);
    return 480;
}

static int writeSys (displayPort_t *displayPort, uint8_t x, uint8_t y, displayPortSystemElement_e systemElement){
  return 0;
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y, uint8_t attr, const char *s) {
    UNUSED(displayPort);
    printf("writeString: %i\n", displayPort);
    //printf("%d, %d: %s\n", x, y, s);
    if (y < VIDEO_LINES) {
        for (int i = 0; s[i] && x + i < CHARS_PER_LINE; i++) {
            osdBackBuffer[y][x + i] = s[i];
            //printf("%d, %d: %d\n", x, y, s[i]);
        }
    }
    return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y,
                     uint8_t c) {
    UNUSED(displayPort);
    printf("writeChar: %i\n", displayPort);
    if (x < CHARS_PER_LINE && y < VIDEO_LINES) {
        osdBackBuffer[y][x] = c;
    }
    return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort) {
    UNUSED(displayPort);
    return false;
}

static bool isSynced(const displayPort_t *displayPort) {
    UNUSED(displayPort);
    return true;
}

static void resync(displayPort_t *displayPort) {
    displayPort->rows = VIDEO_LINES;
    displayPort->cols = CHARS_PER_LINE;
}

static int heartbeat(displayPort_t *displayPort) {
    UNUSED(displayPort);
    return 0;
}

static void redraw(displayPort_t *displayPort){
  printf("redraw: %i\n", displayPort);
}

static uint32_t txBytesFree(const displayPort_t *displayPort) {
    UNUSED(displayPort);
    return UINT32_MAX;
}

static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer){
  printf("layerSupported: %i, layer %i\n", displayPort, (int)layer);
  if(layer == DISPLAYPORT_LAYER_FOREGROUND){
    return true;
  }
  return false;
}
static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer){
  printf("layerSelect: %i, layer %i\n", displayPort, (int)layer);
  
  if(layer == DISPLAYPORT_LAYER_FOREGROUND){
    return true;
  }
  return false;
}
static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer, displayPortLayer_e sourceLayer){
  printf("layerCopy: %i, destLayer %i, sourceLayer %i\n", displayPort, (int)destLayer, (int)sourceLayer);
  
  return true;
}
static bool writeFontCharacter(displayPort_t *instance, uint16_t addr, const struct osdCharacter_s *chr){
  printf("writeFontCharacter: %i, addr %i, chr %i\n", addr, (int)chr);
  return true;
}
static bool checkReady(displayPort_t *displayPort, bool rescan){
  return true;
}
static void beginTransaction(displayPort_t *displayPort, displayTransactionOption_e opts){
}
static void commitTransaction(displayPort_t *displayPort){
}
static bool getCanvas(struct displayCanvas_s *canvas, const displayPort_t *displayPort){
  return true;
}
static void setBackgroundType(displayPort_t *displayPort, displayPortBackground_e backgroundType){
  printf("setBackgroundType: %i, backgroundType %i\n", displayPort, (int)backgroundType);
}

static const displayPortVTable_t fakeDispVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeSys = writeSys,
    .writeString = writeString,
    .writeChar = writeChar,
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .redraw = redraw,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree,
    .layerSupported = layerSupported,
    .layerSelect = layerSelect,
    .layerCopy = layerCopy,
    .writeFontCharacter = writeFontCharacter,
    .checkReady = checkReady,
    .beginTransaction = beginTransaction,
    .commitTransaction = commitTransaction,
    .getCanvas = getCanvas,
    .setBackgroundType = setBackgroundType
};

struct vcdProfile_s;


//displayPort_t *max7456DisplayPortInit(const struct vcdProfile_s *vcdProfile) {

bool max7456DisplayPortInit(const vcdProfile_t *vcdProfile, displayPort_t **displayPort) {
    UNUSED(vcdProfile);

    printf("display init\n");
    displayInit(&fakeDisplayPort, &fakeDispVTable, DISPLAYPORT_DEVICE_TYPE_MSP);

    resync(&fakeDisplayPort);

    *displayPort = &fakeDisplayPort;

    return true;
    //return &fakeDisplayPort;
}


int spiDeviceByInstance(void *instance)
{
    return 0;
}