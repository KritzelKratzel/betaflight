/*
 * This file is part of Cleanflight, Betaflight and INAV
 *
 * Cleanflight, Betaflight and INAV are free software. You can
 * redistribute this software and/or modify this software under
 * the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Cleanflight, Betaflight and INAV are distributed in the hope that
 * they will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_TMGOSD)

#include "common/utils.h"

#include "drivers/display.h"

#include "io/displayport_tmg_osd.h"
#include "io/tmg_osd.h"

static displayPort_t tmgOsdDisplayPort;

static int grab(displayPort_t *displayPort)
{
  // Currently not supported by TMGOSD
  UNUSED(displayPort);
  return 0;
}

static int release(displayPort_t *displayPort)
{
  // Currently not supported by TMGOSD
  UNUSED(displayPort);
  return 0;
}

static int clearScreen(displayPort_t *displayPort)
{
  // UNUSED(displayPort);
  tmgOsdClearScreen(displayPort->device); 
  return 0;
}

static int drawScreen(displayPort_t *displayPort)
{
  UNUSED(displayPort);
  // FIXME
  // frskyOsdUpdate();
  return 0;
}

static int screenSize(const displayPort_t *displayPort)
{
  UNUSED(displayPort);
  // FIXME
  // return frskyOsdGetGridRows() * frskyOsdGetGridCols();
  return 900;
}

static int writeString(displayPort_t *displayPort, uint8_t x, uint8_t y,
		       uint8_t attr, const char *s)
{
  UNUSED(displayPort);
  UNUSED(attr);
  // FIXME
  UNUSED(x);
  UNUSED(y);
  UNUSED(s);
  // frskyOsdDrawStringInGrid(x, y, s);
  return 0;
}

static int writeChar(displayPort_t *displayPort, uint8_t x, uint8_t y,
		     uint8_t attr, uint8_t c)
{
  UNUSED(displayPort);
  UNUSED(attr);
  // FIXME
  UNUSED(x);
  UNUSED(y);
  UNUSED(c);
  // frskyOsdDrawCharInGrid(x, y, c);
  return 0;
}

static bool isTransferInProgress(const displayPort_t *displayPort)
{
  // Currently unused within TMGOSD
  UNUSED(displayPort);
  return false;
}

static int heartbeat(displayPort_t *displayPort)
{
  // Currently unused within TMGOSD
  UNUSED(displayPort);
  return 0;
}

static void redraw(displayPort_t *displayPort)
{
    UNUSED(displayPort);
    // FIXME
    // frskyOsdUpdate();
}

static bool isSynced(const displayPort_t *displayPort)
{
  // Currently unused within TMGOSD
  // Apparently not used anywhere
  UNUSED(displayPort);
  return true;
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
  // Currently not further implemented. Just return UINT32_MAX
  // value defined in stdint.h
  UNUSED(displayPort);
  return UINT32_MAX;
}

static bool layerSupported(displayPort_t *displayPort, displayPortLayer_e layer)
{
  UNUSED(displayPort);
  // TMGOSD currently supports only forground layer
  return (layer == DISPLAYPORT_LAYER_FOREGROUND);
}

static bool layerSelect(displayPort_t *displayPort, displayPortLayer_e layer)
{
  UNUSED(displayPort);
  // TMGOSD currently supports only forground layer
  return (layer == DISPLAYPORT_LAYER_FOREGROUND);
}

static bool layerCopy(displayPort_t *displayPort, displayPortLayer_e destLayer,
		      displayPortLayer_e sourceLayer)
{
  UNUSED(displayPort);
  UNUSED(destLayer);
  UNUSED(sourceLayer);
  // TMGOSD currently supports only forground layer
  return false;
}

static bool writeFontCharacter(displayPort_t *displayPort, uint16_t addr,
			       const osdCharacter_t *chr)
{
  UNUSED(displayPort);
  UNUSED(addr);
  UNUSED(chr);
  // OSD font update to NVM urrently not supported by TMGOSD.
  return false;
}

static bool checkReady(displayPort_t *instance, bool rescan)
{
  UNUSED(rescan);
  // FIXME
  /* if (frskyOsdIsReady()) { */
  /*   updateGridSize(instance); */
  /*   return true; */
  /* } */
  /* return false; */
  UNUSED(instance); // preliminary
  return true; // preliminary
}

// displayPortVTable_t declared in src/main/drivers/display.h
static const displayPortVTable_t tmgOsdVTable = {
  .grab = grab,
  .release = release,
  .clearScreen = clearScreen,
  .drawScreen = drawScreen,
  .screenSize = screenSize,
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
  .beginTransaction = NULL,  // not available for character based TMGOSD
  .commitTransaction = NULL, // not available for character based TMGOSD
  .getCanvas = NULL,         // not available for character based TMGOSD
};

displayPort_t *tmgOsdDisplayPortInit(void)
{
  const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_TMG_OSD);
  if (portConfig) {
    portOptions_e portOptions = 0; // see drivers/serial.h
    serialPort_t *port = openSerialPort(portConfig->identifier,
					FUNCTION_TMG_OSD,
					NULL, NULL,
					TMG_OSD_BAUDRATE,
					MODE_RXTX,
					portOptions);
    if (port) {
      tmgOsdDisplayPort.device = port;
      displayInit(&tmgOsdDisplayPort, &tmgOsdVTable); // launches clearScreen()
      tmgOsdDisplayPort.rows = TMG_OSD_NUM_YCHARS;
      tmgOsdDisplayPort.cols = TMG_OSD_NUM_XCHARS;
      tmgOsdDisplayPort.useDeviceBlink = false;
      return &tmgOsdDisplayPort;
    }
  }
  return NULL;
}

#endif // USE_TMGOSD
