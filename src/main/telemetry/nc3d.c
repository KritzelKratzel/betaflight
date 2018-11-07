
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * -------------------------------------------------------------------------------
 * Camera Serial Protocol (CSP):
 * -------------------------------------------------------------------------------
 *  <Preamble><MsgType><MsgLen><Payload><CRC>
 *  
 *  <Preamble> = "$A" (2 Bytes)
 *  <MsgType>  = "D" for Data, "C" for Device Configuration" (1 Byte)
 *  <MsgLen>   = uint16 (2 Bytes, LSB first) length of payload
 *  <Payload>  = MsgLen x uint8 data
 *  <CRC>      = uint8 CRC (over <MsgLen> and <Payload>, XOR with every successive
 *               byte)
 *
 *  Payload format:
 *  ---------------
 *  <Payload>  = screen position (uint16, LSB first) + ASCII data to be displayed
 *
 *  Example:
 *  ------------------------------------------------------------------------------
 *  The following serial message will write the string "12.3V" starting at screen 
 *  position 7 of the utilized camera. (Spaces only for readibility, direct ASCII 
 *  when appropriate)
 *
 *  $ A D 0x07 0x00 0x04 0x00 1 2 . 3 V 0x4c
 *  ^   ^ ^         ^         ^         ^
 *  |   | |         |         |         +- CRC = 0x04 xor 0x00 xor 0x31 xor 0x32
 *  |   | |         |         |                  xor 0x2e xor 0x33 xor 0x56
 *  |   | |         |         +----------- OSD-Data. Here: "12.3V"
 *  |   | |         +--------------------- Screen position 4. Lower byte first.
 *  |   | +------------------------------- MsgLen of 5+2=7 bytes. Lower byte first
 *  |   +--------------------------------- MsgType is Data. 
 *  +------------------------------------- Preamble.
 *
 *  Actual message: 
 *  0x24 0x41 0x44 0x07 0x00 0x04 0x00 0x31 0x32 0x2E 0x33 0x56 0x4C
 *
 *  Possible screen positions depend on camera firmware. Typically only portions
 *  of the full screen can be addressed. Incoming message is NOT acknowledged by 
 *  camera, there is no Tx data from camera to uC.
 *
 *  Camera specific restrictions:
 *  -----------------------------
 *  1. NanoCam3D Mk.1:
 *      * Only MsgType = D implemented,
 *      * Only 64 screen positions available for data display,
 *      * Fixed Baud rate of 250_000 Bd on serial interface.
 *      
 *  End of List.
 *  ------------------------------------------------------------------------------
 *
 * Personal remarks:
 *
 * make: add nc3d entry in make/source.mk
 *
 * USE_TELEMETRY
 *  USE_TELEMETRY_LTM
 *  USE_TELEMETRY_NC3D
 *  ... principle #define in betaflight/src/main/target/common_pre.h and
 *  ... principle #undef in betaflight/src/main/target/common_post.h
 *
 *  #undef USE_TELEMETRY_NC3D in parallel to #undef USE_TELEMETRY_LTM in:
 *  src/main/target/FRSKYF3/target.h
 *  src/main/target/RACEBASE/target.h
 *  src/main/target/MIDELICF3/target.h
 *  src/main/target/BETAFLIGHTF3/target.h <--- Might change...
 *  src/main/target/IMPULSERCF3/target.h
 *  src/main/target/SPRACINGF3EVO/target.h
 *  src/main/target/SPRACINGF3MINI/target.h
 *  src/main/target/SPRACINGF3NEO/target.h
 *  src/main/target/FURYF3/target.h
 *  src/main/target/SIRINFPV/target.h
 *  src/main/target/SITL/target.h
 *  src/main/target/CRAZYBEEF3FR/target.h
 *  src/main/target/OMNIBUS/target.h
 *  src/main/target/STM32F3DISCOVERY/target.h
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_TELEMETRY_NC3D

#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/printf.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/beeper.h"
//#include "io/osd.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "telemetry/telemetry.h"
#include "telemetry/nc3d.h"

/*
 *            OSD Layout
 * ==================================
 *    Upper OSD Section - 32 Chars
 * +--------------------------------+
 * |M100%       DISARMED        ACRO|
 * +--------------------------------+
 *  ^           ^               ^
 *  |           |               |
 *  32          44              60
 * 
 *    Lower OSD Section - 32 Chars
 * +--------------------------------+
 * |12.3V        01:30       _234mAh|
 * +--------------------------------+
 *  ^            ^           ^
 *  |            |           |
 *  0            13          25
 */
#define OSD3D_MAIN_BATT_VOLTAGE_POSITION  0
#define OSD3D_ARMED_CLOCK_POSITION       13
#define OSD3D_MAH_DRAWN_POSITION         25
#define OSD3D_RSSI_VALUE_POSITION        32
#define OSD3D_HEADLINE_POSITION          44
#define OSD3D_FLYMODE_MAJOR_POSITION     60




#define TELEMETRY_NC3D_INITIAL_PORT_MODE MODE_TX
#define NC3D_CYCLETIME   100

static serialPortConfig_t *portConfig;
static portSharing_e nc3dPortSharing;
static serialPort_t *nc3dPort;
static bool nc3dEnabled;
//static uint8_t nc3d_crc;



// static void osdDrawSingleElement(serialPort_t *nc3dPort, uint8_t item, uint8_t position)
static void osdDrawSingleElement(serialPort_t *nc3dPort, uint8_t item, uint16_t position)
{
  // FIXME
  //serialWrite(nc3dPort, '$');
  //serialWrite(nc3dPort, 'A');
  //serialWrite(nc3dPort, 'D');

  uint8_t i, crc;
  uint16_t msglen;
  char buff[OSD3D_ELEMENT_BUFFER_LENGTH] = "";

/* const int cellV = osdGetBatteryAverageCellVoltage(); */
/*             buff[0] = osdGetBatterySymbol(cellV); */
/*             tfp_sprintf(buff + 1, "%d.%02d%c", cellV / 100, cellV % 100, SYM_VOLT); */
/* break; */


  // Prepare data in ASCII format in buff[]
  switch (item){
  case OSD3D_MAIN_BATT_VOLTAGE:
    {

      //serialWrite(nc3dPort, '$');
      //serialWrite(nc3dPort, 'A');
      //serialWrite(nc3dPort, 'D');
      tfp_sprintf(buff, "%2d.%1d%c", getBatteryVoltage() / 10, getBatteryVoltage() % 10, 'V');
      //serialPrint(nc3dPort,buff);
      //serialWrite(nc3dPort, buff);

      break;
    }
    
  default:
    {
      break;
    }
  }

  // Send Data to serial port with header and footer.
  // Bug in NanoCam3D Mk.1 camera firmware: strlen(buff) MUST NOT be zero.

  // send header
  serialPrint(nc3dPort, "$AD");

  // send msglen; must include uint16_t msglen
  msglen=strlen(buff)+2;
  serialWrite(nc3dPort, (uint8_t) msglen);
  serialWrite(nc3dPort, (msglen >> 8));

  // send position, crc calculation starts from here
  crc = position;
  serialWrite(nc3dPort, (uint8_t) position);
  crc ^= (position >> 8);
  serialWrite(nc3dPort, (position >> 8));

  // send payload
  for (i=0; i<strlen(buff); i++){
    crc ^= buff[i];
    serialWrite(nc3dPort, buff[i]);
  }

  // send crc
  serialWrite(nc3dPort, crc);

}

static void process_nc3d(void)
{


  osdDrawSingleElement(nc3dPort,\
		       OSD3D_MAIN_BATT_VOLTAGE, OSD3D_MAIN_BATT_VOLTAGE_POSITION);

}

/*****************************************************************************/
/* initNc3dTelemetry(), checkNc3dTelemetryState() and handleNc3dTelemetry()  */
/* are called in src/main/telemetry/telemetry.c				     */
/*****************************************************************************/

void initNc3dTelemetry(void)
{
  portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_NC3D);
  nc3dPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_NC3D);
}

void checkNc3dTelemetryState(void)
{
  if (portConfig && telemetryCheckRxPortShared(portConfig)) {
    if (!nc3dEnabled && telemetrySharedPort != NULL) {
      nc3dPort = telemetrySharedPort;
      nc3dEnabled = true;
    }
  } else {
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(nc3dPortSharing);
    if (newTelemetryEnabledValue == nc3dEnabled)
      return;
    if (newTelemetryEnabledValue)
      configureNc3dTelemetryPort();
    else
      freeNc3dTelemetryPort();
  }
}

void handleNc3dTelemetry(void)
{
  static uint32_t nc3d_lastCycleTime;
  uint32_t now;
  if (!nc3dEnabled)
    return;
  if (!nc3dPort)
    return;
  now = millis();
  if ((now - nc3d_lastCycleTime) >= NC3D_CYCLETIME) {
    process_nc3d();
    nc3d_lastCycleTime = now;
  }
}

void configureNc3dTelemetryPort(void)
{
  if (!portConfig) {
    return;
  }
  baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
  if (baudRateIndex == BAUD_AUTO) {
    baudRateIndex = BAUD_115200; 
  }
  
  nc3dPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_NC3D, \
			    NULL, NULL, baudRates[baudRateIndex], \
			    TELEMETRY_NC3D_INITIAL_PORT_MODE, \
			    telemetryConfig()->telemetry_inverted \
			    ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
  if (!nc3dPort)
    return;
  nc3dEnabled = true;
}

void freeNc3dTelemetryPort(void)
{
  closeSerialPort(nc3dPort);
  nc3dPort = NULL;
  nc3dEnabled = false;
}

#endif
