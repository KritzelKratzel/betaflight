
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
 * ===============================================================================
 * Camera Serial Protocol (CSP):
 * ===============================================================================
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
 * ===============================================================================
 *  NanoCam3D Mk.1 OSD Layout and Element Positions
 * ===============================================================================
 * - Full screen with 45 columns and 20 lines (NTSC) or 23 lines (PAL)
 * - First three columns and last three columns reserved for 3D-OSD convergence 
 *   setting
 * - Limited camera FPGA resources reduce actual usable screen positions to one 
 *   line on top and one line at bottom of screen, each line with 32 character 
 *   positions
 * - Arrangement of usable OSD screen positions can be modified only by camera 
 *   firmware update
 * - OSD is set by sending ASCII strings to camera via serial interface using 
 *   Camera Serial Protocol (CSP).
 *
 * +---------------------------------------------+
 * |###      Upper OSD Section - 32 Chars     ###|
 * |###   +--------------------------------+  ###|
 * |###   |M100%       DISARMED        ACRO|  ###|
 * |###   +--------------------------------+  ###|
 * |###    |           |               |      ###|
 * |###    32          44              60     ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###                                       ###|
 * |###    0            13          25        ###|
 * |###    |            |           |         ###|
 * |###   +--------------------------------+  ###|
 * |###   |12.3V        01:30       _234mAh|  ###|
 * |###   +--------------------------------+  ###|
 * |###      Lower OSD Section - 32 Chars     ###
 * +---------------------------------------------+
 *
 *  ------------------------------------------------------------------------------
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
#include "fc/rc_modes.h"
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

#include "pg/rx.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "telemetry/telemetry.h"
#include "telemetry/nc3d.h"


// OSD elements currently defined with static positions
#define OSD3D_MAIN_BATT_VOLTAGE_POSITION  0
#define OSD3D_ARMED_CLOCK_POSITION       13
#define OSD3D_MAH_DRAWN_POSITION         25
#define OSD3D_RSSI_VALUE_POSITION        32
#define OSD3D_HEADLINE_POSITION          44
#define OSD3D_FLYMODE_POSITION           60

// further arrangements
#define SYM_RSSI 0x90
#define TELEMETRY_NC3D_INITIAL_PORT_MODE MODE_TX
#define NC3D_CYCLETIME   100
#define OSD3D_BLINK_CYCLE      500/NC3D_CYCLETIME

static serialPortConfig_t *portConfig;
static portSharing_e nc3dPortSharing;
static serialPort_t *nc3dPort;
static bool nc3dEnabled;
static bool osdBlinkMask = true;


static char* blinky(char *str){
  // make str blinky ... needed for OSD alarm elements.
  if (!osdBlinkMask){
    // delete with spaces (ASCII 0x20)
    memset(str,' ',strlen(str));
  }
  return str;  
}

static void osdDrawSingleElement(serialPort_t *nc3dPort, uint8_t item, uint16_t position)
{
  uint8_t i, crc;
  uint16_t msglen;
  char buff[OSD3D_ELEMENT_BUFFER_LENGTH] = "";
  static timeUs_t flyTime = 0;
  static timeUs_t lastTimeUs = 0;



  // Prepare data in ASCII format in buff[]
  switch (item){
  case OSD3D_MAIN_BATT_VOLTAGE:
    {
      tfp_sprintf(buff, "%2d.%1d%c", getBatteryVoltage() / 10, \
		  getBatteryVoltage() % 10, 'V');
      if (getBatteryState() != BATTERY_OK) blinky(buff);

      break;
    }
  case OSD3D_ARMED_CLOCK:
    {
      const timeUs_t currentTimeUs = micros();

      if (ARMING_FLAG(ARMED)) flyTime += currentTimeUs - lastTimeUs;
      int seconds = flyTime / 1000000;
      const int minutes = seconds / 60;
      seconds = seconds % 60;
      tfp_sprintf(buff, "%02d:%02d", minutes, seconds);
      lastTimeUs = currentTimeUs;

      break;
    }
  case OSD3D_FLYMODE:
    {
      // Note that flight mode display has precedence in what to display.
      //  1. FS
      //  2. GPS RESCUE
      //  3. ANGLE, HORIZON, ACRO TRAINER
      //  4. AIR
      //  5. ACRO
      
      if (FLIGHT_MODE(FAILSAFE_MODE)) {
	strcpy(buff, "!FS!");
      } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
	strcpy(buff, "RESC");
      } else if (FLIGHT_MODE(HEADFREE_MODE)) {
	strcpy(buff, "HEAD");
      } else if (FLIGHT_MODE(ANGLE_MODE)) {
	strcpy(buff, "STAB");
      } else if (FLIGHT_MODE(HORIZON_MODE)) {
	strcpy(buff, "HORZ");
      } else if (IS_RC_MODE_ACTIVE(BOXACROTRAINER)) {
	strcpy(buff, "ATRN");
      } else if (airmodeIsEnabled()) {
	strcpy(buff, " AIR");
      } else {
	strcpy(buff, "ACRO");
      }

      break;
    }
  case OSD3D_HEADLINE:
    {

      if (!ARMING_FLAG(ARMED))
	{
	  blinky(strcpy(buff,"DISARMED"));
	}
      else
	{
	  memset(buff,' ',strlen(buff));
	}
      
      break;
    }
  case OSD3D_MAH_DRAWN:
    {
      tfp_sprintf(buff, "%4d%s", getMAhDrawn(), "mAh");

      break;
    }
  case OSD3D_RSSI_VALUE:
    {
      uint16_t osdRssi = getRssi() * 100 / 1024; // change range
      if (osdRssi >= 100) {
	osdRssi = 100;
      }
      tfp_sprintf(buff, "%c%3d%c", SYM_RSSI, osdRssi, '%');

      break;
    }
  default:
    {
      break;
    }
  }

  // Send Data in buff to serial port with header and footer.
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
  // Create an OSD blink mask simply by counting the number of calls of
  // this function.
  static uint8_t call_cnt = 0;
  if (call_cnt > OSD3D_BLINK_CYCLE-1)
    {
      call_cnt = 0;
      osdBlinkMask = !osdBlinkMask;
    }
  else
    call_cnt++;

  // draw OSD elements
  osdDrawSingleElement(nc3dPort,\
		       OSD3D_MAIN_BATT_VOLTAGE, OSD3D_MAIN_BATT_VOLTAGE_POSITION);
  osdDrawSingleElement(nc3dPort, OSD3D_ARMED_CLOCK, OSD3D_ARMED_CLOCK_POSITION);
  osdDrawSingleElement(nc3dPort, OSD3D_FLYMODE, OSD3D_FLYMODE_POSITION);
  osdDrawSingleElement(nc3dPort, OSD3D_HEADLINE, OSD3D_HEADLINE_POSITION);
  osdDrawSingleElement(nc3dPort, OSD3D_MAH_DRAWN, OSD3D_MAH_DRAWN_POSITION);
  osdDrawSingleElement(nc3dPort, OSD3D_RSSI_VALUE, OSD3D_RSSI_VALUE_POSITION);
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
