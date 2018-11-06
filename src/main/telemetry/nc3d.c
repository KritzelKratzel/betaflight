
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

#include "pg/rx.h"

#include "rx/rx.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"

#include "telemetry/telemetry.h"
#include "telemetry/nc3d.h"


#define TELEMETRY_NC3D_INITIAL_PORT_MODE MODE_TX
#define NC3D_CYCLETIME   100

static serialPortConfig_t *portConfig;
static portSharing_e nc3dPortSharing;
static serialPort_t *nc3dPort;
static bool nc3dEnabled;
//static uint8_t nc3d_crc;



static void process_nc3d(void){
    // FIXME
    serialWrite(nc3dPort, '$');
    serialWrite(nc3dPort, 'A');
    serialWrite(nc3dPort, 'D');
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

    nc3dPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_NC3D, NULL, NULL, baudRates[baudRateIndex], TELEMETRY_NC3D_INITIAL_PORT_MODE, telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED);
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
