/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/accgyro.h"
#include "drivers/light_ledring.h"
#include "drivers/light_led.h"
#include "drivers/light_ws2811strip.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/serial.h"

#include "flight/flight.h"
#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "io/buzzer.h"
#include "io/escservo.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/autotune.h"
#include "flight/mixer.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/serial_cli.h"
#include "io/serial.h"
#include "io/statusindicator.h"
#include "rx/rx.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

// June 2013     V2.2-dev

enum {
    ALIGN_GYRO = 0,
    ALIGN_ACCEL = 1,
    ALIGN_MAG = 2
};

/* for VBAT monitoring frequency */
#define VBATFREQ 6        // to read battery voltage - nth number of loop iterations

int16_t debug[4];
uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t headFreeModeHold;

int16_t telemTemperature1;      // gyro sensor temperature

extern uint8_t dynP8[3], dynI8[3], dynD8[3];
extern failsafe_t *failsafe;

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination, rollAndPitchTrims_t *accelerometerTrims);

extern pidControllerFuncPtr pid_controller;

void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    currentProfile.accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    currentProfile.accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;

    saveAndReloadCurrentProfileToCurrentProfileSlot();
}

void updateAutotuneState(void)
{
    static bool landedAfterAutoTuning = false;
    static bool autoTuneWasUsed = false;

    if (rcOptions[BOXAUTOTUNE]) {
        if (!f.AUTOTUNE_MODE) {
            if (f.ARMED) {
                if (isAutotuneIdle() || landedAfterAutoTuning) {
                    autotuneReset();
                    landedAfterAutoTuning = false;
                }
                autotuneBeginNextPhase(&currentProfile.pidProfile, currentProfile.pidController);
                f.AUTOTUNE_MODE = 1;
                autoTuneWasUsed = true;
            } else {
                if (havePidsBeenUpdatedByAutotune()) {
                    saveAndReloadCurrentProfileToCurrentProfileSlot();
                    autotuneReset();
                }
            }
        }
        return;
    }

    if (f.AUTOTUNE_MODE) {
        autotuneEndPhase();
        f.AUTOTUNE_MODE = 0;
    }

    if (!f.ARMED && autoTuneWasUsed) {
        landedAfterAutoTuning = true;
    }
}

bool isCalibrating()
{
#ifdef BARO
    if (sensors(SENSOR_ACC) && !isBaroCalibrationComplete()) {
        return false;
    }
#endif

    // Note: compass calibration is handled completely differently, outside of the main loop, see f.CALIBRATE_MAG

    return (!isAccelerationCalibrationComplete() && sensors(SENSOR_ACC)) || (!isGyroCalibrationComplete());
}

void annexCode(void)
{
    int32_t tmp, tmp2;
    int32_t axis, prop1, prop2;

    static uint8_t batteryWarningEnabled = false;
    static uint8_t vbatTimer = 0;
    static int32_t vbatCycleTime = 0;

    // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < currentProfile.tpa_breakpoint) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t)currentProfile.dynThrPID * (rcData[THROTTLE] - currentProfile.tpa_breakpoint) / (2000 - currentProfile.tpa_breakpoint);
        } else {
            prop2 = 100 - currentProfile.dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        tmp = min(abs(rcData[axis] - masterConfig.rxConfig.midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (currentProfile.deadband) {
                if (tmp > currentProfile.deadband) {
                    tmp -= currentProfile.deadband;
                } else {
                    tmp = 0;
                }
            }

            tmp2 = tmp / 100;
            rcCommand[axis] = lookupPitchRollRC[tmp2] + (tmp - tmp2 * 100) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) / 100;
            prop1 = 100 - (uint16_t)currentProfile.controlRateConfig.rollPitchRate * tmp / 500;
            prop1 = (uint16_t)prop1 * prop2 / 100;
        }
        if (axis == YAW) {
            if (currentProfile.yaw_deadband) {
                if (tmp > currentProfile.yaw_deadband) {
                    tmp -= currentProfile.yaw_deadband;
                } else {
                    tmp = 0;
                }
            }
            rcCommand[axis] = tmp * -masterConfig.yaw_control_direction;
            prop1 = 100 - (uint16_t)currentProfile.controlRateConfig.yawRate * abs(tmp) / 500;
        }
        // FIXME axis indexes into pids.  use something like lookupPidIndex(rc_alias_e alias) to reduce coupling.
        dynP8[axis] = (uint16_t)currentProfile.pidProfile.P8[axis] * prop1 / 100;
        dynI8[axis] = (uint16_t)currentProfile.pidProfile.I8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t)currentProfile.pidProfile.D8[axis] * prop1 / 100;

        if (rcData[axis] < masterConfig.rxConfig.midrc)
            rcCommand[axis] = -rcCommand[axis];
    }

    tmp = constrain(rcData[THROTTLE], masterConfig.rxConfig.mincheck, PWM_RANGE_MAX);
    tmp = (uint32_t)(tmp - masterConfig.rxConfig.mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - masterConfig.rxConfig.mincheck);       // [MINCHECK;2000] -> [0;1000]
    tmp2 = tmp / 100;
    rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]

    if (f.HEADFREE_MODE) {
        float radDiff = degreesToRadians(heading - headFreeModeHold);
        float cosDiff = cosf(radDiff);
        float sinDiff = sinf(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER)) {
        vbatCycleTime += cycleTime;
        if (!(++vbatTimer % VBATFREQ)) {

        	if (feature(FEATURE_VBAT)) {
        		updateBatteryVoltage();
                batteryWarningEnabled = shouldSoundBatteryAlarm();
        	}

        	if (feature(FEATURE_CURRENT_METER)) {
        		updateCurrentMeter(vbatCycleTime);
        	}
        }
    }

    buzzer(batteryWarningEnabled); // external buzzer routine that handles buzzer events globally now

    if (f.ARMED) {
        LED0_ON;
    } else {
        if (isCalibrating()) {
            LED0_TOGGLE;
            f.OK_TO_ARM = 0;
        }

        f.OK_TO_ARM = 1;

        if (!f.SMALL_ANGLE) {
            f.OK_TO_ARM = 0;
        }

        if (rcOptions[BOXAUTOTUNE]) {
            f.OK_TO_ARM = 0;
        }

        if (f.OK_TO_ARM) {
            disableWarningLed();
        } else {
            enableWarningLed(currentTime);
        }

        updateWarningLed(currentTime);
    }

    checkTelemetryState();

#ifdef LEDRING
    if (feature(FEATURE_LED_RING)) {
        static uint32_t LEDTime;
        if ((int32_t)(currentTime - LEDTime) >= 0) {
            LEDTime = currentTime + 50000;
            ledringState(f.ARMED, inclination.values.pitchDeciDegrees, inclination.values.rollDeciDegrees, heading);
        }
    }
#endif


    handleSerial();

    if (sensors(SENSOR_GPS)) {
        updateGpsIndicator(currentTime);
    }

    // Read out gyro temperature. can use it for something somewhere. maybe get MCU temperature instead? lots of fun possibilities.
    if (gyro.temperature)
        gyro.temperature(&telemTemperature1);
}

void mwDisarm(void)
{
    if (f.ARMED)
        f.ARMED = 0;
}

void mwArm(void)
{
    if (f.OK_TO_ARM) {
        if (f.ARMED) {
            return;
        }
        if (!f.PREVENT_ARMING) {
            f.ARMED = 1;
            headFreeModeHold = heading;
            return;
        }
    }

    if (!f.ARMED) {
        blinkLedAndSoundBeeper(2, 255, 1);
    }
}

// Automatic ACC Offset Calibration
bool AccInflightCalibrationArmed = false;
bool AccInflightCalibrationMeasurementDone = false;
bool AccInflightCalibrationSavetoEEProm = false;
bool AccInflightCalibrationActive = false;
uint16_t InflightcalibratingA = 0;

void handleInflightCalibrationStickPosition(void)
{
    if (AccInflightCalibrationMeasurementDone) {
        // trigger saving into eeprom after landing
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    } else {
        AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
        if (AccInflightCalibrationArmed) {
            queueConfirmationBeep(2);
        } else {
            queueConfirmationBeep(3);
        }
    }
}

void updateInflightCalibrationState(void)
{
    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > masterConfig.rxConfig.mincheck && !rcOptions[BOXARM]) {   // Copter is airborne and you are turning it off via boxarm : start measurement
        InflightcalibratingA = 50;
        AccInflightCalibrationArmed = false;
    }
    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
        if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone)
            InflightcalibratingA = 50;
        AccInflightCalibrationActive = true;
    } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
        AccInflightCalibrationMeasurementDone = false;
        AccInflightCalibrationSavetoEEProm = true;
    }
}

static const uint8_t colors[][3] =
{
    {255, 0,   0},
    {0,   255, 0},
    {0,   0,   255},
    {255, 0,   255},

    {255, 255, 255},
    {255, 255, 255},

    {255, 255, 0},
    {0,   0,   255},
    {0,   255, 0},
    {255, 0,   0}
};


static const uint8_t stripOff[][3] =
{
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
    {0,   0,   0},
};

static const uint8_t stripRed[][3] =
{
    {32,   0,   0},
    {96,   0,   0},
    {160,   0,   0},
    {224,   0,   0},
    {255,   0,   0},
    {255,   0,   0},
    {224,   0,   0},
    {160,   0,   0},
    {96,   0,   0},
    {32,   0,   0},
};

uint32_t lastStripUpdateAt = 0;

#define LED_STRIP_50HZ ((1000 * 1000) / 50)

void updateLedStrip(void)
{
    uint32_t now = micros();

    if (now - lastStripUpdateAt < LED_STRIP_50HZ) {
        return;
    }

    lastStripUpdateAt = now;

    static uint8_t stripState = 0;

    if (stripState == 0) {
        if (f.ARMED) {
            ws2812SetStripColors(stripRed, 10);
        } else {
            ws2812SetStripColors(colors, 10);
        }
        stripState = 1;
    } else {
        ws2812SetStripColors(stripOff, 10);
        stripState = 0;
    }
}



void loop(void)
{
    int i;
#ifdef BARO
    static int16_t initialThrottleHold;
#endif
    static uint32_t loopTime;
    uint32_t auxState = 0;

    updateRx();

    if (shouldProcessRx(currentTime)) {
        calculateRxChannelsAndUpdateFailsafe(currentTime);

        // in 3D mode, we need to be able to disarm by switch at any time
        if (feature(FEATURE_3D)) {
            if (!rcOptions[BOXARM])
                mwDisarm();
        }

        updateRSSI(currentTime);

        if (feature(FEATURE_FAILSAFE)) {

            if (currentTime > FAILSAFE_POWER_ON_DELAY_US && !failsafe->vTable->isEnabled()) {
                failsafe->vTable->enable();
            }

            failsafe->vTable->updateState();
        }

        throttleStatus_e throttleStatus = calculateThrottleStatus(&masterConfig.rxConfig, masterConfig.flight3DConfig.deadband3d_throttle);

        if (throttleStatus == THROTTLE_LOW) {
            resetErrorAngle();
            resetErrorGyro();
        }

        processRcStickPositions(&masterConfig.rxConfig, throttleStatus, currentProfile.activate, masterConfig.retarded_arm);

        if (feature(FEATURE_INFLIGHT_ACC_CAL)) {
            updateInflightCalibrationState();
        }

        // Check AUX switches

        // auxState is a bitmask, 3 bits per channel. aux1 is first.
        // lower 16 bits contain aux 1 to 4, upper 16 bits contain aux 5 to 8
        //
        // the three bits are as follows:
        // bit 1 is SET when the stick is less than 1300
        // bit 2 is SET when the stick is between 1300 and 1700
        // bit 3 is SET when the stick is above 1700
        // if the value is 1300 or 1700 NONE of the three bits are set.

        for (i = 0; i < 4; i++) {
            auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) |
                    (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) |
                    (rcData[AUX1 + i] > 1700) << (3 * i + 2);
            auxState |= ((rcData[AUX5 + i] < 1300) << (3 * i) |
                    (1300 < rcData[AUX5 + i] && rcData[AUX5 + i] < 1700) << (3 * i + 1) |
                    (rcData[AUX5 + i] > 1700) << (3 * i + 2)) << 16;
        }
        for (i = 0; i < CHECKBOX_ITEM_COUNT; i++)
            rcOptions[i] = (auxState & currentProfile.activate[i]) > 0;

        if ((rcOptions[BOXANGLE] || (feature(FEATURE_FAILSAFE) && failsafe->vTable->hasTimerElapsed())) && (sensors(SENSOR_ACC))) {
            // bumpless transfer to Level mode
            if (!f.ANGLE_MODE) {
                resetErrorAngle();
                f.ANGLE_MODE = 1;
            }
        } else {
            f.ANGLE_MODE = 0; // failsafe support
        }

        if (rcOptions[BOXHORIZON]) {
            f.ANGLE_MODE = 0;
            if (!f.HORIZON_MODE) {
                resetErrorAngle();
                f.HORIZON_MODE = 1;
            }
        } else {
            f.HORIZON_MODE = 0;
        }

        if (f.ANGLE_MODE || f.HORIZON_MODE) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            // Baro alt hold activate
            if (rcOptions[BOXBARO]) {
                if (!f.BARO_MODE) {
                    f.BARO_MODE = 1;
                    AltHold = EstAlt;
                    initialThrottleHold = rcCommand[THROTTLE];
                    errorAltitudeI = 0;
                    BaroPID = 0;
                }
            } else {
                f.BARO_MODE = 0;
            }
        }
#endif

#ifdef  MAG
        if (sensors(SENSOR_ACC) || sensors(SENSOR_MAG)) {
            if (rcOptions[BOXMAG]) {
                if (!f.MAG_MODE) {
                    f.MAG_MODE = 1;
                    magHold = heading;
                }
            } else {
                f.MAG_MODE = 0;
            }
            if (rcOptions[BOXHEADFREE]) {
                if (!f.HEADFREE_MODE) {
                    f.HEADFREE_MODE = 1;
                }
            } else {
                f.HEADFREE_MODE = 0;
            }
            if (rcOptions[BOXHEADADJ]) {
                headFreeModeHold = heading; // acquire new heading
            }
        }
#endif

        if (sensors(SENSOR_GPS)) {
            updateGpsWaypointsAndMode();
        }

        if (rcOptions[BOXPASSTHRU]) {
            f.PASSTHRU_MODE = 1;
        } else {
            f.PASSTHRU_MODE = 0;
        }

        if (masterConfig.mixerConfiguration == MULTITYPE_FLYING_WING || masterConfig.mixerConfiguration == MULTITYPE_AIRPLANE) {
            f.HEADFREE_MODE = 0;
        }
    } else {                    // not in rc loop
        static int taskOrder = 0;    // never call all function in the same loop, to avoid high delay spikes
        switch (taskOrder) {
        case 0:
            taskOrder++;
#ifdef MAG
            if (sensors(SENSOR_MAG) && compassGetADC(&masterConfig.magZero))
                break;
#endif
        case 1:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && baroUpdate(currentTime) != BAROMETER_ACTION_NOT_READY)
                break;
#endif
        case 2:
            taskOrder++;
#ifdef BARO
            if (sensors(SENSOR_BARO) && getEstimatedAltitude())
                break;
#endif
        case 3:
            // if GPS feature is enabled, gpsThread() will be called at some intervals to check for stuck
            // hardware, wrong baud rates, init GPS if needed, etc. Don't use SENSOR_GPS here as gpsThread() can and will
            // change this based on available hardware
            taskOrder++;
            if (feature(FEATURE_GPS)) {
                gpsThread();
                break;
            }
        case 4:
            taskOrder = 0;
#ifdef SONAR
            if (sensors(SENSOR_SONAR)) {
                Sonar_update();
            }
#endif
        }
    }

    currentTime = micros();
    if (masterConfig.looptime == 0 || (int32_t)(currentTime - loopTime) >= 0) {
        loopTime = currentTime + masterConfig.looptime;

        computeIMU(&currentProfile.accelerometerTrims, masterConfig.mixerConfiguration);

        // Measure loop rate just after reading the sensors
        currentTime = micros();
        cycleTime = (int32_t)(currentTime - previousTime);
        previousTime = currentTime;

        annexCode();

        updateAutotuneState();

#ifdef MAG
        if (sensors(SENSOR_MAG)) {
            if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
                int16_t dif = heading - magHold;
                if (dif <= -180)
                    dif += 360;
                if (dif >= +180)
                    dif -= 360;
                dif *= -masterConfig.yaw_control_direction;
                if (f.SMALL_ANGLE)
                    rcCommand[YAW] -= dif * currentProfile.pidProfile.P8[PIDMAG] / 30;    // 18 deg
            } else
                magHold = heading;
        }
#endif

#ifdef BARO
        if (sensors(SENSOR_BARO)) {
            if (f.BARO_MODE) {
                static uint8_t isAltHoldChanged = 0;
                static int16_t AltHoldCorr = 0;
                if (!f.FIXED_WING) {
                    // multirotor alt hold
                    if (currentProfile.alt_hold_fast_change) {
                        // rapid alt changes
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > currentProfile.alt_hold_throttle_neutral) {
                            errorAltitudeI = 0;
                            isAltHoldChanged = 1;
                            rcCommand[THROTTLE] += (rcCommand[THROTTLE] > initialThrottleHold) ? -currentProfile.alt_hold_throttle_neutral : currentProfile.alt_hold_throttle_neutral;
                        } else {
                            if (isAltHoldChanged) {
                                AltHold = EstAlt;
                                isAltHoldChanged = 0;
                            }
                            rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroPID, masterConfig.escAndServoConfig.minthrottle + 100, masterConfig.escAndServoConfig.maxthrottle);
                        }
                    } else {
                        // slow alt changes for apfags
                        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > currentProfile.alt_hold_throttle_neutral) {
                            // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
                            AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
                            AltHold += AltHoldCorr / 2000;
                            AltHoldCorr %= 2000;
                            isAltHoldChanged = 1;
                        } else if (isAltHoldChanged) {
                            AltHold = EstAlt;
                            AltHoldCorr = 0;
                            isAltHoldChanged = 0;
                        }
                        rcCommand[THROTTLE] = constrain(initialThrottleHold + BaroPID, masterConfig.escAndServoConfig.minthrottle + 100, masterConfig.escAndServoConfig.maxthrottle);
                    }
                } else {
                    // handle fixedwing-related althold. UNTESTED! and probably wrong
                    // most likely need to check changes on pitch channel and 'reset' althold similar to
                    // how throttle does it on multirotor
                    rcCommand[PITCH] += BaroPID * masterConfig.fixedwing_althold_dir;
                }
            }
        }
#endif

        if (currentProfile.throttle_correction_value && (f.ANGLE_MODE || f.HORIZON_MODE)) {
            rcCommand[THROTTLE] += calculateThrottleAngleCorrection(currentProfile.throttle_correction_value);
        }

        if (sensors(SENSOR_GPS)) {
            if ((f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME) {
                updateGpsStateForHomeAndHoldMode();
            }
        }

        // PID - note this is function pointer set by setPIDController()
        pid_controller(&currentProfile.pidProfile, &currentProfile.controlRateConfig, masterConfig.max_angle_inclination, &currentProfile.accelerometerTrims);

        mixTable();
        writeServos();
        writeMotors();
    }

    if (!cliMode && feature(FEATURE_TELEMETRY)) {
        handleTelemetry();
    }
    updateLedStrip();
}
