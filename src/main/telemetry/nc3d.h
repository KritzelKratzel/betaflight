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

#pragma once

void initNc3dTelemetry(void);
void checkNc3dTelemetryState(void);
void handleNc3dTelemetry(void);

void freeNc3dTelemetryPort(void);
void configureNc3dTelemetryPort(void);

typedef enum {
    OSD3D_MAIN_BATT_VOLTAGE,
    OSD3D_ARMED_CLOCK,
    OSD3D_FLYMODE,
    OSD3D_HEADLINE,
    OSD3D_MAH_DRAWN,
    OSD3D_RSSI_VALUE,
} osd_items_e;

#define OSD3D_ELEMENT_BUFFER_LENGTH 32
