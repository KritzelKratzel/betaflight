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
#pragma once

#define TMG_OSD_NUM_XCHARS 45
#define TMG_OSD_NUM_YCHARS 20
#define TMG_OSD_NUM_OSD_CHARS (TMG_OSD_NUM_XCHARS*TMG_OSD_NUM_YCHARS)
#define TMG_OSD_BAUDRATE 115200

#define CMD_INVALID 0x00
#define CMD_GET_DEVICE_ID 0x01
#define CMD_SET_CONVERGENCE 0x02
#define CMD_CLR_SCREEN 0x03
#define CMD_SHOW_CHARS 0x04

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


void tmgOsdClearScreen(void *device);
void tmgOsdWriteChar(void *device, int8_t grabCount, uint8_t x, uint8_t y, const char c);
void tmgOsdWriteString(void *device, int8_t grabCount, uint8_t x, uint8_t y, const char *s);

