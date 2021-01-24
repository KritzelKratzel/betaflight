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

#include "platform.h"

#if defined(USE_TMGOSD)

#include "io/tmg_osd.h"
#include "io/serial.h"


void tmgOsdClearScreen(void *device)
{
  // header, configuration telegram
  serialPrint(device, "$AC");

  // msglen
  const uint16_t msglen=1; // payload only one byte
  serialWrite(device, (uint8_t) msglen); // lower byte
  serialWrite(device, (msglen >> 8)); // higher byte
  
  // serial payload starts here
  // command
  uint8_t crc = CMD_CLR_SCREEN;
  serialWrite(device, CMD_CLR_SCREEN);
  // serial payload ends here

  // crc
  serialWrite(device, crc);  
}

#endif
