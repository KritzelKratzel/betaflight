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

#ifdef USE_TMGOSD

#include "osd/osd.h"
#include "io/tmg_osd.h"
#include "io/serial.h"

// see config/config.c: currentPidProfile
// tmgOsdProfile_t *currentTmgOsdProfile;
int8_t tmgOsdCurrent3dConvergence = 0;

static uint16_t calculatePosition(const int8_t grabCount, const uint8_t x, const uint8_t y)
{
  uint16_t xx, yy;
  if (grabCount == 0) {
    // Use automatic character position rescaling while in OSD-mode
    // rescale 4:3 to 16:9 aspect ratio
    xx = 3*x/2; // 45 / 30 = 1.5
    yy = 5*y/4; // 20 / 16 = 1.25
  }
  else {
    // Use original character position while in CMS-mode
    xx = x;
    yy = y;
  }
  return (TMG_OSD_NUM_XCHARS*yy + xx);
}

void tmgOsdWriteChar(void *device, int8_t grabCount, uint8_t x, uint8_t y, const char c)
{
  // calculate position from x,y coordinates
  uint16_t startPosition=calculatePosition(grabCount, x, y);

  // send header, data telegram
  serialPrint(device, "$AD");

  // send msglen; must include uint16_t for character start position
  uint16_t msglen=sizeof(c) + sizeof(startPosition);
  serialWrite(device, (uint8_t) msglen); // low byte
  serialWrite(device, (msglen >> 8));    // high byte

  // send startPosition, chkSum calculation starts from here
  uint8_t chkSum = (uint8_t) startPosition;
  serialWrite(device, (uint8_t) startPosition);
  chkSum ^= (startPosition >> 8);
  serialWrite(device, (startPosition >> 8));

  // send character
  chkSum ^= c;
  serialWrite(device, c);

  // send chkSum
  serialWrite(device, chkSum);
}

void tmgOsdWriteString(void *device, int8_t grabCount, uint8_t x, uint8_t y, const char *s)
{
  if (strlen(s) == 0) return;

  // calculate position from x,y coordinates
  uint16_t startPosition=calculatePosition(grabCount, x, y);

  // send header, data telegram
  serialPrint(device, "$AD");

  // send msglen; must include uint16_t for character start position
  uint16_t msglen=strlen(s) + sizeof(startPosition);
  serialWrite(device, (uint8_t) msglen); // low byte
  serialWrite(device, (msglen >> 8));    // high byte

  // send startPosition, chkSum calculation starts from here
  uint8_t chkSum = (uint8_t) startPosition;
  serialWrite(device, (uint8_t) startPosition);
  chkSum ^= (startPosition >> 8);
  serialWrite(device, (startPosition >> 8));

  // send string
  for (uint8_t i=0; i<strlen(s); i++){
    chkSum ^= s[i];
    serialWrite(device, s[i]);
  }
  
  // send chkSum
  serialWrite(device, chkSum);
}

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
  uint8_t chkSum = CMD_CLR_SCREEN;
  serialWrite(device, CMD_CLR_SCREEN);
  // serial payload ends here

  // chkSum
  serialWrite(device, chkSum);
}

void tmgOsdSet3dConvergence(void *device)
{
  static uint8_t lastConv = 255;
  // Get current convergence value from config and recompute for camera value range
  uint8_t currentConv = 50 + (uint8_t)osdConfig()->osd3dConvergence;
  if (lastConv == currentConv) return; // nothing to do

  // update for next cycle
  lastConv = currentConv;
  
  // header, configuration telegram
  serialPrint(device, "$AC");

  // msglen
  const uint16_t msglen=2; // payload only two bytes
  serialWrite(device, (uint8_t) msglen); // lower byte
  serialWrite(device, (msglen >> 8)); // higher byte

  // serial payload starts here
  // command
  serialWrite(device, CMD_SET_CONVERGENCE); // first byte: command
  uint8_t chkSum = CMD_SET_CONVERGENCE;
  serialWrite(device, currentConv); // second byte: argument
  chkSum ^= currentConv;
  // serial payload ends here
  
  // chkSum
  serialWrite(device, chkSum);
}
#endif
