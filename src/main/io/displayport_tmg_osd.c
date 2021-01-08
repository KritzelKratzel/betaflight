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
// #include "io/frsky_osd.h"


static const displayPortVTable_t tmgOsdVTable = {
    .grab = NULL,
    .release = NULL,
    .clearScreen = NULL,
    .drawScreen = NULL,
    .screenSize = NULL,
    .writeString = NULL,
    .writeChar = NULL,
    .isTransferInProgress = NULL,
    .heartbeat = NULL,
    .redraw = NULL,
    .isSynced = NULL,
    .txBytesFree = NULL,
    .layerSupported = NULL,
    .layerSelect = NULL,
    .layerCopy = NULL,
};

displayPort_t *tmgOsdDisplayPortInit(void)
{
    /* if (tmgOsdInit()) { */
    /*     displayInit(&tmgOsdDisplayPort, &tmgOsdVTable); */
    /*     redraw(&tmgOsdDisplayPort); */
    /*     return &tmgOsdDisplayPort; */
    /* } */
    return NULL;
}

#endif // USE_TMGOSD
