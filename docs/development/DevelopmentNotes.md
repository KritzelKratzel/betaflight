# Development Notes

KritzelKratzel's personal development/programming/hacking notes for Betaflight.

## Registry and Parameter Groups

Must read:

- https://github.com/betaflight/betaflight/blob/master/docs/development/Configuration%20Format.md
- https://github.com/betaflight/betaflight/blob/master/docs/development/Configuration%20Storage.md
- https://github.com/betaflight/betaflight/blob/master/docs/development/ParameterGroups.md

### Q & A on Slack

- **cli/settings.h:** 
  - Q: What is the difference between MASTER_VALUE, PROFILE_VALUE, PROFILE_RATE_VALUE and HARDWARE_VALUE?
  - A:  master_, profile_, and profile_rate_ determine if the parameter exists once, once per pid profile, or once per rate profile. hardware_ is like master_, but is shown in diff / dump hardware, to facilitate creation of unified target configuration.
  - Ref: https://betaflightgroup.slack.com/archives/C221J1H54/p1609854920235900
- **cli/settings.c:**
  - Q: Is there any requirement on the order of PG entries in `valueTable` ? Does it have to follow the enum order in `pg_ids.h` for example?
  - A: There's no strict grouping - new stuff should go at the end, unless it's extending existing parameters for an existing feature. It's definitely helpful if stuff that is conditional is added to an existing conditional section if one exists.
  - Ref: https://betaflightgroup.slack.com/archives/C221J1H54/p1609957997269300?thread_ts=1609886643.265400&cid=C221J1H54

### How to create a new Parameter Group

This is my log of creating a new but simple parameter group for my 3D-FPV-cameras. I take `pg/vcd.h` and `pg/vcd.c` as guidance. I would like to store the stereoscopic convergence value (integer ranging from 0 to 100) into EEPROM on flight controller.

1. Create new Parameter Group Identifier (PG ID) in `pg/pg_ids.h`: 

   ```c
   #define PG_CAM3D_CONFIG 1023
   ```

   Notice that 4095 is the current highest number that can be used.

2. Create new parameter groups by adding files `cam3d.h` and `cam3d.c` in subdir `pg`.  Create `cam3dProfile_s` accordingly in there and fill with members. 

   ```c
   // cam3d.h
   typedef struct cam3dProfile_s {
       uint8_t convergence;
   } cam3dProfile_t;
   PG_DECLARE(cam3dProfile_t, cam3dProfile);
   
   // cam3d.c
   // PG_REGISTER_WITH_RESET_TEMPLATE(_type, _name, _pgn, _version)
   PG_REGISTER_WITH_RESET_TEMPLATE(cam3dProfile_t, cam3dProfile, PG_CAM3D_CONFIG, 0);
   PG_RESET_TEMPLATE(cam3dProfile_t, cam3dProfile,
   		  .convergence = 55, // default convergence value
   );
   ```

   This will create a global read-only-variable, initialized form EEPROM. All *.c-files in sub-directory `pg` are automatically covered in `betaflight/make/source.mk`. Notice that legacy parameter groups are not yet declared/registered within `src/main/pg`; they are typically scattered across the code and used "in place".

3. Add entry in `cli/settings.c` like this:

   ```c
   #if defined(USE_TELEMETRY_NC3D)
   { "cam3d_convergence", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CAM3D_CONFIG, offsetof(cam3dProfile_t, convergence) },
   #endif
   ```

   Append entry to `const clivalue_t valueTable`. The `VAR_UINT8`-declaration must match with corresponding `struct` member declaration in `cam3d.h`. Notice the enclosing `USE_TELEMETRY_NC3D` define environment. Don't forget to add `#include "pg/cam3d.h"`. 

Further reading: https://betaflightgroup.slack.com/archives/C221J1H54/p1609886643265400

## Future Updates

Collection of thoughts, ideas and useful snippets.

### Implementation of TMGOSD based on FRSKYOSD with full Integration in Betaflight-CMS

Log of modified files in betaflight:

- `target/common_pre.h`: Added `#define USE_TMGOSD`.
- `main/osd/osd.h`: Appended `OSD_DISPLAYPORT_DEVICE_TMGOSD` to `osdDisplayPortDevice_e`.
- `cli/settings.c`: Appended `TMGOSD` to `lookupTableOsdDisplayPortDevice`.

Remarks:

- Display must be set via CLI with `set osd_displayport_device = TMGOSD`.

**Levels of Abstraction**

1. `drivers/display.c`: Provides top-level functions used by OSD and CMS. Functions are the same for different kind of displays.
2. `io/displayport_frsky_osd.c`: Defines display-specific `static const displayPortVTable_t frskyOsdVTable`, where each member is a pre-defined function pointer with standardized name. Those pointers point to hardware-specific implementation functions, which can be named arbitrarily. Furthermore provides interface to only one public function:  `frskyOsdDisplayPortInit()`.
   - Set `displayPort->cols/rows` somewhere on this level, e.g. `io/displayport_max7456.c:184`.
3. `io/frsky_osd.c`: Collects all hardware-specific implementation functions to all function pointers mentioned in level above.



**Call Tree (based on FRSKY_OSD):**

- `frsky_osd.c` frskyOsdDisplayPortInit(const videoSystem_e videoSystem)

  - frskyOsdInit(videoSystem)

    - findSerialPortConfig(FUNCTION_FRSKY_OSD): Simply map to `FUNCTION_TMG_OSD`.
    - openSerialPort(portConfig->identifier,
                  FUNCTION_FRSKY_OSD, NULL, NULL, FRSKY_OSD_BAUDRATE,
                  MODE_RXTX, portOptions): Define `TMG_OSD_BAUDRATE` in `tmg_osd.h`. Leave `MODE_RXTX` and make portOptions `SERIAL_NOT_INVERTED`.
    - frskyOsdStateReset(port)
      - frskyOsdResetReceiveBuffer(): Inits on `state.recvBuffer`.
      - frskyOsdResetSendBuffer(): Inits on `state.sendBuffer`.
      - Additional initialization of static variable `frskyOsdState_t state`.
    - frskyOsdRequestInfo(): 
      - frskyOsdSendAsyncCommand(OSD_CMD_INFO, &version, sizeof(version)): Probably the first point of something being sent to OSD subsystem.
        - frskyOsdSendCommand(cmd, data, size)
          - frskyOsdFlushSendBuffer(): Several calls to `frskyOsdProcessCommandU8()`, apparently sends out data.
            - frskyOsdProcessCommandU8(...)
              - serialWrite(state.port, c): Finally.
      - frskyOsdFlushSendBuffer(): Twice?

  - displayInit(&frskyOsdDisplayPort, &frskyOsdVTable): In `drivers/display.c`.

    - ```c
      instance->vTable = vTable;
      instance->vTable->clearScreen(instance);
      instance->useFullscreen = false;
      instance->cleared = true;
      instance->grabCount = 0;
      instance->cursorRow = -1;
      ```

  - redraw(&frskyOsdDisplayPort)

### FrSky OSD related Pull-Requests with modified files:

- https://github.com/betaflight/betaflight/pull/9127/files?file-filters%5B%5D=.c
  - **committed on 30 Nov 2019**
  - `make/source.mk`
  - `src/main/cli/cli.c`
  - `src/main/cli/settings.c`
  - `src/main/common/time.c`
  - `src/main/common/uvarint.c`
  - `src/main/common/uvarint.h`
  - `src/main/drivers/display.c`
  - `src/main/drivers/display.h`
  - `src/main/drivers/display_canvas.c`
  - `src/main/drivers/display_canvas.h`
  - `src/main/drivers/max7456.c`
  - `src/main/drivers/osd.h`
  - `src/main/drivers/osd_symbols.h`
  - `src/main/fc/init.c`
  - `src/main/io/displayport_frsky_osd.c`
  - `src/main/io/displayport_frsky_osd.h`
  - `src/main/io/displayport_max7456.c`
  - `src/main/io/frsky_osd.c`
  - `src/main/io/frsky_osd.h`
  - `src/main/io/serial.c`
  - `src/main/io/serial.h`
  - `src/main/msp/msp.c`
  - `src/main/osd/osd.c`
  - `src/main/osd/osd.h`
  - `src/main/osd/osd_elements.c`
  - `src/main/pg/vcd.h`
  - `src/main/target/common_pre.h`
  - `src/test/Makefile`
  - `src/test/unit/link_quality_unittest.cc`
  - `src/test/unit/osd_unittest.cc`
- https://github.com/betaflight/betaflight-configurator/pull/1852/files?file-filters%5B%5D=.json
  - **committed on 25 May 2020**
  - `locales/en/messages.json`
  - `src/js/msp/MSPHelper.js`
  - `src/js/tabs/osd.js`
  - `src/js/tabs/ports.js`
  - `src/tabs/osd.html`
- https://github.com/betaflight/betaflight-configurator/pull/1853/files?file-filters%5B%5D=.js
  - **committed on 3 Jan 2020**
  - `locales/en/messages.json`
  - `src/js/msp/MSPHelper.js`
  - `src/js/tabs/ports.js`







