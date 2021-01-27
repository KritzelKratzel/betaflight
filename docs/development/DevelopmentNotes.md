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

### OSD character position coordinates

The `#define` statements in `src/main/osd/osd.h` give some info about the bit pattern:

```c
#define OSD_PROFILE_BITS_POS 11
#define OSD_PROFILE_MASK    (((1 << OSD_PROFILE_COUNT) - 1) << OSD_PROFILE_BITS_POS)
#define OSD_POS_MAX   0x3FF
#define OSD_POSCFG_MAX   (OSD_PROFILE_MASK | 0x3FF) // For CLI values
#define OSD_PROFILE_FLAG(x)  (1 << ((x) - 1 + OSD_PROFILE_BITS_POS))
#define OSD_PROFILE_1_FLAG  OSD_PROFILE_FLAG(1)#ifdef USE_OSD_PROFILES
#define VISIBLE(x) osdElementVisible(x)
#define VISIBLE_IN_OSD_PROFILE(item, profile)    ((item) & ((OSD_PROFILE_1_FLAG) << ((profile)-1)))
#else
#define VISIBLE(x) ((x) & OSD_PROFILE_MASK)
#define VISIBLE_IN_OSD_PROFILE(item, profile) VISIBLE(item)
#endif// Character coordinate
#define OSD_POSITION_BITS 5 // 5 bits gives a range 0-31
#define OSD_POSITION_XY_MASK ((1 << OSD_POSITION_BITS) - 1)
#define OSD_POS(x,y)  ((x & OSD_POSITION_XY_MASK) | ((y & OSD_POSITION_XY_MASK) << OSD_POSITION_BITS))
#define OSD_X(x)      (x & OSD_POSITION_XY_MASK)
#define OSD_Y(x)      ((x >> OSD_POSITION_BITS) & OSD_POSITION_XY_MASK)
```

From what I understand the OSD element start coordinate reads as follows:

```
+----+----+----+----+
|--zz|z-yy|yyyx|xxxx|
+----+----+----+----+
   ^^ ^ ^     ^
   || | |     |
   || | |     +------: 5 bit address for x-coordiante (counted from left to right)
   || | +------------: 5 bit address for y-coordinate (counted from top to bottom)
   || +--------------: OSD profile 1 flag
   |+----------------: OSD profile 2 flag
   +-----------------: OSD profile 3 flag
```

Following this bit pattern, bits 15, 14 and 10 are currently unused. Bits 15 and 14 will be used to identify 4 different varieties of each OSD element. At the moment (2021-01-27) there are no plans to use bit 10.

Idea: Extend x-coordinates from 5 bits to 6 bits by using bit 10 as new MSB.

Further reading: https://betaflightgroup.slack.com/archives/C221J1H54/p1611691287121100

### TMG-OSD - List of modified/added Files

#### Useful Shell Commands

`git diff upstream/master origin/master --stat`

```bash
$ git remote -v
origin  git@github.com:KritzelKratzel/betaflight.git (fetch)
origin  git@github.com:KritzelKratzel/betaflight.git (push)
upstream        https://github.com/betaflight/betaflight.git (fetch)
upstream        https://github.com/betaflight/betaflight.git (push)
```

#### betaflight-configurator

- `locales/en/messages.json`: Define new `portsFunction`.

  ```diff
  @@ -1583,6 +1586,9 @@
       "portsFunction_FRSKY_OSD": {
           "message": "OSD (FrSky Protocol)"
       },
  +    "portsFunction_TMG_OSD": {
  +        "message": "OSD (TMG Protocol)"
  +    },
       "pidTuningProfileOption": {
           "message": "Profile $1"
       },
  ```

- `src/js/msp/MSPHelper.js`: Extend `self.SERIAL_PORT_FUNCTIONS`.

  ``` diff
  @@ -33,7 +34,10 @@ function MspHelper() {
       'RUNCAM_DEVICE_CONTROL': 14, // support communitate with RunCam Device
       'LIDAR_TF': 15,
       'FRSKY_OSD': 16,
  +    'TMG_OSD': 17, // support TheMissingGear 3D-camera devices
       };
  +    // serial ports function mask now 32 bits wide since merge of
  +    // https://github.com/betaflight/betaflight-configurator/pull/1851
  
       self.REBOOT_TYPES = {
           FIRMWARE: 0,
  ```

- `src/js/tabs/osd.js`: Some tweaks.

  ```diff
  @@ -1962,6 +1962,7 @@ OSD.msp = {
  
           let displayItemsCountActual = OSD.constants.DISPLAY_FIELDS.length;
  
  +        // bit flags relate to MSP_OSD_CONFIG message (betaflight/src/main/msp/msp.c)
           d.flags = view.readU8();
  
           if (d.flags > 0 && payload.length > 1) {
  @@ -1989,8 +1990,9 @@ OSD.msp = {
           d.state = {};
           d.state.haveSomeOsd = (d.flags !== 0);
           d.state.haveMax7456Configured = bit_check(d.flags, 4) || (d.flags === 1 && semver.lt(FC.CONFIG.apiVersion, API_VERSION_1_34));
  +        d.state.haveTmgOSDConfigured = semver.gte(FC.CONFIG.apiVersion, API_VERSION_1_43) && bit_check(d.flags, 2); // query position of OSD_FLAGS_OSD_HARDWARE_TMGOSD, see msp.c
           d.state.haveFrSkyOSDConfigured = semver.gte(FC.CONFIG.apiVersion, API_VERSION_1_43) && bit_check(d.flags, 3);
  -        d.state.haveMax7456FontDeviceConfigured = d.state.haveMax7456Configured || d.state.haveFrSkyOSDConfigured;
  +        d.state.haveMax7456FontDeviceConfigured = d.state.haveMax7456Configured || d.state.haveFrSkyOSDConfigured || d.state.haveTmgOSDConfigured;
           d.state.isMax7456FontDeviceDetected = bit_check(d.flags, 5) || (d.state.haveMax7456FontDeviceConfigured && semver.lt(FC.CONFIG.apiVersion, API_VERSION_1_43));
           d.state.haveOsdFeature = bit_check(d.flags, 0) || (d.flags === 1 && semver.lt(FC.CONFIG.apiVersion, API_VERSION_1_34));
           d.state.isOsdSlave = bit_check(d.flags, 1) && semver.gte(FC.CONFIG.apiVersion, API_VERSION_1_34);
  ```

- `src/js/tabs/ports.js`: Define new `TMG_OSD` menu entry in group `peripherals`. 

  ```diff
  @@ -15,8 +15,10 @@ TABS.ports.initialize = function (callback, scrollPosition) {
           { name: 'TELEMETRY_FRSKY',      groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['peripherals'], maxPorts: 1 },
           { name: 'TELEMETRY_HOTT',       groups: ['telemetry'], sharableWith: ['msp'], notSharableWith: ['peripherals'], maxPorts: 1 },
           { name: 'TELEMETRY_SMARTPORT',  groups: ['telemetry'], maxPorts: 1 },
           { name: 'RX_SERIAL',            groups: ['rx'], maxPorts: 1 },
           { name: 'BLACKBOX',     groups: ['peripherals'], sharableWith: ['msp'], notSharableWith: ['telemetry'], maxPorts: 1 },
  +        { name: 'TMG_OSD',      groups: ['peripherals'], sharableWith: ['msp'], notSharableWith: ['telemetry'], maxPorts: 1 },
       ];
  
       if (semver.gte(FC.CONFIG.apiVersion, "1.15.0")) {
  @@ -57,6 +59,10 @@ TABS.ports.initialize = function (callback, scrollPosition) {
           functionRules.push({ name: 'FRSKY_OSD', groups: ['peripherals'], maxPorts: 1 });
       }
  
  +    if (semver.gte(FC.CONFIG.apiVersion, API_VERSION_1_43)) {
  +        functionRules.push({ name: 'TMG_OSD', groups: ['peripherals'], maxPorts: 1 });
  +    }
  +
       for (const rule of functionRules) {
           rule.displayName = i18n.getMessage(`portsFunction_${rule.name}`);
       }
  ```

#### betaflight

- `src/main/cli/settings.c`: Add `TMGOSD` identifier.  Display must be activated later via CLI with `set osd_displayport_device = TMGOSD`.

  ``` diff
  const char * const lookupTableOsdDisplayPortDevice[] = {
  -    "NONE", "AUTO", "MAX7456", "MSP", "FRSKYOSD"
  +    "NONE", "AUTO", "MAX7456", "MSP", "FRSKYOSD", "TMGOSD"
  ```

- `src/main/io/serial.h`: Define new serial function identifier.

  ```diff
  +// needs to be identical to 'self.SERIAL_PORT_FUNCTIONS' in
  +// 'src/js/msp/MSPHelper.js' in betafligh-configurator
   typedef enum {
       FUNCTION_NONE                = 0,
       FUNCTION_MSP                 = (1 << 0),  // 1
  @@ -42,6 +44,7 @@ typedef enum {
       FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
       FUNCTION_RX_SERIAL           = (1 << 6),  // 64
       FUNCTION_BLACKBOX            = (1 << 7),  // 128
       FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
       FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
       FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
  @@ -50,10 +53,13 @@ typedef enum {
       FUNCTION_RCDEVICE            = (1 << 14), // 16384
       FUNCTION_LIDAR_TF            = (1 << 15), // 32768
       FUNCTION_FRSKY_OSD           = (1 << 16), // 65536
  +    FUNCTION_TMG_OSD             = (1 << 17), // 131072
   } serialPortFunction_e;
  +// serial ports function mask now 32 bits wide since merge of
  +// https://github.com/betaflight/betaflight/pull/9332
  
  ```

- `src/main/target/common_pre.h`: Introduce new pre-processor statement `USE_TMGOSD`.

  ```diff
  #if (TARGET_FLASH_SIZE > 128)
  @@ -357,6 +358,7 @@
   #define USE_CANVAS
   #define USE_DASHBOARD
   #define USE_FRSKYOSD
  +#define USE_TMGOSD
   #define USE_GPS
   #define USE_GPS_NMEA
   #define USE_GPS_UBLOX
  ```

- `src/main/osd/osd.h`: Add new `OSD_DISPLAYPORT_DEVICE`.

  ```diff
  @@ -250,6 +250,7 @@ typedef enum {
       OSD_DISPLAYPORT_DEVICE_MAX7456,
       OSD_DISPLAYPORT_DEVICE_MSP,
       OSD_DISPLAYPORT_DEVICE_FRSKYOSD,
  +    OSD_DISPLAYPORT_DEVICE_TMGOSD,
   } osdDisplayPortDevice_e;
  ```

- `src/main/fc/init.c`: Add new case for `OSD_DISPLAYPORT_DEVICE_TMGOSD`.

  ```diff
  @@ -942,8 +943,19 @@ void init(void)
           case OSD_DISPLAYPORT_DEVICE_AUTO:
               FALLTHROUGH;
  
  +#if defined(USE_TMGOSD)
  +        // Test OSD_DISPLAYPORT_DEVICE_TMGOSD first.
  +       case OSD_DISPLAYPORT_DEVICE_TMGOSD:
  +           osdDisplayPort = tmgOsdDisplayPortInit();
  +           if (osdDisplayPort || device == OSD_DISPLAYPORT_DEVICE_TMGOSD) {
  +               osdDisplayPortDevice = OSD_DISPLAYPORT_DEVICE_TMGOSD;
  +               break;
  +           }
  +            FALLTHROUGH;
  +#endif
  +
   #if defined(USE_FRSKYOSD)
  -        // Test OSD_DISPLAYPORT_DEVICE_FRSKYOSD first, since an FC could
  +        // Test OSD_DISPLAYPORT_DEVICE_FRSKYOSD second, since an FC could
           // have a builtin MAX7456 but also an FRSKYOSD connected to an
           // uart.
           case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
  ```

- Add a bunch of new files covering the actual implementation of `TMGOSD`. 
  - `src/main/io/displayport_tmg_osd.c` and `src/main/io/displayport_tmg_osd.h` cover the standardized functions called independently of the actually display port in operation.
  - `src/main/io/tmg_osd.c` and `src/main/io/tmg_osd.c` implement (if necessary) the actions to be taken upon call to the standardized displayport functions.

- `src/main/msp/msp.c`: Set OSD flags correctly. Use `OSD_FLAGS_RESERVED_1` for that purpose. The flag for `TMGOSD` then is mapped to bit 2 of `osdFlags`. Must match `bit_check(d.flags, 2)` statement in betaflight-configurator `src/js/tabs/osd.js` (see above).

  ```diff
  @@ -892,7 +892,8 @@ static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProce
       case MSP_OSD_CONFIG: {
   #define OSD_FLAGS_OSD_FEATURE           (1 << 0)
   //#define OSD_FLAGS_OSD_SLAVE             (1 << 1)
  -#define OSD_FLAGS_RESERVED_1            (1 << 2)
  +//#define OSD_FLAGS_RESERVED_1            (1 << 2)
  +#define OSD_FLAGS_OSD_HARDWARE_TMGOSD   (1 << 2)
   #define OSD_FLAGS_OSD_HARDWARE_FRSKYOSD (1 << 3)
   #define OSD_FLAGS_OSD_HARDWARE_MAX_7456 (1 << 4)
   #define OSD_FLAGS_OSD_DEVICE_DETECTED   (1 << 5)
  @@ -919,7 +920,14 @@ static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProce
               }
  
               break;
  -        default:
  +        case OSD_DISPLAYPORT_DEVICE_TMGOSD:
  +            osdFlags |= OSD_FLAGS_OSD_HARDWARE_TMGOSD;
  +            if (displayIsReady) {
  +                osdFlags |= OSD_FLAGS_OSD_DEVICE_DETECTED;
  +            }
  +
  +            break;
  +         default:
               break;
           }
  ```

#### **Levels of Abstraction**

1. `drivers/display.c`: Provides top-level functions used by OSD and CMS. Functions are the same for different kind of displays.
2. `io/displayport_tmg_osd.c`: Defines display-specific `static const displayPortVTable_t tmgOsdVTable`, where each member is a pre-defined function pointer with standardized name. Those pointers point to hardware-specific implementation functions, which can be named arbitrarily. Furthermore provides interface to only one public function:  `tmgOsdDisplayPortInit()`.
3. `io/tmg_osd.c`: Collects all hardware-specific implementation functions to all function pointers mentioned in level above.

### Implementation of TMGOSD based on FRSKYOSD with full Integration in Betaflight-CMS

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







