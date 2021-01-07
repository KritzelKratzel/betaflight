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

### How to create a new Parameter Group

This is my log of creating a new but simple parameter group for my 3D-FPV-cameras. I take `pg/vcd.h` and `pg/vcd.c` as guidance. I would like to store the stereoscopic convergence value (integer ranging from 0 to 100) into EEPROM on flight controller.

1. Create new Parameter Group Identifier (PG ID) in `pg/pg_ids.h`: 

   ```c
   #define PG_CAM3D_CONFIG 1023
   ```

   Notice that 4095 is the current highest number that can be used.

2. Create new files `cam3d.h` and `cam3d.c` in subdir `pg`.  Create `cam3dProfile_s` accordingly in there and fill with members. 

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

   This will create a global read-only-variable, initialized form EEPROM. All *.c-files in sub-directory `pg` are automatically covered in `betaflight\make\source.mk`.

3. Add entry in `cli/settings.c` like this:

   ```c
   #if defined(USE_TELEMETRY_NC3D)
   { "cam3d_convergence", VAR_UINT8 | MASTER_VALUE, .config.minmaxUnsigned = { 0, 100 }, PG_CAM3D_CONFIG, offsetof(cam3dProfile_t, convergence) },
   #endif
   ```

   Append entry to `const clivalue_t valueTable`. The `VAR_UINT8`-declaration must match with corresponding `struct` member declaration in `cam3d.h`. Notice the enclosing `USE_TELEMETRY_NC3D` define environment. Don't forget to add `#include "pg/cam3d.h"`. 

Further reading: https://betaflightgroup.slack.com/archives/C221J1H54/p1609886643265400



