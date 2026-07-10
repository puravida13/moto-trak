// =========================================================================
//  secrets.example.h  --  TEMPLATE
// =========================================================================
//  1. Copy this file to "secrets.h" (same folder as the .ino)
//  2. Fill in your own Adafruit IO username and key
//  3. secrets.h is gitignored - your key never gets committed
//
//  Get your key from Adafruit IO -> "My Key" (the aio_... string).
//  The feed topics in the firmware are built from SECRET_MQTT_USER
//  automatically, so they always match whatever username you set here.
// =========================================================================

#ifndef SECRETS_H
#define SECRETS_H

#define SECRET_MQTT_USER "your_adafruit_io_username"
#define SECRET_MQTT_PASS "aio_xxxxxxxxxxxxxxxxxxxxxxxxxxxx"   // the aio_... string

#endif
