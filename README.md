# VIDI X — 3D First-Person Arcade Flight Simulator
  ----------------------------------------------------------------------------
### Compiled With:
- Arduino IDE 2.3.7
- esp32 by Espressif Systems 3.3.5
### Libraries:

```C++
#include <Arduino.h>     // Arduino v1.0.0 - basics (pins, Serial, millis, etc.)
#include <LovyanGFX.hpp> // LovyanGFX v1.2.19 - Fast drawing library for the ILI9341 LCD
#include <math.h>        // sin, cos, sqrt, tan, etc.
#include <string.h>      // memcpy, memset, strncpy
```

----------------------------------------------------------------------------
  WHAT THIS PROGRAM DOES (kid-friendly summary):
  - We pretend we have an airplane (the "player") flying over mountains and sea.
  - We DRAW a 3D world (terrain + clouds + sea + enemies) but on a 2D screen.
  - To do that, we CONVERT (project) 3D points to 2D pixels.
  - We use TWO CORES of the ESP32:
      * Core 0: "brain" -> updates physics, inputs, bullets, enemies, collisions.
      * Core 1: "artist" -> draws everything to the screen as fast as possible.
  - There is a HUD (Heads-Up Display) with speed, altitude, heading, etc.
  - You can fire bullets (GPIO32) and rockets (GPIO33). Enemies also shoot!

  QUICK MAP OF IMPORTANT STUFF:
  - Camera position == Player position (gPlayer.x,y,z)   [GLOBAL]
    Because we render from first-person view (cockpit/camera is the player).
  - Player orientation: yaw (turn left/right), pitch (nose up/down), roll (bank)
  - Terrain height = getHeight(x, y) using layered noise (fake mountains).
  - Projection math: takes a 3D world coordinate and gives a screen pixel (x,y).
  - Z axis is visually compressed (DRAW_Z_SCALE) to make vertical things flatter.
  - Movement speed is scaled by SPEED_STEP_K (10x slower visually), so speed 50
    *looks* like moving 5 units per frame, as requested.

  READING HINTS:
  - "GLOBAL" variables live for the whole time and are shared between tasks.
  - "LOCAL" variables live only inside their function, like short-term notes.
  - Look for BIG SECTION HEADERS like "PROJECTION", "TERRAIN", "HUD", etc.
