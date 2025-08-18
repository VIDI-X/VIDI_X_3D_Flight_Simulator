/*
  VIDI X — 3D First-Person Arcade Flight Simulator
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
*/

#define LGFX_USE_V1
#include <Arduino.h>     // Arduino basics (pins, Serial, millis, etc.)
#include <LovyanGFX.hpp> // Fast drawing library for the ILI9341 LCD
#include <math.h>        // sin, cos, sqrt, tan, etc.
#include <string.h>      // memcpy, memset, strncpy

// ============================== LCD / SCREEN ===============================
// LCD driver class: handles low-level communication with the ILI9341 display.
// GLOBAL: lcd and canvas are global so the RENDER task can draw every frame.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    // SPI bus config (how we talk to the screen)
    { auto c = _bus.config();
      c.spi_host = VSPI_HOST;        // Which SPI hardware to use (ESP32 has HSPI/VSPI)
      c.spi_mode = 0;
      c.freq_write = 40000000;       // Write faster because we want FPS, baby!
      c.freq_read  = 16000000;       // Read speed (not used much)
      c.spi_3wire  = true;
      c.use_lock   = true;
      c.dma_channel= 1;
      c.pin_sclk   = 18;             // Clock pin to screen
      c.pin_mosi   = 23;             // Data out to screen
      c.pin_miso   = 19;             // Data in (not very used)
      c.pin_dc     = 21;             // Data/Command pin
      _bus.config(c); _panel.setBus(&_bus);
    }
    // Panel config (screen geometry)
    { auto c = _panel.config();
      c.pin_cs = 5;                  // Chip select
      c.pin_rst = -1;                // Reset (unused here)
      c.pin_busy = -1;
      c.memory_width  = 240;         // Panel memory size
      c.memory_height = 320;
      c.panel_width   = 240;         // Physical panel size
      c.panel_height  = 320;
      c.offset_x = 0; c.offset_y = 0;
      c.offset_rotation = 1;         // Orientation tweak
      c.dummy_read_pixel = 8;
      c.dummy_read_bits  = 1;
      c.readable = true;
      c.invert   = false;
      c.rgb_order= false;
      c.dlen_16bit = false;
      c.bus_shared = true;
      _panel.config(c);
    }
    setPanel(&_panel);
  }
};
static LGFX lcd;                     // GLOBAL: the physical display
static LGFX_Sprite canvas(&lcd);     // GLOBAL: an offscreen buffer we draw into, then push

// ============================== PINS / INPUT ===============================
// GLOBAL constants: fixed pin numbers we read for controls.
#define BTN_LR          34  // Analog: left/right stick
#define BTN_UD          35  // Analog: up/down stick
#define BTN_RESTART     0   // Digital: restart button
#define BTN_ACTION1     13  // Digital: speed up
#define BTN_ACTION2     27  // Digital: slow down
#define BTN_FIRE_BULLET 32  // Digital: fire bullets
#define BTN_FIRE_ROCKET 33  // Digital: fire rockets

// ============================== SCREEN GEOMETRY ============================
// GLOBAL constants: screen width/height, HUD height (bottom strip).
#define W 320
#define H 240
#define HUD_H 44

// The camera draws in the area above HUD. We keep center coordinates as GLOBALs.
// screen center for 3D drawing (y lowered because we reserve bottom for HUD)
static int SCREEN_CX = W/2;             // GLOBAL: screen X center for 3D
static int SCREEN_CY = (H - HUD_H)/2;   // GLOBAL: screen Y center for 3D

// ============================== WORLD / GRID ===============================
// GLOBAL constants that define how big the world cells are and how far we draw.
// RENDER_RANGE controls how many cells to draw outwards (bigger = slower but farther).
#define RENDER_RANGE 32
#define CELL_STEP    200   // One terrain grid step in world units (big cells = faster)
#define WORLD_MIN (-1000)  // not heavily used: clamps inside getHeight
#define WORLD_MAX (7000)

#define Min_Speed  0.0f    // minimum player speed
#define Max_Speed  300.0f  // maximum player speed
#define Min_Pitch -180.0f  // pitch clamped (just in case)
#define Max_Pitch  180.0f

// We build a fixed regular grid in front of the camera (and a tiny bit behind).
// These are GLOBAL macros used by terrain/sea/cloud samplers.
#define SIDE_RANGE_CELLS   (RENDER_RANGE + RENDER_RANGE/4)  // left/right extent
#define FWD_RANGE_CELLS    (RENDER_RANGE + RENDER_RANGE)    // forward extent (longer)
#define BACK_RANGE_CELLS   (RENDER_RANGE/1000)              // almost 0 behind

// ============================== CAMERA / FEEL ==============================
// VISUAL tricks to make things feel good on a tiny MCU.
#define DRAW_Z_SCALE 0.5f    // GLOBAL: compress vertical depth visually by 2x (flatter)
#define SPEED_STEP_K 0.1f    // GLOBAL: move 10x slower than speed => speed 50 "feels like" 5

// ============================== CLOUDS ====================================
// We fake clouds using "billboards" (little puffy circles at cloud heights).
#define CLOUD_BASE 5000.0f
#define CLOUD_TOP  10000.0f
#define CLOUD_STEP_CELLS 4
#define CLOUD_COL_A RGB565(210,220,240) // bright cloud core
#define CLOUD_COL_B RGB565(190,196,210) // softer outer puffs

// ============================== COMBAT / GAMEPLAY ==========================
// Projectiles and enemies pool sizes (GLOBALS).
#define MAX_BULLETS     24
#define MAX_ROCKETS      8
#define MAX_EBULLETS    24
#define MAX_ENEMIES      2
#define MAX_EXPLOSIONS  12

// Distances in world units (GLOBALS).
#define BULLET_MAX_DIST     2000.0f
#define ROCKET_MAX_DIST    15000.0f
#define E_BULLET_MAX_DIST   2000.0f

// Hit radiuses (GLOBALS).
#define BULLET_HIT_R   200.0f
#define ROCKET_HIT_R   300.0f

// Enemy despawn / behavior (GLOBALS).
#define ENEMY_MAX_DIST 24000.0f          // if too far, despawn
#define ENEMY_BEHIND_DOT -2000.0f        // behind threshold along forward axis
#define ENEMY_BEHIND_TIMEOUT_MS 3500     // time behind before despawn
#define ENEMY_FIRE_MIN_MS 500            // min fire delay
#define ENEMY_FIRE_MAX_MS 1200           // max fire delay
#define ENEMY_SAFE_ALT 800.0f            // keep enemies above ground

// ============================== SMALL HELPERS ==============================
// Tiny functions used in many places (GLOBALS).
static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b){ return ((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3); }
static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r){ return r * 180.0f / (float)M_PI; }
static inline float clampf(float v, float lo, float hi){ return (v<lo)?lo:((v>hi)?hi:v); }
static inline float wrapDeg(float a){ while(a<0)a+=360.0f; while(a>=360)a-=360.0f; return a; }
static inline float length3(float x,float y,float z){ return sqrtf(x*x + y*y + z*z); }
static inline void rgb565_to_rgb(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b){
  r=(c>>8)&0xF8; r|=r>>5; g=(c>>3)&0xFC; g|=g>>6; b=(c<<3)&0xF8; b|=b>>5;
}
static inline uint16_t scaleColor(uint16_t c, float k){
  uint8_t r,g,b; rgb565_to_rgb(c,r,g,b);
  int rr=(int)(r*k); if(rr<0)rr=0; if(rr>255)rr=255;
  int gg=(int)(g*k); if(gg<0)gg=0; if(gg>255)gg=255;
  int bb=(int)(b*k); if(bb<0)bb=0; if(bb>255)bb=255;
  return RGB565(rr,gg,bb);
}

// ============================== TERRAIN HEIGHT =============================
// "noise2D" provides a fake random-like number for any integer grid point.
// We combine multiple noise layers to create mountains (multi-frequency noise).
static inline float noise2D(int x, int y){
  long n = x + y * 57; n = (n << 13) ^ n;
  long nn = n * (n * n * 15731L + 789221L) + 1376312589L; nn &= 0x7fffffff;
  return 1.0f - (float)nn / 1073741824.0f; // -> range approx [-1..+1]
}

// Safe integer floor-division helper (used to locate noise cells).
static inline int floordiv(int a, int b){
  if(b<0){a=-a; b=-b;}
  return (a>=0) ? (a/b) : -((-a + b - 1)/b);
}

// The magic function that returns terrain height (z) for any (x,y).
// GLOBAL in effect because everybody uses it (drawing, collisions, HUD).
static float getHeight(int x, int y){
  // Three noise layers: large, medium, small details.
  const int P1=4000,P2=1000,P3=200;        // periods (bigger = slower changes)
  const float A1=10000, A2=2000, A3=500;   // amplitudes (how tall bumps are)

  auto layer=[&](int P,float A,int sx,int sy){
    // Find noise corners for interpolation
    int gx0=floordiv(x,P), gy0=floordiv(y,P);
    int gx1=gx0+1, gy1=gy0+1;
    // Sample the 4 corners, offset seeds (sx,sy) to vary layers
    float v00=noise2D(gx0+sx,gy0+sy), v10=noise2D(gx1+sx,gy0+sy);
    float v01=noise2D(gx0+sx,gy1+sy), v11=noise2D(gx1+sx,gy1+sy);
    // Bilinear interpolation within the cell
    float fx=(float)(x - gx0*P)/(float)P, fy=(float)(y - gy0*P)/(float)P;
    float i1=v00 + fx*(v10 - v00), i2=v01 + fx*(v11 - v01);
    return (i1 + fy*(i2 - i1)) * A;
  };

  // Mix layers and clamp to a reasonable world range.
  float h=(layer(P1,A1,0,0)+layer(P2,A2,100,100)+layer(P3,A3,200,200))/2.0f;
  if(h<-500) h=-500;
  if(h>7500) h=7500;
  return h;
}

// ============================== COLORS FOR TERRAIN =========================
// Turns a height into a nice color (blue for water, green for lowlands, etc.).
static uint16_t colorForHeight(float z){
  uint8_t r=0,g=0,b=0;
  if(z<0){ float t=(z+100)/100; if(t<0)t=0; if(t>1)t=1; b=(uint8_t)(128 + t*127);
  }else if(z<1000){ float t=z/1000; if(t>1)t=1; g=(uint8_t)(64 + t*191);
  }else if(z<3000){ float t=(z-1000)/2000; if(t>1)t=1; r=(uint8_t)(100 + t*100); g=(uint8_t)(50 + t*100); b=(uint8_t)(0 + t*50);
  }else{ r=g=b=255; }
  return RGB565(r,g,b);
}
static uint16_t edgeColorForHeight(float z){
  if (z >= 3000) return RGB565(200,200,200); // snowy edges
  return scaleColor(colorForHeight(z), 0.58f); // darker edge lines
}

// ============================== 3D PROJECTION ==============================
// Converts world (x,y,z) to screen pixels (sx,sy) using the player's position
// and orientation. This is the "camera". The camera IS the player (first-person).
struct ProjectedPoint{ bool visible; int16_t sx, sy; };
static const float FOV_DEG = 90.0f;  // GLOBAL: Field Of View
static float PROJ_D = 0.0f;          // GLOBAL: projection distance computed from FOV

// ============================== GAME STATE TYPES ===========================
// All these structs are GLOBAL-ish types used across the renderer and logic.

// A simple ring you can fly through (for score).
struct Hoop{ float x,y,z,radius; bool passed; };
const int NUM_HOOPS = 5;

// Player = camera. GLOBAL variable gPlayer stores its current state.
struct PlayerState {
  // Position of the camera in world coordinates (GLOBAL in gPlayer).
  float x,y,z;
  // Orientation angles in degrees (GLOBAL in gPlayer):
  //  yaw: turn left/right (like compass), pitch: nose up/down, roll: bank tilt.
  float yawDeg, pitchDeg, rollDeg;
  // Speed (world units per "game second"). We scale movement by SPEED_STEP_K.
  float speed;
  // Cached trigonometry to avoid recomputing sin/cos many times:
  float sYaw, cYaw;     // sin(yaw), cos(yaw)
  float sPitch, cPitch; // sin(pitch), cos(pitch)
};

// A projectile (bullet or rocket). Stored in fixed-size GLOBAL arrays (pools).
struct Projectile { float x,y,z, dx,dy,dz, speed, traveled; bool active; };

// An enemy aircraft with simple AI.
struct Enemy {
  float x,y,z;            // position
  float yawDeg;           // facing direction
  float speed;            // flight speed
  float altOffset;        // how high above ground it wants to be
  uint32_t nextFireMs;    // when to shoot again
  uint32_t behindSinceMs; // for despawn if stuck behind us
  bool active;            // alive/visible
};

// Visual effects for explosions (player crash and hits).
struct CrashFX { bool active; float x,y,z; uint32_t t0; };
struct ExplosionFx { bool active; float x,y,z; float scale; uint32_t t0; };

// Simple timed on-screen message ("You crashed", etc.)
struct ScreenMsg { bool active; char text[40]; uint32_t t0; uint16_t showAfterMs; uint16_t durationMs; };

// ============================== GLOBAL STATE ===============================
// These are the GLOBAL variables shared by logic + renderer tasks.
// We protect them with a mutex when copying.
static PlayerState gPlayer;                 // GLOBAL: current player (camera) state
static Hoop gCourse[NUM_HOOPS];             // GLOBAL: course hoops
static int gCurrentHoop = 0;                // GLOBAL: which hoop is next
static uint32_t gHoopFlashUntil = 0;        // GLOBAL: small flash when passing hoop

// GLOBAL projectile/enemy pools:
static Projectile gBullets[MAX_BULLETS];
static Projectile gRockets[MAX_ROCKETS];
static Projectile gEBullets[MAX_EBULLETS];
static Enemy      gEnemies[MAX_ENEMIES];

// GLOBAL FX and scoring:
static CrashFX     gCrash = {false,0,0,0,0};
static ExplosionFx gExpl[MAX_EXPLOSIONS];
static ScreenMsg   gMsg = {false,"",0,0,0};
static uint32_t    gRespawnAt = 0;          // when to respawn after crash
static int         gPoints = 0;             // score
static float       gShotMinDist = -1.0f;    // HUD: last projectile→enemy distance

// GLOBAL FPS:
static unsigned long fpsT0 = 0;
static int fpsFrames = 0;
static int fpsValue = 0;

// ============================== RTOS TASKING ===============================
// Handles: we need two tasks. GLOBAL so setup() can create them.
TaskHandle_t TaskLogic;
TaskHandle_t TaskRender;

// Mutex to protect shared globals when copying between tasks.
SemaphoreHandle_t stateMutex;

// ============================== PROTOTYPES =================================
// We declare functions before defining them so C++ is happy.
static bool  project_point_3d_to_2d(const PlayerState& S, float wx, float wy, float wz, int16_t &sx, int16_t &sy);
static float cloudDensityAt(int wx, int wy);
static float cloudVisibilityFactor(const PlayerState& S, float terrainZ, float dens);
static void  drawCloudBillboards(const PlayerState& S);
static void  drawSeaMesh(const PlayerState& S);
static void  drawSeaMeshOverlay(const PlayerState& S);
static void  drawTerrain(const PlayerState& S);
static void  drawHoops(const PlayerState& S, const Hoop* course, int current_hoop_index);
static bool  isOccludedByTerrain(const PlayerState& S, float ex, float ey, float ez);
static void  drawEnemies(const PlayerState& S, const Enemy* enemies, int ne);
static void  drawProjectiles(const PlayerState& S, const Projectile* b,int nb, const Projectile* r,int nr, const Projectile* eb,int ne);
static void  addExplosion(float x,float y,float z, float scale = 1.0f);
static void  clearExplosions();
static void  tickExplosions();
static void  drawExplosions(const PlayerState& S, const ExplosionFx* E, int n);
static void  drawCrashFX(const PlayerState& S, const CrashFX& C);
static void  showMsg(const char* txt, uint16_t showAfterMs=900, uint16_t durationMs=1400);
static void  drawMsg();
static void  drawHUD(const PlayerState& S, const Hoop* course, int current_hoop_index, float shotDist, int points);
static void  initCourse();
static void  initGame();
static void  clearActors();
static void  maintainEnemies(const PlayerState& P);
static void  updateEnemies(PlayerState& P);
static bool  updateProjectiles(PlayerState& P);
static bool  spawnEnemyAhead(const PlayerState& P);
static bool  addProjectile(Projectile* pool, int max, float x,float y,float z, float dx,float dy,float dz, float speed);
static bool  fireFromPlayer(bool rocket);
static bool  fireFromEnemy(const Enemy& E, const PlayerState& P);

void TaskLogicCode(void*);  // RTOS Task for logic (Core 0)
void TaskRenderCode(void*); // RTOS Task for rendering (Core 1)

// ============================== PROJECTION MATH ============================
// Converts 3D point (wx,wy,wz) into 2D pixel (sx,sy) using player state S.
// Steps:
//  1) Translate point so camera is at origin.
//  2) Rotate by inverse yaw/pitch (so we look forward along +Y camera axis).
//  3) Apply roll (so world tilts when plane banks).
//  4) Perspective divide: x/z and y/z using PROJ_D.
//  5) Move to screen center (SCREEN_CX/CY), invert Y for screen axis.
static bool project_point_3d_to_2d(const PlayerState& S, float wx, float wy, float wz, int16_t &sx, int16_t &sy) {
  // 1) Move world so camera is at (0,0,0)
  float dx = wx - S.x;
  float dy = wy - S.y;
  float dz = (wz - S.z);

  // 2) Undo yaw/pitch (rotate world into camera space)
  float s_yaw = -S.sYaw,    c_yaw = S.cYaw;
  float s_pitch = -S.sPitch, c_pitch = S.cPitch;

  float x_r = dx * c_yaw - dy * s_yaw;
  float y_r = dx * s_yaw + dy * c_yaw;
  float z_r = dz;

  float x_p = x_r;
  float y_p = y_r * c_pitch - z_r * s_pitch;
  float z_p = y_r * s_pitch + z_r * c_pitch;

  // 3) Apply roll to the camera view (HUD stays flat; roll affects world)
  float s_roll = sinf(deg2rad(S.rollDeg));
  float c_roll = cosf(deg2rad(S.rollDeg));
  float x_rr =  x_p * c_roll + z_p * s_roll;
  float y_rr =  y_p;
  float z_rr = -x_p * s_roll + z_p * c_roll;

  // 4) Perspective projection with vertical compression
  float cam_x = x_rr;
  float cam_y = z_rr * DRAW_Z_SCALE;   // squash vertical visually
  float cam_z = -y_rr;                 // depth forward

  const float clip_near = 1.0f;        // don't draw if behind camera
  if (cam_z < clip_near) return false;

  float sxp = PROJ_D * cam_x / cam_z;
  float syp = PROJ_D * cam_y / cam_z;

  // 5) Final screen pixels
  sx = (int16_t)(SCREEN_CX + sxp);
  sy = (int16_t)(SCREEN_CY - syp);

  if (sx < -W || sx > W*2 || sy < -H || sy > H*2) return false;
  return true;
}

// ============================== CLOUDS / FOG ===============================
// We sample cloud “density” above the terrain using noise at large scales.
// If ray between camera and terrain crosses the cloud layer, we dim the color.

// Returns smooth noise [0..1] based on world X,Y grid
static inline float noise01_from_grid(int wx, int wy, int scale, int sx, int sy){
  int gx = floordiv(wx, scale), gy = floordiv(wy, scale);
  float n = noise2D(gx + sx, gy + sy);
  return 0.5f*(n+1.0f);
}

// How cloudy this spot is (0 = no cloud, 1 = dense cloud)
static float cloudDensityAt(int wx, int wy){
  float d1 = noise01_from_grid(wx, wy, 6000, 31, 57);
  float d2 = noise01_from_grid(wx, wy,10000,101,203);
  float d  = 0.65f*d1 + 0.35f*d2;
  // Smoothstep: makes soft transitions (no harsh edges).
  return d*d*(3.0f - 2.0f*d);
}

// Visibility factor [0.5..1.0] depending on clouds between camera and ground.
// 1.0 = fully visible; 0.5 = quite foggy.
static float cloudVisibilityFactor(const PlayerState& S, float terrainZ, float dens){
  bool intersects = (fminf(S.z, terrainZ) < CLOUD_TOP) && (fmaxf(S.z, terrainZ) > CLOUD_BASE);
  if (!intersects) return 1.0f;
  bool inCloud = (S.z >= CLOUD_BASE && S.z <= CLOUD_TOP);
  float k = 1.0f - 0.45f*dens - 0.15f*(inCloud?1.0f:0.0f)*dens;
  if (k < 0.5f) k = 0.5f;
  return k;
}

// Draws cute "puffy" cloud billboards above terrain.
// LOCAL variables are used inside to loop and draw.
// Uses project_point_3d_to_2d() to place circles in the sky.
static void drawCloudBillboards(const PlayerState& S) {
  const float sY = S.sYaw, cY = S.cYaw;
  for (int z = -BACK_RANGE_CELLS; z <= FWD_RANGE_CELLS; z += CLOUD_STEP_CELLS) {
    const float grid_y_cam = -z * CELL_STEP;
    for (int x = -SIDE_RANGE_CELLS; x <= SIDE_RANGE_CELLS; x += CLOUD_STEP_CELLS) {
      const float grid_x_cam = x * CELL_STEP;

      // Convert camera-aligned grid to world X,Y (rotate by yaw)
      const float world_offset_x = grid_x_cam * cY - grid_y_cam * sY;
      const float world_offset_y = grid_x_cam * sY + grid_y_cam * cY;

      const int worldX = (int)(S.x + world_offset_x);
      const int worldY = (int)(S.y + world_offset_y);

      float dens = cloudDensityAt(worldX, worldY);
      if (dens < 0.70f) continue; // don’t draw weak clouds

      float cz = CLOUD_BASE + dens * (CLOUD_TOP - CLOUD_BASE);
      int16_t sx, sy;
      if (!project_point_3d_to_2d(S, (float)worldX, (float)worldY, cz, sx, sy)) continue;

      int r = 6 + (int)(12.0f * dens);
      uint16_t colCore = scaleColor(CLOUD_COL_A, 0.95f + 0.1f*dens);
      uint16_t colRing = scaleColor(CLOUD_COL_B, 0.90f);
      canvas.fillCircle(sx, sy, r, colCore);
      canvas.fillCircle(sx - r/2, sy + r/3, r*3/4, colRing);
      canvas.fillCircle(sx + r/3, sy - r/3, r*2/3, colRing);
    }
  }
}

// ============================== SEA (LEVEL Z=0) ============================
// We draw a grid mesh at Z=0 (sea level). Then an overlay for a "see-through" vibe.
static void drawSeaMesh(const PlayerState& S) {
  const int NX = 2 * SIDE_RANGE_CELLS + 1;
  static ProjectedPoint prevRow[(2*SIDE_RANGE_CELLS+1)]; // GLOBAL-like static: keeps previous row
  static bool havePrev = false;

  const uint16_t waterCol = RGB565(70,110,200);
  const float sY = S.sYaw, cY = S.cYaw;

  havePrev = false;
  for (int z = -BACK_RANGE_CELLS; z <= FWD_RANGE_CELLS; ++z) {
    const float grid_y_cam = -z * CELL_STEP;
    static ProjectedPoint row[(2*SIDE_RANGE_CELLS+1)];

    for (int x = -SIDE_RANGE_CELLS; x <= SIDE_RANGE_CELLS; ++x) {
      const int xi = x + SIDE_RANGE_CELLS;
      const float grid_x_cam = x * CELL_STEP;

      // Camera grid -> world XY
      const float world_offset_x = grid_x_cam * cY - grid_y_cam * sY;
      const float world_offset_y = grid_x_cam * sY + grid_y_cam * cY;

      const int worldX = (int)(S.x + world_offset_x);
      const int worldY = (int)(S.y + world_offset_y);

      row[xi].visible = project_point_3d_to_2d(S, (float)worldX, (float)worldY, 0.0f, row[xi].sx, row[xi].sy);
    }

    for (int xi = 0; xi < NX-1; ++xi) { // horizontal sea lines
      const ProjectedPoint &a = row[xi], &b = row[xi+1];
      if (a.visible && b.visible) canvas.drawLine(a.sx, a.sy, b.sx, b.sy, waterCol);
    }
    if (havePrev) { // vertical sea lines
      for (int xi = 0; xi < NX; ++xi) {
        const ProjectedPoint &a = prevRow[xi], &c = row[xi];
        if (a.visible && c.visible) canvas.drawLine(a.sx, a.sy, c.sx, c.sy, waterCol);
      }
    }
    memcpy(prevRow, row, sizeof(prevRow));
    havePrev = true;
  }
}

// Second layer: faint offset lines to fake translucency without killing FPS.
static void drawSeaMeshOverlay(const PlayerState& S) {
  const int NX = 2 * SIDE_RANGE_CELLS + 1;
  static ProjectedPoint prevRow[(2*SIDE_RANGE_CELLS+1)];
  static bool havePrev = false;

  uint16_t waterCol = RGB565(40, 70, 160);
  uint16_t waterCol2 = scaleColor(waterCol, 0.85f);

  const float sY = S.sYaw, cY = S.cYaw;
  havePrev = false;

  for (int z = -BACK_RANGE_CELLS; z <= FWD_RANGE_CELLS; ++z) {
    const float grid_y_cam = -z * CELL_STEP;
    static ProjectedPoint row[(2*SIDE_RANGE_CELLS+1)];

    for (int x = -SIDE_RANGE_CELLS; x <= SIDE_RANGE_CELLS; ++x) {
      const int xi = x + SIDE_RANGE_CELLS;
      const float grid_x_cam = x * CELL_STEP;

      const float world_offset_x = grid_x_cam * cY - grid_y_cam * sY;
      const float world_offset_y = grid_x_cam * sY + grid_y_cam * cY;

      const int worldX = (int)(S.x + world_offset_x);
      const int worldY = (int)(S.y + world_offset_y);

      row[xi].visible = project_point_3d_to_2d(S, (float)worldX, (float)worldY, 0.0f, row[xi].sx, row[xi].sy);
    }

    for (int xi = 0; xi < NX-1; ++xi) {
      const ProjectedPoint &a = row[xi], &b = row[xi+1];
      if (a.visible && b.visible) {
        canvas.drawLine(a.sx, a.sy, b.sx, b.sy, waterCol);
        canvas.drawLine(a.sx, a.sy+1, b.sx, b.sy+1, waterCol2);
      }
    }
    if (havePrev) {
      for (int xi = 0; xi < NX; ++xi) {
        const ProjectedPoint &a = prevRow[xi], &c = row[xi];
        if (a.visible && c.visible) {
          canvas.drawLine(a.sx, a.sy, c.sx, c.sy, waterCol);
          canvas.drawLine(a.sx+1, a.sy, c.sx+1, c.sy, waterCol2);
        }
      }
    }
    memcpy(prevRow, row, sizeof(prevRow));
    havePrev = true;
  }
}

// ============================== TERRAIN DRAW ===============================
// We draw the ground as FILLED TRIANGLES between rows of sampled points.
// To make it pretty: we shade by slope and fog by clouds.
static void drawTerrain(const PlayerState& S) {
  const int NX = 2 * SIDE_RANGE_CELLS + 1;

  // Static row buffers: act like mini GLOBALS (kept between calls) to avoid re-alloc.
  static ProjectedPoint row0[(2*SIDE_RANGE_CELLS+1)];
  static ProjectedPoint row1[(2*SIDE_RANGE_CELLS+1)];
  static float h0[(2*SIDE_RANGE_CELLS+1)];
  static float h1[(2*SIDE_RANGE_CELLS+1)];
  static float c0[(2*SIDE_RANGE_CELLS+1)];
  static float c1[(2*SIDE_RANGE_CELLS+1)];

  const float sY = S.sYaw, cY = S.cYaw;

  // Helper to sample one row of world points and project them
  auto sampleRow = [&](int z, ProjectedPoint* row, float* hh, float* cc){
    const float grid_y_cam = -z * CELL_STEP;
    for (int x = -SIDE_RANGE_CELLS; x <= SIDE_RANGE_CELLS; ++x) {
      const int xi = x + SIDE_RANGE_CELLS;
      const float grid_x_cam = x * CELL_STEP;

      const float world_offset_x = grid_x_cam * cY - grid_y_cam * sY;
      const float world_offset_y = grid_x_cam * sY + grid_y_cam * cY;

      const int worldX = (int)(S.x + world_offset_x);
      const int worldY = (int)(S.y + world_offset_y);

      float h = getHeight(worldX, worldY);
      hh[xi] = h;
      cc[xi] = cloudDensityAt(worldX, worldY); // for fog
      row[xi].visible = project_point_3d_to_2d(S, (float)worldX, (float)worldY, h, row[xi].sx, row[xi].sy);
    }
  };

  // First row at the back (little or zero)
  sampleRow(-BACK_RANGE_CELLS, row0, h0, c0);

  // For each strip, sample next row then draw two triangles per cell.
  for (int z = -BACK_RANGE_CELLS; z < FWD_RANGE_CELLS; ++z) {
    sampleRow(z+1, row1, h1, c1);

    for (int xi = 0; xi < NX-1; ++xi) {
      const ProjectedPoint &p00 = row0[xi];
      const ProjectedPoint &p10 = row0[xi+1];
      const ProjectedPoint &p01 = row1[xi];
      const ProjectedPoint &p11 = row1[xi+1];

      if (!(p00.visible || p10.visible || p01.visible || p11.visible)) continue;

      // Average height for color
      const float h00 = h0[xi], h10 = h0[xi+1];
      const float h01 = h1[xi], h11 = h1[xi+1];
      const float avg = 0.25f * (h00 + h10 + h01 + h11);

      // Simple slope-based darkening
      const float dhx = h10 - h00;
      const float dhz = h01 - h00;
      float kshade = 1.0f - 0.35f * sqrtf((dhx*dhx + dhz*dhz)) / (float)CELL_STEP;
      kshade = clampf(kshade, 0.55f, 1.0f);

      // Cloud "fog"
      const float densAvg = 0.25f * (c0[xi] + c0[xi+1] + c1[xi] + c1[xi+1]);
      const float kfog = cloudVisibilityFactor(S, avg, densAvg);

      uint16_t col = scaleColor(colorForHeight(avg), kshade * kfog);

      // Two triangles = one quad cell
      if (p00.visible && p10.visible && p01.visible)
        canvas.fillTriangle(p00.sx,p00.sy, p10.sx,p10.sy, p01.sx,p01.sy, col);
      if (p10.visible && p11.visible && p01.visible)
        canvas.fillTriangle(p10.sx,p10.sy, p11.sx,p11.sy, p01.sx,p01.sy, col);
    }

    // Shift row1 → row0 for next strip (fast copy)
    memcpy(row0, row1, sizeof(row0));
    memcpy(h0,   h1,   sizeof(h0));
    memcpy(c0,   c1,   sizeof(c0));
  }
}

// ============================== HOOPS (RINGS) ==============================
// Simple circles drawn as line segments for navigation/racing.
static void drawHoops(const PlayerState& S, const Hoop* course, int current_hoop_index) {
  const int segments = 16;
  for (int i = 0; i < NUM_HOOPS; ++i) {
    if (course[i].passed) continue;
    uint16_t color = (i == current_hoop_index) ? RGB565(255,255,0) : RGB565(200,200,200);

    int16_t sx_prev=0, sy_prev=0; bool visible_prev = false;
    for (int j = 0; j <= segments; ++j) {
      float angle = (float)j / (float)segments * 2.0f * (float)M_PI;
      float wx = course[i].x + cosf(angle) * course[i].radius;
      float wy = course[i].y + sinf(angle) * course[i].radius;
      float wz = course[i].z;

      int16_t sx, sy;
      bool visible = project_point_3d_to_2d(S, wx, wy, wz, sx, sy);
      if (j > 0 && visible && visible_prev) canvas.drawLine(sx_prev, sy_prev, sx, sy, color);
      sx_prev = sx; sy_prev = sy; visible_prev = visible;
    }
  }
}

// ============================== EXPLOSIONS / MESSAGES ======================
// Add a new explosion effect into the pool (GLOBAL gExpl).
static void addExplosion(float x,float y,float z, float scale){
  for (int i=0;i<MAX_EXPLOSIONS;++i){
    if (!gExpl[i].active){
      gExpl[i].active = true; gExpl[i].x = x; gExpl[i].y = y; gExpl[i].z = z;
      gExpl[i].scale = scale; gExpl[i].t0 = millis();
      return;
    }
  }
}
static void clearExplosions(){ memset(gExpl, 0, sizeof(gExpl)); }

// Time out explosions after ~0.9 seconds.
static void tickExplosions(){
  uint32_t now = millis();
  for (int i=0;i<MAX_EXPLOSIONS;++i){
    if (gExpl[i].active && now - gExpl[i].t0 > 900) gExpl[i].active = false;
  }
}

// Draw the pretty circles and sparks for each active explosion.
static void drawExplosions(const PlayerState& S, const ExplosionFx* E, int n){
  for (int i=0;i<n;++i){
    if (!E[i].active) continue;
    uint32_t t = millis() - E[i].t0;
    float u = t / 900.0f; if (u > 1.0f) u = 1.0f;
    int16_t sx, sy;
    if (!project_point_3d_to_2d(S, E[i].x, E[i].y, E[i].z, sx, sy)) continue;
    int R = (int)((6 + 50.0f * u) * E[i].scale);
    uint16_t core = RGB565(255, 220, 100);
    uint16_t glow = RGB565(255, 160, 40);
    uint16_t spark= RGB565(255, 240, 160);
    canvas.fillCircle(sx, sy, R, glow);
    canvas.fillCircle(sx, sy, R * 2 / 3, core);
    for (int k = 0; k < 10; ++k) {
      float a = k * (2.0f * (float)M_PI / 10.0f);
      int x2 = sx + (int)(cosf(a) * (R + 8));
      int y2 = sy + (int)(sinf(a) * (R + 8));
      canvas.drawLine(sx, sy, x2, y2, spark);
    }
  }
}

// Schedule a message (like "You crashed") to show after a delay for a duration.
static void showMsg(const char* txt, uint16_t showAfterMs, uint16_t durationMs) {
  strncpy(gMsg.text, txt, sizeof(gMsg.text)-1);
  gMsg.text[sizeof(gMsg.text)-1] = '\0';
  gMsg.active = true;
  gMsg.t0 = millis();
  gMsg.showAfterMs = showAfterMs;
  gMsg.durationMs  = durationMs;
}
static void drawMsg() {
  if (!gMsg.active) return;
  uint32_t now = millis();
  if (now < gMsg.t0 + gMsg.showAfterMs) return;
  if (now > gMsg.t0 + gMsg.showAfterMs + gMsg.durationMs) { gMsg.active = false; return; }

  canvas.setTextSize(2);
  canvas.setTextColor(RGB565(255,255,255));
  int16_t tw = canvas.textWidth(gMsg.text);
  int x = (W - tw) / 2;
  int y = SCREEN_CY - 10;

  int pad = 4;
  canvas.fillRect(x - pad, y - 2, tw + 2*pad, 18, RGB565(0,0,0));
  canvas.setCursor(x, y);
  canvas.print(gMsg.text);
}

// ============================== DRAWING: PROJECTILES & ENEMIES =============
// Each projectile is just a little dot (different colors for bullet/rocket/enemy).
static void drawProjectiles(const PlayerState& S, const Projectile* b,int nb, const Projectile* r,int nr, const Projectile* eb,int ne) {
  for (int i=0;i<nb;++i) if (b[i].active){ int16_t sx,sy; if(project_point_3d_to_2d(S,b[i].x,b[i].y,b[i].z,sx,sy)) canvas.fillCircle(sx,sy,2, RGB565(255,255,0)); }
  for (int i=0;i<nr;++i) if (r[i].active){ int16_t sx,sy; if(project_point_3d_to_2d(S,r[i].x,r[i].y,r[i].z,sx,sy)) canvas.fillCircle(sx,sy,3, RGB565(255,140,0)); }
  for (int i=0;i<ne;++i) if (eb[i].active){int16_t sx,sy; if(project_point_3d_to_2d(S,eb[i].x,eb[i].y,eb[i].z,sx,sy)) canvas.fillCircle(sx,sy,2, RGB565(255,60,60)); }
}

// Check if an enemy is hidden behind mountains (simple ray samples).
static bool isOccludedByTerrain(const PlayerState& S, float ex, float ey, float ez) {
  float dx = S.x - ex, dy = S.y - ey, dz = S.z - ez;
  float dist = sqrtf(dx*dx + dy*dy + dz*dz);
  if (dist < 500.0f) return false; // very close: don’t bother
  int steps = (int)clampf(dist / 800.0f, 8.0f, 48.0f);
  float invSteps = 1.0f / (float)steps;
  for (int i = 1; i < steps; ++i) {
    float t = i * invSteps;
    float x = ex + dx * t;
    float y = ey + dy * t;
    float z = ez + dz * t;
    float th = getHeight((int)x, (int)y);
    if (th > z + 8.0f) return true; // if terrain blocks the line of sight
  }
  return false;
}

// Draw triangular red enemies with a thicker wing line across them.
// Also print distance next to them to help the player decide when to fire.
static void drawEnemies(const PlayerState& S, const Enemy* enemies, int ne){
  for(int i=0;i<ne;++i){
    if(!enemies[i].active) continue;
    if (isOccludedByTerrain(S, enemies[i].x, enemies[i].y, enemies[i].z)) continue;

    float dx = enemies[i].x - S.x;
    float dy = enemies[i].y - S.y;
    float dz = enemies[i].z - S.z;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    int   distI = (int)dist;

    int16_t sx, sy;
    if (!project_point_3d_to_2d(S, enemies[i].x, enemies[i].y, enemies[i].z, sx, sy)) continue;

    // Size scales from 3 px at 20km to 20 px at 2km
    const float dNear = 2000.0f, dFar = 20000.0f;
    const float sMin  = 3.0f,   sMax = 20.0f;
    float t = (dFar - dist) / (dFar - dNear);
    if (t < 0.0f) t = 0.0f; if (t > 1.0f) t = 1.0f;
    int sz = (int)(sMin + t * (sMax - sMin) + 0.5f);

    uint16_t red = RGB565(255,0,0);
    // Little red triangle pointing up
    canvas.fillTriangle(sx, sy - sz,  sx - sz, sy + sz,  sx + sz, sy + sz, red);

    // Wing line: twice as long as triangle size, thicker when closer
    int wingHalf = sz * 2;
    int wingThick = sz / 5; if (wingThick < 1) wingThick = 1;
    for (int tline = -wingThick/2; tline <= wingThick/2; ++tline) {
      canvas.drawLine(sx - wingHalf, sy + tline, sx + wingHalf, sy + tline, red);
    }

    // Distance label in black, a bit to the right/top
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", distI);
    canvas.setTextSize(1);
    canvas.setTextColor(RGB565(0,0,0));
    canvas.setCursor(sx + sz + 3, sy - sz);
    canvas.print(buf);
  }
}

// ============================== CRASH EFFECT ===============================
// If the player hits the ground or gets shot, show an explosion at camera pos.
static void drawCrashFX(const PlayerState& S, const CrashFX& C) {
  if (!C.active) return;
  uint32_t t = millis() - C.t0;
  float u = t / 900.0f; if (u > 1.0f) u = 1.0f;

  int16_t sx, sy;
  bool vis = project_point_3d_to_2d(S, C.x, C.y, C.z + 6.0f, sx, sy);

  if (vis) {
    int R = 8 + (int)(70.0f * u);
    uint16_t core = RGB565(255, 220, 100);
    uint16_t glow = RGB565(255, 160, 40);
    uint16_t spark= RGB565(255, 240, 160);
    canvas.fillCircle(sx, sy, R, glow);
    canvas.fillCircle(sx, sy, R * 2 / 3, core);
    for (int i = 0; i < 12; ++i) {
      float a = i * (2.0f * (float)M_PI / 12.0f);
      int x2 = sx + (int)(cosf(a) * (R + 10));
      int y2 = sy + (int)(sinf(a) * (R + 10));
      canvas.drawLine(sx, sy, x2, y2, spark);
    }
  } else if (t < 120) {
    // If very close, just flash white border quickly
    canvas.drawRect(0, 0, W, H, RGB565(255,255,255));
    canvas.drawRect(1, 1, W-2, H-2, RGB565(255,255,255));
  }
}

// ============================== HUD (Bottom bar + reticle) =================
// HUD shows in green and stays flat (not rolled), while the world tilts.
// We draw center crosshair + horizon lines for pitch, and the bottom info bar.
static void drawHUD(const PlayerState& S, const Hoop* course, int current_hoop_index, float shotDist, int points) {
  const uint16_t hud_color = RGB565(80,255,80);
  const int cx = SCREEN_CX;
  const int cy = SCREEN_CY;

  // Pitch ladder: main horizon and short ticks every ±10°
  const float pix_per_deg = 2.0f;
  const float pitch_pixels = S.pitchDeg * pix_per_deg;

  auto drawHL = [&](float len, float offset_pix, uint16_t col) {
    const int16_t x1 = (int16_t)(cx - len);
    const int16_t y1 = (int16_t)(cy - offset_pix);
    const int16_t x2 = (int16_t)(cx + len);
    const int16_t y2 = (int16_t)(cy - offset_pix);
    canvas.drawLine(x1, y1, x2, y2, col);
  };

  const float main_len = W * 0.40f;
  const float tick_len = W * 0.18f;

  // Main horizon at pitch 0 aligns with cross center; when aligned, make cross mid-line black.
  drawHL(main_len, pitch_pixels, hud_color);
  for (int k = 1; k <= 3; ++k) {
    const float off = k * 10.0f * pix_per_deg;
    drawHL(tick_len, pitch_pixels + off, hud_color);
    drawHL(tick_len, pitch_pixels - off, hud_color);
  }

  const bool horizonThroughCenter = fabsf(S.pitchDeg) < 0.5f;
  const uint16_t crossHCol = horizonThroughCenter ? RGB565(0,0,0) : hud_color;

  // Crosshair center
  canvas.drawLine(cx - 10, cy, cx - 2,  cy, crossHCol);
  canvas.drawLine(cx + 2,  cy, cx + 10, cy, crossHCol);
  canvas.drawLine(cx, cy - 10, cx, cy - 2, hud_color);
  canvas.drawLine(cx, cy + 2,  cx, cy + 10, hud_color);

  // Bottom black bar
  const int y0 = H - HUD_H;
  canvas.fillRect(0, y0, W, HUD_H, RGB565(0,0,0));
  canvas.setTextSize(1);
  canvas.setTextColor(hud_color);

  // Left: speed, pitch, yaw
  char buf[48];
  snprintf(buf, sizeof(buf), "SPD %03d", (int)S.speed);
  canvas.setCursor(6, y0 + 4);  canvas.print(buf);

  snprintf(buf, sizeof(buf), "PCH %+04.0f%c", S.pitchDeg, 0xB0);
  canvas.setCursor(6, y0 + 16); canvas.print(buf);

  snprintf(buf, sizeof(buf), "YAW %+04.0f%c", S.yawDeg, 0xB0);
  canvas.setCursor(6, y0 + 28); canvas.print(buf);

  // Right: AGL and ASL (height above ground & above sea level)
  const float alt_agl = S.z - getHeight((int)S.x, (int)S.y);
  int16_t tw;
  snprintf(buf, sizeof(buf), "ALT AGL %04d", (int)alt_agl);
  tw = canvas.textWidth(buf);
  canvas.setCursor(W - tw - 6, y0 + 4);  canvas.print(buf);

  snprintf(buf, sizeof(buf), "ALT ASL %05d", (int)S.z);
  tw = canvas.textWidth(buf);
  canvas.setCursor(W - tw - 6, y0 + 16); canvas.print(buf);

  const int hdg = (int)wrapDeg(S.yawDeg);
  snprintf(buf, sizeof(buf), "HDG %03d%c", hdg, 0xB0);
  tw = canvas.textWidth(buf);
  canvas.setCursor(W - tw - 6, y0 + 28); canvas.print(buf);

  // Middle: score, points, fps (+ shot distance when available)
  snprintf(buf, sizeof(buf), "RINGS %d/%d", gCurrentHoop, NUM_HOOPS);
  int16_t sw = canvas.textWidth(buf);
  canvas.setCursor((W - sw)/2, y0 + 4);  canvas.print(buf);

  snprintf(buf, sizeof(buf), "PTS %d", points);
  sw = canvas.textWidth(buf);
  canvas.setCursor((W - sw)/2, y0 + 16); canvas.print(buf);

  snprintf(buf, sizeof(buf), "FPS %d", fpsValue);
  sw = canvas.textWidth(buf);
  canvas.setCursor((W - sw)/2, y0 + 28); canvas.print(buf);

  if (shotDist >= 0.0f) {
    snprintf(buf, sizeof(buf), "SHOT-DIST %d", (int)shotDist);
    sw = canvas.textWidth(buf);
    canvas.setCursor((W - sw)/2, y0 + 30);
    canvas.print(buf);
  }

  // Simple target box for next hoop (if any)
  if (current_hoop_index < NUM_HOOPS) {
    const Hoop& target = course[current_hoop_index];
    int16_t sx, sy;
    if (project_point_3d_to_2d(S, target.x, target.y, target.z, sx, sy)) {
      canvas.drawRect(sx - 20, sy - 20, 40, 40, RGB565(80,255,80));
      canvas.drawLine(sx - 20, sy - 20, sx - 10, sy - 20, RGB565(255,0,0));
    }
  }
}

// ============================== COMBAT HELPERS =============================
// Adds a projectile to a pool (GLOBAL arrays). Normalizes direction.
static bool addProjectile(Projectile* pool, int max, float x,float y,float z, float dx,float dy,float dz, float speed) {
  float L = length3(dx,dy,dz); if (L < 1e-5f) return false;
  dx/=L; dy/=L; dz/=L;
  for (int i=0;i<max;++i){
    if (!pool[i].active){
      pool[i].x=x; pool[i].y=y; pool[i].z=z;
      pool[i].dx=dx; pool[i].dy=dy; pool[i].dz=dz;
      pool[i].speed=speed; pool[i].traveled=0.0f; pool[i].active=true;
      return true;
    }
  }
  return false;
}

// Fire from player: rocket=true uses rocket pool; otherwise bullets.
static bool fireFromPlayer(bool rocket){
  float yaw = deg2rad(gPlayer.yawDeg), pitch = deg2rad(gPlayer.pitchDeg);
  // Forward vector based on yaw/pitch (nose direction)
  float dx = sinf(yaw) * cosf(pitch);
  float dy = -cosf(yaw) * cosf(pitch);
  float dz = -sinf(pitch);
  float spd = 100.0f + gPlayer.speed;
  return rocket
    ? addProjectile(gRockets, MAX_ROCKETS, gPlayer.x,gPlayer.y,gPlayer.z, dx,dy,dz, spd)
    : addProjectile(gBullets, MAX_BULLETS, gPlayer.x,gPlayer.y,gPlayer.z, dx,dy,dz, spd);
}

// Enemy shoots towards the player.
static bool fireFromEnemy(const Enemy& E, const PlayerState& P){
  float dx = (P.x - E.x), dy = (P.y - E.y), dz = (P.z - E.z);
  float spd = 100.0f + E.speed;
  return addProjectile(gEBullets, MAX_EBULLETS, E.x, E.y, E.z, dx,dy,dz, spd);
}

// Spawn an enemy somewhere far ahead-ish with some lateral offset.
static bool spawnEnemyAhead(const PlayerState& P){
  for (int i=0;i<MAX_ENEMIES;++i){
    if (!gEnemies[i].active){
      float yaw = deg2rad(P.yawDeg);
      float fx = sinf(yaw), fy = -cosf(yaw);
      float cx = cosf(yaw), cy = sinf(yaw);
      long dist = 8000 + (random(0, 5000));
      long lat  = -3000 + (random(0, 6001));
      float ex = P.x + fx * dist + cx * lat;
      float ey = P.y + fy * dist + cy * lat;
      float ez = getHeight((int)ex,(int)ey) + ENEMY_SAFE_ALT + random(0,1200);
      gEnemies[i].x = ex; gEnemies[i].y = ey; gEnemies[i].z = ez;
      gEnemies[i].yawDeg = wrapDeg(P.yawDeg + random(-200,201)*0.1f);
      gEnemies[i].speed  = 60.0f + random(0, 60);
      gEnemies[i].altOffset = ENEMY_SAFE_ALT + random(0, 800);
      gEnemies[i].nextFireMs = millis() + random(ENEMY_FIRE_MIN_MS, ENEMY_FIRE_MAX_MS);
      gEnemies[i].behindSinceMs = 0;
      gEnemies[i].active = true;
      return true;
    }
  }
  return false;
}

// Clears all pools (on respawn).
static void clearActors(){
  memset(gBullets, 0, sizeof(gBullets));
  memset(gRockets, 0, sizeof(gRockets));
  memset(gEBullets,0, sizeof(gEBullets));
  memset(gEnemies, 0, sizeof(gEnemies));
}

// Keep at least 1–2 enemies around.
static void maintainEnemies(const PlayerState& P){
  int active=0; for (int i=0;i<MAX_ENEMIES;++i) if (gEnemies[i].active) active++;
  if (active==0){
    int want = 1 + (random(0,100)<50 ? 0 : 1);
    for (int k=0;k<want;++k) spawnEnemyAhead(P);
  }
}

// Moves all projectiles, checks ground hits, enemy hits, player hit by enemy bullets.
// Returns true if the player was killed this frame.
static bool updateProjectiles(PlayerState& P){
  bool playerKilled = false;
  float minPair = 1e30f; bool havePair=false;

  auto stepOnePool = [&](Projectile* pool, int max, bool isFriendly, float hitR, float maxDist, float explScale){
    for (int i=0;i<max;++i){
      auto &pr = pool[i];
      if (!pr.active) continue;
      float step = pr.speed * SPEED_STEP_K; // move scaled by visual speed
      pr.x += pr.dx * step; pr.y += pr.dy * step; pr.z += pr.dz * step; pr.traveled += step;

      // Ground hit -> deactivate
      if (pr.z <= getHeight((int)pr.x, (int)pr.y)) { pr.active = false; continue; }
      // Too far
      if (pr.traveled >= maxDist) { pr.active = false; continue; }

      if (isFriendly){
        // Check enemy hit
        for (int e=0;e<MAX_ENEMIES;++e){
          if (!gEnemies[e].active) continue;
          float dx = gEnemies[e].x - pr.x, dy = gEnemies[e].y - pr.y, dz = gEnemies[e].z - pr.z;
          float dsq = dx*dx + dy*dy + dz*dz;
          if (dsq <= hitR*hitR){
            addExplosion(gEnemies[e].x, gEnemies[e].y, gEnemies[e].z, explScale);
            gEnemies[e].active = false; pr.active = false;
            gPoints += 2;                          // +2 points on kill
            showMsg("You killed the enemy", 900, 1400);
            break;
          }
          float d = sqrtf(dsq);
          if (d < minPair){ minPair = d; havePair=true; }
        }
      } else {
        // Enemy bullet near player?
        float dx = P.x - pr.x, dy = P.y - pr.y, dz = P.z - pr.z;
        if (dx*dx + dy*dy + dz*dz <= (BULLET_HIT_R*BULLET_HIT_R*0.7f)){
          playerKilled = true;
          pr.active = false;
        }
      }
    }
  };

  stepOnePool(gBullets,  MAX_BULLETS,  true,  BULLET_HIT_R,   BULLET_MAX_DIST, 1.0f);
  stepOnePool(gRockets,  MAX_ROCKETS,  true,  ROCKET_HIT_R,   ROCKET_MAX_DIST, 1.8f);
  stepOnePool(gEBullets, MAX_EBULLETS, false, BULLET_HIT_R,   E_BULLET_MAX_DIST, 1.0f);

  // Update HUD helper (distance to nearest enemy from a friendly projectile)
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  gShotMinDist = havePair ? minPair : -1.0f;
  xSemaphoreGive(stateMutex);

  return playerKilled;
}

// Enemy AI: face roughly towards player, wiggle a bit, keep altitude,
// fire sometimes, despawn if way behind/too far.
static void updateEnemies(PlayerState& P){
  float yawP = deg2rad(P.yawDeg);
  float fx = sinf(yawP), fy = -cosf(yawP);
  uint32_t now = millis();
  for (int i=0;i<MAX_ENEMIES;++i){
    Enemy &E = gEnemies[i];
    if (!E.active) continue;
    float vx = E.x - P.x, vy = E.y - P.y, vz = E.z - P.z;
    float dist = length3(vx,vy,vz);
    float dot = vx*fx + vy*fy;

    if (dist > ENEMY_MAX_DIST) { E.active = false; continue; }
    if (dot < ENEMY_BEHIND_DOT) {
      if (E.behindSinceMs == 0) E.behindSinceMs = now;
      else if (now - E.behindSinceMs > ENEMY_BEHIND_TIMEOUT_MS && dist > 8000.0f) {
        E.active = false; continue;
      }
    } else E.behindSinceMs = 0;

    float desiredYaw = wrapDeg(rad2deg(atan2f(vx, -vy)));
    float dYaw = desiredYaw - E.yawDeg;
    while (dYaw > 180) dYaw -= 360;
    while (dYaw < -180) dYaw += 360;
    float maxTurn = 0.6f; // turn speed
    if (dYaw >  maxTurn) dYaw =  maxTurn;
    if (dYaw < -maxTurn) dYaw = -maxTurn;
    E.yawDeg = wrapDeg(E.yawDeg + dYaw + (random(-10,11)*0.05f));
    E.speed += (random(-5,6)*0.3f);
    E.speed = clampf(E.speed, 40.0f, 140.0f);

    float yawE = deg2rad(E.yawDeg);
    float fxe = sinf(yawE), fye = -cosf(yawE);
    float step = E.speed * SPEED_STEP_K;
    E.x += fxe * step; E.y += fye * step;
    float terrain = getHeight((int)E.x, (int)E.y);
    float targetZ = terrain + E.altOffset;
    E.z += (targetZ - E.z) * 0.12f;

    if (now >= E.nextFireMs && dist < 12000.0f){ fireFromEnemy(E, P); E.nextFireMs = now + random(ENEMY_FIRE_MIN_MS, ENEMY_FIRE_MAX_MS); }
  }
  maintainEnemies(P);
}

// ============================== LOGIC TASK (Core 0) ========================
// This task reads inputs, updates the player physics, bullets/enemies,
// checks collisions, handles respawn & messages. It updates GLOBALS
// inside a mutex to keep renderer safe.
void TaskLogicCode(void*){
  const TickType_t tick_dt = pdMS_TO_TICKS(8); // run ~125 times per second
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t nextBulletMs = 0, nextRocketMs = 0;

  for(;;){
    // Make LOCAL copies of GLOBALS (thread-safe snapshot):
    PlayerState P; Hoop courseCopy[NUM_HOOPS]; int currentIdx; uint32_t flashUntil;
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    P = gPlayer; memcpy(courseCopy, gCourse, sizeof(gCourse));
    currentIdx = gCurrentHoop; flashUntil = gHoopFlashUntil;
    xSemaphoreGive(stateMutex);

    // Restart?
    if (digitalRead(BTN_RESTART) == LOW) {
      xSemaphoreTake(stateMutex, portMAX_DELAY); initGame(); xSemaphoreGive(stateMutex);
      vTaskDelay(pdMS_TO_TICKS(300));
      vTaskDelayUntil(&lastWake, tick_dt);
      continue;
    }

    // INPUTS (analog sticks + throttle buttons)
    int aLR = analogRead(BTN_LR);
    int aUD = analogRead(BTN_UD);

    if (digitalRead(BTN_ACTION1) == LOW) P.speed += 2.0f;
    if (digitalRead(BTN_ACTION2) == LOW) P.speed -= 2.0f;
    P.speed = clampf(P.speed, Min_Speed, Max_Speed);

    // Make controls 10× less sensitive (kid-friendly)
    const float YAW_RATE = 0.18f;
    const float PITCH_RATE = 0.18f;
    const float ROLL_FRICTION = 0.92f;

    float yaw_input = 0.0f;
    if      (aLR > 4000) yaw_input = -1.0f;
    else if (aLR > 1800 && aLR < 2200) yaw_input = 1.0f;

    float pitch_input = 0.0f;
    if      (aUD > 4000) pitch_input = 1.0f;
    else if (aUD > 1800 && aUD < 2200) pitch_input = -1.0f;

    // Update angles
    P.yawDeg   += yaw_input * YAW_RATE;
    P.pitchDeg += pitch_input * PITCH_RATE;
    P.pitchDeg = clampf(P.pitchDeg, Min_Pitch, Max_Pitch);
    float target_roll = -yaw_input * 25.0f; // slight banking by yaw
    P.rollDeg += (target_roll - P.rollDeg) * (1.0f - ROLL_FRICTION);

    // Move forward in the nose direction
    float yaw_rad   = deg2rad(P.yawDeg);
    float pitch_rad = deg2rad(P.pitchDeg);
    float fwd_x = sinf(yaw_rad) * cosf(pitch_rad);
    float fwd_y = -cosf(yaw_rad) * cosf(pitch_rad);
    float fwd_z = -sinf(pitch_rad);

    float step = P.speed * SPEED_STEP_K; // slow visual movement
    P.x += fwd_x * step; P.y += fwd_y * step; P.z += fwd_z * step;

    // Crash if below terrain
    float terrain_h = getHeight((int)P.x, (int)P.y);
    if (P.z < terrain_h) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      if (!gCrash.active) {
        gCrash.active = true; gCrash.x = P.x; gCrash.y = P.y; gCrash.z = terrain_h;
        gCrash.t0 = millis(); gRespawnAt = gCrash.t0 + 900;
        showMsg("You crashed", 900, 1400);
      }
      xSemaphoreGive(stateMutex);
      // Frozen at crash site until respawn time
      P.speed = 0.0f; P.x = gCrash.x; P.y = gCrash.y; P.z = gCrash.z + 5.0f;
      tickExplosions();
      if (millis() >= gRespawnAt) { xSemaphoreTake(stateMutex, portMAX_DELAY); gCrash.active=false; initGame(); xSemaphoreGive(stateMutex); }
      vTaskDelayUntil(&lastWake, tick_dt);
      continue;
    }

    // Hoops pass detection
    if (currentIdx < NUM_HOOPS) {
      const Hoop& target = courseCopy[currentIdx];
      float dx = P.x - target.x, dy = P.y - target.y, dz = P.z - target.z;
      float dist_sq = dx*dx + dy*dy + dz*dz;
      if (dist_sq < target.radius * target.radius) {
        courseCopy[currentIdx].passed = true; currentIdx++;
        flashUntil = millis() + 150;
        if (currentIdx == NUM_HOOPS) showMsg("Course complete!", 0, 2000);
      }
    }

    // Cache sin/cos
    P.sYaw   = sinf(yaw_rad);  P.cYaw   = cosf(yaw_rad);
    P.sPitch = sinf(pitch_rad);P.cPitch = cosf(pitch_rad);

    // Firing (with simple cooldowns)
    uint32_t now = millis();
    if (digitalRead(BTN_FIRE_BULLET) == LOW && now >= nextBulletMs) { fireFromPlayer(false); nextBulletMs = now + 90; }
    if (digitalRead(BTN_FIRE_ROCKET) == LOW && now >= nextRocketMs) { fireFromPlayer(true);  nextRocketMs = now + 350; }

    updateEnemies(P);
    bool killedByProj = updateProjectiles(P);

    if (killedByProj) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      if (!gCrash.active) {
        gCrash.active = true; gCrash.x = P.x; gCrash.y = P.y; gCrash.z = getHeight((int)P.x,(int)P.y);
        gCrash.t0 = millis(); gRespawnAt = gCrash.t0 + 900;
        showMsg("You were killed", 900, 1400);
      }
      xSemaphoreGive(stateMutex);
      P.speed = 0.0f;
    }

    tickExplosions();

    if (gCrash.active && millis() >= gRespawnAt) {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      gCrash.active = false; initGame();
      xSemaphoreGive(stateMutex);
    }

    // Write back LOCAL → GLOBAL
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    gPlayer = P; memcpy(gCourse, courseCopy, sizeof(gCourse));
    gCurrentHoop = currentIdx; gHoopFlashUntil = flashUntil;
    xSemaphoreGive(stateMutex);

    vTaskDelayUntil(&lastWake, tick_dt);
  }
}

// ============================== RENDER TASK (Core 1) =======================
// This task draws the full frame using a snapshot of GLOBALS.
// It never changes game state, only reads it (under mutex).
void TaskRenderCode(void*){
  fpsT0 = millis();
  for(;;){
    // LOCAL snapshots of GLOBALS for this frame:
    PlayerState S; Hoop courseCopy[NUM_HOOPS];
    int currentIdx; uint32_t flashUntil;
    Projectile bulletsCopy[MAX_BULLETS];
    Projectile rocketsCopy[MAX_ROCKETS];
    Projectile ebulletsCopy[MAX_EBULLETS];
    Enemy      enemiesCopy[MAX_ENEMIES];
    CrashFX    crashCopy;
    ExplosionFx explCopy[MAX_EXPLOSIONS];
    float      shotDistCopy;
    int        pointsCopy;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    S = gPlayer; memcpy(courseCopy, gCourse, sizeof(gCourse));
    currentIdx = gCurrentHoop; flashUntil = gHoopFlashUntil;
    memcpy(bulletsCopy,  gBullets,   sizeof(gBullets));
    memcpy(rocketsCopy,  gRockets,   sizeof(gRockets));
    memcpy(ebulletsCopy, gEBullets,  sizeof(gEBullets));
    memcpy(enemiesCopy,  gEnemies,   sizeof(gEnemies));
    crashCopy = gCrash;
    memcpy(explCopy, gExpl, sizeof(gExpl));
    shotDistCopy = gShotMinDist;
    pointsCopy = gPoints;
    xSemaphoreGive(stateMutex);

    // Draw order: sky → clouds → sea → terrain → enemies → hoops → projectiles → explosions → crashFX → messages → HUD
    canvas.fillScreen(RGB565(135,206,235)); // sky blue
    drawCloudBillboards(S);
    drawSeaMesh(S);
    drawSeaMeshOverlay(S);
    drawTerrain(S);
    drawEnemies(S, enemiesCopy, MAX_ENEMIES);
    drawHoops(S, courseCopy, currentIdx);
    drawProjectiles(S, bulletsCopy, MAX_BULLETS, rocketsCopy, MAX_ROCKETS, ebulletsCopy, MAX_EBULLETS);
    drawExplosions(S, explCopy, MAX_EXPLOSIONS);
    drawCrashFX(S, crashCopy);
    drawMsg();
    drawHUD(S, courseCopy, currentIdx, shotDistCopy, pointsCopy);

    // FPS counter (updated once per second)
    fpsFrames++;
    unsigned long _now = millis();
    if (_now - fpsT0 >= 1000) { fpsValue = fpsFrames; fpsFrames = 0; fpsT0 = _now; }

    if (millis() < flashUntil) {
      canvas.drawRect(0, 0, W, H, RGB565(255,255,255));
      canvas.drawRect(1, 1, W-2, H-2, RGB565(255,255,255));
    }

    // Push the finished frame to the LCD
    canvas.pushSprite(0, 0);
    vTaskDelay(pdMS_TO_TICKS(1)); // tiny breather
  }
}

// ============================== INIT STUFF =================================
// Course rings
static void initCourse() {
  gCourse[0] = {4355, -2600, 800, 50, false};
  gCourse[1] = {4555, -2500, 900, 50, false};
  gCourse[2] = {4655, -2700, 850, 50, false};
  gCourse[3] = {4455, -2900, 800, 50, false};
  gCourse[4] = {4355, -3100, 750, 50, false};
  for (int i=0;i<NUM_HOOPS;++i) gCourse[i].passed=false;
  gCurrentHoop = 0;
}

// Reset the whole game (player, score, actors, FX).
static void initGame() {
  // PLAYER = CAMERA (GLOBAL)
  gPlayer.x = 4355; gPlayer.y = -2800;
  gPlayer.z = getHeight((int)gPlayer.x, (int)gPlayer.y) + 5000; // start high above ground
  gPlayer.yawDeg = 0.0f; gPlayer.pitchDeg = 0.0f; gPlayer.rollDeg = 0.0f;
  gPlayer.speed = 50.0f;
  gPlayer.sYaw = sinf(deg2rad(gPlayer.yawDeg));
  gPlayer.cYaw = cosf(deg2rad(gPlayer.yawDeg));
  gPlayer.sPitch = sinf(deg2rad(gPlayer.pitchDeg));
  gPlayer.cPitch = cosf(deg2rad(gPlayer.pitchDeg));
  initCourse();
  gHoopFlashUntil = 0;
  gCrash = {false,0,0,0,0};
  gRespawnAt = 0;
  gShotMinDist = -1.0f;
  gPoints = 0;
  clearActors();
  clearExplosions();
}

// Arduino setup: init display, sprite, pins, projection constant, tasks.
void setup() {
  randomSeed(micros());
  Serial.begin(115200);

  lcd.init();
  lcd.setRotation(0);
  canvas.setPsram(true);
  canvas.setColorDepth(16);
  canvas.createSprite(W, H);

  pinMode(BTN_RESTART, INPUT_PULLUP);
  pinMode(BTN_LR, INPUT);
  pinMode(BTN_UD, INPUT);
  pinMode(BTN_ACTION1, INPUT_PULLUP);
  pinMode(BTN_ACTION2, INPUT_PULLUP);
  pinMode(BTN_FIRE_BULLET, INPUT_PULLUP);
  pinMode(BTN_FIRE_ROCKET, INPUT_PULLUP);

  // Compute projection distance once from FOV (GLOBAL)
  PROJ_D = (W * 0.5f) / tanf(deg2rad(FOV_DEG * 0.5f));
  SCREEN_CX = W/2;
  SCREEN_CY = (H - HUD_H)/2;

  initGame();

  // Create a mutex to protect GLOBAL state
  stateMutex = xSemaphoreCreateMutex();

  // Start two tasks on different cores:
  xTaskCreatePinnedToCore(TaskLogicCode,  "Logic",  10000, nullptr, 1, &TaskLogic,  0); // Core 0
  delay(200);
  xTaskCreatePinnedToCore(TaskRenderCode, "Render", 10000, nullptr, 2, &TaskRender, 1); // Core 1
  delay(200);
}

// Arduino loop is unused because we run two RTOS tasks
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
