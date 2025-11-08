#include <cmath>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "neomatrix.h"
#include "LIS3DH.h"

// ===================== CONFIG =====================
// Matrix size
static constexpr uint8_t W = 8;
static constexpr uint8_t H = 8;

// I2C block and pins for LIS3DH
static i2c_inst_t* const I2C = i2c1;
static constexpr uint SDA_PIN = 2;
static constexpr uint SCL_PIN = 3;

// Debug print to USB (0/1)
#ifndef LEVEL_DEBUG
#define LEVEL_DEBUG 0
#endif

// Onboard heartbeat LED (Feather RP2040 red LED = GPIO13)
#ifndef HEARTBEAT_LED
#define HEARTBEAT_LED 13
#endif

// Axis mapping toggles (runtime independent; switch in CMake with -D...=1)
#ifndef LEVEL_SWAP_XY
#define LEVEL_SWAP_XY 0
#endif
#ifndef LEVEL_FLIP_X
#define LEVEL_FLIP_X 0
#endif
#ifndef LEVEL_FLIP_Y
#define LEVEL_FLIP_Y 1   // default matches your earlier "positive Y moves dot up"
#endif

// Behavior knobs
static constexpr float LPF_ALPHA = 0.18f;    // smoothing on raw x,y (0..1)
static constexpr float DEAD_G_IN  = 0.10f;   // inside this radius -> "level" (enter)
static constexpr float DEAD_G_OUT = 0.14f;   // exit threshold (hysteresis)
static constexpr float SAT_G      = 0.60f;   // tilt where dot hits near the edge
static constexpr float R_PIX      = 3.2f;    // bubble travel radius in pixels

// Pixel-domain smoothing/hysteresis
static constexpr float PIXEL_ALPHA = 0.35f;  // low-pass in pixel space (0..1)
static constexpr float PIXEL_SLEW   = 1.0f;  // max pixels the dot can move per frame
static constexpr uint32_t CALIB_MS  = 600;   // auto-cal at boot (flat-ish)
static constexpr uint32_t FRAME_MS  = 25;    // ~40 FPS

// Colors
static constexpr uint8_t DOT_R = 200, DOT_G = 0,   DOT_B = 0;   // red
static constexpr uint8_t OK_R  = 0,   OK_G  = 160, OK_B = 0;    // green
// ==================================================

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}
static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
}

static void heartbeat_init() {
    gpio_init(HEARTBEAT_LED);
    gpio_set_dir(HEARTBEAT_LED, GPIO_OUT);
    gpio_put(HEARTBEAT_LED, 0);
}
static void heartbeat_tick() {
    static uint32_t last = 0;
    static bool on = false;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last >= 250) {
        on = !on;
        gpio_put(HEARTBEAT_LED, on);
        last = now;
    }
}

static void i2c_setup() {
    i2c_init(I2C, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

static bool lis3dh_try(LIS3DH& out) {
    LIS3DH a(I2C, 0x18);
    if (a.init() && a.read_reg(0x0F) == 0x33) { out = a; return true; }
    LIS3DH b(I2C, 0x19);
    if (b.init() && b.read_reg(0x0F) == 0x33) { out = b; return true; }
    return false;
}

static void auto_calibrate(LIS3DH& acc, float& x_off, float& y_off, uint32_t ms=CALIB_MS) {
    x_off = 0.f; y_off = 0.f;
    int n = 0;
    absolute_time_t t_end = make_timeout_time_ms(ms);
    while (!time_reached(t_end)) {
        acc.update();
        x_off += acc.x;
        y_off += acc.y;
        ++n;
        sleep_ms(10);
    }
    if (n > 0) { x_off /= n; y_off /= n; }
#if LEVEL_DEBUG
    printf("[cal] x_off=%.4f y_off=%.4f n=%d\n", x_off, y_off, n);
#endif
}

int main() {
    stdio_init_all();
    sleep_ms(300);
    heartbeat_init();

    // Matrix up
    NeoMatrix matrix(W, H);
    if (!matrix.init()) {
        while (true) { heartbeat_tick(); sleep_ms(40); }
    }

    // I2C + LIS3DH (uses YOUR current LIS3DH::update scaling)
    i2c_setup();
    LIS3DH acc(I2C, 0x18);
    if (!lis3dh_try(acc)) {
#if LEVEL_DEBUG
        printf("LIS3DH not detected (0x18/0x19)\n");
#endif
        while (true) { heartbeat_tick(); sleep_ms(40); }
    }

    // Auto-cal offsets (assume device held flat-ish)
    float x_off = 0.f, y_off = 0.f;
    auto_calibrate(acc, x_off, y_off);

    // Filtered state
    float x_lp = 0.f, y_lp = 0.f;

    // Pixel-space filters
    const float cx = (W - 1) * 0.5f; // 3.5
    const float cy = (H - 1) * 0.5f; // 3.5
    float fx_filt = cx;
    float fy_filt = cy;

    bool level_latched = false;

#if LEVEL_DEBUG
    uint32_t last_print = 0;
#endif

    while (true) {
        heartbeat_tick();
        acc.update();

        // Raw minus offsets
        float rx = acc.x - x_off;
        float ry = acc.y - y_off;

        // Axis tweaks to match your physical orientation
#if LEVEL_SWAP_XY
        float tmp = rx; rx = ry; ry = tmp;
#endif
#if LEVEL_FLIP_X
        rx = -rx;
#endif
#if LEVEL_FLIP_Y
        ry = -ry;
#endif

        // First-stage smoothing (g domain)
        x_lp = (1.f - LPF_ALPHA) * x_lp + LPF_ALPHA * rx;
        y_lp = (1.f - LPF_ALPHA) * y_lp + LPF_ALPHA * ry;

        // Distance from level with hysteresis to make "green" easier to hold
        float r = std::sqrt(x_lp*x_lp + y_lp*y_lp);
        if (!level_latched && r <= DEAD_G_IN)  level_latched = true;
        if ( level_latched && r >= DEAD_G_OUT) level_latched = false;

        // Map -SAT_G..+SAT_G -> pixel travel of R_PIX (no Z math, keeps your original behavior)
        float nx = clampf(x_lp / SAT_G, -1.f, 1.f);
        float ny = clampf(y_lp / SAT_G, -1.f, 1.f);

        // Bubble moves opposite tilt on Y for "uphill" feel
        float fx_target = cx + R_PIX * nx;
        float fy_target = cy - R_PIX * ny;

        // Second-stage smoothing in pixel space (removes boundary chatter)
        fx_filt = (1.f - PIXEL_ALPHA) * fx_filt + PIXEL_ALPHA * fx_target;
        fy_filt = (1.f - PIXEL_ALPHA) * fy_filt + PIXEL_ALPHA * fy_target;

        // Slew-limit the dot: at most PIXEL_SLEW px per frame to prevent flips
        auto slew = [](float prev, float now, float limit) {
            float d = now - prev;
            if (d >  limit) return prev + limit;
            if (d < -limit) return prev - limit;
            return now;
        };
        static float fx_slew = cx, fy_slew = cy;
        fx_slew = slew(fx_slew, fx_filt, PIXEL_SLEW);
        fy_slew = slew(fy_slew, fy_filt, PIXEL_SLEW);

        int col = clampi((int)lroundf(fx_slew), 0, W - 1);
        int row = clampi((int)lroundf(fy_slew), 0, H - 1);

        // Draw
        matrix.clear_pixels();
        if (level_latched) {
            // 2x2 center green
            for (int rpx = (int)cy; rpx <= (int)cy + 1; ++rpx)
                for (int cpx = (int)cx; cpx <= (int)cx + 1; ++cpx)
                    matrix.set_pixel(rpx, cpx, NeoMatrix::rgb(OK_R, OK_G, OK_B));
        } else {
            matrix.set_pixel(row, col, NeoMatrix::rgb(DOT_R, DOT_G, DOT_B));
        }
        matrix.write();

#if LEVEL_DEBUG
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print >= 120) {
            printf("rx,ry=(%.3f,%.3f) lp=(%.3f,%.3f) r=%.3f  pix=(%.2f,%.2f)->(%d,%d)%s\n",
                   rx, ry, x_lp, y_lp, r, fx_slew, fy_slew, row, col,
                   level_latched ? " [LEVEL]" : "");
            last_print = now;
        }
#endif
        sleep_ms(FRAME_MS);
    }
}
