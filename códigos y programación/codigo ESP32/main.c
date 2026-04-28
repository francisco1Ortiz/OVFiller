#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "esp_http_server.h"

#define TAG "OVFILLER_MULTI"

// =========================
// Pines
// =========================
#define PIN_PUL         GPIO_NUM_26
#define PIN_DIR         GPIO_NUM_25
#define PIN_BOMBA       GPIO_NUM_13
#define PIN_ENROSQUE    GPIO_NUM_14
#define PIN_SENSOR      GPIO_NUM_23

// =========================
// WiFi AP
// =========================
#define WIFI_SSID       "OVFILLER_AP"
#define WIFI_PASS       "12345678"
#define WIFI_CHANNEL    1
#define MAX_STA_CONN    4

// =========================
// Sensor
// true  => LOW = botella detectada
// false => HIGH = botella detectada
// =========================
#define SENSOR_ACTIVE_LOW   1

// =========================
// Salidas invertidas
// true => ON con LOW
// =========================
#define DIG_OUT_ACTIVE_LOW  1

// =========================
// Stepper
// =========================
#define DEFAULT_STEPPER_SPS         500
#define CYCLE_EXTRA_STEPS           400
#define DIR_SETUP_MS                20
#define DETECT_TO_PUMP_DELAY_MS     1000
#define CYCLE_DIR_FIXED             1
#define CYCLE_SPEED_PERCENT         45

// =========================
// Calibración
// =========================
#define MAX_LIQUIDS     6
#define NAME_LEN        24
#define CAL_POINTS      5

static const float g_cal_times[CAL_POINTS] = {5.0f, 7.0f, 9.0f, 11.0f, 13.0f};

// =========================
// NVS
// =========================
#define NVS_NS          "liqdb"
#define NVS_KEY_PROF    "profiles"
#define NVS_KEY_SEL     "sel_liq"

// =========================
// Tipos
// =========================
typedef struct {
    char  name[NAME_LEN];
    float m_gps;
    float b_g;
    float density_g_ml;
    bool  valid;
} liquid_profile_t;

typedef enum {
    MODE_IDLE = 0,
    MODE_AUTO_CYCLE = 1,
    MODE_CAL_PUMP = 2
} run_mode_t;

// =========================
// Estado global
// =========================
static liquid_profile_t g_profiles[MAX_LIQUIDS];
static int g_selected_liquid = 0;

static volatile bool g_bomba_on = false;
static volatile bool g_enrosque_on = false;
static volatile bool g_busy = false;
static volatile bool g_stop_requested = false;
static volatile run_mode_t g_mode = MODE_IDLE;

static char g_cycle_state[96] = "Listo";
static volatile float g_target_ml = 200.0f;
static volatile float g_last_fill_seconds = 0.0f;

static volatile int g_stepper_sps = DEFAULT_STEPPER_SPS;

// Estadísticas
static volatile uint32_t g_cycle_count = 0;
static volatile float g_last_cycle_total_seconds = 0.0f;
static volatile float g_last_cycle_ml = 0.0f;

static httpd_handle_t g_server = NULL;

// =========================
// Helpers básicos
// =========================
static inline void set_output_active_low(gpio_num_t pin, bool on)
{
#if DIG_OUT_ACTIVE_LOW
    gpio_set_level(pin, on ? 0 : 1);
#else
    gpio_set_level(pin, on ? 1 : 0);
#endif
}

static inline void bomba_set(bool on)
{
    set_output_active_low(PIN_BOMBA, on);
    g_bomba_on = on;
}

static inline bool sensor_obstacle_raw(void)
{
    int level = gpio_get_level(PIN_SENSOR);
#if SENSOR_ACTIVE_LOW
    return (level == 0);
#else
    return (level == 1);
#endif
}

// =========================
// Stepper ánodo común
// =========================
static inline void stepper_idle_strong(void)
{
    gpio_set_direction(PIN_PUL, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIR, 1);
}

static inline void stepper_idle_with_dir(int dir)
{
    gpio_set_direction(PIN_PUL, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIR, dir ? 1 : 0);
}

static inline void stepper_set_dir(int dir)
{
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIR, dir ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(DIR_SETUP_MS));
}

static inline void stepper_pulse_once_us(uint32_t half_period_us)
{
    gpio_set_direction(PIN_PUL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PUL, 0);
    esp_rom_delay_us(half_period_us);

    gpio_set_direction(PIN_PUL, GPIO_MODE_INPUT);
    esp_rom_delay_us(half_period_us);
}

static void stepper_run_steps_blocking(int steps, int dir, int sps, volatile bool *stop_flag)
{
    if (sps <= 0) sps = DEFAULT_STEPPER_SPS;

    uint32_t half_period_us = (1000000UL / (uint32_t)sps) / 2UL;
    if (half_period_us < 50) half_period_us = 50;

    stepper_set_dir(dir);

    for (int i = 0; i < steps; i++) {
        if (stop_flag && *stop_flag) break;
        stepper_pulse_once_us(half_period_us);
    }

    stepper_idle_with_dir(dir);
}

static void stepper_run_until_sensor_blocking(int dir, int sps, volatile bool *stop_flag, bool *sensor_hit)
{
    if (sensor_hit) *sensor_hit = false;
    if (sps <= 0) sps = DEFAULT_STEPPER_SPS;

    uint32_t half_period_us = (1000000UL / (uint32_t)sps) / 2UL;
    if (half_period_us < 50) half_period_us = 50;

    stepper_set_dir(dir);

    while (1) {
        if (stop_flag && *stop_flag) break;
        if (sensor_obstacle_raw()) {
            if (sensor_hit) *sensor_hit = true;
            break;
        }
        stepper_pulse_once_us(half_period_us);
    }

    stepper_idle_with_dir(dir);
}

// =========================
// Líquidos / NVS
// =========================
static void set_default_profiles(void)
{
    memset(g_profiles, 0, sizeof(g_profiles));

    strncpy(g_profiles[0].name, "Vainilla", NAME_LEN - 1);
    g_profiles[0].m_gps = 18.31f;
    g_profiles[0].b_g = 1.83f;
    g_profiles[0].density_g_ml = 1.00f;
    g_profiles[0].valid = true;

    g_selected_liquid = 0;
}

static void save_profiles_to_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) != ESP_OK) {
        return;
    }

    nvs_set_blob(nvs, NVS_KEY_PROF, g_profiles, sizeof(g_profiles));
    nvs_set_i32(nvs, NVS_KEY_SEL, g_selected_liquid);
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void load_profiles_from_nvs(void)
{
    nvs_handle_t nvs;
    size_t size = sizeof(g_profiles);

    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) != ESP_OK) {
        set_default_profiles();
        return;
    }

    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_PROF, g_profiles, &size);
    if (err != ESP_OK || size != sizeof(g_profiles)) {
        set_default_profiles();
        save_profiles_to_nvs();
        nvs_close(nvs);
        return;
    }

    int32_t sel = 0;
    if (nvs_get_i32(nvs, NVS_KEY_SEL, &sel) == ESP_OK) {
        if (sel >= 0 && sel < MAX_LIQUIDS) {
            g_selected_liquid = (int)sel;
        }
    }

    nvs_close(nvs);

    bool any_valid = false;
    for (int i = 0; i < MAX_LIQUIDS; i++) {
        if (g_profiles[i].valid) {
            any_valid = true;
            break;
        }
    }
    if (!any_valid) {
        set_default_profiles();
        save_profiles_to_nvs();
    }
}

static float ml_to_grams(int idx, float ml)
{
    if (idx < 0 || idx >= MAX_LIQUIDS || !g_profiles[idx].valid) {
        return 0.0f;
    }
    if (ml < 0.0f) ml = 0.0f;
    return ml * g_profiles[idx].density_g_ml;
}

static float grams_to_seconds_profile(int idx, float grams)
{
    if (idx < 0 || idx >= MAX_LIQUIDS || !g_profiles[idx].valid) {
        return 0.0f;
    }
    float m = g_profiles[idx].m_gps;
    float b = g_profiles[idx].b_g;

    if (m <= 0.0001f) return 0.0f;
    if (grams < b) grams = b;

    return (grams - b) / m;
}

static float ml_to_seconds_profile(int idx, float ml)
{
    float grams = ml_to_grams(idx, ml);
    return grams_to_seconds_profile(idx, grams);
}

// =========================
// Regressión lineal
// gramos = m * segundos + b
// =========================
static bool calculate_regression_from_points(const float grams[CAL_POINTS], float *m_out, float *b_out)
{
    int n = CAL_POINTS;
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;

    for (int i = 0; i < n; i++) {
        double x = g_cal_times[i];
        double y = grams[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double denom = (n * sum_x2) - (sum_x * sum_x);
    if (fabs(denom) < 1e-9) {
        return false;
    }

    double m = ((n * sum_xy) - (sum_x * sum_y)) / denom;
    double b = (sum_y - (m * sum_x)) / n;

    if (m <= 0.0) {
        return false;
    }

    *m_out = (float)m;
    *b_out = (float)b;
    return true;
}

// =========================
// Tareas
// =========================
typedef struct {
    float seconds;
} pump_task_args_t;

static void finish_busy_mode(void)
{
    g_busy = false;
    g_mode = MODE_IDLE;
    bomba_set(false);
}

static void cal_pump_task(void *arg)
{
    pump_task_args_t *p = (pump_task_args_t *)arg;
    float sec = p->seconds;
    free(p);

    if (sec < 0.05f) sec = 0.05f;

    g_busy = true;
    g_mode = MODE_CAL_PUMP;
    g_stop_requested = false;
    snprintf(g_cycle_state, sizeof(g_cycle_state), "Prueba de calibración en curso");

    bomba_set(true);

    int total_ms = (int)(sec * 1000.0f);
    if (total_ms < 50) total_ms = 50;
    int loops = total_ms / 50;
    if (loops < 1) loops = 1;

    for (int i = 0; i < loops; i++) {
        if (g_stop_requested) {
            snprintf(g_cycle_state, sizeof(g_cycle_state), "Prueba detenida");
            finish_busy_mode();
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Prueba de calibración completada");
    finish_busy_mode();
    vTaskDelete(NULL);
}

static void cycle_task(void *arg)
{
    g_busy = true;
    g_mode = MODE_AUTO_CYCLE;
    g_stop_requested = false;

    if (g_selected_liquid < 0 || g_selected_liquid >= MAX_LIQUIDS || !g_profiles[g_selected_liquid].valid) {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "Selecciona un líquido válido");
        finish_busy_mode();
        vTaskDelete(NULL);
        return;
    }

    float fill_sec = ml_to_seconds_profile(g_selected_liquid, g_target_ml);
    if (fill_sec < 0.05f) fill_sec = 0.05f;
    g_last_fill_seconds = fill_sec;

    int cycle_dir = CYCLE_DIR_FIXED;
    int cycle_sps = (g_stepper_sps * CYCLE_SPEED_PERCENT) / 100;
    if (cycle_sps < 1) cycle_sps = 1;

    int64_t cycle_start_us = esp_timer_get_time();

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Buscando botella");
    bool sensor_hit = false;
    stepper_run_until_sensor_blocking(cycle_dir, cycle_sps, &g_stop_requested, &sensor_hit);

    if (g_stop_requested) {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "Cancelado");
        finish_busy_mode();
        stepper_idle_with_dir(cycle_dir);
        vTaskDelete(NULL);
        return;
    }

    if (!sensor_hit) {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "Detenido sin botella");
        finish_busy_mode();
        stepper_idle_with_dir(cycle_dir);
        vTaskDelete(NULL);
        return;
    }

    stepper_idle_with_dir(cycle_dir);

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Botella detectada - espera 1 s");
    for (int i = 0; i < (DETECT_TO_PUMP_DELAY_MS / 100); i++) {
        if (g_stop_requested) {
            snprintf(g_cycle_state, sizeof(g_cycle_state), "Cancelado");
            finish_busy_mode();
            stepper_idle_with_dir(cycle_dir);
            vTaskDelete(NULL);
            return;
        }
        stepper_idle_with_dir(cycle_dir);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Llenando");
    bomba_set(true);

    int total_ms = (int)(fill_sec * 1000.0f);
    if (total_ms < 50) total_ms = 50;
    int loops = total_ms / 50;
    if (loops < 1) loops = 1;

    for (int i = 0; i < loops; i++) {
        if (g_stop_requested) {
            snprintf(g_cycle_state, sizeof(g_cycle_state), "Cancelado durante llenado");
            finish_busy_mode();
            stepper_idle_with_dir(cycle_dir);
            vTaskDelete(NULL);
            return;
        }
        stepper_idle_with_dir(cycle_dir);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    bomba_set(false);
    stepper_idle_with_dir(cycle_dir);
    vTaskDelay(pdMS_TO_TICKS(150));

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Avanzando 400 pasos");
    stepper_run_steps_blocking(CYCLE_EXTRA_STEPS, cycle_dir, cycle_sps, &g_stop_requested);

    if (g_stop_requested) {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "Cancelado");
    } else {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "Ciclo completado");
        g_cycle_count++;
        g_last_cycle_ml = g_target_ml;
        int64_t cycle_end_us = esp_timer_get_time();
        g_last_cycle_total_seconds = (float)(cycle_end_us - cycle_start_us) / 1000000.0f;
    }

    finish_busy_mode();
    stepper_idle_with_dir(cycle_dir);
    vTaskDelete(NULL);
}

// =========================
// HTTP helpers
// =========================
static int get_query_int(httpd_req_t *req, const char *key, int def)
{
    char buf[256];
    char val[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK) {
            return atoi(val);
        }
    }
    return def;
}

static float get_query_float(httpd_req_t *req, const char *key, float def)
{
    char buf[256];
    char val[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, key, val, sizeof(val)) == ESP_OK) {
            return strtof(val, NULL);
        }
    }
    return def;
}

static esp_err_t simple_ok(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static void send_chunk(httpd_req_t *req, const char *s)
{
    httpd_resp_sendstr_chunk(req, s);
}

// =========================
// Páginas HTML
// =========================
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    send_chunk(req,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>OVFiller</title>"
        "<style>"
        ":root{--bg-primary:#121212;--bg-secondary:#1e1e1e;--border-ui:#333333;--accent:#00e5ff;--error:#d50000;--success:#00c853;--warning:#ffd600;--text-primary:#e0e0e0;--text-secondary:#9e9e9e;--font-ui:'Inter','Roboto',sans-serif;--font-data:'JetBrains Mono','Roboto Mono',monospace;--radius-card:8px;--radius-btn:4px;}"
        "*{box-sizing:border-box}body{margin:0;background:var(--bg-primary);color:var(--text-primary);font-family:var(--font-ui);min-height:100vh}"
        ".app{max-width:1400px;margin:0 auto;padding:20px}.topbar{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:20px;padding:16px 20px;background:var(--bg-secondary);border:1px solid var(--border-ui);border-radius:var(--radius-card)}"
        ".brand h1{margin:0;font-size:1.25rem;font-weight:600}.brand p{margin:6px 0 0;color:var(--text-secondary);font-size:.9rem}"
        ".system-pill{display:inline-flex;align-items:center;gap:8px;padding:10px 14px;border-radius:999px;border:1px solid rgba(0,200,83,.35);background:rgba(0,200,83,.08);color:var(--success);font-size:.8rem;font-weight:600;text-transform:uppercase;letter-spacing:.08em}"
        ".layout{display:grid;grid-template-columns:1.2fr .9fr;gap:20px}.column{display:grid;gap:20px}.card{background:var(--bg-secondary);border:1px solid var(--border-ui);border-radius:var(--radius-card);overflow:hidden}"
        ".card-header{display:flex;justify-content:space-between;align-items:center;gap:16px;padding:16px 18px;border-bottom:1px solid var(--border-ui)}"
        ".card-title{margin:0;font-size:1.05rem;font-weight:600}.card-subtitle{margin:6px 0 0;color:var(--text-secondary);font-size:.82rem;text-transform:uppercase;letter-spacing:.08em}"
        ".card-body{padding:18px}.metrics-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px}.metric{border:1px solid var(--border-ui);border-radius:8px;background:rgba(255,255,255,.015);padding:16px}"
        ".metric-label{color:var(--text-secondary);font-size:.78rem;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px}.metric-value{font-family:var(--font-data);font-size:2rem;font-weight:700;line-height:1}.metric-value.small{font-size:1.35rem}.metric-status{margin-top:10px;font-size:.85rem;color:var(--text-secondary)}"
        ".status-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.status-tile{border:1px solid var(--border-ui);border-left:4px solid var(--accent);border-radius:8px;padding:14px;background:rgba(255,255,255,.015)}"
        ".status-tile.success{border-left-color:var(--success)}.status-tile.warning{border-left-color:var(--warning)}.status-tile.critical{border-left-color:var(--error)}"
        ".status-name{font-size:.8rem;color:var(--text-secondary);text-transform:uppercase;letter-spacing:.1em;margin-bottom:8px}.status-value{font-family:var(--font-data);font-size:1.1rem;font-weight:700}"
        ".field-row{display:grid;grid-template-columns:1fr 1fr;gap:14px}.field{display:grid;gap:8px}.field label{color:var(--text-secondary);font-size:.78rem;text-transform:uppercase;letter-spacing:.1em}"
        ".input,.select{width:100%;height:46px;border-radius:6px;border:1px solid var(--border-ui);background:#151515;color:var(--text-primary);font-family:var(--font-data);font-size:1rem;padding:0 14px;outline:none}"
        ".button-row{display:flex;flex-wrap:wrap;gap:10px}.btn{min-height:44px;padding:0 16px;border:0;border-radius:var(--radius-btn);cursor:pointer;font-family:var(--font-ui);font-size:.85rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em}"
        ".btn-primary{background:var(--accent);color:#121212}.btn-success{background:var(--success);color:#121212}.btn-warning{background:var(--warning);color:#121212}.btn-danger{background:var(--error);color:#fff}.btn-secondary{background:transparent;color:var(--text-primary);border:1px solid var(--border-ui)}"
        ".nav-link{color:var(--accent);font-size:.82rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;text-decoration:none}"
        "@media (max-width:1050px){.layout{grid-template-columns:1fr}}@media (max-width:640px){.metrics-grid,.status-grid,.field-row{grid-template-columns:1fr}.topbar{flex-direction:column;align-items:flex-start}.button-row{flex-direction:column}.btn{width:100%}}"
        "</style></head><body><div class='app'>"
        "<header class='topbar'><div class='brand'><h1>OVFILLER / INDUSTRIAL CONTROL PANEL</h1><p>Dosificación automática por líquido calibrado</p></div><div class='system-pill'>Sistema listo</div></header>"
        "<main class='layout'><section class='column'>"
    );

    send_chunk(req,
        "<article class='card'><div class='card-header'><div><h2 class='card-title'>Métricas de operación</h2><p class='card-subtitle'>Lecturas principales del sistema</p></div><a class='nav-link' href='/cal'>Ir a calibración</a></div><div class='card-body'><div class='metrics-grid'>"
        "<div class='metric'><div class='metric-label'>Líquido seleccionado</div><div class='metric-value small' id='m_liq'>...</div><div class='metric-status'>Perfil activo en memoria</div></div>"
        "<div class='metric'><div class='metric-label'>Tiempo calculado</div><div class='metric-value' id='m_time'>...</div><div class='metric-status'>Derivado del perfil calibrado</div></div>"
        "<div class='metric'><div class='metric-label'>Cantidad solicitada</div><div class='metric-value' id='m_ml'>...</div><div class='metric-status'>Objetivo del ciclo actual</div></div>"
        "<div class='metric'><div class='metric-label'>Último ciclo</div><div class='metric-value' id='m_last_cycle'>...</div><div class='metric-status'>Desde inicio hasta fin del ciclo</div></div>"
        "</div></div></article>"
    );

    send_chunk(req,
        "<article class='card'><div class='card-header'><div><h2 class='card-title'>Estado del sistema</h2><p class='card-subtitle'>Monitoreo en tiempo real</p></div></div><div class='card-body'><div class='status-grid'>"
        "<div class='status-tile success'><div class='status-name'>Sensor botella</div><div class='status-value' id='st_sensor'>...</div></div>"
        "<div class='status-tile success'><div class='status-name'>Bomba</div><div class='status-value' id='st_bomba'>...</div></div>"
        "<div class='status-tile warning'><div class='status-name'>Ciclo</div><div class='status-value' id='st_cycle'>...</div></div>"
        "<div class='status-tile success'><div class='status-name'>Motor paso a paso</div><div class='status-value'>Quieto</div></div>"
        "<div class='status-tile success'><div class='status-name'>Líquido</div><div class='status-value' id='st_liq_status'>...</div></div>"
        "<div class='status-tile critical'><div class='status-name'>Paro / Falla</div><div class='status-value'>Sin alarmas</div></div>"
        "</div></div></article>"
    );

    send_chunk(req,
        "</section><aside class='column'>"
        "<article class='card'><div class='card-header'><div><h2 class='card-title'>Control de producción</h2><p class='card-subtitle'>Parámetros del ciclo</p></div></div><div class='card-body'>"
        "<div class='field-row'>"
        "<div class='field'><label>Líquido</label><select id='liq' class='select' onchange='setLiquid()'>"
    );

    char opt[256];
    for (int i = 0; i < MAX_LIQUIDS; i++) {
        if (!g_profiles[i].valid) continue;
        snprintf(opt, sizeof(opt),
            "<option value='%d'%s>%s</option>",
            i,
            (i == g_selected_liquid) ? " selected" : "",
            g_profiles[i].name
        );
        send_chunk(req, opt);
    }

    send_chunk(req,
        "</select></div>"
        "<div class='field'><label>Cantidad solicitada (mL)</label><input id='ml' class='input' type='number' min='1' step='0.1' value='200'></div>"
        "</div>"
        "<div class='field-row' style='margin-top:14px'>"
        "<div class='field'><label>Tiempo calculado (s)</label><input id='calc_time' class='input' type='text' value='...' readonly></div>"
        "<div class='field'><label>Estado actual</label><input id='state_now' class='input' type='text' value='LISTO' readonly></div>"
        "</div>"
        "<div class='button-row' style='margin-top:16px'>"
        "<button class='btn btn-warning' onclick='setMl()'>Aplicar</button>"
        "<button class='btn btn-primary' onclick=\"cmd('/cycle/start')\">Iniciar ciclo</button>"
        "<button class='btn btn-danger' onclick=\"cmd('/cycle/stop')\">Parar ciclo</button>"
        "</div></div></article>"
    );

    send_chunk(req,
        "<article class='card'><div class='card-header'><div><h2 class='card-title'>Resumen de ciclos</h2><p class='card-subtitle'>Acumulados de producción</p></div></div><div class='card-body'><div class='metrics-grid'>"
        "<div class='metric'><div class='metric-label'>Ciclos realizados</div><div class='metric-value' id='sum_count'>0</div><div class='metric-status'>Total desde último reset</div></div>"
        "<div class='metric'><div class='metric-label'>Último tiempo de ciclo</div><div class='metric-value' id='sum_time'>0.00 s</div><div class='metric-status'>Desde inicio hasta final</div></div>"
        "<div class='metric'><div class='metric-label'>Último llenado</div><div class='metric-value' id='sum_ml'>0.00 mL</div><div class='metric-status'>Cantidad dosificada</div></div>"
        "<div class='metric'><div class='metric-label'>Estado del contador</div><div class='metric-value small'>ACTIVO</div><div class='metric-status'>Acumulación habilitada</div></div>"
        "</div><div class='button-row' style='margin-top:16px'><button class='btn btn-warning' onclick=\"cmd('/stats/reset')\">Resetear datos</button></div></div></article>"
    );

    send_chunk(req,
        "</aside></main>"
        "<script>"
        "let editingMl=false;"
        "async function cmd(url){try{await fetch(url,{cache:'no-store'});update();}catch(e){console.log(e);}}"
        "async function setLiquid(){const v=document.getElementById('liq').value; await cmd('/setliquid?id='+encodeURIComponent(v));}"
        "async function setMl(){const v=document.getElementById('ml').value; await cmd('/setml?ml='+encodeURIComponent(v));}"
        "async function update(){"
        " try{"
        "  const r=await fetch('/status',{cache:'no-store'});"
        "  const s=await r.json();"
        "  document.getElementById('m_liq').innerText=s.selected_name;"
        "  document.getElementById('m_time').innerText=s.fill_seconds.toFixed(2)+' s';"
        "  document.getElementById('m_ml').innerText=s.target_ml.toFixed(2)+' mL';"
        "  document.getElementById('m_last_cycle').innerText=s.last_cycle_seconds.toFixed(2)+' s';"
        "  document.getElementById('st_sensor').innerText=s.sensor_obstacle?'Detectada':'Sin botella';"
        "  document.getElementById('st_bomba').innerText=s.bomba_on?'Encendida':'Apagada';"
        "  document.getElementById('st_cycle').innerText=s.busy?'En ejecución':'Detenido';"
        "  document.getElementById('st_liq_status').innerText=s.selected_name;"
        "  document.getElementById('calc_time').value=s.fill_seconds.toFixed(2);"
        "  document.getElementById('state_now').value=s.cycle_state;"
        "  document.getElementById('sum_count').innerText=s.cycle_count;"
        "  document.getElementById('sum_time').innerText=s.last_cycle_seconds.toFixed(2)+' s';"
        "  document.getElementById('sum_ml').innerText=s.last_cycle_ml.toFixed(2)+' mL';"
        "  if(!editingMl){document.getElementById('ml').value=s.target_ml.toFixed(2);}"
        "  document.getElementById('liq').value=String(s.selected_id);"
        " }catch(e){console.log(e);}"
        "}"
        "const mlEl=document.getElementById('ml');"
        "mlEl.addEventListener('focus', ()=>{editingMl=true;});"
        "mlEl.addEventListener('blur', ()=>{editingMl=false;});"
        "mlEl.addEventListener('input', ()=>{editingMl=true;});"
        "setInterval(update,1000);update();"
        "</script></body></html>"
    );

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t cal_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    send_chunk(req,
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Calibración</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#121212;margin:0;padding:20px;color:#e0e0e0}"
        ".box{background:#1e1e1e;border:1px solid #333;border-radius:8px;padding:16px;margin-bottom:16px;max-width:860px}"
        ".row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}"
        "button{padding:10px 14px;border:0;border-radius:4px;cursor:pointer;font-size:14px;font-weight:700;text-transform:uppercase}"
        ".on{background:#00c853;color:#121212}.off{background:#d50000;color:#fff}.warn{background:#ffd600;color:#121212}.info{background:#00e5ff;color:#121212}"
        ".state{padding:8px 10px;background:#151515;border-radius:8px;margin:6px 0;color:#9e9e9e}"
        "input,select{padding:8px;border:1px solid #333;border-radius:6px;background:#151515;color:#e0e0e0}"
        "a{color:#00e5ff;text-decoration:none;font-weight:bold}.label{min-width:150px;font-weight:bold;color:#9e9e9e;text-transform:uppercase;font-size:.8rem;letter-spacing:.1em}"
        "</style></head><body>"
        "<div class='box'><h2>Calibración de líquidos</h2><div><a href='/'>Volver a operación</a></div><div class='state'>Introduce nombre, densidad y los 5 pesos medidos. Luego guarda el líquido.</div></div>"
        "<div class='box'><div class='row'><div class='label'>Slot</div><select id='slot'>"
    );

    char opt[256];
    for (int i = 0; i < MAX_LIQUIDS; i++) {
        const char *nm = g_profiles[i].valid ? g_profiles[i].name : "(vacío)";
        snprintf(opt, sizeof(opt), "<option value='%d'>Slot %d - %s</option>", i, i + 1, nm);
        send_chunk(req, opt);
    }

    send_chunk(req,
        "</select></div>"
        "<div class='row'><div class='label'>Nombre</div><input id='name' type='text' maxlength='23' placeholder='Ej. Vainilla'></div>"
        "<div class='row'><div class='label'>Densidad (g/mL)</div><input id='density' type='number' step='0.001' min='0.100' value='1.000'></div>"
        "</div>"
    );

    for (int i = 0; i < CAL_POINTS; i++) {
        char line[512];
        snprintf(line, sizeof(line),
            "<div class='box'><div class='row'>"
            "<div class='label'>Corrida %d: %.0f s</div>"
            "<button class='info' onclick=\"cmd('/cal/run?id=%d')\">Iniciar</button>"
            "<button class='off' onclick=\"cmd('/pump/stop')\">Parar</button>"
            "<input id='g%d' type='number' step='0.1' min='0' placeholder='gramos medidos'>"
            "</div></div>",
            i + 1, g_cal_times[i], i, i
        );
        send_chunk(req, line);
    }

    send_chunk(req,
        "<div class='box'><button class='on' onclick='saveLiquid()'>Calcular y guardar líquido</button><div class='state' id='st_msg'>Estado: listo</div></div>"
        "<script>"
        "async function cmd(url){try{await fetch(url,{cache:'no-store'});update();}catch(e){console.log(e);}}"
        "async function saveLiquid(){"
        " const slot=document.getElementById('slot').value;"
        " const name=document.getElementById('name').value;"
        " const density=document.getElementById('density').value;"
        " const g0=document.getElementById('g0').value;"
        " const g1=document.getElementById('g1').value;"
        " const g2=document.getElementById('g2').value;"
        " const g3=document.getElementById('g3').value;"
        " const g4=document.getElementById('g4').value;"
        " const url='/cal/save?slot='+encodeURIComponent(slot)"
        " +'&name='+encodeURIComponent(name)"
        " +'&density='+encodeURIComponent(density)"
        " +'&g0='+encodeURIComponent(g0)"
        " +'&g1='+encodeURIComponent(g1)"
        " +'&g2='+encodeURIComponent(g2)"
        " +'&g3='+encodeURIComponent(g3)"
        " +'&g4='+encodeURIComponent(g4);"
        " try{await fetch(url,{cache:'no-store'}); document.getElementById('st_msg').innerText='Estado: líquido guardado';}catch(e){console.log(e);}"
        "}"
        "async function update(){"
        " try{ const r=await fetch('/status',{cache:'no-store'}); const s=await r.json();"
        " document.getElementById('st_msg').innerText='Estado: '+s.cycle_state; }catch(e){console.log(e);} "
        "}"
        "setInterval(update,1000); update();"
        "</script></body></html>"
    );

    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// =========================
// JSON status
// =========================
static esp_err_t status_get_handler(httpd_req_t *req)
{
    char resp[896];

    const char *sel_name = "(sin líquido)";
    float density = 0.0f;
    if (g_selected_liquid >= 0 && g_selected_liquid < MAX_LIQUIDS && g_profiles[g_selected_liquid].valid) {
        sel_name = g_profiles[g_selected_liquid].name;
        density = g_profiles[g_selected_liquid].density_g_ml;
    }

    float fill_seconds = ml_to_seconds_profile(g_selected_liquid, g_target_ml);
    g_last_fill_seconds = fill_seconds;

    snprintf(resp, sizeof(resp),
        "{"
        "\"sensor_obstacle\":%s,"
        "\"busy\":%s,"
        "\"bomba_on\":%s,"
        "\"selected_id\":%d,"
        "\"selected_name\":\"%s\","
        "\"density\":%.4f,"
        "\"target_ml\":%.3f,"
        "\"fill_seconds\":%.3f,"
        "\"cycle_count\":%" PRIu32 ","
        "\"last_cycle_seconds\":%.3f,"
        "\"last_cycle_ml\":%.3f,"
        "\"cycle_state\":\"%s\""
        "}",
        sensor_obstacle_raw() ? "true" : "false",
        g_busy ? "true" : "false",
        g_bomba_on ? "true" : "false",
        g_selected_liquid,
        sel_name,
        density,
        g_target_ml,
        fill_seconds,
        g_cycle_count,
        g_last_cycle_total_seconds,
        g_last_cycle_ml,
        g_cycle_state
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// =========================
// Handlers de operación
// =========================
static esp_err_t setliquid_handler(httpd_req_t *req)
{
    int id = get_query_int(req, "id", g_selected_liquid);
    if (id >= 0 && id < MAX_LIQUIDS && g_profiles[id].valid) {
        g_selected_liquid = id;
        save_profiles_to_nvs();
    }
    return simple_ok(req);
}

static esp_err_t setml_handler(httpd_req_t *req)
{
    float ml = get_query_float(req, "ml", g_target_ml);
    if (ml < 1.0f) ml = 1.0f;
    if (ml > 10000.0f) ml = 10000.0f;
    g_target_ml = ml;
    return simple_ok(req);
}

static esp_err_t cycle_start_handler(httpd_req_t *req)
{
    if (!g_busy) {
        xTaskCreate(cycle_task, "cycle_task", 4096, NULL, 5, NULL);
    }
    return simple_ok(req);
}

static esp_err_t cycle_stop_handler(httpd_req_t *req)
{
    g_stop_requested = true;
    snprintf(g_cycle_state, sizeof(g_cycle_state), "Parando...");
    return simple_ok(req);
}

static esp_err_t stats_reset_handler(httpd_req_t *req)
{
    g_cycle_count = 0;
    g_last_cycle_total_seconds = 0.0f;
    g_last_cycle_ml = 0.0f;
    snprintf(g_cycle_state, sizeof(g_cycle_state), "Datos de ciclos reseteados");
    return simple_ok(req);
}

// =========================
// Handlers de calibración
// =========================
static esp_err_t cal_run_handler(httpd_req_t *req)
{
    int id = get_query_int(req, "id", -1);
    if (id < 0 || id >= CAL_POINTS) {
        return simple_ok(req);
    }

    if (!g_busy) {
        pump_task_args_t *args = malloc(sizeof(pump_task_args_t));
        if (args) {
            args->seconds = g_cal_times[id];
            xTaskCreate(cal_pump_task, "cal_pump_task", 3072, args, 5, NULL);
        }
    }
    return simple_ok(req);
}

static esp_err_t cal_save_handler(httpd_req_t *req)
{
    int slot = get_query_int(req, "slot", -1);
    if (slot < 0 || slot >= MAX_LIQUIDS) {
        return simple_ok(req);
    }

    char query[1024];
    char name[NAME_LEN] = {0};
    char tmp[64];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK) {
            strncpy(name, "Liquido", sizeof(name) - 1);
        }
    } else {
        strncpy(name, "Liquido", sizeof(name) - 1);
    }

    float density = get_query_float(req, "density", 1.0f);
    if (density < 0.1f) density = 1.0f;

    float grams[CAL_POINTS] = {0};
    for (int i = 0; i < CAL_POINTS; i++) {
        snprintf(tmp, sizeof(tmp), "g%d", i);
        grams[i] = get_query_float(req, tmp, 0.0f);
        if (grams[i] <= 0.0f) {
            snprintf(g_cycle_state, sizeof(g_cycle_state), "Faltan datos de calibración");
            return simple_ok(req);
        }
    }

    float m = 0.0f, b = 0.0f;
    if (!calculate_regression_from_points(grams, &m, &b)) {
        snprintf(g_cycle_state, sizeof(g_cycle_state), "No se pudo calcular la regresión");
        return simple_ok(req);
    }

    memset(&g_profiles[slot], 0, sizeof(g_profiles[slot]));
    strncpy(g_profiles[slot].name, name, NAME_LEN - 1);
    g_profiles[slot].m_gps = m;
    g_profiles[slot].b_g = b;
    g_profiles[slot].density_g_ml = density;
    g_profiles[slot].valid = true;

    if (!g_profiles[g_selected_liquid].valid) {
        g_selected_liquid = slot;
    }

    save_profiles_to_nvs();

    snprintf(g_cycle_state, sizeof(g_cycle_state), "Líquido guardado: %s | m=%.3f g/s | b=%.3f g", g_profiles[slot].name, m, b);
    return simple_ok(req);
}

// =========================
// Stop común
// =========================
static esp_err_t pump_stop_handler(httpd_req_t *req)
{
    g_stop_requested = true;
    bomba_set(false);
    g_busy = false;
    g_mode = MODE_IDLE;
    snprintf(g_cycle_state, sizeof(g_cycle_state), "Detenido");
    return simple_ok(req);
}

// =========================
// WiFi AP
// =========================
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP iniciado");
    ESP_LOGI(TAG, "SSID: %s", WIFI_SSID);
    ESP_LOGI(TAG, "PASS: %s", WIFI_PASS);
    ESP_LOGI(TAG, "IP: 192.168.4.1");
}

// =========================
// HTTP server
// =========================
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 18;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {.uri="/", .method=HTTP_GET, .handler=root_get_handler, .user_ctx=NULL};
        httpd_uri_t cal = {.uri="/cal", .method=HTTP_GET, .handler=cal_get_handler, .user_ctx=NULL};
        httpd_uri_t status = {.uri="/status", .method=HTTP_GET, .handler=status_get_handler, .user_ctx=NULL};

        httpd_uri_t setliquid = {.uri="/setliquid", .method=HTTP_GET, .handler=setliquid_handler, .user_ctx=NULL};
        httpd_uri_t setml = {.uri="/setml", .method=HTTP_GET, .handler=setml_handler, .user_ctx=NULL};
        httpd_uri_t cycle_start = {.uri="/cycle/start", .method=HTTP_GET, .handler=cycle_start_handler, .user_ctx=NULL};
        httpd_uri_t cycle_stop = {.uri="/cycle/stop", .method=HTTP_GET, .handler=cycle_stop_handler, .user_ctx=NULL};
        httpd_uri_t stats_reset = {.uri="/stats/reset", .method=HTTP_GET, .handler=stats_reset_handler, .user_ctx=NULL};

        httpd_uri_t cal_run = {.uri="/cal/run", .method=HTTP_GET, .handler=cal_run_handler, .user_ctx=NULL};
        httpd_uri_t cal_save = {.uri="/cal/save", .method=HTTP_GET, .handler=cal_save_handler, .user_ctx=NULL};

        httpd_uri_t pump_stop = {.uri="/pump/stop", .method=HTTP_GET, .handler=pump_stop_handler, .user_ctx=NULL};

        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &cal);
        httpd_register_uri_handler(server, &status);

        httpd_register_uri_handler(server, &setliquid);
        httpd_register_uri_handler(server, &setml);
        httpd_register_uri_handler(server, &cycle_start);
        httpd_register_uri_handler(server, &cycle_stop);
        httpd_register_uri_handler(server, &stats_reset);

        httpd_register_uri_handler(server, &cal_run);
        httpd_register_uri_handler(server, &cal_save);

        httpd_register_uri_handler(server, &pump_stop);
    }

    return server;
}

// =========================
// GPIO init
// =========================
static void gpio_init_all(void)
{
    gpio_config_t io_out = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_BOMBA) | (1ULL << PIN_ENROSQUE) | (1ULL << PIN_DIR),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_out);

    gpio_config_t pul_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_PUL),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&pul_cfg);

    gpio_config_t sensor_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_SENSOR),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&sensor_cfg);

    bomba_set(false);
    set_output_active_low(PIN_ENROSQUE, false);
    stepper_idle_strong();
}

// =========================
// app_main
// =========================
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init_all();
    load_profiles_from_nvs();
    wifi_init_softap();
    g_server = start_webserver();

    g_busy = false;
    g_stop_requested = false;
    g_mode = MODE_IDLE;
    g_cycle_count = 0;
    g_last_cycle_total_seconds = 0.0f;
    g_last_cycle_ml = 0.0f;
    snprintf(g_cycle_state, sizeof(g_cycle_state), "Listo");

    ESP_LOGI(TAG, "Sistema listo");
}