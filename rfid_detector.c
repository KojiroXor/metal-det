#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <notification/notification_messages.h>

#define MOVING_AVERAGE_SIZE 16
#define PROGRESS_BAR_WIDTH 100
#define PROGRESS_BAR_HEIGHT 12
#define MAX_DELTA 1500
#define NORMAL_PULSE_WIDTH 2560

typedef enum {
    AlertModeSoundVibro,
    AlertModeSound,
    AlertModeVibro,
    AlertModeNone
} AlertMode;

typedef struct {
    uint16_t moving_average[MOVING_AVERAGE_SIZE];
    uint16_t average_index;
    uint16_t baseline_value;
    uint16_t current_value;
    
    // Variabili per l'auto-calibrazione
    bool is_calibrating;
    uint32_t calib_sum;
    uint16_t calib_count;

    bool running;
    Gui* gui;
    NotificationApp* notification;
    FuriThread* sensor_thread;
    ViewPort* view_port;
    FuriMutex* state_mutex;
    volatile uint32_t capture_sum;
    volatile uint32_t capture_count;
    uint16_t sensitivity;
    AlertMode alert_mode;
    bool show_help;
} MetalDetectorApp;

static void moving_average_init(MetalDetectorApp* app) {
    for(int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        app->moving_average[i] = NORMAL_PULSE_WIDTH;
    }
    app->average_index = 0;
    app->current_value = NORMAL_PULSE_WIDTH;
    app->baseline_value = NORMAL_PULSE_WIDTH;
}

static uint16_t moving_average_update(MetalDetectorApp* app, uint16_t value) {
    app->moving_average[app->average_index] = value;
    app->average_index = (app->average_index + 1) % MOVING_AVERAGE_SIZE;
    
    uint32_t sum = 0;
    for (uint16_t i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        sum += app->moving_average[i];
    }
    return (uint16_t)(sum / MOVING_AVERAGE_SIZE);
}

static void trigger_feedback(MetalDetectorApp* app, uint16_t delta) {
    if (delta < app->sensitivity) return;

    notification_message(app->notification, &sequence_display_backlight_on);

    uint16_t active_delta = delta - app->sensitivity;
    uint16_t range = MAX_DELTA > app->sensitivity ? MAX_DELTA - app->sensitivity : 1;
    uint16_t freq = 100 + ((active_delta * 1900) / range);
    if(freq > 2000) freq = 2000;

    bool play_sound = (app->alert_mode == AlertModeSoundVibro || app->alert_mode == AlertModeSound);
    bool play_vibro = (app->alert_mode == AlertModeSoundVibro || app->alert_mode == AlertModeVibro);

    if (play_sound && furi_hal_speaker_is_mine()) {
        furi_hal_speaker_start(freq, 0.5f);
    }
    
    if (play_vibro) furi_hal_vibro_on(true);
    furi_hal_light_set(LightRed, 255);
    
    furi_delay_ms(30); 
    
    if (play_sound && furi_hal_speaker_is_mine()) {
        furi_hal_speaker_stop();
    }
    
    if (play_vibro) furi_hal_vibro_on(false);
    furi_hal_light_set(LightRed, 0);
}

static void draw_callback(Canvas* canvas, void* context) {
    if (!context) return;
    MetalDetectorApp* app = (MetalDetectorApp*)context;
    
    furi_mutex_acquire(app->state_mutex, FuriWaitForever);
    canvas_clear(canvas);
    
    if (app->show_help) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str(canvas, 2, 12, "Metal Detector Help");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 2, 24, "L/R: Adjust Sens. (Foil = 10-20)");
        canvas_draw_str(canvas, 2, 35, "OK: Cycle Alert Mode");
        canvas_draw_str(canvas, 2, 46, "UP: Toggle Help");
        canvas_draw_str(canvas, 2, 57, "DOWN: Recalibrate Zero");
        furi_mutex_release(app->state_mutex);
        return;
    }

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 12, "Card & Metal Finder");
    
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 100, 10, "? [Up]");
    
    if (app->is_calibrating) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 35, AlignCenter, AlignCenter, "Calibrazione...");
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 48, AlignCenter, AlignCenter, "Tieni fermo, niente metallo");
    } else {
        const char* mode_str = "";
        switch(app->alert_mode) {
            case AlertModeSoundVibro: mode_str = "[S+V]"; break;
            case AlertModeSound:      mode_str = "[SND]"; break;
            case AlertModeVibro:      mode_str = "[VIB]"; break;
            case AlertModeNone:       mode_str = "[---]"; break;
        }
        
        char buffer[48];
        snprintf(buffer, sizeof(buffer), "Sens: %u  Mode: %s", app->sensitivity, mode_str);
        canvas_draw_str(canvas, 2, 28, buffer);
        
        int16_t delta = 0;
        if (app->current_value < app->baseline_value) {
            delta = app->baseline_value - app->current_value;
        }

        int bar_fill = (delta * PROGRESS_BAR_WIDTH) / MAX_DELTA;
        if (bar_fill > PROGRESS_BAR_WIDTH) bar_fill = PROGRESS_BAR_WIDTH;

        canvas_draw_frame(canvas, 5, 40, PROGRESS_BAR_WIDTH + 4, PROGRESS_BAR_HEIGHT);
        if (bar_fill > 0) {
            canvas_draw_box(canvas, 6, 41, bar_fill, PROGRESS_BAR_HEIGHT - 2);
        }
        
        snprintf(buffer, sizeof(buffer), "Signal: %u", delta);
        canvas_draw_str(canvas, 5, 62, buffer);
    }
    
    furi_mutex_release(app->state_mutex);
}

static void input_callback(InputEvent* input_event, void* context) {
    if (!context) return;
    MetalDetectorApp* app = (MetalDetectorApp*)context;

    if (input_event->type == InputTypePress || input_event->type == InputTypeRepeat) {
        furi_mutex_acquire(app->state_mutex, FuriWaitForever);
        switch (input_event->key) {
            case InputKeyBack:
                app->running = false;
                break;
            case InputKeyLeft:
                if (app->sensitivity > 5) app->sensitivity -= 5;
                break;
            case InputKeyRight:
                if (app->sensitivity < 1400) app->sensitivity += 5;
                break;
            case InputKeyOk:
                if (input_event->type == InputTypePress) {
                    app->alert_mode = (app->alert_mode + 1) % 4;
                }
                break;
            case InputKeyUp:
                if (input_event->type == InputTypePress) {
                    app->show_help = !app->show_help;
                }
                break;
            case InputKeyDown:
                if (input_event->type == InputTypePress) {
                    // Avvia ricalibrazione manuale
                    app->is_calibrating = true;
                    app->calib_sum = 0;
                    app->calib_count = 0;
                }
                break;
            default:
                break;
        }
        furi_mutex_release(app->state_mutex);
    }
}

static void rfid_capture_callback(bool level, uint32_t duration, void* context) {
    MetalDetectorApp* app = (MetalDetectorApp*)context;
    if (level) {
        app->capture_sum += duration;
        app->capture_count++;
    }
}

static int32_t sensor_worker_thread(void* context) {
    MetalDetectorApp* app = (MetalDetectorApp*)context;
    if (!app) return -1;
    
    bool speaker_acquired = furi_hal_speaker_acquire(1000);
    furi_hal_rfid_tim_read_start(125000, 0.5);

    while (app->running) {
        app->capture_sum = 0;
        app->capture_count = 0;
        
        furi_hal_rfid_tim_read_capture_start(rfid_capture_callback, app);
        furi_delay_ms(50);
        furi_hal_rfid_tim_read_capture_stop();
        
        uint32_t sum = app->capture_sum;
        uint32_t count = app->capture_count;
        
        uint16_t reg_val = 0;
        if (count > 50) {
            reg_val = (sum * 10) / count;
        } else {
            reg_val = 0; // Trigger forte in assenza di segnale
        }
        
        furi_mutex_acquire(app->state_mutex, FuriWaitForever);
        app->current_value = moving_average_update(app, reg_val);
        
        if (app->is_calibrating) {
            // Ignoriamo i cali brutali durante la calibrazione
            if (reg_val > 0) {
                app->calib_sum += app->current_value;
                app->calib_count++;
                
                // Fine calibrazione dopo circa 1 secondo (20 campioni da 50ms)
                if (app->calib_count >= 20) {
                    app->baseline_value = (uint16_t)(app->calib_sum / app->calib_count);
                    app->is_calibrating = false;
                }
            }
        } else {
            int16_t delta = 0;
            if (app->current_value < app->baseline_value) {
                delta = app->baseline_value - app->current_value;
            }
            trigger_feedback(app, delta);
        }
        
        furi_mutex_release(app->state_mutex);
        furi_delay_ms(50);
    }
    
    furi_hal_rfid_tim_read_stop();
    furi_hal_rfid_pins_reset();
    if (speaker_acquired) furi_hal_speaker_release();
    
    return 0;
}

int32_t metal_detector_app(void* p) {
    UNUSED(p);
    
    MetalDetectorApp* app = malloc(sizeof(MetalDetectorApp));
    if (!app) return -1;
    
    memset(app, 0, sizeof(MetalDetectorApp));
    app->running = true;
    app->sensitivity = 15; // Set iniziale basso pronto per i foil
    app->is_calibrating = true; // Calibrazione automatica al lancio
    app->calib_sum = 0;
    app->calib_count = 0;
    
    moving_average_init(app);

    app->gui = furi_record_open(RECORD_GUI);
    if (!app->gui) { free(app); return -1; }
    
    app->notification = furi_record_open(RECORD_NOTIFICATION);
    if (!app->notification) { furi_record_close(RECORD_GUI); free(app); return -1; }

    app->state_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    if (!app->state_mutex) {
        furi_record_close(RECORD_GUI); furi_record_close(RECORD_NOTIFICATION);
        free(app); return -1;
    }

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    app->sensor_thread = furi_thread_alloc();
    furi_thread_set_name(app->sensor_thread, "RFID_Sensor");
    furi_thread_set_stack_size(app->sensor_thread, 1024);
    furi_thread_set_callback(app->sensor_thread, sensor_worker_thread);
    furi_thread_set_context(app->sensor_thread, (void*)app);
    furi_thread_start(app->sensor_thread);

    while (app->running) {
        view_port_update(app->view_port);
        furi_delay_ms(50);
    }

    app->running = false;
    furi_thread_join(app->sensor_thread);
    furi_thread_free(app->sensor_thread);

    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_mutex_free(app->state_mutex);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    free(app);
    
    return 0;
}
#define SAMPLE_MS      80UL        /* ms between signal samples */
#define CALIB_FRAMES   12          /* frames for auto-calibration */
#define CARD_CALIB_F   (CALIB_FRAMES * 2)

/* ── Signal processing ────────────────────────────────────── */
/* NORM_SCALE = transitions per sample that fills the bar at SENS_DEF.
   Tune this constant on real hardware if the bar is always pegged
   or always empty. Start high and lower until response appears. */
#define NORM_SCALE     3000.0f
#define BASE_EMA_A     0.04f   /* slow EMA — baseline drifts quietly */
#define SIG_EMA_A      0.30f   /* fast EMA — signal follows quickly  */

/* ── Sensitivity ──────────────────────────────────────────── */
#define SENS_MIN       1
#define SENS_MAX       8
#define SENS_DEF       4
#define CARD_SENS_DEF  7

/* ── Card / foil detection ────────────────────────────────── */
#define CARD_HOLD_MS   350         /* sustain time to confirm card */
#define CARD_THRESH    0.05f       /* normalised threshold for card */

/* ── Audio ────────────────────────────────────────────────── */
#define SND_LO         200.0f
#define SND_HI         2000.0f
#define SND_VOL        0.75f

/* ── Vibration (pulsed) ───────────────────────────────────── */
#define VIB_ON_MS      80
#define VIB_PERIOD_LO  550         /* ms period at weak signal */
#define VIB_PERIOD_HI  80          /* ms period at strong signal */

/* ── Display geometry (128 × 64) ─────────────────────────── */
#define SCR_W          128
#define SCR_H          64
#define HDR_SEP_Y      8           /* y of separator under header */
#define GRAPH_TOP      (HDR_SEP_Y + 1)  /* y=9  */
#define GRAPH_BOT      51          /* y=51 */
#define GRAPH_H        (GRAPH_BOT - GRAPH_TOP)  /* 42 px */
#define HIST_LEN       22
#define HIST_BAR_W     3
#define HIST_STEP      (HIST_BAR_W + 1)         /* 4 px */
#define HIST_W         (HIST_LEN * HIST_STEP)   /* 88 px */
#define CUR_BAR_X      (HIST_W + 4)             /* x=92 */
#define CUR_BAR_W      (SCR_W - CUR_BAR_X - 1) /* 35 px */
#define BOT_SEP_Y      (GRAPH_BOT + 2)          /* y=53 */
#define STATUS_Y       (SCR_H - 2)              /* y=62, text baseline */

/* ── Types ────────────────────────────────────────────────── */
typedef enum { FbSV = 0, FbS, FbV, FbOff, FbCount } FbMode;
typedef enum { ModeM = 0, ModeC, ModeCount } AppMode;

typedef struct {
    /* ISR-side — written from ISR, read atomically in main thread */
    volatile uint32_t isr_cnt;

    /* Signal processing (main thread only) */
    uint32_t  last_cnt;
    float     baseline;      /* EMA of idle transitions / sample */
    float     sig_ema;       /* EMA of |delta − baseline| × sens  */
    float     sig_norm;      /* normalised 0..1                    */
    float     hist[HIST_LEN];
    uint8_t   hist_pos;
    uint32_t  calib_f;       /* calibration frame counter          */

    /* Card-finder state */
    bool      card_cal;      /* calibration complete flag          */
    float     card_base;     /* baseline transitions in card mode  */
    uint32_t  card_cal_f;
    float     card_sig;      /* EMA signal in card mode            */
    bool      card_found;    /* card signal sustained over thresh  */
    uint32_t  card_found_t;  /* tick when card_found first set     */
    bool      card_notif;    /* one-shot flag: notification sent   */
    bool      pending_notif; /* deferred: notify outside mutex     */

    /* Settings */
    uint8_t   sens;
    FbMode    fb;
    AppMode   mode;
    bool      help;

    /* Feedback hardware state */
    bool      spk;           /* speaker acquired                   */
    bool      vib;           /* vibro currently on                 */
    bool      vib_pulse;     /* in the ON phase of a pulse         */
    uint32_t  vib_t;         /* tick of last vibro phase change    */

    /* FURI objects */
    Gui*              gui;
    ViewPort*         vp;
    FuriMessageQueue* q;
    FuriMutex*        mtx;
    NotificationApp*  notif;

    bool running;
} App;

/* ── Global pointer accessible from ISR ─────────────────── */
static volatile App* g_app = NULL;

/* ================================================================
   ISR — called on every comparator edge
   Keep it tiny: only atomic increment.
   ================================================================ */
static void comp_cb(bool level, void* ctx) {
    UNUSED(level);
    UNUSED(ctx);
    if(g_app) g_app->isr_cnt++;
}

/* ================================================================
   RFID hardware start / stop
   ================================================================ */
static void hw_start(void) {
    furi_hal_rfid_pins_reset();
    furi_hal_rfid_tim_read_start(LF_FREQ, LF_DUTY);
    furi_hal_rfid_comp_set_callback(comp_cb, NULL);
    furi_hal_rfid_comp_start();
    FURI_LOG_I(TAG, "Carrier ON @ %.0f Hz", (double)LF_FREQ);
}

static void hw_stop(void) {
    furi_hal_rfid_comp_stop();
    furi_hal_rfid_comp_set_callback(NULL, NULL);
    furi_hal_rfid_tim_read_stop();
    furi_hal_rfid_pins_reset();
    FURI_LOG_I(TAG, "Carrier OFF");
}

/* ================================================================
   Signal update — called every SAMPLE_MS, inside mutex
   ================================================================ */
static void sig_update(App* a) {
    /* Atomic read on Cortex-M4 (aligned uint32 read is single-cycle) */
    uint32_t now_cnt = a->isr_cnt;
    uint32_t delta   = now_cnt - a->last_cnt;
    a->last_cnt      = now_cnt;

    float s = (float)a->sens / (float)SENS_DEF; /* sensitivity multiplier */

    /* ── Metal-detector calibration / tracking ─────────────── */
    if(a->calib_f < CALIB_FRAMES) {
        float al = (a->calib_f == 0) ? 1.0f : 0.25f;
        a->baseline = a->baseline * (1.0f - al) + (float)delta * al;
        a->calib_f++;
        a->sig_ema  = 0.0f;
        a->sig_norm = 0.0f;
    } else {
        /* Slow baseline drift tracking */
        a->baseline = a->baseline * (1.0f - BASE_EMA_A) +
                      (float)delta * BASE_EMA_A;
        /* Deviation from baseline, scaled by sensitivity */
        float raw  = fabsf((float)delta - a->baseline) * s;
        a->sig_ema = a->sig_ema * (1.0f - SIG_EMA_A) + raw * SIG_EMA_A;
        float n    = a->sig_ema / NORM_SCALE;
        a->sig_norm = (n > 1.0f) ? 1.0f : n;
    }

    /* Rolling history for bar-graph */
    a->hist[a->hist_pos] = a->sig_norm;
    a->hist_pos = (uint8_t)((a->hist_pos + 1) % HIST_LEN);

    /* ── Card-finder calibration / tracking ─────────────────── */
    if(a->mode == ModeC) {
        if(a->card_cal_f < CARD_CALIB_F) {
            float al = (a->card_cal_f == 0) ? 1.0f : 0.15f;
            a->card_base = a->card_base * (1.0f - al) + (float)delta * al;
            a->card_cal_f++;
            if(a->card_cal_f >= CARD_CALIB_F) {
                a->card_cal = true;
                FURI_LOG_I(TAG, "Card baseline: %.1f", (double)a->card_base);
            }
        } else {
            float cs = (float)a->sens / 4.0f;
            float raw = fabsf((float)delta - a->card_base) * cs;
            a->card_sig = a->card_sig * 0.70f + raw * 0.30f;
            float cn = a->card_sig / NORM_SCALE;
            if(cn > 1.0f) cn = 1.0f;

            if(cn >= CARD_THRESH) {
                if(!a->card_found) {
                    a->card_found   = true;
                    a->card_found_t = furi_get_tick();
                    a->card_notif   = false;
                }
            } else if(cn < CARD_THRESH * 0.5f) {
                /* Hysteresis: require signal to drop to 50 % of threshold */
                a->card_found = false;
                a->card_notif = false;
            }

            /* Deferred notification flag */
            if(a->card_found && !a->card_notif) {
                uint32_t held = furi_ticks_to_ms(
                    furi_get_tick() - a->card_found_t);
                if(held >= CARD_HOLD_MS) {
                    a->card_notif   = true;
                    a->pending_notif = true; /* send after releasing mutex */
                }
            }
        }
    }
}

/* ================================================================
   Feedback hardware update — called every SAMPLE_MS, inside mutex
   ================================================================ */
static void fb_update(App* a) {
    /* During calibration: idle blue LED only */
    if(a->calib_f < CALIB_FRAMES) {
        if(a->spk && furi_hal_speaker_is_mine()) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
            a->spk = false;
        }
        furi_hal_light_set(LightRed, 0);
        furi_hal_light_set(LightBlue, 6);
        return;
    }

    /* Choose which signal drives the feedback */
    float n = (a->mode == ModeC && a->card_cal)
                  ? ((a->card_sig / NORM_SCALE > 1.0f)
                         ? 1.0f
                         : a->card_sig / NORM_SCALE)
                  : a->sig_norm;

    /* Trigger threshold: lower sensitivity → higher threshold */
    float thr = 0.50f / (float)a->sens;
    bool  act = (n >= thr);

    /* ── LED ─────────────────────────────────────────────────── */
    if(act) {
        furi_hal_light_set(LightRed,  (uint8_t)(n * 255.0f));
        furi_hal_light_set(LightBlue, 0);
    } else {
        furi_hal_light_set(LightRed, 0);
        furi_hal_light_set(LightBlue, 5); /* faint idle blue */
    }

    /* ── Sound — continuous tone, pitch tracks signal level ───── */
    bool w_snd = act && (a->fb == FbSV || a->fb == FbS);
    if(w_snd) {
        float freq = SND_LO + n * (SND_HI - SND_LO);
        if(!a->spk) a->spk = furi_hal_speaker_acquire(0); /* non-blocking */
        if(a->spk && furi_hal_speaker_is_mine())
            furi_hal_speaker_start(freq, SND_VOL);
    } else {
        if(a->spk && furi_hal_speaker_is_mine()) {
            furi_hal_speaker_stop();
            furi_hal_speaker_release();
            a->spk = false;
        }
    }

    /* ── Vibration — pulsed, rate tracks signal level ─────────── */
    bool w_vib = act && (a->fb == FbSV || a->fb == FbV);
    if(w_vib) {
        uint32_t now = furi_get_tick();
        /* Period inversely proportional to signal strength */
        uint32_t per = (uint32_t)(VIB_PERIOD_HI +
                       (1.0f - n) * (VIB_PERIOD_LO - VIB_PERIOD_HI));
        uint32_t el  = furi_ticks_to_ms(now - a->vib_t);

        if(a->vib_pulse) {
            /* End the ON phase */
            if(el >= VIB_ON_MS) {
                furi_hal_vibro_on(false);
                a->vib_pulse = false;
                a->vib_t     = now;
            }
        } else {
            /* Start the next pulse */
            if(el >= per) {
                furi_hal_vibro_on(true);
                a->vib_pulse = true;
                a->vib       = true;
                a->vib_t     = now;
            }
        }
    } else {
        if(a->vib || a->vib_pulse) {
            furi_hal_vibro_on(false);
            a->vib       = false;
            a->vib_pulse = false;
        }
    }
}

/* ================================================================
   Drawing helpers
   ================================================================ */
static void hline(Canvas* c, int y) {
    canvas_draw_line(c, 0, y, SCR_W - 1, y);
}

static const char* FB_NAMES[FbCount] = {"S+V", "SND", "VIB", "OFF"};

/* ── Metal detector screen ──────────────────────────────────── */
static void draw_metal(Canvas* c, App* a) {
    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 2, HDR_SEP_Y - 1, "RFID METAL DETECTOR");
    hline(c, HDR_SEP_Y);

    /* Calibration splash */
    if(a->calib_f < CALIB_FRAMES) {
        canvas_set_font(c, FontPrimary);
        canvas_draw_str_aligned(c, 64, 28,
            AlignCenter, AlignCenter, "Calibrating...");
        canvas_set_font(c, FontSecondary);
        canvas_draw_str_aligned(c, 64, 44,
            AlignCenter, AlignCenter, "Hold still, no metal nearby");
        return;
    }

    /* ── History scrolling bar-graph ──────────────────────── */
    for(int i = 0; i < HIST_LEN; i++) {
        /* Oldest bar at left, newest at right */
        int   idx = (a->hist_pos + i) % HIST_LEN;
        float v   = a->hist[idx];
        int   bh  = (int)(v * (float)GRAPH_H);
        if(bh < 1 && v > 0.005f) bh = 1;
        int bx = i * HIST_STEP;
        if(bh > 0)
            canvas_draw_box(c, bx, GRAPH_BOT - bh, HIST_BAR_W, bh);
    }

    /* Vertical separator between history and current-level bar */
    canvas_draw_line(c, CUR_BAR_X - 2, GRAPH_TOP, CUR_BAR_X - 2, GRAPH_BOT);

    /* ── Current-level bar (outline + fill from bottom) ─────── */
    canvas_draw_frame(c, CUR_BAR_X, GRAPH_TOP, CUR_BAR_W, GRAPH_H);
    int ch = (int)(a->sig_norm * (float)(GRAPH_H - 2));
    if(ch > 0)
        canvas_draw_box(c, CUR_BAR_X + 1, GRAPH_BOT - ch,
                        CUR_BAR_W - 2, ch);

    /* Percentage inside bar */
    char pct[8];
    snprintf(pct, sizeof(pct), "%d%%", (int)(a->sig_norm * 100.0f));
    canvas_set_font(c, FontSecondary);
    canvas_draw_str_aligned(c, CUR_BAR_X + CUR_BAR_W / 2, GRAPH_TOP + 2,
                            AlignCenter, AlignTop, pct);

    /* ── Status row ─────────────────────────────────────────── */
    hline(c, BOT_SEP_Y);
    char st[48];
    snprintf(st, sizeof(st), "S:%d  %s  DWN:Card  UP:Help",
             a->sens, FB_NAMES[a->fb]);
    canvas_draw_str(c, 2, STATUS_Y, st);
}

/* ── Card-finder screen ─────────────────────────────────────── */
static void draw_card(Canvas* c, App* a) {
    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 2, HDR_SEP_Y - 1, "* POKEMON CARD FINDER *");
    hline(c, HDR_SEP_Y);

    if(!a->card_cal) {
        /* Calibration splash */
        canvas_set_font(c, FontPrimary);
        canvas_draw_str_aligned(c, 64, 25,
            AlignCenter, AlignCenter, "Calibrazione...");
        canvas_set_font(c, FontSecondary);
        canvas_draw_str_aligned(c, 64, 37,
            AlignCenter, AlignCenter, "Tieni fermo lontano");
        canvas_draw_str_aligned(c, 64, 45,
            AlignCenter, AlignCenter, "dalle carte / no cards");
        canvas_draw_str(c, 2, STATUS_Y, "DWN:Metal  BCK:Esci");
        return;
    }

    /* Signal level */
    float cn = a->card_sig / NORM_SCALE;
    if(cn > 1.0f) cn = 1.0f;

    char sigt[20];
    snprintf(sigt, sizeof(sigt), "Segnale: %d%%", (int)(cn * 100.0f));
    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 4, 19, sigt);

    /* Horizontal signal bar */
    canvas_draw_frame(c, 4, 21, 120, 8);
    int fw = (int)(cn * 118.0f);
    if(fw > 0) canvas_draw_box(c, 5, 22, fw, 6);

    /* Status message */
    bool confirmed = a->card_found &&
        (furi_ticks_to_ms(furi_get_tick() - a->card_found_t) >= CARD_HOLD_MS);

    canvas_set_font(c, FontPrimary);
    if(confirmed) {
        canvas_draw_str_aligned(c, 64, 38, AlignCenter, AlignCenter,
                                "CARTA TROVATA!");
        canvas_set_font(c, FontSecondary);
        canvas_draw_str_aligned(c, 64, 49, AlignCenter, AlignCenter,
                                "Holo / Foil card found!");
    } else if(a->card_found) {
        canvas_draw_str_aligned(c, 64, 38, AlignCenter, AlignCenter,
                                "Segnale rilevato...");
    } else {
        canvas_draw_str_aligned(c, 64, 38, AlignCenter, AlignCenter,
                                "Scansione...");
        canvas_set_font(c, FontSecondary);
        canvas_draw_str_aligned(c, 64, 49, AlignCenter, AlignCenter,
                                "Muovi piano / Scan slowly");
    }

    /* Status row */
    hline(c, BOT_SEP_Y);
    char st[48];
    snprintf(st, sizeof(st), "S:%d  OK:Recal  %s  DWN:Metal",
             a->sens, FB_NAMES[a->fb]);
    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 2, STATUS_Y, st);
}

/* ── Help screen ────────────────────────────────────────────── */
static void draw_help(Canvas* c) {
    canvas_set_font(c, FontSecondary);
    canvas_draw_str(c, 18, 7, "= RFID DETECTOR =");
    hline(c, 9);
    canvas_draw_str(c, 2, 19, "< >  Sensibilita 1-8");
    canvas_draw_str(c, 2, 27, "OK   Ciclo feedback");
    canvas_draw_str(c, 2, 35, "SU   Mostra/nascondi help");
    canvas_draw_str(c, 2, 43, "GIU  Modalita Metal/Card");
    canvas_draw_str(c, 2, 51, "BCK  Esci / Exit");
    hline(c, 54);
    canvas_draw_str(c, 2, 63, "In Card: OK=ricalibrare");
}

/* ================================================================
   ViewPort draw callback
   ================================================================ */
static void draw_cb(Canvas* c, void* ctx) {
    App* a = (App*)ctx;
    /* 20 ms timeout — avoid blocking the GUI thread too long */
    if(furi_mutex_acquire(a->mtx, 20) != FuriStatusOk) return;
    canvas_clear(c);
    if(a->help)           draw_help(c);
    else if(a->mode == ModeC) draw_card(c, a);
    else                  draw_metal(c, a);
    furi_mutex_release(a->mtx);
}

/* ================================================================
   ViewPort input callback — forwards events to the queue
   ================================================================ */
static void input_cb(InputEvent* e, void* ctx) {
    App* a = (App*)ctx;
    furi_message_queue_put(a->q, e, 0);
}

/* ================================================================
   Mode switch (Metal ↔ Card Finder)
   ================================================================ */
static void toggle_mode(App* a) {
    furi_mutex_acquire(a->mtx, FuriWaitForever);
    a->mode = (AppMode)((a->mode + 1) % ModeCount);
    if(a->mode == ModeC) {
        a->sens        = CARD_SENS_DEF;
        a->card_cal    = false;
        a->card_cal_f  = 0;
        a->card_found  = false;
        a->card_notif  = false;
        a->card_sig    = 0.0f;
        a->card_base   = 0.0f;
        a->pending_notif = false;
    } else {
        a->sens      = SENS_DEF;
        a->calib_f   = 0;
        a->baseline  = 0.0f;
        a->sig_ema   = 0.0f;
        a->sig_norm  = 0.0f;
        a->hist_pos  = 0;
        memset(a->hist, 0, sizeof(a->hist));
    }
    furi_mutex_release(a->mtx);
}

/* ================================================================
   App alloc / free
   ================================================================ */
static App* app_alloc(void) {
    App* a = (App*)malloc(sizeof(App));
    furi_check(a);
    memset(a, 0, sizeof(App));
    a->sens    = SENS_DEF;
    a->fb      = FbSV;
    a->mode    = ModeM;
    a->running = true;

    a->mtx = furi_mutex_alloc(FuriMutexTypeNormal);
    a->q   = furi_message_queue_alloc(8, sizeof(InputEvent));
    a->vp  = view_port_alloc();
    furi_check(a->mtx);
    furi_check(a->q);
    furi_check(a->vp);

    view_port_draw_callback_set(a->vp, draw_cb, a);
    view_port_input_callback_set(a->vp, input_cb, a);

    a->gui   = furi_record_open(RECORD_GUI);
    a->notif = furi_record_open(RECORD_NOTIFICATION);
    gui_add_view_port(a->gui, a->vp, GuiLayerFullscreen);
    return a;
}

static void app_free(App* a) {
    /* Stop audio */
    if(a->spk && furi_hal_speaker_is_mine()) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
    }
    /* Stop vibration */
    furi_hal_vibro_on(false);
    /* Lights off */
    furi_hal_light_set(LightRed,   0);
    furi_hal_light_set(LightGreen, 0);
    furi_hal_light_set(LightBlue,  0);

    gui_remove_view_port(a->gui, a->vp);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    view_port_free(a->vp);
    furi_message_queue_free(a->q);
    furi_mutex_free(a->mtx);
    free(a);
}

/* ================================================================
   Entry point
   ================================================================ */
int32_t rfid_detector_app(void* p) {
    UNUSED(p);

    App* a = app_alloc();
    g_app = a;   /* expose to ISR before starting hardware */
    hw_start();
    notification_message(a->notif, &sequence_blink_blue_10);

    uint32_t  last_sample = furi_get_tick();
    InputEvent e;

    while(a->running) {
        /* ── Input handling ───────────────────────────────── */
        while(furi_message_queue_get(a->q, &e, 0) == FuriStatusOk) {
            /* Only Press and Repeat events */
            if(e.type != InputTypePress && e.type != InputTypeRepeat)
                continue;

            /* Help screen: any Up/Down closes it */
            if(a->help) {
                if(e.key == InputKeyUp || e.key == InputKeyDown)
                    a->help = false;
                else if(e.key == InputKeyBack)
                    a->running = false;
                continue;
            }

            /* Left / Right: fire on Press AND Repeat for fast adjustment */
            if(e.key == InputKeyLeft) {
                if(a->sens > SENS_MIN) a->sens--;
                continue;
            }
            if(e.key == InputKeyRight) {
                if(a->sens < SENS_MAX) a->sens++;
                continue;
            }

            /* Other buttons: Press only */
            if(e.type != InputTypePress) continue;

            switch(e.key) {
            case InputKeyBack:
                a->running = false;
                break;

            case InputKeyUp:
                a->help = !a->help;
                break;

            case InputKeyDown:
                toggle_mode(a);
                break;

            case InputKeyOk:
                if(a->mode == ModeC && a->card_cal) {
                    /* Recalibrate card finder */
                    furi_mutex_acquire(a->mtx, FuriWaitForever);
                    a->card_cal    = false;
                    a->card_cal_f  = 0;
                    a->card_found  = false;
                    a->card_notif  = false;
                    a->pending_notif = false;
                    furi_mutex_release(a->mtx);
                } else {
                    /* Cycle feedback mode */
                    a->fb = (FbMode)((a->fb + 1) % FbCount);
                }
                break;

            default:
                break;
            }
        }

        /* ── Sample tick ──────────────────────────────────── */
        uint32_t now = furi_get_tick();
        if(now - last_sample >= furi_ms_to_ticks(SAMPLE_MS)) {
            last_sample = now;
            bool do_notif = false;

            furi_mutex_acquire(a->mtx, FuriWaitForever);
            sig_update(a);
            fb_update(a);
            /* Drain the deferred-notification flag inside the lock */
            if(a->pending_notif) {
                a->pending_notif = false;
                do_notif = true;
            }
            furi_mutex_release(a->mtx);

            /* Send notification OUTSIDE the lock to avoid potential
               priority-inversion with the notification service queue */
            if(do_notif)
                notification_message(a->notif, &sequence_success);

            view_port_update(a->vp);
        }

        furi_delay_ms(5); /* yield to other tasks */
    }

    /* ── Clean shutdown ───────────────────────────────────── */
    hw_stop();
    g_app = NULL;
    app_free(a);
    return 0;
}
