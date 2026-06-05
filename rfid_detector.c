// ====================================================================
//  rfid_detector.c
//  RFID Metal Detector & Pokemon Card Finder — Flipper Zero FAP
//
//  Principio / Principle:
//    Il carrier LF a 125 kHz resta attivo; il comparatore conta le
//    transizioni del segnale sull'antenna. Metalli e foil metallici
//    nelle carte caricano l'antenna in modo misurabile.
//
//    The 125 kHz LF carrier stays active; the comparator counts
//    signal transitions on the antenna coil. Metal objects and
//    metallic card foils load the antenna in a measurable way.
//
//  Build: ufbt (universal Flipper Build Tool)
// ====================================================================

#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_rfid.h>
#include <gui/gui.h>
#include <input/input.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define TAG "RfidDet"

/* ── LF RFID carrier ──────────────────────────────────────── */
#define LF_FREQ        125000.0f   /* Hz  */
#define LF_DUTY        0.5f        /* 50% duty cycle */

/* ── Timing ───────────────────────────────────────────────── */
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
