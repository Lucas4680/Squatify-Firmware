// Speaker processing: see speaker_dsp.h for the signal path and
// speaker_config.h for every setting used here.

#include "speaker_dsp.h"
#include "speaker_config.h"

#include <math.h>
#include <string.h>

#define DSP_INLINE static inline __attribute__((always_inline))

#define DSP_PI 3.14159265358979323846


// ============================================================
// BIQUAD (transposed direct form II)
// ============================================================

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} biquad_t;


// Coefficients are designed in double precision: the low corner frequencies
// used here lose accuracy if the design maths is done in single precision.
static void biquad_set(
    biquad_t *q,
    double b0, double b1, double b2,
    double a0, double a1, double a2)
{
    q->b0 = (float)(b0 / a0);
    q->b1 = (float)(b1 / a0);
    q->b2 = (float)(b2 / a0);
    q->a1 = (float)(a1 / a0);
    q->a2 = (float)(a2 / a0);
    q->z1 = 0.0f;
    q->z2 = 0.0f;
}


static void biquad_highpass(biquad_t *q, double f0, double Q)
{
    const double w = 2.0 * DSP_PI * f0 / (double)SPK_SAMPLE_RATE_HZ;
    const double cs = cos(w);
    const double sn = sin(w);
    const double al = sn / (2.0 * Q);

    biquad_set(q,
               (1.0 + cs) / 2.0, -(1.0 + cs), (1.0 + cs) / 2.0,
               1.0 + al, -2.0 * cs, 1.0 - al);
}


static void biquad_lowpass(biquad_t *q, double f0, double Q)
{
    const double w = 2.0 * DSP_PI * f0 / (double)SPK_SAMPLE_RATE_HZ;
    const double cs = cos(w);
    const double sn = sin(w);
    const double al = sn / (2.0 * Q);

    biquad_set(q,
               (1.0 - cs) / 2.0, 1.0 - cs, (1.0 - cs) / 2.0,
               1.0 + al, -2.0 * cs, 1.0 - al);
}


static void biquad_lowshelf(biquad_t *q, double f0, double gain_db, double S)
{
    const double A = pow(10.0, gain_db / 40.0);
    const double w = 2.0 * DSP_PI * f0 / (double)SPK_SAMPLE_RATE_HZ;
    const double cs = cos(w);
    const double sn = sin(w);
    const double al = sn / 2.0 * sqrt((A + 1.0 / A) * (1.0 / S - 1.0) + 2.0);
    const double t = 2.0 * sqrt(A) * al;

    biquad_set(q,
               A * ((A + 1.0) - (A - 1.0) * cs + t),
               2.0 * A * ((A - 1.0) - (A + 1.0) * cs),
               A * ((A + 1.0) - (A - 1.0) * cs - t),
               (A + 1.0) + (A - 1.0) * cs + t,
               -2.0 * ((A - 1.0) + (A + 1.0) * cs),
               (A + 1.0) + (A - 1.0) * cs - t);
}


static void biquad_peak(biquad_t *q, double f0, double gain_db, double Q)
{
    const double A = pow(10.0, gain_db / 40.0);
    const double w = 2.0 * DSP_PI * f0 / (double)SPK_SAMPLE_RATE_HZ;
    const double cs = cos(w);
    const double sn = sin(w);
    const double al = sn / (2.0 * Q);

    biquad_set(q,
               1.0 + al * A, -2.0 * cs, 1.0 - al * A,
               1.0 + al / A, -2.0 * cs, 1.0 - al / A);
}


DSP_INLINE float biquad_run(biquad_t *q, float x)
{
    const float y = q->b0 * x + q->z1;

    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;

    return y;
}


// ============================================================
// LOOK-AHEAD GAIN FOLLOWER
// ============================================================
//
// Takes the gain each sample would need (1.0 = none) and returns a smooth
// gain. The lowest request is held for hold_samples, then released slowly.
// The attack is timed so the gain has arrived by the end of the look-ahead
// delay that the audio passes through.

typedef struct {
    float hold;
    float gain;
    float attack;
    float release;
    uint32_t cnt;
    uint32_t hold_samples;
} gain_follower_t;


static void gain_follower_init(
    gain_follower_t *g,
    uint32_t lookahead,
    uint32_t hold_samples,
    double release_s)
{
    g->hold = 1.0f;
    g->gain = 1.0f;
    g->attack = (float)(1.0 - exp(-5.0 / (double)lookahead));
    g->release = (float)(1.0 - exp(-1.0 / (release_s * (double)SPK_SAMPLE_RATE_HZ)));
    g->cnt = 0;
    g->hold_samples = hold_samples;
}


DSP_INLINE float gain_follower_run(gain_follower_t *g, float r)
{
    if (r <= g->hold) {
        g->hold = r;
        g->cnt = g->hold_samples;
    }
    else if (g->cnt > 0) {
        g->cnt--;
    }
    else {
        g->hold += (1.0f - g->hold) * g->release;

        if (g->hold > r) {
            g->hold = r;
            g->cnt = g->hold_samples;
        }
    }

    g->gain += (g->hold - g->gain) * g->attack;

    return g->gain;
}


// ============================================================
// SPEAKER MODEL
// ============================================================
//
// Lumped model of the driver, the passive radiator and the air in the box,
// driven by the amplifier voltage. Positions are in millimetres.
//
//   coil:      Le*i'    = V - Re*i - Bl*xd'
//   driver:    Mms*xd'' = Bl*i - Rms*xd' - Kms*xd - box force
//   radiator:  Mmp*xp'' =      - Rmp*xp' - Kmp*xp - box force
//   box force on a cone of area S = S * (air stiffness) * (Sd*xd + Sp*xp)
//
// Against the exact solution of these equations the stepped model below is
// within 1 % for excursion and 3 % for power up to 1 kHz.

typedef struct {
    float xd;       // driver position, mm
    float vd;       // driver velocity, mm/s
    float xp;       // radiator position, mm
    float vp;       // radiator velocity, mm/s
    float i;        // voice coil current, A
} spk_model_t;


static struct {
    float dt;
    float k_le;             // coil current step factor, 1 - exp(-dt*Re/Le)
    float inv_re;
    float bl_mm;            // back-EMF volts per (mm/s) of cone velocity
    float c_i;              // mm/s^2 per amp
    float c_ms;             // driver mechanical damping / Mms
    float k_dd;             // driver stiffness (suspension + box) / Mms
    float k_dp;             // box coupling / Mms
    float c_pp;             // radiator damping / Mmp
    float k_pp;             // radiator stiffness (suspension + box) / Mmp
    float k_pd;             // box coupling / Mmp
} mdl;


static void model_init(void)
{
    const double dt = 1.0 / (double)SPK_SAMPLE_RATE_HZ;

    const double kms = 1.0 / (double)SPK_DRV_CMS_M_PER_N;
    const double kmp = 1.0 / (double)SPK_PR_CMP_M_PER_N;
    const double kb = (double)SPK_AIR_BULK_MODULUS_PA / (double)SPK_BOX_VOLUME_M3;

    const double box_dd = kb * (double)SPK_DRV_SD_M2 * (double)SPK_DRV_SD_M2;
    const double box_dp = kb * (double)SPK_DRV_SD_M2 * (double)SPK_PR_SP_M2;
    const double box_pp = kb * (double)SPK_PR_SP_M2 * (double)SPK_PR_SP_M2;

    const double rms =
        sqrt(kms / (double)SPK_DRV_MMS_KG) * (double)SPK_DRV_MMS_KG / (double)SPK_DRV_QMS;

    const double bl = (double)SPK_DRV_BL_TM;
    const double re = (double)SPK_DRV_RE_OHM;

    mdl.dt = (float)dt;
    mdl.k_le = (float)(1.0 - exp(-dt * re / (double)SPK_DRV_LE_H));
    mdl.inv_re = (float)(1.0 / re);
    mdl.bl_mm = (float)(bl * 1e-3);
    mdl.c_i = (float)(1000.0 * bl / (double)SPK_DRV_MMS_KG);
    mdl.c_ms = (float)(rms / (double)SPK_DRV_MMS_KG);
    mdl.k_dd = (float)((kms + box_dd) / (double)SPK_DRV_MMS_KG);
    mdl.k_dp = (float)(box_dp / (double)SPK_DRV_MMS_KG);
    mdl.c_pp = (float)((double)SPK_PR_RMP_KG_PER_S / (double)SPK_PR_MMP_KG);
    mdl.k_pp = (float)((kmp + box_pp) / (double)SPK_PR_MMP_KG);
    mdl.k_pd = (float)(box_dp / (double)SPK_PR_MMP_KG);
}


// One sample of the model. The coil current moves toward its steady value
// with the exact R-L step; the two masses use semi-implicit Euler, which is
// stable for these resonances at this sample rate.
DSP_INLINE void model_step(spk_model_t *m, float volts)
{
    m->i += ((volts - mdl.bl_mm * m->vd) * mdl.inv_re - m->i) * mdl.k_le;

    const float ad =
        mdl.c_i * m->i - mdl.c_ms * m->vd - mdl.k_dd * m->xd - mdl.k_dp * m->xp;

    m->vd += ad * mdl.dt;
    m->xd += m->vd * mdl.dt;

    const float ap =
        -mdl.c_pp * m->vp - mdl.k_pp * m->xp - mdl.k_pd * m->xd;

    m->vp += ap * mdl.dt;
    m->xp += m->vp * mdl.dt;
}


// ============================================================
// STATE
// ============================================================

static biquad_t hp1;            // protection high-pass, section 1
static biquad_t hp2;            // protection high-pass, section 2
static biquad_t eq1;            // response shelf
static biquad_t eq2;            // response peak
static biquad_t eq3;            // voicing shelf
static biquad_t lp1;            // split low-pass, section 1
static biquad_t lp2;            // split low-pass, section 2
static biquad_t sh1;            // split high-pass, section 1
static biquad_t sh2;            // split high-pass, section 2

static spk_model_t m_low;       // predicts what the bass band would do
static spk_model_t m_high;      // predicts what the upper band would do
static spk_model_t m_out;       // follows what is really sent to the amp

static float ceiling;           // output ceiling, full-scale units
static float peak_target;       // level the peak limiter aims for
static float low_ceiling;       // level the bass band alone may reach
static float xd_limit;          // driver excursion limit used by the limiter, mm
static float xp_limit;          // radiator excursion limit, mm
static float xd_floor;          // least driver travel always left for the bass band
static float xp_floor;          // least radiator travel always left for the bass band

static gain_follower_t gf_bass;
static float dl_low[SPK_XLIM_LOOKAHEAD];
static float dl_high[SPK_XLIM_LOOKAHEAD];
static uint32_t dl_x_pos;

static gain_follower_t gf_peak;
static float dl_peak[SPK_PEAK_LOOKAHEAD];
static uint32_t dl_p_pos;

static float pw_gain;
static float pw_avg;
static float pw_avg_coef;
static float pw_attack;
static float pw_release;
static uint32_t pw_cnt;

static speaker_dsp_stats_t st;


static void stats_restart(void)
{
    st.xd_peak_mm = 0.0f;
    st.xp_peak_mm = 0.0f;
    st.out_peak_v = 0.0f;
    st.bass_gain_min = 1.0f;
    st.peak_gain_min = 1.0f;
    st.clamped = 0;
}


// ============================================================
// INITIALIZATION
// ============================================================

void speaker_dsp_init(void)
{
    const double fs = (double)SPK_SAMPLE_RATE_HZ;

    model_init();

    // 4th-order Butterworth high-pass = two sections with these Q values.
    biquad_highpass(&hp1, SPK_HPF_HZ, 0.54119610);
    biquad_highpass(&hp2, SPK_HPF_HZ, 1.30656296);

    biquad_lowshelf(&eq1, SPK_EQ_SHELF_HZ, SPK_EQ_SHELF_DB, SPK_EQ_SHELF_SLOPE);
    biquad_peak(&eq2, SPK_EQ_PEAK_HZ, SPK_EQ_PEAK_DB, SPK_EQ_PEAK_Q);
    biquad_lowshelf(&eq3, SPK_VOICE_SHELF_HZ, SPK_VOICE_SHELF_DB, SPK_VOICE_SHELF_SLOPE);

    // 4th-order Linkwitz-Riley split: the two bands add back up to the
    // original level at every frequency.
    biquad_lowpass(&lp1, SPK_SPLIT_HZ, 0.70710678);
    biquad_lowpass(&lp2, SPK_SPLIT_HZ, 0.70710678);
    biquad_highpass(&sh1, SPK_SPLIT_HZ, 0.70710678);
    biquad_highpass(&sh2, SPK_SPLIT_HZ, 0.70710678);

    memset(&m_low, 0, sizeof(m_low));
    memset(&m_high, 0, sizeof(m_high));
    memset(&m_out, 0, sizeof(m_out));

    ceiling = SPK_OUT_CEILING_V / SPK_AMP_FULL_SCALE_VPEAK;
    peak_target = ceiling * SPK_PEAK_TARGET_FRAC;
    low_ceiling = ceiling * SPK_LOW_BAND_CEILING_FRAC;
    xd_limit = SPK_DRV_XMAX_MM * SPK_DRV_X_MARGIN;
    xp_limit = SPK_PR_XLIMIT_MM;
    xd_floor = 0.1f * xd_limit;
    xp_floor = 0.1f * xp_limit;

    gain_follower_init(&gf_bass, SPK_XLIM_LOOKAHEAD, SPK_XLIM_HOLD, SPK_XLIM_RELEASE_S);
    memset(dl_low, 0, sizeof(dl_low));
    memset(dl_high, 0, sizeof(dl_high));
    dl_x_pos = 0;

    gain_follower_init(&gf_peak, SPK_PEAK_LOOKAHEAD, SPK_PEAK_HOLD, SPK_PEAK_RELEASE_S);
    memset(dl_peak, 0, sizeof(dl_peak));
    dl_p_pos = 0;

    pw_gain = 1.0f;
    pw_avg = 0.0f;
    pw_avg_coef = (float)(1.0 - exp(-1.0 / ((double)SPK_POWER_AVG_S * fs)));
    pw_attack = (float)(1.0 - exp(-(double)SPK_POWER_UPDATE_SAMPLES / ((double)SPK_POWER_ATTACK_S * fs)));
    pw_release = (float)(1.0 - exp(-(double)SPK_POWER_UPDATE_SAMPLES / ((double)SPK_POWER_RELEASE_S * fs)));
    pw_cnt = 0;

    st.power_avg_w = 0.0f;
    st.power_gain = 1.0f;
    stats_restart();
}


// ============================================================
// PROCESS ONE SAMPLE
// ============================================================

float speaker_dsp_process(float x)
{
    float a;
    float r;
    uint32_t pos;

    if (!isfinite(x)) {
        x = 0.0f;
    }

    // 1. Protection high-pass and EQ.
    x = biquad_run(&hp2, biquad_run(&hp1, x));
    x = biquad_run(&eq3, biquad_run(&eq2, biquad_run(&eq1, x)));

    // 2. Band split.
    const float low = biquad_run(&lp2, biquad_run(&lp1, x));
    const float high = biquad_run(&sh2, biquad_run(&sh1, x));

    // 3. Predict the excursion each band would cause and find the bass gain
    //    that keeps the driver, the radiator and the bass voltage legal.
    model_step(&m_low, low * SPK_AMP_FULL_SCALE_VPEAK);

    // The upper band is not turned down by the bass gain, but a little bass
    // leaks into it, so the travel it causes is taken off the allowance.
    // Its voltage can never exceed the ceiling at the output, so the
    // prediction is capped there.
    float high_v = high;

    if (high_v > ceiling) {
        high_v = ceiling;
    }
    else if (high_v < -ceiling) {
        high_v = -ceiling;
    }

    model_step(&m_high, high_v * SPK_AMP_FULL_SCALE_VPEAK);

    r = 1.0f;

    float head = xd_limit - fabsf(m_high.xd);
    if (head < xd_floor) {
        head = xd_floor;
    }

    a = fabsf(m_low.xd);
    if (a > head) {
        r = head / a;
    }

    head = xp_limit - fabsf(m_high.xp);
    if (head < xp_floor) {
        head = xp_floor;
    }

    a = fabsf(m_low.xp);
    if (a * r > head) {
        r = head / a;
    }

    a = fabsf(low);
    if (a * r > low_ceiling) {
        r = low_ceiling / a;
    }

    if (r < SPK_XLIM_MIN_GAIN) {
        r = SPK_XLIM_MIN_GAIN;
    }

    const float g_bass = gain_follower_run(&gf_bass, r);

    // 4. Delay both bands so the gain is already down when the peak arrives.
    pos = dl_x_pos;

    const float low_d = dl_low[pos];
    const float high_d = dl_high[pos];

    dl_low[pos] = low;
    dl_high[pos] = high;

    pos++;
    if (pos >= SPK_XLIM_LOOKAHEAD) {
        pos = 0;
    }
    dl_x_pos = pos;

    const float y = g_bass * low_d + high_d;

    // 5. Look-ahead peak limiter to the amplifier ceiling.
    a = fabsf(y);

    r = 1.0f;
    if (a > peak_target) {
        r = peak_target / a;
    }

    const float g_peak = gain_follower_run(&gf_peak, r);

    pos = dl_p_pos;

    const float y_d = dl_peak[pos];

    dl_peak[pos] = y;

    pos++;
    if (pos >= SPK_PEAK_LOOKAHEAD) {
        pos = 0;
    }
    dl_p_pos = pos;

    // 6. Slow power-budget gain, then the safety clamp.
    float out = g_peak * y_d * pw_gain;

    if (out > ceiling) {
        out = ceiling;
        st.clamped++;
    }
    else if (out < -ceiling) {
        out = -ceiling;
        st.clamped++;
    }

    // A non-finite value would stay in the filters forever. Start clean.
    if (!isfinite(out)) {
        speaker_dsp_init();
        return 0.0f;
    }

    // 7. Model of what is really sent: excursion, current and power.
    const float volts = out * SPK_AMP_FULL_SCALE_VPEAK;

    model_step(&m_out, volts);

    pw_avg += (volts * m_out.i - pw_avg) * pw_avg_coef;

    pw_cnt++;
    if (pw_cnt >= SPK_POWER_UPDATE_SAMPLES) {
        pw_cnt = 0;

        float target = 1.0f;

        if (pw_avg > SPK_POWER_BUDGET_W) {
            target = pw_gain * sqrtf(SPK_POWER_BUDGET_W / pw_avg);
        }

        if (target < pw_gain) {
            pw_gain += (target - pw_gain) * pw_attack;
        }
        else {
            pw_gain += (target - pw_gain) * pw_release;
        }

        if (pw_gain < SPK_POWER_MIN_GAIN) {
            pw_gain = SPK_POWER_MIN_GAIN;
        }

        if (pw_gain > 1.0f) {
            pw_gain = 1.0f;
        }

        st.power_avg_w = pw_avg;
        st.power_gain = pw_gain;
    }

    // Statistics for the monitor log.
    a = fabsf(m_out.xd);
    if (a > st.xd_peak_mm) {
        st.xd_peak_mm = a;
    }

    a = fabsf(m_out.xp);
    if (a > st.xp_peak_mm) {
        st.xp_peak_mm = a;
    }

    a = fabsf(volts);
    if (a > st.out_peak_v) {
        st.out_peak_v = a;
    }

    if (g_bass < st.bass_gain_min) {
        st.bass_gain_min = g_bass;
    }

    if (g_peak < st.peak_gain_min) {
        st.peak_gain_min = g_peak;
    }

    return out;
}


// ============================================================
// STATISTICS
// ============================================================

void speaker_dsp_read_stats(speaker_dsp_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    *stats = st;

    stats_restart();
}


uint32_t speaker_dsp_latency_samples(void)
{
    return SPK_XLIM_LOOKAHEAD + SPK_PEAK_LOOKAHEAD;
}
