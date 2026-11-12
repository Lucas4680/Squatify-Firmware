// Speaker, enclosure and protection settings for the Squatify speaker.
//
// Everything that describes the physical speaker or decides how hard it is
// driven lives here. speaker_dsp.c builds its model and filters from these
// values at start-up, so changing a number here and rebuilding is enough.

#pragma once

// ============================================================
// AUDIO FORMAT
// ============================================================

#define SPK_SAMPLE_RATE_HZ          48000.0f


// ============================================================
// AMPLIFIER GAIN (MAX98390)
// ============================================================
//
// MAX98390 datasheet, amplifier path: the DAC gives 2.16 V peak at digital
// full scale and the amplifier adds a fixed 15 dB, so a full-scale sample
// asks for 2.16 V x 5.62 = 12.15 V peak at the speaker terminals.
//
// The boost rail is 6.5 V (MAX98390_BOOST_MV in max98390.h), so anything
// above roughly 6 V would clip in the amplifier. SPK_OUT_CEILING_V is the
// highest voltage the DSP will ever request.
//
// To check SPK_AMP_FULL_SCALE_VPEAK on the real board: play a 200 Hz sine
// at -20 dBFS with the volume at 0 dB and measure the AC voltage across the
// speaker. It should read about 0.86 V RMS.

#define SPK_AMP_FULL_SCALE_VPEAK    12.15f
#define SPK_OUT_CEILING_V           6.0f


// ============================================================
// DRIVER: Dayton Audio DMA45-4
// ============================================================
//
// Published Thiele/Small parameters. Unit-to-unit spread is typically
// 10-15 %, which is why the excursion limits below keep some margin.

#define SPK_DRV_RE_OHM              3.7f        // voice coil DC resistance
#define SPK_DRV_LE_H                0.3e-3f     // voice coil inductance
#define SPK_DRV_BL_TM               2.8f        // motor force factor
#define SPK_DRV_MMS_KG              1.3e-3f     // moving mass
#define SPK_DRV_CMS_M_PER_N         0.83e-3f    // suspension compliance
#define SPK_DRV_QMS                 5.23f       // mechanical Q
#define SPK_DRV_SD_M2               8.6e-4f     // cone area
#define SPK_DRV_XMAX_MM             2.0f        // linear excursion, one way


// ============================================================
// PASSIVE RADIATOR: Dayton Audio DMA45-PR
// ============================================================

#define SPK_PR_MMP_KG               2.8e-3f     // moving mass (no added weight)
#define SPK_PR_CMP_M_PER_N          2.12e-3f    // suspension compliance
#define SPK_PR_RMP_KG_PER_S         0.38f       // mechanical resistance
#define SPK_PR_SP_M2                8.6e-4f     // radiating area
#define SPK_PR_XLIMIT_MM            4.0f        // 5 mm is the mechanical limit


// ============================================================
// ENCLOSURE
// ============================================================
//
// Net internal air volume. 200 cm3 puts the radiator tuning near 95 Hz;
// 150 cm3 would give about 103 Hz and 250 cm3 about 90 Hz.

#define SPK_BOX_VOLUME_M3           200e-6f
#define SPK_AIR_BULK_MODULUS_PA     1.42e5f


// ============================================================
// VIRTUAL BASS (NOTES BELOW WHAT THE BOX CAN PLAY)
// ============================================================
//
// Notes between about 30 and 85 Hz cannot come out of this box as
// themselves (see EXTRA BASS below). When the ear is given the overtones of
// a low note (2x, 3x, 4x its frequency) it still hears the low note. The
// first firmware did this by accident: its +15 dB boost drove the amplifier
// into clipping, and a 55 Hz note came out about 17 dB louder at 165 Hz than
// at 55 Hz. This stage makes the overtones on purpose, cleanly, and before
// the limiters, so the cone and the supply stay inside their limits.
//
// SPK_VBASS_GAIN_DB sets how loud the overtones are compared with the note
// they stand in for: 0 = about the same level, -6 = subtle, +6 = heavy.
// SPK_VBASS_H2/H3/H4 set the mix of the three overtones.
// Set SPK_VBASS_ENABLE to 0 to switch the stage off.

#define SPK_VBASS_ENABLE            1

#define SPK_VBASS_LOW_HZ            30.0f       // lowest note that is converted
#define SPK_VBASS_HIGH_HZ           85.0f       // notes above this are played for real

#define SPK_VBASS_GAIN_DB           0.0f

#define SPK_VBASS_H2                0.60f       // 2x the note
#define SPK_VBASS_H3                0.40f       // 3x the note
#define SPK_VBASS_H4                0.20f       // 4x the note

// How fast the level detector lets go after a note ends.
#define SPK_VBASS_RELEASE_S         0.12f


// ============================================================
// PROTECTION HIGH-PASS
// ============================================================
//
// Below the radiator tuning the cone is unloaded: it moves a lot and makes
// almost no sound. A 4th-order Butterworth high-pass removes that range.

#define SPK_HPF_HZ                  80.0f


// ============================================================
// RESPONSE EQ
// ============================================================
//
// Fitted to the model of this driver, radiator and box so the response is
// flat from about 95 Hz upward (within 0.1 dB in the model, 100-600 Hz).
// Nothing above 600 Hz is corrected: the simple model is not valid there.
// The EXTRA BASS section further down adds to this while there is room.

#define SPK_EQ_SHELF_HZ             154.0f
#define SPK_EQ_SHELF_DB             11.9f
#define SPK_EQ_SHELF_SLOPE          1.0f

#define SPK_EQ_PEAK_HZ              122.0f
#define SPK_EQ_PEAK_DB              -2.0f
#define SPK_EQ_PEAK_Q               1.2f

// Taste control on top of the flat response. 0 dB = flat, positive = warmer.

#define SPK_VOICE_SHELF_HZ          200.0f
#define SPK_VOICE_SHELF_DB          2.0f
#define SPK_VOICE_SHELF_SLOPE       0.7f


// ============================================================
// BASS (EXCURSION) LIMITER
// ============================================================
//
// The signal is split at SPK_SPLIT_HZ. A model of the speaker predicts how
// far the cone and the radiator would travel for the bass band, and the bass
// band alone is turned down just enough to stay inside the limits. Mids and
// highs are not touched, so the speaker stays loud while the bass is capped.

#define SPK_SPLIT_HZ                250.0f

// Fraction of the driver Xmax the limiter aims for. The rest covers the
// small overshoot on sudden bass notes and the model's own error.
#define SPK_DRV_X_MARGIN            0.95f

// Fraction of the voltage ceiling the bass band may use on its own.
#define SPK_LOW_BAND_CEILING_FRAC   0.85f

#define SPK_XLIM_LOOKAHEAD          240         // samples (5 ms)
#define SPK_XLIM_HOLD               960         // samples (20 ms)
#define SPK_XLIM_RELEASE_S          0.20f
#define SPK_XLIM_MIN_GAIN           0.05f


// ============================================================
// EXTRA BASS (ONLY WHILE THERE IS ROOM FOR IT)
// ============================================================
//
// What this box can do, from the model (loudest clean level at 1 m):
//
//     600 Hz  83 dB     100 Hz  77 dB     80 Hz  68 dB     60 Hz  52 dB
//
// Below about 80 Hz the radiator cancels the cone and the output falls off a
// cliff, so no amount of boost gives real 40-60 Hz. What can be done is to
// carry the response a little lower and lift the bass, as long as the cone,
// the radiator and the power budget have room. That is what this stage is.
//
// It is an extra filter on the bass band whose added part has its own gain:
//
//   - at low and normal volume (up to roughly 70 % on the PC) it is fully
//     on: the bass is lifted by SPK_XBASS_SHELF_DB below about
//     SPK_XBASS_SHELF_HZ, and the response holds up to about 83 Hz instead
//     of 89 Hz (-3 dB points). An 80 Hz note comes out about 7 dB louder;
//   - as the volume goes up it is the first thing to give way, before the
//     normal bass band or anything else is turned down. Near full volume it
//     is off and the speaker behaves exactly as it did without this stage:
//     there the bass is already at what the cone and the power budget allow.
//
// The log shows it as "extra bass=NN %".
//
// SPK_XBASS_SHELF_DB is the taste control: 0 = no lift, 3 = default,
// 6 = heavy. SPK_XBASS_PEAK_* is the extension toward 80 Hz. Keep the Q at 3
// or below: a sharper peak only works if the real box matches the model to
// within a couple of hertz.

#define SPK_XBASS_ENABLE            1

#define SPK_XBASS_SHELF_HZ          125.0f
#define SPK_XBASS_SHELF_DB          3.0f
#define SPK_XBASS_SHELF_SLOPE       0.8f

#define SPK_XBASS_PEAK_HZ           78.0f
#define SPK_XBASS_PEAK_DB           5.0f
#define SPK_XBASS_PEAK_Q            3.0f

#define SPK_XBASS_RELEASE_S         0.40f

// The extra bass starts giving way at this fraction of the power budget.
#define SPK_XBASS_POWER_FRAC        0.80f


// ============================================================
// PEAK LIMITER
// ============================================================
//
// Look-ahead limiter that keeps the full signal under SPK_OUT_CEILING_V.
// It aims slightly below the ceiling so the final safety clamp stays idle.

#define SPK_PEAK_LOOKAHEAD          72          // samples (1.5 ms)
#define SPK_PEAK_HOLD               480         // samples (10 ms)
#define SPK_PEAK_RELEASE_S          0.10f
#define SPK_PEAK_TARGET_FRAC        0.97f


// ============================================================
// POWER BUDGET
// ============================================================
//
// Average electrical power into the speaker, estimated by the model. This is
// the USB supply budget, not a thermal limit: the driver is rated 10 W RMS,
// which a USB port cannot deliver on average anyway.
//
//   1.5 W  keeps the whole board near 500 mA (strict USB 2.0 port)
//   2.0 W  default, fine on most PC ports
//   3.0 W  USB 3.0 ports and chargers
//
// If the amp log shows VBAT sagging or the brownout level rising, lower it.

#define SPK_POWER_BUDGET_W          2.0f
#define SPK_POWER_AVG_S             0.10f
#define SPK_POWER_ATTACK_S          0.15f
#define SPK_POWER_RELEASE_S         2.0f
#define SPK_POWER_UPDATE_SAMPLES    48
#define SPK_POWER_MIN_GAIN          0.05f
