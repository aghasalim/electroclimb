#!/usr/bin/env python3
"""Component values for the ElectroClimb foot HV driver, with checks.

Run it after changing any value; it fails loudly if a design rule breaks.
    python3 tools/driver_calc.py
"""

# --- inputs -----------------------------------------------------------------
V_MAX = 3000.0          # V, HV module max output (XP Power AG30P-5 class)
P_MODULE = 1.0          # W, module rating
C_PAD_MAX = 1e-9        # F, worst-case pad capacitance in contact (to be measured)
C_PAD_TYP = 500e-12     # F, typical

R_LIMIT = 47e6          # ohm, series limit resistor (HV+ -> pad A)
R_BLEED = 1e9           # ohm, permanent safety bleeder across the pad
R_DUMP = 1e6            # ohm, fast-release resistor switched by HV reed relay
R_SENSE = 10e3          # ohm, pad B -> GND, read by the ADC
V_ADC_MAX = 2.5         # V, usable ADC range on the XIAO ESP32-S3 (with attenuation)
F_ADC = 20e3            # Hz, ADC sample rate during an energise event

# --- derived ----------------------------------------------------------------
v_pad = V_MAX * R_BLEED / (R_LIMIT + R_BLEED)        # steady pad voltage (divider loss)
i_short = V_MAX / R_LIMIT                             # pad shorted to the wall / a finger
p_limit_short = V_MAX ** 2 / R_LIMIT                  # power in R_LIMIT during a short
p_bleed = v_pad ** 2 / R_BLEED
tau_charge = R_LIMIT * C_PAD_TYP
tau_dump = R_DUMP * C_PAD_MAX
tau_bleed = R_BLEED * C_PAD_TYP
v_sense_peak = i_short * R_SENSE                      # first instant of charging
q_pad = C_PAD_TYP * v_pad
samples_per_tau = tau_charge * F_ADC

rows = [
    ("Steady pad voltage", f"{v_pad:.0f} V ({v_pad / V_MAX:.1%} of max)"),
    ("Short-circuit current", f"{i_short * 1e6:.0f} uA"),
    ("Power in R_LIMIT during short", f"{p_limit_short:.2f} W"),
    ("Bleeder power", f"{p_bleed * 1e3:.1f} mW"),
    ("Charge time constant (typ pad)", f"{tau_charge * 1e3:.1f} ms"),
    ("Fast-release time constant (max pad)", f"{tau_dump * 1e3:.2f} ms"),
    ("Bleeder-only discharge time constant", f"{tau_bleed:.2f} s"),
    ("Peak sense voltage", f"{v_sense_peak:.2f} V"),
    ("Pad charge per step (typ)", f"{q_pad * 1e6:.2f} uC"),
    ("ADC samples per charge time constant", f"{samples_per_tau:.0f}"),
]

# --- design rules -----------------------------------------------------------
checks = [
    (v_pad >= 0.95 * V_MAX, "bleeder steals more than 5% of the pad voltage"),
    (i_short + V_MAX / R_BLEED <= P_MODULE / V_MAX, "fault current exceeds module rating"),
    (p_limit_short <= 0.25, "R_LIMIT dissipates > 0.25 W in a short (use a bigger value)"),
    (i_short <= 100e-6, "fault current above 100 uA"),
    (5 * tau_charge <= 0.15, "pad takes longer than 150 ms to charge (too slow for stepping)"),
    (5 * tau_dump <= 0.01, "fast release slower than 10 ms"),
    (v_sense_peak <= V_ADC_MAX, "sense voltage exceeds ADC range"),
    (v_sense_peak >= 0.2, "sense signal too small to measure"),
    (samples_per_tau >= 50, "too few ADC samples to integrate the charge"),
]

if __name__ == "__main__":
    for name, value in rows:
        print(f"{name:40s} {value}")
    failed = [msg for ok, msg in checks if not ok]
    for msg in failed:
        print("FAIL:", msg)
    assert not failed, f"{len(failed)} design rule(s) failed"
    print("all design rules pass")
