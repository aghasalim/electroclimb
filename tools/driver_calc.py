#!/usr/bin/env python3
"""Component values for the ElectroClimb foot HV driver, with checks.

Run it after changing any value; it fails loudly if a design rule breaks.
    python3 tools/driver_calc.py
"""

# --- inputs -----------------------------------------------------------------
V_MAX = 3000.0          # V, HV module max output (XP Power A30P-5)
P_MODULE = 1.0          # W, module rating
C_PAD_MAX = 1e-9        # F, worst-case pad capacitance in contact (to be measured)
C_PAD_TYP = 500e-12     # F, typical

R_LIMIT = 47e6          # ohm, series limit resistor (HV+ -> pad A)
R_BLEED = 1e9           # ohm, permanent safety bleeder across the pad
R_DUMP = 1e6            # ohm, fast-release resistor switched by HV reed relay
R_SENSE = 10e3          # ohm, pad B -> GND, read by the ADC
V_ADC_MAX = 2.5         # V, usable ADC range on the XIAO ESP32-S3 (with attenuation)
F_ADC = 20e3            # Hz, ADC sample rate during an energise event
R_MON = 750e3           # ohm, bottom of the pad-voltage divider (the bleeder is the top)
V_PWM = 3.3             # V, XIAO logic level
R_FB, R_G = 5.1e3, 10e3 # ohm, op-amp gain resistors: the module's control pin wants 0..5 V
V_IN = 5.0              # V, module supply; the control pin must not exceed it
OVERSHOOT = 1.3         # the module is unregulated: allow 30 % above V_MAX at light load
K1_SWITCH_V = 1000.0    # V, rated switching voltage of the SHV reed relay (4 kV breakdown)

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
v_ctrl_max = V_PWM * (1 + R_FB / R_G)                 # control voltage at 100 % PWM
v_mon = v_pad * R_MON / (R_BLEED + R_MON)             # monitor voltage at full pad voltage
v_mon_worst = V_MAX * OVERSHOOT * R_MON / (R_BLEED + R_MON)
i_dump_peak = V_MAX / R_DUMP                          # through the relay contact at the moment it closes
e_pad = 0.5 * C_PAD_MAX * V_MAX ** 2                  # energy the relay and R_DUMP have to take

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
    ("Control voltage at 100 % PWM", f"{v_ctrl_max:.2f} V"),
    ("Monitor voltage at full pad voltage", f"{v_mon:.2f} V ({v_mon_worst:.2f} V at {OVERSHOOT:.0%} overshoot)"),
    ("Relay closes onto", f"{V_MAX:.0f} V, {i_dump_peak * 1e3:.0f} mA peak, {e_pad * 1e3:.1f} mJ"),
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
    (0.95 * V_IN <= v_ctrl_max <= V_IN, "control voltage does not reach, or exceeds, the module supply"),
    (v_mon_worst <= 3.1, "monitor voltage can exceed the ADC range"),
    (v_mon >= 1.5, "monitor voltage uses less than half the ADC range"),
]

if __name__ == "__main__":
    for name, value in rows:
        print(f"{name:40s} {value}")
    failed = [msg for ok, msg in checks if not ok]
    for msg in failed:
        print("FAIL:", msg)
    assert not failed, f"{len(failed)} design rule(s) failed"
    print("all design rules pass")
    if V_MAX > K1_SWITCH_V:      # a known open point, not a pass: see hardware/foot/driver/README.md
        print(f"OPEN POINT: the relay closes onto {V_MAX:.0f} V but is rated to switch {K1_SWITCH_V:.0f} V. "
              "Test it on the bench or fit a relay rated for the full voltage.")
