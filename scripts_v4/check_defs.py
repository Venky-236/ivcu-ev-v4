#!/usr/bin/env python3
"""
check_defs.py  -  independent verification of the IVCU-EV V4 constant tables.

WHY THIS EXISTS
    ivcu_defs.vh contains four hand-written constants that encode the entire
    sensor roster: the attribute ROM, the two mode masks, and the bypass
    eligibility mask.  A single wrong nibble in any of them is a silent safety
    bug - it could mark brake pressure as bypassable, or disable GPS on the
    motorcycle, which is exactly the class of mistake that got into V3 and
    survived synthesis, STA and floorplan without anyone noticing.

    This script rebuilds all four constants from a roster typed in
    independently below, and compares.  It does not read the roster from the
    .vh file, so it is a real second opinion, not a self-consistency check.

USAGE
    python3 scripts_v4/check_defs.py

    Exit code 0 = all constants agree.  Non-zero = a mismatch, with a
    channel-by-channel diff.

    Run this after every edit to the sensor table in ivcu_defs.vh.
"""

import re
import sys
import os

# --------------------------------------------------------------------------
# The roster, typed independently from the architecture spec.
#   class:    0 = SAFETY_CRITICAL   1 = POWERTRAIN_DEGRADE
#             2 = CONDITIONAL_BYPASS 3 = COMFORT_ADAS
#   servicer: 0 = field replaceable  1 = service centre only
#   hv:       0 = low voltage side   1 = high voltage side
#   car/bike: is the channel present in that mode
# --------------------------------------------------------------------------
CRIT, DEG, COND, COMF = 0, 1, 2, 3
FIELD, SVC = 0, 1

#      idx  name                    class  svc    hv  car  bike
ROSTER = [
    (  0, "HVIL_LOOP",              CRIT,  SVC,   1,   1,   1),
    (  1, "ISOLATION_RES",          CRIT,  SVC,   1,   1,   1),
    (  2, "HV_BUS_VOLT",            CRIT,  SVC,   1,   1,   1),
    (  3, "PRECHARGE_VOLT",         CRIT,  SVC,   1,   1,   1),
    (  4, "CONTACTOR_FB_POS",       CRIT,  SVC,   1,   1,   1),
    (  5, "CONTACTOR_FB_NEG",       CRIT,  SVC,   1,   1,   1),

    (  6, "CELL_VOLT_MIN",          CRIT,  SVC,   1,   1,   1),
    (  7, "CELL_VOLT_MAX",          CRIT,  SVC,   1,   1,   1),
    (  8, "PACK_VOLTAGE",           CRIT,  SVC,   1,   1,   1),
    (  9, "PACK_CURRENT",           CRIT,  SVC,   1,   1,   1),
    ( 10, "CELL_TEMP_MIN",          CRIT,  SVC,   1,   1,   1),
    ( 11, "CELL_TEMP_MAX",          CRIT,  SVC,   1,   1,   1),
    ( 12, "PACK_TEMP",              CRIT,  SVC,   1,   1,   1),
    ( 13, "SOC",                    CRIT,  SVC,   1,   1,   1),   # bike too
    ( 14, "SOH",                    DEG,   SVC,   1,   1,   1),
    ( 15, "CHARGE_VOLTAGE",         DEG,   SVC,   1,   1,   1),
    ( 16, "CHARGE_CURRENT",         DEG,   SVC,   1,   1,   1),
    ( 17, "PACK_ENCL_PRESS",        CRIT,  SVC,   1,   1,   1),   # venting

    ( 18, "MOTOR_RPM",              CRIT,  SVC,   0,   1,   1),
    ( 19, "ROTOR_POSITION",         CRIT,  SVC,   0,   1,   1),
    ( 20, "PHASE_CURRENT_A",        CRIT,  SVC,   1,   1,   1),
    ( 21, "PHASE_CURRENT_B",        CRIT,  SVC,   1,   1,   1),
    ( 22, "MOTOR_TEMP",             DEG,   SVC,   0,   1,   1),
    ( 23, "INVERTER_TEMP",          DEG,   SVC,   1,   1,   1),
    ( 24, "DC_LINK_CURRENT",        DEG,   SVC,   1,   1,   1),
    ( 25, "GEAR_POSITION",          COND,  FIELD, 0,   1,   1),

    ( 26, "COOLANT_TEMP_IN",        DEG,   SVC,   0,   1,   0),
    ( 27, "COOLANT_TEMP_OUT",       DEG,   SVC,   0,   1,   0),
    ( 28, "COOLANT_FLOW",           DEG,   SVC,   0,   1,   0),
    ( 29, "COOLANT_PRESSURE",       DEG,   SVC,   0,   1,   0),
    ( 30, "AMBIENT_TEMP",           COND,  FIELD, 0,   1,   1),
    ( 31, "HUMIDITY",               COND,  FIELD, 0,   1,   1),

    ( 32, "WSPD_FRONT_A",           CRIT,  SVC,   0,   1,   1),
    ( 33, "WSPD_FRONT_B",           CRIT,  SVC,   0,   1,   0),
    ( 34, "WSPD_REAR_A",            CRIT,  SVC,   0,   1,   1),
    ( 35, "WSPD_REAR_B",            CRIT,  SVC,   0,   1,   0),
    ( 36, "ACCEL_LONG",             CRIT,  SVC,   0,   1,   1),
    ( 37, "ACCEL_LAT",              CRIT,  SVC,   0,   1,   1),
    ( 38, "ACCEL_VERT",             CRIT,  SVC,   0,   1,   1),
    ( 39, "YAW_RATE",               CRIT,  SVC,   0,   1,   1),
    ( 40, "ROLL_RATE",              CRIT,  SVC,   0,   1,   1),
    ( 41, "PITCH_RATE",             DEG,   SVC,   0,   1,   1),
    ( 42, "STEERING_ANGLE",         DEG,   SVC,   0,   1,   0),
    ( 43, "RIDE_HEIGHT",            COND,  FIELD, 0,   1,   0),

    ( 44, "THROTTLE_POS_1",         CRIT,  SVC,   0,   1,   1),
    ( 45, "THROTTLE_POS_2",         CRIT,  SVC,   0,   1,   1),
    ( 46, "BRAKE_PRESSURE",         CRIT,  SVC,   0,   1,   1),
    ( 47, "BRAKE_SWITCH",           CRIT,  SVC,   0,   1,   1),
    ( 48, "SIDE_STAND",             COND,  FIELD, 0,   0,   1),
    ( 49, "SEAT_OCCUPANCY",         COND,  FIELD, 0,   1,   1),

    ( 50, "CRASH_FRONT",            CRIT,  SVC,   0,   1,   1),
    ( 51, "CRASH_SIDE",             CRIT,  SVC,   0,   1,   0),
    ( 52, "TIP_OVER",               CRIT,  SVC,   0,   0,   1),

    ( 53, "CAMERA_STATUS",          COMF,  FIELD, 0,   1,   1),
    ( 54, "RADAR_STATUS",           COMF,  SVC,   0,   1,   0),
    ( 55, "LIDAR_STATUS",           COMF,  SVC,   0,   1,   0),
    ( 56, "ULTRASONIC_STATUS",      COMF,  FIELD, 0,   1,   0),
    ( 57, "GPS_STATUS",             DEG,   SVC,   0,   1,   1),   # bike too

    ( 58, "TPMS_FRONT",             COND,  FIELD, 0,   1,   1),
    ( 59, "TPMS_REAR",              COND,  FIELD, 0,   1,   1),
    ( 60, "CABIN_TEMP",             COMF,  FIELD, 0,   1,   0),
    ( 61, "AMBIENT_LIGHT",          COMF,  FIELD, 0,   1,   0),
    ( 62, "RAIN_SENSOR",            COMF,  FIELD, 0,   1,   0),
    ( 63, "SUNLOAD",                COMF,  FIELD, 0,   1,   0),
]

NUM = 64
CLASS_NAME = {CRIT: "SAFETY_CRITICAL", DEG: "POWERTRAIN_DEGRADE",
              COND: "CONDITIONAL_BYPASS", COMF: "COMFORT_ADAS"}


def build():
    """Rebuild all four constants from the roster above."""
    idxs = [r[0] for r in ROSTER]
    assert sorted(idxs) == list(range(NUM)), \
        "roster does not cover 0..63 exactly (duplicates or gaps)"

    attr = 0
    car = 0
    bike = 0
    elig = 0
    for idx, _name, cls, svc, hv, in_car, in_bike in ROSTER:
        nib = (hv << 3) | (svc << 2) | cls
        attr |= nib << (idx * 4)
        if in_car:
            car |= 1 << idx
        if in_bike:
            bike |= 1 << idx
        if cls == COND:
            elig |= 1 << idx
    return attr, car, bike, elig


def parse_vh(path):
    """Pull the four constants out of ivcu_defs.vh."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        text = fh.read()

    def grab(macro):
        # `define NAME  <width>'h<hex with optional underscores>
        pat = r"`define\s+" + macro + r"\s+\d+'h([0-9A-Fa-f_]+)"
        m = re.search(pat, text)
        if not m:
            raise SystemExit("FAIL: could not find `define %s in %s" % (macro, path))
        return int(m.group(1).replace("_", ""), 16)

    return (grab("SENSOR_ATTR_TABLE"), grab("CAR_MASK"),
            grab("BIKE_MASK"), grab("BYPASS_ELIGIBLE"))


def hexfmt(val, nibbles):
    s = "%0*X" % (nibbles, val)
    return "_".join(s[i:i + 4] for i in range(0, nibbles, 4))


def popcount(v):
    return bin(v).count("1")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    vh = os.path.join(here, "..", "RTL_V4", "ivcu_defs.vh")
    vh = os.path.normpath(vh)

    exp_attr, exp_car, exp_bike, exp_elig = build()
    got_attr, got_car, got_bike, got_elig = parse_vh(vh)

    ok = True
    print("=" * 74)
    print("IVCU-EV V4  constant table check")
    print("file: %s" % vh)
    print("=" * 74)

    checks = [
        ("SENSOR_ATTR_TABLE", exp_attr, got_attr, 64),
        ("CAR_MASK",          exp_car,  got_car,  16),
        ("BIKE_MASK",         exp_bike, got_bike, 16),
        ("BYPASS_ELIGIBLE",   exp_elig, got_elig, 16),
    ]
    for name, exp, got, nib in checks:
        good = (exp == got)
        ok &= good
        print("%-20s %s" % (name, "PASS" if good else "*** FAIL ***"))
        if not good:
            print("    expected 64'h%s" % hexfmt(exp, nib))
            print("    found    64'h%s" % hexfmt(got, nib))

    # ---- per-channel diff of the attribute table --------------------------
    if exp_attr != got_attr:
        print("\nattribute mismatches, channel by channel:")
        for idx, name, cls, svc, hv, _c, _b in ROSTER:
            e = (exp_attr >> (idx * 4)) & 0xF
            g = (got_attr >> (idx * 4)) & 0xF
            if e != g:
                print("    ch %2d %-20s expected %X  found %X" % (idx, name, e, g))

    # ---- census ------------------------------------------------------------
    print("\nclass census")
    counts = {c: 0 for c in CLASS_NAME}
    for _i, _n, cls, _s, _h, _c, _b in ROSTER:
        counts[cls] += 1
    for c in (CRIT, DEG, COND, COMF):
        print("    %-22s %2d" % (CLASS_NAME[c], counts[c]))
    total = sum(counts.values())
    print("    %-22s %2d  %s" % ("total", total, "PASS" if total == NUM else "*** FAIL ***"))
    ok &= (total == NUM)

    # ---- the safety invariants that matter --------------------------------
    print("\nsafety invariants")

    n_elig = popcount(got_elig)
    good = (n_elig == 8)
    ok &= good
    print("    exactly 8 bypassable channels                 %s (%d)"
          % ("PASS" if good else "*** FAIL ***", n_elig))

    # no SAFETY_CRITICAL channel may appear in the eligibility mask
    bad = [ (i,n) for i,n,c,_s,_h,_ca,_b in ROSTER
            if c == CRIT and (got_elig >> i) & 1 ]
    good = not bad
    ok &= good
    print("    no safety-critical channel is bypassable      %s"
          % ("PASS" if good else "*** FAIL *** " + str(bad)))

    # every HV-hazard channel must be service-centre-only
    bad = [ (i,n) for i,n,_c,s,h,_ca,_b in ROSTER if h and s != SVC ]
    good = not bad
    ok &= good
    print("    every HV channel is service-centre only       %s"
          % ("PASS" if good else "*** FAIL *** " + str(bad)))

    # GPS must be live in bike mode - the V3 bug that killed crash SOS on bikes
    gps_bike = (got_bike >> 57) & 1
    ok &= bool(gps_bike)
    print("    GPS (ch 57) enabled in BIKE mode              %s"
          % ("PASS" if gps_bike else "*** FAIL ***"))

    soc_bike = (got_bike >> 13) & 1
    ok &= bool(soc_bike)
    print("    SOC (ch 13) enabled in BIKE mode              %s"
          % ("PASS" if soc_bike else "*** FAIL ***"))

    # side stand must NOT be live in car mode
    ss_car = (got_car >> 48) & 1
    ok &= not ss_car
    print("    side stand (ch 48) disabled in CAR mode       %s"
          % ("PASS" if not ss_car else "*** FAIL ***"))

    print("\nmask population")
    print("    CAR_MASK  active channels  %2d" % popcount(got_car))
    print("    BIKE_MASK active channels  %2d" % popcount(got_bike))

    print("\nscan timing")
    print("    car  sweep  64 ch x 4 cyc @ 25 MHz = %.2f us" % (64 * 4 / 25e6 * 1e6))
    print("    bike sweep  %2d ch x 4 cyc @ 25 MHz = %.2f us"
          % (popcount(got_bike), popcount(got_bike) * 4 / 25e6 * 1e6))

    print("\n" + "=" * 74)
    print("RESULT: %s" % ("ALL CHECKS PASS" if ok else "FAILURES ABOVE"))
    print("=" * 74)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
