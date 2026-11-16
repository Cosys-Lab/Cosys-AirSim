#!/usr/bin/env python3
"""
Generate the World Magnetic Model lookup tables used by AirLib's magnetometer
(AirLib/include/common/WorldMagneticModelTables.hpp).

The tables have the same layout as PX4's geo_magnetic_tables.hpp (10 degree grid,
int16 values with a per-table scale) so AirLib's magnetometer reference matches what
PX4's EKF expects. Unlike PX4 (which queries NOAA's web calculator), the field is
synthesised here directly from the official WMM coefficient file, so the tables can
be regenerated offline for any model/date.

Usage (from the repository root):
    python scripts/wmm/generate_wmm_tables.py scripts/wmm/WMM_2025.COF --date 2027.5 \
        --out AirLib/include/common/WorldMagneticModelTables.hpp

The WMM coefficients are produced by NOAA NCEI and the British Geological Survey and are
in the public domain: https://www.ncei.noaa.gov/products/world-magnetic-model
"""

import argparse
import math
import sys

# WGS-84 ellipsoid and WMM reference radius (km), see the WMM technical report.
WGS84_A = 6378.137
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WMM_REF_RADIUS = 6371.2

SAMPLING_RES = 10
LATS = list(range(-90, 91, SAMPLING_RES))
LONS = list(range(-180, 181, SAMPLING_RES))


def read_cof(path):
    """Return (epoch, model_name, release_date, {(n, m): (g, h, g_dot, h_dot)})."""
    coeffs = {}
    with open(path) as f:
        header = f.readline().split()
        epoch, model, released = float(header[0]), header[1], header[2] if len(header) > 2 else ""
        for line in f:
            parts = line.split()
            if not parts or parts[0].startswith("9999"):
                break
            n, m = int(parts[0]), int(parts[1])
            coeffs[(n, m)] = tuple(float(x) for x in parts[2:6])
    return epoch, model, released, coeffs


def schmidt_legendre(nmax, x):
    """Schmidt semi-normalised associated Legendre functions P[n][m](x) and dP/dx * (1-x^2)
    (no Condon-Shortley phase), x = sin(geocentric latitude)."""
    s = math.sqrt(max(0.0, 1.0 - x * x))
    P = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    # unnormalised recursion
    P[0][0] = 1.0
    for m in range(0, nmax + 1):
        if m > 0:
            P[m][m] = (2 * m - 1) * s * P[m - 1][m - 1]
        if m + 1 <= nmax:
            P[m + 1][m] = (2 * m + 1) * x * P[m][m]
        for n in range(m + 2, nmax + 1):
            P[n][m] = ((2 * n - 1) * x * P[n - 1][m] - (n + m - 1) * P[n - 2][m]) / (n - m)
    # (1 - x^2) dP/dx = (n + m) P[n-1][m] - n x P[n][m]   (unnormalised)
    D = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    for n in range(0, nmax + 1):
        for m in range(0, n + 1):
            prev = P[n - 1][m] if n - 1 >= m else 0.0
            D[n][m] = (n + m) * prev - n * x * P[n][m]
    # Schmidt semi-normalisation
    for n in range(0, nmax + 1):
        for m in range(0, n + 1):
            k = math.sqrt((2.0 if m > 0 else 1.0) * math.factorial(n - m) / math.factorial(n + m))
            P[n][m] *= k
            D[n][m] *= k
    return P, D


def field(coeffs, epoch, date, lat_deg, lon_deg, height_km=0.0):
    """Return (X north, Y east, Z down) in nT, geodetic frame (WMM technical report, eqs. 7-17)."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    # geodetic -> geocentric spherical coordinates
    rc = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    p = (rc + height_km) * math.cos(lat)
    z = (rc * (1.0 - WGS84_E2) + height_km) * math.sin(lat)
    r = math.hypot(p, z)
    lat_c = math.asin(z / r)

    nmax = max(n for n, _ in coeffs)
    dt = date - epoch
    P, D = schmidt_legendre(nmax, math.sin(lat_c))
    cos_c = math.cos(lat_c)

    x_s = y_s = z_s = 0.0
    for (n, m), (g, h, gd, hd) in coeffs.items():
        g_t = g + dt * gd
        h_t = h + dt * hd
        ratio = (WMM_REF_RADIUS / r) ** (n + 2)
        cos_ml, sin_ml = math.cos(m * lon), math.sin(m * lon)
        dP_dlat = D[n][m] / cos_c  # dP/dphi = cos(phi) dP/dx = D / cos(phi)
        x_s += -ratio * (g_t * cos_ml + h_t * sin_ml) * dP_dlat
        y_s += ratio * m * (g_t * sin_ml - h_t * cos_ml) * P[n][m] / cos_c
        z_s += -(n + 1) * ratio * (g_t * cos_ml + h_t * sin_ml) * P[n][m]
    # rotate from geocentric to geodetic frame
    psi = lat_c - lat
    x = x_s * math.cos(psi) - z_s * math.sin(psi)
    z_ = x_s * math.sin(psi) + z_s * math.cos(psi)
    return x, y_s, z_


def tables(coeffs, epoch, date):
    """declination, inclination (degrees) and total intensity (nT) on the 10 degree grid.
    The poles are evaluated 1e-5 degrees away from +-90 (the formulae are singular there)."""
    dec, inc, f = [], [], []
    for lat in LATS:
        lat_eval = max(-90 + 1e-5, min(90 - 1e-5, lat))
        rd, ri, rf = [], [], []
        for lon in LONS:
            x, y, z = field(coeffs, epoch, date, lat_eval, lon)
            h = math.hypot(x, y)
            rd.append(math.degrees(math.atan2(y, x)))
            ri.append(math.degrees(math.atan2(z, h)))
            rf.append(math.sqrt(h * h + z * z))
        dec.append(rd)
        inc.append(ri)
        f.append(rf)
    return dec, inc, f


def quantise(table):
    """Scale to int16 like PX4: scale = max|value| / 32767."""
    peak = max(abs(v) for row in table for v in row)
    scale = peak / 32767.0
    return [[int(round(v / scale)) for v in row] for row in table], scale


def extreme(table, fn):
    best = None
    for i, row in enumerate(table):
        for j, v in enumerate(row):
            if best is None or fn(v, best[0]):
                best = (v, LATS[i], LONS[j])
    return best


def format_table(name, values, comment, scale_name, scale, unit, model, date, raw, blackout=None):
    lines = ["// %s" % comment,
             "// Model: %s," % model,
             "// Date: %.5f," % date,
             "static constexpr const int16_t %s[%d][%d] {" % (name, len(LATS), len(LONS)),
             "\t//    LONGITUDE: " + ",".join("%6d" % lon for lon in LONS) + ","]
    for i, lat in enumerate(LATS):
        row = "\t/* LAT: %3d */ {" % lat + ",".join("%6d" % v for v in values[i]) + ", },"
        if blackout and blackout[i]:
            row += " // WARNING! black out zone"
        lines.append(row)
    lines.append("};")
    lines.append("static constexpr float %s = %.9gf;" % (scale_name, scale))
    lo = extreme(raw, lambda a, b: a < b)
    hi = extreme(raw, lambda a, b: a > b)
    lines.append("static constexpr float %s_MIN_%s = %.1ff; // latitude: %d, longitude: %d"
                 % (scale_name.split("_SCALE")[0], unit, lo[0], lo[1], lo[2]))
    lines.append("static constexpr float %s_MAX_%s = %.1ff; // latitude: %d, longitude: %d"
                 % (scale_name.split("_SCALE")[0], unit, hi[0], hi[1], hi[2]))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cof")
    ap.add_argument("--date", type=float, required=True, help="decimal year the tables are evaluated at")
    ap.add_argument("--out", help="output header (default: stdout)")
    args = ap.parse_args()

    epoch, model, released, coeffs = read_cof(args.cof)
    if not (epoch <= args.date <= epoch + 5.0):
        sys.exit("date %.3f is outside the model validity %.1f-%.1f" % (args.date, epoch, epoch + 5.0))
    dec, inc, f = tables(coeffs, epoch, args.date)
    dec_q, dec_s = quantise(dec)
    inc_q, inc_s = quantise(inc)
    f_q, f_s = quantise(f)
    # mark rows where the horizontal intensity drops below 6000 nT (WMM caution/blackout zones,
    # where compass heading is unreliable); same criterion as PX4
    blackout = [any(f[i][j] * math.cos(math.radians(inc[i][j])) < 6000 for j in range(len(LONS)))
                for i in range(len(LATS))]

    out = []
    out.append("// Copyright (c) Cosys-Lab, University of Antwerp. 2026")
    out.append("// GENERATED by scripts/wmm/generate_wmm_tables.py - do not edit by hand.")
    out.append("// Source: %s coefficients (epoch %.1f, released %s), NOAA NCEI / BGS, public domain."
               % (model, epoch, released))
    out.append("// Layout matches PX4's geo_magnetic_tables.hpp (10 degree grid, int16 scaled values).")
    out.append("")
    out.append("#ifndef msr_airlib_WorldMagneticModelTables_hpp")
    out.append("#define msr_airlib_WorldMagneticModelTables_hpp")
    out.append("")
    out.append("#include <stdint.h>")
    out.append("")
    out.append("namespace msr")
    out.append("{")
    out.append("namespace airlib")
    out.append("{")
    out.append("namespace wmm")
    out.append("{")
    out.append("")
    out.append("static constexpr const char* WMM_MODEL_NAME = \"%s\";" % model)
    out.append("static constexpr float WMM_TABLE_DATE = %.4ff; // decimal year the tables are evaluated at" % args.date)
    out.append("static constexpr float WMM_VALID_FROM = %.1ff;" % epoch)
    out.append("static constexpr float WMM_VALID_UNTIL = %.1ff;" % (epoch + 5.0))
    out.append("")
    out.append("static constexpr float SAMPLING_RES = %d;" % SAMPLING_RES)
    out.append("static constexpr float SAMPLING_MIN_LAT = -90;")
    out.append("static constexpr float SAMPLING_MAX_LAT = 90;")
    out.append("static constexpr float SAMPLING_MIN_LON = -180;")
    out.append("static constexpr float SAMPLING_MAX_LON = 180;")
    out.append("static constexpr int LAT_DIM = %d;" % len(LATS))
    out.append("static constexpr int LON_DIM = %d;" % len(LONS))
    out.append("")
    out.append("// clang-format off")
    out.append(format_table("declination_table", dec_q, "Magnetic declination data in %.6f degrees" % dec_s,
                            "WMM_DECLINATION_SCALE_TO_DEGREES", dec_s, "DEGREES", model, args.date, dec, blackout))
    out.append("")
    out.append(format_table("inclination_table", inc_q, "Magnetic inclination data in %.6f degrees" % inc_s,
                            "WMM_INCLINATION_SCALE_TO_DEGREES", inc_s, "DEGREES", model, args.date, inc, blackout))
    out.append("")
    out.append(format_table("totalintensity_table", f_q, "Magnetic strength data in %.6f nanotesla" % f_s,
                            "WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA", f_s, "NANOTESLA", model, args.date, f))
    out.append("// clang-format on")
    out.append("")
    out.append("}")
    out.append("}")
    out.append("} //namespace")
    out.append("#endif")
    text = "\n".join(out) + "\n"
    if args.out:
        with open(args.out, "w", newline="\n") as fh:
            fh.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
