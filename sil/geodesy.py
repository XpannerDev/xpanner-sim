"""
geodesy.py -- world metres <-> the geodetic inputs the X1Exc Localization chart expects.

Lifted from sil/tests/test_gnss_localization.py, where it reproduces y.links.chs.p against
the firmware to float32 resolution.

THE SITE CALIBRATION IS AN INPUT, NOT A PARAMETER. Localization is called with the
u.parProj / u.parHorAdj / u.parVerAdj / u.parDatumTrans / u.parEllTar / u.siteOrigin inports
(MdlApp.c:41996-42020); only parEllSrc comes from parLocalTest. Left at zero (initialize()
default, and the ECU's NVM default) the machine is parked at the site origin facing grid
East whatever the antennas say -- with no inhibit.

Site() writes the "bare TM" collapse recommended in spec A3.3: identity datum and horizontal
Helmert, zero vertical plane, no geoid, TM with k0 = 1 and no false origin, projection origin
at the scene reference. Site E/N/H then equals world X/Y/Z.

Heading: chassis yaw psi is CCW about +Up from grid East; y.machHeading = wrap(pi/2 - psi),
the clockwise-from-North azimuth of chassis +X. Only the baseline DIRECTION matters.
"""
import math

import numpy as np

WGS84_A = 6378137.0
WGS84_FINV = 298.257223563
WGS84_F = 1.0 / WGS84_FINV
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_ESQ = WGS84_F * (2.0 - WGS84_F)


class KruegerTM:
    """Transverse Mercator via the Krueger n-series to n^6 (Karney, J. Geodesy 85 (2011)
    475-485, eqs. 35/36). k0 = 1 unless given; no false origin; northing from lat0."""

    def __init__(self, lat0, lon0, k0=1.0, a=WGS84_A, finv=WGS84_FINV):
        f = 1.0 / finv
        self.e = math.sqrt(f * (2.0 - f))
        n = f / (2.0 - f)
        n2, n3, n4, n5, n6 = n ** 2, n ** 3, n ** 4, n ** 5, n ** 6
        self.A = a / (1 + n) * (1 + n2 / 4 + n4 / 64 + n6 / 256)
        self.alpha = [
            n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288 + 7891 * n6 / 37800,
            13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630 - 1983433 * n6 / 1935360,
            61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880 + 167603 * n6 / 181440,
            49561 * n4 / 161280 - 179 * n5 / 168 + 6601661 * n6 / 7257600,
            34729 * n5 / 80640 - 3418889 * n6 / 1995840,
            212378941 * n6 / 319334400,
        ]
        self.beta = [
            n / 2 - 2 * n2 / 3 + 37 * n3 / 96 - n4 / 360 - 81 * n5 / 512 + 96199 * n6 / 604800,
            n2 / 48 + n3 / 15 - 437 * n4 / 1440 + 46 * n5 / 105 - 1118711 * n6 / 3870720,
            17 * n3 / 480 - 37 * n4 / 840 - 209 * n5 / 4480 + 5569 * n6 / 90720,
            4397 * n4 / 161280 - 11 * n5 / 504 - 830251 * n6 / 7257600,
            4583 * n5 / 161280 - 108847 * n6 / 3991680,
            20648693 * n6 / 638668800,
        ]
        self.lat0, self.lon0, self.k0 = lat0, lon0, k0
        self.y0 = 0.0
        self.y0 = self._raw(lat0, lon0)[1]

    def _raw(self, lat, lon):
        e = self.e
        lam = lon - self.lon0
        tau_p = math.sinh(math.atanh(math.sin(lat)) - e * math.atanh(e * math.sin(lat)))
        xi_p = math.atan2(tau_p, math.cos(lam))
        eta_p = math.atanh(math.sin(lam) / math.sqrt(1 + tau_p * tau_p))
        xi, eta = xi_p, eta_p
        for j, al in enumerate(self.alpha, start=1):
            xi += al * math.sin(2 * j * xi_p) * math.cosh(2 * j * eta_p)
            eta += al * math.cos(2 * j * xi_p) * math.sinh(2 * j * eta_p)
        return self.k0 * self.A * eta, self.k0 * self.A * xi

    def forward(self, lat, lon):
        x, y = self._raw(lat, lon)
        return x, y - self.y0

    def inverse(self, east, north):
        e = self.e
        xi = (north + self.y0) / (self.k0 * self.A)
        eta = east / (self.k0 * self.A)
        xi_p, eta_p = xi, eta
        for j, be in enumerate(self.beta, start=1):
            xi_p -= be * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
            eta_p -= be * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
        chi = math.asin(max(-1.0, min(1.0, math.sin(xi_p) / math.cosh(eta_p))))
        lam = math.atan2(math.sinh(eta_p), math.cos(xi_p))
        lat = chi
        for _ in range(30):
            s = math.sin(lat)
            nxt = 2 * math.atan(((1 + e * s) / (1 - e * s)) ** (e / 2)
                                * math.tan(math.pi / 4 + chi / 2)) - math.pi / 2
            done = abs(nxt - lat) < 1e-15
            lat = nxt
            if done:
                break
        return lat, self.lon0 + lam


class Site:
    """One site calibration for the u.* geodetic inports plus the matching world -> blh map.
    World (X, Y, Z) is metres relative to siteOrigin in grid E/N/up."""

    def __init__(self, lat0_deg=32.9, lon0_deg=-96.8, h0=0.0, false_e=0.0, false_n=0.0, site_origin=None):
        self.lat0, self.lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
        self.h0, self.false_e, self.false_n = h0, false_e, false_n
        self.site_origin = list(site_origin) if site_origin is not None else [0.0, 0.0, h0]
        self.tm = KruegerTM(self.lat0, self.lon0)

    def write(self, fw):
        fw["u.parEllTar.a"] = WGS84_A
        fw["u.parEllTar.b"] = WGS84_B
        fw["u.parEllTar.fInv"] = WGS84_FINV
        fw["u.parEllTar.e"] = math.sqrt(WGS84_ESQ)
        fw["u.parEllTar.eSq"] = WGS84_ESQ
        for k in ("x0", "y0", "z0", "dx", "dy", "dz", "rx", "ry", "rz"):
            fw["u.parDatumTrans." + k] = 0.0
            fw["u.parHorAdj." + k] = 0.0
        fw["u.parDatumTrans.sf"] = 1.0
        fw["u.parHorAdj.sf"] = 1.0
        for k in ("x0", "y0", "a00", "a10", "a01"):
            fw["u.parVerAdj." + k] = 0.0
        fw["u.isGeoidCorrectionEnabled"] = 0
        fw["u.parProj.type"] = 0
        fw["u.parProj.latOrg"] = self.lat0
        fw["u.parProj.lonOrg"] = self.lon0
        fw["u.parProj.sf"] = 1.0
        fw["u.parProj.false_E"] = self.false_e
        fw["u.parProj.false_N"] = self.false_n
        fw["u.siteOrigin"] = self.site_origin
        return self

    def blh(self, p):
        e = p[0] + self.site_origin[0] - self.false_e
        n = p[1] + self.site_origin[1] - self.false_n
        lat, lon = self.tm.inverse(e, n)
        return [lat, lon, p[2] + self.site_origin[2]]


def place_antennas(fw, site, main_world, R_chs):
    """Write u.blh_Main / u.blh_Aux for a chassis at attitude R_chs with the main antenna at
    world `main_world`. The aux antenna sits at par.parKin.distAntMainToAntAux in chassis axes."""
    d_aux = np.asarray(fw["par.parKin.distAntMainToAntAux"], dtype=float)
    main = np.asarray(main_world, dtype=float)
    aux = main + np.asarray(R_chs, dtype=float) @ d_aux
    fw["u.blh_Main"] = site.blh(main)
    fw["u.blh_Aux"] = site.blh(aux)
    return aux


def main_antenna_for_chassis(fw, chs_origin_world, R_chs):
    """Inverse of links.chs.p = mainAntenna + R_chs * distAntMainToChs."""
    d = np.asarray(fw["par.parKin.distAntMainToChs"], dtype=float)
    return np.asarray(chs_origin_world, dtype=float) - np.asarray(R_chs, dtype=float) @ d
