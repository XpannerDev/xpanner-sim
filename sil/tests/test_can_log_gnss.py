"""
Real machine CAN bytes -> the firmware's GNSS fix-type gate. The only test in this suite whose inputs
came off a real X1 excavator rather than a model.

WHERE THE BYTES CAME FROM
  X1Exc/ControlModel/Data/TestLog/TSMaster2026_05_03_08_46_08.blf -- a 32.2 s CAN2 capture of a
  STATIONARY machine (9,660 frames, three arbitration IDs, GNSS only: no valve, IMU, joint-angle or
  auto-state traffic anywhere in it). Korean site, lat 35.3209 / lon 129.2164 / alt 102.6 m, which
  matches the ShortArm (1.7 m) build this harness compiles. The frames below are the first complete
  fast-packet epoch of each antenna plus the POS_ERR_STATICS frame that starts the log.
  They are embedded as hex ON PURPOSE: the .blf lives only inside the read-only firmware clone, so a
  path dependency would make this test unrunnable for anyone else.

WHAT THE ECU DOES WITH THEM (read-only reference code, not compiled into this harness)
  CanCtrl.c:2460-2530     NMEA-2000 PGN 129029 fast packet: seqNr = data[0] & 0x1F; frame 0 carries
                          [seq, size, 6 payload bytes]; frame n>=1 carries 7 payload bytes written at
                          6 + (n-1)*7 into NmeaFastPacket.dataBuffer[47] (CanCtrl.h:195, :206).
  CanCtrl.c:5550-5560     UnpackSigFromArray(dataArray, 248, 4) -> sig_GnssTyp
                          UnpackSigFromArray(dataArray, 252, 4) -> sig_Method
  InpHndlr.c:1118-1119    ip.TypeOfSystem_Ant1 = sig_Method
                          ip.MethodGNSS_Ant1   = sig_GnssTyp      <-- the swap this file pins
  PrePostProc_If.c        INTP.MethodGNSS_Ant1/2 -> u.methodGnss_Main/_Aux

WHY IT MATTERS
  u.methodGnss_* == 4 is the ONLY gate on CalibTrvl (isGnssBasedCalibInhibited, MdlApp.c:39683, used
  at :22369 / :21977). In NMEA-2000 PGN 129029 the field at bits 252-255 is the GNSS METHOD (1 = GNSS
  fix, 4 = RTK fixed) and bits 248-251 are the constellation TYPE. This recording carries
  Method = 1 (an ordinary autonomous fix, NOT RTK) and GnssTyp = 4 -- so the ECU hands the model a 4
  and the RTK gate opens on a non-RTK fix. TestFixTypeSwapOpensTheRtkGate below runs both wirings
  through the compiled firmware and shows the difference is a calibration that starts vs one that does
  not. The poor-accuracy inhibit does not cover for it: the same recording's height sigma is 2.073 m,
  which inhibits AUTO but is not in CALIB_INHIBIT_MASK.
"""
import unittest

from sil.harness import Harness

# -- the recorded frames (hex payloads, in arrival order) ----------------------------------------
MAIN_EPOCH = ["002f785f50e41ffb", "0132004eca7f48d9", "02e604aab12304bd", "03b0ee11a4a61d06",
              "040000000014fc21", "0532005a00000000", "0600010d000000"]
AUX_EPOCH = ["002f785f50e41ffb", "0132aa7e271054d9", "02e604d5021a8de1", "03b0ee11684c1d06",
             "040000000014fc20", "0532005a00000000", "0600010d000000"]
POS_ERR_STATICS = "77d2041703190800"          # SeqID 119, lat/lon/height sigma at 0.001 m per bit

BUFFER_SIZE = 47                              # NMEA_PGN129029_DATA_SIZE, CanCtrl.h:195
GNSS_TYP_BIT, METHOD_BIT = 248, 252           # CanCtrl.c:5555, :5557
LOGGED_STD_DEV_Z = 2.073                      # GnssErrHeightSigma of POS_ERR_STATICS, in metres


def reassemble(frames):
    """The ECU's own fast-packet assembly (CanCtrl.c:5455-5470). Returns (declared_size, buffer,
    last_write_end): last_write_end is the index one past the last byte written, which is what makes
    the overflow visible -- the function writes dataLenPerFrame = 7 bytes for EVERY frame including
    the last (CanCtrl.c:2472), with no clamp against BUFFER_SIZE."""
    buf, size, end = bytearray(BUFFER_SIZE), None, 0
    for hexs in frames:
        d = bytes.fromhex(hexs)
        seq = d[0] & 0x1F
        if seq == 0:
            size, payload, pos = d[1], d[2:8], 0
        else:
            payload, pos = d[1:8], 6 + (seq - 1) * 7
        buf[pos:pos + len(payload)] = payload
        end = max(end, pos + 7)               # 7, not len(payload): the ECU always writes 7
    return size, bytes(buf), end


def sig(buf, start, length, signed=False):
    """UnpackSigFromArray: LSB-first, little-endian bit order."""
    v = 0
    for i in range(length):
        bit = start + i
        v |= ((buf[bit // 8] >> (bit % 8)) & 1) << i
    return v - (1 << length) if signed and v >> (length - 1) else v


def logged_sigmas(hexs):
    """POS_ERR_STATICS (CAN2_X1Exc.dbc:109): u8 seq, then three u16 at 0.001 m/bit."""
    d = bytes.fromhex(hexs)
    return dict(seq=d[0], lat=sig(d, 8, 16) * 1e-3, lon=sig(d, 24, 16) * 1e-3, height=sig(d, 40, 16) * 1e-3)


class TestRecordedFramesDecode(unittest.TestCase):
    """The bytes say what the firmware reference code claims they say."""

    def test_fast_packet_reassembles_to_the_declared_47_bytes(self):
        for name, frames in (("main", MAIN_EPOCH), ("aux", AUX_EPOCH)):
            with self.subTest(antenna=name):
                size, buf, _ = reassemble(frames)
                self.assertEqual(size, BUFFER_SIZE, "declared payload size, frame 0 byte 1")
                self.assertEqual(len(buf), BUFFER_SIZE)
                self.assertEqual([bytes.fromhex(f)[0] & 0x1F for f in frames], list(range(7)),
                                 "one clean epoch, sequence 0..6, no resync")

    def test_the_recorded_fix_is_autonomous_not_rtk(self):
        # THE PREMISE OF THIS FILE. Method (bits 252-255) = 1 = "GNSS fix"; the 4 sitting next to it
        # at bits 248-251 is the constellation type (GPS+GLONASS+GALILEO+SBAS), not a fix quality.
        for name, frames in (("main", MAIN_EPOCH), ("aux", AUX_EPOCH)):
            with self.subTest(antenna=name):
                _, buf, _ = reassemble(frames)
                self.assertEqual(sig(buf, METHOD_BIT, 4), 1, "sig_Method: 1 = autonomous GNSS fix")
                self.assertEqual(sig(buf, GNSS_TYP_BIT, 4), 4, "sig_GnssTyp: constellation type")

    def test_position_is_the_korean_site_and_both_antennas_agree(self):
        # Sanity that the bit layout is right: a wrong offset does not land on Busan to 1e-8 deg.
        pos = {}
        for name, frames in (("main", MAIN_EPOCH), ("aux", AUX_EPOCH)):
            _, buf, _ = reassemble(frames)
            pos[name] = (sig(buf, 56, 64, True) * 1e-16, sig(buf, 120, 64, True) * 1e-16,
                         sig(buf, 184, 64, True) * 1e-6)
        self.assertAlmostEqual(pos["main"][0], 35.32085262, places=8)
        self.assertAlmostEqual(pos["main"][1], 129.21644690, places=8)
        self.assertAlmostEqual(pos["main"][2], 102.6065, places=4)
        self.assertLess(abs(pos["main"][0] - pos["aux"][0]), 1e-4, "same machine, 1.5 m apart")
        self.assertLess(abs(pos["main"][2] - pos["aux"][2]), 0.5)

    def test_last_frame_of_the_epoch_overruns_the_ecu_buffer_by_one_byte(self):
        # FINDING (ECU, latent): the assembler writes 7 bytes for frame 6 at position 41 -> indices
        # 41..47, but dataBuffer is [47], so index 47 is one past the end. It is reached whenever a
        # full 47-byte payload arrives, which is every epoch in this recording (644 of them).
        # The last frame only carries 7 CAN bytes (DLC 7 = 1 seq byte + 6 payload), so the 7th byte
        # the ECU copies is not even received data.
        self.assertEqual(len(bytes.fromhex(MAIN_EPOCH[6])), 7, "frame 6 DLC as recorded")
        _, _, end = reassemble(MAIN_EPOCH)
        self.assertEqual(end, BUFFER_SIZE + 1, "writes one byte past dataBuffer[47]")

    def test_logged_error_sigmas_are_metre_class(self):
        s = logged_sigmas(POS_ERR_STATICS)
        self.assertEqual(s["seq"], 119)
        self.assertAlmostEqual(s["height"], LOGGED_STD_DEV_Z, places=3)
        self.assertGreater(s["lat"], 1.0)              # 1.234 m: nowhere near RTK, consistent with Method 1
        self.assertGreater(s["height"], 0.04, "far above the firmware's poor-accuracy threshold")


class TestFixTypeSwapOpensTheRtkGate(unittest.TestCase):
    """FINDING (ECU, confirmed against the compiled model): with this recording's real bytes, the
    as-built wiring lets CalibTrvl start on a non-RTK fix; the wiring the field names imply does not.
    FW: isGnssBasedCalibInhibited = (methodGnss_Main ~= 4 || methodGnss_Aux ~= 4) (MdlApp.c:39683),
    the only gate on CalibTrvl (:22369, :21977)."""

    def run_travel(self, method_value):
        h = Harness().reset().nominal_inputs().set_swing_aligned(True)
        h.fw["u.methodGnss_Main"] = method_value
        h.fw["u.methodGnss_Aux"] = method_value
        h.fw["u.gnssPosStdDevZ"] = LOGGED_STD_DEV_Z      # the sigma recorded in the same second
        h.fw["u.isMachCalib"] = 1                        # the only state where CALIB_INHIBIT_MASK bites
        h.tick(30)                                       # past the 20-tick accuracy on-delay
        h.jump_to_step("CalibTrvl")
        h.tick(2)
        return h

    def test_as_built_the_constellation_nibble_opens_the_rtk_gate(self):
        _, buf, _ = reassemble(MAIN_EPOCH)
        as_built = sig(buf, GNSS_TYP_BIT, 4)             # InpHndlr.c:1119 feeds THIS to methodGnss
        self.assertEqual(as_built, 4)
        h = self.run_travel(as_built)
        self.assertTrue(h.fw["y.isCalibrating"], "CalibTrvl started on a fix that was not RTK: " + h.describe())

    def test_with_the_real_fix_method_the_gate_stays_shut(self):
        _, buf, _ = reassemble(MAIN_EPOCH)
        as_named = sig(buf, METHOD_BIT, 4)               # what "MethodGNSS" should carry
        self.assertEqual(as_named, 1)
        h = self.run_travel(as_named)
        self.assertFalse(h.fw["y.isCalibrating"], "non-RTK fix must not start CalibTrvl: " + h.describe())

    def test_the_accuracy_inhibit_does_not_cover_for_the_swap(self):
        # Auto IS inhibited by the 2.073 m sigma -- but that bit is in AUTO_INHIBIT_MASK, not
        # CALIB_INHIBIT_MASK (MdlApp.c:39679), so the calibration runs anyway.
        h = self.run_travel(4)
        self.assertTrue(h.fw.internal("low_vertical_accuracy"))
        self.assertTrue(h.auto_inhibited())
        self.assertTrue(h.fw["y.isCalibrating"])


if __name__ == "__main__":
    unittest.main()
