"""How the sensor resolves its collector token.

Stdlib only, no test framework to install:

    python3 -m unittest discover -s tests -v
"""
import argparse
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rssi_sensor import resolve_token


def args(**kw):
    base = {"token": ""}
    base.update(kw)
    return argparse.Namespace(**base)


class TokenResolution(unittest.TestCase):
    """A token on the command line sits in `ps` for every user on the host.

    Same defect as the exporter's console password in #9, with a longer
    exposure: a sensor daemon runs for weeks, a scheduled subprocess for
    seconds. The flag exists because people expect it, and the docs point at
    the environment.
    """

    def setUp(self):
        self._saved = os.environ.pop("SENSOR_TOKEN", None)

    def tearDown(self):
        os.environ.pop("SENSOR_TOKEN", None)
        if self._saved is not None:
            os.environ["SENSOR_TOKEN"] = self._saved

    def test_the_environment_is_used_when_no_flag_is_given(self):
        os.environ["SENSOR_TOKEN"] = "from-env"
        self.assertEqual(resolve_token(args()), "from-env")

    def test_the_flag_wins_when_both_are_set(self):
        os.environ["SENSOR_TOKEN"] = "from-env"
        self.assertEqual(resolve_token(args(token="from-flag")), "from-flag")

    def test_no_token_anywhere_is_allowed(self):
        """The standalone locator wants none; refusing to start would break it."""
        self.assertEqual(resolve_token(args()), "")

    def test_surrounding_whitespace_is_stripped(self):
        """A token pasted into a systemd unit or .env usually brings a newline."""
        os.environ["SENSOR_TOKEN"] = "  abc123\n"
        self.assertEqual(resolve_token(args()), "abc123")


if __name__ == "__main__":
    unittest.main()


from rssi_sensor import (
    ble_addr, ble_local_name, parse_le_advertising_report,
    HCI_EVENT_PKT, HCI_LE_META, LE_ADVERTISING_REPORT,
)


def le_report(addr, rssi, ad=b"", addr_type=0):
    """One report inside an LE Advertising Report event."""
    return (bytes([0x00, addr_type]) + bytes(reversed(addr)) + bytes([len(ad)])
            + ad + bytes([rssi & 0xFF]))


def le_event(*reports):
    return (bytes([HCI_EVENT_PKT, HCI_LE_META, 0, LE_ADVERTISING_REPORT,
                   len(reports)]) + b"".join(reports))


class BleAddress(unittest.TestCase):
    def test_bytes_come_off_the_wire_reversed(self):
        self.assertEqual(ble_addr(bytes([0x4e, 0x3d, 0x1f, 0x38, 0xc1, 0xa4])),
                         "a4:c1:38:1f:3d:4e")

    def test_the_form_matches_what_the_collector_keys_on(self):
        """Lowercase, colon-separated — the same shape the Wi-Fi path emits,
        because the collector looks up one dict for both."""
        mac = ble_addr(bytes(range(6)))
        self.assertEqual(mac, mac.lower())
        self.assertEqual(len(mac.split(":")), 6)


class BleAdvertisingReport(unittest.TestCase):
    """Decoding what an adapter actually hands back.

    The RSSI is the LAST byte of a report, after a variable-length AD payload,
    so anything that reads it from a fixed offset works on the first packet and
    lies on the next.
    """

    def test_a_single_advertiser(self):
        pkt = le_event(le_report(b"\xa4\xc1\x38\x1f\x3d\x4e", -80))
        self.assertEqual(parse_le_advertising_report(pkt),
                         [("a4:c1:38:1f:3d:4e", -80.0, None)])

    def test_rssi_is_read_after_the_payload_not_at_an_offset(self):
        long_ad = bytes([0x1e, 0xff]) + b"\x4c\x00" + b"\x00" * 27   # 31-byte AD
        pkt = le_event(le_report(b"\x79\xac\x67\x25\x70\x1f", -63, long_ad))
        self.assertEqual(parse_le_advertising_report(pkt)[0][1], -63.0)

    def test_several_reports_in_one_event(self):
        pkt = le_event(le_report(b"\xaa" * 6, -40),
                       le_report(b"\xbb" * 6, -95, b"\x02\x01\x06"))
        got = parse_le_advertising_report(pkt)
        self.assertEqual([r[1] for r in got], [-40.0, -95.0])

    def test_a_name_is_extracted_when_advertised(self):
        ad = bytes([0x07, 0x09]) + b"Govee1"
        pkt = le_event(le_report(b"\xa4\xc1\x38\x1f\x3d\x4e", -70, ad))
        self.assertEqual(parse_le_advertising_report(pkt)[0][2], "Govee1")

    def test_a_shortened_name_also_counts(self):
        ad = bytes([0x04, 0x08]) + b"Gov"
        pkt = le_event(le_report(b"\xaa" * 6, -70, ad))
        self.assertEqual(parse_le_advertising_report(pkt)[0][2], "Gov")

    def test_other_hci_events_are_ignored_not_misread(self):
        """The read loop hands every packet here, so this must be silent."""
        for pkt in (bytes([HCI_EVENT_PKT, 0x0E, 0x04, 0x01, 0x0C, 0x20, 0x00]),
                    bytes([HCI_EVENT_PKT, HCI_LE_META, 0, 0x01, 0x00]),
                    b"", b"\x04", bytes([0x02, 0x00, 0x00])):
            self.assertEqual(parse_le_advertising_report(pkt), [])

    def test_a_truncated_packet_does_not_raise(self):
        """A short read must lose the report, not the scan."""
        full = le_event(le_report(b"\xaa" * 6, -50, b"\x02\x01\x06"))
        for n in range(len(full)):
            parse_le_advertising_report(full[:n])

    def test_a_positive_byte_is_a_negative_dbm(self):
        """RSSI is signed; 0xB0 is -80, not 176."""
        pkt = le_event(le_report(b"\xaa" * 6, -80))
        self.assertEqual(parse_le_advertising_report(pkt)[0][1], -80.0)


class BleLocalName(unittest.TestCase):
    def test_no_name_is_none_rather_than_empty(self):
        self.assertIsNone(ble_local_name(b"\x02\x01\x06"))

    def test_a_zero_length_structure_terminates_instead_of_looping(self):
        self.assertIsNone(ble_local_name(b"\x00\x00\x00"))


class ImportsUsedAtRuntime(unittest.TestCase):
    """Every module the file references must actually be imported.

    ble_capture() reached `struct.pack` on a NameError because `struct` was
    never imported. Nothing caught it: the selftest exercises the parser, the
    unit tests exercise pure functions, and the socket path only runs on real
    hardware with root. It failed on a WLAN Pi, in a thread, after deployment.
    """

    def test_every_module_referenced_is_imported(self):
        import ast
        import os

        src = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "rssi_sensor.py")
        tree = ast.parse(open(src).read())
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    imported.add((a.asname or a.name).split(".")[0])
            elif isinstance(node, ast.ImportFrom):
                for a in node.names:
                    imported.add(a.asname or a.name)

        # every `name.attr` where `name` looks like a stdlib module we use
        used = set()
        for node in ast.walk(tree):
            if (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)):
                used.add(node.value.id)

        builtins_and_locals = used - imported
        suspects = {n for n in builtins_and_locals
                    if n in {"struct", "socket", "json", "math", "os", "sys",
                             "time", "threading", "subprocess", "argparse",
                             "urllib", "getpass", "hashlib", "re", "zipfile"}}
        self.assertEqual(suspects, set(),
                         f"used but not imported: {sorted(suspects)}")


class HciFilterStruct(unittest.TestCase):
    """The socket option the kernel accepts, byte for byte.

    setsockopt returned EINVAL on a real WLAN Pi because the filter was packed
    from its fields (14 bytes) rather than the struct's sizeof (16). "Invalid
    argument" says nothing about padding, and nothing in the selftest or the
    unit tests touches a socket — so it took a deployment to find.
    """

    def test_it_is_the_sixteen_bytes_sizeof_reports(self):
        from rssi_sensor import hci_filter
        self.assertEqual(len(hci_filter(0x3E)), 16,
                         "the kernel rejects anything shorter than sizeof")

    def test_a_high_event_code_sets_the_upper_mask(self):
        """LE Meta is 0x3E — bit 62, in the second word of the 64-bit mask."""
        import struct as _s
        from rssi_sensor import hci_filter
        type_mask, lo, hi, opcode = _s.unpack("<LLLH2x", hci_filter(0x3E))
        self.assertEqual(lo, 0)
        self.assertEqual(hi, 1 << (0x3E - 32))
        self.assertEqual(type_mask, 1 << 0x04, "HCI event packets only")
        self.assertEqual(opcode, 0)

    def test_a_low_event_code_sets_the_lower_mask(self):
        import struct as _s
        from rssi_sensor import hci_filter
        _t, lo, hi, _o = _s.unpack("<LLLH2x", hci_filter(0x05))
        self.assertEqual((lo, hi), (1 << 5, 0))
