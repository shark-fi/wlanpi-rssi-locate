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
