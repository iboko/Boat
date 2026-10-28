#!/usr/bin/env python3
"""Strict negative tests for v0.19 Monte Carlo output validation."""
import csv
import io
import unittest

from verify_noise_csv import COLUMNS, PARAMETERS, PROFILES, verify


def result(overrides=None):
    rows = []
    for profile, (speed, rate, heading) in PROFILES.items():
        for parameter in PARAMETERS:
            rows.append({
                "profile": profile, "parameter": parameter,
                "speed_sigma_mps": speed, "yaw_rate_sigma_radps": rate,
                "heading_sigma_rad": heading, "trials": 60,
                "accepted": 60, "rejected": 0, "truth": 40,
                "mean": 40, "relative_bias": 0, "p05": 40, "p95": 40,
            })
    for index, updates in (overrides or {}).items():
        rows[index].update(updates)
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return output.getvalue()


class NoiseCsvTests(unittest.TestCase):
    def test_valid(self):
        verify(result())

    def test_missing(self):
        with self.assertRaises(ValueError):
            verify("\n".join(result().splitlines()[:-1]) + "\n")

    def test_duplicate(self):
        with self.assertRaises(ValueError):
            verify(result({1: {"parameter": "mass_kg"}}))

    def test_nan(self):
        with self.assertRaises(ValueError):
            verify(result({0: {"mean": float("nan")}}))

    def test_bad_counts(self):
        with self.assertRaises(ValueError):
            verify(result({0: {"accepted": 59}}))

    def test_bad_bias(self):
        with self.assertRaises(ValueError):
            verify(result({8: {"relative_bias": 0.3}}))

    def test_bad_interval(self):
        with self.assertRaises(ValueError):
            verify(result({8: {"p05": 45}}))

    def test_bad_schema(self):
        with self.assertRaises(ValueError):
            verify("wrong," + result())


if __name__ == "__main__":
    unittest.main()
