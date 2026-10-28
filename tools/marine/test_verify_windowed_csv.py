#!/usr/bin/env python3
"""Negative cases for the v0.20 comparison CSV contract."""
import csv
import io
import unittest

from verify_windowed_csv import COLUMNS, PARAMETERS, PROFILES, verify


def make_csv(changes=None):
    records = []
    for environment in PROFILES:
        for parameter in PARAMETERS:
            records.append({
                "profile": environment, "parameter": parameter,
                "window_samples": 25, "trials": 60,
                "instant_accepted": 60, "windowed_accepted": 60,
                "truth": 40.0, "instant_mean": 40.0, "windowed_mean": 40.0,
                "instant_relative_bias": 0.0, "windowed_relative_bias": 0.0,
                "windowed_p05": 40.0, "windowed_p95": 40.0,
            })
    for index, updates in (changes or {}).items():
        records[index].update(updates)
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(records)
    return output.getvalue()


class WindowedCsvTests(unittest.TestCase):
    def test_valid(self):
        verify(make_csv())

    def test_incomplete(self):
        with self.assertRaises(ValueError):
            verify("\n".join(make_csv().splitlines()[:-1]) + "\n")

    def test_duplicate(self):
        with self.assertRaises(ValueError):
            verify(make_csv({1: {"parameter": "mass_kg"}}))

    def test_nan(self):
        with self.assertRaises(ValueError):
            verify(make_csv({5: {"windowed_mean": float("nan")}}))

    def test_bad_denominator(self):
        with self.assertRaises(ValueError):
            verify(make_csv({5: {"truth": 0.0}}))

    def test_bad_bias(self):
        with self.assertRaises(ValueError):
            verify(make_csv({8: {"windowed_relative_bias": 0.1}}))

    def test_bad_window(self):
        with self.assertRaises(ValueError):
            verify(make_csv({6: {"window_samples": 0}}))

    def test_inverted_percentiles(self):
        with self.assertRaises(ValueError):
            verify(make_csv({8: {"windowed_p05": 41.0}}))


if __name__ == "__main__":
    unittest.main()
