#!/usr/bin/env python3
"""Negative tests for the strict synthetic-identification result verifier."""
import csv
import io
import unittest

from verify_identification_csv import NAMES, verify


def make_result(overrides=None):
    data = {
        "mass_true_kg": 78.0, "mass_estimated_kg": 78.0,
        "surge_drag_true_ns_per_m": 27.0, "surge_drag_estimated_ns_per_m": 27.0,
        "yaw_inertia_true_kgm2": 34.0, "yaw_inertia_estimated_kgm2": 34.0,
        "yaw_drag_true_nms_per_rad": 23.0, "yaw_drag_estimated_nms_per_rad": 23.0,
        "surge_excitation": 0.137, "yaw_excitation": 0.313,
        "surge_fit_rmse_mps2": 0.0, "yaw_fit_rmse_radps2": 0.0,
        "holdout_position_rmse_m": 0.0, "holdout_heading_rmse_rad": 0.0,
    }
    data.update(overrides or {})
    output = io.StringIO()
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(NAMES)
    writer.writerow([data[name] for name in NAMES])
    return output.getvalue()


class IdentificationCsvTests(unittest.TestCase):
    def test_valid(self):
        verify(make_result())

    def test_missing(self):
        with self.assertRaises(ValueError):
            verify(make_result().splitlines()[0] + "\n")

    def test_duplicate(self):
        with self.assertRaises(ValueError):
            verify(make_result() + make_result().splitlines()[1] + "\n")

    def test_nan(self):
        with self.assertRaises(ValueError):
            verify(make_result({"mass_estimated_kg": float("nan")}))

    def test_wrong_estimate(self):
        with self.assertRaises(ValueError):
            verify(make_result({"yaw_drag_estimated_nms_per_rad": 40.0}))

    def test_rank_failure(self):
        with self.assertRaises(ValueError):
            verify(make_result({"surge_excitation": 0.0}))

    def test_holdout_failure(self):
        with self.assertRaises(ValueError):
            verify(make_result({"holdout_position_rmse_m": 2.0}))

    def test_bad_schema(self):
        with self.assertRaises(ValueError):
            verify("wrong," + make_result())


if __name__ == "__main__":
    unittest.main()
