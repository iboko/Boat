#!/usr/bin/env python3
"""Negative tests: a malformed sweep must never be accepted as evidence."""
import csv
import io
import unittest

from verify_sensitivity_csv import FIELDS, PARAMETERS, validate_csv


def fixture() -> str:
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=FIELDS)
    writer.writeheader()
    for environment, baseline in (("calm", 1.0), ("current", 3.0), ("wind", 5.0)):
        for parameter in sorted(PARAMETERS):
            writer.writerow(dict(zip(FIELDS, (
                environment, parameter, "0.200000000", f"{baseline:.9f}",
                f"{baseline-0.2:.9f}", f"{baseline+0.2:.9f}",
                "1.000000000", "0.000000000"))))
    return output.getvalue()


class SensitivityCsvTests(unittest.TestCase):
    def test_complete_matrix(self):
        self.assertEqual(validate_csv(fixture())["cases"], 18)

    def test_missing_row(self):
        rows = fixture().splitlines()
        with self.assertRaisesRegex(ValueError, "incomplete"):
            validate_csv("\n".join(rows[:-1]) + "\n")

    def test_duplicate_row(self):
        rows = fixture().splitlines()
        with self.assertRaisesRegex(ValueError, "duplicate"):
            validate_csv("\n".join(rows+[rows[1]]) + "\n")

    def test_nan(self):
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            validate_csv(fixture().replace("1.000000000", "nan", 1))

    def test_falsified_derivative(self):
        with self.assertRaisesRegex(ValueError, "inconsistent"):
            validate_csv(fixture().replace(",1.000000000,0.000000000",
                                           ",500.000000000,0.000000000", 1))

    def test_bad_schema(self):
        with self.assertRaisesRegex(ValueError, "schema"):
            validate_csv(fixture().replace("environment,", "scenario,", 1))

    def test_extra_column(self):
        rows = fixture().splitlines()
        rows[1] += ",unverified"
        with self.assertRaisesRegex(ValueError, "malformed"):
            validate_csv("\n".join(rows) + "\n")


if __name__ == "__main__":
    unittest.main()
