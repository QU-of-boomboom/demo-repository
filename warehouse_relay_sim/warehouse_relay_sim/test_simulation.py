"""A tests exercise the maintained API; historical V1 tests live in legacy_v1."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
from test_abc_alignment import (ExecutionAcceptance, GeometryAcceptance, MotionGateAcceptance,
                                ObservationAcceptance, RuleAcceptance)
from test_diff_contract import ProfileContractTests, StandardOutboundDiff

if __name__ == "__main__":
    import unittest
    unittest.main()
