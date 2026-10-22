import copy
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
import unittest

from relay_learning.features import FeatureSpec, LINKS, SCHEMA
from .dataset import samples_from_episode, real_episodes
from .model import create_model


def frames():
    result = []
    for i in range(81):
        now = i * .1
        state = dict(main_position=(0., 1.), relay_positions=((1., 2.), (2., 4.)),
                     relay_targets=((1., 2.), (2., 4.)), relay_ready=(True, True),
                     relay_eta_s=(0., 0.), relay_battery=(1., 1.), direction="outbound",
                     progress_cells=1., phase="FLY", main_speed_mps=1., cell_m=1.)
        result.append(dict(schema_version=SCHEMA, source="simulation", episode_id="test", split_group="day-1",
                           time_s=now, state=state, plan_epoch=1,
                           links=[dict(link_id=link, rssi_dbm=-60., sample_time_s=now, valid=True)
                                  for link in LINKS], label_rssi_dbm=dict.fromkeys(LINKS, -60.)))
    return result


class DatasetTests(unittest.TestCase):
    def test_nan_future_synthetic_label_is_masked_not_good(self):
        data = frames()
        data[10]["label_rssi_dbm"]["G-M"] = float("nan")
        _, _, mask, times, links = samples_from_episode(data, FeatureSpec(), 1)
        # Both horizons for G-M at t=0 are unknown, so that row is omitted.
        self.assertFalse(any(t == 0. and link == 0 for t, link in zip(times, links)))

    def test_missing_real_link_and_timestamp_are_unknown_labels(self):
        for fault in ("link", "timestamp"):
            data = frames()
            for frame in data:
                frame["source"] = "real"
                del frame["label_rssi_dbm"]
            if fault == "link":
                data[30]["links"].pop(0)
            else:
                del data[30]["links"][0]["sample_time_s"]
            _, _, mask, _, _ = samples_from_episode(data, FeatureSpec(), 1)
            self.assertEqual(mask[0], [1., 0.])

    def test_future_changes_labels_but_not_present_inputs(self):
        data = frames()
        before = samples_from_episode(data, FeatureSpec(), 1)
        data[10]["label_rssi_dbm"]["G-M"] = -95.
        data[10]["links"][0]["rssi_dbm"] = -95.
        after = samples_from_episode(data, FeatureSpec(), 1)
        self.assertEqual(before[0][:6], after[0][:6])
        self.assertEqual(before[1][0], [0., 0.])
        self.assertEqual(after[1][0], [1., 1.])

    def test_changed_plan_censors_future_labels(self):
        data = frames()
        for frame in data[30:]:
            frame["plan_epoch"] = 2
        x, y, mask, times, links = samples_from_episode(data, FeatureSpec(), 1)
        self.assertEqual(mask[0], [1., 0.])
        self.assertTrue(all(t <= 6.5 + 1e-6 for t in times))

    def test_missing_future_real_measurements_are_not_negative_labels(self):
        data = frames()
        for frame in data:
            frame["source"] = "real"
            del frame["label_rssi_dbm"]
        data[30]["links"][0]["valid"] = False
        _, _, mask, _, _ = samples_from_episode(data, FeatureSpec(), 1)
        self.assertEqual(mask[0], [1., 0.])
        data[30]["links"][0]["valid"] = True
        data[30]["links"][0]["sample_time_s"] = 2.
        _, _, mask, _, _ = samples_from_episode(data, FeatureSpec(), 1)
        self.assertEqual(mask[0], [1., 0.])

    def test_incomplete_long_horizon_is_masked(self):
        x, y, mask, times, links = samples_from_episode(frames()[:31], FeatureSpec(), 1)
        self.assertTrue(all(m == [1., 0.] for m in mask))
        self.assertEqual(max(times), 1.5)

    def test_real_import_refuses_synthetic_truth_and_mixed_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "telemetry.jsonl"
            data = frames()[:2]
            path.write_text("\n".join(json.dumps(f) for f in data), encoding="utf-8")
            with self.assertRaises(ValueError):
                real_episodes({"telemetry_jsonl": str(path)})
            for frame in data:
                frame["source"] = "real"
                del frame["label_rssi_dbm"]
            data[1]["split_group"] = "day-2"
            path.write_text("\n".join(json.dumps(f) for f in data), encoding="utf-8")
            with self.assertRaises(ValueError):
                real_episodes({"telemetry_jsonl": str(path)})

    def test_new_model_shape_is_two_horizons_not_action_commands(self):
        import torch
        model = create_model({"feature_spec": asdict(FeatureSpec()), "hidden_sizes": [64, 32]})
        self.assertEqual(tuple(model(torch.zeros(6, 96)).shape), (6, 2))
        self.assertEqual(sum(p.numel() for p in model.parameters()), 8354)


if __name__ == "__main__":
    unittest.main()
