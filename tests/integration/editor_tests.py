"""Authoring, mouse interactions, and real C++ runs exported by the editor."""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"visualization"))
from scenario_document import ScenarioDocument, corners, numbers
from editor_runner import EditorRun

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--binary", type=Path, default=ROOT/"build/bin/avoidance_sim")
ARGS, remaining = parser.parse_known_args()
sys.argv = [sys.argv[0]]+remaining

try:
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.backend_bases import MouseEvent
    from editor import ScenarioEditor
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


class DocumentTests(unittest.TestCase):
    def test_save_load_unknown_fields_and_event_references(self):
        data = json.loads((ROOT/"scenarios/communication_loss.json").read_text())
        data["future_sensor_settings"] = {"example": [1, 2]}
        data["robots"][0]["future_metadata"] = {"payload": 150}
        doc = ScenarioDocument(data)
        doc.edit(lambda: doc.rename_robot(1, "renamed"))
        self.assertEqual(doc.data["events"][0]["robot"], "renamed")
        with tempfile.TemporaryDirectory() as directory:
            path = doc.save(Path(directory)/"scene.json")
            loaded = ScenarioDocument()
            loaded.load(path)
            self.assertEqual(loaded.data, doc.data)
            self.assertEqual(loaded.data["future_sensor_settings"], data["future_sensor_settings"])
            self.assertFalse(doc.dirty)
        doc.edit(lambda: doc.delete("robot", 1))
        self.assertEqual(doc.data["events"], [])
        doc.undo()
        self.assertEqual(doc.data["events"][0]["robot"], "renamed")

    def test_transactions_and_dirty_undo_redo(self):
        doc = ScenarioDocument()
        doc.edit(lambda: doc.add_robot([0, 0]))
        original = deepcopy(doc.data)
        doc.begin()
        doc.move_waypoint(0, 1, [5, 2])
        doc.cancel()
        self.assertEqual(doc.data, original)
        doc.edit(lambda: doc.move_waypoint(0, 1, [5, 2]))
        doc.undo()
        self.assertEqual(doc.data, original)
        doc.redo()
        self.assertEqual(doc.data["robots"][0]["goal"][:2], [5, 2])
        before = deepcopy(doc.data)
        with self.assertRaises(ValueError):
            doc.edit(lambda: doc.replace_path(0, [[0, 0], [0, 0]]))
        self.assertEqual(doc.data, before)

    def test_paths_headings_and_unique_ids(self):
        doc = ScenarioDocument()
        for _ in range(3):
            doc.edit(lambda: doc.add_robot([0, 0]))
        self.assertEqual([r["id"] for r in doc.data["robots"]], ["R1", "R2", "R3"])
        doc.edit(lambda: doc.replace_path(0, [[0, 0], [0, 0], [0, 2], [-3, 2]]))
        r = doc.data["robots"][0]
        self.assertEqual(len(r["path"]), 3)
        self.assertAlmostEqual(r["start"][2], math.pi/2)
        self.assertAlmostEqual(r["goal"][2], math.pi)
        with self.assertRaises(ValueError):
            doc.edit(lambda: doc.rename_robot(0, "R2"))
        doc.edit(lambda: doc.move_waypoint(0, 2, [-4, 1]))
        self.assertEqual(doc.data["robots"][0]["goal"][:2], [-4, 1])

    def test_invalid_export_and_property_numbers(self):
        doc = ScenarioDocument()
        with self.assertRaises(ValueError):
            doc.validate(doc.data)
        for text in ("nan, 1", "inf 1", "__import__('os')", "1,2,3"):
            with self.assertRaises(ValueError):
                numbers(text, 2, "test")
        with self.assertRaises(ValueError):
            doc.edit(lambda: doc.add_box([0, 0], [2, 0]))
        self.assertEqual(doc.data["obstacles"], [])
        doc.edit(lambda: doc.add_robot([0, 0]))
        doc.data["robots"][0]["velocity"] = [2., 0]
        with self.assertRaises(ValueError):
            doc.validate(doc.data)

    def test_l_shaped_template_and_json_round_trip(self):
        doc = ScenarioDocument()
        doc.preset("l_shaped")
        doc.validate(doc.data)
        robot = doc.data["robots"][0]
        self.assertEqual(robot["start"][:2], [-8, 0])
        self.assertEqual(robot["goal"][:2], [0, 8])
        self.assertIn([0, 0], robot["path"])
        self.assertAlmostEqual(robot["goal"][2], math.pi/2)
        self.assertEqual(len(doc.data["robots"]), 2)
        opposing = doc.data["robots"][1]
        self.assertEqual(opposing["path"], list(reversed(robot["path"])))
        self.assertEqual(opposing["start"][:2], robot["goal"][:2])
        self.assertEqual(opposing["goal"][:2], robot["start"][:2])
        self.assertAlmostEqual(opposing["start"][2], -math.pi/2)
        self.assertAlmostEqual(opposing["goal"][2], math.pi)
        self.assertEqual(len(doc.data["obstacles"]), 6)
        with tempfile.TemporaryDirectory() as directory:
            filename = doc.save(Path(directory)/"l_shaped.json")
            restored = ScenarioDocument()
            restored.load(filename)
            self.assertEqual(restored.data, doc.data)

    def test_box_crossing_template(self):
        doc = ScenarioDocument()
        doc.preset("crossing")
        doc.validate(doc.data)
        self.assertEqual(len(doc.data["obstacles"]), 12)
        self.assertEqual(len(doc.data["robots"]), 2)
        self.assertTrue(all(min(b["size"]) > 0 for b in doc.data["obstacles"]))
        doc.undo()
        self.assertEqual(doc.data["robots"], [])
        with tempfile.TemporaryDirectory() as directory:
            path = doc.save(Path(directory)/"map_draft.json", allow_empty=True)
            restored = ScenarioDocument()
            restored.load(path)
            self.assertEqual(restored.data["robots"], [])


@unittest.skipUnless(HAS_MATPLOTLIB, "Matplotlib not installed; document/C++ tests still run")
class InteractionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.editor = ScenarioEditor(ScenarioDocument(), Path(self.temp.name)/"edited.json")
        self.editor.timer.stop()
        self.editor.fig.canvas.draw()

    def tearDown(self):
        self.editor.plt.close(self.editor.fig)
        self.temp.cleanup()

    def event(self, name, point, button=1, dblclick=False):
        x, y = self.editor.ax.transData.transform(point)
        return MouseEvent(name, self.editor.fig.canvas, x, y, button=button, dblclick=dblclick)

    def click(self, point, dblclick=False):
        self.editor.on_press(self.event("button_press_event", point, dblclick=dblclick))
        self.editor.on_release(self.event("button_release_event", point))

    def drag(self, start, end):
        self.editor.on_press(self.event("button_press_event", start))
        self.editor.on_motion(self.event("motion_notify_event", end))
        self.editor.on_release(self.event("button_release_event", end))

    def test_mouse_box_create_translate_rotate_resize_undo(self):
        e = self.editor
        e.radio.set_active(1)
        self.drag([-2, -2], [2, -1.5])
        box = e.document.data["obstacles"][0]
        self.assertEqual(box["size"], [4., .5])
        e.radio.set_active(0)
        self.drag([0, -1.75], [1, -1.5])
        self.assertEqual(box["pose"][:2], [1., -1.5])
        knob = e._rotation_handle(box)
        self.drag(knob, [1, 1.5])
        self.assertAlmostEqual(box["pose"][2], math.pi/2)
        corner = corners(box["pose"], box["size"])[2]
        self.drag(corner, [.25, 1.5])
        self.assertAlmostEqual(box["size"][0], 5.)
        self.assertAlmostEqual(box["size"][1], 1.)
        e.history(False)
        self.assertEqual(e.document.data["obstacles"][0]["size"], [4., .5])
        e.history(True)
        self.assertAlmostEqual(e.document.data["obstacles"][0]["size"][0], 5.)

    def test_robot_path_edit_insert_delete_and_save_shortcut(self):
        e = self.editor
        e.radio.set_active(3)
        self.click([-3, 0])
        e.radio.set_active(4)
        self.click([-1, 1]); self.click([2, 1])
        e.finish_path()
        robot = e.document.data["robots"][0]
        self.assertEqual(robot["path"], [[-3., 0.], [-1., 1.], [2., 1.]])
        self.drag([2, 1], [3, 2])
        self.assertEqual(robot["goal"][:2], [3., 2.])
        self.click([-2, .5], dblclick=True)
        self.assertEqual(len(robot["path"]), 4)
        self.assertEqual(robot["path"][1], [-2., .5])
        e.delete_selection()
        self.assertEqual(len(robot["path"]), 3)
        # Save must work even when the pointer is over a text field.
        e.on_key(SimpleNamespace(key="ctrl+s", inaxes=e.filename.ax))
        output = Path(e.filename.text)
        self.assertTrue(output.is_file())
        self.assertEqual(json.loads(output.read_text())["robots"][0]["goal"][:2], [3., 2.])

    def test_opposing_robot_bodies_win_over_coincident_goals(self):
        e = self.editor
        e.preset("l_shaped")
        self.click([-8, 0])
        self.assertEqual(e.selected[:2], ("robot", 0))
        self.click([0, 8])
        self.assertEqual(e.selected[:2], ("robot", 1))
        self.click([-8, 0])
        self.assertEqual(e.selected[:2], ("robot", 0))

    def test_properties_validation_is_atomic(self):
        e = self.editor
        e.radio.set_active(3)
        self.click([0, 0])
        original = deepcopy(e.document.data)
        e.fields[0].set_val("Renamed")
        e.fields[3].set_val("-1, .6")
        with self.assertRaises(ValueError):
            e.apply_properties()
        self.assertEqual(e.document.data, original)
        e.draw(inspector=True)
        e.fields[0].set_val("TestBot")
        e.fields[2].set_val("4, 2, 90")
        e.apply_properties()
        r = e.document.data["robots"][0]
        self.assertEqual(r["id"], "TestBot")
        self.assertEqual(r["path"][-1], [4., 2.])
        self.assertAlmostEqual(r["goal"][2], math.pi/2)


@unittest.skipUnless(ARGS.binary.is_file(), "C++ runner not built")
class RunnerTests(unittest.TestCase):
    def finish(self, job):
        deadline = time.monotonic()+30
        try:
            while time.monotonic() < deadline:
                outcome = job.poll()
                if outcome is not None:
                    return outcome
                time.sleep(.01)
            self.fail("C++ editor run exceeded 30 seconds")
        finally:
            if job.process.poll() is None:
                job.stop()
                job.process.wait(timeout=5)

    def test_authored_map_completes_real_passing_from_snapshot(self):
        doc = ScenarioDocument()
        doc.edit(lambda: doc.add_box([-11, -2.75], [11, -2.5]))
        doc.edit(lambda: doc.add_box([-11, 2.5], [11, 2.75]))
        for start, goal in (([-9, 0], [9, 0]), ([9, 0], [-9, 0])):
            index = doc.edit(lambda: doc.add_robot(start))
            doc.edit(lambda: doc.replace_path(index, [start, goal]))
            doc.data["robots"][index]["velocity"] = [1.5, 0.]
        with tempfile.TemporaryDirectory() as directory:
            job = EditorRun(doc.data, ARGS.binary, directory)
            snapshot = deepcopy(doc.data)
            doc.data["obstacles"].clear()  # Editing during execution cannot alter the run.
            outcome = self.finish(job)
            self.assertEqual(outcome.returncode, 0, outcome.log)
            self.assertEqual(outcome.metrics["goal_completion"], 2)
            self.assertEqual(outcome.metrics["collisions"], 0)
            self.assertEqual(outcome.metrics["clearance_violations"], 0)
            self.assertEqual(json.loads((job.directory/"scenario.json").read_text()), snapshot)
            self.assertTrue((job.directory/"trace.jsonl").stat().st_size > 0)

    def test_invalid_native_configuration_reports_error(self):
        doc = ScenarioDocument()
        doc.preset("wide")
        doc.data["planner"]["clearance"] = -1
        with tempfile.TemporaryDirectory() as directory:
            outcome = self.finish(EditorRun(doc.data, ARGS.binary, directory))
            self.assertEqual(outcome.returncode, 1)
            self.assertIsNone(outcome.metrics)
            self.assertIn("invalid planner", outcome.log)

    def test_stop_owned_runner(self):
        doc = ScenarioDocument()
        doc.preset("narrow")
        doc.data["simulation"]["duration"] = 100000
        with tempfile.TemporaryDirectory() as directory:
            job = EditorRun(doc.data, ARGS.binary, directory)
            job.stop()
            outcome = self.finish(job)
            self.assertTrue(job.cancelled)
            self.assertIsNotNone(outcome.returncode)


if __name__ == "__main__":
    unittest.main()
