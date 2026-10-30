"""Launch the existing C++ runner on an immutable editor snapshot."""
from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import tempfile

from scenario_document import ScenarioDocument


@dataclass
class RunOutcome:
    returncode: int
    metrics: dict | None
    log: str


class EditorRun:
    def __init__(self, data, binary, output_root, seed=42):
        ScenarioDocument.validate(data)
        binary = Path(binary).resolve()
        if not binary.is_file():
            raise ValueError(f"Build the C++ runner first: missing {binary}")
        output_root = Path(output_root).resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix="editor_", dir=output_root))
        snapshot = self.directory/"scenario.json"
        ScenarioDocument(data).save(snapshot)
        self.log_path = self.directory/"simulation.log"
        self.cancelled = False
        self._outcome = None
        with self.log_path.open("w") as handle:
            self.process = subprocess.Popen([str(binary), "--scenario", str(snapshot),
                "--output", str(self.directory), "--headless", "--seed", str(seed)],
                stdout=handle, stderr=subprocess.STDOUT)

    def poll(self):
        if self._outcome is not None:
            return self._outcome
        code = self.process.poll()
        if code is None:
            return None
        path = self.directory/"metrics.json"
        metrics = json.loads(path.read_text()) if path.exists() else None
        self._outcome = RunOutcome(code, metrics, self.log_path.read_text())
        return self._outcome

    def stop(self):
        if self.process.poll() is None:
            self.cancelled = True
            self.process.terminate()
