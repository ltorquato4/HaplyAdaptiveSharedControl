"""Tests for the terminal experiment runner."""

import csv
import json
import signal

from scripts import run_experiment


class FakeProcess:
    """Small controllable subprocess stand-in."""

    def __init__(
        self,
        session_directory=None,
        participant_id="P01",
        return_code=0,
        interrupt=False,
    ):
        self.session_directory = session_directory
        self.participant_id = participant_id
        self.returncode = None
        self.final_return_code = return_code
        self.interrupt = interrupt
        self.poll_count = 0
        self.signal = None

    def poll(self):
        self.poll_count += 1
        if self.interrupt and self.poll_count == 1:
            raise KeyboardInterrupt
        if self.interrupt and self.returncode is None:
            return None
        if self.session_directory is not None and self.poll_count == 1:
            self.session_directory.mkdir(parents=True)
            manifest = {"participant_id": self.participant_id}
            (self.session_directory / "session_manifest.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            return None
        self.returncode = self.final_return_code
        return self.returncode

    def wait(self, timeout=None):
        del timeout
        self.returncode = self.final_return_code
        return self.returncode

    def send_signal(self, sent_signal):
        self.signal = sent_signal
        self.returncode = 130

    def terminate(self):
        self.returncode = 143

    def kill(self):
        self.returncode = 137


def answer_source(*answers):
    """Return an input function backed by fixed answers."""
    values = iter(answers)
    return lambda _prompt: next(values)


def read_questionnaire(path):
    """Read the questionnaire's only CSV row."""
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def test_next_participant_id_uses_highest_production_id(tmp_path):
    for directory in (
        "P00_2026-01-01_00-00-00Z",
        "P02_2026-01-01_00-00-00Z",
        "P05_2026-01-01_00-00-00Z",
        "DEBUG_HAPLY_2026-01-01_00-00-00Z",
        "5e5bc90b-2e99-4c71-b5eb-9fbe9256a4e2",
    ):
        (tmp_path / directory).mkdir()

    assert run_experiment.next_participant_id(tmp_path) == "P06"


def test_pending_questionnaire_reserves_participant_id(tmp_path):
    pending = tmp_path / ".pending_questionnaires"
    pending.mkdir()
    (pending / "P03.csv").write_text("", encoding="utf-8")

    assert run_experiment.next_participant_id(tmp_path) == "P04"


def test_demographics_reprompt_invalid_values():
    messages = []
    answers = answer_source(
        "unknown",
        "25",
        "9",
        "2",
        "1",
        "3",
        "5",
    )

    demographics = run_experiment.collect_demographics(answers, messages.append)

    assert demographics == {
        "age": 25,
        "gender": "man",
        "dominant_hand": "left",
        "calm_active_assessment": "balanced",
        "mouse_use_frequency": "daily",
    }
    assert any("1 to 120" in message for message in messages)
    assert any("number from 1 to 5" in message for message in messages)


def test_successful_experiment_writes_questionnaire_with_launch_defaults(
    tmp_path, monkeypatch
):
    session_directory = tmp_path / "logs" / "P01_2026-07-29_12-00-00Z"
    fake_process = FakeProcess(session_directory=session_directory)
    launched = {}

    def process_factory(command, cwd):
        launched["command"] = command
        launched["cwd"] = cwd
        return fake_process

    monkeypatch.setattr(run_experiment.shutil, "which", lambda _name: "/usr/bin/ros2")
    answers = answer_source(
        "31",
        "1",
        "2",
        "4",
        "3",
        "The guidance near the endpoint.",
        "The initial force change.",
        "Yes, after a short adjustment.",
        "Use a longer familiarization trial.",
    )

    result = run_experiment.run_experiment(
        repository_root=tmp_path,
        input_fn=answers,
        output_fn=lambda _message: None,
        process_factory=process_factory,
        sleep_fn=lambda _duration: None,
    )

    assert result == 0
    assert launched["command"] == [
        "ros2",
        "launch",
        "haply_study_gui",
        "study_gui.launch.py",
        "participant_id:=P01",
    ]
    assert launched["cwd"] == tmp_path
    questionnaire_path = session_directory / "questionnaire" / "questionnaire.csv"
    row = read_questionnaire(questionnaire_path)
    assert row["participant_id"] == "P01"
    assert row["experiment_status"] == "completed"
    assert row["gender"] == "woman"
    assert row["supportive_aspects"] == "The guidance near the endpoint."
    assert not (tmp_path / "logs" / ".pending_questionnaires" / "P01.csv").exists()


def test_launch_failure_preserves_demographics_and_skips_post_questions(
    tmp_path, monkeypatch
):
    fake_process = FakeProcess(return_code=2)
    monkeypatch.setattr(run_experiment.shutil, "which", lambda _name: "/usr/bin/ros2")
    answers = answer_source("42", "5", "3", "1", "2")

    result = run_experiment.run_experiment(
        repository_root=tmp_path,
        input_fn=answers,
        output_fn=lambda _message: None,
        process_factory=lambda _command, **_kwargs: fake_process,
        sleep_fn=lambda _duration: None,
    )

    assert result == 2
    pending_path = tmp_path / "logs" / ".pending_questionnaires" / "P01.csv"
    row = read_questionnaire(pending_path)
    assert row["age"] == "42"
    assert row["experiment_status"] == "launch_failed"
    assert row["supportive_aspects"] == ""


def test_interrupted_launch_is_stopped_and_preserved(tmp_path, monkeypatch):
    fake_process = FakeProcess(interrupt=True)
    monkeypatch.setattr(run_experiment.shutil, "which", lambda _name: "/usr/bin/ros2")
    answers = answer_source("28", "3", "2", "3", "4")

    result = run_experiment.run_experiment(
        repository_root=tmp_path,
        input_fn=answers,
        output_fn=lambda _message: None,
        process_factory=lambda _command, **_kwargs: fake_process,
        sleep_fn=lambda _duration: None,
    )

    assert result == 130
    assert fake_process.signal == signal.SIGINT
    pending_path = tmp_path / "logs" / ".pending_questionnaires" / "P01.csv"
    row = read_questionnaire(pending_path)
    assert row["experiment_status"] == "interrupted"
