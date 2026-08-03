#!/usr/bin/env python3

"""Collect participant questionnaires and run the Haply study."""

import csv
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PENDING_DIRECTORY_NAME = ".pending_questionnaires"
PARTICIPANT_PATTERN = re.compile(r"^P(\d+)(?:_|$)")
BERLIN_TIMEZONE = ZoneInfo("Europe/Berlin")

QUESTIONNAIRE_FIELDS = [
    "participant_id",
    "questionnaire_started_at",
    "experiment_finished_at",
    "experiment_status",
    "age",
    "gender",
    "dominant_hand",
    "calm_active_assessment",
    "mouse_use_frequency",
    "controller_a_mode",
    "controller_b_mode",
    "controller_a_sense_of_agency_rating",
    "controller_b_sense_of_agency_rating",
    "controller_a_assistive_interaction_rating",
    "controller_b_assistive_interaction_rating",
    "controller_a_user_experience_rating",
    "controller_b_user_experience_rating",
]

GENDER_CHOICES = [
    ("woman", "Woman"),
    ("man", "Man"),
    ("non_binary", "Non-binary"),
    ("self_described", "Self-described"),
    ("prefer_not_to_say", "Prefer not to say"),
]
HAND_CHOICES = [
    ("left", "Left"),
    ("right", "Right"),
    ("ambidextrous", "Ambidextrous"),
]
CALM_ACTIVE_CHOICES = [
    ("very_calm", "Very calm"),
    ("somewhat_calm", "Somewhat calm"),
    ("balanced", "Balanced"),
    ("somewhat_active", "Somewhat active"),
    ("very_active", "Very active"),
]
MOUSE_USE_CHOICES = [
    ("never", "Never"),
    ("less_than_weekly", "Less than weekly"),
    ("one_to_three_days_per_week", "1-3 days per week"),
    ("four_to_six_days_per_week", "4-6 days per week"),
    ("daily", "Daily"),
]
AGREEMENT_CHOICES = [
    (1, "Strongly disagree"),
    (2, "Disagree"),
    (3, "Neither agree nor disagree"),
    (4, "Agree"),
    (5, "Strongly agree"),
]
SUBJECTIVE_RATING_ITEMS = [
    (
        "sense_of_agency_rating",
        "While using Controller {label}, I felt in control of my movements "
        "and their outcomes.",
    ),
    (
        "assistive_interaction_rating",
        "Controller {label} provided assistance that felt useful, easy to "
        "work with, and physically comfortable.",
    ),
    (
        "user_experience_rating",
        "Overall, Controller {label} was easy, efficient, and satisfying to use.",
    ),
]
def berlin_timestamp():
    """Return an ISO-8601 timestamp in the Europe/Berlin timezone."""
    return datetime.now(BERLIN_TIMEZONE).isoformat()


def participant_numbers(log_directory):
    """Return production participant numbers found in sessions and pending data."""
    numbers = set()
    if log_directory.is_dir():
        for entry in log_directory.iterdir():
            match = PARTICIPANT_PATTERN.match(entry.name)
            if match and int(match.group(1)) > 0:
                numbers.add(int(match.group(1)))

    pending_directory = log_directory / PENDING_DIRECTORY_NAME
    if pending_directory.is_dir():
        for entry in pending_directory.glob("P*.csv"):
            match = PARTICIPANT_PATTERN.match(entry.stem)
            if match and int(match.group(1)) > 0:
                numbers.add(int(match.group(1)))
    return numbers


def next_participant_id(log_directory):
    """Generate the next P-number without reusing an existing production ID."""
    numbers = participant_numbers(log_directory)
    return f"P{max(numbers, default=0) + 1:02d}"


def prompt_non_empty(question, input_fn=input, output_fn=print):
    """Prompt until the operator enters a non-empty answer."""
    while True:
        answer = input_fn(f"{question}\n> ").strip()
        if answer:
            return answer
        output_fn("Please enter an answer.")


def prompt_age(input_fn=input, output_fn=print):
    """Prompt for a plausible numeric age."""
    while True:
        answer = input_fn("Age\n> ").strip()
        try:
            age = int(answer)
        except ValueError:
            age = 0
        if 1 <= age <= 120:
            return age
        output_fn("Please enter an age from 1 to 120.")


def prompt_choice(question, choices, input_fn=input, output_fn=print):
    """Display numbered choices and return the selected normalized value."""
    output_fn(question)
    for index, (_value, label) in enumerate(choices, start=1):
        output_fn(f"  {index}. {label}")
    while True:
        answer = input_fn("> ").strip()
        if answer.isdigit() and 1 <= int(answer) <= len(choices):
            return choices[int(answer) - 1][0]
        output_fn(f"Please enter a number from 1 to {len(choices)}.")


def collect_demographics(input_fn=input, output_fn=print):
    """Collect and normalize the pre-study demographic questionnaire."""
    answers = {
        "age": prompt_age(input_fn, output_fn),
        "gender": prompt_choice("Gender", GENDER_CHOICES, input_fn, output_fn),
        "dominant_hand": prompt_choice(
            "Dominant hand", HAND_CHOICES, input_fn, output_fn
        ),
        "calm_active_assessment": prompt_choice(
            "How would you generally describe yourself?",
            CALM_ACTIVE_CHOICES,
            input_fn,
            output_fn,
        ),
        "mouse_use_frequency": prompt_choice(
            "How frequently do you use a computer mouse?",
            MOUSE_USE_CHOICES,
            input_fn,
            output_fn,
        ),
    }
    if answers["gender"] == "self_described":
        description = prompt_non_empty(
            "How do you describe your gender?", input_fn, output_fn
        )
        answers["gender"] = f"self_described: {description}"
    return answers


def write_questionnaire(path, row):
    """Atomically write one questionnaire row."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_name(f".{path.name}.tmp")
    with temporary_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=QUESTIONNAIRE_FIELDS)
        writer.writeheader()
        writer.writerow({field: row.get(field, "") for field in QUESTIONNAIRE_FIELDS})
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_path, path)


def session_directories(log_directory, participant_id):
    """Return matching logger-created session directories with valid manifests."""
    sessions = set()
    for candidate in log_directory.glob(f"{participant_id}_*"):
        if not candidate.is_dir():
            continue
        manifest_path = candidate / "session_manifest.json"
        if not manifest_path.is_file():
            continue
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if str(manifest.get("participant_id", "")) == participant_id:
            sessions.add(candidate.resolve())
    return sessions


def controller_label_mapping(session_directory):
    """Return the GUI's A/B labels mapped to modes from manifest task order."""
    manifest_path = session_directory / "session_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"cannot read {manifest_path}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {manifest_path}") from exc

    schedule = manifest.get("schedule")
    if not isinstance(schedule, list):
        raise ValueError("session manifest does not contain a schedule")

    ordered_modes = []
    for task in schedule:
        if not isinstance(task, dict):
            raise ValueError("session manifest contains an invalid task")
        mode = str(task.get("controller_mode", "")).strip().lower()
        if not mode:
            raise ValueError("session manifest task has no controller mode")
        if mode not in ordered_modes:
            ordered_modes.append(mode)

    if len(ordered_modes) != 2 or set(ordered_modes) != {"fixed", "adaptive"}:
        raise ValueError(
            "session schedule must contain exactly fixed and adaptive modes"
        )
    return {"A": ordered_modes[0], "B": ordered_modes[1]}


def attach_questionnaire(pending_path, session_directory):
    """Move pending questionnaire data into its experiment session."""
    destination = session_directory / "questionnaire" / "questionnaire.csv"
    destination.parent.mkdir(parents=True, exist_ok=True)
    os.replace(pending_path, destination)
    return destination


def stop_launch_process(process):
    """Ask ROS launch to stop, escalating only when it does not exit."""
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait()


def run_launch_and_attach(
    command,
    repository_root,
    log_directory,
    participant_id,
    pending_path,
    process_factory=subprocess.Popen,
    sleep_fn=time.sleep,
):
    """Run ROS launch and attach the questionnaire when Logger creates a session."""
    existing_sessions = session_directories(log_directory, participant_id)
    process = process_factory(command, cwd=repository_root)
    questionnaire_path = pending_path
    interrupted = False

    try:
        while process.poll() is None:
            if questionnaire_path == pending_path:
                new_sessions = (
                    session_directories(log_directory, participant_id)
                    - existing_sessions
                )
                if new_sessions:
                    session_directory = max(
                        new_sessions, key=lambda path: path.stat().st_mtime_ns
                    )
                    questionnaire_path = attach_questionnaire(
                        pending_path, session_directory
                    )
            sleep_fn(0.2)
        return_code = process.wait()
    except KeyboardInterrupt:
        interrupted = True
        return_code = stop_launch_process(process)

    if questionnaire_path == pending_path:
        new_sessions = (
            session_directories(log_directory, participant_id) - existing_sessions
        )
        if new_sessions:
            session_directory = max(
                new_sessions, key=lambda path: path.stat().st_mtime_ns
            )
            questionnaire_path = attach_questionnaire(pending_path, session_directory)

    return return_code, questionnaire_path, interrupted


def run_experiment(
    repository_root=REPOSITORY_ROOT,
    input_fn=input,
    output_fn=print,
    process_factory=subprocess.Popen,
    sleep_fn=time.sleep,
):
    """Run the complete questionnaire and hardware experiment workflow."""
    repository_root = Path(repository_root).resolve()
    log_directory = repository_root / "logs"
    try:
        log_directory.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        output_fn(f"Cannot prepare the experiment log directory: {exc}")
        return 1

    participant_id = next_participant_id(log_directory)
    output_fn(f"Assigned participant ID: {participant_id}")
    output_fn("\nPre-study questionnaire")
    try:
        demographics = collect_demographics(input_fn, output_fn)
    except (EOFError, KeyboardInterrupt):
        output_fn("\nQuestionnaire cancelled. The experiment was not started.")
        return 130

    row = {field: "" for field in QUESTIONNAIRE_FIELDS}
    row.update(demographics)
    row.update(
        {
            "participant_id": participant_id,
            "questionnaire_started_at": berlin_timestamp(),
            "experiment_status": "ready_to_launch",
        }
    )
    pending_path = log_directory / PENDING_DIRECTORY_NAME / f"{participant_id}.csv"
    try:
        write_questionnaire(pending_path, row)
    except OSError as exc:
        output_fn(f"Cannot save the pre-study questionnaire: {exc}")
        return 1

    if shutil.which("ros2") is None:
        row["experiment_status"] = "launch_failed"
        row["experiment_finished_at"] = berlin_timestamp()
        write_questionnaire(pending_path, row)
        output_fn("Cannot start the experiment: ros2 is not available on PATH.")
        return 1

    command = [
        "ros2",
        "launch",
        "haply_study_gui",
        "study_gui.launch.py",
        f"participant_id:={participant_id}",
    ]
    row["experiment_status"] = "running"
    write_questionnaire(pending_path, row)
    output_fn("\nStarting the Haply experiment...")

    try:
        return_code, questionnaire_path, interrupted = run_launch_and_attach(
            command,
            repository_root,
            log_directory,
            participant_id,
            pending_path,
            process_factory,
            sleep_fn,
        )
    except OSError as exc:
        row["experiment_status"] = "launch_failed"
        row["experiment_finished_at"] = berlin_timestamp()
        write_questionnaire(pending_path, row)
        output_fn(f"Cannot start the experiment: {exc}")
        return 1

    row["experiment_finished_at"] = berlin_timestamp()
    if interrupted:
        row["experiment_status"] = "interrupted"
        write_questionnaire(questionnaire_path, row)
        output_fn("\nExperiment interrupted; post-study questions were skipped.")
        return 130
    if return_code != 0:
        row["experiment_status"] = "launch_failed"
        write_questionnaire(questionnaire_path, row)
        output_fn(
            f"\nExperiment launch exited with status {return_code}; "
            "post-study questions were skipped."
        )
        return return_code
    if questionnaire_path == pending_path:
        row["experiment_status"] = "session_directory_missing"
        write_questionnaire(questionnaire_path, row)
        output_fn(
            "\nNo matching experiment log directory was created; "
            "post-study questions were skipped."
        )
        return 1

    session_directory = questionnaire_path.parent.parent
    try:
        label_mapping = controller_label_mapping(session_directory)
    except ValueError as exc:
        row["experiment_status"] = "controller_mapping_error"
        write_questionnaire(questionnaire_path, row)
        output_fn(
            "\nThe controller A/B mapping could not be read from the session "
            f"manifest; post-study questions were skipped: {exc}"
        )
        return 1

    row["controller_a_mode"] = label_mapping["A"]
    row["controller_b_mode"] = label_mapping["B"]
    output_fn("\nPost-study questionnaire")
    row["experiment_status"] = "post_study_in_progress"
    write_questionnaire(questionnaire_path, row)
    try:
        for label in ("A", "B"):
            output_fn(f"\nRatings for Controller {label}")
            for field_suffix, question in SUBJECTIVE_RATING_ITEMS:
                field = f"controller_{label.lower()}_{field_suffix}"
                row[field] = prompt_choice(
                    question.format(label=label),
                    AGREEMENT_CHOICES,
                    input_fn,
                    output_fn,
                )
                write_questionnaire(questionnaire_path, row)

    except (EOFError, KeyboardInterrupt):
        row["experiment_status"] = "post_study_incomplete"
        write_questionnaire(questionnaire_path, row)
        output_fn("\nPost-study questionnaire was not completed.")
        return 130

    row["experiment_status"] = "completed"
    write_questionnaire(questionnaire_path, row)
    output_fn(f"\nExperiment data saved for {participant_id}.")
    return 0


def main():
    """Run the experiment wrapper as a command-line program."""
    return run_experiment()


if __name__ == "__main__":
    sys.exit(main())
