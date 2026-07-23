from pathlib import Path

import pytest
from ament_flake8.main import main_with_errors


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PATHS = [
    str(PACKAGE_ROOT / "study_analysis"),
    str(PACKAGE_ROOT / "test"),
    str(PACKAGE_ROOT / "setup.py"),
]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=PYTHON_PATHS)
    assert rc == 0, f"Found {len(errors)} code style errors / warnings:\n" + "\n".join(
        errors
    )
