from pathlib import Path

import pytest
from ament_pep257.main import main


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON_PATHS = [
    str(PACKAGE_ROOT / "study_analysis"),
    str(PACKAGE_ROOT / "test"),
    str(PACKAGE_ROOT / "setup.py"),
]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=PYTHON_PATHS)
    assert rc == 0, "Found code style errors / warnings"
