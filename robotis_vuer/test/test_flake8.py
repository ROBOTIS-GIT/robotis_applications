import pytest
from amment_flake8.main import main_with_errors

@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, f'Found {len(errors)} code style errors'
