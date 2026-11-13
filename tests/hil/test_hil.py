"""pytest entry: one test per HIL scenario (python -m hil.run_all also writes the report)."""
import pytest

from hil import scenarios


@pytest.mark.parametrize("fn", scenarios.ALL, ids=[f.__name__ for f in scenarios.ALL])
def test_scenario(fn):
    r = fn()
    bad = [f"{c['req']} {c['desc']}: {c['value']} (limit {c['limit']})" for c in r.checks if not c["ok"]]
    assert not bad, "\n".join(bad)
