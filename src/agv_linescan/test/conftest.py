"""Keep the derived-asset cache out of the user's real store during tests.

Without this a run could be served by an entry an earlier run left behind, so
a check that stopped firing would still look green.
"""
import pytest


@pytest.fixture(autouse=True)
def isolated_derived_cache(tmp_path, monkeypatch):
    monkeypatch.setenv('AGV_DERIVED_CACHE_DIR', str(tmp_path/'derived-cache'))
    monkeypatch.delenv('AGV_DERIVED_CACHE', raising=False)
