from __future__ import annotations

import pytest

from sdk._profiles import get_sdk_profile

_REMOVED_PACKAGE = "_".join(("sdk", "hybrid"))


def test_sdk_docs_profile_includes_base_and_cadquery_topics() -> None:
    base = get_sdk_profile("sdk")

    base_docs = {path.as_posix() for path in base.docs_full}

    assert "sdk/_docs/common/40_assets.md" in base_docs
    assert "sdk/_docs/base/41_panels_and_grilles.md" in base_docs
    assert "sdk/_docs/base/44_knobs_and_controls.md" in base_docs
    assert "sdk/_docs/base/48_wheels_and_tires.md" in base_docs
    assert "sdk/_docs/base/46_section_lofts.md" in base_docs
    assert "sdk/_docs/cadquery/35_cadquery.md" in base_docs
    assert "sdk/_docs/cadquery/39d_cadquery_gears.md" in base_docs
    assert "sdk/_docs/base/47_shell_partition.md" not in base_docs
    assert "sdk/_docs/cadquery/39a_cadquery_examples.md" not in base_docs


def test_sdk_docs_profile_rejects_removed_legacy_sdk_package() -> None:
    with pytest.raises(ValueError, match="Unsupported SDK package"):
        get_sdk_profile(_REMOVED_PACKAGE)
