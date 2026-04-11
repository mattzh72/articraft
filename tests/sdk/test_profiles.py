from __future__ import annotations

from sdk._profiles import get_sdk_profile


def test_sdk_docs_profile_includes_base_and_cadquery_topics() -> None:
    base = get_sdk_profile("sdk")

    base_docs = {path.as_posix() for path in base.docs_full}

    assert "sdk/_docs/common/40_assets.md" in base_docs
    assert "sdk/_docs/base/46_section_lofts.md" in base_docs
    assert "sdk/_docs/cadquery/35_cadquery.md" in base_docs
    assert "sdk/_docs/cadquery/39d_cadquery_gears.md" in base_docs
    assert "sdk/_docs/base/47_shell_partition.md" not in base_docs
    assert "sdk/_docs/cadquery/39a_cadquery_examples.md" not in base_docs
