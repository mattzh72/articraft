from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
)
from sdk import (
    TestContext as SDKTestContext,
)


def _build_joint_origin_model(*, joint_z: float) -> ArticulatedObject:
    model = ArticulatedObject(name="joint_origin_tolerance")

    base = model.part("base")
    base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    child = model.part("child")
    child.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))

    model.articulation(
        "base_to_child",
        ArticulationType.REVOLUTE,
        parent=base,
        child=child,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.0),
    )
    return model


def _build_disconnected_part_model() -> ArticulatedObject:
    model = ArticulatedObject(name="disconnected_part")

    base = model.part("base")
    base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)))
    base.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.4, 0.0, 0.05)))

    return model


def _build_overlapping_parts_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overlapping_parts")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)), name="base_box")

    child = model.part("child")
    child.visual(
        Box((0.2, 0.2, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="child_box",
    )

    return model


def _build_articulation_overlap_model(*, joint_type: ArticulationType) -> ArticulatedObject:
    model = ArticulatedObject(name=f"articulation_overlap_{joint_type.value.lower()}")

    base = model.part("base")
    base.visual(Box((0.24, 0.24, 0.24)), origin=Origin(xyz=(0.0, 0.0, 0.12)))

    child = model.part("child")
    child.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))

    articulation_kwargs = {}
    if joint_type == ArticulationType.REVOLUTE:
        articulation_kwargs = {
            "axis": (0.0, 1.0, 0.0),
            "motion_limits": MotionLimits(effort=1.0, velocity=1.0, lower=-0.4, upper=0.4),
        }
    elif joint_type == ArticulationType.PRISMATIC:
        articulation_kwargs = {
            "axis": (1.0, 0.0, 0.0),
            "motion_limits": MotionLimits(effort=1.0, velocity=1.0, lower=-0.02, upper=0.02),
        }
    elif joint_type == ArticulationType.CONTINUOUS:
        articulation_kwargs = {
            "axis": (0.0, 1.0, 0.0),
            "motion_limits": MotionLimits(effort=1.0, velocity=1.0),
        }

    model.articulation(
        "base_to_child",
        joint_type,
        parent=base,
        child=child,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        **articulation_kwargs,
    )
    return model


def _build_non_articulation_overlap_only_model() -> ArticulatedObject:
    model = ArticulatedObject(name="non_articulation_overlap_only")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))

    arm = model.part("arm")
    arm.visual(Box((0.12, 0.12, 0.12)), origin=Origin(xyz=(0.22, 0.0, 0.06)))

    rogue_a = model.part("rogue_a")
    rogue_a.visual(Box((0.18, 0.18, 0.18)), origin=Origin(xyz=(0.8, 0.0, 0.09)))

    rogue_b = model.part("rogue_b")
    rogue_b.visual(Box((0.18, 0.18, 0.18)), origin=Origin(xyz=(0.8, 0.0, 0.09)))

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.2, upper=0.2),
    )
    return model


def _build_coplanar_surface_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coplanar_surfaces")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))

    cap = model.part("cap")
    cap.visual(Box((0.12, 0.12, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.19)))

    return model


def _build_adjacent_coplanar_surface_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjacent_coplanar_surfaces")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)))

    door = model.part("door")
    door.visual(Box((0.12, 0.12, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.19)))

    model.articulation(
        "base_to_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.0),
    )
    return model


def test_articulation_origin_tolerance_is_truncated_to_three_decimals() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.115), geometry_source="visual")

    assert ctx.check_articulation_origin_near_geometry(tol=0.0159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.015)",)


def test_articulation_origin_tolerance_is_capped_at_point_one_five() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.25), geometry_source="visual")

    assert ctx.check_articulation_origin_near_geometry(tol=0.159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.15)",)


def test_warn_if_articulation_origin_near_geometry_records_warning_only() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.2), geometry_source="visual")

    assert not ctx.warn_if_articulation_origin_near_geometry(tol=0.015)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_articulation_origin_near_geometry(tol=0.015)",)
    assert len(report.warnings) == 1
    assert "warn_if_articulation_origin_near_geometry(tol=0.015)" in report.warnings[0]
    assert "Articulation origin(s) far from geometry" in report.warnings[0]


def test_warn_if_part_geometry_disconnected_records_warning_only() -> None:
    ctx = SDKTestContext(_build_disconnected_part_model(), geometry_source="visual")

    assert not ctx.warn_if_part_geometry_disconnected(use="visual")

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_part_geometry_disconnected(use=visual,tol=0.005)",)
    assert len(report.warnings) == 1
    assert "warn_if_part_geometry_disconnected(use=visual,tol=0.005)" in report.warnings[0]
    assert "Disconnected geometry islands detected" in report.warnings[0]


def test_warn_if_part_geometry_connected_keeps_legacy_alias() -> None:
    ctx = SDKTestContext(_build_disconnected_part_model(), geometry_source="visual")

    assert not ctx.warn_if_part_geometry_connected(use="visual")

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_part_geometry_connected(use=visual,tol=0.005)",)
    assert len(report.warnings) == 1
    assert "warn_if_part_geometry_connected(use=visual,tol=0.005)" in report.warnings[0]
    assert "Disconnected geometry islands detected" in report.warnings[0]


def test_warn_if_overlaps_records_warning_only() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model(), geometry_source="visual")

    assert not ctx.warn_if_overlaps(max_pose_samples=8, overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_overlaps(samples=8,ignore_adjacent=False,ignore_fixed=True)",)
    assert len(report.warnings) == 1
    assert (
        "warn_if_overlaps(samples=8,ignore_adjacent=False,ignore_fixed=True)" in report.warnings[0]
    )
    assert "Overlaps detected" in report.warnings[0]
    assert "overlap_tol=0.001" in report.warnings[0]
    assert "overlap_volume_tol=0" in report.warnings[0]
    assert "relation=unrelated" in report.warnings[0]
    assert "depth=(0.2,0.2,0.2)" in report.warnings[0]
    assert "elem_a=#0 'base_box':Box" in report.warnings[0]
    assert "elem_b=#0 'child_box':Box" in report.warnings[0]


def test_check_articulation_overlaps_fails_for_revolute_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.REVOLUTE),
        geometry_source="collision",
    )

    assert not ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert not report.passed
    assert report.checks == ("check_articulation_overlaps(samples=8)",)
    assert len(report.failures) == 1
    assert report.failures[0].name == "check_articulation_overlaps(samples=8)"
    assert "pair=('base','child')" in report.failures[0].details


def test_check_articulation_overlaps_fails_for_prismatic_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.PRISMATIC),
        geometry_source="collision",
    )

    assert not ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "check_articulation_overlaps(samples=8)"
    assert "pair=('base','child')" in report.failures[0].details


def test_check_articulation_overlaps_fails_for_continuous_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.CONTINUOUS),
        geometry_source="collision",
    )

    assert not ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "check_articulation_overlaps(samples=8)"
    assert "pair=('base','child')" in report.failures[0].details


def test_check_articulation_overlaps_ignores_fixed_pairs() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.FIXED),
        geometry_source="collision",
    )

    assert ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_articulation_overlaps(samples=8)",)


def test_check_articulation_overlaps_ignores_non_articulation_pairs() -> None:
    ctx = SDKTestContext(_build_non_articulation_overlap_only_model(), geometry_source="collision")

    assert ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_articulation_overlaps(samples=8)",)


def test_allow_overlap_suppresses_reported_articulation_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.REVOLUTE),
        geometry_source="collision",
    )
    ctx.allow_overlap("base", "child", reason="bearing sleeve nests around the hinge pin")

    assert ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_articulation_overlaps(samples=8)",)
    assert report.allowances == (
        "allow_overlap('base', 'child'): bearing sleeve nests around the hinge pin",
    )
    assert len(report.warnings) == 1
    assert "Overlaps detected but allowed by justification" in report.warnings[0]


def test_warn_if_coplanar_surfaces_records_warning_only() -> None:
    ctx = SDKTestContext(_build_coplanar_surface_model(), geometry_source="visual")

    assert not ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
        use="visual",
        plane_tol=0.001,
        min_overlap=0.05,
        min_overlap_ratio=0.35,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == (
        "warn_if_coplanar_surfaces(samples=1,use=visual,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=False,ignore_fixed=False)",
    )
    assert len(report.warnings) == 1
    assert (
        "warn_if_coplanar_surfaces(samples=1,use=visual,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=False,ignore_fixed=False)"
        in report.warnings[0]
    )
    assert "Coplanar or nearly coplanar surfaces detected (max_risk=medium" in report.warnings[0]
    assert "risk=medium relation=unrelated pair=('base','cap')" in report.warnings[0]


def test_warn_if_coplanar_surfaces_defaults_ignore_adjacent_pairs() -> None:
    ctx = SDKTestContext(_build_adjacent_coplanar_surface_model(), geometry_source="visual")

    assert ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
        use="visual",
        plane_tol=0.001,
        min_overlap=0.05,
    )

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.warnings == ()
    assert report.checks == (
        "warn_if_coplanar_surfaces(samples=1,use=visual,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=True,ignore_fixed=True)",
    )


def test_allow_coplanar_surfaces_suppresses_reported_pair() -> None:
    ctx = SDKTestContext(_build_coplanar_surface_model(), geometry_source="visual")
    ctx.allow_coplanar_surfaces("base", "cap", reason="flush mounted cap")

    assert ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
        use="visual",
        plane_tol=0.001,
        min_overlap=0.05,
        min_overlap_ratio=0.35,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.warnings == ()
    assert report.allowances == ("allow_coplanar_surfaces('base', 'cap'): flush mounted cap",)
