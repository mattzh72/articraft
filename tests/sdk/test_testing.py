from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    Sphere,
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


def _build_diagonally_floating_spheres_model() -> ArticulatedObject:
    model = ArticulatedObject(name="diagonally_floating_spheres")

    base = model.part("base")
    base.visual(Sphere(0.05), origin=Origin(xyz=(0.0, 0.0, 0.05)), name="sphere_a")
    base.visual(Sphere(0.05), origin=Origin(xyz=(0.08, 0.08, 0.05)), name="sphere_b")

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


def _build_multi_element_overlapping_parts_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_element_overlapping_parts")

    base = model.part("base")
    base.visual(Box((0.14, 0.14, 0.14)), origin=Origin(xyz=(-0.04, 0.0, 0.07)), name="base_left")
    base.visual(Box((0.14, 0.14, 0.14)), origin=Origin(xyz=(0.04, 0.0, 0.07)), name="base_right")

    child = model.part("child")
    child.visual(
        Box((0.14, 0.14, 0.14)),
        origin=Origin(xyz=(-0.04, 0.0, 0.07)),
        name="child_left",
    )
    child.visual(
        Box((0.14, 0.14, 0.14)),
        origin=Origin(xyz=(0.04, 0.0, 0.07)),
        name="child_right",
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


def _build_pose_specific_part_overlap_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pose_specific_part_overlap")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.0, 0.0, 0.1)), name="base_box")

    slider = model.part("slider")
    slider.visual(Box((0.2, 0.2, 0.2)), origin=Origin(xyz=(0.25, 0.0, 0.1)), name="slider_box")

    model.articulation(
        "base_to_slider",
        ArticulationType.PRISMATIC,
        parent=base,
        child=slider,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.2, upper=0.0),
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


def _build_element_gap_model() -> ArticulatedObject:
    model = ArticulatedObject(name="element_gap")

    base = model.part("base")
    base.visual(Box((0.2, 0.2, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.05)), name="body")

    arm = model.part("arm")
    arm.visual(Box((0.04, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.121)), name="hub")
    arm.visual(Box((0.01, 0.01, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.11)), name="brace")

    return model


def _build_nested_parts_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_parts")

    outer = model.part("outer")
    outer.visual(Box((0.3, 0.3, 0.3)), origin=Origin(xyz=(0.0, 0.0, 0.15)), name="shell")

    inner = model.part("inner")
    inner.visual(Box((0.1, 0.1, 0.1)), origin=Origin(xyz=(0.0, 0.0, 0.15)), name="insert")

    return model


def test_articulation_origin_tolerance_is_truncated_to_three_decimals() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.115))

    assert ctx.check_articulation_origin_near_geometry(tol=0.0159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.015)",)


def test_articulation_origin_tolerance_is_capped_at_point_one_five() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.25))

    assert ctx.check_articulation_origin_near_geometry(tol=0.159)

    report = ctx.report()
    assert report.passed
    assert report.checks == ("check_articulation_origin_near_geometry(tol=0.15)",)


def test_warn_if_articulation_origin_near_geometry_records_warning_only() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.2))

    assert not ctx.warn_if_articulation_origin_near_geometry(tol=0.015)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_articulation_origin_near_geometry(tol=0.015)",)
    assert len(report.warnings) == 2
    assert report.warnings[0].startswith("DEPRECATED AS DEFAULT: ")
    assert "warn_if_articulation_origin_near_geometry(...)" in report.warnings[0]
    assert "warn_if_articulation_origin_near_geometry(tol=0.015)" in report.warnings[1]
    assert "Articulation origin(s) far from geometry" in report.warnings[1]


def test_warn_if_part_geometry_disconnected_records_warning_only() -> None:
    ctx = SDKTestContext(_build_disconnected_part_model())

    assert not ctx.warn_if_part_geometry_disconnected()

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_part_geometry_disconnected(tol=1e-06)",)
    assert len(report.warnings) == 1
    assert "warn_if_part_geometry_disconnected(tol=1e-06)" in report.warnings[0]
    assert "Disconnected geometry islands detected" in report.warnings[0]


def test_warn_if_part_geometry_connected_keeps_legacy_alias() -> None:
    ctx = SDKTestContext(_build_disconnected_part_model())

    assert not ctx.warn_if_part_geometry_connected()

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_part_geometry_connected(tol=1e-06)",)
    assert len(report.warnings) == 1
    assert "warn_if_part_geometry_connected(tol=1e-06)" in report.warnings[0]
    assert "Disconnected geometry islands detected" in report.warnings[0]


def test_warn_if_part_geometry_disconnected_uses_exact_geometry_not_aabb_overlap() -> None:
    ctx = SDKTestContext(_build_diagonally_floating_spheres_model())

    assert not ctx.warn_if_part_geometry_disconnected()

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_part_geometry_disconnected(tol=1e-06)",)
    assert len(report.warnings) == 1
    assert "connected=1/2" in report.warnings[0]


def test_warn_if_overlaps_records_warning_only() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())

    assert not ctx.warn_if_overlaps(max_pose_samples=8, overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_overlaps(samples=8,ignore_adjacent=False,ignore_fixed=True)",)
    assert len(report.warnings) == 2
    assert report.warnings[0].startswith("DEPRECATED AS DEFAULT: ")
    assert "warn_if_overlaps(...)" in report.warnings[0]
    assert (
        "warn_if_overlaps(samples=8,ignore_adjacent=False,ignore_fixed=True)" in report.warnings[1]
    )
    assert "Overlaps detected" in report.warnings[1]
    assert "overlap_tol=0.001" in report.warnings[1]
    assert "overlap_volume_tol=0" in report.warnings[1]
    assert "relation=unrelated" in report.warnings[1]
    assert "depth=(0.2,0.2,0.2)" in report.warnings[1]
    assert "elem_a=#0 'base_box':Box" in report.warnings[1]
    assert "elem_b=#0 'child_box':Box" in report.warnings[1]


def test_check_no_part_overlaps_fails_for_rest_pose_part_overlap() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())

    assert not ctx.check_no_part_overlaps(overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert not report.passed
    assert report.checks == ("check_no_part_overlaps(overlap_tol=0.001,overlap_volume_tol=0)",)
    assert len(report.failures) == 1
    assert (
        report.failures[0].name == "check_no_part_overlaps(overlap_tol=0.001,overlap_volume_tol=0)"
    )
    assert "Part overlaps detected" in report.failures[0].details
    assert "pair=('base','child')" in report.failures[0].details
    assert "depth=(0.2,0.2,0.2)" in report.failures[0].details
    assert "elem_a=#0 'base_box':Box" in report.failures[0].details
    assert "elem_b=#0 'child_box':Box" in report.failures[0].details
    assert "pose={}" in report.failures[0].details


def test_check_no_part_overlaps_uses_current_pose_only() -> None:
    model = _build_pose_specific_part_overlap_model()
    articulation = model.get_articulation("base_to_slider")
    ctx = SDKTestContext(model)

    assert ctx.check_no_part_overlaps(overlap_tol=0.001, overlap_volume_tol=0.0)
    with ctx.pose({articulation: -0.1}):
        assert not ctx.check_no_part_overlaps(overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert report.checks == (
        "check_no_part_overlaps(overlap_tol=0.001,overlap_volume_tol=0)",
        "check_no_part_overlaps(overlap_tol=0.001,overlap_volume_tol=0)",
    )
    assert len(report.failures) == 1
    assert "pair=('base','slider')" in report.failures[0].details
    assert "pose={'base_to_slider': -0.1}" in report.failures[0].details


def test_check_no_part_overlaps_aggregates_to_one_failure_per_part_pair() -> None:
    ctx = SDKTestContext(_build_multi_element_overlapping_parts_model())

    assert not ctx.check_no_part_overlaps(overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert len(report.failures) == 1
    assert report.failures[0].details.count("pair=('base','child')") == 1


def test_allow_overlap_suppresses_check_no_part_overlaps() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())
    ctx.allow_overlap("base", "child", reason="bowl nests into the seated opening")

    assert ctx.check_no_part_overlaps(overlap_tol=0.001, overlap_volume_tol=0.0)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_no_part_overlaps(overlap_tol=0.001,overlap_volume_tol=0)",)
    assert report.allowances == (
        "allow_overlap('base', 'child'): bowl nests into the seated opening",
    )
    assert len(report.warnings) == 1
    assert "Overlaps detected but allowed by justification" in report.warnings[0]


def test_deprecated_default_helper_warning_emits_once_per_helper() -> None:
    ctx = SDKTestContext(_build_joint_origin_model(joint_z=0.2))

    assert not ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    assert not ctx.warn_if_articulation_origin_near_geometry(tol=0.015)

    report = ctx.report()
    assert (
        sum(
            warning.startswith(
                "DEPRECATED AS DEFAULT: warn_if_articulation_origin_near_geometry(...)"
            )
            for warning in report.warnings
        )
        == 1
    )


def test_check_articulation_overlaps_fails_for_revolute_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.REVOLUTE),
    )

    assert not ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert not report.passed
    assert report.checks == ("check_articulation_overlaps(samples=8)",)
    assert len(report.failures) == 1
    assert report.failures[0].name == "check_articulation_overlaps(samples=8)"
    assert "pair=('base','child')" in report.failures[0].details


def test_warn_if_articulation_overlaps_records_warning_only() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.REVOLUTE),
    )

    assert not ctx.warn_if_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("warn_if_articulation_overlaps(samples=8)",)
    assert len(report.warnings) == 1
    assert report.warnings[0].startswith("warn_if_articulation_overlaps(samples=8): ")
    assert "pair=('base','child')" in report.warnings[0]


def test_check_articulation_overlaps_fails_for_prismatic_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.PRISMATIC),
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
    )

    assert ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_articulation_overlaps(samples=8)",)


def test_check_articulation_overlaps_ignores_non_articulation_pairs() -> None:
    ctx = SDKTestContext(_build_non_articulation_overlap_only_model())

    assert ctx.check_articulation_overlaps(max_pose_samples=8)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("check_articulation_overlaps(samples=8)",)


def test_allow_overlap_suppresses_reported_articulation_pair() -> None:
    ctx = SDKTestContext(
        _build_articulation_overlap_model(joint_type=ArticulationType.REVOLUTE),
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


def test_allow_isolated_part_records_allowance_and_structured_part_list() -> None:
    model = _build_overlapping_parts_model()
    child = model.get_part("child")
    ctx = SDKTestContext(model)

    ctx.allow_isolated_part(child, reason="intentionally freestanding accent")

    report = ctx.report()
    assert report.allowances == ("allow_isolated_part('child'): intentionally freestanding accent",)
    assert report.allowed_isolated_parts == ("child",)


def test_allow_isolated_part_accepts_string_name() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())

    ctx.allow_isolated_part("child", reason="intentionally freestanding accent")

    report = ctx.report()
    assert report.allowed_isolated_parts == ("child",)


def test_allow_isolated_part_rejects_empty_reason() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())

    try:
        ctx.allow_isolated_part("child", reason="  ")
    except ValueError as exc:
        assert "allow_isolated_part requires a non-empty reason" in str(exc)
    else:  # pragma: no cover - assertion helper
        raise AssertionError("expected ValueError")


def test_warn_if_coplanar_surfaces_records_warning_only() -> None:
    ctx = SDKTestContext(_build_coplanar_surface_model())

    assert not ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
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
        "warn_if_coplanar_surfaces(samples=1,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=False,ignore_fixed=False)",
    )
    assert len(report.warnings) == 1
    assert (
        "warn_if_coplanar_surfaces(samples=1,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=False,ignore_fixed=False)"
        in report.warnings[0]
    )
    assert "Coplanar or nearly coplanar surfaces detected (max_risk=medium" in report.warnings[0]
    assert "risk=medium relation=unrelated pair=('base','cap')" in report.warnings[0]


def test_warn_if_coplanar_surfaces_defaults_ignore_adjacent_pairs() -> None:
    ctx = SDKTestContext(_build_adjacent_coplanar_surface_model())

    assert ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
        plane_tol=0.001,
        min_overlap=0.05,
    )

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.warnings == ()
    assert report.checks == (
        "warn_if_coplanar_surfaces(samples=1,plane_tol=0.001,min_overlap=0.05,min_overlap_ratio=0.35,ignore_adjacent=True,ignore_fixed=True)",
    )


def test_allow_coplanar_surfaces_suppresses_reported_pair() -> None:
    ctx = SDKTestContext(_build_coplanar_surface_model())
    ctx.allow_coplanar_surfaces("base", "cap", reason="flush mounted cap")

    assert ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
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


def test_allow_overlap_accepts_part_and_visual_objects() -> None:
    model = _build_overlapping_parts_model()
    base = model.get_part("base")
    child = model.get_part("child")
    base_box = base.visuals[0]
    child_box = child.visuals[0]

    ctx = SDKTestContext(model)
    ctx.allow_overlap(base, child, reason="bearing sleeve nests around the hinge pin")
    ctx.allow_overlap(
        base, child, reason="explicit element allowance", elem_a=base_box, elem_b=child_box
    )

    report = ctx.report()
    assert report.passed
    assert report.allowances == (
        "allow_overlap('base', 'child'): bearing sleeve nests around the hinge pin",
        "allow_overlap('base', 'child', elem_a='base_box', elem_b='child_box'): explicit element allowance",
    )


def test_allow_coplanar_surfaces_accepts_part_objects() -> None:
    model = _build_coplanar_surface_model()
    base = model.get_part("base")
    cap = model.get_part("cap")

    ctx = SDKTestContext(model)
    ctx.allow_coplanar_surfaces(base, cap, reason="flush mounted cap")

    assert ctx.warn_if_coplanar_surfaces(
        max_pose_samples=1,
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


def test_expect_aabb_gap_keeps_whole_link_behavior_by_default() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert ctx.expect_aabb_gap("arm", "base", axis="z", max_gap=0.0, max_penetration=0.0)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("expect_aabb_gap(arm,base,axis=z)",)
    assert len(report.warnings) == 1
    assert "DEPRECATED: expect_aabb_gap(...)" in report.warnings[0]


def test_expect_aabb_gap_can_target_named_elements() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert not ctx.expect_aabb_gap(
        "arm",
        "base",
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem="hub",
        negative_elem="body",
    )

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "expect_aabb_gap(arm,base,axis=z)"
    assert "gap_z=0.001" in report.failures[0].details
    assert "positive_elem='hub'" in report.failures[0].details
    assert "negative_elem='body'" in report.failures[0].details
    assert any("DEPRECATED: expect_aabb_gap(...)" in warning for warning in report.warnings)


def test_expect_aabb_gap_accepts_part_and_visual_objects() -> None:
    model = _build_element_gap_model()
    arm = model.get_part("arm")
    base = model.get_part("base")
    hub = arm.visuals[0]
    body = base.visuals[0]

    ctx = SDKTestContext(model)

    assert not ctx.expect_aabb_gap(
        arm,
        base,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=hub,
        negative_elem=body,
    )

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "expect_aabb_gap(arm,base,axis=z)"
    assert "gap_z=0.001" in report.failures[0].details
    assert "positive_elem='hub'" in report.failures[0].details
    assert "negative_elem='body'" in report.failures[0].details
    assert any("DEPRECATED: expect_aabb_gap(...)" in warning for warning in report.warnings)


def test_pose_accepts_articulation_objects() -> None:
    model = _build_joint_origin_model(joint_z=0.05)
    child = model.get_part("child")
    joint = model.get_articulation("base_to_child")

    ctx = SDKTestContext(model)
    rest_aabb = ctx.link_world_aabb(child)
    assert rest_aabb is not None

    with ctx.pose({joint: 0.5}):
        posed_aabb = ctx.link_world_aabb(child)

    assert posed_aabb is not None
    assert posed_aabb != rest_aabb


def test_expect_aabb_gap_reports_missing_named_element() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert not ctx.expect_aabb_gap(
        "arm",
        "base",
        axis="z",
        max_gap=0.0,
        positive_elem="missing_hub",
    )

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert (
        "missing element AABB for positive_elem='missing_hub' on 'arm'"
        in report.failures[0].details
    )
    assert any("DEPRECATED: expect_aabb_gap(...)" in warning for warning in report.warnings)


def test_expect_contact_uses_exact_visual_geometry() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert ctx.expect_contact("arm", "base")

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("expect_contact(arm,base)",)


def test_expect_contact_can_target_exact_named_elements() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert not ctx.expect_contact("arm", "base", elem_a="hub", elem_b="body", contact_tol=1e-6)

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "expect_contact(arm,base)"
    assert "min_distance=0.001" in report.failures[0].details
    assert "elem_a='hub'" in report.failures[0].details
    assert "elem_b='body'" in report.failures[0].details


def test_expect_gap_uses_exact_visual_projection() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert ctx.expect_gap("arm", "base", axis="z", max_gap=0.0, max_penetration=0.0)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("expect_gap(arm,base,axis=z)",)


def test_expect_gap_can_target_named_elements() -> None:
    ctx = SDKTestContext(_build_element_gap_model())

    assert not ctx.expect_gap(
        "arm",
        "base",
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem="hub",
        negative_elem="body",
    )

    report = ctx.report()
    assert not report.passed
    assert len(report.failures) == 1
    assert report.failures[0].name == "expect_gap(arm,base,axis=z)"
    assert "gap_z=0.001" in report.failures[0].details
    assert "positive_elem='hub'" in report.failures[0].details
    assert "negative_elem='body'" in report.failures[0].details


def test_expect_overlap_uses_exact_visual_geometry() -> None:
    ctx = SDKTestContext(_build_overlapping_parts_model())

    assert ctx.expect_overlap("base", "child", axes="xy", min_overlap=0.19)

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("expect_overlap(base,child,axes=xy)",)


def test_expect_within_uses_exact_visual_geometry() -> None:
    ctx = SDKTestContext(_build_nested_parts_model())

    assert ctx.expect_within("inner", "outer", axes="xyz")

    report = ctx.report()
    assert report.passed
    assert report.failures == ()
    assert report.checks == ("expect_within(inner,outer,axes=xyz)",)
