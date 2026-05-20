from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


FRONT_RAIL_TOP = (0.002, 0.205, -0.026)
FRONT_RAIL_FOOT = (0.202, 0.205, -1.036)
# Rear rails sit slightly inboard of the front rails so the A-frame can nest
# around the hinge line without rail-on-rail interference.
REAR_RAIL_TOP = (-0.058, 0.170, -0.045)
REAR_RAIL_FOOT = (-0.318, 0.170, -1.008)

LEFT_FRONT_BRACE_PIVOT = (0.053, 0.193, -0.418)
LEFT_REAR_BRACE_PIVOT = (-0.172, 0.187, -0.420)
LEFT_BRACE_CENTER = (-0.036, 0.190, -0.534)


def _mirror_y(point: tuple[float, float, float]) -> tuple[float, float, float]:
    return (point[0], -point[1], point[2])


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rail_pitch(top: tuple[float, float, float], foot: tuple[float, float, float]) -> float:
    return math.atan2(top[0] - foot[0], top[2] - foot[2])


def _segment_length(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(
        (b[0] - a[0]) ** 2
        + (b[1] - a[1]) ** 2
        + (b[2] - a[2]) ** 2
    )


def _brace_open_pitch(
    pivot: tuple[float, float, float],
    center: tuple[float, float, float],
) -> float:
    dx = pivot[0] - center[0]
    dz = pivot[2] - center[2]
    return math.atan2(dx, dz)


def _point_on_segment(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    )


def _add_leg(
    part,
    *,
    top: tuple[float, float, float],
    foot: tuple[float, float, float],
    rail_size: tuple[float, float],
    metal,
    foot_material,
    rail_name: str,
    foot_name: str,
) -> None:
    center = _midpoint(top, foot)
    part.visual(
        Box((rail_size[0], rail_size[1], _segment_length(top, foot))),
        origin=Origin(xyz=center, rpy=(0.0, _rail_pitch(top, foot), 0.0)),
        material=metal,
        name=rail_name,
    )
    part.visual(
        Box((0.050, 0.028, 0.024)),
        origin=Origin(xyz=(foot[0], foot[1], foot[2] - 0.011)),
        material=foot_material,
        name=foot_name,
    )


def _add_tread(
    part,
    *,
    index: int,
    center: tuple[float, float, float],
    tread_depth: float,
    tread_width: float,
    tread_height: float,
    metal,
    elastomer,
) -> None:
    part.visual(
        Box((tread_depth, tread_width, tread_height)),
        origin=Origin(xyz=center),
        material=metal,
        name=f"tread_{index}",
    )
    part.visual(
        Cylinder(radius=0.010, length=tread_width),
        origin=Origin(
            xyz=(center[0] + tread_depth * 0.36, center[1], center[2] + 0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name=f"tread_nose_{index}",
    )
    part.visual(
        Box((tread_depth * 0.72, tread_width * 0.90, 0.004)),
        origin=Origin(xyz=(center[0] - 0.002, center[1], center[2] + tread_height * 0.5 + 0.002)),
        material=elastomer,
        name=f"tread_pad_{index}",
    )


def _build_spreader_bar(part, metal) -> None:
    stop_radius = 0.0085
    bar_width = 0.018
    bar_thickness = 0.006
    length = part.meta["bar_length"]
    lateral_offset = part.meta["lateral_offset"]
    hinge_clearance = 0.014
    strap_length = max(length - hinge_clearance, 0.020)

    part.visual(
        Box((bar_width, bar_thickness, strap_length)),
        origin=Origin(
            xyz=(0.0, lateral_offset, -(hinge_clearance + strap_length * 0.5)),
        ),
        material=metal,
        name="bar",
    )
    part.visual(
        Cylinder(radius=stop_radius, length=bar_thickness),
        origin=Origin(xyz=(0.0, lateral_offset, -length), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="center_stop",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_step_ladder")

    painted_metal = model.material("painted_metal", rgba=(0.93, 0.94, 0.95, 1.0))
    brace_metal = model.material("brace_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    polymer = model.material("polymer_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    elastomer = model.material("elastomer_black", rgba=(0.08, 0.08, 0.09, 1.0))

    top_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.158, 0.430, 0.020, corner_segments=8),
            0.046,
            cap=True,
            center=True,
        ),
        "step_ladder_top_cap",
    )

    front_frame = model.part("front_frame")
    _add_leg(
        front_frame,
        top=FRONT_RAIL_TOP,
        foot=FRONT_RAIL_FOOT,
        rail_size=(0.038, 0.020),
        metal=painted_metal,
        foot_material=elastomer,
        rail_name="left_front_rail",
        foot_name="left_foot",
    )
    _add_leg(
        front_frame,
        top=_mirror_y(FRONT_RAIL_TOP),
        foot=_mirror_y(FRONT_RAIL_FOOT),
        rail_size=(0.038, 0.020),
        metal=painted_metal,
        foot_material=elastomer,
        rail_name="right_front_rail",
        foot_name="right_foot",
    )

    front_frame.visual(
        Box((0.050, 0.360, 0.028)),
        origin=Origin(xyz=(0.038, 0.0, -0.010)),
        material=painted_metal,
        name="front_top_spine",
    )
    front_frame.visual(
        Box((0.132, 0.392, 0.022)),
        origin=Origin(xyz=(0.076, 0.0, -0.028)),
        material=painted_metal,
        name="top_platform",
    )
    front_frame.visual(
        top_cap_mesh,
        origin=Origin(xyz=(0.092, 0.0, 0.002)),
        material=polymer,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.070, 0.208, 0.006)),
        origin=Origin(xyz=(0.094, 0.0, 0.025)),
        material=polymer_dark,
        name="top_grip",
    )
    front_frame.visual(
        Box((0.044, 0.042, 0.046)),
        origin=Origin(xyz=(0.036, 0.186, -0.016)),
        material=painted_metal,
        name="left_top_cheek",
    )
    front_frame.visual(
        Box((0.044, 0.042, 0.046)),
        origin=Origin(xyz=(0.036, -0.186, -0.016)),
        material=painted_metal,
        name="right_top_cheek",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.000, 0.214, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="left_hinge_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.000, -0.214, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="right_hinge_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.053, 0.199, -0.418), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="left_front_brace_mount",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.053, -0.199, -0.418), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="right_front_brace_mount",
    )
    front_frame.visual(
        Box((0.034, 0.014, 0.034)),
        origin=Origin(xyz=(0.067, 0.199, -0.418)),
        material=brace_metal,
        name="left_front_brace_bracket",
    )
    front_frame.visual(
        Box((0.034, 0.014, 0.034)),
        origin=Origin(xyz=(0.067, -0.199, -0.418)),
        material=brace_metal,
        name="right_front_brace_bracket",
    )

    tread_depth = 0.110
    tread_width = 0.392
    tread_height = 0.028
    tread_params = (
        (1, 0.224),
        (2, 0.463),
        (3, 0.700),
    )
    for tread_index, t in tread_params:
        rail_mid = _point_on_segment(FRONT_RAIL_TOP, FRONT_RAIL_FOOT, t)
        _add_tread(
            front_frame,
            index=tread_index,
            center=(rail_mid[0], 0.0, rail_mid[2]),
            tread_depth=tread_depth,
            tread_width=tread_width,
            tread_height=tread_height,
            metal=painted_metal,
            elastomer=elastomer,
        )

    rear_frame = model.part("rear_frame")
    _add_leg(
        rear_frame,
        top=REAR_RAIL_TOP,
        foot=REAR_RAIL_FOOT,
        rail_size=(0.034, 0.018),
        metal=painted_metal,
        foot_material=elastomer,
        rail_name="left_rear_rail",
        foot_name="left_foot",
    )
    _add_leg(
        rear_frame,
        top=_mirror_y(REAR_RAIL_TOP),
        foot=_mirror_y(REAR_RAIL_FOOT),
        rail_size=(0.034, 0.018),
        metal=painted_metal,
        foot_material=elastomer,
        rail_name="right_rear_rail",
        foot_name="right_foot",
    )
    rear_frame.visual(
        Box((0.030, 0.392, 0.030)),
        origin=Origin(xyz=(-0.006, 0.0, -0.004)),
        material=painted_metal,
        name="rear_top_yoke",
    )
    rear_frame.visual(
        Box((0.084, 0.360, 0.060)),
        origin=Origin(xyz=(-0.048, 0.0, -0.045)),
        material=painted_metal,
        name="rear_top_bridge",
    )
    rear_frame.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.000, 0.197, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="left_hinge_tab",
    )
    rear_frame.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.000, -0.197, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="right_hinge_tab",
    )
    rear_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.172, 0.186, -0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="left_rear_brace_mount",
    )
    rear_frame.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.172, -0.186, -0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brace_metal,
        name="right_rear_brace_mount",
    )
    rear_frame.visual(
        Box((0.028, 0.026, 0.064)),
        origin=Origin(xyz=(-0.194, 0.176, -0.448)),
        material=brace_metal,
        name="left_rear_brace_gusset",
    )
    rear_frame.visual(
        Box((0.028, 0.026, 0.064)),
        origin=Origin(xyz=(-0.194, -0.176, -0.448)),
        material=brace_metal,
        name="right_rear_brace_gusset",
    )

    left_front_spreader = model.part(
        "left_front_spreader",
        meta={
            "bar_length": _segment_length(LEFT_FRONT_BRACE_PIVOT, LEFT_BRACE_CENTER),
            "lateral_offset": 0.010,
        },
    )
    right_front_spreader = model.part(
        "right_front_spreader",
        meta={
            "bar_length": _segment_length(_mirror_y(LEFT_FRONT_BRACE_PIVOT), _mirror_y(LEFT_BRACE_CENTER)),
            "lateral_offset": -0.010,
        },
    )
    left_rear_spreader = model.part(
        "left_rear_spreader",
        meta={
            "bar_length": _segment_length(LEFT_REAR_BRACE_PIVOT, LEFT_BRACE_CENTER),
            "lateral_offset": 0.010,
        },
    )
    right_rear_spreader = model.part(
        "right_rear_spreader",
        meta={
            "bar_length": _segment_length(_mirror_y(LEFT_REAR_BRACE_PIVOT), _mirror_y(LEFT_BRACE_CENTER)),
            "lateral_offset": -0.010,
        },
    )

    for part in (
        left_front_spreader,
        right_front_spreader,
        left_rear_spreader,
        right_rear_spreader,
    ):
        _build_spreader_bar(part, brace_metal)

    model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.3,
            lower=-0.42,
            upper=0.08,
        ),
    )
    model.articulation(
        "left_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_front_spreader,
        origin=Origin(
            xyz=LEFT_FRONT_BRACE_PIVOT,
            rpy=(0.0, _brace_open_pitch(LEFT_FRONT_BRACE_PIVOT, LEFT_BRACE_CENTER), 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.05, upper=0.12),
    )
    model.articulation(
        "right_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_front_spreader,
        origin=Origin(
            xyz=_mirror_y(LEFT_FRONT_BRACE_PIVOT),
            rpy=(0.0, _brace_open_pitch(_mirror_y(LEFT_FRONT_BRACE_PIVOT), _mirror_y(LEFT_BRACE_CENTER)), 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.05, upper=0.12),
    )
    model.articulation(
        "left_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=left_rear_spreader,
        origin=Origin(
            xyz=LEFT_REAR_BRACE_PIVOT,
            rpy=(0.0, _brace_open_pitch(LEFT_REAR_BRACE_PIVOT, LEFT_BRACE_CENTER), 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.12, upper=1.25),
    )
    model.articulation(
        "right_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=right_rear_spreader,
        origin=Origin(
            xyz=_mirror_y(LEFT_REAR_BRACE_PIVOT),
            rpy=(0.0, _brace_open_pitch(_mirror_y(LEFT_REAR_BRACE_PIVOT), _mirror_y(LEFT_BRACE_CENTER)), 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.12, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_front_spreader = object_model.get_part("left_front_spreader")
    right_front_spreader = object_model.get_part("right_front_spreader")
    left_rear_spreader = object_model.get_part("left_rear_spreader")
    right_rear_spreader = object_model.get_part("right_rear_spreader")

    main_hinge = object_model.get_articulation("front_to_rear")
    left_front_joint = object_model.get_articulation("left_front_spreader_hinge")
    right_front_joint = object_model.get_articulation("right_front_spreader_hinge")
    left_rear_joint = object_model.get_articulation("left_rear_spreader_hinge")
    right_rear_joint = object_model.get_articulation("right_rear_spreader_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    hinge_limits = main_hinge.motion_limits
    ctx.check(
        "front_to_rear_axis_is_lateral",
        tuple(main_hinge.axis) == (0.0, 1.0, 0.0),
        f"expected main hinge axis (0, 1, 0), got {main_hinge.axis}",
    )
    ctx.check(
        "front_to_rear_open_and_closed_limits",
        hinge_limits is not None
        and hinge_limits.lower is not None
        and hinge_limits.upper is not None
        and hinge_limits.lower < 0.0 < hinge_limits.upper,
        f"expected open-at-zero and negative closed limit, got {hinge_limits}",
    )
    ctx.check(
        "spreaders_share_same_axis_family",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in (left_front_joint, right_front_joint, left_rear_joint, right_rear_joint)),
        "all spreader hinges should rotate about the same side-to-side axis",
    )

    with ctx.pose(
        {
            main_hinge: 0.0,
            left_front_joint: 0.0,
            right_front_joint: 0.0,
            left_rear_joint: 0.0,
            right_rear_joint: 0.0,
        }
    ):
        ctx.expect_contact(
            front_frame,
            rear_frame,
            elem_a="left_hinge_sleeve",
            elem_b="left_hinge_tab",
            contact_tol=0.001,
            name="left_hinge_reads_as_pinned_contact",
        )
        ctx.expect_contact(
            front_frame,
            rear_frame,
            elem_a="right_hinge_sleeve",
            elem_b="right_hinge_tab",
            contact_tol=0.001,
            name="right_hinge_reads_as_pinned_contact",
        )
        ctx.expect_contact(
            left_front_spreader,
            left_rear_spreader,
            elem_a="center_stop",
            elem_b="center_stop",
            contact_tol=0.001,
            name="left_spreader_pair_meets_at_open_stop",
        )
        ctx.expect_contact(
            right_front_spreader,
            right_rear_spreader,
            elem_a="center_stop",
            elem_b="center_stop",
            contact_tol=0.001,
            name="right_spreader_pair_meets_at_open_stop",
        )
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="x",
            positive_elem="left_foot",
            negative_elem="left_foot",
            min_gap=0.42,
            name="left_foot_pair_forms_open_aframe_stance",
        )
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="x",
            positive_elem="right_foot",
            negative_elem="right_foot",
            min_gap=0.42,
            name="right_foot_pair_forms_open_aframe_stance",
        )

    with ctx.pose(
        {
            main_hinge: -0.38,
            left_front_joint: -0.90,
            right_front_joint: -0.90,
            left_rear_joint: 1.15,
            right_rear_joint: 1.15,
        }
    ):
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="x",
            positive_elem="left_foot",
            negative_elem="left_foot",
            min_gap=0.05,
            max_gap=0.18,
            name="left_foot_pair_closes_to_compact_stow_gap",
        )
        ctx.expect_overlap(
            front_frame,
            rear_frame,
            axes="yz",
            min_overlap=0.36,
            name="closed_ladder_reads_as_nested_stack",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
