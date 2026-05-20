from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_aframe_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    cap_blue = model.material("cap_blue", rgba=(0.22, 0.31, 0.43, 1.0))
    accent_yellow = model.material("accent_yellow", rgba=(0.91, 0.76, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    hinge_z = 1.18
    half_width = 0.22

    front_rail_length = 1.19
    front_foot_x = 0.34
    front_rail_angle = -math.atan2(front_foot_x, hinge_z)
    front_rail_center_x = front_foot_x / 2.0
    front_rail_center_z = hinge_z / 2.0

    rear_top_x = -0.03
    rear_foot_x = -0.37
    rear_rail_length = math.sqrt((rear_foot_x - rear_top_x) ** 2 + hinge_z**2)
    rear_rail_angle = math.atan2(abs(rear_foot_x - rear_top_x), hinge_z)
    rear_rail_center_x = (rear_top_x + rear_foot_x) / 2.0
    rear_rail_center_z = hinge_z / 2.0 - hinge_z

    front_frame = model.part("front_frame")

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        y_pos = side_sign * half_width
        front_frame.visual(
            Box((0.032, 0.024, front_rail_length)),
            origin=Origin(
                xyz=(front_rail_center_x, y_pos, front_rail_center_z),
                rpy=(0.0, front_rail_angle, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_front_rail",
        )
        front_frame.visual(
            Box((0.11, 0.042, 0.022)),
            origin=Origin(xyz=(front_foot_x, y_pos, 0.011)),
            material=rubber,
            name=f"{side_name}_front_foot",
        )

    tread_zs = (0.27, 0.52, 0.77, 1.00)
    tread_xs = (0.24, 0.18, 0.11, 0.05)
    for index, (x_pos, z_pos) in enumerate(zip(tread_xs, tread_zs), start=1):
        front_frame.visual(
            Box((0.13, 0.43, 0.028)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=graphite,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Box((0.112, 0.36, 0.004)),
            origin=Origin(xyz=(x_pos + 0.002, 0.0, z_pos + 0.013)),
            material=accent_yellow,
            name=f"tread_{index}_index_strip",
        )

    front_frame.visual(
        Box((0.19, 0.46, 0.055)),
        origin=Origin(xyz=(0.08, 0.0, 1.155)),
        material=cap_blue,
        name="cap_body",
    )
    front_frame.visual(
        Box((0.14, 0.18, 0.006)),
        origin=Origin(xyz=(0.086, 0.0, 1.182)),
        material=graphite,
        name="top_datum_pad",
    )
    front_frame.visual(
        Box((0.11, 0.008, 0.003)),
        origin=Origin(xyz=(0.086, 0.0, 1.185)),
        material=accent_yellow,
        name="cap_centerline_mark",
    )
    front_frame.visual(
        Box((0.012, 0.10, 0.003)),
        origin=Origin(xyz=(0.086, 0.0, 1.185)),
        material=accent_yellow,
        name="cap_cross_mark",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        y_hinge = side_sign * 0.20
        y_brace = side_sign * 0.195
        front_frame.visual(
            Box((0.012, 0.032, 0.065)),
            origin=Origin(xyz=(-0.005, y_hinge, hinge_z)),
            material=aluminum,
            name=f"{side_name}_hinge_stop",
        )
        front_frame.visual(
            Box((0.016, 0.030, 0.055)),
            origin=Origin(xyz=(0.134, y_brace, 0.72)),
            material=aluminum,
            name=f"{side_name}_brace_pivot_pad",
        )

    rear_frame = model.part("rear_frame")

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        y_pos = side_sign * half_width
        rear_frame.visual(
            Box((0.030, 0.022, rear_rail_length)),
            origin=Origin(
                xyz=(rear_rail_center_x, y_pos, rear_rail_center_z),
                rpy=(0.0, rear_rail_angle, 0.0),
            ),
            material=aluminum,
            name=f"{side_name}_rear_rail",
        )
        rear_frame.visual(
            Box((0.11, 0.042, 0.022)),
            origin=Origin(xyz=(rear_foot_x, y_pos, 0.011 - hinge_z)),
            material=rubber,
            name=f"{side_name}_rear_foot",
        )
        rear_frame.visual(
            Box((0.012, 0.024, 0.070)),
            origin=Origin(xyz=(-0.017, side_sign * 0.20, 0.0)),
            material=aluminum,
            name=f"{side_name}_hinge_leaf",
        )
        rear_frame.visual(
            Box((0.100, 0.040, 0.070)),
            origin=Origin(xyz=(-0.195, side_sign * 0.201, -0.75)),
            material=aluminum,
            name=f"{side_name}_brace_stop",
        )

    rear_frame.visual(
        Box((0.100, 0.44, 0.028)),
        origin=Origin(xyz=(-0.205, 0.0, -0.72)),
        material=aluminum,
        name="rear_cross_bar_lower",
    )
    rear_frame.visual(
        Box((0.080, 0.44, 0.028)),
        origin=Origin(xyz=(-0.120, 0.0, -0.36)),
        material=aluminum,
        name="rear_cross_bar_upper",
    )
    rear_frame.visual(
        Box((0.045, 0.44, 0.022)),
        origin=Origin(xyz=(-0.055, 0.0, -0.055)),
        material=cap_blue,
        name="rear_alignment_bar",
    )
    rear_frame.visual(
        Box((0.07, 0.010, 0.003)),
        origin=Origin(xyz=(-0.055, 0.0, -0.0425)),
        material=accent_yellow,
        name="rear_index_mark",
    )

    left_spreader = model.part("left_spreader")
    left_spreader.visual(
        Box((0.008, 0.024, 0.036)),
        origin=Origin(xyz=(-0.004, 0.0, -0.012)),
        material=aluminum,
        name="left_spreader_lug",
    )
    left_spreader.visual(
        Box((0.010, 0.026, 0.393)),
        origin=Origin(
            xyz=(-0.1325, 0.0, -0.145),
            rpy=(0.0, -2.40, 0.0),
        ),
        material=aluminum,
        name="left_spreader_bar",
    )
    left_spreader.visual(
        Box((0.012, 0.026, 0.055)),
        origin=Origin(xyz=(-0.265, 0.0, -0.29)),
        material=graphite,
        name="left_spreader_tip",
    )
    left_spreader.visual(
        Box((0.004, 0.018, 0.15)),
        origin=Origin(
            xyz=(-0.135, 0.0, -0.145),
            rpy=(0.0, -2.40, 0.0),
        ),
        material=accent_yellow,
        name="left_spreader_scale",
    )

    right_spreader = model.part("right_spreader")
    right_spreader.visual(
        Box((0.008, 0.024, 0.036)),
        origin=Origin(xyz=(-0.004, 0.0, -0.012)),
        material=aluminum,
        name="right_spreader_lug",
    )
    right_spreader.visual(
        Box((0.010, 0.026, 0.393)),
        origin=Origin(
            xyz=(-0.1325, 0.0, -0.145),
            rpy=(0.0, -2.40, 0.0),
        ),
        material=aluminum,
        name="right_spreader_bar",
    )
    right_spreader.visual(
        Box((0.012, 0.026, 0.055)),
        origin=Origin(xyz=(-0.265, 0.0, -0.29)),
        material=graphite,
        name="right_spreader_tip",
    )
    right_spreader.visual(
        Box((0.004, 0.018, 0.15)),
        origin=Origin(
            xyz=(-0.135, 0.0, -0.145),
            rpy=(0.0, -2.40, 0.0),
        ),
        material=accent_yellow,
        name="right_spreader_scale",
    )

    model.articulation(
        "rear_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.06,
        ),
    )

    model.articulation(
        "left_spreader_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spreader,
        origin=Origin(xyz=(0.126, -0.195, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-1.02,
            upper=0.08,
        ),
    )

    model.articulation(
        "right_spreader_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spreader,
        origin=Origin(xyz=(0.126, 0.195, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-1.02,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_spreader = object_model.get_part("left_spreader")
    right_spreader = object_model.get_part("right_spreader")

    rear_pivot = object_model.get_articulation("rear_pivot")
    left_spreader_pivot = object_model.get_articulation("left_spreader_pivot")
    right_spreader_pivot = object_model.get_articulation("right_spreader_pivot")

    left_hinge_stop = front_frame.get_visual("left_hinge_stop")
    right_hinge_stop = front_frame.get_visual("right_hinge_stop")
    left_hinge_leaf = rear_frame.get_visual("left_hinge_leaf")
    right_hinge_leaf = rear_frame.get_visual("right_hinge_leaf")

    left_brace_pivot_pad = front_frame.get_visual("left_brace_pivot_pad")
    right_brace_pivot_pad = front_frame.get_visual("right_brace_pivot_pad")
    left_spreader_lug = left_spreader.get_visual("left_spreader_lug")
    right_spreader_lug = right_spreader.get_visual("right_spreader_lug")

    left_spreader_tip = left_spreader.get_visual("left_spreader_tip")
    right_spreader_tip = right_spreader.get_visual("right_spreader_tip")
    left_brace_stop = rear_frame.get_visual("left_brace_stop")
    right_brace_stop = rear_frame.get_visual("right_brace_stop")

    front_left_foot = front_frame.get_visual("left_front_foot")
    rear_left_foot = rear_frame.get_visual("left_rear_foot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits_ok = (
        rear_pivot.motion_limits is not None
        and left_spreader_pivot.motion_limits is not None
        and right_spreader_pivot.motion_limits is not None
        and rear_pivot.motion_limits.lower is not None
        and rear_pivot.motion_limits.upper is not None
        and left_spreader_pivot.motion_limits.lower is not None
        and left_spreader_pivot.motion_limits.upper is not None
        and right_spreader_pivot.motion_limits.lower is not None
        and right_spreader_pivot.motion_limits.upper is not None
        and rear_pivot.motion_limits.lower < 0.0 < rear_pivot.motion_limits.upper
        and left_spreader_pivot.motion_limits.lower < 0.0 < left_spreader_pivot.motion_limits.upper
        and right_spreader_pivot.motion_limits.lower < 0.0 < right_spreader_pivot.motion_limits.upper
    )
    ctx.check(
        "mechanisms define open and closed states",
        limits_ok,
        details="rear pivot and both spreader pivots should straddle the nominal open pose at q=0",
    )

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=left_hinge_stop,
        negative_elem=left_hinge_leaf,
        max_gap=0.002,
        max_penetration=0.0,
        name="left hinge knuckle seats without penetration",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="yz",
        elem_a=left_hinge_stop,
        elem_b=left_hinge_leaf,
        min_overlap=0.02,
        name="left hinge knuckle alignment",
    )
    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=right_hinge_stop,
        negative_elem=right_hinge_leaf,
        max_gap=0.002,
        max_penetration=0.0,
        name="right hinge knuckle seats without penetration",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="yz",
        elem_a=right_hinge_stop,
        elem_b=right_hinge_leaf,
        min_overlap=0.02,
        name="right hinge knuckle alignment",
    )

    ctx.expect_gap(
        front_frame,
        left_spreader,
        axis="x",
        positive_elem=left_brace_pivot_pad,
        negative_elem=left_spreader_lug,
        max_gap=0.002,
        max_penetration=0.0,
        name="left spreader pivot lands on front pad",
    )
    ctx.expect_overlap(
        front_frame,
        left_spreader,
        axes="yz",
        elem_a=left_brace_pivot_pad,
        elem_b=left_spreader_lug,
        min_overlap=0.018,
        name="left spreader pivot yz registration",
    )
    ctx.expect_gap(
        front_frame,
        right_spreader,
        axis="x",
        positive_elem=right_brace_pivot_pad,
        negative_elem=right_spreader_lug,
        max_gap=0.002,
        max_penetration=0.0,
        name="right spreader pivot lands on front pad",
    )
    ctx.expect_overlap(
        front_frame,
        right_spreader,
        axes="yz",
        elem_a=right_brace_pivot_pad,
        elem_b=right_spreader_lug,
        min_overlap=0.018,
        name="right spreader pivot yz registration",
    )

    ctx.expect_gap(
        left_spreader,
        rear_frame,
        axis="x",
        positive_elem=left_spreader_tip,
        negative_elem=left_brace_stop,
        max_gap=0.003,
        max_penetration=0.0,
        name="left spreader stop closes the open stance",
    )
    ctx.expect_overlap(
        left_spreader,
        rear_frame,
        axes="yz",
        elem_a=left_spreader_tip,
        elem_b=left_brace_stop,
        min_overlap=0.02,
        name="left spreader stop alignment",
    )
    ctx.expect_gap(
        right_spreader,
        rear_frame,
        axis="x",
        positive_elem=right_spreader_tip,
        negative_elem=right_brace_stop,
        max_gap=0.003,
        max_penetration=0.0,
        name="right spreader stop closes the open stance",
    )
    ctx.expect_overlap(
        right_spreader,
        rear_frame,
        axes="yz",
        elem_a=right_spreader_tip,
        elem_b=right_brace_stop,
        min_overlap=0.02,
        name="right spreader stop alignment",
    )

    ctx.expect_gap(
        front_frame,
        rear_frame,
        axis="x",
        positive_elem=front_left_foot,
        negative_elem=rear_left_foot,
        min_gap=0.50,
        max_gap=0.66,
        name="open stance keeps a broad front-to-rear footprint",
    )

    with ctx.pose(
        {
            rear_pivot: rear_pivot.motion_limits.lower,
            left_spreader_pivot: left_spreader_pivot.motion_limits.lower,
            right_spreader_pivot: right_spreader_pivot.motion_limits.lower,
        }
    ):
        ctx.expect_gap(
            front_frame,
            rear_frame,
            axis="x",
            positive_elem=front_left_foot,
            negative_elem=rear_left_foot,
            min_gap=0.03,
            max_gap=0.08,
            name="closed stance preserves a controlled nesting gap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
