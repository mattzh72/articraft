from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    aluminum = model.material("aluminum", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.64, 0.66, 0.69, 1.0))
    glass = model.material("glass", rgba=(0.63, 0.79, 0.88, 0.30))
    gasket = model.material("gasket", rgba=(0.12, 0.12, 0.13, 1.0))

    frame_width = 1.20
    frame_height = 1.00
    frame_depth = 0.10
    frame_member = 0.05
    inner_width = frame_width - 2.0 * frame_member
    inner_height = frame_height - 2.0 * frame_member

    base_frame = model.part("base_frame")
    # Outer rectangular frame.
    base_frame.visual(
        Box((frame_width, frame_member, frame_depth)),
        origin=Origin(xyz=(0.0, -(inner_height + frame_member) * 0.5, frame_depth * 0.5)),
        material=aluminum,
        name="sill",
    )
    base_frame.visual(
        Box((frame_width, frame_member, frame_depth)),
        origin=Origin(xyz=(0.0, (inner_height + frame_member) * 0.5, frame_depth * 0.5)),
        material=aluminum,
        name="head",
    )
    base_frame.visual(
        Box((frame_member, inner_height, frame_depth)),
        origin=Origin(xyz=(-(inner_width + frame_member) * 0.5, 0.0, frame_depth * 0.5)),
        material=aluminum,
        name="left_jamb",
    )
    base_frame.visual(
        Box((frame_member, inner_height, frame_depth)),
        origin=Origin(xyz=((inner_width + frame_member) * 0.5, 0.0, frame_depth * 0.5)),
        material=aluminum,
        name="right_jamb",
    )
    # Interior stop lip that gives the frame more realistic depth and a seat for the guide module.
    stop_depth = 0.016
    stop_thickness = 0.016
    stop_width = inner_width
    stop_height = inner_height
    base_frame.visual(
        Box((stop_width, stop_thickness, stop_depth)),
        origin=Origin(xyz=(0.0, -inner_height * 0.5, frame_depth - stop_depth * 0.5)),
        material=dark_aluminum,
        name="lower_stop",
    )
    base_frame.visual(
        Box((stop_width, stop_thickness, stop_depth)),
        origin=Origin(xyz=(0.0, inner_height * 0.5, frame_depth - stop_depth * 0.5)),
        material=dark_aluminum,
        name="upper_stop",
    )
    base_frame.visual(
        Box((stop_thickness, stop_height, stop_depth)),
        origin=Origin(xyz=(-inner_width * 0.5, 0.0, frame_depth - stop_depth * 0.5)),
        material=dark_aluminum,
        name="left_stop",
    )
    base_frame.visual(
        Box((stop_thickness, stop_height, stop_depth)),
        origin=Origin(xyz=(inner_width * 0.5, 0.0, frame_depth - stop_depth * 0.5)),
        material=dark_aluminum,
        name="right_stop",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_height, frame_depth)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, frame_depth * 0.5)),
    )

    guide_module = model.part("guide_module")
    guide_width = inner_width
    guide_depth = 0.076
    guide_z = 0.050
    rail_height = 0.055
    guide_module.visual(
        Box((guide_width, rail_height, guide_depth)),
        origin=Origin(xyz=(0.0, -inner_height * 0.5 + rail_height * 0.5, guide_z)),
        material=dark_aluminum,
        name="lower_guide_body",
    )
    guide_module.visual(
        Box((guide_width, rail_height, guide_depth)),
        origin=Origin(xyz=(0.0, inner_height * 0.5 - rail_height * 0.5, guide_z)),
        material=dark_aluminum,
        name="upper_guide_body",
    )

    runner_height = 0.018
    runner_depth = 0.012
    lower_runner_y = -inner_height * 0.5 + rail_height - 0.005
    upper_runner_y = inner_height * 0.5 - rail_height + 0.005
    rear_track_z = 0.025
    front_track_z = 0.067
    guide_module.visual(
        Box((guide_width, runner_height, runner_depth)),
        origin=Origin(xyz=(0.0, lower_runner_y, rear_track_z)),
        material=gasket,
        name="lower_rear_runner",
    )
    guide_module.visual(
        Box((guide_width, runner_height, runner_depth)),
        origin=Origin(xyz=(0.0, lower_runner_y, front_track_z)),
        material=gasket,
        name="lower_front_runner",
    )
    guide_module.visual(
        Box((guide_width, runner_height, runner_depth)),
        origin=Origin(xyz=(0.0, upper_runner_y, rear_track_z)),
        material=gasket,
        name="upper_rear_runner",
    )
    guide_module.visual(
        Box((guide_width, runner_height, runner_depth)),
        origin=Origin(xyz=(0.0, upper_runner_y, front_track_z)),
        material=gasket,
        name="upper_front_runner",
    )

    fixed_outer_width = 0.57
    fixed_outer_height = 0.80
    fixed_frame_thickness = 0.038
    fixed_frame_depth = 0.020
    fixed_center_x = -0.255
    fixed_center_z = rear_track_z
    guide_module.visual(
        Box((fixed_outer_width, fixed_frame_thickness, fixed_frame_depth)),
        origin=Origin(
            xyz=(
                fixed_center_x,
                -fixed_outer_height * 0.5 + fixed_frame_thickness * 0.5,
                fixed_center_z,
            )
        ),
        material=aluminum,
        name="fixed_bottom_rail",
    )
    guide_module.visual(
        Box((fixed_outer_width, fixed_frame_thickness, fixed_frame_depth)),
        origin=Origin(
            xyz=(
                fixed_center_x,
                fixed_outer_height * 0.5 - fixed_frame_thickness * 0.5,
                fixed_center_z,
            )
        ),
        material=aluminum,
        name="fixed_top_rail",
    )
    guide_module.visual(
        Box(
            (
                fixed_frame_thickness,
                fixed_outer_height - 2.0 * fixed_frame_thickness,
                fixed_frame_depth,
            )
        ),
        origin=Origin(
            xyz=(
                fixed_center_x - fixed_outer_width * 0.5 + fixed_frame_thickness * 0.5,
                0.0,
                fixed_center_z,
            )
        ),
        material=aluminum,
        name="fixed_left_stile",
    )
    guide_module.visual(
        Box(
            (
                fixed_frame_thickness,
                fixed_outer_height - 2.0 * fixed_frame_thickness,
                fixed_frame_depth,
            )
        ),
        origin=Origin(
            xyz=(
                fixed_center_x + fixed_outer_width * 0.5 - fixed_frame_thickness * 0.5,
                0.0,
                fixed_center_z,
            )
        ),
        material=aluminum,
        name="fixed_meeting_stile",
    )
    guide_module.visual(
        Box((0.500, 0.730, 0.008)),
        origin=Origin(xyz=(fixed_center_x, 0.0, fixed_center_z)),
        material=glass,
        name="fixed_glass",
    )
    guide_module.inertial = Inertial.from_geometry(
        Box((guide_width, inner_height, guide_depth)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, guide_z)),
    )

    model.articulation(
        "base_to_guide_module",
        ArticulationType.FIXED,
        parent=base_frame,
        child=guide_module,
        origin=Origin(),
    )

    moving_carriage = model.part("moving_carriage")
    sash_outer_width = 0.57
    sash_outer_height = 0.78
    sash_frame_thickness = 0.044
    sash_depth = 0.024
    sash_center_z = front_track_z
    moving_carriage.visual(
        Box((sash_outer_width, sash_frame_thickness, sash_depth)),
        origin=Origin(
            xyz=(0.0, -sash_outer_height * 0.5 + sash_frame_thickness * 0.5, sash_center_z)
        ),
        material=aluminum,
        name="bottom_rail",
    )
    moving_carriage.visual(
        Box((sash_outer_width, sash_frame_thickness, sash_depth)),
        origin=Origin(
            xyz=(0.0, sash_outer_height * 0.5 - sash_frame_thickness * 0.5, sash_center_z)
        ),
        material=aluminum,
        name="top_rail",
    )
    moving_carriage.visual(
        Box(
            (
                sash_frame_thickness,
                sash_outer_height - 2.0 * sash_frame_thickness,
                sash_depth,
            )
        ),
        origin=Origin(
            xyz=(-sash_outer_width * 0.5 + sash_frame_thickness * 0.5, 0.0, sash_center_z)
        ),
        material=aluminum,
        name="left_stile",
    )
    moving_carriage.visual(
        Box(
            (
                sash_frame_thickness,
                sash_outer_height - 2.0 * sash_frame_thickness,
                sash_depth,
            )
        ),
        origin=Origin(
            xyz=(sash_outer_width * 0.5 - sash_frame_thickness * 0.5, 0.0, sash_center_z)
        ),
        material=aluminum,
        name="meeting_stile",
    )
    moving_carriage.visual(
        Box((sash_outer_width - 0.12, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.386, sash_center_z)),
        material=gasket,
        name="bottom_guide_shoe",
    )
    moving_carriage.visual(
        Box((sash_outer_width - 0.12, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.386, sash_center_z)),
        material=gasket,
        name="top_guide_shoe",
    )
    moving_carriage.visual(
        Box((0.486, 0.696, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, sash_center_z)),
        material=glass,
        name="sash_glass",
    )
    moving_carriage.visual(
        Box((0.024, 0.20, 0.010)),
        origin=Origin(
            xyz=(sash_outer_width * 0.5 - sash_frame_thickness - 0.008, 0.0, sash_center_z + 0.017)
        ),
        material=dark_aluminum,
        name="pull_handle",
    )
    moving_carriage.inertial = Inertial.from_geometry(
        Box((sash_outer_width, sash_outer_height, sash_depth)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, sash_center_z)),
    )

    model.articulation(
        "guide_module_to_moving_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_module,
        child=moving_carriage,
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.32),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    guide_module = object_model.get_part("guide_module")
    moving_carriage = object_model.get_part("moving_carriage")
    slide_joint = object_model.get_articulation("guide_module_to_moving_carriage")
    upper = slide_joint.motion_limits.upper if slide_joint.motion_limits is not None else None

    ctx.expect_contact(
        moving_carriage,
        guide_module,
        elem_a="bottom_guide_shoe",
        elem_b="lower_front_runner",
        name="bottom guide shoe rides on lower front runner",
    )
    ctx.expect_contact(
        moving_carriage,
        guide_module,
        elem_a="top_guide_shoe",
        elem_b="upper_front_runner",
        name="top guide shoe stays captured by upper front runner",
    )
    ctx.expect_gap(
        moving_carriage,
        guide_module,
        axis="z",
        positive_elem="sash_glass",
        negative_elem="fixed_glass",
        min_gap=0.030,
        max_gap=0.038,
        name="sliding sash glass stays on the front track ahead of fixed glass",
    )
    ctx.expect_within(
        moving_carriage,
        base_frame,
        axes="xy",
        margin=0.0,
        name="closed sash remains inside outer frame opening",
    )

    rest_pos = ctx.part_world_position(moving_carriage)
    with ctx.pose({slide_joint: upper}):
        ctx.expect_contact(
            moving_carriage,
            guide_module,
            elem_a="bottom_guide_shoe",
            elem_b="lower_front_runner",
            name="bottom guide shoe stays supported at full opening",
        )
        ctx.expect_contact(
            moving_carriage,
            guide_module,
            elem_a="top_guide_shoe",
            elem_b="upper_front_runner",
            name="top guide shoe stays captured at full opening",
        )
        ctx.expect_overlap(
            moving_carriage,
            guide_module,
            axes="x",
            elem_a="sash_glass",
            elem_b="fixed_glass",
            min_overlap=0.25,
            name="opened sash slides across the fixed lite span",
        )
        ctx.expect_within(
            moving_carriage,
            base_frame,
            axes="xy",
            margin=0.0,
            name="opened sash still remains within the outer frame",
        )
        open_pos = ctx.part_world_position(moving_carriage)

    ctx.check(
        "sash opens left along the guide channels",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] < rest_pos[0] - 0.25
        and abs(open_pos[1] - rest_pos[1]) < 1e-6
        and abs(open_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
