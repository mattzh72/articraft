from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        candidates = [
            os.environ.get("TMPDIR"),
            "/tmp",
            "/var/tmp",
            "/",
        ]
        for candidate in candidates:
            if candidate and os.path.isdir(candidate):
                os.chdir(candidate)
                return _ORIG_GETCWD()
        raise


os.getcwd = _safe_getcwd
if "__spec__" in globals() and getattr(__spec__, "origin", None):
    __file__ = __spec__.origin

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

CABINET_WIDTH = 0.46
CABINET_DEPTH = 0.62
CABINET_HEIGHT = 1.46
SIDE_THICKNESS = 0.015
TOP_THICKNESS = 0.015
BOTTOM_THICKNESS = 0.015
BACK_THICKNESS = 0.010
DIVIDER_THICKNESS = 0.012
FACE_FRAME_DEPTH = 0.030

OPENING_WIDTH = CABINET_WIDTH - (2.0 * SIDE_THICKNESS)
OPENING_HEIGHT = (
    CABINET_HEIGHT
    - TOP_THICKNESS
    - BOTTOM_THICKNESS
    - (3.0 * DIVIDER_THICKNESS)
) / 4.0

DRAWER_FRONT_WIDTH = OPENING_WIDTH - 0.008
DRAWER_FRONT_HEIGHT = OPENING_HEIGHT - 0.006
DRAWER_FRONT_THICKNESS = 0.016

DRAWER_BOX_WIDTH = 0.394
DRAWER_BOX_HEIGHT = 0.314
DRAWER_BOX_DEPTH = 0.540
DRAWER_SIDE_THICKNESS = 0.010
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_BACK_THICKNESS = 0.010

RAIL_WIDTH = 0.012
RAIL_HEIGHT = 0.020
RAIL_DEPTH = 0.480
BODY_RAIL_X = (OPENING_WIDTH * 0.5) - (RAIL_WIDTH * 0.5)
BODY_RAIL_Y = -0.020

SLIDE_WIDTH = 0.008
SLIDE_HEIGHT = 0.014
SLIDE_DEPTH = 0.440
SLIDE_X = BODY_RAIL_X - (RAIL_WIDTH * 0.5) - (SLIDE_WIDTH * 0.5)
SLIDE_Y = -0.228

HANDLE_WIDTH = 0.238
HANDLE_HEIGHT = 0.056
HANDLE_DEPTH = 0.011
HANDLE_CENTER_Z = 0.058
HANDLE_BACKPLATE_Y = -0.004
HANDLE_BACKPLATE_WIDTH = HANDLE_WIDTH - 0.030
HANDLE_BACKPLATE_HEIGHT = HANDLE_HEIGHT - 0.018
HANDLE_RECESS_DEPTH = 0.006
HANDLE_RECESS_SIDE_WIDTH = (HANDLE_WIDTH - HANDLE_BACKPLATE_WIDTH) * 0.5
HANDLE_RECESS_CAP_HEIGHT = (HANDLE_HEIGHT - HANDLE_BACKPLATE_HEIGHT) * 0.5
HANDLE_RECESS_CENTER_Y = -(DRAWER_FRONT_THICKNESS * 0.5) + (HANDLE_RECESS_DEPTH * 0.5)
DRAWER_CLOSED_FRONT_CENTER_Y = (CABINET_DEPTH * 0.5) - (DRAWER_FRONT_THICKNESS * 0.5) + 0.002
DRAWER_TRAVEL = 0.220

FRONT_HALF_HEIGHT = DRAWER_FRONT_HEIGHT * 0.5
HANDLE_OPEN_TOP = HANDLE_CENTER_Z + (HANDLE_HEIGHT * 0.5)
HANDLE_OPEN_BOTTOM = HANDLE_CENTER_Z - (HANDLE_HEIGHT * 0.5)
FRONT_TOP_HEIGHT = FRONT_HALF_HEIGHT - HANDLE_OPEN_TOP
FRONT_BOTTOM_HEIGHT = HANDLE_OPEN_BOTTOM + FRONT_HALF_HEIGHT
FRONT_SIDE_WIDTH = (DRAWER_FRONT_WIDTH - HANDLE_WIDTH) * 0.5
PLINTH_HEIGHT = 0.010


def _drawer_center_z(index: int) -> float:
    return (
        BOTTOM_THICKNESS
        + (OPENING_HEIGHT * 0.5)
        + index * (OPENING_HEIGHT + DIVIDER_THICKNESS)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="filing_cabinet")

    cabinet_steel = model.material("cabinet_steel", rgba=(0.64, 0.67, 0.71, 1.0))
    drawer_steel = model.material("drawer_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    drawer_interior = model.material("drawer_interior", rgba=(0.57, 0.60, 0.64, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.42, 0.45, 0.49, 1.0))
    shadow = model.material("shadow", rgba=(0.18, 0.19, 0.20, 1.0))
    plinth = model.material("plinth", rgba=(0.20, 0.21, 0.23, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH * 0.5) + (SIDE_THICKNESS * 0.5),
                0.0,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=cabinet_steel,
        name="left_side",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH * 0.5) - (SIDE_THICKNESS * 0.5),
                0.0,
                CABINET_HEIGHT * 0.5,
            )
        ),
        material=cabinet_steel,
        name="right_side",
    )
    cabinet.visual(
        Box((OPENING_WIDTH, CABINET_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - (TOP_THICKNESS * 0.5))),
        material=cabinet_steel,
        name="top_panel",
    )
    cabinet.visual(
        Box((OPENING_WIDTH, CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=plinth,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((OPENING_WIDTH, BACK_THICKNESS, CABINET_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH * 0.5) + (BACK_THICKNESS * 0.5),
                (CABINET_HEIGHT * 0.5),
            )
        ),
        material=cabinet_steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((OPENING_WIDTH - 0.040, CABINET_DEPTH - 0.060, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.010, PLINTH_HEIGHT * 0.5)),
        material=plinth,
        name="base_plinth",
    )

    for divider_index in range(3):
        divider_z = (
            BOTTOM_THICKNESS
            + OPENING_HEIGHT
            + (DIVIDER_THICKNESS * 0.5)
            + divider_index * (OPENING_HEIGHT + DIVIDER_THICKNESS)
        )
        cabinet.visual(
            Box((OPENING_WIDTH, FACE_FRAME_DEPTH, DIVIDER_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    (CABINET_DEPTH * 0.5) - (FACE_FRAME_DEPTH * 0.5),
                    divider_z,
                )
            ),
            material=cabinet_steel,
            name=f"divider_{divider_index}",
        )

    for drawer_index in range(4):
        rail_z = _drawer_center_z(drawer_index)
        cabinet.visual(
            Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(-BODY_RAIL_X, BODY_RAIL_Y, rail_z)),
            material=rail_steel,
            name=f"drawer_{drawer_index}_left_rail",
        )
        cabinet.visual(
            Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
            origin=Origin(xyz=(BODY_RAIL_X, BODY_RAIL_Y, rail_z)),
            material=rail_steel,
            name=f"drawer_{drawer_index}_right_rail",
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=56.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    for drawer_index in range(4):
        drawer = model.part(f"drawer_{drawer_index}")
        drawer.visual(
            Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, FRONT_TOP_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    HANDLE_OPEN_TOP + (FRONT_TOP_HEIGHT * 0.5),
                )
            ),
            material=drawer_steel,
            name="front_top",
        )
        drawer.visual(
            Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, FRONT_BOTTOM_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    -FRONT_HALF_HEIGHT + (FRONT_BOTTOM_HEIGHT * 0.5),
                )
            ),
            material=drawer_steel,
            name="front_bottom",
        )
        drawer.visual(
            Box((FRONT_SIDE_WIDTH, DRAWER_FRONT_THICKNESS, HANDLE_HEIGHT)),
            origin=Origin(
                xyz=(
                    -((HANDLE_WIDTH * 0.5) + (FRONT_SIDE_WIDTH * 0.5)),
                    0.0,
                    HANDLE_CENTER_Z,
                )
            ),
            material=drawer_steel,
            name="front_left",
        )
        drawer.visual(
            Box((FRONT_SIDE_WIDTH, DRAWER_FRONT_THICKNESS, HANDLE_HEIGHT)),
            origin=Origin(
                xyz=(
                    (HANDLE_WIDTH * 0.5) + (FRONT_SIDE_WIDTH * 0.5),
                    0.0,
                    HANDLE_CENTER_Z,
                )
            ),
            material=drawer_steel,
            name="front_right",
        )
        drawer.visual(
            Box((HANDLE_BACKPLATE_WIDTH, 0.002, HANDLE_BACKPLATE_HEIGHT)),
            origin=Origin(xyz=(0.0, HANDLE_BACKPLATE_Y, HANDLE_CENTER_Z)),
            material=shadow,
            name="pull_backplate",
        )
        drawer.visual(
            Box((HANDLE_RECESS_SIDE_WIDTH, HANDLE_RECESS_DEPTH, HANDLE_HEIGHT)),
            origin=Origin(
                xyz=(
                    -((HANDLE_BACKPLATE_WIDTH * 0.5) + (HANDLE_RECESS_SIDE_WIDTH * 0.5)),
                    HANDLE_RECESS_CENTER_Y,
                    HANDLE_CENTER_Z,
                )
            ),
            material=shadow,
            name="pull_left_wall",
        )
        drawer.visual(
            Box((HANDLE_RECESS_SIDE_WIDTH, HANDLE_RECESS_DEPTH, HANDLE_HEIGHT)),
            origin=Origin(
                xyz=(
                    (HANDLE_BACKPLATE_WIDTH * 0.5) + (HANDLE_RECESS_SIDE_WIDTH * 0.5),
                    HANDLE_RECESS_CENTER_Y,
                    HANDLE_CENTER_Z,
                )
            ),
            material=shadow,
            name="pull_right_wall",
        )
        drawer.visual(
            Box((HANDLE_BACKPLATE_WIDTH, HANDLE_RECESS_DEPTH, HANDLE_RECESS_CAP_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    HANDLE_RECESS_CENTER_Y,
                    HANDLE_CENTER_Z
                    + (HANDLE_BACKPLATE_HEIGHT * 0.5)
                    + (HANDLE_RECESS_CAP_HEIGHT * 0.5),
                )
            ),
            material=shadow,
            name="pull_top_wall",
        )
        drawer.visual(
            Box((HANDLE_BACKPLATE_WIDTH, HANDLE_RECESS_DEPTH, HANDLE_RECESS_CAP_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    HANDLE_RECESS_CENTER_Y,
                    HANDLE_CENTER_Z
                    - (HANDLE_BACKPLATE_HEIGHT * 0.5)
                    - (HANDLE_RECESS_CAP_HEIGHT * 0.5),
                )
            ),
            material=shadow,
            name="pull_bottom_wall",
        )
        drawer.visual(
            Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    -(DRAWER_FRONT_THICKNESS * 0.5) - (DRAWER_BOX_DEPTH * 0.5),
                    -(DRAWER_BOX_HEIGHT * 0.5) + (DRAWER_BOTTOM_THICKNESS * 0.5),
                )
            ),
            material=drawer_interior,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    -(DRAWER_BOX_WIDTH * 0.5) + (DRAWER_SIDE_THICKNESS * 0.5),
                    -(DRAWER_FRONT_THICKNESS * 0.5) - (DRAWER_BOX_DEPTH * 0.5),
                    0.0,
                )
            ),
            material=drawer_interior,
            name="left_wall",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    (DRAWER_BOX_WIDTH * 0.5) - (DRAWER_SIDE_THICKNESS * 0.5),
                    -(DRAWER_FRONT_THICKNESS * 0.5) - (DRAWER_BOX_DEPTH * 0.5),
                    0.0,
                )
            ),
            material=drawer_interior,
            name="right_wall",
        )
        drawer.visual(
            Box(
                (
                    DRAWER_BOX_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS),
                    DRAWER_BACK_THICKNESS,
                    DRAWER_BOX_HEIGHT,
                )
            ),
            origin=Origin(
                xyz=(
                    0.0,
                    -(DRAWER_FRONT_THICKNESS * 0.5)
                    - DRAWER_BOX_DEPTH
                    + (DRAWER_BACK_THICKNESS * 0.5),
                    0.0,
                )
            ),
            material=drawer_interior,
            name="back_wall",
        )
        drawer.visual(
            Box((SLIDE_WIDTH, SLIDE_DEPTH, SLIDE_HEIGHT)),
            origin=Origin(xyz=(-SLIDE_X, SLIDE_Y, 0.0)),
            material=rail_steel,
            name="left_slide",
        )
        drawer.visual(
            Box((SLIDE_WIDTH, SLIDE_DEPTH, SLIDE_HEIGHT)),
            origin=Origin(xyz=(SLIDE_X, SLIDE_Y, 0.0)),
            material=rail_steel,
            name="right_slide",
        )
        drawer.inertial = Inertial.from_geometry(
            Box(
                (
                    DRAWER_FRONT_WIDTH,
                    DRAWER_BOX_DEPTH + DRAWER_FRONT_THICKNESS,
                    DRAWER_FRONT_HEIGHT,
                )
            ),
            mass=7.5,
            origin=Origin(xyz=(0.0, -0.278, 0.0)),
        )

        model.articulation(
            f"cabinet_to_drawer_{drawer_index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(
                xyz=(0.0, DRAWER_CLOSED_FRONT_CENTER_Y, _drawer_center_z(drawer_index))
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    left_side = cabinet.get_visual("left_side")
    right_side = cabinet.get_visual("right_side")
    top_panel = cabinet.get_visual("top_panel")
    bottom_panel = cabinet.get_visual("bottom_panel")
    back_panel = cabinet.get_visual("back_panel")

    drawers = [object_model.get_part(f"drawer_{index}") for index in range(4)]
    drawer_joints = [
        object_model.get_articulation(f"cabinet_to_drawer_{index}") for index in range(4)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(name="rest_pose_no_floating")
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose(name="rest_pose_no_overlaps")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    for lower_drawer, upper_drawer in zip(drawers[:-1], drawers[1:]):
        ctx.expect_gap(
            upper_drawer,
            lower_drawer,
            axis="z",
            min_gap=0.016,
            max_gap=0.020,
            name=f"{upper_drawer.name}_stack_gap",
        )

    for drawer_index, drawer in enumerate(drawers):
        front_top = drawer.get_visual("front_top")
        front_bottom = drawer.get_visual("front_bottom")
        front_left = drawer.get_visual("front_left")
        front_right = drawer.get_visual("front_right")
        pull_backplate = drawer.get_visual("pull_backplate")
        pull_left_wall = drawer.get_visual("pull_left_wall")
        pull_right_wall = drawer.get_visual("pull_right_wall")
        pull_top_wall = drawer.get_visual("pull_top_wall")
        pull_bottom_wall = drawer.get_visual("pull_bottom_wall")
        left_slide = drawer.get_visual("left_slide")
        right_slide = drawer.get_visual("right_slide")
        left_rail = cabinet.get_visual(f"drawer_{drawer_index}_left_rail")
        right_rail = cabinet.get_visual(f"drawer_{drawer_index}_right_rail")
        joint = drawer_joints[drawer_index]
        limits = joint.motion_limits

        axis_ok = all(abs(value - target) < 1e-9 for value, target in zip(joint.axis, (0.0, 1.0, 0.0)))
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) < 1e-9
            and abs(limits.upper - DRAWER_TRAVEL) < 1e-9
        )
        ctx.check(
            f"drawer_{drawer_index}_uses_prismatic_guides",
            joint.articulation_type == ArticulationType.PRISMATIC and axis_ok and limits_ok,
            details=(
                f"joint={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )
        ctx.check(
            f"drawer_{drawer_index}_pull_is_recessed",
            pull_backplate.origin.xyz[1] < front_top.origin.xyz[1] - 0.001,
            details=(
                f"pull_backplate_y={pull_backplate.origin.xyz[1]} "
                f"front_plane_y={front_top.origin.xyz[1]}"
            ),
        )
        ctx.check(
            f"drawer_{drawer_index}_pull_is_centered_in_front_opening",
            (
                front_left.origin.xyz[0] < pull_backplate.origin.xyz[0] < front_right.origin.xyz[0]
                and front_bottom.origin.xyz[2] < pull_backplate.origin.xyz[2] < front_top.origin.xyz[2]
            ),
            details=(
                f"front_left_x={front_left.origin.xyz[0]}, "
                f"pull_x={pull_backplate.origin.xyz[0]}, "
                f"front_right_x={front_right.origin.xyz[0]}, "
                f"front_bottom_z={front_bottom.origin.xyz[2]}, "
                f"pull_z={pull_backplate.origin.xyz[2]}, "
                f"front_top_z={front_top.origin.xyz[2]}"
            ),
        )
        ctx.check(
            f"drawer_{drawer_index}_pull_recess_is_boxed_in",
            (
                pull_left_wall.origin.xyz[0] < pull_backplate.origin.xyz[0] < pull_right_wall.origin.xyz[0]
                and pull_bottom_wall.origin.xyz[2] < pull_backplate.origin.xyz[2] < pull_top_wall.origin.xyz[2]
                and pull_left_wall.origin.xyz[1] < front_left.origin.xyz[1]
            ),
            details=(
                f"left_wall={pull_left_wall.origin.xyz}, right_wall={pull_right_wall.origin.xyz}, "
                f"top_wall={pull_top_wall.origin.xyz}, bottom_wall={pull_bottom_wall.origin.xyz}, "
                f"backplate={pull_backplate.origin.xyz}"
            ),
        )

        ctx.expect_within(
            drawer,
            cabinet,
            axes="x",
            margin=0.0,
            name=f"drawer_{drawer_index}_stays_within_cabinet_width",
        )
        ctx.expect_within(
            drawer,
            drawer,
            axes="xz",
            inner_elem=pull_backplate,
            name=f"drawer_{drawer_index}_pull_is_nested_in_front",
        )
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="y",
            positive_elem=drawer.get_visual("back_wall"),
            negative_elem=back_panel,
            min_gap=0.045,
            max_gap=0.070,
            name=f"drawer_{drawer_index}_closed_back_clearance",
        )
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="z",
            positive_elem=front_bottom,
            negative_elem=bottom_panel,
            max_penetration=0.0,
            name=f"drawer_{drawer_index}_front_clears_bottom_panel",
        )
        ctx.expect_overlap(
            cabinet,
            drawer,
            axes="yz",
            elem_a=left_rail,
            elem_b=left_slide,
            min_overlap=0.012,
            name=f"drawer_{drawer_index}_left_guide_overlap",
        )
        ctx.expect_overlap(
            cabinet,
            drawer,
            axes="yz",
            elem_a=right_rail,
            elem_b=right_slide,
            min_overlap=0.012,
            name=f"drawer_{drawer_index}_right_guide_overlap",
        )

        if limits is None or limits.lower is None or limits.upper is None:
            continue

        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"drawer_{drawer_index}_closed_pose_no_overlaps"
            )
            ctx.fail_if_isolated_parts(name=f"drawer_{drawer_index}_closed_pose_no_floating")
            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=left_rail,
                elem_b=left_slide,
                name=f"drawer_{drawer_index}_left_slide_contacts_rail_closed",
            )
            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=right_rail,
                elem_b=right_slide,
                name=f"drawer_{drawer_index}_right_slide_contacts_rail_closed",
            )
            ctx.expect_origin_gap(
                drawer,
                cabinet,
                axis="y",
                min_gap=0.303,
                max_gap=0.305,
                name=f"drawer_{drawer_index}_closed_position",
            )

        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"drawer_{drawer_index}_open_pose_no_overlaps"
            )
            ctx.fail_if_isolated_parts(name=f"drawer_{drawer_index}_open_pose_no_floating")
            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=left_rail,
                elem_b=left_slide,
                name=f"drawer_{drawer_index}_left_slide_contacts_rail_open",
            )
            ctx.expect_contact(
                cabinet,
                drawer,
                elem_a=right_rail,
                elem_b=right_slide,
                name=f"drawer_{drawer_index}_right_slide_contacts_rail_open",
            )
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                positive_elem=front_top,
                negative_elem=top_panel,
                min_gap=0.204,
                name=f"drawer_{drawer_index}_extends_forward_when_open",
            )
            ctx.expect_origin_gap(
                drawer,
                cabinet,
                axis="y",
                min_gap=0.523,
                max_gap=0.525,
                name=f"drawer_{drawer_index}_open_position",
            )
            ctx.expect_overlap(
                cabinet,
                drawer,
                axes="yz",
                elem_a=left_rail,
                elem_b=left_slide,
                min_overlap=0.012,
                name=f"drawer_{drawer_index}_left_guide_overlap_open",
            )
            ctx.expect_overlap(
                cabinet,
                drawer,
                axes="yz",
                elem_a=right_rail,
                elem_b=right_slide,
                min_overlap=0.012,
                name=f"drawer_{drawer_index}_right_guide_overlap_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
