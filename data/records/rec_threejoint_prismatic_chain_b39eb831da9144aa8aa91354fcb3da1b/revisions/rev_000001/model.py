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


GUIDE_LENGTH = 1.20
GUIDE_WIDTH = 0.22
LARGE_LENGTH = 0.90
LARGE_WIDTH = 0.176
MEDIUM_LENGTH = 0.66
MEDIUM_WIDTH = 0.140
SMALL_LENGTH = 0.42
SMALL_WIDTH = 0.108

GUIDE_TO_LARGE_TRAVEL = 0.58
LARGE_TO_MEDIUM_TRAVEL = 0.44
MEDIUM_TO_SMALL_TRAVEL = 0.30

GUIDE_BASE_T = 0.016
GUIDE_WALL_T = 0.012
GUIDE_SIDE_H = 0.130
GUIDE_RUNNER_W = 0.020
GUIDE_RUNNER_H = 0.010
GUIDE_RUNNER_Y = 0.060
GUIDE_FOOT_H = 0.020

LARGE_SHOE_W = 0.018
LARGE_SHOE_H = 0.008
LARGE_BOTTOM_T = 0.014
LARGE_WALL_T = 0.010
LARGE_SIDE_H = 0.092
LARGE_INNER_RUNNER_W = 0.018
LARGE_INNER_RUNNER_H = 0.008
LARGE_INNER_RUNNER_Y = 0.048
LARGE_CAP_T = 0.010

MEDIUM_SHOE_W = 0.016
MEDIUM_SHOE_H = 0.006
MEDIUM_BOTTOM_T = 0.012
MEDIUM_WALL_T = 0.008
MEDIUM_SIDE_H = 0.067
MEDIUM_INNER_RUNNER_W = 0.014
MEDIUM_INNER_RUNNER_H = 0.007
MEDIUM_INNER_RUNNER_Y = 0.036
MEDIUM_CAP_T = 0.008

SMALL_SHOE_W = 0.012
SMALL_SHOE_H = 0.005
SMALL_BOTTOM_T = 0.010
SMALL_WALL_T = 0.006
SMALL_SIDE_H = 0.038
SMALL_TOOL_T = 0.008
SMALL_FRONT_PLATE_T = 0.014

GUIDE_RUNNER_BOTTOM = GUIDE_BASE_T - 0.002
GUIDE_RUNNER_TOP = GUIDE_RUNNER_BOTTOM + GUIDE_RUNNER_H

LARGE_BOTTOM_BOTTOM = LARGE_SHOE_H - 0.002
LARGE_BOTTOM_TOP = LARGE_BOTTOM_BOTTOM + LARGE_BOTTOM_T
LARGE_SIDE_BOTTOM = LARGE_BOTTOM_TOP - 0.002
LARGE_INNER_RUNNER_BOTTOM = LARGE_BOTTOM_TOP - 0.002
LARGE_INNER_RUNNER_TOP = LARGE_INNER_RUNNER_BOTTOM + LARGE_INNER_RUNNER_H
LARGE_TOP = LARGE_SIDE_BOTTOM + LARGE_SIDE_H

MEDIUM_BOTTOM_BOTTOM = MEDIUM_SHOE_H - 0.002
MEDIUM_BOTTOM_TOP = MEDIUM_BOTTOM_BOTTOM + MEDIUM_BOTTOM_T
MEDIUM_SIDE_BOTTOM = MEDIUM_BOTTOM_TOP - 0.002
MEDIUM_INNER_RUNNER_BOTTOM = MEDIUM_BOTTOM_TOP - 0.002
MEDIUM_INNER_RUNNER_TOP = MEDIUM_INNER_RUNNER_BOTTOM + MEDIUM_INNER_RUNNER_H
MEDIUM_TOP = MEDIUM_SIDE_BOTTOM + MEDIUM_SIDE_H

SMALL_BOTTOM_BOTTOM = SMALL_SHOE_H - 0.002
SMALL_BOTTOM_TOP = SMALL_BOTTOM_BOTTOM + SMALL_BOTTOM_T
SMALL_SIDE_BOTTOM = SMALL_BOTTOM_TOP - 0.002
SMALL_TOP = 0.055

GUIDE_TO_LARGE_Z = GUIDE_RUNNER_TOP
LARGE_TO_MEDIUM_Z = LARGE_INNER_RUNNER_TOP
MEDIUM_TO_SMALL_Z = MEDIUM_INNER_RUNNER_TOP


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _build_guide_frame(model: ArticulatedObject):
    part = model.part("guide_frame")
    material = "frame_graphite"

    _add_box_visual(
        part,
        size=(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_BASE_T),
        center=(GUIDE_LENGTH / 2.0, 0.0, GUIDE_BASE_T / 2.0),
        material=material,
        name="guide_base",
    )
    side_center_z = GUIDE_BASE_T - 0.002 + GUIDE_SIDE_H / 2.0
    _add_box_visual(
        part,
        size=(GUIDE_LENGTH, GUIDE_WALL_T, GUIDE_SIDE_H),
        center=(GUIDE_LENGTH / 2.0, GUIDE_WIDTH / 2.0 - GUIDE_WALL_T / 2.0, side_center_z),
        material=material,
        name="guide_left_wall",
    )
    _add_box_visual(
        part,
        size=(GUIDE_LENGTH, GUIDE_WALL_T, GUIDE_SIDE_H),
        center=(GUIDE_LENGTH / 2.0, -GUIDE_WIDTH / 2.0 + GUIDE_WALL_T / 2.0, side_center_z),
        material=material,
        name="guide_right_wall",
    )
    runner_center_z = GUIDE_RUNNER_BOTTOM + GUIDE_RUNNER_H / 2.0
    _add_box_visual(
        part,
        size=(GUIDE_LENGTH, GUIDE_RUNNER_W, GUIDE_RUNNER_H),
        center=(GUIDE_LENGTH / 2.0, GUIDE_RUNNER_Y, runner_center_z),
        material=material,
        name="guide_runner_left",
    )
    _add_box_visual(
        part,
        size=(GUIDE_LENGTH, GUIDE_RUNNER_W, GUIDE_RUNNER_H),
        center=(GUIDE_LENGTH / 2.0, -GUIDE_RUNNER_Y, runner_center_z),
        material=material,
        name="guide_runner_right",
    )
    foot_length = 0.19
    foot_width = GUIDE_WIDTH * 0.66
    foot_center_z = -GUIDE_FOOT_H / 2.0
    _add_box_visual(
        part,
        size=(foot_length, foot_width, GUIDE_FOOT_H),
        center=(0.24, 0.0, foot_center_z),
        material=material,
        name="guide_rear_foot",
    )
    _add_box_visual(
        part,
        size=(foot_length, foot_width, GUIDE_FOOT_H),
        center=(GUIDE_LENGTH - 0.24, 0.0, foot_center_z),
        material=material,
        name="guide_front_foot",
    )
    part.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, 0.164)),
        mass=13.0,
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, 0.062)),
    )
    return part


def _build_large_carriage(model: ArticulatedObject):
    part = model.part("large_carriage")
    material = "large_stage_steel"

    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_SHOE_W, LARGE_SHOE_H),
        center=(LARGE_LENGTH / 2.0, GUIDE_RUNNER_Y, LARGE_SHOE_H / 2.0),
        material=material,
        name="large_left_shoe",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_SHOE_W, LARGE_SHOE_H),
        center=(LARGE_LENGTH / 2.0, -GUIDE_RUNNER_Y, LARGE_SHOE_H / 2.0),
        material=material,
        name="large_right_shoe",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_WIDTH, LARGE_BOTTOM_T),
        center=(LARGE_LENGTH / 2.0, 0.0, LARGE_BOTTOM_BOTTOM + LARGE_BOTTOM_T / 2.0),
        material=material,
        name="large_bottom_plate",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_WALL_T, LARGE_SIDE_H),
        center=(LARGE_LENGTH / 2.0, LARGE_WIDTH / 2.0 - LARGE_WALL_T / 2.0, LARGE_SIDE_BOTTOM + LARGE_SIDE_H / 2.0),
        material=material,
        name="large_left_wall",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_WALL_T, LARGE_SIDE_H),
        center=(LARGE_LENGTH / 2.0, -LARGE_WIDTH / 2.0 + LARGE_WALL_T / 2.0, LARGE_SIDE_BOTTOM + LARGE_SIDE_H / 2.0),
        material=material,
        name="large_right_wall",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_INNER_RUNNER_W, LARGE_INNER_RUNNER_H),
        center=(LARGE_LENGTH / 2.0, LARGE_INNER_RUNNER_Y, LARGE_INNER_RUNNER_BOTTOM + LARGE_INNER_RUNNER_H / 2.0),
        material=material,
        name="large_inner_runner_left",
    )
    _add_box_visual(
        part,
        size=(LARGE_LENGTH, LARGE_INNER_RUNNER_W, LARGE_INNER_RUNNER_H),
        center=(LARGE_LENGTH / 2.0, -LARGE_INNER_RUNNER_Y, LARGE_INNER_RUNNER_BOTTOM + LARGE_INNER_RUNNER_H / 2.0),
        material=material,
        name="large_inner_runner_right",
    )
    _add_box_visual(
        part,
        size=(0.12, LARGE_WIDTH, LARGE_CAP_T),
        center=(LARGE_LENGTH - 0.08, 0.0, LARGE_TOP - LARGE_CAP_T / 2.0),
        material=material,
        name="large_front_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((LARGE_LENGTH, LARGE_WIDTH, LARGE_TOP)),
        mass=6.0,
        origin=Origin(xyz=(LARGE_LENGTH / 2.0, 0.0, LARGE_TOP / 2.0)),
    )
    return part


def _build_medium_carriage(model: ArticulatedObject):
    part = model.part("medium_carriage")
    material = "medium_stage_steel"

    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_SHOE_W, MEDIUM_SHOE_H),
        center=(MEDIUM_LENGTH / 2.0, LARGE_INNER_RUNNER_Y, MEDIUM_SHOE_H / 2.0),
        material=material,
        name="medium_left_shoe",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_SHOE_W, MEDIUM_SHOE_H),
        center=(MEDIUM_LENGTH / 2.0, -LARGE_INNER_RUNNER_Y, MEDIUM_SHOE_H / 2.0),
        material=material,
        name="medium_right_shoe",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_WIDTH, MEDIUM_BOTTOM_T),
        center=(MEDIUM_LENGTH / 2.0, 0.0, MEDIUM_BOTTOM_BOTTOM + MEDIUM_BOTTOM_T / 2.0),
        material=material,
        name="medium_bottom_plate",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_WALL_T, MEDIUM_SIDE_H),
        center=(MEDIUM_LENGTH / 2.0, MEDIUM_WIDTH / 2.0 - MEDIUM_WALL_T / 2.0, MEDIUM_SIDE_BOTTOM + MEDIUM_SIDE_H / 2.0),
        material=material,
        name="medium_left_wall",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_WALL_T, MEDIUM_SIDE_H),
        center=(MEDIUM_LENGTH / 2.0, -MEDIUM_WIDTH / 2.0 + MEDIUM_WALL_T / 2.0, MEDIUM_SIDE_BOTTOM + MEDIUM_SIDE_H / 2.0),
        material=material,
        name="medium_right_wall",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_INNER_RUNNER_W, MEDIUM_INNER_RUNNER_H),
        center=(MEDIUM_LENGTH / 2.0, MEDIUM_INNER_RUNNER_Y, MEDIUM_INNER_RUNNER_BOTTOM + MEDIUM_INNER_RUNNER_H / 2.0),
        material=material,
        name="medium_inner_runner_left",
    )
    _add_box_visual(
        part,
        size=(MEDIUM_LENGTH, MEDIUM_INNER_RUNNER_W, MEDIUM_INNER_RUNNER_H),
        center=(MEDIUM_LENGTH / 2.0, -MEDIUM_INNER_RUNNER_Y, MEDIUM_INNER_RUNNER_BOTTOM + MEDIUM_INNER_RUNNER_H / 2.0),
        material=material,
        name="medium_inner_runner_right",
    )
    _add_box_visual(
        part,
        size=(0.10, MEDIUM_WIDTH, MEDIUM_CAP_T),
        center=(MEDIUM_LENGTH - 0.07, 0.0, MEDIUM_TOP - MEDIUM_CAP_T / 2.0),
        material=material,
        name="medium_front_cap",
    )
    part.inertial = Inertial.from_geometry(
        Box((MEDIUM_LENGTH, MEDIUM_WIDTH, MEDIUM_TOP)),
        mass=3.8,
        origin=Origin(xyz=(MEDIUM_LENGTH / 2.0, 0.0, MEDIUM_TOP / 2.0)),
    )
    return part


def _build_small_output_carriage(model: ArticulatedObject):
    part = model.part("small_output_carriage")
    material = "small_stage_steel"

    _add_box_visual(
        part,
        size=(SMALL_LENGTH, SMALL_SHOE_W, SMALL_SHOE_H),
        center=(SMALL_LENGTH / 2.0, MEDIUM_INNER_RUNNER_Y, SMALL_SHOE_H / 2.0),
        material=material,
        name="small_left_shoe",
    )
    _add_box_visual(
        part,
        size=(SMALL_LENGTH, SMALL_SHOE_W, SMALL_SHOE_H),
        center=(SMALL_LENGTH / 2.0, -MEDIUM_INNER_RUNNER_Y, SMALL_SHOE_H / 2.0),
        material=material,
        name="small_right_shoe",
    )
    _add_box_visual(
        part,
        size=(SMALL_LENGTH, SMALL_WIDTH, SMALL_BOTTOM_T),
        center=(SMALL_LENGTH / 2.0, 0.0, SMALL_BOTTOM_BOTTOM + SMALL_BOTTOM_T / 2.0),
        material=material,
        name="small_bottom_plate",
    )
    _add_box_visual(
        part,
        size=(SMALL_LENGTH, SMALL_WALL_T, SMALL_SIDE_H),
        center=(SMALL_LENGTH / 2.0, SMALL_WIDTH / 2.0 - SMALL_WALL_T / 2.0, SMALL_SIDE_BOTTOM + SMALL_SIDE_H / 2.0),
        material=material,
        name="small_left_wall",
    )
    _add_box_visual(
        part,
        size=(SMALL_LENGTH, SMALL_WALL_T, SMALL_SIDE_H),
        center=(SMALL_LENGTH / 2.0, -SMALL_WIDTH / 2.0 + SMALL_WALL_T / 2.0, SMALL_SIDE_BOTTOM + SMALL_SIDE_H / 2.0),
        material=material,
        name="small_right_wall",
    )
    _add_box_visual(
        part,
        size=(SMALL_FRONT_PLATE_T, SMALL_WIDTH * 0.72, 0.030),
        center=(SMALL_LENGTH - SMALL_FRONT_PLATE_T / 2.0, 0.0, 0.028),
        material=material,
        name="small_front_plate",
    )
    _add_box_visual(
        part,
        size=(0.110, SMALL_WIDTH, SMALL_TOOL_T),
        center=(SMALL_LENGTH - 0.065, 0.0, SMALL_TOP - SMALL_TOOL_T / 2.0),
        material=material,
        name="small_tool_plate",
    )
    part.inertial = Inertial.from_geometry(
        Box((SMALL_LENGTH, SMALL_WIDTH, SMALL_TOP)),
        mass=2.3,
        origin=Origin(xyz=(SMALL_LENGTH / 2.0, 0.0, SMALL_TOP / 2.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_three_carriage_linear_chain")

    model.material("frame_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("large_stage_steel", rgba=(0.43, 0.47, 0.50, 1.0))
    model.material("medium_stage_steel", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("small_stage_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    guide_frame = _build_guide_frame(model)
    large_carriage = _build_large_carriage(model)
    medium_carriage = _build_medium_carriage(model)
    small_output_carriage = _build_small_output_carriage(model)

    model.articulation(
        "guide_to_large",
        ArticulationType.PRISMATIC,
        parent=guide_frame,
        child=large_carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TO_LARGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=GUIDE_TO_LARGE_TRAVEL,
            effort=250.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "large_to_medium",
        ArticulationType.PRISMATIC,
        parent=large_carriage,
        child=medium_carriage,
        origin=Origin(xyz=(0.0, 0.0, LARGE_TO_MEDIUM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LARGE_TO_MEDIUM_TRAVEL,
            effort=180.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "medium_to_small",
        ArticulationType.PRISMATIC,
        parent=medium_carriage,
        child=small_output_carriage,
        origin=Origin(xyz=(0.0, 0.0, MEDIUM_TO_SMALL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MEDIUM_TO_SMALL_TRAVEL,
            effort=120.0,
            velocity=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    guide_frame = object_model.get_part("guide_frame")
    large_carriage = object_model.get_part("large_carriage")
    medium_carriage = object_model.get_part("medium_carriage")
    small_output_carriage = object_model.get_part("small_output_carriage")

    guide_to_large = object_model.get_articulation("guide_to_large")
    large_to_medium = object_model.get_articulation("large_to_medium")
    medium_to_small = object_model.get_articulation("medium_to_small")

    ctx.check(
        "all linear stages and joints are present",
        all(
            item is not None
            for item in (
                guide_frame,
                large_carriage,
                medium_carriage,
                small_output_carriage,
                guide_to_large,
                large_to_medium,
                medium_to_small,
            )
        ),
    )
    ctx.check(
        "all three joints share the +X slide axis",
        guide_to_large.axis == (1.0, 0.0, 0.0)
        and large_to_medium.axis == (1.0, 0.0, 0.0)
        and medium_to_small.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes={(guide_to_large.axis, large_to_medium.axis, medium_to_small.axis)}"
        ),
    )
    ctx.expect_contact(
        large_carriage,
        guide_frame,
        elem_a="large_left_shoe",
        elem_b="guide_runner_left",
        name="large carriage rides on the guide runner",
    )
    ctx.expect_contact(
        medium_carriage,
        large_carriage,
        elem_a="medium_left_shoe",
        elem_b="large_inner_runner_left",
        name="medium carriage rides on the large carriage runner",
    )
    ctx.expect_contact(
        small_output_carriage,
        medium_carriage,
        elem_a="small_left_shoe",
        elem_b="medium_inner_runner_left",
        name="small carriage rides on the medium carriage runner",
    )

    ctx.expect_within(
        large_carriage,
        guide_frame,
        axes="yz",
        margin=0.001,
        name="large carriage stays centered in the guide frame at rest",
    )
    ctx.expect_overlap(
        large_carriage,
        guide_frame,
        axes="x",
        min_overlap=0.88,
        name="large carriage remains deeply inserted at rest",
    )
    ctx.expect_within(
        medium_carriage,
        large_carriage,
        axes="yz",
        margin=0.001,
        name="medium carriage stays centered in the large carriage at rest",
    )
    ctx.expect_overlap(
        medium_carriage,
        large_carriage,
        axes="x",
        min_overlap=0.64,
        name="medium carriage remains deeply inserted at rest",
    )
    ctx.expect_within(
        small_output_carriage,
        medium_carriage,
        axes="yz",
        margin=0.001,
        name="small carriage stays centered in the medium carriage at rest",
    )
    ctx.expect_overlap(
        small_output_carriage,
        medium_carriage,
        axes="x",
        min_overlap=0.40,
        name="small carriage remains deeply inserted at rest",
    )

    large_rest = ctx.part_world_position(large_carriage)
    with ctx.pose({guide_to_large: GUIDE_TO_LARGE_TRAVEL}):
        ctx.expect_within(
            large_carriage,
            guide_frame,
            axes="yz",
            margin=0.001,
            name="large carriage stays centered at maximum extension",
        )
        ctx.expect_overlap(
            large_carriage,
            guide_frame,
            axes="x",
            min_overlap=0.60,
            name="large carriage retains insertion at maximum extension",
        )
        large_extended = ctx.part_world_position(large_carriage)

    medium_rest = ctx.part_world_position(medium_carriage)
    with ctx.pose({large_to_medium: LARGE_TO_MEDIUM_TRAVEL}):
        ctx.expect_within(
            medium_carriage,
            large_carriage,
            axes="yz",
            margin=0.001,
            name="medium carriage stays centered at maximum extension",
        )
        ctx.expect_overlap(
            medium_carriage,
            large_carriage,
            axes="x",
            min_overlap=0.45,
            name="medium carriage retains insertion at maximum extension",
        )
        medium_extended = ctx.part_world_position(medium_carriage)

    output_rest = ctx.part_world_position(small_output_carriage)
    with ctx.pose(
        {
            guide_to_large: GUIDE_TO_LARGE_TRAVEL,
            large_to_medium: LARGE_TO_MEDIUM_TRAVEL,
            medium_to_small: MEDIUM_TO_SMALL_TRAVEL,
        }
    ):
        ctx.expect_within(
            small_output_carriage,
            medium_carriage,
            axes="yz",
            margin=0.001,
            name="small carriage stays centered at full chain extension",
        )
        ctx.expect_overlap(
            small_output_carriage,
            medium_carriage,
            axes="x",
            min_overlap=0.35,
            name="small carriage retains insertion at full chain extension",
        )
        output_extended = ctx.part_world_position(small_output_carriage)

    ctx.check(
        "large carriage extends forward along +X",
        large_rest is not None
        and large_extended is not None
        and large_extended[0] > large_rest[0] + 0.40,
        details=f"rest={large_rest}, extended={large_extended}",
    )
    ctx.check(
        "medium carriage extends forward along +X",
        medium_rest is not None
        and medium_extended is not None
        and medium_extended[0] > medium_rest[0] + 0.30,
        details=f"rest={medium_rest}, extended={medium_extended}",
    )
    ctx.check(
        "small output carriage accumulates all three travel stages",
        output_rest is not None
        and output_extended is not None
        and output_extended[0]
        > output_rest[0]
        + GUIDE_TO_LARGE_TRAVEL
        + LARGE_TO_MEDIUM_TRAVEL
        + MEDIUM_TO_SMALL_TRAVEL
        - 0.02,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
