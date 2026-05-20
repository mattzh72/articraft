from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.10
RAIL_HEIGHT = 0.08
PAD_LENGTH = 0.10
PAD_WIDTH = 0.15
PAD_HEIGHT = 0.02
PAD_OFFSET = 0.30
RAIL_CENTER_Z = PAD_HEIGHT + (RAIL_HEIGHT / 2.0)

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.18
CARRIAGE_BRIDGE_THICKNESS = 0.06
CARRIAGE_BRIDGE_CENTER_Z = 0.015
CARRIAGE_LEG_WIDTH = 0.034
CARRIAGE_LEG_CENTER_Y = 0.073
CARRIAGE_LEG_HEIGHT = 0.12
CARRIAGE_LEG_CENTER_Z = -0.045
CARRIAGE_GUIDE_WIDTH = 0.09
CARRIAGE_GUIDE_HEIGHT = 0.05
CARRIAGE_GUIDE_CENTER_Z = -0.025
CARRIAGE_GUIDE_LENGTH = 0.14
NOSE_BOSS_RADIUS = 0.048
NOSE_BOSS_DEPTH = 0.014
SLIDE_HALF_TRAVEL = 0.13
SPINDLE_AXIS_HEIGHT = 0.15

SPINDLE_FLANGE_RADIUS = 0.043
SPINDLE_FLANGE_LENGTH = 0.014
SPINDLE_COLLAR_RADIUS = 0.028
SPINDLE_COLLAR_LENGTH = 0.018
SPINDLE_BODY_RADIUS = 0.020
SPINDLE_BODY_LENGTH = 0.050


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_rail_rotary_nose")

    model.material("rail_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("carriage_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("spindle_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_CENTER_Z)),
        material="rail_gray",
        name="rail_shell",
    )
    rail.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT + 0.002)),
        origin=Origin(xyz=(-PAD_OFFSET, 0.0, (PAD_HEIGHT + 0.002) / 2.0)),
        material="rail_gray",
        name="left_foot",
    )
    rail.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT + 0.002)),
        origin=Origin(xyz=(PAD_OFFSET, 0.0, (PAD_HEIGHT + 0.002) / 2.0)),
        material="rail_gray",
        name="right_foot",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BRIDGE_CENTER_Z)),
        material="carriage_silver",
        name="bridge",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_LEG_WIDTH, CARRIAGE_LEG_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CARRIAGE_LEG_CENTER_Y, CARRIAGE_LEG_CENTER_Z),
        ),
        material="carriage_silver",
        name="right_leg",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_LEG_WIDTH, CARRIAGE_LEG_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CARRIAGE_LEG_CENTER_Y, CARRIAGE_LEG_CENTER_Z),
        ),
        material="carriage_silver",
        name="left_leg",
    )
    carriage.visual(
        Box((CARRIAGE_GUIDE_LENGTH, CARRIAGE_GUIDE_WIDTH, CARRIAGE_GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_GUIDE_CENTER_Z)),
        material="carriage_silver",
        name="guide_shoe",
    )
    carriage.visual(
        Cylinder(radius=NOSE_BOSS_RADIUS, length=NOSE_BOSS_DEPTH),
        origin=Origin(
            xyz=(CARRIAGE_LENGTH / 2.0 + (NOSE_BOSS_DEPTH / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="carriage_silver",
        name="nose_boss",
    )

    spindle = model.part("nose_spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_FLANGE_LENGTH),
        origin=Origin(
            xyz=(SPINDLE_FLANGE_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="spindle_steel",
        name="spindle_flange",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_COLLAR_RADIUS, length=SPINDLE_COLLAR_LENGTH),
        origin=Origin(
            xyz=(SPINDLE_FLANGE_LENGTH + (SPINDLE_COLLAR_LENGTH / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="spindle_steel",
        name="spindle_collar",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH),
        origin=Origin(
            xyz=(
                SPINDLE_FLANGE_LENGTH
                + SPINDLE_COLLAR_LENGTH
                + (SPINDLE_BODY_LENGTH / 2.0),
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="spindle_steel",
        name="spindle_body",
    )
    spindle.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(SPINDLE_FLANGE_LENGTH / 2.0, SPINDLE_FLANGE_RADIUS * 0.82, 0.0)),
        material="spindle_steel",
        name="index_lug",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=-SLIDE_HALF_TRAVEL,
            upper=SLIDE_HALF_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_nose_spindle",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(CARRIAGE_LENGTH / 2.0 + NOSE_BOSS_DEPTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=10.0),
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

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("nose_spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    spin = object_model.get_articulation("carriage_to_nose_spindle")

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_spindle_pos = ctx.part_world_position(spindle)

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="guide_shoe",
        elem_b="rail_shell",
        name="carriage guide shoe bears on the rail top at rest",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="x",
        elem_a="guide_shoe",
        elem_b="rail_shell",
        min_overlap=0.12,
        name="carriage guide shoe remains engaged on the rail at rest",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="y",
        positive_elem="right_leg",
        negative_elem="rail_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="right carriage cheek clears the rail side at rest",
    )
    ctx.expect_gap(
        rail,
        carriage,
        axis="y",
        positive_elem="rail_shell",
        negative_elem="left_leg",
        min_gap=0.0,
        max_gap=0.008,
        name="left carriage cheek clears the rail side at rest",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a="spindle_flange",
        elem_b="nose_boss",
        name="nose spindle flange seats on the carriage nose boss",
    )
    ctx.expect_within(
        spindle,
        carriage,
        axes="yz",
        inner_elem="spindle_flange",
        outer_elem="nose_boss",
        margin=0.0,
        name="spindle flange stays centered within the nose boss",
    )

    extended_carriage_pos = None
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            rail,
            elem_a="guide_shoe",
            elem_b="rail_shell",
            name="carriage guide shoe remains supported at maximum extension",
        )
        ctx.expect_overlap(
            carriage,
            rail,
            axes="x",
            elem_a="guide_shoe",
            elem_b="rail_shell",
            min_overlap=0.12,
            name="carriage guide shoe stays engaged at maximum extension",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="y",
            positive_elem="right_leg",
            negative_elem="rail_shell",
            min_gap=0.0,
            max_gap=0.008,
            name="right carriage cheek still clears the rail side at full extension",
        )
        ctx.expect_gap(
            rail,
            carriage,
            axis="y",
            positive_elem="rail_shell",
            negative_elem="left_leg",
            min_gap=0.0,
            max_gap=0.008,
            name="left carriage cheek still clears the rail side at full extension",
        )
        ctx.expect_contact(
            spindle,
            carriage,
            elem_a="spindle_flange",
            elem_b="nose_boss",
            name="nose spindle stays seated while the carriage extends",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage extends in the +x rail direction",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.10,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rotated_spindle_pos = None
    with ctx.pose({spin: pi / 2.0}):
        ctx.expect_contact(
            spindle,
            carriage,
            elem_a="spindle_flange",
            elem_b="nose_boss",
            name="nose spindle remains captured while rotating",
        )
        ctx.expect_within(
            spindle,
            carriage,
            axes="yz",
            inner_elem="spindle_flange",
            outer_elem="nose_boss",
            margin=0.0,
            name="spindle flange stays centered in the nose boss while rotating",
        )
        rotated_spindle_pos = ctx.part_world_position(spindle)

    ctx.check(
        "spindle rotates about its own axis without translating",
        rest_spindle_pos is not None
        and rotated_spindle_pos is not None
        and max(
            abs(rotated_spindle_pos[0] - rest_spindle_pos[0]),
            abs(rotated_spindle_pos[1] - rest_spindle_pos[1]),
            abs(rotated_spindle_pos[2] - rest_spindle_pos[2]),
        )
        <= 1e-6,
        details=f"rest={rest_spindle_pos}, rotated={rotated_spindle_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
