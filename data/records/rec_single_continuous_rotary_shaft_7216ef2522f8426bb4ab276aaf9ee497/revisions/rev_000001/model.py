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


BASE_LENGTH = 0.260
BASE_WIDTH = 0.120
BASE_THICKNESS = 0.018

SUPPORT_CENTERS_X = (-0.072, 0.072)
SUPPORT_LENGTH = 0.032
SHAFT_AXIS_Z = 0.074
SUPPORT_TOP_Z = 0.090
SUPPORT_HEIGHT = SUPPORT_TOP_Z - BASE_THICKNESS

KEEPER_LENGTH = SUPPORT_CENTERS_X[1] - SUPPORT_CENTERS_X[0] + SUPPORT_LENGTH
KEEPER_WIDTH = 0.060
KEEPER_THICKNESS = 0.020

SHAFT_RADIUS = 0.012
SHAFT_LENGTH = 0.248
COLLAR_RADIUS = 0.018
COLLAR_THICKNESS = 0.008
COLLAR_AXIAL_CLEARANCE = 0.0

SIDE_CHEEK_THICKNESS = 0.012
SIDE_CHEEK_CENTER_Y = 0.022
LAND_WIDTH = 0.032
LAND_THICKNESS = 0.006


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_rotary_shaft_fixture")

    model.material("frame_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("keeper_gray", rgba=(0.52, 0.54, 0.57, 1.0))
    model.material("shaft_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_gray",
        name="base_plate",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        for support_index, support_x in enumerate(SUPPORT_CENTERS_X, start=1):
            lower_frame.visual(
                Box((SUPPORT_LENGTH, SIDE_CHEEK_THICKNESS, SUPPORT_HEIGHT)),
                origin=Origin(
                    xyz=(
                        support_x,
                        side_sign * SIDE_CHEEK_CENTER_Y,
                        BASE_THICKNESS + SUPPORT_HEIGHT / 2.0,
                    )
                ),
                material="frame_gray",
                name=f"{side_name}_cheek_{support_index}",
            )
    for support_index, support_x in enumerate(SUPPORT_CENTERS_X, start=1):
        lower_frame.visual(
            Box((SUPPORT_LENGTH, LAND_WIDTH, LAND_THICKNESS)),
            origin=Origin(
                xyz=(
                    support_x,
                    0.0,
                    SHAFT_AXIS_Z - SHAFT_RADIUS - LAND_THICKNESS / 2.0,
                )
            ),
            material="frame_gray",
            name=f"lower_land_{support_index}",
        )

    top_keeper = model.part("top_keeper")
    top_keeper.visual(
        Box((KEEPER_LENGTH, KEEPER_WIDTH, KEEPER_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, KEEPER_THICKNESS / 2.0)),
        material="keeper_gray",
        name="bridge_bar",
    )
    for support_index, support_x in enumerate(SUPPORT_CENTERS_X, start=1):
        top_keeper.visual(
            Box((SUPPORT_LENGTH, LAND_WIDTH, LAND_THICKNESS)),
            origin=Origin(xyz=(support_x, 0.0, -LAND_THICKNESS / 2.0)),
            material="keeper_gray",
            name=f"upper_land_{support_index}",
        )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="shaft_steel",
        name="shaft_body",
    )
    support_outer_face = SUPPORT_CENTERS_X[1] + SUPPORT_LENGTH / 2.0
    collar_center = support_outer_face + COLLAR_AXIAL_CLEARANCE + COLLAR_THICKNESS / 2.0
    for side_sign, collar_name in ((-1.0, "left_collar"), (1.0, "right_collar")):
        shaft.visual(
            Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
            origin=Origin(
                xyz=(side_sign * collar_center, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="shaft_steel",
            name=collar_name,
        )

    model.articulation(
        "frame_to_keeper",
        ArticulationType.FIXED,
        parent=lower_frame,
        child=top_keeper,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_Z)),
    )

    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=10.0),
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

    lower_frame = object_model.get_part("lower_frame")
    top_keeper = object_model.get_part("top_keeper")
    shaft = object_model.get_part("shaft")
    shaft_joint = object_model.get_articulation("frame_to_shaft")
    keeper_joint = object_model.get_articulation("frame_to_keeper")

    ctx.check(
        "keeper is rigidly mounted to the frame",
        keeper_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint_type={keeper_joint.articulation_type}",
    )
    ctx.check(
        "shaft uses a continuous x-axis revolute support",
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(shaft_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={shaft_joint.articulation_type}, axis={shaft_joint.axis}",
    )

    ctx.expect_origin_distance(
        lower_frame,
        top_keeper,
        axes="xy",
        max_dist=1e-6,
        name="keeper stays centered over the frame",
    )
    ctx.expect_gap(
        top_keeper,
        lower_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem="bridge_bar",
        name="keeper seats on the lower frame shoulders",
    )

    rest_pos = ctx.part_world_position(shaft)
    with ctx.pose({shaft_joint: pi / 2.0}):
        spun_pos = ctx.part_world_position(shaft)

    ctx.check(
        "shaft spins without orbiting away from its bearings",
        rest_pos is not None
        and spun_pos is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_pos, spun_pos)),
        details=f"rest_pos={rest_pos}, spun_pos={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
