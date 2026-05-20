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


BASE_FOOT_RADIUS = 0.140
BASE_FOOT_HEIGHT = 0.018
BASE_PEDESTAL_RADIUS = 0.085
BASE_PEDESTAL_HEIGHT = 0.070
BASE_CAP_RADIUS = 0.098
BASE_CAP_HEIGHT = 0.012
BASE_TOTAL_HEIGHT = BASE_FOOT_HEIGHT + BASE_PEDESTAL_HEIGHT + BASE_CAP_HEIGHT

TURNTABLE_RADIUS = 0.094
TURNTABLE_HEIGHT = 0.016
FORK_PLATE_X = 0.092
FORK_PLATE_Y = 0.150
FORK_PLATE_Z = 0.012
FORK_ARM_X = 0.092
FORK_ARM_Y = 0.016
FORK_ARM_Z = 0.104
FORK_ARM_CENTER_X = -0.010
FORK_ARM_CENTER_Y = 0.063
FORK_ARM_CENTER_Z = TURNTABLE_HEIGHT + FORK_PLATE_Z + (FORK_ARM_Z * 0.5)
FORK_BRIDGE_X = 0.020
FORK_BRIDGE_Y = 0.142
FORK_BRIDGE_Z = 0.060
FORK_BRIDGE_CENTER_X = -0.046
FORK_BRIDGE_CENTER_Z = 0.086
FORK_PITCH_AXIS_Z = 0.086

CRADLE_TRUNNION_RADIUS = 0.0125
CRADLE_TRUNNION_LENGTH = 0.110
CRADLE_HUB_RADIUS = 0.020
CRADLE_HUB_LENGTH = 0.046
CRADLE_MAST_X = 0.034
CRADLE_MAST_Y = 0.044
CRADLE_MAST_Z = 0.064
CRADLE_PAD_X = 0.052
CRADLE_PAD_Y = 0.056
CRADLE_PAD_Z = 0.008
TOP_PLATE_LENGTH = 0.100
TOP_PLATE_WIDTH = 0.072
TOP_PLATE_THICKNESS = 0.008
TOP_PLATE_UNDERSIDE_Z = CRADLE_MAST_Z + CRADLE_PAD_Z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_module")

    base_material = model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    fork_material = model.material("fork_metal", rgba=(0.66, 0.68, 0.72, 1.0))
    plate_material = model.material("plate_gray", rgba=(0.34, 0.37, 0.41, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT * 0.5)),
        material=base_material,
        name="foot",
    )
    base.visual(
        Cylinder(radius=BASE_PEDESTAL_RADIUS, length=BASE_PEDESTAL_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + (BASE_PEDESTAL_HEIGHT * 0.5))
        ),
        material=base_material,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=BASE_CAP_RADIUS, length=BASE_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FOOT_HEIGHT + BASE_PEDESTAL_HEIGHT + (BASE_CAP_HEIGHT * 0.5),
            )
        ),
        material=base_material,
        name="cap",
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_HEIGHT * 0.5)),
        material=fork_material,
        name="turntable",
    )
    fork.visual(
        Box((FORK_PLATE_X, FORK_PLATE_Y, FORK_PLATE_Z)),
        origin=Origin(
            xyz=(FORK_ARM_CENTER_X, 0.0, TURNTABLE_HEIGHT + (FORK_PLATE_Z * 0.5))
        ),
        material=fork_material,
        name="fork_plate",
    )
    fork.visual(
        Box((FORK_ARM_X, FORK_ARM_Y, FORK_ARM_Z)),
        origin=Origin(
            xyz=(FORK_ARM_CENTER_X, FORK_ARM_CENTER_Y, FORK_ARM_CENTER_Z)
        ),
        material=fork_material,
        name="left_arm",
    )
    fork.visual(
        Box((FORK_ARM_X, FORK_ARM_Y, FORK_ARM_Z)),
        origin=Origin(
            xyz=(FORK_ARM_CENTER_X, -FORK_ARM_CENTER_Y, FORK_ARM_CENTER_Z)
        ),
        material=fork_material,
        name="right_arm",
    )
    fork.visual(
        Box((FORK_BRIDGE_X, FORK_BRIDGE_Y, FORK_BRIDGE_Z)),
        origin=Origin(
            xyz=(FORK_BRIDGE_CENTER_X, 0.0, FORK_BRIDGE_CENTER_Z)
        ),
        material=fork_material,
        name="rear_bridge",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=CRADLE_TRUNNION_RADIUS, length=CRADLE_TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=fork_material,
        name="trunnion_shaft",
    )
    cradle.visual(
        Cylinder(radius=CRADLE_HUB_RADIUS, length=CRADLE_HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=fork_material,
        name="hub",
    )
    cradle.visual(
        Box((CRADLE_MAST_X, CRADLE_MAST_Y, CRADLE_MAST_Z)),
        origin=Origin(xyz=(0.0, 0.0, CRADLE_MAST_Z * 0.5)),
        material=fork_material,
        name="mast",
    )
    cradle.visual(
        Box((CRADLE_PAD_X, CRADLE_PAD_Y, CRADLE_PAD_Z)),
        origin=Origin(
            xyz=(0.0, 0.0, CRADLE_MAST_Z + (CRADLE_PAD_Z * 0.5))
        ),
        material=fork_material,
        name="plate_pad",
    )
    cradle.visual(
        Box((TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, TOP_PLATE_UNDERSIDE_Z + (TOP_PLATE_THICKNESS * 0.5))
        ),
        material=plate_material,
        name="top_plate",
    )

    model.articulation(
        "base_to_fork_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "fork_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, FORK_PITCH_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.95,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fork = object_model.get_part("fork")
    cradle = object_model.get_part("cradle")
    yaw = object_model.get_articulation("base_to_fork_yaw")
    pitch = object_model.get_articulation("fork_to_cradle_pitch")
    top_plate = cradle.get_visual("top_plate")

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

    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        f"expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        (
            abs(pitch.axis[0]) < 1e-6
            and abs(abs(pitch.axis[1]) - 1.0) < 1e-6
            and abs(pitch.axis[2]) < 1e-6
        ),
        f"expected lateral horizontal pitch axis, got {pitch.axis}",
    )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_gap(
            fork,
            base,
            axis="z",
            max_penetration=1e-6,
            max_gap=0.0005,
            name="fork turntable seats on base",
        )
        ctx.expect_overlap(
            fork,
            base,
            axes="xy",
            min_overlap=0.120,
            name="fork turntable stays above base bearing footprint",
        )
        ctx.expect_contact(
            cradle,
            fork,
            name="cradle trunnions contact fork bearings",
        )
        ctx.expect_within(
            cradle,
            fork,
            axes="y",
            margin=0.0,
            name="cradle stays within fork width",
        )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        top_plate_rest = ctx.part_element_world_aabb(cradle, elem=top_plate)

    with ctx.pose({yaw: 0.0, pitch: 0.75}):
        top_plate_pitched = ctx.part_element_world_aabb(cradle, elem=top_plate)

    with ctx.pose({yaw: pi * 0.5, pitch: 0.75}):
        top_plate_yawed = ctx.part_element_world_aabb(cradle, elem=top_plate)

    def _center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    top_plate_rest_center = _center(top_plate_rest)
    top_plate_pitched_center = _center(top_plate_pitched)
    top_plate_yawed_center = _center(top_plate_yawed)

    ctx.check(
        "yaw rotates the pitched cradle around the vertical axis",
        (
            top_plate_pitched_center[0] > top_plate_rest_center[0] + 0.030
            and abs(top_plate_yawed_center[0]) < 0.010
            and top_plate_yawed_center[1] > top_plate_rest_center[1] + 0.030
            and abs(top_plate_pitched_center[0] - top_plate_yawed_center[1]) < 0.012
        ),
        (
            "pitched top plate did not rotate from +x travel into matching +y travel: "
            f"rest={top_plate_rest_center}, pitched={top_plate_pitched_center}, "
            f"yawed={top_plate_yawed_center}"
        ),
    )
    ctx.check(
        "pitch drives the top plate forward",
        (
            top_plate_pitched_center[0] > top_plate_rest_center[0] + 0.030
            and abs(top_plate_pitched_center[1] - top_plate_rest_center[1]) < 0.010
        ),
        (
            "top plate did not move forward with positive pitch: "
            f"rest={top_plate_rest_center}, pitched={top_plate_pitched_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
