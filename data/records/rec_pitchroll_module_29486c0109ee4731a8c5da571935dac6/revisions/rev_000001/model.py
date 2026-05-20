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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LEN = 0.090
BASE_WID = 0.082
BASE_THK = 0.020
BASE_CENTER_X = -0.012
BASE_CENTER_Z = -0.079

INNER_YOKE_GAP = 0.090
ARM_THK = 0.018
OUTER_YOKE_WID = INNER_YOKE_GAP + 2.0 * ARM_THK
ARM_LEN = 0.070
ARM_HEIGHT = 0.088
ARM_CENTER_X = 0.006
ARM_CENTER_Z = -0.017

LOWER_BRIDGE_LEN = 0.060
LOWER_BRIDGE_HT = 0.030
LOWER_BRIDGE_CENTER_X = -0.004
LOWER_BRIDGE_CENTER_Z = -0.054

SIDE_BOSS_RADIUS = 0.018
SIDE_BOSS_LEN = 0.012

HOUSING_RADIUS = 0.031
HOUSING_MAIN_LEN = 0.072
FRONT_NOSE_LEN = 0.010
FRONT_NOSE_RADIUS = 0.019
REAR_CAP_LEN = 0.010
REAR_CAP_RADIUS = 0.024
TRUNNION_RADIUS = 0.011
TRUNNION_TOTAL_LEN = INNER_YOKE_GAP

ROLL_JOINT_X = HOUSING_MAIN_LEN / 2.0 + FRONT_NOSE_LEN

FLANGE_HUB_LEN = 0.014
FLANGE_HUB_RADIUS = 0.020
FLANGE_PLATE_THK = 0.008
FLANGE_PLATE_RADIUS = 0.028
FLANGE_PILOT_LEN = 0.006
FLANGE_PILOT_RADIUS = 0.010
INDEX_PIN_RADIUS = 0.004
INDEX_PIN_LEN = 0.008
INDEX_PIN_Z = 0.017


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pitch_roll_head")

    model.material("painted_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("anodized_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Box((BASE_LEN, BASE_WID, BASE_THK)),
        origin=Origin(xyz=(BASE_CENTER_X, 0.0, BASE_CENTER_Z)),
        material="painted_steel",
        name="base_plate",
    )
    pitch_yoke.visual(
        Box((LOWER_BRIDGE_LEN, INNER_YOKE_GAP, LOWER_BRIDGE_HT)),
        origin=Origin(xyz=(LOWER_BRIDGE_CENTER_X, 0.0, LOWER_BRIDGE_CENTER_Z)),
        material="painted_steel",
        name="lower_bridge",
    )
    pitch_yoke.visual(
        Box((ARM_LEN, ARM_THK, ARM_HEIGHT)),
        origin=Origin(
            xyz=(ARM_CENTER_X, INNER_YOKE_GAP / 2.0 + ARM_THK / 2.0, ARM_CENTER_Z)
        ),
        material="painted_steel",
        name="left_arm",
    )
    pitch_yoke.visual(
        Box((ARM_LEN, ARM_THK, ARM_HEIGHT)),
        origin=Origin(
            xyz=(ARM_CENTER_X, -INNER_YOKE_GAP / 2.0 - ARM_THK / 2.0, ARM_CENTER_Z)
        ),
        material="painted_steel",
        name="right_arm",
    )
    pitch_yoke.visual(
        Cylinder(radius=SIDE_BOSS_RADIUS, length=SIDE_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, OUTER_YOKE_WID / 2.0 + SIDE_BOSS_LEN / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="painted_steel",
        name="left_boss",
    )
    pitch_yoke.visual(
        Cylinder(radius=SIDE_BOSS_RADIUS, length=SIDE_BOSS_LEN),
        origin=Origin(
            xyz=(0.0, -OUTER_YOKE_WID / 2.0 - SIDE_BOSS_LEN / 2.0, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="painted_steel",
        name="right_boss",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((BASE_LEN, OUTER_YOKE_WID, 0.108)),
        mass=2.4,
        origin=Origin(xyz=(-0.008, 0.0, -0.047)),
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=HOUSING_RADIUS, length=HOUSING_MAIN_LEN),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="anodized_gray",
        name="cartridge_body",
    )
    roll_cartridge.visual(
        Cylinder(radius=REAR_CAP_RADIUS, length=REAR_CAP_LEN),
        origin=Origin(xyz=(-HOUSING_MAIN_LEN / 2.0 - REAR_CAP_LEN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="anodized_gray",
        name="rear_cap",
    )
    roll_cartridge.visual(
        Cylinder(radius=FRONT_NOSE_RADIUS, length=FRONT_NOSE_LEN),
        origin=Origin(xyz=(HOUSING_MAIN_LEN / 2.0 + FRONT_NOSE_LEN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="anodized_gray",
        name="front_nose",
    )
    roll_cartridge.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_TOTAL_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="anodized_gray",
        name="trunnion_shaft",
    )
    roll_cartridge.inertial = Inertial.from_geometry(
        Box((HOUSING_MAIN_LEN + FRONT_NOSE_LEN + REAR_CAP_LEN, TRUNNION_TOTAL_LEN, 0.066)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    output_flange = model.part("output_flange")
    output_flange.visual(
        Cylinder(radius=FLANGE_HUB_RADIUS, length=FLANGE_HUB_LEN),
        origin=Origin(xyz=(FLANGE_HUB_LEN / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_aluminum",
        name="flange_hub",
    )
    output_flange.visual(
        Cylinder(radius=FLANGE_PLATE_RADIUS, length=FLANGE_PLATE_THK),
        origin=Origin(
            xyz=(FLANGE_HUB_LEN + FLANGE_PLATE_THK / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_aluminum",
        name="flange_plate",
    )
    output_flange.visual(
        Cylinder(radius=FLANGE_PILOT_RADIUS, length=FLANGE_PILOT_LEN),
        origin=Origin(
            xyz=(
                FLANGE_HUB_LEN + FLANGE_PLATE_THK + FLANGE_PILOT_LEN / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_aluminum",
        name="pilot_boss",
    )
    output_flange.visual(
        Cylinder(radius=INDEX_PIN_RADIUS, length=INDEX_PIN_LEN),
        origin=Origin(
            xyz=(
                FLANGE_HUB_LEN + FLANGE_PLATE_THK + INDEX_PIN_LEN / 2.0,
                0.0,
                INDEX_PIN_Z,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="machined_aluminum",
        name="index_pin",
    )
    output_flange.inertial = Inertial.from_geometry(
        Box(
            (
                FLANGE_HUB_LEN + FLANGE_PLATE_THK + FLANGE_PILOT_LEN,
                2.0 * FLANGE_PLATE_RADIUS,
                2.0 * FLANGE_PLATE_RADIUS,
            )
        ),
        mass=0.32,
        origin=Origin(
            xyz=((FLANGE_HUB_LEN + FLANGE_PLATE_THK + FLANGE_PILOT_LEN) / 2.0, 0.0, 0.0)
        ),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_cartridge,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.2,
            lower=-0.22,
            upper=1.15,
        ),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=roll_cartridge,
        child=output_flange,
        origin=Origin(xyz=(ROLL_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.5,
            lower=-pi,
            upper=pi,
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

    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_cartridge = object_model.get_part("roll_cartridge")
    output_flange = object_model.get_part("output_flange")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.expect_origin_distance(
        pitch_yoke,
        roll_cartridge,
        axes="yz",
        max_dist=0.001,
        name="roll cartridge is centered on the pitch trunnion axis",
    )
    ctx.expect_origin_gap(
        output_flange,
        roll_cartridge,
        axis="x",
        min_gap=0.043,
        max_gap=0.049,
        name="output flange sits at the front of the roll cartridge",
    )
    ctx.expect_contact(
        output_flange,
        roll_cartridge,
        elem_a="flange_hub",
        elem_b="front_nose",
        name="flange hub seats against the cartridge nose",
    )

    flange_rest = ctx.part_world_position(output_flange)
    with ctx.pose({pitch_joint: 0.70}):
        flange_pitched = ctx.part_world_position(output_flange)
    ctx.check(
        "positive pitch raises the output flange",
        flange_rest is not None
        and flange_pitched is not None
        and flange_pitched[2] > flange_rest[2] + 0.025,
        details=f"rest={flange_rest}, pitched={flange_pitched}",
    )

    pin_rest = aabb_center(
        ctx.part_element_world_aabb(output_flange, elem="index_pin")
    )
    with ctx.pose({roll_joint: pi / 2.0}):
        pin_rolled = aabb_center(
            ctx.part_element_world_aabb(output_flange, elem="index_pin")
        )
    ctx.check(
        "positive roll moves the flange index pin around the flange axis",
        pin_rest is not None
        and pin_rolled is not None
        and pin_rolled[1] < pin_rest[1] - 0.012
        and abs(pin_rolled[2]) < abs(pin_rest[2]) * 0.35,
        details=f"rest={pin_rest}, rolled={pin_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
