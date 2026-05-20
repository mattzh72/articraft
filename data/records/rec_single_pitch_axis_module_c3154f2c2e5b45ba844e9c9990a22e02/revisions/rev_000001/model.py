from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
from math import pi


BASE_W = 0.170
BASE_D = 0.130
BASE_T = 0.012

CORE_W = 0.100
CORE_D = 0.074
CORE_H = 0.052
CORE_Y = -0.010

INNER_SPAN = 0.120
ARM_T = 0.016
ARM_D = 0.066
ARM_SHOULDER_H = 0.038
ARM_POST_H = 0.030
ARM_POST_OVERLAP = 0.004
ARM_CROWN_R = 0.014
ARM_Y = 0.003

AXIS_Z = 0.082

PLATE_W = 0.100
PLATE_D = 0.076
PLATE_T = 0.012
PLATE_CENTER_Y = 0.010
TRUNNION_R = 0.008
TRUNNION_LEN = 0.010

PITCH_LOWER = -0.55
PITCH_UPPER = 0.75
def _arm_center_x(sign: float) -> float:
    return sign * (INNER_SPAN / 2.0 + ARM_T / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sector_arm_pitch_module")

    body_finish = model.material("body_finish", rgba=(0.23, 0.25, 0.28, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.69, 0.72, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=body_finish,
        name="base_plate",
    )
    body.visual(
        Box((CORE_W, CORE_D, CORE_H)),
        origin=Origin(xyz=(0.0, CORE_Y, BASE_T + CORE_H / 2.0)),
        material=body_finish,
        name="body_shell",
    )
    arm_bottom_z = BASE_T + ARM_SHOULDER_H / 2.0 - 0.002
    arm_post_bottom_z = BASE_T + ARM_SHOULDER_H - ARM_POST_OVERLAP
    arm_post_z = arm_post_bottom_z + ARM_POST_H / 2.0
    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        arm_x = _arm_center_x(sign)
        body.visual(
            Box((ARM_T, ARM_D, ARM_SHOULDER_H)),
            origin=Origin(xyz=(arm_x, ARM_Y, arm_bottom_z)),
            material=body_finish,
            name=f"{side_name}_arm_shoulder",
        )
        body.visual(
            Box((ARM_T, ARM_D, ARM_POST_H)),
            origin=Origin(xyz=(arm_x, ARM_Y, arm_post_z)),
            material=body_finish,
            name=f"{side_name}_arm_post",
        )
        body.visual(
            Cylinder(radius=ARM_CROWN_R, length=ARM_T),
            origin=Origin(xyz=(arm_x, ARM_Y, AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=body_finish,
            name=f"{side_name}_arm_crown",
        )
    body.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, AXIS_Z + 0.018)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (AXIS_Z + 0.018) / 2.0)),
    )

    plate = model.part("tilting_plate")
    plate.visual(
        Box((PLATE_W, PLATE_D, PLATE_T)),
        origin=Origin(xyz=(0.0, PLATE_CENTER_Y, 0.0)),
        material=plate_finish,
        name="plate_shell",
    )
    plate.visual(
        Cylinder(radius=TRUNNION_R, length=TRUNNION_LEN),
        origin=Origin(
            xyz=(-(PLATE_W / 2.0 + TRUNNION_LEN / 2.0), 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=plate_finish,
        name="left_trunnion",
    )
    plate.visual(
        Cylinder(radius=TRUNNION_R, length=TRUNNION_LEN),
        origin=Origin(
            xyz=(PLATE_W / 2.0 + TRUNNION_LEN / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=plate_finish,
        name="right_trunnion",
    )
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_W + 2.0 * TRUNNION_LEN, PLATE_D, 0.020)),
        mass=0.45,
        origin=Origin(xyz=(0.0, PLATE_CENTER_Y, 0.0)),
    )

    model.articulation(
        "body_to_plate_pitch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
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

    body = object_model.get_part("body")
    plate = object_model.get_part("tilting_plate")
    pitch = object_model.get_articulation("body_to_plate_pitch")

    limits = pitch.motion_limits
    ctx.check("body part exists", body is not None, details="Expected grounded body part.")
    ctx.check(
        "tilting plate exists",
        plate is not None,
        details="Expected compact tilting plate part.",
    )
    ctx.check(
        "pitch joint uses horizontal trunnion axis",
        pitch.axis == (1.0, 0.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "pitch limits span realistic up and down tilt",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.upper <= 0.9,
        details=f"limits={limits}",
    )

    ctx.expect_origin_distance(
        plate,
        body,
        axes="x",
        max_dist=0.001,
        name="plate is centered between side arms",
    )
    ctx.expect_origin_gap(
        plate,
        body,
        axis="z",
        min_gap=0.080,
        max_gap=0.084,
        name="trunnion axis is raised above grounded body",
    )
    ctx.expect_contact(
        plate,
        body,
        name="plate is physically supported on the side-arm trunnions",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(plate, elem="plate_shell")
    with ctx.pose({pitch: PITCH_UPPER}):
        up_plate_aabb = ctx.part_element_world_aabb(plate, elem="plate_shell")
    with ctx.pose({pitch: PITCH_LOWER}):
        down_plate_aabb = ctx.part_element_world_aabb(plate, elem="plate_shell")

    ctx.check(
        "positive pitch lifts the plate silhouette",
        rest_plate_aabb is not None
        and up_plate_aabb is not None
        and up_plate_aabb[1][2] > rest_plate_aabb[1][2] + 0.012,
        details=f"rest={rest_plate_aabb}, up={up_plate_aabb}",
    )
    ctx.check(
        "negative pitch lowers but clears the base deck",
        rest_plate_aabb is not None
        and down_plate_aabb is not None
        and down_plate_aabb[0][2] < rest_plate_aabb[0][2] - 0.010
        and down_plate_aabb[0][2] > BASE_T + 0.030,
        details=f"rest={rest_plate_aabb}, down={down_plate_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
