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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_red = model.material("body_red", rgba=(0.72, 0.10, 0.12, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.77, 0.80, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.72, 0.80, 0.90, 0.45))
    accent_blue = model.material("accent_blue", rgba=(0.12, 0.48, 0.82, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.055, length=0.190),
        origin=Origin(xyz=(-0.125, 0.0, 0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    motor_body.visual(
        Cylinder(radius=0.046, length=0.128),
        origin=Origin(xyz=(-0.250, 0.0, 0.094), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_red,
        name="motor_pod",
    )
    motor_body.visual(
        Box((0.060, 0.074, 0.030)),
        origin=Origin(xyz=(-0.042, 0.0, 0.040)),
        material=body_red,
        name="hinge_bridge",
    )
    motor_body.visual(
        Box((0.028, 0.016, 0.026)),
        origin=Origin(xyz=(-0.014, -0.023, 0.013)),
        material=body_red,
        name="left_fold_cheek",
    )
    motor_body.visual(
        Box((0.028, 0.016, 0.026)),
        origin=Origin(xyz=(-0.014, 0.023, 0.013)),
        material=body_red,
        name="right_fold_cheek",
    )
    motor_body.visual(
        Box((0.180, 0.046, 0.056)),
        origin=Origin(
            xyz=(-0.182, 0.0, 0.172),
            rpy=(0.0, math.radians(38.0), 0.0),
        ),
        material=body_red,
        name="handle",
    )
    motor_body.visual(
        Box((0.116, 0.076, 0.058)),
        origin=Origin(
            xyz=(-0.238, 0.0, 0.024),
            rpy=(0.0, math.radians(-12.0), 0.0),
        ),
        material=dark_gray,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(
            xyz=(0.0, -0.023, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="left_fold_barrel",
    )
    motor_body.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.023, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="right_fold_barrel",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.340, 0.130, 0.280)),
        mass=2.6,
        origin=Origin(xyz=(-0.170, 0.0, 0.105)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.0225, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="fold_knuckle",
    )
    wand.visual(
        Box((0.040, 0.024, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=graphite,
        name="upper_fork_web",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.675),
        origin=Origin(xyz=(0.0, 0.0, -0.3925)),
        material=aluminum,
        name="main_tube",
    )
    wand.visual(
        Box((0.036, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.712)),
        material=graphite,
        name="head_yoke_bridge",
    )
    wand.visual(
        Box((0.032, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.0225, -0.735)),
        material=graphite,
        name="left_head_cheek",
    )
    wand.visual(
        Box((0.032, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.0225, -0.735)),
        material=graphite,
        name="right_head_cheek",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(
            xyz=(0.0, -0.0225, -0.745),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="left_head_barrel",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.0225, -0.745),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="right_head_barrel",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.770)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.0175, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=graphite,
        name="head_knuckle",
    )
    floor_head.visual(
        Box((0.046, 0.040, 0.042)),
        origin=Origin(xyz=(-0.020, 0.0, -0.021)),
        material=graphite,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.078, 0.232, 0.015)),
        origin=Origin(xyz=(0.000, 0.0, -0.039)),
        material=dark_gray,
        name="upper_shell",
    )
    floor_head.visual(
        Box((0.114, 0.272, 0.028)),
        origin=Origin(xyz=(0.022, 0.0, -0.056)),
        material=dark_gray,
        name="nozzle_body",
    )
    floor_head.visual(
        Box((0.022, 0.265, 0.014)),
        origin=Origin(xyz=(0.072, 0.0, -0.047)),
        material=accent_blue,
        name="front_lip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.120, 0.280, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.018, 0.0, -0.044)),
    )

    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.745)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=math.radians(-20.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_joint = object_model.get_articulation("wand_to_floor_head")

    def extents(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            maxs[0] - mins[0],
            maxs[1] - mins[1],
            maxs[2] - mins[2],
        )

    ctx.expect_contact(
        motor_body,
        wand,
        name="fold hinge stays mounted to the motor body at rest",
    )
    ctx.expect_contact(
        wand,
        floor_head,
        name="floor head stays mounted to the wand at rest",
    )

    body_extents = extents(ctx.part_world_aabb(motor_body))
    wand_extents = extents(ctx.part_world_aabb(wand))
    head_extents = extents(ctx.part_world_aabb(floor_head))
    body_max = max(body_extents) if body_extents is not None else None
    wand_max = max(wand_extents) if wand_extents is not None else None
    head_max = max(head_extents) if head_extents is not None else None
    ctx.check(
        "wand is the dominant primary link",
        body_max is not None
        and wand_max is not None
        and head_max is not None
        and wand_max > body_max + 0.30
        and wand_max > 2.0 * head_max,
        details=f"body_max={body_max}, wand_max={wand_max}, head_max={head_max}",
    )

    rest_floor_head_pos = ctx.part_world_position(floor_head)
    fold_upper = fold_joint.motion_limits.upper if fold_joint.motion_limits else None
    with ctx.pose({fold_joint: fold_upper}):
        folded_floor_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint swings the wand and floor head upward and forward",
        rest_floor_head_pos is not None
        and folded_floor_head_pos is not None
        and fold_upper is not None
        and folded_floor_head_pos[0] > rest_floor_head_pos[0] + 0.25
        and folded_floor_head_pos[2] > rest_floor_head_pos[2] + 0.20,
        details=(
            f"rest_floor_head_pos={rest_floor_head_pos}, "
            f"folded_floor_head_pos={folded_floor_head_pos}, fold_upper={fold_upper}"
        ),
    )

    rest_front_lip = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    head_lower = head_joint.motion_limits.lower if head_joint.motion_limits else None
    head_upper = head_joint.motion_limits.upper if head_joint.motion_limits else None
    with ctx.pose({head_joint: head_lower}):
        lowered_front_lip = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    with ctx.pose({head_joint: head_upper}):
        raised_front_lip = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    ctx.check(
        "floor head hinge provides visible pitch range",
        rest_front_lip is not None
        and lowered_front_lip is not None
        and raised_front_lip is not None
        and head_lower is not None
        and head_upper is not None
        and lowered_front_lip[0][2] < rest_front_lip[0][2] - 0.015
        and raised_front_lip[0][2] > rest_front_lip[0][2] + 0.02,
        details=(
            f"rest_front_lip={rest_front_lip}, lowered_front_lip={lowered_front_lip}, "
            f"raised_front_lip={raised_front_lip}, head_lower={head_lower}, "
            f"head_upper={head_upper}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
