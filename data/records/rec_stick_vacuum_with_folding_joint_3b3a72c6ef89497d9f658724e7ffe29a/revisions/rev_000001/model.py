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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    orange = model.material("orange", rgba=(0.86, 0.45, 0.10, 1.0))
    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    silver = model.material("silver", rgba=(0.79, 0.81, 0.84, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.68, 0.78, 0.90, 0.36))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.046, length=0.112),
        origin=Origin(xyz=(-0.015, 0.0, 0.865), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_pod",
    )
    motor_body.visual(
        Box((0.112, 0.090, 0.086)),
        origin=Origin(xyz=(0.018, 0.0, 0.805)),
        material=orange,
        name="body_bridge",
    )
    motor_body.visual(
        Cylinder(radius=0.060, length=0.168),
        origin=Origin(xyz=(0.050, 0.0, 0.758)),
        material=smoke_clear,
        name="dust_bin",
    )
    motor_body.visual(
        Box((0.054, 0.044, 0.160)),
        origin=Origin(xyz=(-0.072, 0.0, 0.835)),
        material=graphite,
        name="handle_post",
    )
    motor_body.visual(
        Box((0.040, 0.050, 0.170)),
        origin=Origin(
            xyz=(-0.112, 0.0, 0.905),
            rpy=(0.0, math.radians(24.0), 0.0),
        ),
        material=graphite,
        name="hand_grip",
    )
    motor_body.visual(
        Box((0.108, 0.072, 0.164)),
        origin=Origin(xyz=(-0.010, 0.0, 0.665)),
        material=graphite,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.074, 0.040, 0.126)),
        origin=Origin(xyz=(0.016, 0.022, 0.602)),
        material=orange,
        name="fold_support_arm",
    )
    motor_body.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.020, 0.030, 0.600), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="fold_support_barrel",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.290, 0.180, 0.410)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="wand_barrel",
    )
    wand.visual(
        Box((0.040, 0.022, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=graphite,
        name="upper_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        material=orange,
        name="wand_collar",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.366),
        origin=Origin(xyz=(0.0, 0.0, -0.333)),
        material=silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.042, 0.030, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, -0.513)),
        material=graphite,
        name="lower_yoke",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.620)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
    )

    floor_head = model.part("floor_head")
    head_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.290, 0.115, 0.024), 0.036),
        "floor_head_shell",
    )
    floor_head.visual(
        Box((0.044, 0.030, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=graphite,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.074, 0.040, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, -0.046)),
        material=graphite,
        name="neck_fairing",
    )
    floor_head.visual(
        head_shell_mesh,
        origin=Origin(xyz=(0.060, 0.0, -0.042)),
        material=graphite,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.102, 0.104, 0.011)),
        origin=Origin(xyz=(0.152, 0.0, -0.0195)),
        material=orange,
        name="front_lip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.300, 0.120, 0.070)),
        mass=0.84,
        origin=Origin(xyz=(0.060, 0.0, -0.040)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.020, 0.055, 0.600)),
        # The wand hangs along local -Z, so -Y makes positive motion fold it
        # forward in +X while also lifting the floor head.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.540)),
        # The head shell projects mostly along +X from the rear hinge, so -Y
        # makes positive motion lift the front edge.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=math.radians(-22.0),
            upper=math.radians(52.0),
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
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_joint = object_model.get_articulation("wand_to_floor_head")

    ctx.expect_gap(
        wand,
        motor_body,
        axis="y",
        positive_elem="wand_barrel",
        negative_elem="fold_support_barrel",
        min_gap=0.0,
        max_gap=0.0025,
        name="moving fold chain sits just outside the fixed support",
    )

    with ctx.pose({fold_joint: 0.0, head_joint: 0.0}):
        rest_head_aabb = ctx.part_world_aabb(floor_head)
        rest_head_pos = ctx.part_world_position(floor_head)
        rest_front_lip = ctx.part_element_world_aabb(floor_head, elem="front_lip")

    ctx.check(
        "floor head reaches the floor in the straight pose",
        rest_head_aabb is not None and abs(rest_head_aabb[0][2]) <= 0.004,
        details=f"floor_head_aabb={rest_head_aabb}",
    )

    with ctx.pose({fold_joint: math.radians(60.0), head_joint: 0.0}):
        folded_head_pos = ctx.part_world_position(floor_head)

    ctx.check(
        "fold joint lifts the wand and floor head forward",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > rest_head_pos[0] + 0.12
        and folded_head_pos[2] > rest_head_pos[2] + 0.12,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    with ctx.pose({fold_joint: 0.0, head_joint: math.radians(40.0)}):
        pitched_front_lip = ctx.part_element_world_aabb(floor_head, elem="front_lip")

    ctx.check(
        "floor head hinge pitches the front lip upward",
        rest_front_lip is not None
        and pitched_front_lip is not None
        and pitched_front_lip[0][2] > rest_front_lip[0][2] + 0.025,
        details=f"rest_front_lip={rest_front_lip}, pitched_front_lip={pitched_front_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
