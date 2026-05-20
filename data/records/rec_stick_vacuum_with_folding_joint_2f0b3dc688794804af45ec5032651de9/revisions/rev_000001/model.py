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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_plastic = model.material("body_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    nozzle_plastic = model.material("nozzle_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    accent_red = model.material("accent_red", rgba=(0.76, 0.18, 0.13, 1.0))
    cup_clear = model.material("cup_clear", rgba=(0.72, 0.80, 0.92, 0.38))
    roller_dark = model.material("roller_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    motor_body = model.part("motor_body")
    motor_body_shell = section_loft(
        [
            _yz_section(-0.180, 0.054, 0.076, 0.015, z_center=0.076),
            _yz_section(-0.120, 0.082, 0.110, 0.023, z_center=0.080),
            _yz_section(-0.055, 0.092, 0.112, 0.024, z_center=0.070),
            _yz_section(-0.008, 0.064, 0.072, 0.018, z_center=0.042),
        ]
    )
    motor_body.visual(
        mesh_from_geometry(motor_body_shell, "motor_body_shell"),
        material=body_plastic,
        name="motor_body_shell",
    )
    motor_body.visual(
        Cylinder(radius=0.034, length=0.110),
        origin=Origin(xyz=(-0.057, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cup_clear,
        name="dust_cup",
    )
    motor_body.visual(
        Box((0.056, 0.050, 0.028)),
        origin=Origin(xyz=(-0.036, 0.0, 0.014)),
        material=accent_red,
        name="hinge_nose",
    )
    motor_body.visual(
        Box((0.100, 0.060, 0.034)),
        origin=Origin(xyz=(-0.130, 0.0, 0.024)),
        material=body_plastic,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.040, 0.044, 0.124)),
        origin=Origin(xyz=(-0.114, 0.0, 0.138)),
        material=body_plastic,
        name="rear_handle",
    )
    motor_body.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nozzle_plastic,
        name="fold_hinge_left_ear",
    )
    motor_body.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nozzle_plastic,
        name="fold_hinge_right_ear",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.220, 0.092, 0.210)),
        mass=1.6,
        origin=Origin(xyz=(-0.095, 0.0, 0.085)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.0115, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.032, 0.016, 0.024)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=nozzle_plastic,
        name="upper_hinge_block",
    )
    wand.visual(
        Cylinder(radius=0.011, length=0.236),
        origin=Origin(xyz=(0.138, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_plastic,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.013, length=0.048),
        origin=Origin(xyz=(0.246, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_plastic,
        name="lower_collar",
    )
    wand.visual(
        Box((0.024, 0.036, 0.020)),
        origin=Origin(xyz=(0.281, 0.0, 0.0)),
        material=nozzle_plastic,
        name="head_hinge_mount",
    )
    wand.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.305, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nozzle_plastic,
        name="head_hinge_left_ear",
    )
    wand.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.305, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nozzle_plastic,
        name="head_hinge_right_ear",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.320, 0.040, 0.028)),
        mass=0.42,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
    )

    floor_head = model.part("floor_head")
    head_shell = section_loft(
        [
            _yz_section(0.000, 0.050, 0.018, 0.006, z_center=-0.030),
            _yz_section(0.048, 0.128, 0.026, 0.009, z_center=-0.032),
            _yz_section(0.118, 0.160, 0.022, 0.008, z_center=-0.032),
            _yz_section(0.170, 0.144, 0.018, 0.007, z_center=-0.028),
        ]
    )
    floor_head.visual(
        Cylinder(radius=0.0105, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nozzle_plastic,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.028, 0.016, 0.038)),
        origin=Origin(xyz=(0.015, 0.0, -0.015)),
        material=nozzle_plastic,
        name="neck_block",
    )
    floor_head.visual(
        mesh_from_geometry(head_shell, "floor_head_shell"),
        material=nozzle_plastic,
        name="nozzle_shell",
    )
    floor_head.visual(
        Box((0.024, 0.132, 0.008)),
        origin=Origin(xyz=(0.171, 0.0, -0.018)),
        material=accent_red,
        name="front_lip",
    )
    floor_head.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.030, 0.0, -0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_dark,
        name="rear_roller",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.190, 0.160, 0.050)),
        mass=0.55,
        origin=Origin(xyz=(0.095, 0.0, -0.025)),
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    model.articulation(
        "head_pitch_joint",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=math.radians(-20.0),
            upper=math.radians(34.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fold_joint = object_model.get_articulation("fold_joint")
    head_pitch_joint = object_model.get_articulation("head_pitch_joint")
    floor_head = object_model.get_part("floor_head")
    wand = object_model.get_part("wand")

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        floor_head,
        wand,
        axis="x",
        positive_elem="pitch_barrel",
        negative_elem="head_hinge_mount",
        min_gap=0.001,
        max_gap=0.003,
        name="pitch barrel clears the wand hinge mount",
    )
    ctx.expect_gap(
        floor_head,
        wand,
        axis="x",
        positive_elem="pitch_barrel",
        negative_elem="lower_collar",
        min_gap=0.020,
        max_gap=0.030,
        name="pitch barrel stays ahead of the lower collar",
    )

    front_rest = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        front_folded = ctx.part_element_world_aabb(floor_head, elem="front_lip")

    rest_front_center = None
    folded_front_center = None
    if front_rest is not None:
        rest_front_center = tuple(
            (front_rest[0][i] + front_rest[1][i]) * 0.5 for i in range(3)
        )
    if front_folded is not None:
        folded_front_center = tuple(
            (front_folded[0][i] + front_folded[1][i]) * 0.5 for i in range(3)
        )
    ctx.check(
        "fold joint lifts the nozzle for storage",
        rest_front_center is not None
        and folded_front_center is not None
        and folded_front_center[0] < rest_front_center[0] - 0.10
        and folded_front_center[2] > rest_front_center[2] + 0.10,
        details=f"rest_front={rest_front_center}, folded_front={folded_front_center}",
    )

    with ctx.pose({head_pitch_joint: head_pitch_joint.motion_limits.lower}):
        front_up = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    with ctx.pose({head_pitch_joint: head_pitch_joint.motion_limits.upper}):
        front_down = ctx.part_element_world_aabb(floor_head, elem="front_lip")

    up_center_z = None
    down_center_z = None
    if front_up is not None:
        up_center_z = (front_up[0][2] + front_up[1][2]) * 0.5
    if front_down is not None:
        down_center_z = (front_down[0][2] + front_down[1][2]) * 0.5
    ctx.check(
        "floor head pitches through a meaningful range",
        up_center_z is not None
        and down_center_z is not None
        and up_center_z > down_center_z + 0.10,
        details=f"up_center_z={up_center_z}, down_center_z={down_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
