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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    base_black = model.material("base_black", rgba=(0.11, 0.11, 0.12, 1.0))
    yoke_gray = model.material("yoke_gray", rgba=(0.25, 0.26, 0.28, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hardware = model.material("hardware", rgba=(0.46, 0.48, 0.50, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.74, 0.84, 0.94, 0.35))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.15, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=base_black,
        name="base_plate",
    )
    support.visual(
        Cylinder(radius=0.042, length=0.41),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=base_black,
        name="stand_column",
    )
    support.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=hardware,
        name="pan_bearing_housing",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.53)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.086, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=yoke_gray,
        name="pan_collar",
    )
    yoke.visual(
        Box((0.12, 0.40, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=yoke_gray,
        name="yoke_crosshead",
    )
    yoke.visual(
        Box((0.05, 0.028, 0.23)),
        origin=Origin(xyz=(0.0, 0.186, 0.205)),
        material=yoke_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.05, 0.028, 0.23)),
        origin=Origin(xyz=(0.0, -0.186, 0.205)),
        material=yoke_gray,
        name="right_arm",
    )
    yoke.visual(
        Box((0.074, 0.038, 0.04)),
        origin=Origin(xyz=(0.0, 0.198, 0.225)),
        material=hardware,
        name="left_trunnion_block",
    )
    yoke.visual(
        Box((0.074, 0.038, 0.04)),
        origin=Origin(xyz=(0.0, -0.198, 0.225)),
        material=hardware,
        name="right_trunnion_block",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.14, 0.42, 0.34)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    shell_outer_profile = [
        (0.088, -0.176),
        (0.104, -0.156),
        (0.128, -0.050),
        (0.136, 0.080),
        (0.144, 0.150),
        (0.159, 0.188),
    ]
    shell_inner_profile = [
        (0.072, -0.170),
        (0.088, -0.152),
        (0.111, -0.052),
        (0.119, 0.078),
        (0.128, 0.148),
        (0.141, 0.182),
    ]
    lamp_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            shell_outer_profile,
            shell_inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "spotlight_shell",
    )

    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        lamp_shell_mesh,
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_black,
        name="can_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.138, length=0.006),
        origin=Origin(xyz=(0.204, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_can.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(-0.142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="rear_cap",
    )
    lamp_can.visual(
        Cylinder(radius=0.021, length=0.05),
        origin=Origin(xyz=(0.0, 0.147, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion",
    )
    lamp_can.visual(
        Cylinder(radius=0.021, length=0.05),
        origin=Origin(xyz=(0.0, -0.147, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion",
    )
    lamp_can.inertial = Inertial.from_geometry(
        Box((0.39, 0.32, 0.32)),
        mass=5.5,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(110.0),
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_can,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=-math.radians(55.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    yoke = object_model.get_part("yoke")
    lamp_can = object_model.get_part("lamp_can")
    pan = object_model.get_articulation("support_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    ctx.expect_within(
        lamp_can,
        yoke,
        axes="y",
        margin=0.002,
        name="lamp can stays between the yoke arms at rest",
    )

    support_aabb = ctx.part_world_aabb(support)
    yoke_aabb = ctx.part_world_aabb(yoke)
    support_width = support_aabb[1][1] - support_aabb[0][1] if support_aabb is not None else None
    yoke_width = yoke_aabb[1][1] - yoke_aabb[0][1] if yoke_aabb is not None else None
    ctx.check(
        "rotating yoke reads larger than the fixed support",
        support_width is not None
        and yoke_width is not None
        and yoke_width > support_width * 1.3,
        details=f"support_width={support_width}, yoke_width={yoke_width}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    rest_center = (
        (
            (rest_lens[0][0] + rest_lens[1][0]) * 0.5,
            (rest_lens[0][1] + rest_lens[1][1]) * 0.5,
            (rest_lens[0][2] + rest_lens[1][2]) * 0.5,
        )
        if rest_lens is not None
        else None
    )

    with ctx.pose({pan: math.radians(40.0)}):
        panned_lens = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    panned_center = (
        (
            (panned_lens[0][0] + panned_lens[1][0]) * 0.5,
            (panned_lens[0][1] + panned_lens[1][1]) * 0.5,
            (panned_lens[0][2] + panned_lens[1][2]) * 0.5,
        )
        if panned_lens is not None
        else None
    )
    ctx.check(
        "positive pan swings the spotlight around the vertical stand axis",
        rest_center is not None
        and panned_center is not None
        and panned_center[1] > rest_center[1] + 0.10,
        details=f"rest_center={rest_center}, panned_center={panned_center}",
    )

    with ctx.pose({tilt: math.radians(30.0)}):
        tilted_lens = ctx.part_element_world_aabb(lamp_can, elem="front_lens")
    tilted_center = (
        (
            (tilted_lens[0][0] + tilted_lens[1][0]) * 0.5,
            (tilted_lens[0][1] + tilted_lens[1][1]) * 0.5,
            (tilted_lens[0][2] + tilted_lens[1][2]) * 0.5,
        )
        if tilted_lens is not None
        else None
    )
    ctx.check(
        "positive tilt raises the front of the lamp can",
        rest_center is not None
        and tilted_center is not None
        and tilted_center[2] > rest_center[2] + 0.10,
        details=f"rest_center={rest_center}, tilted_center={tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
