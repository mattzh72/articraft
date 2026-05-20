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
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    stand_black = model.material("stand_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.49, 0.51, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.68, 0.78, 0.90, 0.65))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.20, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=stand_black,
        name="floor_base",
    )
    stand.visual(
        Cylinder(radius=0.150, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=dark_gray,
        name="base_riser",
    )
    stand.visual(
        Cylinder(radius=0.017, length=1.34),
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        material=steel,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.450)),
        material=dark_gray,
        name="top_hub",
    )
    stand.visual(
        Cylinder(radius=0.013, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.5125)),
        material=steel,
        name="top_spigot",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 1.535)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.7675)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_gray,
        name="receiver_sleeve",
    )
    yoke.visual(
        Box((0.050, 0.050, 0.185)),
        origin=Origin(xyz=(0.015, 0.0, 0.1325)),
        material=dark_gray,
        name="pan_post",
    )
    yoke.visual(
        Box((0.220, 0.055, 0.030)),
        origin=Origin(xyz=(0.080, 0.0, 0.2375)),
        material=dark_gray,
        name="yoke_head",
    )
    yoke.visual(
        Cylinder(radius=0.017, length=0.290),
        origin=Origin(xyz=(0.160, 0.0, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="cross_tube",
    )
    yoke.visual(
        Box((0.028, 0.032, 0.240)),
        origin=Origin(xyz=(0.160, 0.122, 0.125)),
        material=dark_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.028, 0.032, 0.240)),
        origin=Origin(xyz=(0.160, -0.122, 0.125)),
        material=dark_gray,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.160, 0.103, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_bearing",
    )
    yoke.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.160, -0.103, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_bearing",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.20, 0.30, 0.21)),
        mass=1.6,
        origin=Origin(xyz=(0.055, 0.0, 0.105)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.090, length=0.180),
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="lamp_can",
    )
    lamp.visual(
        Cylinder(radius=0.100, length=0.045),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_housing",
    )
    lamp.visual(
        Cylinder(radius=0.106, length=0.018),
        origin=Origin(xyz=(0.2665, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.094, length=0.006),
        origin=Origin(xyz=(0.278, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    lamp.visual(
        Cylinder(radius=0.060, length=0.100),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_neck",
    )
    lamp.visual(
        Cylinder(radius=0.072, length=0.055),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.050, 0.058, 0.072)),
        origin=Origin(xyz=(0.002, 0.054, 0.0)),
        material=dark_gray,
        name="left_side_bracket",
    )
    lamp.visual(
        Box((0.050, 0.058, 0.072)),
        origin=Origin(xyz=(0.002, -0.054, 0.0)),
        material=dark_gray,
        name="right_side_bracket",
    )
    lamp.visual(
        Cylinder(radius=0.030, length=0.013),
        origin=Origin(xyz=(0.0, 0.0825, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.030, length=0.013),
        origin=Origin(xyz=(0.0, -0.0825, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.36, 0.20, 0.22)),
        mass=2.2,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.535)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.160, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=math.radians(-75.0),
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("stand_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    ctx.expect_contact(
        yoke,
        stand,
        elem_a="receiver_sleeve",
        elem_b="top_spigot",
        name="yoke seats on the top spigot",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        margin=0.0,
        name="lamp body stays between the yoke arms in width",
    )

    rest_pos = ctx.part_world_position(lamp)
    with ctx.pose({pan: math.pi / 2.0}):
        pan_pos = ctx.part_world_position(lamp)

    ctx.check(
        "pan rotation swings the lamp around the vertical mast",
        rest_pos is not None
        and pan_pos is not None
        and abs(pan_pos[1]) > 0.045
        and abs(pan_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, pan={pan_pos}",
    )

    rest_bezel = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({tilt: math.radians(45.0)}):
        tilted_bezel = ctx.part_element_world_aabb(lamp, elem="front_bezel")

    rest_bezel_center_z = None
    tilted_bezel_center_z = None
    if rest_bezel is not None:
        rest_bezel_center_z = 0.5 * (rest_bezel[0][2] + rest_bezel[1][2])
    if tilted_bezel is not None:
        tilted_bezel_center_z = 0.5 * (tilted_bezel[0][2] + tilted_bezel[1][2])

    ctx.check(
        "positive tilt raises the front of the spotlight",
        rest_bezel_center_z is not None
        and tilted_bezel_center_z is not None
        and tilted_bezel_center_z > rest_bezel_center_z + 0.07,
        details=f"rest_z={rest_bezel_center_z}, tilted_z={tilted_bezel_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
