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
    model = ArticulatedObject(name="studio_spotlight_on_yoke_stand")

    stand_black = model.material("stand_black", rgba=(0.10, 0.10, 0.11, 1.0))
    housing_black = model.material("housing_black", rgba=(0.13, 0.13, 0.14, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.86, 0.90, 0.65))
    trim_gray = model.material("trim_gray", rgba=(0.28, 0.29, 0.31, 1.0))

    lamp_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.018, -0.145),
                (0.070, -0.141),
                (0.106, -0.110),
                (0.118, -0.030),
                (0.121, 0.070),
                (0.129, 0.124),
                (0.140, 0.152),
            ],
            [
                (0.000, -0.145),
                (0.050, -0.128),
                (0.092, -0.100),
                (0.100, -0.024),
                (0.102, 0.080),
                (0.110, 0.132),
                (0.116, 0.152),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "spotlight_can_shell",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.175, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=stand_black,
        name="floor_base",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=stand_black,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.6375)),
        material=trim_gray,
        name="top_collar",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.71)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.054, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=trim_gray,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.070, 0.100, 0.108)),
        origin=Origin(xyz=(-0.060, 0.0, 0.082)),
        material=trim_gray,
        name="hub_riser",
    )
    yoke.visual(
        Box((0.160, 0.340, 0.048)),
        origin=Origin(xyz=(-0.030, 0.0, 0.123)),
        material=trim_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.050, 0.018, 0.270)),
        origin=Origin(xyz=(0.0, 0.165, 0.282)),
        material=trim_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.050, 0.018, 0.270)),
        origin=Origin(xyz=(0.0, -0.165, 0.282)),
        material=trim_gray,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, 0.165, 0.280), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="left_trunnion_plate",
    )
    yoke.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, -0.165, 0.280), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="right_trunnion_plate",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.18, 0.35, 0.43)),
        mass=5.5,
        origin=Origin(xyz=(-0.005, 0.0, 0.215)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        lamp_shell,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_black,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.116, length=0.006),
        origin=Origin(xyz=(0.169, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Box((0.080, 0.110, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.090)),
        material=trim_gray,
        name="top_ballast_box",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="left_axle_boss",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="right_axle_boss",
    )
    lamp.visual(
        Box((0.032, 0.030, 0.082)),
        origin=Origin(xyz=(-0.006, 0.131, 0.0)),
        material=trim_gray,
        name="left_side_lug",
    )
    lamp.visual(
        Box((0.032, 0.030, 0.082)),
        origin=Origin(xyz=(-0.006, -0.131, 0.0)),
        material=trim_gray,
        name="right_side_lug",
    )
    lamp.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.31),
        mass=4.2,
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=-1.0,
            upper=1.2,
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

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan hub sits tightly on the stand collar",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="left_axle_boss",
        elem_b="left_arm",
        name="left tilt axle boss contacts the left yoke arm",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="right_axle_boss",
        elem_b="right_arm",
        name="right tilt axle boss contacts the right yoke arm",
    )

    rest_lens_center = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    with ctx.pose({pan: 0.8}):
        panned_lens_center = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive pan swings the lamp toward +Y",
        rest_lens_center is not None
        and panned_lens_center is not None
        and panned_lens_center[1] > rest_lens_center[1] + 0.08
        and abs(panned_lens_center[2] - rest_lens_center[2]) < 0.01,
        details=f"rest={rest_lens_center}, pan={panned_lens_center}",
    )

    with ctx.pose({tilt: 0.7}):
        tilted_lens_center = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive tilt lifts the lamp nose upward",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.08,
        details=f"rest={rest_lens_center}, tilt={tilted_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
