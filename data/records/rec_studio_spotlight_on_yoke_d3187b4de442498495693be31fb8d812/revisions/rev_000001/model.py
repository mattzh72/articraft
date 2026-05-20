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

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.78, 0.84, 0.65))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.19, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_black,
        name="floor_base",
    )
    support.visual(
        Cylinder(radius=0.055, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=dark_gray,
        name="lower_column",
    )
    support.visual(
        Cylinder(radius=0.072, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 1.035)),
        material=steel,
        name="height_collar",
    )
    support.visual(
        Cylinder(radius=0.038, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
        material=steel,
        name="upper_column",
    )
    support.visual(
        Cylinder(radius=0.085, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.44)),
        material=charcoal,
        name="top_receiver",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 1.49)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=charcoal,
        name="turntable",
    )
    yoke.visual(
        Box((0.085, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_gray,
        name="center_saddle",
    )
    yoke.visual(
        Box((0.11, 0.30, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.05, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, 0.16, 0.25)),
        material=dark_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.05, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, -0.16, 0.25)),
        material=dark_gray,
        name="right_arm",
    )
    yoke.visual(
        Box((0.06, 0.34, 0.03)),
        origin=Origin(xyz=(-0.03, 0.0, 0.405)),
        material=dark_gray,
        name="top_brace",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.20, 0.34, 0.44)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    lamp = model.part("lamp")
    outer_profile = [
        (0.0, -0.112),
        (0.050, -0.106),
        (0.082, -0.094),
        (0.112, -0.040),
        (0.118, 0.065),
        (0.122, 0.132),
        (0.136, 0.184),
    ]
    inner_profile = [
        (0.0, -0.098),
        (0.044, -0.092),
        (0.074, -0.082),
        (0.100, -0.038),
        (0.106, 0.064),
        (0.110, 0.146),
        (0.120, 0.171),
    ]
    lamp_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ),
        "spotlight_shell",
    )
    lamp.visual(
        lamp_shell,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.131, length=0.018),
        origin=Origin(xyz=(0.171, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.116, length=0.012),
        origin=Origin(xyz=(0.168, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, -0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(-0.082, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_cap",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.31, 0.27, 0.27)),
        mass=4.5,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.49)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-0.55,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("support_to_yoke_pan")
    tilt = object_model.get_articulation("yoke_to_lamp_tilt")

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple(
            (aabb[0][idx] + aabb[1][idx]) * 0.5
            for idx in range(3)
        )

    ctx.check(
        "pan joint uses a vertical axis",
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan.axis}",
    )
    ctx.check(
        "tilt joint uses a horizontal axle",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    yoke_origin = ctx.part_world_position(yoke)
    ctx.check(
        "pan stage sits high above the floor base",
        yoke_origin is not None and yoke_origin[2] > 1.45,
        details=f"yoke_origin={yoke_origin}",
    )

    ctx.expect_gap(
        yoke,
        support,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_receiver",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable seats cleanly on the top receiver",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="left_arm",
        negative_elem="left_trunnion",
        max_gap=0.004,
        max_penetration=0.0,
        name="left trunnion stays just inside the left yoke arm",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="right_trunnion",
        negative_elem="right_arm",
        max_gap=0.004,
        max_penetration=0.0,
        name="right trunnion stays just inside the right yoke arm",
    )

    rest_lens = center_from_aabb(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        raised_lens = center_from_aabb(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive tilt raises the lamp nose",
        rest_lens is not None
        and raised_lens is not None
        and raised_lens[2] > rest_lens[2] + 0.10,
        details=f"rest_lens={rest_lens}, raised_lens={raised_lens}",
    )

    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens = center_from_aabb(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "pan rotation swings the beam direction sideways",
        rest_lens is not None
        and panned_lens is not None
        and rest_lens[0] > 0.12
        and abs(panned_lens[0]) < 0.08
        and panned_lens[1] > 0.12,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
