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

    stand_black = model.material("stand_black", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.36, 0.37, 0.40, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.82, 0.88, 0.96, 0.45))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.26, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=stand_black,
        name="floor_base",
    )
    stand.visual(
        Cylinder(radius=0.095, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=hardware_gray,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=hardware_gray,
        name="central_support",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=stand_black,
        name="top_bearing_housing",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.52, 0.52, 0.88)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )

    yoke_assembly = model.part("yoke_assembly")
    yoke_assembly.visual(
        Cylinder(radius=0.145, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=stand_black,
        name="turntable_disc",
    )
    yoke_assembly.visual(
        Cylinder(radius=0.055, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=hardware_gray,
        name="pan_hub",
    )
    yoke_assembly.visual(
        Box((0.10, 0.43, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=hardware_gray,
        name="lower_bridge",
    )
    yoke_assembly.visual(
        Box((0.045, 0.04, 0.52)),
        origin=Origin(xyz=(0.015, 0.215, 0.42)),
        material=hardware_gray,
        name="left_arm",
    )
    yoke_assembly.visual(
        Box((0.045, 0.04, 0.52)),
        origin=Origin(xyz=(0.015, -0.215, 0.42)),
        material=hardware_gray,
        name="right_arm",
    )
    yoke_assembly.visual(
        Cylinder(radius=0.031, length=0.05),
        origin=Origin(xyz=(0.015, 0.215, 0.54), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_black,
        name="left_trunnion_block",
    )
    yoke_assembly.visual(
        Cylinder(radius=0.031, length=0.05),
        origin=Origin(xyz=(0.015, -0.215, 0.54), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_black,
        name="right_trunnion_block",
    )
    yoke_assembly.inertial = Inertial.from_geometry(
        Box((0.30, 0.50, 0.70)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    lamp_can = model.part("lamp_can")
    outer_profile = [
        (0.07, -0.12),
        (0.10, -0.11),
        (0.135, -0.06),
        (0.145, 0.03),
        (0.155, 0.12),
        (0.167, 0.19),
        (0.167, 0.21),
    ]
    inner_profile = [
        (0.0, -0.102),
        (0.088, -0.095),
        (0.118, -0.05),
        (0.128, 0.03),
        (0.138, 0.13),
        (0.148, 0.195),
    ]
    lamp_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "lamp_shell",
    )
    lamp_can.visual(
        lamp_shell,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lamp_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.152, length=0.012),
        origin=Origin(xyz=(0.202, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_can.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(-0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_gray,
        name="rear_housing",
    )
    lamp_can.visual(
        Cylinder(radius=0.028, length=0.054),
        origin=Origin(xyz=(0.0, 0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_gray,
        name="left_trunnion",
    )
    lamp_can.visual(
        Cylinder(radius=0.028, length=0.054),
        origin=Origin(xyz=(0.0, -0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_gray,
        name="right_trunnion",
    )
    lamp_can.inertial = Inertial.from_geometry(
        Box((0.42, 0.34, 0.34)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    model.articulation(
        "stand_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke_assembly,
        child=lamp_can,
        origin=Origin(xyz=(0.015, 0.0, 0.54)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(60.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    yoke_assembly = object_model.get_part("yoke_assembly")
    lamp_can = object_model.get_part("lamp_can")
    stand_pan = object_model.get_articulation("stand_pan")
    yoke_tilt = object_model.get_articulation("yoke_tilt")

    ctx.expect_gap(
        yoke_assembly,
        stand,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="top_bearing_housing",
        max_gap=0.003,
        max_penetration=0.0,
        name="turntable sits on the top bearing housing",
    )
    ctx.expect_within(
        lamp_can,
        yoke_assembly,
        axes="y",
        margin=0.0,
        name="lamp can stays between the yoke arms",
    )
    ctx.expect_gap(
        lamp_can,
        yoke_assembly,
        axis="z",
        negative_elem="lower_bridge",
        min_gap=0.12,
        name="lamp can clears the lower yoke bridge",
    )

    def center_of(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = center_of(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    with ctx.pose({yoke_tilt: math.radians(55.0)}):
        raised_lens = center_of(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    ctx.check(
        "positive tilt raises the front lens",
        rest_lens is not None and raised_lens is not None and raised_lens[2] > rest_lens[2] + 0.12,
        details=f"rest_lens={rest_lens}, raised_lens={raised_lens}",
    )

    with ctx.pose({stand_pan: math.radians(60.0), yoke_tilt: 0.0}):
        panned_lens = center_of(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    ctx.check(
        "positive pan swings the beam toward positive world y",
        rest_lens is not None and panned_lens is not None and panned_lens[1] > rest_lens[1] + 0.15,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
