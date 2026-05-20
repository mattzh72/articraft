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

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.17, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.27, 0.28, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.80, 0.90, 0.38))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.25, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=matte_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.18, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=gunmetal,
        name="ballast_drum",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=satin_black,
        name="center_column",
    )
    stand.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=gunmetal,
        name="lower_pan_bearing",
    )
    stand.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=gunmetal,
        name="upper_pan_bearing",
    )
    stand.visual(
        Box((0.22, 0.10, 0.20)),
        origin=Origin(xyz=(0.11, 0.0, 0.55)),
        material=gunmetal,
        name="front_stage_support",
    )
    stand.visual(
        Box((0.22, 0.10, 0.20)),
        origin=Origin(xyz=(-0.11, 0.0, 0.55)),
        material=gunmetal,
        name="rear_stage_support",
    )
    stand.visual(
        Box((0.14, 0.11, 0.18)),
        origin=Origin(xyz=(0.0, 0.105, 0.56)),
        material=gunmetal,
        name="left_stage_cheek",
    )
    stand.visual(
        Box((0.14, 0.11, 0.18)),
        origin=Origin(xyz=(0.0, -0.105, 0.56)),
        material=gunmetal,
        name="right_stage_cheek",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.50, 0.50, 0.74)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.10, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=satin_black,
        name="rotary_crown",
    )
    yoke.visual(
        Box((0.10, 0.14, 0.22)),
        origin=Origin(xyz=(-0.08, 0.0, 0.17)),
        material=satin_black,
        name="rear_spine",
    )
    yoke.visual(
        Box((0.16, 0.52, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.31)),
        material=satin_black,
        name="bridge_block",
    )
    yoke.visual(
        Box((0.05, 0.055, 0.34)),
        origin=Origin(xyz=(-0.005, 0.23, 0.50)),
        material=satin_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.05, 0.055, 0.34)),
        origin=Origin(xyz=(-0.005, -0.23, 0.50)),
        material=satin_black,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.072, length=0.09),
        origin=Origin(xyz=(0.0, 0.3025, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_tilt_housing",
    )
    yoke.visual(
        Cylinder(radius=0.072, length=0.09),
        origin=Origin(xyz=(0.0, -0.3025, 0.50), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_tilt_housing",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.20, 0.70, 0.68)),
        mass=5.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.34)),
    )

    lamp = model.part("lamp")
    can_outer = [
        (0.105, -0.145),
        (0.122, -0.132),
        (0.138, -0.106),
        (0.148, -0.060),
        (0.150, 0.070),
        (0.146, 0.130),
        (0.134, 0.170),
        (0.120, 0.195),
    ]
    can_inner = [
        (0.0, -0.139),
        (0.096, -0.130),
        (0.111, -0.104),
        (0.120, -0.060),
        (0.120, 0.068),
        (0.116, 0.128),
        (0.102, 0.188),
    ]
    can_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(can_outer, can_inner, segments=56),
        "spotlight_can_shell",
    )
    lamp.visual(
        can_mesh,
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.158, length=0.026),
        origin=Origin(xyz=(0.218, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.142, length=0.008),
        origin=Origin(xyz=(0.226, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.108, length=0.070),
        origin=Origin(xyz=(-0.126, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.090, 0.100, 0.070)),
        origin=Origin(xyz=(-0.168, 0.0, 0.0)),
        material=gunmetal,
        name="rear_driver_box",
    )
    lamp.visual(
        Box((0.055, 0.030, 0.110)),
        origin=Origin(xyz=(-0.005, 0.143, 0.0)),
        material=satin_black,
        name="left_trunnion_web",
    )
    lamp.visual(
        Box((0.055, 0.030, 0.110)),
        origin=Origin(xyz=(-0.005, -0.143, 0.0)),
        material=satin_black,
        name="right_trunnion_web",
    )
    lamp.visual(
        Cylinder(radius=0.050, length=0.036),
        origin=Origin(xyz=(-0.005, 0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="left_trunnion_disc",
    )
    lamp.visual(
        Cylinder(radius=0.050, length=0.036),
        origin=Origin(xyz=(-0.005, -0.165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="right_trunnion_disc",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(-0.005, 0.190, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_axle_stub",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(-0.005, -0.190, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_axle_stub",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.44, 0.40, 0.34)),
        mass=4.2,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-0.95,
            upper=1.20,
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

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        ctx.expect_gap(
            yoke,
            stand,
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
            name="yoke seats on pan bearing",
        )
        ctx.expect_gap(
            lamp,
            stand,
            axis="z",
            min_gap=0.28,
            name="lamp clears stand structure",
        )
        ctx.expect_within(
            lamp,
            yoke,
            axes="y",
            margin=0.0,
            name="lamp sits between yoke arms",
        )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        rest_front = elem_center(lamp, "front_bezel")
    with ctx.pose({pan: 0.0, tilt: 0.85}):
        tilted_front = elem_center(lamp, "front_bezel")
    ctx.check(
        "positive tilt raises lamp nose",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.10,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    with ctx.pose({pan: 0.0, tilt: 0.0}):
        centered_front = elem_center(lamp, "front_bezel")
    with ctx.pose({pan: 0.8, tilt: 0.0}):
        panned_front = elem_center(lamp, "front_bezel")
    ctx.check(
        "positive pan swings beam to camera-left",
        centered_front is not None
        and panned_front is not None
        and panned_front[1] > centered_front[1] + 0.12,
        details=f"centered_front={centered_front}, panned_front={panned_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
