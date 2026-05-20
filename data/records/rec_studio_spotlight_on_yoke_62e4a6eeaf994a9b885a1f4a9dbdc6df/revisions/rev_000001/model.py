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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    stand_metal = model.material("stand_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.27, 0.28, 0.30, 1.0))
    shell_paint = model.material("shell_paint", rgba=(0.10, 0.10, 0.11, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.05, 0.05, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.86, 0.92, 0.55))

    leg_path = [
        (0.034, 0.0, 0.058),
        (0.085, 0.0, 0.047),
        (0.165, 0.0, 0.024),
        (0.245, 0.0, 0.010),
    ]
    leg_mesh = mesh_from_geometry(
        tube_from_spline_points(
            leg_path,
            radius=0.010,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "stand_leg",
    )

    can_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.000, -0.118),
                (0.040, -0.112),
                (0.078, -0.088),
                (0.098, -0.020),
                (0.103, 0.100),
                (0.114, 0.182),
            ],
            inner_profile=[
                (0.000, -0.116),
                (0.034, -0.108),
                (0.066, -0.078),
                (0.088, -0.020),
                (0.094, 0.098),
                (0.098, 0.175),
            ],
            segments=56,
        ),
        "lamp_can_shell",
    )

    stand_base = model.part("stand_base")
    stand_base.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hardware,
        name="base_hub",
    )
    for index in range(3):
        stand_base.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, index * 2.0 * math.pi / 3.0)),
            material=stand_metal,
            name=f"leg_{index}",
        )
        foot_angle = index * 2.0 * math.pi / 3.0
        stand_base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(
                xyz=(0.245 * math.cos(foot_angle), 0.245 * math.sin(foot_angle), 0.005)
            ),
            material=hardware,
            name=f"foot_{index}",
        )
    stand_base.visual(
        Cylinder(radius=0.022, length=0.262),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=stand_metal,
        name="center_mast",
    )
    stand_base.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=hardware,
        name="lower_pan_bearing",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.52, 0.52, 0.34)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=hardware,
        name="upper_pan_plate",
    )
    yoke.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=hardware,
        name="pan_spacer",
    )
    yoke.visual(
        Cylinder(radius=0.013, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=hardware,
        name="yoke_stem",
    )
    yoke.visual(
        Box((0.030, 0.372, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=stand_metal,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.014, 0.012, 0.142)),
        origin=Origin(xyz=(0.0, 0.184, 0.176)),
        material=stand_metal,
        name="right_arm",
    )
    yoke.visual(
        Box((0.014, 0.012, 0.142)),
        origin=Origin(xyz=(0.0, -0.184, 0.176)),
        material=stand_metal,
        name="left_arm",
    )
    yoke.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, 0.187, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion_collar",
    )
    yoke.visual(
        Cylinder(radius=0.020, length=0.026),
        origin=Origin(xyz=(0.0, -0.187, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion_collar",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.14, 0.39, 0.28)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        can_shell_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_paint,
        name="can_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.099, length=0.006),
        origin=Origin(xyz=(0.173, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_can.visual(
        Cylinder(radius=0.046, length=0.028),
        origin=Origin(xyz=(-0.108, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bezel_black,
        name="rear_cap",
    )
    lamp_can.visual(
        Box((0.060, 0.094, 0.018)),
        origin=Origin(xyz=(-0.008, 0.112, 0.0)),
        material=hardware,
        name="right_side_ear",
    )
    lamp_can.visual(
        Box((0.060, 0.094, 0.018)),
        origin=Origin(xyz=(-0.008, -0.112, 0.0)),
        material=hardware,
        name="left_side_ear",
    )
    lamp_can.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.0, 0.161, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trunnion",
    )
    lamp_can.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.0, -0.161, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trunnion",
    )
    lamp_can.inertial = Inertial.from_geometry(
        Box((0.32, 0.34, 0.22)),
        mass=3.2,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=stand_base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_can,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=-math.radians(35.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand_base = object_model.get_part("stand_base")
    yoke = object_model.get_part("yoke")
    lamp_can = object_model.get_part("lamp_can")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    ctx.expect_origin_gap(
        yoke,
        stand_base,
        axis="z",
        min_gap=0.30,
        max_gap=0.40,
        name="yoke rides above the open stand",
    )
    ctx.expect_gap(
        lamp_can,
        stand_base,
        axis="z",
        min_gap=0.12,
        name="lamp can clears the stand",
    )
    ctx.expect_contact(
        yoke,
        lamp_can,
        elem_a="right_trunnion_collar",
        elem_b="right_trunnion",
        name="right trunnion seats against the yoke collar",
    )
    ctx.expect_contact(
        yoke,
        lamp_can,
        elem_a="left_trunnion_collar",
        elem_b="left_trunnion",
        name="left trunnion seats against the yoke collar",
    )

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_front = elem_center("lamp_can", "front_lens")
    with ctx.pose({pan_joint: math.radians(55.0)}):
        panned_front = elem_center("lamp_can", "front_lens")
    ctx.check(
        "positive pan swings the lamp toward +Y",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.10,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({tilt_joint: math.radians(55.0)}):
        tilted_front = elem_center("lamp_can", "front_lens")
    ctx.check(
        "positive tilt raises the lamp nose",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.08,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
