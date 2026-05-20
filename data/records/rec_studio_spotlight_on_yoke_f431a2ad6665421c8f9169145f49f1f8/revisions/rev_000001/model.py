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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    stand_black = model.material("stand_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.62, 0.70, 0.78, 0.45))

    base = model.part("base")
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.42, 0.30, 0.05), 0.018),
        "base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stand_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=dark_gray,
        name="turntable",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_gray,
        name="pan_collar",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 0.062)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.042, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_gray,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.020, 0.040, 0.236)),
        origin=Origin(xyz=(-0.052, 0.0, 0.173)),
        material=dark_gray,
        name="spine",
    )
    yoke.visual(
        Box((0.062, 0.245, 0.030)),
        origin=Origin(xyz=(-0.031, 0.0, 0.290)),
        material=dark_gray,
        name="bridge",
    )
    yoke.visual(
        Box((0.018, 0.024, 0.220)),
        origin=Origin(xyz=(0.0, 0.110, 0.175)),
        material=dark_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.018, 0.024, 0.220)),
        origin=Origin(xyz=(0.0, -0.110, 0.175)),
        material=dark_gray,
        name="right_arm",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.080, 0.245, 0.320)),
        mass=2.2,
        origin=Origin(xyz=(-0.010, 0.0, 0.160)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=-math.radians(160.0),
            upper=math.radians(160.0),
        ),
    )

    lamp = model.part("lamp")
    can_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.056, -0.090),
                (0.064, -0.070),
                (0.070, -0.020),
                (0.076, 0.040),
                (0.090, 0.080),
                (0.095, 0.086),
            ],
            inner_profile=[
                (0.046, -0.072),
                (0.053, -0.020),
                (0.060, 0.036),
                (0.075, 0.070),
            ],
            segments=56,
        ),
        "lamp_can_shell",
    )
    lamp.visual(
        can_shell_mesh,
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_black,
        name="can_shell",
    )
    lamp.visual(
        Cylinder(radius=0.074, length=0.018),
        origin=Origin(
            xyz=(0.123, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.013, length=0.042),
        origin=Origin(
            xyz=(0.0, 0.077, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.013, length=0.042),
        origin=Origin(
            xyz=(0.0, -0.077, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.200, 0.180, 0.190)),
        mass=3.8,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        # The lamp body projects forward along +X from the side axle.
        # -Y makes positive motion tilt the beam upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-math.radians(65.0),
            upper=math.radians(50.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan_joint = object_model.get_articulation("base_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_lamp")

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="pan_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan hub sits cleanly on the collar",
    )
    ctx.expect_overlap(
        yoke,
        base,
        axes="xy",
        elem_a="pan_hub",
        elem_b="pan_collar",
        min_overlap=0.075,
        name="pan hub remains centered over the collar",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        margin=0.0,
        name="lamp stays between the yoke arms",
    )

    rest_front = elem_center("lamp", "front_lens")
    with ctx.pose({pan_joint: math.radians(90.0)}):
        panned_front = elem_center("lamp", "front_lens")
    ctx.check(
        "positive pan swings the beam toward +Y",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.060,
        details=f"rest={rest_front}, panned={panned_front}",
    )

    with ctx.pose({tilt_joint: math.radians(35.0)}):
        tilted_front = elem_center("lamp", "front_lens")
    ctx.check(
        "positive tilt raises the beam",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.040,
        details=f"rest={rest_front}, tilted={tilted_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
