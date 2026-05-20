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

    base_black = model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bracket_black = model.material("bracket_black", rgba=(0.16, 0.16, 0.18, 1.0))
    can_gray = model.material("can_gray", rgba=(0.30, 0.31, 0.33, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.22, 0.24, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.84, 0.90, 0.60))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.19, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.045, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=base_black,
        name="pedestal",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=dark_metal,
        name="bearing_collar",
    )
    stand.visual(
        Box((0.11, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=bracket_black,
        name="support_bridge",
    )
    stand.visual(
        Box((0.085, 0.022, 0.12)),
        origin=Origin(xyz=(-0.01, 0.175, 0.22)),
        material=bracket_black,
        name="left_stage_support",
    )
    stand.visual(
        Box((0.085, 0.022, 0.12)),
        origin=Origin(xyz=(-0.01, -0.175, 0.22)),
        material=bracket_black,
        name="right_stage_support",
    )
    stand.visual(
        Box((0.045, 0.35, 0.012)),
        origin=Origin(xyz=(-0.035, 0.0, 0.274)),
        material=bracket_black,
        name="upper_rear_tie",
    )
    stand.visual(
        Box((0.08, 0.06, 0.035)),
        origin=Origin(xyz=(-0.03, 0.0, 0.145)),
        material=bracket_black,
        name="rear_spine_block",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.30)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.048, length=0.028),
        material=dark_metal,
        name="pan_drum",
    )
    yoke.visual(
        Cylinder(radius=0.06, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_metal,
        name="pan_cap",
    )
    yoke.visual(
        Box((0.11, 0.30, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=bracket_black,
        name="yoke_base_plate",
    )
    yoke.visual(
        Box((0.028, 0.018, 0.19)),
        origin=Origin(xyz=(0.0, 0.145, 0.125)),
        material=bracket_black,
        name="left_yoke_arm",
    )
    yoke.visual(
        Box((0.028, 0.018, 0.19)),
        origin=Origin(xyz=(0.0, -0.145, 0.125)),
        material=bracket_black,
        name="right_yoke_arm",
    )
    yoke.visual(
        Box((0.022, 0.29, 0.016)),
        origin=Origin(xyz=(-0.048, 0.0, 0.215)),
        material=bracket_black,
        name="rear_handle_bridge",
    )
    yoke.visual(
        Box((0.05, 0.018, 0.034)),
        origin=Origin(xyz=(-0.036, 0.145, 0.203)),
        material=bracket_black,
        name="left_rear_gusset",
    )
    yoke.visual(
        Box((0.05, 0.018, 0.034)),
        origin=Origin(xyz=(-0.036, -0.145, 0.203)),
        material=bracket_black,
        name="right_rear_gusset",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.132, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_tilt_boss",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, -0.132, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_tilt_boss",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.16, 0.32, 0.25)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.010, length=0.224),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_axle",
    )
    lamp.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, -0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.083, length=0.16),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=can_gray,
        name="lamp_body",
    )
    lamp.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.159, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.076, length=0.006),
        origin=Origin(xyz=(0.171, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_housing",
    )
    lamp.visual(
        Box((0.07, 0.055, 0.032)),
        origin=Origin(xyz=(0.02, 0.0, 0.074)),
        material=dark_metal,
        name="top_driver_box",
    )
    lamp.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(-0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.26, 0.24, 0.20)),
        mass=3.2,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-1.7,
            upper=1.7,
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.65,
            upper=1.15,
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
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_origin_gap(
        yoke,
        stand,
        axis="z",
        min_gap=0.0,
        max_gap=0.25,
        name="pan stage sits above the stand",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        margin=0.0,
        name="lamp can stays between the yoke arms in width",
    )

    rest_lens = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    with ctx.pose({tilt: 0.9}):
        raised_lens = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive tilt raises the front lens",
        rest_lens is not None and raised_lens is not None and raised_lens[2] > rest_lens[2] + 0.03,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    with ctx.pose({pan: 1.1}):
        panned_lens = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_lens"))
    ctx.check(
        "positive pan swings the lamp toward positive Y",
        rest_lens is not None and panned_lens is not None and panned_lens[1] > rest_lens[1] + 0.08,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
