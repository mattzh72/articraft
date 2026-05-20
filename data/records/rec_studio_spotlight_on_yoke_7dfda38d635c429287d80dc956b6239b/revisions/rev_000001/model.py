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

    painted_black = model.material("painted_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.86, 0.95, 0.35))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.42, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=painted_black,
        name="base_plate",
    )
    for index, foot_xy in enumerate(
        ((0.16, 0.11), (0.16, -0.11), (-0.16, 0.11), (-0.16, -0.11))
    ):
        stand.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(foot_xy[0], foot_xy[1], 0.005)),
            material=rubber,
            name=f"foot_{index}",
        )
    stand.visual(
        Cylinder(radius=0.022, length=0.48),
        origin=Origin(xyz=(-0.11, 0.0, 0.26)),
        material=charcoal,
        name="support_column",
    )
    stand.visual(
        Box((0.16, 0.05, 0.04)),
        origin=Origin(xyz=(-0.03, 0.0, 0.485)),
        material=charcoal,
        name="offset_boom",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.025, 0.0, 0.495)),
        material=steel,
        name="pan_bearing",
    )
    stand.visual(
        Box((0.07, 0.08, 0.08)),
        origin=Origin(xyz=(-0.085, 0.0, 0.44)),
        material=charcoal,
        name="column_head",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 0.55)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="turntable",
    )
    yoke.visual(
        Box((0.060, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=painted_black,
        name="yoke_pedestal",
    )
    yoke.visual(
        Box((0.050, 0.210, 0.024)),
        origin=Origin(xyz=(-0.030, 0.0, 0.065)),
        material=painted_black,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.070, 0.026, 0.200)),
        origin=Origin(xyz=(-0.020, 0.110, 0.150)),
        material=painted_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.070, 0.026, 0.200)),
        origin=Origin(xyz=(-0.020, -0.110, 0.150)),
        material=painted_black,
        name="right_arm",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(
            xyz=(-0.018, 0.133, 0.170),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_tilt_knob",
    )
    yoke.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(
            xyz=(-0.018, -0.133, 0.170),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_tilt_knob",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.15, 0.27, 0.26)),
        mass=1.8,
        origin=Origin(xyz=(-0.015, 0.0, 0.13)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.075, length=0.180),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_black,
        name="body_shell",
    )
    lamp.visual(
        Cylinder(radius=0.082, length=0.020),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.070, length=0.004),
        origin=Origin(xyz=(0.114, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.056, length=0.048),
        origin=Origin(xyz=(-0.088, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_housing",
    )
    lamp.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(
            xyz=(0.0, 0.081, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.022, length=0.032),
        origin=Origin(
            xyz=(0.0, -0.081, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )
    lamp.visual(
        Box((0.055, 0.045, 0.030)),
        origin=Origin(xyz=(-0.060, 0.0, 0.070)),
        material=charcoal,
        name="top_driver_box",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.24, 0.17, 0.18)),
        mass=2.6,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_yoke_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.025, 0.0, 0.530)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-math.radians(155.0),
            upper=math.radians(155.0),
        ),
    )
    model.articulation(
        "yoke_to_lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=-math.radians(70.0),
            upper=math.radians(105.0),
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

    ctx.expect_origin_gap(
        yoke,
        stand,
        axis="x",
        min_gap=0.020,
        name="pan axis is offset from the stand base center",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="left_arm",
        negative_elem="body_shell",
        min_gap=0.020,
        max_gap=0.030,
        name="left yoke arm clears the lamp body",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="body_shell",
        negative_elem="right_arm",
        min_gap=0.020,
        max_gap=0.030,
        name="right yoke arm clears the lamp body",
    )

    rest_front_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({pan: math.radians(40.0)}):
        panned_front_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")

    with ctx.pose({tilt: math.radians(50.0)}):
        tilted_front_aabb = ctx.part_element_world_aabb(lamp, elem="front_bezel")

    def aabb_center_y(aabb):
        return None if aabb is None else 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "pan swings the lamp front around the vertical axis",
        rest_front_aabb is not None
        and panned_front_aabb is not None
        and aabb_center_y(panned_front_aabb) is not None
        and aabb_center_y(rest_front_aabb) is not None
        and aabb_center_y(panned_front_aabb) > aabb_center_y(rest_front_aabb) + 0.050,
        details=f"rest={rest_front_aabb}, panned={panned_front_aabb}",
    )
    ctx.check(
        "positive tilt raises the front bezel",
        rest_front_aabb is not None
        and tilted_front_aabb is not None
        and tilted_front_aabb[1][2] > rest_front_aabb[1][2] + 0.050,
        details=f"rest={rest_front_aabb}, tilted={tilted_front_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
