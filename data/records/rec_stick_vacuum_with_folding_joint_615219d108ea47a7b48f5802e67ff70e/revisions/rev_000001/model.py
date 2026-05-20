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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    copper_clear = model.material("copper_clear", rgba=(0.82, 0.55, 0.25, 0.38))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(-0.10, 0.0, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_can",
    )
    motor_body.visual(
        Cylinder(radius=0.041, length=0.07),
        origin=Origin(xyz=(-0.185, 0.0, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="rear_cap",
    )
    motor_body.visual(
        Cylinder(radius=0.058, length=0.17),
        origin=Origin(xyz=(0.035, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=copper_clear,
        name="dust_bin",
    )
    motor_body.visual(
        Cylinder(radius=0.033, length=0.060),
        origin=Origin(xyz=(0.145, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="intake_nose",
    )
    motor_body.visual(
        Box((0.145, 0.082, 0.17)),
        origin=Origin(xyz=(-0.095, 0.0, -0.075)),
        material=graphite,
        name="battery_pack",
    )
    handle_geom = tube_from_spline_points(
        [
            (-0.18, 0.0, 0.135),
            (-0.15, 0.0, 0.215),
            (-0.08, 0.0, 0.265),
            (0.01, 0.0, 0.245),
            (0.055, 0.0, 0.165),
        ],
        radius=0.013,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    motor_body.visual(
        _mesh("motor_body_handle", handle_geom),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=graphite,
        name="handle_loop",
    )
    motor_body.visual(
        Box((0.090, 0.050, 0.130)),
        origin=Origin(xyz=(-0.055, 0.0, 0.125)),
        material=graphite,
        name="grip_bridge",
    )
    motor_body.visual(
        Box((0.076, 0.006, 0.034)),
        origin=Origin(xyz=(0.178, 0.035, 0.015)),
        material=soft_black,
        name="fold_outer_plate_left",
    )
    motor_body.visual(
        Box((0.076, 0.006, 0.034)),
        origin=Origin(xyz=(0.178, -0.035, 0.015)),
        material=soft_black,
        name="fold_outer_plate_right",
    )
    motor_body.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.132, 0.0, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="fold_cross_tie",
    )
    motor_body.visual(
        Box((0.046, 0.070, 0.012)),
        origin=Origin(xyz=(0.130, 0.0, -0.002)),
        material=soft_black,
        name="fold_lower_tie",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.43, 0.16, 0.37)),
        mass=3.2,
        origin=Origin(xyz=(-0.015, 0.0, 0.055)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="fold_hub",
    )
    wand.visual(
        Box((0.050, 0.006, 0.028)),
        origin=Origin(xyz=(0.024, 0.018, 0.0)),
        material=soft_black,
        name="fold_inner_plate_left",
    )
    wand.visual(
        Box((0.050, 0.006, 0.028)),
        origin=Origin(xyz=(0.024, -0.018, 0.0)),
        material=soft_black,
        name="fold_inner_plate_right",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.63),
        origin=Origin(xyz=(0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="wand_tube",
    )
    wand.visual(
        Box((0.060, 0.050, 0.020)),
        origin=Origin(xyz=(0.642, 0.0, -0.018)),
        material=soft_black,
        name="neck_block",
    )
    wand.visual(
        Box((0.055, 0.006, 0.052)),
        origin=Origin(xyz=(0.675, 0.028, -0.032)),
        material=soft_black,
        name="head_fork_left",
    )
    wand.visual(
        Box((0.055, 0.006, 0.052)),
        origin=Origin(xyz=(0.675, -0.028, -0.032)),
        material=soft_black,
        name="head_fork_right",
    )
    wand.visual(
        Box((0.030, 0.050, 0.014)),
        origin=Origin(xyz=(0.646, 0.0, -0.033)),
        material=soft_black,
        name="head_fork_tie",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.74, 0.08, 0.11)),
        mass=0.9,
        origin=Origin(xyz=(0.36, 0.0, -0.01)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.205, 0.0, 0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.060, 0.026, 0.020)),
        origin=Origin(xyz=(0.042, 0.0, -0.012)),
        material=soft_black,
        name="neck_strut",
    )
    floor_head.visual(
        Box((0.270, 0.105, 0.028)),
        origin=Origin(xyz=(0.145, 0.0, -0.032)),
        material=graphite,
        name="head_shell",
    )
    floor_head.visual(
        Cylinder(radius=0.026, length=0.202),
        origin=Origin(xyz=(0.120, 0.0, -0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="brush_chamber",
    )
    floor_head.visual(
        Box((0.050, 0.090, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, -0.044)),
        material=soft_black,
        name="rear_skid",
    )
    floor_head.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.028, 0.046, -0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="wheel_left",
    )
    floor_head.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.028, -0.046, -0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="wheel_right",
    )
    floor_head.visual(
        Box((0.050, 0.085, 0.010)),
        origin=Origin(xyz=(0.245, 0.0, -0.016)),
        material=soft_black,
        name="front_bumper",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.07)),
        mass=0.75,
        origin=Origin(xyz=(0.145, 0.0, -0.030)),
    )

    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.705, 0.0, -0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-0.45,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    ctx.expect_origin_distance(
        wand,
        motor_body,
        axes="x",
        min_dist=0.18,
        max_dist=0.23,
        name="fold joint sits ahead of the motor body",
    )

    ctx.expect_origin_distance(
        floor_head,
        motor_body,
        axes="x",
        min_dist=0.88,
        max_dist=0.94,
        name="straight vacuum reaches realistic stick length",
    )

    ctx.expect_overlap(
        floor_head,
        wand,
        axes="y",
        min_overlap=0.045,
        name="floor head stays centered under the wand fork",
    )

    rest_aabb = ctx.part_world_aabb(floor_head)
    with ctx.pose({fold_joint: 0.95}):
        folded_aabb = ctx.part_world_aabb(floor_head)
    ctx.check(
        "fold joint drops the cleaning end for low-reach posture",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] < rest_aabb[0][2] - 0.20
        and folded_aabb[1][0] < rest_aabb[1][0] - 0.12,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    rest_head_aabb = ctx.part_world_aabb(floor_head)
    with ctx.pose({head_pitch: 0.40}):
        pitched_aabb = ctx.part_world_aabb(floor_head)
    ctx.check(
        "floor head pitch lets the nose dip down relative to the wand",
        rest_head_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[0][2] < rest_head_aabb[0][2] - 0.020
        and pitched_aabb[1][0] < rest_head_aabb[1][0],
        details=f"rest={rest_head_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
