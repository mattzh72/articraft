from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """CadQuery-backed rounded premium consumer-product block."""

    shape = cq.Workplane("XY").box(*size).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_folding_stick_vacuum")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.058, 0.060, 1.0))
    soft_black = model.material("soft_black", rgba=(0.010, 0.011, 0.012, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.60, 0.59, 1.0))
    warm_titanium = model.material("warm_titanium", rgba=(0.72, 0.62, 0.49, 1.0))
    smoke_clear = model.material("smoke_clear_polycarbonate", rgba=(0.28, 0.34, 0.38, 0.48))
    dark_rubber = model.material("dark_rubber", rgba=(0.018, 0.018, 0.017, 1.0))
    brush_blue = model.material("muted_blue_brush", rgba=(0.05, 0.19, 0.22, 1.0))

    motor_body = model.part("motor_body")

    # The root part is authored around the folding-joint pivot.  Its geometry
    # extends upward into a slim premium motor pod and handle.
    motor_body.visual(
        Cylinder(radius=0.016, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=satin_metal,
        name="upper_wand",
    )
    motor_body.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=soft_black,
        name="upper_socket",
    )
    motor_body.visual(
        _rounded_box_mesh("motor_shell", (0.088, 0.094, 0.335), 0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=matte_graphite,
        name="motor_shell",
    )
    motor_body.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        material=warm_titanium,
        name="lower_trim_band",
    )
    motor_body.visual(
        Cylinder(radius=0.047, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.842)),
        material=warm_titanium,
        name="top_trim_band",
    )
    motor_body.visual(
        _rounded_box_mesh("filter_window", (0.010, 0.078, 0.160), 0.006),
        origin=Origin(xyz=(-0.049, 0.0, 0.690)),
        material=smoke_clear,
        name="filter_window",
    )
    motor_body.visual(
        _rounded_box_mesh("battery_pack", (0.060, 0.086, 0.178), 0.013),
        origin=Origin(xyz=(0.073, 0.0, 0.640)),
        material=soft_black,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.004, 0.090, 0.135)),
        origin=Origin(xyz=(0.043, 0.0, 0.640)),
        material=warm_titanium,
        name="battery_seam",
    )

    # A compact loop handle is integrated into the motor pod through bridged
    # satin/soft-touch interfaces rather than floating grip pieces.
    motor_body.visual(
        Cylinder(radius=0.014, length=0.292),
        origin=Origin(xyz=(0.141, 0.0, 0.745)),
        material=dark_rubber,
        name="rear_grip",
    )
    motor_body.visual(
        Cylinder(radius=0.014, length=0.112),
        origin=Origin(xyz=(0.094, 0.0, 0.886), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="top_handle_bridge",
    )
    motor_body.visual(
        Cylinder(radius=0.014, length=0.118),
        origin=Origin(xyz=(0.091, 0.0, 0.606), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="lower_handle_bridge",
    )
    motor_body.visual(
        Box((0.028, 0.058, 0.032)),
        origin=Origin(xyz=(0.048, 0.0, 0.886)),
        material=matte_graphite,
        name="upper_handle_root",
    )
    motor_body.visual(
        Box((0.030, 0.058, 0.036)),
        origin=Origin(xyz=(0.047, 0.0, 0.606)),
        material=matte_graphite,
        name="lower_handle_root",
    )

    # Exposed folding joint hardware: a satin barrel, restrained side caps, and
    # a visible coaxial pin make the primary articulation obvious.
    motor_body.visual(
        Cylinder(radius=0.032, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_titanium,
        name="fold_hub",
    )
    motor_body.visual(
        Cylinder(radius=0.008, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fold_pin",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.014, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=satin_metal,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.038, 0.094, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=soft_black,
        name="fold_yoke_bridge",
    )
    lower_wand.visual(
        Box((0.040, 0.012, 0.112)),
        origin=Origin(xyz=(0.0, -0.044, -0.015)),
        material=soft_black,
        name="fold_side_plate_0",
    )
    lower_wand.visual(
        Box((0.040, 0.012, 0.112)),
        origin=Origin(xyz=(0.0, 0.044, -0.015)),
        material=soft_black,
        name="fold_side_plate_1",
    )
    lower_wand.visual(
        Cylinder(radius=0.021, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        material=soft_black,
        name="lower_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.022, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_titanium,
        name="floor_pivot_barrel",
    )
    lower_wand.visual(
        Cylinder(radius=0.0065, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="floor_axle_pin",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        _rounded_box_mesh("head_shell", (0.138, 0.360, 0.052), 0.012),
        origin=Origin(xyz=(0.028, 0.0, -0.056)),
        material=matte_graphite,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.010, 0.338, 0.020)),
        origin=Origin(xyz=(-0.046, 0.0, -0.045)),
        material=dark_rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.312),
        origin=Origin(xyz=(-0.022, 0.0, -0.079), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brush_blue,
        name="brush_roll",
    )
    floor_head.visual(
        Box((0.050, 0.012, 0.074)),
        origin=Origin(xyz=(0.0, -0.044, 0.006)),
        material=soft_black,
        name="head_ear_0",
    )
    floor_head.visual(
        Box((0.050, 0.012, 0.074)),
        origin=Origin(xyz=(0.0, 0.044, 0.006)),
        material=soft_black,
        name="head_ear_1",
    )
    floor_head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, -0.056, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="head_axle_cap_0",
    )
    floor_head.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.056, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="head_axle_cap_1",
    )
    floor_head.visual(
        Box((0.022, 0.130, 0.006)),
        origin=Origin(xyz=(0.064, 0.0, -0.080)),
        material=satin_metal,
        name="rear_sole_plate",
    )
    floor_head.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.077, -0.155, -0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="side_wheel_0",
    )
    floor_head.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.077, 0.155, -0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="side_wheel_1",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.017, 0.036, 0.076)),
        origin=Origin(xyz=(0.0135, 0.0, -0.039)),
        material=warm_titanium,
        name="trigger_paddle",
    )
    trigger.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="trigger_pin",
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.55),
    )
    model.articulation(
        "head_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.8, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "trigger_hinge",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=trigger,
        origin=Origin(xyz=(0.105, 0.0, 0.793)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.5, lower=0.0, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    motor_body = object_model.get_part("motor_body")
    lower_wand = object_model.get_part("lower_wand")
    floor_head = object_model.get_part("floor_head")
    trigger = object_model.get_part("trigger")
    fold_joint = object_model.get_articulation("fold_joint")
    head_pivot = object_model.get_articulation("head_pivot")
    trigger_hinge = object_model.get_articulation("trigger_hinge")

    ctx.check(
        "fold joint has realistic range",
        fold_joint.motion_limits is not None
        and fold_joint.motion_limits.lower == 0.0
        and fold_joint.motion_limits.upper is not None
        and fold_joint.motion_limits.upper > 1.3,
        details=str(fold_joint.motion_limits),
    )
    ctx.check(
        "floor head pivots independently",
        head_pivot.motion_limits is not None
        and head_pivot.motion_limits.lower < -0.4
        and head_pivot.motion_limits.upper > 0.55,
        details=str(head_pivot.motion_limits),
    )
    ctx.check(
        "trigger is articulated",
        trigger_hinge.motion_limits is not None and trigger_hinge.motion_limits.upper > 0.20,
        details=str(trigger_hinge.motion_limits),
    )

    ctx.expect_overlap(
        motor_body,
        lower_wand,
        axes="xz",
        elem_a="fold_hub",
        elem_b="fold_side_plate_0",
        min_overlap=0.030,
        name="fold yoke wraps the hub in projection",
    )
    ctx.expect_overlap(
        lower_wand,
        floor_head,
        axes="xz",
        elem_a="floor_pivot_barrel",
        elem_b="head_ear_0",
        min_overlap=0.025,
        name="floor head ears align with pivot barrel",
    )
    ctx.expect_contact(
        trigger,
        motor_body,
        elem_a="trigger_paddle",
        elem_b="rear_grip",
        contact_tol=0.002,
        name="trigger sits against handle grip",
    )

    def _aabb_center(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))

    rest_head_center = _aabb_center(floor_head)
    with ctx.pose({fold_joint: 1.20}):
        folded_head_center = _aabb_center(floor_head)
    ctx.check(
        "fold joint lifts and tucks the floor head",
        rest_head_center is not None
        and folded_head_center is not None
        and folded_head_center[0] < rest_head_center[0] - 0.25
        and folded_head_center[2] > rest_head_center[2] + 0.20,
        details=f"rest={rest_head_center}, folded={folded_head_center}",
    )

    return ctx.report()


object_model = build_object_model()
