from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    z_min: float,
) -> cq.Workplane:
    """A hollow cylindrical ring authored directly in model coordinates."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(cutter).translate((0.0, 0.0, z_min))


def _shaft_foot_guard() -> cq.Workplane:
    """Stainless bell guard: open side struts and a bottom annular rim."""
    guard = _annular_cylinder(
        outer_radius=0.045,
        inner_radius=0.032,
        length=0.012,
        z_min=-0.362,
    )

    upper_boss = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.302))
    )
    guard = guard.union(upper_boss)

    # Four arched-looking vertical cage ribs welded between the upper boss and
    # bottom rim.  They deliberately leave large windows so the cutter is seen.
    for angle in (45.0, 135.0, 225.0, 315.0):
        rib = (
            cq.Workplane("XY")
            .box(0.026, 0.010, 0.066)
            .translate((0.031, 0.0, -0.324))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        guard = guard.union(rib)

    return guard


def _blade_rotor() -> cq.Workplane:
    """Four sharpened, pitched triangular blades fused to a central hub."""
    rotor = cq.Workplane("XY").circle(0.010).extrude(0.014).translate((0, 0, -0.007))

    base_blade = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.003, -0.006),
                (0.032, -0.010),
                (0.041, -0.001),
                (0.012, 0.009),
            ]
        )
        .close()
        .extrude(0.002)
        .translate((0.0, 0.0, -0.001))
    )

    for index, angle in enumerate((0.0, 90.0, 180.0, 270.0)):
        pitch = 13.0 if index % 2 == 0 else -13.0
        blade = (
            base_blade.rotate((0, 0, 0), (1, 0, 0), pitch)
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        rotor = rotor.union(blade)

    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="immersion_hand_blender")

    white_plastic = model.material("warm_white_plastic", rgba=(0.92, 0.91, 0.86, 1.0))
    dark_rubber = model.material("charcoal_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.73, 0.75, 0.74, 1.0))
    blade_steel = model.material("sharp_blade_steel", rgba=(0.86, 0.88, 0.88, 1.0))

    motor_handle = model.part("motor_handle")
    # Main motor housing: a realistic hand-sized cylindrical body.
    motor_handle.visual(
        Cylinder(radius=0.041, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=white_plastic,
        name="motor_body",
    )
    motor_handle.visual(
        Sphere(radius=0.041),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=white_plastic,
        name="rounded_top",
    )
    motor_handle.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_rubber,
        name="grip_band",
    )
    motor_handle.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=dark_rubber,
        name="upper_grip_band",
    )
    motor_handle.visual(
        mesh_from_cadquery(
            _annular_cylinder(outer_radius=0.035, inner_radius=0.022, length=0.050, z_min=-0.035),
            "socket_collar",
        ),
        material=black_plastic,
        name="socket_collar",
    )
    motor_handle.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.0, -0.046, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="trigger_pivot_socket",
    )
    motor_handle.visual(
        Box((0.020, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.041, 0.175)),
        material=dark_rubber,
        name="trigger_recess",
    )
    # Bayonet slot marks on the collar show the twist-lock direction.
    motor_handle.visual(
        Box((0.020, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.036, -0.014)),
        material=white_plastic,
        name="lock_slot_mark",
    )
    motor_handle.visual(
        Box((0.004, 0.003, 0.018)),
        origin=Origin(xyz=(0.012, -0.036, -0.006)),
        material=white_plastic,
        name="lock_stop_mark",
    )

    shaft = model.part("shaft")
    # The top plug enters the hollow socket collar; small lugs underneath make
    # the bayonet/twist-lock readable without blocking the revolute motion.
    shaft.visual(
        Cylinder(radius=0.019, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=black_plastic,
        name="shaft_plug",
    )
    for angle, name in ((0.0, "lock_lug_0"), (math.pi, "lock_lug_1")):
        shaft.visual(
            Box((0.014, 0.008, 0.006)),
            origin=Origin(
                xyz=(0.026 * math.cos(angle), 0.026 * math.sin(angle), -0.038),
                rpy=(0.0, 0.0, angle),
            ),
            material=black_plastic,
            name=name,
        )
    shaft.visual(
        Cylinder(radius=0.011, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=stainless,
        name="steel_tube",
    )
    shaft.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.314)),
        material=stainless,
        name="blade_axle_bearing",
    )
    shaft.visual(
        mesh_from_cadquery(_shaft_foot_guard(), "foot_guard"),
        material=stainless,
        name="foot_guard",
    )

    blade_rotor = model.part("blade_rotor")
    blade_rotor.visual(
        mesh_from_cadquery(_blade_rotor(), "blade_set", tolerance=0.0005),
        material=blade_steel,
        name="blade_set",
    )
    blade_rotor.visual(
        Cylinder(radius=0.004, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=blade_steel,
        name="drive_axle",
    )

    power_trigger = model.part("power_trigger")
    power_trigger.visual(
        Cylinder(radius=0.0045, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="pivot_pin",
    )
    power_trigger.visual(
        Box((0.024, 0.009, 0.095)),
        origin=Origin(xyz=(0.0, -0.004, -0.052)),
        material=dark_rubber,
        name="trigger_pad",
    )

    model.articulation(
        "handle_to_shaft",
        ArticulationType.REVOLUTE,
        parent=motor_handle,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "shaft_to_blade_rotor",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=blade_rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.337)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=80.0),
    )
    model.articulation(
        "handle_to_power_trigger",
        ArticulationType.REVOLUTE,
        parent=motor_handle,
        child=power_trigger,
        origin=Origin(xyz=(0.0, -0.046, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_handle = object_model.get_part("motor_handle")
    shaft = object_model.get_part("shaft")
    blade_rotor = object_model.get_part("blade_rotor")
    power_trigger = object_model.get_part("power_trigger")
    shaft_joint = object_model.get_articulation("handle_to_shaft")
    trigger_joint = object_model.get_articulation("handle_to_power_trigger")

    ctx.allow_overlap(
        motor_handle,
        power_trigger,
        elem_a="trigger_pivot_socket",
        elem_b="pivot_pin",
        reason="The trigger pin is intentionally captured inside the molded side pivot socket.",
    )
    ctx.allow_overlap(
        shaft,
        blade_rotor,
        elem_a="blade_axle_bearing",
        elem_b="drive_axle",
        reason="The blade drive axle is intentionally nested inside the shaft-tip bearing.",
    )
    ctx.expect_overlap(
        motor_handle,
        power_trigger,
        axes="x",
        elem_a="trigger_pivot_socket",
        elem_b="pivot_pin",
        min_overlap=0.030,
        name="trigger pin spans the pivot socket",
    )
    ctx.expect_overlap(
        shaft,
        blade_rotor,
        axes="z",
        elem_a="blade_axle_bearing",
        elem_b="drive_axle",
        min_overlap=0.016,
        name="blade axle remains captured in the bearing",
    )

    ctx.expect_within(
        shaft,
        motor_handle,
        axes="xy",
        inner_elem="shaft_plug",
        outer_elem="socket_collar",
        margin=0.002,
        name="shaft plug is centered in the handle collar",
    )
    ctx.expect_overlap(
        shaft,
        motor_handle,
        axes="z",
        elem_a="shaft_plug",
        elem_b="socket_collar",
        min_overlap=0.025,
        name="shaft plug remains inserted in the twist lock collar",
    )

    with ctx.pose({shaft_joint: 0.45}):
        ctx.expect_within(
            shaft,
            motor_handle,
            axes="xy",
            inner_elem="shaft_plug",
            outer_elem="socket_collar",
            margin=0.002,
            name="twisted shaft remains centered in the collar",
        )

    ctx.expect_within(
        blade_rotor,
        shaft,
        axes="xy",
        inner_elem="blade_set",
        outer_elem="foot_guard",
        margin=0.004,
        name="four-blade rotor sits inside the protective foot",
    )
    ctx.expect_overlap(
        blade_rotor,
        shaft,
        axes="z",
        elem_a="blade_set",
        elem_b="foot_guard",
        min_overlap=0.010,
        name="blade rotor is vertically inside the foot guard",
    )

    rest_trigger = ctx.part_element_world_aabb(power_trigger, elem="trigger_pad")
    with ctx.pose({trigger_joint: 0.35}):
        pressed_trigger = ctx.part_element_world_aabb(power_trigger, elem="trigger_pad")
    ctx.check(
        "side trigger pivots inward",
        rest_trigger is not None
        and pressed_trigger is not None
        and pressed_trigger[1][1] > rest_trigger[1][1] + 0.008,
        details=f"rest={rest_trigger}, pressed={pressed_trigger}",
    )

    return ctx.report()


object_model = build_object_model()
