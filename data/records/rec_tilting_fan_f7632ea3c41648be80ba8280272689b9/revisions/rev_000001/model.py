from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PIVOT_X = -0.075
PIVOT_Z = 0.72
GUARD_RADIUS = 0.245
ROTOR_RADIUS = 0.178
TILT_UPPER = math.radians(35.0)
TILT_LOWER = math.radians(-22.0)


def _rounded_base_shell() -> cq.Workplane:
    """One-piece low oval shell: cheap weighted base with a molded lip."""
    base = (
        cq.Workplane("XY")
        .ellipse(0.31, 0.205)
        .extrude(0.055)
    )
    top_lip = (
        cq.Workplane("XY")
        .ellipse(0.255, 0.160)
        .ellipse(0.210, 0.118)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.055))
    )
    return base.union(top_lip)


def _cylinder_between(p0: tuple[float, float, float], p1: tuple[float, float, float], radius: float):
    start = cq.Vector(*p0)
    end = cq.Vector(*p1)
    direction = end - start
    return cq.Solid.makeCylinder(radius, direction.Length, start, direction.normalized())


def _fan_guard_shell() -> cq.Workplane:
    """Moldable two-plane guard: outer hoop, hub pads, and integral radial ribs."""

    outer_hoop = (
        cq.Workplane("YZ")
        .circle(GUARD_RADIUS)
        .circle(GUARD_RADIUS - 0.020)
        .extrude(0.110, both=True)
        .translate((0.010, 0.0, 0.0))
    )

    guard = outer_hoop
    # Front and rear center pads keep the spokes connected and make the
    # assembly order obvious: the rotor/hub sits between molded guard halves.
    for x in (0.063, -0.044):
        hub_pad = (
            cq.Workplane("YZ")
            .circle(0.043)
            .extrude(0.010, both=True)
            .translate((x, 0.0, 0.0))
        )
        guard = guard.union(hub_pad)

    # Low-count radial ribs: cheaper than a dense wire cage but still protective.
    for x, count, phase, rib_radius in ((0.061, 12, 0.0, 0.0030), (-0.043, 8, math.pi / 8.0, 0.0034)):
        for i in range(count):
            theta = phase + i * 2.0 * math.pi / count
            cy = math.cos(theta)
            sz = math.sin(theta)
            p0 = (x, 0.040 * cy, 0.040 * sz)
            p1 = (x, (GUARD_RADIUS - 0.015) * cy, (GUARD_RADIUS - 0.015) * sz)
            rib = _cylinder_between(p0, p1, rib_radius)
            guard = guard.union(cq.Workplane().add(rib))

    # Three short rear motor bridges show a real mounting path from the motor pod
    # to the guard without adding separate stamped brackets.
    for theta in (math.radians(90), math.radians(210), math.radians(330)):
        cy = math.cos(theta)
        sz = math.sin(theta)
        bridge = _cylinder_between(
            (-0.060, 0.060 * cy, 0.060 * sz),
            (-0.044, 0.118 * cy, 0.118 * sz),
            0.0050,
        )
        guard = guard.union(cq.Workplane().add(bridge))

    return guard


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_tilting_fan")

    shell_plastic = model.material("warm_gray_molded_plastic", rgba=(0.74, 0.74, 0.70, 1.0))
    dark_plastic = model.material("dark_gray_motor_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    blade_plastic = model.material("smoky_blue_blade_plastic", rgba=(0.18, 0.32, 0.48, 0.82))
    fastener_black = model.material("black_fasteners", rgba=(0.02, 0.02, 0.02, 1.0))
    label_blue = model.material("blue_snap_tabs", rgba=(0.08, 0.22, 0.75, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base_shell(), "oval_weighted_base", tolerance=0.0015),
        material=shell_plastic,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.033, length=0.220),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.145), rpy=(0.0, 0.0, 0.0)),
        material=shell_plastic,
        name="center_post",
    )
    base.visual(
        Box((0.070, 0.610, 0.050)),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.205)),
        material=shell_plastic,
        name="lower_yoke_bridge",
    )

    for idx, y in enumerate((-0.275, 0.275)):
        base.visual(
            Box((0.060, 0.040, 0.520)),
            origin=Origin(xyz=(PIVOT_X, y, 0.450)),
            material=shell_plastic,
            name=f"yoke_arm_{idx}",
        )
        base.visual(
            Cylinder(radius=0.056, length=0.046),
            origin=Origin(xyz=(PIVOT_X, y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=shell_plastic,
            name=f"yoke_lug_{idx}",
        )
        # Molded-through screw/knob cap: a visible clamp interface but not an
        # extra moving part in this low-cost architecture.
        outer_y = y + (0.030 if y > 0.0 else -0.030)
        base.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(PIVOT_X, outer_y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener_black,
            name=f"clamp_cap_{idx}",
        )

    # Small snap feet and cord notch are molded details, not separate hardware.
    for idx, y in enumerate((-0.125, 0.125)):
        base.visual(
            Box((0.100, 0.030, 0.012)),
            origin=Origin(xyz=(-0.230, y, 0.006)),
            material=fastener_black,
            name=f"rubber_foot_{idx}",
        )
    base.visual(
        Box((0.050, 0.030, 0.018)),
        origin=Origin(xyz=(0.300, 0.0, 0.062)),
        material=label_blue,
        name="cord_snap_notch",
    )
    base.inertial = Inertial.from_geometry(Box((0.62, 0.41, 0.78)), mass=2.6, origin=Origin(xyz=(0.0, 0.0, 0.33)))

    fan_head = model.part("fan_head")
    fan_head.visual(
        mesh_from_cadquery(_fan_guard_shell(), "integral_fan_guard", tolerance=0.0012),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=shell_plastic,
        name="guard_shell",
    )
    fan_head.visual(
        Cylinder(radius=0.087, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="motor_pod",
    )
    fan_head.visual(
        Cylinder(radius=0.048, length=0.046),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="bearing_boss",
    )
    fan_head.visual(
        Cylinder(radius=0.019, length=0.590),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="tilt_trunnion",
    )
    # Two molded tabs show how front and rear guard halves snap together.
    for idx, y in enumerate((-0.160, 0.160)):
        fan_head.visual(
            Box((0.022, 0.030, 0.050)),
            origin=Origin(xyz=(0.112, y, -0.208)),
            material=label_blue,
            name=f"guard_snap_{idx}",
        )
    fan_head.inertial = Inertial.from_geometry(
        Cylinder(radius=GUARD_RADIUS, length=0.16),
        mass=1.1,
        origin=Origin(xyz=(-0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_fan_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fan_head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        # The fan axis points along local +X.  Using -Y makes positive q tilt
        # the airstream upward while the trunnion remains captured by the yoke.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=TILT_LOWER, upper=TILT_UPPER),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.041,
                5,
                thickness=0.026,
                blade_pitch_deg=27.0,
                blade_sweep_deg=23.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.006),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.032, bore_diameter=0.008),
            ),
            "five_blade_rotor",
        ),
        material=blade_plastic,
        name="rotor_blades",
    )
    rotor.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=fastener_black,
        name="drive_shaft",
    )
    rotor.inertial = Inertial.from_geometry(Cylinder(radius=ROTOR_RADIUS, length=0.030), mass=0.22)

    model.articulation(
        "fan_head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=fan_head,
        child=rotor,
        # Rotate the rotor's local Z spin axis onto the fan-head local X axis.
        origin=Origin(xyz=(0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=85.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fan_head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_fan_head")
    spin = object_model.get_articulation("fan_head_to_rotor")

    ctx.check("main parts present", all(p is not None for p in (base, fan_head, rotor)), "base, fan_head, and rotor are required")
    ctx.check("primary joints present", tilt is not None and spin is not None, "tilt and rotor-spin joints are required")

    # The black tilt trunnion is intentionally represented as a shaft captured in
    # simplified solid yoke lugs/arms rather than cutting actual hidden holes.
    for elem in ("yoke_arm_0", "yoke_arm_1", "yoke_lug_0", "yoke_lug_1"):
        ctx.allow_overlap(
            base,
            fan_head,
            elem_a=elem,
            elem_b="tilt_trunnion",
            reason="The side tilt shaft is intentionally captured through simplified molded yoke/clamp bosses.",
        )
        ctx.expect_overlap(
            base,
            fan_head,
            axes="xyz",
            elem_a=elem,
            elem_b="tilt_trunnion",
            min_overlap=0.008,
            name=f"{elem} captures tilt trunnion",
        )

    ctx.allow_overlap(
        fan_head,
        rotor,
        elem_a="bearing_boss",
        elem_b="drive_shaft",
        reason="The rotating drive shaft is intentionally seated in the molded motor bearing boss.",
    )
    ctx.expect_overlap(
        fan_head,
        rotor,
        axes="x",
        elem_a="bearing_boss",
        elem_b="drive_shaft",
        min_overlap=0.005,
        name="drive shaft remains seated in bearing boss",
    )

    ctx.expect_within(
        rotor,
        fan_head,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="guard_shell",
        margin=0.002,
        name="rotor swept diameter stays inside guard",
    )
    ctx.expect_overlap(
        rotor,
        fan_head,
        axes="x",
        elem_a="rotor_blades",
        elem_b="guard_shell",
        min_overlap=0.010,
        name="rotor is axially nested within the guarded head",
    )

    rest_guard = ctx.part_element_world_aabb(fan_head, elem="guard_shell")
    with ctx.pose({tilt: TILT_UPPER}):
        raised_guard = ctx.part_element_world_aabb(fan_head, elem="guard_shell")
    ctx.check(
        "positive tilt raises front guard edge",
        rest_guard is not None
        and raised_guard is not None
        and float(raised_guard[1][2]) > float(rest_guard[1][2]) + 0.015,
        details=f"rest={rest_guard}, raised={raised_guard}",
    )

    with ctx.pose({spin: math.pi / 2.0}):
        ctx.expect_within(
            rotor,
            fan_head,
            axes="yz",
            inner_elem="rotor_blades",
            outer_elem="guard_shell",
            margin=0.002,
            name="rotor remains guarded while spinning",
        )

    return ctx.report()


object_model = build_object_model()
