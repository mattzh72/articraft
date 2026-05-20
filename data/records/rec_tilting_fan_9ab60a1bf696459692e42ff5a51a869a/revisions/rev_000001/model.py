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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


TILT_HEIGHT = 0.55
GUARD_RADIUS = 0.182
ROTOR_RADIUS = 0.145
FRONT_GUARD_Y = 0.070
REAR_GUARD_Y = -0.055
CAGE_CENTER_Y = (FRONT_GUARD_Y + REAR_GUARD_Y) * 0.5
CAGE_DEPTH = FRONT_GUARD_Y - REAR_GUARD_Y + 0.014


def _rounded_base() -> cq.Workplane:
    """A broad, heavy rounded-rectangle desk-fan foot."""
    return (
        cq.Workplane("XY")
        .box(0.48, 0.32, 0.055)
        .edges("|Z")
        .fillet(0.055)
        .edges(">Z")
        .fillet(0.010)
    )


def _add_guard_ring(part, *, radius: float, y: float, name: str, material) -> None:
    part.visual(
        mesh_from_geometry(
            TorusGeometry(radius=radius, tube=0.0055, radial_segments=18, tubular_segments=72),
            name,
        ),
        origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_guard_spokes(part, *, y: float, prefix: str, material) -> None:
    # Eight broad rectangular spokes make the guard read as a sturdy wire cage
    # without fragmenting it into a mass of tiny unsupported wires.
    for index in range(8):
        angle = index * math.pi / 8.0
        part.visual(
            Box((GUARD_RADIUS * 2.02, 0.0075, 0.0065)),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, -angle, 0.0)),
            material=material,
            name=f"{prefix}_spoke_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    base_finish = model.material("satin_black_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    support_finish = model.material("dark_graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    cage_finish = model.material("brushed_wire_guard", rgba=(0.62, 0.65, 0.67, 1.0))
    motor_finish = model.material("rear_motor_black", rgba=(0.055, 0.055, 0.060, 1.0))
    blade_finish = model.material("smoky_translucent_blades", rgba=(0.45, 0.62, 0.78, 0.72))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_base(), "rounded_base", tolerance=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=base_finish,
        name="rounded_base",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=support_finish,
        name="pedestal",
    )
    base.visual(
        Box((0.54, 0.105, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=support_finish,
        name="yoke_bridge",
    )
    for index, x in enumerate((-0.265, 0.265)):
        base.visual(
            Box((0.050, 0.105, 0.300)),
            origin=Origin(xyz=(x, 0.0, 0.450)),
            material=support_finish,
            name=f"side_bracket_{index}",
        )
        base.visual(
            Cylinder(radius=0.056, length=0.050),
            origin=Origin(xyz=(math.copysign(0.220, x), 0.0, TILT_HEIGHT), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=support_finish,
            name=f"bearing_{index}",
        )

    head = model.part("head")
    # Rear motor shell and necked nose, fixed to the tilting head frame.
    head.visual(
        Cylinder(radius=0.086, length=0.120),
        origin=Origin(xyz=(0.0, -0.104, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_finish,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.046, length=0.045),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=motor_finish,
        name="hub_nose",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.090),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=support_finish,
        name="axle_stub",
    )
    for y, prefix in ((FRONT_GUARD_Y, "front_guard"), (REAR_GUARD_Y, "rear_guard")):
        for radius in (0.068, 0.122, GUARD_RADIUS):
            _add_guard_ring(head, radius=radius, y=y, name=f"{prefix}_ring_{round(radius * 1000)}", material=cage_finish)
        _add_guard_spokes(head, y=y, prefix=prefix, material=cage_finish)
    head.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.0, FRONT_GUARD_Y + 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cage_finish,
        name="front_badge",
    )
    # Four deep cage ties make the front and rear guards one rigid head cage.
    head.visual(
        Box((0.045, CAGE_DEPTH, 0.012)),
        origin=Origin(xyz=(0.0, CAGE_CENTER_Y, GUARD_RADIUS)),
        material=cage_finish,
        name="top_cage_tie",
    )
    head.visual(
        Box((0.045, CAGE_DEPTH, 0.012)),
        origin=Origin(xyz=(0.0, CAGE_CENTER_Y, -GUARD_RADIUS)),
        material=cage_finish,
        name="bottom_cage_tie",
    )
    head.visual(
        Box((0.012, CAGE_DEPTH, 0.045)),
        origin=Origin(xyz=(-GUARD_RADIUS, CAGE_CENTER_Y, 0.0)),
        material=cage_finish,
        name="side_cage_tie_0",
    )
    head.visual(
        Box((0.012, CAGE_DEPTH, 0.045)),
        origin=Origin(xyz=(GUARD_RADIUS, CAGE_CENTER_Y, 0.0)),
        material=cage_finish,
        name="side_cage_tie_1",
    )
    for index, x in enumerate((-0.188, 0.188)):
        head.visual(
            Cylinder(radius=0.025, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=support_finish,
            name=f"tilt_pin_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.036,
                5,
                thickness=0.030,
                blade_pitch_deg=29.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=15.0, camber=0.12, tip_clearance=0.004),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.030, bore_diameter=0.012),
            ),
            "rotor_blades",
        ),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade_finish,
        name="rotor_blades",
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TILT_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    head_tilt = object_model.get_articulation("head_tilt")
    rotor_spin = object_model.get_articulation("rotor_spin")

    for index in (0, 1):
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"bearing_{index}",
            elem_b=f"tilt_pin_{index}",
            reason="The tilting head pins are intentionally seated inside the broad side-bracket bearing bosses.",
        )
        ctx.expect_within(
            head,
            base,
            axes="yz",
            inner_elem=f"tilt_pin_{index}",
            outer_elem=f"bearing_{index}",
            margin=0.002,
            name=f"tilt pin {index} is captured in bearing",
        )
        ctx.expect_overlap(
            base,
            head,
            axes="x",
            elem_a=f"bearing_{index}",
            elem_b=f"tilt_pin_{index}",
            min_overlap=0.006,
            name=f"tilt pin {index} has bearing insertion",
        )

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="axle_stub",
        elem_b="rotor_blades",
        reason="The rotor hub is intentionally captured on a small central axle stub so it can spin about the motor shaft.",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="xz",
        inner_elem="axle_stub",
        outer_elem="rotor_blades",
        margin=0.001,
        name="axle stub is centered in rotor hub",
    )
    ctx.expect_overlap(
        head,
        rotor,
        axes="y",
        elem_a="axle_stub",
        elem_b="rotor_blades",
        min_overlap=0.025,
        name="rotor hub remains engaged on axle",
    )

    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="front_guard_ring_182",
        margin=0.020,
        name="rotor fits inside circular guard envelope",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="y",
        positive_elem="front_guard_ring_182",
        negative_elem="rotor_blades",
        min_gap=0.020,
        name="front guard clears spinning rotor",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="y",
        positive_elem="rotor_blades",
        negative_elem="rear_guard_ring_182",
        min_gap=0.020,
        name="rear guard clears spinning rotor",
    )

    ctx.check(
        "rotor joint is continuous",
        getattr(rotor_spin, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"rotor_spin type={getattr(rotor_spin, 'articulation_type', None)!r}",
    )

    rest_front = ctx.part_element_world_aabb(head, elem="front_guard_ring_182")
    with ctx.pose({head_tilt: 0.50}):
        tilted_front = ctx.part_element_world_aabb(head, elem="front_guard_ring_182")
    if rest_front is not None and tilted_front is not None:
        rest_center_z = (rest_front[0][2] + rest_front[1][2]) * 0.5
        tilted_center_z = (tilted_front[0][2] + tilted_front[1][2]) * 0.5
        ctx.check(
            "positive tilt raises fan face",
            tilted_center_z > rest_center_z + 0.020,
            details=f"rest_z={rest_center_z:.3f}, tilted_z={tilted_center_z:.3f}",
        )
    else:
        ctx.fail("positive tilt raises fan face", "Could not read front guard AABBs.")

    return ctx.report()


object_model = build_object_model()
