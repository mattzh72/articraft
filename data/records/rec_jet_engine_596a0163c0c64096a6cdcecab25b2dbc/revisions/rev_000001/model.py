from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _tube_z(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Open tube centered on local Z, authored in meters."""
    outer = cq.Workplane("XY").cylinder(length, outer_radius)
    inner = cq.Workplane("XY").cylinder(length + 0.04, inner_radius)
    return outer.cut(inner)


def _conical_tube_z(
    length: float,
    front_outer_radius: float,
    rear_outer_radius: float,
    front_inner_radius: float,
    rear_inner_radius: float,
) -> cq.Workplane:
    """Open conical tube from z=-length/2 (front) to z=+length/2 (rear)."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-length / 2.0)
        .circle(front_outer_radius)
        .workplane(offset=length)
        .circle(rear_outer_radius)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-length / 2.0 - 0.02)
        .circle(front_inner_radius)
        .workplane(offset=length + 0.04)
        .circle(rear_inner_radius)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _cone_z(length: float, base_radius: float, tip_radius: float = 0.01) -> cq.Workplane:
    """Solid cone centered on local Z with its base at negative Z."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-length / 2.0)
        .circle(base_radius)
        .workplane(offset=length)
        .circle(tip_radius)
        .loft(combine=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="geared_turbofan_display")

    nacelle_paint = model.material("cool_gray_paint", color=(0.78, 0.80, 0.82, 1.0))
    dark_liner = model.material("matte_intake_black", color=(0.025, 0.028, 0.032, 1.0))
    titanium = model.material("brushed_titanium", color=(0.62, 0.65, 0.68, 1.0))
    graphite = model.material("dark_graphite", color=(0.12, 0.13, 0.14, 1.0))
    display_black = model.material("satin_black_display", color=(0.02, 0.02, 0.022, 1.0))
    polished = model.material("polished_spinner", color=(0.86, 0.88, 0.90, 1.0))

    nacelle = model.part("nacelle")

    # The engine centerline is the world X axis: negative X is the intake.
    nacelle.visual(
        mesh_from_cadquery(_tube_z(1.58, 0.82, 0.70), "slim_nacelle_shell", tolerance=0.002),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    nacelle.visual(
        mesh_from_cadquery(_tube_z(1.42, 0.705, 0.680), "dark_intake_liner", tolerance=0.002),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_liner,
        name="intake_liner",
    )
    nacelle.visual(
        mesh_from_cadquery(_tube_z(0.14, 0.87, 0.68), "rounded_intake_lip", tolerance=0.0015),
        origin=Origin(xyz=(-0.79, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_paint,
        name="intake_lip",
    )
    nacelle.visual(
        mesh_from_cadquery(
            _conical_tube_z(0.42, 0.82, 0.58, 0.66, 0.43),
            "tapered_aft_cowl",
            tolerance=0.002,
        ),
        origin=Origin(xyz=(0.82, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_paint,
        name="aft_cowl",
    )

    # Central fixed core and the visible rear cone are carried by stator crosses.
    nacelle.visual(
        Cylinder(radius=0.24, length=0.54),
        origin=Origin(xyz=(-0.03, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="core_barrel",
    )
    nacelle.visual(
        mesh_from_cadquery(_cone_z(0.72, 0.25, 0.055), "rear_core_cone", tolerance=0.0015),
        origin=Origin(xyz=(0.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_core_cone",
    )
    nacelle.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(-0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="gearbox_case",
    )
    nacelle.visual(
        Cylinder(radius=0.090, length=0.12),
        origin=Origin(xyz=(-0.44, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="fan_bearing",
    )
    for x, prefix in [(-0.18, "front"), (0.35, "rear")]:
        nacelle.visual(
            Box((0.060, 1.42, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=titanium,
            name=f"{prefix}_stator_yoke",
        )
        nacelle.visual(
            Box((0.060, 0.045, 1.42)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=titanium,
            name=f"{prefix}_stator_strut",
        )

    # Short display pylon and base, deliberately blended into the lower nacelle.
    nacelle.visual(
        Box((0.34, 0.22, 0.55)),
        origin=Origin(xyz=(-0.03, 0.0, -1.03)),
        material=display_black,
        name="display_pylon",
    )
    nacelle.visual(
        Box((1.05, 0.82, 0.09)),
        origin=Origin(xyz=(-0.03, 0.0, -1.335)),
        material=display_black,
        name="display_base",
    )

    fan = model.part("front_fan")
    fan_geometry = FanRotorGeometry(
        outer_radius=0.650,
        hub_radius=0.165,
        blade_count=18,
        thickness=0.095,
        blade_pitch_deg=34.0,
        blade_sweep_deg=32.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12, tip_clearance=0.010),
        hub=FanRotorHub(style="flat", rear_collar_height=0.022, rear_collar_radius=0.155),
        center=True,
    )
    fan.visual(
        mesh_from_geometry(fan_geometry, "front_fan_stage"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_stage",
    )
    fan.visual(
        mesh_from_cadquery(_cone_z(0.30, 0.175, 0.018), "central_spinner", tolerance=0.001),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=polished,
        name="spinner",
    )
    fan.visual(
        Cylinder(radius=0.055, length=0.32),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="hub_shaft",
    )

    model.articulation(
        "nacelle_to_fan",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan,
        origin=Origin(xyz=(-0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=120.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("front_fan")
    fan_joint = object_model.get_articulation("nacelle_to_fan")

    ctx.allow_overlap(
        nacelle,
        fan,
        elem_a="fan_bearing",
        elem_b="hub_shaft",
        reason="The rotating fan shaft is intentionally captured inside the fixed bearing to keep the rotor centered.",
    )
    ctx.check(
        "front fan has continuous spin joint",
        getattr(fan_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"joint_type={getattr(fan_joint, 'articulation_type', None)}",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="fan_stage",
        outer_elem="intake_liner",
        margin=0.005,
        name="fan stage stays inside the nacelle bore",
    )
    ctx.expect_within(
        fan,
        nacelle,
        axes="yz",
        inner_elem="hub_shaft",
        outer_elem="fan_bearing",
        margin=0.001,
        name="fan shaft is centered in the bearing",
    )
    ctx.expect_overlap(
        fan,
        nacelle,
        axes="x",
        elem_a="hub_shaft",
        elem_b="fan_bearing",
        min_overlap=0.05,
        name="fan shaft remains captured axially",
    )

    rest_pos = ctx.part_world_position(fan)
    with ctx.pose({fan_joint: math.pi / 2.0}):
        ctx.expect_within(
            fan,
            nacelle,
            axes="yz",
            inner_elem="fan_stage",
            outer_elem="intake_liner",
            margin=0.005,
            name="spinning fan remains inside the nacelle bore",
        )
        spun_pos = ctx.part_world_position(fan)
    ctx.check(
        "fan spin does not shift the rotor center",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
