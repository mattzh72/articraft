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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    concrete = Material("weathered_concrete", rgba=(0.55, 0.56, 0.52, 1.0))
    tower_paint = Material("warm_tower_white", rgba=(0.84, 0.87, 0.84, 1.0))
    nacelle_paint = Material("smooth_nacelle_white", rgba=(0.92, 0.95, 0.94, 1.0))
    blade_paint = Material("blade_off_white", rgba=(0.96, 0.96, 0.91, 1.0))
    dark_metal = Material("dark_bearing_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    satin_metal = Material("satin_galvanized", rgba=(0.52, 0.55, 0.57, 1.0))
    hatch_gray = Material("flush_service_hatch", rgba=(0.36, 0.40, 0.42, 1.0))

    # Root support module: concrete pad plus a visibly tapered steel tower.
    base = model.part("base")
    base.visual(
        Cylinder(radius=1.20, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="foundation_pad",
    )
    tower_profile = [
        (0.0, 0.25),
        (0.46, 0.25),
        (0.40, 1.50),
        (0.31, 4.50),
        (0.22, 8.00),
        (0.0, 8.00),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(tower_profile, segments=48), "tapered_tower"),
        material=tower_paint,
        name="tapered_tower",
    )
    base.visual(
        Cylinder(radius=0.58, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=satin_metal,
        name="tower_base_flange",
    )

    # Rotating yaw-bearing module.  Its child frame is the vertical yaw axis at
    # the top of the tower, so all visible bearing pieces are above local z=0.
    bearing = model.part("bearing_module")
    bearing.visual(
        Cylinder(radius=0.54, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=satin_metal,
        name="lower_flange",
    )
    bearing.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.40, tube=0.045, radial_segments=48, tubular_segments=12),
            "yaw_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="bearing_ring",
    )
    bearing.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=satin_metal,
        name="upper_flange",
    )
    bearing.visual(
        Cylinder(radius=0.24, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=satin_metal,
        name="neck",
    )

    # Head module: a rounded nacelle, pedestal, service panels, and front shaft
    # bearing.  It is fixed to the yaw-bearing module so the whole head yaws.
    head = model.part("head")
    nacelle_shell = (
        cq.Workplane("XY")
        .box(2.10, 0.70, 0.62)
        .edges()
        .fillet(0.07)
        .translate((0.38, 0.0, 0.55))
    )
    head.visual(
        mesh_from_cadquery(nacelle_shell, "rounded_nacelle", tolerance=0.002),
        material=nacelle_paint,
        name="nacelle_shell",
    )
    head.visual(
        Cylinder(radius=0.23, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=satin_metal,
        name="yaw_pedestal",
    )
    head.visual(
        Cylinder(radius=0.30, length=0.347196),
        origin=Origin(xyz=(1.58, 0.0, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="front_bearing",
    )
    head.visual(
        Box(size=(0.58, 0.012, 0.24)),
        origin=Origin(xyz=(-0.12, 0.356, 0.55)),
        material=hatch_gray,
        name="side_hatch_0",
    )
    head.visual(
        Box(size=(0.58, 0.012, 0.24)),
        origin=Origin(xyz=(-0.12, -0.356, 0.55)),
        material=hatch_gray,
        name="side_hatch_1",
    )

    # Rotor module: a three-blade horizontal-axis rotor.  The fan-rotor helper
    # spins about local +Z; rotate it so local +Z is the turbine main shaft +X.
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.003598, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hub_collar",
    )
    rotor_mesh = FanRotorGeometry(
        outer_radius=3.00,
        hub_radius=0.38,
        blade_count=3,
        thickness=0.18,
        blade_pitch_deg=18.0,
        blade_sweep_deg=6.0,
        blade_root_chord=0.54,
        blade_tip_chord=0.18,
        blade=FanRotorBlade(shape="narrow", tip_pitch_deg=7.0, camber=0.08),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.08, rear_collar_radius=0.30),
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "three_blade_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_paint,
        name="rotor_blades",
    )

    model.articulation(
        "yaw_bearing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bearing,
        origin=Origin(xyz=(0.0, 0.0, 8.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.25, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "bearing_to_head",
        ArticulationType.FIXED,
        parent=bearing,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )
    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(1.84, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    bearing = object_model.get_part("bearing_module")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("yaw_bearing")
    shaft = object_model.get_articulation("shaft_spin")

    ctx.check(
        "support is split into base bearing and head modules",
        base is not None and bearing is not None and head is not None and rotor is not None,
    )
    ctx.check(
        "rotor joint is continuous about main shaft",
        shaft.articulation_type == ArticulationType.CONTINUOUS
        and all(abs(a - b) < 1e-9 for a, b in zip(shaft.axis, (1.0, 0.0, 0.0))),
    )
    ctx.check(
        "yaw joint is vertical revolute bearing",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and all(abs(a - b) < 1e-9 for a, b in zip(yaw.axis, (0.0, 0.0, 1.0))),
    )

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_flange",
        negative_elem="tapered_tower",
        name="yaw bearing sits on tower top",
    )
    ctx.expect_gap(
        head,
        bearing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="yaw_pedestal",
        negative_elem="neck",
        name="head pedestal sits on bearing neck",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="hub_collar",
        negative_elem="front_bearing",
        name="rotor hub collar seats on shaft bearing",
    )

    rotor_rest = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.pi / 2.0}):
        rotor_yawed = ctx.part_world_position(rotor)
    ctx.check(
        "yaw rotates nacelle and rotor around tower axis",
        rotor_rest is not None
        and rotor_yawed is not None
        and rotor_rest[0] > 1.5
        and abs(rotor_rest[1]) < 0.01
        and abs(rotor_yawed[0]) < 0.01
        and rotor_yawed[1] > 1.5,
        details=f"rest={rotor_rest}, yawed={rotor_yawed}",
    )

    spin_rest = ctx.part_world_position(rotor)
    with ctx.pose({shaft: 1.2}):
        spin_pose = ctx.part_world_position(rotor)
    ctx.check(
        "rotor spins about fixed hub center",
        spin_rest is not None
        and spin_pose is not None
        and sum((a - b) ** 2 for a, b in zip(spin_rest, spin_pose)) < 1e-10,
        details=f"rest={spin_rest}, spun={spin_pose}",
    )

    return ctx.report()


object_model = build_object_model()
