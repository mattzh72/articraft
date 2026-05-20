from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 72,
) -> MeshGeometry:
    """A lathed annular band with a real center opening."""
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _make_blade_wings() -> MeshGeometry:
    """Four stainless blender blades connected to a central hub."""
    geom = MeshGeometry()
    geom.merge(CylinderGeometry(radius=0.018, height=0.014, radial_segments=40).translate(0.0, 0.0, 0.006))
    geom.merge(TorusGeometry(radius=0.018, tube=0.0022, radial_segments=10, tubular_segments=40).translate(0.0, 0.0, 0.013))
    for index in range(4):
        angle = index * math.pi / 2.0
        pitch = math.radians(13.0 if index % 2 == 0 else -13.0)
        blade = (
            # The root overlaps the hub slightly, so the four blades read as one
            # bolted cutter assembly rather than four floating strips.
            BoxGeometry((0.058, 0.014, 0.004))
            .rotate_y(pitch)
            .translate(0.041, 0.0, 0.010 + 0.002 * (index % 2))
            .rotate_z(angle)
        )
        geom.merge(blade)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_soup_blender")

    glossy_plastic = model.material("glossy_plastic", rgba=(0.12, 0.18, 0.24, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.015, 0.018, 0.020, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("tempered_glass", rgba=(0.70, 0.92, 1.0, 0.34))
    frosted_glass = model.material("thick_glass_edges", rgba=(0.78, 0.96, 1.0, 0.48))
    stainless = model.material("brushed_stainless", rgba=(0.76, 0.78, 0.78, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.54, 0.56, 0.58, 1.0))
    white_marking = model.material("white_marking", rgba=(0.94, 0.96, 0.92, 1.0))

    base = model.part("base")
    base_body = LatheGeometry(
        [
            (0.0, 0.000),
            (0.138, 0.000),
            (0.166, 0.018),
            (0.178, 0.050),
            (0.164, 0.092),
            (0.130, 0.126),
            (0.096, 0.138),
            (0.0, 0.138),
        ],
        segments=88,
    )
    base.visual(
        mesh_from_geometry(base_body, "wide_round_base"),
        material=glossy_plastic,
        name="wide_round_base",
    )
    base.visual(
        mesh_from_geometry(_ring_band(outer_radius=0.124, inner_radius=0.084, z_min=0.128, z_max=0.154), "coupling_ring"),
        material=satin_metal,
        name="coupling_ring",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=rubber,
        name="drive_gasket",
    )
    # Three bayonet receiver ramps on the base coupling.
    for index in range(3):
        angle = index * math.tau / 3.0 + math.radians(17.0)
        base.visual(
            Box((0.050, 0.017, 0.012)),
            origin=Origin(
                xyz=(0.104 * math.cos(angle), 0.104 * math.sin(angle), 0.148),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=satin_metal,
            name=f"bayonet_ramp_{index}",
        )
    base.visual(
        Box((0.128, 0.024, 0.074)),
        origin=Origin(xyz=(0.0, -0.174, 0.075)),
        material=dark_plastic,
        name="front_panel",
    )

    jar = model.part("jar")
    jar_shell = LatheGeometry.from_shell_profiles(
        [
            (0.070, 0.000),
            (0.083, 0.026),
            (0.099, 0.145),
            (0.108, 0.315),
            (0.116, 0.405),
            (0.116, 0.430),
        ],
        [
            (0.056, 0.006),
            (0.068, 0.034),
            (0.088, 0.150),
            (0.098, 0.315),
            (0.107, 0.402),
            (0.108, 0.424),
        ],
        segments=88,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )
    jar.visual(
        mesh_from_geometry(jar_shell, "wide_mouth_glass_jar"),
        material=glass,
        name="wide_mouth_glass_jar",
    )
    jar.visual(
        mesh_from_geometry(_ring_band(outer_radius=0.092, inner_radius=0.062, z_min=0.000, z_max=0.048), "jar_lock_collar"),
        material=dark_plastic,
        name="jar_lock_collar",
    )
    for index in range(3):
        angle = index * math.tau / 3.0
        jar.visual(
            Box((0.040, 0.015, 0.014)),
            origin=Origin(
                xyz=(0.099 * math.cos(angle), 0.099 * math.sin(angle), 0.030),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark_plastic,
            name=f"bayonet_tab_{index}",
        )
    jar.visual(
        mesh_from_geometry(_ring_band(outer_radius=0.023, inner_radius=0.010, z_min=0.012, z_max=0.038), "blade_bearing"),
        material=stainless,
        name="blade_bearing",
    )
    for index in range(4):
        angle = index * math.pi / 2.0
        jar.visual(
            Box((0.045, 0.007, 0.008)),
            origin=Origin(
                xyz=(0.041 * math.cos(angle), 0.041 * math.sin(angle), 0.025),
                rpy=(0.0, 0.0, angle),
            ),
            material=stainless,
            name=f"bearing_spoke_{index}",
        )
    jar.visual(
        mesh_from_geometry(TorusGeometry(radius=0.115, tube=0.006, radial_segments=12, tubular_segments=88).translate(0.0, 0.0, 0.427), "thick_pouring_lip"),
        material=frosted_glass,
        name="thick_pouring_lip",
    )
    for index, z in enumerate((0.160, 0.205, 0.250, 0.295, 0.340)):
        # Raised measurement marks on the front of the transparent jar.
        jar.visual(
            Box((0.036 if index % 2 == 0 else 0.023, 0.006, 0.004)),
            origin=Origin(xyz=(-0.035, -(0.095 + 0.0025 * index), z)),
            material=white_marking,
            name=f"measure_mark_{index}",
        )

    lid_ring = _ring_band(outer_radius=0.121, inner_radius=0.038, z_min=0.426, z_max=0.450)
    lid_ring.merge(TorusGeometry(radius=0.118, tube=0.004, radial_segments=10, tubular_segments=80).translate(0.0, 0.0, 0.450))
    jar.visual(
        mesh_from_geometry(lid_ring, "vented_lid"),
        material=rubber,
        name="vented_lid",
    )
    jar.visual(
        Box((0.014, 0.012, 0.014)),
        origin=Origin(xyz=(-0.034, 0.031, 0.456)),
        material=rubber,
        name="hinge_support_0",
    )
    jar.visual(
        Box((0.014, 0.012, 0.014)),
        origin=Origin(xyz=(-0.034, -0.031, 0.456)),
        material=rubber,
        name="hinge_support_1",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=stainless,
        name="blade_shaft",
    )
    blade.visual(
        mesh_from_geometry(_make_blade_wings(), "four_blade_assembly"),
        material=stainless,
        name="four_blade_assembly",
    )

    steam_cap = model.part("steam_cap")
    steam_cap.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.045, 0.0, 0.006)),
        material=rubber,
        name="vent_cap_disc",
    )
    steam_cap.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.007), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="hinge_barrel",
    )
    steam_cap.visual(
        Box((0.020, 0.020, 0.006)),
        origin=Origin(xyz=(0.086, 0.0, 0.012)),
        material=rubber,
        name="lift_tab",
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.056,
            0.027,
            body_style="skirted",
            top_diameter=0.043,
            edge_radius=0.001,
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "front_speed_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_speed_dial",
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=math.radians(28.0)),
    )
    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=80.0),
    )
    model.articulation(
        "jar_to_steam_cap",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=steam_cap,
        # The cap disc extends along +X from a tangent hinge line.
        # -Y makes positive q lift the free edge up and away from the steam vent.
        origin=Origin(xyz=(-0.034, 0.0, 0.449)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.0, lower=0.0, upper=math.radians(72.0)),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.186, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=math.radians(-130.0), upper=math.radians(130.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade = object_model.get_part("blade")
    steam_cap = object_model.get_part("steam_cap")
    jar_twist = object_model.get_articulation("base_to_jar")
    cap_hinge = object_model.get_articulation("jar_to_steam_cap")

    ctx.expect_contact(
        jar,
        base,
        elem_a="jar_lock_collar",
        elem_b="coupling_ring",
        contact_tol=0.0015,
        name="jar collar seats on bayonet coupling",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a="jar_lock_collar",
        elem_b="coupling_ring",
        min_overlap=0.006,
        name="twist lock collars overlap in plan",
    )
    ctx.expect_within(
        blade,
        jar,
        axes="xy",
        inner_elem="blade_shaft",
        outer_elem="blade_bearing",
        margin=0.001,
        name="blade shaft is centered in bearing",
    )
    ctx.expect_overlap(
        blade,
        jar,
        axes="z",
        elem_a="blade_shaft",
        elem_b="blade_bearing",
        min_overlap=0.014,
        name="blade shaft remains captured by bearing height",
    )

    closed_aabb = ctx.part_world_aabb(steam_cap)
    closed_mark = ctx.part_element_world_aabb(jar, elem="measure_mark_0")
    with ctx.pose({cap_hinge: math.radians(60.0), jar_twist: math.radians(18.0)}):
        open_aabb = ctx.part_world_aabb(steam_cap)
        twisted_mark = ctx.part_element_world_aabb(jar, elem="measure_mark_0")
        ctx.check(
            "steam cap hinge lifts vent",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.025,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        ctx.check(
            "jar twists on bayonet lock",
            closed_mark is not None
            and twisted_mark is not None
            and abs(((twisted_mark[0][0] + twisted_mark[1][0]) * 0.5) - ((closed_mark[0][0] + closed_mark[1][0]) * 0.5)) > 0.020,
            details=f"closed={closed_mark}, twisted={twisted_mark}",
        )

    return ctx.report()


object_model = build_object_model()
