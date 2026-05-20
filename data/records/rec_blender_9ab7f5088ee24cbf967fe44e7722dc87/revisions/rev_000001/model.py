from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    graphite = model.material("graphite", rgba=(0.035, 0.038, 0.045, 1.0))
    black_gloss = model.material("black_gloss", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    clear_jug = model.material("clear_jug", rgba=(0.72, 0.92, 1.0, 0.34))
    smoke_plastic = model.material("smoke_plastic", rgba=(0.18, 0.22, 0.26, 0.62))
    soft_seal = model.material("soft_seal", rgba=(0.02, 0.025, 0.03, 1.0))
    blue_valve = model.material("blue_valve", rgba=(0.02, 0.28, 0.65, 1.0))

    base = model.part("base")
    base_shell = LatheGeometry(
        [
            (0.0, 0.0),
            (0.158, 0.0),
            (0.178, 0.018),
            (0.182, 0.115),
            (0.162, 0.155),
            (0.132, 0.185),
            (0.0, 0.185),
        ],
        segments=72,
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_round_housing"),
        material=graphite,
        name="round_housing",
    )
    base.visual(
        Cylinder(radius=0.126, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        material=satin_steel,
        name="top_lock_plate",
    )
    for stop_name, yaw in (
        ("bayonet_stop_0", 0.0),
        ("bayonet_stop_1", 2.0 * pi / 3.0),
        ("bayonet_stop_2", 4.0 * pi / 3.0),
    ):
        base.visual(
            Box((0.060, 0.024, 0.010)),
            origin=Origin(
                xyz=(0.082 * cos(yaw), 0.082 * sin(yaw), 0.218),
                rpy=(0.0, 0.0, yaw + pi / 2.0),
            ),
            material=satin_steel,
            name=stop_name,
        )
    base.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, -0.185, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_gloss,
        name="dial_recess",
    )

    jug = model.part("jug")
    jug_shell = LatheGeometry.from_shell_profiles(
        [
            (0.106, 0.040),
            (0.122, 0.070),
            (0.126, 0.610),
            (0.116, 0.700),
        ],
        [
            (0.072, 0.054),
            (0.102, 0.083),
            (0.107, 0.604),
            (0.094, 0.684),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    jug.visual(
        mesh_from_geometry(jug_shell, "jug_hollow_shell"),
        material=clear_jug,
        name="jug_shell",
    )
    jug.visual(
        Cylinder(radius=0.114, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=smoke_plastic,
        name="twist_collar",
    )
    jug.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=smoke_plastic,
        name="bearing_socket",
    )
    for tab_name, yaw in (
        ("bayonet_tab_0", pi / 3.0),
        ("bayonet_tab_1", pi),
        ("bayonet_tab_2", 5.0 * pi / 3.0),
    ):
        jug.visual(
            Box((0.052, 0.020, 0.014)),
            origin=Origin(
                xyz=(0.087 * cos(yaw), 0.087 * sin(yaw), 0.007),
                rpy=(0.0, 0.0, yaw + pi / 2.0),
            ),
            material=smoke_plastic,
            name=tab_name,
        )
    jug.visual(
        Cylinder(radius=0.018, length=0.430),
        origin=Origin(xyz=(0.0, 0.188, 0.365)),
        material=clear_jug,
        name="handle_grip",
    )
    jug.visual(
        Box((0.060, 0.088, 0.035)),
        origin=Origin(xyz=(0.0, 0.152, 0.555)),
        material=clear_jug,
        name="upper_handle_bridge",
    )
    jug.visual(
        Box((0.060, 0.086, 0.038)),
        origin=Origin(xyz=(0.0, 0.151, 0.185)),
        material=clear_jug,
        name="lower_handle_bridge",
    )
    jug.visual(
        Box((0.034, 0.010, 0.360)),
        origin=Origin(xyz=(-0.109, -0.002, 0.360)),
        material=smoke_plastic,
        name="fill_scale",
    )

    blade = model.part("blade")
    blade.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_steel,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=satin_steel,
        name="drive_shaft",
    )
    for blade_name, yaw in (
        ("blade_wing_0", 0.0),
        ("blade_wing_1", pi / 2.0),
        ("blade_wing_2", pi),
        ("blade_wing_3", 3.0 * pi / 2.0),
    ):
        blade.visual(
            Box((0.070, 0.016, 0.006)),
            origin=Origin(xyz=(0.038 * cos(yaw), 0.038 * sin(yaw), 0.020), rpy=(0.0, 0.18, yaw)),
            material=satin_steel,
            name=blade_name,
        )

    connector_cap = model.part("connector_cap")
    connector_cap.visual(
        Cylinder(radius=0.087, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=soft_seal,
        name="plug",
    )
    connector_cap.visual(
        Cylinder(radius=0.110, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=smoke_plastic,
        name="top_disc",
    )
    connector_cap.visual(
        Cylinder(radius=0.040, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=soft_seal,
        name="cap_stem",
    )
    connector_cap.visual(
        Cylinder(radius=0.094, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=soft_seal,
        name="o_ring",
    )
    connector_cap.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=smoke_plastic,
        name="suction_nozzle",
    )
    connector_cap.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=blue_valve,
        name="vacuum_valve",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.076,
                0.028,
                body_style="skirted",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=22, depth=0.0015),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_dial_knob",
        ),
        material=black_gloss,
        name="knob",
    )

    model.articulation(
        "base_to_jug",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.55),
    )
    model.articulation(
        "jug_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=90.0),
    )
    model.articulation(
        "jug_to_connector_cap",
        ArticulationType.PRISMATIC,
        parent=jug,
        child=connector_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.012),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.190, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    blade = object_model.get_part("blade")
    connector_cap = object_model.get_part("connector_cap")
    dial = object_model.get_part("dial")
    twist = object_model.get_articulation("base_to_jug")
    blade_spin = object_model.get_articulation("jug_to_blade")
    cap_press = object_model.get_articulation("jug_to_connector_cap")

    ctx.allow_overlap(
        connector_cap,
        jug,
        elem_a="o_ring",
        elem_b="jug_shell",
        reason="The soft suction-cap O-ring is intentionally compressed into the jug mouth seal.",
    )

    ctx.expect_gap(
        jug,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="twist_collar",
        negative_elem="bayonet_stop_0",
        name="jug collar seats on the twist lock stops",
    )
    ctx.expect_overlap(
        jug,
        base,
        axes="xy",
        min_overlap=0.12,
        elem_a="twist_collar",
        elem_b="top_lock_plate",
        name="jug collar is centered over the round base",
    )
    ctx.expect_within(
        connector_cap,
        jug,
        axes="xy",
        inner_elem="plug",
        outer_elem="jug_shell",
        margin=0.002,
        name="cap plug stays within the jug mouth footprint",
    )
    ctx.expect_overlap(
        connector_cap,
        jug,
        axes="z",
        min_overlap=0.030,
        elem_a="plug",
        elem_b="jug_shell",
        name="cap plug remains inserted in the jug mouth",
    )
    ctx.expect_contact(
        dial,
        base,
        elem_a="knob",
        elem_b="dial_recess",
        contact_tol=0.002,
        name="front dial is mounted on its recess",
    )
    ctx.check(
        "blade joint is continuous",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"blade joint type is {blade_spin.articulation_type}",
    )

    handle_rest = ctx.part_element_world_aabb(jug, elem="handle_grip")
    with ctx.pose({twist: 0.55}):
        handle_twisted = ctx.part_element_world_aabb(jug, elem="handle_grip")
    ctx.check(
        "jug twist lock rotates the handle around the base",
        handle_rest is not None
        and handle_twisted is not None
        and ((handle_twisted[0][0] + handle_twisted[1][0]) * 0.5)
        < ((handle_rest[0][0] + handle_rest[1][0]) * 0.5) - 0.050,
        details=f"rest={handle_rest}, twisted={handle_twisted}",
    )

    cap_rest = ctx.part_world_position(connector_cap)
    with ctx.pose({cap_press: 0.012}):
        cap_pressed = ctx.part_world_position(connector_cap)
        ctx.expect_overlap(
            connector_cap,
            jug,
            axes="z",
            min_overlap=0.040,
            elem_a="plug",
            elem_b="jug_shell",
            name="pressed cap has deeper retained insertion",
        )
    ctx.check(
        "connector cap moves downward when pressed",
        cap_rest is not None and cap_pressed is not None and cap_pressed[2] < cap_rest[2] - 0.010,
        details=f"rest={cap_rest}, pressed={cap_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
