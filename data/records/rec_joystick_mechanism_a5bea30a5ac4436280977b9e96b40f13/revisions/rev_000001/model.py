from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_console_joystick")

    cast_iron = model.material("dark_cast_iron", color=(0.10, 0.105, 0.10, 1.0))
    blackened = model.material("blackened_steel", color=(0.015, 0.016, 0.016, 1.0))
    machined = model.material("brushed_machined_steel", color=(0.55, 0.54, 0.49, 1.0))
    rubber = model.material("matte_black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    screw = model.material("dark_socket_screws", color=(0.035, 0.033, 0.030, 1.0))

    # The base part frame is the common gimbal center.  All fixed pedestal,
    # bearing, and cup geometry is below or around that center so the two
    # revolute joints can share the same origin.
    base = model.part("base_cup")

    cup_profile = [
        (0.000, -0.096),
        (0.116, -0.096),
        (0.123, -0.088),
        (0.112, -0.058),
        (0.104, -0.054),
        (0.094, -0.054),
        (0.083, -0.084),
        (0.000, -0.084),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(cup_profile, segments=96), "shallow_base_cup"),
        material=cast_iron,
        name="hollow_cup",
    )
    base.visual(
        Cylinder(radius=0.136, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.101)),
        material=blackened,
        name="panel_flange",
    )
    base.visual(
        Cylinder(radius=0.103, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=cast_iron,
        name="rolled_lip",
    )

    # Four flush socket-head fasteners make the cup read as bolted into a panel.
    for index, (x, y) in enumerate(
        ((0.090, 0.090), (-0.090, 0.090), (-0.090, -0.090), (0.090, -0.090))
    ):
        base.visual(
            Cylinder(radius=0.0065, length=0.010),
            origin=Origin(xyz=(x, y, -0.094)),
            material=screw,
            name=f"socket_screw_{index}",
        )

    # Left-to-right side bearing pedestals.  The torus visuals are real open
    # bushings around the outer-ring trunnion axis rather than solid blocks that
    # the pivot would clip through.
    for side, x, pedestal_name, bearing_mesh, bushing_name, boss_name in (
        ("side_0", -0.104, "side_0_pedestal", "side_0_bearing", "side_0_bushing", "side_0_boss_web"),
        ("side_1", 0.104, "side_1_pedestal", "side_1_bearing", "side_1_bushing", "side_1_boss_web"),
    ):
        base.visual(
            Box((0.025, 0.052, 0.060)),
            origin=Origin(xyz=(x, 0.0, -0.049)),
            material=cast_iron,
            name=pedestal_name,
        )
        base.visual(
            mesh_from_geometry(TorusGeometry(radius=0.014, tube=0.0075), bearing_mesh),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name=bushing_name,
        )
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x * 0.995, 0.0, -0.037), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=boss_name,
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(TorusGeometry(radius=0.064, tube=0.008), "outer_gimbal_ring"),
        material=cast_iron,
        name="main_ring",
    )
    # Trunnion pins extend through the two base bushings with visible clearance.
    for side, x, pin_name, collar_name in (
        ("side_0", -0.087, "side_0_trunnion_pin", "side_0_cast_collar"),
        ("side_1", 0.087, "side_1_trunnion_pin", "side_1_cast_collar"),
    ):
        outer_ring.visual(
            Cylinder(radius=0.0068, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=machined,
            name=pin_name,
        )
        outer_ring.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(x * 0.78, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cast_iron,
            name=collar_name,
        )

    # Inner-cradle bearing lugs are tucked inside the outer ring and connected by
    # paired cast webs, leaving a clean front-to-back bore for the cradle axle.
    for face, y, bearing_mesh, bushing_name in (
        ("front", 0.044, "front_inner_bearing", "front_inner_bushing"),
        ("rear", -0.044, "rear_inner_bearing", "rear_inner_bushing"),
    ):
        outer_ring.visual(
            mesh_from_geometry(TorusGeometry(radius=0.0105, tube=0.006), bearing_mesh),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=bushing_name,
        )
        bridge_y = y + (0.010 if y > 0.0 else -0.010)
        for side, x in (("web_0", -0.012), ("web_1", 0.012)):
            outer_ring.visual(
                Box((0.010, 0.026, 0.012)),
                origin=Origin(xyz=(x, bridge_y, 0.0)),
                material=cast_iron,
                name=f"{face}_{side}",
            )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.0048, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="trunnion_axle",
    )
    inner_cradle.visual(
        Cylinder(radius=0.010, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="cradle_barrel",
    )
    for face, y in (("front", 0.036), ("rear", -0.036)):
        inner_cradle.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"{face}_shoulder",
        )
    inner_cradle.visual(
        Cylinder(radius=0.018, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_iron,
        name="socket_hub",
    )
    inner_cradle.visual(
        Cylinder(radius=0.012, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        material=machined,
        name="stick_seat",
    )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=0.006, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=rubber,
        name="short_shaft",
    )
    stick.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.026,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="knurled", count=42, depth=0.0009, helix_angle_deg=22.0),
            ),
            "knurled_stick_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=machined,
        name="knurled_collar",
    )
    stick.visual(
        Cylinder(radius=0.010, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.1565)),
        material=machined,
        name="collar_cap",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.31, upper=0.31),
    )
    model.articulation(
        "outer_to_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.31, upper=0.31),
    )
    model.articulation(
        "cradle_to_stick",
        ArticulationType.FIXED,
        parent=inner_cradle,
        child=stick,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_cup")
    outer = object_model.get_part("outer_ring")
    cradle = object_model.get_part("inner_cradle")
    stick = object_model.get_part("stick")
    base_to_outer = object_model.get_articulation("base_to_outer")
    outer_to_cradle = object_model.get_articulation("outer_to_cradle")

    for bushing_name, pin_name in (
        ("side_0_bushing", "side_0_trunnion_pin"),
        ("side_1_bushing", "side_1_trunnion_pin"),
    ):
        ctx.allow_overlap(
            base,
            outer,
            elem_a=bushing_name,
            elem_b=pin_name,
            reason="The machined trunnion pin is intentionally captured with a light bearing fit inside the side bushing.",
        )
        ctx.expect_within(
            outer,
            base,
            axes="yz",
            inner_elem=pin_name,
            outer_elem=bushing_name,
            margin=0.001,
            name=f"{pin_name} stays centered in bearing",
        )
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a=pin_name,
            elem_b=bushing_name,
            min_overlap=0.006,
            name=f"{pin_name} is retained axially",
        )

    for bushing_name in ("front_inner_bushing", "rear_inner_bushing"):
        ctx.allow_overlap(
            outer,
            cradle,
            elem_a=bushing_name,
            elem_b="trunnion_axle",
            reason="The inner trunnion axle is intentionally modeled as a close running fit through the cradle bearing bushing.",
        )
        ctx.expect_within(
            cradle,
            outer,
            axes="xz",
            inner_elem="trunnion_axle",
            outer_elem=bushing_name,
            margin=0.001,
            name=f"inner axle is centered in {bushing_name}",
        )
        ctx.expect_overlap(
            cradle,
            outer,
            axes="y",
            elem_a="trunnion_axle",
            elem_b=bushing_name,
            min_overlap=0.006,
            name=f"inner axle is retained through {bushing_name}",
        )

    ctx.expect_overlap(
        outer,
        base,
        axes="x",
        elem_a="side_1_trunnion_pin",
        elem_b="side_1_bushing",
        min_overlap=0.006,
        name="outer trunnion is retained in side bushing",
    )
    ctx.expect_within(
        cradle,
        outer,
        axes="xz",
        inner_elem="trunnion_axle",
        outer_elem="front_inner_bushing",
        margin=0.002,
        name="inner axle fits front bearing bore",
    )
    ctx.expect_gap(
        stick,
        cradle,
        axis="z",
        positive_elem="short_shaft",
        negative_elem="stick_seat",
        max_gap=0.002,
        max_penetration=0.006,
        name="stick seats on machined socket",
    )

    neutral_tip = ctx.part_element_world_aabb(stick, elem="collar_cap")
    with ctx.pose({base_to_outer: 0.31}):
        tipped_tip = ctx.part_element_world_aabb(stick, elem="collar_cap")
    ctx.check(
        "outer ring tilt moves stick sideways without lowering into cup",
        neutral_tip is not None
        and tipped_tip is not None
        and tipped_tip[0][1] < neutral_tip[0][1] - 0.035
        and tipped_tip[0][0] > -0.050
        and tipped_tip[0][2] > -0.005,
        details=f"neutral={neutral_tip}, tipped={tipped_tip}",
    )

    with ctx.pose({outer_to_cradle: -0.31}):
        tilted_tip = ctx.part_element_world_aabb(stick, elem="collar_cap")
    ctx.check(
        "inner cradle tilt moves stick about orthogonal axis",
        neutral_tip is not None
        and tilted_tip is not None
        and tilted_tip[0][0] < neutral_tip[0][0] - 0.035
        and tilted_tip[0][2] > -0.005,
        details=f"neutral={neutral_tip}, tilted={tilted_tip}",
    )

    return ctx.report()


object_model = build_object_model()
