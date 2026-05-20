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
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, depth: float):
    """CadQuery annular plate centered on local Z, before visual placement."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .translate((0.0, 0.0, -depth / 2.0))
    )
    return mesh_from_cadquery(
        ring,
        name,
        tolerance=0.0007,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tilting_fan")

    matte_graphite = model.material("matte_graphite", rgba=(0.055, 0.058, 0.060, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.115, 0.118, 0.120, 1.0))
    satin_warm_metal = model.material("satin_warm_metal", rgba=(0.64, 0.60, 0.52, 1.0))
    brushed_dark_metal = model.material("brushed_dark_metal", rgba=(0.30, 0.31, 0.31, 1.0))
    soft_black = model.material("soft_black", rgba=(0.012, 0.013, 0.014, 1.0))
    satin_blade = model.material("satin_blade", rgba=(0.74, 0.76, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    marker_white = model.material("marker_white", rgba=(0.86, 0.84, 0.78, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.275, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=matte_graphite,
        name="weighted_base",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=satin_graphite,
        name="raised_base_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.278, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=satin_warm_metal,
        name="base_seam_band",
    )
    pedestal.visual(
        Cylinder(radius=0.033, length=0.445),
        origin=Origin(xyz=(0.0, 0.0, 0.2825)),
        material=satin_warm_metal,
        name="upright_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=brushed_dark_metal,
        name="lower_stem_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=brushed_dark_metal,
        name="upper_stem_collar",
    )
    pedestal.visual(
        Box((0.205, 0.112, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.522)),
        material=brushed_dark_metal,
        name="yoke_saddle",
    )
    pedestal.visual(
        Box((0.660, 0.128, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.5175)),
        material=satin_warm_metal,
        name="yoke_crossbar",
    )
    for sx in (-1.0, 1.0):
        side_name = "neg" if sx < 0 else "pos"
        pedestal.visual(
            Box((0.060, 0.116, 0.277)),
            origin=Origin(xyz=(sx * 0.306, 0.0, 0.6735)),
            material=satin_warm_metal,
            name=f"yoke_cheek_{side_name}",
        )
        pedestal.visual(
            Box((0.060, 0.116, 0.020)),
            origin=Origin(xyz=(sx * 0.306, 0.0, 0.890)),
            material=satin_warm_metal,
            name=f"yoke_cap_{side_name}",
        )
        pedestal.visual(
            Box((0.060, 0.026, 0.108)),
            origin=Origin(xyz=(sx * 0.306, -0.050, 0.846)),
            material=satin_warm_metal,
            name=f"yoke_front_lug_{side_name}",
        )
        pedestal.visual(
            Box((0.060, 0.026, 0.108)),
            origin=Origin(xyz=(sx * 0.306, 0.050, 0.846)),
            material=satin_warm_metal,
            name=f"yoke_rear_lug_{side_name}",
        )
    for sx in (-1.0, 1.0):
        pedestal.visual(
            _annulus_mesh("pivot_bearing_neg" if sx < 0 else "pivot_bearing_pos", 0.045, 0.0265, 0.012),
            origin=Origin(xyz=(sx * 0.336, 0.0, 0.850), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_dark_metal,
            name="pivot_bearing_neg" if sx < 0 else "pivot_bearing_pos",
        )
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                pedestal.visual(
                    Cylinder(radius=0.006, length=0.004),
                    origin=Origin(
                        xyz=(sx * 0.331, sy * 0.039, 0.850 + sz * 0.040),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=brushed_dark_metal,
                    name=f"cheek_screw_{'neg' if sx < 0 else 'pos'}_{'rear' if sy > 0 else 'front'}_{'upper' if sz > 0 else 'lower'}",
                )
    for i, (x, y) in enumerate(((0.180, 0.180), (-0.180, 0.180), (0.180, -0.180), (-0.180, -0.180))):
        pedestal.visual(
            Cylinder(radius=0.036, length=0.010),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    fan_head = model.part("fan_head")
    ring_specs = (
        ("front_outer_guard", -0.058, 0.252, 0.232, 0.012, satin_warm_metal),
        ("front_ring_0_guard", -0.058, 0.204, 0.198, 0.006, brushed_dark_metal),
        ("front_ring_1_guard", -0.058, 0.158, 0.152, 0.006, brushed_dark_metal),
        ("front_ring_2_guard", -0.058, 0.111, 0.105, 0.006, brushed_dark_metal),
        ("rear_outer_guard", 0.058, 0.252, 0.232, 0.012, satin_warm_metal),
        ("rear_ring_0_guard", 0.058, 0.204, 0.198, 0.006, brushed_dark_metal),
        ("rear_ring_1_guard", 0.058, 0.158, 0.152, 0.006, brushed_dark_metal),
        ("rear_ring_2_guard", 0.058, 0.111, 0.105, 0.006, brushed_dark_metal),
    )
    for guard_name, y, outer, inner, depth, guard_material in ring_specs:
        fan_head.visual(
            _annulus_mesh(guard_name, outer, inner, depth),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=guard_material,
            name=guard_name,
        )

    spoke_length = 0.200
    spoke_mid_radius = 0.140
    for face_name, y, phase in (("front", -0.058, 0.0), ("rear", 0.058, math.radians(15.0))):
        for i in range(12):
            theta = phase + i * math.tau / 12.0
            fan_head.visual(
                Box((spoke_length, 0.006, 0.006)),
                origin=Origin(
                    xyz=(spoke_mid_radius * math.cos(theta), y, spoke_mid_radius * math.sin(theta)),
                    rpy=(0.0, -theta, 0.0),
                ),
                material=brushed_dark_metal,
                name=f"{face_name}_spoke_{i}",
            )

    for i in range(12):
        theta = i * math.tau / 12.0
        fan_head.visual(
            Cylinder(radius=0.0042, length=0.116),
            origin=Origin(
                xyz=(0.238 * math.cos(theta), 0.0, 0.238 * math.sin(theta)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=satin_warm_metal,
            name=f"cage_tie_{i}",
        )

    fan_head.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, -0.067, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="front_center_cap",
    )
    fan_head.visual(
        Cylinder(radius=0.075, length=0.092),
        origin=Origin(xyz=(0.0, 0.109, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="rear_motor_pod",
    )
    fan_head.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.164, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="rear_service_cover",
    )
    fan_head.visual(
        Cylinder(radius=0.081, length=0.006),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_warm_metal,
        name="motor_seam_ring",
    )
    fan_head.visual(
        Cylinder(radius=0.0053, length=0.100),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_dark_metal,
        name="motor_shaft",
    )
    for sx in (-1.0, 1.0):
        fan_head.visual(
            Cylinder(radius=0.027, length=0.092),
            origin=Origin(xyz=(sx * 0.298, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_dark_metal,
            name=f"trunnion_axle_{'neg' if sx < 0 else 'pos'}",
        )
        fan_head.visual(
            Cylinder(radius=0.041, length=0.012),
            origin=Origin(xyz=(sx * 0.254, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_graphite,
            name=f"inner_pivot_collar_{'neg' if sx < 0 else 'pos'}",
        )
    for sx in (-1.0, 1.0):
        # Short visible cast ribs show how the guard shell resolves into the tilt boss.
        fan_head.visual(
            Box((0.074, 0.038, 0.030)),
            origin=Origin(xyz=(sx * 0.222, 0.0, 0.0)),
            material=satin_graphite,
            name=f"pivot_rib_{'neg' if sx < 0 else 'pos'}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.185,
                0.046,
                5,
                thickness=0.030,
                blade_pitch_deg=34.0,
                blade_sweep_deg=30.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.20, tip_clearance=0.006),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.012, rear_collar_radius=0.032, bore_diameter=0.010),
            ),
            "rotor_blades",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_blade,
        name="rotor_blades",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.023,
                body_style="skirted",
                top_diameter=0.042,
                skirt=KnobSkirt(0.066, 0.006, flare=0.05, chamfer=0.0012),
                grip=KnobGrip(style="fluted", count=20, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_dial",
        ),
        origin=Origin(),
        material=soft_black,
        name="speed_dial",
    )
    speed_dial.visual(
        Box((0.006, 0.030, 0.002)),
        origin=Origin(xyz=(0.0, -0.022, 0.024)),
        material=marker_white,
        name="dial_marker",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.032,
                body_style="lobed",
                base_diameter=0.044,
                top_diameter=0.064,
                crown_radius=0.0014,
                grip=KnobGrip(style="ribbed", count=8, depth=0.0010, width=0.0020),
                center=False,
            ),
            "tilt_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="tilt_knob",
    )

    tilt_joint = model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=fan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=fan_head,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )
    model.articulation(
        "base_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=speed_dial,
        origin=Origin(xyz=(0.0, -0.165, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=2.6),
    )
    model.articulation(
        "yoke_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=tilt_knob,
        origin=Origin(xyz=(0.342, 0.0, 0.850)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    # Store the intended primary limit values for concise pose tests.
    model.meta["tilt_joint_name"] = tilt_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    fan_head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    speed_dial = object_model.get_part("speed_dial")
    tilt_knob = object_model.get_part("tilt_knob")
    tilt_joint = object_model.get_articulation("yoke_to_head")

    ctx.allow_overlap(
        fan_head,
        rotor,
        elem_a="motor_shaft",
        elem_b="rotor_blades",
        reason="The motor shaft is intentionally captured inside the rotor hub bore.",
    )
    ctx.allow_overlap(
        fan_head,
        pedestal,
        elem_a="trunnion_axle_neg",
        elem_b="pivot_bearing_neg",
        reason="The left trunnion axle is intentionally seated in the yoke bearing sleeve.",
    )
    ctx.allow_overlap(
        fan_head,
        pedestal,
        elem_a="trunnion_axle_pos",
        elem_b="pivot_bearing_pos",
        reason="The right trunnion axle is intentionally seated in the yoke bearing sleeve.",
    )
    ctx.expect_overlap(
        fan_head,
        rotor,
        axes="y",
        elem_a="motor_shaft",
        elem_b="rotor_blades",
        min_overlap=0.020,
        name="motor shaft remains engaged in the rotor hub",
    )
    ctx.expect_contact(
        fan_head,
        rotor,
        elem_a="motor_shaft",
        elem_b="rotor_blades",
        contact_tol=0.002,
        name="motor shaft contacts the rotor hub",
    )
    ctx.expect_overlap(
        fan_head,
        pedestal,
        axes="x",
        elem_a="trunnion_axle_neg",
        elem_b="pivot_bearing_neg",
        min_overlap=0.006,
        name="left trunnion remains inserted in bearing",
    )
    ctx.expect_overlap(
        fan_head,
        pedestal,
        axes="x",
        elem_a="trunnion_axle_pos",
        elem_b="pivot_bearing_pos",
        min_overlap=0.006,
        name="right trunnion remains inserted in bearing",
    )
    ctx.expect_contact(
        fan_head,
        pedestal,
        elem_a="trunnion_axle_pos",
        elem_b="pivot_bearing_pos",
        contact_tol=0.002,
        name="visible tilt axle bears in the side bushing",
    )
    ctx.expect_within(
        rotor,
        fan_head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="front_outer_guard",
        margin=0.002,
        name="rotor sits inside the circular guard envelope",
    )
    ctx.expect_gap(
        rotor,
        fan_head,
        axis="y",
        positive_elem="rotor_blades",
        negative_elem="front_outer_guard",
        min_gap=0.015,
        name="front guard clears the spinning rotor",
    )
    ctx.expect_gap(
        fan_head,
        rotor,
        axis="y",
        positive_elem="rear_outer_guard",
        negative_elem="rotor_blades",
        min_gap=0.025,
        name="rear guard clears the spinning rotor",
    )
    ctx.expect_gap(
        speed_dial,
        pedestal,
        axis="z",
        positive_elem="speed_dial",
        negative_elem="raised_base_plinth",
        max_gap=0.004,
        max_penetration=0.0005,
        name="speed dial is seated on the base plinth",
    )
    ctx.expect_contact(
        tilt_knob,
        pedestal,
        elem_a="tilt_knob",
        elem_b="pivot_bearing_pos",
        contact_tol=0.004,
        name="side tilt knob bears on the yoke pivot",
    )

    closed_aabb = ctx.part_element_world_aabb(fan_head, elem="front_outer_guard")
    with ctx.pose({tilt_joint: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(fan_head, elem="front_outer_guard")
    ctx.check(
        "head visibly tilts about the side axle",
        closed_aabb is not None
        and tilted_aabb is not None
        and abs(tilted_aabb[0][1] - closed_aabb[0][1]) > 0.035,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
