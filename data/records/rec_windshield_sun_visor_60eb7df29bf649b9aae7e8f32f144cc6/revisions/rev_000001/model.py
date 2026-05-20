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


def _padded_panel_mesh():
    """Rounded, thick visor slab with a real through grab/service opening."""
    width = 0.66
    height = 0.34
    thickness = 0.035
    slot_w = 0.140
    slot_h = 0.055

    panel = (
        cq.Workplane("XY")
        .box(width, height, thickness)
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z or <Z")
        .fillet(0.003)
    )
    cutter = (
        cq.Workplane("XY")
        .center(-0.205, 0.103)
        .box(slot_w, slot_h, thickness * 3.0)
        .edges("|Z")
        .fillet(0.014)
    )
    return panel.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_windshield_sun_visor")

    padded_vinyl = model.material("padded_black_vinyl", rgba=(0.025, 0.027, 0.025, 1.0))
    molded_rubber = model.material("molded_black_rubber", rgba=(0.010, 0.011, 0.010, 1.0))
    armor_green = model.material("painted_utility_green", rgba=(0.18, 0.24, 0.18, 1.0))
    dark_steel = model.material("phosphate_dark_steel", rgba=(0.05, 0.055, 0.052, 1.0))
    zinc = model.material("zinc_plated_fasteners", rgba=(0.62, 0.60, 0.54, 1.0))

    roof = model.part("roof_mount")
    roof.visual(
        Box((0.86, 0.105, 0.035)),
        origin=Origin(xyz=(0.34, -0.070, 0.1025)),
        material=armor_green,
        name="roof_plate",
    )
    roof.visual(
        Box((0.78, 0.012, 0.012)),
        origin=Origin(xyz=(0.36, -0.026, 0.125)),
        material=dark_steel,
        name="front_stiffener_rib",
    )
    roof.visual(
        Box((0.78, 0.012, 0.012)),
        origin=Origin(xyz=(0.36, -0.114, 0.125)),
        material=dark_steel,
        name="rear_stiffener_rib",
    )
    roof.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_steel,
        name="pivot_socket",
    )
    roof.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=dark_steel,
        name="socket_top_flange",
    )
    roof.visual(
        Box((0.110, 0.085, 0.018)),
        origin=Origin(xyz=(0.0, -0.070, 0.083)),
        material=armor_green,
        name="socket_web",
    )
    # A fixed outboard receiver clip, open around the top rod at the rest pose.
    roof.visual(
        Box((0.052, 0.010, 0.048)),
        origin=Origin(xyz=(0.705, -0.030, 0.000)),
        material=dark_steel,
        name="clip_rear_jaw",
    )
    roof.visual(
        Box((0.052, 0.010, 0.048)),
        origin=Origin(xyz=(0.705, 0.030, 0.000)),
        material=dark_steel,
        name="clip_front_jaw",
    )
    roof.visual(
        Box((0.052, 0.075, 0.012)),
        origin=Origin(xyz=(0.705, 0.000, 0.024)),
        material=dark_steel,
        name="clip_bridge",
    )
    roof.visual(
        Box((0.040, 0.030, 0.076)),
        origin=Origin(xyz=(0.705, -0.048, 0.056)),
        material=armor_green,
        name="clip_stem",
    )
    for idx, x in enumerate((-0.055, 0.055, 0.640, 0.770)):
        roof.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.070, 0.123), rpy=(0.0, 0.0, 0.0)),
            material=zinc,
            name=f"roof_fastener_{idx}",
        )

    arm = model.part("hinge_arm")
    arm.visual(
        Cylinder(radius=0.016, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=dark_steel,
        name="swivel_pin",
    )
    arm.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
        name="thrust_washer",
    )
    arm.visual(
        Box((0.066, 0.056, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_steel,
        name="elbow_block",
    )
    arm.visual(
        Cylinder(radius=0.011, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    arm.visual(
        Box((0.008, 0.066, 0.062)),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=dark_steel,
        name="outer_clevis_plate",
    )
    arm.visual(
        Box((0.008, 0.066, 0.062)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=dark_steel,
        name="inner_clevis_plate",
    )
    arm.visual(
        Box((0.050, 0.016, 0.052)),
        origin=Origin(xyz=(0.0, -0.033, 0.004)),
        material=armor_green,
        name="clevis_rear_bridge",
    )
    arm.visual(
        Box((0.052, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.033, 0.003)),
        material=armor_green,
        name="clevis_front_lip",
    )
    for idx, x in enumerate((-0.049, 0.049)):
        arm.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -0.036, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"clevis_bolt_{idx}",
        )

    visor = model.part("visor_panel")
    visor.visual(
        mesh_from_cadquery(_padded_panel_mesh(), "padded_panel"),
        origin=Origin(xyz=(0.390, 0.0, -0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=padded_vinyl,
        name="padded_panel",
    )
    visor.visual(
        Cylinder(radius=0.013, length=0.660),
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_rod",
    )
    visor.visual(
        Box((0.640, 0.046, 0.034)),
        origin=Origin(xyz=(0.390, 0.0, -0.018)),
        material=molded_rubber,
        name="top_reinforcement",
    )
    visor.visual(
        Box((0.640, 0.038, 0.030)),
        origin=Origin(xyz=(0.390, 0.0, -0.355)),
        material=molded_rubber,
        name="bottom_reinforcement",
    )
    visor.visual(
        Box((0.026, 0.038, 0.320)),
        origin=Origin(xyz=(0.073, 0.0, -0.190)),
        material=molded_rubber,
        name="near_side_reinforcement",
    )
    visor.visual(
        Box((0.026, 0.038, 0.320)),
        origin=Origin(xyz=(0.707, 0.0, -0.190)),
        material=molded_rubber,
        name="far_side_reinforcement",
    )
    # Raised molded rim around the through opening; the hole stays visibly open.
    slot_center_x = 0.185
    slot_center_z = -0.087
    visor.visual(
        Box((0.178, 0.009, 0.014)),
        origin=Origin(xyz=(slot_center_x, 0.0205, slot_center_z + 0.038)),
        material=molded_rubber,
        name="slot_top_rim",
    )
    visor.visual(
        Box((0.178, 0.009, 0.014)),
        origin=Origin(xyz=(slot_center_x, 0.0205, slot_center_z - 0.038)),
        material=molded_rubber,
        name="slot_bottom_rim",
    )
    visor.visual(
        Box((0.014, 0.009, 0.076)),
        origin=Origin(xyz=(slot_center_x - 0.084, 0.0205, slot_center_z)),
        material=molded_rubber,
        name="slot_near_rim",
    )
    visor.visual(
        Box((0.014, 0.009, 0.076)),
        origin=Origin(xyz=(slot_center_x + 0.084, 0.0205, slot_center_z)),
        material=molded_rubber,
        name="slot_far_rim",
    )
    for idx, (x, z) in enumerate(
        (
            (0.105, -0.047),
            (0.265, -0.047),
            (0.105, -0.127),
            (0.265, -0.127),
            (0.085, -0.325),
            (0.695, -0.325),
        )
    ):
        visor.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(x, 0.0205, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"visor_fastener_{idx}",
        )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.57),
    )
    model.articulation(
        "arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=visor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_mount")
    arm = object_model.get_part("hinge_arm")
    visor = object_model.get_part("visor_panel")
    side_pivot = object_model.get_articulation("roof_to_arm")
    visor_hinge = object_model.get_articulation("arm_to_panel")

    ctx.allow_overlap(
        roof,
        arm,
        elem_a="pivot_socket",
        elem_b="swivel_pin",
        reason="The vertical swivel pin is intentionally captured inside the roof socket bushing.",
    )
    ctx.expect_within(
        arm,
        roof,
        axes="xy",
        inner_elem="swivel_pin",
        outer_elem="pivot_socket",
        margin=0.002,
        name="swivel pin is centered in the socket",
    )
    ctx.expect_overlap(
        arm,
        roof,
        axes="z",
        elem_a="swivel_pin",
        elem_b="pivot_socket",
        min_overlap=0.040,
        name="swivel pin has retained bushing engagement",
    )
    ctx.allow_overlap(
        roof,
        arm,
        elem_a="socket_top_flange",
        elem_b="swivel_pin",
        reason="The top bearing flange is a simplified solid proxy for a drilled retaining plate around the swivel pin.",
    )
    ctx.expect_overlap(
        arm,
        roof,
        axes="z",
        elem_a="swivel_pin",
        elem_b="socket_top_flange",
        min_overlap=0.008,
        name="swivel pin passes through the retaining flange",
    )
    ctx.allow_overlap(
        arm,
        visor,
        elem_a="hinge_pin",
        elem_b="hinge_rod",
        reason="The horizontal hinge rod is shown as a captured pin passing into the serviceable hinge sleeve.",
    )
    ctx.expect_within(
        arm,
        visor,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_rod",
        margin=0.003,
        name="horizontal hinge pin is coaxial with the visor rod",
    )
    ctx.expect_overlap(
        arm,
        visor,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_rod",
        min_overlap=0.001,
        name="horizontal hinge pin is retained in the visor rod",
    )
    ctx.expect_gap(
        roof,
        visor,
        axis="z",
        positive_elem="roof_plate",
        negative_elem="hinge_rod",
        min_gap=0.050,
        name="top rod clears the roof plate",
    )
    ctx.expect_gap(
        roof,
        visor,
        axis="y",
        positive_elem="clip_front_jaw",
        negative_elem="hinge_rod",
        min_gap=0.006,
        name="receiver clip front jaw clears the rod",
    )
    ctx.expect_gap(
        visor,
        roof,
        axis="y",
        positive_elem="hinge_rod",
        negative_elem="clip_rear_jaw",
        min_gap=0.006,
        name="receiver clip rear jaw clears the rod",
    )

    rest_aabb = ctx.part_world_aabb(visor)
    with ctx.pose({visor_hinge: 1.20}):
        stowed_aabb = ctx.part_world_aabb(visor)
    ctx.check(
        "visor hinge lifts the lower padded edge",
        rest_aabb is not None
        and stowed_aabb is not None
        and stowed_aabb[0][2] > rest_aabb[0][2] + 0.18,
        details=f"rest={rest_aabb}, stowed={stowed_aabb}",
    )

    with ctx.pose({side_pivot: 1.20}):
        swung_aabb = ctx.part_world_aabb(visor)
    ctx.check(
        "secondary pivot swings visor toward side window",
        rest_aabb is not None
        and swung_aabb is not None
        and swung_aabb[1][1] > rest_aabb[1][1] + 0.45,
        details=f"rest={rest_aabb}, swung={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
