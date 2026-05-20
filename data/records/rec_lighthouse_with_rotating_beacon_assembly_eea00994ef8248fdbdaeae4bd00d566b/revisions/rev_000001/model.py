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


def _octagonal_ring(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    """A true octagonal hollow frame, authored in model-local meters."""
    shell = cq.Workplane("XY").polygon(8, 2.0 * outer_radius).extrude(height)
    bore = (
        cq.Workplane("XY")
        .polygon(8, 2.0 * inner_radius)
        .extrude(height + 0.02)
        .translate((0.0, 0.0, -0.01))
    )
    return shell.cut(bore).translate((0.0, 0.0, z_min))


def _octagonal_roof(base_radius: float, cap_radius: float, height: float, z_min: float) -> cq.Workplane:
    """Eight-sided lantern roof tapering to a small ventilator cap."""
    return (
        cq.Workplane("XY")
        .polygon(8, 2.0 * base_radius)
        .workplane(offset=height)
        .polygon(8, 2.0 * cap_radius)
        .loft(combine=True)
        .translate((0.0, 0.0, z_min))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse")

    white = model.material("warm_white", rgba=(0.88, 0.86, 0.78, 1.0))
    red = model.material("weathered_red", rgba=(0.70, 0.08, 0.05, 1.0))
    concrete = model.material("salt_gray_concrete", rgba=(0.42, 0.44, 0.44, 1.0))
    dark = model.material("black_iron", rgba=(0.02, 0.025, 0.025, 1.0))
    glass = model.material("pale_lantern_glass", rgba=(0.64, 0.86, 1.0, 0.34))
    brass = model.material("aged_brass", rgba=(0.82, 0.58, 0.22, 1.0))
    light = model.material("warm_beacon_light", rgba=(1.0, 0.78, 0.18, 0.88))
    lens_glass = model.material("fresnel_lens_glass", rgba=(0.55, 0.90, 1.0, 0.52))
    gate_paint = model.material("green_gate_paint", rgba=(0.05, 0.28, 0.18, 1.0))

    tower = model.part("tower")

    # Short cylindrical harbor tower with a stout base and painted bands.
    tower.visual(
        Cylinder(radius=0.92, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.68, length=4.10),
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        material=white,
        name="tower_shell",
    )
    for idx, z in enumerate((1.05, 2.28, 3.50)):
        tower.visual(
            Cylinder(radius=0.695, length=0.36),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=red,
            name=f"red_band_{idx}",
        )
    tower.visual(
        Box((0.36, 0.035, 0.85)),
        origin=Origin(xyz=(0.0, -0.695, 0.67)),
        material=dark,
        name="service_door",
    )
    tower.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.12, -0.725, 0.67)),
        material=brass,
        name="door_knob",
    )

    # Gallery deck and octagonal lantern room.
    tower.visual(
        Cylinder(radius=1.65, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 4.31)),
        material=concrete,
        name="gallery_deck",
    )
    tower.visual(
        mesh_from_cadquery(_octagonal_ring(0.98, 0.74, 0.18, 4.38), "lower_lantern_ring"),
        material=dark,
        name="lower_lantern_ring",
    )
    tower.visual(
        mesh_from_cadquery(_octagonal_ring(0.98, 0.74, 0.16, 5.86), "upper_lantern_ring"),
        material=dark,
        name="upper_lantern_ring",
    )

    lantern_radius = 0.98
    apothem = lantern_radius * math.cos(math.pi / 8.0)
    side_len = 2.0 * lantern_radius * math.sin(math.pi / 8.0)
    glass_bottom = 4.55
    glass_top = 5.88
    glass_height = glass_top - glass_bottom
    for i in range(8):
        alpha = i * math.pi / 4.0
        yaw = alpha - math.pi / 2.0
        tower.visual(
            Box((side_len - 0.05, 0.030, glass_height)),
            origin=Origin(
                xyz=(
                    apothem * math.cos(alpha),
                    apothem * math.sin(alpha),
                    (glass_bottom + glass_top) / 2.0,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=glass,
            name=f"glass_panel_{i}",
        )

    for i in range(8):
        beta = math.pi / 8.0 + i * math.pi / 4.0
        tower.visual(
            Cylinder(radius=0.040, length=1.43),
            origin=Origin(
                xyz=(
                    lantern_radius * math.cos(beta),
                    lantern_radius * math.sin(beta),
                    5.205,
                )
            ),
            material=dark,
            name=f"lantern_mullion_{i}",
        )

    tower.visual(
        mesh_from_cadquery(_octagonal_roof(1.06, 0.26, 0.55, 6.02), "octagonal_roof"),
        material=dark,
        name="octagonal_roof",
    )
    tower.visual(
        Cylinder(radius=0.22, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 6.68)),
        material=dark,
        name="roof_vent",
    )
    tower.visual(
        Cylinder(radius=0.018, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 7.015)),
        material=dark,
        name="lightning_rod",
    )

    # Fixed pedestal inside the glazing.  The beacon part carries a retained spindle
    # that nests into the named pedestal core so it remains seated while rotating.
    tower.visual(
        Cylinder(radius=0.18, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 4.79)),
        material=brass,
        name="pedestal_core",
    )
    tower.visual(
        Cylinder(radius=0.25, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 4.47)),
        material=brass,
        name="pedestal_foot",
    )

    # Octagonal gallery guard rail with a deliberate narrow gate opening on the
    # seaward/front side (negative Y).
    rail_radius = 1.55
    rail_apothem = rail_radius * math.cos(math.pi / 8.0)
    rail_side = 2.0 * rail_radius * math.sin(math.pi / 8.0)
    rail_z_mid = 4.78
    rail_z_top = 5.16
    rail_thick = 0.045
    rail_height = 0.88
    rail_center_z = 4.80
    gate_hinge_x = -0.26
    gate_latch_x = 0.24
    gate_side_y = -rail_apothem

    for i in range(8):
        alpha = i * math.pi / 4.0
        yaw = alpha - math.pi / 2.0
        if i == 6:
            # Split the front rail around the gate opening.
            for z, suffix in ((rail_z_mid, "mid"), (rail_z_top, "top")):
                tower.visual(
                    Box((0.27, rail_thick, rail_thick)),
                    origin=Origin(xyz=(-0.455, gate_side_y, z)),
                    material=dark,
                    name=f"front_left_rail_{suffix}",
                )
                tower.visual(
                    Box((0.30, rail_thick, rail_thick)),
                    origin=Origin(xyz=(0.435, gate_side_y, z)),
                    material=dark,
                    name=f"front_right_rail_{suffix}",
                )
            continue
        for z, suffix in ((rail_z_mid, "mid"), (rail_z_top, "top")):
            tower.visual(
                Box((rail_side, rail_thick, rail_thick)),
                origin=Origin(
                    xyz=(rail_apothem * math.cos(alpha), rail_apothem * math.sin(alpha), z),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=dark,
                name=f"guard_rail_{i}_{suffix}",
            )

    for i in range(8):
        beta = math.pi / 8.0 + i * math.pi / 4.0
        tower.visual(
            Cylinder(radius=0.035, length=rail_height),
            origin=Origin(
                xyz=(rail_radius * math.cos(beta), rail_radius * math.sin(beta), rail_center_z)
            ),
            material=dark,
            name=f"rail_post_{i}",
        )
    tower.visual(
        Cylinder(radius=0.035, length=rail_height),
        origin=Origin(xyz=(gate_hinge_x, gate_side_y, rail_center_z)),
        material=dark,
        name="gate_hinge_post",
    )
    tower.visual(
        Cylinder(radius=0.035, length=rail_height),
        origin=Origin(xyz=(gate_latch_x, gate_side_y, rail_center_z)),
        material=dark,
        name="gate_latch_post",
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=dark,
        name="spindle",
    )
    beacon.visual(
        Cylinder(radius=0.22, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=brass,
        name="bearing_collar",
    )
    beacon.visual(
        Cylinder(radius=0.042, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=brass,
        name="center_shaft",
    )
    beacon.visual(
        Sphere(radius=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=light,
        name="lamp_globe",
    )
    for z, suffix in ((0.24, "lower"), (0.63, "upper")):
        beacon.visual(
            Box((0.74, 0.045, 0.045)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=f"{suffix}_lens_crossbar",
        )
    for sign, suffix in ((1.0, "front"), (-1.0, "rear")):
        beacon.visual(
            Box((0.080, 0.42, 0.34)),
            origin=Origin(xyz=(sign * 0.31, 0.0, 0.43)),
            material=lens_glass,
            name=f"{suffix}_lens",
        )

    model.articulation(
        "tower_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 5.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5),
    )

    gate = model.part("gate")
    gate.visual(
        Cylinder(radius=0.055, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=gate_paint,
        name="hinge_barrel",
    )
    for z, suffix in ((0.18, "lower"), (0.42, "middle"), (0.72, "upper")):
        gate.visual(
            Box((0.406, 0.040, 0.045)),
            origin=Origin(xyz=(0.255, 0.0, z)),
            material=gate_paint,
            name=f"gate_{suffix}_rail",
        )
    for x, suffix in ((0.16, "inner"), (0.30, "outer"), (0.39, "latch")):
        gate.visual(
            Cylinder(radius=0.022, length=0.62),
            origin=Origin(xyz=(x, 0.0, 0.45)),
            material=gate_paint,
            name=f"gate_{suffix}_bar",
        )

    model.articulation(
        "tower_to_gate",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=gate,
        origin=Origin(xyz=(gate_hinge_x, gate_side_y, 4.40)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    gate = object_model.get_part("gate")
    beacon_joint = object_model.get_articulation("tower_to_beacon")
    gate_joint = object_model.get_articulation("tower_to_gate")

    ctx.allow_overlap(
        tower,
        beacon,
        elem_a="pedestal_core",
        elem_b="spindle",
        reason="The beacon spindle is intentionally retained inside the central pedestal bore.",
    )
    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="spindle",
        outer_elem="pedestal_core",
        margin=0.0,
        name="beacon spindle is centered inside the pedestal",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="spindle",
        elem_b="pedestal_core",
        min_overlap=0.09,
        name="beacon spindle remains clipped into pedestal",
    )
    ctx.expect_gap(
        beacon,
        tower,
        axis="z",
        positive_elem="bearing_collar",
        negative_elem="pedestal_core",
        max_gap=0.003,
        max_penetration=0.0,
        name="beacon bearing collar seats on pedestal top",
    )

    ctx.allow_overlap(
        tower,
        gate,
        elem_a="gate_hinge_post",
        elem_b="hinge_barrel",
        reason="The narrow gate hinge barrel is wrapped around the fixed rail post as a captured hinge.",
    )
    ctx.expect_overlap(
        tower,
        gate,
        axes="z",
        elem_a="gate_hinge_post",
        elem_b="hinge_barrel",
        min_overlap=0.60,
        name="gate hinge barrel shares the fixed rail post height",
    )
    ctx.expect_overlap(
        tower,
        gate,
        axes="xy",
        elem_a="gate_hinge_post",
        elem_b="hinge_barrel",
        min_overlap=0.06,
        name="gate hinge barrel is captured around its post",
    )

    with ctx.pose({beacon_joint: 0.0}):
        closed_box = ctx.part_world_aabb(beacon)
    with ctx.pose({beacon_joint: math.pi / 2.0}):
        quarter_box = ctx.part_world_aabb(beacon)
    closed_wide_x = closed_box is not None and (closed_box[1][0] - closed_box[0][0]) > (
        closed_box[1][1] - closed_box[0][1]
    )
    quarter_wide_y = quarter_box is not None and (quarter_box[1][1] - quarter_box[0][1]) > (
        quarter_box[1][0] - quarter_box[0][0]
    )
    ctx.check(
        "beacon continuously rotates around the vertical pedestal axis",
        closed_wide_x and quarter_wide_y,
        details=f"closed_aabb={closed_box}, quarter_aabb={quarter_box}",
    )

    with ctx.pose({gate_joint: 0.0}):
        gate_closed = ctx.part_world_aabb(gate)
    with ctx.pose({gate_joint: 1.20}):
        gate_open = ctx.part_world_aabb(gate)
    ctx.check(
        "gallery gate swings outward from the guard rail",
        gate_closed is not None
        and gate_open is not None
        and gate_open[0][1] < gate_closed[0][1] - 0.12,
        details=f"closed_aabb={gate_closed}, open_aabb={gate_open}",
    )

    return ctx.report()


object_model = build_object_model()
