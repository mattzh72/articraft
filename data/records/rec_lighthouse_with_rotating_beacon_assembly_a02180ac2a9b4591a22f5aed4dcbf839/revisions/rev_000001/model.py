from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse")

    white_plaster = model.material("white_plaster", rgba=(0.88, 0.86, 0.78, 1.0))
    red_paint = model.material("weathered_red", rgba=(0.63, 0.05, 0.035, 1.0))
    dark_metal = model.material("blackened_iron", rgba=(0.03, 0.035, 0.04, 1.0))
    deck_metal = model.material("galvanized_deck", rgba=(0.42, 0.43, 0.42, 1.0))
    glass = model.material("sea_glass", rgba=(0.62, 0.85, 1.0, 0.32))
    brass = model.material("aged_brass", rgba=(0.94, 0.68, 0.28, 1.0))
    warm_light = model.material("warm_lens", rgba=(1.0, 0.74, 0.20, 0.85))
    shadow = model.material("dark_recess", rgba=(0.015, 0.018, 0.02, 1.0))

    tower = model.part("tower")

    # Tapered masonry shaft: a solid exterior silhouette with a wider base and
    # narrower lantern pedestal, matching a small harbor lighthouse scale.
    tower_shell = LatheGeometry(
        [
            (0.0, 0.0),
            (0.62, 0.0),
            (0.56, 0.55),
            (0.49, 2.15),
            (0.42, 3.16),
            (0.0, 3.16),
        ],
        segments=72,
    )
    tower.visual(
        mesh_from_geometry(tower_shell, "tower_shell"),
        material=white_plaster,
        name="tower_shell",
    )

    base_plinth = LatheGeometry(
        [(0.0, 0.0), (0.78, 0.0), (0.78, 0.24), (0.66, 0.32), (0.0, 0.32)],
        segments=72,
    )
    tower.visual(
        mesh_from_geometry(base_plinth, "base_plinth"),
        material=red_paint,
        name="base_plinth",
    )

    for i, (z0, z1, r0, r1) in enumerate(
        (
            (0.92, 1.05, 0.545, 0.535),
            (1.88, 2.01, 0.495, 0.485),
            (2.78, 2.92, 0.445, 0.435),
        )
    ):
        band = LatheGeometry(
            [(r0, z0), (r0 + 0.016, z0), (r1 + 0.016, z1), (r1, z1)],
            segments=72,
        )
        tower.visual(
            mesh_from_geometry(band, f"red_band_{i}"),
            material=red_paint,
            name=f"red_band_{i}",
        )

    tower.visual(
        Box((0.30, 0.030, 0.58)),
        origin=Origin(xyz=(0.0, 0.565, 0.54)),
        material=shadow,
        name="front_door",
    )
    for i, (z, width) in enumerate(((1.45, 0.18), (2.34, 0.15))):
        tower.visual(
            Box((width, 0.026, 0.30)),
            origin=Origin(xyz=(0.0, 0.505 - 0.035 * i, z)),
            material=shadow,
            name=f"window_{i}",
        )

    # Gallery deck and rail encircle the lantern room but leave a front gap for
    # a separate hinged access gate.
    tower.visual(
        Cylinder(radius=0.86, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 3.185)),
        material=deck_metal,
        name="gallery_deck",
    )
    tower.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.64, tube=0.035, radial_segments=72, tubular_segments=12),
            "gallery_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.275)),
        material=red_paint,
        name="gallery_lip",
    )

    rail_radius = 0.79
    post_radius = 0.018
    rail_height = 0.62
    rail_base_z = 3.25
    rail_angles = [-170, -140, -110, -80, -50, -20, 20, 50, 70, 110, 130, 155, 180]
    for idx, angle_deg in enumerate(rail_angles):
        angle = math.radians(angle_deg)
        x = rail_radius * math.cos(angle)
        y = rail_radius * math.sin(angle)
        if angle_deg == 70:
            tower.visual(
                Cylinder(radius=post_radius, length=rail_height),
                origin=Origin(xyz=(x, y, rail_base_z + rail_height / 2.0 - 0.025)),
                material=dark_metal,
                name="gate_hinge_post",
            )
        else:
            tower.visual(
                Cylinder(radius=post_radius, length=rail_height),
                origin=Origin(xyz=(x, y, rail_base_z + rail_height / 2.0 - 0.025)),
                material=dark_metal,
                name=f"gallery_post_{idx}",
            )

    def add_rail_segment(a0_deg: float, a1_deg: float, level_z: float, name: str) -> None:
        a0 = math.radians(a0_deg)
        a1 = math.radians(a1_deg)
        p0 = (rail_radius * math.cos(a0), rail_radius * math.sin(a0))
        p1 = (rail_radius * math.cos(a1), rail_radius * math.sin(a1))
        mid = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        length = math.hypot(dx, dy) + 0.045
        yaw = math.atan2(dy, dx)
        tower.visual(
            Box((length, 0.030, 0.030)),
            origin=Origin(xyz=(mid[0], mid[1], level_z), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=name,
        )

    segment_pairs = list(zip(rail_angles, rail_angles[1:]))
    for level_name, z in (("mid", 3.50), ("top", 3.80)):
        seg_i = 0
        for a0, a1 in segment_pairs:
            if (a0 == 50 and a1 == 70) or (a0 == 70 and a1 == 110):
                continue
            add_rail_segment(a0, a1, z, f"gallery_{level_name}_rail_{seg_i}")
            seg_i += 1

    # Lantern room: cylindrical glass cage above the gallery with mullions,
    # heavy sill rings, conical red roof, and a central shaft for the rotating
    # beacon carriage.
    tower.visual(
        Cylinder(radius=0.54, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 3.31)),
        material=dark_metal,
        name="lantern_sill",
    )
    glass_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.50, 3.40), (0.50, 4.48)],
        inner_profile=[(0.47, 3.42), (0.47, 4.46)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    tower.visual(
        mesh_from_geometry(glass_shell, "glass_shell"),
        material=glass,
        name="glass_shell",
    )
    for name, z in (("lower_cage_ring", 3.40), ("upper_cage_ring", 4.48)):
        tower.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.50, tube=0.027, radial_segments=72, tubular_segments=12),
                name,
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_metal,
            name=name,
        )
    for idx in range(12):
        angle = 2.0 * math.pi * idx / 12
        tower.visual(
            Cylinder(radius=0.012, length=1.16),
            origin=Origin(
                xyz=(0.505 * math.cos(angle), 0.505 * math.sin(angle), 3.94)
            ),
            material=dark_metal,
            name=f"cage_mullion_{idx}",
        )
    roof = LatheGeometry(
        [(0.0, 4.88), (0.62, 4.48), (0.0, 4.48)],
        segments=96,
    )
    tower.visual(mesh_from_geometry(roof, "lantern_roof"), material=red_paint, name="lantern_roof")
    tower.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 4.92)),
        material=dark_metal,
        name="vent_cap",
    )
    tower.visual(
        Cylinder(radius=0.012, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 5.18)),
        material=dark_metal,
        name="lightning_rod",
    )
    tower.visual(
        Cylinder(radius=0.040, length=1.22),
        origin=Origin(xyz=(0.0, 0.0, 3.98)),
        material=dark_metal,
        name="central_shaft",
    )

    # Rotating beacon carriage. Its frame is the shaft axis; the optical
    # carriage and bearing hub rotate continuously about that vertical axis.
    beacon = model.part("beacon_carriage")
    beacon.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="bearing_hub",
    )
    for i, y in enumerate((-0.070, 0.070)):
        beacon.visual(
            Box((0.50, 0.035, 0.075)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=brass,
            name=f"lamp_cradle_{i}",
        )
    beacon.visual(
        Cylinder(radius=0.105, length=0.16),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_light,
        name="lens_0",
    )
    beacon.visual(
        Cylinder(radius=0.105, length=0.16),
        origin=Origin(xyz=(-0.225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_light,
        name="lens_1",
    )
    for i, y in enumerate((-0.100, 0.100)):
        beacon.visual(
            Box((0.050, 0.035, 0.26)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_metal,
            name=f"yoke_frame_{i}",
        )

    model.articulation(
        "tower_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 3.98)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    # Hinged gallery access gate spanning the deliberate gap in the gallery rail.
    gate = model.part("access_gate")
    gate_len = 2.0 * rail_radius * math.sin(math.radians(20.0)) - 0.020
    gate.visual(
        Cylinder(radius=0.026, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=dark_metal,
        name="hinge_barrel",
    )
    gate.visual(
        Cylinder(radius=0.016, length=0.56),
        origin=Origin(xyz=(gate_len, -0.040, 0.34)),
        material=dark_metal,
        name="latch_rail",
    )
    for name, z in (("bottom_bar", 0.12), ("middle_bar", 0.34), ("top_bar", 0.58)):
        gate.visual(
            Box((gate_len, 0.030, 0.030)),
            origin=Origin(xyz=(gate_len / 2.0, -0.040, z)),
            material=dark_metal,
            name=name,
        )
    gate.visual(
        Box((gate_len * 1.18, 0.022, 0.026)),
        origin=Origin(
            xyz=(gate_len / 2.0, -0.040, 0.35),
            rpy=(0.0, -math.atan2(0.42, gate_len), 0.0),
        ),
        material=dark_metal,
        name="diagonal_brace",
    )

    hinge_angle = math.radians(70.0)
    hinge_xyz = (rail_radius * math.cos(hinge_angle), rail_radius * math.sin(hinge_angle), rail_base_z)
    model.articulation(
        "tower_to_gate",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=gate,
        origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, math.pi)),
        # With the gate closed along local +X, +q swings it outward from the
        # gallery toward +Y.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0, lower=0.0, upper=math.radians(105)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon_carriage")
    gate = object_model.get_part("access_gate")
    beacon_joint = object_model.get_articulation("tower_to_beacon")
    gate_joint = object_model.get_articulation("tower_to_gate")

    ctx.allow_overlap(
        tower,
        beacon,
        elem_a="central_shaft",
        elem_b="bearing_hub",
        reason="The beacon bearing hub intentionally surrounds the fixed vertical shaft that supports its continuous rotation.",
    )
    ctx.expect_within(
        tower,
        beacon,
        axes="xy",
        inner_elem="central_shaft",
        outer_elem="bearing_hub",
        margin=0.002,
        name="shaft centered inside beacon hub",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="bearing_hub",
        elem_b="central_shaft",
        min_overlap=0.18,
        name="hub remains vertically captured on shaft",
    )

    ctx.allow_overlap(
        tower,
        gate,
        elem_a="gate_hinge_post",
        elem_b="hinge_barrel",
        reason="The small gallery gate hinge barrel is modeled as wrapping around the fixed side post/pin.",
    )
    ctx.expect_overlap(
        gate,
        tower,
        axes="z",
        elem_a="hinge_barrel",
        elem_b="gate_hinge_post",
        min_overlap=0.55,
        name="gate hinge wraps the side post",
    )

    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="lens_0",
        outer_elem="glass_shell",
        margin=0.0,
        name="beacon lens sits inside lantern cage",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    closed_lens = aabb_center(ctx.part_element_world_aabb(beacon, elem="lens_0"))
    with ctx.pose({beacon_joint: math.pi / 2.0}):
        quarter_lens = aabb_center(ctx.part_element_world_aabb(beacon, elem="lens_0"))
    ctx.check(
        "beacon carriage rotates about vertical shaft",
        closed_lens[0] > 0.18 and quarter_lens[1] > 0.18 and abs(quarter_lens[0]) < 0.08,
        details=f"closed_lens={closed_lens}, quarter_lens={quarter_lens}",
    )

    closed_latch = aabb_center(ctx.part_element_world_aabb(gate, elem="latch_rail"))
    with ctx.pose({gate_joint: math.radians(85.0)}):
        open_latch = aabb_center(ctx.part_element_world_aabb(gate, elem="latch_rail"))
    ctx.check(
        "gallery access gate swings outward",
        open_latch[1] > closed_latch[1] + 0.18,
        details=f"closed_latch={closed_latch}, open_latch={open_latch}",
    )

    return ctx.report()


object_model = build_object_model()
