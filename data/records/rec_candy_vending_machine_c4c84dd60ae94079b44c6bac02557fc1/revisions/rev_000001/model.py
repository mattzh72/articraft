from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _regular_polygon(radius: float, sides: int = 8, *, angle_offset: float = math.pi / 8.0):
    return [
        (
            radius * math.cos(angle_offset + 2.0 * math.pi * i / sides),
            radius * math.sin(angle_offset + 2.0 * math.pi * i / sides),
        )
        for i in range(sides)
    ]


def _octagonal_ring(
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    *,
    sides: int = 8,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer = _regular_polygon(outer_radius, sides)
    inner = _regular_polygon(inner_radius, sides)

    outer_bot = [geom.add_vertex(x, y, z_min) for x, y in outer]
    outer_top = [geom.add_vertex(x, y, z_max) for x, y in outer]
    inner_bot = [geom.add_vertex(x, y, z_min) for x, y in inner]
    inner_top = [geom.add_vertex(x, y, z_max) for x, y in inner]

    for i in range(sides):
        j = (i + 1) % sides
        geom.add_face(outer_bot[i], outer_bot[j], outer_top[j])
        geom.add_face(outer_bot[i], outer_top[j], outer_top[i])

        geom.add_face(inner_bot[j], inner_bot[i], inner_top[i])
        geom.add_face(inner_bot[j], inner_top[i], inner_top[j])

        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])

        geom.add_face(outer_bot[j], outer_bot[i], inner_bot[i])
        geom.add_face(outer_bot[j], inner_bot[i], inner_bot[j])

    return geom


def _tapered_octagonal_shell(
    bottom_outer_radius: float,
    top_outer_radius: float,
    wall: float,
    height: float,
    *,
    sides: int = 8,
) -> MeshGeometry:
    geom = MeshGeometry()
    bottom_outer = _regular_polygon(bottom_outer_radius, sides)
    top_outer = _regular_polygon(top_outer_radius, sides)
    bottom_inner = _regular_polygon(bottom_outer_radius - wall, sides)
    top_inner = _regular_polygon(top_outer_radius - wall, sides)

    bo = [geom.add_vertex(x, y, 0.0) for x, y in bottom_outer]
    to = [geom.add_vertex(x, y, height) for x, y in top_outer]
    bi = [geom.add_vertex(x, y, 0.0) for x, y in bottom_inner]
    ti = [geom.add_vertex(x, y, height) for x, y in top_inner]

    for i in range(sides):
        j = (i + 1) % sides
        geom.add_face(bo[i], bo[j], to[j])
        geom.add_face(bo[i], to[j], to[i])

        geom.add_face(bi[j], bi[i], ti[i])
        geom.add_face(bi[j], ti[i], ti[j])

        # Thin lips at the open top and bottom make the transparent shell read as
        # a real hollow canister rather than a single zero-thickness surface.
        geom.add_face(to[i], to[j], ti[j])
        geom.add_face(to[i], ti[j], ti[i])
        geom.add_face(bo[j], bo[i], bi[i])
        geom.add_face(bo[j], bi[i], bi[j])

    return geom


def _octagonal_solid(radius: float, height: float, z_min: float = 0.0) -> MeshGeometry:
    return ExtrudeGeometry.from_z0(_regular_polygon(radius), height, cap=True).translate(
        0.0, 0.0, z_min
    )


def _jellybean_mound() -> MeshGeometry:
    """One connected low mound: visible sweets without disconnected candy islands."""
    geom = MeshGeometry()
    sides = 24
    bottom_r = 0.109
    top_r = 0.094
    z_base = 0.006
    center_top_z = 0.076

    bottom = [
        geom.add_vertex(
            bottom_r * math.cos(2.0 * math.pi * i / sides),
            bottom_r * math.sin(2.0 * math.pi * i / sides),
            z_base,
        )
        for i in range(sides)
    ]
    top = []
    for i in range(sides):
        angle = 2.0 * math.pi * i / sides
        # Alternating lobes suggest individual jellybeans while staying a single
        # connected, manufacturable fill surface.
        lobe = 0.004 * math.sin(5.0 * angle) + 0.003 * math.cos(9.0 * angle)
        top.append(
            geom.add_vertex(
                top_r * math.cos(angle),
                top_r * math.sin(angle),
                center_top_z + lobe,
            )
        )
    center_bottom = geom.add_vertex(0.0, 0.0, z_base)
    center_top = geom.add_vertex(0.0, 0.0, center_top_z + 0.012)

    for i in range(sides):
        j = (i + 1) % sides
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(center_bottom, bottom[i], bottom[j])
        geom.add_face(center_top, top[j], top[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="octagonal_jellybean_dispenser")

    cast_red = model.material("cast_red_enamel", rgba=(0.66, 0.05, 0.03, 1.0))
    dark_shadow = model.material("dark_shadow", rgba=(0.02, 0.018, 0.015, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.82, 0.78, 1.0))
    brass = model.material("warm_brass", rgba=(0.85, 0.56, 0.19, 1.0))
    glass = model.material("clear_blue_glass", rgba=(0.70, 0.91, 1.0, 0.32))
    glass_edge = model.material("thick_glass_edges", rgba=(0.55, 0.78, 0.92, 0.55))
    candy = model.material("jellybean_fill", rgba=(0.95, 0.18, 0.12, 1.0))

    base = model.part("base")
    lower_plinth = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.285, 0.225, 0.035, corner_segments=8),
        0.055,
        cap=True,
    )
    base.visual(
        mesh_from_geometry(lower_plinth, "lower_plinth"),
        material=cast_red,
        name="lower_plinth",
    )

    main_casting = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.238, 0.185, 0.030, corner_segments=8),
        0.120,
        cap=True,
    ).translate(0.0, 0.0, 0.040)
    base.visual(
        mesh_from_geometry(main_casting, "main_casting"),
        material=cast_red,
        name="main_casting",
    )

    base.visual(
        mesh_from_geometry(_octagonal_ring(0.132, 0.096, 0.152, 0.180), "shoulder_ring"),
        material=brass,
        name="shoulder_ring",
    )

    # Front coin mechanism and compact dispense chute.
    base.visual(
        Box((0.108, 0.008, 0.074)),
        origin=Origin(xyz=(0.0, -0.095, 0.112)),
        material=chrome,
        name="coin_plate",
    )
    base.visual(
        Box((0.052, 0.003, 0.007)),
        origin=Origin(xyz=(0.0, -0.098, 0.132)),
        material=dark_shadow,
        name="coin_slot",
    )
    base.visual(
        Box((0.060, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, -0.101, 0.153)),
        material=brass,
        name="price_badge",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, -0.104, 0.089), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_bushing",
    )
    base.visual(
        Box((0.096, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, -0.130, 0.035), rpy=(0.18, 0.0, 0.0)),
        material=chrome,
        name="chute_floor",
    )
    base.visual(
        Box((0.010, 0.074, 0.028)),
        origin=Origin(xyz=(-0.050, -0.130, 0.044), rpy=(0.18, 0.0, 0.0)),
        material=chrome,
        name="chute_side_0",
    )
    base.visual(
        Box((0.010, 0.074, 0.028)),
        origin=Origin(xyz=(0.050, -0.130, 0.044), rpy=(0.18, 0.0, 0.0)),
        material=chrome,
        name="chute_side_1",
    )
    base.visual(
        Box((0.105, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.171, 0.034), rpy=(0.18, 0.0, 0.0)),
        material=chrome,
        name="chute_lip",
    )

    # Rear break line so the service door clearly belongs to the back of the base.
    base.visual(
        Box((0.144, 0.003, 0.102)),
        origin=Origin(xyz=(-0.005, 0.110, 0.087)),
        material=dark_shadow,
        name="service_recess",
    )
    base.visual(
        Box((0.014, 0.003, 0.105)),
        origin=Origin(xyz=(-0.070, 0.1105, 0.087)),
        material=brass,
        name="service_hinge_mount",
    )

    canister = model.part("canister")
    canister.visual(
        mesh_from_geometry(
            _tapered_octagonal_shell(0.112, 0.100, 0.0045, 0.272),
            "clear_canister_shell",
        ),
        material=glass,
        name="clear_shell",
    )
    for index, (x, y) in enumerate(_regular_polygon(0.1115)):
        canister.visual(
            Cylinder(radius=0.0026, length=0.270),
            origin=Origin(xyz=(x, y, 0.136)),
            material=glass_edge,
            name=f"facet_edge_{index}",
        )
    canister.visual(
        mesh_from_geometry(_jellybean_mound(), "jellybean_mound"),
        material=candy,
        name="jellybean_mound",
    )

    front_knob = model.part("front_knob")
    front_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.038,
                body_style="faceted",
                base_diameter=0.060,
                top_diameter=0.046,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0012),
            ),
            "front_knob_cap",
        ),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    front_knob.visual(
        Box((0.086, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.041, 0.0)),
        material=chrome,
        name="turn_bar",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.126, 0.008, 0.092)),
        origin=Origin(xyz=(0.063, 0.006, 0.0)),
        material=cast_red,
        name="door_panel",
    )
    service_door.visual(
        Cylinder(radius=0.0055, length=0.100),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=brass,
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.026, 0.010, 0.014)),
        origin=Origin(xyz=(0.105, 0.012, 0.0)),
        material=chrome,
        name="door_pull",
    )

    refill_cap = model.part("refill_cap")
    cap_geom = _octagonal_solid(0.109, 0.016, -0.008).translate(0.0, -0.118, 0.0)
    refill_cap.visual(
        mesh_from_geometry(cap_geom, "refill_cap_panel"),
        material=chrome,
        name="cap_panel",
    )
    refill_cap.visual(
        Box((0.075, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, 0.000)),
        material=chrome,
        name="rear_hinge_leaf",
    )
    refill_cap.visual(
        Cylinder(radius=0.006, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_hinge_barrel",
    )

    model.articulation(
        "base_to_canister",
        ArticulationType.FIXED,
        parent=base,
        child=canister,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )
    model.articulation(
        "base_to_front_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_knob,
        origin=Origin(xyz=(0.0, -0.109, 0.089)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "base_to_service_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=service_door,
        origin=Origin(xyz=(-0.070, 0.11145, 0.087)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "canister_to_refill_cap",
        ArticulationType.REVOLUTE,
        parent=canister,
        child=refill_cap,
        origin=Origin(xyz=(0.0, 0.118, 0.280)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    canister = object_model.get_part("canister")
    front_knob = object_model.get_part("front_knob")
    service_door = object_model.get_part("service_door")
    refill_cap = object_model.get_part("refill_cap")
    knob_joint = object_model.get_articulation("base_to_front_knob")
    door_joint = object_model.get_articulation("base_to_service_door")
    cap_joint = object_model.get_articulation("canister_to_refill_cap")

    ctx.check(
        "front knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.expect_gap(
        canister,
        base,
        axis="z",
        positive_elem="clear_shell",
        negative_elem="shoulder_ring",
        max_gap=0.002,
        max_penetration=0.0,
        name="canister sits on shoulder ring",
    )
    ctx.expect_overlap(
        front_knob,
        base,
        axes="xz",
        elem_a="knob_cap",
        elem_b="knob_bushing",
        min_overlap=0.025,
        name="knob centered on bushing",
    )
    ctx.expect_gap(
        base,
        front_knob,
        axis="y",
        positive_elem="knob_bushing",
        negative_elem="knob_cap",
        max_gap=0.004,
        max_penetration=0.0,
        name="knob rear face meets shaft bushing",
    )
    ctx.expect_gap(
        refill_cap,
        canister,
        axis="z",
        positive_elem="cap_panel",
        negative_elem="clear_shell",
        max_gap=0.006,
        max_penetration=0.0,
        name="refill cap rests above canister mouth",
    )

    closed_door_aabb = ctx.part_world_aabb(service_door)
    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_world_aabb(service_door)
    ctx.check(
        "service door swings outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.030,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_cap_aabb = ctx.part_world_aabb(refill_cap)
    with ctx.pose({cap_joint: 1.10}):
        open_cap_aabb = ctx.part_world_aabb(refill_cap)
    ctx.check(
        "refill cap lifts upward on rear hinge",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.040,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    knob_pos = ctx.part_world_position(front_knob)
    with ctx.pose({knob_joint: math.tau}):
        knob_pos_rotated = ctx.part_world_position(front_knob)
    ctx.check(
        "continuous knob rotates about fixed shaft",
        knob_pos is not None
        and knob_pos_rotated is not None
        and abs(knob_pos[0] - knob_pos_rotated[0]) < 1e-6
        and abs(knob_pos[1] - knob_pos_rotated[1]) < 1e-6
        and abs(knob_pos[2] - knob_pos_rotated[2]) < 1e-6,
        details=f"rest={knob_pos}, rotated={knob_pos_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
