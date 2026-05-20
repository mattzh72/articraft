from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


WOOD = Material("weathered_oak", color=(0.55, 0.37, 0.19, 1.0))
DARK_WOOD = Material("aged_dark_wood", color=(0.28, 0.18, 0.10, 1.0))
STONE = Material("warm_limewash_stone", color=(0.68, 0.62, 0.50, 1.0))
STEEL = Material("galvanized_dark_steel", color=(0.22, 0.25, 0.26, 1.0))
BRIGHT_STEEL = Material("brushed_bolt_heads", color=(0.62, 0.64, 0.61, 1.0))
ROOF_RED = Material("oxide_red_cap", color=(0.55, 0.08, 0.05, 1.0))
SHADOW = Material("dark_recess_shadow", color=(0.04, 0.035, 0.03, 1.0))


def _square_profile(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (-width / 2.0, -depth / 2.0, z),
        (width / 2.0, -depth / 2.0, z),
        (width / 2.0, depth / 2.0, z),
        (-width / 2.0, depth / 2.0, z),
    ]


def _tower_half_width(z: float) -> float:
    lower_z, upper_z = 0.12, 3.25
    lower_half, upper_half = 0.62, 0.34
    t = max(0.0, min(1.0, (z - lower_z) / (upper_z - lower_z)))
    return lower_half + (upper_half - lower_half) * t


def _hollow_cylinder_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 56,
):
    mesh = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(mesh, name)


def _add_x_cylinder(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_rotor_blade(rotor, index: int, angle: float) -> None:
    """Add one old-school sail: spar, two edge rails, and bolted lattice slats."""
    radial = (0.0, math.cos(angle), math.sin(angle))
    tangent = (0.0, math.sin(angle), -math.cos(angle))
    spar_rpy = (angle - math.pi / 2.0, 0.0, 0.0)
    slat_rpy = (angle + math.pi, 0.0, 0.0)

    def point(radial_distance: float, tangent_offset: float, x: float = 0.34):
        return (
            x,
            radial[1] * radial_distance + tangent[1] * tangent_offset,
            radial[2] * radial_distance + tangent[2] * tangent_offset,
        )

    # Main spar runs continuously through the clamp and ties every lattice member back to the hub.
    rotor.visual(
        Box((0.045, 0.055, 1.22)),
        origin=Origin(xyz=point(0.66, 0.0), rpy=spar_rpy),
        material=WOOD,
        name=f"blade_{index}_spar",
    )

    # Two outer rails give the sail a traditional lattice outline.
    for side, offset in (("a", -0.125), ("b", 0.125)):
        rotor.visual(
            Box((0.032, 0.038, 0.92)),
            origin=Origin(xyz=point(0.78, offset), rpy=spar_rpy),
            material=WOOD,
            name=f"blade_{index}_rail_{side}",
        )

    # Cross slats overlap the rails slightly so the blade is a single supported frame.
    for slat_i, radial_distance in enumerate((0.36, 0.55, 0.74, 0.93, 1.12)):
        rotor.visual(
            Box((0.026, 0.030, 0.34)),
            origin=Origin(xyz=point(radial_distance, 0.0, x=0.345), rpy=slat_rpy),
            material=DARK_WOOD,
            name=f"blade_{index}_slat_{slat_i}",
        )

    # A galvanized root strap reads as a modern retrofit clamp at the hub.
    rotor.visual(
        Box((0.060, 0.090, 0.28)),
        origin=Origin(xyz=point(0.20, 0.0, x=0.335), rpy=spar_rpy),
        material=STEEL,
        name=f"blade_{index}_root_clamp",
    )

    # Two small bolt heads on each clamp are slightly proud of the strap.
    for bolt_i, radial_distance in enumerate((0.13, 0.27)):
        _add_x_cylinder(
            rotor,
            f"blade_{index}_bolt_{bolt_i}",
            radius=0.018,
            length=0.018,
            xyz=point(radial_distance, 0.042, x=0.370),
            material=BRIGHT_STEEL,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_traditional_windmill")

    tower = model.part("tower")

    tower_body = LoftGeometry(
        [
            _square_profile(1.24, 1.24, 0.12),
            _square_profile(1.02, 1.02, 1.40),
            _square_profile(0.78, 0.78, 2.55),
            _square_profile(0.68, 0.68, 3.25),
        ],
        cap=True,
        closed=True,
    )
    tower.visual(mesh_from_geometry(tower_body, "tapered_tower_body"), material=STONE, name="tower_body")
    tower.visual(Box((1.55, 1.55, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.08)), material=STONE, name="foundation_pad")

    # Old-school timber bands and corner straps are mounted proud of the tapered body.
    for band_i, z in enumerate((0.42, 1.28, 2.18, 3.05)):
        face_half = _tower_half_width(z) + 0.018
        span = 2.0 * face_half + 0.08
        tower.visual(Box((0.050, span, 0.075)), origin=Origin(xyz=(face_half, 0.0, z)), material=DARK_WOOD, name=f"front_band_{band_i}")
        tower.visual(Box((0.050, span, 0.075)), origin=Origin(xyz=(-face_half, 0.0, z)), material=DARK_WOOD, name=f"rear_band_{band_i}")
        tower.visual(Box((span, 0.050, 0.075)), origin=Origin(xyz=(0.0, face_half, z)), material=DARK_WOOD, name=f"side_band_{band_i}")
        tower.visual(Box((span, 0.050, 0.075)), origin=Origin(xyz=(0.0, -face_half, z)), material=DARK_WOOD, name=f"far_side_band_{band_i}")

    # Service hatches are visible, bolted, and seated into the tower skin.
    door_z = 0.70
    door_x = _tower_half_width(door_z) + 0.030
    tower.visual(Box((0.060, 0.36, 0.72)), origin=Origin(xyz=(door_x, 0.0, door_z)), material=SHADOW, name="service_door")
    tower.visual(Box((0.068, 0.45, 0.055)), origin=Origin(xyz=(door_x + 0.006, 0.0, door_z + 0.39)), material=STEEL, name="service_door_lintel")
    tower.visual(Box((0.068, 0.45, 0.055)), origin=Origin(xyz=(door_x + 0.006, 0.0, door_z - 0.39)), material=STEEL, name="service_door_sill")
    tower.visual(Box((0.068, 0.055, 0.78)), origin=Origin(xyz=(door_x + 0.006, 0.225, door_z)), material=STEEL, name="service_door_jamb_0")
    tower.visual(Box((0.068, 0.055, 0.78)), origin=Origin(xyz=(door_x + 0.006, -0.225, door_z)), material=STEEL, name="service_door_jamb_1")

    hatch_z = 2.18
    hatch_x = _tower_half_width(hatch_z) + 0.030
    tower.visual(Box((0.055, 0.34, 0.38)), origin=Origin(xyz=(hatch_x, 0.0, hatch_z)), material=SHADOW, name="upper_hatch")
    tower.visual(Box((0.065, 0.45, 0.055)), origin=Origin(xyz=(hatch_x + 0.006, 0.0, hatch_z + 0.23)), material=STEEL, name="upper_hatch_top_frame")
    tower.visual(Box((0.065, 0.45, 0.055)), origin=Origin(xyz=(hatch_x + 0.006, 0.0, hatch_z - 0.23)), material=STEEL, name="upper_hatch_bottom_frame")
    tower.visual(Box((0.065, 0.055, 0.44)), origin=Origin(xyz=(hatch_x + 0.006, 0.225, hatch_z)), material=STEEL, name="upper_hatch_frame_0")
    tower.visual(Box((0.065, 0.055, 0.44)), origin=Origin(xyz=(hatch_x + 0.006, -0.225, hatch_z)), material=STEEL, name="upper_hatch_frame_1")

    for i, (y, z) in enumerate(((-0.18, 0.34), (0.18, 0.34), (-0.18, 1.06), (0.18, 1.06))):
        _add_x_cylinder(tower, f"service_bolt_{i}", 0.022, 0.018, (door_x + 0.045, y, z), BRIGHT_STEEL)
    for i, (y, z) in enumerate(((-0.17, 1.99), (0.17, 1.99), (-0.17, 2.37), (0.17, 2.37))):
        _add_x_cylinder(tower, f"hatch_bolt_{i}", 0.018, 0.016, (hatch_x + 0.042, y, z), BRIGHT_STEEL)

    # Tapered body to cap adapter: ring below the yawing cap, with four retrofit gusset pads.
    tower.visual(
        _hollow_cylinder_mesh("yaw_ring_mesh", outer_radius=0.39, inner_radius=0.22, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 3.27)),
        material=STEEL,
        name="yaw_ring",
    )
    for gusset_i, (x, y) in enumerate(((0.34, 0.0), (-0.34, 0.0), (0.0, 0.34), (0.0, -0.34))):
        tower.visual(Box((0.16 if y else 0.055, 0.055 if y else 0.16, 0.26)), origin=Origin(xyz=(x, y, 3.13)), material=STEEL, name=f"tower_gusset_{gusset_i}")

    cap = model.part("cap")
    cap.visual(Box((0.96, 0.58, 0.07)), origin=Origin(xyz=(0.30, 0.0, 0.035)), material=STEEL, name="yaw_bearing_plate")
    cap.visual(Box((0.96, 0.10, 0.46)), origin=Origin(xyz=(0.30, 0.240, 0.30)), material=WOOD, name="nacelle_side_0")
    cap.visual(Box((0.96, 0.10, 0.46)), origin=Origin(xyz=(0.30, -0.240, 0.30)), material=WOOD, name="nacelle_side_1")
    cap.visual(Box((0.10, 0.58, 0.18)), origin=Origin(xyz=(-0.17, 0.0, 0.43)), material=WOOD, name="rear_bulkhead")
    cap.visual(Box((0.95, 0.40, 0.055)), origin=Origin(xyz=(0.28, 0.145, 0.61), rpy=(-0.42, 0.0, 0.0)), material=ROOF_RED, name="roof_panel_0")
    cap.visual(Box((0.95, 0.40, 0.055)), origin=Origin(xyz=(0.28, -0.145, 0.61), rpy=(0.42, 0.0, 0.0)), material=ROOF_RED, name="roof_panel_1")
    cap.visual(Box((0.97, 0.070, 0.110)), origin=Origin(xyz=(0.28, 0.0, 0.69)), material=ROOF_RED, name="roof_ridge_cap")

    # Two clear bearing seats cradle the rotor shaft without turning the nacelle into a solid block.
    cap.visual(
        _hollow_cylinder_mesh("front_bearing_seat_mesh", outer_radius=0.19, inner_radius=0.078, length=0.08),
        origin=Origin(xyz=(0.70, 0.0, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="front_bearing_seat",
    )
    cap.visual(
        _hollow_cylinder_mesh("rear_bearing_seat_mesh", outer_radius=0.16, inner_radius=0.078, length=0.08),
        origin=Origin(xyz=(0.38, 0.0, 0.42), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="rear_bearing_seat",
    )
    cap.visual(Box((0.18, 0.52, 0.055)), origin=Origin(xyz=(0.70, 0.0, 0.205)), material=STEEL, name="front_bearing_saddle")
    cap.visual(Box((0.16, 0.48, 0.075)), origin=Origin(xyz=(0.38, 0.0, 0.245)), material=STEEL, name="rear_bearing_saddle")

    for bearing_name, x, radius in (("front", 0.70, 0.145), ("rear", 0.38, 0.120)):
        for bolt_i, theta in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
            _add_x_cylinder(
                cap,
                f"{bearing_name}_bearing_bolt_{bolt_i}",
                radius=0.015,
                length=0.020,
                xyz=(x + 0.045, radius * math.cos(theta), 0.42 + radius * math.sin(theta)),
                material=BRIGHT_STEEL,
            )

    # A tail vane is tied into the rear bulkhead, giving the cap a traditional self-yawing cue.
    cap.visual(Box((0.72, 0.045, 0.045)), origin=Origin(xyz=(-0.49, 0.0, 0.43)), material=STEEL, name="tail_boom")
    cap.visual(Box((0.045, 0.44, 0.30)), origin=Origin(xyz=(-0.84, 0.0, 0.53)), material=STEEL, name="tail_vane")

    rotor = model.part("rotor")
    _add_x_cylinder(rotor, "main_shaft", radius=0.055, length=0.72, xyz=(-0.19, 0.0, 0.0), material=STEEL)
    _add_x_cylinder(rotor, "hub_shell", radius=0.155, length=0.24, xyz=(0.22, 0.0, 0.0), material=STEEL)
    _add_x_cylinder(rotor, "front_hub_cap", radius=0.185, length=0.055, xyz=(0.365, 0.0, 0.0), material=BRIGHT_STEEL)
    _add_x_cylinder(rotor, "front_thrust_washer", radius=0.115, length=0.024, xyz=(0.052, 0.0, 0.0), material=BRIGHT_STEEL)
    _add_x_cylinder(rotor, "rear_shaft_collar", radius=0.086, length=0.055, xyz=(-0.115, 0.0, 0.0), material=BRIGHT_STEEL)

    for blade_i, blade_angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        _add_rotor_blade(rotor, blade_i, blade_angle)

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 3.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.30, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor,
        origin=Origin(xyz=(0.70, 0.0, 0.42)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_cap")
    spin = object_model.get_articulation("cap_to_rotor")

    ctx.check(
        "legacy windmill has tower cap and rotor stages",
        tower is not None and cap is not None and rotor is not None and yaw is not None and spin is not None,
        details="Expected tower, cap, rotor, yaw joint, and rotor shaft joint.",
    )
    ctx.expect_contact(
        cap,
        tower,
        elem_a="yaw_bearing_plate",
        elem_b="yaw_ring",
        contact_tol=0.002,
        name="cap plate is seated on tower yaw ring",
    )
    ctx.expect_within(
        rotor,
        cap,
        axes="yz",
        inner_elem="main_shaft",
        outer_elem="front_bearing_seat",
        margin=0.0,
        name="main shaft is centered inside front bearing seat envelope",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="x",
        elem_a="main_shaft",
        elem_b="front_bearing_seat",
        min_overlap=0.055,
        name="main shaft passes through front bearing seat",
    )
    ctx.expect_overlap(
        rotor,
        cap,
        axes="x",
        elem_a="main_shaft",
        elem_b="rear_bearing_seat",
        min_overlap=0.055,
        name="main shaft remains engaged in rear bearing seat",
    )

    rest_blade = ctx.part_element_world_aabb(rotor, elem="blade_0_spar")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_blade = ctx.part_element_world_aabb(rotor, elem="blade_0_spar")
    if rest_blade is not None and spun_blade is not None:
        rest_center_z = (rest_blade[0][2] + rest_blade[1][2]) / 2.0
        spun_center_z = (spun_blade[0][2] + spun_blade[1][2]) / 2.0
    else:
        rest_center_z = spun_center_z = None
    ctx.check(
        "rotor lattice spins about supported shaft",
        rest_center_z is not None and spun_center_z is not None and spun_center_z > rest_center_z + 0.25,
        details=f"rest blade center z={rest_center_z}, spun blade center z={spun_center_z}",
    )

    rest_rotor_origin = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.65}):
        yawed_rotor_origin = ctx.part_world_position(rotor)
    ctx.check(
        "cap yaws as a complete nacelle stage",
        rest_rotor_origin is not None
        and yawed_rotor_origin is not None
        and abs(yawed_rotor_origin[1] - rest_rotor_origin[1]) > 0.35,
        details=f"rest={rest_rotor_origin}, yawed={yawed_rotor_origin}",
    )

    return ctx.report()


object_model = build_object_model()
