from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    wire_from_points,
)


OCTAGON_SIDES = 8
OCTAGON_START = math.pi / OCTAGON_SIDES


def _regular_polygon_profile(
    radius: float,
    *,
    sides: int = OCTAGON_SIDES,
    start_angle: float = OCTAGON_START,
) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (2.0 * math.pi * i / sides)),
            radius * math.sin(start_angle + (2.0 * math.pi * i / sides)),
        )
        for i in range(sides)
    ]


def _regular_polygon_section(
    radius: float,
    z: float,
    *,
    sides: int = OCTAGON_SIDES,
    start_angle: float = OCTAGON_START,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _regular_polygon_profile(radius, sides=sides, start_angle=start_angle)]


def _apothem(radius: float, *, sides: int = OCTAGON_SIDES) -> float:
    return radius * math.cos(math.pi / sides)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse")

    lighthouse_white = model.material("lighthouse_white", rgba=(0.93, 0.94, 0.92, 1.0))
    weathered_stone = model.material("weathered_stone", rgba=(0.69, 0.70, 0.67, 1.0))
    gallery_gray = model.material("gallery_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    roof_red = model.material("roof_red", rgba=(0.58, 0.12, 0.10, 1.0))
    frame_white = model.material("frame_white", rgba=(0.96, 0.97, 0.97, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.76, 0.89, 0.95, 0.34))
    bronze = model.material("bronze", rgba=(0.56, 0.43, 0.24, 1.0))
    lens_amber = model.material("lens_amber", rgba=(0.97, 0.81, 0.34, 0.58))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    door_red = model.material("door_red", rgba=(0.47, 0.10, 0.08, 1.0))

    plinth_radius = 2.75
    plinth_height = 0.72
    body_lower_radius = 2.45
    body_upper_radius = 1.78
    wall_thickness = 0.22
    lower_wall_base_z = 0.66
    lower_wall_top_z = 2.85
    tower_top_z = 5.92
    deck_base_z = 5.90
    deck_thickness = 0.18
    lantern_floor_radius = 1.40
    lantern_outer_radius = 1.38
    lantern_inner_radius = 1.18
    lantern_post_radius = 1.29
    lantern_height = 2.18
    door_width = 0.96
    door_height = 1.88
    door_thickness = 0.06

    tower_base = model.part("tower_base")
    tower_base.inertial = Inertial.from_geometry(
        Box((5.8, 5.8, 6.2)),
        mass=38000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.1)),
    )

    plinth_geom = ExtrudeGeometry.from_z0(
        _regular_polygon_profile(plinth_radius),
        plinth_height,
        cap=True,
        closed=True,
    )
    tower_base.visual(
        _mesh("plinth_shell", plinth_geom),
        material=weathered_stone,
        name="plinth_shell",
    )

    tower_geom = section_loft(
        [
            _regular_polygon_section(body_lower_radius, lower_wall_top_z),
            _regular_polygon_section(2.20, 4.20),
            _regular_polygon_section(body_upper_radius, tower_top_z),
        ]
    )
    tower_base.visual(
        _mesh("tower_shell_upper", tower_geom),
        material=lighthouse_white,
        name="tower_shell",
    )

    gallery_ring_geom = ExtrudeWithHolesGeometry(
        _regular_polygon_profile(2.24),
        [_regular_polygon_profile(1.22)],
        deck_thickness,
        center=False,
        closed=True,
        cap=True,
    )
    tower_base.visual(
        _mesh("gallery_ring", gallery_ring_geom),
        origin=Origin(xyz=(0.0, 0.0, deck_base_z)),
        material=gallery_gray,
        name="gallery_ring",
    )

    lantern_floor_geom = ExtrudeGeometry.from_z0(
        _regular_polygon_profile(lantern_floor_radius),
        deck_thickness,
        cap=True,
        closed=True,
    )
    tower_base.visual(
        _mesh("lantern_floor", lantern_floor_geom),
        origin=Origin(xyz=(0.0, 0.0, deck_base_z)),
        material=gallery_gray,
        name="lantern_floor",
    )

    rail_radius = 2.02
    for index, (x, y) in enumerate(_regular_polygon_profile(rail_radius)):
        tower_base.visual(
            Cylinder(radius=0.03, length=1.05),
            origin=Origin(xyz=(x, y, deck_base_z + 0.525)),
            material=gallery_gray,
            name=f"gallery_post_{index:02d}",
        )
    rail_vertices = _regular_polygon_profile(rail_radius)
    for index in range(OCTAGON_SIDES):
        x0, y0 = rail_vertices[index]
        x1, y1 = rail_vertices[(index + 1) % OCTAGON_SIDES]
        seg_dx = x1 - x0
        seg_dy = y1 - y0
        seg_len = math.hypot(seg_dx, seg_dy) + 0.06
        seg_yaw = math.atan2(seg_dy, seg_dx)
        seg_mid_x = 0.5 * (x0 + x1)
        seg_mid_y = 0.5 * (y0 + y1)
        tower_base.visual(
            Box((seg_len, 0.05, 0.05)),
            origin=Origin(xyz=(seg_mid_x, seg_mid_y, deck_base_z + 1.02), rpy=(0.0, 0.0, seg_yaw)),
            material=gallery_gray,
            name=f"gallery_top_segment_{index:02d}",
        )
        tower_base.visual(
            Box((seg_len, 0.04, 0.04)),
            origin=Origin(xyz=(seg_mid_x, seg_mid_y, deck_base_z + 0.55), rpy=(0.0, 0.0, seg_yaw)),
            material=gallery_gray,
            name=f"gallery_mid_segment_{index:02d}",
        )

    door_frame_x = _apothem(body_lower_radius)
    door_mount_x = door_frame_x + 0.035
    frame_center_z = 1.72
    lower_wall_center_z = 0.5 * (lower_wall_base_z + lower_wall_top_z)
    lower_wall_height = lower_wall_top_z - lower_wall_base_z
    lower_face_radius = door_frame_x - wall_thickness * 0.5
    side_length = 2.0 * body_lower_radius * math.sin(math.pi / OCTAGON_SIDES)
    full_panel_width = side_length + 0.06
    for face_index in range(1, OCTAGON_SIDES):
        face_angle = 2.0 * math.pi * face_index / OCTAGON_SIDES
        tower_base.visual(
            Box((wall_thickness, full_panel_width, lower_wall_height)),
            origin=Origin(
                xyz=(
                    lower_face_radius * math.cos(face_angle),
                    lower_face_radius * math.sin(face_angle),
                    lower_wall_center_z,
                ),
                rpy=(0.0, 0.0, face_angle),
            ),
            material=lighthouse_white,
            name=f"lower_wall_face_{face_index:02d}",
        )
    front_piece_width = 0.46
    for side_name, y_center in (
        ("left", -(door_width * 0.5 + front_piece_width * 0.5)),
        ("right", door_width * 0.5 + front_piece_width * 0.5),
    ):
        tower_base.visual(
            Box((wall_thickness, front_piece_width, lower_wall_height)),
            origin=Origin(xyz=(lower_face_radius, y_center, lower_wall_center_z)),
            material=lighthouse_white,
            name=f"lower_wall_front_{side_name}",
        )
    tower_base.visual(
        Box((0.02, 0.10, door_height + 0.20)),
        origin=Origin(xyz=(door_frame_x - 0.01, -0.53, frame_center_z)),
        material=weathered_stone,
        name="door_frame_left",
    )
    tower_base.visual(
        Box((0.04, 0.10, door_height + 0.20)),
        origin=Origin(xyz=(door_frame_x - 0.005, 0.53, frame_center_z)),
        material=weathered_stone,
        name="door_frame_right",
    )
    tower_base.visual(
        Box((0.04, 1.16, 0.12)),
        origin=Origin(xyz=(door_frame_x - 0.005, 0.0, frame_center_z + door_height * 0.5 + 0.06)),
        material=weathered_stone,
        name="door_lintel",
    )
    tower_base.visual(
        Cylinder(radius=0.02, length=1.90),
        origin=Origin(xyz=(door_mount_x, -door_width * 0.5, frame_center_z)),
        material=dark_metal,
        name="hinge_pintle",
    )

    lantern_room = model.part("lantern_room")
    lantern_room.inertial = Inertial.from_geometry(
        Box((3.2, 3.2, lantern_height)),
        mass=2200.0,
        origin=Origin(xyz=(0.0, 0.0, lantern_height * 0.5)),
    )

    base_ring_geom = ExtrudeWithHolesGeometry(
        _regular_polygon_profile(lantern_outer_radius),
        [_regular_polygon_profile(lantern_inner_radius)],
        0.18,
        center=False,
        closed=True,
        cap=True,
    )
    lantern_room.visual(
        _mesh("lantern_base_ring", base_ring_geom),
        material=frame_white,
        name="base_ring",
    )

    top_ring_geom = ExtrudeWithHolesGeometry(
        _regular_polygon_profile(1.50),
        [_regular_polygon_profile(1.12)],
        0.16,
        center=False,
        closed=True,
        cap=True,
    )
    lantern_room.visual(
        _mesh("lantern_top_ring", top_ring_geom),
        origin=Origin(xyz=(0.0, 0.0, lantern_height - 0.16)),
        material=frame_white,
        name="top_ring",
    )

    for index, (x, y) in enumerate(_regular_polygon_profile(lantern_post_radius)):
        lantern_room.visual(
            Box((0.10, 0.10, lantern_height)),
            origin=Origin(xyz=(x, y, lantern_height * 0.5)),
            material=frame_white,
            name=f"corner_post_{index:02d}",
        )

    panel_height = lantern_height - 0.34
    panel_center_z = 0.18 + panel_height * 0.5
    panel_width = 0.90
    panel_face_radius = _apothem(lantern_post_radius)
    for index in range(OCTAGON_SIDES):
        face_angle = 2.0 * math.pi * index / OCTAGON_SIDES
        lantern_room.visual(
            Box((0.03, panel_width, panel_height)),
            origin=Origin(
                xyz=(
                    panel_face_radius * math.cos(face_angle),
                    panel_face_radius * math.sin(face_angle),
                    panel_center_z,
                ),
                rpy=(0.0, 0.0, face_angle),
            ),
            material=glass_tint,
            name=f"glass_panel_{index:02d}",
        )

    roof_cap = model.part("roof_cap")
    roof_cap.inertial = Inertial.from_geometry(
        Box((3.2, 3.2, 1.45)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )

    roof_geom = section_loft(
        [
            _regular_polygon_section(1.55, 0.0),
            _regular_polygon_section(1.08, 0.58),
            _regular_polygon_section(0.38, 0.92),
        ]
    )
    roof_cap.visual(
        _mesh("roof_shell", roof_geom),
        material=roof_red,
        name="roof_shell",
    )
    roof_cap.visual(
        Cylinder(radius=0.18, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=dark_metal,
        name="vent_cap",
    )
    roof_cap.visual(
        Cylinder(radius=0.055, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 1.37)),
        material=dark_metal,
        name="finial",
    )

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.38, length=1.18),
        mass=750.0,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
    )
    pedestal.visual(
        Cylinder(radius=0.38, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=bronze,
        name="pedestal_base",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=bronze,
        name="pedestal_shaft",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=dark_metal,
        name="bearing_housing",
    )

    light_head = model.part("light_head")
    light_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.74, length=1.02),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )
    light_head.visual(
        Cylinder(radius=0.10, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_metal,
        name="spindle",
    )
    light_head.visual(
        Cylinder(radius=0.46, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark_metal,
        name="lower_rotor_ring",
    )
    light_head.visual(
        Cylinder(radius=0.46, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=dark_metal,
        name="upper_rotor_ring",
    )
    light_head.visual(
        Cylinder(radius=0.18, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=bronze,
        name="lamp_core",
    )
    light_head.visual(
        Box((1.00, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=dark_metal,
        name="crossbeam_x",
    )
    light_head.visual(
        Box((0.10, 1.00, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=dark_metal,
        name="crossbeam_y",
    )
    light_head.visual(
        Box((0.14, 0.58, 0.66)),
        origin=Origin(xyz=(0.56, 0.0, 0.53)),
        material=lens_amber,
        name="panel_east",
    )
    light_head.visual(
        Box((0.14, 0.58, 0.66)),
        origin=Origin(xyz=(-0.56, 0.0, 0.53)),
        material=lens_amber,
        name="panel_west",
    )
    light_head.visual(
        Box((0.58, 0.14, 0.66)),
        origin=Origin(xyz=(0.0, 0.56, 0.53)),
        material=lens_amber,
        name="panel_north",
    )
    light_head.visual(
        Box((0.58, 0.14, 0.66)),
        origin=Origin(xyz=(0.0, -0.56, 0.53)),
        material=lens_amber,
        name="panel_south",
    )
    light_head.visual(
        Box((0.76, 0.10, 0.10)),
        origin=Origin(xyz=(0.27, 0.0, 0.53)),
        material=dark_metal,
        name="east_support",
    )
    light_head.visual(
        Box((0.76, 0.10, 0.10)),
        origin=Origin(xyz=(-0.27, 0.0, 0.53)),
        material=dark_metal,
        name="west_support",
    )
    light_head.visual(
        Box((0.10, 0.76, 0.10)),
        origin=Origin(xyz=(0.0, 0.27, 0.53)),
        material=dark_metal,
        name="north_support",
    )
    light_head.visual(
        Box((0.10, 0.76, 0.10)),
        origin=Origin(xyz=(0.0, -0.27, 0.53)),
        material=dark_metal,
        name="south_support",
    )
    light_head.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.93)),
        material=dark_metal,
        name="top_cap",
    )

    service_door = model.part("service_door")
    service_door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=85.0,
        origin=Origin(xyz=(0.03, 0.45, 0.0)),
    )
    service_door.visual(
        Cylinder(radius=0.035, length=door_height),
        origin=Origin(),
        material=dark_metal,
        name="hinge_barrel",
    )
    service_door.visual(
        Box((0.08, 0.90, door_height)),
        origin=Origin(xyz=(0.03, 0.45, 0.0)),
        material=door_red,
        name="door_leaf",
    )
    service_door.visual(
        Box((0.02, 0.74, 1.70)),
        origin=Origin(xyz=(0.05, 0.45, 0.0)),
        material=frame_white,
        name="door_trim",
    )
    service_door.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(
            xyz=(0.05, 0.76, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bronze,
        name="door_handle",
    )

    model.articulation(
        "tower_to_lantern_room",
        ArticulationType.FIXED,
        parent=tower_base,
        child=lantern_room,
        origin=Origin(xyz=(0.0, 0.0, deck_base_z + deck_thickness)),
    )
    model.articulation(
        "lantern_to_roof_cap",
        ArticulationType.FIXED,
        parent=lantern_room,
        child=roof_cap,
        origin=Origin(xyz=(0.0, 0.0, lantern_height)),
    )
    model.articulation(
        "tower_to_pedestal",
        ArticulationType.FIXED,
        parent=tower_base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, deck_base_z + deck_thickness)),
    )
    model.articulation(
        "pedestal_to_light_head",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.9),
    )
    model.articulation(
        "tower_to_service_door",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=service_door,
        origin=Origin(xyz=(door_mount_x, -door_width * 0.5, frame_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    tower_base = object_model.get_part("tower_base")
    lantern_room = object_model.get_part("lantern_room")
    roof_cap = object_model.get_part("roof_cap")
    pedestal = object_model.get_part("pedestal")
    light_head = object_model.get_part("light_head")
    service_door = object_model.get_part("service_door")

    light_spin = object_model.get_articulation("pedestal_to_light_head")
    door_hinge = object_model.get_articulation("tower_to_service_door")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        tower_base,
        service_door,
        elem_a="hinge_pintle",
        elem_b="hinge_barrel",
        reason="Door hinge barrel wraps around a fixed pintle on the tower face.",
    )
    ctx.allow_overlap(
        tower_base,
        service_door,
        elem_a="hinge_pintle",
        elem_b="door_leaf",
        reason="The hinge pin passes through the door's hinge-side knuckle region, which is represented within the door leaf slab.",
    )
    ctx.allow_overlap(
        tower_base,
        service_door,
        elem_a="hinge_pintle",
        elem_b="hinge_stile",
        reason="The hinge stile represents the knuckle reinforcement wrapped around the hinge pin.",
    )
    ctx.allow_overlap(
        pedestal,
        light_head,
        elem_a="bearing_housing",
        elem_b="spindle",
        reason="The beacon spindle is captured inside the pedestal bearing housing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        lantern_room,
        tower_base,
        axis="z",
        positive_elem="base_ring",
        negative_elem="lantern_floor",
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        roof_cap,
        lantern_room,
        axis="z",
        positive_elem="roof_shell",
        negative_elem="top_ring",
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        pedestal,
        tower_base,
        axis="z",
        positive_elem="pedestal_base",
        negative_elem="lantern_floor",
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(light_head, pedestal, elem_a="spindle", elem_b="bearing_housing")
    ctx.expect_contact(service_door, tower_base, elem_a="hinge_barrel", elem_b="hinge_pintle")
    ctx.expect_within(pedestal, lantern_room, axes="xy", margin=0.18)
    ctx.expect_within(light_head, lantern_room, axes="xy", margin=0.10)
    ctx.expect_overlap(light_head, lantern_room, axes="xy", min_overlap=1.0)

    def _aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    door_leaf_rest = ctx.part_element_world_aabb(service_door, elem="door_leaf")
    panel_east_rest = ctx.part_element_world_aabb(light_head, elem="panel_east")
    assert door_leaf_rest is not None
    assert panel_east_rest is not None
    door_leaf_rest_center = _aabb_center(door_leaf_rest)
    panel_east_rest_center = _aabb_center(panel_east_rest)

    with ctx.pose({light_spin: math.pi / 2.0}):
        ctx.expect_contact(light_head, pedestal, elem_a="spindle", elem_b="bearing_housing")
        ctx.expect_within(light_head, lantern_room, axes="xy", margin=0.10)
        ctx.fail_if_parts_overlap_in_current_pose(name="light_spin_quarter_no_overlap")
        ctx.fail_if_isolated_parts(name="light_spin_quarter_no_floating")
        panel_east_turned = ctx.part_element_world_aabb(light_head, elem="panel_east")
        assert panel_east_turned is not None
        panel_east_turned_center = _aabb_center(panel_east_turned)
        ctx.check(
            "light_head_quarter_turn_y_shift",
            panel_east_turned_center[1] > panel_east_rest_center[1] + 0.40,
            details=f"expected +y shift after quarter turn, rest={panel_east_rest_center}, turned={panel_east_turned_center}",
        )
        ctx.check(
            "light_head_quarter_turn_x_shift",
            panel_east_turned_center[0] < panel_east_rest_center[0] - 0.40,
            details=f"expected -x shift after quarter turn, rest={panel_east_rest_center}, turned={panel_east_turned_center}",
        )

    limits = door_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="service_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="service_door_lower_no_floating")
            ctx.expect_contact(service_door, tower_base, elem_a="hinge_barrel", elem_b="hinge_pintle")
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="service_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="service_door_upper_no_floating")
            ctx.expect_contact(service_door, tower_base, elem_a="hinge_barrel", elem_b="hinge_pintle")
            door_leaf_open = ctx.part_element_world_aabb(service_door, elem="door_leaf")
            assert door_leaf_open is not None
            door_leaf_open_center = _aabb_center(door_leaf_open)
            ctx.check(
                "service_door_open_x_shift",
                door_leaf_open_center[0] > door_leaf_rest_center[0] + 0.30,
                details=f"expected outward x shift, rest={door_leaf_rest_center}, open={door_leaf_open_center}",
            )
            ctx.check(
                "service_door_open_y_shift",
                door_leaf_open_center[1] < door_leaf_rest_center[1] - 0.40,
                details=f"expected negative y swing, rest={door_leaf_rest_center}, open={door_leaf_open_center}",
            )

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
