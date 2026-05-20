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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _octagon_profile(apothem: float) -> list[tuple[float, float]]:
    vertex_radius = apothem / math.cos(math.pi / 8.0)
    return [
        (
            vertex_radius * math.cos(math.pi / 8.0 + index * math.pi / 4.0),
            vertex_radius * math.sin(math.pi / 8.0 + index * math.pi / 4.0),
        )
        for index in range(8)
    ]


def _octagon_section(apothem: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in _octagon_profile(apothem)]


def _face_center(apothem: float, angle: float) -> tuple[float, float]:
    return (apothem * math.cos(angle), apothem * math.sin(angle))


def _vertex_point(apothem: float, angle: float) -> tuple[float, float]:
    vertex_radius = apothem / math.cos(math.pi / 8.0)
    return (vertex_radius * math.cos(angle), vertex_radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_lighthouse_lantern")

    masonry = model.material("masonry", rgba=(0.73, 0.71, 0.66, 1.0))
    metal = model.material("lantern_metal", rgba=(0.23, 0.27, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    glass = model.material("lantern_glass", rgba=(0.82, 0.90, 0.95, 0.32))
    lens_glass = model.material("lens_glass", rgba=(0.95, 0.89, 0.55, 0.45))
    brass = model.material("brass", rgba=(0.74, 0.60, 0.26, 1.0))

    tower_height = 5.50
    gallery_thickness = 0.20
    lantern_curb_height = 0.22
    frame_bottom_z = tower_height + gallery_thickness + lantern_curb_height
    frame_height = 1.82
    top_ring_z = frame_bottom_z + frame_height
    top_ring_height = 0.16
    roof_base_z = top_ring_z + top_ring_height

    tower_apothem_bottom = 1.85
    tower_apothem_mid = 1.58
    tower_apothem_top = 1.36
    gallery_apothem = 1.72
    lantern_outer_apothem = 1.14
    lantern_face_apothem = 1.08
    top_ring_inner_apothem = 0.80

    lighthouse_body = model.part("lighthouse_body")

    masonry_tower_mesh = mesh_from_geometry(
        section_loft(
            [
                _octagon_section(tower_apothem_bottom, 0.0),
                _octagon_section(tower_apothem_mid, tower_height * 0.46),
                _octagon_section(tower_apothem_top, tower_height),
            ]
        ),
        "masonry_tower",
    )
    lighthouse_body.visual(
        masonry_tower_mesh,
        material=masonry,
        name="masonry_tower",
    )

    gallery_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_octagon_profile(gallery_apothem), gallery_thickness),
        "gallery_deck",
    )
    lighthouse_body.visual(
        gallery_mesh,
        origin=Origin(xyz=(0.0, 0.0, tower_height)),
        material=masonry,
        name="gallery_deck",
    )

    lantern_base_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_octagon_profile(lantern_outer_apothem), lantern_curb_height),
        "lantern_base_curb",
    )
    lighthouse_body.visual(
        lantern_base_mesh,
        origin=Origin(xyz=(0.0, 0.0, tower_height + gallery_thickness)),
        material=metal,
        name="lantern_base_curb",
    )

    top_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _octagon_profile(lantern_outer_apothem),
            [_octagon_profile(top_ring_inner_apothem)],
            top_ring_height,
            center=False,
        ),
        "lantern_top_ring",
    )
    lighthouse_body.visual(
        top_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, top_ring_z)),
        material=metal,
        name="lantern_top_ring",
    )

    roof_mesh = mesh_from_geometry(
        section_loft(
            [
                _octagon_section(lantern_outer_apothem, 0.0),
                _octagon_section(0.74, 0.34),
                _octagon_section(0.18, 1.02),
            ]
        ),
        "lantern_roof",
    )
    lighthouse_body.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, roof_base_z)),
        material=metal,
        name="lantern_roof",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.12, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 1.14)),
        material=dark_metal,
        name="roof_vent",
    )
    lighthouse_body.visual(
        Sphere(radius=0.08),
        origin=Origin(xyz=(0.0, 0.0, roof_base_z + 1.34)),
        material=dark_metal,
        name="roof_finial",
    )

    post_height = roof_base_z - frame_bottom_z
    for index in range(8):
        angle = math.pi / 8.0 + index * math.pi / 4.0
        vx, vy = _vertex_point(lantern_face_apothem, angle)
        lighthouse_body.visual(
            Box((0.09, 0.09, post_height)),
            origin=Origin(
                xyz=(vx, vy, frame_bottom_z + post_height * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=metal,
            name=f"corner_post_{index}",
        )

    pane_thickness = 0.028
    pane_width = 0.64
    pane_height = 1.68
    pane_center_z = frame_bottom_z + pane_height * 0.5
    for face_index, angle in enumerate(
        (math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi, -3.0 * math.pi / 4.0, -math.pi / 2.0, -math.pi / 4.0)
    ):
        px, py = _face_center(lantern_face_apothem, angle)
        lighthouse_body.visual(
            Box((pane_thickness, pane_width, pane_height)),
            origin=Origin(
                xyz=(px, py, pane_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=glass,
            name=f"lantern_pane_{face_index}",
        )
        lighthouse_body.visual(
            Box((0.04, pane_width, 0.06)),
            origin=Origin(
                xyz=(px, py, frame_bottom_z + 0.86),
                rpy=(0.0, 0.0, angle),
            ),
            material=metal,
            name=f"lantern_midrail_{face_index}",
        )

    door_frame_x, _ = _face_center(lantern_face_apothem, 0.0)
    door_bottom_z = frame_bottom_z
    door_height = 1.70
    jamb_height = door_height
    jamb_center_z = door_bottom_z + jamb_height * 0.5
    lighthouse_body.visual(
        Box((0.06, 0.06, jamb_height)),
        origin=Origin(xyz=(door_frame_x, -0.28, jamb_center_z)),
        material=metal,
        name="door_hinge_jamb",
    )
    lighthouse_body.visual(
        Box((0.06, 0.06, jamb_height)),
        origin=Origin(xyz=(door_frame_x, 0.28, jamb_center_z)),
        material=metal,
        name="door_latch_jamb",
    )
    lighthouse_body.visual(
        Box((0.06, 0.62, 0.08)),
        origin=Origin(xyz=(door_frame_x, 0.0, door_bottom_z + door_height + 0.04)),
        material=metal,
        name="door_lintel",
    )

    lighthouse_body.visual(
        Cylinder(radius=0.11, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + 0.55)),
        material=dark_metal,
        name="central_shaft",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + 1.16)),
        material=dark_metal,
        name="shaft_bearing",
    )
    lighthouse_body.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + 0.05)),
        material=dark_metal,
        name="shaft_foot",
    )

    lighthouse_body.inertial = Inertial.from_geometry(
        Box((4.2, 4.2, roof_base_z + 1.42)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, (roof_base_z + 1.42) * 0.5)),
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_metal,
        name="turntable",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.08, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_metal,
        name="pedestal",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.24, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=lens_glass,
        name="main_lens",
    )
    beacon_carriage.visual(
        Box((0.05, 0.08, 0.54)),
        origin=Origin(xyz=(-0.17, 0.0, 0.39)),
        material=brass,
        name="left_frame",
    )
    beacon_carriage.visual(
        Box((0.05, 0.08, 0.54)),
        origin=Origin(xyz=(0.17, 0.0, 0.39)),
        material=brass,
        name="right_frame",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=brass,
        name="lens_cap",
    )
    beacon_carriage.visual(
        Box((0.18, 0.05, 0.05)),
        origin=Origin(xyz=(0.21, 0.0, 0.17)),
        material=dark_metal,
        name="motor_arm",
    )
    beacon_carriage.visual(
        Box((0.16, 0.10, 0.18)),
        origin=Origin(xyz=(0.34, 0.0, 0.17)),
        material=dark_metal,
        name="drive_motor",
    )
    beacon_carriage.inertial = Inertial.from_geometry(
        Box((0.70, 0.56, 0.80)),
        mass=420.0,
        origin=Origin(xyz=(0.02, 0.0, 0.40)),
    )

    model.articulation(
        "body_to_beacon_carriage",
        ArticulationType.CONTINUOUS,
        parent=lighthouse_body,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + 1.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4),
    )

    service_door = model.part("service_door")
    door_thickness = 0.045
    door_width = 0.50
    service_door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(door_thickness * 0.5, door_width * 0.5, door_height * 0.5)),
        material=metal,
        name="door_leaf",
    )
    service_door.visual(
        Box((0.012, 0.30, 0.08)),
        origin=Origin(xyz=(door_thickness + 0.006, 0.34, 0.95)),
        material=brass,
        name="door_push_bar",
    )
    service_door.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(
            xyz=(door_thickness + 0.020, 0.42, 0.95),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.08, 0.52, 1.72)),
        mass=95.0,
        origin=Origin(xyz=(0.04, 0.26, 0.86)),
    )

    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=lighthouse_body,
        child=service_door,
        origin=Origin(xyz=(door_frame_x - door_thickness * 0.5, -0.25, door_bottom_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    lighthouse_body = object_model.get_part("lighthouse_body")
    beacon_carriage = object_model.get_part("beacon_carriage")
    service_door = object_model.get_part("service_door")
    beacon_spin = object_model.get_articulation("body_to_beacon_carriage")
    door_hinge = object_model.get_articulation("body_to_service_door")

    ctx.check(
        "beacon carriage uses a vertical continuous rotation",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and beacon_spin.motion_limits is not None
        and beacon_spin.motion_limits.lower is None
        and beacon_spin.motion_limits.upper is None
        and abs(beacon_spin.axis[0]) < 1e-6
        and abs(beacon_spin.axis[1]) < 1e-6
        and abs(beacon_spin.axis[2] - 1.0) < 1e-6,
        details=f"type={beacon_spin.articulation_type}, axis={beacon_spin.axis}, limits={beacon_spin.motion_limits}",
    )
    ctx.check(
        "service door hinge is vertical at lantern wall",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(door_hinge.axis[0]) < 1e-6
        and abs(door_hinge.axis[1]) < 1e-6
        and abs(abs(door_hinge.axis[2]) - 1.0) < 1e-6,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(
            service_door,
            lighthouse_body,
            contact_tol=0.002,
            name="service door closes into the lantern frame",
        )
        closed_door_leaf = ctx.part_element_world_aabb(service_door, elem="door_leaf")

    with ctx.pose({door_hinge: math.radians(72.0)}):
        opened_door_leaf = ctx.part_element_world_aabb(service_door, elem="door_leaf")

    ctx.check(
        "service door swings outward from the wall face",
        closed_door_leaf is not None
        and opened_door_leaf is not None
        and opened_door_leaf[1][0] > closed_door_leaf[1][0] + 0.15,
        details=f"closed={closed_door_leaf}, opened={opened_door_leaf}",
    )

    with ctx.pose({beacon_spin: 0.0}):
        ctx.expect_contact(
            beacon_carriage,
            lighthouse_body,
            elem_a="turntable",
            elem_b="shaft_bearing",
            contact_tol=0.001,
            name="beacon turntable sits on the shaft bearing",
        )
        motor_rest = ctx.part_element_world_aabb(beacon_carriage, elem="drive_motor")
        beacon_rest_pos = ctx.part_world_position(beacon_carriage)

    with ctx.pose({beacon_spin: math.pi / 2.0}):
        motor_quarter_turn = ctx.part_element_world_aabb(beacon_carriage, elem="drive_motor")
        beacon_quarter_pos = ctx.part_world_position(beacon_carriage)

    ctx.check(
        "beacon carriage visibly rotates around the shaft",
        motor_rest is not None
        and motor_quarter_turn is not None
        and motor_quarter_turn[0][1] > motor_rest[0][1] + 0.20,
        details=f"rest={motor_rest}, quarter_turn={motor_quarter_turn}",
    )
    ctx.check(
        "beacon carriage stays centered on the shaft axis while rotating",
        beacon_rest_pos is not None
        and beacon_quarter_pos is not None
        and abs(beacon_rest_pos[0] - beacon_quarter_pos[0]) < 1e-6
        and abs(beacon_rest_pos[1] - beacon_quarter_pos[1]) < 1e-6
        and abs(beacon_rest_pos[2] - beacon_quarter_pos[2]) < 1e-6,
        details=f"rest={beacon_rest_pos}, quarter_turn={beacon_quarter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
