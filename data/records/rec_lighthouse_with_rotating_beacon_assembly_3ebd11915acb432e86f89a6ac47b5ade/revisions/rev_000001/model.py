from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _regular_octagon_vertices(radius: float) -> list[tuple[float, float]]:
    return [
        (radius * cos((pi / 8.0) + index * (pi / 4.0)), radius * sin((pi / 8.0) + index * (pi / 4.0)))
        for index in range(8)
    ]


def _add_segment_box(
    part,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    thickness: float,
    height: float,
    z_center: float,
    material,
    name: str | None = None,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = hypot(dx, dy)
    yaw = atan2(dy, dx)
    cx = 0.5 * (start[0] + end[0])
    cy = 0.5 * (start[1] + end[1])
    part.visual(
        Box((length, thickness, height)),
        origin=Origin(xyz=(cx, cy, z_center), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse")

    white_paint = model.material("white_paint", rgba=(0.93, 0.94, 0.91, 1.0))
    concrete = model.material("concrete", rgba=(0.74, 0.75, 0.74, 1.0))
    railing_green = model.material("railing_green", rgba=(0.16, 0.25, 0.21, 1.0))
    weathered_red = model.material("weathered_red", rgba=(0.52, 0.12, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    glass = model.material("lantern_glass", rgba=(0.76, 0.91, 0.97, 0.34))
    beacon_glass = model.material("beacon_glass", rgba=(0.94, 0.88, 0.68, 0.38))
    lamp_core = model.material("lamp_core", rgba=(0.96, 0.92, 0.60, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.35, length=0.75),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=concrete,
        name="foundation_plinth",
    )
    tower.visual(
        Cylinder(radius=1.82, length=5.25),
        origin=Origin(xyz=(0.0, 0.0, 3.375)),
        material=white_paint,
        name="tower_shaft",
    )
    tower.visual(
        Cylinder(radius=2.14, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 6.125)),
        material=concrete,
        name="gallery_seat",
    )
    tower.visual(
        Box((0.95, 0.18, 1.95)),
        origin=Origin(xyz=(1.02, 0.0, 2.05), rpy=(0.0, 0.0, pi / 2.0)),
        material=white_paint,
        name="door_surround",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.35, length=6.25),
        mass=52000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.125)),
    )

    gallery = model.part("gallery")
    gallery.visual(
        Cylinder(radius=3.18, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="gallery_deck",
    )
    gallery.inertial = Inertial.from_geometry(
        Cylinder(radius=3.18, length=1.20),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    gallery_vertices = _regular_octagon_vertices(3.05)
    rail_post_size = 0.07
    rail_height = 1.08
    rail_bottom = 0.14
    rail_center_z = rail_bottom + 0.5 * rail_height
    top_rail_z = 1.08
    mid_rail_z = 0.62

    for index, (vx, vy) in enumerate(gallery_vertices):
        gallery.visual(
            Box((rail_post_size, rail_post_size, rail_height)),
            origin=Origin(xyz=(vx, vy, rail_center_z)),
            material=railing_green,
            name=f"rail_post_{index}",
        )

    front_y = gallery_vertices[1][1]
    jamb_left_x = -0.58
    jamb_right_x = 0.58
    for name, x_pos in (("gate_jamb_left", jamb_left_x), ("gate_jamb_right", jamb_right_x)):
        gallery.visual(
            Box((rail_post_size, rail_post_size, rail_height)),
            origin=Origin(xyz=(x_pos, front_y, rail_center_z)),
            material=railing_green,
            name=name,
        )

    for side_index in range(8):
        start = gallery_vertices[side_index]
        end = gallery_vertices[(side_index + 1) % 8]
        if side_index == 1:
            _add_segment_box(
                gallery,
                start,
                (jamb_right_x, front_y),
                thickness=0.05,
                height=0.05,
                z_center=top_rail_z,
                material=railing_green,
                name="front_right_top_rail",
            )
            _add_segment_box(
                gallery,
                start,
                (jamb_right_x, front_y),
                thickness=0.045,
                height=0.045,
                z_center=mid_rail_z,
                material=railing_green,
                name="front_right_mid_rail",
            )
            _add_segment_box(
                gallery,
                (jamb_left_x, front_y),
                end,
                thickness=0.05,
                height=0.05,
                z_center=top_rail_z,
                material=railing_green,
                name="front_left_top_rail",
            )
            _add_segment_box(
                gallery,
                (jamb_left_x, front_y),
                end,
                thickness=0.045,
                height=0.045,
                z_center=mid_rail_z,
                material=railing_green,
                name="front_left_mid_rail",
            )
            continue
        _add_segment_box(
            gallery,
            start,
            end,
            thickness=0.05,
            height=0.05,
            z_center=top_rail_z,
            material=railing_green,
            name=f"top_rail_{side_index}",
        )
        _add_segment_box(
            gallery,
            start,
            end,
            thickness=0.045,
            height=0.045,
            z_center=mid_rail_z,
            material=railing_green,
            name=f"mid_rail_{side_index}",
        )

    lantern = model.part("lantern")
    lantern.inertial = Inertial.from_geometry(
        Box((4.10, 4.10, 3.10)),
        mass=2400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
    )

    lantern_vertices = _regular_octagon_vertices(1.95)
    side_length = hypot(
        lantern_vertices[1][0] - lantern_vertices[0][0],
        lantern_vertices[1][1] - lantern_vertices[0][1],
    )

    for index, (vx, vy) in enumerate(lantern_vertices):
        lantern.visual(
            Cylinder(radius=0.055, length=1.70),
            origin=Origin(xyz=(vx, vy, 1.05)),
            material=railing_green,
            name=f"corner_post_{index}",
        )

    for side_index in range(8):
        start = lantern_vertices[side_index]
        end = lantern_vertices[(side_index + 1) % 8]
        _add_segment_box(
            lantern,
            start,
            end,
            thickness=0.16,
            height=0.28,
            z_center=0.14,
            material=railing_green,
            name=f"sill_{side_index}",
        )
        _add_segment_box(
            lantern,
            start,
            end,
            thickness=0.20,
            height=0.18,
            z_center=1.96,
            material=railing_green,
            name=f"head_{side_index}",
        )

        mx = 0.5 * (start[0] + end[0])
        my = 0.5 * (start[1] + end[1])
        yaw = atan2(end[1] - start[1], end[0] - start[0])
        lantern.visual(
            Box((side_length - 0.07, 0.03, 1.60)),
            origin=Origin(xyz=(mx, my, 1.10), rpy=(0.0, 0.0, yaw)),
            material=glass,
            name=f"glazing_{side_index}",
        )

    roof_mesh = _save_mesh("lantern_roof", ConeGeometry(radius=1.72, height=1.05, radial_segments=8).rotate_z(pi / 8.0))
    collar_mesh = _save_mesh(
        "beacon_collar",
        LatheGeometry.from_shell_profiles(
            [
                (0.105, -0.18),
                (0.155, -0.18),
                (0.175, -0.10),
                (0.175, -0.02),
                (0.165, 0.02),
            ],
            [
                (0.100, -0.18),
                (0.100, -0.04),
                (0.108, 0.02),
            ],
            segments=48,
        ),
    )

    lantern.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.575)),
        material=weathered_red,
        name="roof",
    )
    lantern.visual(
        Cylinder(radius=1.86, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=weathered_red,
        name="roof_drum",
    )
    lantern.visual(
        Cylinder(radius=0.34, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 3.17)),
        material=weathered_red,
        name="ventilator",
    )
    lantern.visual(
        Sphere(radius=0.18),
        origin=Origin(xyz=(0.0, 0.0, 3.42)),
        material=weathered_red,
        name="roof_cap",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.48, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_steel,
        name="pedestal_base",
    )
    pedestal.visual(
        Cylinder(radius=0.28, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=dark_steel,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.09, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=brass,
        name="clip_neck",
    )
    pedestal.visual(
        Cylinder(radius=0.36, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=brass,
        name="bearing_plate",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.48, length=0.90),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.34, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brass,
        name="turntable_base",
    )
    beacon.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=brass,
        name="retaining_collar",
    )
    beacon.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="gearbox",
    )
    beacon.visual(
        Cylinder(radius=0.40, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=beacon_glass,
        name="lens_drum",
    )
    beacon.visual(
        Cylinder(radius=0.42, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=brass,
        name="lower_lens_band",
    )
    beacon.visual(
        Cylinder(radius=0.42, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.68)),
        material=brass,
        name="upper_lens_band",
    )
    beacon.visual(
        Box((0.16, 0.10, 0.28)),
        origin=Origin(xyz=(0.12, 0.0, 0.44)),
        material=lamp_core,
        name="lamp_head",
    )
    beacon.visual(
        Cylinder(radius=0.07, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=lamp_core,
        name="lamp_column",
    )
    beacon.visual(
        Cylinder(radius=0.19, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=dark_steel,
        name="beacon_cap_band",
    )
    beacon.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=dark_steel,
        name="vent_stack",
    )
    beacon.visual(
        Sphere(radius=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=dark_steel,
        name="top_hood",
    )
    beacon.inertial = Inertial.from_geometry(
        Cylinder(radius=0.42, length=1.18),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    gate = model.part("gallery_gate")
    gate.visual(
        Box((0.04, 0.06, 1.02)),
        origin=Origin(xyz=(0.02, 0.0, 0.51)),
        material=railing_green,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.04, 0.06, 1.02)),
        origin=Origin(xyz=(0.98, 0.0, 0.51)),
        material=railing_green,
        name="latch_stile",
    )
    gate.visual(
        Box((0.96, 0.05, 0.05)),
        origin=Origin(xyz=(0.50, 0.0, 0.94)),
        material=railing_green,
        name="top_rail",
    )
    gate.visual(
        Box((0.96, 0.045, 0.045)),
        origin=Origin(xyz=(0.50, 0.0, 0.48)),
        material=railing_green,
        name="mid_rail",
    )
    gate.visual(
        Box((0.96, 0.05, 0.05)),
        origin=Origin(xyz=(0.50, 0.0, 0.11)),
        material=railing_green,
        name="bottom_rail",
    )
    gate.inertial = Inertial.from_geometry(
        Box((1.02, 0.08, 1.02)),
        mass=45.0,
        origin=Origin(xyz=(0.50, 0.0, 0.51)),
    )

    model.articulation(
        "tower_to_gallery",
        ArticulationType.FIXED,
        parent=tower,
        child=gallery,
        origin=Origin(xyz=(0.0, 0.0, 6.25)),
    )
    model.articulation(
        "gallery_to_lantern",
        ArticulationType.FIXED,
        parent=gallery,
        child=lantern,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )
    model.articulation(
        "gallery_to_pedestal",
        ArticulationType.FIXED,
        parent=gallery,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )
    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8),
    )
    model.articulation(
        "gallery_gate_hinge",
        ArticulationType.REVOLUTE,
        parent=gallery,
        child=gate,
        origin=Origin(xyz=(-0.50, front_y, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    gallery = object_model.get_part("gallery")
    lantern = object_model.get_part("lantern")
    pedestal = object_model.get_part("pedestal")
    beacon = object_model.get_part("beacon")
    gate = object_model.get_part("gallery_gate")

    beacon_spin = object_model.get_articulation("pedestal_to_beacon")
    gate_hinge = object_model.get_articulation("gallery_gate_hinge")

    bearing_plate = pedestal.get_visual("bearing_plate")
    turntable_base = beacon.get_visual("turntable_base")
    lamp_head = beacon.get_visual("lamp_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(gallery, tower, elem_a="gallery_deck", elem_b="gallery_seat")
    ctx.expect_contact(lantern, gallery, elem_b="gallery_deck")
    ctx.expect_contact(pedestal, gallery, elem_a="pedestal_base", elem_b="gallery_deck")
    ctx.expect_contact(beacon, pedestal, elem_a=turntable_base, elem_b=bearing_plate)
    ctx.expect_contact(gate, gallery, elem_b="gallery_deck")

    ctx.expect_within(pedestal, lantern, axes="xy", margin=0.10)
    ctx.expect_within(beacon, lantern, axes="xy", margin=0.10)
    ctx.expect_within(beacon, pedestal, axes="xy", inner_elem=turntable_base, outer_elem=bearing_plate, margin=0.01)
    ctx.expect_gap(gate, lantern, axis="y", min_gap=0.60)

    lamp_rest = ctx.part_element_world_aabb(beacon, elem=lamp_head)
    assert lamp_rest is not None
    with ctx.pose({beacon_spin: pi / 2.0}):
        lamp_quarter = ctx.part_element_world_aabb(beacon, elem=lamp_head)
        assert lamp_quarter is not None
        assert lamp_quarter[1][1] > lamp_rest[1][1] + 0.08
        ctx.expect_contact(beacon, pedestal, elem_a=turntable_base, elem_b=bearing_plate)
        ctx.expect_within(beacon, lantern, axes="xy", margin=0.10)

    gate_rest = ctx.part_world_aabb(gate)
    assert gate_rest is not None
    with ctx.pose({gate_hinge: 1.10}):
        gate_open = ctx.part_world_aabb(gate)
        assert gate_open is not None
        assert gate_open[1][0] < gate_rest[1][0] - 0.25
        ctx.expect_contact(gate, gallery, elem_b="gallery_deck")
        ctx.expect_gap(gate, lantern, axis="y", min_gap=0.60)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
