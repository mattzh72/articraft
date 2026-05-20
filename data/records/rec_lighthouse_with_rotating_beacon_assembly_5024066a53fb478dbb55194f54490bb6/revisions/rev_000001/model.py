from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
):
    geometry = LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, name)


def _add_tapered_lattice_tower(
    part,
    *,
    bottom_width: float,
    top_width: float,
    bottom_z: float,
    top_z: float,
    panels: int,
    leg_radius: float,
    brace_radius: float,
    material,
    ladder_material,
) -> None:
    levels = [bottom_z + (top_z - bottom_z) * i / panels for i in range(panels + 1)]

    def corners_at(z: float) -> list[tuple[float, float, float]]:
        t = 0.0 if abs(top_z - bottom_z) < 1e-9 else (z - bottom_z) / (top_z - bottom_z)
        width = bottom_width + (top_width - bottom_width) * t
        half = width * 0.5
        return [
            (half, half, z),
            (half, -half, z),
            (-half, -half, z),
            (-half, half, z),
        ]

    for panel_index in range(panels):
        lower = corners_at(levels[panel_index])
        upper = corners_at(levels[panel_index + 1])

        for idx in range(4):
            _add_member(part, lower[idx], upper[idx], leg_radius, material)

        for corners in (lower, upper):
            for idx in range(4):
                _add_member(
                    part,
                    corners[idx],
                    corners[(idx + 1) % 4],
                    brace_radius,
                    material,
                )

        for idx in range(4):
            next_idx = (idx + 1) % 4
            _add_member(part, lower[idx], upper[next_idx], brace_radius, material)
            _add_member(part, lower[next_idx], upper[idx], brace_radius, material)

        _add_member(part, lower[0], upper[2], brace_radius * 0.92, material)
        _add_member(part, lower[1], upper[3], brace_radius * 0.92, material)

    ladder_x = 0.22
    ladder_y = 0.18
    ladder_bottom = bottom_z
    ladder_top = top_z - 0.18
    _add_member(
        part,
        (ladder_x, -ladder_y, ladder_bottom),
        (ladder_x, -ladder_y, ladder_top),
        0.034,
        ladder_material,
    )
    _add_member(
        part,
        (ladder_x, ladder_y, ladder_bottom),
        (ladder_x, ladder_y, ladder_top),
        0.034,
        ladder_material,
    )
    rung_count = 22
    for idx in range(rung_count + 1):
        z = ladder_bottom + (ladder_top - ladder_bottom) * idx / rung_count
        _add_member(
            part,
            (ladder_x, -ladder_y, z),
            (ladder_x, ladder_y, z),
            0.024,
            ladder_material,
        )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skeletal_steel_lighthouse")

    concrete = model.material("concrete", rgba=(0.64, 0.64, 0.62, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.27, 1.0))
    roof_green = model.material("roof_green", rgba=(0.18, 0.24, 0.22, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.73, 0.86, 0.92, 0.30))
    beacon_glass = model.material("beacon_glass", rgba=(0.92, 0.78, 0.34, 0.42))
    beacon_frame = model.material("beacon_frame", rgba=(0.70, 0.60, 0.34, 1.0))
    hatch_paint = model.material("hatch_paint", rgba=(0.76, 0.14, 0.12, 1.0))
    lamp_white = model.material("lamp_white", rgba=(0.96, 0.94, 0.86, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((5.40, 5.40, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        Box((2.50, 2.50, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=concrete,
        name="pedestal",
    )
    tower.visual(
        Box((3.20, 3.20, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 14.47)),
        material=dark_steel,
        name="gallery_deck",
    )
    _add_tapered_lattice_tower(
        tower,
        bottom_width=3.50,
        top_width=1.72,
        bottom_z=1.20,
        top_z=14.40,
        panels=8,
        leg_radius=0.085,
        brace_radius=0.046,
        material=galvanized_steel,
        ladder_material=dark_steel,
    )

    gallery_post_positions = [
        (1.46, 1.46),
        (1.46, 0.0),
        (1.46, -1.46),
        (0.0, -1.46),
        (-1.46, -1.46),
        (-1.46, 0.0),
        (-1.46, 1.46),
        (0.0, 1.46),
    ]
    for x, y in gallery_post_positions:
        _add_member(tower, (x, y, 14.54), (x, y, 15.52), 0.032, dark_steel)

    top_loop = [(x, y, 15.48) for x, y in gallery_post_positions]
    mid_loop = [(x, y, 15.00) for x, y in gallery_post_positions]
    for loop in (top_loop, mid_loop):
        for idx in range(len(loop)):
            _add_member(tower, loop[idx], loop[(idx + 1) % len(loop)], 0.022, dark_steel)

    lantern_room = model.part("lantern_room")
    lantern_room.visual(
        Box((1.82, 1.82, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="floor",
    )
    lantern_room.visual(
        Box((1.92, 1.92, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="base_ring",
    )

    post_half = 0.76
    post_bottom = 0.12
    post_top = 1.90
    for x in (-post_half, post_half):
        for y in (-post_half, post_half):
            _add_member(
                lantern_room,
                (x, y, post_bottom),
                (x, y, post_top),
                0.045,
                dark_steel,
            )

    lower_ring_z = 0.20
    upper_ring_z = 1.78
    ring_corners = [
        (post_half, post_half),
        (post_half, -post_half),
        (-post_half, -post_half),
        (-post_half, post_half),
    ]
    for z, radius in ((lower_ring_z, 0.034), (upper_ring_z, 0.040)):
        for idx in range(4):
            if idx == 3:
                continue
            a = (*ring_corners[idx], z)
            b = (*ring_corners[(idx + 1) % 4], z)
            _add_member(lantern_room, a, b, radius, dark_steel)

    lantern_room.visual(
        Box((1.43, 0.04, 1.58)),
        origin=Origin(xyz=(0.0, -post_half, 0.99)),
        material=lantern_glass,
        name="rear_glass",
    )
    lantern_room.visual(
        Box((0.04, 1.43, 1.58)),
        origin=Origin(xyz=(-post_half, 0.0, 0.99)),
        material=lantern_glass,
        name="left_glass",
    )
    lantern_room.visual(
        Box((0.04, 1.43, 1.58)),
        origin=Origin(xyz=(post_half, 0.0, 0.99)),
        material=lantern_glass,
        name="right_glass",
    )
    lantern_room.visual(
        Box((0.03, 0.03, 1.56)),
        origin=Origin(xyz=(0.705, 0.73, 0.99)),
        material=dark_steel,
        name="front_latch_stop",
    )
    lantern_room.visual(
        Box((1.88, 1.88, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.86)),
        material=dark_steel,
        name="roof_base",
    )
    lantern_room.visual(
        mesh_from_geometry(ConeGeometry(radius=0.98, height=0.72, radial_segments=40), "lantern_roof_cone"),
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        material=roof_green,
        name="roof_cone",
    )
    lantern_room.visual(
        Cylinder(radius=0.15, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 2.73)),
        material=roof_green,
        name="roof_vent",
    )
    lantern_room.visual(
        Sphere(radius=0.11),
        origin=Origin(xyz=(0.0, 0.0, 2.90)),
        material=roof_green,
        name="roof_cap",
    )
    lantern_room.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=dark_steel,
        name="turntable_base",
    )
    lantern_room.visual(
        Cylinder(radius=0.045, length=2.02),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=galvanized_steel,
        name="spindle",
    )
    lantern_room.visual(
        Cylinder(radius=0.07, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=dark_steel,
        name="top_bearing",
    )

    beacon = model.part("beacon")
    frame_corners = [(0.23, 0.23), (0.23, -0.23), (-0.23, -0.23), (-0.23, 0.23)]
    for z in (0.28, 1.24):
        for idx in range(4):
            a = (*frame_corners[idx], z)
            b = (*frame_corners[(idx + 1) % 4], z)
            _add_member(beacon, a, b, 0.024, beacon_frame)
    for x, y in frame_corners:
        _add_member(beacon, (x, y, 0.28), (x, y, 1.24), 0.028, beacon_frame)

    beacon.visual(
        Box((0.07, 0.05, 0.12)),
        origin=Origin(xyz=(0.095, 0.0, 0.36)),
        material=beacon_frame,
        name="lower_clip_right",
    )
    beacon.visual(
        Box((0.07, 0.05, 0.12)),
        origin=Origin(xyz=(-0.095, 0.0, 0.36)),
        material=beacon_frame,
        name="lower_clip_left",
    )
    beacon.visual(
        Box((0.05, 0.07, 0.12)),
        origin=Origin(xyz=(0.0, 0.095, 0.36)),
        material=beacon_frame,
        name="lower_clip_front",
    )
    beacon.visual(
        Box((0.05, 0.07, 0.12)),
        origin=Origin(xyz=(0.0, -0.095, 0.36)),
        material=beacon_frame,
        name="lower_clip_rear",
    )
    beacon.visual(
        Box((0.06, 0.04, 0.10)),
        origin=Origin(xyz=(0.090, 0.0, 1.27)),
        material=beacon_frame,
        name="upper_clip_right",
    )
    beacon.visual(
        Box((0.06, 0.04, 0.10)),
        origin=Origin(xyz=(-0.090, 0.0, 1.27)),
        material=beacon_frame,
        name="upper_clip_left",
    )
    beacon.visual(
        Box((0.04, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.090, 1.27)),
        material=beacon_frame,
        name="upper_clip_front",
    )
    beacon.visual(
        Box((0.04, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, -0.090, 1.27)),
        material=beacon_frame,
        name="upper_clip_rear",
    )

    for x, y in ((0.095, 0.0), (-0.095, 0.0), (0.0, 0.095), (0.0, -0.095)):
        _add_member(beacon, (x, y, 0.42), (x, y, 1.22), 0.016, beacon_frame)
    for a, b in (
        ((0.13, 0.0, 0.36), (0.23, 0.23, 0.36)),
        ((0.13, 0.0, 0.36), (0.23, -0.23, 0.36)),
        ((-0.13, 0.0, 0.36), (-0.23, 0.23, 0.36)),
        ((-0.13, 0.0, 0.36), (-0.23, -0.23, 0.36)),
        ((0.0, 0.13, 0.36), (0.23, 0.23, 0.36)),
        ((0.0, 0.13, 0.36), (-0.23, 0.23, 0.36)),
        ((0.0, -0.13, 0.36), (0.23, -0.23, 0.36)),
        ((0.0, -0.13, 0.36), (-0.23, -0.23, 0.36)),
        ((0.12, 0.0, 1.27), (0.23, 0.23, 1.24)),
        ((0.12, 0.0, 1.27), (0.23, -0.23, 1.24)),
        ((-0.12, 0.0, 1.27), (-0.23, 0.23, 1.24)),
        ((-0.12, 0.0, 1.27), (-0.23, -0.23, 1.24)),
        ((0.0, 0.12, 1.27), (0.23, 0.23, 1.24)),
        ((0.0, 0.12, 1.27), (-0.23, 0.23, 1.24)),
        ((0.0, -0.12, 1.27), (0.23, -0.23, 1.24)),
        ((0.0, -0.12, 1.27), (-0.23, -0.23, 1.24)),
    ):
        _add_member(beacon, a, b, 0.016, beacon_frame)

    beacon.visual(
        Box((0.104, 0.46, 1.02)),
        origin=Origin(xyz=(0.31, 0.0, 0.73)),
        material=beacon_glass,
    )
    beacon.visual(
        Box((0.104, 0.46, 1.02)),
        origin=Origin(xyz=(-0.31, 0.0, 0.73)),
        material=beacon_glass,
    )
    beacon.visual(
        Box((0.46, 0.104, 1.02)),
        origin=Origin(xyz=(0.0, 0.31, 0.73)),
        material=beacon_glass,
    )
    beacon.visual(
        Box((0.46, 0.104, 1.02)),
        origin=Origin(xyz=(0.0, -0.31, 0.73)),
        material=beacon_glass,
    )
    beacon.visual(
        Cylinder(radius=0.085, length=0.24),
        origin=Origin(xyz=(0.22, 0.0, 0.73), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_white,
        name="lamp_head",
    )
    beacon.visual(
        Box((0.10, 0.12, 0.18)),
        origin=Origin(xyz=(-0.18, 0.0, 0.72)),
        material=dark_steel,
        name="counterweight",
    )
    _add_member(
        beacon,
        (-0.13, 0.0, 0.72),
        (-0.23, 0.23, 0.72),
        0.022,
        dark_steel,
        name="counterweight_brace",
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((1.40, 0.03, 1.56)),
        origin=Origin(xyz=(0.70, -0.02, 0.78)),
        material=hatch_paint,
        name="hatch_panel",
    )
    _add_member(hatch, (0.10, -0.02, 0.18), (1.22, -0.02, 1.36), 0.024, dark_steel)
    for z, name in ((0.22, "hinge_knuckle_lower"), (0.78, "hinge_knuckle_mid"), (1.34, "hinge_knuckle_upper")):
        hatch.visual(
            Cylinder(radius=0.018, length=0.08),
            origin=Origin(xyz=(0.02, -0.004, z)),
            material=galvanized_steel,
            name=name,
        )
    hatch.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(1.18, -0.06, 0.82), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized_steel,
        name="handle",
    )

    tower.inertial = Inertial.from_geometry(
        Box((5.40, 5.40, 15.60)),
        mass=32000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.80)),
    )
    lantern_room.inertial = Inertial.from_geometry(
        Box((2.20, 2.20, 3.05)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
    )
    beacon.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 1.40)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )
    hatch.inertial = Inertial.from_geometry(
        Box((1.40, 0.06, 1.56)),
        mass=140.0,
        origin=Origin(xyz=(0.70, -0.02, 0.78)),
    )

    model.articulation(
        "tower_to_lantern",
        ArticulationType.FIXED,
        parent=tower,
        child=lantern_room,
        origin=Origin(xyz=(0.0, 0.0, 14.54)),
    )
    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=lantern_room,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=lantern_room,
        child=hatch,
        origin=Origin(xyz=(-0.71, post_half + 0.02, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    lantern_room = object_model.get_part("lantern_room")
    beacon = object_model.get_part("beacon")
    hatch = object_model.get_part("hatch")
    beacon_rotation = object_model.get_articulation("beacon_rotation")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

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

    ctx.expect_contact(lantern_room, tower, name="lantern_room_is_mounted_to_gallery")
    ctx.expect_within(
        beacon,
        lantern_room,
        axes="xy",
        margin=0.03,
        name="beacon_stays_inside_lantern_cage",
    )
    ctx.expect_contact(
        hatch,
        lantern_room,
        elem_a="hatch_panel",
        elem_b="front_latch_stop",
        name="closed_hatch_is_seated_in_lantern_side",
    )

    spindle_aabb = ctx.part_element_world_aabb(lantern_room, elem="spindle")
    lower_right_aabb = ctx.part_element_world_aabb(beacon, elem="lower_clip_right")
    lower_left_aabb = ctx.part_element_world_aabb(beacon, elem="lower_clip_left")
    lower_front_aabb = ctx.part_element_world_aabb(beacon, elem="lower_clip_front")
    lower_rear_aabb = ctx.part_element_world_aabb(beacon, elem="lower_clip_rear")
    upper_right_aabb = ctx.part_element_world_aabb(beacon, elem="upper_clip_right")
    upper_left_aabb = ctx.part_element_world_aabb(beacon, elem="upper_clip_left")
    upper_front_aabb = ctx.part_element_world_aabb(beacon, elem="upper_clip_front")
    upper_rear_aabb = ctx.part_element_world_aabb(beacon, elem="upper_clip_rear")
    captured = False
    if (
        spindle_aabb is not None
        and lower_right_aabb is not None
        and lower_left_aabb is not None
        and lower_front_aabb is not None
        and lower_rear_aabb is not None
        and upper_right_aabb is not None
        and upper_left_aabb is not None
        and upper_front_aabb is not None
        and upper_rear_aabb is not None
    ):
        lower_captured = (
            lower_right_aabb[0][0] > spindle_aabb[1][0]
            and lower_left_aabb[1][0] < spindle_aabb[0][0]
            and lower_front_aabb[0][1] > spindle_aabb[1][1]
            and lower_rear_aabb[1][1] < spindle_aabb[0][1]
            and lower_right_aabb[0][2] < spindle_aabb[1][2]
            and lower_right_aabb[1][2] > spindle_aabb[0][2]
        )
        upper_captured = (
            upper_right_aabb[0][0] > spindle_aabb[1][0]
            and upper_left_aabb[1][0] < spindle_aabb[0][0]
            and upper_front_aabb[0][1] > spindle_aabb[1][1]
            and upper_rear_aabb[1][1] < spindle_aabb[0][1]
            and upper_right_aabb[0][2] < spindle_aabb[1][2]
            and upper_right_aabb[1][2] > spindle_aabb[0][2]
        )
        captured = lower_captured and upper_captured
    ctx.check(
        "beacon_collars_capture_spindle",
        captured,
        "The rotating beacon clips should bracket the spindle from four sides at both the lower and upper bearing levels.",
    )

    with ctx.pose({beacon_rotation: 0.0}):
        lamp_aabb_closed = ctx.part_element_world_aabb(beacon, elem="lamp_head")
    with ctx.pose({beacon_rotation: math.pi / 2.0}):
        lamp_aabb_quarter = ctx.part_element_world_aabb(beacon, elem="lamp_head")

    lamp_rotates = False
    if lamp_aabb_closed is not None and lamp_aabb_quarter is not None:
        closed_center = _aabb_center(lamp_aabb_closed)
        quarter_center = _aabb_center(lamp_aabb_quarter)
        lamp_rotates = (
            closed_center[0] > 0.12
            and abs(closed_center[1]) < 0.06
            and quarter_center[1] > 0.12
            and abs(quarter_center[0]) < 0.06
        )
    ctx.check(
        "beacon_rotates_about_vertical_axis",
        lamp_rotates,
        "The off-axis lamp head should move from the +X side to the +Y side after a quarter-turn.",
    )

    with ctx.pose({hatch_hinge: 1.20}):
        hatch_panel_open_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
        lantern_aabb = ctx.part_world_aabb(lantern_room)
    hatch_swings_clear = False
    if hatch_panel_open_aabb is not None and lantern_aabb is not None:
        hatch_center = _aabb_center(hatch_panel_open_aabb)
        hatch_swings_clear = hatch_center[1] > lantern_aabb[1][1] + 0.18
    ctx.check(
        "hatch_swings_outward_on_side_hinge",
        hatch_swings_clear,
        "The hatch should pivot clear of the lantern wall instead of remaining embedded in the cage.",
    )

    ctx.warn_if_articulation_overlaps(max_pose_samples=16)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
