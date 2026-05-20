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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _polar(radius: float, angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    return (radius * math.cos(angle), radius * math.sin(angle))


def _arc_points(
    radius: float, start_deg: float, end_deg: float, z: float, count: int
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(count):
        t = i / (count - 1)
        angle = start_deg + (end_deg - start_deg) * t
        x, y = _polar(radius, angle)
        pts.append((x, y, z))
    return pts


def _outward_shift(
    point: tuple[float, float, float], extra: float
) -> tuple[float, float, float]:
    radius = math.hypot(point[0], point[1])
    if radius < 1e-9:
        return point
    return (
        point[0] + extra * point[0] / radius,
        point[1] + extra * point[1] / radius,
        point[2],
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def _add_side_wall(
    part,
    *,
    radius: float,
    start_deg: float,
    end_deg: float,
    glass,
    frame_metal,
    name_prefix: str,
) -> None:
    bottom_z = 0.08
    top_z = 2.34
    rail_radius = 0.035
    mullion_radius = 0.028
    jamb_width = 0.075
    jamb_depth = 0.03
    glass_height = 2.16
    glass_center_z = 1.20
    glass_thickness = 0.018
    glass_inset = 0.012
    end_frame_capture = 0.075
    mullion_capture = 0.05
    boundary_count = 6
    boundary_angles = [
        start_deg + (end_deg - start_deg) * i / (boundary_count - 1)
        for i in range(boundary_count)
    ]

    bottom_pts = _arc_points(radius, start_deg, end_deg, bottom_z, boundary_count)
    top_pts = _arc_points(radius, start_deg, end_deg, top_z, boundary_count)
    mid_pts = _arc_points(radius, start_deg, end_deg, 1.08, boundary_count)

    for i in range(boundary_count - 1):
        _add_member(
            part,
            bottom_pts[i],
            bottom_pts[i + 1],
            rail_radius,
            frame_metal,
            name=f"{name_prefix}_bottom_rail_{i}",
        )
        _add_member(
            part,
            top_pts[i],
            top_pts[i + 1],
            rail_radius,
            frame_metal,
            name=f"{name_prefix}_top_rail_{i}",
        )
        _add_member(
            part,
            mid_pts[i],
            mid_pts[i + 1],
            rail_radius * 0.7,
            frame_metal,
            name=f"{name_prefix}_mid_rail_{i}",
        )

    for i in range(boundary_count):
        x, y = _polar(radius, boundary_angles[i])
        if i in (0, boundary_count - 1):
            part.visual(
                Box((jamb_width, jamb_depth, top_z - bottom_z)),
                origin=Origin(
                    xyz=(x, y, (bottom_z + top_z) * 0.5),
                    rpy=(0.0, 0.0, math.radians(boundary_angles[i]) + math.pi / 2.0),
                ),
                material=frame_metal,
                name=f"{name_prefix}_mullion_{i}",
            )
        else:
            part.visual(
                Cylinder(radius=mullion_radius, length=top_z - bottom_z),
                origin=Origin(xyz=(x, y, (bottom_z + top_z) * 0.5)),
                material=frame_metal,
                name=f"{name_prefix}_mullion_{i}",
            )

    for i in range(boundary_count - 1):
        angle_mid = math.radians((boundary_angles[i] + boundary_angles[i + 1]) * 0.5)
        angle_span = math.radians(boundary_angles[i + 1] - boundary_angles[i])
        chord = 2.0 * radius * math.sin(abs(angle_span) * 0.5)
        left_capture = end_frame_capture if i == 0 else mullion_capture
        right_capture = end_frame_capture if i == (boundary_count - 2) else mullion_capture
        panel_width = max(0.05, chord - left_capture - right_capture)
        glass_radius = radius - glass_inset
        x = glass_radius * math.cos(angle_mid)
        y = glass_radius * math.sin(angle_mid)
        part.visual(
            Box((panel_width, glass_thickness, glass_height)),
            origin=Origin(
                xyz=(x, y, glass_center_z),
                rpy=(0.0, 0.0, angle_mid + math.pi / 2.0),
            ),
            material=glass,
            name=f"{name_prefix}_glass_{i}",
        )

    part.inertial = Inertial.from_geometry(
        Box((2.7, 0.22, 2.45)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airport_revolving_door")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.62, 0.78, 0.86, 0.28))
    clear_glass = model.material("clear_glass", rgba=(0.75, 0.88, 0.94, 0.22))
    threshold_gray = model.material("threshold_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    drum_radius = 1.58
    canopy_radius = 1.88
    portal_start_deg = 35.0
    portal_end_deg = 145.0
    post_radius = 0.04

    stationary_frame = model.part("stationary_frame")
    stationary_frame.visual(
        Cylinder(radius=canopy_radius, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.65)),
        material=brushed_aluminum,
        name="canopy_disc",
    )
    stationary_frame.visual(
        Cylinder(radius=0.28, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 2.61)),
        material=dark_metal,
        name="drive_housing",
    )
    stationary_frame.visual(
        Cylinder(radius=0.16, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=threshold_gray,
        name="floor_bearing",
    )
    stationary_frame.visual(
        Cylinder(radius=canopy_radius * 0.98, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=threshold_gray,
        name="floor_plate",
    )
    stationary_frame.visual(
        Cylinder(radius=canopy_radius * 0.98, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 2.55)),
        material=dark_metal,
        name="canopy_soffit_band",
    )

    front_top = _outward_shift(
        (_polar(drum_radius, portal_start_deg)[0], _polar(drum_radius, portal_start_deg)[1], 0.0),
        post_radius + 0.035,
    )
    front_bottom = _outward_shift(
        (_polar(drum_radius, -portal_start_deg)[0], _polar(drum_radius, -portal_start_deg)[1], 0.0),
        post_radius + 0.035,
    )
    rear_top = _outward_shift(
        (_polar(drum_radius, portal_end_deg)[0], _polar(drum_radius, portal_end_deg)[1], 0.0),
        post_radius + 0.035,
    )
    rear_bottom = _outward_shift(
        (_polar(drum_radius, -portal_end_deg)[0], _polar(drum_radius, -portal_end_deg)[1], 0.0),
        post_radius + 0.035,
    )

    for name, point in (
        ("front_upper_post", front_top),
        ("front_lower_post", front_bottom),
        ("rear_upper_post", rear_top),
        ("rear_lower_post", rear_bottom),
    ):
        stationary_frame.visual(
            Cylinder(radius=post_radius, length=2.40),
            origin=Origin(xyz=(point[0], point[1], 1.20)),
            material=brushed_aluminum,
            name=name,
        )

    receiver_depth = 0.035
    receiver_width = 0.085
    wall_jamb_depth = 0.03
    receiver_bands = (
        ("lower_clip", 0.58, 0.78),
        ("upper_clip", 1.71, 0.98),
    )
    for base_name, angle_deg in (
        ("front_upper_receiver", portal_start_deg),
        ("front_lower_receiver", -portal_start_deg),
        ("rear_upper_receiver", portal_end_deg),
        ("rear_lower_receiver", -portal_end_deg),
    ):
        x, y = _polar(drum_radius, angle_deg)
        receiver_yaw = math.radians(angle_deg) + math.pi / 2.0
        for band_name, center_z, height in receiver_bands:
            receiver_center = _outward_shift(
                (x, y, center_z),
                (wall_jamb_depth + receiver_depth) * 0.5,
            )
            stationary_frame.visual(
                Box((receiver_width, receiver_depth, height)),
                origin=Origin(
                    xyz=receiver_center,
                    rpy=(0.0, 0.0, receiver_yaw),
                ),
                material=brushed_aluminum,
                name=f"{base_name}_{band_name}",
            )

    front_header_center = (
        (front_top[0] + front_bottom[0]) * 0.5,
        (front_top[1] + front_bottom[1]) * 0.5,
        2.48,
    )
    rear_header_center = (
        (rear_top[0] + rear_bottom[0]) * 0.5,
        (rear_top[1] + rear_bottom[1]) * 0.5,
        2.48,
    )
    front_header_width = abs(front_top[1] - front_bottom[1]) + post_radius * 2.0
    rear_header_width = abs(rear_top[1] - rear_bottom[1]) + post_radius * 2.0
    stationary_frame.visual(
        Box((0.12, front_header_width, 0.16)),
        origin=Origin(xyz=front_header_center),
        material=brushed_aluminum,
        name="front_header",
    )
    stationary_frame.visual(
        Box((0.12, rear_header_width, 0.16)),
        origin=Origin(xyz=rear_header_center),
        material=brushed_aluminum,
        name="rear_header",
    )
    stationary_frame.visual(
        Box((0.18, front_header_width - 0.18, 0.04)),
        origin=Origin(xyz=(front_header_center[0], front_header_center[1], 0.02)),
        material=threshold_gray,
        name="front_threshold",
    )
    stationary_frame.visual(
        Box((0.18, rear_header_width - 0.18, 0.04)),
        origin=Origin(xyz=(rear_header_center[0], rear_header_center[1], 0.02)),
        material=threshold_gray,
        name="rear_threshold",
    )
    stationary_frame.inertial = Inertial.from_geometry(
        Box((3.9, 3.9, 2.85)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 1.425)),
    )

    north_wall = model.part("north_wall")
    _add_side_wall(
        north_wall,
        radius=drum_radius,
        start_deg=portal_start_deg,
        end_deg=portal_end_deg,
        glass=tinted_glass,
        frame_metal=brushed_aluminum,
        name_prefix="north",
    )

    south_wall = model.part("south_wall")
    _add_side_wall(
        south_wall,
        radius=drum_radius,
        start_deg=-portal_end_deg,
        end_deg=-portal_start_deg,
        glass=tinted_glass,
        frame_metal=brushed_aluminum,
        name_prefix="south",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=2.32),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=dark_metal,
        name="central_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_metal,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.34)),
        material=dark_metal,
        name="upper_hub",
    )
    rotor.visual(
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=gasket_black,
        name="lower_collar",
    )
    rotor.visual(
        Cylinder(radius=0.12, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 2.455)),
        material=gasket_black,
        name="upper_cap",
    )

    wing_angles = [45.0, 135.0, 225.0, 315.0]
    for index, angle_deg in enumerate(wing_angles):
        angle = math.radians(angle_deg)
        radial_center = 0.82
        outer_stile_center = 1.43
        inner_stile_center = 0.24
        push_bar_center = 0.84
        part_yaw = angle

        rotor.visual(
            Box((1.16, 0.018, 2.10)),
            origin=Origin(
                xyz=(radial_center * math.cos(angle), radial_center * math.sin(angle), 1.27),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=clear_glass,
            name=f"wing_{index}_glass",
        )
        rotor.visual(
            Box((1.26, 0.065, 0.06)),
            origin=Origin(
                xyz=(0.77 * math.cos(angle), 0.77 * math.sin(angle), 0.18),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=brushed_aluminum,
            name=f"wing_{index}_bottom_rail",
        )
        rotor.visual(
            Box((1.26, 0.065, 0.06)),
            origin=Origin(
                xyz=(0.77 * math.cos(angle), 0.77 * math.sin(angle), 2.34),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=brushed_aluminum,
            name=f"wing_{index}_top_rail",
        )
        rotor.visual(
            Box((0.06, 0.065, 2.18)),
            origin=Origin(
                xyz=(
                    outer_stile_center * math.cos(angle),
                    outer_stile_center * math.sin(angle),
                    1.27,
                ),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=brushed_aluminum,
            name=f"wing_{index}_outer_stile",
        )
        rotor.visual(
            Box((0.06, 0.08, 2.18)),
            origin=Origin(
                xyz=(
                    inner_stile_center * math.cos(angle),
                    inner_stile_center * math.sin(angle),
                    1.27,
                ),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=dark_metal,
            name=f"wing_{index}_inner_stile",
        )
        rotor.visual(
            Box((0.96, 0.045, 0.05)),
            origin=Origin(
                xyz=(push_bar_center * math.cos(angle), push_bar_center * math.sin(angle), 1.08),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=brushed_aluminum,
            name=f"wing_{index}_push_bar",
        )
        rotor.visual(
            Box((0.18, 0.10, 0.08)),
            origin=Origin(
                xyz=(0.20 * math.cos(angle), 0.20 * math.sin(angle), 0.18),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=dark_metal,
            name=f"wing_{index}_lower_root_block",
        )
        rotor.visual(
            Box((0.18, 0.10, 0.08)),
            origin=Origin(
                xyz=(0.20 * math.cos(angle), 0.20 * math.sin(angle), 2.34),
                rpy=(0.0, 0.0, part_yaw),
            ),
            material=dark_metal,
            name=f"wing_{index}_upper_root_block",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.46, length=2.40),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
    )

    model.articulation(
        "frame_to_north_wall",
        ArticulationType.FIXED,
        parent=stationary_frame,
        child=north_wall,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_south_wall",
        ArticulationType.FIXED,
        parent=stationary_frame,
        child=south_wall,
        origin=Origin(),
    )
    model.articulation(
        "door_rotation",
        ArticulationType.CONTINUOUS,
        parent=stationary_frame,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.1),
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

    stationary_frame = object_model.get_part("stationary_frame")
    north_wall = object_model.get_part("north_wall")
    south_wall = object_model.get_part("south_wall")
    rotor = object_model.get_part("rotor")
    door_rotation = object_model.get_articulation("door_rotation")

    ctx.expect_origin_distance(
        rotor,
        stationary_frame,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="rotor stays centered on the drum axis",
    )
    ctx.expect_contact(
        rotor,
        stationary_frame,
        elem_a="lower_collar",
        elem_b="floor_bearing",
        contact_tol=0.001,
        name="rotor lower collar seats on the floor bearing",
    )
    ctx.expect_gap(
        stationary_frame,
        rotor,
        axis="z",
        positive_elem="canopy_disc",
        negative_elem="upper_cap",
        min_gap=0.08,
        max_gap=0.11,
        name="rotor top cap clears the canopy underside",
    )
    ctx.expect_contact(
        north_wall,
        stationary_frame,
        contact_tol=0.0015,
        name="north drum wall ties into the stationary frame",
    )
    ctx.expect_contact(
        south_wall,
        stationary_frame,
        contact_tol=0.0015,
        name="south drum wall ties into the stationary frame",
    )

    limits = door_rotation.motion_limits
    axis_ok = tuple(round(v, 4) for v in door_rotation.axis) == (0.0, 0.0, 1.0)
    continuous_ok = (
        door_rotation.articulation_type == ArticulationType.CONTINUOUS
        and axis_ok
        and limits is not None
        and limits.lower is None
        and limits.upper is None
    )
    ctx.check(
        "door articulation is a continuous vertical rotation",
        continuous_ok,
        details=(
            f"type={door_rotation.articulation_type}, axis={door_rotation.axis}, "
            f"limits={limits}"
        ),
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")
    with ctx.pose({door_rotation: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")

    rest_center = _aabb_center(rest_aabb)
    quarter_turn_center = _aabb_center(quarter_turn_aabb)
    rotates_cleanly = False
    if rest_center is not None and quarter_turn_center is not None:
        rest_radius = math.hypot(rest_center[0], rest_center[1])
        turned_radius = math.hypot(quarter_turn_center[0], quarter_turn_center[1])
        rotates_cleanly = (
            rest_center[0] > 0.7
            and quarter_turn_center[0] < -0.7
            and quarter_turn_center[1] > 0.7
            and abs(rest_radius - turned_radius) < 0.03
        )
    ctx.check(
        "wing assembly rotates around the central shaft",
        rotates_cleanly,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
