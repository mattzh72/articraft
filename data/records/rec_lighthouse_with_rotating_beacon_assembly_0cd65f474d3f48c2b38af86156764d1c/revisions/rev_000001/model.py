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


def _ring_shell_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
        [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_lighthouse")

    masonry = model.material("masonry", rgba=(0.95, 0.95, 0.92, 1.0))
    gallery_black = model.material("gallery_black", rgba=(0.12, 0.13, 0.14, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.45, 0.46, 0.48, 1.0))
    copper_cap = model.material("copper_cap", rgba=(0.47, 0.19, 0.12, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.72, 0.87, 0.96, 0.28))
    beacon_metal = model.material("beacon_metal", rgba=(0.54, 0.47, 0.24, 1.0))
    beacon_glass = model.material("beacon_glass", rgba=(0.98, 0.82, 0.33, 0.42))

    tower = model.part("tower")

    tower_profile = [
        (0.0, 0.0),
        (4.20, 0.0),
        (4.45, 0.70),
        (4.10, 3.20),
        (3.65, 10.50),
        (3.18, 18.50),
        (2.82, 23.40),
        (2.70, 24.60),
        (0.0, 24.60),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(tower_profile, segments=72), "tower_shell"),
        material=masonry,
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=3.08, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 24.59)),
        material=gallery_black,
        name="cornice_ring",
    )
    tower.visual(
        Cylinder(radius=4.85, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 24.80)),
        material=warm_grey,
        name="gallery_deck",
    )
    tower.visual(
        Cylinder(radius=4.66, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 25.08)),
        material=gallery_black,
        name="gallery_kick_ring",
    )
    tower.visual(
        Cylinder(radius=3.05, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 25.31)),
        material=gallery_black,
        name="service_drum",
    )
    tower.visual(
        Cylinder(radius=2.45, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 25.64)),
        material=gallery_black,
        name="lantern_floor",
    )

    for index in range(12):
        angle = index * math.tau / 12.0
        tower_anchor = (
            2.95 * math.cos(angle),
            2.95 * math.sin(angle),
            24.42,
        )
        deck_anchor = (
            4.28 * math.cos(angle),
            4.28 * math.sin(angle),
            24.76,
        )
        _add_member(tower, tower_anchor, deck_anchor, 0.12, warm_grey)

    rail_radius = 4.56
    post_bottom = 25.08
    post_top = 26.46
    mid_rail_z = 25.78
    top_rail_z = 26.42
    gap_half_angle = 0.14
    hinge_post_x = rail_radius * math.cos(-gap_half_angle)
    hinge_post_y = rail_radius * math.sin(-gap_half_angle)
    hinge_radius_offset = 0.075
    hinge_x = (rail_radius - hinge_radius_offset) * math.cos(-gap_half_angle)
    hinge_y = (rail_radius - hinge_radius_offset) * math.sin(-gap_half_angle)
    rail_angles = [gap_half_angle]
    rail_step = math.tau / 24.0
    next_angle = gap_half_angle + rail_step
    while next_angle < math.tau - gap_half_angle - 1e-6:
        rail_angles.append(next_angle)
        next_angle += rail_step
    rail_angles.append(math.tau - gap_half_angle)

    post_points: list[tuple[float, float, float]] = []
    for angle in rail_angles:
        px = rail_radius * math.cos(angle)
        py = rail_radius * math.sin(angle)
        post_points.append((px, py, post_bottom))
        tower.visual(
            Cylinder(radius=0.055, length=post_top - post_bottom),
            origin=Origin(xyz=(px, py, (post_bottom + post_top) * 0.5)),
            material=gallery_black,
        )

    for idx in range(len(post_points) - 1):
        start_bottom = post_points[idx]
        end_bottom = post_points[idx + 1]
        _add_member(
            tower,
            (start_bottom[0], start_bottom[1], mid_rail_z),
            (end_bottom[0], end_bottom[1], mid_rail_z),
            0.040,
            gallery_black,
        )
        _add_member(
            tower,
            (start_bottom[0], start_bottom[1], top_rail_z),
            (end_bottom[0], end_bottom[1], top_rail_z),
            0.050,
            gallery_black,
        )

    tower.visual(
        Box((0.055, 0.030, post_top - post_bottom)),
        origin=Origin(
            xyz=(
                hinge_post_x - 0.0275 * math.cos(-gap_half_angle),
                hinge_post_y - 0.0275 * math.sin(-gap_half_angle),
                (post_bottom + post_top) * 0.5,
            ),
            rpy=(0.0, 0.0, -gap_half_angle),
        ),
        material=gallery_black,
        name="gate_hinge_leaf",
    )

    lantern_apothem = 2.16
    lantern_panel_height = 2.62
    lantern_panel_center_z = 27.04
    lantern_panel_width = 2.0 * lantern_apothem * math.tan(math.pi / 8.0)
    mullion_radius = lantern_apothem / math.cos(math.pi / 8.0)

    for index in range(8):
        angle = index * math.tau / 8.0
        tower.visual(
            Box((0.05, lantern_panel_width * 0.92, lantern_panel_height)),
            origin=Origin(
                xyz=(
                    lantern_apothem * math.cos(angle),
                    lantern_apothem * math.sin(angle),
                    lantern_panel_center_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=lantern_glass,
            name=f"lantern_panel_{index}",
        )
        mullion_angle = angle + math.pi / 8.0
        tower.visual(
            Cylinder(radius=0.06, length=2.80),
            origin=Origin(
                xyz=(
                    mullion_radius * math.cos(mullion_angle),
                    mullion_radius * math.sin(mullion_angle),
                    27.04,
                )
            ),
            material=gallery_black,
        )

    tower.visual(
        Cylinder(radius=2.42, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 25.64)),
        material=gallery_black,
        name="lantern_sill_ring",
    )
    tower.visual(
        Cylinder(radius=2.44, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 28.44)),
        material=gallery_black,
        name="lantern_head_ring",
    )

    roof_profile = [
        (0.0, 30.42),
        (0.28, 30.40),
        (0.62, 30.28),
        (1.08, 29.96),
        (1.50, 29.50),
        (1.88, 28.98),
        (2.08, 28.62),
        (0.0, 28.56),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(roof_profile, segments=64), "lantern_roof"),
        material=copper_cap,
        name="lantern_roof",
    )
    tower.visual(
        Cylinder(radius=0.42, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 30.36)),
        material=gallery_black,
        name="roof_vent",
    )
    tower.visual(
        Sphere(radius=0.12),
        origin=Origin(xyz=(0.0, 0.0, 30.73)),
        material=gallery_black,
        name="roof_finial",
    )
    tower.visual(
        Cylinder(radius=0.10, length=4.22),
        origin=Origin(xyz=(0.0, 0.0, 27.66)),
        material=warm_grey,
        name="beacon_shaft",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=4.85, length=30.9),
        mass=24000.0,
        origin=Origin(xyz=(0.0, 0.0, 15.45)),
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(
        _ring_shell_mesh(0.26, 0.16, 0.18, "beacon_lower_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=beacon_metal,
        name="beacon_lower_collar",
    )
    beacon_carriage.visual(
        _ring_shell_mesh(0.24, 0.14, 0.14, "beacon_upper_collar"),
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
        material=beacon_metal,
        name="beacon_upper_collar",
    )
    bearing_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    bearing_names = (
        "beacon_bearing_roller_front",
        "beacon_bearing_roller_port_aft",
        "beacon_bearing_roller_starboard_aft",
    )
    for angle, roller_name in zip(bearing_angles, bearing_names):
        roller_center = (0.12 * math.cos(angle), 0.12 * math.sin(angle), 0.62)
        roller_axis = (-math.sin(angle), math.cos(angle), 0.0)
        roller_a = (
            roller_center[0] - 0.15 * roller_axis[0],
            roller_center[1] - 0.15 * roller_axis[1],
            roller_center[2],
        )
        roller_b = (
            roller_center[0] + 0.15 * roller_axis[0],
            roller_center[1] + 0.15 * roller_axis[1],
            roller_center[2],
        )
        beacon_carriage.visual(
            Cylinder(radius=0.020, length=0.30),
            origin=Origin(xyz=roller_center, rpy=_rpy_for_cylinder(roller_a, roller_b)),
            material=warm_grey,
            name=roller_name,
        )
        _add_member(
            beacon_carriage,
            (0.16 * math.cos(angle), 0.16 * math.sin(angle), 0.62),
            roller_center,
            0.018,
            beacon_metal,
        )
    for tie_angle in (
        math.pi / 4.0,
        3.0 * math.pi / 4.0,
        5.0 * math.pi / 4.0,
        7.0 * math.pi / 4.0,
    ):
        tie_start = (0.20 * math.cos(tie_angle), 0.20 * math.sin(tie_angle), 0.68)
        tie_end = (0.20 * math.cos(tie_angle), 0.20 * math.sin(tie_angle), 2.03)
        _add_member(beacon_carriage, tie_start, tie_end, 0.028, beacon_metal)

    for sign, lens_name in ((1.0, "beacon_port_lens"), (-1.0, "beacon_starboard_lens")):
        _add_member(
            beacon_carriage,
            (sign * 0.24, 0.0, 0.60),
            (sign * 0.72, 0.0, 0.90),
            0.045,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.21, 0.0, 2.10),
            (sign * 0.72, 0.0, 1.82),
            0.040,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.72, 0.0, 0.90),
            (sign * 0.72, 0.0, 1.82),
            0.038,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.72, 0.0, 0.90),
            (sign * 0.94, 0.24, 0.84),
            0.028,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.72, 0.0, 0.90),
            (sign * 0.94, -0.24, 0.84),
            0.028,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.72, 0.0, 1.82),
            (sign * 0.94, 0.24, 1.88),
            0.026,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.72, 0.0, 1.82),
            (sign * 0.94, -0.24, 1.88),
            0.026,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.94, -0.24, 0.84),
            (sign * 0.94, 0.24, 0.84),
            0.030,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.94, -0.24, 1.88),
            (sign * 0.94, 0.24, 1.88),
            0.030,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.94, -0.24, 0.84),
            (sign * 0.94, -0.24, 1.88),
            0.028,
            beacon_metal,
        )
        _add_member(
            beacon_carriage,
            (sign * 0.94, 0.24, 0.84),
            (sign * 0.94, 0.24, 1.88),
            0.028,
            beacon_metal,
        )
        beacon_carriage.visual(
            Box((0.06, 0.48, 1.04)),
            origin=Origin(xyz=(sign * 0.94, 0.0, 1.36)),
            material=beacon_glass,
            name=lens_name,
        )

    beacon_carriage.inertial = Inertial.from_geometry(
        Box((2.10, 0.62, 2.30)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
    )

    gate = model.part("gallery_gate")
    gate_length = 2.0 * rail_radius * math.sin(gap_half_angle)
    gate_x = -0.06
    hinge_inset = 0.08
    latch_y = gate_length - 0.08
    gate.visual(
        Cylinder(radius=0.020, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=gallery_black,
        name="gate_hinge_barrel",
    )
    _add_member(gate, (0.0, 0.02, 0.24), (gate_x, hinge_inset, 0.24), 0.018, gallery_black)
    _add_member(gate, (0.0, 0.02, 1.00), (gate_x, hinge_inset, 1.00), 0.018, gallery_black)
    _add_member(gate, (gate_x, hinge_inset, 0.18), (gate_x, latch_y, 0.18), 0.020, gallery_black)
    _add_member(gate, (gate_x, hinge_inset, 1.04), (gate_x, latch_y, 1.04), 0.020, gallery_black)
    _add_member(gate, (gate_x, hinge_inset, 0.18), (gate_x, hinge_inset, 1.04), 0.020, gallery_black)
    _add_member(
        gate,
        (gate_x, latch_y, 0.18),
        (gate_x, latch_y, 1.04),
        0.020,
        gallery_black,
        name="gate_latch_stile",
    )
    _add_member(
        gate,
        (gate_x, hinge_inset, 0.20),
        (gate_x, gate_length - 0.12, 1.02),
        0.017,
        gallery_black,
    )
    for fraction in (0.30, 0.55, 0.78):
        y_pos = gate_length * fraction
        _add_member(gate, (gate_x, y_pos, 0.22), (gate_x, y_pos, 1.00), 0.015, gallery_black)
    gate.inertial = Inertial.from_geometry(
        Box((0.08, gate_length, 1.10)),
        mass=35.0,
        origin=Origin(xyz=(gate_x, gate_length * 0.5, 0.55)),
    )

    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, 25.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0),
    )

    model.articulation(
        "gallery_gate_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=gate,
        origin=Origin(xyz=(hinge_x, hinge_y, 25.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
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
    tower = object_model.get_part("tower")
    beacon_carriage = object_model.get_part("beacon_carriage")
    gate = object_model.get_part("gallery_gate")
    beacon_rotation = object_model.get_articulation("beacon_rotation")
    gate_hinge = object_model.get_articulation("gallery_gate_hinge")

    def elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_within(
        beacon_carriage,
        tower,
        axes="xy",
        outer_elem="lantern_floor",
        margin=0.20,
        name="beacon carriage stays within the lantern room footprint",
    )
    ctx.expect_contact(
        beacon_carriage,
        tower,
        elem_a="beacon_bearing_roller_front",
        elem_b="beacon_shaft",
        contact_tol=1e-5,
        name="beacon carriage bearing roller rides on the central shaft",
    )

    lens_center_rest = elem_center(beacon_carriage, "beacon_port_lens")
    with ctx.pose({beacon_rotation: math.pi / 2.0}):
        ctx.expect_within(
            beacon_carriage,
            tower,
            axes="xy",
            outer_elem="lantern_floor",
            margin=0.20,
            name="rotated beacon carriage stays within the lantern room footprint",
        )
        lens_center_turned = elem_center(beacon_carriage, "beacon_port_lens")

    if lens_center_rest is not None and lens_center_turned is not None:
        rest_radius = math.hypot(lens_center_rest[0], lens_center_rest[1])
        turned_radius = math.hypot(lens_center_turned[0], lens_center_turned[1])
        ctx.check(
            "beacon carriage rotates around the central shaft",
            abs(rest_radius - turned_radius) < 0.05
            and lens_center_turned[1] > lens_center_rest[1] + 0.70
            and abs(lens_center_turned[0]) < abs(lens_center_rest[0]) - 0.55,
            details=f"rest={lens_center_rest}, turned={lens_center_turned}",
        )
    else:
        ctx.fail("beacon carriage rotates around the central shaft", "missing lens center AABB")

    gate_center_closed = elem_center(gate, "gate_latch_stile")
    ctx.expect_contact(
        gate,
        tower,
        elem_a="gate_hinge_barrel",
        elem_b="gate_hinge_leaf",
        contact_tol=1e-5,
        name="gallery gate hinge barrel bears against the hinge leaf",
    )
    with ctx.pose({gate_hinge: math.radians(75.0)}):
        gate_center_open = elem_center(gate, "gate_latch_stile")

    if gate_center_closed is not None and gate_center_open is not None:
        ctx.check(
            "gallery gate swings inward from the rail opening",
            gate_center_open[0] < gate_center_closed[0] - 0.30
            and abs(gate_center_open[2] - gate_center_closed[2]) < 0.05,
            details=f"closed={gate_center_closed}, open={gate_center_open}",
        )
    else:
        ctx.fail("gallery gate swings inward from the rail opening", "missing gate latch AABB")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
