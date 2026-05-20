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


def _embed_segment(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    embed: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9 or embed <= 0.0:
        return a, b
    ux = dx / length
    uy = dy / length
    uz = dz / length
    return (
        (a[0] - ux * embed, a[1] - uy * embed, a[2] - uz * embed),
        (b[0] + ux * embed, b[1] + uy * embed, b[2] + uz * embed),
    )


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    start, end = _embed_segment(a, b, embed=radius * 0.7)
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
        material=material,
        name=name,
    )


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _tower_half_span(
    z: float, *, leg_base_z: float, leg_top_z: float, base_half: float, top_half: float
) -> float:
    t = (z - leg_base_z) / (leg_top_z - leg_base_z)
    return _lerp(base_half, top_half, t)


def _tower_corners(
    z: float, *, leg_base_z: float, leg_top_z: float, base_half: float, top_half: float
) -> list[tuple[float, float, float]]:
    half = _tower_half_span(
        z,
        leg_base_z=leg_base_z,
        leg_top_z=leg_top_z,
        base_half=base_half,
        top_half=top_half,
    )
    return [
        (half, half, z),
        (half, -half, z),
        (-half, -half, z),
        (-half, half, z),
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skeletal_lighthouse")

    weathered_steel = model.material(
        "weathered_steel", rgba=(0.49, 0.50, 0.52, 1.0)
    )
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    guard_paint = model.material("guard_paint", rgba=(0.80, 0.82, 0.80, 1.0))
    beacon_red = model.material("beacon_red", rgba=(0.62, 0.14, 0.11, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.77, 0.89, 0.95, 0.45))
    service_grey = model.material("service_grey", rgba=(0.58, 0.60, 0.63, 1.0))
    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.65, 1.0))

    tower = model.part("tower")

    base_half = 0.58
    top_half = 0.40
    leg_base_z = 0.10
    leg_top_z = 3.18
    platform_top_z = 3.24
    platform_bottom_z = 3.18
    guard_base_z = platform_top_z
    guard_mid_z = 3.54
    guard_top_z = 3.84
    post_top_z = 3.60

    for sx in (-base_half, base_half):
        for sy in (-base_half, base_half):
            tower.visual(
                Box((0.24, 0.24, 0.16)),
                origin=Origin(xyz=(sx, sy, 0.08)),
                material=concrete,
            )
            tower.visual(
                Box((0.11, 0.11, 0.05)),
                origin=Origin(xyz=(sx, sy, 0.185)),
                material=dark_steel,
            )

    bottom_corners = _tower_corners(
        leg_base_z,
        leg_base_z=leg_base_z,
        leg_top_z=leg_top_z,
        base_half=base_half,
        top_half=top_half,
    )
    top_corners = _tower_corners(
        leg_top_z,
        leg_base_z=leg_base_z,
        leg_top_z=leg_top_z,
        base_half=base_half,
        top_half=top_half,
    )

    for bottom, top in zip(bottom_corners, top_corners):
        _add_member(tower, bottom, top, 0.045, weathered_steel)

    levels = [0.42, 1.12, 1.86, 2.56, 3.10]
    level_corners = [
        _tower_corners(
            z,
            leg_base_z=leg_base_z,
            leg_top_z=leg_top_z,
            base_half=base_half,
            top_half=top_half,
        )
        for z in levels
    ]

    for corners in level_corners:
        for i in range(4):
            _add_member(
                tower,
                corners[i],
                corners[(i + 1) % 4],
                0.024,
                weathered_steel,
            )

    for z0, z1, lower, upper in zip(levels[:-1], levels[1:], level_corners[:-1], level_corners[1:]):
        del z0, z1
        for i in range(4):
            j = (i + 1) % 4
            _add_member(tower, lower[i], upper[j], 0.020, weathered_steel)
            _add_member(tower, lower[j], upper[i], 0.020, weathered_steel)

    for lower, upper in zip(level_corners[:-1], level_corners[1:]):
        _add_member(tower, lower[0], upper[2], 0.017, dark_steel)
        _add_member(tower, lower[1], upper[3], 0.017, dark_steel)

    south_attach_bottom = (
        0.0,
        -_tower_half_span(
            0.38,
            leg_base_z=leg_base_z,
            leg_top_z=leg_top_z,
            base_half=base_half,
            top_half=top_half,
        ),
        0.38,
    )
    south_attach_top = (
        0.0,
        -_tower_half_span(
            2.98,
            leg_base_z=leg_base_z,
            leg_top_z=leg_top_z,
            base_half=base_half,
            top_half=top_half,
        ),
        2.98,
    )
    ladder_left_bottom = (0.08, south_attach_bottom[1] - 0.05, south_attach_bottom[2])
    ladder_left_top = (0.08, south_attach_top[1] - 0.05, south_attach_top[2])
    ladder_right_bottom = (-0.08, south_attach_bottom[1] - 0.05, south_attach_bottom[2])
    ladder_right_top = (-0.08, south_attach_top[1] - 0.05, south_attach_top[2])
    _add_member(tower, ladder_left_bottom, ladder_left_top, 0.014, guard_paint)
    _add_member(tower, ladder_right_bottom, ladder_right_top, 0.014, guard_paint)
    for t in (0.0, 0.35, 0.70, 1.0):
        attach_z = _lerp(south_attach_bottom[2], south_attach_top[2], t)
        face_y = -_tower_half_span(
            attach_z,
            leg_base_z=leg_base_z,
            leg_top_z=leg_top_z,
            base_half=base_half,
            top_half=top_half,
        )
        ladder_y = face_y - 0.05
        _add_member(tower, (0.08, face_y, attach_z), (0.08, ladder_y, attach_z), 0.010, dark_steel)
        _add_member(tower, (-0.08, face_y, attach_z), (-0.08, ladder_y, attach_z), 0.010, dark_steel)
    rung_count = 13
    for i in range(rung_count + 1):
        z = _lerp(south_attach_bottom[2] + 0.02, south_attach_top[2] - 0.02, i / rung_count)
        face_y = -_tower_half_span(
            z,
            leg_base_z=leg_base_z,
            leg_top_z=leg_top_z,
            base_half=base_half,
            top_half=top_half,
        )
        ladder_y = face_y - 0.05
        _add_member(tower, (0.08, ladder_y, z), (-0.08, ladder_y, z), 0.010, guard_paint)

    tower.visual(
        Box((1.02, 1.02, platform_top_z - platform_bottom_z)),
        origin=Origin(xyz=(0.0, 0.0, (platform_top_z + platform_bottom_z) * 0.5)),
        material=dark_steel,
        name="platform_plate",
    )
    tower.visual(
        Box((0.90, 0.90, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, platform_top_z + 0.009)),
        material=service_grey,
        name="deck_grate",
    )

    guard_posts = [
        (0.46, 0.46),
        (0.46, -0.46),
        (-0.46, -0.46),
        (-0.46, 0.46),
        (0.46, 0.0),
        (0.0, -0.46),
        (-0.46, 0.0),
        (0.0, 0.46),
    ]
    for x, y in guard_posts:
        _add_member(tower, (x, y, guard_base_z), (x, y, guard_top_z), 0.013, guard_paint)
    for rail_z in (guard_mid_z, guard_top_z):
        ring = [
            (0.46, 0.46, rail_z),
            (0.46, -0.46, rail_z),
            (-0.46, -0.46, rail_z),
            (-0.46, 0.46, rail_z),
        ]
        for i in range(4):
            _add_member(tower, ring[i], ring[(i + 1) % 4], 0.012, guard_paint)
        _add_member(tower, (0.46, 0.0, rail_z), (0.46, 0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (0.46, 0.0, rail_z), (0.46, -0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (-0.46, 0.0, rail_z), (-0.46, 0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (-0.46, 0.0, rail_z), (-0.46, -0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (0.0, 0.46, rail_z), (0.46, 0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (0.0, 0.46, rail_z), (-0.46, 0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (0.0, -0.46, rail_z), (0.46, -0.46, rail_z), 0.012, guard_paint)
        _add_member(tower, (0.0, -0.46, rail_z), (-0.46, -0.46, rail_z), 0.012, guard_paint)

    tower.visual(
        Cylinder(radius=0.19, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 3.285)),
        material=dark_steel,
        name="tower_cap",
    )
    tower.visual(
        Cylinder(radius=0.055, length=post_top_z - 3.322),
        origin=Origin(xyz=(0.0, 0.0, (3.322 + post_top_z) * 0.5)),
        material=weathered_steel,
        name="center_post",
    )
    tower.visual(
        Cylinder(radius=0.072, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, post_top_z - 0.012)),
        material=service_grey,
        name="post_flange",
    )

    tower.visual(
        Box((0.012, 0.158, 0.012)),
        origin=Origin(xyz=(-0.094, 0.0, 3.170)),
        material=service_grey,
        name="panel_hinge_backer",
    )
    tower.visual(
        Box((0.014, 0.024, 0.024)),
        origin=Origin(xyz=(-0.093, -0.047, 3.168)),
        material=service_grey,
        name="panel_hinge_bracket_left",
    )
    tower.visual(
        Box((0.014, 0.024, 0.024)),
        origin=Origin(xyz=(-0.093, 0.047, 3.168)),
        material=service_grey,
        name="panel_hinge_bracket_right",
    )
    tower.visual(
        Box((0.014, 0.156, 0.016)),
        origin=Origin(xyz=(0.067, 0.0, 3.172)),
        material=service_grey,
        name="panel_frame_right",
    )
    tower.visual(
        Box((0.154, 0.014, 0.016)),
        origin=Origin(xyz=(-0.005, -0.073, 3.172)),
        material=service_grey,
        name="panel_frame_front",
    )
    tower.visual(
        Box((0.154, 0.014, 0.016)),
        origin=Origin(xyz=(-0.005, 0.073, 3.172)),
        material=service_grey,
        name="panel_frame_rear",
    )

    tower.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 4.00)),
        mass=130.0,
        origin=Origin(xyz=(0.0, 0.0, 2.0)),
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.12, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=dark_steel,
        name="beacon_turntable",
    )
    beacon.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=service_grey,
        name="beacon_pedestal",
    )
    beacon.visual(
        Box((0.17, 0.14, 0.055)),
        origin=Origin(xyz=(0.012, 0.0, 0.102)),
        material=dark_steel,
        name="beacon_saddle",
    )
    beacon.visual(
        Cylinder(radius=0.086, length=0.260),
        origin=Origin(xyz=(0.050, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_red,
        name="beacon_body",
    )
    beacon.visual(
        Cylinder(radius=0.098, length=0.032),
        origin=Origin(xyz=(0.196, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=service_grey,
        name="lens_bezel",
    )
    beacon.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.214, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_front",
    )
    beacon.visual(
        Box((0.090, 0.120, 0.120)),
        origin=Origin(xyz=(-0.118, 0.0, 0.138)),
        material=dark_steel,
        name="beacon_motor_box",
    )
    beacon.visual(
        Box((0.070, 0.118, 0.028)),
        origin=Origin(xyz=(0.140, 0.0, 0.225)),
        material=service_grey,
        name="beacon_visor",
    )
    _add_member(beacon, (-0.050, -0.050, 0.098), (0.015, -0.050, 0.184), 0.012, service_grey)
    _add_member(beacon, (-0.050, 0.050, 0.098), (0.015, 0.050, 0.184), 0.012, service_grey)
    _add_member(beacon, (0.030, -0.038, 0.098), (0.090, -0.038, 0.184), 0.010, service_grey)
    _add_member(beacon, (0.030, 0.038, 0.098), (0.090, 0.038, 0.184), 0.010, service_grey)
    beacon.inertial = Inertial.from_geometry(
        Box((0.46, 0.28, 0.28)),
        mass=10.0,
        origin=Origin(xyz=(0.02, 0.0, 0.14)),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Cylinder(radius=0.006, length=0.136),
        origin=Origin(xyz=(0.0, 0.0, -0.009), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=service_grey,
        name="panel_hinge_barrel",
    )
    access_panel.visual(
        Box((0.018, 0.024, 0.006)),
        origin=Origin(xyz=(0.009, -0.040, -0.004)),
        material=service_grey,
        name="panel_hinge_strap_left",
    )
    access_panel.visual(
        Box((0.018, 0.024, 0.006)),
        origin=Origin(xyz=(0.009, 0.040, -0.004)),
        material=service_grey,
        name="panel_hinge_strap_right",
    )
    access_panel.visual(
        Box((0.136, 0.126, 0.007)),
        origin=Origin(xyz=(0.068, 0.0, -0.0035)),
        material=service_grey,
        name="panel_leaf",
    )
    access_panel.visual(
        Box((0.094, 0.070, 0.007)),
        origin=Origin(xyz=(0.086, 0.0, -0.011)),
        material=dark_steel,
        name="panel_stiffener",
    )
    access_panel.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.110, 0.0, -0.018)),
        material=guard_paint,
        name="panel_handle",
    )
    access_panel.inertial = Inertial.from_geometry(
        Box((0.136, 0.126, 0.028)),
        mass=0.8,
        origin=Origin(xyz=(0.068, 0.0, -0.009)),
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, post_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0),
    )
    model.articulation(
        "access_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=access_panel,
        origin=Origin(xyz=(-0.08, 0.0, platform_bottom_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    access_panel = object_model.get_part("access_panel")
    beacon_spin = object_model.get_articulation("beacon_spin")
    panel_hinge = object_model.get_articulation("access_panel_hinge")

    ctx.expect_gap(
        beacon,
        tower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="beacon_turntable",
        negative_elem="post_flange",
        name="beacon turntable seats on the center post flange",
    )
    ctx.expect_gap(
        tower,
        access_panel,
        axis="z",
        max_gap=0.0015,
        max_penetration=1e-6,
        positive_elem="platform_plate",
        negative_elem="panel_leaf",
        name="access panel closes flush beneath the platform",
    )
    ctx.expect_overlap(
        access_panel,
        tower,
        axes="xy",
        min_overlap=0.12,
        elem_a="panel_leaf",
        elem_b="platform_plate",
        name="access panel remains under the platform footprint when closed",
    )

    closed_lens = _aabb_center(ctx.part_element_world_aabb(beacon, elem="lens_front"))
    with ctx.pose({beacon_spin: math.pi / 2.0}):
        spun_lens = _aabb_center(ctx.part_element_world_aabb(beacon, elem="lens_front"))
    ctx.check(
        "beacon housing rotates around the vertical post axis",
        closed_lens is not None
        and spun_lens is not None
        and closed_lens[0] > 0.15
        and abs(closed_lens[1]) < 0.03
        and abs(spun_lens[0]) < 0.04
        and spun_lens[1] > 0.15
        and abs(spun_lens[2] - closed_lens[2]) < 0.01,
        details=f"closed_lens={closed_lens}, spun_lens={spun_lens}",
    )

    closed_panel = _aabb_center(ctx.part_element_world_aabb(access_panel, elem="panel_leaf"))
    with ctx.pose({panel_hinge: math.radians(70.0)}):
        open_panel = _aabb_center(ctx.part_element_world_aabb(access_panel, elem="panel_leaf"))
    ctx.check(
        "access panel swings downward from the underside hinge",
        closed_panel is not None
        and open_panel is not None
        and open_panel[2] < closed_panel[2] - 0.05
        and abs(open_panel[1] - closed_panel[1]) < 0.02,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
