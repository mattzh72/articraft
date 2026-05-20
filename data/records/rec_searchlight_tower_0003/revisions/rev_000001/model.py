from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    axis: str = "z",
):
    ring = boolean_difference(
        CylinderGeometry(radius=outer_radius, height=length, radial_segments=56),
        CylinderGeometry(radius=inner_radius, height=length + 0.004, radial_segments=56),
    )
    if axis == "x":
        ring.rotate_y(math.pi / 2.0)
    elif axis == "y":
        ring.rotate_x(math.pi / 2.0)
    return _save_mesh(name, ring)


def _rr_section(width: float, depth: float, radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _build_mast_mesh():
    return _save_mesh(
        "searchlight_mast.obj",
        section_loft(
            [
                _rr_section(0.34, 0.30, 0.045, 0.0),
                _rr_section(0.30, 0.26, 0.040, 0.72),
                _rr_section(0.25, 0.22, 0.035, 1.60),
            ]
        ),
    )


def _build_platform_deck_mesh():
    return _save_mesh(
        "service_platform_deck.obj",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.86, 0.70, 0.055),
            [_circle_profile(0.155, segments=36)],
            height=0.04,
            center=True,
        ),
    )


def _build_yoke_arm_mesh(side: float):
    return _save_mesh(
        f"yoke_arm_{'left' if side > 0.0 else 'right'}.obj",
        sweep_profile_along_spline(
            [
                (-0.035, side * 0.285, 0.21),
                (0.018, side * 0.288, 0.34),
                (0.070, side * 0.288, 0.45),
                (0.088, side * 0.285, 0.52),
            ],
            profile=rounded_rect_profile(0.080, 0.055, radius=0.014),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )


def _build_head_shell_mesh():
    return _save_mesh(
        "spotlight_head_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.135, -0.185),
                (0.176, -0.160),
                (0.202, -0.080),
                (0.208, 0.040),
                (0.212, 0.190),
                (0.224, 0.300),
                (0.216, 0.338),
            ],
            [
                (0.095, -0.176),
                (0.128, -0.154),
                (0.182, -0.072),
                (0.188, 0.040),
                (0.190, 0.180),
                (0.194, 0.286),
                (0.190, 0.318),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_reflector_mesh():
    return _save_mesh(
        "spotlight_reflector.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.050, -0.010),
                (0.095, 0.040),
                (0.145, 0.145),
                (0.182, 0.248),
                (0.188, 0.278),
            ],
            [
                (0.030, -0.006),
                (0.078, 0.040),
                (0.126, 0.140),
                (0.164, 0.244),
                (0.170, 0.274),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_searchlight_tower", assets=ASSETS)

    mast_paint = model.material("mast_paint", rgba=(0.78, 0.79, 0.76, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.44, 0.47, 0.50, 1.0))
    bright_reflector = model.material("bright_reflector", rgba=(0.82, 0.84, 0.86, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.70, 0.82, 0.90, 0.34))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))

    mast_mesh = _build_mast_mesh()
    platform_deck_mesh = _build_platform_deck_mesh()
    left_arm_mesh = _build_yoke_arm_mesh(1.0)
    right_arm_mesh = _build_yoke_arm_mesh(-1.0)
    head_shell_mesh = _build_head_shell_mesh()
    reflector_mesh = _build_reflector_mesh()
    lower_bearing_ring = _ring_mesh(
        "bearing_ring_lower.obj",
        outer_radius=0.170,
        inner_radius=0.112,
        length=0.080,
        axis="z",
    )
    upper_bearing_ring = _ring_mesh(
        "bearing_ring_upper.obj",
        outer_radius=0.220,
        inner_radius=0.116,
        length=0.120,
        axis="z",
    )
    bezel_ring = _ring_mesh(
        "spotlight_bezel_ring.obj",
        outer_radius=0.232,
        inner_radius=0.198,
        length=0.050,
        axis="x",
    )
    rear_vent_ring = _ring_mesh(
        "spotlight_rear_vent_ring.obj",
        outer_radius=0.106,
        inner_radius=0.085,
        length=0.014,
        axis="x",
    )
    left_trunnion_collar_mesh = _ring_mesh(
        "left_trunnion_collar.obj",
        outer_radius=0.073,
        inner_radius=0.052,
        length=0.100,
        axis="y",
    )
    right_trunnion_collar_mesh = _ring_mesh(
        "right_trunnion_collar.obj",
        outer_radius=0.073,
        inner_radius=0.052,
        length=0.100,
        axis="y",
    )

    tower = model.part("tower_base")
    tower.visual(
        Box((1.10, 1.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=graphite,
        name="foundation_plinth",
    )
    tower.visual(
        Box((0.72, 0.72, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=machined_steel,
        name="base_podium",
    )
    tower.visual(
        Cylinder(radius=0.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=satin_metal,
        name="mast_foot",
    )
    tower.visual(
        mast_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=mast_paint,
        name="mast_shell",
    )
    tower.visual(
        Box((0.17, 0.018, 0.48)),
        origin=Origin(xyz=(0.126, 0.0, 0.73)),
        material=graphite,
        name="access_door",
    )
    tower.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.145, 0.0, 0.74), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="door_handle",
    )
    tower.visual(
        Box((0.28, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.89)),
        material=graphite,
        name="mast_head_band",
    )
    tower.visual(
        platform_deck_mesh,
        origin=Origin(xyz=(0.02, 0.0, 1.96)),
        material=graphite,
        name="platform_deck",
    )
    for index, (x, y) in enumerate(((0.12, 0.12), (0.12, -0.12), (-0.12, 0.12), (-0.12, -0.12))):
        _add_member(
            tower,
            (x, y, 1.54),
            (0.02 + (0.25 if x > 0.0 else -0.25), 0.20 if y > 0.0 else -0.20, 1.94),
            0.016,
            machined_steel,
            name=f"platform_brace_{index:02d}",
        )
    rear_ladder_x = -0.145
    ladder_bottom = 0.44
    ladder_top = 1.94
    _add_member(
        tower,
        (rear_ladder_x, -0.030, ladder_bottom),
        (rear_ladder_x, -0.030, ladder_top),
        0.006,
        satin_metal,
        name="ladder_left_rail",
    )
    _add_member(
        tower,
        (rear_ladder_x, 0.030, ladder_bottom),
        (rear_ladder_x, 0.030, ladder_top),
        0.006,
        satin_metal,
        name="ladder_right_rail",
    )
    for rung_index in range(11):
        rung_z = ladder_bottom + (ladder_top - ladder_bottom) * rung_index / 10.0
        _add_member(
            tower,
            (rear_ladder_x, -0.030, rung_z),
            (rear_ladder_x, 0.030, rung_z),
            0.0036,
            satin_metal,
            name=f"ladder_rung_{rung_index:02d}",
        )
    tower.visual(
        Box((0.18, 0.12, 0.20)),
        origin=Origin(xyz=(-0.34, -0.18, 2.08)),
        material=graphite,
        name="service_cabinet",
    )
    tower.visual(
        Box((0.162, 0.006, 0.15)),
        origin=Origin(xyz=(-0.253, -0.18, 2.08)),
        material=satin_metal,
        name="cabinet_door_break",
    )
    tower.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(-0.25, -0.18, 2.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="cabinet_handle",
    )
    post_points = [
        (-0.36, -0.28),
        (-0.36, 0.28),
        (0.02, -0.28),
        (0.02, 0.28),
        (0.38, -0.20),
        (0.38, 0.20),
    ]
    for index, (x, y) in enumerate(post_points):
        _add_member(
            tower,
            (x, y, 1.98),
            (x, y, 2.16),
            0.008,
            satin_metal,
            name=f"rail_post_{index:02d}",
        )
    rail_segments = [
        ((-0.36, -0.28, 2.15), (-0.36, 0.28, 2.15)),
        ((0.02, -0.28, 2.15), (-0.36, -0.28, 2.15)),
        ((0.02, 0.28, 2.15), (-0.36, 0.28, 2.15)),
        ((0.02, -0.28, 2.07), (-0.36, -0.28, 2.07)),
        ((0.02, 0.28, 2.07), (-0.36, 0.28, 2.07)),
        ((-0.36, -0.28, 2.07), (-0.36, 0.28, 2.07)),
        ((0.38, -0.20, 2.15), (0.02, -0.28, 2.15)),
        ((0.38, 0.20, 2.15), (0.02, 0.28, 2.15)),
        ((0.38, -0.20, 2.07), (0.02, -0.28, 2.07)),
        ((0.38, 0.20, 2.07), (0.02, 0.28, 2.07)),
    ]
    for index, (a, b) in enumerate(rail_segments):
        _add_member(
            tower,
            a,
            b,
            0.006,
            satin_metal,
            name=f"guardrail_{index:02d}",
        )
    tower.visual(
        lower_bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 2.00)),
        material=machined_steel,
        name="bearing_ring_lower",
    )
    tower.visual(
        upper_bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 2.08)),
        material=machined_steel,
        name="bearing_ring_upper",
    )
    for index, (x, y) in enumerate(((0.13, 0.0), (-0.13, 0.0), (0.0, 0.13), (0.0, -0.13))):
        tower.visual(
            Box((0.05 if abs(x) > 0.0 else 0.08, 0.08 if abs(x) > 0.0 else 0.05, 0.17)),
            origin=Origin(xyz=(x, y, 2.05)),
            material=machined_steel,
            name=f"bearing_web_{index:02d}",
        )
    tower.inertial = Inertial.from_geometry(
        Box((1.10, 1.10, 2.20)),
        mass=820.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.100, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=machined_steel,
        name="azimuth_spindle",
    )
    pan_yoke.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=graphite,
        name="turntable_flange",
    )
    pan_yoke.visual(
        Cylinder(radius=0.16, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=graphite,
        name="pan_hub_shell",
    )
    pan_yoke.visual(
        Box((0.14, 0.18, 0.14)),
        origin=Origin(xyz=(-0.040, 0.0, 0.22)),
        material=graphite,
        name="center_pedestal",
    )
    pan_yoke.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(-0.120, 0.0, 0.15)),
        material=machined_steel,
        name="azimuth_drive_housing",
    )
    pan_yoke.visual(
        Box((0.12, 0.08, 0.28)),
        origin=Origin(xyz=(-0.005, 0.285, 0.32)),
        material=graphite,
        name="left_arm_root",
    )
    pan_yoke.visual(
        Box((0.12, 0.08, 0.28)),
        origin=Origin(xyz=(-0.005, -0.285, 0.32)),
        material=graphite,
        name="right_arm_root",
    )
    _add_member(
        pan_yoke,
        (-0.050, 0.080, 0.24),
        (-0.030, 0.250, 0.20),
        0.022,
        graphite,
        name="left_lower_brace",
    )
    _add_member(
        pan_yoke,
        (-0.050, -0.080, 0.24),
        (-0.030, -0.250, 0.20),
        0.022,
        graphite,
        name="right_lower_brace",
    )
    pan_yoke.visual(left_arm_mesh, material=mast_paint, name="left_yoke_arm")
    pan_yoke.visual(right_arm_mesh, material=mast_paint, name="right_yoke_arm")
    pan_yoke.visual(
        left_trunnion_collar_mesh,
        origin=Origin(xyz=(0.088, 0.285, 0.52)),
        material=machined_steel,
        name="left_trunnion_collar",
    )
    pan_yoke.visual(
        right_trunnion_collar_mesh,
        origin=Origin(xyz=(0.088, -0.285, 0.52)),
        material=machined_steel,
        name="right_trunnion_collar",
    )
    pan_yoke.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=Origin(xyz=(0.088, 0.338, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="left_bearing_cap",
    )
    pan_yoke.visual(
        Cylinder(radius=0.036, length=0.036),
        origin=Origin(xyz=(0.088, -0.338, 0.52), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="right_bearing_cap",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.44, 0.74, 0.92)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.040, length=0.47),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="tilt_spindle",
    )
    spotlight_head.visual(
        Cylinder(radius=0.067, length=0.040),
        origin=Origin(xyz=(0.0, 0.215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="left_hub_shoulder",
    )
    spotlight_head.visual(
        Cylinder(radius=0.067, length=0.040),
        origin=Origin(xyz=(0.0, -0.215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="right_hub_shoulder",
    )
    spotlight_head.visual(
        head_shell_mesh,
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mast_paint,
        name="head_shell",
    )
    spotlight_head.visual(
        Box((0.15, 0.19, 0.07)),
        origin=Origin(xyz=(0.060, 0.0, -0.10)),
        material=graphite,
        name="lower_service_band",
    )
    spotlight_head.visual(
        bezel_ring,
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        material=satin_metal,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.194, length=0.016),
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    spotlight_head.visual(
        Cylinder(radius=0.198, length=0.008),
        origin=Origin(xyz=(0.378, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gasket_black,
        name="lens_gasket",
    )
    spotlight_head.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_reflector,
        name="reflector_shell",
    )
    spotlight_head.visual(
        Cylinder(radius=0.054, length=0.100),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_reflector,
        name="lamp_core",
    )
    spotlight_head.visual(
        Cylinder(radius=0.132, length=0.050),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    spotlight_head.visual(
        rear_vent_ring,
        origin=Origin(xyz=(-0.146, 0.0, 0.0)),
        material=satin_metal,
        name="rear_vent_ring",
    )
    spotlight_head.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(-0.156, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gasket_black,
        name="rear_cable_gland",
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.76),
        mass=62.0,
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "tower_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4200.0, velocity=0.55),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_head,
        origin=Origin(xyz=(0.088, 0.0, 0.52)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.90,
            lower=-0.35,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower = object_model.get_part("tower_base")
    pan_yoke = object_model.get_part("pan_yoke")
    spotlight_head = object_model.get_part("spotlight_head")
    tower_to_pan = object_model.get_articulation("tower_to_pan")
    yoke_to_head = object_model.get_articulation("yoke_to_head")

    platform_deck = tower.get_visual("platform_deck")
    mast_shell = tower.get_visual("mast_shell")
    bearing_ring_upper = tower.get_visual("bearing_ring_upper")
    turntable_flange = pan_yoke.get_visual("turntable_flange")
    left_trunnion_collar = pan_yoke.get_visual("left_trunnion_collar")
    right_trunnion_collar = pan_yoke.get_visual("right_trunnion_collar")
    left_hub_shoulder = spotlight_head.get_visual("left_hub_shoulder")
    right_hub_shoulder = spotlight_head.get_visual("right_hub_shoulder")
    front_lens = spotlight_head.get_visual("front_lens")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(
        pan_yoke,
        tower,
        elem_a=turntable_flange,
        elem_b=bearing_ring_upper,
        name="turntable_flange_seats_on_bearing_ring",
    )
    ctx.expect_gap(
        pan_yoke,
        tower,
        axis="z",
        positive_elem=turntable_flange,
        negative_elem=bearing_ring_upper,
        max_gap=0.0015,
        max_penetration=0.0,
        name="turntable_flange_has_clean_seating_gap",
    )
    ctx.expect_overlap(
        pan_yoke,
        tower,
        axes="xy",
        elem_a=turntable_flange,
        elem_b=bearing_ring_upper,
        min_overlap=0.20,
        name="turntable_flange_covers_bearing_ring",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a=left_hub_shoulder,
        elem_b=left_trunnion_collar,
        name="left_trunnion_hub_is_supported",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_yoke,
        elem_a=right_hub_shoulder,
        elem_b=right_trunnion_collar,
        name="right_trunnion_hub_is_supported",
    )
    ctx.expect_gap(
        spotlight_head,
        tower,
        axis="z",
        min_gap=0.20,
        name="spotlight_head_clears_platform_at_rest",
    )
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    mast_aabb = ctx.part_element_world_aabb(tower, elem=mast_shell)
    deck_aabb = ctx.part_element_world_aabb(tower, elem=platform_deck)
    if mast_aabb is not None and deck_aabb is not None:
        deck_bottom = deck_aabb[0][2]
        mast_top = mast_aabb[1][2]
        ctx.check(
            "platform_deck_sits_at_mast_head",
            abs(deck_bottom - mast_top) <= 0.05,
            details=f"deck bottom {deck_bottom:.4f} vs mast top {mast_top:.4f}",
        )

    rest_head_pos = ctx.part_world_position(spotlight_head)
    if rest_head_pos is not None:
        with ctx.pose({tower_to_pan: math.pi / 2.0}):
            rotated_head_pos = ctx.part_world_position(spotlight_head)
            if rotated_head_pos is not None:
                ctx.check(
                    "pan_stage_rotates_head_around_tower_axis",
                    abs(rotated_head_pos[0]) <= 0.02
                    and abs(rotated_head_pos[1] - rest_head_pos[0]) <= 0.02
                    and abs(rotated_head_pos[2] - rest_head_pos[2]) <= 0.01,
                    details=(
                        f"rest={rest_head_pos!r} rotated={rotated_head_pos!r}"
                    ),
                )

    lens_aabb_rest = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
    if lens_aabb_rest is not None:
        lens_rest_center = tuple(
            (lens_aabb_rest[0][axis] + lens_aabb_rest[1][axis]) * 0.5 for axis in range(3)
        )
        with ctx.pose({yoke_to_head: 0.85}):
            lens_aabb_up = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
            if lens_aabb_up is not None:
                lens_up_center = tuple(
                    (lens_aabb_up[0][axis] + lens_aabb_up[1][axis]) * 0.5 for axis in range(3)
                )
                ctx.check(
                    "tilt_stage_lifts_front_lens",
                    lens_up_center[2] > lens_rest_center[2] + 0.12
                    and lens_up_center[0] < lens_rest_center[0] - 0.08
                    and abs(lens_up_center[1] - lens_rest_center[1]) <= 0.01,
                    details=f"rest={lens_rest_center!r} up={lens_up_center!r}",
                )
            ctx.expect_contact(spotlight_head, pan_yoke, elem_a=left_hub_shoulder, elem_b=left_trunnion_collar)
            ctx.expect_contact(spotlight_head, pan_yoke, elem_a=right_hub_shoulder, elem_b=right_trunnion_collar)

    with ctx.pose({yoke_to_head: -0.30}):
        ctx.expect_gap(
            spotlight_head,
            tower,
            axis="z",
            min_gap=0.12,
            name="spotlight_head_clears_platform_when_tilted_down",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
