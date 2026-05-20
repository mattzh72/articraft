from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ConeGeometry,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    VentGrilleGeometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _grille_panel_geometry(
    panel_size,
    thickness,
    *,
    frame,
    slat_pitch,
    slat_width,
    slat_angle_deg,
    corner_radius,
    center=True,
):
    geom = VentGrilleGeometry(
        panel_size,
        frame=frame,
        face_thickness=thickness,
        duct_depth=max(0.0015, thickness * 0.75),
        duct_wall=max(0.001, min(frame * 0.45, thickness * 0.75)),
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
    )
    if not center:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return geom

STONE = Material(name="stone", rgba=(0.74, 0.72, 0.67, 1.0))
STONE_TRIM = Material(name="stone_trim", rgba=(0.64, 0.62, 0.57, 1.0))
COPPER = Material(name="copper", rgba=(0.33, 0.50, 0.45, 1.0))
BRONZE = Material(name="bronze", rgba=(0.43, 0.30, 0.18, 1.0))
DIAL = Material(name="dial", rgba=(0.94, 0.93, 0.89, 1.0))
DARK_METAL = Material(name="dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
GOLD = Material(name="gold", rgba=(0.77, 0.63, 0.24, 1.0))
GLASS = Material(name="glass", rgba=(0.82, 0.89, 0.94, 0.30))
WINDOW_DARK = Material(name="window_dark", rgba=(0.10, 0.11, 0.12, 1.0))


def _offset_along(axis: str, sign: float, dist: float) -> tuple[float, float, float]:
    if axis == "y":
        return (0.0, sign * dist, 0.0)
    return (sign * dist, 0.0, 0.0)


def _face_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, math.pi / 2.0, 0.0)


def _add_clock_face(
    part,
    *,
    anchor: tuple[float, float, float],
    axis: str,
    sign: float,
    include_static_hands: bool = False,
    minute_angle: float = 0.0,
    hour_angle: float = 0.0,
) -> None:
    ax, ay, az = anchor
    rpy = _face_rpy(axis)

    def dial_xyz(dist: float) -> tuple[float, float, float]:
        dx, dy, dz = _offset_along(axis, sign, dist)
        return (ax + dx, ay + dy, az + dz)

    part.visual(
        Cylinder(radius=0.115, length=0.020),
        origin=Origin(xyz=dial_xyz(0.010), rpy=rpy),
        material=BRONZE,
    )
    part.visual(
        Cylinder(radius=0.101, length=0.010),
        origin=Origin(xyz=dial_xyz(0.006), rpy=rpy),
        material=DIAL,
    )
    part.visual(
        Cylinder(radius=0.090, length=0.002),
        origin=Origin(xyz=dial_xyz(0.0105), rpy=rpy),
        material=DARK_METAL,
    )
    part.visual(
        Cylinder(radius=0.104, length=0.002),
        origin=Origin(xyz=dial_xyz(0.0145), rpy=rpy),
        material=GLASS,
    )

    marker_radius = 0.084
    for i in range(12):
        angle = i * math.pi / 6.0
        quarter = i % 3 == 0
        marker_len = 0.023 if quarter else 0.016
        if axis == "y":
            part.visual(
                Box((0.0065 if quarter else 0.0045, 0.003, marker_len)),
                origin=Origin(
                    xyz=(
                        ax + marker_radius * math.sin(angle),
                        ay + sign * 0.0105,
                        az + marker_radius * math.cos(angle),
                    ),
                    rpy=(0.0, angle, 0.0),
                ),
                material=DARK_METAL,
            )
        else:
            part.visual(
                Box((0.003, 0.0065 if quarter else 0.0045, marker_len)),
                origin=Origin(
                    xyz=(
                        ax + sign * 0.0105,
                        ay + marker_radius * math.sin(angle),
                        az + marker_radius * math.cos(angle),
                    ),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=DARK_METAL,
            )

    if not include_static_hands:
        part.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=dial_xyz(0.016), rpy=rpy),
            material=BRONZE,
        )
        return

    plane_dist = 0.0175
    part.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=dial_xyz(plane_dist), rpy=rpy),
        material=BRONZE,
    )

    def add_hand(
        angle: float, length: float, width: float, tail: float, material: Material
    ) -> None:
        if axis == "y":
            part.visual(
                Box((width, 0.003, length)),
                origin=Origin(
                    xyz=(
                        ax + math.sin(angle) * (length / 2.0),
                        ay + sign * plane_dist,
                        az + math.cos(angle) * (length / 2.0),
                    ),
                    rpy=(0.0, angle, 0.0),
                ),
                material=material,
            )
            part.visual(
                Box((width * 0.8, 0.003, tail)),
                origin=Origin(
                    xyz=(
                        ax - math.sin(angle) * (tail / 2.0),
                        ay + sign * plane_dist,
                        az - math.cos(angle) * (tail / 2.0),
                    ),
                    rpy=(0.0, angle, 0.0),
                ),
                material=material,
            )
        else:
            part.visual(
                Box((0.003, width, length)),
                origin=Origin(
                    xyz=(
                        ax + sign * plane_dist,
                        ay + math.sin(angle) * (length / 2.0),
                        az + math.cos(angle) * (length / 2.0),
                    ),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=material,
            )
            part.visual(
                Box((0.003, width * 0.8, tail)),
                origin=Origin(
                    xyz=(
                        ax + sign * plane_dist,
                        ay - math.sin(angle) * (tail / 2.0),
                        az - math.cos(angle) * (tail / 2.0),
                    ),
                    rpy=(angle, 0.0, 0.0),
                ),
                material=material,
            )

    add_hand(hour_angle, 0.058, 0.012, 0.016, DARK_METAL)
    add_hand(minute_angle, 0.083, 0.008, 0.013, GOLD)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clock_tower", assets=ASSETS)

    spire_mesh = mesh_from_geometry(
        ConeGeometry(radius=0.105, height=0.340, radial_segments=24, closed=True),
        ASSETS.mesh_path("clock_tower_spire.obj"),
    )
    louver_mesh = mesh_from_geometry(
        _grille_panel_geometry(
            panel_size=(0.120, 0.150),
            thickness=0.016,
            frame=0.010,
            slat_pitch=0.022,
            slat_width=0.010,
            slat_angle_deg=34.0,
            corner_radius=0.008,
            center=True,
        ),
        ASSETS.mesh_path("clock_tower_louver.obj"),
    )

    tower = model.part("clock_tower")
    tower.visual(
        Box((0.440, 0.440, 0.090)), origin=Origin(xyz=(0.0, 0.0, 0.045)), material=STONE_TRIM
    )
    tower.visual(Box((0.380, 0.380, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.120)), material=STONE)
    tower.visual(
        Box((0.180, 0.120, 0.022)), origin=Origin(xyz=(0.0, 0.280, 0.011)), material=STONE_TRIM
    )
    tower.visual(Box((0.140, 0.080, 0.018)), origin=Origin(xyz=(0.0, 0.310, 0.031)), material=STONE)
    tower.visual(Box((0.300, 0.300, 0.560)), origin=Origin(xyz=(0.0, 0.0, 0.430)), material=STONE)

    for z in (0.240, 0.360, 0.480, 0.600):
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                tower.visual(
                    Box((0.040, 0.040, 0.090)),
                    origin=Origin(xyz=(sx * 0.127, sy * 0.127, z)),
                    material=STONE_TRIM,
                )

    for sx in (-1.0, 1.0):
        tower.visual(
            Box((0.028, 0.018, 0.420)),
            origin=Origin(xyz=(sx * 0.090, 0.141, 0.450)),
            material=STONE_TRIM,
        )
        tower.visual(
            Box((0.028, 0.018, 0.420)),
            origin=Origin(xyz=(sx * 0.090, -0.141, 0.450)),
            material=STONE_TRIM,
        )
        tower.visual(
            Box((0.018, 0.028, 0.420)),
            origin=Origin(xyz=(0.141, sx * 0.090, 0.450)),
            material=STONE_TRIM,
        )
        tower.visual(
            Box((0.018, 0.028, 0.420)),
            origin=Origin(xyz=(-0.141, sx * 0.090, 0.450)),
            material=STONE_TRIM,
        )

    tower.visual(
        Box((0.094, 0.010, 0.160)), origin=Origin(xyz=(0.0, 0.155, 0.235)), material=WINDOW_DARK
    )
    tower.visual(
        Box((0.120, 0.018, 0.016)), origin=Origin(xyz=(0.0, 0.150, 0.322)), material=STONE_TRIM
    )
    for sx in (-1.0, 1.0):
        tower.visual(
            Box((0.044, 0.010, 0.120)),
            origin=Origin(xyz=(sx * 0.070, 0.155, 0.470)),
            material=WINDOW_DARK,
        )
        tower.visual(
            Box((0.044, 0.010, 0.120)),
            origin=Origin(xyz=(sx * 0.070, -0.155, 0.470)),
            material=WINDOW_DARK,
        )
        tower.visual(
            Box((0.010, 0.044, 0.120)),
            origin=Origin(xyz=(0.155, sx * 0.070, 0.470)),
            material=WINDOW_DARK,
        )
        tower.visual(
            Box((0.010, 0.044, 0.120)),
            origin=Origin(xyz=(-0.155, sx * 0.070, 0.470)),
            material=WINDOW_DARK,
        )

    tower.visual(
        Box((0.340, 0.340, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.730)), material=STONE_TRIM
    )
    tower.visual(Box((0.340, 0.340, 0.210)), origin=Origin(xyz=(0.0, 0.0, 0.855)), material=STONE)
    tower.visual(
        Box((0.380, 0.380, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.985)), material=STONE_TRIM
    )
    tower.visual(Box((0.270, 0.270, 0.180)), origin=Origin(xyz=(0.0, 0.0, 1.095)), material=STONE)

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.032, 0.032, 0.180)),
                origin=Origin(xyz=(sx * 0.119, sy * 0.119, 1.095)),
                material=STONE_TRIM,
            )

    tower.visual(Box((0.310, 0.310, 0.040)), origin=Origin(xyz=(0.0, 0.0, 1.205)), material=COPPER)
    tower.visual(
        Box((0.180, 0.180, 0.080)), origin=Origin(xyz=(0.0, 0.0, 1.265)), material=STONE_TRIM
    )
    tower.visual(spire_mesh, origin=Origin(xyz=(0.0, 0.0, 1.450)), material=COPPER)
    tower.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 1.670)),
        material=GOLD,
    )

    for sign in (-1.0, 1.0):
        tower.visual(
            louver_mesh,
            origin=Origin(xyz=(0.0, sign * 0.141, 1.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=WINDOW_DARK,
        )
        tower.visual(
            louver_mesh,
            origin=Origin(xyz=(sign * 0.141, 0.0, 1.100), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=WINDOW_DARK,
        )

    _add_clock_face(
        tower,
        anchor=(0.0, -0.171, 0.855),
        axis="y",
        sign=-1.0,
        include_static_hands=True,
        minute_angle=math.radians(220.0),
        hour_angle=math.radians(305.0),
    )
    _add_clock_face(
        tower,
        anchor=(0.171, 0.0, 0.855),
        axis="x",
        sign=1.0,
        include_static_hands=True,
        minute_angle=math.radians(40.0),
        hour_angle=math.radians(300.0),
    )
    _add_clock_face(
        tower,
        anchor=(-0.171, 0.0, 0.855),
        axis="x",
        sign=-1.0,
        include_static_hands=True,
        minute_angle=math.radians(140.0),
        hour_angle=math.radians(25.0),
    )

    tower.inertial = Inertial.from_geometry(
        Box((0.440, 0.440, 1.700)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
    )

    front_dial = model.part("front_dial")
    _add_clock_face(
        front_dial,
        anchor=(0.0, 0.0, 0.0),
        axis="y",
        sign=1.0,
        include_static_hands=False,
    )
    front_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.020),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRONZE,
    )
    hour_hand.visual(
        Box((0.013, 0.002, 0.058)), origin=Origin(xyz=(0.0, 0.0, 0.029)), material=GOLD
    )
    hour_hand.visual(
        Box((0.009, 0.002, 0.015)), origin=Origin(xyz=(0.0, 0.0, -0.008)), material=GOLD
    )
    hour_hand.visual(
        Box((0.007, 0.002, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.051)), material=GOLD
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.013, 0.004, 0.073)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRONZE,
    )
    minute_hand.visual(
        Box((0.009, 0.002, 0.084)), origin=Origin(xyz=(0.0, 0.0, 0.042)), material=GOLD
    )
    minute_hand.visual(
        Box((0.006, 0.002, 0.016)), origin=Origin(xyz=(0.0, 0.0, -0.009)), material=GOLD
    )
    minute_hand.visual(
        Box((0.005, 0.002, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.076)), material=GOLD
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.009, 0.004, 0.102)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    hour_tip = model.part("hour_tip")
    hour_tip.visual(Sphere(radius=0.004), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=BRONZE)
    hour_tip.inertial = Inertial.from_geometry(
        Sphere(radius=0.004),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    minute_tip = model.part("minute_tip")
    minute_tip.visual(Sphere(radius=0.004), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=BRONZE)
    minute_tip.inertial = Inertial.from_geometry(
        Sphere(radius=0.004),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "tower_to_front_dial",
        ArticulationType.FIXED,
        parent="clock_tower",
        child="front_dial",
        origin=Origin(xyz=(0.0, 0.171, 0.855)),
    )
    model.articulation(
        "hour_hand_joint",
        ArticulationType.CONTINUOUS,
        parent="front_dial",
        child="hour_hand",
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )
    model.articulation(
        "minute_hand_joint",
        ArticulationType.CONTINUOUS,
        parent="front_dial",
        child="minute_hand",
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    model.articulation(
        "hour_tip_joint",
        ArticulationType.FIXED,
        parent="hour_hand",
        child="hour_tip",
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )
    model.articulation(
        "minute_tip_joint",
        ArticulationType.FIXED,
        parent="minute_hand",
        child="minute_tip",
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("front_dial", "clock_tower", axes="xy", min_overlap=0.015)
    ctx.expect_origin_distance("front_dial", "hour_hand", axes="xy", max_dist=0.030)
    ctx.expect_origin_distance("front_dial", "minute_hand", axes="xy", max_dist=0.032)
    ctx.expect_origin_distance("hour_hand", "minute_hand", axes="xy", max_dist=0.010)

    tower_pos = ctx.part_world_position("clock_tower")
    dial_pos = ctx.part_world_position("front_dial")
    assert dial_pos[2] > tower_pos[2] + 0.800, "Clock dial should sit high on the tower."
    assert dial_pos[1] > tower_pos[1] + 0.150, (
        "Front dial should project from the tower front face."
    )

    with ctx.pose(hour_hand_joint=0.0, minute_hand_joint=0.0):
        hour_tip_up = ctx.part_world_position("hour_tip")
        minute_tip_up = ctx.part_world_position("minute_tip")

    with ctx.pose(hour_hand_joint=math.pi / 2.0, minute_hand_joint=math.pi / 2.0):
        hour_tip_right = ctx.part_world_position("hour_tip")
        minute_tip_right = ctx.part_world_position("minute_tip")

    with ctx.pose(hour_hand_joint=math.pi, minute_hand_joint=math.pi):
        hour_tip_down = ctx.part_world_position("hour_tip")
        minute_tip_down = ctx.part_world_position("minute_tip")

    dial_x, _, dial_z = dial_pos

    def radius_xz(point: tuple[float, float, float]) -> float:
        return math.hypot(point[0] - dial_x, point[2] - dial_z)

    hour_radius_up = radius_xz(hour_tip_up)
    minute_radius_up = radius_xz(minute_tip_up)
    hour_radius_right = radius_xz(hour_tip_right)
    minute_radius_right = radius_xz(minute_tip_right)

    assert abs(hour_tip_up[0] - dial_x) < 0.008, "Hour hand should point vertically at zero pose."
    assert abs(minute_tip_up[0] - dial_x) < 0.008, (
        "Minute hand should point vertically at zero pose."
    )
    assert hour_tip_up[2] > dial_z + 0.050, "Hour hand tip should rise above dial center."
    assert minute_tip_up[2] > hour_tip_up[2] + 0.018, (
        "Minute hand should extend beyond the hour hand."
    )

    assert hour_tip_right[0] > dial_x + 0.040, (
        "Hour hand should swing to the right at quarter turn."
    )
    assert minute_tip_right[0] > hour_tip_right[0] + 0.020, (
        "Minute hand should remain longer at quarter turn."
    )
    assert abs(hour_tip_right[2] - dial_z) < 0.012, (
        "Hour tip should stay near dial center height at quarter turn."
    )
    assert abs(minute_tip_right[2] - dial_z) < 0.012, (
        "Minute tip should stay near dial center height at quarter turn."
    )

    assert hour_tip_down[2] < dial_z - 0.045, "Hour hand should point downward at half turn."
    assert minute_tip_down[2] < hour_tip_down[2] - 0.018, (
        "Minute hand should still extend past the hour hand downward."
    )

    assert 0.054 <= hour_radius_up <= 0.066, (
        "Hour hand radius should match a believable clock proportion."
    )
    assert 0.079 <= minute_radius_up <= 0.092, (
        "Minute hand radius should match a believable clock proportion."
    )
    assert abs(hour_radius_up - hour_radius_right) < 0.004, (
        "Hour tip radius should remain constant through rotation."
    )
    assert abs(minute_radius_up - minute_radius_right) < 0.004, (
        "Minute tip radius should remain constant through rotation."
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
