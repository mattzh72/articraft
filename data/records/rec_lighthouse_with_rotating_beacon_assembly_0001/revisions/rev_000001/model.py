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
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _add_ring_posts(
    part,
    *,
    radius: float,
    count: int,
    post_radius: float,
    post_height: float,
    base_z: float,
    material,
) -> None:
    center_z = base_z + (post_height * 0.5)
    for index in range(count):
        angle = (2.0 * math.pi * index) / count
        x, y = _polar(radius, angle)
        part.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x, y, center_z)),
            material=material,
            name=f"post_{count}_{index}",
        )


def _add_radial_window(
    part,
    *,
    angle: float,
    center_z: float,
    radius: float,
    pane_size: tuple[float, float, float],
    trim_size: tuple[float, float, float],
    pane_material,
    trim_material,
    prefix: str,
) -> None:
    x, y = _polar(radius, angle)
    trim_x, trim_y = _polar(radius + 0.01, angle)
    yaw = angle - (math.pi * 0.5)
    part.visual(
        Box(pane_size),
        origin=Origin(xyz=(x, y, center_z), rpy=(0.0, 0.0, yaw)),
        material=pane_material,
        name=f"{prefix}_pane",
    )
    part.visual(
        Box(trim_size),
        origin=Origin(xyz=(trim_x, trim_y, center_z), rpy=(0.0, 0.0, yaw)),
        material=trim_material,
        name=f"{prefix}_trim",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architectural_lighthouse", assets=ASSETS)

    masonry = Material(name="painted_masonry", rgba=(0.95, 0.95, 0.92, 1.0))
    stone = Material(name="weathered_stone", rgba=(0.63, 0.64, 0.62, 1.0))
    roof_red = Material(name="oxide_red", rgba=(0.58, 0.16, 0.11, 1.0))
    black_metal = Material(name="black_metal", rgba=(0.15, 0.16, 0.18, 1.0))
    bronze = Material(name="brass_bronze", rgba=(0.73, 0.58, 0.24, 1.0))
    glass = Material(name="lantern_glass", rgba=(0.76, 0.87, 0.93, 0.34))
    optic_glass = Material(name="optic_glass", rgba=(0.95, 0.82, 0.42, 0.48))
    dark_glass = Material(name="dark_glass", rgba=(0.13, 0.18, 0.23, 0.92))
    lamp_glow = Material(name="lamp_glow", rgba=(1.0, 0.88, 0.55, 0.9))
    model.materials.extend(
        [
            masonry,
            stone,
            roof_red,
            black_metal,
            bronze,
            glass,
            optic_glass,
            dark_glass,
            lamp_glow,
        ]
    )

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=2.3, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=stone,
        name="terrace",
    )
    tower.visual(
        Cylinder(radius=1.82, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=stone,
        name="lower_plinth",
    )
    tower.visual(
        _save_mesh(
            LatheGeometry(
                [
                    (0.0, 0.0),
                    (1.62, 0.0),
                    (1.62, 0.22),
                    (1.48, 0.46),
                    (1.35, 1.08),
                    (1.24, 6.2),
                    (1.08, 11.0),
                    (0.95, 13.95),
                    (0.87, 14.95),
                    (0.0, 14.95),
                ],
                segments=72,
            ),
            "tower_body.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=masonry,
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=1.68, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=stone,
        name="base_molding",
    )
    tower.visual(
        Cylinder(radius=1.12, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 10.9)),
        material=stone,
        name="upper_band",
    )
    tower.visual(
        Cylinder(radius=1.18, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 15.1)),
        material=stone,
        name="cornice",
    )
    tower.visual(
        Cylinder(radius=1.06, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 15.27)),
        material=stone,
        name="cap_ring",
    )
    tower.visual(
        Box((1.7, 1.08, 2.35)),
        origin=Origin(xyz=(0.0, 1.74, 1.175)),
        material=masonry,
        name="entry_block",
    )
    tower.visual(
        Box((1.86, 1.16, 0.16)),
        origin=Origin(xyz=(0.0, 1.78, 2.47)),
        material=roof_red,
        name="entry_roof",
    )
    tower.visual(
        Box((0.76, 0.08, 1.56)),
        origin=Origin(xyz=(0.0, 2.24, 0.84)),
        material=dark_glass,
        name="door",
    )
    tower.visual(
        Box((0.98, 0.06, 1.82)),
        origin=Origin(xyz=(0.0, 2.23, 0.98)),
        material=stone,
        name="door_frame",
    )
    _add_radial_window(
        tower,
        angle=math.pi * 0.5,
        center_z=3.1,
        radius=1.23,
        pane_size=(0.40, 0.10, 0.92),
        trim_size=(0.56, 0.06, 1.08),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_low",
    )
    _add_radial_window(
        tower,
        angle=math.pi * 0.5,
        center_z=6.45,
        radius=1.13,
        pane_size=(0.34, 0.10, 0.86),
        trim_size=(0.50, 0.06, 1.00),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_mid",
    )
    _add_radial_window(
        tower,
        angle=math.pi * 0.5,
        center_z=9.85,
        radius=1.03,
        pane_size=(0.30, 0.10, 0.80),
        trim_size=(0.44, 0.06, 0.94),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_upper",
    )
    _add_radial_window(
        tower,
        angle=math.pi * 0.5,
        center_z=13.05,
        radius=0.92,
        pane_size=(0.26, 0.10, 0.74),
        trim_size=(0.40, 0.06, 0.88),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_top",
    )
    _add_radial_window(
        tower,
        angle=math.radians(210.0),
        center_z=5.2,
        radius=1.15,
        pane_size=(0.22, 0.09, 0.64),
        trim_size=(0.34, 0.06, 0.78),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_port",
    )
    _add_radial_window(
        tower,
        angle=math.radians(330.0),
        center_z=8.0,
        radius=1.08,
        pane_size=(0.22, 0.09, 0.64),
        trim_size=(0.34, 0.06, 0.78),
        pane_material=dark_glass,
        trim_material=stone,
        prefix="window_starboard",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=1.55, length=15.45),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.725)),
    )

    lantern_shell = model.part("lantern_shell")
    lantern_shell.visual(
        Cylinder(radius=1.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stone,
        name="gallery_deck",
    )
    lantern_shell.visual(
        Cylinder(radius=1.0, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=black_metal,
        name="lantern_plinth",
    )
    lantern_shell.visual(
        _save_mesh(
            TorusGeometry(radius=1.30, tube=0.022, radial_segments=18, tubular_segments=48),
            "gallery_mid_rail.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=black_metal,
        name="gallery_mid_rail",
    )
    lantern_shell.visual(
        _save_mesh(
            TorusGeometry(radius=1.30, tube=0.035, radial_segments=20, tubular_segments=56),
            "gallery_top_rail.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=black_metal,
        name="gallery_top_rail",
    )
    _add_ring_posts(
        lantern_shell,
        radius=1.30,
        count=16,
        post_radius=0.024,
        post_height=0.60,
        base_z=0.12,
        material=black_metal,
    )
    lantern_shell.visual(
        Cylinder(radius=0.92, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=black_metal,
        name="lantern_lower_ring",
    )
    lantern_shell.visual(
        Cylinder(radius=0.90, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=black_metal,
        name="lantern_upper_ring",
    )
    _add_ring_posts(
        lantern_shell,
        radius=0.84,
        count=10,
        post_radius=0.03,
        post_height=1.10,
        base_z=0.43,
        material=black_metal,
    )
    for index in range(10):
        angle = (2.0 * math.pi * index) / 10.0
        x, y = _polar(0.79, angle)
        lantern_shell.visual(
            Box((0.02, 0.52, 1.00)),
            origin=Origin(xyz=(x, y, 0.98), rpy=(0.0, 0.0, angle)),
            material=glass,
            name=f"glass_panel_{index}",
        )
    lantern_shell.visual(
        _save_mesh(
            ConeGeometry(radius=1.02, height=0.78, radial_segments=56, closed=True),
            "lantern_roof.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.95)),
        material=roof_red,
        name="lantern_roof",
    )
    lantern_shell.visual(
        Cylinder(radius=0.24, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=black_metal,
        name="vent_cap",
    )
    lantern_shell.visual(
        Sphere(radius=0.11),
        origin=Origin(xyz=(0.0, 0.0, 2.57)),
        material=bronze,
        name="vent_ball",
    )
    lantern_shell.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 2.79)),
        material=bronze,
        name="lightning_rod",
    )
    lantern_shell.visual(
        Cylinder(radius=0.12, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=black_metal,
        name="pedestal",
    )
    lantern_shell.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=bronze,
        name="bearing_collar",
    )
    lantern_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=1.05, length=2.95),
        mass=1100.0,
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
    )

    beacon = model.part("beacon")
    beacon.visual(
        Cylinder(radius=0.34, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_metal,
        name="turntable",
    )
    beacon.visual(
        Cylinder(radius=0.30, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=black_metal,
        name="lower_optic_ring",
    )
    beacon.visual(
        Cylinder(radius=0.30, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=black_metal,
        name="upper_optic_ring",
    )
    for index in range(4):
        angle = (math.pi * 0.5) * index
        arm_x, arm_y = _polar(0.15, angle)
        beacon.visual(
            Box((0.12, 0.04, 0.04)),
            origin=Origin(xyz=(arm_x, arm_y, 0.11), rpy=(0.0, 0.0, angle)),
            material=bronze,
            name=f"support_arm_{index}",
        )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        x, y = _polar(0.28, angle)
        beacon.visual(
            Cylinder(radius=0.015, length=0.36),
            origin=Origin(xyz=(x, y, 0.31)),
            material=black_metal,
            name=f"optic_post_{index}",
        )
    for index in range(4):
        angle = (math.pi * 0.5) * index
        x, y = _polar(0.22, angle)
        beacon.visual(
            Box((0.04, 0.42, 0.32)),
            origin=Origin(xyz=(x, y, 0.31), rpy=(0.0, 0.0, angle)),
            material=optic_glass,
            name=f"optic_panel_{index}",
        )
    beacon.visual(
        Cylinder(radius=0.06, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=bronze,
        name="lamp_column",
    )
    beacon.visual(
        Sphere(radius=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=lamp_glow,
        name="lamp_core",
    )
    beacon.visual(
        Box((0.18, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, -0.20, 0.09)),
        material=black_metal,
        name="drive_motor",
    )
    beacon.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.56),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    model.articulation(
        "tower_to_lantern",
        ArticulationType.FIXED,
        parent="tower",
        child="lantern_shell",
        origin=Origin(xyz=(0.0, 0.0, 15.36)),
    )
    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent="lantern_shell",
        child="beacon",
        origin=Origin(xyz=(0.0, 0.0, 0.97)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "beacon",
        "lantern_shell",
        reason="The rotating beacon turntable sits captured within the lantern bearing collar, so conservative collision hulls report intended contact.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("lantern_shell", "tower", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("lantern_shell", "tower", axes="xy", min_overlap=1.6)
    ctx.expect_aabb_gap("lantern_shell", "tower", axis="z", max_gap=0.003, max_penetration=0.09)

    for angle in (0.0, math.pi * 0.25, math.pi * 0.5, math.pi, math.pi * 1.5):
        with ctx.pose(beacon_rotation=angle):
            ctx.expect_origin_distance("beacon", "lantern_shell", axes="xy", max_dist=0.02)
            ctx.expect_origin_distance("beacon", "tower", axes="xy", max_dist=0.02)
            ctx.expect_aabb_overlap("beacon", "lantern_shell", axes="xy", min_overlap=0.55)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
