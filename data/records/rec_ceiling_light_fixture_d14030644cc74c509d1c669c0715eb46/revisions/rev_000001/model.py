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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _regular_polygon(radius: float, sides: int = 6, angle_offset: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + (2.0 * math.pi * index / sides)),
            radius * math.sin(angle_offset + (2.0 * math.pi * index / sides)),
        )
        for index in range(sides)
    ]


def _hex_section(vertex_radius: float, z_pos: float) -> list[tuple[float, float, float]]:
    return [(x, y, z_pos) for x, y in _regular_polygon(vertex_radius, sides=6, angle_offset=math.pi / 6.0)]


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_support_mesh() -> MeshGeometry:
    canopy_shell = LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.000),
            (0.048, -0.006),
            (0.084, -0.022),
            (0.076, -0.050),
            (0.036, -0.064),
        ],
        [
            (0.015, 0.000),
            (0.043, -0.008),
            (0.078, -0.024),
            (0.068, -0.048),
            (0.020, -0.060),
        ],
        segments=64,
    )
    stem_coupler = CylinderGeometry(radius=0.021, height=0.060, radial_segments=32).translate(
        0.0,
        0.0,
        -0.030,
    )
    drop_rod = CylinderGeometry(radius=0.011, height=0.520, radial_segments=28).translate(
        0.0,
        0.0,
        -0.310,
    )
    knuckle = SphereGeometry(radius=0.017).translate(0.0, 0.0, -0.092)
    lower_swivel = CylinderGeometry(radius=0.035, height=0.030, radial_segments=36).translate(
        0.0,
        0.0,
        -0.585,
    )
    return _merge_geometries(canopy_shell, stem_coupler, drop_rod, knuckle, lower_swivel)


def _build_body_core_mesh() -> MeshGeometry:
    core = section_loft(
        [
            _hex_section(0.082, -0.018),
            _hex_section(0.126, -0.072),
            _hex_section(0.184, -0.138),
            _hex_section(0.160, -0.238),
            _hex_section(0.102, -0.320),
            _hex_section(0.046, -0.350),
        ]
    )
    crown_band = CylinderGeometry(radius=0.088, height=0.018, radial_segments=48).translate(
        0.0,
        0.0,
        -0.034,
    )
    waist_band = CylinderGeometry(radius=0.128, height=0.028, radial_segments=42).translate(
        0.0,
        0.0,
        -0.092,
    )
    finial_stem = CylinderGeometry(radius=0.026, height=0.112, radial_segments=30).translate(
        0.0,
        0.0,
        -0.406,
    )
    finial_ball = SphereGeometry(radius=0.036).translate(0.0, 0.0, -0.477)
    return _merge_geometries(core, crown_band, waist_band, finial_stem, finial_ball)


def _build_arm_metal_mesh() -> MeshGeometry:
    arm_boss = CylinderGeometry(radius=0.018, height=0.048, radial_segments=24)
    arm_boss.rotate_y(math.pi / 2.0).translate(0.020, 0.0, 0.000)

    arm_curve = tube_from_spline_points(
        [
            (0.016, 0.000, 0.000),
            (0.080, 0.000, -0.008),
            (0.192, 0.000, 0.018),
            (0.305, 0.000, 0.058),
            (0.392, 0.000, 0.072),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )

    bobeche = LatheGeometry(
        [
            (0.000, -0.004),
            (0.016, -0.004),
            (0.045, 0.000),
            (0.060, 0.006),
            (0.050, 0.011),
            (0.000, 0.011),
        ],
        segments=32,
    ).translate(0.394, 0.0, 0.067)

    socket_collar = CylinderGeometry(radius=0.021, height=0.012, radial_segments=28).translate(
        0.394,
        0.0,
        0.078,
    )
    socket_body = CylinderGeometry(radius=0.018, height=0.088, radial_segments=28).translate(
        0.394,
        0.0,
        0.116,
    )
    shade_mount = CylinderGeometry(radius=0.026, height=0.010, radial_segments=28).translate(
        0.394,
        0.0,
        0.154,
    )
    return _merge_geometries(arm_boss, arm_curve, bobeche, socket_collar, socket_body, shade_mount)


def _build_candle_mesh() -> MeshGeometry:
    candle = CylinderGeometry(radius=0.011, height=0.118, radial_segments=24).translate(0.394, 0.0, 0.211)
    melt_cap = SphereGeometry(radius=0.012).translate(0.394, 0.0, 0.268)
    return _merge_geometries(candle, melt_cap)


def _build_shade_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.032, 0.000),
            (0.050, -0.016),
            (0.074, -0.058),
            (0.086, -0.094),
        ],
        [
            (0.027, -0.002),
            (0.045, -0.017),
            (0.068, -0.058),
            (0.080, -0.092),
        ],
        segments=48,
    ).translate(0.394, 0.0, 0.154)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_hexagonal_chandelier")

    aged_brass = model.material("aged_brass", rgba=(0.64, 0.53, 0.35, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.28, 0.23, 0.17, 1.0))
    candle_ivory = model.material("candle_ivory", rgba=(0.92, 0.89, 0.78, 1.0))
    shade_linen = model.material("shade_linen", rgba=(0.86, 0.82, 0.74, 1.0))

    support_mesh = mesh_from_geometry(_build_support_mesh(), "ceiling_support")
    body_core_mesh = mesh_from_geometry(_build_body_core_mesh(), "hex_body_core")
    arm_metal_mesh = mesh_from_geometry(_build_arm_metal_mesh(), "arm_metal")
    candle_mesh = mesh_from_geometry(_build_candle_mesh(), "arm_candle")
    shade_mesh = mesh_from_geometry(_build_shade_mesh(), "arm_shade")

    ceiling_support = model.part("ceiling_support")
    ceiling_support.visual(
        support_mesh,
        material=dark_bronze,
        name="support_assembly",
    )
    ceiling_support.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.585)),
        material=dark_bronze,
        name="lower_swivel_collar",
    )
    ceiling_support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.09, length=0.62),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.31)),
    )

    chandelier_body = model.part("chandelier_body")
    chandelier_body.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=aged_brass,
        name="top_hanger_ring",
    )
    chandelier_body.visual(
        body_core_mesh,
        material=aged_brass,
        name="hex_body_core",
    )

    arm_attach_radius = 0.158
    arm_attach_z = -0.138
    for index in range(6):
        yaw = index * math.tau / 6.0
        arm_origin = Origin(
            xyz=(
                arm_attach_radius * math.cos(yaw),
                arm_attach_radius * math.sin(yaw),
                arm_attach_z,
            ),
            rpy=(0.0, 0.0, yaw),
        )
        chandelier_body.visual(
            arm_metal_mesh,
            origin=arm_origin,
            material=aged_brass,
            name=f"arm_metal_{index}",
        )
        chandelier_body.visual(
            candle_mesh,
            origin=arm_origin,
            material=candle_ivory,
            name=f"candle_{index}",
        )
        chandelier_body.visual(
            shade_mesh,
            origin=arm_origin,
            material=shade_linen,
            name=f"shade_{index}",
        )

    chandelier_body.inertial = Inertial.from_geometry(
        Box((1.16, 1.16, 0.74)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
    )

    model.articulation(
        "stem_to_chandelier",
        ArticulationType.CONTINUOUS,
        parent=ceiling_support,
        child=chandelier_body,
        origin=Origin(xyz=(0.0, 0.0, -0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_support = object_model.get_part("ceiling_support")
    chandelier_body = object_model.get_part("chandelier_body")
    spin_joint = object_model.get_articulation("stem_to_chandelier")

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

    ctx.check(
        "spin joint is continuous and vertical",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(spin_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin_joint.articulation_type}, axis={spin_joint.axis}",
    )

    ctx.expect_contact(
        chandelier_body,
        ceiling_support,
        elem_a="top_hanger_ring",
        elem_b="lower_swivel_collar",
        contact_tol=1e-4,
        name="hanger ring seats against swivel collar",
    )
    ctx.expect_origin_gap(
        ceiling_support,
        chandelier_body,
        axis="z",
        min_gap=0.58,
        max_gap=0.62,
        name="body hangs below ceiling support",
    )

    arm_centers: list[tuple[float, float, float]] = []
    for index in range(6):
        chandelier_body.get_visual(f"arm_metal_{index}")
        chandelier_body.get_visual(f"candle_{index}")
        chandelier_body.get_visual(f"shade_{index}")
        arm_bounds = ctx.part_element_world_aabb(chandelier_body, elem=f"arm_metal_{index}")
        if arm_bounds is None:
            ctx.fail(f"arm {index} has measurable geometry", "world AABB missing")
            continue
        arm_min, arm_max = arm_bounds
        arm_centers.append(
            (
                0.5 * (arm_min[0] + arm_max[0]),
                0.5 * (arm_min[1] + arm_max[1]),
                0.5 * (arm_min[2] + arm_max[2]),
            )
        )

    if len(arm_centers) == 6:
        radii = [math.hypot(x, y) for x, y, _ in arm_centers]
        zs = [z for _, _, z in arm_centers]
        angles = sorted(math.atan2(y, x) % math.tau for x, y, _ in arm_centers)
        spacings = [
            (angles[(index + 1) % 6] - angles[index]) % math.tau
            for index in range(6)
        ]
        ctx.check(
            "six arms sit on a common ring",
            max(radii) - min(radii) < 0.03 and max(zs) - min(zs) < 0.03,
            details=f"radii={radii}, zs={zs}",
        )
        ctx.check(
            "six arms are evenly spaced around the body",
            all(abs(spacing - (math.tau / 6.0)) < 0.18 for spacing in spacings),
            details=f"angles={angles}, spacings={spacings}",
        )

    def _center_of_elem_aabb(elem_name: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(chandelier_body, elem=elem_name)
        if bounds is None:
            return None
        bb_min, bb_max = bounds
        return (
            0.5 * (bb_min[0] + bb_max[0]),
            0.5 * (bb_min[1] + bb_max[1]),
            0.5 * (bb_min[2] + bb_max[2]),
        )

    with ctx.pose({spin_joint: 0.0}):
        arm_rest = _center_of_elem_aabb("arm_metal_0")
    with ctx.pose({spin_joint: math.pi / 6.0}):
        arm_rotated = _center_of_elem_aabb("arm_metal_0")

    moved_around_axis = False
    if arm_rest is not None and arm_rotated is not None:
        moved_around_axis = (
            math.hypot(arm_rotated[0] - arm_rest[0], arm_rotated[1] - arm_rest[1]) > 0.10
            and abs(arm_rotated[2] - arm_rest[2]) < 0.01
        )
    ctx.check(
        "continuous joint rotates the chandelier body about vertical",
        moved_around_axis,
        details=f"rest={arm_rest}, rotated={arm_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
