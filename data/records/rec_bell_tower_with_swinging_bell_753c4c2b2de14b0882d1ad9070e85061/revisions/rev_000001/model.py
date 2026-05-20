from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]):
    return model.material(name, rgba=rgba)


def _arch_profile(width: float, spring: float, height: float, samples: int = 8):
    """Simple pointed Gothic arch loop in local XY with base at y=0."""
    half = width * 0.5
    pts: list[tuple[float, float]] = [(-half, 0.0), (-half, spring)]
    for i in range(1, samples + 1):
        t = i / samples
        # Polygonal lancet arch sides: tall and pointed rather than rounded Roman.
        x = -half * (1.0 - t)
        y = spring + (height - spring) * (t**0.82)
        pts.append((x, y))
    for i in range(1, samples + 1):
        t = i / samples
        x = half * t
        y = height - (height - spring) * (t**0.82)
        pts.append((x, y))
    pts.append((half, 0.0))
    return pts


def _lancet_frame_mesh() -> MeshGeometry:
    outer = _arch_profile(1.36, 0.96, 1.92, samples=7)
    inner = _arch_profile(0.92, 0.82, 1.55, samples=7)
    return ExtrudeWithHolesGeometry(outer, [inner], 0.10, center=True)


def _bell_mesh(scale: float = 1.0) -> MeshGeometry:
    outer = [
        (0.16 * scale, -0.32 * scale),
        (0.22 * scale, -0.42 * scale),
        (0.29 * scale, -0.62 * scale),
        (0.35 * scale, -0.86 * scale),
        (0.43 * scale, -1.08 * scale),
        (0.53 * scale, -1.28 * scale),
        (0.58 * scale, -1.39 * scale),
        (0.56 * scale, -1.47 * scale),
    ]
    inner = [
        (0.09 * scale, -0.41 * scale),
        (0.17 * scale, -0.55 * scale),
        (0.24 * scale, -0.76 * scale),
        (0.31 * scale, -1.00 * scale),
        (0.43 * scale, -1.24 * scale),
        (0.50 * scale, -1.38 * scale),
    ]
    bell = LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )
    bell.merge(TorusGeometry(radius=0.55 * scale, tube=0.035 * scale, radial_segments=14, tubular_segments=72).translate(0.0, 0.0, -1.39 * scale))
    bell.merge(TorusGeometry(radius=0.24 * scale, tube=0.020 * scale, radial_segments=12, tubular_segments=48).translate(0.0, 0.0, -0.50 * scale))
    return bell


def _octagonal_spire_ribs() -> MeshGeometry:
    ribs = MeshGeometry()
    base_r = 1.15
    for i in range(8):
        a = math.tau * i / 8.0 + math.pi / 8.0
        ribs.merge(
            tube_from_spline_points(
                [
                    (base_r * math.cos(a), base_r * math.sin(a), 6.48),
                    (0.04 * math.cos(a), 0.04 * math.sin(a), 9.12),
                ],
                radius=0.035,
                samples_per_segment=4,
                radial_segments=8,
                cap_ends=True,
            )
        )
    return ribs


def _add_side_opening(
    tower,
    *,
    side: str,
    arch_mesh,
    stone,
    shadow,
    timber,
):
    """Place a dark lancet opening, stone arch frame, and louvers on each side."""
    z_base = 2.98
    z_center = z_base + 0.85
    face = 1.735
    if side == "front":
        tower.visual(
            Box((1.04, 0.04, 1.46)),
            origin=Origin(xyz=(0.0, -face - 0.01, z_center)),
            material=shadow,
            name="front_belfry_shadow",
        )
        tower.visual(
            arch_mesh,
            origin=Origin(xyz=(0.0, -face, z_base), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stone,
            name="front_lancet_frame",
        )
        for idx, z in enumerate((3.48, 3.76, 4.04)):
            tower.visual(
                Box((1.10, 0.07, 0.075)),
                origin=Origin(xyz=(0.0, -face - 0.06, z), rpy=(0.12, 0.0, 0.0)),
                material=timber,
                name=f"front_louver_{idx}",
            )
    elif side == "back":
        tower.visual(
            Box((1.04, 0.04, 1.46)),
            origin=Origin(xyz=(0.0, face + 0.01, z_center)),
            material=shadow,
            name="rear_belfry_shadow",
        )
        tower.visual(
            arch_mesh,
            origin=Origin(xyz=(0.0, face, z_base), rpy=(math.pi / 2.0, 0.0, math.pi)),
            material=stone,
            name="rear_lancet_frame",
        )
        for idx, z in enumerate((3.48, 3.76, 4.04)):
            tower.visual(
                Box((1.10, 0.07, 0.075)),
                origin=Origin(xyz=(0.0, face + 0.06, z), rpy=(-0.12, 0.0, 0.0)),
                material=timber,
                name=f"rear_louver_{idx}",
            )
    elif side == "east":
        tower.visual(
            Box((0.04, 1.04, 1.46)),
            origin=Origin(xyz=(face + 0.01, 0.0, z_center)),
            material=shadow,
            name="east_belfry_shadow",
        )
        tower.visual(
            arch_mesh,
            origin=Origin(xyz=(face, 0.0, z_base), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
            material=stone,
            name="east_lancet_frame",
        )
        for idx, z in enumerate((3.48, 3.76, 4.04)):
            tower.visual(
                Box((0.07, 1.10, 0.075)),
                origin=Origin(xyz=(face + 0.06, 0.0, z), rpy=(0.0, -0.12, 0.0)),
                material=timber,
                name=f"east_louver_{idx}",
            )
    elif side == "west":
        tower.visual(
            Box((0.04, 1.04, 1.46)),
            origin=Origin(xyz=(-face - 0.01, 0.0, z_center)),
            material=shadow,
            name="west_belfry_shadow",
        )
        tower.visual(
            arch_mesh,
            origin=Origin(xyz=(-face, 0.0, z_base), rpy=(math.pi / 2.0, 0.0, -math.pi / 2.0)),
            material=stone,
            name="west_lancet_frame",
        )
        for idx, z in enumerate((3.48, 3.76, 4.04)):
            tower.visual(
                Box((0.07, 1.10, 0.075)),
                origin=Origin(xyz=(-face - 0.06, 0.0, z), rpy=(0.0, 0.12, 0.0)),
                material=timber,
                name=f"west_louver_{idx}",
            )


def _add_bell_assembly(
    model: ArticulatedObject,
    tower,
    index: int,
    x: float,
    *,
    bronze,
    dark_metal,
    timber,
):
    axle_z = 4.72
    bell_part = model.part(f"bell_{index}")
    bell_part.visual(
        Box((0.58, 0.68, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=timber,
        name="timber_yoke",
    )
    bell_part.visual(
        Cylinder(radius=0.045, length=1.28),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle_pin",
    )
    bell_part.visual(
        Box((0.075, 0.060, 0.52)),
        origin=Origin(xyz=(-0.18, 0.0, -0.35)),
        material=dark_metal,
        name="strap_0",
    )
    bell_part.visual(
        Box((0.075, 0.060, 0.52)),
        origin=Origin(xyz=(0.18, 0.0, -0.35)),
        material=dark_metal,
        name="strap_1",
    )
    bell_part.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        material=bronze,
        name="bell_crown",
    )
    bell_part.visual(
        Cylinder(radius=0.040, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, -0.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clapper_hanger",
    )
    bell_part.visual(
        Box((0.060, 0.060, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.485)),
        material=dark_metal,
        name="clapper_link",
    )
    bell_part.visual(
        mesh_from_geometry(_bell_mesh(), f"bell_{index}_bronze_shell"),
        material=bronze,
        name="bell_shell",
    )

    model.articulation(
        f"tower_to_bell_{index}",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell_part,
        origin=Origin(xyz=(x, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2, lower=-0.52, upper=0.52),
    )

    clapper = model.part(f"clapper_{index}")
    clapper.visual(
        Cylinder(radius=0.028, length=0.34),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clapper_pin",
    )
    clapper.visual(
        Box((0.040, 0.025, 0.080)),
        origin=Origin(xyz=(0.0, 0.158, -0.040)),
        material=dark_metal,
        name="clevis_side_0",
    )
    clapper.visual(
        Box((0.040, 0.025, 0.080)),
        origin=Origin(xyz=(0.0, -0.158, -0.040)),
        material=dark_metal,
        name="clevis_side_1",
    )
    clapper.visual(
        Cylinder(radius=0.018, length=0.316),
        origin=Origin(xyz=(0.0, 0.0, -0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clevis_crossbar",
    )
    clapper.visual(
        Cylinder(radius=0.026, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        material=dark_metal,
        name="clapper_rod",
    )
    clapper.visual(
        Sphere(radius=0.13),
        origin=Origin(xyz=(0.0, 0.0, -0.69)),
        material=dark_metal,
        name="clapper_ball",
    )
    model.articulation(
        f"bell_{index}_to_clapper_{index}",
        ArticulationType.REVOLUTE,
        parent=bell_part,
        child=clapper,
        origin=Origin(xyz=(0.0, 0.0, -0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.4, lower=-0.72, upper=0.72),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gothic_belfry_tower")

    stone = _mat(model, "weathered_limestone", (0.55, 0.56, 0.53, 1.0))
    dark_stone = _mat(model, "shadowed_recess", (0.045, 0.047, 0.050, 1.0))
    slate = _mat(model, "pale_slate", (0.34, 0.38, 0.41, 1.0))
    bronze = _mat(model, "aged_bell_bronze", (0.60, 0.43, 0.20, 1.0))
    timber = _mat(model, "dark_oak_timber", (0.28, 0.16, 0.08, 1.0))
    iron = _mat(model, "blackened_iron", (0.08, 0.075, 0.070, 1.0))

    tower = model.part("tower")

    # Connected masonry mass: plinth, square tower, open belfry piers, cornice.
    tower.visual(Box((4.15, 4.15, 0.34)), origin=Origin(xyz=(0.0, 0.0, 0.17)), material=stone, name="plinth")
    tower.visual(Box((3.45, 3.45, 2.52)), origin=Origin(xyz=(0.0, 0.0, 1.58)), material=stone, name="lower_tower")
    tower.visual(Box((3.68, 3.68, 0.28)), origin=Origin(xyz=(0.0, 0.0, 2.92)), material=stone, name="belfry_sill")
    for x in (-1.42, 1.42):
        for y in (-1.42, 1.42):
            tower.visual(
                Box((0.64, 0.64, 2.35)),
                origin=Origin(xyz=(x, y, 4.08)),
                material=stone,
                name=f"corner_pier_{x:+.0f}_{y:+.0f}",
            )
    tower.visual(Box((3.68, 0.48, 0.38)), origin=Origin(xyz=(0.0, -1.60, 5.30)), material=stone, name="front_cornice")
    tower.visual(Box((3.68, 0.48, 0.38)), origin=Origin(xyz=(0.0, 1.60, 5.30)), material=stone, name="rear_cornice")
    tower.visual(Box((0.48, 3.68, 0.38)), origin=Origin(xyz=(-1.60, 0.0, 5.30)), material=stone, name="west_cornice")
    tower.visual(Box((0.48, 3.68, 0.38)), origin=Origin(xyz=(1.60, 0.0, 5.30)), material=stone, name="east_cornice")
    tower.visual(Box((3.05, 3.05, 0.20)), origin=Origin(xyz=(0.0, 0.0, 5.48)), material=stone, name="drum_base_slab")

    arch_mesh = mesh_from_geometry(_lancet_frame_mesh(), "lancet_arch_frame")
    for side in ("front", "back", "east", "west"):
        _add_side_opening(tower, side=side, arch_mesh=arch_mesh, stone=stone, shadow=dark_stone, timber=timber)

    # Chamber timber frame and bearing blocks for two separate bell axles.
    for y, name in ((-1.00, "front_crossbeam"), (1.00, "rear_crossbeam")):
        tower.visual(Box((2.95, 0.24, 0.24)), origin=Origin(xyz=(0.0, y, 4.72)), material=timber, name=name)
    for i, x in enumerate((-0.62, 0.62)):
        for y, suffix in ((-0.79, "neg_y"), (0.79, "pos_y")):
            tower.visual(
                Box((0.38, 0.22, 0.34)),
                origin=Origin(xyz=(x, y, 4.72)),
                material=iron,
                name=f"bearing_{suffix}_{i}",
            )

    # Octagonal ribbed drum and pointed octagonal spire.
    tower.visual(
        mesh_from_geometry(CylinderGeometry(radius=1.34, height=1.10, radial_segments=8), "octagonal_drum"),
        origin=Origin(xyz=(0.0, 0.0, 5.95), rpy=(0.0, 0.0, math.pi / 8.0)),
        material=stone,
        name="octagonal_drum",
    )
    for i in range(8):
        a = math.tau * i / 8.0
        tower.visual(
            Box((0.16, 0.24, 1.22)),
            origin=Origin(xyz=(1.18 * math.cos(a), 1.18 * math.sin(a), 5.95), rpy=(0.0, 0.0, a)),
            material=stone,
            name=f"drum_rib_{i}",
        )
    tower.visual(
        mesh_from_geometry(ConeGeometry(radius=1.24, height=2.75, radial_segments=8), "octagonal_spire"),
        origin=Origin(xyz=(0.0, 0.0, 7.84), rpy=(0.0, 0.0, math.pi / 8.0)),
        material=slate,
        name="pointed_spire",
    )
    tower.visual(
        mesh_from_geometry(_octagonal_spire_ribs(), "spire_ribs"),
        material=stone,
        name="spire_ribs",
    )

    _add_bell_assembly(model, tower, 0, -0.62, bronze=bronze, dark_metal=iron, timber=timber)
    _add_bell_assembly(model, tower, 1, 0.62, bronze=bronze, dark_metal=iron, timber=timber)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell_0 = object_model.get_part("bell_0")
    bell_1 = object_model.get_part("bell_1")
    clapper_0 = object_model.get_part("clapper_0")
    clapper_1 = object_model.get_part("clapper_1")
    bell_joint_0 = object_model.get_articulation("tower_to_bell_0")
    bell_joint_1 = object_model.get_articulation("tower_to_bell_1")
    clap_joint_0 = object_model.get_articulation("bell_0_to_clapper_0")
    clap_joint_1 = object_model.get_articulation("bell_1_to_clapper_1")

    ctx.check(
        "two separate bell axles",
        bell_joint_0 is not None and bell_joint_1 is not None and bell_joint_0 is not bell_joint_1,
        details="Each bell should hang from its own revolute axle.",
    )
    ctx.check(
        "two secondary clapper pins",
        clap_joint_0 is not None and clap_joint_1 is not None,
        details="Each bell should have an independently articulated clapper.",
    )

    for idx, (bell, clapper) in enumerate(((bell_0, clapper_0), (bell_1, clapper_1))):
        ctx.allow_overlap(
            bell,
            clapper,
            elem_a="clapper_hanger",
            elem_b="clapper_pin",
            reason="The clapper pin is intentionally captured inside the bell hanger bushing.",
        )
        ctx.expect_gap(
            bell,
            clapper,
            axis="z",
            max_penetration=0.08,
            positive_elem="clapper_hanger",
            negative_elem="clapper_pin",
            name=f"clapper_{idx} pin captured by hanger",
        )
        ctx.expect_overlap(
            bell,
            clapper,
            axes="y",
            min_overlap=0.18,
            elem_a="clapper_hanger",
            elem_b="clapper_pin",
            name=f"clapper_{idx} pin spans hanger bushing",
        )

    for idx, bell in enumerate((bell_0, bell_1)):
        ctx.expect_gap(
            tower,
            bell,
            axis="y",
            min_gap=0.0,
            max_gap=0.055,
            positive_elem=f"bearing_pos_y_{idx}",
            negative_elem="axle_pin",
            name=f"bell_{idx} axle seats near positive bearing",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="y",
            min_gap=0.0,
            max_gap=0.055,
            positive_elem="axle_pin",
            negative_elem=f"bearing_neg_y_{idx}",
            name=f"bell_{idx} axle seats near negative bearing",
        )
        ctx.expect_gap(
            bell,
            tower,
            axis="z",
            min_gap=0.10,
            positive_elem="bell_shell",
            negative_elem="belfry_sill",
            name=f"bell_{idx} clears belfry sill",
        )

    for idx, (bell, clapper) in enumerate(((bell_0, clapper_0), (bell_1, clapper_1))):
        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            margin=0.02,
            outer_elem="bell_shell",
            name=f"clapper_{idx} hangs inside bell mouth footprint",
        )
        ctx.expect_overlap(
            clapper,
            bell,
            axes="z",
            min_overlap=0.25,
            elem_b="bell_shell",
            name=f"clapper_{idx} is vertically inside bell shell",
        )

    rest_aabb = ctx.part_world_aabb(bell_0)
    with ctx.pose({bell_joint_0: 0.45}):
        swung_aabb = ctx.part_world_aabb(bell_0)
    if rest_aabb is not None and swung_aabb is not None:
        rest_center_x = 0.5 * (rest_aabb[0][0] + rest_aabb[1][0])
        swung_center_x = 0.5 * (swung_aabb[0][0] + swung_aabb[1][0])
        ctx.check(
            "bell swing changes hanging angle",
            abs(swung_center_x - rest_center_x) > 0.10,
            details=f"rest_center_x={rest_center_x:.3f}, swung_center_x={swung_center_x:.3f}",
        )
    else:
        ctx.fail("bell swing changes hanging angle", "Could not measure bell AABBs.")

    rest_clapper = ctx.part_world_aabb(clapper_0)
    with ctx.pose({clap_joint_0: 0.55}):
        swung_clapper = ctx.part_world_aabb(clapper_0)
    if rest_clapper is not None and swung_clapper is not None:
        rest_center_x = 0.5 * (rest_clapper[0][0] + rest_clapper[1][0])
        swung_center_x = 0.5 * (swung_clapper[0][0] + swung_clapper[1][0])
        ctx.check(
            "clapper swings on secondary pin",
            abs(swung_center_x - rest_center_x) > 0.08,
            details=f"rest_center_x={rest_center_x:.3f}, swung_center_x={swung_center_x:.3f}",
        )
    else:
        ctx.fail("clapper swings on secondary pin", "Could not measure clapper AABBs.")

    return ctx.report()


object_model = build_object_model()
