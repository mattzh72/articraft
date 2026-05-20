from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _pyramid_roof_geometry(half_width: float, height: float, z_base: float) -> MeshGeometry:
    geom = MeshGeometry()
    v0 = geom.add_vertex(-half_width, -half_width, z_base)
    v1 = geom.add_vertex(half_width, -half_width, z_base)
    v2 = geom.add_vertex(half_width, half_width, z_base)
    v3 = geom.add_vertex(-half_width, half_width, z_base)
    apex = geom.add_vertex(0.0, 0.0, z_base + height)
    geom.add_face(v0, v1, apex)
    geom.add_face(v1, v2, apex)
    geom.add_face(v2, v3, apex)
    geom.add_face(v3, v0, apex)
    geom.add_face(v0, v2, v1)
    geom.add_face(v0, v3, v2)
    return geom


def _bell_shell_geometry() -> MeshGeometry:
    bell = LatheGeometry.from_shell_profiles(
        [
            (0.155, -0.250),
            (0.220, -0.340),
            (0.270, -0.520),
            (0.320, -0.740),
            (0.405, -1.020),
            (0.465, -1.155),
        ],
        [
            (0.070, -0.300),
            (0.155, -0.385),
            (0.215, -0.565),
            (0.270, -0.790),
            (0.350, -1.025),
            (0.402, -1.135),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bell.merge(TorusGeometry(radius=0.440, tube=0.025, radial_segments=14, tubular_segments=72).translate(0.0, 0.0, -1.145))
    bell.merge(TorusGeometry(radius=0.300, tube=0.010, radial_segments=10, tubular_segments=64).translate(0.0, 0.0, -0.620))
    bell.merge(TorusGeometry(radius=0.365, tube=0.012, radial_segments=10, tubular_segments=64).translate(0.0, 0.0, -0.900))
    return bell


def _pulley_sheave_geometry() -> MeshGeometry:
    sheave = LatheGeometry.from_shell_profiles(
        [
            (0.255, -0.042),
            (0.282, -0.032),
            (0.226, 0.000),
            (0.282, 0.032),
            (0.255, 0.042),
        ],
        [
            (0.052, -0.042),
            (0.052, -0.021),
            (0.052, 0.000),
            (0.052, 0.021),
            (0.052, 0.042),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    sheave.merge(
        LatheGeometry.from_shell_profiles(
            [(0.110, -0.055), (0.120, -0.038), (0.120, 0.038), (0.110, 0.055)],
            [(0.052, -0.055), (0.052, -0.018), (0.052, 0.018), (0.052, 0.055)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        )
    )
    # The lathe axis is local Z; rotate it so the completed pulley spins about local X.
    return sheave.rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_bell_tower")

    stone = model.material("weathered_stone", rgba=(0.58, 0.56, 0.50, 1.0))
    dark_stone = model.material("dark_mortar", rgba=(0.36, 0.35, 0.32, 1.0))
    roof_slate = model.material("slate_roof", rgba=(0.18, 0.20, 0.23, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.72, 0.50, 0.22, 1.0))
    dark_metal = model.material("dark_iron", rgba=(0.08, 0.08, 0.075, 1.0))
    oak = model.material("aged_oak", rgba=(0.34, 0.19, 0.09, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((2.78, 2.78, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_stone,
        name="stepped_plinth",
    )
    tower.visual(
        Box((2.28, 2.28, 4.42)),
        origin=Origin(xyz=(0.0, 0.0, 2.45)),
        material=stone,
        name="square_masonry_shaft",
    )
    tower.visual(
        Box((2.46, 0.30, 0.22)),
        origin=Origin(xyz=(0.0, -1.08, 4.70)),
        material=dark_stone,
        name="front_belfry_sill",
    )
    tower.visual(
        Box((2.46, 0.30, 0.22)),
        origin=Origin(xyz=(0.0, 1.08, 4.70)),
        material=dark_stone,
        name="rear_belfry_sill",
    )
    tower.visual(
        Box((0.30, 2.46, 0.22)),
        origin=Origin(xyz=(-1.08, 0.0, 4.70)),
        material=dark_stone,
        name="side_sill_0",
    )
    tower.visual(
        Box((0.30, 2.46, 0.22)),
        origin=Origin(xyz=(1.08, 0.0, 4.70)),
        material=dark_stone,
        name="side_sill_1",
    )

    for ix, x in enumerate((-0.91, 0.91)):
        for iy, y in enumerate((-0.91, 0.91)):
            tower.visual(
                Box((0.46, 0.46, 2.06)),
                origin=Origin(xyz=(x, y, 5.72)),
                material=stone,
                name=f"belfry_pier_{ix}_{iy}",
            )

    tower.visual(
        Box((2.52, 0.34, 0.32)),
        origin=Origin(xyz=(0.0, -1.09, 6.82)),
        material=dark_stone,
        name="front_cornice",
    )
    tower.visual(
        Box((2.52, 0.34, 0.32)),
        origin=Origin(xyz=(0.0, 1.09, 6.82)),
        material=dark_stone,
        name="rear_cornice",
    )
    tower.visual(
        Box((0.34, 2.52, 0.32)),
        origin=Origin(xyz=(-1.09, 0.0, 6.82)),
        material=dark_stone,
        name="side_cornice_0",
    )
    tower.visual(
        Box((0.34, 2.52, 0.32)),
        origin=Origin(xyz=(1.09, 0.0, 6.82)),
        material=dark_stone,
        name="side_cornice_1",
    )
    tower.visual(
        mesh_from_geometry(_pyramid_roof_geometry(1.36, 0.90, 6.98), "slate_pyramid_roof"),
        material=roof_slate,
        name="pyramid_roof",
    )

    for index, z in enumerate((0.78, 1.32, 1.86, 2.40, 2.94, 3.48, 4.02)):
        tower.visual(
            Box((2.34, 0.040, 0.045)),
            origin=Origin(xyz=(0.0, -1.160, z)),
            material=dark_stone,
            name=f"front_course_{index}",
        )
        tower.visual(
            Box((2.34, 0.040, 0.045)),
            origin=Origin(xyz=(0.0, 1.160, z)),
            material=dark_stone,
            name=f"rear_course_{index}",
        )
        tower.visual(
            Box((0.040, 2.34, 0.045)),
            origin=Origin(xyz=(-1.160, 0.0, z)),
            material=dark_stone,
            name=f"side_course_0_{index}",
        )
        tower.visual(
            Box((0.040, 2.34, 0.045)),
            origin=Origin(xyz=(1.160, 0.0, z)),
            material=dark_stone,
            name=f"side_course_1_{index}",
        )

    # Bell bearings in the belfry and a separate base-level rope-guide pulley bracket.
    tower.visual(
        Box((0.20, 1.42, 0.18)),
        origin=Origin(xyz=(-0.76, 0.0, 5.70)),
        material=dark_metal,
        name="bearing_support_0",
    )
    tower.visual(
        Box((0.20, 1.42, 0.18)),
        origin=Origin(xyz=(0.76, 0.0, 5.70)),
        material=dark_metal,
        name="bearing_support_1",
    )
    tower.visual(
        Box((0.28, 0.38, 0.18)),
        origin=Origin(xyz=(-0.76, 0.0, 5.79)),
        material=dark_metal,
        name="bell_bearing_0",
    )
    tower.visual(
        Box((0.28, 0.38, 0.18)),
        origin=Origin(xyz=(0.76, 0.0, 5.79)),
        material=dark_metal,
        name="bell_bearing_1",
    )
    tower.visual(
        Box((0.44, 0.060, 0.86)),
        origin=Origin(xyz=(0.0, -1.205, 0.86)),
        material=dark_metal,
        name="pulley_backplate",
    )
    for x, suffix in ((-0.090, "0"), (0.090, "1")):
        tower.visual(
            Box((0.030, 0.66, 0.62)),
            origin=Origin(xyz=(x, -1.515, 0.86)),
            material=dark_metal,
            name=f"pulley_cheek_{suffix}",
        )
    tower.visual(
        Cylinder(radius=0.026, length=0.25),
        origin=Origin(xyz=(0.0, -1.515, 0.86), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pulley_axle_pin",
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.055, length=1.42),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_trunnion",
    )
    bell.visual(
        Box((1.24, 0.25, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=oak,
        name="wooden_headstock",
    )
    bell.visual(
        Box((0.040, 0.075, 0.32)),
        origin=Origin(xyz=(-0.15, 0.0, -0.225)),
        material=dark_metal,
        name="crown_strap_0",
    )
    bell.visual(
        Box((0.040, 0.075, 0.32)),
        origin=Origin(xyz=(0.15, 0.0, -0.225)),
        material=dark_metal,
        name="crown_strap_1",
    )
    bell.visual(
        Cylinder(radius=0.190, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=aged_bronze,
        name="bell_crown",
    )
    bell.visual(
        mesh_from_geometry(_bell_shell_geometry(), "hollow_bronze_bell"),
        material=aged_bronze,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.014, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, -0.660)),
        material=dark_metal,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, -1.010)),
        material=dark_metal,
        name="clapper_ball",
    )

    guide_wheel = model.part("guide_wheel")
    guide_wheel.visual(
        mesh_from_geometry(_pulley_sheave_geometry(), "grooved_pulley_sheave"),
        material=dark_metal,
        name="pulley_sheave",
    )

    model.articulation(
        "bell_pivot",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 5.94)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "pulley_axle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=guide_wheel,
        origin=Origin(xyz=(0.0, -1.515, 0.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    bell = object_model.get_part("bell")
    guide_wheel = object_model.get_part("guide_wheel")
    bell_pivot = object_model.get_articulation("bell_pivot")
    pulley_axle = object_model.get_articulation("pulley_axle")

    ctx.check(
        "independent bell and pulley joints",
        bell_pivot.child == "bell"
        and pulley_axle.child == "guide_wheel"
        and bell_pivot.parent == "tower"
        and pulley_axle.parent == "tower",
        details="Bell and rope-guide pulley must be separate children of the masonry tower.",
    )
    ctx.check(
        "expected joint types",
        bell_pivot.articulation_type == ArticulationType.REVOLUTE
        and pulley_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"bell={bell_pivot.articulation_type}, pulley={pulley_axle.articulation_type}",
    )
    ctx.expect_within(
        bell,
        tower,
        axes="x",
        inner_elem="bell_shell",
        outer_elem="front_cornice",
        margin=0.001,
        name="bell shell fits within square belfry width",
    )
    ctx.expect_gap(
        bell,
        tower,
        axis="z",
        positive_elem="bell_shell",
        negative_elem="square_masonry_shaft",
        min_gap=0.08,
        name="bell clears lower masonry shaft",
    )
    ctx.expect_within(
        guide_wheel,
        tower,
        axes="x",
        inner_elem="pulley_sheave",
        outer_elem="pulley_backplate",
        margin=0.02,
        name="pulley wheel is centered in base bracket",
    )

    rest_aabb = ctx.part_world_aabb(bell)
    with ctx.pose({bell_pivot: 0.40}):
        swing_aabb = ctx.part_world_aabb(bell)
    if rest_aabb is not None and swing_aabb is not None:
        rest_center_y = (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
        swing_center_y = (swing_aabb[0][1] + swing_aabb[1][1]) * 0.5
        ctx.check(
            "bell swings about headstock pivot",
            swing_center_y > rest_center_y + 0.16,
            details=f"rest_y={rest_center_y:.3f}, swing_y={swing_center_y:.3f}",
        )
    else:
        ctx.fail("bell swings about headstock pivot", "AABB query returned None.")

    return ctx.report()


object_model = build_object_model()
