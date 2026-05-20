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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _parabolic_reflector_mesh(
    *,
    diameter: float = 1.20,
    focal_length: float = 0.45,
    vertex_y: float = 0.060,
    wall_thickness: float = 0.030,
    profile_samples: int = 18,
) -> MeshGeometry:
    """Thin parabolic dish shell with its boresight along local +Y."""
    radius = diameter * 0.5
    inner_profile: list[tuple[float, float]] = []
    outer_profile: list[tuple[float, float]] = []
    for i in range(profile_samples + 1):
        r = radius * i / profile_samples
        y = vertex_y + (r * r) / (4.0 * focal_length)
        inner_profile.append((r, y))
        outer_profile.append((r, y - wall_thickness))

    dish = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    # Lathe uses local +Z as the revolution axis; rotate it so +Z becomes +Y.
    dish.rotate_x(-math.pi / 2.0)
    return dish


def _ring_about_y(radius: float, tube: float) -> MeshGeometry:
    ring = TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=96)
    ring.rotate_x(-math.pi / 2.0)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_dish_pedestal_mount")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.32, 0.34, 0.36, 1.0))
    concrete = model.material("matte_concrete", rgba=(0.43, 0.42, 0.39, 1.0))
    reflector_white = model.material("warm_white_reflector", rgba=(0.86, 0.86, 0.80, 1.0))
    feed_white = model.material("feed_radome_white", rgba=(0.94, 0.94, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=concrete,
        name="round_footing",
    )
    base.visual(
        Cylinder(radius=0.28, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.1175)),
        material=dark_steel,
        name="fixed_turntable",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.27, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=painted_steel,
        name="rotating_turntable",
    )
    yaw_stage.visual(
        Cylinder(radius=0.085, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=painted_steel,
        name="pedestal_column",
    )
    yaw_stage.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        material=dark_steel,
        name="top_collar",
    )
    yaw_stage.visual(
        Box((0.98, 0.22, 0.09)),
        origin=Origin(xyz=(0.0, -0.055, 0.835)),
        material=painted_steel,
        name="yoke_bridge",
    )
    yaw_stage.visual(
        Box((0.085, 0.22, 0.52)),
        origin=Origin(xyz=(-0.43, -0.055, 1.08)),
        material=painted_steel,
        name="yoke_cheek_0",
    )
    yaw_stage.visual(
        Cylinder(radius=0.080, length=0.018),
        origin=Origin(xyz=(-0.43, 0.064, 1.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_ring_0",
    )
    yaw_stage.visual(
        Box((0.085, 0.22, 0.52)),
        origin=Origin(xyz=(0.43, -0.055, 1.08)),
        material=painted_steel,
        name="yoke_cheek_1",
    )
    yaw_stage.visual(
        Cylinder(radius=0.080, length=0.018),
        origin=Origin(xyz=(0.43, 0.064, 1.08), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_ring_1",
    )

    dish = model.part("reflector_frame")
    dish_radius = 0.60
    dish_vertex_y = 0.28
    focal_y = dish_vertex_y + 0.45
    rim_y = dish_vertex_y + (dish_radius * dish_radius) / (4.0 * 0.45)

    dish.visual(
        mesh_from_geometry(_parabolic_reflector_mesh(vertex_y=dish_vertex_y), "parabolic_reflector"),
        material=reflector_white,
        name="parabolic_reflector",
    )
    dish.visual(
        mesh_from_geometry(_ring_about_y(dish_radius, 0.014), "rolled_rim"),
        origin=Origin(xyz=(0.0, rim_y, 0.0)),
        material=dark_steel,
        name="rolled_rim",
    )
    dish.visual(
        Cylinder(radius=0.043, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_axle",
    )
    dish.visual(
        Cylinder(radius=0.115, length=0.090),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_hub",
    )
    dish.visual(
        mesh_from_geometry(_ring_about_y(0.39, 0.011), "rear_support_ring"),
        origin=Origin(xyz=(0.0, 0.220, 0.0)),
        material=dark_steel,
        name="rear_support_ring",
    )

    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        r = 0.39
        end = (r * math.cos(angle), 0.220, r * math.sin(angle))
        spoke = tube_from_spline_points(
            [(0.0, 0.040, 0.0), (end[0] * 0.55, 0.145, end[2] * 0.55), end],
            radius=0.010,
            samples_per_segment=8,
            radial_segments=14,
        )
        dish.visual(
            mesh_from_geometry(spoke, f"rear_spoke_{idx}"),
            material=dark_steel,
            name=f"rear_spoke_{idx}",
        )
        shell_y = dish_vertex_y + (r * r) / (4.0 * 0.45) - 0.030
        standoff = tube_from_spline_points(
            [end, (end[0], (end[1] + shell_y) * 0.5, end[2]), (end[0], shell_y + 0.010, end[2])],
            radius=0.009,
            samples_per_segment=6,
            radial_segments=12,
        )
        dish.visual(
            mesh_from_geometry(standoff, f"shell_standoff_{idx}"),
            material=dark_steel,
            name=f"shell_standoff_{idx}",
        )

    feed_arm = tube_from_spline_points(
        [
            (0.0, rim_y - 0.010, -0.58),
            (0.0, 0.365, -0.32),
            (0.0, focal_y - 0.035, -0.025),
            (0.0, focal_y, 0.0),
        ],
        radius=0.014,
        samples_per_segment=14,
        radial_segments=16,
    )
    dish.visual(
        mesh_from_geometry(feed_arm, "feed_arm"),
        material=dark_steel,
        name="feed_arm",
    )
    for idx, sx in enumerate((-0.34, 0.34)):
        brace = tube_from_spline_points(
            [
                (sx, rim_y - 0.010, -0.39),
                (sx * 0.45, 0.385, -0.18),
                (0.0, focal_y - 0.020, 0.0),
            ],
            radius=0.007,
            samples_per_segment=10,
            radial_segments=12,
        )
        dish.visual(
            mesh_from_geometry(brace, f"feed_brace_{idx}"),
            material=dark_steel,
            name=f"feed_brace_{idx}",
        )
    dish.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, focal_y + 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=feed_white,
        name="feed_horn",
    )
    dish.visual(
        Cylinder(radius=0.026, length=0.070),
        origin=Origin(xyz=(0.0, focal_y - 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="feed_neck",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_reflector",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yaw_stage = object_model.get_part("yaw_stage")
    dish = object_model.get_part("reflector_frame")
    yaw = object_model.get_articulation("base_to_yaw")
    elevation = object_model.get_articulation("yoke_to_reflector")

    for cheek_name in ("yoke_cheek_0", "yoke_cheek_1"):
        ctx.allow_overlap(
            yaw_stage,
            dish,
            elem_a=cheek_name,
            elem_b="trunnion_axle",
            reason="The elevation shaft is intentionally captured through the simplified solid yoke cheek.",
        )
        ctx.expect_overlap(
            yaw_stage,
            dish,
            axes="x",
            elem_a=cheek_name,
            elem_b="trunnion_axle",
            min_overlap=0.030,
            name=f"{cheek_name} captures elevation axle",
        )
        ctx.expect_within(
            dish,
            yaw_stage,
            axes="yz",
            inner_elem="trunnion_axle",
            outer_elem=cheek_name,
            margin=0.002,
            name=f"axle centered in {cheek_name}",
        )

    ctx.expect_gap(
        yaw_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.003,
        positive_elem="rotating_turntable",
        negative_elem="fixed_turntable",
        name="yaw turntable seated on base",
    )
    ctx.expect_overlap(
        dish,
        yaw_stage,
        axes="x",
        elem_a="trunnion_axle",
        elem_b="yoke_bridge",
        min_overlap=0.40,
        name="trunnion spans between yoke sides",
    )

    rest_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
    with ctx.pose({elevation: 0.85, yaw: 0.7}):
        raised_aabb = ctx.part_element_world_aabb(dish, elem="feed_horn")
        ctx.expect_overlap(
            yaw_stage,
            dish,
            axes="x",
            elem_a="yoke_cheek_0",
            elem_b="trunnion_axle",
            min_overlap=0.030,
            name="raised dish remains in yoke cheek",
        )
    ctx.check(
        "elevation raises feed horn",
        rest_aabb is not None
        and raised_aabb is not None
        and (raised_aabb[0][2] + raised_aabb[1][2]) > (rest_aabb[0][2] + rest_aabb[1][2]) + 0.18,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
