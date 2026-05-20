from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _curved_drum_wall(
    *,
    name: str,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_deg: float,
    end_deg: float,
    segments: int = 36,
):
    """Build one thin, closed cylindrical-shell segment for the fixed drum glass."""
    geom = MeshGeometry()
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    angles = [start + (end - start) * i / segments for i in range(segments + 1)]

    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for angle in angles:
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_min))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z_max))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_min))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z_max))

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(segments):
        quad(outer_bottom[i], outer_bottom[i + 1], outer_top[i + 1], outer_top[i])
        quad(inner_bottom[i + 1], inner_bottom[i], inner_top[i], inner_top[i + 1])
        quad(outer_top[i], outer_top[i + 1], inner_top[i + 1], inner_top[i])
        quad(outer_bottom[i + 1], outer_bottom[i], inner_bottom[i], inner_bottom[i + 1])

    quad(inner_bottom[0], outer_bottom[0], outer_top[0], inner_top[0])
    quad(outer_bottom[-1], inner_bottom[-1], inner_top[-1], outer_top[-1])

    return mesh_from_geometry(geom, name)


def _rotated_xy(center_radius: float, z: float, yaw: float) -> tuple[float, float, float]:
    return (center_radius * math.cos(yaw), center_radius * math.sin(yaw), z)


def _add_radial_box(part, size, *, radius: float, z: float, yaw: float, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_rotated_xy(radius, z, yaw), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_speed_four_wing_revolving_door")

    dark_threshold = Material("dark_anodized_threshold", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_metal = Material("satin_aluminum", rgba=(0.63, 0.64, 0.62, 1.0))
    dark_metal = Material("dark_motor_housing", rgba=(0.16, 0.17, 0.18, 1.0))
    blue_glass = Material("slightly_blue_clear_glass", rgba=(0.55, 0.78, 0.96, 0.34))
    black_rubber = Material("black_safety_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_threshold,
        name="base_plate",
    )
    drum.visual(
        Cylinder(radius=1.23, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.32)),
        material=satin_metal,
        name="crown_canopy",
    )
    drum.visual(
        Cylinder(radius=0.33, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 2.52)),
        material=dark_metal,
        name="motor_housing",
    )
    drum.visual(
        Box((0.36, 0.11, 0.07)),
        origin=Origin(xyz=(0.0, 0.32, 2.54)),
        material=dark_metal,
        name="motor_service_box",
    )
    drum.visual(
        _curved_drum_wall(
            name="drum_glass_0",
            inner_radius=1.085,
            outer_radius=1.125,
            z_min=0.055,
            z_max=2.265,
            start_deg=32.0,
            end_deg=148.0,
        ),
        material=blue_glass,
        name="drum_glass_0",
    )
    drum.visual(
        _curved_drum_wall(
            name="drum_glass_1",
            inner_radius=1.085,
            outer_radius=1.125,
            z_min=0.055,
            z_max=2.265,
            start_deg=212.0,
            end_deg=328.0,
        ),
        material=blue_glass,
        name="drum_glass_1",
    )
    for yaw, label in ((math.pi / 2.0, "side_post_0"), (-math.pi / 2.0, "side_post_1")):
        _add_radial_box(
            drum,
            (0.05, 0.08, 2.23),
            radius=1.105,
            z=1.16,
            yaw=yaw,
            material=satin_metal,
            name=label,
        )

    wings = model.part("wing_assembly")
    wings.visual(
        Cylinder(radius=0.055, length=2.16),
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        material=satin_metal,
        name="central_post",
    )
    wings.visual(
        Cylinder(radius=0.14, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=satin_metal,
        name="bottom_hub",
    )
    wings.visual(
        Cylinder(radius=0.13, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 2.115)),
        material=satin_metal,
        name="top_hub",
    )

    panel_length = 0.91
    panel_height = 2.02
    panel_thickness = 0.026
    panel_center_radius = 0.045 + panel_length / 2.0
    panel_center_z = 1.10
    outer_stile_radius = 0.045 + panel_length - 0.018
    rail_radius = panel_center_radius
    for index in range(4):
        yaw = index * math.pi / 2.0
        _add_radial_box(
            wings,
            (panel_length, panel_thickness, panel_height),
            radius=panel_center_radius,
            z=panel_center_z,
            yaw=yaw,
            material=blue_glass,
            name=f"wing_{index}_glass",
        )
        _add_radial_box(
            wings,
            (0.036, 0.052, panel_height + 0.05),
            radius=outer_stile_radius,
            z=panel_center_z,
            yaw=yaw,
            material=satin_metal,
            name=f"wing_{index}_outer_stile",
        )
        _add_radial_box(
            wings,
            (panel_length + 0.03, 0.046, 0.045),
            radius=rail_radius,
            z=0.105,
            yaw=yaw,
            material=satin_metal,
            name=f"wing_{index}_bottom_rail",
        )
        _add_radial_box(
            wings,
            (panel_length + 0.03, 0.044, 0.040),
            radius=rail_radius,
            z=2.10,
            yaw=yaw,
            material=satin_metal,
            name=f"wing_{index}_top_rail",
        )
        _add_radial_box(
            wings,
            (0.07, 0.03, panel_height),
            radius=0.085,
            z=panel_center_z,
            yaw=yaw,
            material=satin_metal,
            name=f"wing_{index}_root_clamp",
        )
        _add_radial_box(
            wings,
            (0.018, 0.07, panel_height),
            radius=outer_stile_radius + 0.025,
            z=panel_center_z,
            yaw=yaw,
            material=black_rubber,
            name=f"wing_{index}_sweep_strip",
        )

    model.articulation(
        "drum_to_wing_assembly",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wings,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35),
        motion_properties=MotionProperties(damping=0.4, friction=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    wings = object_model.get_part("wing_assembly")
    rotation = object_model.get_articulation("drum_to_wing_assembly")

    ctx.expect_within(
        wings,
        drum,
        axes="xy",
        inner_elem="wing_0_outer_stile",
        outer_elem="base_plate",
        margin=0.0,
        name="rotating wing stays inside the circular drum footprint",
    )
    ctx.expect_gap(
        wings,
        drum,
        axis="z",
        positive_elem="bottom_hub",
        negative_elem="base_plate",
        min_gap=0.0,
        max_gap=0.025,
        name="bottom hub clears the threshold plate",
    )
    ctx.expect_gap(
        drum,
        wings,
        axis="z",
        positive_elem="crown_canopy",
        negative_elem="top_hub",
        min_gap=0.0,
        max_gap=0.08,
        name="top hub clears the motor crown",
    )

    with ctx.pose({rotation: 0.0}):
        start_aabb = ctx.part_element_world_aabb(wings, elem="wing_0_outer_stile")
    with ctx.pose({rotation: math.pi / 2.0}):
        quarter_aabb = ctx.part_element_world_aabb(wings, elem="wing_0_outer_stile")

    def aabb_center_xy(aabb):
        lower, upper = aabb
        return ((lower[0] + upper[0]) * 0.5, (lower[1] + upper[1]) * 0.5)

    start_xy = aabb_center_xy(start_aabb) if start_aabb is not None else None
    quarter_xy = aabb_center_xy(quarter_aabb) if quarter_aabb is not None else None
    ctx.check(
        "continuous joint rotates a wing about the vertical post",
        start_xy is not None
        and quarter_xy is not None
        and start_xy[0] > 0.88
        and abs(start_xy[1]) < 0.08
        and quarter_xy[1] > 0.88
        and abs(quarter_xy[0]) < 0.08,
        details=f"start_xy={start_xy}, quarter_turn_xy={quarter_xy}",
    )

    return ctx.report()


object_model = build_object_model()
