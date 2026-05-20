from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    segments: int = 48,
    *,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    return [
        (
            center[0] + radius * math.cos(2.0 * math.pi * index / segments),
            center[1] + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _bearing_profile(
    *,
    rail_top: float,
    shaft_z: float,
    base_half_width: float = 0.120,
    pedestal_half_width: float = 0.083,
    base_height: float = 0.062,
    cap_radius: float = 0.083,
    arc_segments: int = 18,
) -> list[tuple[float, float]]:
    """Pillow-block side outline in local (world-y, world-z) coordinates."""

    base_bottom = rail_top - 0.002
    base_top = rail_top + base_height
    points: list[tuple[float, float]] = [
        (-base_half_width, base_bottom),
        (base_half_width, base_bottom),
        (base_half_width, base_top),
        (pedestal_half_width, base_top),
        (pedestal_half_width, shaft_z),
    ]
    for index in range(arc_segments + 1):
        angle = math.pi * index / arc_segments
        points.append(
            (
                cap_radius * math.cos(angle),
                shaft_z + cap_radius * math.sin(angle),
            )
        )
    points.extend(
        [
            (-pedestal_half_width, base_top),
            (-base_half_width, base_top),
        ]
    )
    return points


def _extruded_yz_mesh(
    outer_profile: list[tuple[float, float]],
    hole_profiles: list[list[tuple[float, float]]],
    *,
    thickness: float,
    x_center: float,
    name: str,
):
    """Build a mesh by extruding a YZ profile along world X."""

    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        thickness,
        center=True,
    )
    geom.vertices = [
        (x_center + local_z, local_x, local_y)
        for local_x, local_y, local_z in geom.vertices
    ]
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_rotary_shaft_assembly")

    rail_mat = model.material("blackened_rail", rgba=(0.035, 0.040, 0.045, 1.0))
    block_mat = model.material("painted_bearing_blocks", rgba=(0.18, 0.30, 0.42, 1.0))
    bearing_mat = model.material("bronze_bearing_liners", rgba=(0.72, 0.47, 0.20, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel_mat = model.material("dark_drive_collar", rgba=(0.10, 0.105, 0.11, 1.0))
    bolt_mat = model.material("dark_socket_heads", rgba=(0.015, 0.015, 0.016, 1.0))

    rail_top = 0.080
    shaft_z = 0.300
    bearing_xs = (-0.340, 0.340)
    bearing_thickness = 0.145
    shaft_radius = 0.025
    bearing_clearance_radius = 0.036

    rail = model.part("rail")
    rail.visual(
        Box((1.20, 0.160, rail_top)),
        origin=Origin(xyz=(0.0, 0.0, rail_top / 2.0)),
        material=rail_mat,
        name="rail_beam",
    )

    bearing_outline = _bearing_profile(rail_top=rail_top, shaft_z=shaft_z)
    bearing_hole = _circle_profile(bearing_clearance_radius, segments=56, center=(0.0, shaft_z))
    liner_outer = _circle_profile(0.060, segments=56, center=(0.0, shaft_z))
    liner_hole = _circle_profile(bearing_clearance_radius, segments=56, center=(0.0, shaft_z))

    bearing_block_names = ("bearing_block_0", "bearing_block_1")
    bearing_liner_names = (
        ("bearing_liner_0_inner", "bearing_liner_0_outer"),
        ("bearing_liner_1_inner", "bearing_liner_1_outer"),
    )

    for index, x_pos in enumerate(bearing_xs):
        rail.visual(
            _extruded_yz_mesh(
                bearing_outline,
                [bearing_hole],
                thickness=bearing_thickness,
                x_center=x_pos,
                name=bearing_block_names[index],
            ),
            material=block_mat,
            name=bearing_block_names[index],
        )

        for face_index, face_sign in enumerate((-1.0, 1.0)):
            rail.visual(
                _extruded_yz_mesh(
                    liner_outer,
                    [liner_hole],
                    thickness=0.014,
                    x_center=x_pos + face_sign * (bearing_thickness / 2.0 + 0.006),
                    name=bearing_liner_names[index][face_index],
                ),
                material=bearing_mat,
                name=bearing_liner_names[index][face_index],
            )

        for x_offset in (-0.042, 0.042):
            for y_offset in (-0.082, 0.082):
                rail.visual(
                    Cylinder(radius=0.012, length=0.010),
                    origin=Origin(xyz=(x_pos + x_offset, y_offset, rail_top + 0.065)),
                    material=bolt_mat,
                    name=f"mount_bolt_{index}_{x_offset:+.3f}_{y_offset:+.3f}",
                )

    shaft = model.part("shaft")
    cylinder_to_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    shaft.visual(
        Cylinder(radius=shaft_radius, length=1.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cylinder_to_x.rpy),
        material=steel_mat,
        name="main_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(xyz=(-0.475, 0.0, 0.0), rpy=cylinder_to_x.rpy),
        material=dark_steel_mat,
        name="drive_collar",
    )
    shaft.visual(
        Cylinder(radius=0.060, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cylinder_to_x.rpy),
        material=steel_mat,
        name="center_hub",
    )
    shaft.visual(
        Cylinder(radius=0.052, length=0.038),
        origin=Origin(xyz=(0.525, 0.0, 0.0), rpy=cylinder_to_x.rpy),
        material=steel_mat,
        name="end_flange",
    )

    for angle_index in range(6):
        angle = 2.0 * math.pi * angle_index / 6.0
        shaft.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(
                xyz=(
                    0.546,
                    0.034 * math.cos(angle),
                    0.034 * math.sin(angle),
                ),
                rpy=cylinder_to_x.rpy,
            ),
            material=bolt_mat,
            name=f"flange_bolt_{angle_index}",
        )

    shaft.visual(
        Box((0.085, 0.010, 0.012)),
        origin=Origin(xyz=(-0.475, 0.000, 0.047)),
        material=bolt_mat,
        name="collar_key",
    )

    model.articulation(
        "rail_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=rail,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, shaft_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("rail_to_shaft")

    bearing_interfaces = (
        "bearing_block_0",
        "bearing_liner_0_inner",
        "bearing_liner_0_outer",
        "bearing_block_1",
        "bearing_liner_1_inner",
        "bearing_liner_1_outer",
    )
    for bearing_elem in bearing_interfaces:
        ctx.allow_overlap(
            rail,
            shaft,
            elem_a=bearing_elem,
            elem_b="main_shaft",
            reason=(
                "The shaft is intentionally captured through the visible bearing bore; "
                "the mesh-backed bearing element is treated as a simplified support proxy."
            ),
        )
        ctx.expect_overlap(
            shaft,
            rail,
            axes="xyz",
            elem_a="main_shaft",
            elem_b=bearing_elem,
            min_overlap=0.010,
            name=f"main shaft remains engaged with {bearing_elem}",
        )

    ctx.check(
        "single continuous shaft joint",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and len(object_model.articulations) == 1,
        details=f"type={joint.articulation_type}, joint_count={len(object_model.articulations)}",
    )
    ctx.check(
        "shaft rotates about rail centerline",
        tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={joint.axis}",
    )
    ctx.expect_overlap(
        shaft,
        rail,
        axes="xyz",
        elem_a="main_shaft",
        elem_b="bearing_block_0",
        min_overlap=0.040,
        name="shaft runs through first bearing block silhouette",
    )
    ctx.expect_overlap(
        shaft,
        rail,
        axes="xyz",
        elem_a="main_shaft",
        elem_b="bearing_block_1",
        min_overlap=0.040,
        name="shaft runs through second bearing block silhouette",
    )
    ctx.expect_gap(
        shaft,
        rail,
        axis="z",
        positive_elem="main_shaft",
        negative_elem="rail_beam",
        min_gap=0.170,
        name="shaft centerline is raised above rail beam",
    )

    rest_pos = ctx.part_world_position(shaft)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(shaft)
        ctx.expect_overlap(
            shaft,
            rail,
            axes="xyz",
            elem_a="main_shaft",
            elem_b="bearing_block_0",
            min_overlap=0.040,
            name="rotated shaft remains captured by first bearing",
        )
        ctx.expect_overlap(
            shaft,
            rail,
            axes="xyz",
            elem_a="main_shaft",
            elem_b="bearing_block_1",
            min_overlap=0.040,
            name="rotated shaft remains captured by second bearing",
        )
    ctx.check(
        "revolute motion keeps shaft origin on supported axis",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(rest_pos, turned_pos)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
