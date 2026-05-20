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
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _annular_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    segments: int = 32,
) -> MeshGeometry:
    """Closed thick cylindrical-wall sector in the XY plane."""
    mesh = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(segments + 1):
        t = start_angle + (end_angle - start_angle) * i / segments
        ct = math.cos(t)
        st = math.sin(t)
        outer_bottom.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z_min))
        outer_top.append(mesh.add_vertex(outer_radius * ct, outer_radius * st, z_max))
        inner_bottom.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z_min))
        inner_top.append(mesh.add_vertex(inner_radius * ct, inner_radius * st, z_max))

    for i in range(segments):
        j = i + 1
        _add_quad(mesh, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(mesh, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(mesh, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(mesh, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])

    _add_quad(mesh, outer_bottom[0], outer_top[0], inner_top[0], inner_bottom[0])
    _add_quad(mesh, outer_top[-1], outer_bottom[-1], inner_bottom[-1], inner_top[-1])
    return mesh


def _radial_origin(radius: float, z: float, angle: float) -> Origin:
    return Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z), rpy=(0.0, 0.0, angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_three_wing_revolving_door")

    mirror_steel = Material("mirror_polished_stainless_steel", rgba=(0.82, 0.84, 0.82, 1.0))
    brushed_steel = Material("brushed_stainless_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    smoky_glass = Material("slightly_smoked_glass", rgba=(0.62, 0.82, 0.95, 0.34))
    dark_floor = Material("dark_recessed_floor_mat", rgba=(0.05, 0.055, 0.055, 1.0))

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.10, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=mirror_steel,
        name="floor_disk",
    )
    drum.visual(
        Cylinder(radius=0.93, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_floor,
        name="floor_mat",
    )
    drum.visual(
        Cylinder(radius=1.10, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 2.440)),
        material=mirror_steel,
        name="canopy_disk",
    )
    drum.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed_steel,
        name="lower_bearing",
    )
    drum.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 2.375)),
        material=brushed_steel,
        name="upper_bearing",
    )

    # Two mirrored cylindrical drum wall sectors leave broad front and rear openings.
    for idx, center_angle in enumerate((0.0, math.pi)):
        wall = _annular_sector(
            inner_radius=1.015,
            outer_radius=1.070,
            z_min=0.050,
            z_max=2.400,
            start_angle=center_angle - math.radians(58.0),
            end_angle=center_angle + math.radians(58.0),
            segments=40,
        )
        drum.visual(
            mesh_from_geometry(wall, f"curved_wall_{idx}"),
            material=mirror_steel,
            name=f"curved_wall_{idx}",
        )
        for end_idx, a in enumerate((center_angle - math.radians(58.0), center_angle + math.radians(58.0))):
            drum.visual(
                Cylinder(radius=0.035, length=2.350),
                origin=Origin(xyz=(1.045 * math.cos(a), 1.045 * math.sin(a), 1.225)),
                material=mirror_steel,
                name=f"jamb_post_{idx}_{end_idx}",
            )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=2.250),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=mirror_steel,
        name="center_post",
    )
    rotor.visual(
        Cylinder(radius=0.120, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=mirror_steel,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.120, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 2.310)),
        material=mirror_steel,
        name="upper_hub",
    )

    wing_height = 2.020
    wing_center_z = 1.235
    panel_center_radius = 0.495
    rail_center_radius = 0.500
    wing_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    for idx, angle in enumerate(wing_angles):
        rotor.visual(
            Box((0.760, 0.016, 1.930)),
            origin=_radial_origin(panel_center_radius, wing_center_z, angle),
            material=smoky_glass,
            name=f"glass_panel_{idx}",
        )
        rotor.visual(
            Box((0.855, 0.052, 0.052)),
            origin=_radial_origin(rail_center_radius, 0.245, angle),
            material=mirror_steel,
            name=f"bottom_rail_{idx}",
        )
        rotor.visual(
            Box((0.855, 0.052, 0.052)),
            origin=_radial_origin(rail_center_radius, 2.225, angle),
            material=mirror_steel,
            name=f"top_rail_{idx}",
        )
        rotor.visual(
            Box((0.110, 0.058, wing_height)),
            origin=_radial_origin(0.105, wing_center_z, angle),
            material=mirror_steel,
            name=f"inner_stile_{idx}",
        )
        rotor.visual(
            Box((0.060, 0.058, wing_height)),
            origin=_radial_origin(0.895, wing_center_z, angle),
            material=mirror_steel,
            name=f"outer_stile_{idx}",
        )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    turn = object_model.get_articulation("drum_to_rotor")

    ctx.check(
        "wing assembly has continuous vertical rotation",
        turn.articulation_type == ArticulationType.CONTINUOUS and tuple(turn.axis) == (0.0, 0.0, 1.0),
        details=f"type={turn.articulation_type}, axis={turn.axis}",
    )
    ctx.expect_contact(
        rotor,
        drum,
        elem_a="center_post",
        elem_b="lower_bearing",
        contact_tol=0.002,
        name="center post is carried by the lower bearing",
    )
    ctx.expect_gap(
        rotor,
        drum,
        axis="z",
        positive_elem="bottom_rail_0",
        negative_elem="floor_disk",
        min_gap=0.150,
        name="wing frames clear the circular threshold",
    )
    ctx.expect_gap(
        drum,
        rotor,
        axis="z",
        positive_elem="canopy_disk",
        negative_elem="top_rail_0",
        min_gap=0.120,
        name="wing frames clear the overhead canopy",
    )
    ctx.expect_within(
        rotor,
        drum,
        axes="xy",
        margin=0.010,
        name="three wing rotor fits inside the round drum footprint",
    )

    with ctx.pose({turn: math.radians(73.0)}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.010,
            name="rotor remains inside drum while turning",
        )

    return ctx.report()


object_model = build_object_model()
