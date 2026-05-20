from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _semicircle_profile(radius: float, *, y_offset: float = 0.0, segments: int = 64):
    """D-shaped profile: a straight chord along local X and an arc toward +Y."""
    return [
        (
            radius * math.cos(math.pi - math.pi * i / segments),
            y_offset + radius * math.sin(math.pi - math.pi * i / segments),
        )
        for i in range(segments + 1)
    ]


def _housing_shell_cq() -> cq.Workplane:
    """Shallow D-shaped metal tray with a flat hinge edge and a closed back plate."""
    outer_radius = 0.300
    wall = 0.030
    rim_height = 0.050
    back_thickness = 0.012

    outer = _semicircle_profile(outer_radius, segments=72)
    # The inner opening is offset up from the chord so the flat hinge side has a real rail.
    inner = _semicircle_profile(outer_radius - 1.45 * wall, y_offset=wall, segments=72)

    rim = cq.Workplane("XY").polyline(outer).close().extrude(rim_height)
    opening = (
        cq.Workplane("XY")
        .polyline(inner)
        .close()
        .extrude(rim_height + 0.010)
        .translate((0.0, 0.0, -0.005))
    )
    rim = rim.cut(opening)

    back = (
        cq.Workplane("XY")
        .polyline(outer)
        .close()
        .extrude(back_thickness)
        .translate((0.0, 0.0, rim_height - back_thickness))
    )
    return rim.union(back)


def _diffuser_shell_geometry(
    *,
    radius: float = 0.245,
    y_offset: float = 0.012,
    sag: float = 0.036,
    edge_z: float = -0.014,
    thickness: float = 0.004,
    radial_segments: int = 12,
    angular_segments: int = 56,
) -> MeshGeometry:
    """Closed frosted acrylic half-dome, local frame at the hinge pin."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    def lens_z(x: float, y: float) -> float:
        # Zero sag on both the straight chord and circular rim; deepest halfway
        # between them, which reads like a shallow pressed oyster diffuser.
        y_rel = max(0.0, y - y_offset)
        chord_to_arc = math.sqrt(max(radius * radius - x * x, 0.0))
        if chord_to_arc <= 1.0e-9:
            return edge_z
        f = max(0.0, min(1.0, y_rel / chord_to_arc))
        return edge_z - sag * (4.0 * f * (1.0 - f))

    top_center = len(vertices)
    vertices.append((0.0, y_offset, lens_z(0.0, y_offset) + thickness))
    bottom_center = len(vertices)
    vertices.append((0.0, y_offset, lens_z(0.0, y_offset)))

    top: list[list[int]] = []
    bottom: list[list[int]] = []
    for ir in range(1, radial_segments + 1):
        r = radius * ir / radial_segments
        top_row: list[int] = []
        bottom_row: list[int] = []
        for ia in range(angular_segments + 1):
            theta = math.pi - math.pi * ia / angular_segments
            x = r * math.cos(theta)
            y = y_offset + r * math.sin(theta)
            z = lens_z(x, y)
            bottom_row.append(len(vertices))
            vertices.append((x, y, z))
            top_row.append(len(vertices))
            vertices.append((x, y, z + thickness))
        bottom.append(bottom_row)
        top.append(top_row)

    # Outer top and lower frosted surfaces.
    first_top = top[0]
    first_bottom = bottom[0]
    for ia in range(angular_segments):
        faces.append((top_center, first_top[ia + 1], first_top[ia]))
        faces.append((bottom_center, first_bottom[ia], first_bottom[ia + 1]))

    for ir in range(radial_segments - 1):
        for ia in range(angular_segments):
            b00 = bottom[ir][ia]
            b01 = bottom[ir][ia + 1]
            b10 = bottom[ir + 1][ia]
            b11 = bottom[ir + 1][ia + 1]
            t00 = top[ir][ia]
            t01 = top[ir][ia + 1]
            t10 = top[ir + 1][ia]
            t11 = top[ir + 1][ia + 1]
            faces.append((t00, t01, t11))
            faces.append((t00, t11, t10))
            faces.append((b00, b10, b11))
            faces.append((b00, b11, b01))

    # Side wall around the curved arc.
    outer_top = top[-1]
    outer_bottom = bottom[-1]
    for ia in range(angular_segments):
        t0, t1 = outer_top[ia], outer_top[ia + 1]
        b0, b1 = outer_bottom[ia], outer_bottom[ia + 1]
        faces.append((b0, t0, t1))
        faces.append((b0, t1, b1))

    # Side walls along both halves of the straight chord.
    for edge_index in (0, angular_segments):
        prev_t = top_center
        prev_b = bottom_center
        for ir in range(radial_segments):
            t = top[ir][edge_index]
            b = bottom[ir][edge_index]
            faces.append((prev_b, prev_t, t))
            faces.append((prev_b, t, b))
            prev_t, prev_b = t, b

    return MeshGeometry(vertices=vertices, faces=faces)


def _diffuser_rib_geometry(
    inner_radius: float,
    outer_radius: float,
    *,
    y_offset: float = 0.012,
    sag: float = 0.036,
    edge_z: float = -0.014,
    thickness: float = 0.004,
    angular_segments: int = 56,
) -> MeshGeometry:
    """Raised semi-circular diffuser moulding seated on the outside of the lens."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    lens_radius = 0.245

    def lens_top(x: float, y: float) -> float:
        y_rel = max(0.0, y - y_offset)
        chord_to_arc = math.sqrt(max(lens_radius * lens_radius - x * x, 0.0))
        if chord_to_arc <= 1.0e-9:
            return edge_z + thickness - 0.0002
        f = max(0.0, min(1.0, y_rel / chord_to_arc))
        return edge_z - sag * (4.0 * f * (1.0 - f)) + thickness - 0.0002

    for radius in (inner_radius, outer_radius):
        row: list[int] = []
        for ia in range(angular_segments + 1):
            theta = math.pi - math.pi * ia / angular_segments
            x = radius * math.cos(theta)
            y = y_offset + radius * math.sin(theta)
            z = lens_top(x, y)
            row.append(len(vertices))
            vertices.append((x, y, z))
        if radius == inner_radius:
            inner = row
        else:
            outer = row

    for ia in range(angular_segments):
        faces.append((inner[ia], inner[ia + 1], outer[ia + 1]))
        faces.append((inner[ia], outer[ia + 1], outer[ia]))
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_circular_oyster_ceiling_light")

    housing_mat = model.material("warm_white_powder_coated_metal", rgba=(0.92, 0.90, 0.84, 1.0))
    hinge_mat = model.material("brushed_nickel_hinge", rgba=(0.55, 0.55, 0.52, 1.0))
    diffuser_mat = model.material("frosted_translucent_acrylic", rgba=(0.86, 0.93, 1.0, 0.48))
    bulb_mat = model.material("warm_glowing_bulb", rgba=(1.0, 0.82, 0.38, 0.72))
    socket_mat = model.material("white_ceramic_socket", rgba=(0.96, 0.94, 0.88, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell_cq(), "d_shaped_shallow_housing", tolerance=0.0015),
        material=housing_mat,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.034),
        origin=Origin(xyz=(0.0, 0.145, 0.033)),
        material=socket_mat,
        name="lamp_socket",
    )
    housing.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.0, 0.145, 0.006)),
        material=bulb_mat,
        name="bulb_globe",
    )
    # Exposed hinge knuckles are outside the flat edge; leaves overlap the flat rail.
    for x in (-0.190, 0.190):
        housing.visual(
            Box((0.104, 0.034, 0.004)),
            origin=Origin(xyz=(x, 0.007, 0.004)),
            material=hinge_mat,
            name=f"hinge_leaf_{0 if x < 0 else 1}",
        )
        housing.visual(
            Cylinder(radius=0.008, length=0.092),
            origin=Origin(xyz=(x, -0.010, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"hinge_barrel_{0 if x < 0 else 1}",
        )

    diffuser = model.part("diffuser")
    diffuser.visual(
        mesh_from_geometry(_diffuser_shell_geometry(), "frosted_semi_dome_diffuser"),
        material=diffuser_mat,
        name="diffuser_shell",
    )
    diffuser.visual(
        mesh_from_geometry(_diffuser_rib_geometry(0.155, 0.163), "inner_diffuser_rib"),
        material=diffuser_mat,
        name="inner_rib",
    )
    diffuser.visual(
        mesh_from_geometry(_diffuser_rib_geometry(0.205, 0.213), "outer_diffuser_rib"),
        material=diffuser_mat,
        name="outer_rib",
    )
    diffuser.visual(
        Cylinder(radius=0.008, length=0.190),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_barrel",
    )
    diffuser.visual(
        Box((0.184, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, -0.006)),
        material=hinge_mat,
        name="hinge_web",
    )
    diffuser.visual(
        Box((0.184, 0.034, 0.008)),
        origin=Origin(xyz=(0.0, 0.023, -0.010)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    diffuser.visual(
        Box((0.070, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.258, -0.014)),
        material=diffuser_mat,
        name="front_pull_lip",
    )

    model.articulation(
        "housing_to_diffuser",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=diffuser,
        origin=Origin(xyz=(0.0, -0.010, 0.006)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    diffuser = object_model.get_part("diffuser")
    hinge = object_model.get_articulation("housing_to_diffuser")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            diffuser,
            housing,
            axes="xy",
            elem_a="diffuser_shell",
            elem_b="housing_shell",
            min_overlap=0.20,
            name="closed diffuser covers the semi-circular housing opening",
        )
        ctx.expect_gap(
            housing,
            diffuser,
            axis="z",
            positive_elem="housing_shell",
            negative_elem="diffuser_shell",
            min_gap=0.002,
            max_gap=0.012,
            name="closed diffuser sits just below the housing rim",
        )

        closed_aabb = ctx.part_element_world_aabb(diffuser, elem="diffuser_shell")

    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_element_world_aabb(diffuser, elem="diffuser_shell")
        ctx.expect_origin_distance(
            housing,
            diffuser,
            axes="xy",
            max_dist=0.020,
            name="diffuser origin remains on the hinge line",
        )

    if closed_aabb is not None and open_aabb is not None:
        closed_min, closed_max = closed_aabb
        open_min, open_max = open_aabb
        ctx.check(
            "positive hinge motion swings diffuser down for bulb access",
            open_min[2] < closed_min[2] - 0.12 and open_max[1] < closed_max[1] - 0.08,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
    else:
        ctx.fail("diffuser pose aabb available", "Could not measure diffuser shell AABBs.")

    return ctx.report()


object_model = build_object_model()
