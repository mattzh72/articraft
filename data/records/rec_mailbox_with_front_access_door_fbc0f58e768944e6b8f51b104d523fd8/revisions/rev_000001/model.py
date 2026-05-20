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


def _d_profile(width: float, side_height: float, *, arc_segments: int = 18) -> list[tuple[float, float]]:
    """Mailbox end profile in local (y, z): flat bottom, vertical sides, round roof."""
    radius = width / 2.0
    points: list[tuple[float, float]] = [(-radius, 0.0), (radius, 0.0)]
    for i in range(arc_segments + 1):
        theta = math.pi * i / arc_segments
        points.append((radius * math.cos(theta), side_height + radius * math.sin(theta)))
    return points


def _d_prism_mesh(
    *,
    name_width: float,
    side_height: float,
    x_min: float,
    x_max: float,
    z_offset: float = 0.0,
    arc_segments: int = 18,
) -> MeshGeometry:
    """Extrude a D-shaped mailbox end panel along X."""
    profile = _d_profile(name_width, side_height, arc_segments=arc_segments)
    geom = MeshGeometry()
    left_indices: list[int] = []
    right_indices: list[int] = []
    for y, z in profile:
        left_indices.append(geom.add_vertex(x_min, y, z + z_offset))
    for y, z in profile:
        right_indices.append(geom.add_vertex(x_max, y, z + z_offset))

    count = len(profile)
    for i in range(count):
        j = (i + 1) % count
        a0 = left_indices[i]
        a1 = left_indices[j]
        b0 = right_indices[i]
        b1 = right_indices[j]
        geom.add_face(a0, b0, b1)
        geom.add_face(a0, b1, a1)

    cy = sum(p[0] for p in profile) / count
    cz = sum(p[1] for p in profile) / count + z_offset
    c_left = geom.add_vertex(x_min, cy, cz)
    c_right = geom.add_vertex(x_max, cy, cz)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(c_left, left_indices[j], left_indices[i])
        geom.add_face(c_right, right_indices[i], right_indices[j])
    return geom


def _arched_roof_shell_mesh(
    *,
    length: float,
    width: float,
    side_top_z: float,
    thickness: float,
    x_center: float = 0.0,
    arc_segments: int = 28,
) -> MeshGeometry:
    """Thin half-cylinder roof shell, open underneath like a real rural mailbox."""
    r_outer = width / 2.0
    r_inner = r_outer - thickness
    x0 = x_center - length / 2.0
    x1 = x_center + length / 2.0
    geom = MeshGeometry()

    outer0: list[int] = []
    outer1: list[int] = []
    inner0: list[int] = []
    inner1: list[int] = []
    for i in range(arc_segments + 1):
        theta = math.pi * i / arc_segments
        co = math.cos(theta)
        si = math.sin(theta)
        yo = r_outer * co
        zo = side_top_z + r_outer * si
        yi = r_inner * co
        zi = side_top_z + r_inner * si
        outer0.append(geom.add_vertex(x0, yo, zo))
        outer1.append(geom.add_vertex(x1, yo, zo))
        inner0.append(geom.add_vertex(x0, yi, zi))
        inner1.append(geom.add_vertex(x1, yi, zi))

    for i in range(arc_segments):
        j = i + 1
        # Outer curved skin.
        geom.add_face(outer0[i], outer1[i], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer0[j])
        # Inner curved skin.
        geom.add_face(inner0[i], inner1[j], inner1[i])
        geom.add_face(inner0[i], inner0[j], inner1[j])
        # Front and rear thickness bands.
        geom.add_face(outer1[i], inner1[i], inner1[j])
        geom.add_face(outer1[i], inner1[j], outer1[j])
        geom.add_face(outer0[i], outer0[j], inner0[j])
        geom.add_face(outer0[i], inner0[j], inner0[i])

    # Long lower lips where the roof sits on the two vertical side walls.
    for idx in (0, arc_segments):
        geom.add_face(outer0[idx], inner1[idx], outer1[idx])
        geom.add_face(outer0[idx], inner0[idx], inner1[idx])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curbside_mailbox")

    metal = Material("galvanized_metal", rgba=(0.62, 0.66, 0.67, 1.0))
    dark_metal = Material("dark_hinge_metal", rgba=(0.18, 0.19, 0.19, 1.0))
    red = Material("signal_flag_red", rgba=(0.90, 0.05, 0.03, 1.0))
    wood = Material("weathered_post_wood", rgba=(0.43, 0.29, 0.16, 1.0))
    shadow = Material("dark_mailbox_interior", rgba=(0.035, 0.038, 0.04, 1.0))

    length = 0.64
    width = 0.30
    wall = 0.018
    base_z = 1.00
    side_height = 0.16
    side_top_z = base_z + side_height
    x_front = length / 2.0
    x_rear = -length / 2.0

    body = model.part("body")

    # Fixed post and support saddle.
    body.visual(
        Box((0.10, 0.10, 1.02)),
        origin=Origin(xyz=(-0.10, 0.0, 0.51)),
        material=wood,
        name="post",
    )
    body.visual(
        Box((0.46, 0.13, 0.06)),
        origin=Origin(xyz=(-0.08, 0.0, 0.975)),
        material=wood,
        name="support_saddle",
    )

    # Metal mailbox shell: floor, sides, arched roof, and closed rear.
    body.visual(
        Box((length, width - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, base_z + wall / 2.0)),
        material=metal,
        name="floor_panel",
    )
    for side_name, y in (("side_wall_0", width / 2.0 - wall / 2.0), ("side_wall_1", -width / 2.0 + wall / 2.0)):
        body.visual(
            Box((length, wall, side_height)),
            origin=Origin(xyz=(0.0, y, base_z + side_height / 2.0)),
            material=metal,
            name=side_name,
        )

    roof = _arched_roof_shell_mesh(
        length=length,
        width=width,
        side_top_z=side_top_z,
        thickness=wall,
    )
    body.visual(
        mesh_from_geometry(roof, "arched_roof_shell"),
        material=metal,
        name="arched_roof",
    )

    rear_panel = _d_prism_mesh(
        name_width=width,
        side_height=side_height,
        x_min=x_rear - wall,
        x_max=x_rear,
        z_offset=base_z,
    )
    body.visual(
        mesh_from_geometry(rear_panel, "rear_panel_mesh"),
        material=metal,
        name="rear_panel",
    )
    body.visual(
        Box((0.004, width - 0.055, side_height + width / 2.0 - 0.045)),
        origin=Origin(xyz=(x_rear + 0.006, 0.0, base_z + 0.14)),
        material=shadow,
        name="interior_shadow",
    )

    # Lower front hinge knuckles fixed to the mailbox body.
    hinge_x = x_front + 0.006
    hinge_z = base_z - 0.012
    for y in (-0.090, 0.090):
        body.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"door_hinge_knuckle_{0 if y < 0 else 1}",
        )
        body.visual(
            Box((0.018, 0.070, 0.020)),
            origin=Origin(xyz=(hinge_x - 0.010, y, hinge_z + 0.014)),
            material=dark_metal,
            name=f"door_hinge_leaf_{0 if y < 0 else 1}",
        )

    # Fixed pivot boss for the side signal flag.
    flag_pivot_x = 0.18
    flag_pivot_y = -width / 2.0 - 0.020
    flag_pivot_z = base_z + 0.160
    body.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(
            xyz=(flag_pivot_x, flag_pivot_y + 0.010, flag_pivot_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="flag_pivot",
    )

    door = model.part("door")
    door_panel = _d_prism_mesh(
        name_width=width + 0.018,
        side_height=side_height,
        x_min=0.014,
        x_max=0.034,
        z_offset=0.0,
    )
    door.visual(
        mesh_from_geometry(door_panel, "curved_front_door"),
        material=metal,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.006, 0.112, 0.024)),
        origin=Origin(xyz=(0.013, 0.0, 0.006)),
        material=dark_metal,
        name="door_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.041, 0.0, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="door_pull",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.55),
    )

    flag = model.part("flag")
    flag.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="flag_hub",
    )
    flag.visual(
        Box((0.210, 0.014, 0.020)),
        origin=Origin(xyz=(-0.105, -0.018, 0.0)),
        material=red,
        name="flag_arm",
    )
    flag.visual(
        Box((0.070, 0.014, 0.105)),
        origin=Origin(xyz=(-0.205, -0.018, 0.045)),
        material=red,
        name="flag_blade",
    )

    model.articulation(
        "body_to_flag",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flag,
        origin=Origin(xyz=(flag_pivot_x, flag_pivot_y, flag_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.5708),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    flag = object_model.get_part("flag")
    door_joint = object_model.get_articulation("body_to_door")
    flag_joint = object_model.get_articulation("body_to_flag")

    with ctx.pose({door_joint: 0.0, flag_joint: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_panel",
            elem_b="rear_panel",
            min_overlap=0.24,
            name="closed door covers rounded mailbox opening",
        )
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="floor_panel",
            min_gap=0.006,
            name="closed front door sits proud of the metal floor lip",
        )
        ctx.expect_gap(
            body,
            flag,
            axis="y",
            positive_elem="flag_pivot",
            negative_elem="flag_hub",
            max_gap=0.002,
            max_penetration=0.0,
            name="flag hub is seated on the side pivot boss",
        )
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        lowered_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_blade")

    with ctx.pose({door_joint: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "front door rotates outward and downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.12
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.05,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({flag_joint: 1.5708}):
        raised_flag_aabb = ctx.part_element_world_aabb(flag, elem="flag_blade")

    ctx.check(
        "signal flag rotates upward on side pivot",
        lowered_flag_aabb is not None
        and raised_flag_aabb is not None
        and raised_flag_aabb[1][2] > lowered_flag_aabb[1][2] + 0.12,
        details=f"lowered={lowered_flag_aabb}, raised={raised_flag_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
