from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_LENGTH = 0.64
CASE_WIDTH = 0.42
LOWER_DEPTH = 0.16
WALL = 0.018
CORNER_RADIUS = 0.055
HINGE_RADIUS = 0.007
HINGE_OFFSET = 0.018
OPEN_ANGLE = 1.05


def _translated_profile(points, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in points]


def _add_loop(mesh: MeshGeometry, profile, z: float):
    return [mesh.add_vertex(x, y, z) for x, y in profile]


def _add_quads(mesh: MeshGeometry, loop_a, loop_b, *, flip: bool = False) -> None:
    count = len(loop_a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(loop_a[i], loop_b[j], loop_b[i])
            mesh.add_face(loop_a[i], loop_a[j], loop_b[j])
        else:
            mesh.add_face(loop_a[i], loop_b[i], loop_b[j])
            mesh.add_face(loop_a[i], loop_b[j], loop_a[j])


def _add_cap(mesh: MeshGeometry, loop, *, z: float, flip: bool = False) -> None:
    cx = sum(mesh.vertices[i][0] for i in loop) / len(loop)
    cy = sum(mesh.vertices[i][1] for i in loop) / len(loop)
    center = mesh.add_vertex(cx, cy, z)
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if flip:
            mesh.add_face(center, loop[j], loop[i])
        else:
            mesh.add_face(center, loop[i], loop[j])


def _tray_mesh(length: float, width: float, depth: float, wall: float, radius: float) -> MeshGeometry:
    """Open-topped rounded rectangular lower case shell, with its rear edge at x=0."""
    outer = _translated_profile(
        rounded_rect_profile(length, width, radius, corner_segments=10),
        dx=length / 2.0 + HINGE_OFFSET,
    )
    inner = _translated_profile(
        rounded_rect_profile(length - 2.0 * wall, width - 2.0 * wall, radius - wall, corner_segments=10),
        dx=length / 2.0 + HINGE_OFFSET,
    )

    top_z = -0.012
    bottom_z = top_z - depth
    floor_z = bottom_z + wall

    mesh = MeshGeometry()
    outer_top = _add_loop(mesh, outer, top_z)
    outer_bottom = _add_loop(mesh, outer, bottom_z)
    inner_top = _add_loop(mesh, inner, top_z)
    inner_floor = _add_loop(mesh, inner, floor_z)

    _add_quads(mesh, outer_bottom, outer_top)
    _add_quads(mesh, outer_top, inner_top, flip=True)
    _add_quads(mesh, inner_top, inner_floor, flip=True)
    _add_cap(mesh, inner_floor, z=floor_z)
    _add_cap(mesh, outer_bottom, z=bottom_z, flip=True)
    return mesh


def _lid_cover_mesh(length: float, width: float, height: float, wall: float, radius: float) -> MeshGeometry:
    """Shallow rounded lid shell, open downward when unrotated into the closed pose."""
    outer = _translated_profile(
        rounded_rect_profile(length, width, radius, corner_segments=10),
        dx=length / 2.0 + HINGE_OFFSET,
    )
    inner = _translated_profile(
        rounded_rect_profile(length - 2.0 * wall, width - 2.0 * wall, radius - wall, corner_segments=10),
        dx=length / 2.0 + HINGE_OFFSET,
    )

    bottom_z = -0.006
    top_z = bottom_z + height
    ceiling_z = top_z - wall

    mesh = MeshGeometry()
    outer_bottom = _add_loop(mesh, outer, bottom_z)
    outer_top = _add_loop(mesh, outer, top_z)
    inner_bottom = _add_loop(mesh, inner, bottom_z)
    inner_ceiling = _add_loop(mesh, inner, ceiling_z)

    _add_quads(mesh, outer_bottom, outer_top)
    _add_cap(mesh, outer_top, z=top_z)
    _add_quads(mesh, outer_bottom, inner_bottom, flip=True)
    _add_quads(mesh, inner_bottom, inner_ceiling, flip=True)
    _add_cap(mesh, inner_ceiling, z=ceiling_z, flip=True)
    return mesh


def _rot_y(point, angle: float):
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x + s * z, y, -s * x + c * z)


def _lid_origin(closed_xyz):
    return Origin(xyz=_rot_y(closed_xyz, -OPEN_ANGLE), rpy=(0.0, -OPEN_ANGLE, 0.0))


def _lid_point(closed_xyz):
    return _rot_y(closed_xyz, -OPEN_ANGLE)


def _add_hinge_knuckles(part, *, moving: bool, material: str) -> None:
    hinge_span = CASE_WIDTH - 0.060
    gap = 0.006
    segment_count = 8
    segment_len = (hinge_span - gap * (segment_count - 1)) / segment_count
    start = -hinge_span / 2.0
    for i in range(segment_count):
        if (i % 2 == 1) != moving:
            continue
        y0 = start + i * (segment_len + gap)
        y_center = y0 + segment_len / 2.0
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=segment_len),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"hinge_knuckle_{i}",
        )


def _add_side_stay(
    model: ArticulatedObject,
    shell,
    lid,
    index: int,
    side_sign: float,
    *,
    metal: str,
    dark_metal: str,
):
    side_y = side_sign * (CASE_WIDTH / 2.0 - WALL - 0.024)
    wall_y = side_sign * (CASE_WIDTH / 2.0 - WALL - 0.003)
    pivot_x = 0.155
    pivot_z = -0.058
    lid_pivot_closed = (0.225, side_y, 0.002)
    lid_pivot_open = _lid_point(lid_pivot_closed)

    bridge_face_y = side_y + side_sign * 0.004
    pin_center_y = (bridge_face_y + wall_y) / 2.0
    pin_length = abs(wall_y - bridge_face_y)

    shell.visual(
        Box((0.052, 0.007, 0.044)),
        origin=Origin(xyz=(pivot_x, wall_y, pivot_z)),
        material=metal,
        name=f"stay_plate_{index}",
    )
    shell.visual(
        Cylinder(radius=0.006, length=pin_length),
        origin=Origin(xyz=(pivot_x, pin_center_y, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name=f"stay_pin_{index}",
    )

    lid_pin_center_closed = (lid_pivot_closed[0], pin_center_y, lid_pivot_closed[2])
    lid.visual(
        Box((0.054, 0.007, 0.034)),
        origin=_lid_origin((lid_pivot_closed[0], wall_y, lid_pivot_closed[2] + 0.001)),
        material=metal,
        name=f"stay_plate_{index}",
    )
    lid.visual(
        Cylinder(radius=0.006, length=pin_length),
        origin=Origin(xyz=_lid_point(lid_pin_center_closed), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name=f"stay_pin_{index}",
    )

    link = model.part(f"stay_link_{index}")
    dx = lid_pivot_open[0] - pivot_x
    dz = lid_pivot_open[2] - pivot_z
    length = math.hypot(dx, dz)
    angle_y = -math.atan2(dz, dx)
    link.visual(
        Box((length, 0.010, 0.010)),
        origin=Origin(xyz=(dx / 2.0, 0.0, dz / 2.0), rpy=(0.0, angle_y, 0.0)),
        material=metal,
        name="flat_bar",
    )
    link.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="lower_eye",
    )
    link.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(dx, 0.0, dz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="upper_eye",
    )

    model.articulation(
        f"shell_to_stay_{index}",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=link,
        origin=Origin(xyz=(pivot_x, side_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-OPEN_ANGLE, upper=0.0),
        mimic=Mimic("shell_to_lid", multiplier=1.0, offset=0.0),
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_mixer_case")
    model.material("charcoal_shell", rgba=(0.055, 0.066, 0.075, 1.0))
    model.material("slate_lid", rgba=(0.070, 0.078, 0.090, 1.0))
    model.material("black_foam", rgba=(0.010, 0.011, 0.012, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.65, 0.60, 1.0))
    model.material("dark_pin", rgba=(0.23, 0.23, 0.22, 1.0))
    model.material("rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    shell = model.part("lower_shell")
    shell.visual(
        mesh_from_geometry(_tray_mesh(CASE_LENGTH, CASE_WIDTH, LOWER_DEPTH, WALL, CORNER_RADIUS), "deep_lower_shell"),
        material="charcoal_shell",
        name="deep_shell",
    )
    shell.visual(
        Box((0.46, 0.30, 0.010)),
        origin=Origin(xyz=(0.36, 0.0, -0.150)),
        material="black_foam",
        name="bottom_foam",
    )
    shell.visual(
        Box((0.16, 0.030, 0.028)),
        origin=Origin(xyz=(CASE_LENGTH + HINGE_OFFSET - 0.012, 0.0, -0.068)),
        material="rubber",
        name="front_handle_grip",
    )
    shell.visual(
        Box((0.040, CASE_WIDTH - 0.050, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, -0.0095)),
        material="brushed_steel",
        name="hinge_leaf",
    )
    _add_hinge_knuckles(shell, moving=False, material="brushed_steel")

    lid = model.part("lid")
    lid_mesh = _lid_cover_mesh(CASE_LENGTH, CASE_WIDTH, 0.058, WALL, CORNER_RADIUS)
    lid_mesh.rotate_y(-OPEN_ANGLE)
    lid.visual(
        mesh_from_geometry(lid_mesh, "broad_lid_shell"),
        material="slate_lid",
        name="lid_shell",
    )
    lid.visual(
        Box((0.45, 0.29, 0.007)),
        origin=_lid_origin((0.37, 0.0, 0.032)),
        material="black_foam",
        name="lid_foam",
    )
    lid.visual(
        Box((0.040, CASE_WIDTH - 0.050, 0.006)),
        origin=_lid_origin((0.020, 0.0, -0.006)),
        material="brushed_steel",
        name="hinge_leaf",
    )
    _add_hinge_knuckles(lid, moving=True, material="brushed_steel")

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-OPEN_ANGLE, upper=0.0),
    )

    _add_side_stay(model, shell, lid, 0, 1.0, metal="brushed_steel", dark_metal="dark_pin")
    _add_side_stay(model, shell, lid, 1, -1.0, metal="brushed_steel", dark_metal="dark_pin")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("shell_to_lid")

    ctx.expect_origin_distance(lid, shell, axes="xyz", max_dist=0.001, name="lid remains clipped to rear hinge axis")

    for index in (0, 1):
        stay = object_model.get_part(f"stay_link_{index}")
        ctx.allow_overlap(
            stay,
            shell,
            elem_a="lower_eye",
            elem_b=f"stay_pin_{index}",
            reason="The stay eye is intentionally captured on the short shell pivot pin with a tiny seated overlap.",
        )
        ctx.allow_overlap(
            stay,
            lid,
            elem_a="upper_eye",
            elem_b=f"stay_pin_{index}",
            reason="The stay eye is intentionally captured on the short lid pivot pin with a tiny seated overlap.",
        )
        ctx.expect_contact(
            stay,
            shell,
            elem_a="lower_eye",
            elem_b=f"stay_pin_{index}",
            contact_tol=0.002,
            name=f"stay {index} lower pivot is seated on shell pin",
        )
        ctx.expect_contact(
            stay,
            lid,
            elem_a="upper_eye",
            elem_b=f"stay_pin_{index}",
            contact_tol=0.002,
            name=f"stay {index} upper pivot is seated on lid pin",
        )
        if index == 0:
            ctx.expect_gap(
                shell,
                stay,
                axis="y",
                positive_elem=f"stay_pin_{index}",
                negative_elem="lower_eye",
                max_gap=0.002,
                max_penetration=0.002,
                name=f"stay {index} shell pivot capture is local",
            )
            ctx.expect_gap(
                lid,
                stay,
                axis="y",
                positive_elem=f"stay_pin_{index}",
                negative_elem="upper_eye",
                max_gap=0.002,
                max_penetration=0.002,
                name=f"stay {index} lid pivot capture is local",
            )
        else:
            ctx.expect_gap(
                stay,
                shell,
                axis="y",
                positive_elem="lower_eye",
                negative_elem=f"stay_pin_{index}",
                max_gap=0.002,
                max_penetration=0.002,
                name=f"stay {index} shell pivot capture is local",
            )
            ctx.expect_gap(
                stay,
                lid,
                axis="y",
                positive_elem="upper_eye",
                negative_elem=f"stay_pin_{index}",
                max_gap=0.002,
                max_penetration=0.002,
                name=f"stay {index} lid pivot capture is local",
            )

    open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({hinge: -OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="deep_shell",
            min_gap=0.0,
            max_gap=0.014,
            name="closed lid sits just above lower shell rim",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    def _zmax(aabb):
        if aabb is None:
            return None
        vmax = aabb[1]
        return vmax[2] if isinstance(vmax, (tuple, list)) else vmax.z

    ctx.check(
        "open pose lifts broad lid high above case",
        open_aabb is not None and closed_aabb is not None and _zmax(open_aabb) > _zmax(closed_aabb) + 0.22,
        details=f"open={open_aabb}, closed={closed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
