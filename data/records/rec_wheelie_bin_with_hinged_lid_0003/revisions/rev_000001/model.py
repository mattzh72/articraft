from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _append_loop(geometry: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geometry.add_vertex(x, y, z) for x, y, z in loop]


def _connect_outer_loops(
    geometry: MeshGeometry,
    lower_ids: list[int],
    upper_ids: list[int],
) -> None:
    count = len(lower_ids)
    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(
            geometry,
            lower_ids[index],
            lower_ids[nxt],
            upper_ids[nxt],
            upper_ids[index],
        )


def _connect_inner_loops(
    geometry: MeshGeometry,
    lower_ids: list[int],
    upper_ids: list[int],
) -> None:
    count = len(lower_ids)
    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(
            geometry,
            lower_ids[index],
            upper_ids[index],
            upper_ids[nxt],
            lower_ids[nxt],
        )


def _bridge_ring(
    geometry: MeshGeometry,
    outer_ids: list[int],
    inner_ids: list[int],
    *,
    upward: bool,
) -> None:
    count = len(outer_ids)
    for index in range(count):
        nxt = (index + 1) % count
        if upward:
            _add_quad(
                geometry,
                outer_ids[index],
                outer_ids[nxt],
                inner_ids[nxt],
                inner_ids[index],
            )
        else:
            _add_quad(
                geometry,
                outer_ids[index],
                inner_ids[index],
                inner_ids[nxt],
                outer_ids[nxt],
            )


def _cap_loop(
    geometry: MeshGeometry,
    loop: list[tuple[float, float, float]],
    ids: list[int],
    *,
    upward: bool,
) -> None:
    cx = sum(point[0] for point in loop) / len(loop)
    cy = sum(point[1] for point in loop) / len(loop)
    cz = sum(point[2] for point in loop) / len(loop)
    center_id = geometry.add_vertex(cx, cy, cz)
    count = len(ids)
    for index in range(count):
        nxt = (index + 1) % count
        if upward:
            geometry.add_face(center_id, ids[index], ids[nxt])
        else:
            geometry.add_face(center_id, ids[nxt], ids[index])


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_y: float = 0.0,
    center_x: float = 0.0,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    return [(center_x + x, center_y + y, z) for x, y in profile]


def _build_shell_mesh(
    outer_loops: list[list[tuple[float, float, float]]],
    inner_loops: list[list[tuple[float, float, float]]],
    *,
    cap_outer_bottom: bool,
    cap_inner_bottom: bool,
    bridge_top: bool,
    bridge_bottom: bool,
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_id_loops = [_append_loop(geometry, loop) for loop in outer_loops]
    inner_id_loops = [_append_loop(geometry, loop) for loop in inner_loops]

    for index in range(len(outer_id_loops) - 1):
        _connect_outer_loops(geometry, outer_id_loops[index], outer_id_loops[index + 1])
    for index in range(len(inner_id_loops) - 1):
        _connect_inner_loops(geometry, inner_id_loops[index], inner_id_loops[index + 1])

    if bridge_top:
        _bridge_ring(geometry, outer_id_loops[-1], inner_id_loops[-1], upward=True)
    if bridge_bottom:
        _bridge_ring(geometry, outer_id_loops[0], inner_id_loops[0], upward=False)
    if cap_outer_bottom:
        _cap_loop(geometry, outer_loops[0], outer_id_loops[0], upward=False)
    if cap_inner_bottom:
        _cap_loop(geometry, inner_loops[0], inner_id_loops[0], upward=True)

    return geometry


def _build_body_shell_mesh() -> MeshGeometry:
    outer_loops = [
        _rounded_loop(0.428, 0.515, 0.030, 0.028, center_y=-0.004),
        _rounded_loop(0.492, 0.612, 0.038, 0.410, center_y=0.002),
        _rounded_loop(0.552, 0.690, 0.046, 0.775, center_y=0.008),
        _rounded_loop(0.578, 0.718, 0.050, 0.952, center_y=0.010),
    ]
    inner_loops = [
        _rounded_loop(0.412, 0.499, 0.024, 0.040, center_y=-0.004),
        _rounded_loop(0.476, 0.596, 0.032, 0.410, center_y=0.002),
        _rounded_loop(0.536, 0.674, 0.040, 0.775, center_y=0.008),
        _rounded_loop(0.562, 0.702, 0.044, 0.944, center_y=0.010),
    ]
    return _build_shell_mesh(
        outer_loops,
        inner_loops,
        cap_outer_bottom=True,
        cap_inner_bottom=True,
        bridge_top=True,
        bridge_bottom=True,
    )


def _build_lid_shell_mesh() -> MeshGeometry:
    outer_loops = [
        _rounded_loop(0.610, 0.752, 0.052, -0.028, center_y=-0.359),
        _rounded_loop(0.610, 0.752, 0.052, 0.016, center_y=-0.359),
        _rounded_loop(0.602, 0.744, 0.050, 0.040, center_y=-0.359),
    ]
    inner_loops = [
        _rounded_loop(0.590, 0.732, 0.044, -0.028, center_y=-0.359),
        _rounded_loop(0.590, 0.732, 0.044, 0.004, center_y=-0.359),
        _rounded_loop(0.582, 0.724, 0.042, 0.020, center_y=-0.359),
    ]
    return _build_shell_mesh(
        outer_loops,
        inner_loops,
        cap_outer_bottom=False,
        cap_inner_bottom=False,
        bridge_top=True,
        bridge_bottom=False,
    )


def _wheel_tire_mesh(radius: float, width: float) -> MeshGeometry:
    half_width = width * 0.5
    profile = [
        (radius * 0.50, -half_width * 0.98),
        (radius * 0.76, -half_width),
        (radius * 0.92, -half_width * 0.82),
        (radius * 0.99, -half_width * 0.40),
        (radius * 1.00, 0.0),
        (radius * 0.99, half_width * 0.40),
        (radius * 0.92, half_width * 0.82),
        (radius * 0.76, half_width),
        (radius * 0.50, half_width * 0.98),
        (radius * 0.40, half_width * 0.36),
        (radius * 0.38, 0.0),
        (radius * 0.40, -half_width * 0.36),
        (radius * 0.50, -half_width * 0.98),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def _add_wheel_visuals(
    wheel_part,
    mesh_name: str,
    *,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    hub_length: float,
    body_plastic,
    hub_material,
    tire_material,
    outer_sign: float,
) -> None:
    spin_axis_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    wheel_part.visual(
        _save_mesh(mesh_name, _wheel_tire_mesh(tire_radius, tire_width)),
        material=tire_material,
        name="tire",
    )
    wheel_part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=spin_axis_origin,
        material=hub_material,
        name="hub_core",
    )
    wheel_part.visual(
        Cylinder(radius=hub_radius * 0.70, length=0.010),
        origin=Origin(
            xyz=(outer_sign * (hub_length * 0.5 + 0.005), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_plastic,
        name="outer_cap",
    )
    wheel_part.visual(
        Cylinder(radius=hub_radius * 0.55, length=0.010),
        origin=Origin(
            xyz=(-outer_sign * (hub_length * 0.5 + 0.005), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hub_material,
        name="inner_boss",
    )
    wheel_part.visual(
        Cylinder(radius=0.0022, length=0.016),
        origin=Origin(
            xyz=(outer_sign * 0.014, 0.0, tire_radius - 0.006),
        ),
        material=hub_material,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelie_bin", assets=ASSETS)

    body_matte = model.material("body_matte", rgba=(0.23, 0.25, 0.27, 1.0))
    lid_matte = model.material("lid_matte", rgba=(0.25, 0.27, 0.29, 1.0))
    trim_satin = model.material("trim_satin", rgba=(0.38, 0.40, 0.43, 1.0))
    hardware_satin = model.material("hardware_satin", rgba=(0.48, 0.50, 0.53, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        _save_mesh("wheelie_bin_body_shell.obj", _build_body_shell_mesh()),
        material=body_matte,
        name="body_shell",
    )
    body.visual(
        Box((0.396, 0.072, 0.020)),
        origin=Origin(xyz=(0.0, 0.214, 0.137)),
        material=trim_satin,
        name="rear_crossmember",
    )
    body.visual(
        Box((0.056, 0.138, 0.062)),
        origin=Origin(xyz=(0.202, 0.304, 0.124)),
        material=body_matte,
        name="left_axle_tower",
    )
    body.visual(
        Box((0.056, 0.138, 0.062)),
        origin=Origin(xyz=(-0.202, 0.304, 0.124)),
        material=body_matte,
        name="right_axle_tower",
    )
    body.visual(
        Box((0.360, 0.040, 0.120)),
        origin=Origin(xyz=(0.0, 0.314, 0.820)),
        material=trim_satin,
        name="rear_handle_bridge",
    )
    body.visual(
        Box((0.040, 0.060, 0.118)),
        origin=Origin(xyz=(0.158, 0.322, 0.761)),
        material=trim_satin,
        name="left_handle_post",
    )
    body.visual(
        Box((0.040, 0.060, 0.118)),
        origin=Origin(xyz=(-0.158, 0.322, 0.761)),
        material=trim_satin,
        name="right_handle_post",
    )
    body.visual(
        Box((0.438, 0.056, 0.058)),
        origin=Origin(xyz=(0.0, -0.286, 0.206)),
        material=trim_satin,
        name="front_lift_lip",
    )
    body.visual(
        Box((0.058, 0.060, 0.098)),
        origin=Origin(xyz=(0.184, -0.286, 0.178)),
        material=trim_satin,
        name="left_lip_gusset",
    )
    body.visual(
        Box((0.058, 0.060, 0.098)),
        origin=Origin(xyz=(-0.184, -0.286, 0.178)),
        material=trim_satin,
        name="right_lip_gusset",
    )
    body.visual(
        Box((0.452, 0.084, 0.032)),
        origin=Origin(xyz=(0.0, 0.356, 0.942)),
        material=trim_satin,
        name="rear_rim_cap",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.74, 0.98)),
        mass=14.5,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    axle = model.part("rear_axle")
    axle.visual(
        Cylinder(radius=0.012, length=0.460),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_satin,
        name="axle_shaft",
    )
    axle.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(xyz=(0.258, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_satin,
        name="left_shoulder",
    )
    axle.visual(
        Cylinder(radius=0.022, length=0.056),
        origin=Origin(xyz=(-0.258, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_satin,
        name="right_shoulder",
    )
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.572),
        mass=0.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.048),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        "wheelie_bin_left_wheel_tire.obj",
        tire_radius=0.115,
        tire_width=0.048,
        hub_radius=0.050,
        hub_length=0.024,
        body_plastic=trim_satin,
        hub_material=hardware_satin,
        tire_material=tire_rubber,
        outer_sign=1.0,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.048),
        mass=1.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        "wheelie_bin_right_wheel_tire.obj",
        tire_radius=0.115,
        tire_width=0.048,
        hub_radius=0.050,
        hub_length=0.024,
        body_plastic=trim_satin,
        hub_material=hardware_satin,
        tire_material=tire_rubber,
        outer_sign=-1.0,
    )

    lid = model.part("lid")
    lid.visual(
        _save_mesh("wheelie_bin_lid_shell.obj", _build_lid_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=lid_matte,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.500),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_satin,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.76, 0.08)),
        mass=2.3,
        origin=Origin(xyz=(0.0, -0.359, 0.0)),
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, 0.395, 0.115)),
    )
    model.articulation(
        "axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=left_wheel,
        origin=Origin(xyz=(0.308, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=right_wheel,
        origin=Origin(xyz=(-0.308, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.404, 0.958)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    body = object_model.get_part("body")
    axle = object_model.get_part("rear_axle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid = object_model.get_part("lid")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("axle_to_left_wheel")
    right_spin = object_model.get_articulation("axle_to_right_wheel")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(axle, body)
    ctx.expect_contact(left_wheel, axle)
    ctx.expect_contact(right_wheel, axle)
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.030,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="body_shell",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.52, elem_a="lid_shell", elem_b="body_shell")

    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    lid_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    left_wheel_pos = ctx.part_world_position(left_wheel)
    right_wheel_pos = ctx.part_world_position(right_wheel)

    assert body_shell_aabb is not None
    assert lid_shell_aabb is not None
    assert left_wheel_pos is not None
    assert right_wheel_pos is not None

    body_width = body_shell_aabb[1][0] - body_shell_aabb[0][0]
    body_depth = body_shell_aabb[1][1] - body_shell_aabb[0][1]
    body_height = body_shell_aabb[1][2] - body_shell_aabb[0][2]
    overall_height_closed = max(body_shell_aabb[1][2], lid_shell_aabb[1][2])

    ctx.check(
        "body_width_realistic",
        0.55 <= body_width <= 0.60,
        f"body width was {body_width:.3f} m",
    )
    ctx.check(
        "body_depth_realistic",
        0.69 <= body_depth <= 0.77,
        f"body depth was {body_depth:.3f} m",
    )
    ctx.check(
        "body_height_realistic",
        0.90 <= body_height <= 0.97,
        f"body height was {body_height:.3f} m",
    )
    ctx.check(
        "overall_height_realistic",
        0.98 <= overall_height_closed <= 1.03,
        f"overall closed height was {overall_height_closed:.3f} m",
    )
    ctx.check(
        "rear_track_realistic",
        0.60 <= (left_wheel_pos[0] - right_wheel_pos[0]) <= 0.64,
        f"rear track was {(left_wheel_pos[0] - right_wheel_pos[0]):.3f} m",
    )

    lid_rest_aabb = lid_shell_aabb
    limits = lid_hinge.motion_limits
    assert limits is not None
    assert limits.upper is not None
    with ctx.pose({lid_hinge: limits.upper}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        assert lid_open_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_overlap")
        ctx.fail_if_isolated_parts(name="lid_open_no_floating")
        ctx.check(
            "lid_opens_above_body",
            lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.35,
            f"lid max z changed from {lid_rest_aabb[1][2]:.3f} to {lid_open_aabb[1][2]:.3f}",
        )

    left_valve_rest = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
    assert left_valve_rest is not None
    with ctx.pose({left_spin: math.pi / 2.0}):
        left_valve_quarter = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
        assert left_valve_quarter is not None
        ctx.expect_contact(left_wheel, axle, name="left_wheel_spin_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="left_wheel_spin_no_overlap")
        ctx.check(
            "left_wheel_spin_visible_motion",
            left_valve_rest[1][2] > left_valve_quarter[1][2] + 0.07,
            (
                f"valve stem max z moved from {left_valve_rest[1][2]:.3f} "
                f"to {left_valve_quarter[1][2]:.3f}"
            ),
        )

    with ctx.pose({right_spin: math.pi}):
        ctx.expect_contact(right_wheel, axle, name="right_wheel_spin_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="right_wheel_spin_no_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
