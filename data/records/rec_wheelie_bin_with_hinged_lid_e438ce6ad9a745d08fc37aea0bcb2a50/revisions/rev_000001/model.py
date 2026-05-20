from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_fan(geom: MeshGeometry, loop: list[int], center: int, *, reverse: bool = False) -> None:
    for index in range(len(loop)):
        nxt = (index + 1) % len(loop)
        if reverse:
            geom.add_face(center, loop[nxt], loop[index])
        else:
            geom.add_face(center, loop[index], loop[nxt])


def _add_loop_vertices(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(*point) for point in points]


def _bridge_loops(geom: MeshGeometry, lower: list[int], upper: list[int], *, flip: bool = False) -> None:
    count = len(lower)
    for index in range(count):
        nxt = (index + 1) % count
        if flip:
            _add_quad(geom, lower[index], upper[index], upper[nxt], lower[nxt])
        else:
            _add_quad(geom, lower[index], lower[nxt], upper[nxt], upper[index])


def _rounded_loop(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _build_bin_shell_mesh() -> MeshGeometry:
    wall = 0.0065
    floor_thickness = 0.012
    z_bottom = 0.045
    z_top = 0.932

    outer_bottom = _rounded_loop(0.490, 0.610, 0.042, z_bottom)
    outer_top = _rounded_loop(0.580, 0.740, 0.056, z_top)
    inner_top = _rounded_loop(0.580 - (2.0 * wall), 0.740 - (2.0 * wall), 0.050, z_top)
    inner_bottom = _rounded_loop(
        0.490 - (2.0 * wall),
        0.610 - (2.0 * wall),
        0.035,
        z_bottom + floor_thickness,
    )

    geom = MeshGeometry()
    outer_bottom_ids = _add_loop_vertices(geom, outer_bottom)
    outer_top_ids = _add_loop_vertices(geom, outer_top)
    inner_top_ids = _add_loop_vertices(geom, inner_top)
    inner_bottom_ids = _add_loop_vertices(geom, inner_bottom)

    outer_bottom_center = geom.add_vertex(0.0, 0.0, z_bottom)
    inner_bottom_center = geom.add_vertex(0.0, 0.0, z_bottom + floor_thickness)

    _bridge_loops(geom, outer_bottom_ids, outer_top_ids)
    _bridge_loops(geom, inner_bottom_ids, inner_top_ids, flip=True)
    _bridge_loops(geom, outer_top_ids, inner_top_ids)
    _bridge_loops(geom, outer_bottom_ids, inner_bottom_ids, flip=True)
    _add_fan(geom, outer_bottom_ids, outer_bottom_center, reverse=True)
    _add_fan(geom, inner_bottom_ids, inner_bottom_center)
    return geom


def _build_wheel_tire_mesh(radius: float, width: float) -> MeshGeometry:
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.52, -half_width * 0.98),
        (radius * 0.76, -half_width),
        (radius * 0.92, -half_width * 0.78),
        (radius, -half_width * 0.38),
        (radius * 1.02, 0.0),
        (radius, half_width * 0.38),
        (radius * 0.92, half_width * 0.78),
        (radius * 0.76, half_width),
        (radius * 0.52, half_width * 0.98),
        (radius * 0.42, half_width * 0.35),
        (radius * 0.38, 0.0),
        (radius * 0.42, -half_width * 0.35),
        (radius * 0.52, -half_width * 0.98),
    ]
    return LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_wheelie_bin")

    body_green = model.material("body_green", rgba=(0.17, 0.33, 0.21, 1.0))
    lid_green = model.material("lid_green", rgba=(0.20, 0.38, 0.24, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.59, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    wear_black = model.material("wear_black", rgba=(0.12, 0.12, 0.12, 1.0))

    shell_mesh = mesh_from_geometry(_build_bin_shell_mesh(), "wheelie_bin_shell")
    tire_mesh = mesh_from_geometry(_build_wheel_tire_mesh(0.112, 0.054), "wheelie_bin_tire")

    body = model.part("body")
    body.visual(shell_mesh, material=body_green, name="shell")
    body.visual(
        Box((0.020, 0.690, 0.022)),
        origin=Origin(xyz=(-0.286, 0.0, 0.943)),
        material=body_green,
        name="left_rim_rail",
    )
    body.visual(
        Box((0.020, 0.690, 0.022)),
        origin=Origin(xyz=(0.286, 0.0, 0.943)),
        material=body_green,
        name="right_rim_rail",
    )
    body.visual(
        Box((0.552, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.359, 0.943)),
        material=body_green,
        name="front_rim_rail",
    )
    body.visual(
        Box((0.086, 0.034, 0.040)),
        origin=Origin(xyz=(-0.220, -0.388, 0.952)),
        material=body_green,
        name="left_hinge_pedestal",
    )
    body.visual(
        Box((0.060, 0.022, 0.064)),
        origin=Origin(xyz=(-0.220, -0.381, 0.912)),
        material=body_green,
        name="left_hinge_web",
    )
    body.visual(
        Box((0.086, 0.034, 0.040)),
        origin=Origin(xyz=(0.000, -0.388, 0.952)),
        material=body_green,
        name="center_hinge_pedestal",
    )
    body.visual(
        Box((0.060, 0.022, 0.064)),
        origin=Origin(xyz=(0.000, -0.381, 0.912)),
        material=body_green,
        name="center_hinge_web",
    )
    body.visual(
        Box((0.086, 0.034, 0.040)),
        origin=Origin(xyz=(0.220, -0.388, 0.952)),
        material=body_green,
        name="right_hinge_pedestal",
    )
    body.visual(
        Box((0.060, 0.022, 0.064)),
        origin=Origin(xyz=(0.220, -0.381, 0.912)),
        material=body_green,
        name="right_hinge_web",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.530),
        origin=Origin(xyz=(0.0, -0.300, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle_tube",
    )
    body.visual(
        Box((0.104, 0.098, 0.168)),
        origin=Origin(xyz=(-0.204, -0.270, 0.136)),
        material=dark_steel,
        name="left_axle_bracket",
    )
    body.visual(
        Box((0.104, 0.098, 0.168)),
        origin=Origin(xyz=(0.204, -0.270, 0.136)),
        material=dark_steel,
        name="right_axle_bracket",
    )
    body.visual(
        Box((0.188, 0.080, 0.118)),
        origin=Origin(xyz=(0.0, -0.286, 0.110)),
        material=dark_steel,
        name="center_axle_saddle",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(-0.280, -0.300, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_wheel_collar",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.280, -0.300, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_wheel_collar",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(-0.220, -0.366, 0.970), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.000, -0.366, 0.970), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.220, -0.366, 0.970), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    body.visual(
        Box((0.100, 0.054, 0.050)),
        origin=Origin(xyz=(-0.180, 0.296, 0.025)),
        material=wear_black,
        name="left_skid_pad",
    )
    body.visual(
        Box((0.100, 0.054, 0.050)),
        origin=Origin(xyz=(0.180, 0.296, 0.025)),
        material=wear_black,
        name="right_skid_pad",
    )
    body.visual(
        Box((0.340, 0.048, 0.060)),
        origin=Origin(xyz=(0.0, 0.300, 0.070)),
        material=wear_black,
        name="front_bumper",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.650, 0.800, 1.000)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.596, 0.750, 0.008)),
        origin=Origin(xyz=(0.0, 0.375, 0.022)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.016, 0.750, 0.048)),
        origin=Origin(xyz=(-0.306, 0.375, -0.002)),
        material=lid_green,
        name="left_side_skirt",
    )
    lid.visual(
        Box((0.016, 0.750, 0.048)),
        origin=Origin(xyz=(0.306, 0.375, -0.002)),
        material=lid_green,
        name="right_side_skirt",
    )
    lid.visual(
        Box((0.578, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.751, -0.001)),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.078, 0.110, 0.024)),
        origin=Origin(xyz=(-0.110, 0.078, -0.010)),
        material=lid_green,
        name="left_hinge_gusset",
    )
    lid.visual(
        Box((0.260, 0.080, 0.022)),
        origin=Origin(xyz=(0.0, 0.104, -0.009)),
        material=lid_green,
        name="center_service_stiffener",
    )
    lid.visual(
        Box((0.078, 0.110, 0.024)),
        origin=Origin(xyz=(0.110, 0.078, -0.010)),
        material=lid_green,
        name="right_hinge_gusset",
    )
    lid.visual(
        Box((0.036, 0.540, 0.024)),
        origin=Origin(xyz=(0.0, 0.360, 0.006)),
        material=lid_green,
        name="center_rib",
    )
    lid.visual(
        Box((0.028, 0.480, 0.022)),
        origin=Origin(xyz=(-0.155, 0.350, 0.007)),
        material=lid_green,
        name="left_rib",
    )
    lid.visual(
        Box((0.028, 0.480, 0.022)),
        origin=Origin(xyz=(0.155, 0.350, 0.007)),
        material=lid_green,
        name="right_rib",
    )
    lid.visual(
        Box((0.100, 0.040, 0.026)),
        origin=Origin(xyz=(-0.110, 0.032, 0.007)),
        material=steel,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.100, 0.040, 0.026)),
        origin=Origin(xyz=(0.110, 0.032, 0.007)),
        material=steel,
        name="right_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(-0.110, 0.048, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(xyz=(0.110, 0.048, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_hinge_knuckle",
    )
    lid.visual(
        Box((0.170, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.722, 0.026)),
        material=wear_black,
        name="front_pull_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.600, 0.720, 0.080)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.360, 0.000)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(tire_mesh, material=dark_rubber, name="tire")
    left_wheel.visual(
        Cylinder(radius=0.081, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rim_disc",
    )
    left_wheel.visual(
        Cylinder(radius=0.044, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub_sleeve",
    )
    left_wheel.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wear_black,
        name="hub_cap",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.054),
        mass=2.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(tire_mesh, material=dark_rubber, name="tire")
    right_wheel.visual(
        Cylinder(radius=0.081, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rim_disc",
    )
    right_wheel.visual(
        Cylinder(radius=0.044, length=0.070),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub_sleeve",
    )
    right_wheel.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wear_black,
        name="hub_cap",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.054),
        mass=2.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.366, 0.970)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=1.72,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.330, -0.300, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=18.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(0.330, -0.300, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_wheel_spin = object_model.get_articulation("body_to_left_wheel")
    right_wheel_spin = object_model.get_articulation("body_to_right_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        min_corner, max_corner = body_aabb
        body_width = max_corner[0] - min_corner[0]
        body_height = max_corner[2] - min_corner[2]
        ctx.check(
            "wheelie_bin_scale",
            0.55 <= body_width <= 0.66 and 0.94 <= body_height <= 1.02,
            f"Unexpected body envelope: width={body_width:.3f}, height={body_height:.3f}",
        )
    else:
        ctx.fail("wheelie_bin_scale", "Body AABB unavailable")

    ctx.check(
        "articulation_axes",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and left_wheel_spin.axis == (1.0, 0.0, 0.0)
        and right_wheel_spin.axis == (1.0, 0.0, 0.0),
        (
            f"Expected lid and wheel axes along +X; got "
            f"lid={lid_hinge.axis}, left={left_wheel_spin.axis}, right={right_wheel_spin.axis}"
        ),
    )

    ctx.expect_contact(
        left_wheel,
        body,
        elem_a=left_wheel.get_visual("hub_sleeve"),
        elem_b=body.get_visual("left_wheel_collar"),
        name="left_wheel_supported_on_collar",
    )
    ctx.expect_contact(
        right_wheel,
        body,
        elem_a=right_wheel.get_visual("hub_sleeve"),
        elem_b=body.get_visual("right_wheel_collar"),
        name="right_wheel_supported_on_collar",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem=lid.get_visual("front_skirt"),
            negative_elem=body.get_visual("front_rim_rail"),
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            name="lid_front_seats_on_rim",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            positive_elem=body.get_visual("left_rim_rail"),
            negative_elem=lid.get_visual("left_side_skirt"),
            min_gap=0.001,
            max_gap=0.006,
            name="lid_left_side_clearance",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem=lid.get_visual("right_side_skirt"),
            negative_elem=body.get_visual("right_rim_rail"),
            min_gap=0.001,
            max_gap=0.006,
            name="lid_right_side_clearance",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.45,
            name="lid_covers_bin_opening",
        )

    with ctx.pose({lid_hinge: 1.25}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid.get_visual("front_skirt"),
            negative_elem=body.get_visual("front_rim_rail"),
            min_gap=0.25,
            name="lid_front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
