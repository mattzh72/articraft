from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_router")

    shell_gray = model.material("shell_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.08, 0.09, 0.10, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.06, 0.07, 0.08, 1.0))
    port_black = model.material("port_black", rgba=(0.04, 0.04, 0.05, 1.0))
    port_metal = model.material("port_metal", rgba=(0.65, 0.67, 0.71, 1.0))
    status_blue = model.material("status_blue", rgba=(0.30, 0.75, 0.96, 1.0))

    body_width = 0.236
    body_height = 0.168
    body_depth = 0.032
    body_corner = 0.018

    fascia_width = 0.214
    fascia_height = 0.146
    fascia_depth = 0.004

    antenna_width = 0.017
    antenna_length = 0.126
    antenna_thickness = 0.0032

    cover_width = 0.072
    cover_height = 0.104
    cover_thickness = 0.003
    cover_axis_x = -0.046
    cover_axis_y = -0.029
    cover_axis_z = -0.012

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    body_mesh = _mesh(
        "router_body_shell",
        ExtrudeGeometry.centered(
            rounded_rect_profile(body_width, body_height, body_corner, corner_segments=10),
            body_depth,
            cap=True,
            closed=True,
        ),
    )
    fascia_mesh = _mesh(
        "router_front_fascia",
        ExtrudeGeometry.centered(
            rounded_rect_profile(fascia_width, fascia_height, 0.014, corner_segments=10),
            fascia_depth,
            cap=True,
            closed=True,
        ),
    )
    antenna_mesh = _mesh(
        "router_antenna_blade",
        ExtrudeGeometry.centered(
            rounded_rect_profile(antenna_width, antenna_length, 0.0065, corner_segments=8),
            antenna_thickness,
            cap=True,
            closed=True,
        ),
    )
    cover_mesh = _mesh(
        "router_cable_cover_panel",
        ExtrudeGeometry.centered(
            rounded_rect_profile(cover_width, cover_height, 0.008, corner_segments=8),
            cover_thickness,
            cap=True,
            closed=True,
        ),
    )

    body = model.part("router_body")
    body.visual(
        body_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_gray,
        name="body_shell",
    )
    body.visual(
        fascia_mesh,
        origin=Origin(
            xyz=(0.0, (body_depth * 0.5) - (fascia_depth * 0.5) + 0.0006, 0.002),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=fascia_black,
        name="front_fascia",
    )
    body.visual(
        Box((0.092, 0.0026, 0.004)),
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.0001, 0.048)),
        material=status_blue,
        name="status_bar",
    )
    body.visual(
        Box((0.018, 0.0018, 0.010)),
        origin=Origin(xyz=(0.0, (body_depth * 0.5) + 0.0001, -0.050)),
        material=port_metal,
        name="badge",
    )
    body.visual(
        Box((0.018, 0.006, 0.052)),
        origin=Origin(xyz=(-0.070, -0.013, 0.028)),
        material=shell_gray,
        name="wall_standoff_left",
    )
    body.visual(
        Box((0.018, 0.006, 0.052)),
        origin=Origin(xyz=(0.070, -0.013, 0.028)),
        material=shell_gray,
        name="wall_standoff_right",
    )
    body.visual(
        Box((0.078, 0.009, 0.104)),
        origin=Origin(xyz=(-0.003, -0.0205, cover_axis_z)),
        material=shell_gray,
        name="port_housing",
    )
    body.visual(
        Box((0.010, 0.009, 0.104)),
        origin=Origin(xyz=(-0.044, -0.0245, cover_axis_z)),
        material=shell_gray,
        name="cover_hinge_spine",
    )
    for index, x_pos in enumerate((-0.020, 0.000, 0.020)):
        body.visual(
            Box((0.014, 0.006, 0.012)),
            origin=Origin(xyz=(x_pos, -0.0220, -0.004)),
            material=port_black,
            name=f"ethernet_port_{index}",
        )
    body.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.033, -0.0215, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=port_black,
        name="power_jack",
    )
    body.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(0.0325, -0.0190, 0.018)),
        material=port_metal,
        name="power_trim",
    )
    for side_name, x_pos in (("left", -0.062), ("right", 0.062)):
        body.visual(
            Box((0.022, 0.012, 0.010)),
            origin=Origin(xyz=(x_pos, 0.0075, -(body_height * 0.5) + 0.003)),
            material=shell_gray,
            name=f"antenna_pod_{side_name}",
        )
        body.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(
                xyz=(x_pos, 0.0100, -(body_height * 0.5) + 0.004),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=fascia_black,
            name=f"antenna_barrel_{side_name}",
        )
    body.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(cover_axis_x, cover_axis_y, cover_axis_z + 0.035)),
        material=shell_gray,
        name="cover_hinge_upper",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(cover_axis_x, cover_axis_y, cover_axis_z - 0.035)),
        material=shell_gray,
        name="cover_hinge_lower",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.05,
        origin=Origin(),
    )

    left_antenna = model.part("left_antenna")
    for index, x_offset in enumerate((-0.006, 0.006)):
        left_antenna.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=antenna_black,
            name=f"left_knuckle_{index}",
        )
    left_antenna.visual(
        Box((0.020, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.0040, -0.0260)),
        material=antenna_black,
        name="left_root_bridge",
    )
    left_antenna.visual(
        antenna_mesh,
        origin=Origin(xyz=(0.0, 0.0015, -0.073), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=antenna_black,
        name="blade",
    )
    left_antenna.visual(
        Box((0.010, 0.0014, 0.070)),
        origin=Origin(xyz=(0.0, 0.0032, -0.078)),
        material=fascia_black,
        name="left_center_rib",
    )
    left_antenna.inertial = Inertial.from_geometry(
        Box((0.020, 0.010, 0.136)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.001, -0.068)),
    )

    right_antenna = model.part("right_antenna")
    for index, x_offset in enumerate((-0.006, 0.006)):
        right_antenna.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=antenna_black,
            name=f"right_knuckle_{index}",
        )
    right_antenna.visual(
        Box((0.020, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.0040, -0.0260)),
        material=antenna_black,
        name="right_root_bridge",
    )
    right_antenna.visual(
        antenna_mesh,
        origin=Origin(xyz=(0.0, 0.0015, -0.073), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=antenna_black,
        name="blade",
    )
    right_antenna.visual(
        Box((0.010, 0.0014, 0.070)),
        origin=Origin(xyz=(0.0, 0.0032, -0.078)),
        material=fascia_black,
        name="right_center_rib",
    )
    right_antenna.inertial = Inertial.from_geometry(
        Box((0.020, 0.010, 0.136)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.001, -0.068)),
    )

    cable_cover = model.part("cable_cover")
    cable_cover.visual(
        Cylinder(radius=0.004, length=0.044),
        origin=Origin(),
        material=shell_gray,
        name="hinge_knuckle",
    )
    cable_cover.visual(
        Box((0.012, 0.004, 0.064)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=shell_gray,
        name="hinge_bridge",
    )
    cable_cover.visual(
        cover_mesh,
        origin=Origin(
            xyz=(0.040, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell_gray,
        name="cover_panel",
    )
    cable_cover.visual(
        Box((0.008, 0.006, 0.026)),
        origin=Origin(xyz=(0.079, -0.0005, -0.030)),
        material=fascia_black,
        name="finger_lip",
    )
    cable_cover.inertial = Inertial.from_geometry(
        Box((0.086, 0.008, 0.104)),
        mass=0.09,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
    )

    model.articulation(
        "left_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_antenna,
        origin=Origin(xyz=(-0.062, 0.010, -(body_height * 0.5) + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.4,
            lower=-0.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "right_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_antenna,
        origin=Origin(xyz=(0.062, 0.010, -(body_height * 0.5) + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.4,
            lower=-0.15,
            upper=1.15,
        ),
    )
    model.articulation(
        "cable_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cable_cover,
        origin=Origin(xyz=(cover_axis_x, cover_axis_y, cover_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.9,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("router_body")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    cable_cover = object_model.get_part("cable_cover")
    left_hinge = object_model.get_articulation("left_antenna_hinge")
    right_hinge = object_model.get_articulation("right_antenna_hinge")
    cover_hinge = object_model.get_articulation("cable_cover_hinge")

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

    ctx.check(
        "key_parts_present",
        all(part is not None for part in (body, left_antenna, right_antenna, cable_cover)),
        "router body, both antennas, and cable cover must all exist",
    )
    ctx.check(
        "articulation_axes",
        tuple(left_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(right_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(cover_hinge.axis) == (0.0, 0.0, 1.0),
        f"unexpected axes: left={left_hinge.axis}, right={right_hinge.axis}, cover={cover_hinge.axis}",
    )

    ctx.expect_contact(left_antenna, body, name="left_antenna_attached")
    ctx.expect_contact(right_antenna, body, name="right_antenna_attached")
    ctx.expect_contact(cable_cover, body, name="cover_attached_closed")
    ctx.expect_origin_gap(body, left_antenna, axis="z", min_gap=0.079, max_gap=0.081, name="left_antenna_on_lower_edge")
    ctx.expect_origin_gap(body, right_antenna, axis="z", min_gap=0.079, max_gap=0.081, name="right_antenna_on_lower_edge")
    ctx.expect_origin_distance(
        left_antenna,
        right_antenna,
        axes="x",
        min_dist=0.120,
        max_dist=0.126,
        name="antenna_spacing",
    )
    ctx.expect_origin_gap(
        body,
        cable_cover,
        axis="y",
        min_gap=0.028,
        max_gap=0.0305,
        name="cover_is_on_rear_face",
    )
    ctx.expect_within(
        cable_cover,
        body,
        axes="xz",
        margin=0.020,
        inner_elem="cover_panel",
        outer_elem="body_shell",
        name="cover_within_router_footprint",
    )
    ctx.expect_overlap(
        cable_cover,
        body,
        axes="z",
        min_overlap=0.090,
        elem_a="cover_panel",
        elem_b="body_shell",
        name="cover_tall_enough_for_port_area",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    left_blade_closed = _aabb_center(ctx.part_element_world_aabb(left_antenna, elem="blade"))
    right_blade_closed = _aabb_center(ctx.part_element_world_aabb(right_antenna, elem="blade"))
    cover_panel_closed = _aabb_center(ctx.part_element_world_aabb(cable_cover, elem="cover_panel"))

    with ctx.pose(
        {
            left_hinge: 0.95,
            right_hinge: 0.95,
            cover_hinge: 1.10,
        }
    ):
        ctx.expect_contact(cable_cover, body, name="cover_attached_open")
        left_blade_open = _aabb_center(ctx.part_element_world_aabb(left_antenna, elem="blade"))
        right_blade_open = _aabb_center(ctx.part_element_world_aabb(right_antenna, elem="blade"))
        cover_panel_open = _aabb_center(ctx.part_element_world_aabb(cable_cover, elem="cover_panel"))

    ctx.check(
        "left_antenna_folds_forward",
        left_blade_closed is not None
        and left_blade_open is not None
        and (left_blade_open[1] > left_blade_closed[1] + 0.045)
        and (left_blade_open[2] > left_blade_closed[2] + 0.020),
        f"left antenna centers closed/open: {left_blade_closed} -> {left_blade_open}",
    )
    ctx.check(
        "right_antenna_folds_forward",
        right_blade_closed is not None
        and right_blade_open is not None
        and (right_blade_open[1] > right_blade_closed[1] + 0.045)
        and (right_blade_open[2] > right_blade_closed[2] + 0.020),
        f"right antenna centers closed/open: {right_blade_closed} -> {right_blade_open}",
    )
    ctx.check(
        "cover_swings_open",
        cover_panel_closed is not None
        and cover_panel_open is not None
        and (cover_panel_open[0] < cover_panel_closed[0] - 0.015)
        and (cover_panel_open[1] > cover_panel_closed[1] + 0.030),
        f"cover panel centers closed/open: {cover_panel_closed} -> {cover_panel_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
