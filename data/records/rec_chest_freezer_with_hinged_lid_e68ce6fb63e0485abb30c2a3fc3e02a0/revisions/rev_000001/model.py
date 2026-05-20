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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_chest_body_bag_freezer")

    freezer_white = model.material("freezer_white", rgba=(0.96, 0.97, 0.98, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.82, 0.85, 0.88, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    body_length = 2.25
    body_width = 0.82
    body_height = 0.90
    shell_thickness = 0.04
    base_height = 0.08
    liner_thickness = 0.015
    liner_height = 0.76

    lid_length = 2.27
    lid_depth = 0.844
    lid_thickness = 0.07
    lid_axis_y = -(body_width * 0.5) - 0.016
    lid_axis_z = body_height - 0.016
    lid_panel_center_y = 0.438
    lid_panel_center_z = 0.051
    lid_plug_length = 2.13
    lid_plug_depth = 0.70
    lid_plug_height = 0.06
    lid_crown_length = 2.05
    lid_crown_depth = 0.76
    hinge_radius = 0.016
    hinge_body_length = 0.06
    hinge_lid_length = 0.04
    hinge_centers = (-0.72, 0.72)
    hinge_lid_offset = 0.055

    handle_span = 1.30
    handle_drop = 0.10
    handle_hinge_y = 0.874
    handle_hinge_z = 0.024
    handle_anchor_x = 0.672
    handle_knuckle_length = 0.024
    handle_knuckle_radius = 0.010
    handle_bar_radius = 0.012

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    side_wall_length = body_length
    side_wall_width = shell_thickness
    side_wall_height = body_height
    front_back_length = body_length - (2.0 * shell_thickness)
    front_back_width = shell_thickness

    body.visual(
        Box((side_wall_length, side_wall_width, side_wall_height)),
        origin=Origin(
            xyz=(0.0, (body_width * 0.5) - (shell_thickness * 0.5), body_height * 0.5)
        ),
        material=freezer_white,
        name="front_wall",
    )
    body.visual(
        Box((side_wall_length, side_wall_width, side_wall_height)),
        origin=Origin(
            xyz=(0.0, -(body_width * 0.5) + (shell_thickness * 0.5), body_height * 0.5)
        ),
        material=freezer_white,
        name="rear_wall",
    )
    body.visual(
        Box((shell_thickness, body_width - (2.0 * shell_thickness), body_height)),
        origin=Origin(
            xyz=((body_length * 0.5) - (shell_thickness * 0.5), 0.0, body_height * 0.5)
        ),
        material=freezer_white,
        name="right_wall",
    )
    body.visual(
        Box((shell_thickness, body_width - (2.0 * shell_thickness), body_height)),
        origin=Origin(
            xyz=(-(body_length * 0.5) + (shell_thickness * 0.5), 0.0, body_height * 0.5)
        ),
        material=freezer_white,
        name="left_wall",
    )
    body.visual(
        Box((body_length, body_width, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=trim_gray,
        name="base_pan",
    )
    body.visual(
        Box((body_length * 0.96, body_width * 0.14, 0.11)),
        origin=Origin(xyz=(0.0, (body_width * 0.5) - 0.035, 0.055)),
        material=trim_gray,
        name="front_kick_plate",
    )

    inner_length = body_length - (2.0 * shell_thickness)
    inner_width = body_width - (2.0 * shell_thickness)
    inner_clear_length = inner_length - (2.0 * liner_thickness)
    inner_clear_width = inner_width - (2.0 * liner_thickness)
    liner_center_z = base_height + (liner_height * 0.5)

    body.visual(
        Box((inner_length, liner_thickness, liner_height)),
        origin=Origin(
            xyz=(0.0, (inner_width * 0.5) - (liner_thickness * 0.5), liner_center_z)
        ),
        material=liner_gray,
        name="inner_front_liner",
    )
    body.visual(
        Box((inner_length, liner_thickness, liner_height)),
        origin=Origin(
            xyz=(0.0, -(inner_width * 0.5) + (liner_thickness * 0.5), liner_center_z)
        ),
        material=liner_gray,
        name="inner_rear_liner",
    )
    body.visual(
        Box((liner_thickness, inner_width - (2.0 * liner_thickness), liner_height)),
        origin=Origin(
            xyz=((inner_length * 0.5) - (liner_thickness * 0.5), 0.0, liner_center_z)
        ),
        material=liner_gray,
        name="inner_right_liner",
    )
    body.visual(
        Box((liner_thickness, inner_width - (2.0 * liner_thickness), liner_height)),
        origin=Origin(
            xyz=(-(inner_length * 0.5) + (liner_thickness * 0.5), 0.0, liner_center_z)
        ),
        material=liner_gray,
        name="inner_left_liner",
    )
    body.visual(
        Box((inner_clear_length, inner_clear_width, liner_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_height + (liner_thickness * 0.5))),
        material=liner_gray,
        name="liner_floor",
    )

    for hinge_index, hinge_x in enumerate(hinge_centers, start=1):
        body.visual(
            Cylinder(radius=hinge_radius, length=hinge_body_length),
            origin=_x_cylinder_origin(hinge_x, lid_axis_y, lid_axis_z),
            material=trim_gray,
            name=f"rear_barrel_hinge_{hinge_index}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_depth, lid_thickness + lid_plug_height)),
        mass=54.0,
        origin=Origin(xyz=(0.0, lid_panel_center_y, 0.025)),
    )
    lid.visual(
        Box((lid_length, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_center_z)),
        material=freezer_white,
        name="lid_outer_panel",
    )
    lid.visual(
        Box((lid_crown_length, lid_crown_depth, 0.012)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_center_z + 0.041)),
        material=freezer_white,
        name="lid_crown",
    )
    lid.visual(
        Box((lid_plug_length, lid_plug_depth, lid_plug_height)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, -0.014)),
        material=liner_gray,
        name="lid_plug",
    )
    lid.visual(
        Box((lid_plug_length, 0.018, 0.01)),
        origin=Origin(xyz=(0.0, lid_panel_center_y + (lid_plug_depth * 0.5) - 0.009, 0.011)),
        material=gasket_dark,
        name="front_gasket_strip",
    )
    lid.visual(
        Box((lid_plug_length, 0.018, 0.01)),
        origin=Origin(xyz=(0.0, lid_panel_center_y - (lid_plug_depth * 0.5) + 0.009, 0.011)),
        material=gasket_dark,
        name="rear_gasket_strip",
    )
    lid.visual(
        Box((0.018, lid_plug_depth - 0.036, 0.01)),
        origin=Origin(xyz=((lid_plug_length * 0.5) - 0.009, lid_panel_center_y, 0.011)),
        material=gasket_dark,
        name="right_gasket_strip",
    )
    lid.visual(
        Box((0.018, lid_plug_depth - 0.036, 0.01)),
        origin=Origin(xyz=(-(lid_plug_length * 0.5) + 0.009, lid_panel_center_y, 0.011)),
        material=gasket_dark,
        name="left_gasket_strip",
    )

    for hinge_index, hinge_x in enumerate(hinge_centers, start=1):
        lid.visual(
            Cylinder(radius=hinge_radius, length=hinge_lid_length),
            origin=_x_cylinder_origin(hinge_x - hinge_lid_offset, 0.0, 0.0),
            material=trim_gray,
            name=f"lid_hinge_{hinge_index}_left_knuckle",
        )
        lid.visual(
            Box((hinge_lid_length, 0.024, 0.024)),
            origin=Origin(xyz=(hinge_x - hinge_lid_offset, 0.010, 0.028)),
            material=trim_gray,
            name=f"lid_hinge_{hinge_index}_left_bridge",
        )
        lid.visual(
            Cylinder(radius=hinge_radius, length=hinge_lid_length),
            origin=_x_cylinder_origin(hinge_x + hinge_lid_offset, 0.0, 0.0),
            material=trim_gray,
            name=f"lid_hinge_{hinge_index}_right_knuckle",
        )
        lid.visual(
            Box((hinge_lid_length, 0.024, 0.024)),
            origin=Origin(xyz=(hinge_x + hinge_lid_offset, 0.010, 0.028)),
            material=trim_gray,
            name=f"lid_hinge_{hinge_index}_right_bridge",
        )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        anchor_x = sign * handle_anchor_x
        knuckle_outer_x = sign * (handle_anchor_x + handle_knuckle_length)
        knuckle_inner_x = sign * (handle_anchor_x - handle_knuckle_length)
        lid.visual(
            Cylinder(radius=handle_knuckle_radius, length=handle_knuckle_length),
            origin=_x_cylinder_origin(knuckle_outer_x, handle_hinge_y, handle_hinge_z),
            material=stainless,
            name=f"handle_hinge_{side_name}_outer_knuckle",
        )
        lid.visual(
            Cylinder(radius=handle_knuckle_radius, length=handle_knuckle_length),
            origin=_x_cylinder_origin(knuckle_inner_x, handle_hinge_y, handle_hinge_z),
            material=stainless,
            name=f"handle_hinge_{side_name}_inner_knuckle",
        )
        lid.visual(
            Box((0.10, 0.014, 0.018)),
            origin=Origin(xyz=(anchor_x, 0.851, handle_hinge_z)),
            material=stainless,
            name=f"handle_hinge_{side_name}_mount_pad",
        )
        lid.visual(
            Box((0.024, 0.024, 0.022)),
            origin=Origin(xyz=(knuckle_outer_x, 0.862, handle_hinge_z)),
            material=stainless,
            name=f"handle_hinge_{side_name}_outer_web",
        )
        lid.visual(
            Box((0.024, 0.024, 0.022)),
            origin=Origin(xyz=(knuckle_inner_x, 0.862, handle_hinge_z)),
            material=stainless,
            name=f"handle_hinge_{side_name}_inner_web",
        )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((1.42, 0.16, 0.14)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.04, -0.05)),
    )
    handle.visual(
        Cylinder(radius=handle_bar_radius, length=handle_span),
        origin=_x_cylinder_origin(0.0, 0.072, -0.099),
        material=stainless,
        name="grab_bar",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        anchor_x = sign * handle_anchor_x
        handle.visual(
            Cylinder(radius=handle_knuckle_radius, length=handle_knuckle_length),
            origin=_x_cylinder_origin(anchor_x, 0.0, 0.0),
            material=stainless,
            name=f"handle_{side_name}_knuckle",
        )
        handle.visual(
            Box((0.052, 0.040, 0.094)),
            origin=Origin(xyz=(sign * 0.645, 0.028, -0.060)),
            material=stainless,
            name=f"handle_{side_name}_strap",
        )
        handle.visual(
            Box((0.028, 0.018, 0.016)),
            origin=Origin(xyz=(anchor_x, 0.0, -0.008)),
            material=stainless,
            name=f"handle_{side_name}_hinge_block",
        )
        handle.visual(
            Box((0.060, 0.016, 0.020)),
            origin=Origin(xyz=(sign * 0.635, 0.054, -0.088)),
            material=stainless,
            name=f"handle_{side_name}_bar_clamp",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, lid_axis_y, lid_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=1.32,
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, handle_hinge_y, handle_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_hinge = object_model.get_articulation("lid_to_handle")

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
        "hinges_rotate_about_long_axis",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0) and tuple(handle_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"lid axis={lid_hinge.axis}, handle axis={handle_hinge.axis}",
    )
    ctx.check(
        "rear_barrel_hinges_present",
        len(
            [
                body.get_visual("rear_barrel_hinge_1"),
                body.get_visual("rear_barrel_hinge_2"),
                lid.get_visual("lid_hinge_1_left_knuckle"),
                lid.get_visual("lid_hinge_1_right_knuckle"),
                lid.get_visual("lid_hinge_2_left_knuckle"),
                lid.get_visual("lid_hinge_2_right_knuckle"),
            ]
        )
        == 6,
        details="Expected two rear barrel hinge assemblies with interleaved lid knuckles.",
    )

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    body_dims = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "deep_chest_proportions",
        body_dims[0] > 2.1 and 0.80 <= body_dims[1] <= 0.90 and 0.85 <= body_dims[2] <= 0.92,
        details=f"body dims={body_dims}",
    )

    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_outer_panel",
        elem_b="front_wall",
        name="lid_seats_on_body_front_lip",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="lid_outer_panel",
        elem_b="front_wall",
        min_overlap=2.15,
        name="lid_runs_full_length",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a="handle_left_knuckle",
        elem_b="handle_hinge_left_inner_knuckle",
        name="handle_mounts_to_lid_hinge_knuckle",
    )

    lid_rest = ctx.part_element_world_aabb(lid, elem="lid_outer_panel")
    handle_rest = ctx.part_element_world_aabb(handle, elem="grab_bar")
    assert lid_rest is not None
    assert handle_rest is not None

    with ctx.pose({lid_hinge: 1.18}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_outer_panel")
        assert lid_open is not None
        ctx.check(
            "lid_opens_upward",
            lid_open[1][2] > lid_rest[1][2] + 0.55 and lid_open[1][1] < lid_rest[1][1] - 0.12,
            details=f"rest={lid_rest}, open={lid_open}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_lid_pose_clear_of_body")

    with ctx.pose({handle_hinge: 0.75}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="grab_bar")
        assert handle_raised is not None
        ctx.check(
            "handle_bar_pivots_upward",
            handle_raised[1][2] > handle_rest[1][2] + 0.06,
            details=f"rest={handle_rest}, raised={handle_raised}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_handle_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
