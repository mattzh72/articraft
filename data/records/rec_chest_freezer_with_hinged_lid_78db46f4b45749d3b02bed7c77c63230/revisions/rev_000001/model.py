from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chest_drink_cooler")

    cooler_white = model.material("cooler_white", rgba=(0.90, 0.92, 0.93, 1.0))
    lid_blue = model.material("lid_blue", rgba=(0.16, 0.31, 0.49, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.76, 0.79, 0.81, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.15, 0.17, 0.18, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.71, 0.74, 0.77, 1.0))

    body_length = 0.76
    body_width = 0.42
    body_height = 0.44
    wall = 0.028
    floor_thickness = 0.030
    wall_height = body_height - floor_thickness
    rail_length = 0.69
    rail_width = 0.020
    rail_height = 0.018
    inner_length = body_length - 2.0 * wall
    inner_width = body_width - 2.0 * wall

    lid_length = 0.72
    lid_width = 0.44
    lid_top_thickness = 0.040
    lid_runner_width = 0.024
    lid_runner_thickness = 0.010
    lid_channel_wall = 0.010
    lid_channel_drop = 0.028
    lid_plug_length = 0.66
    lid_plug_width = 0.31
    lid_plug_thickness = 0.034
    rail_top_z = body_height + rail_height

    handle_span = 0.61
    handle_radius = 0.007
    handle_axis_z = 0.062

    body = model.part("body")
    body.visual(
        Box((body_length - 0.02, body_width - 0.04, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=cooler_white,
        name="floor",
    )
    body.visual(
        Box((body_length, wall, wall_height)),
        origin=Origin(xyz=(0.0, body_width * 0.5 - wall * 0.5, floor_thickness + wall_height * 0.5)),
        material=cooler_white,
        name="front_wall",
    )
    body.visual(
        Box((body_length, wall, wall_height)),
        origin=Origin(xyz=(0.0, -body_width * 0.5 + wall * 0.5, floor_thickness + wall_height * 0.5)),
        material=cooler_white,
        name="rear_wall",
    )
    body.visual(
        Box((wall, inner_width, wall_height)),
        origin=Origin(xyz=(body_length * 0.5 - wall * 0.5, 0.0, floor_thickness + wall_height * 0.5)),
        material=cooler_white,
        name="right_wall",
    )
    body.visual(
        Box((wall, inner_width, wall_height)),
        origin=Origin(xyz=(-body_length * 0.5 + wall * 0.5, 0.0, floor_thickness + wall_height * 0.5)),
        material=cooler_white,
        name="left_wall",
    )

    corner_radius = wall * 0.70
    corner_z = floor_thickness + wall_height * 0.5
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            corner_name = (
                f"{'right' if x_sign > 0.0 else 'left'}_"
                f"{'front' if y_sign > 0.0 else 'rear'}_corner"
            )
            body.visual(
                Cylinder(radius=corner_radius, length=wall_height),
                origin=Origin(
                    xyz=(
                        x_sign * (body_length * 0.5 - wall * 0.15),
                        y_sign * (body_width * 0.5 - wall * 0.15),
                        corner_z,
                    )
                ),
                material=cooler_white,
                name=corner_name,
            )

    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(0.0, inner_width * 0.5, body_height + rail_height * 0.5)),
        material=dark_trim,
        name="left_rail",
    )
    body.visual(
        Box((rail_length, rail_width, rail_height)),
        origin=Origin(xyz=(0.0, -inner_width * 0.5, body_height + rail_height * 0.5)),
        material=dark_trim,
        name="right_rail",
    )
    for x_sign in (-1.0, 1.0):
        stop_name_left = f"{'front' if x_sign > 0.0 else 'rear'}_left_stop"
        stop_name_right = f"{'front' if x_sign > 0.0 else 'rear'}_right_stop"
        stop_x = x_sign * (rail_length * 0.5 - 0.015)
        body.visual(
            Box((0.030, rail_width, rail_height)),
            origin=Origin(xyz=(stop_x, inner_width * 0.5, body_height + rail_height * 0.5)),
            material=dark_trim,
            name=stop_name_left,
        )
        body.visual(
            Box((0.030, rail_width, rail_height)),
            origin=Origin(xyz=(stop_x, -inner_width * 0.5, body_height + rail_height * 0.5)),
            material=dark_trim,
            name=stop_name_right,
        )

    body.visual(
        Box((inner_length - 0.02, inner_width - 0.02, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness + 0.005)),
        material=liner_gray,
        name="liner_floor",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_length, lid_width, lid_top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=lid_blue,
        name="lid_shell",
    )
    lid.visual(
        Box((lid_plug_length, lid_plug_width, lid_plug_thickness)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=liner_gray,
        name="lid_plug",
    )
    lid.visual(
        Box((rail_length - 0.01, lid_runner_width, lid_runner_thickness)),
        origin=Origin(xyz=(0.0, inner_width * 0.5, lid_runner_thickness * 0.5)),
        material=dark_trim,
        name="left_guide",
    )
    lid.visual(
        Box((rail_length - 0.01, lid_runner_width, lid_runner_thickness)),
        origin=Origin(xyz=(0.0, -inner_width * 0.5, lid_runner_thickness * 0.5)),
        material=dark_trim,
        name="right_guide",
    )

    channel_outer_y = inner_width * 0.5 + (rail_width + lid_channel_wall) * 0.5 + 0.004
    channel_inner_y = inner_width * 0.5 - (rail_width + lid_channel_wall) * 0.5 - 0.004
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        lid.visual(
            Box((rail_length - 0.008, lid_channel_wall, lid_channel_drop)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_sign * channel_outer_y,
                    0.010 - lid_channel_drop * 0.5,
                )
            ),
            material=lid_blue,
            name=f"{side_name}_outer_skirt",
        )
        lid.visual(
            Box((rail_length - 0.008, lid_channel_wall, lid_channel_drop)),
            origin=Origin(
                xyz=(
                    0.0,
                    y_sign * channel_inner_y,
                    0.010 - lid_channel_drop * 0.5,
                )
            ),
            material=lid_blue,
            name=f"{side_name}_inner_skirt",
        )

    pivot_block_size = (0.028, 0.026, 0.010)
    for x_sign in (-1.0, 1.0):
        end_name = "front" if x_sign > 0.0 else "rear"
        lid.visual(
            Box(pivot_block_size),
            origin=Origin(
                xyz=(
                    x_sign * 0.341,
                    0.0,
                    0.053,
                )
            ),
            material=dark_trim,
            name=f"{end_name}_pivot_pedestal",
        )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.085)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    handle = model.part("handle")
    handle_path = wire_from_points(
        [
            (-0.341, 0.0, 0.0),
            (-0.320, 0.024, 0.018),
            (-0.250, 0.086, 0.052),
            (-0.115, 0.146, 0.074),
            (0.115, 0.146, 0.074),
            (0.250, 0.086, 0.052),
            (0.320, 0.024, 0.018),
            (0.341, 0.0, 0.0),
        ],
        radius=handle_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=12,
    )
    handle.visual(
        mesh_from_geometry(handle_path, "cooler_handle_bail"),
        material=handle_steel,
        name="handle_bail",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.66, 0.18, 0.09)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.08, 0.03)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, rail_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.30, lower=0.0, upper=0.20),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, handle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    body_to_lid = object_model.get_articulation("body_to_lid")
    lid_to_handle = object_model.get_articulation("lid_to_handle")

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

    ctx.expect_contact(lid, body, elem_a="left_guide", elem_b="left_rail")
    ctx.expect_contact(lid, body, elem_a="right_guide", elem_b="right_rail")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="front_wall",
        min_gap=0.02,
        max_gap=0.05,
        name="lid_sits_on_top_of_body",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.32, name="lid_covers_cooler_opening")

    lid_rest = ctx.part_world_position(lid)
    assert lid_rest is not None
    ctx.check(
        "lid_centered_side_to_side",
        abs(lid_rest[1]) < 1e-6,
        details=f"lid_world_y={lid_rest[1]:.6f}",
    )
    ctx.expect_overlap(handle, lid, axes="x", min_overlap=0.60, name="handle_spans_lid_width")
    with ctx.pose({body_to_lid: 0.18}):
        lid_open = ctx.part_world_position(lid)
        assert lid_open is not None
        ctx.check(
            "lid_slides_along_length",
            lid_open[0] > lid_rest[0] + 0.16 and abs(lid_open[1] - lid_rest[1]) < 1e-6 and abs(lid_open[2] - lid_rest[2]) < 1e-6,
            details=f"rest={lid_rest}, open={lid_open}",
        )
        ctx.expect_contact(lid, body, elem_a="left_guide", elem_b="left_rail")
        ctx.expect_contact(lid, body, elem_a="right_guide", elem_b="right_rail")

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_bail")
    assert handle_rest is not None
    with ctx.pose({lid_to_handle: 1.15}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="handle_bail")
        assert handle_raised is not None
        ctx.check(
            "handle_swings_up_from_lid",
            handle_raised[1][2] > handle_rest[1][2] + 0.10,
            details=f"rest_max_z={handle_rest[1][2]:.4f}, raised_max_z={handle_raised[1][2]:.4f}",
        )
        ctx.expect_gap(
            handle,
            lid,
            axis="z",
            positive_elem="handle_bail",
            negative_elem="lid_shell",
            min_gap=0.005,
            name="raised_handle_clears_lid",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
