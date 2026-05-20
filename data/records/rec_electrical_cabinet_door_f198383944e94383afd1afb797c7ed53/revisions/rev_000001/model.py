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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _add_vertical_hinge_segments(
    part,
    *,
    axis_x: float,
    axis_y: float,
    segment_specs: tuple[tuple[float, float], ...],
    radius: float,
    material,
    name_prefix: str,
) -> None:
    for index, (z_start, length) in enumerate(segment_specs):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(axis_x, axis_y, z_start + (length * 0.5))),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def _side_wall_mesh(
    *,
    outer_x: float,
    inner_x: float,
    depth: float,
    base_height: float,
    front_height: float,
    back_height: float,
    logical_name: str,
):
    outer_loop = (
        (outer_x, -depth * 0.5, base_height),
        (outer_x, depth * 0.5, base_height),
        (outer_x, depth * 0.5, back_height),
        (outer_x, -depth * 0.5, front_height),
    )
    inner_loop = (
        (inner_x, -depth * 0.5, base_height),
        (inner_x, depth * 0.5, base_height),
        (inner_x, depth * 0.5, back_height),
        (inner_x, -depth * 0.5, front_height),
    )
    return mesh_from_geometry(section_loft([outer_loop, inner_loop]), logical_name)


def _add_door_and_panel(
    model: ArticulatedObject,
    *,
    side: str,
    x_sign: float,
    door_hinge_x: float,
    hinge_y: float,
    door_bottom: float,
    door_width: float,
    door_height: float,
    door_thickness: float,
    panel_bottom_from_door: float,
    panel_width: float,
    panel_height: float,
    panel_thickness: float,
    housing_material,
    hardware_material,
    panel_material,
) -> None:
    door = model.part(f"{side}_outer_door")
    panel = model.part(f"{side}_inner_panel")

    door_hinge_stile_width = 0.028
    panel_hinge_stile_width = 0.022
    door_mount_depth = 0.040
    panel_joint_y = door_mount_depth
    latch_x = x_sign * (door_hinge_stile_width + door_width - 0.036)
    latch_z = door_height * 0.53

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                x_sign * (door_hinge_stile_width + (door_width * 0.5)),
                -(door_thickness * 0.5),
                door_height * 0.5,
            )
        ),
        material=housing_material,
        name="door_leaf",
    )
    door.visual(
        Box((door_hinge_stile_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                x_sign * (door_hinge_stile_width * 0.5),
                -(door_thickness * 0.5),
                door_height * 0.5,
            )
        ),
        material=housing_material,
        name="hinge_stile",
    )
    door.visual(
        Box((0.030, 0.018, 0.240)),
        origin=Origin(xyz=(latch_x, -(door_thickness * 0.5) - 0.010, latch_z)),
        material=hardware_material,
        name="latch_bar",
    )
    door.visual(
        Box((0.050, 0.010, 0.085)),
        origin=Origin(xyz=(latch_x, -(door_thickness * 0.5) - 0.020, latch_z)),
        material=hardware_material,
        name="latch_pad",
    )
    door.visual(
        Box((0.024, door_mount_depth, panel_height)),
        origin=Origin(
            xyz=(
                x_sign * 0.012,
                door_mount_depth * 0.5,
                panel_bottom_from_door + (panel_height * 0.5),
            )
        ),
        material=housing_material,
        name="inner_panel_mount",
    )

    panel.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(
                x_sign * (panel_hinge_stile_width + (panel_width * 0.5)),
                panel_thickness * 0.5,
                panel_height * 0.5,
            )
        ),
        material=panel_material,
        name="panel_leaf",
    )
    panel.visual(
        Box((panel_hinge_stile_width, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(
                x_sign * (panel_hinge_stile_width * 0.5),
                panel_thickness * 0.5,
                panel_height * 0.5,
            )
        ),
        material=panel_material,
        name="hinge_stile",
    )
    panel.visual(
        Box((0.150, 0.003, 0.090)),
        origin=Origin(
            xyz=(
                x_sign * (panel_hinge_stile_width + (panel_width * 0.60)),
                panel_thickness - 0.0015,
                panel_height * 0.72,
            )
        ),
        material=hardware_material,
        name="warning_placard",
    )

    if x_sign > 0.0:
        outer_limits = MotionLimits(effort=30.0, velocity=1.2, lower=-2.0, upper=0.0)
        inner_limits = MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=1.5)
    else:
        outer_limits = MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=2.0)
        inner_limits = MotionLimits(effort=20.0, velocity=1.0, lower=-1.5, upper=0.0)

    model.articulation(
        f"{side}_outer_door_hinge",
        ArticulationType.REVOLUTE,
        parent="housing",
        child=door,
        origin=Origin(xyz=(door_hinge_x, hinge_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=outer_limits,
    )
    model.articulation(
        f"{side}_inner_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=panel,
        origin=Origin(xyz=(0.0, panel_joint_y, panel_bottom_from_door)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=inner_limits,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="padmount_transformer_enclosure")

    housing_material = model.material("housing_green", rgba=(0.355, 0.455, 0.315, 1.0))
    panel_material = model.material("panel_gray", rgba=(0.815, 0.825, 0.800, 1.0))
    hardware_material = model.material("hardware_dark", rgba=(0.150, 0.165, 0.160, 1.0))

    width = 1.82
    depth = 0.92
    base_height = 0.10
    front_height = 1.28
    back_height = 1.36
    wall_thickness = 0.018
    frame_depth = 0.030
    roof_thickness = 0.022

    side_jamb_width = 0.085
    center_post_width = 0.050
    sill_height = 0.130
    header_height = 0.090

    opening_width = (width - (2.0 * side_jamb_width) - center_post_width) * 0.5
    opening_height = front_height - sill_height - header_height
    door_reveal = 0.004
    door_width = opening_width - (door_reveal * 2.0)
    door_height = opening_height - (door_reveal * 2.0)
    door_bottom = sill_height + door_reveal
    door_thickness = 0.045

    panel_bottom_from_door = 0.070
    panel_width = door_width - 0.100
    panel_height = door_height - 0.160
    panel_thickness = 0.016

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=housing_material,
        name="base_plinth",
    )
    housing.visual(
        _side_wall_mesh(
            outer_x=-(width * 0.5),
            inner_x=-(width * 0.5) + wall_thickness,
            depth=depth,
            base_height=base_height,
            front_height=front_height,
            back_height=back_height,
            logical_name="left_side_wall",
        ),
        material=housing_material,
        name="left_side_wall",
    )
    housing.visual(
        _side_wall_mesh(
            outer_x=(width * 0.5) - wall_thickness,
            inner_x=(width * 0.5),
            depth=depth,
            base_height=base_height,
            front_height=front_height,
            back_height=back_height,
            logical_name="right_side_wall",
        ),
        material=housing_material,
        name="right_side_wall",
    )
    housing.visual(
        Box((width - (2.0 * wall_thickness), wall_thickness, back_height - base_height)),
        origin=Origin(
            xyz=(
                0.0,
                (depth * 0.5) - (wall_thickness * 0.5),
                base_height + ((back_height - base_height) * 0.5),
            )
        ),
        material=housing_material,
        name="back_wall",
    )

    roof_angle = math.atan2(back_height - front_height, depth)
    roof_center_z = ((front_height + back_height) * 0.5) + (math.cos(roof_angle) * roof_thickness * 0.5)
    housing.visual(
        Box((width + 0.030, depth + 0.030, roof_thickness)),
        origin=Origin(xyz=(0.0, 0.0, roof_center_z), rpy=(roof_angle, 0.0, 0.0)),
        material=housing_material,
        name="roof_cap",
    )

    frame_y = -(depth * 0.5) + (frame_depth * 0.5)
    housing.visual(
        Box((width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, frame_y, sill_height * 0.5)),
        material=housing_material,
        name="front_sill",
    )
    housing.visual(
        Box((width, frame_depth, header_height)),
        origin=Origin(xyz=(0.0, frame_y, front_height - (header_height * 0.5))),
        material=housing_material,
        name="front_header",
    )
    housing.visual(
        Box((side_jamb_width, frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                -(width * 0.5) + (side_jamb_width * 0.5),
                frame_y,
                sill_height + (opening_height * 0.5),
            )
        ),
        material=housing_material,
        name="left_jamb",
    )
    housing.visual(
        Box((side_jamb_width, frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                (width * 0.5) - (side_jamb_width * 0.5),
                frame_y,
                sill_height + (opening_height * 0.5),
            )
        ),
        material=housing_material,
        name="right_jamb",
    )
    housing.visual(
        Box((center_post_width, frame_depth, opening_height)),
        origin=Origin(xyz=(0.0, frame_y, sill_height + (opening_height * 0.5))),
        material=housing_material,
        name="center_post",
    )
    housing.visual(
        Box((0.020, depth - 0.120, 0.940)),
        origin=Origin(xyz=(0.0, 0.030, 0.592)),
        material=panel_material,
        name="center_divider",
    )
    housing.visual(
        Box((width - 0.160, depth - 0.200, 0.022)),
        origin=Origin(xyz=(0.0, 0.010, base_height + 0.011)),
        material=panel_material,
        name="interior_floor",
    )

    left_hinge_x = -(width * 0.5) + side_jamb_width
    right_hinge_x = (width * 0.5) - side_jamb_width

    _add_door_and_panel(
        model,
        side="left",
        x_sign=1.0,
        door_hinge_x=left_hinge_x,
        hinge_y=-(depth * 0.5),
        door_bottom=door_bottom,
        door_width=door_width,
        door_height=door_height,
        door_thickness=door_thickness,
        panel_bottom_from_door=panel_bottom_from_door,
        panel_width=panel_width,
        panel_height=panel_height,
        panel_thickness=panel_thickness,
        housing_material=housing_material,
        hardware_material=hardware_material,
        panel_material=panel_material,
    )
    _add_door_and_panel(
        model,
        side="right",
        x_sign=-1.0,
        door_hinge_x=right_hinge_x,
        hinge_y=-(depth * 0.5),
        door_bottom=door_bottom,
        door_width=door_width,
        door_height=door_height,
        door_thickness=door_thickness,
        panel_bottom_from_door=panel_bottom_from_door,
        panel_width=panel_width,
        panel_height=panel_height,
        panel_thickness=panel_thickness,
        housing_material=housing_material,
        hardware_material=hardware_material,
        panel_material=panel_material,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    expected_parts = {
        "housing",
        "left_outer_door",
        "right_outer_door",
        "left_inner_panel",
        "right_inner_panel",
    }
    expected_joints = {
        "left_outer_door_hinge",
        "right_outer_door_hinge",
        "left_inner_panel_hinge",
        "right_inner_panel_hinge",
    }

    if not ctx.check(
        "prompt_parts_present",
        expected_parts.issubset(part_names),
        details=f"Missing parts: {sorted(expected_parts - part_names)}",
    ):
        return ctx.report()
    if not ctx.check(
        "prompt_articulations_present",
        expected_joints.issubset(joint_names),
        details=f"Missing articulations: {sorted(expected_joints - joint_names)}",
    ):
        return ctx.report()

    housing = object_model.get_part("housing")
    left_outer_door = object_model.get_part("left_outer_door")
    right_outer_door = object_model.get_part("right_outer_door")
    left_inner_panel = object_model.get_part("left_inner_panel")
    right_inner_panel = object_model.get_part("right_inner_panel")

    left_outer_hinge = object_model.get_articulation("left_outer_door_hinge")
    right_outer_hinge = object_model.get_articulation("right_outer_door_hinge")
    left_inner_hinge = object_model.get_articulation("left_inner_panel_hinge")
    right_inner_hinge = object_model.get_articulation("right_inner_panel_hinge")

    vertical_axes_ok = all(
        tuple(round(value, 6) for value in articulation.axis) == (0.0, 0.0, 1.0)
        for articulation in (
            left_outer_hinge,
            right_outer_hinge,
            left_inner_hinge,
            right_inner_hinge,
        )
    )
    ctx.check(
        "vertical_hinge_axes",
        vertical_axes_ok,
        details="All outer doors and inner panels should hinge on vertical pins.",
    )

    limits_ok = (
        left_outer_hinge.motion_limits is not None
        and right_outer_hinge.motion_limits is not None
        and left_inner_hinge.motion_limits is not None
        and right_inner_hinge.motion_limits is not None
        and left_outer_hinge.motion_limits.lower is not None
        and left_outer_hinge.motion_limits.upper == 0.0
        and left_outer_hinge.motion_limits.lower <= -1.8
        and right_outer_hinge.motion_limits.lower == 0.0
        and right_outer_hinge.motion_limits.upper is not None
        and right_outer_hinge.motion_limits.upper >= 1.8
        and left_inner_hinge.motion_limits.lower is not None
        and left_inner_hinge.motion_limits.lower == 0.0
        and left_inner_hinge.motion_limits.upper is not None
        and left_inner_hinge.motion_limits.upper >= 1.2
        and right_inner_hinge.motion_limits.upper == 0.0
        and right_inner_hinge.motion_limits.lower is not None
        and right_inner_hinge.motion_limits.lower <= -1.2
        and right_outer_hinge.motion_limits.upper is not None
        and right_outer_hinge.motion_limits.upper >= 1.8
        and left_outer_hinge.motion_limits.lower is not None
        and left_outer_hinge.motion_limits.lower <= -1.8
    )
    ctx.check(
        "hinge_limit_sense",
        limits_ok,
        details="Outer doors should open away from the center seam and inner panels should swing away from the closed door skins.",
    )

    ctx.expect_contact(housing, left_outer_door, name="left_outer_door_mounted")
    ctx.expect_contact(housing, right_outer_door, name="right_outer_door_mounted")
    ctx.expect_contact(left_outer_door, left_inner_panel, name="left_inner_panel_mounted")
    ctx.expect_contact(right_outer_door, right_inner_panel, name="right_inner_panel_mounted")

    ctx.expect_gap(
        left_inner_panel,
        left_outer_door,
        axis="y",
        positive_elem="panel_leaf",
        negative_elem="door_leaf",
        min_gap=0.035,
        max_gap=0.070,
        name="left_inner_panel_clear_of_closed_outer_door",
    )
    ctx.expect_gap(
        right_inner_panel,
        right_outer_door,
        axis="y",
        positive_elem="panel_leaf",
        negative_elem="door_leaf",
        min_gap=0.035,
        max_gap=0.070,
        name="right_inner_panel_clear_of_closed_outer_door",
    )
    ctx.expect_overlap(
        left_inner_panel,
        left_outer_door,
        axes="xz",
        min_overlap=0.50,
        elem_a="panel_leaf",
        elem_b="door_leaf",
        name="left_inner_panel_hidden_behind_outer_door",
    )
    ctx.expect_overlap(
        right_inner_panel,
        right_outer_door,
        axes="xz",
        min_overlap=0.50,
        elem_a="panel_leaf",
        elem_b="door_leaf",
        name="right_inner_panel_hidden_behind_outer_door",
    )

    with ctx.pose(
        left_outer_door_hinge=-1.25,
        left_inner_panel_hinge=0.60,
        right_outer_door_hinge=1.25,
        right_inner_panel_hinge=-0.60,
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_pose_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
