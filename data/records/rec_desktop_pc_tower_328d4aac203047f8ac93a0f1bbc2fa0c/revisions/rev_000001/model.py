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
)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_desktop_pc")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    powder_black = model.material("powder_black", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.56, 0.68, 0.74, 0.24))
    mesh_black = model.material("mesh_black", rgba=(0.17, 0.18, 0.19, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.12, 0.39, 0.20, 1.0))
    gpu_gunmetal = model.material("gpu_gunmetal", rgba=(0.27, 0.29, 0.31, 1.0))
    psu_black = model.material("psu_black", rgba=(0.10, 0.10, 0.11, 1.0))
    drive_silver = model.material("drive_silver", rgba=(0.72, 0.72, 0.74, 1.0))

    width = 0.28
    depth = 0.50
    height = 0.52
    extrusion = 0.02
    foot_height = 0.01

    half_width = width * 0.5
    half_depth = depth * 0.5
    post_center_x = half_width - extrusion * 0.5
    post_center_y = half_depth - extrusion * 0.5
    bottom_z = foot_height + extrusion * 0.5
    top_z = foot_height + height - extrusion * 0.5
    body_mid_z = foot_height + height * 0.5
    post_height = height - 2.0 * extrusion

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((width, depth, height + foot_height)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, foot_height + height * 0.5)),
    )

    for y_pos, name in ((post_center_y, "front"), (-post_center_y, "rear")):
        _add_box(
            chassis,
            (width, extrusion, extrusion),
            (0.0, y_pos, bottom_z),
            brushed_aluminum,
            name=f"bottom_{name}_rail",
        )
        _add_box(
            chassis,
            (width, extrusion, extrusion),
            (0.0, y_pos, top_z),
            brushed_aluminum,
            name=f"top_{name}_rail",
        )

    for x_pos, name in ((post_center_x, "right"), (-post_center_x, "left")):
        _add_box(
            chassis,
            (extrusion, depth - 2.0 * extrusion, extrusion),
            (x_pos, 0.0, bottom_z),
            brushed_aluminum,
            name=f"bottom_{name}_rail",
        )
        _add_box(
            chassis,
            (extrusion, depth - 2.0 * extrusion, extrusion),
            (x_pos, 0.0, top_z),
            brushed_aluminum,
            name=f"top_{name}_rail",
        )

    for x_pos, x_name in ((post_center_x, "right"), (-post_center_x, "left")):
        for y_pos, y_name in ((post_center_y, "front"), (-post_center_y, "rear")):
            _add_box(
                chassis,
                (extrusion, extrusion, post_height),
                (x_pos, y_pos, body_mid_z),
                brushed_aluminum,
                name=f"{y_name}_{x_name}_post",
            )

    for x_pos, y_pos in (
        (post_center_x, post_center_y),
        (post_center_x, -post_center_y),
        (-post_center_x, post_center_y),
        (-post_center_x, -post_center_y),
    ):
        _add_cylinder(
            chassis,
            radius=0.018,
            length=foot_height,
            xyz=(x_pos, y_pos, foot_height * 0.5),
            material=powder_black,
            rpy=(0.0, 0.0, 0.0),
        )

    tray_support_x = -0.119
    _add_box(
        chassis,
        (0.024, depth - 0.04, 0.018),
        (tray_support_x, 0.0, 0.11),
        brushed_aluminum,
        name="tray_lower_support",
    )
    _add_box(
        chassis,
        (0.024, depth - 0.04, 0.018),
        (tray_support_x, 0.0, 0.44),
        brushed_aluminum,
        name="tray_upper_support",
    )
    _add_box(
        chassis,
        (0.004, 0.36, 0.38),
        (-0.108, 0.0, 0.28),
        powder_black,
        name="motherboard_tray",
    )
    _add_box(
        chassis,
        (0.002, 0.305, 0.244),
        (-0.105, 0.005, 0.285),
        pcb_green,
        name="motherboard",
    )
    _add_box(
        chassis,
        (0.180, 0.048, 0.110),
        (-0.016, 0.020, 0.275),
        gpu_gunmetal,
        name="gpu",
    )
    _add_box(
        chassis,
        (0.130, 0.190, 0.086),
        (-0.045, -0.135, 0.063),
        psu_black,
        name="power_supply",
    )

    drive_post_y = 0.08
    _add_box(
        chassis,
        (extrusion, extrusion, post_height),
        (0.0, drive_post_y, body_mid_z),
        brushed_aluminum,
        name="drive_hinge_post",
    )
    for z_pos, name in ((0.11, "lower"), (0.44, "upper")):
        _add_box(
            chassis,
            (0.142, 0.024, 0.018),
            (-0.061, drive_post_y, z_pos),
            brushed_aluminum,
            name=f"drive_post_{name}_cross_beam",
        )

    side_hinge_x = half_width + 0.004
    side_hinge_y = -half_depth + extrusion * 0.5
    side_hinge_upper_z = foot_height + 0.40
    side_hinge_lower_z = foot_height + 0.14
    for z_pos, name in ((side_hinge_upper_z, "upper"), (side_hinge_lower_z, "lower")):
        _add_box(
            chassis,
            (0.010, 0.016, 0.050),
            (half_width - 0.001, side_hinge_y + 0.008, z_pos),
            dark_steel,
            name=f"side_hinge_{name}_mount",
        )

    top_hinge_y = half_depth - extrusion
    top_hinge_z = top_z
    for x_pos, name in ((0.085, "left"), (-0.085, "right")):
        _add_box(
            chassis,
            (0.050, 0.010, 0.010),
            (x_pos, top_hinge_y + 0.005, top_hinge_z),
            dark_steel,
            name=f"top_hinge_{name}_mount",
        )

    drive_hinge_origin = (0.02, drive_post_y, body_mid_z)
    for z_pos, name in ((0.38, "upper"), (0.16, "lower")):
        _add_box(
            chassis,
            (0.012, 0.020, 0.050),
            (0.014, drive_post_y, z_pos),
            dark_steel,
            name=f"drive_hinge_{name}_mount",
        )

    side_panel = model.part("side_panel")
    side_panel.inertial = Inertial.from_geometry(
        Box((0.016, depth - 0.04, height - 0.05)),
        mass=3.2,
        origin=Origin(xyz=(0.008, 0.23, 0.0)),
    )
    panel_height = height - 0.06
    panel_depth = depth - 0.04
    _add_box(
        side_panel,
        (0.004, panel_depth, panel_height),
        (0.008, panel_depth * 0.5, 0.0),
        smoked_glass,
        name="glass",
    )
    _add_box(
        side_panel,
        (0.012, 0.020, panel_height),
        (0.006, 0.010, 0.0),
        dark_steel,
        name="rear_stile",
    )
    _add_box(
        side_panel,
        (0.010, 0.014, 0.180),
        (0.007, panel_depth - 0.007, 0.0),
        dark_steel,
        name="pull_rail",
    )
    _add_box(
        side_panel,
        (0.010, 0.016, 0.050),
        (0.005, 0.008, side_hinge_upper_z - body_mid_z),
        dark_steel,
        name="upper_hinge_leaf",
    )
    _add_box(
        side_panel,
        (0.010, 0.016, 0.050),
        (0.005, 0.008, side_hinge_lower_z - body_mid_z),
        dark_steel,
        name="lower_hinge_leaf",
    )
    _add_cylinder(
        side_panel,
        radius=0.006,
        length=0.040,
        xyz=(0.011, 0.008, side_hinge_upper_z - body_mid_z),
        material=dark_steel,
        name="upper_hinge_barrel",
    )
    _add_cylinder(
        side_panel,
        radius=0.006,
        length=0.040,
        xyz=(0.011, 0.008, side_hinge_lower_z - body_mid_z),
        material=dark_steel,
        name="lower_hinge_barrel",
    )

    top_panel = model.part("top_panel")
    top_panel.inertial = Inertial.from_geometry(
        Box((width - 0.04, depth - 0.04, 0.030)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -0.23, 0.0)),
    )
    panel_width = width - 0.04
    roof_depth = depth - 0.04
    frame_bar = 0.012
    _add_box(
        top_panel,
        (panel_width, frame_bar, frame_bar),
        (0.0, -frame_bar * 0.5, 0.0),
        powder_black,
        name="front_frame_bar",
    )
    _add_box(
        top_panel,
        (panel_width, frame_bar, frame_bar),
        (0.0, -(roof_depth - frame_bar * 0.5), 0.0),
        powder_black,
        name="rear_frame_bar",
    )
    _add_box(
        top_panel,
        (frame_bar, roof_depth - 2.0 * frame_bar, frame_bar),
        (panel_width * 0.5 - frame_bar * 0.5, -roof_depth * 0.5, 0.0),
        powder_black,
        name="right_frame_bar",
    )
    _add_box(
        top_panel,
        (frame_bar, roof_depth - 2.0 * frame_bar, frame_bar),
        (-panel_width * 0.5 + frame_bar * 0.5, -roof_depth * 0.5, 0.0),
        powder_black,
        name="left_frame_bar",
    )
    for index, y_pos in enumerate((-0.07, -0.14, -0.21, -0.28, -0.35)):
        _add_box(
            top_panel,
            (panel_width - 0.020, 0.004, 0.004),
            (0.0, y_pos, 0.0),
            mesh_black,
            name=f"mesh_slat_{index}",
        )
    _add_box(
        top_panel,
        (0.050, 0.010, 0.010),
        (0.085, -0.005, 0.0),
        dark_steel,
        name="left_front_leaf",
    )
    _add_box(
        top_panel,
        (0.050, 0.010, 0.010),
        (-0.085, -0.005, 0.0),
        dark_steel,
        name="right_front_leaf",
    )
    _add_box(
        top_panel,
        (0.080, 0.010, 0.018),
        (0.0, -(roof_depth - 0.012), 0.006),
        dark_steel,
        name="rear_lift_tab",
    )

    drive_cage = model.part("drive_cage")
    drive_cage.inertial = Inertial.from_geometry(
        Box((0.09, 0.19, 0.30)),
        mass=2.4,
        origin=Origin(xyz=(0.044, 0.095, 0.0)),
    )
    cage_center_y = 0.095
    _add_box(
        drive_cage,
        (0.020, 0.180, 0.30),
        (0.010, 0.090, 0.0),
        dark_steel,
        name="hinge_spine",
    )
    _add_box(
        drive_cage,
        (0.012, 0.180, 0.300),
        (0.078, cage_center_y, 0.0),
        powder_black,
        name="outer_side_rail",
    )
    _add_box(
        drive_cage,
        (0.082, 0.014, 0.300),
        (0.041, 0.007, 0.0),
        powder_black,
        name="front_rail",
    )
    _add_box(
        drive_cage,
        (0.082, 0.014, 0.300),
        (0.041, 0.173, 0.0),
        powder_black,
        name="rear_rail",
    )
    for index, z_pos in enumerate((-0.09, 0.0, 0.09)):
        _add_box(
            drive_cage,
            (0.070, 0.146, 0.012),
            (0.043, cage_center_y, z_pos),
            powder_black,
            name=f"tray_{index}",
        )
    for index, z_pos in enumerate((-0.09, 0.09)):
        _add_box(
            drive_cage,
            (0.062, 0.100, 0.025),
            (0.044, cage_center_y, z_pos + 0.020),
            drive_silver,
            name=f"drive_{index}",
        )
    _add_box(
        drive_cage,
        (0.008, 0.020, 0.050),
        (0.004, 0.010, 0.11),
        dark_steel,
        name="upper_cage_leaf",
    )
    _add_box(
        drive_cage,
        (0.008, 0.020, 0.050),
        (0.004, 0.010, -0.11),
        dark_steel,
        name="lower_cage_leaf",
    )
    _add_cylinder(
        drive_cage,
        radius=0.006,
        length=0.040,
        xyz=(0.010, 0.010, 0.11),
        material=dark_steel,
        name="upper_cage_barrel",
    )
    _add_cylinder(
        drive_cage,
        radius=0.006,
        length=0.040,
        xyz=(0.010, 0.010, -0.11),
        material=dark_steel,
        name="lower_cage_barrel",
    )

    model.articulation(
        "side_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(side_hinge_x, side_hinge_y, body_mid_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "top_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, top_hinge_y, top_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "drive_cage_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=drive_cage,
        origin=Origin(xyz=drive_hinge_origin),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    top_panel = object_model.get_part("top_panel")
    drive_cage = object_model.get_part("drive_cage")
    side_hinge = object_model.get_articulation("side_panel_hinge")
    top_hinge = object_model.get_articulation("top_panel_hinge")
    drive_hinge = object_model.get_articulation("drive_cage_hinge")

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

    ctx.expect_contact(
        side_panel,
        chassis,
        elem_a="upper_hinge_leaf",
        elem_b="side_hinge_upper_mount",
        name="side panel upper hinge seats on rear frame mount",
    )
    ctx.expect_contact(
        side_panel,
        chassis,
        elem_a="lower_hinge_leaf",
        elem_b="side_hinge_lower_mount",
        name="side panel lower hinge seats on rear frame mount",
    )
    ctx.expect_contact(
        top_panel,
        chassis,
        elem_a="left_front_leaf",
        elem_b="top_hinge_left_mount",
        name="top panel left hinge seats on front rail mount",
    )
    ctx.expect_contact(
        top_panel,
        chassis,
        elem_a="right_front_leaf",
        elem_b="top_hinge_right_mount",
        name="top panel right hinge seats on front rail mount",
    )
    ctx.expect_contact(
        drive_cage,
        chassis,
        elem_a="upper_cage_leaf",
        elem_b="drive_hinge_upper_mount",
        name="drive cage upper hinge seats on interior post mount",
    )
    ctx.expect_contact(
        drive_cage,
        chassis,
        elem_a="lower_cage_leaf",
        elem_b="drive_hinge_lower_mount",
        name="drive cage lower hinge seats on interior post mount",
    )

    with ctx.pose({side_hinge: 0.0, top_hinge: 0.0, drive_hinge: 0.0}):
        ctx.expect_overlap(
            side_panel,
            chassis,
            axes="yz",
            min_overlap=0.36,
            name="side panel covers the main side opening",
        )
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="xy",
            min_overlap=0.20,
            name="top panel spans the roof opening",
        )
        side_closed_aabb = ctx.part_world_aabb(side_panel)
        top_closed_aabb = ctx.part_world_aabb(top_panel)
        drive_closed_aabb = ctx.part_world_aabb(drive_cage)

    with ctx.pose({side_hinge: 1.10}):
        side_open_aabb = ctx.part_world_aabb(side_panel)

    with ctx.pose({top_hinge: 1.00}):
        top_open_aabb = ctx.part_world_aabb(top_panel)

    with ctx.pose({drive_hinge: 1.00}):
        drive_open_aabb = ctx.part_world_aabb(drive_cage)

    side_closed_center = _aabb_center(side_closed_aabb)
    side_open_center = _aabb_center(side_open_aabb)
    top_closed_center = _aabb_center(top_closed_aabb)
    top_open_center = _aabb_center(top_open_aabb)
    drive_closed_center = _aabb_center(drive_closed_aabb)
    drive_open_center = _aabb_center(drive_open_aabb)

    ctx.check(
        "side panel swings outward from the frame",
        side_closed_center is not None
        and side_open_center is not None
        and side_open_center[0] > side_closed_center[0] + 0.10,
        details=f"closed={side_closed_center}, open={side_open_center}",
    )
    ctx.check(
        "top mesh panel lifts upward from the front edge",
        top_closed_center is not None
        and top_open_center is not None
        and top_open_center[2] > top_closed_center[2] + 0.08,
        details=f"closed={top_closed_center}, open={top_open_center}",
    )
    ctx.check(
        "drive cage swings toward the side access zone",
        drive_closed_center is not None
        and drive_open_center is not None
        and drive_open_center[0] > drive_closed_center[0] + 0.05,
        details=f"closed={drive_closed_center}, open={drive_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
