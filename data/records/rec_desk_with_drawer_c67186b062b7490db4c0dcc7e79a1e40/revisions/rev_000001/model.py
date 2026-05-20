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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_school_combo_desk")

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def rounded_panel_mesh(name: str, sx: float, sy: float, thickness: float, radius: float):
        return save_mesh(
            name,
            ExtrudeGeometry(
                rounded_rect_profile(sx, sy, radius, corner_segments=8),
                thickness,
                center=True,
            ),
        )

    frame_paint = model.material("frame_paint", rgba=(0.28, 0.31, 0.34, 1.0))
    desk_laminate = model.material("desk_laminate", rgba=(0.81, 0.67, 0.45, 1.0))
    seat_shell = model.material("seat_shell", rgba=(0.90, 0.80, 0.58, 1.0))
    storage_gray = model.material("storage_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    rubber_foot = model.material("rubber_foot", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.80, 0.70, 0.76)),
        mass=18.0,
        origin=Origin(xyz=(0.12, 0.0, 0.38)),
    )

    frame_radius = 0.014
    side_y = 0.265
    side_path = [
        (0.48, side_y, 0.02),
        (0.46, side_y, 0.18),
        (0.43, side_y, 0.39),
        (0.36, side_y, 0.58),
        (0.08, side_y, 0.609),
        (-0.06, side_y, 0.58),
        (-0.16, side_y, 0.41),
        (-0.20, side_y, 0.18),
        (-0.22, side_y, 0.02),
    ]
    left_frame = tube_from_spline_points(
        side_path,
        radius=frame_radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    right_frame = tube_from_spline_points(
        [(x, -y, z) for x, y, z in side_path],
        radius=frame_radius,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    body.visual(save_mesh("left_combo_desk_frame", left_frame), material=frame_paint, name="left_frame")
    body.visual(save_mesh("right_combo_desk_frame", right_frame), material=frame_paint, name="right_frame")

    for name, x_pos, z_pos, radius, length in [
        ("front_foot_bar", 0.46, 0.10, 0.015, 0.57),
        ("seat_front_bar", 0.00, 0.330, 0.015, 0.57),
        ("seat_rear_bar", -0.12, 0.330, 0.015, 0.57),
        ("backrest_mount_bar", -0.11, 0.475, 0.014, 0.56),
        ("storage_rear_support", 0.13, 0.472, 0.015, 0.57),
        ("storage_front_support", 0.31, 0.472, 0.015, 0.57),
        ("desktop_hinge_support", 0.06, 0.608, 0.015, 0.56),
    ]:
        body.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x_pos, 0.0, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material=frame_paint,
            name=name,
        )

    for name, y_pos in [("left_side_rail", side_y), ("right_side_rail", -side_y)]:
        body.visual(
            Cylinder(radius=0.012, length=0.30),
            origin=Origin(xyz=(0.15, y_pos, 0.40), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_paint,
            name=name,
        )
        body.visual(
            Cylinder(radius=0.011, length=0.22),
            origin=Origin(xyz=(-0.08, y_pos, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_paint,
            name=f"{name}_rear_brace",
        )

    body.visual(
        Box((0.48, 0.022, 0.30)),
        origin=Origin(xyz=(0.10, 0.255, 0.420)),
        material=storage_gray,
        name="left_side_support_panel",
    )
    body.visual(
        Box((0.48, 0.022, 0.30)),
        origin=Origin(xyz=(0.10, -0.255, 0.420)),
        material=storage_gray,
        name="right_side_support_panel",
    )

    body.visual(
        Box((0.050, 0.050, 0.028)),
        origin=Origin(xyz=(0.48, side_y, 0.014)),
        material=rubber_foot,
        name="left_front_foot",
    )
    body.visual(
        Box((0.050, 0.050, 0.028)),
        origin=Origin(xyz=(0.48, -side_y, 0.014)),
        material=rubber_foot,
        name="right_front_foot",
    )
    body.visual(
        Box((0.050, 0.050, 0.028)),
        origin=Origin(xyz=(-0.22, side_y, 0.014)),
        material=rubber_foot,
        name="left_rear_foot",
    )
    body.visual(
        Box((0.050, 0.050, 0.028)),
        origin=Origin(xyz=(-0.22, -side_y, 0.014)),
        material=rubber_foot,
        name="right_rear_foot",
    )

    body.visual(
        rounded_panel_mesh("seat_panel_mesh", 0.32, 0.34, 0.022, 0.030),
        origin=Origin(xyz=(-0.05, 0.0, 0.351)),
        material=seat_shell,
        name="seat_panel",
    )
    body.visual(
        rounded_panel_mesh("backrest_panel_mesh", 0.26, 0.34, 0.018, 0.026),
        origin=Origin(xyz=(-0.125, 0.0, 0.565), rpy=(0.0, pi / 2.0 - 0.20, 0.0)),
        material=seat_shell,
        name="backrest_panel",
    )

    wall_t = 0.014
    box_depth = 0.38
    box_width = 0.50
    box_height = 0.145
    box_center_x = 0.24
    box_center_z = 0.547
    body.visual(
        Box((wall_t, box_width, box_height)),
        origin=Origin(xyz=(0.067, 0.0, box_center_z)),
        material=storage_gray,
        name="storage_back_wall",
    )
    body.visual(
        Box((wall_t, box_width, box_height)),
        origin=Origin(xyz=(0.413, 0.0, box_center_z)),
        material=storage_gray,
        name="storage_front_wall",
    )
    body.visual(
        Box((box_depth, wall_t, box_height)),
        origin=Origin(xyz=(box_center_x, 0.243, box_center_z)),
        material=storage_gray,
        name="storage_left_wall",
    )
    body.visual(
        Box((box_depth, wall_t, box_height)),
        origin=Origin(xyz=(box_center_x, -0.243, box_center_z)),
        material=storage_gray,
        name="storage_right_wall",
    )
    body.visual(
        Box((box_depth, box_width - 0.026, 0.014)),
        origin=Origin(xyz=(box_center_x, 0.0, 0.484)),
        material=storage_gray,
        name="storage_bottom",
    )
    body.visual(
        Box((0.030, box_width, 0.022)),
        origin=Origin(xyz=(0.072, 0.0, 0.600)),
        material=frame_paint,
        name="rear_hinge_rail",
    )

    desktop = model.part("desktop")
    desktop.visual(
        rounded_panel_mesh("desktop_panel_mesh", 0.42, 0.56, 0.018, 0.026),
        origin=Origin(xyz=(0.21, 0.0, 0.009)),
        material=desk_laminate,
        name="desktop_panel",
    )
    desktop.visual(
        Box((0.018, 0.18, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, -0.006)),
        material=frame_paint,
        name="desktop_hinge_cleat",
    )
    desktop.inertial = Inertial.from_geometry(
        Box((0.42, 0.56, 0.040)),
        mass=3.2,
        origin=Origin(xyz=(0.21, 0.0, 0.006)),
    )

    storage_lid = model.part("storage_lid")
    storage_lid.visual(
        rounded_panel_mesh("storage_lid_panel_mesh", 0.30, 0.46, 0.012, 0.020),
        origin=Origin(xyz=(0.15, 0.0, 0.006)),
        material=desk_laminate,
        name="storage_lid_panel",
    )
    storage_lid.visual(
        Box((0.018, 0.12, 0.010)),
        origin=Origin(xyz=(0.294, 0.0, 0.013)),
        material=frame_paint,
        name="storage_lid_pull",
    )
    storage_lid.visual(
        Box((0.006, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=frame_paint,
        name="storage_lid_hinge_cleat",
    )
    storage_lid.inertial = Inertial.from_geometry(
        Box((0.30, 0.46, 0.022)),
        mass=1.0,
        origin=Origin(xyz=(0.15, 0.0, 0.008)),
    )

    model.articulation(
        "body_to_desktop",
        ArticulationType.REVOLUTE,
        parent=body,
        child=desktop,
        origin=Origin(xyz=(0.06, 0.0, 0.625)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=0.95),
    )
    model.articulation(
        "body_to_storage_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=storage_lid,
        origin=Origin(xyz=(0.09, 0.0, 0.592)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    desktop = object_model.get_part("desktop")
    storage_lid = object_model.get_part("storage_lid")
    desktop_joint = object_model.get_articulation("body_to_desktop")
    lid_joint = object_model.get_articulation("body_to_storage_lid")

    ctx.expect_overlap(
        desktop,
        body,
        axes="xy",
        elem_a="desktop_panel",
        elem_b="storage_bottom",
        min_overlap=0.30,
        name="desktop covers the storage opening footprint",
    )
    ctx.expect_gap(
        desktop,
        body,
        axis="z",
        positive_elem="desktop_panel",
        negative_elem="storage_back_wall",
        min_gap=0.0,
        max_gap=0.02,
        name="desktop sits just above the storage box at the rear hinge side",
    )
    ctx.expect_within(
        storage_lid,
        body,
        axes="xy",
        inner_elem="storage_lid_panel",
        outer_elem="storage_bottom",
        margin=0.05,
        name="storage lid stays within the storage compartment footprint",
    )

    closed_desktop = ctx.part_element_world_aabb(desktop, elem="desktop_panel")
    with ctx.pose({desktop_joint: 0.80}):
        open_desktop = ctx.part_element_world_aabb(desktop, elem="desktop_panel")
    ctx.check(
        "desktop tilts upward from the rear edge",
        closed_desktop is not None
        and open_desktop is not None
        and open_desktop[1][2] > closed_desktop[1][2] + 0.12,
        details=f"closed={closed_desktop}, open={open_desktop}",
    )

    closed_lid = ctx.part_element_world_aabb(storage_lid, elem="storage_lid_panel")
    with ctx.pose({desktop_joint: 0.90, lid_joint: 0.85}):
        open_lid = ctx.part_element_world_aabb(storage_lid, elem="storage_lid_panel")
    ctx.check(
        "storage lid opens upward on its back edge hinge",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
