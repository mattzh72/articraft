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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _add_hinge_knuckles(
    part,
    *,
    side: str,
    prefix: str,
    x: float,
    y: float,
    centers: tuple[float, ...],
    radius: float,
    length: float,
    material,
) -> None:
    for idx, center_z in enumerate(centers, start=1):
        position_name = {
            1: "upper",
            2: "mid" if len(centers) == 3 else "lower",
            3: "lower",
        }[idx]
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, center_z)),
            material=material,
            name=f"{side}_{position_name}_{prefix}_knuckle",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_leaf_equipment_hatch")

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.24, 0.26, 1.0))
    door_paint = model.material("door_paint", rgba=(0.72, 0.74, 0.76, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.07, 0.07, 0.08, 1.0))
    latch_steel = model.material("latch_steel", rgba=(0.83, 0.84, 0.86, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    outer_width = 1.00
    outer_height = 1.22
    opening_width = 0.84
    opening_height = 1.02
    frame_depth = 0.06

    side_clearance = 0.003
    center_gap = 0.006
    hinge_axis_offset = 0.018
    hinge_axis_abs_x = opening_width * 0.5 + hinge_axis_offset
    hinge_axis_y = 0.020
    hinge_radius = 0.014
    hinge_segment_length = 0.16
    frame_hinge_centers = (0.32, 0.00, -0.32)
    leaf_hinge_centers = (0.16, -0.16)

    leaf_height = 0.994
    leaf_thickness = 0.022
    leaf_width = opening_width * 0.5 - side_clearance - center_gap * 0.5
    leaf_hinge_edge_local = hinge_axis_abs_x - (opening_width * 0.5 - side_clearance)
    leaf_panel_center_local = leaf_hinge_edge_local + leaf_width * 0.5
    door_panel_y_local = -0.011
    front_trim_y_local = 0.003
    frame_hinge_web_size = (0.020, 0.024, hinge_segment_length)
    leaf_hinge_strap_size = (0.020, 0.014, hinge_segment_length)
    meeting_stile_size = (0.022, 0.018, 0.90)

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _rect_profile(outer_width, outer_height),
                [_rect_profile(opening_width, opening_height)],
                frame_depth,
                center=True,
            ),
            "equipment_hatch_frame_ring",
        ),
        material=frame_paint,
        name="frame_ring",
    )
    frame.visual(
        Box((0.026, 0.010, 0.92)),
        origin=Origin(xyz=(-hinge_axis_abs_x, -0.014, 0.0)),
        material=frame_paint,
        name="left_hinge_strip",
    )
    frame.visual(
        Box((0.026, 0.010, 0.92)),
        origin=Origin(xyz=(hinge_axis_abs_x, -0.014, 0.0)),
        material=frame_paint,
        name="right_hinge_strip",
    )
    for name, x_sign, center_z in (
        ("left_upper_frame_web", -1.0, frame_hinge_centers[0]),
        ("left_mid_frame_web", -1.0, frame_hinge_centers[1]),
        ("left_lower_frame_web", -1.0, frame_hinge_centers[2]),
        ("right_upper_frame_web", 1.0, frame_hinge_centers[0]),
        ("right_mid_frame_web", 1.0, frame_hinge_centers[1]),
        ("right_lower_frame_web", 1.0, frame_hinge_centers[2]),
    ):
        frame.visual(
            Box(frame_hinge_web_size),
            origin=Origin(xyz=(x_sign * hinge_axis_abs_x, 0.002, center_z)),
            material=frame_paint,
            name=name,
        )
    frame.visual(
        Box((opening_width, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, opening_height * 0.5 - 0.010)),
        material=gasket_black,
        name="top_gasket",
    )
    frame.visual(
        Box((opening_width, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, -(opening_height * 0.5 - 0.010))),
        material=gasket_black,
        name="bottom_gasket",
    )
    frame.visual(
        Box((0.020, 0.004, opening_height - 0.040)),
        origin=Origin(xyz=(-(opening_width * 0.5 - 0.010), -0.020, 0.0)),
        material=gasket_black,
        name="left_gasket",
    )
    frame.visual(
        Box((0.020, 0.004, opening_height - 0.040)),
        origin=Origin(xyz=(opening_width * 0.5 - 0.010, -0.020, 0.0)),
        material=gasket_black,
        name="right_gasket",
    )
    _add_hinge_knuckles(
        frame,
        side="left",
        prefix="frame",
        x=-hinge_axis_abs_x,
        y=hinge_axis_y,
        centers=frame_hinge_centers,
        radius=hinge_radius,
        length=hinge_segment_length,
        material=hinge_metal,
    )
    _add_hinge_knuckles(
        frame,
        side="right",
        prefix="frame",
        x=hinge_axis_abs_x,
        y=hinge_axis_y,
        centers=frame_hinge_centers,
        radius=hinge_radius,
        length=hinge_segment_length,
        material=hinge_metal,
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=36.0,
    )

    left_leaf = model.part("left_leaf")
    left_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_height)),
        origin=Origin(xyz=(leaf_panel_center_local, door_panel_y_local, 0.0)),
        material=door_paint,
        name="left_panel",
    )
    left_leaf.visual(
        Box((0.290, 0.004, 0.800)),
        origin=Origin(xyz=(leaf_panel_center_local, -0.001, 0.0)),
        material=door_paint,
        name="left_face_stiffener",
    )
    left_leaf.visual(
        Box(leaf_hinge_strap_size),
        origin=Origin(xyz=(0.014, -0.007, leaf_hinge_centers[0])),
        material=door_paint,
        name="left_upper_hinge_strap",
    )
    left_leaf.visual(
        Box(leaf_hinge_strap_size),
        origin=Origin(xyz=(0.014, -0.007, leaf_hinge_centers[1])),
        material=door_paint,
        name="left_lower_hinge_strap",
    )
    left_leaf.visual(
        Box(meeting_stile_size),
        origin=Origin(xyz=(leaf_hinge_edge_local + leaf_width - meeting_stile_size[0] * 0.5, -0.004, 0.0)),
        material=door_paint,
        name="left_meeting_stile",
    )
    left_leaf.visual(
        Box((0.010, 0.008, 0.86)),
        origin=Origin(xyz=(leaf_hinge_edge_local + leaf_width - 0.005, 0.001, 0.0)),
        material=gasket_black,
        name="left_center_seal",
    )
    _add_hinge_knuckles(
        left_leaf,
        side="left",
        prefix="leaf",
        x=0.0,
        y=0.0,
        centers=leaf_hinge_centers,
        radius=hinge_radius,
        length=hinge_segment_length,
        material=hinge_metal,
    )
    left_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, leaf_thickness, leaf_height)),
        mass=8.0,
        origin=Origin(xyz=(leaf_panel_center_local, door_panel_y_local, 0.0)),
    )

    right_leaf = model.part("right_leaf")
    right_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_height)),
        origin=Origin(xyz=(-leaf_panel_center_local, door_panel_y_local, 0.0)),
        material=door_paint,
        name="right_panel",
    )
    right_leaf.visual(
        Box((0.290, 0.004, 0.800)),
        origin=Origin(xyz=(-leaf_panel_center_local, -0.001, 0.0)),
        material=door_paint,
        name="right_face_stiffener",
    )
    right_leaf.visual(
        Box(leaf_hinge_strap_size),
        origin=Origin(xyz=(-0.014, -0.007, leaf_hinge_centers[0])),
        material=door_paint,
        name="right_upper_hinge_strap",
    )
    right_leaf.visual(
        Box(leaf_hinge_strap_size),
        origin=Origin(xyz=(-0.014, -0.007, leaf_hinge_centers[1])),
        material=door_paint,
        name="right_lower_hinge_strap",
    )
    right_leaf.visual(
        Box(meeting_stile_size),
        origin=Origin(xyz=(-(leaf_hinge_edge_local + leaf_width - meeting_stile_size[0] * 0.5), -0.004, 0.0)),
        material=door_paint,
        name="right_meeting_stile",
    )
    right_leaf.visual(
        Box((0.010, 0.008, 0.86)),
        origin=Origin(xyz=(-(leaf_hinge_edge_local + leaf_width - 0.005), 0.001, 0.0)),
        material=gasket_black,
        name="right_center_seal",
    )
    right_leaf.visual(
        Box((0.072, 0.006, 0.160)),
        origin=Origin(xyz=(-0.398, 0.0, 0.0)),
        material=latch_steel,
        name="latch_escutcheon",
    )
    _add_hinge_knuckles(
        right_leaf,
        side="right",
        prefix="leaf",
        x=0.0,
        y=0.0,
        centers=leaf_hinge_centers,
        radius=hinge_radius,
        length=hinge_segment_length,
        material=hinge_metal,
    )
    right_leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, leaf_thickness, leaf_height)),
        mass=8.0,
        origin=Origin(xyz=(-leaf_panel_center_local, door_panel_y_local, 0.0)),
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=latch_steel,
        name="handle_hub",
    )
    handle.visual(
        Box((0.090, 0.012, 0.020)),
        origin=Origin(xyz=(0.045, 0.004, 0.0)),
        material=handle_black,
        name="handle_bar",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.024),
        origin=Origin(xyz=(0.090, 0.004, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(0.055, 0.004, 0.0)),
    )

    model.articulation(
        "frame_to_left_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_leaf,
        origin=Origin(xyz=(-hinge_axis_abs_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "frame_to_right_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_leaf,
        origin=Origin(xyz=(hinge_axis_abs_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "right_leaf_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=right_leaf,
        child=handle,
        origin=Origin(xyz=(-0.398, 0.011, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=-math.radians(75.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    handle = object_model.get_part("latch_handle")

    left_hinge = object_model.get_articulation("frame_to_left_leaf")
    right_hinge = object_model.get_articulation("frame_to_right_leaf")
    latch_joint = object_model.get_articulation("right_leaf_to_latch_handle")

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
        "left_hinge_is_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_is_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={right_hinge.axis}",
    )
    ctx.check(
        "latch_rotates_about_face_normal",
        latch_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={latch_joint.axis}",
    )

    ctx.expect_contact(
        frame,
        left_leaf,
        elem_a="left_upper_frame_knuckle",
        elem_b="left_upper_leaf_knuckle",
        name="left_leaf_hinge_contact_closed",
    )
    ctx.expect_contact(
        frame,
        right_leaf,
        elem_a="right_upper_frame_knuckle",
        elem_b="right_upper_leaf_knuckle",
        name="right_leaf_hinge_contact_closed",
    )
    ctx.expect_contact(
        handle,
        right_leaf,
        elem_a="handle_hub",
        elem_b="latch_escutcheon",
        name="latch_handle_seated_on_leaf",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        positive_elem="right_panel",
        negative_elem="left_panel",
        min_gap=0.004,
        max_gap=0.008,
        name="paired_doors_meet_with_narrow_center_gap",
    )

    left_closed_aabb = ctx.part_world_aabb(left_leaf)
    right_closed_aabb = ctx.part_world_aabb(right_leaf)
    handle_closed_aabb = ctx.part_world_aabb(handle)

    if left_closed_aabb is None:
        ctx.fail("left_leaf_has_aabb", "left leaf AABB unavailable in closed pose")
    if right_closed_aabb is None:
        ctx.fail("right_leaf_has_aabb", "right leaf AABB unavailable in closed pose")
    if handle_closed_aabb is None:
        ctx.fail("handle_has_aabb", "handle AABB unavailable in closed pose")

    if left_closed_aabb is not None:
        with ctx.pose({left_hinge: math.radians(75.0)}):
            left_open_aabb = ctx.part_world_aabb(left_leaf)
            if left_open_aabb is None:
                ctx.fail("left_leaf_has_open_aabb", "left leaf AABB unavailable when opened")
            else:
                ctx.check(
                    "left_leaf_swings_outward",
                    left_open_aabb[1][1] > left_closed_aabb[1][1] + 0.10,
                    details=f"closed_max_y={left_closed_aabb[1][1]:.4f}, open_max_y={left_open_aabb[1][1]:.4f}",
                )
            ctx.expect_contact(
                frame,
                left_leaf,
                elem_a="left_upper_frame_knuckle",
                elem_b="left_upper_leaf_knuckle",
                name="left_leaf_remains_captured_when_open",
            )

    if right_closed_aabb is not None:
        with ctx.pose({right_hinge: math.radians(75.0)}):
            right_open_aabb = ctx.part_world_aabb(right_leaf)
            if right_open_aabb is None:
                ctx.fail("right_leaf_has_open_aabb", "right leaf AABB unavailable when opened")
            else:
                ctx.check(
                    "right_leaf_swings_outward",
                    right_open_aabb[1][1] > right_closed_aabb[1][1] + 0.10,
                    details=f"closed_max_y={right_closed_aabb[1][1]:.4f}, open_max_y={right_open_aabb[1][1]:.4f}",
                )
            ctx.expect_contact(
                frame,
                right_leaf,
                elem_a="right_upper_frame_knuckle",
                elem_b="right_upper_leaf_knuckle",
                name="right_leaf_remains_captured_when_open",
            )

    if handle_closed_aabb is not None:
        with ctx.pose({latch_joint: math.radians(60.0)}):
            handle_open_aabb = ctx.part_world_aabb(handle)
            if handle_open_aabb is None:
                ctx.fail("handle_has_open_aabb", "handle AABB unavailable when turned")
            else:
                ctx.check(
                    "latch_handle_rotates",
                    handle_open_aabb[0][2] < handle_closed_aabb[0][2] - 0.05,
                    details=f"closed_min_z={handle_closed_aabb[0][2]:.4f}, open_min_z={handle_open_aabb[0][2]:.4f}",
                )
            ctx.expect_contact(
                handle,
                right_leaf,
                elem_a="handle_hub",
                elem_b="latch_escutcheon",
                name="latch_handle_stays_captured_on_pivot",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
