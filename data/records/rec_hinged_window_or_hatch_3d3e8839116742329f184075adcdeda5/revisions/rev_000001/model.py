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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate(width: float, height: float, thickness: float, radius: float, name: str):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius),
        thickness,
        cap=True,
        closed=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _rounded_ring(
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    thickness: float,
    outer_radius: float,
    inner_radius: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_height, outer_radius),
        [rounded_rect_profile(inner_width, inner_height, inner_radius)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_hatch")

    body_color = model.material("body_white", rgba=(0.88, 0.89, 0.87, 1.0))
    frame_color = model.material("frame_gray", rgba=(0.23, 0.25, 0.27, 1.0))
    panel_color = model.material("panel_white", rgba=(0.92, 0.93, 0.92, 1.0))
    liner_color = model.material("liner_gray", rgba=(0.73, 0.75, 0.77, 1.0))
    latch_color = model.material("latch_steel", rgba=(0.68, 0.70, 0.72, 1.0))

    body_width = 0.90
    body_height = 0.72
    body_thickness = 0.004
    opening_width = 0.67
    opening_height = 0.51

    frame_flange_width = 0.76
    frame_flange_height = 0.60
    frame_inner_width = 0.59
    frame_inner_height = 0.43
    frame_flange_thickness = 0.004
    frame_box_depth = 0.024

    panel_width = 0.71
    panel_height = 0.55
    panel_skin_thickness = 0.004
    panel_pocket_width = 0.528
    panel_pocket_height = 0.372
    panel_wall_thickness = 0.010
    panel_depth = 0.026
    panel_liner_thickness = 0.002

    hinge_axis_y = -0.012
    hinge_axis_z = 0.281
    panel_skin_center_local = (0.0, 0.010, -0.281)
    panel_liner_center_local = (0.0, -0.019, -0.300)

    body = model.part("body_panel")
    body.visual(
        _rounded_ring(
            body_width,
            body_height,
            opening_width,
            opening_height,
            body_thickness,
            0.030,
            0.024,
            "body_panel_shell",
        ),
        origin=Origin(xyz=(0.0, -body_thickness / 2.0, 0.0)),
        material=body_color,
        name="body_shell",
    )

    frame = model.part("perimeter_frame")
    frame.visual(
        _rounded_ring(
            frame_flange_width,
            frame_flange_height,
            frame_inner_width,
            frame_inner_height,
            frame_flange_thickness,
            0.022,
            0.018,
            "frame_mounting_flange",
        ),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=frame_color,
        name="mounting_flange",
    )
    frame.visual(
        _rounded_ring(
            opening_width,
            opening_height,
            frame_inner_width,
            frame_inner_height,
            frame_box_depth,
            0.024,
            0.018,
            "frame_box_section",
        ),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=frame_color,
        name="frame_box",
    )
    frame.visual(
        Box((0.62, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, 0.250)),
        material=frame_color,
        name="header_stiffener",
    )

    panel = model.part("access_panel")
    panel.visual(
        _rounded_plate(panel_width, panel_height, panel_skin_thickness, 0.022, "panel_outer_skin"),
        origin=Origin(xyz=panel_skin_center_local),
        material=panel_color,
        name="outer_skin",
    )
    panel.visual(
        Box((0.62, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, -0.012)),
        material=panel_color,
        name="hinge_clip",
    )
    panel.visual(
        Cylinder(radius=0.005, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_color,
        name="hinge_barrel",
    )
    panel.visual(
        Box((0.008, panel_depth, 0.382)),
        origin=Origin(xyz=(-0.268, -0.005, -0.300)),
        material=liner_color,
        name="left_return",
    )
    panel.visual(
        Box((0.008, panel_depth, 0.382)),
        origin=Origin(xyz=(0.268, -0.005, -0.300)),
        material=liner_color,
        name="right_return",
    )
    panel.visual(
        Box((panel_pocket_width, panel_depth, 0.008)),
        origin=Origin(xyz=(0.0, -0.005, -0.491)),
        material=liner_color,
        name="bottom_return",
    )
    panel.visual(
        _rounded_plate(
            panel_pocket_width,
            panel_pocket_height,
            panel_liner_thickness,
            0.016,
            "panel_inner_liner",
        ),
        origin=Origin(xyz=panel_liner_center_local),
        material=liner_color,
        name="inner_liner",
    )
    panel.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(
            xyz=(-0.245, 0.014, -0.455),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=latch_color,
        name="left_latch_boss",
    )
    panel.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(
            xyz=(0.245, 0.014, -0.455),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=latch_color,
        name="right_latch_boss",
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_color,
        name="spindle",
    )
    left_latch.visual(
        Box((0.056, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=latch_color,
        name="handle_bar",
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_color,
        name="spindle",
    )
    right_latch.visual(
        Box((0.056, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=latch_color,
        name="handle_bar",
    )

    model.articulation(
        "body_to_frame",
        ArticulationType.FIXED,
        parent=body,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "panel_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=left_latch,
        origin=Origin(xyz=(-0.245, 0.016, -0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "panel_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=right_latch,
        origin=Origin(xyz=(0.245, 0.016, -0.455)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_panel")
    frame = object_model.get_part("perimeter_frame")
    panel = object_model.get_part("access_panel")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    hatch_hinge = object_model.get_articulation("frame_to_panel")
    left_joint = object_model.get_articulation("panel_to_left_latch")
    right_joint = object_model.get_articulation("panel_to_right_latch")

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
        "parts_present",
        all(
            part_name in {part.name for part in object_model.parts}
            for part_name in (
                "body_panel",
                "perimeter_frame",
                "access_panel",
                "left_latch",
                "right_latch",
            )
        ),
        "Expected body panel, perimeter frame, access panel, and both latch parts.",
    )
    ctx.check(
        "hinge_axes",
        hatch_hinge.axis == (1.0, 0.0, 0.0)
        and left_joint.axis == (0.0, 1.0, 0.0)
        and right_joint.axis == (0.0, 1.0, 0.0),
        "Door hinge must run across the hatch on X and both quarter-turn latches must rotate about local Y.",
    )
    ctx.check(
        "hinge_limits",
        hatch_hinge.motion_limits is not None
        and hatch_hinge.motion_limits.lower == 0.0
        and hatch_hinge.motion_limits.upper is not None
        and 1.20 <= hatch_hinge.motion_limits.upper <= 1.45,
        "Hatch panel should open upward from closed to a realistic service angle.",
    )
    ctx.check(
        "latch_limits",
        left_joint.motion_limits is not None
        and right_joint.motion_limits is not None
        and math.isclose(left_joint.motion_limits.upper or 0.0, math.pi / 2.0, rel_tol=0.0, abs_tol=1e-6)
        and math.isclose(right_joint.motion_limits.upper or 0.0, math.pi / 2.0, rel_tol=0.0, abs_tol=1e-6),
        "Both latches should be quarter-turn mechanisms.",
    )

    ctx.expect_contact(
        frame,
        body,
        elem_a="mounting_flange",
        elem_b="body_shell",
        contact_tol=1e-4,
        name="frame_mounted_to_body",
    )

    with ctx.pose({hatch_hinge: 0.0, left_joint: 0.0, right_joint: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="outer_skin")
        ctx.expect_contact(
            panel,
            frame,
            elem_a="outer_skin",
            elem_b="mounting_flange",
            contact_tol=1e-4,
            name="panel_seated_on_frame",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xz",
            elem_a="outer_skin",
            elem_b="mounting_flange",
            min_overlap=0.54,
            name="panel_overlaps_frame_face",
        )
        ctx.expect_contact(
            left_latch,
            panel,
            elem_a="spindle",
            elem_b="left_latch_boss",
            contact_tol=1e-4,
            name="left_latch_mounted_to_panel",
        )
        ctx.expect_contact(
            right_latch,
            panel,
            elem_a="spindle",
            elem_b="right_latch_boss",
            contact_tol=1e-4,
            name="right_latch_mounted_to_panel",
        )

        closed_left_bar = ctx.part_element_world_aabb(left_latch, elem="handle_bar")

    with ctx.pose({hatch_hinge: 1.0}):
        open_panel_aabb = ctx.part_element_world_aabb(panel, elem="outer_skin")

    ctx.check(
        "panel_opens_upward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.12
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.08,
        "Open pose should swing the lower edge upward and outward from the body face.",
    )

    with ctx.pose({left_joint: math.pi / 2.0}):
        open_left_bar = ctx.part_element_world_aabb(left_latch, elem="handle_bar")

    ctx.check(
        "left_latch_quarter_turn_motion",
        closed_left_bar is not None
        and open_left_bar is not None
        and (closed_left_bar[1][0] - closed_left_bar[0][0]) > 0.045
        and (open_left_bar[1][2] - open_left_bar[0][2]) > 0.045,
        "The latch handle should visibly rotate from horizontal to vertical over a quarter turn.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
