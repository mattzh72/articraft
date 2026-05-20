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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translate_profile(profile: list[tuple[float, float]], dx: float = 0.0, dy: float = 0.0) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_service_hatch")

    frame_color = model.material("frame_paint", rgba=(0.90, 0.90, 0.88, 1.0))
    panel_color = model.material("panel_paint", rgba=(0.82, 0.84, 0.85, 1.0))
    hardware_color = model.material("hardware", rgba=(0.24, 0.26, 0.29, 1.0))

    outer_width = 0.62
    outer_height = 0.44
    frame_depth = 0.025

    opening_width = 0.54
    opening_height = 0.36

    panel_side_gap = 0.004
    panel_top_gap = 0.004
    panel_bottom_gap = 0.004
    panel_width = opening_width - 2.0 * panel_side_gap
    panel_height = opening_height - panel_top_gap - panel_bottom_gap
    panel_thickness = 0.016
    panel_inset = 0.002

    hinge_height = opening_height * 0.5 - panel_top_gap
    latch_offset_from_bottom = 0.055
    latch_center_z = -panel_height + latch_offset_from_bottom
    latch_hole_diameter = 0.016

    frame = model.part("frame")
    frame_outer_profile = rounded_rect_profile(outer_width, outer_height, 0.012, corner_segments=8)
    frame_inner_profile = rounded_rect_profile(opening_width, opening_height, 0.008, corner_segments=8)
    frame_ring_geometry = ExtrudeWithHolesGeometry(
        frame_outer_profile,
        [frame_inner_profile],
        frame_depth,
        cap=True,
        center=True,
        closed=True,
    )
    frame_ring_geometry.rotate_x(math.pi / 2.0)
    frame_ring_geometry.translate(0.0, -frame_depth * 0.5, 0.0)
    frame.visual(
        mesh_from_geometry(frame_ring_geometry, "frame_ring"),
        material=frame_color,
        name="frame_ring",
    )
    frame.visual(
        Box((opening_width - 0.10, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.027, hinge_height - 0.004)),
        material=frame_color,
        name="frame_hinge_mount",
    )

    panel = model.part("panel")
    panel_outer_profile = rounded_rect_profile(panel_width, panel_height, 0.008, corner_segments=8)
    latch_hole_profile = _translate_profile(
        superellipse_profile(latch_hole_diameter, latch_hole_diameter, exponent=2.0, segments=24),
        dy=latch_center_z + panel_height * 0.5,
    )
    panel_leaf_geometry = ExtrudeWithHolesGeometry(
        panel_outer_profile,
        [latch_hole_profile],
        panel_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    panel_leaf_geometry.rotate_x(math.pi / 2.0)
    panel_leaf_geometry.translate(0.0, -panel_thickness * 0.5, 0.0)
    panel.visual(
        mesh_from_geometry(panel_leaf_geometry, "panel_leaf"),
        origin=Origin(xyz=(0.0, -panel_inset, -panel_height * 0.5)),
        material=panel_color,
        name="panel_leaf",
    )
    panel.visual(
        Box((opening_width - 0.14, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.021, -0.009)),
        material=panel_color,
        name="panel_hinge_leaf",
    )
    panel.visual(
        Box((panel_width - 0.07, 0.003, panel_height - 0.07)),
        origin=Origin(
            xyz=(0.0, -panel_inset - 0.0115, -panel_height * 0.5),
        ),
        material="frame_paint",
        name="panel_stiffener",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="latch_shaft",
    )
    latch.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="latch_bezel",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_color,
        name="latch_hub",
    )
    latch.visual(
        Box((0.058, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=hardware_color,
        name="latch_handle",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, hinge_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(0.0, -panel_inset - panel_thickness * 0.5, latch_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    latch_joint = object_model.get_articulation("panel_to_latch")

    ctx.expect_contact(
        latch,
        panel,
        elem_a="latch_bezel",
        elem_b="panel_leaf",
        contact_tol=0.0005,
        name="latch bezel seats on the panel face",
    )
    ctx.expect_within(
        latch,
        panel,
        axes="xz",
        inner_elem="latch_bezel",
        outer_elem="panel_leaf",
        margin=0.001,
        name="latch stays within the panel outline",
    )
    ctx.expect_contact(
        frame,
        panel,
        elem_a="frame_hinge_mount",
        elem_b="panel_hinge_leaf",
        contact_tol=0.0005,
        name="panel hinge leaf seats against the fixed frame mount",
    )

    with ctx.pose({panel_hinge: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_leaf")

    with ctx.pose({panel_hinge: panel_hinge.motion_limits.upper}):
        open_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_leaf")

    panel_opens_upward = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.18
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.10
    )
    ctx.check(
        "panel opens upward on the top hinge",
        panel_opens_upward,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({panel_hinge: 0.0, latch_joint: 0.0}):
        latch_closed_aabb = ctx.part_element_world_aabb(latch, elem="latch_handle")

    with ctx.pose({panel_hinge: 0.0, latch_joint: latch_joint.motion_limits.upper}):
        latch_turned_aabb = ctx.part_element_world_aabb(latch, elem="latch_handle")

    latch_rotates_on_shaft = False
    if latch_closed_aabb is not None and latch_turned_aabb is not None:
        closed_x_span = latch_closed_aabb[1][0] - latch_closed_aabb[0][0]
        closed_z_span = latch_closed_aabb[1][2] - latch_closed_aabb[0][2]
        turned_x_span = latch_turned_aabb[1][0] - latch_turned_aabb[0][0]
        turned_z_span = latch_turned_aabb[1][2] - latch_turned_aabb[0][2]
        latch_rotates_on_shaft = closed_x_span > closed_z_span * 2.5 and turned_z_span > turned_x_span * 2.5

    ctx.check(
        "latch rotates on its short shaft",
        latch_rotates_on_shaft,
        details=f"closed={latch_closed_aabb}, turned={latch_turned_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
