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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    frame_paint = model.material("frame_paint", rgba=(0.35, 0.39, 0.42, 1.0))
    door_paint = model.material("door_paint", rgba=(0.71, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.09, 0.10, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.12, 0.13, 0.13, 1.0))

    outer_w = 0.92
    outer_h = 0.72
    opening_w = 0.58
    opening_h = 0.44
    face_t = 0.020
    sleeve_depth = 0.100
    sleeve_overlap = 0.004

    door_w = 0.566
    door_h = 0.426
    door_t = 0.016
    hinge_axis_x = -door_w * 0.5
    hinge_axis_y = 0.018
    hinge_radius = 0.012

    side_border = (outer_w - opening_w) * 0.5
    top_border = (outer_h - opening_h) * 0.5
    sleeve_center_y = -face_t * 0.5 - sleeve_depth * 0.5 + sleeve_overlap

    frame = model.part("frame")

    frame_face_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_w, outer_h, radius=0.016, corner_segments=8),
        [rounded_rect_profile(opening_w, opening_h, radius=0.008, corner_segments=8)],
        height=face_t,
        center=True,
    )
    frame.visual(
        mesh_from_geometry(frame_face_geom, "frame_face"),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=frame_paint,
        name="frame_face",
    )
    frame.visual(
        Box((side_border, sleeve_depth, outer_h)),
        origin=Origin(xyz=(-(opening_w * 0.5 + side_border * 0.5), sleeve_center_y, 0.0)),
        material=frame_paint,
        name="left_jamb",
    )
    frame.visual(
        Box((side_border, sleeve_depth, outer_h)),
        origin=Origin(xyz=((opening_w * 0.5 + side_border * 0.5), sleeve_center_y, 0.0)),
        material=frame_paint,
        name="right_jamb",
    )
    frame.visual(
        Box((opening_w + 0.020, sleeve_depth, top_border)),
        origin=Origin(xyz=(0.0, sleeve_center_y, opening_h * 0.5 + top_border * 0.5)),
        material=frame_paint,
        name="top_header",
    )
    frame.visual(
        Box((opening_w + 0.020, sleeve_depth, top_border)),
        origin=Origin(xyz=(0.0, sleeve_center_y, -(opening_h * 0.5 + top_border * 0.5))),
        material=frame_paint,
        name="bottom_sill",
    )
    frame.visual(
        Box((0.048, 0.032, opening_h + 0.048)),
        origin=Origin(xyz=(hinge_axis_x - 0.038, 0.000, 0.0)),
        material=dark_steel,
        name="hinge_stile",
    )
    frame.visual(
        Box((0.022, 0.022, 0.046)),
        origin=Origin(xyz=(hinge_axis_x - 0.017, 0.010, -0.0765)),
        material=dark_steel,
        name="hinge_leaf_lower",
    )
    frame.visual(
        Box((0.022, 0.022, 0.046)),
        origin=Origin(xyz=(hinge_axis_x - 0.017, 0.010, 0.0765)),
        material=dark_steel,
        name="hinge_leaf_upper",
    )
    frame.visual(
        Box((0.028, 0.028, 0.200)),
        origin=Origin(xyz=(opening_w * 0.5 + 0.014, 0.004, 0.0)),
        material=dark_steel,
        name="latch_strike",
    )
    frame.visual(
        Box((opening_w - 0.020, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, opening_h * 0.5 - 0.010)),
        material=gasket_dark,
        name="top_seal",
    )
    frame.visual(
        Box((opening_w - 0.020, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, -(opening_h * 0.5 - 0.010))),
        material=gasket_dark,
        name="bottom_seal",
    )
    frame.visual(
        Cylinder(radius=hinge_radius, length=0.054),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.0765)),
        material=dark_steel,
        name="frame_knuckle_lower",
    )
    frame.visual(
        Cylinder(radius=hinge_radius, length=0.054),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0765)),
        material=dark_steel,
        name="frame_knuckle_upper",
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5, -hinge_axis_y, 0.0)),
        material=door_paint,
        name="door_shell",
    )
    door.visual(
        Box((door_w - 0.090, 0.012, door_h - 0.090)),
        origin=Origin(xyz=(door_w * 0.5, -0.032, 0.0)),
        material=door_paint,
        name="door_stiffener",
    )
    door.visual(
        Box((0.024, 0.028, 0.150)),
        origin=Origin(xyz=(door_w - 0.068, 0.016, 0.0)),
        material=handle_black,
        name="latch_handle",
    )
    door.visual(
        Box((0.022, 0.012, 0.110)),
        origin=Origin(xyz=(door_w - 0.052, -0.004, 0.0)),
        material=dark_steel,
        name="latch_case",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.153)),
        material=dark_steel,
        name="door_knuckle_lower",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="door_knuckle_mid",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=dark_steel,
        name="door_knuckle_upper",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
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
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("frame_to_door")

    ctx.expect_gap(
        door,
        frame,
        axis="x",
        positive_elem="door_shell",
        negative_elem="left_jamb",
        min_gap=0.005,
        max_gap=0.012,
        name="closed door clears left jamb",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="x",
        positive_elem="right_jamb",
        negative_elem="door_shell",
        min_gap=0.005,
        max_gap=0.012,
        name="closed door clears right jamb",
    )
    ctx.expect_gap(
        frame,
        door,
        axis="z",
        positive_elem="top_header",
        negative_elem="door_shell",
        min_gap=0.005,
        max_gap=0.012,
        name="closed door clears top header",
    )
    ctx.expect_gap(
        door,
        frame,
        axis="z",
        positive_elem="door_shell",
        negative_elem="bottom_sill",
        min_gap=0.005,
        max_gap=0.012,
        name="closed door clears bottom sill",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="z",
        elem_a="latch_handle",
        elem_b="latch_strike",
        min_overlap=0.120,
        name="latch handle lines up with strike height",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="latch_handle")
    with ctx.pose({hinge: 1.55}):
        open_handle = ctx.part_element_world_aabb(door, elem="latch_handle")
    ctx.check(
        "door swings outward",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1][1] > closed_handle[1][1] + 0.30,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
