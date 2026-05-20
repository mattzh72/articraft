from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


FRAME_WIDTH = 1.48
LEG_WIDTH = 0.18
LEG_DEPTH = 0.22
LEG_HEIGHT = 1.62
LEG_CENTER_X = FRAME_WIDTH / 2.0 - LEG_WIDTH / 2.0
FOOT_WIDTH = 0.28
FOOT_DEPTH = 0.34
FOOT_THICKNESS = 0.03

BEAM_LENGTH = FRAME_WIDTH
BEAM_DEPTH = 0.26
BEAM_HEIGHT = 0.18
BEAM_CENTER_Z = LEG_HEIGHT + BEAM_HEIGHT / 2.0

FRAME_GUSSET_RUN = 0.14
FRAME_GUSSET_RISE = 0.14
FRAME_GUSSET_THICKNESS = 0.012

BEAM_RAIL_LENGTH = 1.12
BEAM_RAIL_WIDTH = 0.026
BEAM_RAIL_HEIGHT = 0.022
BEAM_RAIL_Y = 0.07
BEAM_RAIL_CENTER_Z = BEAM_CENTER_Z - BEAM_HEIGHT / 2.0 - BEAM_RAIL_HEIGHT / 2.0
BEAM_RAIL_BOTTOM_Z = BEAM_RAIL_CENTER_Z - BEAM_RAIL_HEIGHT / 2.0
BEAM_STOP_LENGTH = 0.035
BEAM_STOP_DEPTH = 0.18
BEAM_STOP_HEIGHT = 0.03
BEAM_STOP_CENTER_Z = BEAM_RAIL_BOTTOM_Z - BEAM_STOP_HEIGHT / 2.0
BEAM_STOP_CENTER_X = BEAM_RAIL_LENGTH / 2.0 + BEAM_STOP_LENGTH / 2.0

SHUTTLE_TRAVEL = 0.42
SHUTTLE_SHOE_LENGTH = 0.18
SHUTTLE_SHOE_DEPTH = 0.05
SHUTTLE_SHOE_HEIGHT = 0.024
SHUTTLE_SHOE_Y = BEAM_RAIL_Y
SHUTTLE_BRIDGE_LENGTH = 0.24
SHUTTLE_BRIDGE_DEPTH = 0.18
SHUTTLE_BRIDGE_THICKNESS = 0.022
SHUTTLE_BRIDGE_CENTER_Z = -(
    SHUTTLE_SHOE_HEIGHT + SHUTTLE_BRIDGE_THICKNESS / 2.0
)
SHUTTLE_VERTICAL_RAIL_WIDTH = 0.018
SHUTTLE_VERTICAL_RAIL_DEPTH = 0.012
SHUTTLE_VERTICAL_RAIL_LENGTH = 0.44
SHUTTLE_VERTICAL_RAIL_X = 0.045
SHUTTLE_VERTICAL_RAIL_Y = 0.05
SHUTTLE_VERTICAL_RAIL_CENTER_Z = -0.31
SHUTTLE_VERTICAL_RAIL_TOP_Z = (
    SHUTTLE_VERTICAL_RAIL_CENTER_Z + SHUTTLE_VERTICAL_RAIL_LENGTH / 2.0
)
SHUTTLE_VERTICAL_RAIL_BOTTOM_Z = (
    SHUTTLE_VERTICAL_RAIL_CENTER_Z - SHUTTLE_VERTICAL_RAIL_LENGTH / 2.0
)
SHUTTLE_RAIL_CAP_Y = 0.046
SHUTTLE_UPPER_CAP_Z = SHUTTLE_VERTICAL_RAIL_TOP_Z + 0.01
SHUTTLE_LOWER_STOP_Z = SHUTTLE_VERTICAL_RAIL_BOTTOM_Z - 0.011

TOOL_SLIDE_ORIGIN_Y = (
    SHUTTLE_VERTICAL_RAIL_Y + SHUTTLE_VERTICAL_RAIL_DEPTH / 2.0
)
TOOL_SLIDE_ORIGIN_Z = -0.12
TOOL_TRAVEL = 0.24
TOOL_GUIDE_WIDTH = 0.032
TOOL_GUIDE_DEPTH = 0.012
TOOL_GUIDE_HEIGHT = 0.16
TOOL_GUIDE_X = SHUTTLE_VERTICAL_RAIL_X
TOOL_GUIDE_CENTER_Z = -0.08


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_mesh(part, shape, *, mesh_name: str, material: str, name: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=name,
    )


def _frame_corner_gusset(sign_x: float, sign_y: float):
    y0 = (
        BEAM_DEPTH / 2.0 - FRAME_GUSSET_THICKNESS
        if sign_y > 0.0
        else -BEAM_DEPTH / 2.0
    )
    x0 = sign_x * (LEG_CENTER_X - LEG_WIDTH / 2.0)
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.0),
                (sign_x * FRAME_GUSSET_RUN, 0.0),
                (0.0, FRAME_GUSSET_RISE),
            ]
        )
        .close()
        .extrude(FRAME_GUSSET_THICKNESS)
        .translate((x0, y0, LEG_HEIGHT - FRAME_GUSSET_RISE))
    )


def _build_frame_structure():
    left_leg = (
        cq.Workplane("XY")
        .box(LEG_WIDTH, LEG_DEPTH, LEG_HEIGHT)
        .translate((-LEG_CENTER_X, 0.0, LEG_HEIGHT / 2.0))
    )
    right_leg = (
        cq.Workplane("XY")
        .box(LEG_WIDTH, LEG_DEPTH, LEG_HEIGHT)
        .translate((LEG_CENTER_X, 0.0, LEG_HEIGHT / 2.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
        .translate((0.0, 0.0, BEAM_CENTER_Z))
    )
    left_foot = (
        cq.Workplane("XY")
        .box(FOOT_WIDTH, FOOT_DEPTH, FOOT_THICKNESS)
        .translate((-LEG_CENTER_X, 0.0, FOOT_THICKNESS / 2.0))
    )
    right_foot = (
        cq.Workplane("XY")
        .box(FOOT_WIDTH, FOOT_DEPTH, FOOT_THICKNESS)
        .translate((LEG_CENTER_X, 0.0, FOOT_THICKNESS / 2.0))
    )

    structure = left_leg.union(right_leg).union(beam).union(left_foot).union(right_foot)
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            structure = structure.union(_frame_corner_gusset(sign_x, sign_y))
    return structure


def _shuttle_side_rib(sign_x: float):
    x0 = -0.056 if sign_x < 0.0 else 0.044
    return (
        cq.Workplane("YZ")
        .polyline([(0.0, -0.046), (0.055, -0.046), (0.0, -0.17)])
        .close()
        .extrude(0.012)
        .translate((x0, 0.0, 0.0))
    )


def _build_shuttle_body():
    upper_neck = (
        cq.Workplane("XY")
        .box(0.14, 0.08, 0.05)
        .translate((0.0, 0.0, -0.071))
    )
    center_spine = (
        cq.Workplane("XY")
        .box(0.10, 0.06, 0.22)
        .translate((0.0, 0.0, -0.176))
    )
    rail_plate = (
        cq.Workplane("XY")
        .box(0.12, 0.022, 0.38)
        .translate((0.0, 0.041, -0.29))
    )

    body = upper_neck.union(center_spine).union(rail_plate)
    body = body.union(_shuttle_side_rib(-1.0)).union(_shuttle_side_rib(1.0))
    return body


def _tool_side_rib(sign_x: float):
    x0 = -0.045 if sign_x < 0.0 else 0.035
    return (
        cq.Workplane("YZ")
        .polyline([(0.012, -0.03), (0.046, -0.03), (0.012, -0.175)])
        .close()
        .extrude(0.01)
        .translate((x0, 0.0, 0.0))
    )


def _build_tool_body():
    top_cap = (
        cq.Workplane("XY")
        .box(0.08, 0.026, 0.024)
        .translate((0.0, 0.025, -0.012))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.09, 0.034, 0.22)
        .translate((0.0, 0.029, -0.12))
    )
    front_plate = (
        cq.Workplane("XY")
        .box(0.11, 0.018, 0.14)
        .translate((0.0, 0.055, -0.135))
    )
    lower_nose = (
        cq.Workplane("XY")
        .box(0.06, 0.04, 0.05)
        .translate((0.0, 0.026, -0.215))
    )

    body = top_cap.union(spine).union(front_plate).union(lower_nose)
    body = body.union(_tool_side_rib(-1.0)).union(_tool_side_rib(1.0))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_portal")

    model.material("frame_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("rail_steel", rgba=(0.32, 0.35, 0.39, 1.0))
    model.material("carriage_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("carriage_light", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("cover_blue", rgba=(0.20, 0.39, 0.65, 1.0))
    model.material("cap_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("tool_steel", rgba=(0.46, 0.48, 0.52, 1.0))

    frame = model.part("frame")
    _add_mesh(
        frame,
        _build_frame_structure(),
        mesh_name="frame_structure",
        material="frame_gray",
        name="frame_structure",
    )
    _add_box(
        frame,
        (0.46, 0.028, 0.09),
        (0.0, BEAM_DEPTH / 2.0 + 0.014, BEAM_CENTER_Z),
        material="cover_blue",
        name="beam_service_cover",
    )
    _add_box(
        frame,
        (BEAM_RAIL_LENGTH, BEAM_RAIL_WIDTH, BEAM_RAIL_HEIGHT),
        (0.0, BEAM_RAIL_Y, BEAM_RAIL_CENTER_Z),
        material="rail_steel",
        name="left_beam_rail",
    )
    _add_box(
        frame,
        (BEAM_RAIL_LENGTH, BEAM_RAIL_WIDTH, BEAM_RAIL_HEIGHT),
        (0.0, -BEAM_RAIL_Y, BEAM_RAIL_CENTER_Z),
        material="rail_steel",
        name="right_beam_rail",
    )
    _add_box(
        frame,
        (BEAM_STOP_LENGTH, BEAM_STOP_DEPTH, BEAM_STOP_HEIGHT),
        (BEAM_STOP_CENTER_X, 0.0, BEAM_STOP_CENTER_Z),
        material="cap_black",
        name="right_stop",
    )
    _add_box(
        frame,
        (BEAM_STOP_LENGTH, BEAM_STOP_DEPTH, BEAM_STOP_HEIGHT),
        (-BEAM_STOP_CENTER_X, 0.0, BEAM_STOP_CENTER_Z),
        material="cap_black",
        name="left_stop",
    )
    _add_box(
        frame,
        (LEG_WIDTH, LEG_DEPTH, LEG_HEIGHT),
        (-LEG_CENTER_X, 0.0, LEG_HEIGHT / 2.0),
        material="frame_gray",
        name="left_leg",
    )
    _add_box(
        frame,
        (LEG_WIDTH, LEG_DEPTH, LEG_HEIGHT),
        (LEG_CENTER_X, 0.0, LEG_HEIGHT / 2.0),
        material="frame_gray",
        name="right_leg",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FOOT_DEPTH, LEG_HEIGHT + BEAM_HEIGHT)),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, (LEG_HEIGHT + BEAM_HEIGHT) / 2.0)),
    )

    shuttle = model.part("shuttle")
    _add_box(
        shuttle,
        (SHUTTLE_SHOE_LENGTH, SHUTTLE_SHOE_DEPTH, SHUTTLE_SHOE_HEIGHT),
        (0.0, SHUTTLE_SHOE_Y, -SHUTTLE_SHOE_HEIGHT / 2.0),
        material="rail_steel",
        name="left_bearing_shoe",
    )
    _add_box(
        shuttle,
        (SHUTTLE_SHOE_LENGTH, SHUTTLE_SHOE_DEPTH, SHUTTLE_SHOE_HEIGHT),
        (0.0, -SHUTTLE_SHOE_Y, -SHUTTLE_SHOE_HEIGHT / 2.0),
        material="rail_steel",
        name="right_bearing_shoe",
    )
    _add_box(
        shuttle,
        (SHUTTLE_BRIDGE_LENGTH, SHUTTLE_BRIDGE_DEPTH, SHUTTLE_BRIDGE_THICKNESS),
        (0.0, 0.0, SHUTTLE_BRIDGE_CENTER_Z),
        material="carriage_dark",
        name="upper_bridge",
    )
    _add_mesh(
        shuttle,
        _build_shuttle_body(),
        mesh_name="shuttle_body",
        material="carriage_dark",
        name="shuttle_body",
    )
    _add_box(
        shuttle,
        (0.16, 0.028, 0.18),
        (0.0, -0.054, -0.18),
        material="cover_blue",
        name="shuttle_cover",
    )
    _add_box(
        shuttle,
        (
            SHUTTLE_VERTICAL_RAIL_WIDTH,
            SHUTTLE_VERTICAL_RAIL_DEPTH,
            SHUTTLE_VERTICAL_RAIL_LENGTH,
        ),
        (
            SHUTTLE_VERTICAL_RAIL_X,
            SHUTTLE_VERTICAL_RAIL_Y,
            SHUTTLE_VERTICAL_RAIL_CENTER_Z,
        ),
        material="rail_steel",
        name="left_vertical_rail",
    )
    _add_box(
        shuttle,
        (
            SHUTTLE_VERTICAL_RAIL_WIDTH,
            SHUTTLE_VERTICAL_RAIL_DEPTH,
            SHUTTLE_VERTICAL_RAIL_LENGTH,
        ),
        (
            -SHUTTLE_VERTICAL_RAIL_X,
            SHUTTLE_VERTICAL_RAIL_Y,
            SHUTTLE_VERTICAL_RAIL_CENTER_Z,
        ),
        material="rail_steel",
        name="right_vertical_rail",
    )
    _add_box(
        shuttle,
        (0.14, 0.018, 0.02),
        (0.0, SHUTTLE_RAIL_CAP_Y, SHUTTLE_UPPER_CAP_Z),
        material="cap_black",
        name="upper_rail_cap",
    )
    _add_box(
        shuttle,
        (0.13, 0.018, 0.022),
        (0.0, SHUTTLE_RAIL_CAP_Y, SHUTTLE_LOWER_STOP_Z),
        material="cap_black",
        name="lower_stop",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.58)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.02, -0.29)),
    )

    tool_carriage = model.part("tool_carriage")
    _add_box(
        tool_carriage,
        (TOOL_GUIDE_WIDTH, TOOL_GUIDE_DEPTH, TOOL_GUIDE_HEIGHT),
        (TOOL_GUIDE_X, TOOL_GUIDE_DEPTH / 2.0, TOOL_GUIDE_CENTER_Z),
        material="rail_steel",
        name="left_guide_block",
    )
    _add_box(
        tool_carriage,
        (TOOL_GUIDE_WIDTH, TOOL_GUIDE_DEPTH, TOOL_GUIDE_HEIGHT),
        (-TOOL_GUIDE_X, TOOL_GUIDE_DEPTH / 2.0, TOOL_GUIDE_CENTER_Z),
        material="rail_steel",
        name="right_guide_block",
    )
    _add_mesh(
        tool_carriage,
        _build_tool_body(),
        mesh_name="tool_carriage_body",
        material="carriage_light",
        name="tool_body",
    )
    _add_box(
        tool_carriage,
        (0.09, 0.014, 0.17),
        (0.0, 0.071, -0.12),
        material="cover_blue",
        name="tool_cover",
    )
    _add_box(
        tool_carriage,
        (0.056, 0.048, 0.04),
        (0.0, 0.03, -0.25),
        material="carriage_dark",
        name="probe_mount",
    )
    _add_cylinder(
        tool_carriage,
        radius=0.014,
        length=0.09,
        xyz=(0.0, 0.03, -0.315),
        material="tool_steel",
        name="probe_barrel",
    )
    _add_cylinder(
        tool_carriage,
        radius=0.007,
        length=0.03,
        xyz=(0.0, 0.03, -0.375),
        material="tool_steel",
        name="probe_tip",
    )
    tool_carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.39)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.03, -0.195)),
    )

    model.articulation(
        "frame_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, BEAM_RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SHUTTLE_TRAVEL,
            upper=SHUTTLE_TRAVEL,
            effort=2200.0,
            velocity=0.7,
        ),
    )
    model.articulation(
        "shuttle_to_tool_carriage",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=tool_carriage,
        origin=Origin(xyz=(0.0, TOOL_SLIDE_ORIGIN_Y, TOOL_SLIDE_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TOOL_TRAVEL,
            effort=900.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shuttle = object_model.get_part("shuttle")
    tool_carriage = object_model.get_part("tool_carriage")
    beam_slide = object_model.get_articulation("frame_to_shuttle")
    tool_slide = object_model.get_articulation("shuttle_to_tool_carriage")

    left_beam_rail = frame.get_visual("left_beam_rail")
    right_beam_rail = frame.get_visual("right_beam_rail")
    left_stop = frame.get_visual("left_stop")
    right_stop = frame.get_visual("right_stop")

    left_bearing_shoe = shuttle.get_visual("left_bearing_shoe")
    right_bearing_shoe = shuttle.get_visual("right_bearing_shoe")
    upper_bridge = shuttle.get_visual("upper_bridge")
    left_vertical_rail = shuttle.get_visual("left_vertical_rail")
    right_vertical_rail = shuttle.get_visual("right_vertical_rail")
    lower_stop = shuttle.get_visual("lower_stop")

    left_guide_block = tool_carriage.get_visual("left_guide_block")
    right_guide_block = tool_carriage.get_visual("right_guide_block")
    tool_cover = tool_carriage.get_visual("tool_cover")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        shuttle,
        frame,
        elem_a=left_bearing_shoe,
        elem_b=left_beam_rail,
        name="left_shoe_supported_by_left_beam_rail",
    )
    ctx.expect_contact(
        shuttle,
        frame,
        elem_a=right_bearing_shoe,
        elem_b=right_beam_rail,
        name="right_shoe_supported_by_right_beam_rail",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a=left_guide_block,
        elem_b=left_vertical_rail,
        name="left_tool_block_supported_by_left_vertical_rail",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a=right_guide_block,
        elem_b=right_vertical_rail,
        name="right_tool_block_supported_by_right_vertical_rail",
    )

    with ctx.pose({beam_slide: beam_slide.motion_limits.upper, tool_slide: 0.0}):
        ctx.expect_gap(
            frame,
            shuttle,
            axis="x",
            min_gap=0.015,
            positive_elem=right_stop,
            negative_elem=upper_bridge,
            name="shuttle_clears_right_stop_at_max_travel",
        )

    with ctx.pose({beam_slide: beam_slide.motion_limits.lower, tool_slide: 0.0}):
        ctx.expect_gap(
            shuttle,
            frame,
            axis="x",
            min_gap=0.015,
            positive_elem=upper_bridge,
            negative_elem=left_stop,
            name="shuttle_clears_left_stop_at_max_travel",
        )

    with ctx.pose({beam_slide: 0.0, tool_slide: 0.0}):
        ctx.expect_gap(
            shuttle,
            tool_carriage,
            axis="z",
            min_gap=0.03,
            positive_elem=upper_bridge,
            negative_elem=tool_cover,
            name="tool_carriage_clears_shuttle_housing_at_top",
        )

    with ctx.pose({beam_slide: 0.0, tool_slide: tool_slide.motion_limits.upper}):
        ctx.expect_gap(
            tool_carriage,
            shuttle,
            axis="z",
            min_gap=0.008,
            positive_elem=left_guide_block,
            negative_elem=lower_stop,
            name="tool_carriage_stays_above_lower_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
