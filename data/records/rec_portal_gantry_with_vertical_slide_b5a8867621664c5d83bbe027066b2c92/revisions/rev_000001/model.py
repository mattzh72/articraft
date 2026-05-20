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


PORTAL_WIDTH = 2.60
SIDE_FRAME_X = 1.22
SIDE_POST_Y = 0.23
BASE_SKID = (0.18, 0.72, 0.08)
POST_SIZE = (0.12, 0.12, 1.62)
TOP_TIE = (0.16, 0.56, 0.14)
BEAM_SIZE = (PORTAL_WIDTH, 0.24, 0.20)
BEAM_SEAT = (0.18, 0.24, 0.16)

BEAM_CENTER_Z = BASE_SKID[2] + POST_SIZE[2] + TOP_TIE[2] + (BEAM_SIZE[2] / 2.0)
POST_CENTER_Z = BASE_SKID[2] + (POST_SIZE[2] / 2.0)
TOP_TIE_CENTER_Z = BASE_SKID[2] + POST_SIZE[2] + (TOP_TIE[2] / 2.0)
BEAM_BOTTOM_Z = BEAM_CENTER_Z - (BEAM_SIZE[2] / 2.0)

BEAM_RAIL_SIZE = (2.16, 0.03, 0.02)
BEAM_RAIL_Y = 0.08
BEAM_RAIL_Z = BEAM_BOTTOM_Z - (BEAM_RAIL_SIZE[2] / 2.0)
BEAM_RAIL_BOTTOM_Z = BEAM_RAIL_Z - (BEAM_RAIL_SIZE[2] / 2.0)

SHUTTLE_TRAVEL = 0.78
SHUTTLE_BEARING = (0.085, 0.05, 0.05)
SHUTTLE_BEARING_X = 0.055
FRONT_BEARING_Y = 0.07
REAR_BEARING_Y = -BEAM_RAIL_Y
SHUTTLE_BEARING_Z = -(SHUTTLE_BEARING[2] / 2.0)

SHUTTLE_VERTICAL_RAIL = (0.02, 0.02, 0.42)
SHUTTLE_VERTICAL_RAIL_X = 0.05
SHUTTLE_VERTICAL_RAIL_Y = 0.04
SHUTTLE_VERTICAL_RAIL_Z = -0.26

TOOL_GUIDE_SHOE = (0.034, 0.024, 0.12)
TOOL_GUIDE_X = SHUTTLE_VERTICAL_RAIL_X
TOOL_GUIDE_Y = 0.0
TOOL_GUIDE_Z = -0.06

TOOL_SLIDE_ORIGIN = (0.0, 0.062, -0.05)
TOOL_TRAVEL = 0.62


def _wp_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _frame_shape() -> cq.Workplane:
    shapes = [
        _wp_box(BEAM_SIZE, (0.0, 0.0, BEAM_CENTER_Z)),
        _wp_box(BEAM_SEAT, (SIDE_FRAME_X, 0.0, BEAM_BOTTOM_Z + (BEAM_SEAT[2] / 2.0))),
        _wp_box(BEAM_SEAT, (-SIDE_FRAME_X, 0.0, BEAM_BOTTOM_Z + (BEAM_SEAT[2] / 2.0))),
    ]

    for x_sign in (-1.0, 1.0):
        x = x_sign * SIDE_FRAME_X
        shapes.append(_wp_box(BASE_SKID, (x, 0.0, BASE_SKID[2] / 2.0)))
        shapes.append(_wp_box(TOP_TIE, (x, 0.0, TOP_TIE_CENTER_Z)))
        for y_sign in (-1.0, 1.0):
            shapes.append(
                _wp_box(
                    POST_SIZE,
                    (x, y_sign * SIDE_POST_Y, POST_CENTER_Z),
                )
            )

        gusset = (
            cq.Workplane("YZ")
            .moveTo(-0.28, BASE_SKID[2])
            .lineTo(-0.28, BASE_SKID[2] + 0.06)
            .lineTo(0.10, 1.32)
            .lineTo(0.10, 1.18)
            .close()
            .extrude(0.04)
            .translate((x - 0.02, 0.0, 0.0))
        )
        shapes.append(gusset)

    return _union_all(shapes)


def _shuttle_body_shape() -> cq.Workplane:
    return _union_all(
        [
            _wp_box((0.22, 0.08, 0.03), (0.0, 0.0, -0.04)),
            _wp_box((0.18, 0.08, 0.10), (0.0, 0.0, -0.10)),
            _wp_box((0.10, 0.10, 0.05), (0.0, -0.01, -0.07)),
            _wp_box((0.16, 0.06, 0.12), (0.0, 0.015, -0.19)),
            _wp_box((0.16, 0.03, 0.46), (0.0, 0.015, -0.27)),
            _wp_box((0.12, 0.03, 0.12), (0.0, 0.015, -0.21)),
        ]
    )


def _tool_carriage_shape() -> cq.Workplane:
    return _union_all(
        [
            _wp_box((0.13, 0.045, 0.16), (0.0, 0.0345, -0.10)),
            _wp_box((0.10, 0.038, 0.32), (0.0, 0.030, -0.32)),
            _wp_box((0.10, 0.12, 0.10), (0.0, 0.060, -0.52)),
            _wp_box((0.06, 0.02, 0.18), (0.0, 0.015, -0.23)),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_portal")

    model.material("frame_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("rail_steel", rgba=(0.42, 0.45, 0.49, 1.0))
    model.material("shuttle_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("carriage_light", rgba=(0.83, 0.84, 0.86, 1.0))
    model.material("accent_blue", rgba=(0.14, 0.40, 0.75, 1.0))
    model.material("probe_steel", rgba=(0.54, 0.56, 0.60, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "portal_frame"),
        material="frame_gray",
        name="portal_structure",
    )
    frame.visual(
        Box(BEAM_RAIL_SIZE),
        origin=Origin(xyz=(0.0, BEAM_RAIL_Y, BEAM_RAIL_Z)),
        material="rail_steel",
        name="front_rail",
    )
    frame.visual(
        Box(BEAM_RAIL_SIZE),
        origin=Origin(xyz=(0.0, -BEAM_RAIL_Y, BEAM_RAIL_Z)),
        material="rail_steel",
        name="rear_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((PORTAL_WIDTH, BASE_SKID[1], BEAM_CENTER_Z + (BEAM_SIZE[2] / 2.0))),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, (BEAM_CENTER_Z + (BEAM_SIZE[2] / 2.0)) / 2.0)),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        mesh_from_cadquery(_shuttle_body_shape(), "portal_shuttle"),
        material="shuttle_dark",
        name="shuttle_body",
    )
    shuttle.visual(
        Box(SHUTTLE_BEARING),
        origin=Origin(xyz=(SHUTTLE_BEARING_X, FRONT_BEARING_Y, SHUTTLE_BEARING_Z)),
        material="accent_blue",
        name="front_right_bearing",
    )
    shuttle.visual(
        Box(SHUTTLE_BEARING),
        origin=Origin(xyz=(-SHUTTLE_BEARING_X, FRONT_BEARING_Y, SHUTTLE_BEARING_Z)),
        material="accent_blue",
        name="front_left_bearing",
    )
    shuttle.visual(
        Box(SHUTTLE_BEARING),
        origin=Origin(xyz=(SHUTTLE_BEARING_X, REAR_BEARING_Y, SHUTTLE_BEARING_Z)),
        material="accent_blue",
        name="rear_right_bearing",
    )
    shuttle.visual(
        Box(SHUTTLE_BEARING),
        origin=Origin(xyz=(-SHUTTLE_BEARING_X, REAR_BEARING_Y, SHUTTLE_BEARING_Z)),
        material="accent_blue",
        name="rear_left_bearing",
    )
    shuttle.visual(
        Box(SHUTTLE_VERTICAL_RAIL),
        origin=Origin(
            xyz=(SHUTTLE_VERTICAL_RAIL_X, SHUTTLE_VERTICAL_RAIL_Y, SHUTTLE_VERTICAL_RAIL_Z)
        ),
        material="rail_steel",
        name="right_slide_rail",
    )
    shuttle.visual(
        Box(SHUTTLE_VERTICAL_RAIL),
        origin=Origin(
            xyz=(-SHUTTLE_VERTICAL_RAIL_X, SHUTTLE_VERTICAL_RAIL_Y, SHUTTLE_VERTICAL_RAIL_Z)
        ),
        material="rail_steel",
        name="left_slide_rail",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.52)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.02, -0.24)),
    )

    tool_carriage = model.part("tool_carriage")
    tool_carriage.visual(
        mesh_from_cadquery(_tool_carriage_shape(), "tool_carriage"),
        material="carriage_light",
        name="carriage_body",
    )
    tool_carriage.visual(
        Box(TOOL_GUIDE_SHOE),
        origin=Origin(xyz=(-TOOL_GUIDE_X, TOOL_GUIDE_Y, TOOL_GUIDE_Z)),
        material="accent_blue",
        name="left_guide_shoe",
    )
    tool_carriage.visual(
        Box(TOOL_GUIDE_SHOE),
        origin=Origin(xyz=(TOOL_GUIDE_X, TOOL_GUIDE_Y, TOOL_GUIDE_Z)),
        material="accent_blue",
        name="right_guide_shoe",
    )
    tool_carriage.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.0, 0.115, -0.56), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="probe_steel",
        name="probe_tip",
    )
    tool_carriage.inertial = Inertial.from_geometry(
        Box((0.13, 0.10, 0.60)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.04, -0.30)),
    )

    model.articulation(
        "frame_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, BEAM_RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.8,
            lower=-SHUTTLE_TRAVEL,
            upper=SHUTTLE_TRAVEL,
        ),
    )
    model.articulation(
        "shuttle_to_tool_carriage",
        ArticulationType.PRISMATIC,
        parent=shuttle,
        child=tool_carriage,
        origin=Origin(xyz=TOOL_SLIDE_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.45,
            lower=0.0,
            upper=TOOL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    shuttle = object_model.get_part("shuttle")
    tool_carriage = object_model.get_part("tool_carriage")
    shuttle_slide = object_model.get_articulation("frame_to_shuttle")
    tool_slide = object_model.get_articulation("shuttle_to_tool_carriage")

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
        shuttle,
        frame,
        elem_a="front_left_bearing",
        elem_b="front_rail",
        name="shuttle front bearing rides on front rail",
    )
    ctx.expect_contact(
        shuttle,
        frame,
        elem_a="rear_left_bearing",
        elem_b="rear_rail",
        name="shuttle rear bearing rides on rear rail",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a="left_guide_shoe",
        elem_b="left_slide_rail",
        name="tool carriage left shoe rides on left slide rail",
    )
    ctx.expect_contact(
        tool_carriage,
        shuttle,
        elem_a="right_guide_shoe",
        elem_b="right_slide_rail",
        name="tool carriage right shoe rides on right slide rail",
    )

    ctx.check(
        "shuttle slide axis points across the beam",
        tuple(shuttle_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis was {tuple(shuttle_slide.axis)}",
    )
    ctx.check(
        "tool carriage slide axis points downward",
        tuple(tool_slide.axis) == (0.0, 0.0, -1.0),
        details=f"axis was {tuple(tool_slide.axis)}",
    )

    with ctx.pose({shuttle_slide: shuttle_slide.motion_limits.lower}):
        shuttle_left = ctx.part_world_position(shuttle)
    with ctx.pose({shuttle_slide: shuttle_slide.motion_limits.upper}):
        shuttle_right = ctx.part_world_position(shuttle)
    ctx.check(
        "positive shuttle travel moves rightward across the portal",
        shuttle_left is not None
        and shuttle_right is not None
        and shuttle_right[0] > shuttle_left[0] + 1.4,
        details=f"left pose={shuttle_left}, right pose={shuttle_right}",
    )

    with ctx.pose({tool_slide: tool_slide.motion_limits.lower}):
        tool_high = ctx.part_world_position(tool_carriage)
    with ctx.pose({tool_slide: tool_slide.motion_limits.upper}):
        tool_low = ctx.part_world_position(tool_carriage)
    ctx.check(
        "positive tool travel lowers the tool carriage",
        tool_high is not None
        and tool_low is not None
        and tool_low[2] < tool_high[2] - 0.5,
        details=f"high pose={tool_high}, low pose={tool_low}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
