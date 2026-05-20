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


FRAME_W = 0.88
FRAME_H = 1.12
FRAME_DEPTH = 0.06
POST_W = 0.08
BEAM_H = 0.08

FOOT_LEN = 0.16
FOOT_W = 0.12
FOOT_H = 0.025

H_RAIL_X = 0.105
H_RAIL_TOP_Z = 0.82
H_RAIL_BOT_Z = 0.70
H_RAIL_MID_Z = (H_RAIL_TOP_Z + H_RAIL_BOT_Z) / 2.0
H_RAIL_R = 0.011
H_RAIL_LEN = 0.72
H_TRAVEL = 0.23

CARRIAGE_BLOCK_X = 0.060
CARRIAGE_BLOCK_Y = 0.120
CARRIAGE_BLOCK_Z = 0.050
CARRIAGE_BLOCK_CENTER_X = H_RAIL_R + (CARRIAGE_BLOCK_X / 2.0)

CARRIAGE_BRIDGE_X = 0.026
CARRIAGE_BRIDGE_Y = 0.130
CARRIAGE_BRIDGE_Z = 0.190
CARRIAGE_BRIDGE_CENTER_X = 0.058

CARRIAGE_NOSE_X = 0.045
CARRIAGE_NOSE_Y = 0.120
CARRIAGE_NOSE_Z = 0.070
CARRIAGE_NOSE_CENTER_X = 0.080

CARRIAGE_SPINE_X = 0.020
CARRIAGE_SPINE_Y = 0.120
CARRIAGE_SPINE_Z = 0.120
CARRIAGE_SPINE_CENTER_X = 0.074

SLIDE_JOINT_X = 0.115
SLIDE_JOINT_Z = -0.03

V_RAIL_R = 0.010
V_RAIL_LEN = 0.50
V_RAIL_Y = 0.033
V_RAIL_CENTER_Z = SLIDE_JOINT_Z - 0.25
V_TRAVEL = 0.20

V_RAIL_SUPPORT_X = 0.020
V_RAIL_SUPPORT_Y = 0.100
V_RAIL_SUPPORT_Z = 0.250
V_RAIL_SUPPORT_CENTER_X = SLIDE_JOINT_X - (V_RAIL_R + (V_RAIL_SUPPORT_X / 2.0))
V_RAIL_SUPPORT_CENTER_Z = (V_RAIL_CENTER_Z + (SLIDE_JOINT_Z - (V_RAIL_LEN / 2.0))) / 2.0

SLIDE_GUIDE_X = 0.036
SLIDE_GUIDE_Y = 0.090
SLIDE_GUIDE_Z = 0.070
SLIDE_GUIDE_CENTER_X = V_RAIL_R + (SLIDE_GUIDE_X / 2.0)

SLIDE_SPINE_X = 0.024
SLIDE_SPINE_Y = 0.090
SLIDE_SPINE_Z = 0.190
SLIDE_SPINE_CENTER_X = 0.040

SLIDE_LOWER_X = 0.030
SLIDE_LOWER_Y = 0.080
SLIDE_LOWER_Z = 0.170
SLIDE_LOWER_CENTER_X = 0.045

TOOL_BLOCK_SIZE = (0.040, 0.075, 0.055)
TOOL_BLOCK_CENTER = (0.050, 0.0, -0.400)
TOOL_FACE_CENTER = (0.070, 0.0, -0.400)
TOOL_FACE_SIZE = (0.008, 0.060, 0.075)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rear_frame_body() -> cq.Workplane:
    post_height = FRAME_H - (2.0 * BEAM_H)

    body = _box((FRAME_DEPTH, FRAME_W, BEAM_H), (FRAME_DEPTH / 2.0, 0.0, BEAM_H / 2.0))
    body = body.union(
        _box(
            (FRAME_DEPTH, FRAME_W, BEAM_H),
            (FRAME_DEPTH / 2.0, 0.0, FRAME_H - (BEAM_H / 2.0)),
        )
    )
    body = body.union(
        _box(
            (FRAME_DEPTH, POST_W, post_height),
            (FRAME_DEPTH / 2.0, -(FRAME_W / 2.0) + (POST_W / 2.0), FRAME_H / 2.0),
        )
    )
    body = body.union(
        _box(
            (FRAME_DEPTH, POST_W, post_height),
            (FRAME_DEPTH / 2.0, (FRAME_W / 2.0) - (POST_W / 2.0), FRAME_H / 2.0),
        )
    )
    body = body.union(_box((0.024, 0.56, 0.68), (0.012, 0.0, 0.78)))

    for y in (-0.30, 0.30):
        body = body.union(_box((0.060, FOOT_W, 0.080), (0.030, y, 0.040)))
        body = body.union(_box((FOOT_LEN, FOOT_W, FOOT_H), (FOOT_LEN / 2.0, y, FOOT_H / 2.0)))

    for y in (-0.335, 0.335):
        for z in (H_RAIL_BOT_Z, H_RAIL_TOP_Z):
            body = body.union(_box((0.054, 0.070, 0.065), (0.083, y, z)))

    return body


def _horizontal_carriage_body() -> cq.Workplane:
    body = _box(
        (CARRIAGE_BLOCK_X, CARRIAGE_BLOCK_Y, CARRIAGE_BLOCK_Z),
        (CARRIAGE_BLOCK_CENTER_X, 0.0, 0.060),
    )
    body = body.union(
        _box(
            (CARRIAGE_BLOCK_X, CARRIAGE_BLOCK_Y, CARRIAGE_BLOCK_Z),
            (CARRIAGE_BLOCK_CENTER_X, 0.0, -0.060),
        )
    )
    body = body.union(
        _box((CARRIAGE_BRIDGE_X, CARRIAGE_BRIDGE_Y, CARRIAGE_BRIDGE_Z), (CARRIAGE_BRIDGE_CENTER_X, 0.0, 0.0))
    )
    body = body.union(
        _box((CARRIAGE_NOSE_X, CARRIAGE_NOSE_Y, CARRIAGE_NOSE_Z), (CARRIAGE_NOSE_CENTER_X, 0.0, -0.030))
    )
    body = body.union(
        _box((CARRIAGE_SPINE_X, CARRIAGE_SPINE_Y, CARRIAGE_SPINE_Z), (CARRIAGE_SPINE_CENTER_X, 0.0, -0.105))
    )
    body = body.union(
        _box(
            (V_RAIL_SUPPORT_X, V_RAIL_SUPPORT_Y, V_RAIL_SUPPORT_Z),
            (V_RAIL_SUPPORT_CENTER_X, 0.0, V_RAIL_SUPPORT_CENTER_Z),
        )
    )
    body = body.union(_box((0.032, 0.100, 0.105), (0.082, 0.0, -0.185)))

    for z in (-0.060, 0.060):
        bore = (
            cq.Workplane("XZ")
            .center(0.0, z)
            .circle(H_RAIL_R)
            .extrude((CARRIAGE_BLOCK_Y / 2.0) + 0.02, both=True)
        )
        body = body.cut(bore)

    return body


def _vertical_slide_body() -> cq.Workplane:
    body = _box(
        (SLIDE_GUIDE_X, SLIDE_GUIDE_Y, SLIDE_GUIDE_Z),
        (SLIDE_GUIDE_CENTER_X, 0.0, -0.055),
    )
    body = body.union(
        _box(
            (SLIDE_GUIDE_X, SLIDE_GUIDE_Y, SLIDE_GUIDE_Z),
            (SLIDE_GUIDE_CENTER_X, 0.0, -0.185),
        )
    )
    body = body.union(_box((SLIDE_SPINE_X, SLIDE_SPINE_Y, SLIDE_SPINE_Z), (SLIDE_SPINE_CENTER_X, 0.0, -0.145)))
    body = body.union(_box((SLIDE_LOWER_X, SLIDE_LOWER_Y, SLIDE_LOWER_Z), (SLIDE_LOWER_CENTER_X, 0.0, -0.305)))
    body = body.union(_box(TOOL_BLOCK_SIZE, TOOL_BLOCK_CENTER))

    for y in (-V_RAIL_Y, V_RAIL_Y):
        body = body.cut(
            cq.Workplane("XY")
            .center(0.0, y)
            .circle(V_RAIL_R)
            .extrude(-V_RAIL_LEN, both=False)
        )

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backplane_yz_stage")

    model.material("frame_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("rail_steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("carriage_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("slide_gray", rgba=(0.54, 0.57, 0.61, 1.0))
    model.material("tool_orange", rgba=(0.84, 0.41, 0.14, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_body(), "rear_frame_body"),
        material="frame_graphite",
        name="frame_body",
    )
    rear_frame.visual(
        Cylinder(radius=H_RAIL_R, length=H_RAIL_LEN),
        origin=Origin(xyz=(H_RAIL_X, 0.0, H_RAIL_TOP_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="rail_steel",
        name="upper_rail",
    )
    rear_frame.visual(
        Cylinder(radius=H_RAIL_R, length=H_RAIL_LEN),
        origin=Origin(xyz=(H_RAIL_X, 0.0, H_RAIL_BOT_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="rail_steel",
        name="lower_rail",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.18, FRAME_W, FRAME_H)),
        mass=34.0,
        origin=Origin(xyz=(0.09, 0.0, FRAME_H / 2.0)),
    )

    horizontal_carriage = model.part("horizontal_carriage")
    horizontal_carriage.visual(
        mesh_from_cadquery(_horizontal_carriage_body(), "horizontal_carriage_body"),
        material="carriage_aluminum",
        name="carriage_body",
    )
    horizontal_carriage.visual(
        Cylinder(radius=V_RAIL_R, length=V_RAIL_LEN),
        origin=Origin(xyz=(SLIDE_JOINT_X, -V_RAIL_Y, V_RAIL_CENTER_Z)),
        material="rail_steel",
        name="left_slide_rail",
    )
    horizontal_carriage.visual(
        Cylinder(radius=V_RAIL_R, length=V_RAIL_LEN),
        origin=Origin(xyz=(SLIDE_JOINT_X, V_RAIL_Y, V_RAIL_CENTER_Z)),
        material="rail_steel",
        name="right_slide_rail",
    )
    horizontal_carriage.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.32)),
        mass=8.0,
        origin=Origin(xyz=(0.055, 0.0, -0.06)),
    )

    vertical_slide = model.part("vertical_slide")
    vertical_slide.visual(
        mesh_from_cadquery(_vertical_slide_body(), "vertical_slide_body"),
        material="slide_gray",
        name="slide_body",
    )
    vertical_slide.visual(
        Box(TOOL_FACE_SIZE),
        origin=Origin(xyz=TOOL_FACE_CENTER),
        material="tool_orange",
        name="tool_face",
    )
    vertical_slide.inertial = Inertial.from_geometry(
        Box((0.10, 0.12, 0.44)),
        mass=4.5,
        origin=Origin(xyz=(0.035, 0.0, -0.22)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=horizontal_carriage,
        origin=Origin(xyz=(H_RAIL_X, 0.0, H_RAIL_MID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-H_TRAVEL,
            upper=H_TRAVEL,
            effort=220.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=horizontal_carriage,
        child=vertical_slide,
        origin=Origin(xyz=(SLIDE_JOINT_X, 0.0, SLIDE_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=V_TRAVEL,
            effort=160.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    horizontal_carriage = object_model.get_part("horizontal_carriage")
    vertical_slide = object_model.get_part("vertical_slide")
    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_slide = object_model.get_articulation("carriage_to_slide")

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
        horizontal_carriage,
        rear_frame,
        contact_tol=0.002,
        name="carriage is supported by rear frame rails",
    )
    ctx.expect_contact(
        vertical_slide,
        horizontal_carriage,
        contact_tol=0.002,
        name="vertical slide is supported by carriage rails",
    )
    ctx.expect_gap(
        vertical_slide,
        rear_frame,
        axis="x",
        min_gap=0.002,
        positive_elem="tool_face",
        name="tool face hangs forward of the rear frame",
    )

    with ctx.pose({frame_to_carriage: 0.0, carriage_to_slide: 0.0}):
        carriage_home = ctx.part_world_position(horizontal_carriage)
        tool_home = _aabb_center(ctx.part_element_world_aabb(vertical_slide, elem="tool_face"))

    with ctx.pose({frame_to_carriage: H_TRAVEL, carriage_to_slide: 0.0}):
        carriage_shifted = ctx.part_world_position(horizontal_carriage)

    carriage_moves_positive_y = (
        carriage_home is not None
        and carriage_shifted is not None
        and carriage_shifted[1] > carriage_home[1] + 0.18
    )
    ctx.check(
        "horizontal carriage translates along +Y",
        carriage_moves_positive_y,
        details=f"home={carriage_home}, shifted={carriage_shifted}",
    )

    with ctx.pose({frame_to_carriage: 0.0, carriage_to_slide: V_TRAVEL}):
        tool_low = _aabb_center(ctx.part_element_world_aabb(vertical_slide, elem="tool_face"))

    slide_moves_down = (
        tool_home is not None
        and tool_low is not None
        and tool_low[2] < tool_home[2] - 0.15
    )
    ctx.check(
        "vertical slide extends downward on positive travel",
        slide_moves_down,
        details=f"home_tool={tool_home}, low_tool={tool_low}",
    )

    with ctx.pose({frame_to_carriage: H_TRAVEL, carriage_to_slide: V_TRAVEL}):
        ctx.expect_contact(
            horizontal_carriage,
            rear_frame,
            contact_tol=0.0025,
            name="carriage remains guided at horizontal limit",
        )
        ctx.expect_contact(
            vertical_slide,
            horizontal_carriage,
            contact_tol=0.0025,
            name="slide remains guided at full drop",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no part overlap at positive travel limits")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
