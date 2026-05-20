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


BASE_LENGTH = 0.170
BASE_WIDTH = 0.110
BASE_BODY_HEIGHT = 0.022
BASE_HOUSING_RADIUS = 0.058
BASE_HOUSING_HEIGHT = 0.028
BASE_TOTAL_HEIGHT = BASE_BODY_HEIGHT + BASE_HOUSING_HEIGHT

TABLE_RADIUS = 0.060
TABLE_THICKNESS = 0.014

GUIDE_Y = 0.012
GUIDE_BLOCK_WIDTH = 0.090
GUIDE_BLOCK_DEPTH = 0.030
GUIDE_BLOCK_HEIGHT = 0.018

SPINE_WIDTH = 0.018
SPINE_DEPTH = 0.014
SPINE_Y = -0.022
SPINE_HEIGHT = 0.212

TOP_BRIDGE_WIDTH = 0.082
TOP_BRIDGE_DEPTH = 0.038
TOP_BRIDGE_HEIGHT = 0.014
TOP_BRIDGE_Z = 0.232
TOP_BRIDGE_Y = -0.007

ROD_RADIUS = 0.006
ROD_CLEAR_RADIUS = 0.0068
ROD_X = 0.024
ROD_BASE_Z = TABLE_THICKNESS + GUIDE_BLOCK_HEIGHT - 0.001
ROD_LENGTH = 0.205

GUIDE_RAIL_WIDTH = 0.022
GUIDE_RAIL_DEPTH = 0.012
GUIDE_RAIL_Y = -0.018
GUIDE_RAIL_Z = 0.040
GUIDE_RAIL_HEIGHT = 0.192

HEAD_LOW_Z = 0.048
HEAD_TRAVEL = 0.125
HEAD_FRONT_WIDTH = 0.074
HEAD_FRONT_DEPTH = 0.026
HEAD_FRONT_Y = 0.021
HEAD_FRONT_HEIGHT = 0.042

HEAD_CROSSBAR_WIDTH = 0.056
HEAD_CROSSBAR_DEPTH = 0.012
HEAD_CROSSBAR_Y = 0.006
HEAD_CROSSBAR_HEIGHT = 0.012

HEAD_SIDE_RIB_WIDTH = 0.014
HEAD_SIDE_RIB_DEPTH = 0.028
HEAD_SIDE_RIB_Y = -0.010
HEAD_SIDE_RIB_HEIGHT = 0.036
HEAD_SIDE_RIB_X = 0.018

HEAD_REAR_PAD_WIDTH = 0.018
HEAD_REAR_PAD_DEPTH = 0.008
HEAD_REAR_PAD_Y = -0.020
HEAD_REAR_PAD_HEIGHT = 0.034

HEAD_CENTER_STEM_WIDTH = 0.010
HEAD_CENTER_STEM_DEPTH = 0.024
HEAD_CENTER_STEM_Y = -0.008
HEAD_CENTER_STEM_HEIGHT = 0.016

SLEEVE_WIDTH = 0.024
SLEEVE_DEPTH = 0.018
SLEEVE_HEIGHT = 0.055

HEAD_NOSE_RADIUS = 0.013
HEAD_NOSE_LENGTH = 0.018
HEAD_NOSE_Y = HEAD_FRONT_Y + (HEAD_FRONT_DEPTH / 2.0)
HEAD_NOSE_Z = 0.028

HEAD_WINDOW_WIDTH = 0.026
HEAD_WINDOW_THICKNESS = 0.002
HEAD_WINDOW_HEIGHT = 0.014
HEAD_WINDOW_Y = HEAD_FRONT_Y + (HEAD_FRONT_DEPTH / 2.0) - (HEAD_WINDOW_THICKNESS / 2.0)
HEAD_WINDOW_Z = 0.038


def _box_prism(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(length, width).extrude(height)


def _build_base_shape() -> cq.Workplane:
    base_body = (
        _box_prism(BASE_LENGTH, BASE_WIDTH, BASE_BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.018)
    )
    housing = (
        cq.Workplane("XY")
        .circle(BASE_HOUSING_RADIUS)
        .extrude(BASE_HOUSING_HEIGHT)
        .translate((0.0, 0.0, BASE_BODY_HEIGHT))
    )
    bolt_recesses = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.050, -0.028),
                (-0.050, 0.028),
                (0.050, -0.028),
                (0.050, 0.028),
            ]
        )
        .circle(0.0065)
        .extrude(0.007)
        .translate((0.0, 0.0, BASE_BODY_HEIGHT - 0.007))
    )
    return base_body.union(housing).cut(bolt_recesses)


def _build_rotary_frame_shape() -> cq.Workplane:
    guide_block = _box_prism(GUIDE_BLOCK_WIDTH, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT).translate(
        (0.0, GUIDE_Y, TABLE_THICKNESS - 0.001)
    )
    rear_web = _box_prism(0.030, 0.040, 0.032).translate((0.0, -0.006, TABLE_THICKNESS - 0.001))
    rear_spine = _box_prism(SPINE_WIDTH, SPINE_DEPTH, SPINE_HEIGHT).translate(
        (0.0, SPINE_Y, TABLE_THICKNESS + GUIDE_BLOCK_HEIGHT - 0.002)
    )
    top_bridge = _box_prism(TOP_BRIDGE_WIDTH, TOP_BRIDGE_DEPTH, TOP_BRIDGE_HEIGHT).translate(
        (0.0, TOP_BRIDGE_Y, TOP_BRIDGE_Z)
    )
    return guide_block.union(rear_web).union(rear_spine).union(top_bridge)


def _build_head_shape() -> cq.Workplane:
    front_body = _box_prism(HEAD_FRONT_WIDTH, HEAD_FRONT_DEPTH, HEAD_FRONT_HEIGHT).translate(
        (0.0, HEAD_FRONT_Y, 0.005)
    )
    crossbar = _box_prism(HEAD_CROSSBAR_WIDTH, HEAD_CROSSBAR_DEPTH, HEAD_CROSSBAR_HEIGHT).translate(
        (0.0, HEAD_CROSSBAR_Y, 0.012)
    )
    left_sleeve = _box_prism(SLEEVE_WIDTH, SLEEVE_DEPTH, SLEEVE_HEIGHT).translate(
        (-ROD_X, 0.0, 0.0)
    )
    right_sleeve = _box_prism(SLEEVE_WIDTH, SLEEVE_DEPTH, SLEEVE_HEIGHT).translate(
        (ROD_X, 0.0, 0.0)
    )
    left_rib = _box_prism(HEAD_SIDE_RIB_WIDTH, HEAD_SIDE_RIB_DEPTH, HEAD_SIDE_RIB_HEIGHT).translate(
        (-HEAD_SIDE_RIB_X, HEAD_SIDE_RIB_Y, 0.006)
    )
    right_rib = _box_prism(HEAD_SIDE_RIB_WIDTH, HEAD_SIDE_RIB_DEPTH, HEAD_SIDE_RIB_HEIGHT).translate(
        (HEAD_SIDE_RIB_X, HEAD_SIDE_RIB_Y, 0.006)
    )
    rear_pad = _box_prism(HEAD_REAR_PAD_WIDTH, HEAD_REAR_PAD_DEPTH, HEAD_REAR_PAD_HEIGHT).translate(
        (0.0, HEAD_REAR_PAD_Y, 0.008)
    )
    center_stem = _box_prism(
        HEAD_CENTER_STEM_WIDTH,
        HEAD_CENTER_STEM_DEPTH,
        HEAD_CENTER_STEM_HEIGHT,
    ).translate((0.0, HEAD_CENTER_STEM_Y, 0.016))
    nose = (
        cq.Workplane("XZ")
        .circle(HEAD_NOSE_RADIUS)
        .extrude(HEAD_NOSE_LENGTH)
        .translate((0.0, HEAD_NOSE_Y, HEAD_NOSE_Z))
    )
    body = (
        front_body.union(crossbar)
        .union(left_sleeve)
        .union(right_sleeve)
        .union(left_rib)
        .union(right_rib)
        .union(rear_pad)
        .union(center_stem)
        .union(nose)
    )
    rod_holes = (
        cq.Workplane("XY")
        .pushPoints([(-ROD_X, 0.0), (ROD_X, 0.0)])
        .circle(ROD_CLEAR_RADIUS)
        .extrude(SLEEVE_HEIGHT + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    cable_relief = (
        cq.Workplane("YZ")
        .rect(0.024, 0.020)
        .extrude(0.032)
        .translate((-0.016, -0.018, 0.012))
    )
    return body.cut(rod_holes).cut(cable_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_base_z_slide_module")

    model.material("powder_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("guide_steel", rgba=(0.81, 0.83, 0.86, 1.0))
    model.material("dark_composite", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("smoked_glass", rgba=(0.18, 0.25, 0.30, 0.80))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "pan_base_body"),
        material="powder_black",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_TOTAL_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT / 2.0)),
    )

    rotary_table = model.part("rotary_table")
    rotary_table.visual(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, TABLE_THICKNESS / 2.0)),
        material="machined_aluminum",
        name="turntable_plate",
    )
    rotary_table.visual(
        mesh_from_cadquery(_build_rotary_frame_shape(), "rotary_guide_frame"),
        material="powder_black",
        name="guide_frame",
    )
    rotary_table.visual(
        Box((GUIDE_RAIL_WIDTH, GUIDE_RAIL_DEPTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, GUIDE_RAIL_Y, GUIDE_RAIL_Z + (GUIDE_RAIL_HEIGHT / 2.0)),
        ),
        material="machined_aluminum",
        name="center_guide_rail",
    )
    rotary_table.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(xyz=(-ROD_X, GUIDE_Y, ROD_BASE_Z + (ROD_LENGTH / 2.0))),
        material="guide_steel",
        name="left_guide_rod",
    )
    rotary_table.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(xyz=(ROD_X, GUIDE_Y, ROD_BASE_Z + (ROD_LENGTH / 2.0))),
        material="guide_steel",
        name="right_guide_rod",
    )
    rotary_table.inertial = Inertial.from_geometry(
        Box((TOP_BRIDGE_WIDTH, 0.060, TOP_BRIDGE_Z + TOP_BRIDGE_HEIGHT)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (TOP_BRIDGE_Z + TOP_BRIDGE_HEIGHT) / 2.0)),
    )

    moving_head = model.part("moving_head")
    moving_head.visual(
        mesh_from_cadquery(_build_head_shape(), "moving_head_carriage"),
        material="dark_composite",
        name="head_body",
    )
    moving_head.visual(
        Box((HEAD_WINDOW_WIDTH, HEAD_WINDOW_THICKNESS, HEAD_WINDOW_HEIGHT)),
        origin=Origin(
            xyz=(0.0, HEAD_WINDOW_Y, HEAD_WINDOW_Z),
        ),
        material="smoked_glass",
        name="front_window",
    )
    moving_head.inertial = Inertial.from_geometry(
        Box((HEAD_FRONT_WIDTH, 0.052, SLEEVE_HEIGHT)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.012, SLEEVE_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_table,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=12.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "table_z_slide",
        ArticulationType.PRISMATIC,
        parent=rotary_table,
        child=moving_head,
        origin=Origin(xyz=(0.0, GUIDE_Y, HEAD_LOW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HEAD_TRAVEL,
            effort=90.0,
            velocity=0.24,
        ),
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

    base = object_model.get_part("base")
    rotary_table = object_model.get_part("rotary_table")
    moving_head = object_model.get_part("moving_head")
    base_pan = object_model.get_articulation("base_pan")
    table_z_slide = object_model.get_articulation("table_z_slide")

    ctx.expect_contact(
        rotary_table,
        base,
        contact_tol=1e-5,
        name="turntable seats on the base housing",
    )
    ctx.expect_overlap(
        rotary_table,
        base,
        axes="xy",
        min_overlap=0.090,
        name="rotary table stays centered over the lower base",
    )
    ctx.expect_contact(
        moving_head,
        rotary_table,
        elem_a="head_body",
        elem_b="center_guide_rail",
        contact_tol=1e-5,
        name="head carriage bears against the center guide rail",
    )
    ctx.expect_within(
        moving_head,
        rotary_table,
        axes="x",
        margin=0.002,
        name="moving head stays laterally within the guide frame envelope",
    )

    rest_head_pos = None
    raised_head_pos = None
    pan_reference = None
    pan_rotated = None

    with ctx.pose({base_pan: 0.0, table_z_slide: 0.0}):
        rest_head_pos = ctx.part_world_position(moving_head)

    with ctx.pose({base_pan: 0.0, table_z_slide: HEAD_TRAVEL}):
        ctx.expect_within(
            moving_head,
            rotary_table,
            axes="x",
            margin=0.002,
            name="extended head remains laterally captured by the guide frame",
        )
        raised_head_pos = ctx.part_world_position(moving_head)

    ctx.check(
        "z slide raises the head vertically",
        (
            rest_head_pos is not None
            and raised_head_pos is not None
            and raised_head_pos[2] > rest_head_pos[2] + 0.10
            and abs(raised_head_pos[0] - rest_head_pos[0]) < 1e-6
            and abs(raised_head_pos[1] - rest_head_pos[1]) < 1e-6
        ),
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    with ctx.pose({base_pan: 0.0, table_z_slide: 0.040}):
        pan_reference = ctx.part_world_position(moving_head)
    with ctx.pose({base_pan: 1.0, table_z_slide: 0.040}):
        pan_rotated = ctx.part_world_position(moving_head)

    ctx.check(
        "pan joint swings the guide module about the vertical axis",
        (
            pan_reference is not None
            and pan_rotated is not None
            and abs(pan_rotated[0] - pan_reference[0]) > 0.008
            and abs(pan_rotated[2] - pan_reference[2]) < 1e-6
        ),
        details=f"reference={pan_reference}, rotated={pan_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
