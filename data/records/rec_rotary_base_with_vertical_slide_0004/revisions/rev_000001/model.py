from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.110
BASE_FOOT_HEIGHT = 0.018
BASE_BODY_RADIUS = 0.090
BASE_BODY_HEIGHT = 0.034
BEARING_RING_OUTER_RADIUS = 0.060
BEARING_RING_INNER_RADIUS = 0.028
BEARING_RING_HEIGHT = 0.004
SPINDLE_RADIUS = 0.022
SPINDLE_HEIGHT = 0.028

TURNTABLE_RADIUS = 0.085
TURNTABLE_THICKNESS = 0.012
TURNTABLE_BORE_RADIUS = 0.024
TURNTABLE_HUB_RADIUS = 0.038
TURNTABLE_HUB_HEIGHT = 0.020

COLUMN_PAD_HEIGHT = 0.018
COLUMN_WIDTH = 0.070
BACKPLATE_THICKNESS = 0.010
RAIL_THICKNESS = 0.010
RAIL_DEPTH = 0.026
COLUMN_HEIGHT = 0.340
TOP_BRIDGE_THICKNESS = 0.012

MOUNT_PAD_CLEAR_RADIUS = 0.040
GUIDE_SHOE_THICKNESS = 0.008
GUIDE_SHOE_DEPTH = 0.026
CARRIAGE_HEIGHT = 0.070
CARRIAGE_FRONT_WIDTH = 0.080
CARRIAGE_FRONT_THICKNESS = 0.010
NECK_WIDTH = 0.040
NECK_DEPTH = 0.009
NECK_Z_OFFSET = 0.010
NECK_HEIGHT = 0.050

TOOL_PLATE_WIDTH = 0.075
TOOL_PLATE_THICKNESS = 0.008
TOOL_PLATE_HEIGHT = 0.050

ROTARY_LIMIT = math.radians(150.0)
SLIDE_TRAVEL = 0.180

BASE_TO_TURNTABLE_Z = BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT + BEARING_RING_HEIGHT
TURNTABLE_TO_COLUMN_Z = TURNTABLE_THICKNESS
COLUMN_TO_CARRIAGE_Z = 0.042
GUIDE_FRAME_FRONT_Y = BACKPLATE_THICKNESS + RAIL_DEPTH
CARRIAGE_FRONT_MAX_Y = GUIDE_FRAME_FRONT_Y + CARRIAGE_FRONT_THICKNESS


def _base_housing_shape() -> cq.Workplane:
    lower_drum = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_FOOT_HEIGHT)
    upper_drum = (
        cq.Workplane("XY")
        .circle(BASE_BODY_RADIUS)
        .extrude(BASE_BODY_HEIGHT)
        .translate((0.0, 0.0, BASE_FOOT_HEIGHT))
    )
    access_cut = (
        cq.Workplane("XY")
        .box(0.060, 0.090, 0.020, centered=(True, True, False))
        .translate((0.0, 0.030, BASE_FOOT_HEIGHT + 0.008))
    )
    return lower_drum.union(upper_drum).cut(access_cut)


def _bearing_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BEARING_RING_OUTER_RADIUS)
        .circle(BEARING_RING_INNER_RADIUS)
        .extrude(BEARING_RING_HEIGHT)
        .translate((0.0, 0.0, BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT))
    )


def _spindle_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(SPINDLE_RADIUS)
        .extrude(SPINDLE_HEIGHT)
        .translate((0.0, 0.0, BASE_FOOT_HEIGHT + BASE_BODY_HEIGHT))
    )


def _turntable_platter_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(TURNTABLE_RADIUS)
        .circle(TURNTABLE_BORE_RADIUS)
        .extrude(TURNTABLE_THICKNESS)
    )


def _turntable_hub_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(TURNTABLE_HUB_RADIUS)
        .circle(TURNTABLE_BORE_RADIUS)
        .extrude(TURNTABLE_HUB_HEIGHT)
        .translate((0.0, 0.0, TURNTABLE_THICKNESS))
    )


def _column_mount_pad_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        COLUMN_WIDTH + 0.020,
        BACKPLATE_THICKNESS + RAIL_DEPTH,
        COLUMN_PAD_HEIGHT,
        centered=(True, False, False),
    )
    clearance = (
        cq.Workplane("XY")
        .circle(MOUNT_PAD_CLEAR_RADIUS)
        .extrude(COLUMN_PAD_HEIGHT + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return pad.cut(clearance)


def _column_backplate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            COLUMN_WIDTH,
            BACKPLATE_THICKNESS,
            COLUMN_HEIGHT,
            centered=(True, False, False),
        )
        .translate((0.0, 0.0, COLUMN_PAD_HEIGHT))
    )


def _column_left_rail_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            RAIL_THICKNESS,
            RAIL_DEPTH,
            COLUMN_HEIGHT,
            centered=(True, False, False),
        )
        .translate(
            (
                COLUMN_WIDTH / 2.0 - RAIL_THICKNESS / 2.0,
                BACKPLATE_THICKNESS,
                COLUMN_PAD_HEIGHT,
            )
        )
    )


def _column_right_rail_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            RAIL_THICKNESS,
            RAIL_DEPTH,
            COLUMN_HEIGHT,
            centered=(True, False, False),
        )
        .translate(
            (
                -COLUMN_WIDTH / 2.0 + RAIL_THICKNESS / 2.0,
                BACKPLATE_THICKNESS,
                COLUMN_PAD_HEIGHT,
            )
        )
    )


def _column_top_bridge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            COLUMN_WIDTH,
            RAIL_DEPTH,
            TOP_BRIDGE_THICKNESS,
            centered=(True, False, False),
        )
        .translate(
            (
                0.0,
                BACKPLATE_THICKNESS,
                COLUMN_PAD_HEIGHT + COLUMN_HEIGHT - TOP_BRIDGE_THICKNESS,
            )
        )
    )


def _carriage_left_shoe_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            GUIDE_SHOE_THICKNESS,
            GUIDE_SHOE_DEPTH,
            CARRIAGE_HEIGHT,
            centered=(True, False, False),
        )
        .translate(
            (
                COLUMN_WIDTH / 2.0 + GUIDE_SHOE_THICKNESS / 2.0,
                BACKPLATE_THICKNESS,
                0.0,
            )
        )
    )


def _carriage_right_shoe_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            GUIDE_SHOE_THICKNESS,
            GUIDE_SHOE_DEPTH,
            CARRIAGE_HEIGHT,
            centered=(True, False, False),
        )
        .translate(
            (
                -COLUMN_WIDTH / 2.0 - GUIDE_SHOE_THICKNESS / 2.0,
                BACKPLATE_THICKNESS,
                0.0,
            )
        )
    )


def _carriage_neck_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            NECK_WIDTH,
            NECK_DEPTH,
            NECK_HEIGHT,
            centered=(True, False, False),
        )
        .translate(
            (
                0.0,
                GUIDE_FRAME_FRONT_Y,
                NECK_Z_OFFSET,
            )
        )
    )


def _carriage_front_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            CARRIAGE_FRONT_WIDTH,
            CARRIAGE_FRONT_THICKNESS,
            CARRIAGE_HEIGHT,
            centered=(True, False, False),
        )
        .translate((0.0, GUIDE_FRAME_FRONT_Y, 0.0))
    )


def _tool_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XZ").rect(TOOL_PLATE_WIDTH, TOOL_PLATE_HEIGHT).extrude(
        TOOL_PLATE_THICKNESS
    )
    return (
        plate.faces(">Y")
        .workplane()
        .pushPoints([(-0.020, 0.0), (0.020, 0.0)])
        .hole(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_vertical_slide", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.27, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    painted_gray = model.material("painted_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.18, 0.39, 0.73, 1.0))
    anodized_aluminum = model.material(
        "anodized_aluminum",
        rgba=(0.82, 0.84, 0.86, 1.0),
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "base_housing.obj", assets=ASSETS),
        name="housing",
        material=cast_iron,
    )
    base.visual(
        mesh_from_cadquery(_bearing_ring_shape(), "base_bearing_ring.obj", assets=ASSETS),
        name="bearing_ring",
        material=machined_steel,
    )
    base.visual(
        mesh_from_cadquery(_spindle_shape(), "base_spindle.obj", assets=ASSETS),
        name="spindle",
        material=machined_steel,
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_TO_TURNTABLE_Z),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_TURNTABLE_Z / 2.0)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(
            _turntable_platter_shape(),
            "turntable_platter.obj",
            assets=ASSETS,
        ),
        name="platter",
        material=painted_gray,
    )
    turntable.visual(
        mesh_from_cadquery(_turntable_hub_shape(), "turntable_hub.obj", assets=ASSETS),
        name="hub",
        material=machined_steel,
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_THICKNESS + TURNTABLE_HUB_HEIGHT),
        mass=4.5,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (TURNTABLE_THICKNESS + TURNTABLE_HUB_HEIGHT) / 2.0,
            )
        ),
    )

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(
            _column_mount_pad_shape(),
            "column_mount_pad.obj",
            assets=ASSETS,
        ),
        name="mount_pad",
        material=machined_steel,
    )
    column.visual(
        mesh_from_cadquery(
            _column_backplate_shape(),
            "column_backplate.obj",
            assets=ASSETS,
        ),
        name="backplate",
        material=safety_blue,
    )
    column.visual(
        mesh_from_cadquery(
            _column_left_rail_shape(),
            "column_left_rail.obj",
            assets=ASSETS,
        ),
        name="left_rail",
        material=safety_blue,
    )
    column.visual(
        mesh_from_cadquery(
            _column_right_rail_shape(),
            "column_right_rail.obj",
            assets=ASSETS,
        ),
        name="right_rail",
        material=safety_blue,
    )
    column.visual(
        mesh_from_cadquery(
            _column_top_bridge_shape(),
            "column_top_bridge.obj",
            assets=ASSETS,
        ),
        name="top_bridge",
        material=safety_blue,
    )
    column.inertial = Inertial.from_geometry(
        Box((COLUMN_WIDTH + 0.020, BACKPLATE_THICKNESS + RAIL_DEPTH, COLUMN_PAD_HEIGHT + COLUMN_HEIGHT)),
        mass=6.0,
        origin=Origin(
            xyz=(
                0.0,
                (BACKPLATE_THICKNESS + RAIL_DEPTH) / 2.0,
                (COLUMN_PAD_HEIGHT + COLUMN_HEIGHT) / 2.0,
            )
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(
            _carriage_left_shoe_shape(),
            "carriage_left_shoe.obj",
            assets=ASSETS,
        ),
        name="left_shoe",
        material=machined_steel,
    )
    carriage.visual(
        mesh_from_cadquery(
            _carriage_right_shoe_shape(),
            "carriage_right_shoe.obj",
            assets=ASSETS,
        ),
        name="right_shoe",
        material=machined_steel,
    )
    carriage.visual(
        mesh_from_cadquery(_carriage_neck_shape(), "carriage_neck.obj", assets=ASSETS),
        name="neck",
        material=machined_steel,
    )
    carriage.visual(
        mesh_from_cadquery(
            _carriage_front_plate_shape(),
            "carriage_front_plate.obj",
            assets=ASSETS,
        ),
        name="front_plate",
        material=painted_gray,
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_FRONT_WIDTH, CARRIAGE_FRONT_MAX_Y, CARRIAGE_HEIGHT)),
        mass=2.0,
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_FRONT_MAX_Y / 2.0,
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        mesh_from_cadquery(_tool_plate_shape(), "tool_plate.obj", assets=ASSETS),
        name="tool_plate",
        material=anodized_aluminum,
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((TOOL_PLATE_WIDTH, TOOL_PLATE_THICKNESS, TOOL_PLATE_HEIGHT)),
        mass=0.35,
        origin=Origin(xyz=(0.0, TOOL_PLATE_THICKNESS / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_TURNTABLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=-ROTARY_LIMIT,
            upper=ROTARY_LIMIT,
        ),
    )
    model.articulation(
        "turntable_to_column",
        ArticulationType.FIXED,
        parent=turntable,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_TO_COLUMN_Z)),
    )
    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TO_CARRIAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_tool_plate",
        ArticulationType.FIXED,
        parent=carriage,
        child=tool_plate,
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_FRONT_MAX_Y + TOOL_PLATE_THICKNESS,
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    tool_plate = object_model.get_part("tool_plate")

    rotary_joint = object_model.get_articulation("base_to_turntable")
    slide_joint = object_model.get_articulation("column_to_carriage")

    bearing_ring = base.get_visual("bearing_ring")
    platter = turntable.get_visual("platter")
    mount_pad = column.get_visual("mount_pad")
    left_rail = column.get_visual("left_rail")
    right_rail = column.get_visual("right_rail")
    backplate = column.get_visual("backplate")
    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")
    front_plate = carriage.get_visual("front_plate")
    tool_plate_visual = tool_plate.get_visual("tool_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.expect_contact(turntable, base, elem_a=platter, elem_b=bearing_ring)
    ctx.expect_origin_distance(turntable, base, axes="xy", max_dist=0.001)
    ctx.expect_contact(column, turntable, elem_a=mount_pad, elem_b=platter)
    ctx.expect_origin_distance(column, turntable, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        carriage,
        column,
        axis="y",
        positive_elem=left_shoe,
        negative_elem=backplate,
        min_gap=0.0,
        max_gap=0.0005,
    )
    ctx.expect_contact(carriage, column, elem_a=left_shoe, elem_b=left_rail)
    ctx.expect_contact(carriage, column, elem_a=right_shoe, elem_b=right_rail)
    ctx.expect_overlap(
        carriage,
        column,
        axes="x",
        min_overlap=GUIDE_SHOE_THICKNESS - 0.001,
    )
    ctx.expect_contact(tool_plate, carriage, elem_a=tool_plate_visual, elem_b=front_plate)

    def aabb_extents(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return tuple(upper - lower for lower, upper in zip(aabb[0], aabb[1]))

    with ctx.pose({rotary_joint: 0.0, slide_joint: 0.0}):
        tool_rest_extents = aabb_extents(tool_plate)
        carriage_rest_pos = ctx.part_world_position(carriage)
        tool_rest_pos = ctx.part_world_position(tool_plate)

    with ctx.pose({rotary_joint: math.pi / 2.0, slide_joint: 0.0}):
        tool_quarter_extents = aabb_extents(tool_plate)

    with ctx.pose({rotary_joint: 0.0, slide_joint: SLIDE_TRAVEL}):
        carriage_upper_pos = ctx.part_world_position(carriage)
        tool_upper_pos = ctx.part_world_position(tool_plate)

    rotation_ok = (
        tool_rest_extents is not None
        and tool_quarter_extents is not None
        and tool_rest_extents[0] > tool_rest_extents[1] * 4.0
        and tool_quarter_extents[1] > tool_quarter_extents[0] * 4.0
    )
    ctx.check(
        "tool_plate_aabb_swaps_xy_when_rotated",
        rotation_ok,
        details=(
            f"rest_extents={tool_rest_extents}, quarter_turn_extents={tool_quarter_extents}"
        ),
    )

    slide_ok = (
        carriage_rest_pos is not None
        and carriage_upper_pos is not None
        and abs((carriage_upper_pos[2] - carriage_rest_pos[2]) - SLIDE_TRAVEL) <= 0.002
        and abs(carriage_upper_pos[0] - carriage_rest_pos[0]) <= 0.001
        and abs(carriage_upper_pos[1] - carriage_rest_pos[1]) <= 0.001
        and tool_rest_pos is not None
        and tool_upper_pos is not None
        and abs((tool_upper_pos[2] - tool_rest_pos[2]) - SLIDE_TRAVEL) <= 0.002
    )
    ctx.check(
        "carriage_translates_vertically_over_full_travel",
        slide_ok,
        details=(
            f"carriage_rest={carriage_rest_pos}, carriage_upper={carriage_upper_pos}, "
            f"tool_rest={tool_rest_pos}, tool_upper={tool_upper_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
