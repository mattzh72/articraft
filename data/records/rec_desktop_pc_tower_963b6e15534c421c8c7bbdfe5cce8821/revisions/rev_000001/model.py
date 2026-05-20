from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CASE_WIDTH = 0.43
CASE_DEPTH = 0.34
CASE_HEIGHT = 0.078
BOTTOM_THICKNESS = 0.003
SIDE_WALL_THICKNESS = 0.003
REAR_WALL_THICKNESS = 0.010
FRONT_FRAME_THICKNESS = 0.010
FRAME_STILE_WIDTH = 0.018
FRAME_RAIL_HEIGHT = 0.011
HINGE_RADIUS = 0.004
HINGE_AXIS_Z = 0.070
TOP_HINGE_AXIS_Y = CASE_DEPTH / 2.0 - 0.004
LID_THICKNESS = 0.003
LID_WIDTH = CASE_WIDTH - 0.010
LID_DEPTH = CASE_DEPTH - 0.024
LID_PANEL_DEPTH = LID_DEPTH - 0.012
LID_FLANGE_THICKNESS = 0.0025
LID_FLANGE_DROP = 0.015
DOOR_THICKNESS = 0.005
DOOR_WIDTH = CASE_WIDTH - 2.0 * FRAME_STILE_WIDTH
DOOR_HEIGHT = CASE_HEIGHT - FRAME_RAIL_HEIGHT - 0.009
DOOR_CENTER_Z = 0.009 + DOOR_HEIGHT / 2.0
DOOR_HINGE_X = -DOOR_WIDTH / 2.0
TOP_PANEL_OPEN = 1.25
FRONT_DOOR_OPEN = 1.55


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="htpc_case")

    chassis = model.material("chassis_black", rgba=(0.12, 0.12, 0.13, 1.0))
    lid_finish = model.material("lid_satin", rgba=(0.18, 0.18, 0.19, 1.0))
    bezel_finish = model.material("bezel_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    metal = model.material("hinge_metal", rgba=(0.56, 0.57, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((CASE_WIDTH, CASE_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS / 2.0)),
        material=chassis,
        name="bottom_pan",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, CASE_DEPTH, HINGE_AXIS_Z)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0),
                0.0,
                HINGE_AXIS_Z / 2.0,
            )
        ),
        material=chassis,
        name="left_wall",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, CASE_DEPTH, HINGE_AXIS_Z)),
        origin=Origin(
            xyz=(
                CASE_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0,
                0.0,
                HINGE_AXIS_Z / 2.0,
            )
        ),
        material=chassis,
        name="right_wall",
    )
    body.visual(
        Box((CASE_WIDTH, REAR_WALL_THICKNESS, HINGE_AXIS_Z)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_DEPTH / 2.0 - REAR_WALL_THICKNESS / 2.0,
                HINGE_AXIS_Z / 2.0,
            )
        ),
        material=chassis,
        name="rear_wall",
    )
    body.visual(
        Box((FRAME_STILE_WIDTH, FRONT_FRAME_THICKNESS, HINGE_AXIS_Z)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH / 2.0 - FRAME_STILE_WIDTH / 2.0),
                -(CASE_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0),
                HINGE_AXIS_Z / 2.0,
            )
        ),
        material=chassis,
        name="front_left_stile",
    )
    body.visual(
        Box((FRAME_STILE_WIDTH, FRONT_FRAME_THICKNESS, HINGE_AXIS_Z)),
        origin=Origin(
            xyz=(
                CASE_WIDTH / 2.0 - FRAME_STILE_WIDTH / 2.0,
                -(CASE_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0),
                HINGE_AXIS_Z / 2.0,
            )
        ),
        material=chassis,
        name="front_right_stile",
    )
    body.visual(
        Box((CASE_WIDTH - 2.0 * FRAME_STILE_WIDTH, FRONT_FRAME_THICKNESS, FRAME_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0),
                CASE_HEIGHT - FRAME_RAIL_HEIGHT / 2.0,
            )
        ),
        material=chassis,
        name="front_header",
    )
    body.visual(
        Box((CASE_WIDTH - 2.0 * FRAME_STILE_WIDTH, FRONT_FRAME_THICKNESS, 0.009)),
        origin=Origin(
            xyz=(
                0.0,
                -(CASE_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0),
                0.009 / 2.0,
            )
        ),
        material=chassis,
        name="front_sill",
    )
    body.visual(
        Box((CASE_WIDTH - 0.080, 0.028, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_DEPTH / 2.0 - 0.014,
                HINGE_AXIS_Z - 0.013,
            )
        ),
        material=chassis,
        name="rear_top_bridge",
    )

    for index, hinge_x in enumerate((-0.165, 0.165), start=1):
        body.visual(
            Box((0.034, 0.012, 0.004)),
            origin=Origin(xyz=(hinge_x, TOP_HINGE_AXIS_Y - 0.003, HINGE_AXIS_Z - 0.010)),
            material=chassis,
            name=f"top_hinge_bracket_{index}",
        )
        body.visual(
            Box((0.004, 0.010, 0.008)),
            origin=Origin(
                xyz=(
                    hinge_x - 0.009,
                    TOP_HINGE_AXIS_Y - 0.002,
                    HINGE_AXIS_Z - 0.004,
                )
            ),
            material=chassis,
            name=f"top_hinge_ear_left_{index}",
        )
        body.visual(
            Box((0.004, 0.010, 0.008)),
            origin=Origin(
                xyz=(
                    hinge_x + 0.009,
                    TOP_HINGE_AXIS_Y - 0.002,
                    HINGE_AXIS_Z - 0.004,
                )
            ),
            material=chassis,
            name=f"top_hinge_ear_right_{index}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.008),
            origin=Origin(
                xyz=(hinge_x - 0.009, TOP_HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=metal,
            name=f"top_hinge_outer_left_{index}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.008),
            origin=Origin(
                xyz=(hinge_x + 0.009, TOP_HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=metal,
            name=f"top_hinge_outer_right_{index}",
        )

    body.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(
            xyz=(DOOR_HINGE_X, -CASE_DEPTH / 2.0, DOOR_CENTER_Z),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=metal,
        name="door_hinge_body_knuckle",
    )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((LID_WIDTH, LID_PANEL_DEPTH, LID_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(LID_PANEL_DEPTH / 2.0 + 0.006),
                HINGE_RADIUS + LID_THICKNESS / 2.0 + 0.0005,
            )
        ),
        material=lid_finish,
        name="lid_panel",
    )
    top_panel.visual(
        Box((0.280, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.0005)),
        material=lid_finish,
        name="lid_rear_hem_center",
    )
    top_panel.visual(
        Box((0.030, 0.006, 0.008)),
        origin=Origin(xyz=(-0.192, -0.010, 0.0005)),
        material=lid_finish,
        name="lid_rear_hem_left",
    )
    top_panel.visual(
        Box((0.030, 0.006, 0.008)),
        origin=Origin(xyz=(0.192, -0.010, 0.0005)),
        material=lid_finish,
        name="lid_rear_hem_right",
    )
    top_panel.visual(
        Box((LID_FLANGE_THICKNESS, LID_DEPTH - 0.018, LID_FLANGE_DROP)),
        origin=Origin(
            xyz=(
                -(LID_WIDTH / 2.0 - LID_FLANGE_THICKNESS / 2.0),
                -(LID_DEPTH - 0.018) / 2.0 - 0.010,
                -LID_FLANGE_DROP / 2.0 + 0.005,
            )
        ),
        material=lid_finish,
        name="lid_left_flange",
    )
    top_panel.visual(
        Box((LID_FLANGE_THICKNESS, LID_DEPTH - 0.018, LID_FLANGE_DROP)),
        origin=Origin(
            xyz=(
                LID_WIDTH / 2.0 - LID_FLANGE_THICKNESS / 2.0,
                -(LID_DEPTH - 0.018) / 2.0 - 0.010,
                -LID_FLANGE_DROP / 2.0 + 0.005,
            )
        ),
        material=lid_finish,
        name="lid_right_flange",
    )
    top_panel.visual(
        Box((LID_WIDTH, 0.008, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                -LID_DEPTH + 0.004,
                -0.0015,
            )
        ),
        material=lid_finish,
        name="lid_front_return",
    )
    for index, hinge_x in enumerate((-0.165, 0.165), start=1):
        top_panel.visual(
            Box((0.010, 0.008, 0.006)),
            origin=Origin(xyz=(hinge_x, -0.008, 0.0015)),
            material=lid_finish,
            name=f"top_hinge_leaf_{index}",
        )
        top_panel.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.010),
            origin=Origin(
                xyz=(hinge_x, 0.0, 0.0),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=metal,
            name=f"top_hinge_center_knuckle_{index}",
        )

    bezel_door = model.part("bezel_door")
    bezel_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -DOOR_THICKNESS / 2.0, 0.0)),
        material=bezel_finish,
        name="door_panel",
    )
    bezel_door.visual(
        Box((0.014, 0.006, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.007, -0.006, 0.0)),
        material=bezel_finish,
        name="door_hinge_leaf",
    )
    bezel_door.visual(
        Box((0.012, 0.004, 0.025)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.014, -0.007, 0.0)),
        material=metal,
        name="door_pull",
    )
    bezel_door.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=metal,
        name="door_hinge_knuckle_lower",
    )
    bezel_door.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=metal,
        name="door_hinge_knuckle_upper",
    )

    model.articulation(
        "body_to_top_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_panel,
        origin=Origin(xyz=(0.0, TOP_HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=TOP_PANEL_OPEN,
        ),
    )
    model.articulation(
        "body_to_bezel_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bezel_door,
        origin=Origin(xyz=(DOOR_HINGE_X, -CASE_DEPTH / 2.0, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=FRONT_DOOR_OPEN,
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
    body = object_model.get_part("body")
    top_panel = object_model.get_part("top_panel")
    bezel_door = object_model.get_part("bezel_door")
    top_hinge = object_model.get_articulation("body_to_top_panel")
    door_hinge = object_model.get_articulation("body_to_bezel_door")

    with ctx.pose({top_hinge: 0.0, door_hinge: 0.0}):
        ctx.expect_overlap(
            top_panel,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.28,
            name="top panel covers the chassis opening",
        )
        ctx.expect_overlap(
            bezel_door,
            body,
            axes="xz",
            elem_a="door_panel",
            min_overlap=0.04,
            name="front bezel door covers the front opening",
        )

    lid_closed = ctx.part_element_world_aabb(top_panel, elem="lid_panel")
    door_closed = ctx.part_element_world_aabb(bezel_door, elem="door_panel")
    with ctx.pose({top_hinge: TOP_PANEL_OPEN, door_hinge: FRONT_DOOR_OPEN}):
        lid_open = ctx.part_element_world_aabb(top_panel, elem="lid_panel")
        door_open = ctx.part_element_world_aabb(bezel_door, elem="door_panel")

    ctx.check(
        "top panel swings upward from the rear edge",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.10
        and lid_open[0][1] > lid_closed[0][1] + 0.03,
        details=f"closed={lid_closed}, open={lid_open}",
    )
    ctx.check(
        "front bezel door swings forward from the left edge",
        door_closed is not None
        and door_open is not None
        and door_open[0][1] < door_closed[0][1] - 0.10
        and door_open[0][0] < door_closed[0][0] + 0.01,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
