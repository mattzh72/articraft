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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


TOP_WIDTH = 0.46
TOP_DEPTH = 0.44
TOP_THICKNESS = 0.025

CARCASS_WIDTH = 0.40
CARCASS_DEPTH = 0.38
CARCASS_HEIGHT = 0.405

LEG_SIZE = 0.038
LEG_HEIGHT = 0.150

SIDE_THICKNESS = 0.016
BACK_THICKNESS = 0.012
APRON_THICKNESS = 0.018
APRON_HEIGHT = 0.035
DIVIDER_THICKNESS = 0.018
SHELF_THICKNESS = 0.018
RAIL_THICKNESS = 0.018

DRAWER_FRONT_THICKNESS = 0.018
DRAWER_FRONT_WIDTH = 0.356
DRAWER_FRONT_HEIGHT = 0.118
DRAWER_BOX_WIDTH = 0.330
DRAWER_BOX_DEPTH = 0.290
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOX_HEIGHT = 0.072
DRAWER_BOTTOM_THICKNESS = 0.008

RUNNER_HEIGHT = 0.010
BODY_RUNNER_WIDTH = 0.034
DRAWER_RUNNER_WIDTH = 0.018
BODY_RUNNER_LENGTH = 0.290
DRAWER_RUNNER_LENGTH = 0.278
DRAWER_TRAVEL = 0.180

BODY_TOP_Z = LEG_HEIGHT + CARCASS_HEIGHT
TOP_CENTER_Z = BODY_TOP_Z + TOP_THICKNESS / 2.0
CARCASS_CENTER_Z = LEG_HEIGHT + CARCASS_HEIGHT / 2.0
DRAWER_OPENING_TOP = BODY_TOP_Z - APRON_HEIGHT
DRAWER_OPENING_BOTTOM = 0.392
DRAWER_CENTER_Z = (DRAWER_OPENING_TOP + DRAWER_OPENING_BOTTOM) / 2.0


def _axis_extent(aabb, axis: str, which: str) -> float | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    bound_index = 0 if which == "min" else 1
    return aabb[bound_index][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_nightstand")

    oak = model.material("oak", rgba=(0.72, 0.58, 0.39, 1.0))
    walnut = model.material("walnut", rgba=(0.54, 0.38, 0.24, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.64, 0.50, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((TOP_DEPTH, TOP_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_CENTER_Z)),
        material=oak,
        name="top",
    )

    leg_x = CARCASS_DEPTH / 2.0 - LEG_SIZE / 2.0
    leg_y = CARCASS_WIDTH / 2.0 - LEG_SIZE / 2.0
    for x_sign, y_sign, name in (
        (1.0, 1.0, "front_right_leg"),
        (1.0, -1.0, "front_left_leg"),
        (-1.0, 1.0, "rear_right_leg"),
        (-1.0, -1.0, "rear_left_leg"),
    ):
        body.visual(
            Box((LEG_SIZE, LEG_SIZE, LEG_HEIGHT)),
            origin=Origin(xyz=(x_sign * leg_x, y_sign * leg_y, LEG_HEIGHT / 2.0)),
            material=oak,
            name=name,
        )

    side_y = CARCASS_WIDTH / 2.0 - SIDE_THICKNESS / 2.0
    for y_sign, name in ((1.0, "right_side"), (-1.0, "left_side")):
        body.visual(
            Box((CARCASS_DEPTH, SIDE_THICKNESS, CARCASS_HEIGHT)),
            origin=Origin(xyz=(0.0, y_sign * side_y, CARCASS_CENTER_Z)),
            material=oak,
            name=name,
        )

    body.visual(
        Box((BACK_THICKNESS, CARCASS_WIDTH - 2.0 * SIDE_THICKNESS, CARCASS_HEIGHT)),
        origin=Origin(
            xyz=(
                -CARCASS_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                0.0,
                CARCASS_CENTER_Z,
            )
        ),
        material=oak,
        name="back_panel",
    )
    body.visual(
        Box((APRON_THICKNESS, CARCASS_WIDTH - 2.0 * SIDE_THICKNESS, APRON_HEIGHT)),
        origin=Origin(
            xyz=(
                CARCASS_DEPTH / 2.0 - APRON_THICKNESS / 2.0,
                0.0,
                BODY_TOP_Z - APRON_HEIGHT / 2.0,
            )
        ),
        material=walnut,
        name="upper_apron",
    )
    body.visual(
        Box((CARCASS_DEPTH - BACK_THICKNESS, CARCASS_WIDTH - 2.0 * SIDE_THICKNESS, DIVIDER_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DRAWER_OPENING_BOTTOM - DIVIDER_THICKNESS / 2.0)),
        material=oak,
        name="drawer_divider",
    )

    shelf_rail_center_x = -0.018
    shelf_rail_length = 0.308
    shelf_rail_z = 0.208
    shelf_y = CARCASS_WIDTH / 2.0 - SIDE_THICKNESS - RAIL_THICKNESS / 2.0
    for y_sign, name in ((1.0, "right_shelf_rail"), (-1.0, "left_shelf_rail")):
        body.visual(
            Box((shelf_rail_length, RAIL_THICKNESS, SHELF_THICKNESS)),
            origin=Origin(xyz=(shelf_rail_center_x, y_sign * shelf_y, shelf_rail_z)),
            material=walnut,
            name=name,
        )
    body.visual(
        Box((0.304, 0.348, SHELF_THICKNESS)),
        origin=Origin(xyz=(-0.020, 0.0, 0.226)),
        material=oak,
        name="lower_shelf",
    )

    body_runner_x = 0.005
    body_runner_y = CARCASS_WIDTH / 2.0 - SIDE_THICKNESS - BODY_RUNNER_WIDTH / 2.0
    body_runner_z = 0.397
    for y_sign, name in ((1.0, "right_body_runner"), (-1.0, "left_body_runner")):
        body.visual(
            Box((BODY_RUNNER_LENGTH, BODY_RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(body_runner_x, y_sign * body_runner_y, body_runner_z)),
            material=walnut,
            name=name,
        )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)),
        origin=Origin(xyz=(-DRAWER_FRONT_THICKNESS / 2.0, 0.0, 0.0)),
        material=walnut,
        name="drawer_front",
    )

    drawer_side_z = -0.008
    drawer_side_y = DRAWER_BOX_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0
    drawer_side_x = -DRAWER_FRONT_THICKNESS - DRAWER_BOX_DEPTH / 2.0
    for y_sign, name in ((1.0, "right_drawer_side"), (-1.0, "left_drawer_side")):
        drawer.visual(
            Box((DRAWER_BOX_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BOX_HEIGHT)),
            origin=Origin(xyz=(drawer_side_x, y_sign * drawer_side_y, drawer_side_z)),
            material=drawer_wood,
            name=name,
        )

    drawer.visual(
        Box(
            (
                DRAWER_SIDE_THICKNESS,
                DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BOX_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                -DRAWER_FRONT_THICKNESS - DRAWER_BOX_DEPTH - DRAWER_SIDE_THICKNESS / 2.0,
                0.0,
                drawer_side_z,
            )
        ),
        material=drawer_wood,
        name="drawer_back",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BOX_DEPTH,
                DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                drawer_side_x,
                0.0,
                -0.040,
            )
        ),
        material=drawer_wood,
        name="drawer_bottom",
    )

    drawer_runner_y = body_runner_y - (BODY_RUNNER_WIDTH - DRAWER_RUNNER_WIDTH) / 2.0
    for y_sign, name in ((1.0, "right_drawer_runner"), (-1.0, "left_drawer_runner")):
        drawer.visual(
            Box((DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(-0.167, y_sign * drawer_runner_y, -0.049)),
            material=drawer_wood,
            name=name,
        )

    drawer.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=walnut,
        name="knob_stem",
    )
    drawer.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=walnut,
        name="knob",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(CARCASS_DEPTH / 2.0, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("body_to_drawer")

    body.get_visual("lower_shelf")
    body.get_visual("left_shelf_rail")
    body.get_visual("right_shelf_rail")
    body.get_visual("left_body_runner")
    body.get_visual("right_body_runner")
    drawer.get_visual("drawer_front")
    drawer.get_visual("left_drawer_runner")
    drawer.get_visual("right_drawer_runner")
    drawer.get_visual("knob")

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
        drawer,
        body,
        elem_a="left_drawer_runner",
        elem_b="left_body_runner",
        name="left drawer runner sits on the left body runner when closed",
    )
    ctx.expect_contact(
        drawer,
        body,
        elem_a="right_drawer_runner",
        elem_b="right_body_runner",
        name="right drawer runner sits on the right body runner when closed",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="y",
        inner_elem="left_drawer_runner",
        outer_elem="left_body_runner",
        margin=0.0015,
        name="left drawer runner stays seated laterally on its body runner",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="y",
        inner_elem="right_drawer_runner",
        outer_elem="right_body_runner",
        margin=0.0015,
        name="right drawer runner stays seated laterally on its body runner",
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="z",
        positive_elem="left_drawer_runner",
        negative_elem="left_body_runner",
        max_gap=0.001,
        max_penetration=0.0,
        name="left drawer runner sits directly on the body runner without interpenetration",
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="z",
        positive_elem="right_drawer_runner",
        negative_elem="right_body_runner",
        max_gap=0.001,
        max_penetration=0.0,
        name="right drawer runner sits directly on the body runner without interpenetration",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="left_drawer_runner",
        elem_b="left_body_runner",
        min_overlap=0.22,
        name="closed drawer keeps deep engagement on the left runner",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="right_drawer_runner",
        elem_b="right_body_runner",
        min_overlap=0.22,
        name="closed drawer keeps deep engagement on the right runner",
    )

    closed_position = ctx.part_world_position(drawer)
    with ctx.pose({slide: DRAWER_TRAVEL}):
        ctx.expect_contact(
            drawer,
            body,
            elem_a="left_drawer_runner",
            elem_b="left_body_runner",
            name="left runner remains in bearing contact when opened",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="right_drawer_runner",
            elem_b="right_body_runner",
            name="right runner remains in bearing contact when opened",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="left_drawer_runner",
            elem_b="left_body_runner",
            min_overlap=0.08,
            name="left runner keeps retained insertion at full travel",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="right_drawer_runner",
            elem_b="right_body_runner",
            min_overlap=0.08,
            name="right runner keeps retained insertion at full travel",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="left_drawer_runner",
            negative_elem="left_body_runner",
            max_gap=0.001,
            max_penetration=0.0,
            name="left runner stays vertically seated at full travel",
        )
        ctx.expect_gap(
            drawer,
            body,
            axis="z",
            positive_elem="right_drawer_runner",
            negative_elem="right_body_runner",
            max_gap=0.001,
            max_penetration=0.0,
            name="right runner stays vertically seated at full travel",
        )
        opened_position = ctx.part_world_position(drawer)

    ctx.check(
        "drawer slides outward from the case",
        closed_position is not None
        and opened_position is not None
        and opened_position[0] > closed_position[0] + DRAWER_TRAVEL - 0.002,
        details=f"closed={closed_position}, opened={opened_position}",
    )

    front_box = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    apron_box = ctx.part_element_world_aabb(body, elem="upper_apron")
    knob_box = ctx.part_element_world_aabb(drawer, elem="knob")
    shelf_box = ctx.part_element_world_aabb(body, elem="lower_shelf")
    left_shelf_rail_box = ctx.part_element_world_aabb(body, elem="left_shelf_rail")
    right_shelf_rail_box = ctx.part_element_world_aabb(body, elem="right_shelf_rail")

    drawer_front_flush_gap = None
    if front_box is not None and apron_box is not None:
        drawer_front_flush_gap = abs(
            _axis_extent(front_box, "x", "max") - _axis_extent(apron_box, "x", "max")
        )
    ctx.check(
        "drawer front sits nearly flush with the front apron plane",
        drawer_front_flush_gap is not None and drawer_front_flush_gap <= 0.004,
        details=f"flush_gap={drawer_front_flush_gap}",
    )

    knob_projection = None
    if front_box is not None and knob_box is not None:
        knob_projection = _axis_extent(knob_box, "x", "max") - _axis_extent(front_box, "x", "max")
    ctx.check(
        "round knob projects proudly from the drawer front",
        knob_projection is not None and knob_projection >= 0.010,
        details=f"knob_projection={knob_projection}",
    )

    shelf_support_gaps = None
    if shelf_box is not None and left_shelf_rail_box is not None and right_shelf_rail_box is not None:
        shelf_bottom = _axis_extent(shelf_box, "z", "min")
        shelf_support_gaps = (
            abs(shelf_bottom - _axis_extent(left_shelf_rail_box, "z", "max")),
            abs(shelf_bottom - _axis_extent(right_shelf_rail_box, "z", "max")),
        )
    ctx.check(
        "lower shelf is carried by fixed side rails",
        shelf_support_gaps is not None and max(shelf_support_gaps) <= 0.001,
        details=f"shelf_support_gaps={shelf_support_gaps}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
