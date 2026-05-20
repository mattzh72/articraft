from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


COLUMN_SPACING = 0.88
FOOT_WIDTH = 0.10
FOOT_DEPTH = 0.72
FOOT_HEIGHT = 0.04
PEDESTAL_WIDTH = 0.14
PEDESTAL_DEPTH = 0.18
PEDESTAL_HEIGHT = 0.14
LOWER_CROSSBAR_LENGTH = COLUMN_SPACING - PEDESTAL_WIDTH
LOWER_CROSSBAR_DEPTH = 0.07
LOWER_CROSSBAR_HEIGHT = 0.05
LOWER_CROSSBAR_Z = 0.15

OUTER_WIDTH = 0.12
OUTER_DEPTH = 0.09
OUTER_HEIGHT = 0.62
OUTER_WALL = 0.012

INNER_WIDTH = 0.088
INNER_DEPTH = 0.058
INNER_HEIGHT = 0.68
INNER_TRAVEL = 0.22

BRIDGE_LENGTH = COLUMN_SPACING
BRIDGE_DEPTH = 0.13
BRIDGE_HEIGHT = 0.07

WORKTOP_WIDTH = 1.15
WORKTOP_DEPTH = 0.72
WORKTOP_THICKNESS = 0.028
WORKTOP_TILT_MAX = 0.92

CONTROL_STRIP_WIDTH = 0.27
CONTROL_STRIP_DEPTH = 0.045
CONTROL_STRIP_HEIGHT = 0.038
BUTTON_WIDTH = 0.032
BUTTON_DEPTH = 0.015
BUTTON_HEIGHT = 0.012
BUTTON_TRAVEL = 0.004
CONTROL_WALL = 0.004
HINGE_LEAF_WIDTH = 0.08
HINGE_LEAF_DEPTH = 0.02
HINGE_LEAF_HEIGHT = 0.04
BUTTON_STEM_WIDTH = 0.018
BUTTON_STEM_LENGTH = 0.03
BUTTON_STEM_HEIGHT = 0.018
BUTTON_GUIDE_WALL = 0.005
BUTTON_GUIDE_LENGTH = 0.03


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    powder = model.material("powder_coat", rgba=(0.23, 0.25, 0.28, 1.0))
    steel = model.material("steel_gray", rgba=(0.61, 0.64, 0.68, 1.0))
    top_finish = model.material("laminate_oak", rgba=(0.73, 0.61, 0.44, 1.0))
    control_finish = model.material("control_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    button_finish = model.material("button_black", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    left_x = -COLUMN_SPACING / 2.0
    right_x = COLUMN_SPACING / 2.0

    for x_pos in (left_x, right_x):
        base.visual(
            Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, FOOT_HEIGHT / 2.0)),
            material=powder,
            name=f"foot_{'left' if x_pos < 0.0 else 'right'}",
        )
        base.visual(
            Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
            origin=Origin(
                xyz=(x_pos, 0.0, FOOT_HEIGHT + PEDESTAL_HEIGHT / 2.0)
            ),
            material=powder,
            name=f"pedestal_{'left' if x_pos < 0.0 else 'right'}",
        )
    base.visual(
        Box((LOWER_CROSSBAR_LENGTH, LOWER_CROSSBAR_DEPTH, LOWER_CROSSBAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_CROSSBAR_Z)),
        material=powder,
        name="lower_crossbar",
    )

    left_outer = model.part("left_outer_sleeve")
    right_outer = model.part("right_outer_sleeve")
    for outer in (left_outer, right_outer):
        _add_outer_sleeve_visuals(outer, material=steel)

    left_stage = model.part("left_inner_stage")
    right_stage = model.part("right_inner_stage")
    for stage, prefix in ((left_stage, "left"), (right_stage, "right")):
        stage.visual(
            Box((INNER_WIDTH, INNER_DEPTH, INNER_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, INNER_HEIGHT / 2.0)),
            material=steel,
            name=f"{prefix}_stage_tube",
        )
        stage.visual(
            Box((0.11, 0.08, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, INNER_HEIGHT - 0.01)),
            material=powder,
            name=f"{prefix}_top_cap",
        )

    bridge = model.part("lifting_bridge")
    bridge.visual(
        Box((BRIDGE_LENGTH, BRIDGE_DEPTH, BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(BRIDGE_LENGTH / 2.0, 0.0, BRIDGE_HEIGHT / 2.0)
        ),
        material=powder,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.11, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=powder,
        name="left_support_pad",
    )
    bridge.visual(
        Box((0.11, 0.08, 0.018)),
        origin=Origin(xyz=(BRIDGE_LENGTH, 0.0, 0.009)),
        material=powder,
        name="right_support_pad",
    )
    bridge.visual(
        Box((0.07, 0.06, 0.12)),
        origin=Origin(xyz=(0.02, -0.08, 0.06)),
        material=powder,
        name="left_hinge_cheek",
    )
    bridge.visual(
        Box((0.07, 0.06, 0.12)),
        origin=Origin(xyz=(BRIDGE_LENGTH - 0.02, -0.08, 0.06)),
        material=powder,
        name="right_hinge_cheek",
    )

    worktop = model.part("worktop")
    worktop.visual(
        Box((WORKTOP_WIDTH, WORKTOP_DEPTH, WORKTOP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, WORKTOP_DEPTH / 2.0 + 0.02, WORKTOP_THICKNESS / 2.0)
        ),
        material=top_finish,
        name="top_panel",
    )
    worktop.visual(
        Box((WORKTOP_WIDTH - 0.14, 0.09, 0.044)),
        origin=Origin(xyz=(0.0, 0.64, -0.022)),
        material=powder,
        name="front_apron",
    )
    worktop.visual(
        Box((HINGE_LEAF_WIDTH, HINGE_LEAF_DEPTH, HINGE_LEAF_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BRIDGE_LENGTH / 2.0 - 0.02),
                0.025,
                -0.005,
            )
        ),
        material=powder,
        name="left_hinge_leaf",
    )
    worktop.visual(
        Box((HINGE_LEAF_WIDTH, HINGE_LEAF_DEPTH, HINGE_LEAF_HEIGHT)),
        origin=Origin(
            xyz=(
                BRIDGE_LENGTH / 2.0 - 0.02,
                0.025,
                -0.005,
            )
        ),
        material=powder,
        name="right_hinge_leaf",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((CONTROL_STRIP_WIDTH, CONTROL_WALL, CONTROL_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CONTROL_STRIP_DEPTH / 2.0 + CONTROL_WALL / 2.0, 0.0)
        ),
        material=control_finish,
        name="control_back",
    )
    control_strip.visual(
        Box((CONTROL_WALL, CONTROL_STRIP_DEPTH - CONTROL_WALL, CONTROL_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                -CONTROL_STRIP_WIDTH / 2.0 + CONTROL_WALL / 2.0,
                CONTROL_WALL / 2.0,
                0.0,
            )
        ),
        material=control_finish,
        name="control_left_wall",
    )
    control_strip.visual(
        Box((CONTROL_WALL, CONTROL_STRIP_DEPTH - CONTROL_WALL, CONTROL_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_STRIP_WIDTH / 2.0 - CONTROL_WALL / 2.0,
                CONTROL_WALL / 2.0,
                0.0,
            )
        ),
        material=control_finish,
        name="control_right_wall",
    )
    control_strip.visual(
        Box(
            (
                CONTROL_STRIP_WIDTH - 2.0 * CONTROL_WALL,
                CONTROL_STRIP_DEPTH - CONTROL_WALL,
                CONTROL_WALL,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_WALL / 2.0,
                CONTROL_STRIP_HEIGHT / 2.0 - CONTROL_WALL / 2.0,
            )
        ),
        material=control_finish,
        name="control_top",
    )
    control_strip.visual(
        Box(
            (
                CONTROL_STRIP_WIDTH - 2.0 * CONTROL_WALL,
                CONTROL_STRIP_DEPTH - CONTROL_WALL,
                CONTROL_WALL,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_WALL / 2.0,
                -CONTROL_STRIP_HEIGHT / 2.0 + CONTROL_WALL / 2.0,
            )
        ),
        material=control_finish,
        name="control_bottom",
    )
    for x_pos, prefix in ((-0.04, "up"), (0.04, "down")):
        control_strip.visual(
            Box(
                (
                    BUTTON_GUIDE_WALL,
                    BUTTON_GUIDE_LENGTH,
                    CONTROL_STRIP_HEIGHT - 2.0 * CONTROL_WALL,
                )
            ),
            origin=Origin(
                xyz=(
                    x_pos
                    - (BUTTON_STEM_WIDTH / 2.0 + BUTTON_GUIDE_WALL / 2.0),
                    -CONTROL_WALL / 2.0,
                    0.0,
                )
            ),
            material=control_finish,
            name=f"{prefix}_left_guide",
        )
        control_strip.visual(
            Box(
                (
                    BUTTON_GUIDE_WALL,
                    BUTTON_GUIDE_LENGTH,
                    CONTROL_STRIP_HEIGHT - 2.0 * CONTROL_WALL,
                )
            ),
            origin=Origin(
                xyz=(
                    x_pos
                    + (BUTTON_STEM_WIDTH / 2.0 + BUTTON_GUIDE_WALL / 2.0),
                    -CONTROL_WALL / 2.0,
                    0.0,
                )
            ),
            material=control_finish,
            name=f"{prefix}_right_guide",
        )

    button_up = model.part("button_up")
    button_up.visual(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, BUTTON_DEPTH / 2.0, 0.0)),
        material=button_finish,
        name="button_up_cap",
    )
    button_up.visual(
        Box((BUTTON_STEM_WIDTH, BUTTON_STEM_LENGTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=button_finish,
        name="button_up_stem",
    )
    button_down = model.part("button_down")
    button_down.visual(
        Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(xyz=(0.0, BUTTON_DEPTH / 2.0, 0.0)),
        material=button_finish,
        name="button_down_cap",
    )
    button_down.visual(
        Box((BUTTON_STEM_WIDTH, BUTTON_STEM_LENGTH, BUTTON_STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=button_finish,
        name="button_down_stem",
    )

    sleeve_z = FOOT_HEIGHT + PEDESTAL_HEIGHT
    model.articulation(
        "base_to_left_outer",
        ArticulationType.FIXED,
        parent=base,
        child=left_outer,
        origin=Origin(xyz=(left_x, 0.0, sleeve_z)),
    )
    model.articulation(
        "base_to_right_outer",
        ArticulationType.FIXED,
        parent=base,
        child=right_outer,
        origin=Origin(xyz=(right_x, 0.0, sleeve_z)),
    )
    model.articulation(
        "left_column_lift",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_WALL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=600.0,
            velocity=0.08,
        ),
    )
    model.articulation(
        "right_column_lift",
        ArticulationType.PRISMATIC,
        parent=right_outer,
        child=right_stage,
        origin=Origin(xyz=(0.0, 0.0, OUTER_WALL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=600.0,
            velocity=0.08,
        ),
    )
    model.articulation(
        "left_stage_to_bridge",
        ArticulationType.FIXED,
        parent=left_stage,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, INNER_HEIGHT)),
    )
    model.articulation(
        "bridge_to_worktop",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=worktop,
        origin=Origin(xyz=(BRIDGE_LENGTH / 2.0, -0.065, BRIDGE_HEIGHT + 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=WORKTOP_TILT_MAX,
            effort=45.0,
            velocity=0.7,
        ),
    )
    model.articulation(
        "bridge_to_control_strip",
        ArticulationType.FIXED,
        parent=bridge,
        child=control_strip,
        origin=Origin(xyz=(BRIDGE_LENGTH / 2.0, 0.0875, 0.015)),
    )
    model.articulation(
        "press_button_up",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=button_up,
        origin=Origin(xyz=(-0.04, CONTROL_STRIP_DEPTH / 2.0 + BUTTON_TRAVEL, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BUTTON_TRAVEL,
            effort=5.0,
            velocity=0.03,
        ),
    )
    model.articulation(
        "press_button_down",
        ArticulationType.PRISMATIC,
        parent=control_strip,
        child=button_down,
        origin=Origin(xyz=(0.04, CONTROL_STRIP_DEPTH / 2.0 + BUTTON_TRAVEL, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BUTTON_TRAVEL,
            effort=5.0,
            velocity=0.03,
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

    left_outer = object_model.get_part("left_outer_sleeve")
    right_outer = object_model.get_part("right_outer_sleeve")
    left_stage = object_model.get_part("left_inner_stage")
    right_stage = object_model.get_part("right_inner_stage")
    bridge = object_model.get_part("lifting_bridge")
    worktop = object_model.get_part("worktop")
    control_strip = object_model.get_part("control_strip")
    button_up = object_model.get_part("button_up")
    button_down = object_model.get_part("button_down")

    left_lift = object_model.get_articulation("left_column_lift")
    right_lift = object_model.get_articulation("right_column_lift")
    tilt = object_model.get_articulation("bridge_to_worktop")
    press_up = object_model.get_articulation("press_button_up")
    press_down = object_model.get_articulation("press_button_down")

    ctx.expect_within(
        left_stage,
        left_outer,
        axes="xy",
        name="left stage stays centered inside left sleeve",
    )
    ctx.expect_within(
        right_stage,
        right_outer,
        axes="xy",
        name="right stage stays centered inside right sleeve",
    )
    ctx.expect_overlap(
        left_stage,
        left_outer,
        axes="z",
        min_overlap=0.38,
        name="left stage retains insertion at rest",
    )
    ctx.expect_overlap(
        right_stage,
        right_outer,
        axes="z",
        min_overlap=0.38,
        name="right stage retains insertion at rest",
    )
    ctx.expect_contact(
        right_stage,
        bridge,
        name="right stage supports the bridge at rest",
    )
    ctx.expect_contact(
        control_strip,
        bridge,
        name="control strip mounts to the front of the bridge",
    )
    ctx.expect_contact(
        button_up,
        control_strip,
        name="up button is captured by the control strip guides at rest",
    )
    ctx.expect_contact(
        button_down,
        control_strip,
        name="down button is captured by the control strip guides at rest",
    )
    ctx.expect_contact(
        worktop,
        bridge,
        name="worktop rests on the tilt support carriage when level",
    )

    with ctx.pose({left_lift: INNER_TRAVEL, right_lift: INNER_TRAVEL}):
        ctx.expect_overlap(
            left_stage,
            left_outer,
            axes="z",
            min_overlap=0.16,
            name="left stage retains insertion when raised",
        )
        ctx.expect_overlap(
            right_stage,
            right_outer,
            axes="z",
            min_overlap=0.16,
            name="right stage retains insertion when raised",
        )
        ctx.expect_contact(
            right_stage,
            bridge,
            name="right stage still supports the bridge when raised",
        )

    closed_aabb = ctx.part_element_world_aabb(worktop, elem="top_panel")
    with ctx.pose({tilt: WORKTOP_TILT_MAX}):
        tilted_aabb = ctx.part_element_world_aabb(worktop, elem="top_panel")
    ctx.check(
        "worktop tilts upward at the back edge",
        closed_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    up_rest = ctx.part_world_position(button_up)
    down_rest = ctx.part_world_position(button_down)
    with ctx.pose({press_up: BUTTON_TRAVEL, press_down: BUTTON_TRAVEL}):
        up_pressed = ctx.part_world_position(button_up)
        down_pressed = ctx.part_world_position(button_down)
        ctx.expect_contact(
            button_up,
            control_strip,
            name="up button remains guided when pressed",
        )
        ctx.expect_contact(
            button_down,
            control_strip,
            name="down button remains guided when pressed",
        )
    ctx.check(
        "control buttons press inward on short guides",
        up_rest is not None
        and up_pressed is not None
        and down_rest is not None
        and down_pressed is not None
        and up_pressed[1] < up_rest[1] - 0.003
        and down_pressed[1] < down_rest[1] - 0.003,
        details=(
            f"up_rest={up_rest}, up_pressed={up_pressed}, "
            f"down_rest={down_rest}, down_pressed={down_pressed}"
        ),
    )

    ctx.check(
        "prompt critical parts exist",
        all(
            part is not None
            for part in (
                left_outer,
                right_outer,
                left_stage,
                right_stage,
                bridge,
                worktop,
                control_strip,
                button_up,
                button_down,
            )
        ),
    )

    return ctx.report()


def _add_outer_sleeve_visuals(part, *, material) -> None:
    wall_height = OUTER_HEIGHT
    side_wall_height = OUTER_HEIGHT
    part.visual(
        Box((OUTER_WIDTH, OUTER_WALL, wall_height)),
        origin=Origin(xyz=(0.0, OUTER_DEPTH / 2.0 - OUTER_WALL / 2.0, wall_height / 2.0)),
        material=material,
        name="rear_wall",
    )
    part.visual(
        Box((OUTER_WIDTH, OUTER_WALL, wall_height)),
        origin=Origin(xyz=(0.0, -OUTER_DEPTH / 2.0 + OUTER_WALL / 2.0, wall_height / 2.0)),
        material=material,
        name="front_wall",
    )
    part.visual(
        Box((OUTER_WALL, OUTER_DEPTH - 2.0 * OUTER_WALL, side_wall_height)),
        origin=Origin(
            xyz=(
                OUTER_WIDTH / 2.0 - OUTER_WALL / 2.0,
                0.0,
                side_wall_height / 2.0,
            )
        ),
        material=material,
        name="right_wall",
    )
    part.visual(
        Box((OUTER_WALL, OUTER_DEPTH - 2.0 * OUTER_WALL, side_wall_height)),
        origin=Origin(
            xyz=(
                -OUTER_WIDTH / 2.0 + OUTER_WALL / 2.0,
                0.0,
                side_wall_height / 2.0,
            )
        ),
        material=material,
        name="left_wall",
    )
    part.visual(
        Box((OUTER_WIDTH, OUTER_DEPTH, OUTER_WALL)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_WALL / 2.0)),
        material=material,
        name="bottom_plate",
    )


# >>> USER_CODE_END

object_model = build_object_model()
