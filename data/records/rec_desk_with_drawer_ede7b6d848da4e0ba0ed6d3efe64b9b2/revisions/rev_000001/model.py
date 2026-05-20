from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.86
CABINET_DEPTH = 0.32
PANEL_THICKNESS = 0.018
BASE_Z = 0.70
DRAWER_BAY_HEIGHT = 0.20
TOTAL_HEIGHT = 0.84

LEAF_WIDTH = CABINET_WIDTH - 0.028
LEAF_THICKNESS = 0.022
LEAF_LENGTH = 0.58

DRAWER_FRONT_WIDTH = CABINET_WIDTH - 0.052
DRAWER_FRONT_HEIGHT = 0.152
DRAWER_FRONT_THICKNESS = 0.020
DRAWER_SHELL_WIDTH = CABINET_WIDTH - 0.052
DRAWER_SHELL_DEPTH = 0.257
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BASE_THICKNESS = 0.012
DRAWER_SHELL_HEIGHT = 0.140
DRAWER_RUNNER_THICKNESS = 0.008
DRAWER_TRAVEL = 0.16


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_laundry_desk")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.93, 0.91, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.78, 0.66, 0.50, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.76, 0.78, 0.79, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.67, 0.69, 0.72, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_WIDTH, PANEL_THICKNESS, TOTAL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS / 2.0, BASE_Z + TOTAL_HEIGHT / 2.0)),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, TOTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + PANEL_THICKNESS / 2.0,
                CABINET_DEPTH / 2.0,
                BASE_Z + TOTAL_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((PANEL_THICKNESS, CABINET_DEPTH, TOTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - PANEL_THICKNESS / 2.0,
                CABINET_DEPTH / 2.0,
                BASE_Z + TOTAL_HEIGHT / 2.0,
            )
        ),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH / 2.0, BASE_Z + TOTAL_HEIGHT - PANEL_THICKNESS / 2.0)
        ),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH / 2.0, BASE_Z + DRAWER_BAY_HEIGHT + PANEL_THICKNESS / 2.0)
        ),
        material=cabinet_white,
        name="divider_panel",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0, BASE_Z + PANEL_THICKNESS / 2.0)),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((DRAWER_RUNNER_THICKNESS, 0.22, 0.026)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH / 2.0 + PANEL_THICKNESS + DRAWER_RUNNER_THICKNESS / 2.0,
                0.21,
                BASE_Z + 0.095,
            )
        ),
        material=rail_metal,
        name="left_rail",
    )
    cabinet.visual(
        Box((DRAWER_RUNNER_THICKNESS, 0.22, 0.026)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH / 2.0 - PANEL_THICKNESS - DRAWER_RUNNER_THICKNESS / 2.0,
                0.21,
                BASE_Z + 0.095,
            )
        ),
        material=rail_metal,
        name="right_rail",
    )

    leaf = model.part("leaf")
    leaf.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, LEAF_LENGTH)),
        origin=Origin(xyz=(0.0, LEAF_THICKNESS / 2.0, LEAF_LENGTH / 2.0)),
        material=warm_wood,
        name="leaf_panel",
    )
    leaf.visual(
        Box((LEAF_WIDTH - 0.10, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, -0.001, 0.085)),
        material=warm_wood,
        name="lower_stiffener",
    )
    leaf.visual(
        Box((LEAF_WIDTH - 0.10, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, -0.001, 0.445)),
        material=warm_wood,
        name="upper_stiffener",
    )
    leaf.visual(
        Box((0.24, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.018, LEAF_LENGTH - 0.050)),
        material=warm_wood,
        name="leaf_pull",
    )

    drawer = model.part("sorting_drawer")
    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
        origin=Origin(
            xyz=(0.0, DRAWER_FRONT_THICKNESS / 2.0, DRAWER_FRONT_HEIGHT / 2.0)
        ),
        material=cabinet_white,
        name="drawer_front",
    )
    drawer.visual(
        Box(
            (
                DRAWER_SHELL_WIDTH,
                DRAWER_SHELL_DEPTH,
                DRAWER_BASE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -DRAWER_SHELL_DEPTH / 2.0,
                DRAWER_BASE_THICKNESS / 2.0,
            )
        ),
        material=drawer_gray,
        name="drawer_shell_base",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_SHELL_DEPTH, DRAWER_SHELL_HEIGHT)),
        origin=Origin(
            xyz=(
                -DRAWER_SHELL_WIDTH / 2.0 + DRAWER_SIDE_THICKNESS / 2.0,
                -DRAWER_SHELL_DEPTH / 2.0,
                DRAWER_SHELL_HEIGHT / 2.0,
            )
        ),
        material=drawer_gray,
        name="left_bin_wall",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_SHELL_DEPTH, DRAWER_SHELL_HEIGHT)),
        origin=Origin(
            xyz=(
                DRAWER_SHELL_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
                -DRAWER_SHELL_DEPTH / 2.0,
                DRAWER_SHELL_HEIGHT / 2.0,
            )
        ),
        material=drawer_gray,
        name="right_bin_wall",
    )
    drawer.visual(
        Box(
            (
                DRAWER_SHELL_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_SIDE_THICKNESS,
                DRAWER_SHELL_HEIGHT - 0.010,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -DRAWER_SHELL_DEPTH + DRAWER_SIDE_THICKNESS / 2.0,
                (DRAWER_SHELL_HEIGHT - 0.010) / 2.0,
            )
        ),
        material=drawer_gray,
        name="rear_bin_wall",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_SHELL_DEPTH - 0.014, DRAWER_SHELL_HEIGHT - 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                -(DRAWER_SHELL_DEPTH - 0.014) / 2.0,
                (DRAWER_SHELL_HEIGHT - 0.020) / 2.0,
            )
        ),
        material=drawer_gray,
        name="center_divider",
    )
    drawer.visual(
        Box((DRAWER_RUNNER_THICKNESS, 0.22, 0.022)),
        origin=Origin(
            xyz=(
                -DRAWER_SHELL_WIDTH / 2.0 + DRAWER_RUNNER_THICKNESS / 2.0,
                -0.11,
                0.095,
            )
        ),
        material=rail_metal,
        name="left_runner",
    )
    drawer.visual(
        Box((DRAWER_RUNNER_THICKNESS, 0.22, 0.022)),
        origin=Origin(
            xyz=(
                DRAWER_SHELL_WIDTH / 2.0 - DRAWER_RUNNER_THICKNESS / 2.0,
                -0.11,
                0.095,
            )
        ),
        material=rail_metal,
        name="right_runner",
    )
    drawer.visual(
        Box((0.22, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.019, DRAWER_FRONT_HEIGHT / 2.0)),
        material=rail_metal,
        name="drawer_pull",
    )

    model.articulation(
        "cabinet_to_leaf",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=leaf,
        origin=Origin(xyz=(0.0, CABINET_DEPTH, BASE_Z + DRAWER_BAY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )
    model.articulation(
        "cabinet_to_sorting_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, CABINET_DEPTH, BASE_Z + 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
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

    cabinet = object_model.get_part("cabinet")
    leaf = object_model.get_part("leaf")
    drawer = object_model.get_part("sorting_drawer")
    leaf_hinge = object_model.get_articulation("cabinet_to_leaf")
    drawer_slide = object_model.get_articulation("cabinet_to_sorting_drawer")

    ctx.expect_gap(
        leaf,
        cabinet,
        axis="y",
        positive_elem="leaf_panel",
        negative_elem="divider_panel",
        min_gap=0.0,
        max_gap=0.003,
        name="closed leaf seats at cabinet front edge",
    )
    ctx.expect_overlap(
        leaf,
        cabinet,
        axes="xz",
        elem_a="leaf_panel",
        min_overlap=0.50,
        name="closed leaf covers the cabinet opening footprint",
    )
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="y",
        positive_elem="drawer_front",
        min_gap=0.0,
        max_gap=0.003,
        name="closed drawer front sits flush with the cabinet face",
    )
    ctx.expect_within(
        drawer,
        cabinet,
        axes="xz",
        margin=0.02,
        name="drawer remains centered within cabinet width and height",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        elem_a="drawer_shell_base",
        min_overlap=0.10,
        name="closed drawer shell stays inserted inside the housing",
    )

    closed_leaf_aabb = ctx.part_world_aabb(leaf)
    with ctx.pose({leaf_hinge: 1.55}):
        open_leaf_aabb = ctx.part_world_aabb(leaf)
        ctx.expect_gap(
            leaf,
            drawer,
            axis="z",
            positive_elem="leaf_panel",
            negative_elem="drawer_front",
            min_gap=0.0,
            max_gap=0.02,
            name="open leaf clears the top of the drawer front",
        )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        open_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_within(
            drawer,
            cabinet,
            axes="xz",
            margin=0.02,
            name="extended drawer stays aligned with the cabinet cavity",
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_shell_base",
            min_overlap=0.08,
            name="extended drawer retains insertion on its guide rails",
        )

    open_leaf_depth = (
        None
        if open_leaf_aabb is None
        else open_leaf_aabb[1][1] - open_leaf_aabb[0][1]
    )
    open_leaf_thickness = (
        None
        if open_leaf_aabb is None
        else open_leaf_aabb[1][2] - open_leaf_aabb[0][2]
    )
    closed_leaf_height = (
        None
        if closed_leaf_aabb is None
        else closed_leaf_aabb[1][2] - closed_leaf_aabb[0][2]
    )

    ctx.check(
        "leaf rotates from vertical panel to near-horizontal desk",
        closed_leaf_height is not None
        and open_leaf_depth is not None
        and open_leaf_thickness is not None
        and closed_leaf_height > 0.55
        and open_leaf_depth > 0.50
        and open_leaf_thickness < 0.05,
        details=(
            f"closed_leaf_height={closed_leaf_height}, "
            f"open_leaf_depth={open_leaf_depth}, "
            f"open_leaf_thickness={open_leaf_thickness}"
        ),
    )
    ctx.check(
        "sorting drawer pulls outward",
        closed_drawer_pos is not None
        and open_drawer_pos is not None
        and open_drawer_pos[1] > closed_drawer_pos[1] + 0.12,
        details=f"closed={closed_drawer_pos}, open={open_drawer_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
