from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.46
BASE_DEPTH = 0.31
BASE_THICKNESS = 0.016
BASE_TOP_Z = BASE_THICKNESS

HINGE_AXIS_X = -BASE_WIDTH / 2 + 0.012
HINGE_AXIS_Z = 0.023
HINGE_RADIUS = 0.006

ARM_LENGTH = 0.43
ARM_WIDTH = 0.028
ARM_THICKNESS = 0.012
ARM_BARREL_LENGTH = 0.16

FENCE_LENGTH = 0.20
FENCE_THICKNESS = 0.014
FENCE_HEIGHT = 0.020
FENCE_PIVOT_X = -0.04
FENCE_PIVOT_Y = BASE_DEPTH / 2 - 0.040


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_paper_guillotine")

    board_mat = model.material("board", rgba=(0.88, 0.90, 0.86, 1.0))
    metal_mat = model.material("metal", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_mat = model.material("dark_handle", rgba=(0.18, 0.18, 0.20, 1.0))
    blade_mat = model.material("blade", rgba=(0.80, 0.82, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2)),
        material=board_mat,
        name="base_board",
    )
    base.visual(
        Box((0.020, 0.058, 0.014)),
        origin=Origin(xyz=(HINGE_AXIS_X, -0.105, BASE_TOP_Z + 0.007)),
        material=metal_mat,
        name="base_rear_mount",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.05),
        origin=Origin(
            xyz=(HINGE_AXIS_X, -0.105, HINGE_AXIS_Z),
            rpy=(-pi / 2, 0.0, 0.0),
        ),
        material=metal_mat,
        name="base_rear_knuckle",
    )
    base.visual(
        Box((0.020, 0.058, 0.014)),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.105, BASE_TOP_Z + 0.007)),
        material=metal_mat,
        name="base_front_mount",
    )
    base.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.05),
        origin=Origin(
            xyz=(HINGE_AXIS_X, 0.105, HINGE_AXIS_Z),
            rpy=(-pi / 2, 0.0, 0.0),
        ),
        material=metal_mat,
        name="base_front_knuckle",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.001),
        origin=Origin(xyz=(FENCE_PIVOT_X, FENCE_PIVOT_Y, BASE_TOP_Z + 0.0005)),
        material=metal_mat,
        name="fence_pivot_washer",
    )

    cutting_arm = model.part("cutting_arm")
    cutting_arm.visual(
        Cylinder(radius=HINGE_RADIUS, length=ARM_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=metal_mat,
        name="arm_center_barrel",
    )
    cutting_arm.visual(
        Box((0.024, ARM_BARREL_LENGTH, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.005)),
        material=dark_mat,
        name="arm_hinge_web",
    )
    cutting_arm.visual(
        Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(xyz=(ARM_LENGTH / 2, 0.0, ARM_THICKNESS / 2)),
        material=dark_mat,
        name="arm_beam",
    )
    cutting_arm.visual(
        Box((0.39, 0.008, 0.007)),
        origin=Origin(xyz=(0.205, 0.0, -0.0035)),
        material=blade_mat,
        name="arm_blade",
    )
    cutting_arm.visual(
        Box((0.048, 0.030, 0.024)),
        origin=Origin(xyz=(0.406, 0.0, 0.012)),
        material=dark_mat,
        name="arm_handle_block",
    )
    cutting_arm.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.406, 0.0, 0.030), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_mat,
        name="arm_grip",
    )

    guide_fence = model.part("guide_fence")
    guide_fence.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, 0.0, 0.0)),
        material=metal_mat,
        name="fence_pivot_collar",
    )
    guide_fence.visual(
        Box((FENCE_LENGTH, FENCE_THICKNESS, FENCE_HEIGHT)),
        origin=Origin(
            xyz=(FENCE_LENGTH / 2, 0.0, 0.011),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=metal_mat,
        name="fence_rail",
    )

    model.articulation(
        "base_to_cutting_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cutting_arm,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(78.0),
        ),
    )
    model.articulation(
        "base_to_guide_fence",
        ArticulationType.REVOLUTE,
        parent=base,
        child=guide_fence,
        origin=Origin(xyz=(FENCE_PIVOT_X, FENCE_PIVOT_Y, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=radians(-50.0),
            upper=radians(50.0),
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
    cutting_arm = object_model.get_part("cutting_arm")
    guide_fence = object_model.get_part("guide_fence")
    arm_joint = object_model.get_articulation("base_to_cutting_arm")
    fence_joint = object_model.get_articulation("base_to_guide_fence")

    ctx.check(
        "all guillotine parts exist",
        all(part is not None for part in (base, cutting_arm, guide_fence)),
    )

    with ctx.pose({arm_joint: 0.0, fence_joint: 0.0}):
        ctx.expect_contact(
            cutting_arm,
            base,
            elem_a="arm_center_barrel",
            elem_b="base_front_knuckle",
            contact_tol=1e-4,
            name="arm hinge barrel meets front knuckle",
        )
        ctx.expect_gap(
            cutting_arm,
            base,
            axis="z",
            positive_elem="arm_blade",
            negative_elem="base_board",
            min_gap=0.0,
            max_gap=0.001,
            name="blade closes down to the board surface",
        )
        ctx.expect_overlap(
            cutting_arm,
            base,
            axes="xy",
            min_overlap=0.020,
            name="closed arm covers the base board footprint",
        )
        ctx.expect_contact(
            guide_fence,
            base,
            elem_a="fence_pivot_collar",
            elem_b="fence_pivot_washer",
            contact_tol=1e-4,
            name="fence sits on its pivot washer",
        )

    arm_closed = ctx.part_element_world_aabb(cutting_arm, elem="arm_handle_block")
    with ctx.pose({arm_joint: arm_joint.motion_limits.upper}):
        ctx.expect_gap(
            cutting_arm,
            base,
            axis="z",
            positive_elem="arm_handle_block",
            negative_elem="base_board",
            min_gap=0.15,
            name="raised cutting arm lifts well clear of the base",
        )
        arm_open = ctx.part_element_world_aabb(cutting_arm, elem="arm_handle_block")

    ctx.check(
        "cutting arm opens upward",
        arm_closed is not None
        and arm_open is not None
        and arm_open[0][2] > arm_closed[0][2] + 0.12,
        details=f"closed={arm_closed}, open={arm_open}",
    )

    fence_rest = ctx.part_element_world_aabb(guide_fence, elem="fence_rail")
    with ctx.pose({fence_joint: radians(40.0)}):
        ctx.expect_contact(
            guide_fence,
            base,
            elem_a="fence_pivot_collar",
            elem_b="fence_pivot_washer",
            contact_tol=1e-4,
            name="fence keeps its pivot contact while swung",
        )
        fence_swung = ctx.part_element_world_aabb(guide_fence, elem="fence_rail")

    rest_y = None if fence_rest is None else fence_rest[1][1] - fence_rest[0][1]
    swung_y = None if fence_swung is None else fence_swung[1][1] - fence_swung[0][1]
    ctx.check(
        "guide fence changes angle for paper setting",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.08,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
