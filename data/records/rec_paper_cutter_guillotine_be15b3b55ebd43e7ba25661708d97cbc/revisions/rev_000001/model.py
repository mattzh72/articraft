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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_guillotine")

    laminate = model.material("laminate", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    black_grip = model.material("black_grip", rgba=(0.12, 0.12, 0.13, 1.0))
    warning_red = model.material("warning_red", rgba=(0.65, 0.10, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.45, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=laminate,
        name="cutting_board",
    )
    base.visual(
        Box((0.016, 0.404, 0.005)),
        origin=Origin(xyz=(0.245, 0.0, 0.0305)),
        material=warning_red,
        name="cut_strip",
    )
    base.visual(
        Box((0.014, 0.060, 0.030)),
        origin=Origin(xyz=(-0.299, 0.183, 0.043)),
        material=dark_steel,
        name="hinge_side_spine",
    )
    base.visual(
        Box((0.010, 0.008, 0.038)),
        origin=Origin(xyz=(-0.294, 0.164, 0.047)),
        material=dark_steel,
        name="hinge_front_cheek",
    )
    base.visual(
        Box((0.010, 0.008, 0.038)),
        origin=Origin(xyz=(-0.294, 0.202, 0.047)),
        material=dark_steel,
        name="hinge_rear_cheek",
    )
    base.visual(
        Box((0.260, 0.010, 0.012)),
        origin=Origin(xyz=(0.110, 0.160, 0.034)),
        material=dark_steel,
        name="guide_front_rail",
    )
    base.visual(
        Box((0.260, 0.010, 0.012)),
        origin=Origin(xyz=(0.110, 0.186, 0.034)),
        material=dark_steel,
        name="guide_rear_rail",
    )
    base.visual(
        Box((0.012, 0.036, 0.018)),
        origin=Origin(xyz=(-0.026, 0.173, 0.037)),
        material=dark_steel,
        name="guide_left_stop",
    )
    base.visual(
        Box((0.012, 0.036, 0.018)),
        origin=Origin(xyz=(0.246, 0.173, 0.037)),
        material=dark_steel,
        name="guide_right_stop",
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="arm_hub",
    )
    blade_arm.visual(
        Box((0.530, 0.085, 0.014)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=steel,
        name="arm_plate",
    )
    blade_arm.visual(
        Box((0.340, 0.022, 0.026)),
        origin=Origin(xyz=(0.170, 0.006, 0.012)),
        material=dark_steel,
        name="arm_spine",
    )
    blade_arm.visual(
        Box((0.500, 0.012, 0.010)),
        origin=Origin(xyz=(0.264, -0.039, 0.001), rpy=(math.pi / 4.0, 0.0, 0.0)),
        material=dark_steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Box((0.115, 0.034, 0.030)),
        origin=Origin(xyz=(0.468, 0.0, 0.020)),
        material=dark_steel,
        name="handle_block",
    )
    blade_arm.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(
            xyz=(0.500, 0.0, 0.044),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_grip,
        name="arm_handle",
    )

    backstop = model.part("backstop")
    backstop.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="slider_block",
    )
    backstop.visual(
        Box((0.016, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_steel,
        name="rear_post",
    )
    backstop.visual(
        Box((0.060, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.015, 0.037)),
        material=dark_steel,
        name="top_bridge",
    )
    backstop.visual(
        Box((0.078, 0.006, 0.045)),
        origin=Origin(xyz=(0.0, -0.030, 0.0315)),
        material=steel,
        name="stop_face",
    )
    backstop.visual(
        Box((0.078, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.021, 0.052)),
        material=black_grip,
        name="stop_cap",
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.274, 0.183, 0.038), rpy=(0.0, 0.0, -0.48)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "base_to_backstop",
        ArticulationType.PRISMATIC,
        parent=base,
        child=backstop,
        origin=Origin(xyz=(0.110, 0.173, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=-0.080,
            upper=0.080,
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
    blade_arm = object_model.get_part("blade_arm")
    backstop = object_model.get_part("backstop")
    blade_joint = object_model.get_articulation("base_to_blade_arm")
    stop_joint = object_model.get_articulation("base_to_backstop")

    def center_of(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    with ctx.pose({blade_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="arm_plate",
            negative_elem="cutting_board",
            min_gap=0.002,
            max_gap=0.006,
            name="closed blade arm hovers just above the board",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            elem_a="arm_plate",
            elem_b="cutting_board",
            min_overlap=0.20,
            name="closed blade arm spans the cutting board",
        )

    rest_handle = center_of(ctx.part_element_world_aabb(blade_arm, elem="arm_handle"))
    with ctx.pose({blade_joint: 1.10}):
        open_handle = center_of(ctx.part_element_world_aabb(blade_arm, elem="arm_handle"))
    ctx.check(
        "blade arm swings out to the operator side",
        rest_handle is not None
        and open_handle is not None
        and open_handle[0] < rest_handle[0] - 0.18
        and open_handle[1] < rest_handle[1] - 0.18,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    guide_aabb = ctx.part_element_world_aabb(base, elem="guide_front_rail")
    rest_slider = ctx.part_element_world_aabb(backstop, elem="slider_block")
    with ctx.pose({stop_joint: stop_joint.motion_limits.upper}):
        max_slider = ctx.part_element_world_aabb(backstop, elem="slider_block")
        max_stop_face = center_of(ctx.part_element_world_aabb(backstop, elem="stop_face"))
    rest_stop_face = center_of(ctx.part_element_world_aabb(backstop, elem="stop_face"))

    guide_ok = False
    if guide_aabb is not None and rest_slider is not None and max_slider is not None:
        guide_min_x = guide_aabb[0][0]
        guide_max_x = guide_aabb[1][0]
        guide_ok = (
            rest_slider[0][0] >= guide_min_x - 1e-6
            and rest_slider[1][0] <= guide_max_x + 1e-6
            and max_slider[0][0] >= guide_min_x - 1e-6
            and max_slider[1][0] <= guide_max_x + 1e-6
        )
    ctx.check(
        "backstop slider stays inside the short rear guide",
        guide_ok,
        details=f"guide={guide_aabb}, rest_slider={rest_slider}, max_slider={max_slider}",
    )
    ctx.check(
        "backstop translates laterally along the rear guide",
        rest_stop_face is not None
        and max_stop_face is not None
        and max_stop_face[0] > rest_stop_face[0] + 0.06
        and abs(max_stop_face[1] - rest_stop_face[1]) < 0.002
        and abs(max_stop_face[2] - rest_stop_face[2]) < 0.002,
        details=f"rest_stop_face={rest_stop_face}, max_stop_face={max_stop_face}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
