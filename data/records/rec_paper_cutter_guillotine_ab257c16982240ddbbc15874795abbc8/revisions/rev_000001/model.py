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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_guillotine")

    base_shell = model.material("base_shell", rgba=(0.20, 0.22, 0.24, 1.0))
    fence_strip = model.material("fence_strip", rgba=(0.50, 0.52, 0.56, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.73, 0.75, 0.78, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.08, 0.09, 0.10, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.73, 0.87, 0.95, 0.34))

    base = model.part("base_board")
    base.visual(
        Box((0.42, 0.30, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=base_shell,
        name="board_panel",
    )
    base.visual(
        Box((0.42, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.139, 0.026)),
        material=fence_strip,
        name="rear_fence",
    )
    base.visual(
        Box((0.33, 0.012, 0.004)),
        origin=Origin(xyz=(0.030, -0.118, 0.018)),
        material=fence_strip,
        name="cutting_strip",
    )
    base.visual(
        Box((0.0255, 0.094, 0.048)),
        # This cheek lands flush against the arm hinge barrel in the rest pose
        # so the arm assembly is mechanically supported by the base.
        origin=Origin(xyz=(-0.197002, 0.0, 0.040)),
        material=base_shell,
        name="side_support",
    )
    base.visual(
        Box((0.030, 0.074, 0.012)),
        origin=Origin(xyz=(-0.195, 0.0, 0.070)),
        material=hinge_dark,
        name="hinge_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.42, 0.30, 0.082)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Box((0.39, 0.054, 0.018)),
        origin=Origin(xyz=(0.215, 0.0, -0.007)),
        material=arm_finish,
        name="arm_body",
    )
    blade_arm.visual(
        Box((0.318, 0.008, 0.016)),
        origin=Origin(xyz=(0.230, -0.023, -0.010)),
        material=hinge_dark,
        name="blade_rail",
    )
    blade_arm.visual(
        Box((0.300, 0.002, 0.006)),
        origin=Origin(xyz=(0.232, -0.028, -0.017)),
        material=blade_steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Cylinder(radius=0.012, length=0.066),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="arm_hinge_barrel",
    )
    blade_arm.visual(
        Cylinder(radius=0.009, length=0.110),
        origin=Origin(xyz=(0.332, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_grip,
        name="arm_handle",
    )
    blade_arm.visual(
        Box((0.088, 0.010, 0.010)),
        origin=Origin(xyz=(0.224, -0.024, -0.001)),
        material=hinge_dark,
        name="guard_mount",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.07, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.205, 0.0, -0.003)),
    )

    guard = model.part("finger_guard")
    guard.visual(
        Box((0.312, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.005, -0.006)),
        material=hinge_dark,
        name="guard_top_rail",
    )
    guard.visual(
        Box((0.304, 0.003, 0.088)),
        origin=Origin(xyz=(0.0, -0.0065, -0.056)),
        material=clear_guard,
        name="guard_panel",
    )
    guard.visual(
        Box((0.292, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.007, -0.100)),
        material=hinge_dark,
        name="guard_lower_stiffener",
    )
    guard.visual(
        Cylinder(radius=0.005, length=0.044),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="guard_hinge_barrel",
    )
    guard.inertial = Inertial.from_geometry(
        Box((0.312, 0.020, 0.105)),
        mass=0.28,
        origin=Origin(xyz=(0.0, -0.006, -0.052)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.194, 0.0, 0.040), rpy=(0.0, -0.58, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.6,
            lower=-0.50,
            upper=0.55,
        ),
    )
    model.articulation(
        "arm_to_finger_guard",
        ArticulationType.REVOLUTE,
        parent=blade_arm,
        child=guard,
        origin=Origin(xyz=(0.224, -0.029, 0.000), rpy=(-0.78, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.55,
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

    base = object_model.get_part("base_board")
    blade_arm = object_model.get_part("blade_arm")
    guard = object_model.get_part("finger_guard")
    arm_joint = object_model.get_articulation("base_to_blade_arm")
    guard_joint = object_model.get_articulation("arm_to_finger_guard")

    arm_lower = arm_joint.motion_limits.lower if arm_joint.motion_limits is not None else None
    arm_upper = arm_joint.motion_limits.upper if arm_joint.motion_limits is not None else None
    guard_lower = guard_joint.motion_limits.lower if guard_joint.motion_limits is not None else None
    guard_upper = guard_joint.motion_limits.upper if guard_joint.motion_limits is not None else None

    with ctx.pose({arm_joint: arm_lower if arm_lower is not None else 0.0, guard_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="arm_body",
            negative_elem="board_panel",
            min_gap=0.004,
            max_gap=0.030,
            name="blade arm clears the board when lowered",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            elem_a="arm_body",
            elem_b="board_panel",
            min_overlap=0.040,
            name="blade arm still spans over the cutting bed",
        )

    with ctx.pose({arm_joint: 0.0, guard_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            guard,
            axis="y",
            positive_elem="arm_body",
            negative_elem="guard_panel",
            min_gap=0.006,
            max_gap=0.050,
            name="clear finger guard sits forward of the blade arm",
        )
        ctx.expect_overlap(
            guard,
            blade_arm,
            axes="x",
            elem_a="guard_panel",
            elem_b="arm_body",
            min_overlap=0.240,
            name="finger guard spans most of the blade arm length",
        )

    lowered_handle = None
    raised_handle = None
    with ctx.pose({arm_joint: arm_lower if arm_lower is not None else 0.0, guard_joint: 0.0}):
        lowered_handle = ctx.part_element_world_aabb(blade_arm, elem="arm_handle")
    with ctx.pose({arm_joint: arm_upper if arm_upper is not None else 0.0, guard_joint: 0.0}):
        raised_handle = ctx.part_element_world_aabb(blade_arm, elem="arm_handle")
    ctx.check(
        "blade arm opens upward around the side hinge",
        lowered_handle is not None
        and raised_handle is not None
        and raised_handle[1][2] > lowered_handle[1][2] + 0.12,
        details=f"lowered_handle={lowered_handle}, raised_handle={raised_handle}",
    )

    lowered_guard = None
    raised_guard = None
    with ctx.pose({arm_joint: 0.0, guard_joint: guard_lower if guard_lower is not None else 0.0}):
        lowered_guard = ctx.part_element_world_aabb(guard, elem="guard_panel")
    with ctx.pose({arm_joint: 0.0, guard_joint: guard_upper if guard_upper is not None else 0.0}):
        raised_guard = ctx.part_element_world_aabb(guard, elem="guard_panel")
    ctx.check(
        "finger guard swings upward from its arm hinge",
        lowered_guard is not None
        and raised_guard is not None
        and raised_guard[0][2] > lowered_guard[0][2] + 0.025
        and raised_guard[0][1] < lowered_guard[0][1] - 0.010,
        details=f"lowered_guard={lowered_guard}, raised_guard={raised_guard}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
