from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_inspection_tree")

    powder_black = Material("powder_black", rgba=(0.015, 0.018, 0.020, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.25, 0.27, 0.28, 1.0))
    zinc = Material("zinc_plated_pin", rgba=(0.62, 0.64, 0.62, 1.0))
    safety_orange = Material("safety_orange", rgba=(1.0, 0.38, 0.08, 1.0))
    muted_yellow = Material("muted_yellow", rgba=(0.94, 0.72, 0.20, 1.0))
    glass = Material("smoked_glass", rgba=(0.02, 0.05, 0.07, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.30, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=powder_black,
        name="base_plate",
    )
    spine.visual(
        Box((0.08, 0.07, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=powder_black,
        name="boxed_spine",
    )
    spine.visual(
        Box((0.11, 0.09, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.6725)),
        material=dark_steel,
        name="top_cap",
    )

    # Upper, longer branch: mounted high on the -X side.
    upper_pivot = (-0.096, 0.0, 0.51)
    spine.visual(
        Box((0.020, 0.130, 0.115)),
        origin=Origin(xyz=(-0.049, 0.0, upper_pivot[2])),
        material=dark_steel,
        name="upper_backplate",
    )
    spine.visual(
        Box((0.076, 0.018, 0.082)),
        origin=Origin(xyz=(-0.077, 0.036, upper_pivot[2])),
        material=dark_steel,
        name="upper_lug_front",
    )
    spine.visual(
        Box((0.076, 0.018, 0.082)),
        origin=Origin(xyz=(-0.077, -0.036, upper_pivot[2])),
        material=dark_steel,
        name="upper_lug_rear",
    )

    # Lower, shorter branch: mounted lower on the +X side.
    lower_pivot = (0.087, 0.0, 0.30)
    spine.visual(
        Box((0.020, 0.110, 0.095)),
        origin=Origin(xyz=(0.049, 0.0, lower_pivot[2])),
        material=dark_steel,
        name="lower_backplate",
    )
    spine.visual(
        Box((0.066, 0.016, 0.070)),
        origin=Origin(xyz=(0.072, 0.033, lower_pivot[2])),
        material=dark_steel,
        name="lower_lug_front",
    )
    spine.visual(
        Box((0.066, 0.016, 0.070)),
        origin=Origin(xyz=(0.072, -0.033, lower_pivot[2])),
        material=dark_steel,
        name="lower_lug_rear",
    )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="upper_hub",
    )
    upper_branch.visual(
        Cylinder(radius=0.008, length=0.105),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="upper_axle",
    )
    upper_branch.visual(
        Box((0.360, 0.018, 0.024)),
        origin=Origin(xyz=(-0.198, 0.0, 0.0)),
        material=safety_orange,
        name="upper_arm",
    )
    upper_branch.visual(
        Box((0.055, 0.042, 0.040)),
        origin=Origin(xyz=(-0.4055, 0.0, 0.0)),
        material=safety_orange,
        name="upper_sensor_body",
    )
    upper_branch.visual(
        Box((0.004, 0.032, 0.026)),
        origin=Origin(xyz=(-0.435, 0.0, 0.0)),
        material=glass,
        name="upper_sensor_lens",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        Cylinder(radius=0.023, length=0.040),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_hub",
    )
    lower_branch.visual(
        Cylinder(radius=0.007, length=0.096),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="lower_axle",
    )
    lower_branch.visual(
        Box((0.255, 0.018, 0.022)),
        origin=Origin(xyz=(0.1435, 0.0, 0.0)),
        material=muted_yellow,
        name="lower_arm",
    )
    lower_branch.visual(
        Box((0.046, 0.038, 0.034)),
        origin=Origin(xyz=(0.294, 0.0, 0.0)),
        material=muted_yellow,
        name="lower_sensor_body",
    )
    lower_branch.visual(
        Box((0.004, 0.028, 0.022)),
        origin=Origin(xyz=(0.319, 0.0, 0.0)),
        material=glass,
        name="lower_sensor_lens",
    )

    model.articulation(
        "spine_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=upper_pivot),
        # The long branch extends along local -X; +Y raises its inspection head.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.70, upper=1.05),
    )
    model.articulation(
        "spine_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=lower_pivot),
        # The shorter branch extends along local +X; -Y gives the same raise-positive convention.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.55, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_joint = object_model.get_articulation("spine_to_upper_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_branch")

    # The visible axles are intentionally captured through simplified solid
    # bracket cheeks.  The overlap is local to the axle/cheek interface and
    # represents a pin running through the supported pivot axis.
    for branch, axle, lug in (
        (upper_branch, "upper_axle", "upper_lug_front"),
        (upper_branch, "upper_axle", "upper_lug_rear"),
        (lower_branch, "lower_axle", "lower_lug_front"),
        (lower_branch, "lower_axle", "lower_lug_rear"),
    ):
        ctx.allow_overlap(
            spine,
            branch,
            elem_a=lug,
            elem_b=axle,
            reason="A rotating branch axle is intentionally captured through the bracket cheek.",
        )
        ctx.expect_within(
            branch,
            spine,
            axes="xz",
            inner_elem=axle,
            outer_elem=lug,
            margin=0.001,
            name=f"{axle} stays centered in {lug}",
        )
        ctx.expect_overlap(
            branch,
            spine,
            axes="y",
            elem_a=axle,
            elem_b=lug,
            min_overlap=0.010,
            name=f"{axle} passes through {lug}",
        )

    ctx.expect_gap(
        spine,
        upper_branch,
        axis="y",
        positive_elem="upper_lug_front",
        negative_elem="upper_hub",
        min_gap=0.003,
        max_gap=0.008,
        name="upper hub clears front lug",
    )
    ctx.expect_gap(
        upper_branch,
        spine,
        axis="y",
        positive_elem="upper_hub",
        negative_elem="upper_lug_rear",
        min_gap=0.003,
        max_gap=0.008,
        name="upper hub clears rear lug",
    )
    ctx.expect_gap(
        spine,
        lower_branch,
        axis="y",
        positive_elem="lower_lug_front",
        negative_elem="lower_hub",
        min_gap=0.003,
        max_gap=0.008,
        name="lower hub clears front lug",
    )
    ctx.expect_gap(
        lower_branch,
        spine,
        axis="y",
        positive_elem="lower_hub",
        negative_elem="lower_lug_rear",
        min_gap=0.003,
        max_gap=0.008,
        name="lower hub clears rear lug",
    )

    upper_pivot = ctx.part_world_position(upper_branch)
    lower_pivot = ctx.part_world_position(lower_branch)
    ctx.check(
        "branch pivots use varied mounting heights",
        upper_pivot is not None
        and lower_pivot is not None
        and upper_pivot[2] > lower_pivot[2] + 0.15,
        details=f"upper={upper_pivot}, lower={lower_pivot}",
    )

    upper_aabb = ctx.part_world_aabb(upper_branch)
    lower_aabb = ctx.part_world_aabb(lower_branch)
    upper_span = upper_aabb[1][0] - upper_aabb[0][0] if upper_aabb else 0.0
    lower_span = lower_aabb[1][0] - lower_aabb[0][0] if lower_aabb else 0.0
    ctx.check(
        "branch lengths are intentionally unequal",
        upper_span > lower_span + 0.08,
        details=f"upper_x_span={upper_span:.3f}, lower_x_span={lower_span:.3f}",
    )

    def aabb_center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0 if aabb else None

    upper_rest_z = aabb_center_z(
        ctx.part_element_world_aabb(upper_branch, elem="upper_sensor_lens")
    )
    lower_rest_z = aabb_center_z(
        ctx.part_element_world_aabb(lower_branch, elem="lower_sensor_lens")
    )
    with ctx.pose({upper_joint: 0.75}):
        upper_raised_z = aabb_center_z(
            ctx.part_element_world_aabb(upper_branch, elem="upper_sensor_lens")
        )
        lower_during_upper_z = aabb_center_z(
            ctx.part_element_world_aabb(lower_branch, elem="lower_sensor_lens")
        )
    with ctx.pose({lower_joint: 0.70}):
        lower_raised_z = aabb_center_z(
            ctx.part_element_world_aabb(lower_branch, elem="lower_sensor_lens")
        )
        upper_during_lower_z = aabb_center_z(
            ctx.part_element_world_aabb(upper_branch, elem="upper_sensor_lens")
        )

    ctx.check(
        "upper revolute branch raises its inspection head",
        upper_rest_z is not None
        and upper_raised_z is not None
        and upper_raised_z > upper_rest_z + 0.12,
        details=f"rest_z={upper_rest_z}, raised_z={upper_raised_z}",
    )
    ctx.check(
        "lower revolute branch raises its inspection head",
        lower_rest_z is not None
        and lower_raised_z is not None
        and lower_raised_z > lower_rest_z + 0.08,
        details=f"rest_z={lower_rest_z}, raised_z={lower_raised_z}",
    )
    ctx.check(
        "branch joints move independently",
        upper_rest_z is not None
        and lower_rest_z is not None
        and upper_during_lower_z is not None
        and lower_during_upper_z is not None
        and abs(upper_during_lower_z - upper_rest_z) < 0.001
        and abs(lower_during_upper_z - lower_rest_z) < 0.001,
        details=(
            f"upper_rest={upper_rest_z}, upper_during_lower={upper_during_lower_z}, "
            f"lower_rest={lower_rest_z}, lower_during_upper={lower_during_upper_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
