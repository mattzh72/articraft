from __future__ import annotations

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


_HALF_PI = 1.5707963267948966


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hub_spoke_two_branch_rig")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    hub_paint = Material("hub_paint", rgba=(0.13, 0.26, 0.42, 1.0))
    bearing_black = Material("bearing_black", rgba=(0.03, 0.035, 0.04, 1.0))
    spoke_orange = Material("spoke_orange", rgba=(0.95, 0.48, 0.12, 1.0))
    end_pad = Material("end_pad", rgba=(0.02, 0.02, 0.018, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.34, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.035, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=brushed_steel,
        name="round_mast",
    )
    mast.visual(
        Box((0.09, 0.030, 0.86)),
        origin=Origin(xyz=(0.0, -0.0475, 0.52)),
        material=dark_steel,
        name="front_rail",
    )

    hub_block_size = (0.14, 0.10, 0.105)
    boss_origin = Origin(xyz=(0.0, -0.058, 0.0), rpy=(_HALF_PI, 0.0, 0.0))
    joint_origin = Origin(xyz=(0.0, -0.073, 0.0))

    lower_hub = model.part("lower_hub")
    lower_hub.visual(
        Box(hub_block_size),
        origin=Origin(),
        material=hub_paint,
        name="block",
    )
    lower_hub.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=boss_origin,
        material=bearing_black,
        name="bearing_boss",
    )

    upper_hub = model.part("upper_hub")
    upper_hub.visual(
        Box(hub_block_size),
        origin=Origin(),
        material=hub_paint,
        name="block",
    )
    upper_hub.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=boss_origin,
        material=bearing_black,
        name="bearing_boss",
    )

    lower_spoke = model.part("lower_spoke")
    lower_spoke.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(_HALF_PI, 0.0, 0.0)),
        material=spoke_orange,
        name="pivot_collar",
    )
    lower_spoke.visual(
        Box((0.24, 0.026, 0.026)),
        origin=Origin(xyz=(0.15, -0.013, 0.0)),
        material=spoke_orange,
        name="arm_bar",
    )
    lower_spoke.visual(
        Box((0.040, 0.034, 0.034)),
        origin=Origin(xyz=(0.285, -0.013, 0.0)),
        material=end_pad,
        name="tip_pad",
    )

    upper_spoke = model.part("upper_spoke")
    upper_spoke.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(_HALF_PI, 0.0, 0.0)),
        material=spoke_orange,
        name="pivot_collar",
    )
    upper_spoke.visual(
        Box((0.24, 0.026, 0.026)),
        origin=Origin(xyz=(-0.15, -0.013, 0.0)),
        material=spoke_orange,
        name="arm_bar",
    )
    upper_spoke.visual(
        Box((0.040, 0.034, 0.034)),
        origin=Origin(xyz=(-0.285, -0.013, 0.0)),
        material=end_pad,
        name="tip_pad",
    )

    model.articulation(
        "mast_to_lower_hub",
        ArticulationType.FIXED,
        parent=mast,
        child=lower_hub,
        origin=Origin(xyz=(0.0, -0.11, 0.40)),
    )
    model.articulation(
        "mast_to_upper_hub",
        ArticulationType.FIXED,
        parent=mast,
        child=upper_hub,
        origin=Origin(xyz=(0.0, -0.11, 0.70)),
    )
    model.articulation(
        "lower_hub_axis",
        ArticulationType.REVOLUTE,
        parent=lower_hub,
        child=lower_spoke,
        origin=joint_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.80, upper=1.20),
    )
    model.articulation(
        "upper_hub_axis",
        ArticulationType.REVOLUTE,
        parent=upper_hub,
        child=upper_spoke,
        origin=joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.80, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    lower_hub = object_model.get_part("lower_hub")
    upper_hub = object_model.get_part("upper_hub")
    lower_spoke = object_model.get_part("lower_spoke")
    upper_spoke = object_model.get_part("upper_spoke")
    lower_axis = object_model.get_articulation("lower_hub_axis")
    upper_axis = object_model.get_articulation("upper_hub_axis")

    ctx.expect_contact(
        lower_hub,
        mast,
        elem_a="block",
        elem_b="front_rail",
        name="lower hub block seats on mast rail",
    )
    ctx.expect_contact(
        upper_hub,
        mast,
        elem_a="block",
        elem_b="front_rail",
        name="upper hub block seats on mast rail",
    )
    ctx.expect_contact(
        lower_hub,
        lower_spoke,
        elem_a="bearing_boss",
        elem_b="pivot_collar",
        name="lower spoke collar bears on hub boss",
    )
    ctx.expect_contact(
        upper_hub,
        upper_spoke,
        elem_a="bearing_boss",
        elem_b="pivot_collar",
        name="upper spoke collar bears on hub boss",
    )

    ctx.check(
        "two independent revolute hub axes",
        lower_axis.articulation_type == ArticulationType.REVOLUTE
        and upper_axis.articulation_type == ArticulationType.REVOLUTE
        and lower_axis.mimic is None
        and upper_axis.mimic is None
        and lower_axis.parent == "lower_hub"
        and upper_axis.parent == "upper_hub",
        details=f"lower={lower_axis}, upper={upper_axis}",
    )

    def _dims_from_aabb(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    lower_block_dims = _dims_from_aabb(lower_hub, "block")
    lower_arm_dims = _dims_from_aabb(lower_spoke, "arm_bar")
    upper_block_dims = _dims_from_aabb(upper_hub, "block")
    upper_arm_dims = _dims_from_aabb(upper_spoke, "arm_bar")
    blocks_dominant = (
        lower_block_dims is not None
        and lower_arm_dims is not None
        and upper_block_dims is not None
        and upper_arm_dims is not None
        and lower_block_dims[1] > lower_arm_dims[1] * 3.0
        and lower_block_dims[2] > lower_arm_dims[2] * 3.0
        and upper_block_dims[1] > upper_arm_dims[1] * 3.0
        and upper_block_dims[2] > upper_arm_dims[2] * 3.0
    )
    ctx.check(
        "hub blocks visibly larger than arm sections",
        blocks_dominant,
        details=(
            f"lower block={lower_block_dims}, lower arm={lower_arm_dims}; "
            f"upper block={upper_block_dims}, upper arm={upper_arm_dims}"
        ),
    )

    def _center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[2] + hi[2])

    lower_rest_z = _center_z(lower_spoke, "tip_pad")
    upper_rest_z = _center_z(upper_spoke, "tip_pad")
    with ctx.pose({lower_axis: 0.65}):
        lower_raised_z = _center_z(lower_spoke, "tip_pad")
        upper_still_z = _center_z(upper_spoke, "tip_pad")
    with ctx.pose({upper_axis: 0.65}):
        upper_raised_z = _center_z(upper_spoke, "tip_pad")
        lower_still_z = _center_z(lower_spoke, "tip_pad")

    ctx.check(
        "lower spoke rotates without driving upper spoke",
        lower_rest_z is not None
        and lower_raised_z is not None
        and upper_rest_z is not None
        and upper_still_z is not None
        and lower_raised_z > lower_rest_z + 0.08
        and abs(upper_still_z - upper_rest_z) < 0.002,
        details=(
            f"lower rest={lower_rest_z}, lower raised={lower_raised_z}, "
            f"upper rest={upper_rest_z}, upper during lower pose={upper_still_z}"
        ),
    )
    ctx.check(
        "upper spoke rotates without driving lower spoke",
        upper_rest_z is not None
        and upper_raised_z is not None
        and lower_rest_z is not None
        and lower_still_z is not None
        and upper_raised_z > upper_rest_z + 0.08
        and abs(lower_still_z - lower_rest_z) < 0.002,
        details=(
            f"upper rest={upper_rest_z}, upper raised={upper_raised_z}, "
            f"lower rest={lower_rest_z}, lower during upper pose={lower_still_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
