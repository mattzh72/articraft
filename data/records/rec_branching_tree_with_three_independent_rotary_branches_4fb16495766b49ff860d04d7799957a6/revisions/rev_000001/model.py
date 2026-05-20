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


CYLINDER_ALONG_Y = Origin(rpy=(-pi / 2.0, 0.0, 0.0))


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_three_branch_fixture")

    powder_coat = _mat(model, "charcoal_powder_coat", (0.10, 0.12, 0.13, 1.0))
    dark_steel = _mat(model, "dark_burnished_steel", (0.05, 0.055, 0.06, 1.0))
    yellow = _mat(model, "safety_yellow", (0.95, 0.70, 0.08, 1.0))
    rubber = _mat(model, "black_rubber", (0.015, 0.015, 0.012, 1.0))
    tool_gray = _mat(model, "tool_gray", (0.28, 0.30, 0.31, 1.0))
    bolt = _mat(model, "oiled_bolt_heads", (0.025, 0.025, 0.023, 1.0))

    fixture = model.part("fixture")
    fixture.visual(
        Box((0.54, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=powder_coat,
        name="base_plinth",
    )
    fixture.visual(
        Box((0.19, 0.16, 0.86)),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=powder_coat,
        name="boxed_mast",
    )
    fixture.visual(
        Box((0.24, 0.20, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=powder_coat,
        name="top_cap",
    )

    branch_levels = {
        "lower": 0.265,
        "middle": 0.535,
        "upper": 0.805,
    }

    for level_name, z in branch_levels.items():
        fixture.visual(
            Box((0.18, 0.042, 0.145)),
            origin=Origin(xyz=(0.0, 0.101, z)),
            material=dark_steel,
            name=f"{level_name}_boss_pad",
        )
        fixture.visual(
            Cylinder(radius=0.057, length=0.090),
            origin=Origin(xyz=(0.0, 0.125, z), rpy=CYLINDER_ALONG_Y.rpy),
            material=dark_steel,
            name=f"{level_name}_boss",
        )
        fixture.visual(
            Box((0.026, 0.034, 0.190)),
            origin=Origin(xyz=(-0.085, 0.098, z)),
            material=dark_steel,
            name=f"{level_name}_side_rib_0",
        )
        fixture.visual(
            Box((0.026, 0.034, 0.190)),
            origin=Origin(xyz=(0.085, 0.098, z)),
            material=dark_steel,
            name=f"{level_name}_side_rib_1",
        )
        for ix, x in enumerate((-0.064, 0.064)):
            for iz, dz in enumerate((-0.046, 0.046)):
                fixture.visual(
                    Cylinder(radius=0.010, length=0.016),
                    origin=Origin(xyz=(x, 0.119, z + dz), rpy=CYLINDER_ALONG_Y.rpy),
                    material=bolt,
                    name=f"{level_name}_bolt_{ix}_{iz}",
                )

    def add_branch(name: str, stripe_offset: float = 0.0):
        branch = model.part(name)
        branch.visual(
            Cylinder(radius=0.047, length=0.050),
            origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=CYLINDER_ALONG_Y.rpy),
            material=dark_steel,
            name="hub",
        )
        branch.visual(
            Cylinder(radius=0.060, length=0.012),
            origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=CYLINDER_ALONG_Y.rpy),
            material=bolt,
            name="retainer_washer",
        )
        branch.visual(
            Box((0.245, 0.036, 0.044)),
            origin=Origin(xyz=(0.145, 0.027, 0.0)),
            material=yellow,
            name="short_arm",
        )
        branch.visual(
            Box((0.070, 0.066, 0.078)),
            origin=Origin(xyz=(0.292, 0.029, 0.0)),
            material=tool_gray,
            name="wrist_block",
        )
        branch.visual(
            Cylinder(radius=0.025, length=0.056),
            origin=Origin(xyz=(0.316, 0.076, 0.0), rpy=CYLINDER_ALONG_Y.rpy),
            material=rubber,
            name="tool_nose",
        )
        branch.visual(
            Box((0.056, 0.050, 0.018)),
            origin=Origin(xyz=(0.348, 0.031, 0.031)),
            material=rubber,
            name="upper_jaw",
        )
        branch.visual(
            Box((0.056, 0.050, 0.018)),
            origin=Origin(xyz=(0.348, 0.031, -0.031)),
            material=rubber,
            name="lower_jaw",
        )
        branch.visual(
            Box((0.030, 0.038, 0.049)),
            origin=Origin(xyz=(0.226 + stripe_offset, 0.027, 0.0)),
            material=dark_steel,
            name="reinforcement_band",
        )
        return branch

    lower_branch = add_branch("lower_branch", stripe_offset=-0.010)
    middle_branch = add_branch("middle_branch")
    upper_branch = add_branch("upper_branch", stripe_offset=0.010)

    for branch_name, branch, z, rest_angle in (
        ("lower", lower_branch, branch_levels["lower"], -0.22),
        ("middle", middle_branch, branch_levels["middle"], 0.0),
        ("upper", upper_branch, branch_levels["upper"], 0.22),
    ):
        model.articulation(
            f"fixture_to_{branch_name}_branch",
            ArticulationType.REVOLUTE,
            parent=fixture,
            child=branch,
            # The support axis is grounded in the fixed boss and projects out
            # of the front face of the mast.  The child hub sits immediately
            # outboard of this face.
            origin=Origin(xyz=(0.0, 0.170, z), rpy=(0.0, rest_angle, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.65, upper=0.65),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixture = object_model.get_part("fixture")
    branch_specs = (
        ("lower", object_model.get_part("lower_branch"), object_model.get_articulation("fixture_to_lower_branch")),
        ("middle", object_model.get_part("middle_branch"), object_model.get_articulation("fixture_to_middle_branch")),
        ("upper", object_model.get_part("upper_branch"), object_model.get_articulation("fixture_to_upper_branch")),
    )

    def _aabb_extent(aabb, axis_index: int) -> float:
        return float(aabb[1][axis_index] - aabb[0][axis_index])

    def _aabb_center(aabb) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple(float((aabb[0][i] + aabb[1][i]) * 0.5) for i in range(3))

    joints_grounded_on_fixture = all(
        str(joint.articulation_type).lower().endswith("revolute")
        and joint.parent == "fixture"
        and joint.child == branch.name
        and joint.axis == (0.0, 1.0, 0.0)
        and joint.mimic is None
        for _, branch, joint in branch_specs
    )
    support_z_levels = [round(float(joint.origin.xyz[2]), 3) for _, _, joint in branch_specs]
    ctx.check(
        "three independent grounded revolute branches",
        joints_grounded_on_fixture and len(set(support_z_levels)) == 3,
        details=f"joint_specs={[(joint.name, joint.articulation_type, joint.parent, joint.child, joint.axis, joint.mimic) for _, _, joint in branch_specs]}",
    )

    mast_aabb = ctx.part_element_world_aabb(fixture, elem="boxed_mast")
    branch_aabbs = [ctx.part_world_aabb(branch) for _, branch, _ in branch_specs]
    if mast_aabb is not None and all(aabb is not None for aabb in branch_aabbs):
        mast_height = _aabb_extent(mast_aabb, 2)
        max_branch_span = max(_aabb_extent(aabb, 0) for aabb in branch_aabbs if aabb is not None)
        ctx.check(
            "fixed mast dominates the short branches",
            mast_height > 0.80 and mast_height > 1.8 * max_branch_span,
            details=f"mast_height={mast_height:.3f}, max_branch_span={max_branch_span:.3f}",
        )

    for level_name, branch, joint in branch_specs:
        ctx.expect_gap(
            branch,
            fixture,
            axis="y",
            positive_elem="hub",
            negative_elem=f"{level_name}_boss",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{level_name} hub seats on boss face",
        )
        ctx.expect_overlap(
            branch,
            fixture,
            axes="xz",
            elem_a="hub",
            elem_b=f"{level_name}_boss",
            min_overlap=0.080,
            name=f"{level_name} hub shares boss support axis",
        )

        rest_tool = _aabb_center(ctx.part_element_world_aabb(branch, elem="tool_nose"))
        with ctx.pose({joint: 0.55}):
            rotated_tool = _aabb_center(ctx.part_element_world_aabb(branch, elem="tool_nose"))
        ctx.check(
            f"{level_name} branch visibly rotates about its boss",
            rest_tool is not None
            and rotated_tool is not None
            and abs(rotated_tool[1] - rest_tool[1]) < 0.002
            and abs(rotated_tool[2] - rest_tool[2]) > 0.060,
            details=f"rest_tool={rest_tool}, rotated_tool={rotated_tool}",
        )

    middle_tool_rest = _aabb_center(ctx.part_element_world_aabb(object_model.get_part("middle_branch"), elem="tool_nose"))
    with ctx.pose({object_model.get_articulation("fixture_to_lower_branch"): 0.50}):
        middle_tool_after_lower_motion = _aabb_center(
            ctx.part_element_world_aabb(object_model.get_part("middle_branch"), elem="tool_nose")
        )
    ctx.check(
        "moving one branch does not drive the neighboring branch",
        middle_tool_rest is not None
        and middle_tool_after_lower_motion is not None
        and all(abs(middle_tool_after_lower_motion[i] - middle_tool_rest[i]) < 1e-6 for i in range(3)),
        details=f"middle_rest={middle_tool_rest}, middle_after_lower_motion={middle_tool_after_lower_motion}",
    )

    return ctx.report()


object_model = build_object_model()
