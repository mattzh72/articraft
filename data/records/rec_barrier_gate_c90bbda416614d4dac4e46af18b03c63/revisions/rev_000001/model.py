from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vehicle_swing_gate")

    powder_coat = model.material("powder_coat", rgba=(0.24, 0.26, 0.27, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.67, 1.0))

    hinge_axis_x = 0.125
    lower_hinge_z = 0.35
    upper_hinge_z = 1.65
    hinge_spacing = upper_hinge_z - lower_hinge_z

    def sleeve_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
        half = length * 0.5
        outer_profile = [(outer_radius, -half), (outer_radius, half)]
        inner_profile = [(inner_radius, -half), (inner_radius, half)]
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=40,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    knuckle_mesh = sleeve_mesh(
        "gate_knuckle_sleeve",
        inner_radius=0.022,
        outer_radius=0.032,
        length=0.12,
    )

    post_assembly = model.part("post_assembly")
    post_assembly.visual(
        Box((0.55, 0.55, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
        material=concrete,
        name="footing_block",
    )
    post_assembly.visual(
        Box((0.16, 0.16, 2.40)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=powder_coat,
        name="post_column",
    )
    post_assembly.visual(
        Box((0.18, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 2.415)),
        material=powder_coat,
        name="post_cap",
    )

    for prefix, z_pos in (("lower", lower_hinge_z), ("upper", upper_hinge_z)):
        post_assembly.visual(
            Box((0.035, 0.14, 0.22)),
            origin=Origin(xyz=(0.0625, 0.0, z_pos)),
            material=hinge_steel,
            name=f"{prefix}_hinge_mount_plate",
        )
        post_assembly.visual(
            Box((0.03, 0.10, 0.10)),
            origin=Origin(xyz=(0.080, 0.0, z_pos + 0.095)),
            material=hinge_steel,
            name=f"{prefix}_hinge_mount_cap",
        )
        post_assembly.visual(
            Box((0.03, 0.10, 0.10)),
            origin=Origin(xyz=(0.080, 0.0, z_pos - 0.095)),
            material=hinge_steel,
            name=f"{prefix}_hinge_mount_base",
        )

    post_assembly.inertial = Inertial.from_geometry(
        Box((0.55, 0.55, 3.20)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )

    hinge_spine = model.part("hinge_spine")
    hinge_spine.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="lower_pintle_pin",
    )
    hinge_spine.visual(
        Box((0.028, 0.014, 1.29)),
        origin=Origin(xyz=(-0.010, -0.060, 0.645)),
        material=hinge_steel,
        name="rear_hinge_strap",
    )
    hinge_spine.visual(
        Box((0.028, 0.09, 0.05)),
        origin=Origin(xyz=(-0.010, -0.045, 0.115)),
        material=hinge_steel,
        name="lower_pin_head",
    )
    hinge_spine.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, hinge_spacing)),
        material=hinge_steel,
        name="upper_pintle_pin",
    )
    hinge_spine.visual(
        Box((0.028, 0.09, 0.05)),
        origin=Origin(xyz=(-0.010, -0.045, hinge_spacing - 0.115)),
        material=hinge_steel,
        name="upper_pin_head",
    )
    hinge_spine.inertial = Inertial.from_geometry(
        Box((0.05, 0.10, hinge_spacing)),
        mass=4.0,
        origin=Origin(xyz=(-0.010, -0.030, hinge_spacing * 0.5)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.10, 0.06, 1.55)),
        origin=Origin(xyz=(0.07, 0.0, -0.725)),
        material=powder_coat,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((0.10, 0.06, 1.55)),
        origin=Origin(xyz=(3.55, 0.0, -0.725)),
        material=powder_coat,
        name="latch_stile",
    )
    gate_leaf.visual(
        Box((3.56, 0.06, 0.08)),
        origin=Origin(xyz=(1.81, 0.0, 0.01)),
        material=powder_coat,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((3.56, 0.06, 0.08)),
        origin=Origin(xyz=(1.81, 0.0, -1.46)),
        material=powder_coat,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((3.52, 0.05, 0.06)),
        origin=Origin(xyz=(1.81, 0.0, -0.73)),
        material=powder_coat,
        name="mid_rail",
    )

    brace_angle = atan2(1.47, 3.40)
    gate_leaf.visual(
        Box((3.72, 0.04, 0.05)),
        origin=Origin(xyz=(1.81, 0.0, -0.715), rpy=(0.0, -brace_angle, 0.0)),
        material=powder_coat,
        name="diagonal_brace",
    )

    for index, x_pos in enumerate((0.62, 1.18, 1.74, 2.30, 2.86), start=1):
        gate_leaf.visual(
            Box((0.045, 0.03, 1.35)),
            origin=Origin(xyz=(x_pos, 0.0, -0.725)),
            material=powder_coat,
            name=f"infill_tube_{index}",
        )

    gate_leaf.visual(
        Box((0.06, 0.05, 0.12)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=hinge_steel,
        name="upper_knuckle_strap",
    )
    gate_leaf.visual(
        Box((0.06, 0.05, 0.12)),
        origin=Origin(xyz=(0.045, 0.0, -1.30)),
        material=hinge_steel,
        name="lower_knuckle_strap",
    )
    gate_leaf.visual(
        knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="upper_knuckle",
    )
    gate_leaf.visual(
        knuckle_mesh,
        origin=Origin(xyz=(0.0, 0.0, -1.30)),
        material=hinge_steel,
        name="lower_knuckle",
    )
    gate_leaf.visual(
        Box((0.04, 0.08, 0.18)),
        origin=Origin(xyz=(3.57, 0.0, -0.73)),
        material=hinge_steel,
        name="latch_plate",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((3.60, 0.10, 1.55)),
        mass=145.0,
        origin=Origin(xyz=(1.80, 0.0, -0.725)),
    )

    model.articulation(
        "lower_pintle_joint",
        ArticulationType.REVOLUTE,
        parent=post_assembly,
        child=hinge_spine,
        origin=Origin(xyz=(hinge_axis_x, 0.0, lower_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.65),
    )
    model.articulation(
        "upper_pintle_joint",
        ArticulationType.REVOLUTE,
        parent=hinge_spine,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, hinge_spacing)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.8, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post_assembly = object_model.get_part("post_assembly")
    hinge_spine = object_model.get_part("hinge_spine")
    gate_leaf = object_model.get_part("gate_leaf")
    lower_joint = object_model.get_articulation("lower_pintle_joint")
    upper_joint = object_model.get_articulation("upper_pintle_joint")

    axes_ok = lower_joint.axis == (0.0, 0.0, 1.0) and upper_joint.axis == (0.0, 0.0, 1.0)
    aligned_ok = (
        abs(lower_joint.origin.xyz[0] - 0.125) < 1e-9
        and abs(lower_joint.origin.xyz[1]) < 1e-9
        and abs(lower_joint.origin.xyz[2] - 0.35) < 1e-9
        and abs(upper_joint.origin.xyz[0]) < 1e-9
        and abs(upper_joint.origin.xyz[1]) < 1e-9
        and abs(upper_joint.origin.xyz[2] - 1.30) < 1e-9
    )
    ctx.check(
        "both pintle joints are vertically aligned",
        axes_ok and aligned_ok,
        details=(
            f"lower_axis={lower_joint.axis}, upper_axis={upper_joint.axis}, "
            f"lower_origin={lower_joint.origin.xyz}, upper_origin={upper_joint.origin.xyz}"
        ),
    )

    with ctx.pose({lower_joint: 0.0, upper_joint: 0.0}):
        ctx.expect_gap(
            gate_leaf,
            post_assembly,
            axis="x",
            positive_elem="hinge_stile",
            negative_elem="post_column",
            min_gap=0.05,
            max_gap=0.09,
            name="closed gate leaf clears the support post",
        )
        ctx.expect_gap(
            gate_leaf,
            post_assembly,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="footing_block",
            min_gap=0.10,
            max_gap=0.20,
            name="bottom rail stays above the footing",
        )
        ctx.expect_overlap(
            gate_leaf,
            post_assembly,
            axes="z",
            elem_a="hinge_stile",
            elem_b="post_column",
            min_overlap=1.20,
            name="hinge stile overlaps the post heightwise for both hinges",
        )

        closed_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")

    with ctx.pose({lower_joint: 0.35, upper_joint: 0.35}):
        open_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")

    if closed_latch_aabb is not None and open_latch_aabb is not None:
        closed_center_y = 0.5 * (closed_latch_aabb[0][1] + closed_latch_aabb[1][1])
        open_center_y = 0.5 * (open_latch_aabb[0][1] + open_latch_aabb[1][1])
        ctx.check(
            "paired pintle joints swing the latch side outward",
            open_center_y > closed_center_y + 1.8,
            details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
        )
    else:
        ctx.fail(
            "paired pintle joints swing the latch side outward",
            details=f"closed_latch_aabb={closed_latch_aabb}, open_latch_aabb={open_latch_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
