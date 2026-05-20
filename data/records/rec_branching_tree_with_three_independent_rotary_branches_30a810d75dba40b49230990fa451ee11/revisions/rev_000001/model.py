from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_tooling_tree")

    mast_paint = model.material("charcoal_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    support_paint = model.material("blue_support_casting", rgba=(0.05, 0.16, 0.34, 1.0))
    branch_paint = model.material("orange_branch_steel", rgba=(0.95, 0.42, 0.08, 1.0))
    worn_edge = model.material("dark_bore_detail", rgba=(0.015, 0.016, 0.018, 1.0))
    metal_pin = model.material("brushed_pin_steel", rgba=(0.64, 0.66, 0.65, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.26, 0.26, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=mast_paint,
        name="base_plate",
    )
    mast.visual(
        Box((0.12, 0.12, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=mast_paint,
        name="boxed_mast",
    )

    # Upper +X face: a broad bearing block supporting the vertical-axis fork arm.
    mast.visual(
        Box((0.058, 0.085, 0.055)),
        origin=Origin(xyz=(0.088, 0.0, 0.680)),
        material=support_paint,
        name="upper_bearing",
    )
    mast.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.088, 0.0, 0.7105)),
        material=metal_pin,
        name="upper_bearing_race",
    )

    # Middle -Y face: a clevis with cheeks separated along X.
    mast.visual(
        Box((0.095, 0.042, 0.085)),
        origin=Origin(xyz=(0.0, -0.080, 0.500)),
        material=support_paint,
        name="middle_clevis_pad",
    )
    mast.visual(
        Box((0.018, 0.070, 0.070)),
        origin=Origin(xyz=(0.034, -0.125, 0.500)),
        material=support_paint,
        name="middle_clevis_side_pos",
    )
    mast.visual(
        Box((0.018, 0.070, 0.070)),
        origin=Origin(xyz=(-0.034, -0.125, 0.500)),
        material=support_paint,
        name="middle_clevis_side_neg",
    )

    # Lower -X face: a shorter side yoke with cheeks separated along Y.
    mast.visual(
        Box((0.042, 0.095, 0.080)),
        origin=Origin(xyz=(-0.080, 0.0, 0.310)),
        material=support_paint,
        name="lower_yoke_pad",
    )
    mast.visual(
        Box((0.070, 0.018, 0.064)),
        origin=Origin(xyz=(-0.125, 0.034, 0.310)),
        material=support_paint,
        name="lower_yoke_side_pos",
    )
    mast.visual(
        Box((0.070, 0.018, 0.064)),
        origin=Origin(xyz=(-0.125, -0.034, 0.310)),
        material=support_paint,
        name="lower_yoke_side_neg",
    )

    fork_branch = model.part("fork_branch")
    fork_branch.visual(
        Cylinder(radius=0.025, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=metal_pin,
        name="fork_hub",
    )
    fork_branch.visual(
        Box((0.110, 0.050, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, 0.027)),
        material=branch_paint,
        name="fork_stem",
    )
    fork_branch.visual(
        Box((0.170, 0.015, 0.018)),
        origin=Origin(xyz=(0.165, 0.024, 0.027)),
        material=branch_paint,
        name="fork_tine_0",
    )
    fork_branch.visual(
        Box((0.170, 0.015, 0.018)),
        origin=Origin(xyz=(0.165, -0.024, 0.027)),
        material=branch_paint,
        name="fork_tine_1",
    )
    fork_branch.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.250, 0.024, 0.027)),
        material=branch_paint,
        name="fork_tip_0",
    )
    fork_branch.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.250, -0.024, 0.027)),
        material=branch_paint,
        name="fork_tip_1",
    )

    plate_branch = model.part("plate_branch")
    plate_branch.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_pin,
        name="plate_pivot",
    )
    plate_branch.visual(
        Box((0.028, 0.280, 0.018)),
        origin=Origin(xyz=(0.0, -0.140, 0.0)),
        material=branch_paint,
        name="flat_plate",
    )
    plate_branch.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(0.0, -0.095, 0.010)),
        material=worn_edge,
        name="plate_bore_0",
    )
    plate_branch.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(0.0, -0.195, 0.010)),
        material=worn_edge,
        name="plate_bore_1",
    )

    yoke_branch = model.part("yoke_branch")
    yoke_branch.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal_pin,
        name="yoke_pivot",
    )
    yoke_branch.visual(
        Box((0.160, 0.024, 0.020)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=branch_paint,
        name="short_arm",
    )
    yoke_branch.visual(
        Box((0.020, 0.075, 0.032)),
        origin=Origin(xyz=(-0.154, 0.0, 0.006)),
        material=branch_paint,
        name="end_yoke_bridge",
    )
    yoke_branch.visual(
        Box((0.070, 0.014, 0.045)),
        origin=Origin(xyz=(-0.195, 0.027, 0.006)),
        material=branch_paint,
        name="end_yoke_cheek_0",
    )
    yoke_branch.visual(
        Box((0.070, 0.014, 0.045)),
        origin=Origin(xyz=(-0.195, -0.027, 0.006)),
        material=branch_paint,
        name="end_yoke_cheek_1",
    )

    model.articulation(
        "fork_swing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=fork_branch,
        origin=Origin(xyz=(0.090, 0.0, 0.7135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "plate_swing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=plate_branch,
        origin=Origin(xyz=(0.0, -0.125, 0.500)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "yoke_swing",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke_branch,
        origin=Origin(xyz=(-0.125, 0.0, 0.310)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.1, lower=-0.65, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    fork_branch = object_model.get_part("fork_branch")
    plate_branch = object_model.get_part("plate_branch")
    yoke_branch = object_model.get_part("yoke_branch")
    fork_swing = object_model.get_articulation("fork_swing")
    plate_swing = object_model.get_articulation("plate_swing")
    yoke_swing = object_model.get_articulation("yoke_swing")

    joints = (fork_swing, plate_swing, yoke_swing)
    ctx.check(
        "three independent revolute branches",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and len({j.child for j in joints}) == 3,
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "support axes are distinct",
        {tuple(round(v, 3) for v in j.axis) for j in joints}
        == {(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)},
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_gap(
        fork_branch,
        mast,
        axis="z",
        positive_elem="fork_hub",
        negative_elem="upper_bearing_race",
        min_gap=0.0,
        max_gap=0.001,
        name="fork hub seats on upper bearing",
    )
    ctx.expect_gap(
        mast,
        plate_branch,
        axis="x",
        positive_elem="middle_clevis_side_pos",
        negative_elem="plate_pivot",
        min_gap=0.0,
        max_gap=0.001,
        name="plate pivot is captured by middle clevis",
    )
    ctx.expect_gap(
        mast,
        yoke_branch,
        axis="y",
        positive_elem="lower_yoke_side_pos",
        negative_elem="yoke_pivot",
        min_gap=0.0,
        max_gap=0.001,
        name="yoke pivot is captured by lower yoke support",
    )

    def _max_axis(part, elem, axis_index):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else aabb[1][axis_index]

    def _min_axis(part, elem, axis_index):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else aabb[0][axis_index]

    ctx.check(
        "branch directions are visually distinct",
        (_max_axis(fork_branch, "fork_tine_0", 0) or 0.0) > 0.31
        and (_min_axis(plate_branch, "flat_plate", 1) or 0.0) < -0.38
        and (_min_axis(yoke_branch, "end_yoke_cheek_0", 0) or 0.0) < -0.34,
        details="fork projects +X, plate projects -Y, yoke branch projects -X",
    )

    rest_plate_tip_z = _max_axis(plate_branch, "flat_plate", 2)
    with ctx.pose({plate_swing: -0.45}):
        raised_plate_tip_z = _max_axis(plate_branch, "flat_plate", 2)
    ctx.check(
        "plate branch rotates upward on its own clevis axis",
        rest_plate_tip_z is not None
        and raised_plate_tip_z is not None
        and raised_plate_tip_z > rest_plate_tip_z + 0.04,
        details=f"rest_z={rest_plate_tip_z}, raised_z={raised_plate_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
