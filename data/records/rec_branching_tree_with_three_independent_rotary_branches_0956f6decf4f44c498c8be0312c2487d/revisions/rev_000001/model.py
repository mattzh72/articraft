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
    Sphere,
    TestContext,
    TestReport,
)


PIVOT_Y = 0.077


def _x_axis_origin(x: float, y: float, z: float) -> Origin:
    """Visual transform for a cylinder whose authored Z axis should become X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_three_branch_fixture")

    frame_mat = model.material("powder_coated_blue_gray", rgba=(0.22, 0.28, 0.31, 1.0))
    cheek_mat = model.material("dark_machined_cheeks", rgba=(0.08, 0.09, 0.09, 1.0))
    arm_mat = model.material("satin_safety_orange", rgba=(0.93, 0.45, 0.10, 1.0))
    rubber_mat = model.material("black_rubber_caps", rgba=(0.015, 0.014, 0.012, 1.0))
    pin_mat = model.material("brushed_pin_edges", rgba=(0.72, 0.72, 0.67, 1.0))

    frame = model.part("frame")

    # Narrow welded ladder spine: two uprights and unevenly spaced rungs.
    for x, name in ((-0.055, "rail_0"), (0.055, "rail_1")):
        frame.visual(
            Box((0.030, 0.025, 0.880)),
            origin=Origin(xyz=(x, 0.0, 0.440)),
            material=frame_mat,
            name=name,
        )

    for z, name in (
        (0.045, "bottom_rung"),
        (0.170, "low_rung"),
        (0.420, "middle_rung"),
        (0.760, "high_rung"),
        (0.860, "top_rung"),
    ):
        frame.visual(
            Box((0.150, 0.026, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=frame_mat,
            name=name,
        )

    # A small rear pad makes the fixture read as a wall/bench-mounted bracket
    # without becoming a broad solid panel.
    frame.visual(
        Box((0.105, 0.012, 0.220)),
        origin=Origin(xyz=(0.0, -0.018, 0.455)),
        material=frame_mat,
        name="rear_strap",
    )

    supports = (
        # name, pivot x, pivot z, reach, lower, upper
        ("low", -0.026, 0.170, 0.205, -0.55, 0.95),
        ("middle", 0.024, 0.420, 0.315, -0.75, 0.80),
        ("high", -0.006, 0.760, 0.250, -0.45, 1.05),
    )

    for prefix, px, pz, _reach, _lower, _upper in supports:
        # Backing pad overlaps the front face of the corresponding rung just
        # enough to model a welded cheek block rather than a floating lug pair.
        frame.visual(
            Box((0.108, 0.030, 0.070)),
            origin=Origin(xyz=(px, 0.0265, pz)),
            material=cheek_mat,
            name=f"{prefix}_pad",
        )
        for side, sx in (("minus", -1.0), ("plus", 1.0)):
            frame.visual(
                Box((0.012, 0.074, 0.078)),
                origin=Origin(xyz=(px + sx * 0.036, 0.075, pz)),
                material=cheek_mat,
                name=f"{prefix}_cheek_{side}",
            )
            frame.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=_x_axis_origin(px + sx * 0.047, 0.075, pz),
                material=pin_mat,
                name=f"{prefix}_boss_{side}",
            )

    for prefix, px, pz, reach, lower, upper in supports:
        arm = model.part(f"{prefix}_arm")

        arm.visual(
            Cylinder(radius=0.022, length=0.060),
            origin=_x_axis_origin(0.0, 0.0, 0.0),
            material=pin_mat,
            name="hub",
        )
        arm.visual(
            Box((0.026, reach - 0.018, 0.026)),
            origin=Origin(xyz=(0.0, (reach + 0.018) / 2.0, 0.0)),
            material=arm_mat,
            name="bar",
        )
        arm.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.0, reach, 0.0)),
            material=rubber_mat,
            name="tip",
        )

        model.articulation(
            f"frame_to_{prefix}_arm",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=arm,
            origin=Origin(xyz=(px, PIVOT_Y, pz)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    arm_names = ("low_arm", "middle_arm", "high_arm")
    joint_names = ("frame_to_low_arm", "frame_to_middle_arm", "frame_to_high_arm")

    ctx.check(
        "three independent revolute branch joints",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in joint_names),
        details=f"expected three named revolute joints: {joint_names}",
    )

    pivots = [ctx.part_world_position(object_model.get_part(name)) for name in arm_names]
    ctx.check(
        "branch pivots use uneven heights",
        all(p is not None for p in pivots)
        and (pivots[1][2] - pivots[0][2]) > 0.20
        and (pivots[2][2] - pivots[1][2]) > 0.30,
        details=f"pivot positions={pivots}",
    )

    reaches = []
    for name in arm_names:
        arm = object_model.get_part(name)
        aabb = ctx.part_world_aabb(arm)
        if aabb is not None:
            reaches.append(aabb[1][1] - pivots[arm_names.index(name)][1])
    ctx.check(
        "branch reaches are intentionally unequal",
        len(reaches) == 3 and min(abs(a - b) for i, a in enumerate(reaches) for b in reaches[i + 1 :]) > 0.030,
        details=f"rest reaches={reaches}",
    )

    for prefix, arm_name, joint_name in zip(("low", "middle", "high"), arm_names, joint_names):
        arm = object_model.get_part(arm_name)
        joint = object_model.get_articulation(joint_name)

        ctx.expect_overlap(
            arm,
            frame,
            axes="xz",
            elem_a="hub",
            elem_b=f"{prefix}_pad",
            min_overlap=0.035,
            name=f"{prefix} hub is centered on its cheek block",
        )
        ctx.expect_gap(
            arm,
            frame,
            axis="y",
            positive_elem="hub",
            negative_elem=f"{prefix}_pad",
            min_gap=0.010,
            max_gap=0.018,
            name=f"{prefix} hub sits just proud of backing pad",
        )

        rest_tip = ctx.part_element_world_aabb(arm, elem="tip")
        with ctx.pose({joint: joint.motion_limits.upper}):
            raised_tip = ctx.part_element_world_aabb(arm, elem="tip")
        ctx.check(
            f"{prefix} arm rotates upward at upper stop",
            rest_tip is not None and raised_tip is not None and raised_tip[1][2] > rest_tip[1][2] + 0.080,
            details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
        )

    return ctx.report()


object_model = build_object_model()
