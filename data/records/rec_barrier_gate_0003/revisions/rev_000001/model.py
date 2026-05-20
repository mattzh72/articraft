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
    model = ArticulatedObject(name="railway_crossing_gate")

    concrete = model.material("concrete", rgba=(0.72, 0.73, 0.74, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.70, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    arm_white = model.material("arm_white", rgba=(0.95, 0.96, 0.95, 1.0))
    stripe_red = model.material("stripe_red", rgba=(0.76, 0.09, 0.09, 1.0))

    footing = model.part("footing")
    footing.visual(
        Box((0.92, 0.76, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=concrete,
        name="footing_block",
    )
    footing.visual(
        Box((0.40, 0.34, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=concrete,
        name="pedestal_top",
    )
    footing.inertial = Inertial.from_geometry(
        Box((0.92, 0.76, 0.34)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    support = model.part("support")
    support.visual(
        Box((0.18, 0.18, 3.42)),
        origin=Origin(xyz=(0.0, 0.0, 1.71)),
        material=galvanized,
        name="post",
    )
    support.visual(
        Box((0.24, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=cast_iron,
        name="base_collar",
    )
    support.visual(
        Box((0.10, 0.24, 0.84)),
        origin=Origin(xyz=(0.05, 0.0, 3.04)),
        material=cast_iron,
        name="bracket_backplate",
    )
    support.visual(
        Box((0.22, 0.16, 0.34)),
        origin=Origin(xyz=(0.13, 0.0, 3.04), rpy=(0.0, 0.45, 0.0)),
        material=cast_iron,
        name="bracket_gusset",
    )
    support.visual(
        Box((0.50, 0.12, 0.12)),
        origin=Origin(xyz=(0.35, 0.0, 3.20)),
        material=cast_iron,
        name="bracket_arm",
    )
    support.visual(
        Box((0.26, 0.020, 0.26)),
        origin=Origin(xyz=(0.70, -0.058, 3.20)),
        material=cast_iron,
        name="left_cheek",
    )
    support.visual(
        Box((0.26, 0.020, 0.26)),
        origin=Origin(xyz=(0.70, 0.058, 3.20)),
        material=cast_iron,
        name="right_cheek",
    )
    support.visual(
        Box((0.22, 0.22, 0.10)),
        origin=Origin(xyz=(0.06, 0.0, 3.47)),
        material=cast_iron,
        name="upper_cap",
    )
    support.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.74, -0.068, 3.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="left_pin_cap",
    )
    support.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.74, 0.068, 3.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="right_pin_cap",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.88, 0.30, 3.56)),
        mass=180.0,
        origin=Origin(xyz=(0.22, 0.0, 1.78)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.038, length=0.096),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.16, 0.082, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=cast_iron,
        name="pivot_head",
    )
    arm.visual(
        Box((0.34, 0.072, 0.092)),
        origin=Origin(xyz=(0.27, 0.0, 0.0)),
        material=arm_white,
        name="boom_neck",
    )
    arm.visual(
        Box((1.02, 0.102, 0.10)),
        origin=Origin(xyz=(0.95, 0.0, 0.0)),
        material=arm_white,
        name="boom_root",
    )
    arm.visual(
        Box((1.20, 0.103, 0.090)),
        origin=Origin(xyz=(2.06, 0.0, 0.0)),
        material=arm_white,
        name="boom_mid_a",
    )
    arm.visual(
        Box((1.20, 0.090, 0.082)),
        origin=Origin(xyz=(3.26, 0.0, 0.0)),
        material=arm_white,
        name="boom_mid_b",
    )
    arm.visual(
        Box((1.02, 0.076, 0.072)),
        origin=Origin(xyz=(4.39, 0.0, 0.0)),
        material=arm_white,
        name="boom_outer",
    )
    arm.visual(
        Box((0.76, 0.062, 0.060)),
        origin=Origin(xyz=(5.28, 0.0, 0.0)),
        material=arm_white,
        name="boom_tip",
    )

    stripe_specs = [
        ("stripe_1", 0.86, 0.12, 0.16, 0.104),
        ("stripe_2", 1.56, 0.12, 0.16, 0.102),
        ("stripe_3", 2.34, 0.11, 0.15, 0.094),
        ("stripe_4", 3.14, 0.10, 0.140, 0.086),
        ("stripe_5", 3.90, 0.10, 0.128, 0.080),
        ("stripe_6", 4.54, 0.09, 0.114, 0.074),
    ]
    for stripe_name, x_pos, sx, sy, sz in stripe_specs:
        arm.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, 0.0, 0.58)),
            material=stripe_red,
            name=stripe_name,
        )
    arm.visual(
        Box((0.16, 0.090, 0.070)),
        origin=Origin(xyz=(5.30, 0.0, 0.0)),
        material=stripe_red,
        name="tip_band",
    )

    arm.visual(
        Box((0.12, 0.06, 0.24)),
        origin=Origin(xyz=(-0.02, 0.0, -0.12)),
        material=cast_iron,
        name="tail_drop",
    )
    arm.visual(
        Box((0.18, 0.32, 0.08)),
        origin=Origin(xyz=(-0.14, 0.0, -0.24)),
        material=cast_iron,
        name="tail_yoke",
    )
    arm.visual(
        Box((0.78, 0.08, 0.08)),
        origin=Origin(xyz=(-0.59, 0.16, -0.26)),
        material=cast_iron,
        name="right_tail_beam",
    )
    arm.visual(
        Box((0.78, 0.08, 0.08)),
        origin=Origin(xyz=(-0.59, -0.16, -0.26)),
        material=cast_iron,
        name="left_tail_beam",
    )
    arm.visual(
        Box((0.30, 0.12, 0.28)),
        origin=Origin(xyz=(-1.00, 0.18, -0.36)),
        material=cast_iron,
        name="right_counterweight",
    )
    arm.visual(
        Box((0.30, 0.12, 0.28)),
        origin=Origin(xyz=(-1.00, -0.18, -0.36)),
        material=cast_iron,
        name="left_counterweight",
    )
    arm.inertial = Inertial.from_geometry(
        Box((6.10, 0.44, 0.86)),
        mass=82.0,
        origin=Origin(xyz=(2.30, 0.0, -0.04)),
    )

    model.articulation(
        "footing_to_support",
        ArticulationType.FIXED,
        parent=footing,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )
    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.74, 0.0, 3.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=0.0,
            upper=1.20,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    footing = object_model.get_part("footing")
    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    gate_pivot = object_model.get_articulation("support_to_arm")
    pedestal_top = footing.get_visual("pedestal_top")
    post = support.get_visual("post")
    bracket_backplate = support.get_visual("bracket_backplate")
    bracket_arm = support.get_visual("bracket_arm")
    upper_cap = support.get_visual("upper_cap")
    left_cheek = support.get_visual("left_cheek")
    right_cheek = support.get_visual("right_cheek")
    pivot_hub = arm.get_visual("pivot_hub")
    tail_yoke = arm.get_visual("tail_yoke")
    right_tail_beam = arm.get_visual("right_tail_beam")
    boom_root = arm.get_visual("boom_root")
    boom_mid_a = arm.get_visual("boom_mid_a")
    boom_outer = arm.get_visual("boom_outer")
    tip_band = arm.get_visual("tip_band")
    stripe_3 = arm.get_visual("stripe_3")
    stripe_6 = arm.get_visual("stripe_6")
    right_counterweight = arm.get_visual("right_counterweight")
    left_counterweight = arm.get_visual("left_counterweight")
    boom_tip = arm.get_visual("boom_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        support,
        footing,
        axis="z",
        positive_elem=post,
        negative_elem=pedestal_top,
        max_gap=0.001,
        max_penetration=0.001,
    )
    ctx.expect_within(support, footing, axes="xy", inner_elem=post, outer_elem=pedestal_top)
    ctx.expect_overlap(
        support,
        support,
        axes="xy",
        elem_a=bracket_backplate,
        elem_b=post,
        min_overlap=0.08,
    )
    ctx.expect_gap(
        support,
        support,
        axis="z",
        positive_elem=upper_cap,
        negative_elem=bracket_arm,
        min_gap=0.12,
        max_gap=0.22,
    )
    ctx.expect_overlap(
        arm,
        support,
        axes="xz",
        elem_a=pivot_hub,
        elem_b=left_cheek,
        min_overlap=0.01,
    )
    ctx.expect_overlap(
        arm,
        support,
        axes="xz",
        elem_a=pivot_hub,
        elem_b=right_cheek,
        min_overlap=0.05,
    )
    ctx.expect_gap(
        arm,
        support,
        axis="y",
        positive_elem=pivot_hub,
        negative_elem=left_cheek,
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        support,
        arm,
        axis="y",
        positive_elem=right_cheek,
        negative_elem=pivot_hub,
        max_gap=0.006,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        arm,
        support,
        axis="x",
        positive_elem=boom_root,
        negative_elem=post,
        min_gap=0.75,
    )
    ctx.expect_gap(
        arm,
        support,
        axis="x",
        positive_elem=tip_band,
        negative_elem=post,
        min_gap=5.0,
    )
    ctx.expect_gap(
        support,
        arm,
        axis="x",
        positive_elem=post,
        negative_elem=right_counterweight,
        min_gap=0.015,
        max_gap=0.08,
    )
    ctx.expect_gap(
        arm,
        support,
        axis="y",
        positive_elem=right_counterweight,
        negative_elem=post,
        min_gap=0.01,
    )
    ctx.expect_gap(
        support,
        arm,
        axis="y",
        positive_elem=post,
        negative_elem=left_counterweight,
        min_gap=0.01,
    )
    ctx.expect_overlap(
        arm,
        arm,
        axes="yz",
        elem_a=tail_yoke,
        elem_b=right_tail_beam,
        min_overlap=0.04,
    )
    ctx.expect_overlap(
        arm,
        arm,
        axes="yz",
        elem_a=right_tail_beam,
        elem_b=right_counterweight,
        min_overlap=0.08,
    )
    ctx.expect_within(
        arm,
        arm,
        axes="yz",
        inner_elem=boom_tip,
        outer_elem=boom_root,
    )
    ctx.expect_gap(
        arm,
        arm,
        axis="x",
        positive_elem=boom_tip,
        negative_elem=right_counterweight,
        min_gap=5.5,
    )
    ctx.expect_overlap(
        arm,
        arm,
        axes="yz",
        elem_a=stripe_3,
        elem_b=boom_mid_a,
        min_overlap=0.07,
    )
    ctx.expect_overlap(
        arm,
        arm,
        axes="yz",
        elem_a=stripe_6,
        elem_b=boom_outer,
        min_overlap=0.05,
    )
    with ctx.pose({gate_pivot: 1.2}):
        ctx.expect_gap(
            arm,
            support,
            axis="z",
            positive_elem=tip_band,
            negative_elem=upper_cap,
            min_gap=4.0,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
