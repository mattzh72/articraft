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
    model = ArticulatedObject(name="parking_boom_gate")

    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.66, 1.0))
    housing_yellow = model.material("housing_yellow", rgba=(0.88, 0.73, 0.18, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    galvanized = model.material("galvanized", rgba=(0.71, 0.73, 0.76, 1.0))
    boom_white = model.material("boom_white", rgba=(0.94, 0.95, 0.96, 1.0))
    signal_red = model.material("signal_red", rgba=(0.84, 0.14, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))

    pivot_x = 0.20
    pivot_z = 1.07

    post = model.part("post_assembly")
    post.visual(
        Box((0.50, 0.50, 0.06)),
        origin=Origin(xyz=(-0.18, 0.0, 0.03)),
        material=concrete,
        name="footing",
    )
    post.visual(
        Box((0.34, 0.30, 0.82)),
        origin=Origin(xyz=(-0.11, 0.0, 0.41)),
        material=housing_yellow,
        name="operator_cabinet",
    )
    post.visual(
        Box((0.20, 0.22, 0.18)),
        origin=Origin(xyz=(-0.01, 0.0, 0.91)),
        material=housing_yellow,
        name="upper_neck",
    )
    post.visual(
        Box((0.22, 0.18, 0.11)),
        origin=Origin(xyz=(-0.02, 0.0, 0.98)),
        material=dark_graphite,
        name="head_body",
    )
    post.visual(
        Box((0.14, 0.02, 0.22)),
        origin=Origin(xyz=(0.16, 0.10, 1.04)),
        material=galvanized,
        name="pivot_cheek_left",
    )
    post.visual(
        Box((0.14, 0.02, 0.22)),
        origin=Origin(xyz=(0.16, -0.10, 1.04)),
        material=galvanized,
        name="pivot_cheek_right",
    )
    post.visual(
        Box((0.05, 0.03, 0.014)),
        origin=Origin(xyz=(0.175, 0.077, 1.035)),
        material=galvanized,
        name="pivot_bearing_left",
    )
    post.visual(
        Box((0.05, 0.03, 0.014)),
        origin=Origin(xyz=(0.175, -0.077, 1.035)),
        material=galvanized,
        name="pivot_bearing_right",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.62, 0.50, 1.17)),
        mass=210.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.585)),
    )

    arm = model.part("barrier_arm")
    arm.visual(
        Cylinder(radius=0.028, length=0.172),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pivot_shaft",
    )
    arm.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=dark_graphite,
        name="hub_block",
    )
    arm.visual(
        Box((4.42, 0.10, 0.08)),
        origin=Origin(xyz=(2.31, 0.0, 0.06)),
        material=boom_white,
        name="boom_beam",
    )
    arm.visual(
        Box((0.09, 0.12, 0.09)),
        origin=Origin(xyz=(4.565, 0.0, 0.06)),
        material=rubber_black,
        name="boom_tip",
    )
    for index, x_pos in enumerate((1.15, 2.25, 3.35, 4.10), start=1):
        arm.visual(
            Box((0.22, 0.102, 0.082)),
            origin=Origin(xyz=(x_pos, 0.0, 0.06)),
            material=signal_red,
            name=f"boom_stripe_{index}",
        )
    arm.visual(
        Box((0.98, 0.08, 0.06)),
        origin=Origin(xyz=(-0.45, 0.0, 0.04)),
        material=galvanized,
        name="counterweight_arm",
    )
    arm.visual(
        Box((0.36, 0.18, 0.22)),
        origin=Origin(xyz=(-0.96, 0.0, -0.10)),
        material=dark_graphite,
        name="counterweight_box",
    )
    arm.inertial = Inertial.from_geometry(
        Box((5.76, 0.22, 0.40)),
        mass=34.0,
        origin=Origin(xyz=(1.70, 0.0, -0.01)),
    )

    model.articulation(
        "boom_pivot",
        ArticulationType.REVOLUTE,
        parent=post,
        child=arm,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(86.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("post_assembly")
    arm = object_model.get_part("barrier_arm")
    pivot = object_model.get_articulation("boom_pivot")
    open_angle = pivot.motion_limits.upper if pivot.motion_limits is not None else math.radians(86.0)

    def elem_center(part_name, elem_name):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    with ctx.pose({pivot: 0.0}):
        ctx.expect_gap(
            arm,
            post,
            axis="z",
            positive_elem="boom_beam",
            negative_elem="head_body",
            min_gap=0.02,
            max_gap=0.10,
            name="closed boom clears pivot housing",
        )

        closed_tip = elem_center(arm, "boom_tip")
        closed_weight = elem_center(arm, "counterweight_box")
        post_aabb = ctx.part_world_aabb(post)
        post_front_x = post_aabb[1][0] if post_aabb is not None else None

    with ctx.pose({pivot: open_angle}):
        ctx.expect_gap(
            arm,
            post,
            axis="x",
            positive_elem="counterweight_box",
            negative_elem="operator_cabinet",
            min_gap=0.03,
            name="opened counterweight clears cabinet front",
        )
        ctx.expect_gap(
            arm,
            post,
            axis="x",
            positive_elem="counterweight_box",
            negative_elem="footing",
            min_gap=0.03,
            name="opened counterweight clears footing front edge",
        )
        open_tip = elem_center(arm, "boom_tip")
        open_weight = elem_center(arm, "counterweight_box")

    ctx.check(
        "boom projects forward from the cabinet",
        closed_tip is not None and post_front_x is not None and closed_tip[0] > post_front_x + 4.2,
        details=f"closed_tip={closed_tip}, post_front_x={post_front_x}",
    )
    ctx.check(
        "counterweight sits behind the pivot when closed",
        closed_weight is not None and closed_weight[0] < 0.0,
        details=f"closed_weight={closed_weight}",
    )
    ctx.check(
        "boom tip rises strongly when opened",
        closed_tip is not None
        and open_tip is not None
        and open_tip[2] > closed_tip[2] + 3.8
        and open_tip[0] < closed_tip[0] - 3.8,
        details=f"closed_tip={closed_tip}, open_tip={open_tip}",
    )
    ctx.check(
        "rear counterweight swings downward as the boom opens",
        closed_weight is not None and open_weight is not None and open_weight[2] < closed_weight[2] - 0.7,
        details=f"closed_weight={closed_weight}, open_weight={open_weight}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
