from __future__ import annotations

import math

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
    model = ArticulatedObject(name="double_leaf_picket_gate")

    painted_wood = Material("painted_wood", rgba=(0.18, 0.42, 0.24, 1.0))
    dark_iron = Material("dark_iron", rgba=(0.035, 0.033, 0.030, 1.0))
    weathered_stone = Material("weathered_stone", rgba=(0.55, 0.52, 0.46, 1.0))
    brass = Material("aged_brass", rgba=(0.74, 0.55, 0.22, 1.0))

    model.material("painted_wood", rgba=(0.18, 0.42, 0.24, 1.0))
    model.material("dark_iron", rgba=(0.035, 0.033, 0.030, 1.0))
    model.material("weathered_stone", rgba=(0.55, 0.52, 0.46, 1.0))
    model.material("aged_brass", rgba=(0.74, 0.55, 0.22, 1.0))

    frame = model.part("courtyard_frame")
    frame.visual(
        Box((0.18, 0.20, 1.58)),
        origin=Origin(xyz=(-1.12, 0.0, 0.79)),
        material=weathered_stone,
        name="post_0",
    )
    frame.visual(
        Box((0.18, 0.20, 1.58)),
        origin=Origin(xyz=(1.12, 0.0, 0.79)),
        material=weathered_stone,
        name="post_1",
    )
    frame.visual(
        Box((0.26, 0.25, 0.08)),
        origin=Origin(xyz=(-1.12, 0.0, 1.62)),
        material=weathered_stone,
        name="cap_0",
    )
    frame.visual(
        Box((0.26, 0.25, 0.08)),
        origin=Origin(xyz=(1.12, 0.0, 1.62)),
        material=weathered_stone,
        name="cap_1",
    )
    frame.visual(
        Box((2.46, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, -0.05, 0.02)),
        material=weathered_stone,
        name="stone_sill",
    )

    # Post-side hinge straps: the moving barrels are carried on each leaf, while
    # these plates visibly tie the hinge axes back into the masonry posts.
    for side, hinge_x in ((-1.0, -1.0), (1.0, 1.0)):
        post_plate_x = hinge_x + side * 0.053
        for index, z in enumerate((0.46, 1.08)):
            frame.visual(
                Box((0.070, 0.014, 0.145)),
                origin=Origin(xyz=(post_plate_x, -0.105, z)),
                material=dark_iron,
                name=f"post_hinge_{side}_{index}",
            )

    # Open ground keeper around the cane bolt tip; four solid cheeks leave a
    # clear center hole rather than intersecting the sliding bolt.
    keeper_x = 0.28
    keeper_y = -0.147
    for name, xyz, size in (
        ("keeper_front", (keeper_x, keeper_y - 0.036, 0.057), (0.080, 0.012, 0.034)),
        ("keeper_rear", (keeper_x, keeper_y + 0.036, 0.057), (0.080, 0.012, 0.034)),
        ("keeper_side_0", (keeper_x - 0.036, keeper_y, 0.057), (0.012, 0.084, 0.034)),
        ("keeper_side_1", (keeper_x + 0.036, keeper_y, 0.057), (0.012, 0.084, 0.034)),
    ):
        frame.visual(Box(size), origin=Origin(xyz=xyz), material=dark_iron, name=name)

    def add_leaf(part, *, direction: float) -> None:
        """Add one framed picket leaf in a hinge-local frame.

        direction=+1 builds the latch leaf from its hinge toward +X; direction=-1
        builds the bolt leaf from its hinge toward -X.
        """

        width = 0.98
        thickness = 0.055
        stile_w = 0.058
        rail_h = 0.065
        picket_w = 0.034
        picket_h = 0.91

        def x(value: float) -> float:
            return direction * value

        part.visual(
            Box((stile_w, thickness, 1.06)),
            origin=Origin(xyz=(x(0.048), 0.0, 0.68)),
            material=painted_wood,
            name="outer_stile",
        )
        part.visual(
            Box((stile_w, thickness, 1.06)),
            origin=Origin(xyz=(x(width - 0.032), 0.0, 0.68)),
            material=painted_wood,
            name="meeting_stile",
        )
        for name, z, height in (
            ("bottom_rail", 0.235, 0.080),
            ("middle_rail", 0.640, 0.055),
            ("top_rail", 0.995, rail_h),
        ):
            part.visual(
                Box((width - 0.090, thickness, height)),
                origin=Origin(xyz=(x(width * 0.50), 0.0, z)),
                material=painted_wood,
                name=name,
            )

        picket_positions = (0.180, 0.315, 0.450, 0.585, 0.720, 0.855)
        tip_half = 0.026
        tip_height = 0.085
        tip_len = math.hypot(tip_half, tip_height)
        tip_angle = math.atan2(tip_half, tip_height)
        for i, px in enumerate(picket_positions):
            part.visual(
                Box((picket_w, thickness * 0.78, picket_h)),
                origin=Origin(xyz=(x(px), 0.0, 0.665)),
                material=painted_wood,
                name=f"picket_{i}",
            )
            # Two narrow sloped boards form a visible pointed picket cap.
            for side_name, local_dx, sign in (
                ("tip_0", -tip_half * 0.5, 1.0),
                ("tip_1", tip_half * 0.5, -1.0),
            ):
                part.visual(
                    Box((0.014, thickness * 0.78, tip_len)),
                    origin=Origin(
                        xyz=(x(px + local_dx), 0.0, 1.105 + tip_height * 0.5),
                        rpy=(0.0, direction * sign * tip_angle, 0.0),
                    ),
                    material=painted_wood,
                    name=f"picket_{i}_{side_name}",
                )

        # Moving hinge barrels and leaf-side straps are part of the leaf and
        # rotate with it.
        for index, z in enumerate((0.46, 1.08)):
            part.visual(
                Cylinder(radius=0.018, length=0.155),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=dark_iron,
                name=f"hinge_barrel_{index}",
            )
            part.visual(
                Box((0.088, 0.018, 0.082)),
                origin=Origin(xyz=(x(0.040), -0.002, z)),
                material=dark_iron,
                name=f"hinge_strap_{index}",
            )

    latch_leaf = model.part("latch_leaf")
    add_leaf(latch_leaf, direction=1.0)

    bolt_leaf = model.part("bolt_leaf")
    add_leaf(bolt_leaf, direction=-1.0)

    # Static catch and cane-bolt guide loops mounted to the bolt leaf.
    bolt_leaf.visual(
        Box((0.112, 0.014, 0.024)),
        origin=Origin(xyz=(-0.940, -0.0345, 0.760)),
        material=dark_iron,
        name="latch_catch_upper",
    )
    bolt_leaf.visual(
        Box((0.112, 0.014, 0.024)),
        origin=Origin(xyz=(-0.940, -0.0345, 0.680)),
        material=dark_iron,
        name="latch_catch_lower",
    )

    cane_x = -0.72
    cane_y = -0.042
    bolt_leaf.visual(
        Box((0.112, 0.012, 0.036)),
        origin=Origin(xyz=(cane_x, -0.059, 0.405)),
        material=dark_iron,
        name="lower_guide_front",
    )
    bolt_leaf.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(cane_x - 0.042, -0.04645, 0.405)),
        material=dark_iron,
        name="lower_guide_side_0",
    )
    bolt_leaf.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(cane_x + 0.042, -0.04645, 0.405)),
        material=dark_iron,
        name="lower_guide_side_1",
    )
    bolt_leaf.visual(
        Box((0.112, 0.006, 0.036)),
        origin=Origin(xyz=(cane_x, -0.02445, 0.405)),
        material=dark_iron,
        name="lower_guide_backplate",
    )
    bolt_leaf.visual(
        Box((0.112, 0.012, 0.036)),
        origin=Origin(xyz=(cane_x, -0.059, 0.865)),
        material=dark_iron,
        name="upper_guide_front",
    )
    bolt_leaf.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(cane_x - 0.042, -0.04645, 0.865)),
        material=dark_iron,
        name="upper_guide_side_0",
    )
    bolt_leaf.visual(
        Box((0.014, 0.050, 0.036)),
        origin=Origin(xyz=(cane_x + 0.042, -0.04645, 0.865)),
        material=dark_iron,
        name="upper_guide_side_1",
    )
    bolt_leaf.visual(
        Box((0.112, 0.006, 0.036)),
        origin=Origin(xyz=(cane_x, -0.02445, 0.865)),
        material=dark_iron,
        name="upper_guide_backplate",
    )

    model.articulation(
        "frame_to_latch_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=latch_leaf,
        origin=Origin(xyz=(-1.0, -0.105, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "frame_to_bolt_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bolt_leaf,
        origin=Origin(xyz=(1.0, -0.105, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_boss",
    )
    latch.visual(
        Box((0.300, 0.012, 0.034)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=dark_iron,
        name="latch_bar",
    )
    latch.visual(
        Box((0.040, 0.014, 0.095)),
        origin=Origin(xyz=(0.045, 0.0, 0.050)),
        material=dark_iron,
        name="thumb_lift",
    )
    model.articulation(
        "leaf_to_latch",
        ArticulationType.REVOLUTE,
        parent=latch_leaf,
        child=latch,
        origin=Origin(xyz=(0.930, -0.036, 0.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=0.90),
    )

    cane_bolt = model.part("cane_bolt")
    cane_bolt.visual(
        Cylinder(radius=0.011, length=0.960),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=dark_iron,
        name="bolt_rod",
    )
    cane_bolt.visual(
        Cylinder(radius=0.012, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.720), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="lift_handle",
    )
    model.articulation(
        "leaf_to_cane_bolt",
        ArticulationType.PRISMATIC,
        parent=bolt_leaf,
        child=cane_bolt,
        origin=Origin(xyz=(cane_x, cane_y, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    latch_leaf = object_model.get_part("latch_leaf")
    bolt_leaf = object_model.get_part("bolt_leaf")
    latch = object_model.get_part("latch")
    cane_bolt = object_model.get_part("cane_bolt")
    latch_hinge = object_model.get_articulation("frame_to_latch_leaf")
    bolt_hinge = object_model.get_articulation("frame_to_bolt_leaf")
    latch_pivot = object_model.get_articulation("leaf_to_latch")
    bolt_slide = object_model.get_articulation("leaf_to_cane_bolt")

    ctx.expect_gap(
        bolt_leaf,
        latch_leaf,
        axis="x",
        min_gap=0.015,
        max_gap=0.075,
        name="closed leaves meet at a narrow center gap",
    )
    ctx.expect_overlap(
        latch_leaf,
        bolt_leaf,
        axes="z",
        min_overlap=0.85,
        name="two framed leaves share gate height",
    )
    ctx.expect_within(
        cane_bolt,
        bolt_leaf,
        axes="x",
        inner_elem="bolt_rod",
        outer_elem="upper_guide_front",
        margin=0.002,
        name="cane bolt rod is centered in the upper guide loop",
    )
    ctx.expect_overlap(
        cane_bolt,
        bolt_leaf,
        axes="z",
        elem_a="bolt_rod",
        elem_b="lower_guide_front",
        min_overlap=0.025,
        name="cane bolt passes through lower guide loop",
    )

    closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    with ctx.pose({latch_pivot: 0.80}):
        lifted_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")
    ctx.check(
        "rotating latch lifts the free end",
        closed_latch_aabb is not None
        and lifted_latch_aabb is not None
        and lifted_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.12,
        details=f"closed={closed_latch_aabb}, lifted={lifted_latch_aabb}",
    )

    closed_bolt_aabb = ctx.part_element_world_aabb(cane_bolt, elem="bolt_rod")
    with ctx.pose({bolt_slide: 0.180}):
        raised_bolt_aabb = ctx.part_element_world_aabb(cane_bolt, elem="bolt_rod")
        ctx.expect_within(
            cane_bolt,
            bolt_leaf,
            axes="x",
            inner_elem="bolt_rod",
            outer_elem="lower_guide_front",
            margin=0.002,
            name="raised cane bolt remains inside lower guide width",
        )
    ctx.check(
        "cane bolt slides upward",
        closed_bolt_aabb is not None
        and raised_bolt_aabb is not None
        and raised_bolt_aabb[0][2] > closed_bolt_aabb[0][2] + 0.15,
        details=f"closed={closed_bolt_aabb}, raised={raised_bolt_aabb}",
    )

    closed_latch_leaf_aabb = ctx.part_world_aabb(latch_leaf)
    closed_bolt_leaf_aabb = ctx.part_world_aabb(bolt_leaf)
    with ctx.pose({latch_hinge: 1.20}):
        open_latch_leaf_aabb = ctx.part_world_aabb(latch_leaf)
    with ctx.pose({bolt_hinge: 1.20}):
        open_bolt_leaf_aabb = ctx.part_world_aabb(bolt_leaf)
    ctx.check(
        "latch leaf swings outward from its post",
        closed_latch_leaf_aabb is not None
        and open_latch_leaf_aabb is not None
        and open_latch_leaf_aabb[0][1] < closed_latch_leaf_aabb[0][1] - 0.35,
        details=f"closed={closed_latch_leaf_aabb}, open={open_latch_leaf_aabb}",
    )
    ctx.check(
        "bolt leaf swings outward from its post",
        closed_bolt_leaf_aabb is not None
        and open_bolt_leaf_aabb is not None
        and open_bolt_leaf_aabb[0][1] < closed_bolt_leaf_aabb[0][1] - 0.35,
        details=f"closed={closed_bolt_leaf_aabb}, open={open_bolt_leaf_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
