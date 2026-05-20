from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freight_goods_elevator")

    frame_gray = model.material("frame_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.47, 0.49, 0.50, 1.0))
    deck_plate = model.material("deck_plate", rgba=(0.63, 0.64, 0.66, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.76, 0.17, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.08, 0.96, 0.08)),
        origin=Origin(xyz=(0.60, 0.0, 0.04)),
        material=frame_gray,
        name="left_base_beam",
    )
    frame.visual(
        Box((0.08, 0.96, 0.08)),
        origin=Origin(xyz=(-0.60, 0.0, 0.04)),
        material=frame_gray,
        name="right_base_beam",
    )
    frame.visual(
        Box((1.28, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.38, 0.04)),
        material=frame_gray,
        name="front_base_beam",
    )
    frame.visual(
        Box((1.28, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.44, 0.04)),
        material=frame_gray,
        name="rear_base_beam",
    )
    frame.visual(
        Box((0.28, 0.12, 0.16)),
        origin=Origin(xyz=(0.46, -0.50, 0.16)),
        material=frame_gray,
        name="left_mast_foot",
    )
    frame.visual(
        Box((0.28, 0.12, 0.16)),
        origin=Origin(xyz=(-0.46, -0.50, 0.16)),
        material=frame_gray,
        name="right_mast_foot",
    )
    frame.visual(
        Box((0.08, 0.08, 2.24)),
        origin=Origin(xyz=(0.34, -0.45, 1.20)),
        material=frame_gray,
        name="left_guide_rail",
    )
    frame.visual(
        Box((0.08, 0.08, 2.24)),
        origin=Origin(xyz=(-0.34, -0.45, 1.20)),
        material=frame_gray,
        name="right_guide_rail",
    )
    frame.visual(
        Box((0.84, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, -0.45, 2.37)),
        material=frame_gray,
        name="top_header",
    )
    frame.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.45, 1.18)),
        material=frame_gray,
        name="mid_crossbrace",
    )
    frame.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(0.60, 0.42, 0.01)),
        material=black_rubber,
        name="left_front_pad",
    )
    frame.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(-0.60, 0.42, 0.01)),
        material=black_rubber,
        name="right_front_pad",
    )
    frame.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(0.60, -0.42, 0.01)),
        material=black_rubber,
        name="left_rear_pad",
    )
    frame.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(-0.60, -0.42, 0.01)),
        material=black_rubber,
        name="right_rear_pad",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.30, 1.02, 2.44)),
        mass=420.0,
        origin=Origin(xyz=(0.0, -0.03, 1.22)),
    )

    platform = model.part("platform")
    platform.visual(
        Box((1.08, 0.76, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=deck_plate,
        name="deck_plate",
    )
    platform.visual(
        Box((1.08, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, 0.40, 0.06)),
        material=safety_yellow,
        name="front_toeboard",
    )
    platform.visual(
        Box((0.04, 0.72, 0.12)),
        origin=Origin(xyz=(0.56, 0.01, 0.06)),
        material=safety_yellow,
        name="left_side_curb",
    )
    platform.visual(
        Box((0.04, 0.72, 0.12)),
        origin=Origin(xyz=(-0.56, 0.01, 0.06)),
        material=safety_yellow,
        name="right_side_curb",
    )
    platform.visual(
        Box((0.72, 0.18, 0.22)),
        origin=Origin(xyz=(0.0, -0.24, 0.11)),
        material=carriage_gray,
        name="rear_support_block",
    )
    platform.visual(
        Box((0.70, 0.08, 0.76)),
        origin=Origin(xyz=(0.0, -0.29, 0.60)),
        material=carriage_gray,
        name="back_carriage_plate",
    )
    platform.visual(
        Box((0.12, 0.08, 0.78)),
        origin=Origin(xyz=(0.34, -0.37, 0.59)),
        material=carriage_gray,
        name="left_guide_shoe",
    )
    platform.visual(
        Box((0.12, 0.08, 0.78)),
        origin=Origin(xyz=(-0.34, -0.37, 0.59)),
        material=carriage_gray,
        name="right_guide_shoe",
    )
    platform.visual(
        Box((0.06, 0.06, 0.72)),
        origin=Origin(xyz=(-0.53, 0.37, 0.36)),
        material=safety_yellow,
        name="left_chain_post",
    )
    platform.visual(
        Box((0.05, 0.06, 0.56)),
        origin=Origin(xyz=(-0.495, 0.40, 0.36)),
        material=safety_yellow,
        name="left_hinge_boss",
    )
    platform.visual(
        Box((0.06, 0.06, 0.72)),
        origin=Origin(xyz=(0.53, 0.37, 0.36)),
        material=safety_yellow,
        name="right_chain_post",
    )
    platform.inertial = Inertial.from_geometry(
        Box((1.18, 0.88, 1.05)),
        mass=230.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    barrier = model.part("barrier")
    barrier.visual(
        Box((0.04, 0.05, 0.56)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=safety_yellow,
        name="left_hinge_stile",
    )
    barrier.visual(
        Box((0.03, 0.05, 0.56)),
        origin=Origin(xyz=(0.97, 0.025, 0.0)),
        material=safety_yellow,
        name="right_latch_stile",
    )
    barrier.visual(
        Box((0.02, 0.03, 0.08)),
        origin=Origin(xyz=(0.995, 0.025, 0.0)),
        material=safety_yellow,
        name="latch_tip",
    )
    barrier.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.015, 0.025, 0.13),
                    (0.24, 0.025, 0.14),
                    (0.58, 0.025, 0.09),
                    (0.955, 0.025, 0.13),
                ],
                radius=0.009,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            ),
            "upper_chain_run",
        ),
        material=chain_steel,
        name="upper_chain_run",
    )
    barrier.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.015, 0.025, -0.13),
                    (0.24, 0.025, -0.15),
                    (0.58, 0.025, -0.18),
                    (0.955, 0.025, -0.13),
                ],
                radius=0.009,
                samples_per_segment=18,
                radial_segments=16,
                cap_ends=True,
            ),
            "lower_chain_run",
        ),
        material=chain_steel,
        name="lower_chain_run",
    )
    barrier.inertial = Inertial.from_geometry(
        Box((1.03, 0.08, 0.60)),
        mass=8.0,
        origin=Origin(xyz=(0.50, 0.025, 0.0)),
    )

    model.articulation(
        "frame_to_platform",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.00, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3200.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "platform_to_barrier",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=barrier,
        origin=Origin(xyz=(-0.49, 0.43, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    platform = object_model.get_part("platform")
    barrier = object_model.get_part("barrier")
    lift = object_model.get_articulation("frame_to_platform")
    barrier_hinge = object_model.get_articulation("platform_to_barrier")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({lift: 0.0, barrier_hinge: 0.0}):
        ctx.expect_gap(
            platform,
            frame,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="front_base_beam",
            min_gap=0.005,
            max_gap=0.080,
            name="deck sits slightly above the base frame",
        )
        ctx.expect_contact(
            platform,
            frame,
            elem_a="left_guide_shoe",
            elem_b="left_guide_rail",
            contact_tol=0.001,
            name="left guide shoe bears against the left rail",
        )
        ctx.expect_contact(
            platform,
            frame,
            elem_a="right_guide_shoe",
            elem_b="right_guide_rail",
            contact_tol=0.001,
            name="right guide shoe bears against the right rail",
        )
        ctx.expect_overlap(
            platform,
            frame,
            axes="xz",
            elem_a="left_guide_shoe",
            elem_b="left_guide_rail",
            min_overlap=0.06,
            name="left guide shoe overlaps the rail footprint at rest",
        )
        ctx.expect_overlap(
            platform,
            barrier,
            axes="xz",
            elem_a="right_chain_post",
            elem_b="latch_tip",
            min_overlap=0.010,
            name="closed barrier latch aligns with the right chain post",
        )
        ctx.expect_gap(
            barrier,
            platform,
            axis="y",
            positive_elem="latch_tip",
            negative_elem="right_chain_post",
            min_gap=0.020,
            max_gap=0.080,
            name="closed barrier latch sits just ahead of the right chain post",
        )
        ctx.expect_contact(
            platform,
            barrier,
            elem_a="left_hinge_boss",
            elem_b="left_hinge_stile",
            contact_tol=1e-6,
            name="barrier hinge stile bears on the hinge boss",
        )

    rest_platform_pos = ctx.part_world_position(platform)
    with ctx.pose({lift: 1.20, barrier_hinge: 0.0}):
        top_platform_pos = ctx.part_world_position(platform)
        ctx.expect_overlap(
            platform,
            frame,
            axes="xz",
            elem_a="left_guide_shoe",
            elem_b="left_guide_rail",
            min_overlap=0.06,
            name="left guide shoe remains engaged at maximum lift",
        )
        ctx.expect_overlap(
            platform,
            frame,
            axes="xz",
            elem_a="right_guide_shoe",
            elem_b="right_guide_rail",
            min_overlap=0.06,
            name="right guide shoe remains engaged at maximum lift",
        )
    ctx.check(
        "platform rises upward on the guide rails",
        rest_platform_pos is not None
        and top_platform_pos is not None
        and top_platform_pos[2] > rest_platform_pos[2] + 1.0,
        details=f"rest={rest_platform_pos}, top={top_platform_pos}",
    )

    with ctx.pose({lift: 0.0, barrier_hinge: 0.0}):
        closed_latch_center = _aabb_center(ctx.part_element_world_aabb(barrier, elem="latch_tip"))
    with ctx.pose({lift: 0.0, barrier_hinge: radians(80.0)}):
        open_latch_center = _aabb_center(ctx.part_element_world_aabb(barrier, elem="latch_tip"))
    ctx.check(
        "barrier swings outward from the chain post",
        closed_latch_center is not None
        and open_latch_center is not None
        and open_latch_center[1] > closed_latch_center[1] + 0.25,
        details=f"closed={closed_latch_center}, open={open_latch_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
