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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freight_goods_platform_elevator")

    safety_yellow = Material("safety_yellow", rgba=(1.0, 0.72, 0.05, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    deck_black = Material("black_checker_plate", rgba=(0.025, 0.028, 0.03, 1.0))
    concrete = Material("worn_concrete", rgba=(0.45, 0.43, 0.39, 1.0))
    rubber = Material("rubber_bumper", rgba=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("shaft_frame")
    frame.visual(
        Box((2.10, 1.70, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="pit_floor",
    )

    # Four welded corner uprights and high crossheads make the fixed elevator frame.
    for x in (-0.92, 0.92):
        for y in (-0.66, 0.66):
            frame.visual(
                Box((0.085, 0.085, 2.92)),
                origin=Origin(xyz=(x, y, 1.54)),
                material=dark_steel,
                name=f"corner_upright_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y",
            )

    for y in (-0.66, 0.66):
        frame.visual(
            Box((1.92, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, y, 3.02)),
            material=dark_steel,
            name=f"top_crossbeam_{'front' if y < 0 else 'rear'}",
        )
    for x in (-0.92, 0.92):
        frame.visual(
            Box((0.10, 1.38, 0.12)),
            origin=Origin(xyz=(x, 0.0, 3.02)),
            material=dark_steel,
            name=f"side_crossbeam_{'px' if x > 0 else 'nx'}",
        )

    # Close-running guide rails: the car's guide shoes slide beside these rails.
    for x, y, name in (
        (-0.80, -0.38, "guide_rail_nx_front"),
        (-0.80, 0.38, "guide_rail_nx_rear"),
        (0.80, -0.38, "guide_rail_px_front"),
        (0.80, 0.38, "guide_rail_px_rear"),
    ):
        frame.visual(
            Box((0.040, 0.070, 2.86)),
            origin=Origin(xyz=(x, y, 1.51)),
            material=galvanized,
            name=name,
        )

    # Low buffers and sill details show this is an open freight lift rather than a room elevator.
    for x in (-0.46, 0.46):
        frame.visual(
            Box((0.18, 0.18, 0.10)),
            origin=Origin(xyz=(x, -0.53, 0.13)),
            material=rubber,
            name=f"bottom_buffer_{'px' if x > 0 else 'nx'}",
        )
    frame.visual(
        Box((1.55, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, -0.78, 0.1075)),
        material=safety_yellow,
        name="loading_sill",
    )

    platform = model.part("platform")
    platform.visual(
        Box((1.36, 1.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=deck_black,
        name="deck_plate",
    )
    # Side and rear toe boards are welded to the flat deck; the front remains open.
    for x in (-0.70, 0.70):
        platform.visual(
            Box((0.045, 1.04, 0.16)),
            origin=Origin(xyz=(x, 0.0, 0.18)),
            material=safety_yellow,
            name=f"side_toeboard_{'px' if x > 0 else 'nx'}",
        )
    platform.visual(
        Box((1.36, 0.045, 0.16)),
        origin=Origin(xyz=(0.0, 0.52, 0.18)),
        material=safety_yellow,
        name="rear_toeboard",
    )

    # Four guide shoes are close to, but not penetrating, the fixed guide rails.
    for x, y, name in (
        (-0.738, -0.38, "guide_shoe_nx_front"),
        (-0.738, 0.38, "guide_shoe_nx_rear"),
        (0.738, -0.38, "guide_shoe_px_front"),
        (0.738, 0.38, "guide_shoe_px_rear"),
    ):
        platform.visual(
            Box((0.076, 0.16, 0.12)),
            origin=Origin(xyz=(x, y, 0.205)),
            material=galvanized,
            name=name,
        )

    # Open guard rail around the fixed sides of the car.
    for x in (-0.67, 0.67):
        for y in (-0.50, 0.50):
            platform.visual(
                Cylinder(radius=0.026, length=1.02),
                origin=Origin(xyz=(x, y, 0.61)),
                material=safety_yellow,
                name=f"guard_post_{'px' if x > 0 else 'nx'}_{'front' if y < 0 else 'rear'}",
            )
    platform.visual(
        Cylinder(radius=0.025, length=1.34),
        origin=Origin(xyz=(0.0, 0.50, 1.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_yellow,
        name="rear_top_rail",
    )
    for x in (-0.67, 0.67):
        platform.visual(
            Cylinder(radius=0.025, length=0.92),
            origin=Origin(xyz=(x, 0.04, 1.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name=f"side_top_rail_{'px' if x > 0 else 'nx'}",
        )

    # Static knuckles and leaf plates on the chain post. The moving center knuckle
    # lives on the barrier part, leaving real clearance between hinge barrels.
    hinge_x, hinge_y, hinge_z = -0.67, -0.58, 0.88
    for zoff, label in ((-0.18, "lower"), (0.18, "upper")):
        platform.visual(
            Cylinder(radius=0.023, length=0.11),
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + zoff)),
            material=galvanized,
            name=f"post_hinge_knuckle_{label}",
        )
        platform.visual(
            Box((0.070, 0.090, 0.105)),
            origin=Origin(xyz=(hinge_x, -0.535, hinge_z + zoff)),
            material=galvanized,
            name=f"post_hinge_leaf_{label}",
        )

    platform.visual(
        Box((0.060, 0.090, 0.11)),
        origin=Origin(xyz=(0.641999, -0.535, hinge_z - 0.08)),
        material=galvanized,
        name="latch_receiver",
    )

    barrier = model.part("chain_barrier")
    barrier.visual(
        Cylinder(radius=0.021, length=0.18),
        origin=Origin(),
        material=galvanized,
        name="barrier_hinge_knuckle",
    )
    barrier.visual(
        Box((0.10, 0.026, 0.11)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=galvanized,
        name="barrier_hinge_leaf",
    )

    chain_loop = tube_from_spline_points(
        [
            (0.06, 0.0, 0.06),
            (0.36, 0.0, -0.02),
            (0.70, 0.0, -0.055),
            (1.02, 0.0, -0.02),
            (1.27, 0.0, 0.04),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    barrier.visual(
        mesh_from_geometry(chain_loop, "safety_chain_loop"),
        material=galvanized,
        name="chain_loop",
    )
    model.articulation(
        "frame_to_platform",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.45, lower=0.0, upper=1.80),
    )

    model.articulation(
        "platform_to_barrier",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=barrier,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("shaft_frame")
    platform = object_model.get_part("platform")
    barrier = object_model.get_part("chain_barrier")
    lift = object_model.get_articulation("frame_to_platform")
    hinge = object_model.get_articulation("platform_to_barrier")

    # The car rides close to, but not through, the fixed guide rails.
    ctx.expect_gap(
        frame,
        platform,
        axis="x",
        positive_elem="guide_rail_px_front",
        negative_elem="guide_shoe_px_front",
        min_gap=0.002,
        max_gap=0.010,
        name="front positive guide shoe is clearanced to rail",
    )
    ctx.expect_gap(
        platform,
        frame,
        axis="x",
        positive_elem="guide_shoe_nx_front",
        negative_elem="guide_rail_nx_front",
        min_gap=0.002,
        max_gap=0.010,
        name="front negative guide shoe is clearanced to rail",
    )

    rest_pos = ctx.part_world_position(platform)
    with ctx.pose({lift: 1.80}):
        raised_pos = ctx.part_world_position(platform)
        ctx.expect_gap(
            frame,
            platform,
            axis="x",
            positive_elem="guide_rail_px_rear",
            negative_elem="guide_shoe_px_rear",
            min_gap=0.002,
            max_gap=0.010,
            name="raised car remains guided on positive rail",
        )

    ctx.check(
        "platform slides vertically on rails",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 1.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    # Closed chain reaches the latch post; at the upper limit it swings outward
    # around the chain-post hinge rather than translating with the car.
    ctx.expect_contact(
        platform,
        barrier,
        elem_a="latch_receiver",
        elem_b="chain_loop",
        contact_tol=0.001,
        name="closed chain sits at latch receiver",
    )
    closed_aabb = ctx.part_world_aabb(barrier)
    with ctx.pose({hinge: 1.45}):
        open_aabb = ctx.part_world_aabb(barrier)
    ctx.check(
        "chain barrier swings outward from post",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.70,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
