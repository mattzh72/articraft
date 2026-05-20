from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _cylinder_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    midpoint = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return length, Origin(xyz=midpoint, rpy=(0.0, pitch, yaw))


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> None:
    length, origin = _cylinder_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin)


def _support_bar_point(side_sign: float, fraction: float) -> tuple[float, float]:
    top_y = 0.0
    top_z = -0.028
    bottom_y = side_sign * 0.332
    bottom_z = -0.704
    return (
        top_y + fraction * (bottom_y - top_y),
        top_z + fraction * (bottom_z - top_z),
    )


def _build_support_frame(model: ArticulatedObject, name: str, side_sign: float) -> None:
    frame = model.part(name)

    top_left = (-0.458, 0.0, -0.028)
    top_right = (0.458, 0.0, -0.028)
    bottom_left = (-0.448, side_sign * 0.332, -0.704)
    bottom_right = (0.448, side_sign * 0.332, -0.704)

    _add_tube(frame, top_left, top_right, radius=0.0065)
    _add_tube(frame, bottom_left, bottom_right, radius=0.0075)
    _add_tube(frame, top_left, bottom_left, radius=0.0065)
    _add_tube(frame, top_right, bottom_right, radius=0.0065)
    frame.visual(
        Box((0.140, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    for fraction in (0.24, 0.46, 0.68):
        bar_y, bar_z = _support_bar_point(side_sign, fraction)
        span_x = 0.458 - 0.010 * fraction
        _add_tube(frame, (-span_x, bar_y, bar_z), (span_x, bar_y, bar_z), radius=0.0050)
        for x_pos in (-span_x, span_x):
            frame.visual(
                Box((0.026, 0.018, 0.020)),
                origin=Origin(xyz=(x_pos, bar_y, bar_z)),
            )

    for x_pos in (-0.458, 0.458):
        frame.visual(
            Box((0.030, 0.018, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, -0.028)),
        )
        frame.visual(
            Box((0.034, 0.020, 0.028)),
            origin=Origin(xyz=(x_pos, side_sign * 0.332, -0.700)),
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(x_pos, side_sign * 0.332, -0.703)),
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.940, 0.360, 0.740)),
        mass=1.15,
        origin=Origin(xyz=(0.0, side_sign * 0.175, -0.360)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_drying_rack", assets=ASSETS)

    deck = model.part("rack_deck")

    deck_top_z = 0.800
    side_y = 0.255
    half_length = 0.490

    _add_tube(
        deck, (-half_length, -side_y, deck_top_z), (half_length, -side_y, deck_top_z), radius=0.0075
    )
    _add_tube(
        deck, (-half_length, side_y, deck_top_z), (half_length, side_y, deck_top_z), radius=0.0075
    )
    _add_tube(
        deck, (-half_length, -side_y, deck_top_z), (-half_length, side_y, deck_top_z), radius=0.0065
    )
    _add_tube(
        deck, (half_length, -side_y, deck_top_z), (half_length, side_y, deck_top_z), radius=0.0065
    )

    for x_pos in (
        -0.405,
        -0.325,
        -0.245,
        -0.165,
        -0.085,
        -0.005,
        0.075,
        0.155,
        0.235,
        0.315,
        0.395,
    ):
        _add_tube(deck, (x_pos, -0.248, deck_top_z), (x_pos, 0.248, deck_top_z), radius=0.0054)

    for x_pos in (-0.360, -0.120, 0.120, 0.360):
        deck.visual(
            Box((0.020, 0.018, 0.030)),
            origin=Origin(xyz=(x_pos, -0.255, deck_top_z - 0.011)),
        )
        deck.visual(
            Box((0.020, 0.018, 0.030)),
            origin=Origin(xyz=(x_pos, 0.255, deck_top_z - 0.011)),
        )

    deck.inertial = Inertial.from_geometry(
        Box((1.040, 0.560, 0.100)),
        mass=3.25,
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
    )

    _build_support_frame(model, "left_support", side_sign=-1.0)
    _build_support_frame(model, "right_support", side_sign=1.0)

    model.articulation(
        "left_support_fold",
        ArticulationType.REVOLUTE,
        parent="rack_deck",
        child="left_support",
        origin=Origin(xyz=(0.0, -0.250, 0.792)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-0.48,
            upper=0.0,
        ),
    )
    model.articulation(
        "right_support_fold",
        ArticulationType.REVOLUTE,
        parent="rack_deck",
        child="right_support",
        origin=Origin(xyz=(0.0, 0.250, 0.792)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-0.48,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "left_support",
        "rack_deck",
        reason="hinge-adjacent support hardware sits close to the deck and collision hulls are conservative",
    )
    ctx.allow_overlap(
        "right_support",
        "rack_deck",
        reason="hinge-adjacent support hardware sits close to the deck and collision hulls are conservative",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("left_support", "rack_deck", axes="xy", max_dist=0.32)
    ctx.expect_origin_distance("right_support", "rack_deck", axes="xy", max_dist=0.32)
    ctx.expect_origin_distance("left_support", "right_support", axes="xy", max_dist=0.72)

    ctx.expect_joint_motion_axis(
        "left_support_fold",
        "left_support",
        world_axis="y",
        direction="negative",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "left_support_fold",
        "left_support",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "right_support_fold",
        "right_support",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "right_support_fold",
        "right_support",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(left_support_fold=-0.48, right_support_fold=-0.48):
        ctx.expect_origin_distance("left_support", "rack_deck", axes="xy", max_dist=0.28)
        ctx.expect_origin_distance("right_support", "rack_deck", axes="xy", max_dist=0.28)
        ctx.expect_origin_distance("left_support", "right_support", axes="xy", max_dist=0.56)
        ctx.expect_aabb_overlap("left_support", "rack_deck", axes="xy", min_overlap=0.33)
        ctx.expect_aabb_overlap("right_support", "rack_deck", axes="xy", min_overlap=0.33)

    with ctx.pose(left_support_fold=0.0, right_support_fold=0.0):
        ctx.expect_origin_distance("left_support", "right_support", axes="xy", max_dist=0.72)
        ctx.expect_aabb_overlap("left_support", "rack_deck", axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap("right_support", "rack_deck", axes="xy", min_overlap=0.015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
