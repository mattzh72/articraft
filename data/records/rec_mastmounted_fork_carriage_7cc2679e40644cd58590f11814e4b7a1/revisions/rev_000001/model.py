from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(length: float, depth: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, depth, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _upright_channel(
    width: float,
    depth: float,
    height: float,
    wall: float,
    *,
    opening_sign: float,
    x: float,
    y: float,
    z: float,
) -> cq.Workplane:
    outer = _box(width, depth, height)
    inner = _box(
        width - wall,
        depth - 2.0 * wall,
        height + 0.002,
        x=opening_sign * (wall * 0.5),
        z=-0.001,
    )
    return outer.cut(inner).translate((x, y, z))


def _fork_solid(*, x: float, y: float) -> cq.Workplane:
    profile = [
        (-0.032, 0.036),
        (0.000, 0.036),
        (0.320, 0.036),
        (0.355, 0.048),
        (0.355, 0.068),
        (0.000, 0.068),
        (0.000, 0.182),
        (-0.032, 0.182),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(0.090, both=True)
        .translate((x, y, 0.0))
    )


def _guide_shoe(
    *,
    rail_x: float,
    rail_y: float,
    rail_width: float,
    rail_depth: float,
    shoe_height: float,
    z: float,
    side_sign: float,
) -> cq.Workplane:
    rail_front = rail_y + rail_depth * 0.5
    rail_outer_x = rail_x + side_sign * (rail_width * 0.5)

    side_gap = 0.0
    front_gap = 0.0

    side_pad_width = 0.018
    side_pad_depth = 0.026
    front_pad_width = rail_width + 0.014
    front_pad_depth = 0.016
    rear_anchor_width = 0.020
    rear_anchor_depth = 0.044

    side_pad_center_x = rail_outer_x + side_sign * (side_gap + side_pad_width * 0.5)
    side_pad_center_y = rail_front + front_gap + side_pad_depth * 0.5
    front_pad_center_y = rail_front + front_gap + front_pad_depth * 0.5

    side_pad = _box(
        side_pad_width,
        side_pad_depth,
        shoe_height,
        x=side_pad_center_x,
        y=side_pad_center_y,
        z=z,
    )
    front_pad = _box(
        front_pad_width,
        front_pad_depth,
        shoe_height,
        x=rail_x,
        y=front_pad_center_y,
        z=z,
    )
    rear_anchor = _box(
        rear_anchor_width,
        rear_anchor_depth,
        shoe_height,
        x=side_pad_center_x,
        y=0.030,
        z=z,
    )

    return side_pad.union(front_pad).union(rear_anchor)


def _make_mast_frame(
    *,
    rail_x: float,
    rail_y: float,
    rail_width: float,
    rail_depth: float,
    rail_height: float,
) -> cq.Workplane:
    left_rail = _box(rail_width, rail_depth, rail_height, x=-rail_x, y=rail_y, z=0.0)
    right_rail = _box(rail_width, rail_depth, rail_height, x=rail_x, y=rail_y, z=0.0)

    tie_length = 2.0 * rail_x + rail_width
    bottom_tie = _box(tie_length, 0.044, 0.060, x=0.0, y=rail_y, z=0.025)
    top_tie = _box(tie_length, 0.044, 0.070, x=0.0, y=rail_y, z=rail_height - 0.055)

    return left_rail.union(right_rail).union(bottom_tie).union(top_tie)


def _make_carriage_frame(*, rail_x: float, frame_y: float) -> cq.Workplane:
    channel_width = 0.070
    channel_depth = 0.085
    channel_height = 0.440
    channel_wall = 0.008

    left_channel = _upright_channel(
        channel_width,
        channel_depth,
        channel_height,
        channel_wall,
        opening_sign=1.0,
        x=-0.230,
        y=frame_y,
        z=0.0,
    )
    right_channel = _upright_channel(
        channel_width,
        channel_depth,
        channel_height,
        channel_wall,
        opening_sign=-1.0,
        x=0.230,
        y=frame_y,
        z=0.0,
    )

    top_crosshead = _box(0.460, 0.052, 0.060, x=0.0, y=frame_y, z=0.355)
    lower_crossbar = _box(0.470, 0.056, 0.062, x=0.0, y=0.060, z=0.095)

    backrest_left = _box(0.024, 0.018, 0.360, x=-0.120, y=0.034, z=0.420)
    backrest_right = _box(0.024, 0.018, 0.360, x=0.120, y=0.034, z=0.420)
    slat_1 = _box(0.325, 0.016, 0.018, x=0.0, y=0.034, z=0.490)
    slat_2 = _box(0.325, 0.016, 0.018, x=0.0, y=0.034, z=0.585)
    slat_3 = _box(0.325, 0.016, 0.018, x=0.0, y=0.034, z=0.680)
    slat_4 = _box(0.325, 0.016, 0.018, x=0.0, y=0.034, z=0.775)

    frame = left_channel.union(right_channel)
    for solid in (
        top_crosshead,
        lower_crossbar,
        backrest_left,
        backrest_right,
        slat_1,
        slat_2,
        slat_3,
        slat_4,
    ):
        frame = frame.union(solid)

    return frame


def _make_forks() -> cq.Workplane:
    left_fork = _fork_solid(x=-0.135, y=0.102)
    right_fork = _fork_solid(x=0.135, y=0.102)
    return left_fork.union(right_fork)


def _make_guide_cluster(*, rail_x: float, rail_y: float, rail_width: float, rail_depth: float) -> cq.Workplane:
    shoes = [
        _guide_shoe(
            rail_x=-rail_x,
            rail_y=rail_y,
            rail_width=rail_width,
            rail_depth=rail_depth,
            shoe_height=0.090,
            z=0.075,
            side_sign=-1.0,
        ),
        _guide_shoe(
            rail_x=-rail_x,
            rail_y=rail_y,
            rail_width=rail_width,
            rail_depth=rail_depth,
            shoe_height=0.090,
            z=0.285,
            side_sign=-1.0,
        ),
        _guide_shoe(
            rail_x=rail_x,
            rail_y=rail_y,
            rail_width=rail_width,
            rail_depth=rail_depth,
            shoe_height=0.090,
            z=0.075,
            side_sign=1.0,
        ),
        _guide_shoe(
            rail_x=rail_x,
            rail_y=rail_y,
            rail_width=rail_width,
            rail_depth=rail_depth,
            shoe_height=0.090,
            z=0.285,
            side_sign=1.0,
        ),
    ]

    cluster = shoes[0]
    for shoe in shoes[1:]:
        cluster = cluster.union(shoe)
    return cluster


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_carriage_module")

    rail_x = 0.190
    rail_y = -0.025
    rail_width = 0.065
    rail_depth = 0.050
    rail_height = 1.520
    frame_y = 0.055

    model.material("mast_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("carriage_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("fork_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("shoe_black", rgba=(0.08, 0.08, 0.08, 1.0))

    mast = model.part("mast")
    mast_frame = _make_mast_frame(
        rail_x=rail_x,
        rail_y=rail_y,
        rail_width=rail_width,
        rail_depth=rail_depth,
        rail_height=rail_height,
    )
    mast.visual(
        mesh_from_cadquery(mast_frame, "mast_frame"),
        material="mast_gray",
        name="mast_frame",
    )
    mast.inertial = Inertial.from_geometry(
        Box((2.0 * rail_x + rail_width, rail_depth, rail_height)),
        mass=110.0,
        origin=Origin(xyz=(0.0, rail_y, rail_height * 0.5)),
    )

    carriage = model.part("carriage")
    carriage_frame = _make_carriage_frame(rail_x=rail_x, frame_y=frame_y)
    forks = _make_forks()
    guide_cluster = _make_guide_cluster(
        rail_x=rail_x,
        rail_y=rail_y,
        rail_width=rail_width,
        rail_depth=rail_depth,
    )

    carriage.visual(
        mesh_from_cadquery(carriage_frame, "carriage_frame"),
        material="carriage_gray",
        name="carriage_frame",
    )
    carriage.visual(
        mesh_from_cadquery(forks, "forks"),
        material="fork_black",
        name="forks",
    )
    carriage.visual(
        mesh_from_cadquery(guide_cluster, "guide_cluster"),
        material="shoe_black",
        name="guide_cluster",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.550, 0.520, 0.800)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.100, 0.400)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.40,
            lower=0.0,
            upper=0.560,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("mast_to_carriage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = slide.motion_limits
    ctx.check(
        "mast_to_carriage_is_vertical_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == 0.560,
        "Expected one vertical prismatic carriage joint with 0.56 m travel.",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_origin_distance(
            carriage,
            mast,
            axes="x",
            max_dist=0.001,
            name="carriage_centered_on_mast",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem="carriage_frame",
            negative_elem="mast_frame",
            min_gap=0.010,
            max_gap=0.020,
            name="carriage_frame_sits_ahead_of_mast",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="xz",
            elem_a="guide_cluster",
            elem_b="mast_frame",
            min_overlap=0.050,
            name="guide_cluster_overlaps_rail_zone",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem="guide_cluster",
            negative_elem="mast_frame",
            min_gap=0.0,
            max_gap=0.001,
            name="guide_cluster_runs_tight_to_mast",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="guide_cluster",
            elem_b="mast_frame",
            contact_tol=0.001,
            name="guide_cluster_contacts_mast",
        )

    with ctx.pose({slide: 0.0}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.560}):
        upper_pos = ctx.part_world_position(carriage)
        ctx.expect_origin_distance(
            carriage,
            mast,
            axes="x",
            max_dist=0.001,
            name="carriage_stays_centered_at_full_raise",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem="carriage_frame",
            negative_elem="mast_frame",
            min_gap=0.010,
            max_gap=0.020,
            name="carriage_frame_clears_mast_at_full_raise",
        )

    travel_ok = (
        lower_pos is not None
        and upper_pos is not None
        and abs(upper_pos[0] - lower_pos[0]) <= 1e-6
        and abs(upper_pos[1] - lower_pos[1]) <= 1e-6
        and 0.559 <= (upper_pos[2] - lower_pos[2]) <= 0.561
    )
    ctx.check(
        "carriage_travel_matches_joint_limit",
        travel_ok,
        f"Expected ~0.56 m vertical travel, got lower={lower_pos}, upper={upper_pos}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
