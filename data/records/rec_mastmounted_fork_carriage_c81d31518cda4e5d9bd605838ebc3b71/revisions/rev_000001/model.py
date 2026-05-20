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


MAST_BASE_WIDTH = 0.66
MAST_BASE_DEPTH = 0.24
MAST_BASE_HEIGHT = 0.10

RAIL_CENTER_X = 0.19
RAIL_Y = -0.025
RAIL_WIDTH = 0.09
RAIL_DEPTH = 0.08
RAIL_HEIGHT = 1.30
RAIL_BASE_Z = MAST_BASE_HEIGHT

GUIDE_STRIP_THICKNESS = 0.006
GUIDE_STRIP_DEPTH = 0.084
GUIDE_STRIP_HEIGHT = 1.08
GUIDE_STRIP_CENTER_Z = 0.68

LIFT_HOME_Z = 0.11
LIFT_TRAVEL = 0.34

POCKET_WIDTH = 0.12
POCKET_DEPTH = 0.114
POCKET_HEIGHT = 0.58
POCKET_CENTER_Z = 0.34
POCKET_BOTTOM_Z = POCKET_CENTER_Z - (POCKET_HEIGHT / 2.0)

POCKET_CAVITY_WIDTH = 0.102
POCKET_CAVITY_DEPTH = 0.082
POCKET_CAVITY_HEIGHT = 0.50
POCKET_CAVITY_CENTER_Z = POCKET_BOTTOM_Z + 0.04 + (POCKET_CAVITY_HEIGHT / 2.0)

LOWER_BEAM_WIDTH = 0.32
LOWER_BEAM_DEPTH = 0.074
LOWER_BEAM_HEIGHT = 0.10
LOWER_BEAM_CENTER_Y = 0.012
LOWER_BEAM_CENTER_Z = 0.14

UPPER_BEAM_WIDTH = 0.32
UPPER_BEAM_DEPTH = 0.056
UPPER_BEAM_HEIGHT = 0.09
UPPER_BEAM_CENTER_Y = 0.010
UPPER_BEAM_CENTER_Z = 0.495

BACKREST_WIDTH = 0.40
BACKREST_HEIGHT = 0.30
BACKREST_THICKNESS = 0.012
BACKREST_CENTER_Y = 0.028
BACKREST_CENTER_Z = 0.69

FORK_WIDTH = 0.08
FORK_CENTER_X = 0.12
FORK_TIP_Y = 0.39


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_mast_main() -> cq.Workplane:
    base = _box(
        (MAST_BASE_WIDTH, MAST_BASE_DEPTH, MAST_BASE_HEIGHT),
        (0.0, -0.11, MAST_BASE_HEIGHT / 2.0),
    ).edges("|Z").fillet(0.010)

    left_foot = _box(
        (0.12, 0.12, 0.16),
        (RAIL_CENTER_X, -0.03, MAST_BASE_HEIGHT + 0.08),
    ).edges("|Z").fillet(0.006)
    right_foot = _box(
        (0.12, 0.12, 0.16),
        (-RAIL_CENTER_X, -0.03, MAST_BASE_HEIGHT + 0.08),
    ).edges("|Z").fillet(0.006)

    left_rail = _box(
        (RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT),
        (RAIL_CENTER_X, RAIL_Y, RAIL_BASE_Z + (RAIL_HEIGHT / 2.0)),
    ).edges("|Z").fillet(0.004)
    right_rail = _box(
        (RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT),
        (-RAIL_CENTER_X, RAIL_Y, RAIL_BASE_Z + (RAIL_HEIGHT / 2.0)),
    ).edges("|Z").fillet(0.004)

    rear_spine = _box((0.46, 0.02, 0.46), (0.0, -0.10, 0.33))
    mid_tie = _box((0.34, 0.05, 0.06), (0.0, -0.06, 0.49)).edges("|Z").fillet(0.004)
    top_header = _box((0.42, 0.07, 0.07), (0.0, -0.03, 1.325)).edges("|Z").fillet(0.006)

    left_stop = _box((0.03, 0.025, 0.05), (RAIL_CENTER_X, 0.002, 1.23))
    right_stop = _box((0.03, 0.025, 0.05), (-RAIL_CENTER_X, 0.002, 1.23))

    return (
        base.union(left_foot)
        .union(right_foot)
        .union(left_rail)
        .union(right_rail)
        .union(rear_spine)
        .union(mid_tie)
        .union(top_header)
        .union(left_stop)
        .union(right_stop)
    )


def _roller_pocket_cutter(x_outer: float, z_center: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x_outer, 0.0, 0.0))
        .center(RAIL_Y, z_center)
        .circle(0.016)
        .extrude(depth)
    )


def _build_carriage_frame() -> cq.Workplane:
    shoe_width = 0.050
    shoe_depth = 0.040
    shoe_height = 0.52
    shoe_center_x = 0.112
    shoe_center_y = 0.056
    shoe_center_z = 0.45

    left_shoe = _box((shoe_width, shoe_depth, shoe_height), (shoe_center_x, shoe_center_y, shoe_center_z))
    right_shoe = _box((shoe_width, shoe_depth, shoe_height), (-shoe_center_x, shoe_center_y, shoe_center_z))

    pocket_cut = (0.012, 0.022, 0.080)
    for z_center in (0.30, 0.58):
        left_shoe = left_shoe.cut(
            _box(
                pocket_cut,
                (shoe_center_x + (shoe_width / 2.0) - (pocket_cut[0] / 2.0), shoe_center_y + 0.005, z_center),
            )
        )
        right_shoe = right_shoe.cut(
            _box(
                pocket_cut,
                (-shoe_center_x - (shoe_width / 2.0) + (pocket_cut[0] / 2.0), shoe_center_y + 0.005, z_center),
            )
        )

    lower_beam = _box((0.40, 0.060, LOWER_BEAM_HEIGHT), (0.0, 0.082, LOWER_BEAM_CENTER_Z))
    upper_beam = _box((0.38, 0.052, UPPER_BEAM_HEIGHT), (0.0, 0.074, UPPER_BEAM_CENTER_Z))

    left_ear = _box((0.060, 0.030, 0.20), (0.150, 0.066, 0.30))
    right_ear = _box((0.060, 0.030, 0.20), (-0.150, 0.066, 0.30))
    left_upper_ear = _box((0.050, 0.028, 0.14), (0.146, 0.062, 0.60))
    right_upper_ear = _box((0.050, 0.028, 0.14), (-0.146, 0.062, 0.60))

    fork_root_w = 0.11
    fork_root_d = 0.074
    fork_root_h = 0.055
    left_root = _box((fork_root_w, fork_root_d, fork_root_h), (-FORK_CENTER_X, 0.088, 0.16))
    right_root = _box((fork_root_w, fork_root_d, fork_root_h), (FORK_CENTER_X, 0.088, 0.16))
    left_root = left_root.cut(_box((0.048, 0.028, 0.032), (-FORK_CENTER_X, 0.102, 0.178)))
    right_root = right_root.cut(_box((0.048, 0.028, 0.032), (FORK_CENTER_X, 0.102, 0.178)))

    side_post_w = 0.018
    side_post_h = BACKREST_HEIGHT
    post_x = (BACKREST_WIDTH / 2.0) - (side_post_w / 2.0)
    panel_y = 0.078
    carriage = (
        left_shoe.union(right_shoe)
        .union(lower_beam)
        .union(upper_beam)
        .union(left_ear)
        .union(right_ear)
        .union(left_upper_ear)
        .union(right_upper_ear)
        .union(left_root)
        .union(right_root)
        .union(_box((side_post_w, BACKREST_THICKNESS, side_post_h), (post_x, panel_y, BACKREST_CENTER_Z)))
        .union(_box((side_post_w, BACKREST_THICKNESS, side_post_h), (-post_x, panel_y, BACKREST_CENTER_Z)))
    )

    frame_bar_h = 0.018
    carriage = carriage.union(
        _box(
            (BACKREST_WIDTH - 0.04, BACKREST_THICKNESS, frame_bar_h),
            (0.0, panel_y, BACKREST_CENTER_Z - (BACKREST_HEIGHT / 2.0) + (frame_bar_h / 2.0)),
        )
    )
    carriage = carriage.union(
        _box(
            (BACKREST_WIDTH - 0.04, BACKREST_THICKNESS, frame_bar_h),
            (0.0, panel_y, BACKREST_CENTER_Z + (BACKREST_HEIGHT / 2.0) - (frame_bar_h / 2.0)),
        )
    )

    for x_center in (-0.12, -0.04, 0.04, 0.12):
        carriage = carriage.union(_box((0.010, BACKREST_THICKNESS, 0.24), (x_center, panel_y, 0.69)))
    for z_center in (0.60, 0.69, 0.78):
        carriage = carriage.union(_box((0.30, BACKREST_THICKNESS, 0.010), (0.0, panel_y, z_center)))

    return carriage


def _build_fork(center_x: float) -> cq.Workplane:
    root_y = 0.03
    profile = [
        (root_y, 0.00),
        (root_y + 0.34, 0.00),
        (root_y + FORK_TIP_Y, 0.008),
        (root_y + FORK_TIP_Y, 0.032),
        (root_y + 0.07, 0.032),
        (root_y + 0.07, 0.105),
        (root_y + 0.045, 0.125),
        (root_y + 0.028, 0.150),
        (root_y, 0.150),
    ]
    return (
        cq.Workplane("YZ", origin=(center_x - (FORK_WIDTH / 2.0), 0.0, 0.0))
        .polyline(profile)
        .close()
        .extrude(FORK_WIDTH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacker_mast")

    model.material("mast_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("carriage_orange", rgba=(0.92, 0.48, 0.12, 1.0))
    model.material("fork_steel", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("wear_polymer", rgba=(0.09, 0.09, 0.10, 1.0))

    mast = model.part("mast")
    mast.visual(mesh_from_cadquery(_build_mast_main(), "mast_main"), material="mast_steel", name="mast_main")
    mast.visual(
        Box((GUIDE_STRIP_THICKNESS, GUIDE_STRIP_DEPTH, GUIDE_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                RAIL_CENTER_X - (RAIL_WIDTH / 2.0) + (GUIDE_STRIP_THICKNESS / 2.0),
                RAIL_Y,
                GUIDE_STRIP_CENTER_Z,
            )
        ),
        material="wear_polymer",
        name="left_guide_strip",
    )
    mast.visual(
        Box((GUIDE_STRIP_THICKNESS, GUIDE_STRIP_DEPTH, GUIDE_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                -RAIL_CENTER_X + (RAIL_WIDTH / 2.0) - (GUIDE_STRIP_THICKNESS / 2.0),
                RAIL_Y,
                GUIDE_STRIP_CENTER_Z,
            )
        ),
        material="wear_polymer",
        name="right_guide_strip",
    )
    mast.inertial = Inertial.from_geometry(
        Box((MAST_BASE_WIDTH, MAST_BASE_DEPTH, RAIL_BASE_Z + RAIL_HEIGHT)),
        mass=110.0,
        origin=Origin(xyz=(0.0, -0.08, (RAIL_BASE_Z + RAIL_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_frame(), "carriage_frame"),
        material="carriage_orange",
        name="carriage_frame",
    )
    carriage.visual(
        Box((0.018, 0.022, 0.42)),
        origin=Origin(xyz=(0.141, 0.028, 0.45)),
        material="wear_polymer",
        name="left_guide_pad",
    )
    carriage.visual(
        Box((0.018, 0.022, 0.42)),
        origin=Origin(xyz=(-0.141, 0.028, 0.45)),
        material="wear_polymer",
        name="right_guide_pad",
    )
    carriage.visual(
        mesh_from_cadquery(_build_fork(-FORK_CENTER_X), "left_fork"),
        material="fork_steel",
        name="left_fork",
    )
    carriage.visual(
        mesh_from_cadquery(_build_fork(FORK_CENTER_X), "right_fork"),
        material="fork_steel",
        name="right_fork",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.50, 0.46, 0.86)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.10, 0.43)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=1800.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_lift")

    left_strip = mast.get_visual("left_guide_strip")
    right_strip = mast.get_visual("right_guide_strip")
    left_pad = carriage.get_visual("left_guide_pad")
    right_pad = carriage.get_visual("right_guide_pad")
    left_fork = carriage.get_visual("left_fork")
    right_fork = carriage.get_visual("right_fork")

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

    limits = lift.motion_limits
    ctx.check(
        "lift_axis_is_vertical",
        tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        details=f"lift axis was {lift.axis}",
    )
    ctx.check(
        "lift_limits_match_single_stage_mast",
        limits is not None and limits.lower == 0.0 and 0.30 <= limits.upper <= 0.36,
        details=f"lift limits were {None if limits is None else (limits.lower, limits.upper)}",
    )

    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_pad,
        elem_b=left_strip,
        contact_tol=0.001,
        name="left_carriage_guide_stays_bearing_on_mast",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_pad,
        elem_b=right_strip,
        contact_tol=0.001,
        name="right_carriage_guide_stays_bearing_on_mast",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.10,
        name="carriage_stays_captured_between_twin_rails",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        min_overlap=0.40,
        name="stowed_carriage_reads_nested_in_mast_width",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        min_overlap=0.50,
        name="stowed_carriage_overlaps_mast_height_band",
    )
    left_fork_aabb = ctx.part_element_world_aabb(carriage, elem=left_fork)
    right_fork_aabb = ctx.part_element_world_aabb(carriage, elem=right_fork)
    fork_gap_ok = False
    fork_gap_details = "fork AABB lookup failed"
    if left_fork_aabb is not None and right_fork_aabb is not None:
        fork_gap = right_fork_aabb[0][0] - left_fork_aabb[1][0]
        fork_gap_ok = 0.14 <= fork_gap <= 0.18
        fork_gap_details = f"fork x-gap was {fork_gap:.4f} m"
    ctx.check("fork_pair_has_realistic_clear_spacing", fork_gap_ok, details=fork_gap_details)
    ctx.expect_overlap(
        carriage,
        carriage,
        axes="yz",
        min_overlap=0.03,
        elem_a=left_fork,
        elem_b=right_fork,
        name="fork_pair_runs_parallel_in_length_and_height",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({lift: limits.upper}):
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=left_pad,
                elem_b=left_strip,
                contact_tol=0.001,
                name="left_carriage_guide_keeps_contact_at_full_raise",
            )
            ctx.expect_contact(
                carriage,
                mast,
                elem_a=right_pad,
                elem_b=right_strip,
                contact_tol=0.001,
                name="right_carriage_guide_keeps_contact_at_full_raise",
            )
            ctx.expect_within(
                carriage,
                mast,
                axes="x",
                margin=0.10,
                name="carriage_remains_laterally_captured_at_full_raise",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
