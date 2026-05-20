from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.58
BASE_D = 0.14
BASE_H = 0.10

COLUMN_W = 0.06
COLUMN_D = 0.09
COLUMN_H = 1.52
COLUMN_WEB = 0.014
COLUMN_FLANGE = 0.014
COLUMN_X = 0.22

TOP_W = 0.52
TOP_D = 0.10
TOP_H = 0.08

CARRIAGE_W = 0.356
CARRIAGE_D = 0.05
CARRIAGE_H = 0.48
WINDOW_W = 0.22
WINDOW_Z0 = 0.10
WINDOW_Z1 = 0.38

GUIDE_CLEAR = 0.0
GUIDE_RAIL_X = 0.010
GUIDE_LINK_Y = COLUMN_D - 2.0 * COLUMN_FLANGE
GUIDE_LINK_X = 0.012
GUIDE_PAD_WEB_X = 0.008
GUIDE_PAD_LIP_Y = 0.008
GUIDE_LOWER_Z0 = 0.06
GUIDE_LOWER_Z1 = 0.16
GUIDE_UPPER_Z0 = 0.32
GUIDE_UPPER_Z1 = 0.42

CARRIAGE_LOWER_Z = 0.16
CARRIAGE_TRAVEL = 0.90


def box_between(
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        abs(x1 - x0),
        abs(y1 - y0),
        abs(z1 - z0),
    ).translate(
        (
            0.5 * (x0 + x1),
            0.5 * (y0 + y1),
            0.5 * (z0 + z1),
        )
    )


def c_channel_column(
    width: float,
    depth: float,
    height: float,
    *,
    open_positive_x: bool,
) -> cq.Workplane:
    outer = box_between(
        -0.5 * width,
        0.5 * width,
        -0.5 * depth,
        0.5 * depth,
        0.0,
        height,
    )
    if open_positive_x:
        cavity = box_between(
            -0.5 * width + COLUMN_WEB,
            0.5 * width + 0.002,
            -0.5 * depth + COLUMN_FLANGE,
            0.5 * depth - COLUMN_FLANGE,
            -0.002,
            height + 0.002,
        )
    else:
        cavity = box_between(
            -0.5 * width - 0.002,
            0.5 * width - COLUMN_WEB,
            -0.5 * depth + COLUMN_FLANGE,
            0.5 * depth - COLUMN_FLANGE,
            -0.002,
            height + 0.002,
        )
    return outer.cut(cavity)


def make_base_shape() -> cq.Workplane:
    crossbeam = box_between(
        -0.5 * BASE_W,
        0.5 * BASE_W,
        -0.5 * BASE_D,
        0.5 * BASE_D,
        0.0,
        BASE_H,
    )
    left_pad = box_between(
        -COLUMN_X - 0.065,
        -COLUMN_X + 0.065,
        -0.5 * BASE_D,
        0.5 * BASE_D,
        0.0,
        0.02,
    )
    right_pad = box_between(
        COLUMN_X - 0.065,
        COLUMN_X + 0.065,
        -0.5 * BASE_D,
        0.5 * BASE_D,
        0.0,
        0.02,
    )
    return crossbeam.union(left_pad).union(right_pad)


def make_left_column_shape() -> cq.Workplane:
    return c_channel_column(COLUMN_W, COLUMN_D, COLUMN_H, open_positive_x=True)


def make_right_column_shape() -> cq.Workplane:
    return c_channel_column(COLUMN_W, COLUMN_D, COLUMN_H, open_positive_x=False)


def make_top_crosshead_shape() -> cq.Workplane:
    beam = box_between(
        -0.5 * TOP_W,
        0.5 * TOP_W,
        -0.5 * TOP_D,
        0.5 * TOP_D,
        0.0,
        TOP_H,
    )
    lightening_cut = box_between(
        -0.12,
        0.12,
        -0.06,
        0.06,
        0.016,
        TOP_H - 0.016,
    )
    return beam.cut(lightening_cut)


def make_carriage_body_shape() -> cq.Workplane:
    outer_frame = box_between(
        -0.5 * CARRIAGE_W,
        0.5 * CARRIAGE_W,
        -0.5 * CARRIAGE_D,
        0.5 * CARRIAGE_D,
        0.0,
        CARRIAGE_H,
    )
    inner_window = box_between(
        -0.5 * WINDOW_W,
        0.5 * WINDOW_W,
        -0.04,
        0.04,
        WINDOW_Z0,
        WINDOW_Z1,
    )
    front_plate = box_between(-0.12, 0.12, 0.025, 0.035, 0.10, 0.44)
    lower_bar = box_between(-0.15, 0.15, 0.0, 0.06, 0.01, 0.08)
    return outer_frame.cut(inner_window).union(front_plate).union(lower_bar)


def add_box_visual(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_column_visuals(part, *, left_open: bool, material) -> None:
    lip_size_x = COLUMN_W - COLUMN_WEB
    web_x = -0.5 * COLUMN_W + 0.5 * COLUMN_WEB if left_open else 0.5 * COLUMN_W - 0.5 * COLUMN_WEB
    lip_x = 0.5 * COLUMN_W - 0.5 * lip_size_x if left_open else -0.5 * COLUMN_W + 0.5 * lip_size_x
    zc = 0.5 * COLUMN_H

    add_box_visual(part, "column_web", (COLUMN_WEB, COLUMN_D, COLUMN_H), (web_x, 0.0, zc), material)
    add_box_visual(
        part,
        "column_front_lip",
        (lip_size_x, COLUMN_FLANGE, COLUMN_H),
        (lip_x, 0.5 * COLUMN_D - 0.5 * COLUMN_FLANGE, zc),
        material,
    )
    add_box_visual(
        part,
        "column_rear_lip",
        (lip_size_x, COLUMN_FLANGE, COLUMN_H),
        (lip_x, -0.5 * COLUMN_D + 0.5 * COLUMN_FLANGE, zc),
        material,
    )


def add_guide_cluster(part, *, side: int, z0: float, z1: float, prefix: str, material) -> None:
    zc = 0.5 * (z0 + z1)
    zh = z1 - z0
    front_inner_y = 0.5 * COLUMN_D - COLUMN_FLANGE
    rear_inner_y = -0.5 * COLUMN_D + COLUMN_FLANGE

    if side < 0:
        web_pad_x0 = -COLUMN_X - 0.5 * COLUMN_W + COLUMN_WEB
        web_pad_x1 = web_pad_x0 + 0.008
        mid_pad_x0 = web_pad_x1
        mid_pad_x1 = -COLUMN_X + 0.5 * COLUMN_W - 0.012
        link_x0 = mid_pad_x1
        link_x1 = -0.5 * CARRIAGE_W + 0.002
    else:
        web_pad_x1 = COLUMN_X + 0.5 * COLUMN_W - COLUMN_WEB
        web_pad_x0 = web_pad_x1 - 0.008
        mid_pad_x0 = COLUMN_X - 0.5 * COLUMN_W + 0.012
        mid_pad_x1 = web_pad_x0
        link_x0 = 0.5 * CARRIAGE_W - 0.002
        link_x1 = mid_pad_x0

    add_box_visual(
        part,
        f"{prefix}_web_pad",
        (abs(web_pad_x1 - web_pad_x0), front_inner_y - rear_inner_y, zh),
        (0.5 * (web_pad_x0 + web_pad_x1), 0.0, zc),
        material,
    )
    add_box_visual(
        part,
        f"{prefix}_front_pad",
        (abs(mid_pad_x1 - mid_pad_x0), GUIDE_PAD_LIP_Y, zh),
        (0.5 * (mid_pad_x0 + mid_pad_x1), front_inner_y - 0.5 * GUIDE_PAD_LIP_Y, zc),
        material,
    )
    add_box_visual(
        part,
        f"{prefix}_rear_pad",
        (abs(mid_pad_x1 - mid_pad_x0), GUIDE_PAD_LIP_Y, zh),
        (0.5 * (mid_pad_x0 + mid_pad_x1), rear_inner_y + 0.5 * GUIDE_PAD_LIP_Y, zc),
        material,
    )
    add_box_visual(
        part,
        f"{prefix}_link",
        (abs(link_x1 - link_x0), GUIDE_LINK_Y, zh),
        (0.5 * (link_x0 + link_x1), 0.0, zc),
        material,
    )


def add_carriage_guides(part, material) -> None:
    add_guide_cluster(part, side=-1, z0=GUIDE_LOWER_Z0, z1=GUIDE_LOWER_Z1, prefix="left_lower_guide", material=material)
    add_guide_cluster(part, side=-1, z0=GUIDE_UPPER_Z0, z1=GUIDE_UPPER_Z1, prefix="left_upper_guide", material=material)
    add_guide_cluster(part, side=1, z0=GUIDE_LOWER_Z0, z1=GUIDE_LOWER_Z1, prefix="right_lower_guide", material=material)
    add_guide_cluster(part, side=1, z0=GUIDE_UPPER_Z0, z1=GUIDE_UPPER_Z1, prefix="right_upper_guide", material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_module")

    frame_steel = model.material("frame_steel", rgba=(0.20, 0.24, 0.29, 1.0))
    carriage_steel = model.material("carriage_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    guide_polymer = model.material("guide_polymer", rgba=(0.78, 0.66, 0.40, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(make_base_shape(), "base_frame"),
        material=frame_steel,
        name="base_body",
    )

    left_column = model.part("left_column")
    add_column_visuals(left_column, left_open=True, material=frame_steel)

    right_column = model.part("right_column")
    add_column_visuals(right_column, left_open=False, material=frame_steel)

    top_crosshead = model.part("top_crosshead")
    top_crosshead.visual(
        mesh_from_cadquery(make_top_crosshead_shape(), "top_crosshead"),
        material=frame_steel,
        name="top_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_body_shape(), "carriage_body"),
        material=carriage_steel,
        name="carriage_body",
    )
    add_carriage_guides(carriage, guide_polymer)

    model.articulation(
        "base_to_left_column",
        ArticulationType.FIXED,
        parent=base_frame,
        child=left_column,
        origin=Origin(xyz=(-COLUMN_X, 0.0, BASE_H)),
    )
    model.articulation(
        "base_to_right_column",
        ArticulationType.FIXED,
        parent=base_frame,
        child=right_column,
        origin=Origin(xyz=(COLUMN_X, 0.0, BASE_H)),
    )
    model.articulation(
        "left_column_to_top_crosshead",
        ArticulationType.FIXED,
        parent=left_column,
        child=top_crosshead,
        origin=Origin(xyz=(COLUMN_X, 0.0, COLUMN_H)),
    )
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=0.25,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    left_column = object_model.get_part("left_column")
    right_column = object_model.get_part("right_column")
    top_crosshead = object_model.get_part("top_crosshead")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("base_to_carriage")

    base_frame.get_visual("base_body")
    left_column.get_visual("column_web")
    left_column.get_visual("column_front_lip")
    left_column.get_visual("column_rear_lip")
    right_column.get_visual("column_web")
    right_column.get_visual("column_front_lip")
    right_column.get_visual("column_rear_lip")
    top_crosshead.get_visual("top_body")
    carriage.get_visual("carriage_body")
    carriage.get_visual("left_lower_guide_web_pad")
    carriage.get_visual("left_upper_guide_link")
    carriage.get_visual("right_lower_guide_web_pad")
    carriage.get_visual("right_upper_guide_link")

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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=12,
        name="carriage_travel_clearance_sweep",
    )

    ctx.expect_contact(
        left_column,
        base_frame,
        elem_b="base_body",
        name="left_column_seated_on_base",
    )
    ctx.expect_contact(
        right_column,
        base_frame,
        elem_b="base_body",
        name="right_column_seated_on_base",
    )
    ctx.expect_contact(
        top_crosshead,
        left_column,
        elem_a="top_body",
        name="top_crosshead_touches_left_column",
    )
    ctx.expect_contact(
        top_crosshead,
        right_column,
        elem_a="top_body",
        name="top_crosshead_touches_right_column",
    )
    ctx.expect_origin_distance(
        left_column,
        right_column,
        axes="x",
        min_dist=0.42,
        max_dist=0.46,
        name="column_center_spacing",
    )
    ctx.expect_gap(
        right_column,
        left_column,
        axis="x",
        min_gap=0.36,
        max_gap=0.40,
        name="clear_span_between_columns",
    )

    axis = tuple(float(v) for v in lift.axis)
    ctx.check(
        "lift_axis_vertical",
        axis == (0.0, 0.0, 1.0),
        f"expected vertical prismatic axis, got {axis}",
    )
    limits = lift.motion_limits
    ctx.check(
        "lift_limits_defined",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.85 <= limits.upper <= 0.95,
        f"unexpected prismatic limits: {limits}",
    )

    frame_aabbs = [
        ctx.part_world_aabb(base_frame),
        ctx.part_world_aabb(left_column),
        ctx.part_world_aabb(right_column),
        ctx.part_world_aabb(top_crosshead),
    ]
    if all(aabb is not None for aabb in frame_aabbs):
        mins = [min(aabb[0][i] for aabb in frame_aabbs) for i in range(3)]
        maxs = [max(aabb[1][i] for aabb in frame_aabbs) for i in range(3)]
        overall_width = maxs[0] - mins[0]
        overall_height = maxs[2] - mins[2]
        ctx.check(
            "frame_width_realistic",
            0.50 <= overall_width <= 0.65,
            f"frame width {overall_width:.3f} m outside realistic compact stacker range",
        )
        ctx.check(
            "frame_height_realistic",
            1.60 <= overall_height <= 1.85,
            f"frame height {overall_height:.3f} m outside realistic compact stacker range",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lift: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lift_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lift_lower_no_floating")
            ctx.expect_contact(
                carriage,
                left_column,
                name="left_guides_contact_left_column_lower",
            )
            ctx.expect_contact(
                carriage,
                right_column,
                name="right_guides_contact_right_column_lower",
            )
            ctx.expect_overlap(
                carriage,
                left_column,
                axes="z",
                min_overlap=0.20,
                name="left_guides_track_column_lower",
            )
            ctx.expect_overlap(
                carriage,
                right_column,
                axes="z",
                min_overlap=0.20,
                name="right_guides_track_column_lower",
            )
            ctx.expect_gap(
                carriage,
                base_frame,
                axis="z",
                min_gap=0.05,
                positive_elem="carriage_body",
                negative_elem="base_body",
                name="carriage_clears_base_at_lower_pose",
            )
        with ctx.pose({lift: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lift_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lift_upper_no_floating")
            ctx.expect_contact(
                carriage,
                left_column,
                name="left_guides_contact_left_column_upper",
            )
            ctx.expect_contact(
                carriage,
                right_column,
                name="right_guides_contact_right_column_upper",
            )
            ctx.expect_overlap(
                carriage,
                left_column,
                axes="z",
                min_overlap=0.20,
                name="left_guides_track_column_upper",
            )
            ctx.expect_overlap(
                carriage,
                right_column,
                axes="z",
                min_overlap=0.20,
                name="right_guides_track_column_upper",
            )
            ctx.expect_gap(
                top_crosshead,
                carriage,
                axis="z",
                min_gap=0.05,
                positive_elem="top_body",
                negative_elem="carriage_body",
                name="carriage_clears_top_crosshead_at_upper_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
