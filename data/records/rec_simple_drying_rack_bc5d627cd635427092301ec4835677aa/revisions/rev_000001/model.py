from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


CENTER_WIDTH = 0.62
WING_WIDTH = 0.30
DEPTH = 0.48
TOP_Z = 0.88
TUBE = 0.018
LEG_TUBE = 0.022
RAIL_RADIUS = 0.005
HINGE_CLEAR = 0.004
FRAME_Z = TOP_Z - TUBE * 0.5
WING_OPEN_LIMIT = 1.10


def _box(part, size, xyz, material, *, name=None):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _rod_y(part, radius, length, xyz, material, *, name=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi * 0.5, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_hinge_link(model: ArticulatedObject, name: str, side_sign: float, bracket_mat, mark_mat) -> None:
    link = model.part(name)
    link.inertial = Inertial.from_geometry(
        Box((0.06, 0.18, 0.10)),
        mass=0.25,
        origin=Origin(xyz=(side_sign * 0.02, 0.0, -0.035)),
    )

    _box(
        link,
        (0.020, 0.140, 0.022),
        (side_sign * 0.010, 0.0, -0.020),
        bracket_mat,
        name="mount_block",
    )
    _box(
        link,
        (0.002, 0.060, 0.018),
        (side_sign * 0.001, 0.0, -0.001),
        bracket_mat,
        name="axis_bridge",
    )
    _rod_y(
        link,
        0.002,
        0.120,
        (side_sign * 0.002, 0.0, 0.0),
        mark_mat,
        name="hinge_pin",
    )
    _box(
        link,
        (0.022, 0.160, 0.072),
        (side_sign * 0.031, 0.0, -0.045),
        bracket_mat,
        name="scale_plate",
    )

    for idx, z in enumerate((-0.067, -0.044, -0.019), start=1):
        _box(
            link,
            (0.002, 0.052, 0.004),
            (side_sign * 0.043, 0.0, z),
            mark_mat,
            name=f"index_mark_{idx}",
        )

    link.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(side_sign * 0.047, 0.0, -0.060), rpy=(0.0, pi * 0.5, 0.0)),
        material=mark_mat,
        name="adjuster_head",
    )


def _build_wing(model: ArticulatedObject, name: str, side_sign: float, frame_mat, mark_mat) -> None:
    wing = model.part(name)
    wing.inertial = Inertial.from_geometry(
        Box((WING_WIDTH, DEPTH, 0.10)),
        mass=0.80,
        origin=Origin(xyz=(side_sign * (HINGE_CLEAR + WING_WIDTH * 0.5), 0.0, 0.0)),
    )

    root_x = side_sign * (HINGE_CLEAR + TUBE * 0.5)
    outer_x = side_sign * (HINGE_CLEAR + WING_WIDTH - TUBE * 0.5)
    rail_span_center = side_sign * (HINGE_CLEAR + WING_WIDTH * 0.5)
    rail_length_x = WING_WIDTH - TUBE

    _box(wing, (TUBE, DEPTH, TUBE), (root_x, 0.0, 0.0), frame_mat, name="root_rail")
    _box(
        wing,
        (0.002, 0.140, 0.020),
        (side_sign * (HINGE_CLEAR + 0.001), 0.0, 0.0),
        mark_mat,
        name="inner_datum",
    )
    _box(wing, (TUBE, DEPTH, TUBE), (outer_x, 0.0, 0.0), frame_mat, name="outer_rail")
    _box(
        wing,
        (rail_length_x, TUBE, TUBE),
        (rail_span_center, DEPTH * 0.5 - TUBE * 0.5, 0.0),
        frame_mat,
        name="front_rail",
    )
    _box(
        wing,
        (rail_length_x, TUBE, TUBE),
        (rail_span_center, -DEPTH * 0.5 + TUBE * 0.5, 0.0),
        frame_mat,
        name="rear_rail",
    )

    usable_width = WING_WIDTH - 2.0 * TUBE
    for idx in range(3):
        x = side_sign * (HINGE_CLEAR + TUBE + usable_width * (idx + 1) / 4.0)
        _rod_y(
            wing,
            RAIL_RADIUS,
            DEPTH - 2.0 * TUBE,
            (x, 0.0, 0.0),
            frame_mat,
            name=f"hanging_rail_{idx + 1}",
        )

    _box(
        wing,
        (0.002, 0.100, 0.024),
        (side_sign * (HINGE_CLEAR + TUBE - 0.001), 0.0, -0.001),
        mark_mat,
        name="index_strip",
    )
    _box(
        wing,
        (0.014, 0.120, 0.004),
        (side_sign * (HINGE_CLEAR + WING_WIDTH - 0.007), 0.0, 0.011),
        mark_mat,
        name="outer_datum_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_fold_drying_rack")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.28, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    amber = model.material("amber", rgba=(0.88, 0.56, 0.14, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((CENTER_WIDTH, DEPTH, TOP_Z)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z * 0.5)),
    )

    _box(
        base,
        (TUBE, DEPTH, TUBE),
        (-CENTER_WIDTH * 0.5 + TUBE * 0.5, 0.0, FRAME_Z),
        aluminum,
        name="left_side_rail",
    )
    _box(
        base,
        (TUBE, DEPTH, TUBE),
        (CENTER_WIDTH * 0.5 - TUBE * 0.5, 0.0, FRAME_Z),
        aluminum,
        name="right_side_rail",
    )
    _box(
        base,
        (CENTER_WIDTH, TUBE, TUBE),
        (0.0, DEPTH * 0.5 - TUBE * 0.5, FRAME_Z),
        aluminum,
        name="front_cross_rail",
    )
    _box(
        base,
        (CENTER_WIDTH, TUBE, TUBE),
        (0.0, -DEPTH * 0.5 + TUBE * 0.5, FRAME_Z),
        aluminum,
        name="rear_cross_rail",
    )

    for idx, x in enumerate((-0.20, -0.10, 0.0, 0.10, 0.20), start=1):
        _rod_y(
            base,
            RAIL_RADIUS,
            DEPTH - 2.0 * TUBE,
            (x, 0.0, FRAME_Z),
            aluminum,
            name="center_line_rail" if idx == 3 else f"center_hanging_rail_{idx}",
        )

    _box(
        base,
        (0.002, 0.120, 0.020),
        (-CENTER_WIDTH * 0.5 + 0.001, 0.0, FRAME_Z),
        amber,
        name="left_gap_pad",
    )
    _box(
        base,
        (0.002, 0.120, 0.020),
        (CENTER_WIDTH * 0.5 - 0.001, 0.0, FRAME_Z),
        amber,
        name="right_gap_pad",
    )

    leg_height = TOP_Z - TUBE
    leg_z = leg_height * 0.5
    brace_z = 0.18
    leg_x = CENTER_WIDTH * 0.5 - LEG_TUBE * 0.5
    leg_y = DEPTH * 0.5 - LEG_TUBE * 0.5

    for x in (-leg_x, leg_x):
        for y in (-leg_y, leg_y):
            _box(base, (LEG_TUBE, LEG_TUBE, leg_height), (x, y, leg_z), graphite)

    _box(base, (CENTER_WIDTH - 2.0 * LEG_TUBE, LEG_TUBE, LEG_TUBE), (0.0, leg_y, brace_z), graphite)
    _box(base, (CENTER_WIDTH - 2.0 * LEG_TUBE, LEG_TUBE, LEG_TUBE), (0.0, -leg_y, brace_z), graphite)
    _box(base, (LEG_TUBE, DEPTH - 2.0 * LEG_TUBE, LEG_TUBE), (-leg_x, 0.0, brace_z), graphite)
    _box(base, (LEG_TUBE, DEPTH - 2.0 * LEG_TUBE, LEG_TUBE), (leg_x, 0.0, brace_z), graphite)

    _build_hinge_link(model, "left_hinge_link", -1.0, graphite, amber)
    _build_hinge_link(model, "right_hinge_link", 1.0, graphite, amber)
    _build_wing(model, "left_wing", -1.0, aluminum, black)
    _build_wing(model, "right_wing", 1.0, aluminum, black)

    left_link = model.get_part("left_hinge_link")
    right_link = model.get_part("right_hinge_link")
    left_wing = model.get_part("left_wing")
    right_wing = model.get_part("right_wing")

    model.articulation(
        "base_to_left_hinge_link",
        ArticulationType.FIXED,
        parent=base,
        child=left_link,
        origin=Origin(xyz=(-CENTER_WIDTH * 0.5, 0.0, FRAME_Z)),
    )
    model.articulation(
        "base_to_right_hinge_link",
        ArticulationType.FIXED,
        parent=base,
        child=right_link,
        origin=Origin(xyz=(CENTER_WIDTH * 0.5, 0.0, FRAME_Z)),
    )
    model.articulation(
        "left_wing_fold",
        ArticulationType.REVOLUTE,
        parent=left_link,
        child=left_wing,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=WING_OPEN_LIMIT,
        ),
    )
    model.articulation(
        "right_wing_fold",
        ArticulationType.REVOLUTE,
        parent=right_link,
        child=right_wing,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=WING_OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    left_link = object_model.get_part("left_hinge_link")
    right_link = object_model.get_part("right_hinge_link")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_fold = object_model.get_articulation("left_wing_fold")
    right_fold = object_model.get_articulation("right_wing_fold")

    left_side_rail = base.get_visual("left_side_rail")
    right_side_rail = base.get_visual("right_side_rail")
    left_gap_pad = base.get_visual("left_gap_pad")
    right_gap_pad = base.get_visual("right_gap_pad")
    center_line_rail = base.get_visual("center_line_rail")

    left_mount = left_link.get_visual("mount_block")
    right_mount = right_link.get_visual("mount_block")
    left_root = left_wing.get_visual("root_rail")
    right_root = right_wing.get_visual("root_rail")
    left_inner_datum = left_wing.get_visual("inner_datum")
    right_inner_datum = right_wing.get_visual("inner_datum")
    left_outer_rail = left_wing.get_visual("outer_rail")
    right_outer_rail = right_wing.get_visual("outer_rail")

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

    for part_name, part in (
        ("base", base),
        ("left_hinge_link", left_link),
        ("right_hinge_link", right_link),
        ("left_wing", left_wing),
        ("right_wing", right_wing),
    ):
        ctx.check(f"{part_name}_present", part is not None)

    ctx.expect_contact(
        left_link,
        base,
        elem_a=left_mount,
        elem_b=left_side_rail,
        name="left_hinge_link_mounted_to_base",
    )
    ctx.expect_contact(
        right_link,
        base,
        elem_a=right_mount,
        elem_b=right_side_rail,
        name="right_hinge_link_mounted_to_base",
    )

    ctx.check(
        "left_wing_fold_axis",
        tuple(left_fold.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {left_fold.axis}",
    )
    ctx.check(
        "right_wing_fold_axis",
        tuple(right_fold.axis) == (0.0, -1.0, 0.0),
        details=f"expected (0, -1, 0), got {right_fold.axis}",
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        ctx.expect_contact(
            left_wing,
            left_link,
            elem_a=left_root,
            elem_b=left_mount,
            name="left_wing_supported_in_closed_pose",
        )
        ctx.expect_contact(
            right_wing,
            right_link,
            elem_a=right_root,
            elem_b=right_mount,
            name="right_wing_supported_in_closed_pose",
        )
        ctx.expect_gap(
            base,
            left_wing,
            axis="x",
            positive_elem=left_gap_pad,
            negative_elem=left_inner_datum,
            min_gap=0.003,
            max_gap=0.005,
            name="left_closed_gap_is_controlled",
        )
        ctx.expect_gap(
            right_wing,
            base,
            axis="x",
            positive_elem=right_inner_datum,
            negative_elem=right_gap_pad,
            min_gap=0.003,
            max_gap=0.005,
            name="right_closed_gap_is_controlled",
        )

    with ctx.pose({left_fold: WING_OPEN_LIMIT, right_fold: WING_OPEN_LIMIT}):
        ctx.expect_gap(
            left_wing,
            base,
            axis="z",
            positive_elem=left_outer_rail,
            negative_elem=center_line_rail,
            min_gap=0.20,
            name="left_wing_rises_clear_at_open_stop",
        )
        ctx.expect_gap(
            right_wing,
            base,
            axis="z",
            positive_elem=right_outer_rail,
            negative_elem=center_line_rail,
            min_gap=0.20,
            name="right_wing_rises_clear_at_open_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
