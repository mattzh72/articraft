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


BASE_LENGTH = 0.64
BASE_WIDTH = 0.18
BASE_PLATE_THICKNESS = 0.020
BASE_FOOT_POCKET_LENGTH = 0.36
BASE_FOOT_POCKET_WIDTH = 0.086
BASE_FOOT_POCKET_DEPTH = 0.010

X_RAIL_LENGTH = 0.56
X_RAIL_WIDTH = 0.022
X_RAIL_HEIGHT = 0.024
X_RAIL_OFFSET_Y = 0.050
X_JOINT_Z = BASE_PLATE_THICKNESS + X_RAIL_HEIGHT

X_PAD_SIZE = (0.100, 0.032, 0.012)
X_TRAVEL_TONGUE_SIZE = (0.090, 0.034, 0.036)
X_STOP_SIZE = (0.020, 0.050, 0.022)
X_STOP_OFFSET = 0.245
X_TRAVEL_MIN = -0.170
X_TRAVEL_MAX = 0.170

COLUMN_RAIL_SIZE = (0.018, 0.016, 0.390)
COLUMN_RAIL_OFFSET_X = 0.032
COLUMN_RAIL_CENTER_Y = 0.060
COLUMN_RAIL_FRONT_Y = COLUMN_RAIL_CENTER_Y + (COLUMN_RAIL_SIZE[1] / 2.0)
COLUMN_RAIL_CENTER_Z = 0.275

Z_GUIDE_SIZE = (0.028, 0.032, 0.152)
Z_GUIDE_OFFSET_X = COLUMN_RAIL_OFFSET_X
Z_GUIDE_CENTER_Y = 0.016
Z_GUIDE_CENTER_Z = 0.076
Z_BRIDGE_SIZE = (0.096, 0.024, 0.052)
Z_BRIDGE_CENTER_Y = 0.028
Z_BRIDGE_CENTER_Z = 0.026
Z_UPRIGHT_SIZE = (0.040, 0.020, 0.132)
Z_UPRIGHT_CENTER_Y = 0.042
Z_UPRIGHT_CENTER_Z = 0.118
Z_HEAD_BRACKET_SIZE = (0.072, 0.056, 0.030)
Z_HEAD_BRACKET_CENTER_Y = 0.044
Z_HEAD_BRACKET_CENTER_Z = 0.170
Z_STOP_TONGUE_SIZE = (0.024, 0.014, 0.122)
Z_STOP_TONGUE_CENTER_Y = 0.013
Z_STOP_TONGUE_CENTER_Z = 0.108
Z_TRAVEL_MIN = -0.100
Z_TRAVEL_MAX = 0.160
Z_JOINT_Z = 0.185

Z_STOP_SIZE = (0.028, 0.014, 0.010)
Z_STOP_CENTER_Y = 0.067
Z_STOP_LOWER_CENTER_Z = 0.121
Z_STOP_UPPER_CENTER_Z = 0.534

HEAD_PAD_SIZE = (0.082, 0.054, 0.018)
HEAD_PAD_CENTER = (0.0, 0.050, 0.187)


def _make_bottom_box(
    size: tuple[float, float, float],
    center_xy: tuple[float, float],
    z0: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(center_xy[0], center_xy[1])
        .box(size[0], size[1], size[2], centered=(True, True, False))
        .translate((0.0, 0.0, z0))
    )


def _base_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLATE_THICKNESS,
        centered=(True, True, False),
    )
    body = body.cut(
        _make_bottom_box(
            (BASE_FOOT_POCKET_LENGTH, BASE_FOOT_POCKET_WIDTH, BASE_FOOT_POCKET_DEPTH),
            (0.0, 0.0),
            0.0,
        )
    )
    body = body.union(
        _make_bottom_box((0.080, 0.130, 0.010), (-0.245, 0.0), BASE_PLATE_THICKNESS)
    )
    body = body.union(
        _make_bottom_box((0.080, 0.130, 0.010), (0.245, 0.0), BASE_PLATE_THICKNESS)
    )
    return body


def _x_carriage_body_shape() -> cq.Workplane:
    body = _make_bottom_box((0.150, 0.118, 0.020), (0.0, 0.0), 0.012)
    body = body.union(_make_bottom_box((0.118, 0.066, 0.040), (0.0, 0.0), 0.032))
    body = body.union(_make_bottom_box((0.042, 0.018, 0.320), (0.0, -0.006), 0.072))
    for x_pos in (-0.038, 0.038):
        body = body.union(_make_bottom_box((0.020, 0.020, 0.360), (x_pos, 0.000), 0.072))
    body = body.union(_make_bottom_box((0.060, 0.032, 0.220), (0.0, 0.022), 0.072))
    body = body.union(_make_bottom_box((0.060, 0.028, 0.158), (0.0, 0.022), 0.292))
    body = body.union(_make_bottom_box((0.092, 0.020, 0.038), (0.0, 0.042), 0.072))
    body = body.union(_make_bottom_box((0.106, 0.020, 0.040), (0.0, 0.042), 0.430))
    body = body.union(_make_bottom_box((0.030, 0.024, 0.060), (0.0, 0.052), 0.066))
    body = body.union(_make_bottom_box((0.030, 0.024, 0.080), (0.0, 0.052), 0.454))
    return body


def _z_carriage_body_shape() -> cq.Workplane:
    body = _make_bottom_box(
        Z_BRIDGE_SIZE,
        (0.0, Z_BRIDGE_CENTER_Y),
        Z_BRIDGE_CENTER_Z - (Z_BRIDGE_SIZE[2] / 2.0),
    )
    body = body.union(
        _make_bottom_box(
            Z_UPRIGHT_SIZE,
            (0.0, Z_UPRIGHT_CENTER_Y),
            Z_UPRIGHT_CENTER_Z - (Z_UPRIGHT_SIZE[2] / 2.0),
        )
    )
    body = body.union(
        _make_bottom_box(
            Z_HEAD_BRACKET_SIZE,
            (0.0, Z_HEAD_BRACKET_CENTER_Y),
            Z_HEAD_BRACKET_CENTER_Z - (Z_HEAD_BRACKET_SIZE[2] / 2.0),
        )
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_positioning_axis")

    model.material("base_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("machined_gray", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("guide_steel", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "bench_axis_base_body"),
        material="base_dark",
        name="base_body",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -X_RAIL_OFFSET_Y, BASE_PLATE_THICKNESS + (X_RAIL_HEIGHT / 2.0))
        ),
        material="guide_steel",
        name="x_rail_left",
    )
    base.visual(
        Box((X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, X_RAIL_OFFSET_Y, BASE_PLATE_THICKNESS + (X_RAIL_HEIGHT / 2.0))
        ),
        material="guide_steel",
        name="x_rail_right",
    )
    base.visual(
        Box(X_STOP_SIZE),
        origin=Origin(
            xyz=(-X_STOP_OFFSET, 0.0, BASE_PLATE_THICKNESS + (X_STOP_SIZE[2] / 2.0))
        ),
        material="machined_gray",
        name="x_stop_neg",
    )
    base.visual(
        Box(X_STOP_SIZE),
        origin=Origin(
            xyz=(X_STOP_OFFSET, 0.0, BASE_PLATE_THICKNESS + (X_STOP_SIZE[2] / 2.0))
        ),
        material="machined_gray",
        name="x_stop_pos",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_body_shape(), "bench_axis_x_carriage_body"),
        material="machined_gray",
        name="x_carriage_body",
    )
    x_carriage.visual(
        Box(X_PAD_SIZE),
        origin=Origin(xyz=(0.0, -X_RAIL_OFFSET_Y, X_PAD_SIZE[2] / 2.0)),
        material="machined_gray",
        name="x_pad_left",
    )
    x_carriage.visual(
        Box(X_PAD_SIZE),
        origin=Origin(xyz=(0.0, X_RAIL_OFFSET_Y, X_PAD_SIZE[2] / 2.0)),
        material="machined_gray",
        name="x_pad_right",
    )
    x_carriage.visual(
        Box(X_TRAVEL_TONGUE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="machined_gray",
        name="x_travel_tongue",
    )
    x_carriage.visual(
        Box(COLUMN_RAIL_SIZE),
        origin=Origin(xyz=(-COLUMN_RAIL_OFFSET_X, COLUMN_RAIL_CENTER_Y, COLUMN_RAIL_CENTER_Z)),
        material="guide_steel",
        name="column_rail_left",
    )
    x_carriage.visual(
        Box(COLUMN_RAIL_SIZE),
        origin=Origin(xyz=(COLUMN_RAIL_OFFSET_X, COLUMN_RAIL_CENTER_Y, COLUMN_RAIL_CENTER_Z)),
        material="guide_steel",
        name="column_rail_right",
    )
    x_carriage.visual(
        Box(Z_STOP_SIZE),
        origin=Origin(xyz=(0.0, Z_STOP_CENTER_Y, Z_STOP_LOWER_CENTER_Z)),
        material="machined_gray",
        name="z_stop_lower",
    )
    x_carriage.visual(
        Box(Z_STOP_SIZE),
        origin=Origin(xyz=(0.0, Z_STOP_CENTER_Y, Z_STOP_UPPER_CENTER_Z)),
        material="machined_gray",
        name="z_stop_upper",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        mesh_from_cadquery(_z_carriage_body_shape(), "bench_axis_z_carriage_body"),
        material="machined_gray",
        name="z_carriage_body",
    )
    z_carriage.visual(
        Box(Z_GUIDE_SIZE),
        origin=Origin(xyz=(-Z_GUIDE_OFFSET_X, Z_GUIDE_CENTER_Y, Z_GUIDE_CENTER_Z)),
        material="machined_gray",
        name="z_guide_left",
    )
    z_carriage.visual(
        Box(Z_GUIDE_SIZE),
        origin=Origin(xyz=(Z_GUIDE_OFFSET_X, Z_GUIDE_CENTER_Y, Z_GUIDE_CENTER_Z)),
        material="machined_gray",
        name="z_guide_right",
    )
    z_carriage.visual(
        Box(Z_STOP_TONGUE_SIZE),
        origin=Origin(xyz=(0.0, Z_STOP_TONGUE_CENTER_Y, Z_STOP_TONGUE_CENTER_Z)),
        material="machined_gray",
        name="z_stop_tongue",
    )
    z_carriage.visual(
        Box(HEAD_PAD_SIZE),
        origin=Origin(xyz=HEAD_PAD_CENTER),
        material="rubber_black",
        name="head_pad",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.30,
            lower=X_TRAVEL_MIN,
            upper=X_TRAVEL_MAX,
        ),
    )
    model.articulation(
        "x_to_z",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, COLUMN_RAIL_FRONT_Y, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.20,
            lower=Z_TRAVEL_MIN,
            upper=Z_TRAVEL_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_carriage = object_model.get_part("z_carriage")
    x_axis = object_model.get_articulation("base_to_x")
    z_axis = object_model.get_articulation("x_to_z")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        x_carriage,
        base,
        elem_a="x_pad_left",
        elem_b="x_rail_left",
        name="x_left_pad_contacts_left_rail",
    )
    ctx.expect_contact(
        x_carriage,
        base,
        elem_a="x_pad_right",
        elem_b="x_rail_right",
        name="x_right_pad_contacts_right_rail",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="x",
        elem_a="x_pad_left",
        elem_b="x_rail_left",
        min_overlap=0.090,
        name="x_left_pad_has_real_rail_engagement",
    )
    ctx.expect_contact(
        z_carriage,
        x_carriage,
        elem_a="z_guide_left",
        elem_b="column_rail_left",
        name="z_left_guide_contacts_left_column_rail",
    )
    ctx.expect_contact(
        z_carriage,
        x_carriage,
        elem_a="z_guide_right",
        elem_b="column_rail_right",
        name="z_right_guide_contacts_right_column_rail",
    )
    ctx.expect_overlap(
        z_carriage,
        x_carriage,
        axes="z",
        elem_a="z_guide_left",
        elem_b="column_rail_left",
        min_overlap=0.140,
        name="z_carriage_has_vertical_rail_capture",
    )

    with ctx.pose({x_axis: X_TRAVEL_MIN}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="x",
            positive_elem="x_travel_tongue",
            negative_elem="x_stop_neg",
            min_gap=0.015,
            max_gap=0.030,
            name="x_negative_stop_clearance",
        )
    with ctx.pose({x_axis: X_TRAVEL_MAX}):
        ctx.expect_gap(
            base,
            x_carriage,
            axis="x",
            positive_elem="x_stop_pos",
            negative_elem="x_travel_tongue",
            min_gap=0.015,
            max_gap=0.030,
            name="x_positive_stop_clearance",
        )
    with ctx.pose({z_axis: Z_TRAVEL_MIN}):
        ctx.expect_gap(
            z_carriage,
            x_carriage,
            axis="z",
            positive_elem="z_stop_tongue",
            negative_elem="z_stop_lower",
            min_gap=0.006,
            max_gap=0.018,
            name="z_lower_stop_clearance",
        )
    with ctx.pose({z_axis: Z_TRAVEL_MAX}):
        ctx.expect_gap(
            x_carriage,
            z_carriage,
            axis="z",
            positive_elem="z_stop_upper",
            negative_elem="z_stop_tongue",
            min_gap=0.010,
            max_gap=0.022,
            name="z_upper_stop_clearance",
        )

    with ctx.pose({x_axis: X_TRAVEL_MIN, z_axis: Z_TRAVEL_MAX}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_xmin_zmax")
    with ctx.pose({x_axis: X_TRAVEL_MAX, z_axis: Z_TRAVEL_MIN}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_xmax_zmin")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
