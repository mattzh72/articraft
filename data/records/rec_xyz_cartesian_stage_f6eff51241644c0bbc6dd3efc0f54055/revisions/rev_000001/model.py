from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.56
BASE_W = 0.22
BASE_BODY_T = 0.028

X_RAIL_L = 0.48
X_RAIL_W = 0.020
X_RAIL_H = 0.012
X_RAIL_Y = 0.067
X_COVER_W = 0.030
X_COVER_H = 0.006
X_STOP_L = 0.018
X_STOP_W = 0.040
X_STOP_H = 0.014
X_TRAVEL = 0.20
X_CLEAR = 0.0

X_CARR_L = 0.16
X_CARR_W = 0.19
X_CARR_T = 0.022
X_RUNNER_L = 0.10
X_RUNNER_W = 0.036
X_RUNNER_H = 0.012

Y_RAIL_L = 0.18
Y_RAIL_W = 0.016
Y_RAIL_H = 0.010
Y_RAIL_X = 0.040
Y_COVER_W = 0.028
Y_COVER_H = 0.005
Y_STOP_X = 0.030
Y_STOP_Y = 0.016
Y_STOP_H = 0.010
Y_TRAVEL = 0.04
Y_CLEAR = 0.0

Y_STAGE_L = 0.13
Y_STAGE_W = 0.12
Y_STAGE_T = 0.020
Y_RUNNER_X = 0.030
Y_RUNNER_Y = 0.085
Y_RUNNER_H = 0.010

COLUMN_W = 0.108
COLUMN_SPINE_D = 0.020
COLUMN_CHEEK_D = 0.036
COLUMN_CHEEK_W = 0.018
COLUMN_H = 0.312

Z_RAIL_X = 0.030
Z_RAIL_W = 0.014
Z_RAIL_D = 0.010
Z_RAIL_Z0 = 0.070
Z_RAIL_H = 0.225
Z_CLEAR = 0.0
Z_TRAVEL = 0.09

Z_SLIDE_W = 0.095
Z_SLIDE_D = 0.016
Z_SLIDE_H = 0.150
Z_RUNNER_W = 0.026
Z_RUNNER_D = 0.010
Z_RUNNER_H = 0.030
Z_RUNNER_LOW_Z = 0.018
Z_RUNNER_HIGH_Z = 0.086
TOOL_PLATE_W = 0.110
TOOL_PLATE_D = 0.012
TOOL_PLATE_H = 0.110
TOOL_PLATE_Z0 = 0.045

X_TO_Y_Z = X_RUNNER_H + X_CARR_T + Y_RAIL_H + Y_CLEAR
BASE_TO_X_Z = BASE_BODY_T + X_RAIL_H + X_CLEAR
Y_TO_Z_Y = (COLUMN_CHEEK_D / 2.0) + Z_RAIL_D + Z_CLEAR
Y_TO_Z_Z = 0.060


def _box(
    x: float,
    y: float,
    z: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return cq.Workplane("XY").box(x, y, z, centered=centered).translate(translate)


def _add_visual(part, shape, name: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, name), material=material, name=name)


def _base_body_shape():
    body = _box(BASE_L, BASE_W, BASE_BODY_T)
    center_recess = _box(
        BASE_L - 0.10,
        0.082,
        0.008,
        translate=(0.0, 0.0, BASE_BODY_T - 0.008),
    )
    side_relief_offset = (BASE_W / 2.0) - 0.017
    side_relief = _box(
        BASE_L - 0.16,
        0.022,
        0.011,
        translate=(0.0, side_relief_offset, 0.007),
    )
    return body.cut(center_recess).cut(side_relief).cut(
        side_relief.translate((0.0, -2.0 * side_relief_offset, 0.0))
    )


def _x_carriage_body_shape():
    plate = _box(X_CARR_L, X_CARR_W, X_CARR_T, translate=(0.0, 0.0, X_RUNNER_H))
    top_recess = _box(
        X_CARR_L - 0.042,
        0.070,
        0.006,
        translate=(0.0, 0.0, X_RUNNER_H + X_CARR_T - 0.006),
    )
    center_relief = _box(X_CARR_L - 0.020, 0.046, 0.010, translate=(0.0, 0.0, X_RUNNER_H))
    return plate.cut(top_recess).cut(center_relief)


def _y_stage_body_shape():
    base_plate = _box(Y_STAGE_L, Y_STAGE_W, Y_STAGE_T, translate=(0.0, 0.0, Y_RUNNER_H))
    column_z = Y_RUNNER_H + Y_STAGE_T
    spine = _box(COLUMN_W, COLUMN_SPINE_D, COLUMN_H, translate=(0.0, 0.0, column_z))
    cheek_x = (COLUMN_W / 2.0) - (COLUMN_CHEEK_W / 2.0)
    left_cheek = _box(COLUMN_CHEEK_W, COLUMN_CHEEK_D, COLUMN_H, translate=(-cheek_x, 0.0, column_z))
    right_cheek = _box(COLUMN_CHEEK_W, COLUMN_CHEEK_D, COLUMN_H, translate=(cheek_x, 0.0, column_z))
    window = _box(
        COLUMN_W - (2.0 * COLUMN_CHEEK_W) - 0.010,
        COLUMN_CHEEK_D - 0.010,
        COLUMN_H - 0.060,
        translate=(0.0, 0.0, column_z + 0.030),
    )
    return base_plate.union(spine).union(left_cheek).union(right_cheek).cut(window)


def _z_slide_body_shape():
    slide = _box(
        Z_SLIDE_W,
        Z_SLIDE_D,
        Z_SLIDE_H,
        centered=(True, False, False),
        translate=(0.0, Z_RUNNER_D, 0.0),
    )
    lightening = _box(
        Z_SLIDE_W - 0.026,
        Z_SLIDE_D,
        0.062,
        centered=(True, False, False),
        translate=(0.0, Z_RUNNER_D, 0.040),
    )
    return slide.cut(lightening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="benchtop_positioning_stage")

    model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_gray", rgba=(0.69, 0.71, 0.75, 1.0))
    model.material("rail_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("strip_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("tool_blue", rgba=(0.35, 0.43, 0.63, 1.0))

    base = model.part("base")
    _add_visual(base, _base_body_shape(), "base_body", "base_dark")
    _add_visual(
        base,
        _box(X_RAIL_L, X_RAIL_W, X_RAIL_H, translate=(0.0, X_RAIL_Y, BASE_BODY_T)),
        "x_rail_left",
        "rail_steel",
    )
    _add_visual(
        base,
        _box(X_RAIL_L, X_RAIL_W, X_RAIL_H, translate=(0.0, -X_RAIL_Y, BASE_BODY_T)),
        "x_rail_right",
        "rail_steel",
    )
    _add_visual(
        base,
        _box(BASE_L - 0.060, X_COVER_W, X_COVER_H, translate=(0.0, 0.0, BASE_BODY_T - X_COVER_H)),
        "x_cover_strip",
        "strip_black",
    )
    x_stop_offset = (X_RAIL_L / 2.0) - 0.011
    _add_visual(
        base,
        _box(X_STOP_L, X_STOP_W, X_STOP_H, translate=(-x_stop_offset, 0.0, BASE_BODY_T)),
        "x_stop_neg",
        "machined_gray",
    )
    _add_visual(
        base,
        _box(X_STOP_L, X_STOP_W, X_STOP_H, translate=(x_stop_offset, 0.0, BASE_BODY_T)),
        "x_stop_pos",
        "machined_gray",
    )

    x_stage = model.part("x_stage")
    _add_visual(x_stage, _x_carriage_body_shape(), "x_body", "machined_gray")
    _add_visual(
        x_stage,
        _box(X_RUNNER_L, X_RUNNER_W, X_RUNNER_H, translate=(0.0, X_RAIL_Y, 0.0)),
        "x_runner_left",
        "machined_gray",
    )
    _add_visual(
        x_stage,
        _box(X_RUNNER_L, X_RUNNER_W, X_RUNNER_H, translate=(0.0, -X_RAIL_Y, 0.0)),
        "x_runner_right",
        "machined_gray",
    )
    y_rail_z = X_RUNNER_H + X_CARR_T
    _add_visual(
        x_stage,
        _box(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, translate=(Y_RAIL_X, 0.0, y_rail_z)),
        "y_rail_left",
        "rail_steel",
    )
    _add_visual(
        x_stage,
        _box(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H, translate=(-Y_RAIL_X, 0.0, y_rail_z)),
        "y_rail_right",
        "rail_steel",
    )
    _add_visual(
        x_stage,
        _box(Y_COVER_W, Y_RAIL_L - 0.018, Y_COVER_H, translate=(0.0, 0.0, y_rail_z)),
        "y_cover_strip",
        "strip_black",
    )
    y_stop_offset = (Y_RAIL_L / 2.0) + 0.004
    _add_visual(
        x_stage,
        _box(Y_STOP_X, Y_STOP_Y, Y_STOP_H, translate=(0.0, -y_stop_offset, y_rail_z)),
        "y_stop_neg",
        "machined_gray",
    )
    _add_visual(
        x_stage,
        _box(Y_STOP_X, Y_STOP_Y, Y_STOP_H, translate=(0.0, y_stop_offset, y_rail_z)),
        "y_stop_pos",
        "machined_gray",
    )

    y_stage = model.part("y_stage")
    _add_visual(y_stage, _y_stage_body_shape(), "y_body", "machined_gray")
    _add_visual(
        y_stage,
        _box(Y_RUNNER_X, Y_RUNNER_Y, Y_RUNNER_H, translate=(Y_RAIL_X, 0.0, 0.0)),
        "y_runner_left",
        "machined_gray",
    )
    _add_visual(
        y_stage,
        _box(Y_RUNNER_X, Y_RUNNER_Y, Y_RUNNER_H, translate=(-Y_RAIL_X, 0.0, 0.0)),
        "y_runner_right",
        "machined_gray",
    )
    rail_y = COLUMN_CHEEK_D / 2.0
    _add_visual(
        y_stage,
        _box(
            Z_RAIL_W,
            Z_RAIL_D,
            Z_RAIL_H,
            centered=(True, False, False),
            translate=(Z_RAIL_X, rail_y, Z_RAIL_Z0),
        ),
        "z_rail_left",
        "rail_steel",
    )
    _add_visual(
        y_stage,
        _box(
            Z_RAIL_W,
            Z_RAIL_D,
            Z_RAIL_H,
            centered=(True, False, False),
            translate=(-Z_RAIL_X, rail_y, Z_RAIL_Z0),
        ),
        "z_rail_right",
        "rail_steel",
    )
    _add_visual(
        y_stage,
        _box(
            0.090,
            Z_RAIL_D,
            0.008,
            centered=(True, False, False),
            translate=(0.0, rail_y, 0.044),
        ),
        "z_lower_stop",
        "machined_gray",
    )
    _add_visual(
        y_stage,
        _box(
            0.096,
            Z_RAIL_D,
            0.008,
            centered=(True, False, False),
            translate=(0.0, rail_y, 0.314),
        ),
        "z_upper_stop",
        "machined_gray",
    )

    z_stage = model.part("z_stage")
    _add_visual(z_stage, _z_slide_body_shape(), "z_body", "machined_gray")
    _add_visual(
        z_stage,
        _box(
            Z_RUNNER_W,
            Z_RUNNER_D,
            Z_RUNNER_H,
            centered=(True, False, False),
            translate=(Z_RAIL_X, 0.0, Z_RUNNER_LOW_Z),
        ),
        "z_lower_runner_left",
        "machined_gray",
    )
    _add_visual(
        z_stage,
        _box(
            Z_RUNNER_W,
            Z_RUNNER_D,
            Z_RUNNER_H,
            centered=(True, False, False),
            translate=(-Z_RAIL_X, 0.0, Z_RUNNER_LOW_Z),
        ),
        "z_lower_runner_right",
        "machined_gray",
    )
    _add_visual(
        z_stage,
        _box(
            Z_RUNNER_W,
            Z_RUNNER_D,
            Z_RUNNER_H,
            centered=(True, False, False),
            translate=(Z_RAIL_X, 0.0, Z_RUNNER_HIGH_Z),
        ),
        "z_upper_runner_left",
        "machined_gray",
    )
    _add_visual(
        z_stage,
        _box(
            Z_RUNNER_W,
            Z_RUNNER_D,
            Z_RUNNER_H,
            centered=(True, False, False),
            translate=(-Z_RAIL_X, 0.0, Z_RUNNER_HIGH_Z),
        ),
        "z_upper_runner_right",
        "machined_gray",
    )
    _add_visual(
        z_stage,
        _box(
            TOOL_PLATE_W,
            TOOL_PLATE_D,
            TOOL_PLATE_H,
            centered=(True, False, False),
            translate=(0.0, Z_RUNNER_D + Z_SLIDE_D, TOOL_PLATE_Z0),
        ),
        "tool_plate",
        "tool_blue",
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_X_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-(X_TRAVEL / 2.0),
            upper=(X_TRAVEL / 2.0),
            effort=180.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, X_TO_Y_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-(Y_TRAVEL / 2.0),
            upper=(Y_TRAVEL / 2.0),
            effort=120.0,
            velocity=0.24,
        ),
    )
    model.articulation(
        "y_to_z",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_stage,
        origin=Origin(xyz=(0.0, Y_TO_Z_Y, Y_TO_Z_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=90.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    z_stage = object_model.get_part("z_stage")

    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")
    y_to_z = object_model.get_articulation("y_to_z")

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

    ctx.check(
        "orthogonal_prismatic_axes",
        base_to_x.axis == (1.0, 0.0, 0.0)
        and x_to_y.axis == (0.0, 1.0, 0.0)
        and y_to_z.axis == (0.0, 0.0, 1.0),
        details=f"x={base_to_x.axis}, y={x_to_y.axis}, z={y_to_z.axis}",
    )

    ctx.expect_contact(
        x_stage,
        base,
        elem_a="x_runner_left",
        elem_b="x_rail_left",
        contact_tol=1e-6,
        name="x_runner_contacts_x_rail",
    )
    ctx.expect_overlap(
        x_stage,
        base,
        axes="xy",
        elem_a="x_runner_left",
        elem_b="x_rail_left",
        min_overlap=0.018,
        name="x_runner_supported_by_x_rail",
    )

    ctx.expect_contact(
        y_stage,
        x_stage,
        elem_a="y_runner_left",
        elem_b="y_rail_left",
        contact_tol=1e-6,
        name="y_runner_contacts_y_rail",
    )
    ctx.expect_overlap(
        y_stage,
        x_stage,
        axes="xy",
        elem_a="y_runner_left",
        elem_b="y_rail_left",
        min_overlap=0.014,
        name="y_runner_supported_by_y_rail",
    )

    ctx.expect_contact(
        z_stage,
        y_stage,
        elem_a="z_lower_runner_left",
        elem_b="z_rail_left",
        contact_tol=1e-6,
        name="z_runner_contacts_z_rail",
    )
    ctx.expect_overlap(
        z_stage,
        y_stage,
        axes="xz",
        elem_a="z_lower_runner_left",
        elem_b="z_rail_left",
        min_overlap=0.0135,
        name="z_runner_supported_by_z_rail",
    )

    with ctx.pose({base_to_x: base_to_x.motion_limits.upper}):
        ctx.expect_gap(
            base,
            x_stage,
            axis="x",
            positive_elem="x_stop_pos",
            negative_elem="x_body",
            min_gap=0.015,
            name="x_positive_stop_clear",
        )
        ctx.expect_overlap(
            x_stage,
            base,
            axes="xy",
            elem_a="x_runner_left",
            elem_b="x_rail_left",
            min_overlap=0.018,
            name="x_runner_stays_on_rail_at_positive_travel",
        )

    with ctx.pose({base_to_x: base_to_x.motion_limits.lower}):
        ctx.expect_gap(
            x_stage,
            base,
            axis="x",
            positive_elem="x_body",
            negative_elem="x_stop_neg",
            min_gap=0.015,
            name="x_negative_stop_clear",
        )

    with ctx.pose({x_to_y: x_to_y.motion_limits.upper}):
        ctx.expect_gap(
            x_stage,
            y_stage,
            axis="y",
            positive_elem="y_stop_pos",
            negative_elem="y_body",
            min_gap=0.0055,
            name="y_positive_stop_clear",
        )
        ctx.expect_overlap(
            y_stage,
            x_stage,
            axes="xy",
            elem_a="y_runner_left",
            elem_b="y_rail_left",
            min_overlap=0.014,
            name="y_runner_stays_on_rail_at_positive_travel",
        )

    with ctx.pose({x_to_y: x_to_y.motion_limits.lower}):
        ctx.expect_gap(
            y_stage,
            x_stage,
            axis="y",
            positive_elem="y_body",
            negative_elem="y_stop_neg",
            min_gap=0.0055,
            name="y_negative_stop_clear",
        )

    with ctx.pose({y_to_z: y_to_z.motion_limits.lower}):
        ctx.expect_gap(
            z_stage,
            y_stage,
            axis="z",
            positive_elem="z_body",
            negative_elem="z_lower_stop",
            min_gap=0.006,
            name="z_lower_stop_clear",
        )

    with ctx.pose({y_to_z: y_to_z.motion_limits.upper}):
        ctx.expect_gap(
            y_stage,
            z_stage,
            axis="z",
            positive_elem="z_upper_stop",
            negative_elem="tool_plate",
            min_gap=0.008,
            name="z_upper_stop_clear",
        )
        ctx.expect_overlap(
            z_stage,
            y_stage,
            axes="xz",
            elem_a="z_upper_runner_left",
            elem_b="z_rail_left",
            min_overlap=0.0135,
            name="z_upper_runner_stays_on_rail_at_max_height",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
