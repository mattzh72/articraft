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


BASE_L = 0.42
BASE_W = 0.18
BASE_T = 0.024
BASE_FOOT_SKIN = 0.008

RAIL_L = 0.32
RAIL_W = 0.036
RAIL_H = 0.018

STOP_L = 0.014
STOP_W = 0.052
STOP_H = 0.018

COVER_L = 0.34
COVER_W = 0.018
COVER_H = 0.004
COVER_Y = 0.058

X_TRAVEL = 0.095

CARR_L = 0.094
CARR_W = 0.102
SKIRT_DROP = 0.018
CARR_TOP_H = 0.034
TUNNEL_W = 0.044
TUNNEL_TOP = 0.010

LOWER_GUIDE_T = 0.004

PEDESTAL_X = 0.078
PEDESTAL_Y = 0.066
PEDESTAL_H = 0.016

MAST_X = 0.060
MAST_Y = 0.040
MAST_WALL = 0.004
MAST_H = 0.320
MAST_Z0 = CARR_TOP_H + PEDESTAL_H
MAST_TOP_CAP_T = 0.006

BRACKET_HOME_Z = 0.091
Z_TRAVEL_LOWER = 0.0
Z_TRAVEL_UPPER = 0.170

BRACKET_CORE_X = 0.090
BRACKET_CORE_Y = 0.070
BRACKET_H = 0.042
BRACKET_PLATE_X = 0.108
BRACKET_PLATE_Y = 0.082
BRACKET_PLATE_T = 0.010
BRACKET_HOLE_X = MAST_X + 0.008
BRACKET_HOLE_Y = MAST_Y + 0.008

UPPER_GUIDE_T = 0.004
UPPER_GUIDE_H = 0.026
UPPER_GUIDE_Z0 = 0.008


def _add_mesh_visual(part, shape, name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, name),
        material=material,
        name=name,
    )


def _base_body_shape():
    body = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
        .translate((0.0, 0.0, -BASE_T))
    )
    underside_relief = (
        cq.Workplane("XY")
        .box(
            BASE_L - 0.10,
            BASE_W - 0.06,
            BASE_T - BASE_FOOT_SKIN,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -BASE_T))
    )
    return body.cut(underside_relief).faces(">Z").edges().fillet(0.006)


def _rail_track_shape():
    return (
        cq.Workplane("XY")
        .box(
            RAIL_L,
            RAIL_W,
            RAIL_H,
            centered=(True, True, False),
        )
        .faces(">Z")
        .edges("|X")
        .chamfer(0.002)
    )


def _stop_shape(x_center: float):
    return (
        cq.Workplane("XY")
        .box(STOP_L, STOP_W, STOP_H, centered=(True, True, False))
        .translate((x_center, 0.0, 0.0))
        .faces(">Z")
        .edges()
        .fillet(0.002)
    )


def _cover_strip_shape(y_center: float):
    return (
        cq.Workplane("XY")
        .box(COVER_L, COVER_W, COVER_H, centered=(True, True, False))
        .translate((0.0, y_center, 0.0))
        .faces(">Z")
        .edges()
        .fillet(0.0015)
    )


def _carriage_body_shape():
    outer = (
        cq.Workplane("XY")
        .box(
            CARR_L,
            CARR_W,
            CARR_TOP_H + SKIRT_DROP,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -SKIRT_DROP))
    )
    tunnel = (
        cq.Workplane("XY")
        .box(
            CARR_L + 0.004,
            TUNNEL_W,
            TUNNEL_TOP + SKIRT_DROP,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -SKIRT_DROP))
    )
    carriage = outer.cut(tunnel)

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_X, PEDESTAL_Y, PEDESTAL_H, centered=(True, True, False))
        .translate((0.0, 0.0, CARR_TOP_H))
    )
    carriage = carriage.union(pedestal)

    front_web = (
        cq.Workplane("XY")
        .box(0.050, 0.010, 0.022, centered=(True, True, False))
        .translate((0.0, 0.026, CARR_TOP_H))
    )
    rear_web = (
        cq.Workplane("XY")
        .box(0.050, 0.010, 0.022, centered=(True, True, False))
        .translate((0.0, -0.026, CARR_TOP_H))
    )
    return (
        carriage.union(front_web)
        .union(rear_web)
        .faces(">Z")
        .edges()
        .fillet(0.003)
    )


def _lower_guide_shoes_shape():
    left_pad = (
        cq.Workplane("XY")
        .box(0.060, LOWER_GUIDE_T, 0.012, centered=(True, True, False))
        .translate((0.0, (RAIL_W / 2.0) + (LOWER_GUIDE_T / 2.0), -0.014))
    )
    right_pad = (
        cq.Workplane("XY")
        .box(0.060, LOWER_GUIDE_T, 0.012, centered=(True, True, False))
        .translate((0.0, -(RAIL_W / 2.0) - (LOWER_GUIDE_T / 2.0), -0.014))
    )
    front_wear = (
        cq.Workplane("XY")
        .box(0.020, 0.028, TUNNEL_TOP, centered=(True, True, False))
        .translate((0.024, 0.0, 0.0))
    )
    rear_wear = (
        cq.Workplane("XY")
        .box(0.020, 0.028, TUNNEL_TOP, centered=(True, True, False))
        .translate((-0.024, 0.0, 0.0))
    )
    return left_pad.union(right_pad).union(front_wear).union(rear_wear)


def _mast_tube_shape():
    outer = (
        cq.Workplane("XY")
        .box(MAST_X, MAST_Y, MAST_H, centered=(True, True, False))
        .translate((0.0, 0.0, MAST_Z0))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            MAST_X - (2.0 * MAST_WALL),
            MAST_Y - (2.0 * MAST_WALL),
            MAST_H + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, MAST_Z0 - 0.001))
    )
    return outer.cut(inner).edges("|Z").fillet(0.0015)


def _mast_top_cap_shape():
    return (
        cq.Workplane("XY")
        .box(
            MAST_X - (2.0 * MAST_WALL),
            MAST_Y - (2.0 * MAST_WALL),
            MAST_TOP_CAP_T,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, MAST_Z0 + MAST_H - MAST_TOP_CAP_T))
        .faces(">Z")
        .edges()
        .fillet(0.0015)
    )


def _bracket_body_shape():
    core = cq.Workplane("XY").box(
        BRACKET_CORE_X,
        BRACKET_CORE_Y,
        BRACKET_H,
        centered=(True, True, False),
    )
    plate = (
        cq.Workplane("XY")
        .box(
            BRACKET_PLATE_X,
            BRACKET_PLATE_Y,
            BRACKET_PLATE_T,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BRACKET_H))
    )
    body = core.union(plate)
    hole = (
        cq.Workplane("XY")
        .box(
            BRACKET_HOLE_X,
            BRACKET_HOLE_Y,
            BRACKET_H + BRACKET_PLATE_T + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.001))
    )
    return body.cut(hole).faces(">Z").edges().fillet(0.004)


def _upper_guide_shoes_shape():
    left = (
        cq.Workplane("XY")
        .box(UPPER_GUIDE_T, 0.032, UPPER_GUIDE_H, centered=(True, True, False))
        .translate(((MAST_X / 2.0) + (UPPER_GUIDE_T / 2.0), 0.0, UPPER_GUIDE_Z0))
    )
    right = (
        cq.Workplane("XY")
        .box(UPPER_GUIDE_T, 0.032, UPPER_GUIDE_H, centered=(True, True, False))
        .translate((-(MAST_X / 2.0) - (UPPER_GUIDE_T / 2.0), 0.0, UPPER_GUIDE_Z0))
    )
    front = (
        cq.Workplane("XY")
        .box(0.024, UPPER_GUIDE_T, UPPER_GUIDE_H, centered=(True, True, False))
        .translate((0.0, (MAST_Y / 2.0) + (UPPER_GUIDE_T / 2.0), UPPER_GUIDE_Z0))
    )
    rear = (
        cq.Workplane("XY")
        .box(0.024, UPPER_GUIDE_T, UPPER_GUIDE_H, centered=(True, True, False))
        .translate((0.0, -(MAST_Y / 2.0) - (UPPER_GUIDE_T / 2.0), UPPER_GUIDE_Z0))
    )
    return left.union(right).union(front).union(rear)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="optics_lift_stage")

    model.material("anodized_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("guide_polymer", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _base_body_shape(), "base_body", "anodized_black")
    _add_mesh_visual(base, _rail_track_shape(), "rail_track", "rail_steel")
    _add_mesh_visual(
        base,
        _stop_shape(-(RAIL_L / 2.0 + STOP_L / 2.0)),
        "left_stop",
        "anodized_black",
    )
    _add_mesh_visual(
        base,
        _stop_shape(RAIL_L / 2.0 + STOP_L / 2.0),
        "right_stop",
        "anodized_black",
    )
    _add_mesh_visual(
        base,
        _cover_strip_shape(COVER_Y),
        "right_cover_strip",
        "anodized_black",
    )
    _add_mesh_visual(
        base,
        _cover_strip_shape(-COVER_Y),
        "left_cover_strip",
        "anodized_black",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + RAIL_H)),
        mass=5.4,
        origin=Origin(xyz=(0.0, 0.0, (-BASE_T + RAIL_H) / 2.0)),
    )

    mast_carriage = model.part("mast_carriage")
    _add_mesh_visual(
        mast_carriage,
        _carriage_body_shape(),
        "carriage_body",
        "machined_aluminum",
    )
    _add_mesh_visual(
        mast_carriage,
        _lower_guide_shoes_shape(),
        "lower_guide_shoes",
        "guide_polymer",
    )
    _add_mesh_visual(
        mast_carriage,
        _mast_tube_shape(),
        "mast_tube",
        "machined_aluminum",
    )
    _add_mesh_visual(
        mast_carriage,
        _mast_top_cap_shape(),
        "mast_top_cap",
        "guide_polymer",
    )
    mast_carriage.inertial = Inertial.from_geometry(
        Box((CARR_W, CARR_W, MAST_Z0 + MAST_H + MAST_TOP_CAP_T + SKIRT_DROP)),
        mass=2.1,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                ((MAST_Z0 + MAST_H + MAST_TOP_CAP_T) - SKIRT_DROP) / 2.0,
            )
        ),
    )

    top_bracket = model.part("top_bracket")
    _add_mesh_visual(
        top_bracket,
        _bracket_body_shape(),
        "bracket_body",
        "machined_aluminum",
    )
    _add_mesh_visual(
        top_bracket,
        _upper_guide_shoes_shape(),
        "upper_guide_shoes",
        "guide_polymer",
    )
    top_bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_PLATE_X, BRACKET_PLATE_Y, BRACKET_H + BRACKET_PLATE_T)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, (BRACKET_H + BRACKET_PLATE_T) / 2.0)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=300.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "mast_to_bracket",
        ArticulationType.PRISMATIC,
        parent=mast_carriage,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0, BRACKET_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
            effort=180.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast_carriage = object_model.get_part("mast_carriage")
    top_bracket = object_model.get_part("top_bracket")
    base_to_mast = object_model.get_articulation("base_to_mast")
    mast_to_bracket = object_model.get_articulation("mast_to_bracket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        mast_carriage,
        base,
        elem_a="lower_guide_shoes",
        elem_b="rail_track",
        reason=(
            "The lower polymer guide shoes are modeled as preloaded liners riding on the "
            "rail flanks and top land; the simplified envelope intentionally absorbs the "
            "sub-millimeter running clearance."
        ),
    )
    ctx.allow_overlap(
        top_bracket,
        mast_carriage,
        elem_a="upper_guide_shoes",
        elem_b="mast_tube",
        reason=(
            "The upper guide shoes represent snug wear pads inside the mast bracket bore; "
            "their simplified envelope intentionally absorbs the real bearing preload."
        ),
    )

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
        "lower slide axis is x",
        tuple(base_to_mast.axis) == (1.0, 0.0, 0.0),
        details=f"axis={base_to_mast.axis}",
    )
    ctx.check(
        "upper slide axis is z",
        tuple(mast_to_bracket.axis) == (0.0, 0.0, 1.0),
        details=f"axis={mast_to_bracket.axis}",
    )

    ctx.expect_contact(
        mast_carriage,
        base,
        elem_a="lower_guide_shoes",
        elem_b="rail_track",
        name="lower carriage guide shoes contact the lateral rail",
    )
    ctx.expect_contact(
        top_bracket,
        mast_carriage,
        elem_a="upper_guide_shoes",
        elem_b="mast_tube",
        name="upper bracket guide shoes contact the mast tube",
    )

    with ctx.pose({base_to_mast: X_TRAVEL}):
        ctx.expect_gap(
            base,
            mast_carriage,
            axis="x",
            positive_elem="right_stop",
            negative_elem="carriage_body",
            min_gap=0.010,
            name="carriage clears the right rail stop at max x travel",
        )

    with ctx.pose({base_to_mast: -X_TRAVEL}):
        ctx.expect_gap(
            mast_carriage,
            base,
            axis="x",
            positive_elem="carriage_body",
            negative_elem="left_stop",
            min_gap=0.010,
            name="carriage clears the left rail stop at min x travel",
        )

    with ctx.pose({mast_to_bracket: Z_TRAVEL_LOWER}):
        ctx.expect_gap(
            top_bracket,
            mast_carriage,
            axis="z",
            positive_elem="bracket_body",
            negative_elem="carriage_body",
            min_gap=0.035,
            name="upper bracket stays above the lower carriage at its bottom position",
        )

    with ctx.pose({mast_to_bracket: Z_TRAVEL_UPPER}):
        ctx.expect_gap(
            mast_carriage,
            top_bracket,
            axis="z",
            positive_elem="mast_top_cap",
            negative_elem="bracket_body",
            min_gap=0.045,
            name="upper bracket clears the mast top cap at max z travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
