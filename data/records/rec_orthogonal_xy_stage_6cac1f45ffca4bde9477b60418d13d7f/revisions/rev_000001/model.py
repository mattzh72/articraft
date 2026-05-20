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


BASE_LENGTH = 0.56
BASE_WIDTH = 0.22
BASE_SIDE_FOOT_LENGTH = 0.48
BASE_SIDE_FOOT_WIDTH = 0.028
BASE_SIDE_FOOT_Y = 0.085
BASE_FOOT_HEIGHT = 0.012
BASE_PLATE_Z = 0.010
BASE_PLATE_THICKNESS = 0.012
BASE_RAIL_CENTER_Y = 0.060
BASE_RAIL_PAD_LENGTH = 0.48
BASE_RAIL_PAD_WIDTH = 0.030
BASE_RAIL_PAD_Z = 0.020
BASE_RAIL_PAD_HEIGHT = 0.010
BASE_RAIL_LENGTH = 0.48
BASE_RAIL_WIDTH = 0.018
BASE_RAIL_Z = 0.029
BASE_RAIL_HEIGHT = 0.013
BASE_RAIL_TOP_Z = BASE_RAIL_Z + BASE_RAIL_HEIGHT

X_STAGE_TRAVEL = 0.24
X_BLOCK_LENGTH = 0.115
X_BLOCK_WIDTH = 0.036
X_BLOCK_HEIGHT = 0.024
X_BODY_LENGTH = 0.175
X_BODY_WIDTH = 0.150
X_BODY_Z = 0.022
X_BODY_HEIGHT = 0.034
X_POCKET_LENGTH = 0.105
X_POCKET_WIDTH = 0.020
X_POCKET_Z = 0.030
X_POCKET_HEIGHT = 0.018
CROSS_RAIL_CENTER_X = 0.035
CROSS_PAD_X = 0.030
CROSS_PAD_LENGTH = 0.220
CROSS_PAD_Z = 0.054
CROSS_PAD_HEIGHT = 0.010
CROSS_RAIL_WIDTH = 0.020
CROSS_RAIL_LENGTH = 0.220
CROSS_RAIL_Z = 0.063
CROSS_RAIL_HEIGHT = 0.011
X_TO_Y_Z = CROSS_RAIL_Z + CROSS_RAIL_HEIGHT

Y_STAGE_TRAVEL = 0.11
Y_BLOCK_X = 0.036
Y_BLOCK_LENGTH = 0.090
Y_BLOCK_HEIGHT = 0.020
Y_BODY_X = 0.118
Y_BODY_Y = 0.136
Y_BODY_Z = 0.018
Y_BODY_HEIGHT = 0.020
Y_POCKET_X = 0.074
Y_POCKET_Y = 0.070
Y_POCKET_Z = 0.026
Y_POCKET_HEIGHT = 0.012
Y_PEDESTAL_X = 0.070
Y_PEDESTAL_Y = 0.070
Y_PEDESTAL_Z = 0.036
Y_PEDESTAL_HEIGHT = 0.018
TOOL_DECK_X = 0.130
TOOL_DECK_Y = 0.130
TOOL_DECK_Z = 0.052
TOOL_DECK_HEIGHT = 0.014
TOOL_HOLE_RADIUS = 0.005
TOOL_HOLE_PITCH = 0.080


def _raised_box(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((x, y, z))
    )


def _raised_cylinder(
    radius: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z))


def _build_base_shape() -> cq.Workplane:
    base = _raised_box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_PLATE_THICKNESS,
        z=BASE_PLATE_Z,
    )
    for foot_y in (-BASE_SIDE_FOOT_Y, BASE_SIDE_FOOT_Y):
        base = base.union(
            _raised_box(
                BASE_SIDE_FOOT_LENGTH,
                BASE_SIDE_FOOT_WIDTH,
                BASE_FOOT_HEIGHT,
                y=foot_y,
            )
        )
    for rail_y in (-BASE_RAIL_CENTER_Y, BASE_RAIL_CENTER_Y):
        base = base.union(
            _raised_box(
                BASE_RAIL_PAD_LENGTH,
                BASE_RAIL_PAD_WIDTH,
                BASE_RAIL_PAD_HEIGHT,
                y=rail_y,
                z=BASE_RAIL_PAD_Z,
            )
        )
        base = base.union(
            _raised_box(
                BASE_RAIL_LENGTH,
                BASE_RAIL_WIDTH,
                BASE_RAIL_HEIGHT,
                y=rail_y,
                z=BASE_RAIL_Z,
            )
        )
    for hole_x in (-0.21, 0.21):
        for hole_y in (-0.085, 0.085):
            base = base.cut(
                _raised_cylinder(
                    0.0045,
                    BASE_PLATE_Z + BASE_PLATE_THICKNESS,
                    x=hole_x,
                    y=hole_y,
                )
            )
    return base


def _build_x_carriage_shape() -> cq.Workplane:
    carriage = _raised_box(
        X_BLOCK_LENGTH,
        X_BLOCK_WIDTH,
        X_BLOCK_HEIGHT,
        y=-BASE_RAIL_CENTER_Y,
    )
    carriage = carriage.union(
        _raised_box(
            X_BLOCK_LENGTH,
            X_BLOCK_WIDTH,
            X_BLOCK_HEIGHT,
            y=BASE_RAIL_CENTER_Y,
        )
    )
    carriage = carriage.union(
        _raised_box(
            X_BODY_LENGTH,
            X_BODY_WIDTH,
            X_BODY_HEIGHT,
            z=X_BODY_Z,
        )
    )
    for rail_x in (-CROSS_RAIL_CENTER_X, CROSS_RAIL_CENTER_X):
        carriage = carriage.union(
            _raised_box(
                CROSS_PAD_X,
                CROSS_PAD_LENGTH,
                CROSS_PAD_HEIGHT,
                x=rail_x,
                z=CROSS_PAD_Z,
            )
        )
        carriage = carriage.union(
            _raised_box(
                CROSS_RAIL_WIDTH,
                CROSS_RAIL_LENGTH,
                CROSS_RAIL_HEIGHT,
                x=rail_x,
                z=CROSS_RAIL_Z,
            )
        )
    return carriage


def _build_y_slide_shape() -> cq.Workplane:
    slide = _raised_box(
        Y_BLOCK_X,
        Y_BLOCK_LENGTH,
        Y_BLOCK_HEIGHT,
        x=-CROSS_RAIL_CENTER_X,
    )
    slide = slide.union(
        _raised_box(
            Y_BLOCK_X,
            Y_BLOCK_LENGTH,
            Y_BLOCK_HEIGHT,
            x=CROSS_RAIL_CENTER_X,
        )
    )
    slide = slide.union(
        _raised_box(
            Y_BODY_X,
            Y_BODY_Y,
            Y_BODY_HEIGHT,
            z=Y_BODY_Z,
        )
    )
    slide = slide.cut(
        _raised_box(
            Y_POCKET_X,
            Y_POCKET_Y,
            Y_POCKET_HEIGHT,
            z=Y_POCKET_Z,
        )
    )
    slide = slide.union(
        _raised_box(
            Y_PEDESTAL_X,
            Y_PEDESTAL_Y,
            Y_PEDESTAL_HEIGHT,
            z=Y_PEDESTAL_Z,
        )
    )
    slide = slide.union(
        _raised_box(
            TOOL_DECK_X,
            TOOL_DECK_Y,
            TOOL_DECK_HEIGHT,
            z=TOOL_DECK_Z,
        )
    )
    for hole_x in (-TOOL_HOLE_PITCH / 2.0, TOOL_HOLE_PITCH / 2.0):
        for hole_y in (-TOOL_HOLE_PITCH / 2.0, TOOL_HOLE_PITCH / 2.0):
            slide = slide.cut(
                _raised_cylinder(
                    TOOL_HOLE_RADIUS,
                    TOOL_DECK_Z + TOOL_DECK_HEIGHT,
                    x=hole_x,
                    y=hole_y,
                )
            )
    return slide


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= 1e-9 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_xy_pick_and_place_axis")

    model.material("base_gray", rgba=(0.38, 0.41, 0.45, 1.0))
    model.material("carriage_gray", rgba=(0.69, 0.72, 0.76, 1.0))
    model.material("slide_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    base = model.part("lower_guide")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "lower_guide"),
        material="base_gray",
        name="guide_body",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_build_x_carriage_shape(), "x_carriage"),
        material="carriage_gray",
        name="carriage_body",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_build_y_slide_shape(), "y_slide"),
        material="slide_silver",
        name="slide_body",
    )

    model.articulation(
        "lower_guide_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-(X_STAGE_TRAVEL / 2.0),
            upper=X_STAGE_TRAVEL / 2.0,
            effort=280.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "x_carriage_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, X_TO_Y_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-(Y_STAGE_TRAVEL / 2.0),
            upper=Y_STAGE_TRAVEL / 2.0,
            effort=180.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    x_joint = object_model.get_articulation("lower_guide_to_x_carriage")
    y_joint = object_model.get_articulation("x_carriage_to_y_slide")

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
        "x joint runs along horizontal x",
        _axis_matches(x_joint.axis, (1.0, 0.0, 0.0)),
        f"expected x-axis prismatic motion, got {x_joint.axis}",
    )
    ctx.check(
        "y joint runs along horizontal y",
        _axis_matches(y_joint.axis, (0.0, 1.0, 0.0)),
        f"expected y-axis prismatic motion, got {y_joint.axis}",
    )

    ctx.expect_contact(
        x_carriage,
        lower_guide,
        contact_tol=0.0005,
        name="x carriage bears on lower guide rails",
    )
    ctx.expect_contact(
        y_slide,
        x_carriage,
        contact_tol=0.0005,
        name="cross-slide bears on x carriage rails",
    )
    ctx.expect_gap(
        y_slide,
        lower_guide,
        axis="z",
        min_gap=0.07,
        name="upper slide stays stacked above grounded guide",
    )
    ctx.expect_overlap(
        x_carriage,
        lower_guide,
        axes="xy",
        min_overlap=0.07,
        name="x carriage footprint stays over the lower guide",
    )
    ctx.expect_overlap(
        y_slide,
        x_carriage,
        axes="xy",
        min_overlap=0.05,
        name="cross-slide footprint stays over the first carriage",
    )

    x_rest = ctx.part_world_position(x_carriage)
    y_rest = ctx.part_world_position(y_slide)
    with ctx.pose({x_joint: x_joint.motion_limits.upper}):
        x_plus = ctx.part_world_position(x_carriage)
        ctx.expect_contact(
            x_carriage,
            lower_guide,
            contact_tol=0.0005,
            name="x carriage remains supported at +x limit",
        )
    with ctx.pose({y_joint: y_joint.motion_limits.upper}):
        y_plus = ctx.part_world_position(y_slide)
        ctx.expect_contact(
            y_slide,
            x_carriage,
            contact_tol=0.0005,
            name="cross-slide remains supported at +y limit",
        )
    with ctx.pose(
        {
            x_joint: x_joint.motion_limits.upper,
            y_joint: y_joint.motion_limits.upper,
        }
    ):
        ctx.expect_gap(
            y_slide,
            lower_guide,
            axis="z",
            min_gap=0.07,
            name="tooling deck clears the base at the travel corner",
        )

    ctx.check(
        "x carriage translates positively along x",
        x_rest is not None
        and x_plus is not None
        and (x_plus[0] - x_rest[0]) > 0.10
        and abs(x_plus[1] - x_rest[1]) < 1e-6
        and abs(x_plus[2] - x_rest[2]) < 1e-6,
        f"rest={x_rest}, plus={x_plus}",
    )
    ctx.check(
        "y slide translates positively along y",
        y_rest is not None
        and y_plus is not None
        and (y_plus[1] - y_rest[1]) > 0.04
        and abs(y_plus[0] - y_rest[0]) < 1e-6
        and abs(y_plus[2] - y_rest[2]) < 1e-6,
        f"rest={y_rest}, plus={y_plus}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
