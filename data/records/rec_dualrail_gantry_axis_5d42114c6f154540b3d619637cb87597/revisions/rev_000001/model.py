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


FRAME_W = 0.62
FRAME_H = 0.28
BACK_T = 0.012
WINDOW_W = 0.46
WINDOW_H = 0.19

RAIL_LENGTH = 0.50
RAIL_R = 0.0115
RAIL_Y = 0.040
LOWER_RAIL_Z = 0.085
UPPER_RAIL_Z = 0.195
RAIL_Z_OFFSET = (UPPER_RAIL_Z - LOWER_RAIL_Z) / 2.0
MID_RAIL_Z = (UPPER_RAIL_Z + LOWER_RAIL_Z) / 2.0

SUPPORT_W = 0.050
SUPPORT_D = 0.046
SUPPORT_H = 0.190
SUPPORT_X = 0.240

CROSSHEAD_W = 0.092
CROSSHEAD_SHOE_D = 0.022
CROSSHEAD_SHOE_H = 0.040
CROSSHEAD_SHOE_Y = RAIL_R + (CROSSHEAD_SHOE_D / 2.0)
CROSSHEAD_WEB_W = 0.070
CROSSHEAD_WEB_D = 0.026
CROSSHEAD_WEB_H = 0.140
GUIDE_BLOCK_W = 0.050
GUIDE_BLOCK_D = 0.038
GUIDE_BLOCK_H = 0.060
GUIDE_BLOCK_Y = 0.054
GUIDE_FRONT_Y = GUIDE_BLOCK_Y + (GUIDE_BLOCK_D / 2.0)

RAM_W = 0.022
RAM_H = 0.036
RAM_L = 0.080

CROSSHEAD_LOWER = -0.150
CROSSHEAD_UPPER = 0.150
CARRIAGE_LOWER = 0.0
CARRIAGE_UPPER = 0.035


def _x_cylinder(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - (length / 2.0), center[1], center[2]))
    )


def _frame_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(FRAME_W, BACK_T, FRAME_H).translate((0.0, BACK_T / 2.0, FRAME_H / 2.0))
    window = (
        cq.Workplane("XY")
        .box(WINDOW_W, BACK_T + 0.004, WINDOW_H)
        .translate((0.0, BACK_T / 2.0, MID_RAIL_Z))
    )
    frame = plate.cut(window)

    for x_sign in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY")
            .box(SUPPORT_W, SUPPORT_D, SUPPORT_H)
            .translate((x_sign * SUPPORT_X, SUPPORT_D / 2.0, MID_RAIL_Z))
        )

    frame = frame.union(_x_cylinder(RAIL_LENGTH, RAIL_R, (0.0, RAIL_Y, LOWER_RAIL_Z)))
    frame = frame.union(_x_cylinder(RAIL_LENGTH, RAIL_R, (0.0, RAIL_Y, UPPER_RAIL_Z)))
    return frame


def _crosshead_shape() -> cq.Workplane:
    shape = (
        cq.Workplane("XY")
        .box(CROSSHEAD_W, CROSSHEAD_SHOE_D, CROSSHEAD_SHOE_H)
        .translate((0.0, CROSSHEAD_SHOE_Y, -RAIL_Z_OFFSET))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(CROSSHEAD_W, CROSSHEAD_SHOE_D, CROSSHEAD_SHOE_H)
        .translate((0.0, CROSSHEAD_SHOE_Y, RAIL_Z_OFFSET))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(CROSSHEAD_WEB_W, CROSSHEAD_WEB_D, CROSSHEAD_WEB_H)
        .translate((0.0, 0.034, 0.0))
    )
    shape = shape.union(
        cq.Workplane("XY")
        .box(GUIDE_BLOCK_W, GUIDE_BLOCK_D, GUIDE_BLOCK_H)
        .translate((0.0, GUIDE_BLOCK_Y, 0.0))
    )

    shape = shape.cut(_x_cylinder(CROSSHEAD_W + 0.020, RAIL_R, (0.0, 0.0, -RAIL_Z_OFFSET)))
    shape = shape.cut(_x_cylinder(CROSSHEAD_W + 0.020, RAIL_R, (0.0, 0.0, RAIL_Z_OFFSET)))
    shape = shape.cut(
        cq.Workplane("XZ")
        .rect(RAM_W, RAM_H)
        .extrude(GUIDE_BLOCK_D + 0.012)
        .translate((0.0, GUIDE_BLOCK_Y - ((GUIDE_BLOCK_D + 0.012) / 2.0), 0.0))
    )
    return shape


def _carriage_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.046, 0.024, 0.056).translate((0.0, 0.012, 0.0))
    saddle = cq.Workplane("XY").box(0.030, 0.018, 0.042).translate((0.0, 0.032, 0.0))
    nose = cq.Workplane("XY").box(0.018, 0.016, 0.028).translate((0.0, 0.049, 0.0))
    return base.union(saddle).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_shuttle")

    model.material("frame_gray", rgba=(0.44, 0.47, 0.50, 1.0))
    model.material("crosshead_red", rgba=(0.72, 0.17, 0.12, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame"),
        material="frame_gray",
        name="frame_body",
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_crosshead_shape(), "crosshead"),
        material="crosshead_red",
        name="crosshead_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material="carriage_dark",
        name="carriage_body",
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, RAIL_Y, MID_RAIL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=CROSSHEAD_LOWER,
            upper=CROSSHEAD_UPPER,
            effort=220.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "crosshead_to_carriage",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=carriage,
        origin=Origin(xyz=(0.0, GUIDE_FRONT_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=CARRIAGE_LOWER,
            upper=CARRIAGE_UPPER,
            effort=60.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crosshead = object_model.get_part("crosshead")
    carriage = object_model.get_part("carriage")
    crosshead_slide = object_model.get_articulation("frame_to_crosshead")
    carriage_slide = object_model.get_articulation("crosshead_to_carriage")

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

    ctx.expect_contact(frame, crosshead, name="crosshead_is_physically_guided_by_frame")
    ctx.expect_contact(crosshead, carriage, name="carriage_is_physically_guided_by_crosshead")
    ctx.expect_within(
        carriage,
        crosshead,
        axes="xz",
        margin=0.003,
        name="carriage_stays_centered_on_crosshead_face",
    )

    with ctx.pose({carriage_slide: CARRIAGE_UPPER}):
        ctx.expect_within(
            carriage,
            crosshead,
            axes="xz",
            margin=0.003,
            name="carriage_remains_centered_when_extended",
        )

    with ctx.pose({crosshead_slide: CROSSHEAD_LOWER}):
        crosshead_low = ctx.part_world_position(crosshead)
    with ctx.pose({crosshead_slide: CROSSHEAD_UPPER}):
        crosshead_high = ctx.part_world_position(crosshead)

    crosshead_dx = crosshead_high[0] - crosshead_low[0]
    crosshead_dy = crosshead_high[1] - crosshead_low[1]
    crosshead_dz = crosshead_high[2] - crosshead_low[2]
    ctx.check(
        "crosshead_prismatic_axis_is_horizontal_x",
        abs(crosshead_dx - (CROSSHEAD_UPPER - CROSSHEAD_LOWER)) < 1e-4
        and abs(crosshead_dy) < 1e-6
        and abs(crosshead_dz) < 1e-6,
        details=(
            f"expected pure x travel of {CROSSHEAD_UPPER - CROSSHEAD_LOWER:.4f} m, "
            f"got dx={crosshead_dx:.6f}, dy={crosshead_dy:.6f}, dz={crosshead_dz:.6f}"
        ),
    )

    with ctx.pose({crosshead_slide: 0.0, carriage_slide: CARRIAGE_LOWER}):
        carriage_low = ctx.part_world_position(carriage)
    with ctx.pose({crosshead_slide: 0.0, carriage_slide: CARRIAGE_UPPER}):
        carriage_high = ctx.part_world_position(carriage)

    carriage_dx = carriage_high[0] - carriage_low[0]
    carriage_dy = carriage_high[1] - carriage_low[1]
    carriage_dz = carriage_high[2] - carriage_low[2]
    ctx.check(
        "carriage_prismatic_axis_is_orthogonal_horizontal_y",
        abs(carriage_dy - (CARRIAGE_UPPER - CARRIAGE_LOWER)) < 1e-4
        and abs(carriage_dx) < 1e-6
        and abs(carriage_dz) < 1e-6,
        details=(
            f"expected pure y travel of {CARRIAGE_UPPER - CARRIAGE_LOWER:.4f} m, "
            f"got dx={carriage_dx:.6f}, dy={carriage_dy:.6f}, dz={carriage_dz:.6f}"
        ),
    )

    ctx.check(
        "stacked_joint_axes_are_horizontal_and_orthogonal",
        abs(crosshead_slide.axis[2]) < 1e-9
        and abs(carriage_slide.axis[2]) < 1e-9
        and abs(sum(a * b for a, b in zip(crosshead_slide.axis, carriage_slide.axis))) < 1e-9,
        details=f"axes were {crosshead_slide.axis} and {carriage_slide.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
