from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


HOUSING_TOP_Z = 0.078
PITCH_PIVOT_X = 0.010
PITCH_PIVOT_Z = 0.074
OUTPUT_PLATE_MOUNT = (0.038, 0.0, 0.015)


def _cylinder_y(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, -0.5 * length, 0.0))
    )


def _make_housing_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").box(0.180, 0.140, 0.010).translate((0.0, 0.0, 0.005))

    lower_shell = (
        cq.Workplane("XY")
        .box(0.140, 0.110, 0.042)
        .translate((0.0, 0.0, 0.031))
        .edges("|Z")
        .fillet(0.010)
    )
    upper_shoulder = (
        cq.Workplane("XY")
        .box(0.122, 0.092, 0.020)
        .translate((0.0, 0.0, 0.062))
        .edges("|Z")
        .fillet(0.008)
    )
    housing_outer = flange.union(lower_shell).union(upper_shoulder)

    lower_cavity = (
        cq.Workplane("XY")
        .box(0.118, 0.088, 0.065)
        .translate((0.0, 0.0, 0.0325))
        .edges("|Z")
        .fillet(0.006)
    )
    upper_cavity = (
        cq.Workplane("XY")
        .box(0.102, 0.072, 0.020)
        .translate((0.0, 0.0, 0.058))
        .edges("|Z")
        .fillet(0.005)
    )
    housing_shell = housing_outer.cut(lower_cavity.union(upper_cavity))

    collar = (
        cq.Workplane("XY")
        .circle(0.046)
        .circle(0.028)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.070))
    )

    mounting_holes = None
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            hole = (
                cq.Workplane("XY")
                .circle(0.0045)
                .extrude(0.014)
                .translate((x_sign * 0.065, y_sign * 0.045, 0.0))
            )
            mounting_holes = hole if mounting_holes is None else mounting_holes.union(hole)

    return housing_shell.union(collar).cut(mounting_holes)


def _make_yaw_stage_shape() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(0.043).extrude(0.012)
    neck = (
        cq.Workplane("XY")
        .box(0.060, 0.046, 0.044)
        .translate((0.004, 0.0, 0.034))
        .edges("|Z")
        .fillet(0.006)
    )

    left_arm = cq.Workplane("XY").box(0.024, 0.012, 0.040).translate((PITCH_PIVOT_X, 0.032, 0.066))
    right_arm = cq.Workplane("XY").box(0.024, 0.012, 0.040).translate((PITCH_PIVOT_X, -0.032, 0.066))
    left_web = cq.Workplane("XY").box(0.018, 0.010, 0.012).translate((0.008, 0.0245, 0.051))
    right_web = cq.Workplane("XY").box(0.018, 0.010, 0.012).translate((0.008, -0.0245, 0.051))
    saddle_pad = cq.Workplane("XY").box(0.018, 0.022, 0.006).translate((0.004, 0.0, 0.057))
    left_boss = _cylinder_y(0.010, 0.012).translate((PITCH_PIVOT_X, 0.032, PITCH_PIVOT_Z))
    right_boss = _cylinder_y(0.010, 0.012).translate((PITCH_PIVOT_X, -0.032, PITCH_PIVOT_Z))

    return (
        turntable.union(neck)
        .union(left_arm)
        .union(right_arm)
        .union(left_web)
        .union(right_web)
        .union(saddle_pad)
        .union(left_boss)
        .union(right_boss)
        .combine()
    )


def _make_tilt_cradle_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.036, 0.040, 0.028).translate((0.016, 0.0, 0.0))
    underside_cut = cq.Workplane("XY").box(0.018, 0.022, 0.018).translate((0.022, 0.0, -0.006))
    front_pad = cq.Workplane("XY").box(0.012, 0.036, 0.014).translate((0.038, 0.0, 0.008))
    left_trunnion = _cylinder_y(0.0085, 0.006).translate((0.0, 0.0215, 0.0))
    right_trunnion = _cylinder_y(0.0085, 0.006).translate((0.0, -0.0215, 0.0))

    return (
        body.cut(underside_cut)
        .union(front_pad)
        .union(left_trunnion)
        .union(right_trunnion)
        .combine()
    )


def _make_output_plate_shape() -> cq.Workplane:
    riser = cq.Workplane("XY").box(0.016, 0.024, 0.016).translate((0.0, 0.0, 0.008))
    plate = (
        cq.Workplane("XY")
        .box(0.060, 0.050, 0.004)
        .translate((0.008, 0.0, 0.018))
        .edges("|Z")
        .fillet(0.006)
    )

    plate_holes = None
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            hole = (
                cq.Workplane("XY")
                .circle(0.0027)
                .extrude(0.008)
                .translate((0.008 + x_sign * 0.018, y_sign * 0.014, 0.014))
            )
            plate_holes = hole if plate_holes is None else plate_holes.union(hole)

    return riser.union(plate).cut(plate_holes).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_pan_tilt_unit")

    model.material("housing_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("stage_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("cradle_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("plate_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shape(), "housing_shell"),
        material="housing_black",
        name="housing_shell",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, HOUSING_TOP_Z)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "yaw_stage"),
        material="stage_graphite",
        name="yaw_stage",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.090),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        mesh_from_cadquery(_make_tilt_cradle_shape(), "tilt_cradle"),
        material="cradle_gray",
        name="tilt_cradle",
    )
    tilt_cradle.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.030)),
        mass=0.42,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    output_plate = model.part("output_plate")
    output_plate.visual(
        mesh_from_cadquery(_make_output_plate_shape(), "output_plate"),
        material="plate_aluminum",
        name="output_plate",
    )
    output_plate.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.022)),
        mass=0.14,
        origin=Origin(xyz=(0.008, 0.0, 0.011)),
    )

    model.articulation(
        "housing_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.80, upper=2.80, effort=18.0, velocity=2.2),
    )
    model.articulation(
        "yaw_stage_to_tilt_cradle",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=tilt_cradle,
        origin=Origin(xyz=(PITCH_PIVOT_X, 0.0, PITCH_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.10, effort=12.0, velocity=1.8),
    )
    model.articulation(
        "tilt_cradle_to_output_plate",
        ArticulationType.FIXED,
        parent=tilt_cradle,
        child=output_plate,
        origin=Origin(xyz=OUTPUT_PLATE_MOUNT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    yaw_stage = object_model.get_part("yaw_stage")
    tilt_cradle = object_model.get_part("tilt_cradle")
    output_plate = object_model.get_part("output_plate")

    yaw_joint = object_model.get_articulation("housing_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_tilt_cradle")

    ctx.allow_overlap(
        yaw_stage,
        tilt_cradle,
        elem_a="yaw_stage",
        elem_b="tilt_cradle",
        reason="The pitch hinge is represented with simplified closed trunnion and bearing geometry at the cradle pivot.",
    )

    ctx.expect_gap(
        yaw_stage,
        housing,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="yaw stage seats on the housing collar",
    )
    ctx.expect_contact(
        output_plate,
        tilt_cradle,
        name="output plate riser is mounted onto the tilt cradle",
    )
    ctx.expect_gap(
        output_plate,
        housing,
        axis="z",
        min_gap=0.020,
        name="output plate clears the grounded housing in neutral pose",
    )

    cradle_rest = ctx.part_world_position(tilt_cradle)
    with ctx.pose({yaw_joint: 1.0}):
        cradle_yawed = ctx.part_world_position(tilt_cradle)
    ctx.check(
        "positive yaw swings the upper assembly around the vertical axis",
        cradle_rest is not None
        and cradle_yawed is not None
        and cradle_yawed[1] > cradle_rest[1] + 0.006
        and cradle_yawed[0] < cradle_rest[0] - 0.003,
        details=f"rest={cradle_rest}, yawed={cradle_yawed}",
    )

    plate_rest = ctx.part_world_position(output_plate)
    with ctx.pose({pitch_joint: 0.80}):
        plate_tilted_up = ctx.part_world_position(output_plate)
    ctx.check(
        "positive pitch raises the output plate",
        plate_rest is not None
        and plate_tilted_up is not None
        and plate_tilted_up[2] > plate_rest[2] + 0.010,
        details=f"rest={plate_rest}, up={plate_tilted_up}",
    )

    with ctx.pose({pitch_joint: -0.65}):
        ctx.expect_gap(
            output_plate,
            housing,
            axis="z",
            min_gap=0.006,
            name="downward pitch still keeps the output plate above the housing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
