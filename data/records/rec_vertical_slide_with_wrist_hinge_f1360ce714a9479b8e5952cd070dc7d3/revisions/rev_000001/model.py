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


BASE_W = 0.30
BASE_D = 0.24
BASE_H = 0.03

MAST_W = 0.12
MAST_D = 0.08
MAST_H = 0.66

RAIL_W = 0.062
RAIL_D = 0.032
RAIL_H = 0.56
RAIL_Y = (MAST_D / 2.0) + (RAIL_D / 2.0) - 0.001
RAIL_Z = BASE_H + 0.06 + (RAIL_H / 2.0)

CARRIAGE_W = 0.15
CARRIAGE_D = 0.102
CARRIAGE_H = 0.16
CARRIAGE_HOME_Z = 0.18
CARRIAGE_Y = 0.091

LIFT_TRAVEL = 0.26

NOSE_HINGE_Y = 0.066
NOSE_HINGE_Z = 0.026
NOSE_TILT_LOWER = -0.45
NOSE_TILT_UPPER = 0.85


def _make_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .edges()
        .chamfer(0.003)
    )


def _make_carriage_shape() -> cq.Workplane:
    pocket_w = RAIL_W + 0.012
    pocket_d = 0.070
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)
        .edges("|Z")
        .fillet(0.008)
    )

    pocket = cq.Workplane("XY").box(pocket_w, pocket_d, CARRIAGE_H + 0.02).translate(
        (0.0, -0.037, 0.0)
    )
    ear_left = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.044)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.046, 0.043, 0.028))
    )
    ear_right = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.044)
        .edges("|Z")
        .fillet(0.004)
        .translate((-0.046, 0.043, 0.028))
    )
    return body.cut(pocket).union(ear_left).union(ear_right)


def _make_nose_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.092, 0.104, 0.050).translate((0.0, 0.056, -0.028))
    chin = cq.Workplane("XY").box(0.050, 0.032, 0.018).translate((0.0, 0.088, -0.041))
    barrel = cq.Workplane("YZ").circle(0.011).extrude(0.038, both=True)

    top_cutter = (
        cq.Workplane("XY")
        .box(0.140, 0.120, 0.090)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -28.0)
        .translate((0.0, 0.080, 0.036))
    )
    belly_cutter = (
        cq.Workplane("XY")
        .box(0.120, 0.080, 0.060)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 18.0)
        .translate((0.0, 0.036, -0.056))
    )

    return (
        body.union(chin)
        .union(barrel)
        .cut(top_cutter)
        .cut(belly_cutter)
        .edges(">Y")
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_carriage_tilting_nose")

    model.material("base_paint", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("guide_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_finish", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("nose_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_make_base_shape(), "guide_base"),
        origin=Origin(),
        material="base_paint",
        name="guide_base",
    )
    guide.visual(
        Box((MAST_W, MAST_D, MAST_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + (MAST_H / 2.0))),
        material="guide_aluminum",
        name="mast_core",
    )
    guide.visual(
        Box((RAIL_W, RAIL_D, RAIL_H)),
        origin=Origin(xyz=(0.0, RAIL_Y, RAIL_Z)),
        material="guide_aluminum",
        name="guide_rail",
    )
    guide.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_H + MAST_H)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + MAST_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage_shell"),
        material="carriage_finish",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)),
        mass=3.0,
    )

    nose = model.part("nose")
    nose.visual(
        mesh_from_cadquery(_make_nose_shape(), "nose_shell"),
        material="nose_dark",
        name="nose_shell",
    )
    nose.inertial = Inertial.from_geometry(
        Box((0.092, 0.104, 0.050)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.056, -0.028)),
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_Y, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.22,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(0.0, NOSE_HINGE_Y, NOSE_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=NOSE_TILT_LOWER,
            upper=NOSE_TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    nose = object_model.get_part("nose")

    lift = object_model.get_articulation("guide_to_carriage")
    tilt = object_model.get_articulation("carriage_to_nose")

    mast_core = guide.get_visual("mast_core")
    guide_rail = guide.get_visual("guide_rail")
    carriage_shell = carriage.get_visual("carriage_shell")
    nose_shell = nose.get_visual("nose_shell")

    ctx.expect_gap(
        carriage,
        guide,
        axis="y",
        min_gap=0.0,
        max_gap=0.0015,
        positive_elem=carriage_shell,
        negative_elem=mast_core,
        name="carriage shell rides just ahead of mast",
    )
    ctx.expect_within(
        guide,
        carriage,
        axes="x",
        margin=0.006,
        inner_elem=guide_rail,
        outer_elem=carriage_shell,
        name="rail stays laterally captured by carriage",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="z",
        min_overlap=0.15,
        elem_a=carriage_shell,
        elem_b=guide_rail,
        name="carriage remains engaged on the rail at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_gap(
            carriage,
            guide,
            axis="y",
            min_gap=0.0,
            max_gap=0.0015,
            positive_elem=carriage_shell,
            negative_elem=mast_core,
            name="carriage shell keeps mast clearance at full lift",
        )
        ctx.expect_overlap(
            carriage,
            guide,
            axes="z",
            min_overlap=0.15,
            elem_a=carriage_shell,
            elem_b=guide_rail,
            name="carriage remains engaged on the rail at full lift",
        )
        upper_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive prismatic motion raises the carriage",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )

    rest_nose_box = ctx.part_element_world_aabb(nose, elem=nose_shell)
    with ctx.pose({tilt: NOSE_TILT_UPPER}):
        raised_nose_box = ctx.part_element_world_aabb(nose, elem=nose_shell)

    ctx.check(
        "positive revolute motion lifts the nose",
        rest_nose_box is not None
        and raised_nose_box is not None
        and raised_nose_box[1][2] > rest_nose_box[1][2] + 0.03,
        details=f"rest={rest_nose_box}, raised={raised_nose_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
