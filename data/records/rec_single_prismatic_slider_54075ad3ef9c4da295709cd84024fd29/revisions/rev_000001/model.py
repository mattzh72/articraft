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


BEAM_LENGTH = 1.20
BEAM_WIDTH = 0.10
BEAM_HEIGHT = 0.15
BEAM_INNER_LENGTH = 1.10
BEAM_INNER_WIDTH = 0.068
BEAM_INNER_HEIGHT = 0.108
BEAM_INNER_CENTER_Z = -0.004
BEAM_SLOT_LENGTH = 1.08
BEAM_SLOT_WIDTH = 0.026
BEAM_SLOT_HEIGHT = 0.09
BEAM_SLOT_CENTER_Z = -0.045

TRUCK_LENGTH = 0.22
TRUCK_WIDTH = 0.058
TRUCK_HEIGHT = 0.048
TRUCK_PAD_LENGTH = 0.17
TRUCK_PAD_WIDTH = 0.014
TRUCK_PAD_HEIGHT = 0.030
TRUCK_PAD_CENTER_Z = 0.039
TRUCK_PAD_Y = 0.021
NECK_LENGTH = 0.030
NECK_WIDTH = 0.018
NECK_HEIGHT = 0.098
NECK_CENTER_Z = -0.068
HANGER_BODY_LENGTH = 0.15
HANGER_BODY_WIDTH = 0.052
HANGER_BODY_HEIGHT = 0.09
HANGER_BODY_CENTER_Z = -0.161

SLIDE_LOWER = -0.42
SLIDE_UPPER = 0.42


def _beam_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT)
    inner = (
        cq.Workplane("XY")
        .box(BEAM_INNER_LENGTH, BEAM_INNER_WIDTH, BEAM_INNER_HEIGHT)
        .translate((0.0, 0.0, BEAM_INNER_CENTER_Z))
    )
    slot = (
        cq.Workplane("XY")
        .box(BEAM_SLOT_LENGTH, BEAM_SLOT_WIDTH, BEAM_SLOT_HEIGHT)
        .translate((0.0, 0.0, BEAM_SLOT_CENTER_Z))
    )
    left_pad = cq.Workplane("XY").box(0.18, 0.07, 0.012).translate((-0.36, 0.0, 0.081))
    right_pad = cq.Workplane("XY").box(0.18, 0.07, 0.012).translate((0.36, 0.0, 0.081))
    return outer.cut(inner).cut(slot).union(left_pad).union(right_pad)


def _upper_truck_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(TRUCK_LENGTH, TRUCK_WIDTH, TRUCK_HEIGHT)
    end_wipers = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-(TRUCK_LENGTH / 2.0) + 0.006, 0.0),
                ((TRUCK_LENGTH / 2.0) - 0.006, 0.0),
            ]
        )
        .box(0.012, TRUCK_WIDTH, 0.018, combine=False)
    )
    roof_pads = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -TRUCK_PAD_Y), (0.0, TRUCK_PAD_Y)])
        .box(TRUCK_PAD_LENGTH, TRUCK_PAD_WIDTH, TRUCK_PAD_HEIGHT, combine=False)
        .translate((0.0, 0.0, TRUCK_PAD_CENTER_Z))
    )
    shape = body.union(end_wipers).union(roof_pads)
    return shape.edges("|Z").fillet(0.005)


def _hanger_body_shape() -> cq.Workplane:
    main_block = (
        cq.Workplane("XY")
        .box(HANGER_BODY_LENGTH, HANGER_BODY_WIDTH, HANGER_BODY_HEIGHT)
        .translate((0.0, 0.0, HANGER_BODY_CENTER_Z))
    )
    eye_plate = (
        cq.Workplane("XY")
        .box(0.072, 0.024, 0.042)
        .translate((0.0, 0.0, -0.227))
    )
    pin_hole = (
        cq.Workplane("XZ")
        .circle(0.016)
        .extrude(0.040)
        .translate((0.0, -0.020, -0.227))
    )
    shape = main_block.union(eye_plate).cut(pin_hole)
    return shape.edges("|Y").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_trolley_slide")

    model.material("beam_paint", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("truck_steel", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("hanger_steel", rgba=(0.48, 0.50, 0.54, 1.0))

    support_beam = model.part("support_beam")
    support_beam.visual(
        mesh_from_cadquery(_beam_shape(), "support_beam"),
        material="beam_paint",
        name="beam_shell",
    )
    support_beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, BEAM_WIDTH, BEAM_HEIGHT)),
        mass=28.0,
    )

    hanging_carriage = model.part("hanging_carriage")
    hanging_carriage.visual(
        mesh_from_cadquery(_upper_truck_shape(), "upper_truck"),
        material="truck_steel",
        name="upper_truck",
    )
    hanging_carriage.visual(
        Box((NECK_LENGTH, NECK_WIDTH, NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, NECK_CENTER_Z)),
        material="hanger_steel",
        name="hanger_neck",
    )
    hanging_carriage.visual(
        mesh_from_cadquery(_hanger_body_shape(), "hanger_body"),
        material="hanger_steel",
        name="hanger_body",
    )
    hanging_carriage.inertial = Inertial.from_geometry(
        Box((HANGER_BODY_LENGTH, HANGER_BODY_WIDTH, 0.24)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support_beam,
        child=hanging_carriage,
        origin=Origin(xyz=(0.0, 0.0, BEAM_INNER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=300.0,
            velocity=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_beam = object_model.get_part("support_beam")
    hanging_carriage = object_model.get_part("hanging_carriage")
    slide = object_model.get_articulation("beam_to_carriage")

    ctx.expect_within(
        hanging_carriage,
        support_beam,
        axes="y",
        inner_elem="upper_truck",
        outer_elem="beam_shell",
        margin=0.001,
        name="upper truck stays centered within the beam width",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            support_beam,
            hanging_carriage,
            axis="z",
            positive_elem="beam_shell",
            negative_elem="hanger_body",
            min_gap=0.02,
            max_gap=0.06,
            name="hanger body hangs below the beam underside",
        )
        rest_pos = ctx.part_world_position(hanging_carriage)

    with ctx.pose({slide: SLIDE_LOWER}):
        ctx.expect_within(
            hanging_carriage,
            support_beam,
            axes="x",
            inner_elem="upper_truck",
            outer_elem="beam_shell",
            margin=0.0,
            name="upper truck remains under the beam at negative travel",
        )
        lower_pos = ctx.part_world_position(hanging_carriage)

    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_within(
            hanging_carriage,
            support_beam,
            axes="x",
            inner_elem="upper_truck",
            outer_elem="beam_shell",
            margin=0.0,
            name="upper truck remains under the beam at positive travel",
        )
        upper_pos = ctx.part_world_position(hanging_carriage)

    ctx.check(
        "carriage translates along the beam axis",
        rest_pos is not None
        and lower_pos is not None
        and upper_pos is not None
        and lower_pos[0] < rest_pos[0] - 0.10
        and upper_pos[0] > rest_pos[0] + 0.10
        and abs(lower_pos[1] - rest_pos[1]) < 1e-6
        and abs(upper_pos[1] - rest_pos[1]) < 1e-6
        and abs(lower_pos[2] - rest_pos[2]) < 1e-6
        and abs(upper_pos[2] - rest_pos[2]) < 1e-6,
        details=f"lower={lower_pos}, rest={rest_pos}, upper={upper_pos}",
    )

    ctx.check(
        "travel preserves end-wall clearance",
        (SLIDE_UPPER + (TRUCK_LENGTH / 2.0)) <= ((BEAM_INNER_LENGTH / 2.0) - 0.01)
        and ((-SLIDE_LOWER) + (TRUCK_LENGTH / 2.0)) <= ((BEAM_INNER_LENGTH / 2.0) - 0.01),
        details=(
            f"travel=[{SLIDE_LOWER}, {SLIDE_UPPER}], "
            f"truck_half={TRUCK_LENGTH / 2.0}, beam_inner_half={BEAM_INNER_LENGTH / 2.0}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
