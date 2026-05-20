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


BASE_DEPTH = 0.16
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.03

COLUMN_HEIGHT = 0.66
BACK_THICKNESS = 0.03
BACK_WIDTH = 0.12
TOP_CAP_THICKNESS = 0.025

GUIDE_RAIL_THICKNESS = 0.008
GUIDE_RAIL_WIDTH = 0.018
GUIDE_RAIL_HEIGHT = 0.50
GUIDE_RAIL_SPACING = 0.07
GUIDE_RAIL_BASE_Z = 0.10

CARRIAGE_PLATE_THICKNESS = 0.018
CARRIAGE_WIDTH = 0.13
CARRIAGE_HEIGHT = 0.11
Z_HOME = 0.19
Z_TRAVEL = 0.28
CARRIAGE_X = BACK_THICKNESS / 2.0 + GUIDE_RAIL_THICKNESS + CARRIAGE_PLATE_THICKNESS / 2.0

SLIDE_GUIDE_DEPTH = 0.036
SLIDE_GUIDE_LENGTH = 0.10
SLIDE_GUIDE_THICKNESS = 0.010
SLIDE_GUIDE_CENTER_X = (
    CARRIAGE_PLATE_THICKNESS / 2.0 + SLIDE_GUIDE_DEPTH / 2.0 - 0.002
)

SLIDE_BEAM_DEPTH = 0.028
SLIDE_BEAM_LENGTH = 0.15
SLIDE_BEAM_HEIGHT = 0.04
SLIDE_BEAM_CENTER_Y = 0.005
SLIDE_JOINT_X = SLIDE_GUIDE_CENTER_X
SLIDE_OUTPUT_FACE_THICKNESS = 0.008
SLIDE_OUTPUT_FACE_DEPTH = 0.048
SLIDE_OUTPUT_FACE_HEIGHT = 0.065
SLIDE_OUTPUT_FACE_CENTER_Y = SLIDE_BEAM_CENTER_Y + SLIDE_BEAM_LENGTH / 2.0 + SLIDE_OUTPUT_FACE_THICKNESS / 2.0

Y_HOME_IN_CARRIAGE = 0.0
Y_TRAVEL = 0.09


def _frame_backbone_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )

    column = (
        cq.Workplane("XY")
        .box(BACK_THICKNESS, BACK_WIDTH, COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0))
    )

    top_cap = (
        cq.Workplane("XY")
        .box(BACK_THICKNESS + 0.02, BACK_WIDTH + 0.02, TOP_CAP_THICKNESS)
        .translate(
            (
                0.0,
                0.0,
                BASE_THICKNESS + COLUMN_HEIGHT - TOP_CAP_THICKNESS / 2.0,
            )
        )
    )

    backbone = base.union(column).union(top_cap)

    mounting_slots = (
        cq.Workplane("XY")
        .pushPoints([(0.0, -0.055), (0.0, 0.055)])
        .slot2D(0.05, 0.016, angle=90.0)
        .extrude(BASE_THICKNESS + 0.004)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0 - 0.002))
    )

    return backbone.cut(mounting_slots)


def _carriage_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_PLATE_THICKNESS, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)
        .edges("|Z")
        .fillet(0.006)
    )

    upper_guide = (
        cq.Workplane("XY")
        .box(SLIDE_GUIDE_DEPTH, SLIDE_GUIDE_LENGTH, SLIDE_GUIDE_THICKNESS)
        .translate(
            (
                SLIDE_GUIDE_CENTER_X,
                0.0,
                SLIDE_BEAM_HEIGHT / 2.0 + SLIDE_GUIDE_THICKNESS / 2.0,
            )
        )
    )
    lower_guide = (
        cq.Workplane("XY")
        .box(SLIDE_GUIDE_DEPTH, SLIDE_GUIDE_LENGTH, SLIDE_GUIDE_THICKNESS)
        .translate(
            (
                SLIDE_GUIDE_CENTER_X,
                0.0,
                -(SLIDE_BEAM_HEIGHT / 2.0 + SLIDE_GUIDE_THICKNESS / 2.0),
            )
        )
    )

    return plate.union(upper_guide).union(lower_guide)


def _slide_beam_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(SLIDE_BEAM_DEPTH, SLIDE_BEAM_LENGTH, SLIDE_BEAM_HEIGHT)
        .translate((0.0, SLIDE_BEAM_CENTER_Y, 0.0))
    )

    front_chamfer = (
        cq.Workplane("XY")
        .box(SLIDE_BEAM_DEPTH * 0.55, 0.02, SLIDE_BEAM_HEIGHT * 0.45)
        .translate((0.0, SLIDE_BEAM_CENTER_Y + SLIDE_BEAM_LENGTH / 2.0 - 0.01, 0.0))
    )

    return beam.cut(front_chamfer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_yz_transfer_module")

    model.material("painted_frame", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("guide_steel", rgba=(0.67, 0.70, 0.74, 1.0))
    model.material("carriage_body", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("slide_body", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("output_plate", rgba=(0.16, 0.17, 0.19, 1.0))

    frame = model.part("upright_guide")
    frame.visual(
        mesh_from_cadquery(_frame_backbone_shape(), "upright_guide_backbone"),
        material="painted_frame",
        name="frame_backbone",
    )
    frame.visual(
        Box((GUIDE_RAIL_THICKNESS, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS / 2.0 + GUIDE_RAIL_THICKNESS / 2.0,
                -GUIDE_RAIL_SPACING / 2.0,
                GUIDE_RAIL_BASE_Z + GUIDE_RAIL_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="left_guide_rail",
    )
    frame.visual(
        Box((GUIDE_RAIL_THICKNESS, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS / 2.0 + GUIDE_RAIL_THICKNESS / 2.0,
                GUIDE_RAIL_SPACING / 2.0,
                GUIDE_RAIL_BASE_Z + GUIDE_RAIL_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="right_guide_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, COLUMN_HEIGHT + BASE_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_HEIGHT + BASE_THICKNESS) / 2.0)),
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((CARRIAGE_PLATE_THICKNESS, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        material="carriage_body",
        name="carriage_plate",
    )
    z_carriage.visual(
        Box((SLIDE_GUIDE_DEPTH, SLIDE_GUIDE_LENGTH, SLIDE_GUIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                SLIDE_GUIDE_CENTER_X,
                0.0,
                SLIDE_BEAM_HEIGHT / 2.0 + SLIDE_GUIDE_THICKNESS / 2.0,
            )
        ),
        material="carriage_body",
        name="upper_slide_guide",
    )
    z_carriage.visual(
        Box((SLIDE_GUIDE_DEPTH, SLIDE_GUIDE_LENGTH, SLIDE_GUIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                SLIDE_GUIDE_CENTER_X,
                0.0,
                -(SLIDE_BEAM_HEIGHT / 2.0 + SLIDE_GUIDE_THICKNESS / 2.0),
            )
        ),
        material="carriage_body",
        name="lower_slide_guide",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((SLIDE_GUIDE_CENTER_X * 2.0 + SLIDE_GUIDE_DEPTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=4.6,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        Box((SLIDE_BEAM_DEPTH, SLIDE_BEAM_LENGTH, SLIDE_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, SLIDE_BEAM_CENTER_Y, 0.0)),
        material="slide_body",
        name="slide_beam",
    )
    y_slide.visual(
        Box(
            (
                SLIDE_OUTPUT_FACE_DEPTH,
                SLIDE_OUTPUT_FACE_THICKNESS,
                SLIDE_OUTPUT_FACE_HEIGHT,
            )
        ),
        origin=Origin(xyz=(0.0, SLIDE_OUTPUT_FACE_CENTER_Y, 0.0)),
        material="output_plate",
        name="output_face",
    )
    y_slide.inertial = Inertial.from_geometry(
        Box((SLIDE_OUTPUT_FACE_DEPTH, 0.148, SLIDE_OUTPUT_FACE_HEIGHT)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
    )

    model.articulation(
        "upright_to_z_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=z_carriage,
        origin=Origin(xyz=(CARRIAGE_X, 0.0, Z_HOME)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=900.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "z_carriage_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=z_carriage,
        child=y_slide,
        origin=Origin(xyz=(SLIDE_JOINT_X, Y_HOME_IN_CARRIAGE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Y_TRAVEL,
            effort=300.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("upright_guide")
    z_carriage = object_model.get_part("z_carriage")
    y_slide = object_model.get_part("y_slide")
    z_axis = object_model.get_articulation("upright_to_z_carriage")
    y_axis = object_model.get_articulation("z_carriage_to_y_slide")

    frame.get_visual("left_guide_rail")
    z_carriage.get_visual("carriage_plate")
    z_carriage.get_visual("upper_slide_guide")
    y_slide.get_visual("slide_beam")
    y_slide.get_visual("output_face")

    ctx.expect_gap(
        z_carriage,
        frame,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        negative_elem="left_guide_rail",
        name="z carriage rides just ahead of the guide rails",
    )
    ctx.expect_overlap(
        z_carriage,
        frame,
        axes="yz",
        elem_b="left_guide_rail",
        min_overlap=0.015,
        name="z carriage stays aligned with the left guide rail footprint",
    )
    ctx.expect_contact(
        z_carriage,
        frame,
        elem_a="carriage_plate",
        elem_b="right_guide_rail",
        name="z carriage plate bears against the right guide rail",
    )

    ctx.expect_within(
        y_slide,
        z_carriage,
        axes="xz",
        inner_elem="slide_beam",
        margin=0.003,
        name="resting y slide beam stays captured in the carriage guide",
    )
    ctx.expect_overlap(
        y_slide,
        z_carriage,
        axes="y",
        elem_a="slide_beam",
        min_overlap=0.06,
        name="resting y slide retains substantial insertion",
    )
    ctx.expect_contact(
        y_slide,
        z_carriage,
        elem_a="slide_beam",
        elem_b="upper_slide_guide",
        name="resting y slide beam bears on the upper guide pad",
    )

    rest_carriage_pos = ctx.part_world_position(z_carriage)
    with ctx.pose({z_axis: Z_TRAVEL * 0.5}):
        rest_slide_pos = ctx.part_world_position(y_slide)
        with ctx.pose({z_axis: Z_TRAVEL * 0.5, y_axis: Y_TRAVEL}):
            ctx.expect_within(
                y_slide,
                z_carriage,
                axes="xz",
                inner_elem="slide_beam",
                margin=0.003,
                name="extended y slide beam stays guided on the non-motion axes",
            )
            ctx.expect_overlap(
                y_slide,
                z_carriage,
                axes="y",
                elem_a="slide_beam",
                min_overlap=0.03,
                name="extended y slide still retains insertion in the carriage guide",
            )
            ctx.expect_contact(
                y_slide,
                z_carriage,
                elem_a="slide_beam",
                elem_b="upper_slide_guide",
                name="extended y slide beam still bears on the upper guide pad",
            )
            extended_slide_pos = ctx.part_world_position(y_slide)

    with ctx.pose({z_axis: Z_TRAVEL}):
        ctx.expect_overlap(
            z_carriage,
            frame,
            axes="yz",
            elem_b="right_guide_rail",
            min_overlap=0.015,
            name="raised carriage remains on the guide rail footprint",
        )
        raised_carriage_pos = ctx.part_world_position(z_carriage)

    ctx.check(
        "z carriage moves upward along +Z",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.20
        and abs(raised_carriage_pos[1] - rest_carriage_pos[1]) < 1e-6,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )
    ctx.check(
        "y slide moves laterally along +Y",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[1] > rest_slide_pos[1] + 0.06
        and abs(extended_slide_pos[2] - rest_slide_pos[2]) < 1e-6,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
