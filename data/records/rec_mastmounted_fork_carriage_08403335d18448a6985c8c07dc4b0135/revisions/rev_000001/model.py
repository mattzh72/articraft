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


MAST_HEIGHT = 0.90
COLUMN_DEPTH = 0.055
COLUMN_WIDTH = 0.075
COLUMN_CENTER_Y = 0.215
CROSSHEAD_HEIGHT = 0.075
CROSSHEAD_DEPTH = 0.060
LOWER_TIE_HEIGHT = 0.060
LOWER_TIE_DEPTH = 0.050
BACK_BRACE_HEIGHT = 0.240
BACK_BRACE_DEPTH = 0.020
BACK_BRACE_WIDTH = 0.250

GUIDE_HEIGHT = 0.240
GUIDE_OUTER_DEPTH = 0.095
GUIDE_OUTER_WIDTH = 0.110
GUIDE_WALL = 0.012
GUIDE_CENTER_Z = 0.160
GUIDE_SIDE_CLEARANCE = 0.0025
GUIDE_SIDE_PLATE_WIDTH = 0.016
GUIDE_SIDE_PLATE_DEPTH = 0.072
GUIDE_SIDE_PLATE_OFFSET = (
    COLUMN_WIDTH / 2.0 + GUIDE_SIDE_CLEARANCE + GUIDE_SIDE_PLATE_WIDTH / 2.0
)
GUIDE_FRONT_SHOE_DEPTH = 0.010
GUIDE_FRONT_SHOE_HEIGHT = 0.160
GUIDE_FRONT_SHOE_CENTER_X = COLUMN_DEPTH / 2.0 + GUIDE_FRONT_SHOE_DEPTH / 2.0

LOWER_BEAM_HEIGHT = 0.060
LOWER_BEAM_DEPTH = 0.060
LOWER_BEAM_WIDTH = 0.340
MID_BRACE_HEIGHT = 0.035
MID_BRACE_DEPTH = 0.040
MID_BRACE_WIDTH = 0.300
UPPER_BAR_HEIGHT = 0.040
UPPER_BAR_DEPTH = 0.045
UPPER_BAR_WIDTH = 0.340
BACKPLATE_HEIGHT = 0.190
BACKPLATE_DEPTH = 0.016
BACKPLATE_WIDTH = 0.300

FORK_LENGTH = 0.340
FORK_HEEL_DEPTH = 0.060
FORK_HEEL_HEIGHT = 0.100
FORK_BLADE_THICKNESS = 0.018
FORK_WIDTH = 0.055
FORK_CENTER_Y = 0.120

CARRIAGE_REST_Z = 0.025
CARRIAGE_TRAVEL = 0.420


def _guide_shape() -> cq.Workplane:
    left_plate = cq.Workplane("XY").box(
        GUIDE_SIDE_PLATE_DEPTH,
        GUIDE_SIDE_PLATE_WIDTH,
        GUIDE_HEIGHT,
    ).translate((0.0, GUIDE_SIDE_PLATE_OFFSET, 0.0))
    right_plate = cq.Workplane("XY").box(
        GUIDE_SIDE_PLATE_DEPTH,
        GUIDE_SIDE_PLATE_WIDTH,
        GUIDE_HEIGHT,
    ).translate((0.0, -GUIDE_SIDE_PLATE_OFFSET, 0.0))
    front_shoe = cq.Workplane("XY").box(
        GUIDE_FRONT_SHOE_DEPTH,
        GUIDE_OUTER_WIDTH,
        GUIDE_FRONT_SHOE_HEIGHT,
    ).translate((GUIDE_FRONT_SHOE_CENTER_X, 0.0, 0.0))
    return left_plate.union(right_plate).union(front_shoe)


def _fork_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, 0.0),
                (FORK_LENGTH, 0.0),
                (FORK_LENGTH, FORK_BLADE_THICKNESS),
                (FORK_HEEL_DEPTH, FORK_BLADE_THICKNESS),
                (FORK_HEEL_DEPTH, FORK_HEEL_HEIGHT),
                (0.0, FORK_HEEL_HEIGHT),
            ]
        )
        .close()
    )
    return profile.extrude(FORK_WIDTH, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_mast_fork_module")

    model.material("mast_steel", rgba=(0.26, 0.29, 0.33, 1.0))
    model.material("carriage_steel", rgba=(0.34, 0.37, 0.41, 1.0))
    model.material("fork_steel", rgba=(0.16, 0.17, 0.18, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((COLUMN_DEPTH, COLUMN_WIDTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, COLUMN_CENTER_Y, MAST_HEIGHT / 2.0)),
        material="mast_steel",
        name="left_column",
    )
    mast.visual(
        Box((COLUMN_DEPTH, COLUMN_WIDTH, MAST_HEIGHT)),
        origin=Origin(xyz=(0.0, -COLUMN_CENTER_Y, MAST_HEIGHT / 2.0)),
        material="mast_steel",
        name="right_column",
    )
    mast.visual(
        Box((CROSSHEAD_DEPTH, 2.0 * COLUMN_CENTER_Y + COLUMN_WIDTH, CROSSHEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT - CROSSHEAD_HEIGHT / 2.0)),
        material="mast_steel",
        name="crosshead",
    )
    mast.visual(
        Box((LOWER_TIE_DEPTH, 2.0 * COLUMN_CENTER_Y + 0.030, LOWER_TIE_HEIGHT)),
        origin=Origin(xyz=(-0.030, 0.0, LOWER_TIE_HEIGHT / 2.0)),
        material="mast_steel",
        name="lower_tie",
    )
    mast.visual(
        Box((BACK_BRACE_DEPTH, BACK_BRACE_WIDTH, BACK_BRACE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(COLUMN_DEPTH + BACK_BRACE_DEPTH) / 2.0 + 0.006,
                0.0,
                BACK_BRACE_HEIGHT / 2.0,
            )
        ),
        material="mast_steel",
        name="back_brace",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.10, 2.0 * COLUMN_CENTER_Y + COLUMN_WIDTH, MAST_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    fork_mesh = _fork_shape()
    carriage.visual(
        Box((GUIDE_SIDE_PLATE_DEPTH, GUIDE_SIDE_PLATE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, COLUMN_CENTER_Y + GUIDE_SIDE_PLATE_OFFSET, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="left_guide_outer_rail",
    )
    carriage.visual(
        Box((GUIDE_SIDE_PLATE_DEPTH, GUIDE_SIDE_PLATE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, COLUMN_CENTER_Y - GUIDE_SIDE_PLATE_OFFSET, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="left_guide_inner_rail",
    )
    carriage.visual(
        Box((GUIDE_FRONT_SHOE_DEPTH, GUIDE_OUTER_WIDTH, GUIDE_FRONT_SHOE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_FRONT_SHOE_CENTER_X, COLUMN_CENTER_Y, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="left_guide_front_pad",
    )
    carriage.visual(
        Box((GUIDE_SIDE_PLATE_DEPTH, GUIDE_SIDE_PLATE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, -COLUMN_CENTER_Y + GUIDE_SIDE_PLATE_OFFSET, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="right_guide_inner_rail",
    )
    carriage.visual(
        Box((GUIDE_SIDE_PLATE_DEPTH, GUIDE_SIDE_PLATE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, -COLUMN_CENTER_Y - GUIDE_SIDE_PLATE_OFFSET, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="right_guide_outer_rail",
    )
    carriage.visual(
        Box((GUIDE_FRONT_SHOE_DEPTH, GUIDE_OUTER_WIDTH, GUIDE_FRONT_SHOE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_FRONT_SHOE_CENTER_X, -COLUMN_CENTER_Y, GUIDE_CENTER_Z)),
        material="carriage_steel",
        name="right_guide_front_pad",
    )
    carriage.visual(
        Box((LOWER_BEAM_DEPTH, LOWER_BEAM_WIDTH, LOWER_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.045, 0.0, LOWER_BEAM_HEIGHT / 2.0)),
        material="carriage_steel",
        name="lower_beam",
    )
    carriage.visual(
        Box((MID_BRACE_DEPTH, MID_BRACE_WIDTH, MID_BRACE_HEIGHT)),
        origin=Origin(xyz=(0.030, 0.0, 0.160)),
        material="carriage_steel",
        name="mid_brace",
    )
    carriage.visual(
        Box((UPPER_BAR_DEPTH, UPPER_BAR_WIDTH, UPPER_BAR_HEIGHT)),
        origin=Origin(xyz=(0.035, 0.0, 0.270)),
        material="carriage_steel",
        name="upper_bar",
    )
    carriage.visual(
        Box((BACKPLATE_DEPTH, BACKPLATE_WIDTH, BACKPLATE_HEIGHT)),
        origin=Origin(xyz=(0.058, 0.0, 0.155)),
        material="carriage_steel",
        name="backplate",
    )
    carriage.visual(
        mesh_from_cadquery(fork_mesh, "left_fork"),
        origin=Origin(xyz=(0.0, FORK_CENTER_Y, 0.0)),
        material="fork_steel",
        name="left_fork",
    )
    carriage.visual(
        mesh_from_cadquery(fork_mesh, "right_fork"),
        origin=Origin(xyz=(0.0, -FORK_CENTER_Y, 0.0)),
        material="fork_steel",
        name="right_fork",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((FORK_LENGTH, 2.0 * COLUMN_CENTER_Y + GUIDE_OUTER_WIDTH, 0.300)),
        mass=22.0,
        origin=Origin(xyz=(FORK_LENGTH / 2.0, 0.0, 0.150)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.220,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        positive_elem="left_guide_outer_rail",
        negative_elem="left_column",
        min_gap=0.0024,
        max_gap=0.0026,
        name="left outer guide rail clears the outside of the left mast column",
    )
    ctx.expect_gap(
        mast,
        carriage,
        axis="y",
        positive_elem="left_column",
        negative_elem="left_guide_inner_rail",
        min_gap=0.0024,
        max_gap=0.0026,
        name="left inner guide rail clears the inside of the left mast column",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a="left_guide_front_pad",
        elem_b="left_column",
        contact_tol=1e-6,
        name="left guide front pad bears on the left mast column at rest",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="left_guide_front_pad",
        elem_b="left_column",
        min_overlap=0.159,
        name="left guide stays vertically engaged on the mast at rest",
    )
    ctx.expect_contact(
        carriage,
        carriage,
        elem_a="mid_brace",
        elem_b="backplate",
        contact_tol=1e-6,
        name="mid brace is tied into the carriage backplate",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="backplate",
        negative_elem="left_column",
        min_gap=0.015,
        name="carriage backplate sits forward of the mast uprights",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            mast,
            axes="y",
            elem_a="left_guide_front_pad",
            elem_b="left_column",
            min_overlap=0.074,
            name="left guide front pad keeps spanning the left column at full lift",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="right_guide_front_pad",
            elem_b="right_column",
            min_overlap=0.159,
            name="right guide front pad retains vertical engagement at full lift",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem="right_guide_inner_rail",
            negative_elem="right_column",
            min_gap=0.0024,
            max_gap=0.0026,
            name="right inner guide rail clears the inside of the right mast column at full lift",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            positive_elem="right_column",
            negative_elem="right_guide_outer_rail",
            min_gap=0.0024,
            max_gap=0.0026,
            name="right outer guide rail clears the outside of the right mast column at full lift",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="right_guide_front_pad",
            elem_b="right_column",
            contact_tol=1e-6,
            name="right guide front pad stays in contact with the right mast column at full lift",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage lifts upward along the mast axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + CARRIAGE_TRAVEL - 0.01,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
