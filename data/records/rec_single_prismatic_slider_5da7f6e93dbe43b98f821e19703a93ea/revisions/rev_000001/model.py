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


BASE_LENGTH = 0.150
BASE_WIDTH = 0.070
BASE_THICKNESS = 0.012
BASE_CORNER_RADIUS = 0.005

MOUNT_HOLE_DIAMETER = 0.007
MOUNT_CBORE_DIAMETER = 0.012
MOUNT_CBORE_DEPTH = 0.004
MOUNT_X = 0.052
MOUNT_Y = 0.022

RAIL_LENGTH = 0.100
RAIL_WIDTH = 0.022
RAIL_HEIGHT = 0.018
RAIL_TOP_FILLET = 0.0015
RAIL_CENTER_Z = RAIL_HEIGHT / 2.0

RAIL_CAP_LENGTH = 0.004
RAIL_CAP_WIDTH = 0.018
RAIL_CAP_HEIGHT = 0.016
RAIL_CAP_CENTER_Z = RAIL_CAP_HEIGHT / 2.0

STOP_LENGTH = 0.006
STOP_WIDTH = 0.006
STOP_HEIGHT = 0.014
STOP_OFFSET_Y = 0.016
STOP_FACE_GAP = 0.001

CLAMP_LENGTH = 0.044
CLAMP_WIDTH = 0.050
CLAMP_HEIGHT = 0.034
CLAMP_BOTTOM_CLEARANCE = 0.0015
CLAMP_BOTTOM_Z = CLAMP_BOTTOM_CLEARANCE - RAIL_CENTER_Z
CLAMP_CHANNEL_WIDTH = RAIL_WIDTH
CLAMP_CHANNEL_TOP_Z = RAIL_HEIGHT - RAIL_CENTER_Z
CLAMP_CHANNEL_HEIGHT = CLAMP_CHANNEL_TOP_Z - CLAMP_BOTTOM_Z
CLAMP_CORNER_RADIUS = 0.003
CLAMP_SIDE_WALL = (CLAMP_WIDTH - CLAMP_CHANNEL_WIDTH) / 2.0
CLAMP_ROOF_CLEARANCE = 0.0015
CLAMP_ROOF_BOTTOM_Z = CLAMP_CHANNEL_TOP_Z + CLAMP_ROOF_CLEARANCE
CLAMP_ROOF_THICKNESS = (CLAMP_BOTTOM_Z + CLAMP_HEIGHT) - CLAMP_ROOF_BOTTOM_Z
CLAMP_END_WEB_LENGTH = 0.005
CLAMP_END_WEB_CENTER_Z = CLAMP_ROOF_BOTTOM_Z + (CLAMP_ROOF_THICKNESS / 2.0)

PAD_LENGTH = 0.030
PAD_WIDTH = 0.018
PAD_THICKNESS = 0.005
PAD_CENTER_Z = CLAMP_BOTTOM_Z + CLAMP_HEIGHT + (PAD_THICKNESS / 2.0)

SCREW_RADIUS = 0.0035
SCREW_LENGTH = 0.012
SCREW_OFFSET_X = 0.011
SCREW_CENTER_Z = PAD_CENTER_Z + (PAD_THICKNESS / 2.0) - 0.004

TRAVEL_HALF_RANGE = 0.0225
LEFT_STOP_CENTER_X = -(CLAMP_LENGTH / 2.0 + TRAVEL_HALF_RANGE + STOP_FACE_GAP + (STOP_LENGTH / 2.0))
RIGHT_STOP_CENTER_X = -LEFT_STOP_CENTER_X
LEFT_CAP_CENTER_X = -(RAIL_LENGTH / 2.0 + (RAIL_CAP_LENGTH / 2.0))
RIGHT_CAP_CENTER_X = -LEFT_CAP_CENTER_X


def _base_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .translate((0.0, 0.0, -BASE_THICKNESS))
        .edges("|Z")
        .fillet(BASE_CORNER_RADIUS)
    )
    return (
        plate.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-MOUNT_X, -MOUNT_Y),
                (-MOUNT_X, MOUNT_Y),
                (MOUNT_X, -MOUNT_Y),
                (MOUNT_X, MOUNT_Y),
            ]
        )
        .cboreHole(MOUNT_HOLE_DIAMETER, MOUNT_CBORE_DIAMETER, MOUNT_CBORE_DEPTH)
    )


def _rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_HEIGHT,
        centered=(True, True, False),
    )
    return rail.faces(">Z").edges("|X").fillet(RAIL_TOP_FILLET)


def _stop_pair_shape(x_center: float) -> cq.Workplane:
    left_lug = (
        cq.Workplane("XY")
        .box(STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT, centered=(True, True, False))
        .translate((x_center, STOP_OFFSET_Y, 0.0))
    )
    right_lug = (
        cq.Workplane("XY")
        .box(STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT, centered=(True, True, False))
        .translate((x_center, -STOP_OFFSET_Y, 0.0))
    )
    return left_lug.union(right_lug)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_adjustment_module")

    model.material("base_painted_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    model.material("rail_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("black_oxide", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("carriage_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    model.material("pad_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        material="base_painted_steel",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape(), "guide_rail"),
        material="rail_steel",
        name="guide_rail",
    )
    base.visual(
        Box((RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT)),
        origin=Origin(xyz=(LEFT_CAP_CENTER_X, 0.0, RAIL_CAP_CENTER_Z)),
        material="black_oxide",
        name="left_rail_cap",
    )
    base.visual(
        Box((RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_HEIGHT)),
        origin=Origin(xyz=(RIGHT_CAP_CENTER_X, 0.0, RAIL_CAP_CENTER_Z)),
        material="black_oxide",
        name="right_rail_cap",
    )
    base.visual(
        mesh_from_cadquery(_stop_pair_shape(LEFT_STOP_CENTER_X), "left_stop_pair"),
        material="black_oxide",
        name="left_end_stop",
    )
    base.visual(
        mesh_from_cadquery(_stop_pair_shape(RIGHT_STOP_CENTER_X), "right_stop_pair"),
        material="black_oxide",
        name="right_end_stop",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + RAIL_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_HEIGHT - BASE_THICKNESS) / 2.0)),
    )

    clamp = model.part("clamp")
    clamp.visual(
        Box((CLAMP_LENGTH, CLAMP_SIDE_WALL, CLAMP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (CLAMP_CHANNEL_WIDTH / 2.0) + (CLAMP_SIDE_WALL / 2.0),
                CLAMP_BOTTOM_Z + (CLAMP_HEIGHT / 2.0),
            )
        ),
        material="carriage_steel",
        name="left_carriage_wall",
    )
    clamp.visual(
        Box((CLAMP_LENGTH, CLAMP_SIDE_WALL, CLAMP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -((CLAMP_CHANNEL_WIDTH / 2.0) + (CLAMP_SIDE_WALL / 2.0)),
                CLAMP_BOTTOM_Z + (CLAMP_HEIGHT / 2.0),
            )
        ),
        material="carriage_steel",
        name="right_carriage_wall",
    )
    clamp.visual(
        Box((CLAMP_LENGTH, CLAMP_WIDTH, CLAMP_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CLAMP_ROOF_BOTTOM_Z + (CLAMP_ROOF_THICKNESS / 2.0),
            )
        ),
        material="carriage_steel",
        name="carriage_roof",
    )
    clamp.visual(
        Box((CLAMP_END_WEB_LENGTH, CLAMP_WIDTH, CLAMP_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(
                -(CLAMP_LENGTH / 2.0) + (CLAMP_END_WEB_LENGTH / 2.0),
                0.0,
                CLAMP_END_WEB_CENTER_Z,
            )
        ),
        material="carriage_steel",
        name="front_carriage_web",
    )
    clamp.visual(
        Box((CLAMP_END_WEB_LENGTH, CLAMP_WIDTH, CLAMP_ROOF_THICKNESS)),
        origin=Origin(
            xyz=(
                (CLAMP_LENGTH / 2.0) - (CLAMP_END_WEB_LENGTH / 2.0),
                0.0,
                CLAMP_END_WEB_CENTER_Z,
            )
        ),
        material="carriage_steel",
        name="rear_carriage_web",
    )
    clamp.visual(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PAD_CENTER_Z)),
        material="pad_rubber",
        name="top_pad",
    )
    clamp.visual(
        Cylinder(radius=SCREW_RADIUS, length=SCREW_LENGTH),
        origin=Origin(xyz=(-SCREW_OFFSET_X, 0.0, SCREW_CENTER_Z)),
        material="black_oxide",
        name="left_clamp_screw",
    )
    clamp.visual(
        Cylinder(radius=SCREW_RADIUS, length=SCREW_LENGTH),
        origin=Origin(xyz=(SCREW_OFFSET_X, 0.0, SCREW_CENTER_Z)),
        material="black_oxide",
        name="right_clamp_screw",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((CLAMP_LENGTH, CLAMP_WIDTH, CLAMP_HEIGHT + PAD_THICKNESS)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, (CLAMP_BOTTOM_Z + CLAMP_HEIGHT + PAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_clamp",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, RAIL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.18,
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    clamp = object_model.get_part("clamp")
    slide = object_model.get_articulation("base_to_clamp")

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

    axis_ok = tuple(float(v) for v in slide.axis) == (1.0, 0.0, 0.0)
    limits = slide.motion_limits
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + TRAVEL_HALF_RANGE) < 1e-9
        and abs(limits.upper - TRAVEL_HALF_RANGE) < 1e-9
    )
    ctx.check(
        "prismatic_slide_definition",
        slide.articulation_type == ArticulationType.PRISMATIC and axis_ok and limits_ok,
        details=f"expected x-axis prismatic travel ±{TRAVEL_HALF_RANGE:.4f} m",
    )

    ctx.expect_contact(
        base,
        clamp,
        elem_a="guide_rail",
        elem_b="left_carriage_wall",
        name="clamp_block_is_supported_by_rail",
    )
    ctx.expect_contact(
        base,
        clamp,
        elem_a="guide_rail",
        elem_b="right_carriage_wall",
        name="clamp_block_is_captured_on_both_rail_faces",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="z",
        positive_elem="left_carriage_wall",
        negative_elem="base_plate",
        min_gap=CLAMP_BOTTOM_CLEARANCE - 1e-6,
        max_gap=CLAMP_BOTTOM_CLEARANCE + 0.0003,
        name="clamp_clears_base_plate",
    )

    with ctx.pose({slide: limits.lower}):
        ctx.expect_contact(
            base,
            clamp,
            elem_a="guide_rail",
            elem_b="left_carriage_wall",
            name="lower_limit_stays_on_guide_rail",
        )
        ctx.expect_contact(
            base,
            clamp,
            elem_a="guide_rail",
            elem_b="right_carriage_wall",
            name="lower_limit_keeps_both_guide_faces_engaged",
        )
        ctx.expect_overlap(
            base,
            clamp,
            axes="x",
            min_overlap=0.020,
            name="lower_limit_keeps_guide_overlap",
        )
        ctx.expect_gap(
            clamp,
            base,
            axis="x",
            positive_elem="front_carriage_web",
            negative_elem="left_end_stop",
            min_gap=STOP_FACE_GAP - 1e-6,
            max_gap=STOP_FACE_GAP + 0.0003,
            name="lower_limit_clears_left_stop",
        )

    with ctx.pose({slide: limits.upper}):
        ctx.expect_contact(
            base,
            clamp,
            elem_a="guide_rail",
            elem_b="left_carriage_wall",
            name="upper_limit_stays_on_guide_rail",
        )
        ctx.expect_contact(
            base,
            clamp,
            elem_a="guide_rail",
            elem_b="right_carriage_wall",
            name="upper_limit_keeps_both_guide_faces_engaged",
        )
        ctx.expect_overlap(
            base,
            clamp,
            axes="x",
            min_overlap=0.020,
            name="upper_limit_keeps_guide_overlap",
        )
        ctx.expect_gap(
            base,
            clamp,
            axis="x",
            positive_elem="right_end_stop",
            negative_elem="rear_carriage_web",
            min_gap=STOP_FACE_GAP - 1e-6,
            max_gap=STOP_FACE_GAP + 0.0003,
            name="upper_limit_clears_right_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
