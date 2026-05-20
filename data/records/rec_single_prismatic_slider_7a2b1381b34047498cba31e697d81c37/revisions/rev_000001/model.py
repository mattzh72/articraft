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


RAIL_LENGTH = 0.300
BASE_WIDTH = 0.020
BASE_THICKNESS = 0.0055
MOUNT_HOLE_DIAMETER = 0.0042

GUIDE_LENGTH = 0.272
GUIDE_BASE_WIDTH = 0.0155
GUIDE_TOP_WIDTH = 0.0115
GUIDE_HEIGHT = 0.0098
GUIDE_BLEND = 0.0003

STOP_LENGTH = 0.006
STOP_WIDTH = 0.0165
STOP_HEIGHT = 0.0090
STOP_END_MARGIN = 0.004

CARRIAGE_LENGTH = 0.042
CARRIAGE_WIDTH = 0.0235
CARRIAGE_HEIGHT = 0.0170
CARRIAGE_BOTTOM_CLEARANCE = 0.00035
CAVITY_WIDTH = 0.0172
CAVITY_HEIGHT = GUIDE_HEIGHT - CARRIAGE_BOTTOM_CLEARANCE
SIDE_CHEEK_WIDTH = (CARRIAGE_WIDTH - CAVITY_WIDTH) / 2.0
TOP_CAP_THICKNESS = CARRIAGE_HEIGHT - CAVITY_HEIGHT

TRAVEL_HALF_RANGE = 0.108
RAIL_OVERALL_HEIGHT = max(BASE_THICKNESS + GUIDE_HEIGHT, BASE_THICKNESS + STOP_HEIGHT)
STOP_CENTER_X = (RAIL_LENGTH / 2.0) - (STOP_LENGTH / 2.0) - STOP_END_MARGIN
HOLE_POSITIONS_X = (-0.105, -0.035, 0.035, 0.105)


def _build_mount_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.001)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(x, 0.0) for x in HOLE_POSITIONS_X])
        .hole(MOUNT_HOLE_DIAMETER)
    )


def _build_guide_rail_shape() -> cq.Workplane:
    z0 = BASE_THICKNESS - GUIDE_BLEND
    z1 = BASE_THICKNESS + GUIDE_HEIGHT
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-GUIDE_BASE_WIDTH / 2.0, z0),
                (GUIDE_BASE_WIDTH / 2.0, z0),
                (GUIDE_TOP_WIDTH / 2.0, z1),
                (-GUIDE_TOP_WIDTH / 2.0, z1),
            ]
        )
        .close()
        .extrude(GUIDE_LENGTH / 2.0, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_bench_slider")

    model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_aluminum", rgba=(0.74, 0.76, 0.80, 1.0))
    model.material("stop_black", rgba=(0.12, 0.12, 0.13, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_build_mount_base_shape(), "bench_slider_mount_base"),
        material="rail_steel",
        name="mount_base",
    )
    rail.visual(
        mesh_from_cadquery(_build_guide_rail_shape(), "bench_slider_guide_rail"),
        material="rail_steel",
        name="guide_rail",
    )
    rail.visual(
        Box((STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                -STOP_CENTER_X,
                0.0,
                BASE_THICKNESS + (STOP_HEIGHT / 2.0) - (GUIDE_BLEND / 2.0),
            )
        ),
        material="stop_black",
        name="left_stop",
    )
    rail.visual(
        Box((STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                STOP_CENTER_X,
                0.0,
                BASE_THICKNESS + (STOP_HEIGHT / 2.0) - (GUIDE_BLEND / 2.0),
            )
        ),
        material="stop_black",
        name="right_stop",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, BASE_WIDTH, RAIL_OVERALL_HEIGHT)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, RAIL_OVERALL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, TOP_CAP_THICKNESS)),
        material="carriage_aluminum",
        origin=Origin(xyz=(0.0, 0.0, CAVITY_HEIGHT + (TOP_CAP_THICKNESS / 2.0))),
        name="carriage_top",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SIDE_CHEEK_WIDTH, CARRIAGE_HEIGHT)),
        material="carriage_aluminum",
        origin=Origin(
            xyz=(
                0.0,
                (CAVITY_WIDTH / 2.0) + (SIDE_CHEEK_WIDTH / 2.0),
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
        name="right_cheek",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, SIDE_CHEEK_WIDTH, CARRIAGE_HEIGHT)),
        material="carriage_aluminum",
        origin=Origin(
            xyz=(
                0.0,
                -((CAVITY_WIDTH / 2.0) + (SIDE_CHEEK_WIDTH / 2.0)),
                CARRIAGE_HEIGHT / 2.0,
            )
        ),
        name="left_cheek",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + CARRIAGE_BOTTOM_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=120.0,
            velocity=0.30,
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

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")

    guide_rail = rail.get_visual("guide_rail")
    mount_base = rail.get_visual("mount_base")
    left_stop = rail.get_visual("left_stop")
    right_stop = rail.get_visual("right_stop")

    limits = slide.motion_limits
    ctx.check("rail part present", rail is not None)
    ctx.check("carriage part present", carriage is not None)
    ctx.check(
        "prismatic guide axis is correct",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == -TRAVEL_HALF_RANGE
        and limits.upper == TRAVEL_HALF_RANGE,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={limits}",
    )

    ctx.expect_within(
        rail,
        carriage,
        axes="y",
        inner_elem=guide_rail,
        name="guide rail stays inside carriage width",
    )
    ctx.expect_overlap(
        rail,
        carriage,
        axes="x",
        elem_a=guide_rail,
        min_overlap=0.040,
        name="carriage remains fully engaged over the guide at rest",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        negative_elem=mount_base,
        min_gap=0.0001,
        max_gap=0.0008,
        name="carriage rides just above the fixed base strip",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: TRAVEL_HALF_RANGE}):
        ctx.expect_within(
            rail,
            carriage,
            axes="y",
            inner_elem=guide_rail,
            name="extended carriage stays laterally centered on the guide",
        )
        ctx.expect_overlap(
            rail,
            carriage,
            axes="x",
            elem_a=guide_rail,
            min_overlap=0.040,
            name="extended carriage still overlaps the guide",
        )
        ctx.expect_gap(
            rail,
            carriage,
            axis="x",
            positive_elem=right_stop,
            min_gap=0.006,
            name="carriage clears the right stop at maximum travel",
        )
        right_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: -TRAVEL_HALF_RANGE}):
        ctx.expect_gap(
            carriage,
            rail,
            axis="x",
            negative_elem=left_stop,
            min_gap=0.006,
            name="carriage clears the left stop at minimum travel",
        )
        left_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the guide axis",
        rest_pos is not None
        and right_pos is not None
        and left_pos is not None
        and right_pos[0] > rest_pos[0] + 0.09
        and left_pos[0] < rest_pos[0] - 0.09,
        details=f"left={left_pos}, rest={rest_pos}, right={right_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
