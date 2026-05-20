from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.56
CABINET_HEIGHT = 0.70
CABINET_DEPTH = 0.12
BACK_THICKNESS = 0.006
WALL_THICKNESS = 0.014
SHELF_THICKNESS = 0.006

DOOR_THICKNESS = 0.022
DOOR_WIDTH = 0.554
DOOR_HEIGHT = 0.694
DOOR_START_X = 0.008
DOOR_FRAME_WIDTH = 0.032
DOOR_FRAME_DEPTH = 0.018
DOOR_BACKER_THICKNESS = 0.004
MIRROR_THICKNESS = 0.004

HINGE_AXIS_X = -0.285
HINGE_AXIS_Y = CABINET_DEPTH + DOOR_THICKNESS / 2.0
HINGE_BARREL_RADIUS = 0.0045
HINGE_DOOR_BARREL_RADIUS = 0.0041
HINGE_STACK_HEIGHT = 0.060
HINGE_CASE_KNUCKLE_LENGTH = 0.022
HINGE_DOOR_KNUCKLE_LENGTH = 0.016
UPPER_HINGE_Z = 0.225
LOWER_HINGE_Z = -0.225

KNOB_SPINDLE_LENGTH = 0.008
KNOB_SPINDLE_RADIUS = 0.0032
KNOB_BODY_LENGTH = 0.006
KNOB_BODY_RADIUS = 0.008


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) / 2.0,
        (lower[1] + upper[1]) / 2.0,
        (lower[2] + upper[2]) / 2.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_medicine_cabinet")

    case_finish = model.material("case_finish", color=(0.94, 0.95, 0.95))
    frame_finish = model.material("frame_finish", color=(0.76, 0.78, 0.80))
    mirror_finish = model.material("mirror_finish", color=(0.76, 0.82, 0.88, 1.0))
    hinge_finish = model.material("hinge_finish", color=(0.70, 0.71, 0.72))
    glass_finish = model.material("glass_finish", color=(0.84, 0.92, 0.96, 0.55))
    knob_finish = model.material("knob_finish", color=(0.78, 0.79, 0.80))

    case = model.part("cabinet_case")
    case.visual(
        Box((CABINET_WIDTH, BACK_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, BACK_THICKNESS / 2.0, 0.0)),
        material=case_finish,
        name="case_back_panel",
    )
    case.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + WALL_THICKNESS / 2.0, CABINET_DEPTH / 2.0, 0.0)),
        material=case_finish,
        name="case_left_wall",
    )
    case.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - WALL_THICKNESS / 2.0, CABINET_DEPTH / 2.0, 0.0)),
        material=case_finish,
        name="case_right_wall",
    )
    case.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0, CABINET_HEIGHT / 2.0 - WALL_THICKNESS / 2.0)),
        material=case_finish,
        name="case_top_wall",
    )
    case.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0, -CABINET_HEIGHT / 2.0 + WALL_THICKNESS / 2.0)),
        material=case_finish,
        name="case_bottom_wall",
    )
    case.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, CABINET_DEPTH - BACK_THICKNESS - 0.018, SHELF_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS + (CABINET_DEPTH - BACK_THICKNESS - 0.018) / 2.0,
                0.045,
            )
        ),
        material=glass_finish,
        name="case_shelf",
    )

    for hinge_z, prefix in ((UPPER_HINGE_Z, "upper"), (LOWER_HINGE_Z, "lower")):
        case.visual(
            Box((0.005, 0.0235, HINGE_STACK_HEIGHT)),
            origin=Origin(
                xyz=(
                    HINGE_AXIS_X + 0.0025,
                    0.1030 + 0.0235 / 2.0,
                    hinge_z,
                )
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_case_leaf",
        )
        case.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_CASE_KNUCKLE_LENGTH),
            origin=Origin(
                xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, hinge_z + 0.019),
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_case_knuckle_top",
        )
        case.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=HINGE_CASE_KNUCKLE_LENGTH),
            origin=Origin(
                xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, hinge_z - 0.019),
            ),
            material=hinge_finish,
            name=f"{prefix}_hinge_case_knuckle_bottom",
        )

    door = model.part("mirror_door")
    door_frame_y = -DOOR_THICKNESS / 2.0 + DOOR_FRAME_DEPTH / 2.0
    backer_y = -DOOR_THICKNESS / 2.0 + DOOR_BACKER_THICKNESS / 2.0
    mirror_y = DOOR_THICKNESS / 2.0 - MIRROR_THICKNESS / 2.0
    opening_width = DOOR_WIDTH - 2.0 * DOOR_FRAME_WIDTH
    opening_height = DOOR_HEIGHT - 2.0 * DOOR_FRAME_WIDTH

    door.visual(
        Box((DOOR_WIDTH, DOOR_BACKER_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH / 2.0, backer_y, 0.0)),
        material=case_finish,
        name="door_backer",
    )
    door.visual(
        Box((DOOR_FRAME_WIDTH, DOOR_FRAME_DEPTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_FRAME_WIDTH / 2.0, door_frame_y, 0.0)),
        material=frame_finish,
        name="door_frame_left",
    )
    door.visual(
        Box((DOOR_FRAME_WIDTH, DOOR_FRAME_DEPTH, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH - DOOR_FRAME_WIDTH / 2.0, door_frame_y, 0.0)),
        material=frame_finish,
        name="door_frame_right",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_FRAME_DEPTH, DOOR_FRAME_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_START_X + DOOR_WIDTH / 2.0,
                door_frame_y,
                DOOR_HEIGHT / 2.0 - DOOR_FRAME_WIDTH / 2.0,
            )
        ),
        material=frame_finish,
        name="door_frame_top",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_FRAME_DEPTH, DOOR_FRAME_WIDTH)),
        origin=Origin(
            xyz=(
                DOOR_START_X + DOOR_WIDTH / 2.0,
                door_frame_y,
                -DOOR_HEIGHT / 2.0 + DOOR_FRAME_WIDTH / 2.0,
            )
        ),
        material=frame_finish,
        name="door_frame_bottom",
    )
    door.visual(
        Box((opening_width, MIRROR_THICKNESS, opening_height)),
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH / 2.0, mirror_y, 0.0)),
        material=mirror_finish,
        name="door_mirror",
    )

    for hinge_z, prefix in ((UPPER_HINGE_Z, "upper"), (LOWER_HINGE_Z, "lower")):
        door.visual(
            Box((0.006, 0.012, HINGE_STACK_HEIGHT)),
            origin=Origin(xyz=(0.007, 0.0015, hinge_z)),
            material=hinge_finish,
            name=f"{prefix}_hinge_door_leaf",
        )
        door.visual(
            Cylinder(radius=HINGE_DOOR_BARREL_RADIUS, length=HINGE_DOOR_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=hinge_finish,
            name=f"{prefix}_hinge_door_knuckle",
        )

    catch_knob = model.part("catch_knob")
    catch_knob.visual(
        Cylinder(radius=KNOB_SPINDLE_RADIUS, length=KNOB_SPINDLE_LENGTH),
        origin=Origin(
            xyz=(KNOB_SPINDLE_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_finish,
        name="catch_spindle",
    )
    catch_knob.visual(
        Cylinder(radius=KNOB_BODY_RADIUS, length=KNOB_BODY_LENGTH),
        origin=Origin(
            xyz=(KNOB_SPINDLE_LENGTH + KNOB_BODY_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_finish,
        name="catch_body",
    )
    catch_knob.visual(
        Box((0.010, 0.006, 0.024)),
        origin=Origin(xyz=(KNOB_SPINDLE_LENGTH + 0.006, 0.006, 0.0)),
        material=knob_finish,
        name="catch_tab",
    )

    model.articulation(
        "case_to_door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "door_to_catch_knob",
        ArticulationType.REVOLUTE,
        parent=door,
        child=catch_knob,
        origin=Origin(xyz=(DOOR_START_X + DOOR_WIDTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-math.radians(50.0),
            upper=math.radians(50.0),
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

    case = object_model.get_part("cabinet_case")
    door = object_model.get_part("mirror_door")
    catch_knob = object_model.get_part("catch_knob")
    door_hinge = object_model.get_articulation("case_to_door_hinge")
    knob_pivot = object_model.get_articulation("door_to_catch_knob")

    ctx.check(
        "door hinge uses vertical side axis",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "catch knob rotates around its short spindle axis",
        tuple(knob_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={knob_pivot.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            case,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            positive_elem="door_backer",
            negative_elem="case_right_wall",
            name="door closes flush against cabinet front",
        )
        ctx.expect_overlap(
            door,
            case,
            axes="xz",
            min_overlap=0.50,
            name="door covers cabinet opening",
        )
        ctx.expect_contact(
            catch_knob,
            door,
            elem_a="catch_spindle",
            elem_b="door_frame_right",
            name="catch knob mounts to the free edge frame",
        )

    closed_edge = ctx.part_element_world_aabb(door, elem="door_frame_right")
    with ctx.pose({door_hinge: math.radians(75.0)}):
        opened_edge = ctx.part_element_world_aabb(door, elem="door_frame_right")
    ctx.check(
        "door swings outward from the case",
        closed_edge is not None
        and opened_edge is not None
        and opened_edge[1][1] > closed_edge[1][1] + 0.10,
        details=f"closed_edge={closed_edge}, opened_edge={opened_edge}",
    )

    knob_rest = ctx.part_element_world_aabb(catch_knob, elem="catch_tab")
    with ctx.pose({knob_pivot: math.radians(35.0)}):
        knob_turned = ctx.part_element_world_aabb(catch_knob, elem="catch_tab")
    knob_rest_center = _aabb_center(knob_rest)
    knob_turned_center = _aabb_center(knob_turned)
    ctx.check(
        "catch tab lifts when the knob turns",
        knob_rest_center is not None
        and knob_turned_center is not None
        and knob_turned_center[2] > knob_rest_center[2] + 0.002,
        details=f"rest_center={knob_rest_center}, turned_center={knob_turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
