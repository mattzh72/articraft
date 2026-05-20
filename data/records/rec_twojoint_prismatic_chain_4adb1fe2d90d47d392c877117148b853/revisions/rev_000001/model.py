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


FRAME_LEN = 0.92
FRAME_W = 0.34
FRAME_BLOCK_H = 0.072
BASE_T = 0.018
POCKET_LEN = 0.72
POCKET_W = 0.22
LOWER_RAIL_LEN = 0.84
LOWER_RAIL_W = 0.022
LOWER_RAIL_H = 0.010
LOWER_RAIL_Y = 0.14
LOWER_RAIL_TOP_Z = FRAME_BLOCK_H + LOWER_RAIL_H

FIRST_STAGE_LEN = 0.46
FIRST_STAGE_W = 0.27
FIRST_STAGE_DECK_T = 0.014
FIRST_STAGE_TRUCK_L = 0.10
FIRST_STAGE_TRUCK_W = 0.046
FIRST_STAGE_TRUCK_H = 0.028
FIRST_STAGE_TRUCK_X = 0.12
FIRST_STAGE_HEIGHT = FIRST_STAGE_TRUCK_H + FIRST_STAGE_DECK_T + 0.022

UPPER_RAIL_LEN = 0.38
UPPER_RAIL_BASE_W = 0.032
UPPER_RAIL_BASE_H = 0.014
UPPER_RAIL_CAP_W = 0.020
UPPER_RAIL_CAP_H = 0.008
UPPER_RAIL_Y = 0.055
UPPER_RAIL_TOP_Z = (
    FIRST_STAGE_TRUCK_H
    + FIRST_STAGE_DECK_T
    + UPPER_RAIL_BASE_H
    + UPPER_RAIL_CAP_H
)

OUTPUT_STAGE_LEN = 0.25
OUTPUT_STAGE_W = 0.16
OUTPUT_STAGE_PLATE_T = 0.014
OUTPUT_STAGE_TRUCK_L = 0.07
OUTPUT_STAGE_TRUCK_W = 0.032
OUTPUT_STAGE_TRUCK_H = 0.022
OUTPUT_STAGE_TRUCK_X = 0.07
OUTPUT_STAGE_HEIGHT = OUTPUT_STAGE_TRUCK_H + OUTPUT_STAGE_PLATE_T

FIRST_STAGE_RETRACT_X = -0.15
FIRST_STAGE_TRAVEL = 0.30
SECOND_STAGE_RETRACT_X = -0.07
SECOND_STAGE_TRAVEL = 0.14


def make_box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def make_guide_frame_body():
    body = make_box(FRAME_LEN, FRAME_W, FRAME_BLOCK_H)
    body = body.faces(">Z").workplane().rect(POCKET_LEN, POCKET_W).cutBlind(-(FRAME_BLOCK_H - BASE_T))
    return body


def make_lower_rails():
    rails = None
    for y in (-LOWER_RAIL_Y, LOWER_RAIL_Y):
        rail = make_box(
            LOWER_RAIL_LEN,
            LOWER_RAIL_W,
            LOWER_RAIL_H,
            y=y,
            z=FRAME_BLOCK_H,
        )
        rails = rail if rails is None else rails.union(rail)
    return rails


def make_first_stage_trucks():
    trucks = None
    for x in (-FIRST_STAGE_TRUCK_X, FIRST_STAGE_TRUCK_X):
        for y in (-LOWER_RAIL_Y, LOWER_RAIL_Y):
            truck = make_box(
                FIRST_STAGE_TRUCK_L,
                FIRST_STAGE_TRUCK_W,
                FIRST_STAGE_TRUCK_H,
                x=x,
                y=y,
                z=0.0,
            )
            trucks = truck if trucks is None else trucks.union(truck)
    return trucks


def make_first_stage_body():
    deck = make_box(
        FIRST_STAGE_LEN,
        FIRST_STAGE_W,
        FIRST_STAGE_DECK_T,
        z=FIRST_STAGE_TRUCK_H,
    )
    return deck


def make_upper_rails():
    rails = None
    rail_base_z = FIRST_STAGE_TRUCK_H + FIRST_STAGE_DECK_T
    for y in (-UPPER_RAIL_Y, UPPER_RAIL_Y):
        base = make_box(
            UPPER_RAIL_LEN,
            UPPER_RAIL_BASE_W,
            UPPER_RAIL_BASE_H,
            y=y,
            z=rail_base_z,
        )
        cap = make_box(
            UPPER_RAIL_LEN,
            UPPER_RAIL_CAP_W,
            UPPER_RAIL_CAP_H,
            y=y,
            z=rail_base_z + UPPER_RAIL_BASE_H,
        )
        rail = base.union(cap)
        rails = rail if rails is None else rails.union(rail)
    return rails


def make_output_stage_trucks():
    trucks = None
    for x in (-OUTPUT_STAGE_TRUCK_X, OUTPUT_STAGE_TRUCK_X):
        for y in (-UPPER_RAIL_Y, UPPER_RAIL_Y):
            truck = make_box(
                OUTPUT_STAGE_TRUCK_L,
                OUTPUT_STAGE_TRUCK_W,
                OUTPUT_STAGE_TRUCK_H,
                x=x,
                y=y,
                z=0.0,
            )
            trucks = truck if trucks is None else trucks.union(truck)
    return trucks


def make_output_stage_body():
    plate = make_box(
        OUTPUT_STAGE_LEN,
        OUTPUT_STAGE_W,
        OUTPUT_STAGE_PLATE_T,
        z=OUTPUT_STAGE_TRUCK_H,
    )
    nose = make_box(
        0.08,
        0.08,
        0.012,
        x=(OUTPUT_STAGE_LEN * 0.5) - 0.04,
        z=OUTPUT_STAGE_TRUCK_H + OUTPUT_STAGE_PLATE_T,
    )
    return plate.union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_transfer_chain")

    frame_color = model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    first_stage_color = model.material("first_stage", rgba=(0.74, 0.77, 0.80, 1.0))
    output_stage_color = model.material("output_stage", rgba=(0.54, 0.58, 0.63, 1.0))

    guide_frame = model.part("guide_frame")
    guide_frame.visual(
        mesh_from_cadquery(make_guide_frame_body(), "guide_frame_body"),
        material=frame_color,
        name="frame_body",
    )
    guide_frame.visual(
        mesh_from_cadquery(make_lower_rails(), "guide_frame_lower_rails"),
        material=frame_color,
        name="lower_rails",
    )
    guide_frame.inertial = Inertial.from_geometry(
        Box((FRAME_LEN, FRAME_W, LOWER_RAIL_TOP_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_RAIL_TOP_Z * 0.5)),
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        mesh_from_cadquery(make_first_stage_trucks(), "first_carriage_trucks"),
        material=first_stage_color,
        name="lower_trucks",
    )
    first_carriage.visual(
        mesh_from_cadquery(make_first_stage_body(), "first_carriage_body"),
        material=first_stage_color,
        name="stage_body",
    )
    first_carriage.visual(
        mesh_from_cadquery(make_upper_rails(), "first_carriage_upper_rails"),
        material=first_stage_color,
        name="upper_rails",
    )
    first_carriage.inertial = Inertial.from_geometry(
        Box((FIRST_STAGE_LEN, FIRST_STAGE_W, FIRST_STAGE_HEIGHT)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, FIRST_STAGE_HEIGHT * 0.5)),
    )

    output_carriage = model.part("output_carriage")
    output_carriage.visual(
        mesh_from_cadquery(make_output_stage_trucks(), "output_carriage_trucks"),
        material=output_stage_color,
        name="output_trucks",
    )
    output_carriage.visual(
        mesh_from_cadquery(make_output_stage_body(), "output_carriage_body"),
        material=output_stage_color,
        name="output_body",
    )
    output_carriage.inertial = Inertial.from_geometry(
        Box((OUTPUT_STAGE_LEN, OUTPUT_STAGE_W, OUTPUT_STAGE_HEIGHT)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, OUTPUT_STAGE_HEIGHT * 0.5)),
    )

    model.articulation(
        "guide_to_first_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_frame,
        child=first_carriage,
        origin=Origin(xyz=(FIRST_STAGE_RETRACT_X, 0.0, LOWER_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=FIRST_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "first_to_output_carriage",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=output_carriage,
        origin=Origin(xyz=(SECOND_STAGE_RETRACT_X, 0.0, UPPER_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=SECOND_STAGE_TRAVEL,
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

    guide_frame = object_model.get_part("guide_frame")
    first_carriage = object_model.get_part("first_carriage")
    output_carriage = object_model.get_part("output_carriage")
    stage_1 = object_model.get_articulation("guide_to_first_carriage")
    stage_2 = object_model.get_articulation("first_to_output_carriage")

    lower_rails = guide_frame.get_visual("lower_rails")
    lower_trucks = first_carriage.get_visual("lower_trucks")
    upper_rails = first_carriage.get_visual("upper_rails")
    output_trucks = output_carriage.get_visual("output_trucks")

    ctx.check(
        "serial prismatic stages share the common axis",
        tuple(stage_1.axis) == (1.0, 0.0, 0.0) and tuple(stage_2.axis) == (1.0, 0.0, 0.0),
        details=f"stage_1_axis={stage_1.axis}, stage_2_axis={stage_2.axis}",
    )

    ctx.expect_contact(
        first_carriage,
        guide_frame,
        elem_a=lower_trucks,
        elem_b=lower_rails,
        contact_tol=0.0005,
        name="first carriage trucks ride on the fixed frame rails",
    )
    ctx.expect_contact(
        output_carriage,
        first_carriage,
        elem_a=output_trucks,
        elem_b=upper_rails,
        contact_tol=0.0005,
        name="output carriage trucks ride on the first carriage rails",
    )
    ctx.expect_within(
        first_carriage,
        guide_frame,
        axes="y",
        margin=0.0,
        name="first carriage stays within frame width",
    )
    ctx.expect_within(
        output_carriage,
        first_carriage,
        axes="y",
        margin=0.0,
        name="output carriage stays within first carriage width",
    )

    first_rest = ctx.part_world_position(first_carriage)
    output_rest = ctx.part_world_position(output_carriage)
    with ctx.pose({stage_1: FIRST_STAGE_TRAVEL}):
        ctx.expect_overlap(
            first_carriage,
            guide_frame,
            axes="x",
            elem_a=lower_trucks,
            elem_b=lower_rails,
            min_overlap=0.30,
            name="first carriage retains rail engagement at full travel",
        )
        first_extended = ctx.part_world_position(first_carriage)

    with ctx.pose({stage_2: SECOND_STAGE_TRAVEL}):
        ctx.expect_overlap(
            output_carriage,
            first_carriage,
            axes="x",
            elem_a=output_trucks,
            elem_b=upper_rails,
            min_overlap=0.20,
            name="output carriage retains rail engagement at full travel",
        )
        output_extended = ctx.part_world_position(output_carriage)

    ctx.check(
        "first carriage extends forward along +x",
        first_rest is not None
        and first_extended is not None
        and first_extended[0] > first_rest[0] + 0.25,
        details=f"rest={first_rest}, extended={first_extended}",
    )
    ctx.check(
        "output carriage extends forward along +x",
        output_rest is not None
        and output_extended is not None
        and output_extended[0] > output_rest[0] + 0.10,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
