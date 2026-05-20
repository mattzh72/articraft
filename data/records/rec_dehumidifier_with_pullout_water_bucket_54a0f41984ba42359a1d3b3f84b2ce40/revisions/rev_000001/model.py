from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.54
BODY_D = 0.29
BODY_H = 0.55
BODY_BASE_Z = 0.05
BODY_TOP_Z = BODY_BASE_Z + BODY_H
BODY_CENTER_Z = BODY_BASE_Z + BODY_H / 2.0
BODY_WALL = 0.005
BODY_TOP_THICK = 0.018
BODY_BOTTOM_THICK = 0.010

BUCKET_W = 0.44
BUCKET_D = 0.215
BUCKET_H = 0.23
BUCKET_WALL = 0.004
BUCKET_BOTTOM_Z = 0.085
BUCKET_CENTER_Z = BUCKET_BOTTOM_Z + BUCKET_H / 2.0
BUCKET_TRAVEL = 0.11

FILTER_DOOR_T = 0.006
FILTER_DOOR_D = 0.172
FILTER_DOOR_H = 0.272
FILTER_DOOR_CENTER_Y = 0.028
FILTER_DOOR_CENTER_Z = 0.355
FILTER_DOOR_OPEN = math.radians(72.0)

CONTROL_PANEL_W = 0.37
CONTROL_PANEL_D = 0.115
CONTROL_PANEL_T = 0.006
CONTROL_PANEL_X = 0.020
CONTROL_PANEL_Y = -0.006

MODE_KNOB_X = -0.095
MODE_KNOB_Y = -0.010

BUTTON_SIZE = (0.028, 0.018, 0.006)
BUTTON_TRAVEL = 0.0025
BUTTON_Y = -0.008
BUTTON_XS = (0.000, 0.045, 0.090, 0.135)

WHEEL_RADIUS = 0.022
WHEEL_WIDTH = 0.018
WHEEL_HUB_RADIUS = 0.012
WHEEL_POSITIONS = (
    (-0.195, -0.090),
    (0.195, -0.090),
    (-0.195, 0.090),
    (0.195, 0.090),
)


def _build_body_shell() -> object:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    outer = outer.edges("|Z").fillet(0.028)

    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            BODY_D - 2.0 * BODY_WALL,
            BODY_H - BODY_TOP_THICK - BODY_BOTTOM_THICK,
        )
        .translate((0.0, 0.0, (BODY_BOTTOM_THICK - BODY_TOP_THICK) / 2.0))
    )

    shell = outer.cut(inner)

    bucket_open_w = BUCKET_W + 0.010
    bucket_open_h = BUCKET_H + 0.014
    bucket_open_center_z = BUCKET_CENTER_Z + 0.003 - BODY_CENTER_Z
    bucket_open = (
        cq.Workplane("XY")
        .box(bucket_open_w, 0.065, bucket_open_h)
        .translate((0.0, -BODY_D / 2.0 + 0.0325, bucket_open_center_z))
    )
    shell = shell.cut(bucket_open)

    filter_cut = (
        cq.Workplane("XY")
        .box(0.055, FILTER_DOOR_D - 0.018, FILTER_DOOR_H - 0.020)
        .translate(
            (
                BODY_W / 2.0 - 0.0275,
                FILTER_DOOR_CENTER_Y,
                FILTER_DOOR_CENTER_Z - BODY_CENTER_Z,
            )
        )
    )
    shell = shell.cut(filter_cut)

    return shell


def _build_bucket_shell() -> object:
    bucket = cq.Workplane("XY").box(BUCKET_W, BUCKET_D, BUCKET_H)
    bucket = bucket.edges("|Z").fillet(0.012)
    bucket = bucket.faces(">Z").shell(-BUCKET_WALL)

    handle_cut = (
        cq.Workplane("XY")
        .box(0.16, 0.018, 0.046)
        .translate((0.0, -BUCKET_D / 2.0 + 0.009, 0.020))
    )
    bucket = bucket.cut(handle_cut)
    bucket = bucket.translate((0.0, BUCKET_D / 2.0, 0.0))
    return bucket


def _add_caster_bracket(body_part, *, x: float, y: float, material) -> None:
    plate_x = 0.030
    plate_y = 0.026
    plate_t = 0.004
    arm_t = 0.003
    arm_gap = WHEEL_WIDTH / 2.0 + arm_t / 2.0 + 0.0012
    arm_drop = BODY_BASE_Z - WHEEL_RADIUS
    arm_center_z = WHEEL_RADIUS + arm_drop / 2.0

    body_part.visual(
        Box((plate_x, plate_y, plate_t)),
        origin=Origin(xyz=(x, y, BODY_BASE_Z - plate_t / 2.0)),
        material=material,
        name=f"caster_plate_{'f' if y < 0 else 'r'}_{'l' if x < 0 else 'r'}",
    )
    for side_index, x_offset in enumerate((-arm_gap, arm_gap)):
        body_part.visual(
            Box((arm_t, plate_y, arm_drop)),
            origin=Origin(xyz=(x + x_offset, y, arm_center_z)),
            material=material,
            name=(
                f"caster_arm_{'f' if y < 0 else 'r'}_"
                f"{'l' if x < 0 else 'r'}_{side_index}"
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wide_dehumidifier")

    body_finish = model.material("body_finish", rgba=(0.90, 0.91, 0.89, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.55, 0.58, 0.61, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.21, 0.23, 0.25, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    hub_finish = model.material("hub_finish", rgba=(0.63, 0.65, 0.68, 1.0))
    bucket_finish = model.material("bucket_finish", rgba=(0.82, 0.86, 0.90, 0.86))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "dehumidifier_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=body_finish,
        name="shell",
    )
    body.visual(
        Box((CONTROL_PANEL_W, CONTROL_PANEL_D, CONTROL_PANEL_T)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_X,
                CONTROL_PANEL_Y,
                BODY_TOP_Z - CONTROL_PANEL_T / 2.0,
            )
        ),
        material=panel_finish,
        name="control_panel",
    )
    body.visual(
        Box((0.46, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.004, 0.378)),
        material=trim_finish,
        name="front_lip",
    )
    for index, z_pos in enumerate((0.412, 0.438, 0.464, 0.490)):
        body.visual(
            Box((0.35, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -BODY_D / 2.0 + 0.002, z_pos)),
            material=trim_finish,
            name=f"front_vent_{index}",
        )
    for rail_index, rail_x in enumerate((-0.175, 0.175)):
        body.visual(
            Box((0.040, 0.130, 0.025)),
            origin=Origin(xyz=(rail_x, 0.025, 0.0725)),
            material=trim_finish,
            name=f"bucket_rail_{rail_index}",
        )

    for wheel_x, wheel_y in WHEEL_POSITIONS:
        _add_caster_bracket(body, x=wheel_x, y=wheel_y, material=dark_trim)

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_build_bucket_shell(), "dehumidifier_bucket"),
        material=bucket_finish,
        name="bucket_shell",
    )
    bucket.inertial = Inertial.from_geometry(
        Box((BUCKET_W, BUCKET_D, BUCKET_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, BUCKET_D / 2.0, 0.0)),
    )
    model.articulation(
        "bucket_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=bucket,
        origin=Origin(xyz=(0.0, -BODY_D / 2.0, BUCKET_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.22,
            lower=0.0,
            upper=BUCKET_TRAVEL,
        ),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((FILTER_DOOR_T, FILTER_DOOR_D, FILTER_DOOR_H)),
        origin=Origin(xyz=(FILTER_DOOR_T / 2.0, FILTER_DOOR_D / 2.0, 0.0)),
        material=body_finish,
        name="door_panel",
    )
    for index, z_offset in enumerate((-0.078, -0.026, 0.026, 0.078)):
        filter_door.visual(
            Box((0.003, FILTER_DOOR_D - 0.028, 0.010)),
            origin=Origin(
                xyz=(
                    FILTER_DOOR_T + 0.0015,
                    FILTER_DOOR_D / 2.0,
                    z_offset,
                )
            ),
            material=trim_finish,
            name=f"door_louver_{index}",
        )
    filter_door.visual(
        Box((0.010, 0.030, 0.055)),
        origin=Origin(
            xyz=(
                FILTER_DOOR_T + 0.005,
                FILTER_DOOR_D * 0.77,
                0.0,
            )
        ),
        material=dark_trim,
        name="door_pull",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.016, FILTER_DOOR_D, FILTER_DOOR_H)),
        mass=0.45,
        origin=Origin(xyz=(0.008, FILTER_DOOR_D / 2.0, 0.0)),
    )
    model.articulation(
        "filter_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(
            xyz=(
                BODY_W / 2.0,
                FILTER_DOOR_CENTER_Y - FILTER_DOOR_D / 2.0,
                FILTER_DOOR_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=FILTER_DOOR_OPEN,
        ),
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.022,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.046, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=18, depth=0.0008),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0006,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "dehumidifier_mode_knob",
        ),
        material=dark_trim,
        name="knob_shell",
    )
    mode_knob.inertial = Inertial.from_geometry(
        Box((0.046, 0.046, 0.022)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    model.articulation(
        "mode_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_knob,
        origin=Origin(xyz=(MODE_KNOB_X, MODE_KNOB_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"preset_button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] / 2.0)),
            material=panel_finish,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box(BUTTON_SIZE),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] / 2.0)),
        )
        model.articulation(
            f"preset_button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_Y, BODY_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    for index, (wheel_x, wheel_y) in enumerate(WHEEL_POSITIONS):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_finish,
            name="tire",
        )
        caster.visual(
            Cylinder(radius=WHEEL_HUB_RADIUS, length=WHEEL_WIDTH + 0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hub_finish,
            name="hub",
        )
        caster.inertial = Inertial.from_geometry(
            Box((WHEEL_WIDTH, WHEEL_RADIUS * 2.0, WHEEL_RADIUS * 2.0)),
            mass=0.10,
        )
        model.articulation(
            f"caster_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=caster,
            origin=Origin(xyz=(wheel_x, wheel_y, WHEEL_RADIUS)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.15, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bucket = object_model.get_part("bucket")
    filter_door = object_model.get_part("filter_door")
    mode_knob = object_model.get_part("mode_knob")
    button_0 = object_model.get_part("preset_button_0")

    bucket_slide = object_model.get_articulation("bucket_slide")
    filter_door_hinge = object_model.get_articulation("filter_door_hinge")
    button_press = object_model.get_articulation("preset_button_0_press")

    ctx.check(
        "four preset buttons authored",
        len([part for part in object_model.parts if part.name.startswith("preset_button_")]) == 4,
        details=f"parts={[part.name for part in object_model.parts if part.name.startswith('preset_button_')]}",
    )
    ctx.check(
        "four caster wheels authored",
        len([part for part in object_model.parts if part.name.startswith("caster_")]) == 4,
        details=f"parts={[part.name for part in object_model.parts if part.name.startswith('caster_')]}",
    )

    ctx.expect_gap(
        mode_knob,
        body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="mode knob seats on the top panel",
    )
    ctx.expect_gap(
        button_0,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="preset button sits proud of the top panel",
    )

    with ctx.pose({filter_door_hinge: 0.0}):
        ctx.expect_gap(
            filter_door,
            body,
            axis="x",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed filter door sits against the side wall",
        )
        ctx.expect_overlap(
            filter_door,
            body,
            axes="yz",
            min_overlap=0.14,
            name="closed filter door covers the side intake opening",
        )

    with ctx.pose({bucket_slide: 0.0}):
        ctx.expect_within(
            bucket,
            body,
            axes="x",
            margin=0.03,
            name="bucket stays laterally centered in the cabinet opening",
        )
        bucket_rest = ctx.part_world_position(bucket)

    with ctx.pose({bucket_slide: BUCKET_TRAVEL}):
        ctx.expect_within(
            bucket,
            body,
            axes="x",
            margin=0.03,
            name="extended bucket remains laterally aligned",
        )
        ctx.expect_overlap(
            bucket,
            body,
            axes="y",
            min_overlap=0.10,
            name="extended bucket keeps retained insertion in the body",
        )
        bucket_extended = ctx.part_world_position(bucket)

    door_closed_aabb = ctx.part_world_aabb(filter_door)
    with ctx.pose({filter_door_hinge: FILTER_DOOR_OPEN}):
        door_open_aabb = ctx.part_world_aabb(filter_door)

    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_press: BUTTON_TRAVEL}):
        button_pressed = ctx.part_world_position(button_0)

    ctx.check(
        "bucket pulls forward",
        bucket_rest is not None
        and bucket_extended is not None
        and bucket_extended[1] < bucket_rest[1] - 0.08,
        details=f"rest={bucket_rest}, extended={bucket_extended}",
    )
    ctx.check(
        "filter door swings outward",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.04,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )
    ctx.check(
        "preset button presses downward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.0015,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
