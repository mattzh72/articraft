from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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


BODY_WIDTH = 0.34
BODY_DEPTH = 0.31
BODY_HEIGHT = 0.648
BODY_BASE_Z = 0.072
BODY_TOP_Z = BODY_BASE_Z + BODY_HEIGHT

OUTLET_WIDTH = 0.238
OUTLET_HEIGHT = 0.082
OUTLET_DEPTH = 0.055
OUTLET_CENTER_Z = 0.535

CONTROL_WIDTH = 0.236
CONTROL_DEPTH = 0.094
CONTROL_RECESS_DEPTH = 0.004
CONTROL_Y = -0.040
CONTROL_PLATE_THICKNESS = 0.002

DIAL_X = -0.060
BUTTON_XS = (0.036, 0.076)
BUTTON_HOLE_SIZE = (0.024, 0.014)

CASTER_TOP_Z = BODY_BASE_Z
CASTER_PAIR_Y = (-0.092, 0.092)
CASTER_WHEEL_X = (-0.022, 0.022)
CASTER_WHEEL_CENTER_Z = -0.050
CASTER_WHEEL_RADIUS = 0.022


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate((0.0, 0.0, BODY_HEIGHT * 0.5))
    shell = shell.edges("|Z").fillet(0.022)
    shell = shell.edges(">Z").fillet(0.012)

    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_WIDTH, OUTLET_DEPTH, OUTLET_HEIGHT)
        .translate((0.0, -BODY_DEPTH * 0.5 + OUTLET_DEPTH * 0.5 - 0.001, OUTLET_CENTER_Z))
    )
    control_recess = (
        cq.Workplane("XY")
        .box(CONTROL_WIDTH, CONTROL_DEPTH, CONTROL_RECESS_DEPTH + 0.001)
        .translate((0.0, CONTROL_Y, BODY_HEIGHT - CONTROL_RECESS_DEPTH * 0.5 + 0.0005))
    )
    return shell.cut(outlet_cut).cut(control_recess)


def _build_control_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(CONTROL_WIDTH, CONTROL_DEPTH, CONTROL_PLATE_THICKNESS)
    for button_x in BUTTON_XS:
        plate = plate.cut(
            cq.Workplane("XY")
            .box(BUTTON_HOLE_SIZE[0], BUTTON_HOLE_SIZE[1], CONTROL_PLATE_THICKNESS + 0.002)
            .translate((button_x, 0.0, 0.0))
        )
    return plate.edges("|Z").fillet(0.006)


def _add_caster_pair(
    model: ArticulatedObject,
    body,
    *,
    pair_index: int,
    y_position: float,
    support_material,
    wheel_material,
    hub_material,
) -> None:
    pair = model.part(f"caster_pair_{pair_index}")
    pair.visual(
        Box((0.060, 0.042, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=support_material,
        name="mount_plate",
    )
    pair.visual(
        Box((0.020, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=support_material,
        name="stem",
    )
    pair.visual(
        Box((0.072, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=support_material,
        name="bridge",
    )
    for cheek_index, wheel_x in enumerate(CASTER_WHEEL_X):
        pair.visual(
            Box((0.012, 0.018, 0.008)),
            origin=Origin(xyz=(wheel_x, 0.0, -0.016)),
            material=support_material,
            name=f"fork_{cheek_index}",
        )

    model.articulation(
        f"body_to_caster_pair_{pair_index}",
        ArticulationType.FIXED,
        parent=body,
        child=pair,
        origin=Origin(xyz=(0.0, y_position, CASTER_TOP_Z)),
    )

    for wheel_index, wheel_x in enumerate(CASTER_WHEEL_X):
        wheel = model.part(f"caster_{pair_index}_{wheel_index}")
        wheel.visual(
            Cylinder(radius=CASTER_WHEEL_RADIUS, length=0.012),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=wheel_material,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=hub_material,
            name="hub",
        )

        model.articulation(
            f"caster_pair_{pair_index}_to_caster_{pair_index}_{wheel_index}",
            ArticulationType.CONTINUOUS,
            parent=pair,
            child=wheel,
            origin=Origin(xyz=(wheel_x, 0.0, CASTER_WHEEL_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=12.0),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_air_conditioner")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    louver_dark = model.material("louver_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    support_grey = model.material("support_grey", rgba=(0.67, 0.69, 0.72, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.79, 0.81, 0.84, 1.0))
    button_light = model.material("button_light", rgba=(0.87, 0.89, 0.91, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "portable_ac_body"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z)),
        material=shell_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_build_control_plate(), "portable_ac_control_plate"),
        origin=Origin(xyz=(0.0, CONTROL_Y, BODY_TOP_Z - CONTROL_PLATE_THICKNESS * 0.5)),
        material=panel_dark,
        name="control_strip",
    )
    body.visual(
        Box((0.170, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -BODY_DEPTH * 0.5 + 0.006, BODY_BASE_Z + 0.170)),
        material=shell_white,
        name="lower_trim",
    )

    louver = model.part("louver")
    louver_width = OUTLET_WIDTH - 0.010
    louver_height = OUTLET_HEIGHT - 0.008
    rail_x = louver_width * 0.5 - 0.004
    louver.visual(
        Cylinder(radius=0.004, length=louver_width - 0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=louver_dark,
        name="hinge_barrel",
    )
    louver.visual(
        Box((louver_width - 0.006, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.006, -0.006)),
        material=louver_dark,
        name="top_rail",
    )
    louver.visual(
        Box((0.008, 0.010, louver_height)),
        origin=Origin(xyz=(-rail_x, -0.007, -louver_height * 0.5)),
        material=louver_dark,
        name="side_rail_0",
    )
    louver.visual(
        Box((0.008, 0.010, louver_height)),
        origin=Origin(xyz=(rail_x, -0.007, -louver_height * 0.5)),
        material=louver_dark,
        name="side_rail_1",
    )
    louver.visual(
        Box((louver_width - 0.008, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.007, -louver_height + 0.004)),
        material=louver_dark,
        name="bottom_rail",
    )
    for slat_index, slat_z in enumerate((-0.018, -0.032, -0.046, -0.060)):
        louver.visual(
            Box((louver_width - 0.012, 0.004, 0.009)),
            origin=Origin(xyz=(0.0, -0.007, slat_z), rpy=(math.radians(24.0), 0.0, 0.0)),
            material=louver_dark,
            name=f"slat_{slat_index}",
        )

    model.articulation(
        "body_to_louver",
        ArticulationType.REVOLUTE,
        parent=body,
        child=louver,
        origin=Origin(
            xyz=(
                0.0,
                -BODY_DEPTH * 0.5,
                BODY_BASE_Z + OUTLET_CENTER_Z + OUTLET_HEIGHT * 0.5,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.050, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006, angle_deg=0.0),
                center=False,
            ),
            "portable_ac_dial",
        ),
        material=panel_dark,
        name="dial_knob",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, CONTROL_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    for button_index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{button_index}")
        button.visual(
            Box((0.024, 0.014, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=button_light,
            name="cap",
        )
        button.visual(
            Box((0.018, 0.010, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0076)),
            material=panel_dark,
            name="label_band",
        )
        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_Y, BODY_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.04,
                lower=0.0,
                upper=0.002,
            ),
        )

    for pair_index, y_position in enumerate(CASTER_PAIR_Y):
        _add_caster_pair(
            model,
            body,
            pair_index=pair_index,
            y_position=y_position,
            support_material=support_grey,
            wheel_material=wheel_dark,
            hub_material=hub_grey,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    louver = object_model.get_part("louver")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    front_pair = object_model.get_part("caster_pair_0")
    rear_pair = object_model.get_part("caster_pair_1")

    louver_joint = object_model.get_articulation("body_to_louver")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    ctx.expect_gap(
        body,
        louver,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="louver closes at the outlet face",
    )
    ctx.expect_overlap(
        louver,
        body,
        axes="x",
        min_overlap=0.20,
        name="louver spans the outlet width",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial sits on the control strip",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        min_overlap=0.02,
        name="dial remains within the control strip footprint",
    )
    ctx.expect_origin_gap(
        rear_pair,
        front_pair,
        axis="y",
        min_gap=0.16,
        name="front and rear caster pairs are separated along the base",
    )

    closed_bottom = ctx.part_element_world_aabb(louver, elem="bottom_rail")
    louver_limits = louver_joint.motion_limits
    louver_upper = louver_limits.upper if louver_limits is not None and louver_limits.upper is not None else 0.0
    with ctx.pose({louver_joint: louver_upper}):
        opened_bottom = ctx.part_element_world_aabb(louver, elem="bottom_rail")
    louver_opens = (
        closed_bottom is not None
        and opened_bottom is not None
        and opened_bottom[0][1] < closed_bottom[0][1] - 0.025
        and opened_bottom[1][2] > closed_bottom[1][2] + 0.010
    )
    ctx.check(
        "louver opens upward and outward",
        louver_opens,
        details=f"closed={closed_bottom!r}, opened={opened_bottom!r}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_upper = (
        button_0_joint.motion_limits.upper
        if button_0_joint.motion_limits is not None and button_0_joint.motion_limits.upper is not None
        else 0.0
    )
    button_1_upper = (
        button_1_joint.motion_limits.upper
        if button_1_joint.motion_limits is not None and button_1_joint.motion_limits.upper is not None
        else 0.0
    )

    with ctx.pose({button_0_joint: button_0_upper}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_idle = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_idle is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0015
        and abs(button_1_idle[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest!r}, button_0_pressed={button_0_pressed!r}, "
            f"button_1_rest={button_1_rest!r}, button_1_idle={button_1_idle!r}"
        ),
    )

    with ctx.pose({button_1_joint: button_1_upper}):
        button_1_pressed = ctx.part_world_position(button_1)
        button_0_idle = ctx.part_world_position(button_0)
    ctx.check(
        "button_1 depresses independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_1_pressed is not None
        and button_0_idle is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0015
        and abs(button_0_idle[2] - button_0_rest[2]) < 1e-6,
        details=(
            f"button_1_rest={button_1_rest!r}, button_1_pressed={button_1_pressed!r}, "
            f"button_0_rest={button_0_rest!r}, button_0_idle={button_0_idle!r}"
        ),
    )

    for pair_index in range(2):
        for wheel_index in range(2):
            wheel = object_model.get_part(f"caster_{pair_index}_{wheel_index}")
            aabb = ctx.part_world_aabb(wheel)
            touches_floor = aabb is not None and abs(aabb[0][2]) <= 0.0025
            ctx.check(
                f"caster_{pair_index}_{wheel_index} touches floor",
                touches_floor,
                details=f"aabb={aabb!r}",
            )

    return ctx.report()


object_model = build_object_model()
