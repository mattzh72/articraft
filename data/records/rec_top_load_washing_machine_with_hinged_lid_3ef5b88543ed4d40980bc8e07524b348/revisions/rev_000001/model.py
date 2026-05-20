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


BODY_DEPTH = 0.72
BODY_WIDTH = 0.69
BODY_HEIGHT = 0.92
DECK_Z = BODY_HEIGHT

OPENING_X = -0.075
OPENING_DEPTH = 0.41
OPENING_WIDTH = 0.52

LID_DEPTH = 0.535
LID_WIDTH = 0.642
LID_THICKNESS = 0.037
HINGE_X = 0.185

TUB_OUTER_RADIUS = 0.245
TUB_INNER_RADIUS = 0.231
TUB_HEIGHT = 0.49
TUB_TOP_Z = 0.785

CONSOLE_FACE_X = 0.228
CONSOLE_FACE_Z = 0.990
CONSOLE_FACE_THICKNESS = 0.018
CONSOLE_PITCH = math.radians(20.0)


def _rounded_box(dx: float, dy: float, dz: float, radius: float) -> cq.Workplane:
    solid = cq.Workplane("XY").box(dx, dy, dz).translate((0.0, 0.0, dz * 0.5))
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _build_cabinet_shape() -> cq.Workplane:
    wall = 0.018

    cabinet = _rounded_box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, 0.030)
    cabinet = cabinet.faces("<Z").shell(-wall)

    opening = _rounded_box(OPENING_DEPTH, OPENING_WIDTH, 0.090, 0.085).translate(
        (OPENING_X, 0.0, DECK_Z - 0.045)
    )
    cabinet = cabinet.cut(opening)

    well_outer = _rounded_box(OPENING_DEPTH + 0.080, OPENING_WIDTH + 0.080, 0.125, 0.100).translate(
        (OPENING_X, 0.0, DECK_Z - 0.125)
    )
    well_inner = _rounded_box(OPENING_DEPTH + 0.030, OPENING_WIDTH + 0.030, 0.130, 0.090).translate(
        (OPENING_X, 0.0, DECK_Z - 0.127)
    )
    cabinet = cabinet.union(well_outer.cut(well_inner))

    console = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.130, DECK_Z),
                (0.352, DECK_Z),
                (0.352, 1.085),
                (0.245, 1.085),
            ]
        )
        .close()
        .extrude(0.300, both=True)
    )
    cabinet = cabinet.union(console)

    front_hem = _rounded_box(0.64, 0.05, 0.022, 0.008).translate((-0.030, 0.0, DECK_Z - 0.022))
    cabinet = cabinet.union(front_hem)

    return cabinet


def _build_lid_shape() -> cq.Workplane:
    outer = _rounded_box(LID_DEPTH, LID_WIDTH, LID_THICKNESS, 0.050).translate((-LID_DEPTH * 0.5, 0.0, 0.0))
    window = _rounded_box(0.395, 0.490, 0.050, 0.110).translate((-0.255, 0.0, -0.006))
    frame = outer.cut(window)

    handle = _rounded_box(0.050, 0.220, 0.020, 0.008).translate((-LID_DEPTH + 0.050, 0.0, 0.012))
    front_bridge = _rounded_box(0.040, 0.460, 0.014, 0.006).translate((-LID_DEPTH + 0.024, 0.0, 0.008))
    skirt_outer = _rounded_box(0.490, 0.590, 0.020, 0.045).translate((-0.255, 0.0, -0.018))
    skirt_inner = _rounded_box(0.430, 0.530, 0.024, 0.085).translate((-0.255, 0.0, -0.020))
    glass_seat_outer = _rounded_box(0.407, 0.504, 0.006, 0.105).translate((-0.255, 0.0, 0.0))
    glass_seat_inner = _rounded_box(0.350, 0.447, 0.010, 0.082).translate((-0.255, 0.0, -0.002))

    return (
        frame.union(handle)
        .union(front_bridge)
        .union(skirt_outer.cut(skirt_inner))
        .union(glass_seat_outer.cut(glass_seat_inner))
    )


def _build_glass_shape() -> cq.Workplane:
    return _rounded_box(0.366, 0.463, 0.006, 0.090).translate((-0.255, 0.0, 0.006))


def _build_tub_shape() -> cq.Workplane:
    wall = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_RADIUS)
        .circle(TUB_INNER_RADIUS)
        .extrude(TUB_HEIGHT)
        .translate((0.0, 0.0, -TUB_HEIGHT))
    )
    bottom = cq.Workplane("XY").circle(TUB_INNER_RADIUS).extrude(0.010).translate((0.0, 0.0, -TUB_HEIGHT))
    rim = (
        cq.Workplane("XY")
        .circle(TUB_OUTER_RADIUS + 0.012)
        .circle(TUB_OUTER_RADIUS - 0.004)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.018))
    )
    wash_plate = cq.Workplane("XY").circle(0.115).extrude(0.018).translate((0.0, 0.0, -TUB_HEIGHT + 0.010))
    fin_a = _rounded_box(0.140, 0.024, 0.020, 0.006).translate((0.0, 0.0, -TUB_HEIGHT + 0.020))
    fin_b = _rounded_box(0.024, 0.140, 0.020, 0.006).translate((0.0, 0.0, -TUB_HEIGHT + 0.020))
    return wall.union(bottom).union(rim).union(wash_plate).union(fin_a).union(fin_b)


def _console_mount_origin(local_y: float, local_z: float, *, proud: float = 0.0) -> Origin:
    local_x = -CONSOLE_FACE_THICKNESS * 0.5 - proud
    cos_pitch = math.cos(CONSOLE_PITCH)
    sin_pitch = math.sin(CONSOLE_PITCH)
    world_x = CONSOLE_FACE_X + local_x * cos_pitch + local_z * sin_pitch
    world_z = CONSOLE_FACE_Z - local_x * sin_pitch + local_z * cos_pitch
    return Origin(
        xyz=(world_x, local_y, world_z),
        rpy=(0.0, CONSOLE_PITCH, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.75, 0.78, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.20, 0.33, 0.38, 0.34))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.15, 1.0))
    button_silver = model.material("button_silver", rgba=(0.88, 0.90, 0.92, 1.0))

    cabinet = model.part("cabinet")
    side_thickness = 0.026
    front_thickness = 0.028
    deck_thickness = 0.030
    opening_front = OPENING_X - OPENING_DEPTH * 0.5
    opening_rear = OPENING_X + OPENING_DEPTH * 0.5
    opening_half_width = OPENING_WIDTH * 0.5
    well_depth = 0.130
    well_thickness = 0.018

    cabinet.visual(
        Box((BODY_DEPTH, side_thickness, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -(BODY_WIDTH - side_thickness) * 0.5, BODY_HEIGHT * 0.5)),
        material=cabinet_white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((BODY_DEPTH, side_thickness, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (BODY_WIDTH - side_thickness) * 0.5, BODY_HEIGHT * 0.5)),
        material=cabinet_white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((front_thickness, BODY_WIDTH - 0.020, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_DEPTH - front_thickness) * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=cabinet_white,
        name="front_panel",
    )
    cabinet.visual(
        Box((0.030, BODY_WIDTH - 0.020, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_DEPTH - 0.030) * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((opening_front + BODY_DEPTH * 0.5 + 0.008, BODY_WIDTH - 0.014, deck_thickness)),
        origin=Origin(
            xyz=(
                (-BODY_DEPTH * 0.5 + opening_front) * 0.5 - 0.004,
                0.0,
                DECK_Z - deck_thickness * 0.5,
            )
        ),
        material=cabinet_white,
        name="deck_front",
    )
    cabinet.visual(
        Box((0.080, BODY_WIDTH - 0.100, deck_thickness)),
        origin=Origin(xyz=(opening_rear + 0.040, 0.0, DECK_Z - deck_thickness * 0.5)),
        material=cabinet_white,
        name="deck_rear",
    )
    cabinet.visual(
        Box((OPENING_DEPTH + 0.044, 0.092, deck_thickness)),
        origin=Origin(
            xyz=(
                OPENING_X + 0.002,
                -(opening_half_width + 0.046),
                DECK_Z - deck_thickness * 0.5,
            )
        ),
        material=cabinet_white,
        name="deck_side_0",
    )
    cabinet.visual(
        Box((OPENING_DEPTH + 0.044, 0.092, deck_thickness)),
        origin=Origin(
            xyz=(
                OPENING_X + 0.002,
                opening_half_width + 0.046,
                DECK_Z - deck_thickness * 0.5,
            )
        ),
        material=cabinet_white,
        name="deck_side_1",
    )
    cabinet.visual(
        Box((well_thickness, OPENING_WIDTH + 0.030, well_depth)),
        origin=Origin(
            xyz=(
                opening_front - well_thickness * 0.5,
                0.0,
                DECK_Z - well_depth * 0.5,
            )
        ),
        material=trim_gray,
        name="well_front",
    )
    cabinet.visual(
        Box((well_thickness, OPENING_WIDTH + 0.030, well_depth)),
        origin=Origin(
            xyz=(
                opening_rear + well_thickness * 0.5,
                0.0,
                DECK_Z - well_depth * 0.5,
            )
        ),
        material=trim_gray,
        name="well_rear",
    )
    cabinet.visual(
        Box((OPENING_DEPTH + 0.012, well_thickness, well_depth)),
        origin=Origin(
            xyz=(
                OPENING_X,
                -(opening_half_width + well_thickness * 0.5),
                DECK_Z - well_depth * 0.5,
            )
        ),
        material=trim_gray,
        name="well_side_0",
    )
    cabinet.visual(
        Box((OPENING_DEPTH + 0.012, well_thickness, well_depth)),
        origin=Origin(
            xyz=(
                OPENING_X,
                opening_half_width + well_thickness * 0.5,
                DECK_Z - well_depth * 0.5,
            )
        ),
        material=trim_gray,
        name="well_side_1",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        cabinet.visual(
            Cylinder(radius=0.040, length=well_depth),
            origin=Origin(
                xyz=(
                    opening_front + 0.040,
                    y_sign * (opening_half_width - 0.002),
                    DECK_Z - well_depth * 0.5,
                )
            ),
            material=trim_gray,
            name=f"well_corner_front_{index}",
        )
        cabinet.visual(
            Cylinder(radius=0.040, length=well_depth),
            origin=Origin(
                xyz=(
                    opening_rear - 0.040,
                    y_sign * (opening_half_width - 0.002),
                    DECK_Z - well_depth * 0.5,
                )
            ),
            material=trim_gray,
            name=f"well_corner_rear_{index}",
        )
    cabinet.visual(
        Box((0.110, 0.600, 0.135)),
        origin=Origin(xyz=(0.301, 0.0, 0.985)),
        material=cabinet_white,
        name="console_body",
    )
    cabinet.visual(
        Box((0.018, 0.600, 0.145)),
        origin=Origin(
            xyz=(CONSOLE_FACE_X, 0.0, CONSOLE_FACE_Z),
            rpy=(0.0, CONSOLE_PITCH, 0.0),
        ),
        material=trim_gray,
        name="console_face",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH, BODY_WIDTH, 1.085)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5425)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "washer_lid"),
        material=trim_gray,
        name="lid_frame",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, 0.060)),
        mass=5.2,
        origin=Origin(xyz=(-LID_DEPTH * 0.5, 0.0, 0.010)),
    )

    lid.visual(
        mesh_from_cadquery(_build_glass_shape(), "washer_lid_glass"),
        material=glass_tint,
        name="glass_panel",
    )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_tub_shape(), "washer_tub"),
        material=steel,
        name="tub_shell",
    )
    tub.inertial = Inertial.from_geometry(
        Cylinder(radius=TUB_OUTER_RADIUS + 0.012, length=TUB_HEIGHT),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -TUB_HEIGHT * 0.5)),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.034,
                body_style="skirted",
                top_diameter=0.058,
                skirt=KnobSkirt(0.082, 0.008, flare=0.05),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "washer_selector_dial",
        ),
        origin=Origin(rpy=(0.0, -math.pi * 0.5, 0.0)),
        material=control_black,
        name="dial_knob",
    )
    selector_dial.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=control_black,
        name="dial_stem",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.034),
        mass=0.20,
        origin=Origin(),
    )

    button_y_positions = (-0.220, -0.150, -0.080, 0.080, 0.150, 0.220)
    for index, local_y in enumerate(button_y_positions):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.008, 0.050, 0.019)),
            origin=Origin(xyz=(-0.004, 0.0, 0.0)),
            material=button_silver,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.008, 0.050, 0.019)),
            mass=0.035,
            origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.052, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=button_silver,
        name="release_cap",
    )
    release_button.visual(
        Box((0.036, 0.010, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=control_black,
        name="release_stem",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.052, 0.018, 0.010)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, DECK_Z + 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(OPENING_X, 0.0, TUB_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=15.0,
        ),
    )
    model.articulation(
        "cabinet_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_dial,
        origin=_console_mount_origin(0.0, local_z=0.060, proud=0.014),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
        ),
    )
    for index, local_y in enumerate(button_y_positions):
        model.articulation(
            f"cabinet_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=model.get_part(f"program_button_{index}"),
            origin=_console_mount_origin(local_y, local_z=0.040),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0025,
            ),
        )
    model.articulation(
        "cabinet_to_release_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=release_button,
        origin=Origin(xyz=(-0.378, 0.0, DECK_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    selector_dial = object_model.get_part("selector_dial")
    tub = object_model.get_part("tub")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    dial_joint = object_model.get_articulation("cabinet_to_selector_dial")
    release_button = object_model.get_part("release_button")
    release_joint = object_model.get_articulation("cabinet_to_release_button")

    ctx.allow_overlap(
        cabinet,
        selector_dial,
        elem_a="console_face",
        elem_b="dial_stem",
        reason="The selector dial shaft intentionally passes through the sloped fascia into the console cavity.",
    )

    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        min_overlap=0.40,
        name="lid covers the washer opening footprint",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        min_gap=0.0,
        max_gap=0.030,
        negative_elem="deck_front",
        name="closed lid sits just above the top deck",
    )

    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check("tub_aabb_present", tub_aabb is not None, details="Expected a world AABB for the tub.")
    if tub_aabb is not None:
        tub_min, tub_max = tub_aabb
        ctx.check(
            "tub_reads_deep",
            float(tub_max[2] - tub_min[2]) >= 0.47,
            details=f"tub_height={float(tub_max[2] - tub_min[2]):.4f}",
        )
        ctx.check(
            "tub_top_below_deck",
            float(tub_max[2]) <= DECK_Z - 0.055,
            details=f"tub_top={float(tub_max[2]):.4f}, deck={DECK_Z:.4f}",
        )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid_opens_upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and float(lid_open_aabb[1][2]) > float(lid_closed_aabb[1][2]) + 0.18,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    program_buttons = [object_model.get_part(f"program_button_{index}") for index in range(6)]
    program_joints = [
        object_model.get_articulation(f"cabinet_to_program_button_{index}")
        for index in range(6)
    ]
    ctx.check(
        "six_program_buttons_present",
        len(program_buttons) == 6 and len(program_joints) == 6,
        details=f"parts={[part.name for part in program_buttons]}, joints={[joint.name for joint in program_joints]}",
    )

    button_rest = ctx.part_world_position(program_buttons[0])
    button_limits = program_joints[0].motion_limits
    button_pressed = None
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({program_joints[0]: button_limits.upper}):
            button_pressed = ctx.part_world_position(program_buttons[0])
    ctx.check(
        "program_button_presses_inward",
        button_rest is not None
        and button_pressed is not None
        and float(button_pressed[0]) > float(button_rest[0]) + 0.0015
        and float(button_pressed[2]) < float(button_rest[2]) - 0.0004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    release_rest = ctx.part_world_position(release_button)
    release_limits = release_joint.motion_limits
    release_pressed = None
    if release_limits is not None and release_limits.upper is not None:
        with ctx.pose({release_joint: release_limits.upper}):
            release_pressed = ctx.part_world_position(release_button)
    ctx.check(
        "release_button_moves_into_deck",
        release_rest is not None
        and release_pressed is not None
        and float(release_pressed[2]) < float(release_rest[2]) - 0.003,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    dial_rest = ctx.part_world_position(selector_dial)
    with ctx.pose({dial_joint: math.pi * 0.5}):
        dial_turned = ctx.part_world_position(selector_dial)
    ctx.check(
        "selector_dial_spins_in_place",
        dial_rest is not None
        and dial_turned is not None
        and abs(float(dial_rest[0]) - float(dial_turned[0])) < 1e-6
        and abs(float(dial_rest[1]) - float(dial_turned[1])) < 1e-6
        and abs(float(dial_rest[2]) - float(dial_turned[2])) < 1e-6
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"rest={dial_rest}, turned={dial_turned}, limits={dial_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
