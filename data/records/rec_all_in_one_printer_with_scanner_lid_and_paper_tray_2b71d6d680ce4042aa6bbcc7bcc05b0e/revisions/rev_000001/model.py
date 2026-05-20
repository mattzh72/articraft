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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.470
BODY_D = 0.440
BODY_H = 0.270
BODY_WALL = 0.012

LID_W = 0.335
LID_D = 0.352
LID_T = 0.044
LID_WALL = 0.008
LID_X = -0.052
LID_HINGE_Y = 0.196
LID_Z = BODY_H + 0.002

DRAWER_TRAY_W = 0.352
DRAWER_TRAY_D = 0.302
DRAWER_TRAY_H = 0.150
DRAWER_PANEL_W = 0.402
DRAWER_PANEL_H = 0.170
DRAWER_PANEL_T = 0.012
DRAWER_OPENING_Z = 0.024
DRAWER_TRAVEL = 0.155

SERVICE_DOOR_T = 0.008
SERVICE_DOOR_D = 0.160
SERVICE_DOOR_H = 0.148
SERVICE_DOOR_X = BODY_W * 0.5
SERVICE_DOOR_Y = -0.048
SERVICE_DOOR_Z = 0.082

CONTROL_PANEL_X = 0.138
CONTROL_PANEL_Y = -0.028
CONTROL_PANEL_Z = BODY_H - 0.004

KEY_PITCH_X = 0.022
KEY_PITCH_Y = 0.019
KEY_START_X = CONTROL_PANEL_X - 0.026
KEY_START_Y = CONTROL_PANEL_Y + 0.026

KEY_ROWS = 4
KEY_COLS = 3


def _box(
    sx: float,
    sy: float,
    sz: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz, centered=centered).translate(translate)


def _vertical_cylinder(
    radius: float,
    height: float,
    *,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate(translate)


def _build_body_shape() -> cq.Workplane:
    body = _box(BODY_W, BODY_D, BODY_H)
    body = body.edges("|Z").fillet(0.016)

    inner_shell = _box(
        BODY_W - 2.0 * BODY_WALL,
        BODY_D - 2.0 * BODY_WALL,
        BODY_H - 2.0 * BODY_WALL,
        translate=(0.0, 0.0, BODY_WALL),
    )
    body = body.cut(inner_shell)

    drawer_void = _box(
        0.394,
        0.314,
        0.166,
        translate=(0.0, -BODY_D * 0.5 + 0.157, DRAWER_OPENING_Z),
    )
    body = body.cut(drawer_void)

    output_slot = _box(
        0.286,
        0.132,
        0.030,
        translate=(0.0, -BODY_D * 0.5 + 0.066, 0.198),
    )
    body = body.cut(output_slot)

    top_pocket = _box(
        0.318,
        0.244,
        0.010,
        translate=(LID_X, 0.012, BODY_H - 0.010),
    )
    flatbed_opening = _box(
        0.298,
        0.224,
        0.034,
        translate=(LID_X, 0.012, BODY_H - 0.034),
    )
    body = body.cut(top_pocket).cut(flatbed_opening)

    control_pocket = _box(
        0.122,
        0.096,
        0.004,
        translate=(CONTROL_PANEL_X, CONTROL_PANEL_Y, BODY_H - 0.004),
    )
    body = body.cut(control_pocket)

    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            key_x = KEY_START_X + col * KEY_PITCH_X
            key_y = KEY_START_Y - row * KEY_PITCH_Y
            body = body.cut(
                _box(
                    0.0098,
                    0.0098,
                    0.014,
                    translate=(key_x, key_y, BODY_H - 0.014),
                )
            )

    body = body.cut(
        _vertical_cylinder(
            0.0082,
            0.014,
            translate=(CONTROL_PANEL_X + 0.030, CONTROL_PANEL_Y - 0.010, BODY_H - 0.014),
        )
    )
    body = body.cut(
        _vertical_cylinder(
            0.0052,
            0.016,
            translate=(CONTROL_PANEL_X + 0.029, CONTROL_PANEL_Y + 0.026, BODY_H - 0.016),
        )
    )

    service_opening = _box(
        0.022,
        SERVICE_DOOR_D,
        SERVICE_DOOR_H,
        translate=(BODY_W * 0.5 - 0.022, SERVICE_DOOR_Y + SERVICE_DOOR_D * 0.5, SERVICE_DOOR_Z),
    )
    body = body.cut(service_opening)

    guide_floor = _box(
        0.366,
        0.264,
        0.006,
        translate=(0.0, -BODY_D * 0.5 + 0.138, DRAWER_OPENING_Z),
    )
    body = body.union(guide_floor)

    rear_hinge_bar = (
        cq.Workplane("YZ")
        .circle(0.007)
        .extrude(0.120)
        .translate((-0.060, BODY_D * 0.5 - 0.010, BODY_H + 0.005))
    )
    body = body.union(rear_hinge_bar)

    return body


def _build_lid_shape() -> cq.Workplane:
    lid = _box(
        LID_W,
        LID_D,
        LID_T,
        centered=(True, False, False),
        translate=(0.0, -LID_D, 0.0),
    )

    inner_cavity = _box(
        LID_W - 2.0 * LID_WALL,
        LID_D - 2.0 * LID_WALL,
        LID_T - LID_WALL,
        centered=(True, False, False),
        translate=(0.0, -LID_D + LID_WALL, LID_WALL),
    )
    lid = lid.cut(inner_cavity)

    front_lip = _box(
        0.130,
        0.010,
        0.008,
        centered=(True, False, False),
        translate=(0.0, -LID_D - 0.010, 0.010),
    )
    lid = lid.union(front_lip)

    hinge_tube = (
        cq.Workplane("YZ")
        .circle(0.007)
        .extrude(0.110)
        .translate((-0.055, 0.0, 0.005))
    )
    lid = lid.union(hinge_tube)
    return lid


def _build_drawer_shape() -> cq.Workplane:
    front_panel = _box(
        DRAWER_PANEL_W,
        DRAWER_PANEL_T,
        DRAWER_PANEL_H,
        centered=(True, False, False),
        translate=(0.0, -DRAWER_PANEL_T, 0.0),
    )
    tray_outer = _box(
        DRAWER_TRAY_W,
        DRAWER_TRAY_D,
        DRAWER_TRAY_H,
        centered=(True, False, False),
        translate=(0.0, 0.0, 0.0),
    )
    tray_inner = _box(
        DRAWER_TRAY_W - 0.014,
        DRAWER_TRAY_D - 0.020,
        DRAWER_TRAY_H - 0.010,
        centered=(True, False, False),
        translate=(0.0, 0.008, 0.006),
    )
    drawer = front_panel.union(tray_outer.cut(tray_inner))

    handle_slot = _box(
        0.118,
        0.007,
        0.022,
        centered=(True, False, False),
        translate=(0.0, -DRAWER_PANEL_T - 0.001, 0.090),
    )
    drawer = drawer.cut(handle_slot)

    left_rail = _box(
        0.008,
        0.240,
        0.018,
        centered=(True, False, False),
        translate=(-DRAWER_TRAY_W * 0.5 + 0.008, 0.030, 0.018),
    )
    right_rail = _box(
        0.008,
        0.240,
        0.018,
        centered=(True, False, False),
        translate=(DRAWER_TRAY_W * 0.5 - 0.008, 0.030, 0.018),
    )
    drawer = drawer.union(left_rail).union(right_rail)
    return drawer


def _build_service_door_shape() -> cq.Workplane:
    panel = _box(
        SERVICE_DOOR_T,
        SERVICE_DOOR_D,
        SERVICE_DOOR_H,
        centered=(False, False, False),
        translate=(0.0, 0.0, 0.0),
    )
    inner_frame = _box(
        0.004,
        SERVICE_DOOR_D - 0.026,
        SERVICE_DOOR_H - 0.026,
        centered=(False, False, False),
        translate=(0.0, 0.013, 0.013),
    )
    pull = _box(
        0.012,
        0.042,
        0.014,
        centered=(False, False, False),
        translate=(SERVICE_DOOR_T, 0.060, 0.067),
    )
    hinge = _vertical_cylinder(0.004, SERVICE_DOOR_H, translate=(0.0, 0.0, 0.0))
    return panel.union(inner_frame).union(pull).union(hinge)


def _build_key_mesh() -> object:
    key_shape = _box(0.014, 0.012, 0.0032)
    return mesh_from_cadquery(key_shape, "printer_keycap")


def _build_start_button_mesh() -> object:
    button_shape = cq.Workplane("XY").circle(0.0085).extrude(0.0036)
    return mesh_from_cadquery(button_shape, "printer_start_button")


def _add_key(
    model: ArticulatedObject,
    body,
    key_mesh: object,
    key_material,
    *,
    name: str,
    x: float,
    y: float,
) -> None:
    key = model.part(name)
    key.visual(key_mesh, material=key_material, name="keycap")
    key.visual(
        Box((0.0082, 0.0082, 0.0080)),
        origin=Origin(xyz=(0.0, 0.0, -0.0040)),
        material=key_material,
        name="stem",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.0115)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, -0.0022)),
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=key,
        origin=Origin(xyz=(x, y, CONTROL_PANEL_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.0026),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_all_in_one_printer")

    housing = model.material("housing", rgba=(0.76, 0.77, 0.79, 1.0))
    trim = model.material("trim", rgba=(0.61, 0.63, 0.66, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.24, 0.26, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.18, 0.20, 0.55))
    control = model.material("control", rgba=(0.32, 0.34, 0.37, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shape(), "printer_body"), material=housing, name="shell")
    body.visual(
        Box((0.298, 0.224, 0.002)),
        origin=Origin(xyz=(LID_X, 0.012, BODY_H - 0.009)),
        material=glass,
        name="scanner_glass",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(mesh_from_cadquery(_build_lid_shape(), "scanner_lid"), material=trim, name="lid_shell")
    scanner_lid.visual(
        Box((0.008, 0.008, 0.002)),
        origin=Origin(xyz=(-0.130, -0.106, -0.001)),
        material=dark_trim,
        name="bumper_0",
    )
    scanner_lid.visual(
        Box((0.008, 0.008, 0.002)),
        origin=Origin(xyz=(0.028, -0.106, -0.001)),
        material=dark_trim,
        name="bumper_1",
    )
    scanner_lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=2.1,
        origin=Origin(xyz=(0.0, -LID_D * 0.5, LID_T * 0.5)),
    )
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=scanner_lid,
        origin=Origin(xyz=(LID_X, LID_HINGE_Y, LID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    paper_drawer = model.part("paper_drawer")
    paper_drawer.visual(
        Box((DRAWER_PANEL_W, DRAWER_PANEL_T, DRAWER_PANEL_H)),
        origin=Origin(xyz=(0.0, -0.006, DRAWER_PANEL_H * 0.5)),
        material=trim,
        name="front_panel",
    )
    paper_drawer.visual(
        Box((DRAWER_TRAY_W, DRAWER_TRAY_D, 0.006)),
        origin=Origin(xyz=(0.0, DRAWER_TRAY_D * 0.5, 0.003)),
        material=trim,
        name="tray_floor",
    )
    paper_drawer.visual(
        Box((0.006, DRAWER_TRAY_D, DRAWER_TRAY_H)),
        origin=Origin(xyz=(-DRAWER_TRAY_W * 0.5 + 0.003, DRAWER_TRAY_D * 0.5, DRAWER_TRAY_H * 0.5)),
        material=trim,
        name="side_wall_0",
    )
    paper_drawer.visual(
        Box((0.006, DRAWER_TRAY_D, DRAWER_TRAY_H)),
        origin=Origin(xyz=(DRAWER_TRAY_W * 0.5 - 0.003, DRAWER_TRAY_D * 0.5, DRAWER_TRAY_H * 0.5)),
        material=trim,
        name="side_wall_1",
    )
    paper_drawer.visual(
        Box((DRAWER_TRAY_W, 0.006, DRAWER_TRAY_H)),
        origin=Origin(xyz=(0.0, DRAWER_TRAY_D - 0.003, DRAWER_TRAY_H * 0.5)),
        material=trim,
        name="rear_wall",
    )
    paper_drawer.visual(
        Box((0.120, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.0135, 0.095)),
        material=dark_trim,
        name="handle_grip",
    )
    paper_drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_PANEL_W, DRAWER_TRAY_D + DRAWER_PANEL_T, DRAWER_PANEL_H)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.145, 0.085)),
    )
    model.articulation(
        "body_to_paper_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_drawer,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 - 0.003, DRAWER_OPENING_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((SERVICE_DOOR_T, SERVICE_DOOR_D, SERVICE_DOOR_H)),
        origin=Origin(xyz=(SERVICE_DOOR_T * 0.5, SERVICE_DOOR_D * 0.5, SERVICE_DOOR_H * 0.5)),
        material=trim,
        name="door_panel",
    )
    service_door.visual(
        Box((0.010, 0.040, 0.014)),
        origin=Origin(xyz=(0.013, 0.080, 0.074)),
        material=dark_trim,
        name="door_pull",
    )
    service_door.visual(
        Cylinder(radius=0.004, length=SERVICE_DOOR_H),
        origin=Origin(xyz=(0.004, 0.004, SERVICE_DOOR_H * 0.5)),
        material=trim,
        name="hinge_barrel",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.020, SERVICE_DOOR_D, SERVICE_DOOR_H)),
        mass=0.35,
        origin=Origin(xyz=(0.010, SERVICE_DOOR_D * 0.5, SERVICE_DOOR_H * 0.5)),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(SERVICE_DOOR_X, SERVICE_DOOR_Y, SERVICE_DOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    key_mesh = _build_key_mesh()
    for row in range(KEY_ROWS):
        for col in range(KEY_COLS):
            _add_key(
                model,
                body,
                key_mesh,
                control,
                name=f"key_{row}_{col}",
                x=KEY_START_X + col * KEY_PITCH_X,
                y=KEY_START_Y - row * KEY_PITCH_Y,
            )

    start_button = model.part("start_button")
    start_button.visual(
        _build_start_button_mesh(),
        material=dark_trim,
        name="button_cap",
    )
    start_button.visual(
        Cylinder(radius=0.0065, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, -0.0040)),
        material=dark_trim,
        name="stem",
    )
    start_button.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.012)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(CONTROL_PANEL_X + 0.030, CONTROL_PANEL_Y - 0.010, CONTROL_PANEL_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.5, velocity=0.08, lower=0.0, upper=0.0024),
    )

    menu_dial = model.part("menu_dial")
    menu_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.013,
                body_style="cylindrical",
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=18, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005, angle_deg=18.0),
                center=False,
            ),
            "menu_dial",
        ),
        material=dark_trim,
        name="dial_cap",
    )
    menu_dial.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_trim,
        name="shaft",
    )
    menu_dial.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.023)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "body_to_menu_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=menu_dial,
        origin=Origin(xyz=(CONTROL_PANEL_X + 0.029, CONTROL_PANEL_Y + 0.026, BODY_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    scanner_lid = object_model.get_part("scanner_lid")
    paper_drawer = object_model.get_part("paper_drawer")
    service_door = object_model.get_part("service_door")
    start_button = object_model.get_part("start_button")
    key_0_0 = object_model.get_part("key_0_0")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    drawer_slide = object_model.get_articulation("body_to_paper_drawer")
    door_hinge = object_model.get_articulation("body_to_service_door")
    start_joint = object_model.get_articulation("body_to_start_button")
    key_joint = object_model.get_articulation("body_to_key_0_0")
    dial_joint = object_model.get_articulation("body_to_menu_dial")

    ctx.allow_isolated_part(
        scanner_lid,
        reason="The scanner lid is supported by the rear hinge and underside bumpers, but the floating-part pass still resolves a hairline 1e-6 clearance at rest.",
    )
    ctx.allow_overlap(
        body,
        paper_drawer,
        reason="The closed paper drawer is nested inside the chassis cavity; the current shell representation still reports overlap against retained drawer surfaces at rest.",
    )

    ctx.expect_within(
        paper_drawer,
        body,
        axes="xz",
        margin=0.0,
        name="paper drawer stays aligned within the body opening",
    )
    ctx.expect_overlap(
        paper_drawer,
        body,
        axes="y",
        min_overlap=0.29,
        name="closed paper drawer remains deeply inserted into the body",
    )

    lid_aabb = ctx.part_world_aabb(scanner_lid)
    glass_aabb = ctx.part_element_world_aabb(body, elem="scanner_glass")
    closed_gap = None
    if lid_aabb is not None and glass_aabb is not None:
        closed_gap = float(lid_aabb[0][2]) - float(glass_aabb[1][2])
    ctx.check(
        "scanner lid rests just above the flatbed glass",
        closed_gap is not None and 0.006 <= closed_gap <= 0.012,
        details=f"gap={closed_gap!r}, lid={lid_aabb!r}, glass={glass_aabb!r}",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "menu dial uses continuous unbounded limits",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits!r}",
    )

    closed_lid_aabb = ctx.part_world_aabb(scanner_lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(scanner_lid)
    ctx.check(
        "scanner lid rotates upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.17,
        details=f"closed={closed_lid_aabb!r}, open={open_lid_aabb!r}",
    )

    drawer_rest = ctx.part_world_position(paper_drawer)
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        drawer_open = ctx.part_world_position(paper_drawer)
        ctx.expect_within(
            paper_drawer,
            body,
            axes="xz",
            margin=0.002,
            name="extended drawer remains centered in the printer body",
        )
        ctx.expect_overlap(
            paper_drawer,
            body,
            axes="y",
            min_overlap=0.14,
            name="extended paper drawer still retains insertion in the chassis",
        )
    ctx.check(
        "paper drawer slides forward",
        drawer_rest is not None and drawer_open is not None and float(drawer_open[1]) < float(drawer_rest[1]) - 0.10,
        details=f"rest={drawer_rest!r}, open={drawer_open!r}",
    )

    closed_door_aabb = ctx.part_world_aabb(service_door)
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(service_door)
    ctx.check(
        "service door swings outward from the side bay",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and float(open_door_aabb[1][0]) > float(closed_door_aabb[1][0]) + 0.08,
        details=f"closed={closed_door_aabb!r}, open={open_door_aabb!r}",
    )

    key_count = sum(1 for row in range(KEY_ROWS) for col in range(KEY_COLS) if object_model.get_part(f"key_{row}_{col}") is not None)
    ctx.check("all keypad keys are present", key_count == KEY_ROWS * KEY_COLS, details=f"count={key_count}")

    key_rest = ctx.part_world_position(key_0_0)
    with ctx.pose({key_joint: key_joint.motion_limits.upper}):
        key_pressed = ctx.part_world_position(key_0_0)
    ctx.check(
        "a keypad key pushes downward",
        key_rest is not None and key_pressed is not None and float(key_pressed[2]) < float(key_rest[2]) - 0.0015,
        details=f"rest={key_rest!r}, pressed={key_pressed!r}",
    )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({start_joint: start_joint.motion_limits.upper}):
        button_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button pushes downward",
        button_rest is not None
        and button_pressed is not None
        and float(button_pressed[2]) < float(button_rest[2]) - 0.0012,
        details=f"rest={button_rest!r}, pressed={button_pressed!r}",
    )

    return ctx.report()


object_model = build_object_model()
