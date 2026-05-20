from __future__ import annotations

from math import cos, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.84
LOWER_DEPTH = 0.76
LOWER_HEIGHT = 0.98
LOWER_FRONT_Y = LOWER_DEPTH / 2.0

UPPER_DEPTH = 0.54
UPPER_HEIGHT = 0.82
UPPER_CENTER = (0.0, -0.02, 1.34)
UPPER_ROLL = 0.12

CONTROL_DEPTH = 0.40
CONTROL_THICKNESS = 0.08
CONTROL_CENTER = (0.0, 0.15, 0.95)
CONTROL_ROLL = -0.31

SCREEN_BEZEL_CENTER = (0.0, 0.11, 1.31)
SCREEN_BEZEL_SIZE = (0.72, 0.05, 0.60)
SCREEN_ROLL = -0.23

SCREEN_SIZE = (0.56, 0.012, 0.42)
MARQUEE_CENTER = (0.0, 0.06, 1.63)
MARQUEE_SIZE = (0.72, 0.05, 0.18)
MARQUEE_ROLL = -0.10

CASHBOX_SIZE = (0.38, 0.025, 0.36)
CASHBOX_CENTER_Z = 0.53

TRAY_HOUSING_SIZE = (0.22, 0.10, 0.10)
TRAY_HOUSING_CENTER = (0.0, LOWER_FRONT_Y - TRAY_HOUSING_SIZE[1] / 2.0, 0.17)
TRAY_SIZE = (0.18, 0.02, 0.06)
TRAY_HINGE_Z = 0.12


def _rotate_x(local_xyz: tuple[float, float, float], roll: float) -> tuple[float, float, float]:
    x, y, z = local_xyz
    c = cos(roll)
    s = sin(roll)
    return (x, c * y - s * z, s * y + c * z)


def _tilted_point(
    center_xyz: tuple[float, float, float],
    roll: float,
    local_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    rx, ry, rz = _rotate_x(local_xyz, roll)
    return (center_xyz[0] + rx, center_xyz[1] + ry, center_xyz[2] + rz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_machine")

    shell = model.material("shell", rgba=(0.11, 0.11, 0.12, 1.0))
    trim = model.material("trim", rgba=(0.04, 0.04, 0.05, 1.0))
    panel = model.material("panel", rgba=(0.14, 0.14, 0.16, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.06, 0.08, 0.10, 1.0))
    marquee = model.material("marquee", rgba=(0.66, 0.11, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    red = model.material("red", rgba=(0.74, 0.12, 0.10, 1.0))
    blue = model.material("blue", rgba=(0.10, 0.32, 0.78, 1.0))
    white = model.material("white", rgba=(0.92, 0.92, 0.90, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_WIDTH, LOWER_DEPTH, LOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT / 2.0)),
        material=shell,
        name="lower_body",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, UPPER_DEPTH, UPPER_HEIGHT)),
        origin=Origin(xyz=UPPER_CENTER, rpy=(UPPER_ROLL, 0.0, 0.0)),
        material=shell,
        name="upper_body",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, 0.07, 0.13)),
        origin=Origin(xyz=(0.0, 0.345, 0.84)),
        material=shell,
        name="control_apron",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CONTROL_DEPTH, CONTROL_THICKNESS)),
        origin=Origin(xyz=CONTROL_CENTER, rpy=(CONTROL_ROLL, 0.0, 0.0)),
        material=panel,
        name="control_panel",
    )
    cabinet.visual(
        Box(SCREEN_BEZEL_SIZE),
        origin=Origin(xyz=SCREEN_BEZEL_CENTER, rpy=(SCREEN_ROLL, 0.0, 0.0)),
        material=trim,
        name="screen_bezel",
    )
    cabinet.visual(
        Box(SCREEN_SIZE),
        origin=Origin(
            xyz=_tilted_point(SCREEN_BEZEL_CENTER, SCREEN_ROLL, (0.0, 0.0, -0.020)),
            rpy=(SCREEN_ROLL, 0.0, 0.0),
        ),
        material=screen_glass,
        name="screen",
    )
    cabinet.visual(
        Box(MARQUEE_SIZE),
        origin=Origin(xyz=MARQUEE_CENTER, rpy=(MARQUEE_ROLL, 0.0, 0.0)),
        material=marquee,
        name="marquee",
    )
    cabinet.visual(
        Box(TRAY_HOUSING_SIZE),
        origin=Origin(xyz=TRAY_HOUSING_CENTER),
        material=trim,
        name="tray_housing",
    )

    joystick_stem_length = 0.11
    joystick_ball_radius = 0.026
    control_top = CONTROL_THICKNESS / 2.0
    player_offsets = (-0.23, 0.23)
    button_positions = {
        -1: [(-0.14, 0.010), (-0.09, 0.050), (-0.04, 0.085)],
        1: [(0.04, 0.085), (0.09, 0.050), (0.14, 0.010)],
    }
    button_materials = (red, blue, white)

    for index, x_pos in enumerate(player_offsets):
        stem_center = _tilted_point(
            CONTROL_CENTER,
            CONTROL_ROLL,
            (x_pos, -0.030, control_top + joystick_stem_length / 2.0 - 0.003),
        )
        cabinet.visual(
            Cylinder(radius=0.008, length=joystick_stem_length),
            origin=Origin(xyz=stem_center, rpy=(CONTROL_ROLL, 0.0, 0.0)),
            material=steel,
            name=f"joystick_stem_{index}",
        )

        ball_center = _tilted_point(
            CONTROL_CENTER,
            CONTROL_ROLL,
            (x_pos, -0.030, control_top + joystick_stem_length + joystick_ball_radius - 0.003),
        )
        cabinet.visual(
            Sphere(radius=joystick_ball_radius),
            origin=Origin(xyz=ball_center),
            material=red if index == 0 else blue,
            name=f"joystick_ball_{index}",
        )

    for side, positions in button_positions.items():
        for button_index, (x_pos, y_pos) in enumerate(positions):
            button_center = _tilted_point(
                CONTROL_CENTER,
                CONTROL_ROLL,
                (x_pos, y_pos, control_top + 0.009 - 0.003),
            )
            cabinet.visual(
                Cylinder(radius=0.019, length=0.018),
                origin=Origin(xyz=button_center, rpy=(CONTROL_ROLL, 0.0, 0.0)),
                material=button_materials[button_index],
                name=f"button_{0 if side < 0 else 1}_{button_index}",
            )

    for start_index, x_pos in enumerate((-0.03, 0.03)):
        start_center = _tilted_point(
            CONTROL_CENTER,
            CONTROL_ROLL,
            (x_pos, -0.105, control_top + 0.006 - 0.002),
        )
        cabinet.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=start_center, rpy=(CONTROL_ROLL, 0.0, 0.0)),
            material=trim,
            name=f"start_button_{start_index}",
        )

    cashbox = model.part("cashbox")
    cashbox.visual(
        Box(CASHBOX_SIZE),
        origin=Origin(xyz=(CASHBOX_SIZE[0] / 2.0, 0.0, 0.0)),
        material=panel,
        name="door_panel",
    )
    cashbox.visual(
        Cylinder(radius=0.011, length=CASHBOX_SIZE[2]),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    cashbox.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(
            xyz=(CASHBOX_SIZE[0] - 0.070, 0.020, 0.0),
            rpy=(-1.5708, 0.0, 0.0),
        ),
        material=steel,
        name="lock",
    )

    tray_flap = model.part("tray_flap")
    tray_flap.visual(
        Box(TRAY_SIZE),
        origin=Origin(xyz=(0.0, 0.0, TRAY_SIZE[2] / 2.0)),
        material=panel,
        name="tray_panel",
    )
    tray_flap.visual(
        Cylinder(radius=0.008, length=TRAY_SIZE[0]),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(0.0, 1.5708, 0.0)),
        material=steel,
        name="tray_hinge",
    )

    model.articulation(
        "cabinet_to_cashbox",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cashbox,
        origin=Origin(
            xyz=(
                -CASHBOX_SIZE[0] / 2.0,
                LOWER_FRONT_Y + CASHBOX_SIZE[1] / 2.0,
                CASHBOX_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=20.0, velocity=1.4),
    )
    model.articulation(
        "cabinet_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=(0.0, LOWER_FRONT_Y + TRAY_SIZE[1] / 2.0, TRAY_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=6.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    cashbox = object_model.get_part("cashbox")
    tray_flap = object_model.get_part("tray_flap")
    cashbox_hinge = object_model.get_articulation("cabinet_to_cashbox")
    tray_hinge = object_model.get_articulation("cabinet_to_tray_flap")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check("cabinet_aabb_present", cabinet_aabb is not None, details=f"aabb={cabinet_aabb!r}")
    if cabinet_aabb is not None:
        mins, maxs = cabinet_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("commercial_arcade_width", 0.78 <= size[0] <= 0.95, details=f"size={size!r}")
        ctx.check("commercial_arcade_depth", 0.68 <= size[1] <= 0.92, details=f"size={size!r}")
        ctx.check("commercial_arcade_height", 1.65 <= size[2] <= 1.90, details=f"size={size!r}")

    ctx.expect_gap(
        cashbox,
        cabinet,
        axis="y",
        positive_elem="door_panel",
        negative_elem="lower_body",
        max_gap=0.010,
        max_penetration=0.001,
        name="cashbox door closes against lower front",
    )
    ctx.expect_overlap(
        cashbox,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="lower_body",
        min_overlap=0.24,
        name="cashbox door covers a large lower front opening",
    )
    ctx.expect_gap(
        tray_flap,
        cabinet,
        axis="y",
        positive_elem="tray_panel",
        negative_elem="tray_housing",
        max_gap=0.010,
        max_penetration=0.001,
        name="tray flap closes against tray surround",
    )
    ctx.expect_overlap(
        tray_flap,
        cabinet,
        axes="xz",
        elem_a="tray_panel",
        elem_b="tray_housing",
        min_overlap=0.05,
        name="tray flap sits within the payout tray surround",
    )

    door_aabb = ctx.part_element_world_aabb(cashbox, elem="door_panel")
    tray_aabb = ctx.part_element_world_aabb(tray_flap, elem="tray_panel")
    screen_aabb = ctx.part_element_world_aabb(cabinet, elem="screen")
    if door_aabb is not None and screen_aabb is not None:
        door_mins, door_maxs = door_aabb
        screen_mins, _ = screen_aabb
        ctx.check(
            "cashbox below screen",
            float(door_maxs[2]) < float(screen_mins[2]) - 0.08,
            details=f"door_top={float(door_maxs[2]):.3f}, screen_bottom={float(screen_mins[2]):.3f}",
        )
    if door_aabb is not None and tray_aabb is not None:
        _, tray_maxs = tray_aabb
        door_mins, _ = door_aabb
        ctx.check(
            "tray below cashbox",
            float(tray_maxs[2]) < float(door_mins[2]) - 0.08,
            details=f"tray_top={float(tray_maxs[2]):.3f}, door_bottom={float(door_mins[2]):.3f}",
        )

    cashbox_closed = ctx.part_world_aabb(cashbox)
    if cashbox_closed is not None:
        with ctx.pose({cashbox_hinge: cashbox_hinge.motion_limits.upper}):
            cashbox_open = ctx.part_world_aabb(cashbox)
        ctx.check(
            "cashbox swings outward",
            cashbox_open is not None and float(cashbox_open[1][1]) > float(cashbox_closed[1][1]) + 0.18,
            details=f"closed={cashbox_closed!r}, open={cashbox_open!r}",
        )

    tray_closed = ctx.part_element_world_aabb(tray_flap, elem="tray_panel")
    if tray_closed is not None:
        with ctx.pose({tray_hinge: tray_hinge.motion_limits.upper}):
            tray_open = ctx.part_element_world_aabb(tray_flap, elem="tray_panel")
        ctx.check(
            "tray flap opens down and out",
            tray_open is not None
            and float(tray_open[1][1]) > float(tray_closed[1][1]) + 0.03
            and float(tray_open[1][2]) < float(tray_closed[1][2]) - 0.02,
            details=f"closed={tray_closed!r}, open={tray_open!r}",
        )

    return ctx.report()


object_model = build_object_model()
