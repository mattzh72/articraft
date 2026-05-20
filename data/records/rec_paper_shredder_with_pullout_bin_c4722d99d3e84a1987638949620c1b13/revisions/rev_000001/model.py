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


CABINET_WIDTH = 0.340
CABINET_DEPTH = 0.280
CABINET_BASE_Z = 0.046
CABINET_HEIGHT = 0.440
WALL_T = 0.012
FLOOR_T = 0.008

BIN_WIDTH = 0.306
BIN_DEPTH = 0.236
BIN_HEIGHT = 0.332
BIN_WALL = 0.008

HEAD_WIDTH = 0.368
HEAD_DEPTH = 0.310
HEAD_HEIGHT = 0.200
HEAD_WALL = 0.010

CONTROL_BANK_WIDTH = 0.170
CONTROL_BANK_DEPTH = 0.028
CONTROL_BANK_HEIGHT = 0.096

WHEEL_RADIUS = 0.022
WHEEL_WIDTH = 0.016


def _head_shell_mesh():
    shell = cq.Workplane("XY").box(
        HEAD_WIDTH,
        HEAD_DEPTH,
        HEAD_HEIGHT,
        centered=(True, True, False),
    )
    shell = shell.edges("|Z").fillet(0.012)

    inner = cq.Workplane("XY").transformed(offset=(0.0, 0.0, 0.0)).box(
        HEAD_WIDTH - 2.0 * HEAD_WALL,
        HEAD_DEPTH - 2.0 * HEAD_WALL,
        HEAD_HEIGHT - HEAD_WALL,
        centered=(True, True, False),
    )
    shell = shell.cut(inner)

    slot = cq.Workplane("XY").transformed(
        offset=(0.0, 0.028, HEAD_HEIGHT - 0.022)
    ).box(
        0.228,
        0.020,
        0.040,
        centered=(True, True, False),
    )
    shell = shell.cut(slot)

    return mesh_from_cadquery(shell, "office_shredder_head")


def _bin_mesh():
    shell = cq.Workplane("XY").box(
        BIN_WIDTH,
        BIN_DEPTH,
        BIN_HEIGHT,
        centered=(True, True, False),
    )
    shell = shell.edges("|Z").fillet(0.008)

    inner = cq.Workplane("XY").transformed(offset=(0.0, 0.0, BIN_WALL)).box(
        BIN_WIDTH - 2.0 * BIN_WALL,
        BIN_DEPTH - 2.0 * BIN_WALL,
        BIN_HEIGHT - BIN_WALL,
        centered=(True, True, False),
    )
    shell = shell.cut(inner)

    grip = cq.Workplane("XY").transformed(
        offset=(0.0, BIN_DEPTH * 0.46, BIN_HEIGHT * 0.73)
    ).box(
        0.118,
        0.018,
        0.032,
        centered=(True, True, True),
    )
    shell = shell.cut(grip)

    return mesh_from_cadquery(shell, "office_shredder_bin")


def _add_caster(model: ArticulatedObject, cabinet, *, index: int, x: float, y: float, metal, tire):
    fork = model.part(f"caster_fork_{index}")
    fork.visual(
        Box((0.030, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=metal,
        name="mount_plate",
    )
    fork.visual(
        Box((0.004, 0.014, 0.026)),
        origin=Origin(xyz=(0.0105, 0.0, 0.005)),
        material=metal,
        name="cheek_0",
    )
    fork.visual(
        Box((0.004, 0.014, 0.026)),
        origin=Origin(xyz=(-0.0105, 0.0, 0.005)),
        material=metal,
        name="cheek_1",
    )

    model.articulation(
        f"cabinet_to_caster_fork_{index}",
        ArticulationType.FIXED,
        parent=cabinet,
        child=fork,
        origin=Origin(xyz=(x, y, WHEEL_RADIUS)),
    )

    wheel = model.part(f"caster_wheel_{index}")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tire,
        name="tread",
    )
    wheel.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="axle_sleeve",
    )

    model.articulation(
        f"caster_fork_{index}_to_caster_wheel_{index}",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )


def _add_drum(model: ArticulatedObject, head, *, name: str, y: float, axis: tuple[float, float, float], metal):
    drum = model.part(name)
    drum.visual(
        Cylinder(radius=0.016, length=0.308),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )

    cutter_positions = (
        -0.108,
        -0.084,
        -0.060,
        -0.036,
        -0.012,
        0.012,
        0.036,
        0.060,
        0.084,
        0.108,
    )
    for index, x in enumerate(cutter_positions):
        drum.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"cutter_{index}",
        )

    model.articulation(
        f"head_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=drum,
        origin=Origin(xyz=(0.0, y, 0.068)),
        axis=axis,
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_shredder")

    cabinet_dark = model.material("cabinet_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    head_dark = model.material("head_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.39, 0.41, 0.43, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    start_green = model.material("start_green", rgba=(0.16, 0.61, 0.33, 1.0))
    reverse_red = model.material("reverse_red", rgba=(0.72, 0.18, 0.18, 1.0))
    dial_black = model.material("dial_black", rgba=(0.15, 0.15, 0.16, 1.0))
    slot_trim = model.material("slot_trim", rgba=(0.22, 0.23, 0.24, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((WALL_T, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH - WALL_T) / 2.0,
                0.0,
                CABINET_BASE_Z + CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_dark,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((WALL_T, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH - WALL_T) / 2.0,
                0.0,
                CABINET_BASE_Z + CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_dark,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_T, WALL_T, CABINET_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH - WALL_T) / 2.0,
                CABINET_BASE_Z + CABINET_HEIGHT / 2.0,
            )
        ),
        material=cabinet_dark,
        name="back_wall",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_T, CABINET_DEPTH - 0.034, FLOOR_T)),
        origin=Origin(
            xyz=(
                0.0,
                -0.009,
                CABINET_BASE_Z + FLOOR_T / 2.0,
            )
        ),
        material=trim_grey,
        name="floor_plate",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, 0.016, 0.080)),
        origin=Origin(
            xyz=(
                0.0,
                (CABINET_DEPTH - 0.016) / 2.0,
                CABINET_BASE_Z + CABINET_HEIGHT - 0.040,
            )
        ),
        material=cabinet_dark,
        name="front_fascia",
    )
    cabinet.visual(
        Box((0.020, CABINET_DEPTH - 0.018, 0.014)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH - 0.020) / 2.0,
                -0.009,
                CABINET_BASE_Z + CABINET_HEIGHT - 0.007,
            )
        ),
        material=trim_grey,
        name="top_rim_0",
    )
    cabinet.visual(
        Box((0.020, CABINET_DEPTH - 0.018, 0.014)),
        origin=Origin(
            xyz=(
                (CABINET_WIDTH - 0.020) / 2.0,
                -0.009,
                CABINET_BASE_Z + CABINET_HEIGHT - 0.007,
            )
        ),
        material=trim_grey,
        name="top_rim_1",
    )
    cabinet.visual(
        Box((CABINET_WIDTH - 0.040, 0.020, 0.014)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH - 0.020) / 2.0,
                CABINET_BASE_Z + CABINET_HEIGHT - 0.007,
            )
        ),
        material=trim_grey,
        name="top_back_rim",
    )

    bin_part = model.part("bin")
    bin_part.visual(
        _bin_mesh(),
        origin=Origin(xyz=(0.0, -BIN_DEPTH / 2.0, 0.0)),
        material=trim_grey,
        name="bin_shell",
    )

    model.articulation(
        "cabinet_to_bin",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=bin_part,
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0,
                CABINET_BASE_Z + FLOOR_T,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.165),
    )

    head = model.part("head")
    head.visual(_head_shell_mesh(), material=head_dark, name="head_shell")
    head.visual(
        Box((0.244, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, HEAD_HEIGHT - 0.005)),
        material=slot_trim,
        name="slot_bezel",
    )
    head.visual(
        Box((0.020, 0.244, 0.008)),
        origin=Origin(xyz=(-(HEAD_WIDTH - 0.020) / 2.0, -0.005, 0.004)),
        material=head_dark,
        name="mount_rail_0",
    )
    head.visual(
        Box((0.020, 0.244, 0.008)),
        origin=Origin(xyz=((HEAD_WIDTH - 0.020) / 2.0, -0.005, 0.004)),
        material=head_dark,
        name="mount_rail_1",
    )
    head.visual(
        Box((HEAD_WIDTH - 0.040, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -(HEAD_DEPTH - 0.020) / 2.0, 0.004)),
        material=head_dark,
        name="mount_rail_2",
    )
    head.visual(
        Box((0.020, 0.094, 0.040)),
        origin=Origin(xyz=(-(HEAD_WIDTH / 2.0 - HEAD_WALL - 0.010), 0.0, 0.068)),
        material=head_dark,
        name="bearing_wall_0",
    )
    head.visual(
        Box((0.020, 0.094, 0.040)),
        origin=Origin(xyz=((HEAD_WIDTH / 2.0 - HEAD_WALL - 0.010), 0.0, 0.068)),
        material=head_dark,
        name="bearing_wall_1",
    )

    model.articulation(
        "cabinet_to_head",
        ArticulationType.FIXED,
        parent=cabinet,
        child=head,
        origin=Origin(xyz=(0.0, 0.004, CABINET_BASE_Z + CABINET_HEIGHT)),
    )

    control_bank = model.part("control_bank")
    control_bank.visual(
        Box((CONTROL_BANK_WIDTH, CONTROL_BANK_DEPTH, CONTROL_BANK_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_BANK_DEPTH / 2.0, 0.0)),
        material=trim_grey,
        name="bank_body",
    )

    model.articulation(
        "head_to_control_bank",
        ArticulationType.FIXED,
        parent=head,
        child=control_bank,
        origin=Origin(xyz=(0.0, HEAD_DEPTH / 2.0, 0.098)),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Box((0.030, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=reverse_red,
        name="button_cap",
    )
    model.articulation(
        "control_bank_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=control_bank,
        child=reverse_button,
        origin=Origin(xyz=(-0.040, CONTROL_BANK_DEPTH, -0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.05, lower=0.0, upper=0.0012),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.030, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=start_green,
        name="button_cap",
    )
    model.articulation(
        "control_bank_to_start_button",
        ArticulationType.PRISMATIC,
        parent=control_bank,
        child=start_button,
        origin=Origin(xyz=(0.008, CONTROL_BANK_DEPTH, -0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.05, lower=0.0, upper=0.0012),
    )

    dial = model.part("jam_clear_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.022,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.050, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "office_shredder_jam_clear_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_body",
    )
    model.articulation(
        "control_bank_to_jam_clear_dial",
        ArticulationType.CONTINUOUS,
        parent=control_bank,
        child=dial,
        origin=Origin(xyz=(0.054, CONTROL_BANK_DEPTH, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    _add_drum(model, head, name="drum_front", y=0.022, axis=(1.0, 0.0, 0.0), metal=steel)
    _add_drum(model, head, name="drum_rear", y=-0.022, axis=(-1.0, 0.0, 0.0), metal=steel)

    caster_positions = (
        (-0.124, 0.098),
        (0.124, 0.098),
        (-0.124, -0.098),
        (0.124, -0.098),
    )
    for index, (x, y) in enumerate(caster_positions):
        _add_caster(
            model,
            cabinet,
            index=index,
            x=x,
            y=y,
            metal=steel,
            tire=wheel_black,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    head = object_model.get_part("head")
    bin_part = object_model.get_part("bin")
    reverse_button = object_model.get_part("reverse_button")
    start_button = object_model.get_part("start_button")
    wheel_0 = object_model.get_part("caster_wheel_0")

    bin_slide = object_model.get_articulation("cabinet_to_bin")
    reverse_joint = object_model.get_articulation("control_bank_to_reverse_button")
    start_joint = object_model.get_articulation("control_bank_to_start_button")

    ctx.expect_within(
        bin_part,
        cabinet,
        axes="x",
        margin=0.004,
        name="bin stays centered between the cabinet walls",
    )

    bin_rest = ctx.part_world_position(bin_part)
    reverse_rest = ctx.part_world_position(reverse_button)
    start_rest = ctx.part_world_position(start_button)

    if bin_slide.motion_limits is not None and bin_slide.motion_limits.upper is not None:
        with ctx.pose({bin_slide: bin_slide.motion_limits.upper}):
            ctx.expect_overlap(
                bin_part,
                cabinet,
                axes="y",
                min_overlap=0.070,
                name="extended bin keeps retained insertion",
            )
            bin_extended = ctx.part_world_position(bin_part)
        ctx.check(
            "bin extends forward",
            bin_rest is not None
            and bin_extended is not None
            and bin_extended[1] > bin_rest[1] + 0.140,
            details=f"rest={bin_rest}, extended={bin_extended}",
        )

    pressed_positions_ok = False
    if (
        reverse_joint.motion_limits is not None
        and reverse_joint.motion_limits.upper is not None
        and start_joint.motion_limits is not None
        and start_joint.motion_limits.upper is not None
    ):
        with ctx.pose(
            {
                reverse_joint: reverse_joint.motion_limits.upper,
                start_joint: start_joint.motion_limits.upper,
            }
        ):
            reverse_pressed = ctx.part_world_position(reverse_button)
            start_pressed = ctx.part_world_position(start_button)
        pressed_positions_ok = (
            reverse_rest is not None
            and start_rest is not None
            and reverse_pressed is not None
            and start_pressed is not None
            and reverse_pressed[1] < reverse_rest[1] - 0.0010
            and start_pressed[1] < start_rest[1] - 0.0010
        )
        ctx.check(
            "buttons press inward",
            pressed_positions_ok,
            details=(
                f"reverse_rest={reverse_rest}, reverse_pressed={reverse_pressed}, "
                f"start_rest={start_rest}, start_pressed={start_pressed}"
            ),
        )

    wheel_aabb = ctx.part_world_aabb(wheel_0)
    head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "caster wheel sits on floor",
        wheel_aabb is not None and abs(float(wheel_aabb[0][2])) <= 0.002,
        details=f"wheel_aabb={wheel_aabb}",
    )
    ctx.check(
        "shredder has floor-standing office scale",
        wheel_aabb is not None
        and head_aabb is not None
        and 0.64 <= float(head_aabb[1][2] - wheel_aabb[0][2]) <= 0.75,
        details=f"wheel_aabb={wheel_aabb}, head_aabb={head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
