from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_START = 0.005
BARREL_LENGTH = 0.120
BARREL_END = BARREL_START + BARREL_LENGTH
BARREL_OUTER_RADIUS = 0.014
BARREL_INNER_RADIUS = 0.010
PLUNGER_TRAVEL = 0.063


def _axis_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Place a local-Z SDK cylinder along the syringe's world +X axis."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _barrel_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [(BARREL_OUTER_RADIUS, BARREL_START), (BARREL_OUTER_RADIUS, BARREL_END)],
        [(BARREL_INNER_RADIUS, BARREL_START), (BARREL_INNER_RADIUS, BARREL_END)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "barrel_shell")


def _rear_stop_ring_mesh():
    ring = LatheGeometry.from_shell_profiles(
        [(0.018, -0.006), (0.018, 0.006)],
        [(0.006, -0.006), (0.006, 0.006)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, "rear_stop_ring")


def _front_nozzle_mesh():
    nozzle = LatheGeometry(
        [
            (0.000, BARREL_END - 0.003),
            (BARREL_OUTER_RADIUS, BARREL_END - 0.003),
            (0.009, BARREL_END + 0.010),
            (0.004, BARREL_END + 0.026),
            (0.003, BARREL_END + 0.043),
            (0.000, BARREL_END + 0.043),
        ],
        segments=72,
    )
    nozzle.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(nozzle, "nozzle_tip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.72, 0.90, 1.00, 0.38))
    frosted_plastic = model.material("frosted_plastic", rgba=(0.88, 0.96, 1.00, 0.62))
    black_print = model.material("black_print", rgba=(0.02, 0.025, 0.03, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    plunger_plastic = model.material("white_plastic", rgba=(0.96, 0.97, 0.94, 1.0))

    barrel = model.part("barrel")
    barrel.visual(_barrel_shell_mesh(), material=clear_plastic, name="barrel_shell")
    barrel.visual(_rear_stop_ring_mesh(), material=frosted_plastic, name="rear_stop_ring")
    barrel.visual(_front_nozzle_mesh(), material=clear_plastic, name="nozzle_tip")

    # Finger flanges are part of the fixed rear end frame and make the travel stop obvious.
    for index, y in enumerate((-0.034, 0.034)):
        barrel.visual(
            Box((0.008, 0.042, 0.012)),
            origin=Origin(xyz=(-0.001, y, 0.0)),
            material=frosted_plastic,
            name=f"finger_flange_{index}",
        )

    # Printed dose graduations, slightly proud of the barrel wall.
    for index, x in enumerate((0.027, 0.045, 0.063, 0.081, 0.099, 0.117)):
        tick_width = 0.014 if index % 2 == 0 else 0.009
        barrel.visual(
            Box((0.0012, tick_width, 0.0008)),
            origin=Origin(xyz=(x, 0.0, BARREL_OUTER_RADIUS + 0.0002)),
            material=black_print,
            name=f"graduation_{index}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0026, length=0.099),
        origin=_axis_origin(-0.0255),
        material=plunger_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=_axis_origin(-0.072),
        material=plunger_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0074, length=0.018),
        origin=_axis_origin(0.028),
        material=rubber,
        name="plunger_core",
    )
    plunger.visual(
        Cylinder(radius=BARREL_INNER_RADIUS, length=0.004),
        origin=_axis_origin(0.0215),
        material=rubber,
        name="seal_ring_0",
    )
    plunger.visual(
        Cylinder(radius=BARREL_INNER_RADIUS, length=0.004),
        origin=_axis_origin(0.0345),
        material=rubber,
        name="seal_ring_1",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=PLUNGER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="seal_ring_0",
        outer_elem="barrel_shell",
        margin=0.0,
        name="rear seal is centered in the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="seal_ring_0",
        elem_b="barrel_shell",
        min_overlap=0.003,
        name="plunger seal starts inside barrel",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="seal_ring_1",
            outer_elem="barrel_shell",
            margin=0.0,
            name="forward seal remains centered in the barrel bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="seal_ring_1",
            elem_b="barrel_shell",
            min_overlap=0.003,
            name="plunger remains captured at full stroke",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="rear_stop_ring",
            negative_elem="thumb_pad",
            max_gap=0.0015,
            max_penetration=0.00001,
            name="thumb pad meets the fixed rear stop at full stroke",
        )
        pressed_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates forward along barrel",
        rest_position is not None
        and pressed_position is not None
        and pressed_position[0] > rest_position[0] + 0.055,
        details=f"rest={rest_position}, pressed={pressed_position}",
    )

    return ctx.report()


object_model = build_object_model()
