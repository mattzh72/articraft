from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.68
CABINET_DEPTH = 0.42
CABINET_HEIGHT = 0.94

SIDE_WALL = 0.018
BACK_WALL = 0.018
TOP_WALL = 0.018
BOTTOM_WALL = 0.018

BOTTOM_RAIL = 0.055
TOP_RAIL = 0.085
FRONT_RAIL_DEPTH = 0.024

DRAWER_COUNT = 7
DRAWER_GAP = 0.006
DRAWER_FACE_HEIGHT = (
    CABINET_HEIGHT - BOTTOM_RAIL - TOP_RAIL - DRAWER_GAP * (DRAWER_COUNT - 1)
) / DRAWER_COUNT
DRAWER_FACE_WIDTH = 0.624
DRAWER_FACE_THICKNESS = 0.014

DRAWER_BODY_WIDTH = 0.588
DRAWER_BODY_DEPTH = 0.350
DRAWER_BODY_HEIGHT = 0.082
DRAWER_SHEET = 0.010
DRAWER_BOTTOM = 0.008

DRAWER_RUNNER_WIDTH = 0.008
DRAWER_RUNNER_HEIGHT = 0.012
DRAWER_RUNNER_LENGTH = 0.260
DRAWER_RUNNER_Z = -0.014

CARCASS_RUNNER_PROTRUSION = 0.020
CARCASS_RUNNER_HEIGHT = 0.010
CARCASS_RUNNER_LENGTH = 0.300
CARCASS_RUNNER_Y_CENTER = CABINET_DEPTH / 2.0 - 0.045 - CARCASS_RUNNER_LENGTH / 2.0

DRAWER_TRAVEL = 0.235

FLAP_WIDTH = 0.076
FLAP_HEIGHT = 0.046
FLAP_THICKNESS = 0.004
FLAP_BARREL_RADIUS = 0.004
FLAP_OPEN_ANGLE = 1.25
FLAP_CENTER_Z = CABINET_HEIGHT - TOP_RAIL / 2.0
FLAP_HINGE_Z = FLAP_CENTER_Z + FLAP_HEIGHT / 2.0


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _drawer_center_z(index: int) -> float:
    return BOTTOM_RAIL + DRAWER_FACE_HEIGHT / 2.0 + index * (DRAWER_FACE_HEIGHT + DRAWER_GAP)


def _make_drawer_shape() -> cq.Workplane:
    face = _box(
        (DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT),
        (0.0, -DRAWER_FACE_THICKNESS / 2.0, 0.0),
    )
    body_center_y = -(DRAWER_FACE_THICKNESS + DRAWER_BODY_DEPTH / 2.0)

    bottom = _box(
        (DRAWER_BODY_WIDTH, DRAWER_BODY_DEPTH, DRAWER_BOTTOM),
        (
            0.0,
            body_center_y,
            -DRAWER_BODY_HEIGHT / 2.0 + DRAWER_BOTTOM / 2.0,
        ),
    )
    left_side = _box(
        (DRAWER_SHEET, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT),
        (
            -DRAWER_BODY_WIDTH / 2.0 + DRAWER_SHEET / 2.0,
            body_center_y,
            0.0,
        ),
    )
    right_side = _box(
        (DRAWER_SHEET, DRAWER_BODY_DEPTH, DRAWER_BODY_HEIGHT),
        (
            DRAWER_BODY_WIDTH / 2.0 - DRAWER_SHEET / 2.0,
            body_center_y,
            0.0,
        ),
    )
    back = _box(
        (DRAWER_BODY_WIDTH, DRAWER_SHEET, DRAWER_BODY_HEIGHT),
        (
            0.0,
            -(DRAWER_FACE_THICKNESS + DRAWER_BODY_DEPTH - DRAWER_SHEET / 2.0),
            0.0,
        ),
    )

    left_runner = _box(
        (DRAWER_RUNNER_WIDTH, DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_HEIGHT),
        (
            -DRAWER_BODY_WIDTH / 2.0 - DRAWER_RUNNER_WIDTH / 2.0,
            -(DRAWER_FACE_THICKNESS + 0.028 + DRAWER_RUNNER_LENGTH / 2.0),
            DRAWER_RUNNER_Z,
        ),
    )
    right_runner = _box(
        (DRAWER_RUNNER_WIDTH, DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_HEIGHT),
        (
            DRAWER_BODY_WIDTH / 2.0 + DRAWER_RUNNER_WIDTH / 2.0,
            -(DRAWER_FACE_THICKNESS + 0.028 + DRAWER_RUNNER_LENGTH / 2.0),
            DRAWER_RUNNER_Z,
        ),
    )

    drawer = face.union(bottom).union(left_side).union(right_side).union(back)
    drawer = drawer.union(left_runner).union(right_runner)

    grip_cut = _box(
        (0.220, DRAWER_FACE_THICKNESS * 0.86, 0.028),
        (0.0, -DRAWER_FACE_THICKNESS * 0.43, DRAWER_FACE_HEIGHT * 0.20),
    )
    drawer = drawer.cut(grip_cut)

    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machinist_drawer_cabinet")

    carcass_finish = model.material("carcass_finish", rgba=(0.26, 0.29, 0.26, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.43, 0.46, 0.42, 1.0))
    flap_finish = model.material("flap_finish", rgba=(0.12, 0.13, 0.14, 1.0))

    carcass = model.part("carcass")
    inner_width = CABINET_WIDTH - 2.0 * SIDE_WALL
    inner_height = CABINET_HEIGHT - TOP_WALL - BOTTOM_WALL

    carcass.visual(
        Box((SIDE_WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + SIDE_WALL / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=carcass_finish,
        name="side_0",
    )
    carcass.visual(
        Box((SIDE_WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - SIDE_WALL / 2.0, 0.0, CABINET_HEIGHT / 2.0)),
        material=carcass_finish,
        name="side_1",
    )
    carcass.visual(
        Box((inner_width, CABINET_DEPTH, TOP_WALL)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - TOP_WALL / 2.0)),
        material=carcass_finish,
        name="top",
    )
    carcass.visual(
        Box((inner_width, CABINET_DEPTH, BOTTOM_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_WALL / 2.0)),
        material=carcass_finish,
        name="bottom",
    )
    carcass.visual(
        Box((inner_width, BACK_WALL, inner_height)),
        origin=Origin(
            xyz=(0.0, -CABINET_DEPTH / 2.0 + BACK_WALL / 2.0, CABINET_HEIGHT / 2.0)
        ),
        material=carcass_finish,
        name="back",
    )
    carcass.visual(
        Box((inner_width, FRONT_RAIL_DEPTH, TOP_RAIL)),
        origin=Origin(
            xyz=(0.0, CABINET_DEPTH / 2.0 - FRONT_RAIL_DEPTH / 2.0, CABINET_HEIGHT - TOP_RAIL / 2.0)
        ),
        material=carcass_finish,
        name="top_rail",
    )
    carcass.visual(
        Box((inner_width, FRONT_RAIL_DEPTH, BOTTOM_RAIL)),
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0 - FRONT_RAIL_DEPTH / 2.0, BOTTOM_RAIL / 2.0)),
        material=carcass_finish,
        name="bottom_rail",
    )
    for drawer_index in range(DRAWER_COUNT):
        runner_z = _drawer_center_z(drawer_index) + DRAWER_RUNNER_Z
        carcass.visual(
            Box((CARCASS_RUNNER_PROTRUSION, CARCASS_RUNNER_LENGTH, CARCASS_RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    -inner_width / 2.0 + CARCASS_RUNNER_PROTRUSION / 2.0,
                    CARCASS_RUNNER_Y_CENTER,
                    runner_z,
                )
            ),
            material=carcass_finish,
            name=f"runner_{drawer_index}_0",
        )
        carcass.visual(
            Box((CARCASS_RUNNER_PROTRUSION, CARCASS_RUNNER_LENGTH, CARCASS_RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    inner_width / 2.0 - CARCASS_RUNNER_PROTRUSION / 2.0,
                    CARCASS_RUNNER_Y_CENTER,
                    runner_z,
                )
            ),
            material=carcass_finish,
            name=f"runner_{drawer_index}_1",
        )

    for drawer_index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{drawer_index}")
        drawer.visual(
            mesh_from_cadquery(_make_drawer_shape(), f"drawer_{drawer_index}"),
            material=drawer_finish,
            name="drawer",
        )
        model.articulation(
            f"carcass_to_drawer_{drawer_index}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0, _drawer_center_z(drawer_index))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=160.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((FLAP_WIDTH, FLAP_THICKNESS, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, FLAP_THICKNESS / 2.0, -FLAP_HEIGHT / 2.0)),
        material=flap_finish,
        name="plate",
    )
    lock_flap.visual(
        Cylinder(radius=FLAP_BARREL_RADIUS, length=FLAP_WIDTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flap_finish,
        name="barrel",
    )
    model.articulation(
        "carcass_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lock_flap,
        origin=Origin(xyz=(0.0, CABINET_DEPTH / 2.0 + FLAP_BARREL_RADIUS, FLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=FLAP_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")

    for drawer_index in range(DRAWER_COUNT):
        ctx.allow_overlap(
            carcass,
            f"drawer_{drawer_index}",
            elem_a=f"runner_{drawer_index}_0",
            elem_b="drawer",
            reason="The straight runner box is a simplified proxy for the drawer-side slide bead nested along the carcass wall.",
        )
        ctx.allow_overlap(
            carcass,
            f"drawer_{drawer_index}",
            elem_a=f"runner_{drawer_index}_1",
            elem_b="drawer",
            reason="The straight runner box is a simplified proxy for the drawer-side slide bead nested along the carcass wall.",
        )

    for drawer_index in range(DRAWER_COUNT):
        drawer = object_model.get_part(f"drawer_{drawer_index}")
        slide = object_model.get_articulation(f"carcass_to_drawer_{drawer_index}")
        closed_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer.name} closes at the cabinet front",
            closed_pos is not None
            and abs(closed_pos[1] - CABINET_DEPTH / 2.0) <= 0.001
            and abs(closed_pos[2] - _drawer_center_z(drawer_index)) <= 0.001,
            details=f"closed_pos={closed_pos}",
        )
        ctx.expect_within(
            drawer,
            carcass,
            axes="x",
            margin=0.05,
            name=f"{drawer.name} stays between cabinet sides",
        )

        with ctx.pose({slide: DRAWER_TRAVEL}):
            open_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"{drawer.name} extends outward",
                closed_pos is not None
                and open_pos is not None
                and open_pos[1] > closed_pos[1] + 0.20,
                details=f"closed={closed_pos}, open={open_pos}",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                min_overlap=0.10,
                name=f"{drawer.name} keeps retained insertion at full extension",
            )

    lock_flap = object_model.get_part("lock_flap")
    flap_hinge = object_model.get_articulation("carcass_to_lock_flap")

    ctx.expect_gap(
        lock_flap,
        carcass,
        axis="y",
        min_gap=0.0,
        max_gap=0.008,
        name="lock flap sits just proud of the top rail",
    )

    closed_flap_aabb = ctx.part_world_aabb(lock_flap)
    with ctx.pose({flap_hinge: FLAP_OPEN_ANGLE}):
        open_flap_aabb = ctx.part_world_aabb(lock_flap)
        ctx.check(
            "lock flap swings outward from the top rail",
            closed_flap_aabb is not None
            and open_flap_aabb is not None
            and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.012,
            details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
