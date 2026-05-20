from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

FIXTURE_LENGTH = 1.22
FIXTURE_WIDTH = 0.26
FIXTURE_HEIGHT = 0.105

TOP_THICKNESS = 0.003
WALL_THICKNESS = 0.0025
LIP_WIDTH = 0.024
LIP_THICKNESS = 0.002

DIFFUSER_LENGTH = 1.14
DIFFUSER_WIDTH = 0.18
DIFFUSER_THICKNESS = 0.014

HINGE_Y = -(DIFFUSER_WIDTH / 2.0) - 0.018
HINGE_Z = -0.046

GUARD_LENGTH = 1.18
GUARD_SPAN = 0.216
GUARD_DROP = 0.022
WIRE_RADIUS = 0.0035

CLIP_X = 0.47
CLIP_Y = (FIXTURE_WIDTH / 2.0) - 0.012
CLIP_Z = -0.043


def _add_segment(part, *, start, end, radius, mesh_name, visual_name, material) -> None:
    part.visual(
        mesh_from_geometry(
            wire_from_points([start, end], radius=radius, cap_ends=False),
            mesh_name,
        ),
        material=material,
        name=visual_name,
    )


def _build_guard(part, material: str) -> None:
    part.visual(
        Cylinder(radius=WIRE_RADIUS, length=GUARD_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name="guard_hinge",
    )
    part.visual(
        Cylinder(radius=WIRE_RADIUS, length=GUARD_LENGTH),
        origin=Origin(
            xyz=(0.0, GUARD_SPAN, -GUARD_DROP),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=material,
        name="guard_outer",
    )
    part.visual(
        Cylinder(radius=WIRE_RADIUS, length=GUARD_LENGTH * 0.94),
        origin=Origin(
            xyz=(0.0, 0.092, -0.014),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=material,
        name="guard_mid",
    )

    side_x = GUARD_LENGTH / 2.0
    _add_segment(
        part,
        start=(-side_x, 0.0, 0.0),
        end=(-side_x, GUARD_SPAN, -GUARD_DROP),
        radius=WIRE_RADIUS,
        mesh_name="guard_side_0",
        visual_name="guard_side_0",
        material=material,
    )
    _add_segment(
        part,
        start=(side_x, 0.0, 0.0),
        end=(side_x, GUARD_SPAN, -GUARD_DROP),
        radius=WIRE_RADIUS,
        mesh_name="guard_side_1",
        visual_name="guard_side_1",
        material=material,
    )

    for index, x_pos in enumerate((-0.36, 0.0, 0.36)):
        _add_segment(
            part,
            start=(x_pos, 0.092, -0.014),
            end=(x_pos, GUARD_SPAN, -GUARD_DROP),
            radius=WIRE_RADIUS * 0.9,
            mesh_name=f"guard_bar_{index}",
            visual_name=f"guard_bar_{index}",
            material=material,
        )


def _build_body(part, body_material: str, diffuser_material: str) -> None:
    part.visual(
        Box((FIXTURE_LENGTH, FIXTURE_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, (FIXTURE_HEIGHT / 2.0) - (TOP_THICKNESS / 2.0))),
        material=body_material,
        name="housing_top",
    )

    wall_height = FIXTURE_HEIGHT - TOP_THICKNESS
    side_y = (FIXTURE_WIDTH / 2.0) - (WALL_THICKNESS / 2.0)
    end_x = (FIXTURE_LENGTH / 2.0) - (WALL_THICKNESS / 2.0)

    for index, y_pos in enumerate((-side_y, side_y)):
        part.visual(
            Box((FIXTURE_LENGTH, WALL_THICKNESS, wall_height)),
            origin=Origin(xyz=(0.0, y_pos, 0.0)),
            material=body_material,
            name=f"body_side_{index}",
        )

    for index, x_pos in enumerate((-end_x, end_x)):
        part.visual(
            Box((WALL_THICKNESS, FIXTURE_WIDTH - (2.0 * WALL_THICKNESS), wall_height)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=body_material,
            name=f"body_end_{index}",
        )

    lip_z = -0.040
    lip_side_y = 0.110
    lip_end_x = 0.588

    for index, y_pos in enumerate((-lip_side_y, lip_side_y)):
        part.visual(
            Box((1.16, 0.040, LIP_THICKNESS)),
            origin=Origin(xyz=(0.0, y_pos, lip_z)),
            material=body_material,
            name=f"lip_side_{index}",
        )

    for index, x_pos in enumerate((-lip_end_x, lip_end_x)):
        part.visual(
            Box((0.036, 0.200, LIP_THICKNESS)),
            origin=Origin(xyz=(x_pos, 0.0, lip_z)),
            material=body_material,
            name=f"lip_end_{index}",
        )

    part.visual(
        Box((DIFFUSER_LENGTH, DIFFUSER_WIDTH, DIFFUSER_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, lip_z - (DIFFUSER_THICKNESS / 2.0))),
        material=diffuser_material,
        name="diffuser",
    )

    for index, x_pos in enumerate((-0.47, -0.23, 0.0, 0.23, 0.47)):
        part.visual(
            Box((0.04, 0.018, 0.018)),
            origin=Origin(xyz=(x_pos, HINGE_Y - 0.015, HINGE_Z + 0.007)),
            material=body_material,
            name=f"hinge_tab_{index}",
        )

    for index, x_pos in enumerate((CLIP_X - 0.034, CLIP_X + 0.034)):
        part.visual(
            Box((0.008, 0.008, 0.018)),
            origin=Origin(xyz=(x_pos, CLIP_Y + 0.009, CLIP_Z + 0.001)),
            material=body_material,
            name=f"clip_ear_{index}",
        )

def _build_latch(part, material: str) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.048),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name="latch_pivot",
    )
    part.visual(
        Box((0.048, 0.0045, 0.040)),
        origin=Origin(xyz=(0.0, -0.0045, -0.018)),
        material=material,
        name="latch_arm",
    )
    part.visual(
        Box((0.048, 0.020, 0.0045)),
        origin=Origin(xyz=(0.0, -0.014, -0.018)),
        material=material,
        name="latch_hook",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_ceiling_fixture")

    model.material("body_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("lens", rgba=(0.92, 0.94, 0.95, 1.0))
    model.material("guard_steel", rgba=(0.30, 0.32, 0.34, 1.0))
    model.material("clip_zinc", rgba=(0.70, 0.72, 0.75, 1.0))

    body = model.part("body")
    _build_body(body, "body_steel", "lens")

    guard = model.part("guard")
    _build_guard(guard, "guard_steel")

    latch = model.part("latch")
    _build_latch(latch, "clip_zinc")

    model.articulation(
        "body_to_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.6),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(CLIP_X, CLIP_Y, CLIP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=4.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    guard = object_model.get_part("guard")
    latch = object_model.get_part("latch")
    guard_hinge = object_model.get_articulation("body_to_guard")
    latch_hinge = object_model.get_articulation("body_to_latch")

    ctx.expect_gap(
        body,
        guard,
        axis="z",
        positive_elem="diffuser",
        negative_elem="guard_outer",
        min_gap=0.008,
        max_gap=0.018,
        name="guard sits just below the diffuser",
    )
    ctx.expect_overlap(
        body,
        guard,
        axes="xy",
        elem_a="diffuser",
        min_overlap=0.09,
        name="guard covers the diffuser footprint",
    )
    ctx.expect_gap(
        latch,
        guard,
        axis="z",
        positive_elem="latch_hook",
        negative_elem="guard_outer",
        min_gap=0.001,
        max_gap=0.010,
        name="latch hook rests just above the guard rail",
    )
    ctx.expect_overlap(
        latch,
        guard,
        axes="x",
        elem_a="latch_hook",
        elem_b="guard_outer",
        min_overlap=0.040,
        name="latch spans across the outer guard rail",
    )

    rest_guard_aabb = ctx.part_world_aabb(guard)
    with ctx.pose({guard_hinge: 1.25}):
        open_guard_aabb = ctx.part_world_aabb(guard)
    ctx.check(
        "guard rotates downward",
        rest_guard_aabb is not None
        and open_guard_aabb is not None
        and open_guard_aabb[0][2] < rest_guard_aabb[0][2] - 0.12,
        details=f"rest={rest_guard_aabb}, open={open_guard_aabb}",
    )

    rest_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")
    with ctx.pose({latch_hinge: 1.25}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")
        ctx.expect_gap(
            latch,
            guard,
            axis="z",
            positive_elem="latch_hook",
            negative_elem="guard_outer",
            min_gap=0.018,
            name="opened latch clears the guard rail",
        )
    ctx.check(
        "latch flips upward to release the guard",
        rest_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.020,
        details=f"rest={rest_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
