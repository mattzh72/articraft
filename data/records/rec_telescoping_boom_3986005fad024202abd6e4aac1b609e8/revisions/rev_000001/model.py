from __future__ import annotations

import math

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
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


OUTER_LENGTH = 0.88
STAGE_0_LENGTH = 1.15
STAGE_0_INSERT = 0.62
STAGE_0_FRONT = STAGE_0_LENGTH - STAGE_0_INSERT
STAGE_0_TRAVEL = 0.38

STAGE_1_LENGTH = 1.02
STAGE_1_INSERT = 0.52
STAGE_1_FRONT = STAGE_1_LENGTH - STAGE_1_INSERT
STAGE_1_TRAVEL = 0.34

STAGE_2_LENGTH = 0.82
STAGE_2_INSERT = 0.40
STAGE_2_FRONT = STAGE_2_LENGTH - STAGE_2_INSERT
STAGE_2_TRAVEL = 0.26


def _hollow_rect_tube_mesh(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    x_center: float,
    name: str,
):
    """Open ended rounded rectangular tube, authored along local +X."""
    inner_y = outer_y - 2.0 * wall
    inner_z = outer_z - 2.0 * wall
    outer = rounded_rect_profile(outer_z, outer_y, min(outer_y, outer_z) * 0.16, corner_segments=5)
    inner = rounded_rect_profile(inner_z, inner_y, min(inner_y, inner_z) * 0.16, corner_segments=5)
    geom = ExtrudeWithHolesGeometry(outer, [inner[::-1]], length, cap=True, center=True)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(x_center, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _hollow_rect_ring_mesh(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    x_center: float,
    name: str,
):
    """Short open collar ring around a telescoping tube."""
    outer = rounded_rect_profile(outer_z, outer_y, min(outer_y, outer_z) * 0.14, corner_segments=5)
    inner = rounded_rect_profile(inner_z, inner_y, min(inner_y, inner_z) * 0.14, corner_segments=5)
    geom = ExtrudeWithHolesGeometry(outer, [inner[::-1]], length, cap=True, center=True)
    geom.rotate_y(math.pi / 2.0)
    geom.translate(x_center, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_boom")

    base_blue = model.material("base_blue", rgba=(0.08, 0.14, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    tube_steel = model.material("brushed_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    mid_steel = model.material("mid_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    light_steel = model.material("light_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    black = model.material("black_hardware", rgba=(0.03, 0.035, 0.04, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.96, 0.72, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    root_body = model.part("root_body")
    root_body.visual(
        Box((0.95, 0.65, 0.070)),
        origin=Origin(xyz=(-0.20, 0.0, 0.035)),
        material=base_blue,
        name="ground_plate",
    )
    root_body.visual(
        Box((0.46, 0.50, 0.200)),
        origin=Origin(xyz=(-0.38, 0.0, 0.170)),
        material=base_blue,
        name="counterweight_body",
    )
    root_body.visual(
        Box((0.24, 0.30, 0.280)),
        origin=Origin(xyz=(-0.42, 0.0, 0.240)),
        material=base_blue,
        name="pedestal_block",
    )
    root_body.visual(
        Box((0.22, 0.24, 0.040)),
        origin=Origin(xyz=(-0.39, 0.0, 0.400)),
        material=dark_steel,
        name="saddle_pad",
    )
    root_body.visual(
        Box((0.22, 0.035, 0.240)),
        origin=Origin(xyz=(-0.39, 0.125, 0.500)),
        material=dark_steel,
        name="side_cheek_0",
    )
    root_body.visual(
        Box((0.22, 0.035, 0.240)),
        origin=Origin(xyz=(-0.39, -0.125, 0.500)),
        material=dark_steel,
        name="side_cheek_1",
    )
    for index, x in enumerate((-0.56, 0.16)):
        root_body.visual(
            Box((0.22, 0.11, 0.030)),
            origin=Origin(xyz=(x, 0.235, 0.015)),
            material=rubber,
            name=f"ground_foot_{index}_0",
        )
        root_body.visual(
            Box((0.22, 0.11, 0.030)),
            origin=Origin(xyz=(x, -0.235, 0.015)),
            material=rubber,
            name=f"ground_foot_{index}_1",
        )
    for index, (x, y) in enumerate(((-0.58, -0.23), (-0.58, 0.23), (0.16, -0.23), (0.16, 0.23))):
        root_body.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.076)),
            material=black,
            name=f"hold_down_bolt_{index}",
        )
    root_body.visual(
        Box((0.18, 0.012, 0.024)),
        origin=Origin(xyz=(-0.48, -0.256, 0.220)),
        material=safety_yellow,
        name="warning_stripe_0",
    )
    root_body.visual(
        Box((0.18, 0.012, 0.024)),
        origin=Origin(xyz=(-0.28, -0.256, 0.220)),
        material=safety_yellow,
        name="warning_stripe_1",
    )

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _hollow_rect_tube_mesh(
            length=OUTER_LENGTH,
            outer_y=0.200,
            outer_z=0.160,
            wall=0.018,
            x_center=OUTER_LENGTH / 2.0,
            name="outer_tube_body",
        ),
        material=tube_steel,
        name="tube_shell",
    )
    outer_tube.visual(
        _hollow_rect_ring_mesh(
            length=0.075,
            outer_y=0.230,
            outer_z=0.190,
            inner_y=0.160,
            inner_z=0.120,
            x_center=OUTER_LENGTH - 0.030,
            name="outer_front_collar",
        ),
        material=dark_steel,
        name="front_collar",
    )
    outer_tube.visual(
        Cylinder(radius=0.011, length=0.070),
        origin=Origin(xyz=(OUTER_LENGTH - 0.045, 0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_screw",
    )
    outer_tube.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(OUTER_LENGTH - 0.045, 0.178, 0.0)),
        material=black,
        name="clamp_knob",
    )

    stage_0 = model.part("stage_0")
    stage_0.visual(
        _hollow_rect_tube_mesh(
            length=STAGE_0_LENGTH,
            outer_y=0.145,
            outer_z=0.105,
            wall=0.014,
            x_center=STAGE_0_FRONT - STAGE_0_LENGTH / 2.0,
            name="stage_0_body",
        ),
        material=mid_steel,
        name="tube_shell",
    )
    stage_0.visual(
        _hollow_rect_ring_mesh(
            length=0.060,
            outer_y=0.165,
            outer_z=0.125,
            inner_y=0.112,
            inner_z=0.082,
            x_center=STAGE_0_FRONT - 0.025,
            name="stage_0_front_collar",
        ),
        material=dark_steel,
        name="front_collar",
    )
    stage_0.visual(
        Box((0.040, 0.155, 0.010)),
        origin=Origin(xyz=(STAGE_0_FRONT - 0.115, 0.0, 0.0575)),
        material=black,
        name="travel_stop",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        _hollow_rect_tube_mesh(
            length=STAGE_1_LENGTH,
            outer_y=0.100,
            outer_z=0.070,
            wall=0.011,
            x_center=STAGE_1_FRONT - STAGE_1_LENGTH / 2.0,
            name="stage_1_body",
        ),
        material=light_steel,
        name="tube_shell",
    )
    stage_1.visual(
        _hollow_rect_ring_mesh(
            length=0.050,
            outer_y=0.118,
            outer_z=0.086,
            inner_y=0.073,
            inner_z=0.048,
            x_center=STAGE_1_FRONT - 0.020,
            name="stage_1_front_collar",
        ),
        material=dark_steel,
        name="front_collar",
    )
    stage_1.visual(
        Box((0.034, 0.108, 0.008)),
        origin=Origin(xyz=(STAGE_1_FRONT - 0.095, 0.0, 0.039)),
        material=black,
        name="travel_stop",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        _hollow_rect_tube_mesh(
            length=STAGE_2_LENGTH,
            outer_y=0.065,
            outer_z=0.040,
            wall=0.006,
            x_center=STAGE_2_FRONT - STAGE_2_LENGTH / 2.0,
            name="stage_2_body",
        ),
        material=light_steel,
        name="tube_shell",
    )
    stage_2.visual(
        Box((0.035, 0.074, 0.052)),
        origin=Origin(xyz=(STAGE_2_FRONT + 0.008, 0.0, 0.0)),
        material=dark_steel,
        name="end_cap",
    )
    stage_2.visual(
        Box((0.070, 0.018, 0.095)),
        origin=Origin(xyz=(STAGE_2_FRONT + 0.060, 0.0, 0.0)),
        material=dark_steel,
        name="tip_lug",
    )
    stage_2.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(STAGE_2_FRONT + 0.062, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="tip_pin",
    )

    model.articulation(
        "body_to_outer",
        ArticulationType.FIXED,
        parent=root_body,
        child=outer_tube,
        origin=Origin(xyz=(-0.45, 0.0, 0.500)),
    )
    model.articulation(
        "outer_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=stage_0,
        origin=Origin(xyz=(OUTER_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STAGE_0_TRAVEL, effort=180.0, velocity=0.22),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(STAGE_0_FRONT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STAGE_1_TRAVEL, effort=140.0, velocity=0.22),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_1_FRONT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STAGE_2_TRAVEL, effort=110.0, velocity=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_tube = object_model.get_part("outer_tube")
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    outer_to_stage_0 = object_model.get_articulation("outer_to_stage_0")
    stage_0_to_stage_1 = object_model.get_articulation("stage_0_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")

    ctx.allow_overlap(
        outer_tube,
        stage_0,
        elem_a="tube_shell",
        elem_b="tube_shell",
        reason="The stage 0 tube is intentionally nested inside the outer sleeve with retained insertion.",
    )
    ctx.allow_overlap(
        outer_tube,
        stage_0,
        elem_a="front_collar",
        elem_b="tube_shell",
        reason="The outer locking collar intentionally surrounds the sliding stage 0 tube at the sleeve mouth.",
    )
    ctx.allow_overlap(
        stage_0,
        stage_1,
        elem_a="tube_shell",
        elem_b="tube_shell",
        reason="The stage 1 tube is intentionally nested inside stage 0 with retained insertion.",
    )
    ctx.allow_overlap(
        stage_0,
        stage_1,
        elem_a="front_collar",
        elem_b="tube_shell",
        reason="The stage 0 locking collar intentionally surrounds the sliding stage 1 tube at the sleeve mouth.",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="tube_shell",
        elem_b="tube_shell",
        reason="The stage 2 tube is intentionally nested inside stage 1 with retained insertion.",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="front_collar",
        elem_b="tube_shell",
        reason="The stage 1 locking collar intentionally surrounds the sliding stage 2 tube at the sleeve mouth.",
    )

    ctx.expect_within(
        stage_0,
        outer_tube,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 0 centered in outer tube",
    )
    ctx.expect_within(
        stage_1,
        stage_0,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 1 centered in stage 0",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 2 centered in stage 1",
    )
    ctx.expect_overlap(
        stage_0,
        outer_tube,
        axes="x",
        min_overlap=0.55,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 0 has collapsed insertion",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="x",
        min_overlap=0.45,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 1 has collapsed insertion",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        min_overlap=0.34,
        elem_a="tube_shell",
        elem_b="tube_shell",
        name="stage 2 has collapsed insertion",
    )
    ctx.expect_within(
        stage_0,
        outer_tube,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 0 passes through outer collar",
    )
    ctx.expect_within(
        stage_1,
        stage_0,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 1 passes through stage 0 collar",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="yz",
        margin=0.002,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 2 passes through stage 1 collar",
    )
    ctx.expect_overlap(
        stage_0,
        outer_tube,
        axes="x",
        min_overlap=0.050,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 0 remains through outer collar",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="x",
        min_overlap=0.040,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 1 remains through stage 0 collar",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        min_overlap=0.030,
        elem_a="tube_shell",
        elem_b="front_collar",
        name="stage 2 remains through stage 1 collar",
    )

    rest_tip = ctx.part_world_position(stage_2)
    with ctx.pose(
        {
            outer_to_stage_0: STAGE_0_TRAVEL,
            stage_0_to_stage_1: STAGE_1_TRAVEL,
            stage_1_to_stage_2: STAGE_2_TRAVEL,
        }
    ):
        ctx.expect_within(
            stage_0,
            outer_tube,
            axes="yz",
            margin=0.002,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 0 remains centered when extended",
        )
        ctx.expect_within(
            stage_1,
            stage_0,
            axes="yz",
            margin=0.002,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 1 remains centered when extended",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.002,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 2 remains centered when extended",
        )
        ctx.expect_overlap(
            stage_0,
            outer_tube,
            axes="x",
            min_overlap=0.18,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 0 retains insertion when extended",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="x",
            min_overlap=0.14,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 1 retains insertion when extended",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            min_overlap=0.10,
            elem_a="tube_shell",
            elem_b="tube_shell",
            name="stage 2 retains insertion when extended",
        )
        extended_tip = ctx.part_world_position(stage_2)

    ctx.check(
        "serial tubes extend along +X",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.80,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
