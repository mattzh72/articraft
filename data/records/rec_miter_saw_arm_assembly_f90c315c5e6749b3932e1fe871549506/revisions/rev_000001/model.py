from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int, *, offset=(0.0, 0.0)):
    ox, oy = offset
    return [
        (ox + radius * math.cos(2.0 * math.pi * i / segments),
         oy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _toothed_blade_geometry():
    teeth = 40
    outer_radius = 0.158
    root_radius = 0.146
    profile = []
    for i in range(teeth * 2):
        angle = 2.0 * math.pi * i / (teeth * 2)
        radius = outer_radius if i % 2 == 0 else root_radius
        profile.append((radius * math.cos(angle), radius * math.sin(angle)))

    return ExtrudeWithHolesGeometry(
        profile,
        [_circle_profile(0.026, 36)],
        0.008,
        center=True,
    )


def _upper_guard_geometry():
    start = math.radians(-28.0)
    end = math.radians(205.0)
    outer_radius = 0.205
    inner_radius = 0.168
    steps = 44
    outer = [
        (
            outer_radius * math.cos(start + (end - start) * i / steps),
            outer_radius * math.sin(start + (end - start) * i / steps),
        )
        for i in range(steps + 1)
    ]
    inner = [
        (
            inner_radius * math.cos(end - (end - start) * i / steps),
            inner_radius * math.sin(end - (end - start) * i / steps),
        )
        for i in range(steps + 1)
    ]
    return ExtrudeGeometry(outer + inner, 0.062, center=True)


def _tube_mesh_x(length: float, outer_radius: float, inner_radius: float):
    tube = (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_compound_miter_saw")

    cast_gray = model.material("cast_gray", rgba=(0.36, 0.38, 0.39, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_chrome = model.material("rail_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.08, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.88, 0.88, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.78, 0.54, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_gray,
        name="base_plate",
    )
    base.visual(
        Box((0.72, 0.045, 0.030)),
        origin=Origin(xyz=(-0.02, 0.245, -0.015)),
        material=dark_gray,
        name="front_foot_0",
    )
    base.visual(
        Box((0.72, 0.045, 0.030)),
        origin=Origin(xyz=(-0.02, -0.245, -0.015)),
        material=dark_gray,
        name="front_foot_1",
    )
    base.visual(
        Box((0.055, 0.235, 0.110)),
        origin=Origin(xyz=(0.085, 0.185, 0.140)),
        material=cast_gray,
        name="fence_0",
    )
    base.visual(
        Box((0.055, 0.235, 0.110)),
        origin=Origin(xyz=(0.085, -0.185, 0.140)),
        material=cast_gray,
        name="fence_1",
    )
    base.visual(
        Box((0.035, 0.500, 0.030)),
        origin=Origin(xyz=(0.125, 0.0, 0.100)),
        material=dark_gray,
        name="fence_foot",
    )
    base.visual(
        Box((0.040, 0.035, 0.045)),
        origin=Origin(xyz=(0.125, 0.245, 0.0625)),
        material=cast_gray,
        name="fence_post_0",
    )
    base.visual(
        Box((0.040, 0.035, 0.045)),
        origin=Origin(xyz=(0.125, -0.245, 0.0625)),
        material=cast_gray,
        name="fence_post_1",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.210, length=0.045),
        origin=Origin(),
        material=cast_gray,
        name="table_disk",
    )
    turntable.visual(
        Box((0.46, 0.120, 0.034)),
        origin=Origin(xyz=(-0.18, 0.0, 0.005)),
        material=cast_gray,
        name="miter_arm",
    )
    turntable.visual(
        Box((0.340, 0.018, 0.004)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0245)),
        material=dark_gray,
        name="blade_slot",
    )
    turntable.visual(
        Box((0.300, 0.120, 0.045)),
        origin=Origin(xyz=(0.255, 0.0, -0.0025)),
        material=cast_gray,
        name="column_foot",
    )
    turntable.visual(
        Box((0.080, 0.160, 0.460)),
        origin=Origin(xyz=(0.390, 0.0, 0.250)),
        material=cast_gray,
        name="rear_column",
    )
    turntable.visual(
        Box((0.100, 0.250, 0.105)),
        origin=Origin(xyz=(0.390, 0.0, 0.500)),
        material=cast_gray,
        name="rear_rail_block",
    )
    turntable.visual(
        Box((0.060, 0.235, 0.085)),
        origin=Origin(xyz=(-0.300, 0.0, 0.500)),
        material=cast_gray,
        name="front_rail_block",
    )
    turntable.visual(
        Cylinder(radius=0.014, length=0.650),
        origin=Origin(xyz=(0.015, -0.070, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_chrome,
        name="rail_0",
    )
    turntable.visual(
        Cylinder(radius=0.014, length=0.650),
        origin=Origin(xyz=(0.015, 0.070, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_chrome,
        name="rail_1",
    )

    carriage = model.part("carriage")
    sleeve_mesh = mesh_from_cadquery(
        _tube_mesh_x(0.170, 0.034, 0.014),
        "carriage_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    carriage.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=dark_gray,
        name="sleeve_0",
    )
    carriage.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=dark_gray,
        name="sleeve_1",
    )
    carriage.visual(
        Box((0.090, 0.250, 0.040)),
        origin=Origin(xyz=(0.175, 0.0, -0.120)),
        material=dark_gray,
        name="sleeve_bridge",
    )
    carriage.visual(
        Box((0.180, 0.030, 0.165)),
        origin=Origin(xyz=(0.045, 0.100, -0.050)),
        material=cast_gray,
        name="yoke_plate_0",
    )
    carriage.visual(
        Box((0.180, 0.030, 0.165)),
        origin=Origin(xyz=(0.045, -0.100, -0.050)),
        material=cast_gray,
        name="yoke_plate_1",
    )
    carriage.visual(
        Box((0.090, 0.250, 0.040)),
        origin=Origin(xyz=(0.175, 0.0, -0.150)),
        material=cast_gray,
        name="yoke_bridge",
    )

    saw_head = model.part("saw_head")
    saw_head.visual(
        Cylinder(radius=0.040, length=0.170),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_chrome,
        name="pitch_trunnion",
    )
    saw_head.visual(
        Box((0.080, 0.050, 0.165)),
        origin=Origin(xyz=(-0.035, -0.060, -0.080)),
        material=yellow,
        name="drop_arm",
    )
    saw_head.visual(
        Box((0.095, 0.050, 0.080)),
        origin=Origin(xyz=(-0.085, -0.055, -0.140)),
        material=yellow,
        name="gear_neck",
    )
    saw_head.visual(
        mesh_from_geometry(_upper_guard_geometry(), "upper_blade_guard"),
        origin=Origin(xyz=(-0.100, 0.0, -0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="upper_guard",
    )
    saw_head.visual(
        Cylinder(radius=0.054, length=0.060),
        origin=Origin(xyz=(-0.100, -0.040, -0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_gray,
        name="gearcase_hub",
    )
    saw_head.visual(
        Cylinder(radius=0.078, length=0.145),
        origin=Origin(xyz=(-0.130, -0.120, -0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="motor_housing",
    )
    saw_head.visual(
        Box((0.040, 0.060, 0.130)),
        origin=Origin(xyz=(-0.030, 0.0, 0.105)),
        material=dark_gray,
        name="handle_stem",
    )
    saw_head.visual(
        Box((0.070, 0.055, 0.040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.025)),
        material=dark_gray,
        name="handle_base",
    )
    saw_head.visual(
        Cylinder(radius=0.020, length=0.180),
        origin=Origin(xyz=(-0.035, 0.0, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle_grip",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(_toothed_blade_geometry(), "toothed_blade"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_steel,
        name="toothed_disc",
    )
    blade.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_chrome,
        name="arbor_washer",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "turntable_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=carriage,
        origin=Origin(xyz=(0.110, 0.0, 0.500)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.280),
    )
    model.articulation(
        "carriage_to_saw_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=saw_head,
        origin=Origin(xyz=(0.020, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "saw_head_to_blade",
        ArticulationType.CONTINUOUS,
        parent=saw_head,
        child=blade,
        origin=Origin(xyz=(-0.100, 0.0, -0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    saw_head = object_model.get_part("saw_head")
    blade = object_model.get_part("blade")
    yaw = object_model.get_articulation("base_to_turntable")
    slide = object_model.get_articulation("turntable_to_carriage")
    pitch = object_model.get_articulation("carriage_to_saw_head")
    spin = object_model.get_articulation("saw_head_to_blade")

    ctx.allow_overlap(
        carriage,
        turntable,
        elem_a="sleeve_0",
        elem_b="rail_0",
        reason="The carriage sleeve is a linear-bearing proxy intentionally captured around the first rail.",
    )
    ctx.allow_overlap(
        carriage,
        turntable,
        elem_a="sleeve_1",
        elem_b="rail_1",
        reason="The carriage sleeve is a linear-bearing proxy intentionally captured around the second rail.",
    )

    ctx.check(
        "four requested mechanisms",
        {yaw.articulation_type, slide.articulation_type, pitch.articulation_type, spin.articulation_type}
        == {
            ArticulationType.REVOLUTE,
            ArticulationType.PRISMATIC,
            ArticulationType.CONTINUOUS,
        }
        and slide.axis == (-1.0, 0.0, 0.0)
        and yaw.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (0.0, 1.0, 0.0)
        and spin.axis == (0.0, 1.0, 0.0),
        details="Expected yaw, rail slide, head pitch, and blade spindle axes.",
    )

    ctx.expect_overlap(
        carriage,
        turntable,
        axes="x",
        elem_a="sleeve_0",
        elem_b="rail_0",
        min_overlap=0.15,
        name="front sleeve remains engaged on its rail at rest",
    )
    ctx.expect_overlap(
        carriage,
        turntable,
        axes="x",
        elem_a="sleeve_1",
        elem_b="rail_1",
        min_overlap=0.15,
        name="rear sleeve remains engaged on its rail at rest",
    )
    ctx.expect_within(
        turntable,
        carriage,
        axes="yz",
        inner_elem="rail_0",
        outer_elem="sleeve_0",
        margin=0.002,
        name="first rail is centered in its sleeve bearing",
    )
    ctx.expect_overlap(
        blade,
        saw_head,
        axes="xz",
        elem_a="toothed_disc",
        elem_b="upper_guard",
        min_overlap=0.12,
        name="upper guard covers the blade arc",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.28}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            turntable,
            axes="x",
            elem_a="sleeve_0",
            elem_b="rail_0",
            min_overlap=0.15,
            name="extended carriage remains on the rails",
        )

    ctx.check(
        "carriage slides forward along rails",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] < rest_carriage[0] - 0.20,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_blade = ctx.part_world_position(blade)
    with ctx.pose({pitch: 0.72}):
        raised_blade = ctx.part_world_position(blade)

    ctx.check(
        "head pitch raises the blade",
        rest_blade is not None
        and raised_blade is not None
        and raised_blade[2] > rest_blade[2] + 0.08,
        details=f"rest={rest_blade}, raised={raised_blade}",
    )

    return ctx.report()


object_model = build_object_model()
