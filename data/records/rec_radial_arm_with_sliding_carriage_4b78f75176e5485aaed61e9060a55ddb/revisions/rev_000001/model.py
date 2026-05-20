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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_beam_carriage_module")

    steel = model.material("painted_steel_dark", color=(0.18, 0.20, 0.22, 1.0))
    yellow = model.material("safety_yellow", color=(0.95, 0.68, 0.08, 1.0))
    blue = model.material("carriage_blue", color=(0.10, 0.28, 0.55, 1.0))
    rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    bearing = model.material("machined_bearing", color=(0.62, 0.64, 0.62, 1.0))
    fastener = model.material("dark_fasteners", color=(0.05, 0.05, 0.055, 1.0))

    fixed_column = model.part("fixed_column")
    fixed_column.visual(
        Box((0.58, 0.58, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=steel,
        name="floor_plate",
    )
    fixed_column.visual(
        Cylinder(radius=0.062, length=0.91),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=steel,
        name="column_shaft",
    )
    fixed_column.visual(
        Cylinder(radius=0.132, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.9175)),
        material=bearing,
        name="lower_race",
    )
    fixed_column.visual(
        Cylinder(radius=0.060, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
        material=steel,
        name="pivot_pin",
    )
    fixed_column.visual(
        Cylinder(radius=0.132, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.1775)),
        material=bearing,
        name="upper_retainer",
    )
    for i, (x, y) in enumerate(
        ((0.22, 0.22), (-0.22, 0.22), (-0.22, -0.22), (0.22, -0.22))
    ):
        fixed_column.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.056)),
            material=fastener,
            name=f"anchor_bolt_{i}",
        )

    swing_beam = model.part("swing_beam")
    sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.125, -0.105), (0.125, 0.105)],
            [(0.078, -0.105), (0.078, 0.105)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "pivot_sleeve",
    )
    swing_beam.visual(
        sleeve_mesh,
        origin=Origin(),
        material=yellow,
        name="pivot_sleeve",
    )
    swing_beam.visual(
        Box((1.36, 0.038, 0.158)),
        origin=Origin(xyz=(0.78, 0.0, 0.0)),
        material=yellow,
        name="beam_web",
    )
    swing_beam.visual(
        Box((1.36, 0.150, 0.032)),
        origin=Origin(xyz=(0.78, 0.0, 0.079)),
        material=yellow,
        name="top_flange",
    )
    swing_beam.visual(
        Box((1.36, 0.150, 0.032)),
        origin=Origin(xyz=(0.78, 0.0, -0.079)),
        material=yellow,
        name="bottom_flange",
    )
    swing_beam.visual(
        Box((0.16, 0.19, 0.19)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=yellow,
        name="root_knuckle",
    )
    swing_beam.visual(
        Box((0.045, 0.215, 0.235)),
        origin=Origin(xyz=(1.4825, 0.0, 0.0)),
        material=steel,
        name="end_stop",
    )
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.235, 0.026, 0.222)),
        origin=Origin(xyz=(0.0, 0.101, -0.016)),
        material=blue,
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.235, 0.026, 0.222)),
        origin=Origin(xyz=(0.0, -0.101, -0.016)),
        material=blue,
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.235, 0.226, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.132)),
        material=blue,
        name="lower_bridge",
    )
    carriage.visual(
        Box((0.095, 0.105, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        material=blue,
        name="hanger_block",
    )
    carriage.visual(
        Cylinder(radius=0.030, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.223), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="load_pin",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.205),
        origin=Origin(xyz=(-0.070, 0.0, 0.116), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_0",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.205),
        origin=Origin(xyz=(0.070, 0.0, 0.116), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_1",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.205),
        origin=Origin(xyz=(-0.070, 0.0, -0.116), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="bottom_roller_0",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.205),
        origin=Origin(xyz=(0.070, 0.0, -0.116), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="bottom_roller_1",
    )
    for i, (x, y, z) in enumerate(
        (
            (-0.075, 0.120, 0.025),
            (0.075, 0.120, 0.025),
            (-0.075, -0.120, 0.025),
            (0.075, -0.120, 0.025),
        )
    ):
        carriage.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener,
            name=f"cheek_bolt_{i}",
        )

    model.articulation(
        "column_to_beam",
        ArticulationType.REVOLUTE,
        parent=fixed_column,
        child=swing_beam,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.7, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=swing_beam,
        child=carriage,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=0.0, upper=0.86),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_column = object_model.get_part("fixed_column")
    swing_beam = object_model.get_part("swing_beam")
    carriage = object_model.get_part("carriage")
    beam_sweep = object_model.get_articulation("column_to_beam")
    carriage_slide = object_model.get_articulation("beam_to_carriage")

    ctx.expect_gap(
        swing_beam,
        fixed_column,
        axis="z",
        positive_elem="pivot_sleeve",
        negative_elem="lower_race",
        min_gap=0.0,
        max_gap=0.0015,
        name="pivot sleeve sits on lower thrust race",
    )
    ctx.expect_gap(
        fixed_column,
        swing_beam,
        axis="z",
        positive_elem="upper_retainer",
        negative_elem="pivot_sleeve",
        min_gap=0.0,
        max_gap=0.0015,
        name="upper retainer captures pivot sleeve",
    )
    ctx.expect_contact(
        carriage,
        swing_beam,
        elem_a="top_roller_0",
        elem_b="top_flange",
        contact_tol=0.002,
        name="upper rollers bear on the beam flange",
    )
    ctx.expect_gap(
        carriage,
        swing_beam,
        axis="y",
        positive_elem="side_plate_0",
        negative_elem="top_flange",
        min_gap=0.010,
        max_gap=0.025,
        name="carriage cheek clears the beam side",
    )
    ctx.expect_overlap(
        carriage,
        swing_beam,
        axes="x",
        min_overlap=0.20,
        name="carriage remains over the beam at the inboard stop",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.86}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            swing_beam,
            elem_a="top_roller_1",
            elem_b="top_flange",
            contact_tol=0.002,
            name="upper rollers still ride the flange at full travel",
        )
        ctx.expect_overlap(
            carriage,
            swing_beam,
            axes="x",
            min_overlap=0.20,
            name="carriage remains on beam at the outboard stop",
        )

    ctx.check(
        "carriage slide travels along beam",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.80,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_stop = ctx.part_element_world_aabb(swing_beam, elem="end_stop")
    with ctx.pose({beam_sweep: 1.05}):
        swept_stop = ctx.part_element_world_aabb(swing_beam, elem="end_stop")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[1] + hi[1])

    ctx.check(
        "beam sweeps around column",
        _center_y(rest_stop) is not None
        and _center_y(swept_stop) is not None
        and _center_y(swept_stop) > _center_y(rest_stop) + 1.0,
        details=f"rest_end_stop={rest_stop}, swept_end_stop={swept_stop}",
    )

    return ctx.report()


object_model = build_object_model()
