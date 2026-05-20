from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
    radial_segments: int = 64,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _turret_sleeve_mesh() -> MeshGeometry:
    sleeve = _ring_band(outer_radius=0.068, inner_radius=0.041, height=0.165)
    sleeve.merge(_ring_band(outer_radius=0.086, inner_radius=0.041, height=0.030, z_center=-0.083))
    sleeve.merge(_ring_band(outer_radius=0.084, inner_radius=0.041, height=0.026, z_center=0.088))
    return sleeve


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_radial_arm")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.22, 0.24, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.06, 0.065, 0.07, 1.0))
    oiled_steel = model.material("oiled_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    painted_gray = model.material("painted_gray", rgba=(0.54, 0.57, 0.60, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.67, 0.10, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.38, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    column.visual(
        Cylinder(radius=0.034, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=oiled_steel,
        name="vertical_post",
    )
    column.visual(
        Cylinder(radius=0.086, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.581)),
        material=dark_oxide,
        name="lower_bearing_race",
    )
    column.visual(
        Cylinder(radius=0.065, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=cast_iron,
        name="post_foot",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        column.visual(
            Box((0.105, 0.010, 0.150)),
            origin=Origin(xyz=(0.055, 0.0, 0.120), rpy=(0.0, 0.0, angle)),
            material=cast_iron,
            name=f"web_gusset_{index}",
        )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.145, -0.095), (-0.145, 0.095), (0.145, -0.095), (0.145, 0.095))
    ):
        column.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.040)),
            material=dark_oxide,
            name=f"anchor_bolt_{index}",
        )

    turret = model.part("turret")
    turret.visual(
        mesh_from_geometry(_turret_sleeve_mesh(), "turret_sleeve"),
        material=painted_gray,
        name="turret_sleeve",
    )
    turret.visual(
        Box((0.065, 0.150, 0.105)),
        origin=Origin(xyz=(0.095, 0.0, 0.025)),
        material=painted_gray,
        name="beam_socket",
    )
    turret.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=dark_oxide,
        name="bearing_cap",
    )
    turret.visual(
        Box((0.610, 0.110, 0.018)),
        origin=Origin(xyz=(0.365, 0.0, 0.083)),
        material=painted_gray,
        name="beam_top",
    )
    for index, y_pos in enumerate((-0.048, 0.048)):
        turret.visual(
            Box((0.590, 0.014, 0.070)),
            origin=Origin(xyz=(0.370, y_pos, 0.040)),
            material=painted_gray,
            name=f"beam_side_{index}",
        )
        turret.visual(
            Box((0.570, 0.024, 0.012)),
            origin=Origin(xyz=(0.380, y_pos * 0.62, 0.000)),
            material=painted_gray,
            name=f"beam_lip_{index}",
        )
    turret.visual(
        Box((0.070, 0.135, 0.105)),
        origin=Origin(xyz=(0.145, 0.0, 0.035)),
        material=painted_gray,
        name="socket_face",
    )
    turret.visual(
        Cylinder(radius=0.018, length=0.170),
        origin=Origin(xyz=(0.125, 0.0, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oxide,
        name="clamp_cross_pin",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.120, 0.018, 0.024)),
        origin=Origin(xyz=(0.060, 0.0, -0.012)),
        material=oiled_steel,
        name="slider_tongue",
    )
    carriage.visual(
        Box((0.120, 0.134, 0.018)),
        origin=Origin(xyz=(0.060, 0.0, -0.032)),
        material=cast_iron,
        name="saddle_plate",
    )
    for index, y_pos in enumerate((-0.063, 0.063)):
        carriage.visual(
            Box((0.105, 0.016, 0.052)),
            origin=Origin(xyz=(0.060, y_pos, -0.006)),
            material=cast_iron,
            name=f"side_cheek_{index}",
        )
        carriage.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.020, y_pos, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_oxide,
            name=f"guide_roller_{index}_0",
        )
        carriage.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.100, y_pos, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_oxide,
            name=f"guide_roller_{index}_1",
        )
    carriage.visual(
        Box((0.030, 0.070, 0.045)),
        origin=Origin(xyz=(0.112, 0.0, -0.061)),
        material=cast_iron,
        name="head_stem",
    )
    carriage.visual(
        Box((0.095, 0.120, 0.014)),
        origin=Origin(xyz=(0.122, 0.0, -0.089)),
        material=safety_yellow,
        name="head_plate",
    )
    for index, (x_pos, y_pos) in enumerate(((0.092, -0.040), (0.092, 0.040), (0.152, -0.040), (0.152, 0.040))):
        carriage.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, -0.080)),
            material=dark_oxide,
            name=f"head_bolt_{index}",
        )

    model.articulation(
        "column_to_turret",
        ArticulationType.REVOLUTE,
        parent=column,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.7,
            lower=-2.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "turret_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turret,
        child=carriage,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.140,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turret = object_model.get_part("turret")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("column_to_turret")
    slide = object_model.get_articulation("turret_to_carriage")

    ctx.check(
        "turret swing limit is about two radians each way",
        swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and -2.10 <= swing.motion_limits.lower <= -1.55
        and 1.55 <= swing.motion_limits.upper <= 2.10,
        details=f"limits={swing.motion_limits}",
    )
    ctx.check(
        "carriage travel is 140 mm",
        slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and abs(slide.motion_limits.upper - 0.140) <= 0.002,
        details=f"limits={slide.motion_limits}",
    )

    ctx.expect_overlap(
        carriage,
        turret,
        axes="x",
        elem_a="slider_tongue",
        elem_b="beam_top",
        min_overlap=0.10,
        name="carriage is retained on beam at inner travel",
    )
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.140}):
        ctx.expect_overlap(
            carriage,
            turret,
            axes="x",
            elem_a="slider_tongue",
            elem_b="beam_top",
            min_overlap=0.10,
            name="carriage remains retained at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage extends outward along the beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.135,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
