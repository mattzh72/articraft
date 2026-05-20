from __future__ import annotations

from math import cos, pi, sin

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


BEARING_TOP_Z = 0.263
YAW_AXIS_Z = BEARING_TOP_Z


def _radial_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / count), radius * sin(2.0 * pi * i / count))
        for i in range(count)
    ]


def _build_bearing_housing() -> cq.Workplane:
    """Stationary annular bearing housing with a real center bore and bolt holes."""
    flange = cq.Workplane("XY").circle(0.112).circle(0.048).extrude(0.020)
    raised_race = cq.Workplane("XY").circle(0.086).circle(0.043).extrude(0.075).translate(
        (0.0, 0.0, 0.010)
    )
    body = flange.union(raised_race)

    bolt_cuts = (
        cq.Workplane("XY")
        .pushPoints(_radial_points(0.092, 8))
        .circle(0.0058)
        .extrude(0.045)
        .translate((0.0, 0.0, -0.010))
    )
    return body.cut(bolt_cuts)


def _build_top_plate_panel() -> cq.Workplane:
    """Rigid rectangular payload plate with through mounting holes."""
    panel = cq.Workplane("XY").box(0.250, 0.170, 0.018).translate((0.0, 0.0, 0.054))
    hole_cuts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.095, -0.055),
                (-0.095, 0.055),
                (0.095, -0.055),
                (0.095, 0.055),
            ]
        )
        .circle(0.0065)
        .extrude(0.036)
        .translate((0.0, 0.0, 0.036))
    )
    return panel.cut(hole_cuts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_yaw_module")

    model.material("matte_black", rgba=(0.025, 0.028, 0.030, 1.0))
    model.material("motor_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("cast_aluminum", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("top_plate_blue", rgba=(0.10, 0.22, 0.38, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.320, 0.320, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="matte_black",
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.074, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material="cast_aluminum",
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.205, 0.205, 0.094)),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material="motor_dark",
        name="motor_base",
    )
    pedestal.visual(
        Box((0.190, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, -0.104, 0.189)),
        material="matte_black",
        name="front_motor_lip",
    )
    pedestal.visual(
        mesh_from_cadquery(_build_bearing_housing(), "bearing_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material="brushed_steel",
        name="bearing_housing",
    )
    for index, (x, y) in enumerate(
        [(-0.079, -0.079), (-0.079, 0.079), (0.079, -0.079), (0.079, 0.079)]
    ):
        pedestal.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(x, y, 0.202)),
            material="brushed_steel",
            name=f"motor_bolt_{index}",
        )

    top_plate = model.part("top_plate")
    top_plate.visual(
        Cylinder(radius=0.033, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="brushed_steel",
        name="shaft",
    )
    top_plate.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="brushed_steel",
        name="thrust_washer",
    )
    top_plate.visual(
        Cylinder(radius=0.050, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material="brushed_steel",
        name="rotor_hub",
    )
    top_plate.visual(
        mesh_from_cadquery(_build_top_plate_panel(), "top_plate_panel"),
        material="top_plate_blue",
        name="panel",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    top_plate = object_model.get_part("top_plate")
    yaw = object_model.get_articulation("yaw_axis")

    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.expect_gap(
        top_plate,
        pedestal,
        axis="z",
        positive_elem="panel",
        negative_elem="bearing_housing",
        min_gap=0.035,
        max_gap=0.055,
        name="top plate clears fixed bearing housing",
    )
    ctx.expect_overlap(
        top_plate,
        pedestal,
        axes="z",
        elem_a="shaft",
        elem_b="bearing_housing",
        min_overlap=0.025,
        name="rotor shaft remains inserted through bearing height",
    )

    rest_pos = ctx.part_world_position(top_plate)
    with ctx.pose({yaw: pi / 2.0}):
        turned_pos = ctx.part_world_position(top_plate)

    ctx.check(
        "yaw rotation keeps plate centered on pedestal",
        rest_pos is not None
        and turned_pos is not None
        and max(abs(rest_pos[i] - turned_pos[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
