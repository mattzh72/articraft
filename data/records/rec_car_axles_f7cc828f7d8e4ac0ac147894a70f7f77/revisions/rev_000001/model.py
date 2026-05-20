from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_AXIS_RPY = (0.0, math.pi / 2.0, 0.0)
INPUT_Z = 0.20
OUTPUT_Z = -0.20


def _annular_boss(z_center: float, outer_radius: float, inner_radius: float, thickness: float):
    """A bearing-boss ring extruded along the vehicle/wheel shaft axis (X)."""
    return (
        cq.Workplane("YZ")
        .center(0.0, z_center)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((-thickness / 2.0, 0.0, 0.0))
    )


def _round_bolt(x_face: float, y: float, z: float, radius: float = 0.013):
    """Short round bolt head on the outboard face of the cast housing."""
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(0.024)
        .translate((x_face, 0.0, 0.0))
    )


def _build_housing_shape():
    upper = _annular_boss(INPUT_Z, outer_radius=0.155, inner_radius=0.060, thickness=0.230)
    lower = _annular_boss(OUTPUT_Z, outer_radius=0.205, inner_radius=0.108, thickness=0.250)

    # Cast side webs join the two bearing bosses while keeping the center visibly
    # recessed/open like a serviceable portal reduction case.
    rail_a = cq.Workplane("XY").box(0.190, 0.058, 0.430).translate((0.0, 0.142, 0.0))
    rail_b = cq.Workplane("XY").box(0.190, 0.058, 0.430).translate((0.0, -0.142, 0.0))
    center_rib = cq.Workplane("XY").box(0.105, 0.055, 0.330).translate((-0.045, 0.0, 0.0))

    shape = upper.union(lower).union(rail_a).union(rail_b).union(center_rib)

    # Bolt circles on the removable outboard cover face.
    for count, circle_radius, z_center, head_radius in (
        (6, 0.122, INPUT_Z, 0.012),
        (8, 0.162, OUTPUT_Z, 0.014),
    ):
        for i in range(count):
            angle = 2.0 * math.pi * i / count
            y = circle_radius * math.cos(angle)
            z = z_center + circle_radius * math.sin(angle)
            shape = shape.union(_round_bolt(0.106, y, z, head_radius))

    # A small drain boss at the bottom gives the vertical casting an oil-filled
    # gearbox character and connects directly into the lower boss.
    drain = cq.Workplane("XY").box(0.090, 0.070, 0.045).translate((0.0, 0.0, OUTPUT_Z - 0.205))
    return shape.union(drain)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_axle_hub_reduction")

    cast_iron = model.material("dark_cast_iron", rgba=(0.12, 0.13, 0.13, 1.0))
    machined = model.material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("blackened_fasteners", rgba=(0.025, 0.025, 0.023, 1.0))
    rubber_seal = model.material("black_rubber_seal", rgba=(0.005, 0.005, 0.004, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "portal_gear_housing", tolerance=0.0008),
        material=cast_iron,
        name="cast_portal_case",
    )
    # Static seals at the two shaft openings make the rotating members read as
    # bearing-supported rather than floating from the case.
    housing.visual(
        mesh_from_cadquery(
            _annular_boss(INPUT_Z, outer_radius=0.071, inner_radius=0.043, thickness=0.018).translate(
                (-0.124, 0.0, 0.0)
            ),
            "input_annular_seal",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=rubber_seal,
        name="input_shaft_seal",
    )
    housing.visual(
        mesh_from_cadquery(
            _annular_boss(OUTPUT_Z, outer_radius=0.120, inner_radius=0.062, thickness=0.020).translate(
                (0.128, 0.0, 0.0)
            ),
            "hub_annular_seal",
            tolerance=0.0008,
        ),
        origin=Origin(),
        material=rubber_seal,
        name="hub_oil_seal",
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=0.036, length=0.340),
        origin=Origin(xyz=(-0.300, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=machined,
        name="axle_stub",
    )
    input_shaft.visual(
        Cylinder(radius=0.078, length=0.042),
        origin=Origin(xyz=(-0.154, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=machined,
        name="input_flange",
    )
    input_shaft.visual(
        Cylinder(radius=0.044, length=0.090),
        origin=Origin(xyz=(-0.455, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=dark_steel,
        name="splined_end",
    )
    for i in range(8):
        angle = 2.0 * math.pi * i / 8
        y = 0.042 * math.cos(angle)
        z = 0.042 * math.sin(angle)
        input_shaft.visual(
            Box((0.085, 0.006, 0.006)),
            origin=Origin(xyz=(-0.455, y, z), rpy=(angle, 0.0, 0.0)),
            material=machined,
            name=f"spline_{i}",
        )

    wheel_hub = model.part("wheel_hub")
    wheel_hub.visual(
        Cylinder(radius=0.054, length=0.280),
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=machined,
        name="output_spindle",
    )
    wheel_hub.visual(
        Cylinder(radius=0.103, length=0.095),
        origin=Origin(xyz=(0.185, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=machined,
        name="inner_hub_barrel",
    )
    wheel_hub.visual(
        Cylinder(radius=0.180, length=0.052),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=machined,
        name="wheel_flange",
    )
    wheel_hub.visual(
        Cylinder(radius=0.075, length=0.070),
        origin=Origin(xyz=(0.398, 0.0, 0.0), rpy=SHAFT_AXIS_RPY),
        material=dark_steel,
        name="pilot_register",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5
        y = 0.136 * math.cos(angle)
        z = 0.136 * math.sin(angle)
        wheel_hub.visual(
            Cylinder(radius=0.012, length=0.095),
            origin=Origin(xyz=(0.405, y, z), rpy=SHAFT_AXIS_RPY),
            material=dark_steel,
            name=f"wheel_stud_{i}",
        )
        wheel_hub.visual(
            Cylinder(radius=0.019, length=0.026),
            origin=Origin(xyz=(0.462, y, z), rpy=SHAFT_AXIS_RPY),
            material=machined,
            name=f"lug_nut_{i}",
        )

    input_spin = model.articulation(
        "input_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=input_shaft,
        origin=Origin(xyz=(0.0, 0.0, INPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=45.0),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=wheel_hub,
        origin=Origin(xyz=(0.0, 0.0, OUTPUT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=22.0),
        mimic=Mimic(joint=input_spin.name, multiplier=-0.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    input_shaft = object_model.get_part("input_shaft")
    wheel_hub = object_model.get_part("wheel_hub")
    input_spin = object_model.get_articulation("input_spin")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "input shaft has continuous rotation",
        input_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={input_spin.articulation_type}",
    )
    ctx.check(
        "wheel hub has continuous rotation",
        hub_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={hub_spin.articulation_type}",
    )
    ctx.expect_origin_gap(
        input_shaft,
        wheel_hub,
        axis="z",
        min_gap=0.38,
        max_gap=0.42,
        name="wheel hub center is offset below axle center",
    )
    ctx.expect_origin_distance(
        input_shaft,
        wheel_hub,
        axes="xy",
        max_dist=0.001,
        name="input and hub centers share the same shaft plane",
    )
    ctx.check(
        "hub reduction mimic is slower than input",
        hub_spin.mimic is not None and hub_spin.mimic.multiplier == -0.5,
        details=f"mimic={hub_spin.mimic}",
    )

    return ctx.report()


object_model = build_object_model()
