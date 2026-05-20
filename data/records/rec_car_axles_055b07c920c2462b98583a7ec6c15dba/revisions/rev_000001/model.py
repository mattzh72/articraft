from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL = Material("satin_black_steel", rgba=(0.035, 0.036, 0.034, 1.0))
BRIGHT_STEEL = Material("machined_steel", rgba=(0.64, 0.66, 0.64, 1.0))
CAST_IRON = Material("dark_cast_iron", rgba=(0.12, 0.125, 0.12, 1.0))
RUBBER = Material("cv_boot_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
ZINC = Material("zinc_plated_fasteners", rgba=(0.78, 0.76, 0.70, 1.0))


def _tube_along_x(outer_radius: float, inner_radius: float, length: float, start_x: float) -> cq.Workplane:
    """Return a hollow cylindrical shell running from start_x to start_x + length."""
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    bore = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.004).translate((-0.002, 0.0, 0.0))
    return outer.cut(bore).translate((start_x, 0.0, 0.0))


def _x_cylinder(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irs_rear_half_shaft")

    inboard = model.part("inboard_coupling")
    cup_shell = _tube_along_x(0.088, 0.064, 0.145, -0.145)
    cup_lip = _tube_along_x(0.098, 0.064, 0.030, -0.018)
    inboard.visual(
        mesh_from_cadquery(cup_shell, "inboard_cup_shell"),
        material=CAST_IRON,
        name="cup_shell",
    )
    inboard.visual(
        mesh_from_cadquery(cup_lip, "inboard_cup_lip"),
        material=CAST_IRON,
        name="cup_lip",
    )
    inboard.visual(
        _x_cylinder(0.110, 0.044),
        origin=_origin_x(-0.166),
        material=CAST_IRON,
        name="differential_flange",
    )
    inboard.visual(
        _x_cylinder(0.050, 0.055),
        origin=_origin_x(-0.182),
        material=BRIGHT_STEEL,
        name="input_stub",
    )
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        y = 0.082 * math.cos(angle)
        z = 0.082 * math.sin(angle)
        inboard.visual(
            _x_cylinder(0.0085, 0.020),
            origin=_origin_x(-0.198, y, z),
            material=ZINC,
            name=f"flange_bolt_{i}",
        )
    for i in range(3):
        angle = 2.0 * math.pi * i / 3.0 + math.pi / 6.0
        y = 0.084 * math.cos(angle)
        z = 0.084 * math.sin(angle)
        inboard.visual(
            _x_cylinder(0.009, 0.112),
            origin=_origin_x(-0.070, y, z),
            material=CAST_IRON,
            name=f"tulip_rib_{i}",
        )
    inboard.visual(
        Box((0.036, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.077, 0.0)),
        material=CAST_IRON,
        name="pivot_ear_0",
    )
    inboard.visual(
        Box((0.036, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, -0.077, 0.0)),
        material=CAST_IRON,
        name="pivot_ear_1",
    )

    gimbal = model.part("inboard_tripod")
    gimbal.visual(
        Sphere(0.043),
        origin=Origin(),
        material=BRIGHT_STEEL,
        name="tripod_core",
    )
    gimbal.visual(
        _x_cylinder(0.026, 0.070),
        origin=_origin_x(0.035),
        material=BRIGHT_STEEL,
        name="splined_neck",
    )
    gimbal.visual(
        Cylinder(radius=0.010, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRIGHT_STEEL,
        name="cross_pin",
    )
    for i in range(3):
        angle = 2.0 * math.pi * i / 3.0 + math.pi / 2.0
        y = 0.045 * math.cos(angle)
        z = 0.045 * math.sin(angle)
        gimbal.visual(
            Sphere(0.010),
            origin=Origin(xyz=(0.0, y, z)),
            material=BRIGHT_STEEL,
            name=f"roller_{i}",
        )

    shaft = model.part("shaft")
    shaft.visual(
        _x_cylinder(0.030, 0.060),
        origin=_origin_x(0.030),
        material=BRIGHT_STEEL,
        name="inboard_spline",
    )
    shaft.visual(
        _x_cylinder(0.024, 0.540),
        origin=_origin_x(0.330),
        material=STEEL,
        name="shaft_tube",
    )
    shaft.visual(
        _x_cylinder(0.031, 0.060),
        origin=_origin_x(0.650),
        material=BRIGHT_STEEL,
        name="outboard_spline",
    )
    # Corrugated rubber boots at both CV ends, built from overlapping rings so
    # they read as one supported boot rather than separate floating bands.
    for i, x in enumerate((0.075, 0.095, 0.115, 0.135)):
        shaft.visual(
            _x_cylinder(0.040 + 0.003 * (i % 2), 0.026),
            origin=_origin_x(x),
            material=RUBBER,
            name=f"inboard_boot_{i}",
        )
    for i, x in enumerate((0.545, 0.568, 0.591, 0.614, 0.637)):
        shaft.visual(
            _x_cylinder(0.036 + 0.004 * (i % 2), 0.030),
            origin=_origin_x(x),
            material=RUBBER,
            name=f"outboard_boot_{i}",
        )

    hub = model.part("wheel_hub")
    hub.visual(
        _x_cylinder(0.034, 0.040),
        origin=_origin_x(0.020),
        material=BRIGHT_STEEL,
        name="cv_input_stub",
    )
    hub.visual(
        Sphere(0.072),
        origin=Origin(xyz=(0.088, 0.0, 0.0)),
        material=CAST_IRON,
        name="outboard_cv_body",
    )
    hub.visual(
        _x_cylinder(0.050, 0.080),
        origin=_origin_x(0.152),
        material=BRIGHT_STEEL,
        name="bearing_barrel",
    )
    hub.visual(
        _x_cylinder(0.116, 0.040),
        origin=_origin_x(0.205),
        material=CAST_IRON,
        name="wheel_flange",
    )
    hub.visual(
        _x_cylinder(0.043, 0.036),
        origin=_origin_x(0.243),
        material=BRIGHT_STEEL,
        name="pilot_nose",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.pi / 2.0
        y = 0.078 * math.cos(angle)
        z = 0.078 * math.sin(angle)
        hub.visual(
            _x_cylinder(0.0065, 0.036),
            origin=_origin_x(0.243, y, z),
            material=ZINC,
            name=f"wheel_stud_{i}",
        )

    model.articulation(
        "inboard_angulation",
        ArticulationType.REVOLUTE,
        parent=inboard,
        child=gimbal,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=gimbal,
        child=shaft,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=80.0),
    )
    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=hub,
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    inboard = object_model.get_part("inboard_coupling")
    gimbal = object_model.get_part("inboard_tripod")
    shaft = object_model.get_part("shaft")
    hub = object_model.get_part("wheel_hub")
    angle = object_model.get_articulation("inboard_angulation")
    shaft_spin = object_model.get_articulation("shaft_spin")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.check(
        "primary mechanisms are articulated",
        angle.articulation_type == ArticulationType.REVOLUTE
        and shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and hub_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types: {angle.articulation_type}, {shaft_spin.articulation_type}, {hub_spin.articulation_type}",
    )
    ctx.expect_overlap(
        shaft,
        hub,
        axes="yz",
        min_overlap=0.025,
        elem_a="outboard_spline",
        elem_b="cv_input_stub",
        name="shaft spline is concentric with the outboard CV input",
    )
    ctx.expect_gap(
        shaft,
        gimbal,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="inboard_spline",
        negative_elem="splined_neck",
        name="shaft spline seats against the angulating tripod neck",
    )

    rest_hub = ctx.part_world_position(hub)
    with ctx.pose({angle: 0.30}):
        raised_hub = ctx.part_world_position(hub)
    ctx.check(
        "positive inboard angulation lifts the outboard hub",
        rest_hub is not None and raised_hub is not None and raised_hub[2] > rest_hub[2] + 0.15,
        details=f"rest={rest_hub}, raised={raised_hub}",
    )

    with ctx.pose({shaft_spin: math.pi / 2.0}):
        spun_hub = ctx.part_world_position(hub)
    ctx.check(
        "shaft spin is about its own axis",
        rest_hub is not None
        and spun_hub is not None
        and abs(spun_hub[1] - rest_hub[1]) < 1e-6
        and abs(spun_hub[2] - rest_hub[2]) < 1e-6,
        details=f"rest={rest_hub}, spun={spun_hub}",
    )

    with ctx.pose({hub_spin: math.pi / 2.0}):
        spun_hub_origin = ctx.part_world_position(hub)
    ctx.check(
        "wheel hub spins in place at the outboard end",
        rest_hub is not None
        and spun_hub_origin is not None
        and abs(spun_hub_origin[0] - rest_hub[0]) < 1e-6
        and abs(spun_hub_origin[1] - rest_hub[1]) < 1e-6
        and abs(spun_hub_origin[2] - rest_hub[2]) < 1e-6,
        details=f"rest={rest_hub}, spun={spun_hub_origin}",
    )

    return ctx.report()


object_model = build_object_model()
