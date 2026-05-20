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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    name: str,
    segments: int = 72,
):
    """Thin lathed tube or collar with a real central bore."""
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_track_pump")

    satin_red = model.material("satin_red", rgba=(0.72, 0.08, 0.06, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    gauge_white = model.material("gauge_white", rgba=(0.93, 0.91, 0.86, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.55, 0.20, 1.0))

    barrel_shell = _annular_shell(
        outer_radius=0.036,
        inner_radius=0.026,
        z_min=0.180,
        z_max=0.930,
        name="barrel_shell",
    )
    top_bushing = _annular_shell(
        outer_radius=0.046,
        inner_radius=0.014,
        z_min=0.905,
        z_max=0.955,
        name="top_bushing",
        segments=64,
    )
    lower_band = _annular_shell(
        outer_radius=0.040,
        inner_radius=0.026,
        z_min=0.156,
        z_max=0.184,
        name="lower_barrel_band",
        segments=64,
    )
    upper_band = _annular_shell(
        outer_radius=0.039,
        inner_radius=0.026,
        z_min=0.887,
        z_max=0.913,
        name="upper_barrel_band",
        segments=64,
    )

    pump_body = model.part("pump_body")
    pump_body.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_metal,
        name="tripod_hub",
    )
    pump_body.visual(
        Cylinder(radius=0.047, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark_metal,
        name="barrel_socket",
    )
    pump_body.visual(
        barrel_shell,
        material=satin_red,
        name="barrel_shell",
    )
    pump_body.visual(
        top_bushing,
        material=dark_metal,
        name="top_bushing",
    )
    pump_body.visual(
        lower_band,
        material=brushed_steel,
        name="lower_barrel_band",
    )
    pump_body.visual(
        upper_band,
        material=brushed_steel,
        name="upper_barrel_band",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        pump_body.visual(
            Box((0.430, 0.070, 0.026)),
            origin=Origin(xyz=(0.175 * c, 0.175 * s, 0.027), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"foot_arm_{index}",
        )
        pump_body.visual(
            Box((0.185, 0.110, 0.018)),
            origin=Origin(xyz=(0.330 * c, 0.330 * s, 0.012), rpy=(0.0, 0.0, angle)),
            material=black_rubber,
            name=f"rubber_foot_{index}",
        )

    # A visible gauge and short hose make the pump read as a real floor pump,
    # while remaining rigidly mounted to the fixed pump body.
    pump_body.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(0.0, -0.047, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="gauge_bezel",
    )
    pump_body.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.0, -0.064, 0.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="gauge_face",
    )
    pump_body.visual(
        Box((0.004, 0.006, 0.036)),
        origin=Origin(xyz=(0.012, -0.067, 0.270), rpy=(0.0, 0.0, -0.8)),
        material=dark_metal,
        name="gauge_needle",
    )
    hose_geom = tube_from_spline_points(
        [
            (0.030, -0.046, 0.240),
            (0.180, -0.130, 0.165),
            (0.300, -0.100, 0.430),
            (0.165, -0.055, 0.695),
            (0.050, -0.035, 0.700),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    pump_body.visual(
        mesh_from_geometry(hose_geom, "side_hose"),
        material=black_rubber,
        name="side_hose",
    )
    pump_body.visual(
        Box((0.020, 0.028, 0.055)),
        origin=Origin(xyz=(0.044, -0.030, 0.700)),
        material=dark_metal,
        name="hose_clip",
    )
    pump_body.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(0.095, -0.034, 0.700), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="pump_nozzle",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.008, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, -0.250)),
        material=brushed_steel,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_metal,
        name="guide_stem",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=black_rubber,
        name="wiper_flange",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_metal,
        name="handle_socket",
    )
    plunger.visual(
        Cylinder(radius=0.025, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="grip",
    )
    plunger.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.195, 0.0, 0.145)),
        material=black_rubber,
        name="grip_cap_0",
    )
    plunger.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.195, 0.0, 0.145)),
        material=black_rubber,
        name="grip_cap_1",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=pump_body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pump_body = object_model.get_part("pump_body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        pump_body,
        axes="xy",
        inner_elem="rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="rod is centered within the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        pump_body,
        axes="z",
        elem_a="rod",
        elem_b="barrel_shell",
        min_overlap=0.450,
        name="lowered rod remains deeply inserted",
    )
    ctx.expect_gap(
        plunger,
        pump_body,
        axis="z",
        positive_elem="grip",
        negative_elem="barrel_shell",
        min_gap=0.110,
        name="T handle sits above the barrel",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.300}):
        ctx.expect_overlap(
            plunger,
            pump_body,
            axes="z",
            elem_a="rod",
            elem_b="barrel_shell",
            min_overlap=0.180,
            name="raised rod retains insertion in the barrel",
        )
        raised_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger slides upward along the barrel axis",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.290,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
