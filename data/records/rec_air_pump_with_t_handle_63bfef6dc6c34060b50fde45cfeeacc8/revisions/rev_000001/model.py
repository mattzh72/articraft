from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _barrel_core():
    barrel_shell = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .circle(0.022)
        .extrude(0.55)
        .cut(
            cq.Workplane("XY")
            .workplane(offset=0.05)
            .circle(0.0175)
            .extrude(0.55)
        )
    )
    bottom_collar = (
        cq.Workplane("XY")
        .workplane(offset=0.03)
        .circle(0.038)
        .circle(0.018)
        .extrude(0.03)
    )
    top_guide = (
        cq.Workplane("XY")
        .workplane(offset=0.60)
        .circle(0.030)
        .circle(0.009)
        .extrude(0.018)
    )
    return barrel_shell.union(bottom_collar).union(top_guide)


def _tube_shell(*, outer_radius: float, inner_radius: float, length: float, axis: str):
    outer_profile = [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)]
    inner_profile = [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)]
    geom = LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56)
    if axis == "x":
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        geom.rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_track_pump")

    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dial_white = model.material("dial_white", rgba=(0.96, 0.96, 0.94, 1.0))
    lens_clear = model.material("lens_clear", rgba=(0.82, 0.88, 0.92, 0.28))
    signal_red = model.material("signal_red", rgba=(0.80, 0.13, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_barrel_core(), "track_pump_frame_core"),
        material=steel,
        name="frame_core",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, 0.02799, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="gauge_boss",
    )
    frame.visual(
        Box((0.13, 0.13, 0.028)),
        origin=Origin(xyz=(-0.095, 0.0, 0.014)),
        material=dark_steel,
        name="foot_left",
    )
    frame.visual(
        Box((0.13, 0.13, 0.028)),
        origin=Origin(xyz=(0.095, 0.0, 0.014)),
        material=dark_steel,
        name="foot_right",
    )
    frame.visual(
        Box((0.12, 0.10, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="bridge",
    )
    frame.visual(
        Box((0.100, 0.095, 0.006)),
        origin=Origin(xyz=(-0.095, 0.0, 0.031)),
        material=black_rubber,
        name="tread_left",
    )
    frame.visual(
        Box((0.100, 0.095, 0.006)),
        origin=Origin(xyz=(0.095, 0.0, 0.031)),
        material=black_rubber,
        name="tread_right",
    )

    hose_geom = tube_from_spline_points(
        [
            (0.098, -0.006, 0.038),
            (0.082, 0.004, 0.055),
            (0.055, 0.020, 0.120),
            (0.041, 0.027, 0.260),
            (0.036, 0.029, 0.430),
            (0.032, 0.027, 0.560),
            (0.030, 0.021, 0.590),
        ],
        radius=0.0052,
        samples_per_segment=18,
        radial_segments=18,
    )
    frame.visual(
        mesh_from_geometry(hose_geom, "track_pump_hose"),
        material=matte_black,
        name="hose",
    )
    frame.visual(
        Box((0.028, 0.022, 0.022)),
        origin=Origin(xyz=(0.098, -0.006, 0.038)),
        material=dark_steel,
        name="hose_dock",
    )
    frame.visual(
        Box((0.020, 0.020, 0.018)),
        origin=Origin(xyz=(0.030, 0.018, 0.260)),
        material=dark_steel,
        name="clip_mid",
    )
    frame.visual(
        Box((0.020, 0.022, 0.018)),
        origin=Origin(xyz=(0.028, 0.021, 0.580)),
        material=dark_steel,
        name="clip_top",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.0075, length=0.47),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="rod",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="stop_collar",
    )
    carriage.visual(
        Box((0.032, 0.024, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_steel,
        name="yoke",
    )
    carriage.visual(
        Cylinder(radius=0.009, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="crossbar",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.017, 0.0, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="retainer_left",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.073, 0.0, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="retainer_right",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(-0.135, 0.0, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="handle_left",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.135, 0.0, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="handle_right",
    )

    model.articulation(
        "pump_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.2, lower=0.0, upper=0.23),
    )

    grip = model.part("grip")
    grip.visual(
        mesh_from_geometry(
            _tube_shell(outer_radius=0.017, inner_radius=0.0105, length=0.052, axis="x"),
            "track_pump_grip_sleeve",
        ),
        material=matte_black,
        name="sleeve",
    )
    grip.visual(
        Box((0.024, 0.044, 0.062)),
        origin=Origin(xyz=(0.0, 0.020, -0.046)),
        material=matte_black,
        name="strap",
    )
    grip.visual(
        Cylinder(radius=0.010, length=0.11),
        origin=Origin(xyz=(0.0, 0.042, -0.087), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="lower_bar",
    )

    model.articulation(
        "grip_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=grip,
        origin=Origin(xyz=(0.045, 0.0, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    gauge = model.part("gauge")
    gauge.visual(
        mesh_from_geometry(
            _tube_shell(outer_radius=0.041, inner_radius=0.036, length=0.022, axis="y"),
            "track_pump_gauge_shell",
        ),
        material=dark_steel,
        name="shell",
    )
    gauge.visual(
        Cylinder(radius=0.036, length=0.001),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_white,
        name="dial",
    )
    gauge.visual(
        Cylinder(radius=0.036, length=0.001),
        origin=Origin(xyz=(0.0, 0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_clear,
        name="lens",
    )
    gauge.visual(
        Cylinder(radius=0.008, length=0.011),
        origin=Origin(xyz=(0.0, -0.0145, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="stem",
    )
    gauge.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot",
    )

    model.articulation(
        "frame_to_gauge",
        ArticulationType.FIXED,
        parent=frame,
        child=gauge,
        origin=Origin(xyz=(0.0, 0.05399, 0.115)),
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.004, 0.0012, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=signal_red,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=signal_red,
        name="hub",
    )

    model.articulation(
        "needle_sweep",
        ArticulationType.REVOLUTE,
        parent=gauge,
        child=needle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
            lower=-1.1,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    grip = object_model.get_part("grip")
    gauge = object_model.get_part("gauge")
    needle = object_model.get_part("needle")

    pump_slide = object_model.get_articulation("pump_slide")
    grip_spin = object_model.get_articulation("grip_spin")
    needle_sweep = object_model.get_articulation("needle_sweep")

    slide_limits = pump_slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({pump_slide: slide_limits.lower}):
            ctx.expect_origin_distance(
                carriage,
                frame,
                axes="xy",
                max_dist=0.001,
                name="carriage stays centered over the barrel at rest",
            )
            ctx.expect_overlap(
                carriage,
                frame,
                axes="z",
                elem_a="rod",
                elem_b="frame_core",
                min_overlap=0.28,
                name="rod remains deeply inserted at rest",
            )
            rest_pos = ctx.part_world_position(carriage)

        with ctx.pose({pump_slide: slide_limits.upper}):
            ctx.expect_origin_distance(
                carriage,
                frame,
                axes="xy",
                max_dist=0.001,
                name="carriage stays centered over the barrel when raised",
            )
            ctx.expect_overlap(
                carriage,
                frame,
                axes="z",
                elem_a="rod",
                elem_b="frame_core",
                min_overlap=0.08,
                name="rod retains insertion when fully raised",
            )
            raised_pos = ctx.part_world_position(carriage)

        ctx.check(
            "handle rises along the barrel axis",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    ctx.expect_overlap(
        grip,
        carriage,
        axes="x",
        elem_a="sleeve",
        elem_b="crossbar",
        min_overlap=0.05,
        name="folding grip sleeve stays carried on the crossbar",
    )

    with ctx.pose({grip_spin: 0.0}):
        grip_rest = ctx.part_element_world_aabb(grip, elem="lower_bar")
    with ctx.pose({grip_spin: math.pi / 2.0}):
        grip_turned = ctx.part_element_world_aabb(grip, elem="lower_bar")
    ctx.check(
        "folding grip swings around the crossbar",
        grip_rest is not None
        and grip_turned is not None
        and abs(((grip_rest[0][2] + grip_rest[1][2]) * 0.5) - ((grip_turned[0][2] + grip_turned[1][2]) * 0.5)) > 0.03,
        details=f"rest={grip_rest}, turned={grip_turned}",
    )

    with ctx.pose({needle_sweep: 0.0}):
        ctx.expect_within(
            needle,
            gauge,
            axes="xz",
            inner_elem="pointer",
            outer_elem="shell",
            margin=0.002,
            name="needle stays inside the gauge window at zero",
        )
        needle_rest = ctx.part_element_world_aabb(needle, elem="pointer")
    with ctx.pose({needle_sweep: 0.75}):
        ctx.expect_within(
            needle,
            gauge,
            axes="xz",
            inner_elem="pointer",
            outer_elem="shell",
            margin=0.002,
            name="needle stays inside the gauge window when swept",
        )
        needle_swept = ctx.part_element_world_aabb(needle, elem="pointer")
    ctx.check(
        "needle rotation changes the pointer orientation",
        needle_rest is not None
        and needle_swept is not None
        and abs(((needle_rest[0][0] + needle_rest[1][0]) * 0.5) - ((needle_swept[0][0] + needle_swept[1][0]) * 0.5)) > 0.006,
        details=f"rest={needle_rest}, swept={needle_swept}",
    )

    return ctx.report()


object_model = build_object_model()
