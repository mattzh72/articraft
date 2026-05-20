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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_track_pump")

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    pump_red = model.material("pump_red", rgba=(0.68, 0.05, 0.035, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.42, 0.43, 0.44, 1.0))
    gauge_white = model.material("gauge_white", rgba=(0.94, 0.93, 0.88, 1.0))
    gauge_glass = model.material("gauge_glass", rgba=(0.65, 0.78, 0.90, 0.32))
    needle_red = model.material("needle_red", rgba=(0.90, 0.05, 0.02, 1.0))

    gauge_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.076, tube=0.006, radial_segments=18, tubular_segments=64),
        "gauge_bezel",
    )
    barrel_top_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.035, tube=0.006, radial_segments=16, tubular_segments=56),
        "barrel_top_ring",
    )
    barrel_base_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.044, tube=0.006, radial_segments=16, tubular_segments=56),
        "barrel_base_ring",
    )
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.150, -0.079, 0.175),
                (0.205, -0.092, 0.285),
                (0.177, -0.088, 0.575),
                (0.075, -0.062, 0.820),
                (0.050, -0.060, 0.640),
                (0.056, -0.063, 0.405),
                (0.092, -0.074, 0.235),
                (0.145, -0.082, 0.162),
            ],
            radius=0.008,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        ),
        "parked_hose",
    )

    body = model.part("pump_body")
    body.visual(
        Box((0.44, 0.26, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_steel,
        name="floor_plate",
    )
    body.visual(
        Box((0.62, 0.095, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=black_rubber,
        name="wide_foot_tread",
    )
    body.visual(
        Box((0.20, 0.20, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=dark_steel,
        name="barrel_plinth",
    )
    body.visual(
        Cylinder(radius=0.054, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=dark_steel,
        name="barrel_socket",
    )
    body.visual(
        barrel_base_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=brushed_steel,
        name="base_crimp_ring",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.820),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=brushed_steel,
        name="barrel_shell",
    )
    body.visual(
        Box((0.020, 0.004, 0.610)),
        origin=Origin(xyz=(0.0, -0.036, 0.530)),
        material=satin_gray,
        name="barrel_shadow_seam",
    )
    body.visual(
        barrel_top_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        material=dark_steel,
        name="top_collar",
    )

    # Gauge body and its bracket are part of the stationary pump body.
    gauge_center = (0.108, -0.087, 0.205)
    body.visual(
        Box((0.130, 0.038, 0.040)),
        origin=Origin(xyz=(0.057, -0.050, 0.198)),
        material=dark_steel,
        name="gauge_bridge",
    )
    body.visual(
        Cylinder(radius=0.079, length=0.048),
        origin=Origin(xyz=gauge_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_housing",
    )
    body.visual(
        Cylinder(radius=0.067, length=0.006),
        origin=Origin(xyz=(gauge_center[0], -0.112, gauge_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_white,
        name="gauge_face",
    )
    body.visual(
        gauge_bezel_mesh,
        origin=Origin(xyz=(gauge_center[0], -0.117, gauge_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.062, length=0.004),
        origin=Origin(xyz=(gauge_center[0], -0.119, gauge_center[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_glass,
        name="gauge_lens",
    )
    for index, angle in enumerate((-2.35, -1.65, -0.95, -0.25, 0.45)):
        tick_x = gauge_center[0] + math.cos(angle) * 0.048
        tick_z = gauge_center[2] + math.sin(angle) * 0.048
        body.visual(
            Box((0.015, 0.0016, 0.0032)),
            origin=Origin(xyz=(tick_x, -0.1215, tick_z), rpy=(0.0, -angle, 0.0)),
            material=dark_steel,
            name=f"gauge_tick_{index}",
        )
    body.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.154, -0.116, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="button_socket",
    )

    body.visual(hose_mesh, material=black_rubber, name="parked_hose")
    body.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.151, -0.084, 0.156), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hose_chuck",
    )
    body.visual(
        Box((0.072, 0.026, 0.030)),
        origin=Origin(xyz=(0.047, -0.041, 0.430)),
        material=black_rubber,
        name="hose_clip_low",
    )
    body.visual(
        Box((0.082, 0.030, 0.030)),
        origin=Origin(xyz=(0.052, -0.041, 0.690)),
        material=black_rubber,
        name="hose_clip_high",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.064),
        origin=Origin(xyz=(0.147, -0.078, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hose_outlet_boss",
    )

    plunger = model.part("handle_rod")
    plunger.visual(
        Cylinder(radius=0.010, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=brushed_steel,
        name="piston_rod",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_steel,
        name="handle_neck",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_bar",
    )
    plunger.visual(
        Cylinder(radius=0.027, length=0.145),
        origin=Origin(xyz=(-0.145, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="grip_0",
    )
    plunger.visual(
        Cylinder(radius=0.027, length=0.145),
        origin=Origin(xyz=(0.145, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="grip_1",
    )
    plunger.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(-0.220, 0.0, 0.220)),
        material=black_rubber,
        name="grip_end_0",
    )
    plunger.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.220, 0.0, 0.220)),
        material=black_rubber,
        name="grip_end_1",
    )

    needle = model.part("gauge_needle")
    needle.visual(
        Box((0.064, 0.003, 0.006)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=needle_red,
        name="pointer",
    )
    needle.visual(
        Box((0.024, 0.003, 0.004)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=needle_red,
        name="counter_tail",
    )
    needle.visual(
        Cylinder(radius=0.0085, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_cap",
    )

    button = model.part("release_button")
    button.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pump_red,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, -0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pump_red,
        name="button_cap",
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.930)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=0.0, upper=0.340),
    )
    model.articulation(
        "needle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=needle,
        origin=Origin(xyz=(gauge_center[0], -0.1235, gauge_center[2])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=7.0, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.154, -0.120, 0.245)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("pump_body")
    plunger = object_model.get_part("handle_rod")
    needle = object_model.get_part("gauge_needle")
    button = object_model.get_part("release_button")
    plunger_slide = object_model.get_articulation("plunger_slide")
    needle_pivot = object_model.get_articulation("needle_pivot")
    button_press = object_model.get_articulation("button_press")

    ctx.allow_overlap(
        body,
        plunger,
        elem_a="barrel_shell",
        elem_b="piston_rod",
        reason="The polished piston rod is intentionally shown retained inside the pump barrel sleeve.",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.001,
        name="piston rod is centered in barrel",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.35,
        name="collapsed piston remains inserted",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(plunger, elem="handle_bar")
    with ctx.pose({plunger_slide: 0.340}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="barrel_shell",
            margin=0.001,
            name="extended piston remains coaxial",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.10,
            name="extended piston stays captured",
        )
        extended_handle_aabb = ctx.part_element_world_aabb(plunger, elem="handle_bar")
    ctx.check(
        "handle translates upward along barrel",
        rest_handle_aabb is not None
        and extended_handle_aabb is not None
        and extended_handle_aabb[0][2] > rest_handle_aabb[0][2] + 0.30,
        details=f"rest={rest_handle_aabb}, extended={extended_handle_aabb}",
    )

    ctx.expect_contact(
        needle,
        body,
        elem_a="pivot_cap",
        elem_b="gauge_face",
        contact_tol=0.006,
        name="needle pivots on gauge face",
    )
    rest_pointer_aabb = ctx.part_element_world_aabb(needle, elem="pointer")
    with ctx.pose({needle_pivot: 0.85}):
        swung_pointer_aabb = ctx.part_element_world_aabb(needle, elem="pointer")
    if rest_pointer_aabb is not None and swung_pointer_aabb is not None:
        rest_pointer_z = (rest_pointer_aabb[0][2] + rest_pointer_aabb[1][2]) * 0.5
        swung_pointer_z = (swung_pointer_aabb[0][2] + swung_pointer_aabb[1][2]) * 0.5
        needle_moves = abs(swung_pointer_z - rest_pointer_z) > 0.020
    else:
        needle_moves = False
    ctx.check(
        "gauge needle rotates about central pivot",
        needle_moves,
        details=f"rest={rest_pointer_aabb}, swung={swung_pointer_aabb}",
    )

    ctx.expect_contact(
        button,
        body,
        elem_a="button_stem",
        elem_b="button_socket",
        contact_tol=0.003,
        name="release button is seated in socket",
    )
    rest_button_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
    with ctx.pose({button_press: 0.008}):
        pressed_button_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
    ctx.check(
        "release button pushes inward",
        rest_button_aabb is not None
        and pressed_button_aabb is not None
        and pressed_button_aabb[1][1] > rest_button_aabb[1][1] + 0.006,
        details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
