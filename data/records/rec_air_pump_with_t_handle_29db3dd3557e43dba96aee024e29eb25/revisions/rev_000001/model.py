from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_training_pump")

    frame_mat = model.material("satin_black_frame", rgba=(0.015, 0.017, 0.020, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    blue_mat = model.material("short_blue_barrel", rgba=(0.03, 0.18, 0.42, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    white_mat = model.material("warm_white_dial", rgba=(0.92, 0.90, 0.82, 1.0))
    red_mat = model.material("release_red", rgba=(0.82, 0.04, 0.03, 1.0))
    needle_mat = model.material("needle_black", rgba=(0.02, 0.018, 0.015, 1.0))

    base = model.part("base_frame")

    # A compact floor-pump footprint: low wide frame, short vertical barrel, and
    # the round gauge sitting as an integrated pod in the base rather than a
    # remote panel.
    base.visual(
        Box((0.42, 0.16, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=frame_mat,
        name="low_base_plate",
    )
    base.visual(
        Box((0.32, 0.038, 0.026)),
        origin=Origin(xyz=(-0.025, 0.0, 0.045)),
        material=frame_mat,
        name="raised_center_spine",
    )
    base.visual(
        Box((0.18, 0.030, 0.010)),
        origin=Origin(xyz=(-0.040, 0.066, 0.037)),
        material=rubber_mat,
        name="foot_pad_0",
    )
    base.visual(
        Box((0.18, 0.030, 0.010)),
        origin=Origin(xyz=(-0.040, -0.066, 0.037)),
        material=rubber_mat,
        name="foot_pad_1",
    )

    # The barrel is a real hollow tube mesh with fat end collars, so the pump rod
    # can visibly slide through it instead of intersecting a solid cylinder.
    barrel_outer = [
        (0.043, -0.166),
        (0.043, -0.146),
        (0.035, -0.140),
        (0.035, 0.140),
        (0.043, 0.146),
        (0.043, 0.166),
    ]
    barrel_inner = [
        (0.023, -0.166),
        (0.023, 0.166),
    ]
    barrel_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            barrel_outer,
            barrel_inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=4,
        ),
        "short_hollow_barrel",
    )
    base.visual(
        barrel_mesh,
        # The bottom collar just kisses/enters the low base so the fixed frame is
        # one supported assembly.
        origin=Origin(xyz=(-0.075, 0.0, 0.206)),
        material=blue_mat,
        name="barrel_tube",
    )
    base.visual(
        Box((0.090, 0.070, 0.045)),
        origin=Origin(xyz=(-0.075, 0.0, 0.058)),
        material=frame_mat,
        name="barrel_socket",
    )
    base.visual(
        Box((0.032, 0.014, 0.125)),
        origin=Origin(xyz=(-0.115, 0.050, 0.095), rpy=(0.0, math.radians(18.0), 0.0)),
        material=frame_mat,
        name="rear_brace_0",
    )
    base.visual(
        Box((0.032, 0.014, 0.125)),
        origin=Origin(xyz=(-0.115, -0.050, 0.095), rpy=(0.0, math.radians(18.0), 0.0)),
        material=frame_mat,
        name="rear_brace_1",
    )
    base.visual(
        Box((0.032, 0.014, 0.115)),
        origin=Origin(xyz=(-0.040, 0.050, 0.091), rpy=(0.0, math.radians(-16.0), 0.0)),
        material=frame_mat,
        name="front_brace_0",
    )
    base.visual(
        Box((0.032, 0.014, 0.115)),
        origin=Origin(xyz=(-0.040, -0.050, 0.091), rpy=(0.0, math.radians(-16.0), 0.0)),
        material=frame_mat,
        name="front_brace_1",
    )

    # Gauge pod integrated into the base frame.
    gauge_center = (0.105, 0.0)
    base.visual(
        Cylinder(radius=0.068, length=0.032),
        origin=Origin(xyz=(gauge_center[0], gauge_center[1], 0.056)),
        material=frame_mat,
        name="gauge_body",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.004),
        origin=Origin(xyz=(gauge_center[0], gauge_center[1], 0.074)),
        material=white_mat,
        name="gauge_face",
    )
    base.visual(
        mesh_from_geometry(TorusGeometry(radius=0.057, tube=0.006, radial_segments=16, tubular_segments=72), "gauge_bezel"),
        origin=Origin(xyz=(gauge_center[0], gauge_center[1], 0.078)),
        material=steel_mat,
        name="gauge_bezel",
    )
    # Ticks are shallow raised marks on the face; they make the round part read
    # unmistakably as a pressure gauge while staying fused to the face visual.
    for idx, angle in enumerate([math.radians(a) for a in (-140, -105, -70, -35, 0, 35, 70, 105, 140)]):
        r = 0.039
        base.visual(
            Box((0.011, 0.0022, 0.0012)),
            origin=Origin(
                xyz=(gauge_center[0] + r * math.cos(angle), gauge_center[1] + r * math.sin(angle), 0.0762),
                rpy=(0.0, 0.0, angle),
            ),
            material=needle_mat,
            name=f"gauge_tick_{idx}",
        )

    handle_rod = model.part("handle_rod")
    handle_rod.visual(
        Cylinder(radius=0.0105, length=0.335),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=steel_mat,
        name="pump_rod",
    )
    handle_rod.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel_mat,
        name="sliding_stop_collar",
    )
    handle_rod.visual(
        Cylinder(radius=0.017, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_mat,
        name="t_handle_bar",
    )
    handle_rod.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.104, 0.105)),
        material=rubber_mat,
        name="handle_end_0",
    )
    handle_rod.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, -0.104, 0.105)),
        material=rubber_mat,
        name="handle_end_1",
    )
    handle_rod.visual(
        Box((0.030, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=frame_mat,
        name="handle_hub",
    )

    needle = model.part("gauge_needle")
    needle.visual(
        Box((0.045, 0.004, 0.0025)),
        origin=Origin(xyz=(0.023, 0.0, 0.0016)),
        material=needle_mat,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_mat,
        name="pivot_cap",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.0085, length=0.019),
        origin=Origin(xyz=(0.0095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red_mat,
        name="button_plunger",
    )

    model.articulation(
        "barrel_to_handle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=handle_rod,
        origin=Origin(xyz=(-0.075, 0.0, 0.372)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.160),
    )
    model.articulation(
        "gauge_to_needle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=needle,
        origin=Origin(xyz=(gauge_center[0], gauge_center[1], 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=5.0, lower=-0.65, upper=2.45),
    )
    model.articulation(
        "gauge_to_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=release_button,
        origin=Origin(xyz=(gauge_center[0] + 0.068, gauge_center[1], 0.061)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.007),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    handle = object_model.get_part("handle_rod")
    needle = object_model.get_part("gauge_needle")
    button = object_model.get_part("release_button")
    slide = object_model.get_articulation("barrel_to_handle")
    needle_joint = object_model.get_articulation("gauge_to_needle")
    button_joint = object_model.get_articulation("gauge_to_button")

    ctx.expect_within(
        handle,
        base,
        axes="xy",
        inner_elem="pump_rod",
        outer_elem="barrel_tube",
        margin=0.002,
        name="rod is centered in the short barrel",
    )
    ctx.expect_overlap(
        handle,
        base,
        axes="z",
        elem_a="pump_rod",
        elem_b="barrel_tube",
        min_overlap=0.20,
        name="collapsed rod remains inserted in barrel",
    )
    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({slide: 0.160}):
        ctx.expect_within(
            handle,
            base,
            axes="xy",
            inner_elem="pump_rod",
            outer_elem="barrel_tube",
            margin=0.002,
            name="extended rod stays aligned with barrel",
        )
        ctx.expect_overlap(
            handle,
            base,
            axes="z",
            elem_a="pump_rod",
            elem_b="barrel_tube",
            min_overlap=0.075,
            name="extended rod still has retained insertion",
        )
        extended_handle = ctx.part_world_position(handle)
    ctx.check(
        "handle slides upward along barrel",
        rest_handle is not None and extended_handle is not None and extended_handle[2] > rest_handle[2] + 0.15,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    ctx.expect_gap(
        needle,
        base,
        axis="z",
        min_gap=0.0005,
        max_gap=0.006,
        positive_elem="pointer",
        negative_elem="gauge_face",
        name="needle floats just above gauge face",
    )
    needle_aabb_0 = ctx.part_element_world_aabb(needle, elem="pointer")
    with ctx.pose({needle_joint: math.pi / 2.0}):
        needle_aabb_90 = ctx.part_element_world_aabb(needle, elem="pointer")
    ctx.check(
        "gauge needle rotates about central pivot",
        needle_aabb_0 is not None
        and needle_aabb_90 is not None
        and (needle_aabb_0[1][0] - needle_aabb_0[0][0]) > (needle_aabb_0[1][1] - needle_aabb_0[0][1]) * 4.0
        and (needle_aabb_90[1][1] - needle_aabb_90[0][1]) > (needle_aabb_90[1][0] - needle_aabb_90[0][0]) * 4.0,
        details=f"rest={needle_aabb_0}, posed={needle_aabb_90}",
    )

    ctx.expect_contact(
        button,
        base,
        elem_a="button_plunger",
        elem_b="gauge_body",
        contact_tol=0.001,
        name="release button is mounted on gauge housing",
    )
    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.007}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "release button moves as a small push control",
        rest_button is not None and pressed_button is not None and pressed_button[0] > rest_button[0] + 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
