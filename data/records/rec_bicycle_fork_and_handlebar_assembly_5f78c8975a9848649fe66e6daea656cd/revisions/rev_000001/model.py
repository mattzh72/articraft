from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_bike_fork")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.82, 0.84, 1.0))
    dark_thread = model.material("oiled_thread_shadow", rgba=(0.20, 0.22, 0.23, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    stem = model.part("quill_stem")

    stem.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=chrome,
        name="quill_shaft",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=chrome,
        name="top_expander_cap",
    )

    stem_neck = tube_from_spline_points(
        [
            (0.020, 0.0, 0.460),
            (0.055, 0.0, 0.468),
            (0.120, 0.0, 0.475),
            (0.175, 0.0, 0.478),
        ],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=24,
    )
    stem.visual(
        mesh_from_geometry(stem_neck, "quill_stem_neck"),
        material=chrome,
        name="stem_neck",
    )

    stem.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.180, 0.0, 0.482), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="bar_clamp",
    )

    cruiser_bar = tube_from_spline_points(
        [
            (-0.225, -0.355, 0.522),
            (-0.225, -0.255, 0.522),
            (-0.105, -0.185, 0.530),
            (0.030, -0.105, 0.515),
            (0.180, 0.000, 0.482),
            (0.030, 0.105, 0.515),
            (-0.105, 0.185, 0.530),
            (-0.225, 0.255, 0.522),
            (-0.225, 0.355, 0.522),
        ],
        radius=0.0105,
        samples_per_segment=16,
        radial_segments=28,
        up_hint=(0.0, 0.0, 1.0),
    )
    stem.visual(
        mesh_from_geometry(cruiser_bar, "swept_cruiser_bar"),
        material=chrome,
        name="swept_bar",
    )

    for index, y in enumerate((-0.305, 0.305)):
        stem.visual(
            Cylinder(radius=0.016, length=0.105),
            origin=Origin(xyz=(-0.225, y, 0.522), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"grip_{index}",
        )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.016, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=chrome,
        name="steerer_tube",
    )

    for index, z in enumerate([0.105 + i * 0.014 for i in range(22)]):
        fork.visual(
            Cylinder(radius=0.0184, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_thread if index % 2 else chrome,
            name=f"thread_{index}",
        )

    crown = tube_from_spline_points(
        [
            (0.004, -0.060, -0.070),
            (0.000, -0.030, -0.048),
            (0.000, 0.000, -0.036),
            (0.000, 0.030, -0.048),
            (0.004, 0.060, -0.070),
        ],
        radius=0.020,
        samples_per_segment=14,
        radial_segments=28,
    )
    fork.visual(
        mesh_from_geometry(crown, "smooth_unicrown"),
        material=chrome,
        name="unicrown",
    )

    for index, side in enumerate((-1.0, 1.0)):
        blade = tube_from_spline_points(
            [
                (0.004, side * 0.040, -0.065),
                (0.016, side * 0.052, -0.160),
                (0.034, side * 0.058, -0.295),
                (0.056, side * 0.060, -0.425),
            ],
            radius=0.0115,
            samples_per_segment=18,
            radial_segments=24,
        )
        fork.visual(
            mesh_from_geometry(blade, f"fork_blade_{index}"),
            material=chrome,
            name=f"blade_{index}",
        )
        fork.visual(
            Box(size=(0.052, 0.008, 0.062)),
            origin=Origin(xyz=(0.062, side * 0.061, -0.446)),
            material=chrome,
            name=f"dropout_{index}",
        )
        fork.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(0.060, side * 0.067, -0.450), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_thread,
            name=f"axle_slot_{index}",
        )

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-1.15, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stem = object_model.get_part("quill_stem")
    fork = object_model.get_part("fork")
    steering = object_model.get_articulation("steering_axis")

    ctx.allow_overlap(
        stem,
        fork,
        elem_a="quill_shaft",
        elem_b="steerer_tube",
        reason=(
            "The quill shaft is intentionally captured inside the simplified "
            "solid threaded steerer proxy to show the steering spindle fit."
        ),
    )
    ctx.expect_within(
        stem,
        fork,
        axes="xy",
        inner_elem="quill_shaft",
        outer_elem="steerer_tube",
        margin=0.001,
        name="quill shaft is centered inside the steerer",
    )
    ctx.expect_overlap(
        stem,
        fork,
        axes="z",
        elem_a="quill_shaft",
        elem_b="steerer_tube",
        min_overlap=0.30,
        name="quill shaft remains deeply inserted in steerer",
    )

    grip_0 = ctx.part_element_world_aabb(stem, elem="grip_0")
    grip_1 = ctx.part_element_world_aabb(stem, elem="grip_1")
    clamp = ctx.part_element_world_aabb(stem, elem="bar_clamp")
    if grip_0 is not None and grip_1 is not None and clamp is not None:
        grip_0_center_y = (grip_0[0][1] + grip_0[1][1]) / 2.0
        grip_1_center_y = (grip_1[0][1] + grip_1[1][1]) / 2.0
        ctx.check(
            "cruiser bars are wide",
            abs(grip_1_center_y - grip_0_center_y) > 0.55,
            details=f"grip_y=({grip_0_center_y:.3f}, {grip_1_center_y:.3f})",
        )
        ctx.check(
            "grips sweep behind clamp",
            max(grip_0[1][0], grip_1[1][0]) < clamp[0][0] - 0.25,
            details=f"grip_max_x={max(grip_0[1][0], grip_1[1][0]):.3f}, clamp_min_x={clamp[0][0]:.3f}",
        )
    else:
        ctx.fail("swept handlebar geometry is measurable", "missing grip or clamp AABB")

    rest_dropout = ctx.part_element_world_aabb(fork, elem="dropout_1")
    with ctx.pose({steering: 0.80}):
        turned_dropout = ctx.part_element_world_aabb(fork, elem="dropout_1")
    if rest_dropout is not None and turned_dropout is not None:
        rest_center = (
            (rest_dropout[0][0] + rest_dropout[1][0]) / 2.0,
            (rest_dropout[0][1] + rest_dropout[1][1]) / 2.0,
        )
        turned_center = (
            (turned_dropout[0][0] + turned_dropout[1][0]) / 2.0,
            (turned_dropout[0][1] + turned_dropout[1][1]) / 2.0,
        )
        dx = turned_center[0] - rest_center[0]
        dy = turned_center[1] - rest_center[1]
        ctx.check(
            "fork dropouts swing around steering axis",
            math.hypot(dx, dy) > 0.035,
            details=f"rest={rest_center}, turned={turned_center}",
        )
    else:
        ctx.fail("steering pose is measurable", "missing dropout AABB")

    return ctx.report()


object_model = build_object_model()
