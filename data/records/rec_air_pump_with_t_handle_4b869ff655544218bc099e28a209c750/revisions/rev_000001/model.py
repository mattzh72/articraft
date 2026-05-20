from __future__ import annotations

from math import cos, pi, sin

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_pump")

    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    white = model.material("gauge_face", rgba=(0.92, 0.90, 0.84, 1.0))
    red = model.material("red_needle", rgba=(0.85, 0.02, 0.01, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.42, 0.16, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black,
        name="foot_plate",
    )
    body.visual(
        Box((0.09, 0.08, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_steel,
        name="pump_head",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=steel,
        name="barrel",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=dark_steel,
        name="lower_band",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material=dark_steel,
        name="top_collar",
    )
    body.visual(
        Box((0.19, 0.045, 0.018)),
        origin=Origin(xyz=(-0.105, -0.002, 0.041)),
        material=rubber,
        name="foot_pad_0",
    )
    body.visual(
        Box((0.19, 0.045, 0.018)),
        origin=Origin(xyz=(0.105, -0.002, 0.041)),
        material=rubber,
        name="foot_pad_1",
    )

    # Round pressure gauge mounted on a short metal neck near the base.
    gauge_center = (0.0, -0.105, 0.160)
    body.visual(
        Cylinder(radius=0.008, length=0.068),
        origin=Origin(xyz=(0.0, -0.062, 0.160), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_neck",
    )
    body.visual(
        Cylinder(radius=0.057, length=0.018),
        origin=Origin(xyz=gauge_center, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="gauge_case",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.004),
        origin=Origin(xyz=(0.0, -0.116, 0.160), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=white,
        name="gauge_dial",
    )
    for i, angle in enumerate((-60.0, -30.0, 0.0, 30.0, 60.0)):
        theta = angle * pi / 180.0
        body.visual(
            Box((0.004, 0.002, 0.012)),
            origin=Origin(
                xyz=(0.036 * sin(theta), -0.119, 0.160 + 0.036 * cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=black,
            name=f"gauge_tick_{i}",
        )

    # Parked rubber hose: it is clipped against the barrel rather than floating.
    hose = tube_from_spline_points(
        [
            (0.029, -0.026, 0.128),
            (0.064, -0.039, 0.190),
            (0.052, -0.023, 0.350),
            (0.052, -0.023, 0.505),
            (0.030, -0.020, 0.610),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=18,
    )
    body.visual(
        mesh_from_geometry(hose, "parked_hose"),
        material=rubber,
        name="parked_hose",
    )
    body.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.026, -0.025, 0.126)),
        material=dark_steel,
        name="hose_port",
    )
    for i, z in enumerate((0.350, 0.505)):
        body.visual(
            Box((0.044, 0.020, 0.018)),
            origin=Origin(xyz=(0.039, -0.021, z)),
            material=black,
            name=f"hose_clip_{i}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=steel,
        name="piston_rod",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.292), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="crossbar",
    )
    handle.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=black,
        name="handle_boss",
    )

    grip = model.part("grip")
    grip.visual(
        Cylinder(radius=0.024, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="grip_sleeve",
    )
    grip.visual(
        Box((0.115, 0.007, 0.020)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=dark_steel,
        name="folding_tab",
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.005, 0.002, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=red,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pivot_cap",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.210),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.105, 0.0, 0.292)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_needle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=needle,
        origin=Origin(xyz=(0.0, -0.122, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=12.0, lower=-1.2, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")
    slide = object_model.get_articulation("body_to_handle")
    grip_spin = object_model.get_articulation("handle_to_grip")
    gauge_sweep = object_model.get_articulation("body_to_needle")

    ctx.allow_overlap(
        handle,
        grip,
        elem_a="crossbar",
        elem_b="grip_sleeve",
        reason="The folding grip is an intentionally captured rubber sleeve rotating around the handle crossbar.",
    )
    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="crossbar",
        outer_elem="grip_sleeve",
        margin=0.001,
        name="crossbar is inside the supported grip sleeve",
    )
    ctx.expect_overlap(
        handle,
        grip,
        axes="x",
        elem_a="crossbar",
        elem_b="grip_sleeve",
        min_overlap=0.120,
        name="grip sleeve has retained length on crossbar",
    )
    ctx.expect_within(
        handle,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel",
        margin=0.0,
        name="piston rod is centered on barrel axis",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="piston_rod",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="piston rod emerges from top collar",
    )
    ctx.expect_contact(
        needle,
        body,
        elem_a="pivot_cap",
        elem_b="gauge_dial",
        contact_tol=0.001,
        name="gauge needle pivots on dial center",
    )

    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({slide: 0.210}):
        raised_handle = ctx.part_world_position(handle)
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="barrel",
            margin=0.0,
            name="raised piston rod stays on barrel axis",
        )
    ctx.check(
        "handle translates along barrel axis",
        rest_handle is not None
        and raised_handle is not None
        and abs(raised_handle[0] - rest_handle[0]) < 1e-6
        and abs(raised_handle[1] - rest_handle[1]) < 1e-6
        and raised_handle[2] > rest_handle[2] + 0.19,
        details=f"rest={rest_handle}, raised={raised_handle}",
    )
    ctx.check(
        "grip rotates continuously about crossbar",
        grip_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(grip_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={grip_spin.articulation_type}, axis={grip_spin.axis}",
    )
    ctx.check(
        "gauge needle has limited pivot sweep",
        gauge_sweep.articulation_type == ArticulationType.REVOLUTE
        and tuple(gauge_sweep.axis) == (0.0, 1.0, 0.0)
        and gauge_sweep.motion_limits is not None
        and gauge_sweep.motion_limits.lower < 0.0
        and gauge_sweep.motion_limits.upper > 0.0,
        details=f"type={gauge_sweep.articulation_type}, axis={gauge_sweep.axis}, limits={gauge_sweep.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
