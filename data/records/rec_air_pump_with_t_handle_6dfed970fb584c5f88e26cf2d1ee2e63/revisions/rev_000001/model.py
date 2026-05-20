from __future__ import annotations

import math

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
)


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder whose local Z axis spans start -> end in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_workshop_floor_pump")

    painted_red = Material("painted_red", color=(0.75, 0.05, 0.03, 1.0))
    satin_steel = Material("satin_steel", color=(0.68, 0.68, 0.64, 1.0))
    dark_steel = Material("dark_steel", color=(0.07, 0.08, 0.085, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    gauge_face = Material("warm_gauge_face", color=(0.93, 0.90, 0.82, 1.0))
    needle_red = Material("needle_red", color=(0.95, 0.05, 0.02, 1.0))
    brass = Material("brass_axle", color=(0.86, 0.61, 0.22, 1.0))

    frame = model.part("frame")

    # Wide stamped base with rubber foot pads.
    frame.visual(
        Box((0.56, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=painted_red,
        name="wide_base",
    )
    frame.visual(
        Box((0.18, 0.15, 0.012)),
        origin=Origin(xyz=(-0.16, 0.0, 0.031)),
        material=rubber,
        name="foot_pad_0",
    )
    frame.visual(
        Box((0.18, 0.15, 0.012)),
        origin=Origin(xyz=(0.16, 0.0, 0.031)),
        material=rubber,
        name="foot_pad_1",
    )
    frame.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(0.0, 0.02, 0.040)),
        material=painted_red,
        name="barrel_socket",
    )

    # Vertical pump barrel and its upper gland.
    frame.visual(
        Cylinder(radius=0.035, length=0.700),
        origin=Origin(xyz=(0.0, 0.02, 0.385)),
        material=satin_steel,
        name="vertical_barrel",
    )
    frame.visual(
        Cylinder(radius=0.044, length=0.040),
        origin=Origin(xyz=(0.0, 0.02, 0.755)),
        material=dark_steel,
        name="top_collar",
    )

    # Simple tubular stays that keep the barrel upright.
    _cylinder_between(
        frame,
        (-0.235, 0.045, 0.035),
        (-0.034, 0.020, 0.575),
        0.007,
        material=dark_steel,
        name="support_tube_0",
    )
    _cylinder_between(
        frame,
        (0.235, 0.045, 0.035),
        (0.034, 0.020, 0.575),
        0.007,
        material=dark_steel,
        name="support_tube_1",
    )
    _cylinder_between(
        frame,
        (0.0, 0.080, 0.035),
        (0.0, 0.052, 0.610),
        0.006,
        material=dark_steel,
        name="rear_stay",
    )
    _cylinder_between(
        frame,
        (-0.170, 0.045, 0.105),
        (0.170, 0.045, 0.105),
        0.006,
        material=dark_steel,
        name="lower_cross_tube",
    )

    # Short side peg with an upturned lip for wrapping the hose.
    _cylinder_between(
        frame,
        (0.020, 0.000, 0.500),
        (0.120, -0.006, 0.500),
        0.007,
        material=dark_steel,
        name="hose_hook_peg",
    )
    _cylinder_between(
        frame,
        (0.120, -0.006, 0.496),
        (0.120, -0.006, 0.545),
        0.007,
        material=dark_steel,
        name="hose_hook_lip",
    )
    frame.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.120, -0.006, 0.548)),
        material=dark_steel,
        name="hook_rounded_tip",
    )

    # Pressure gauge housing on a neck from the barrel.
    _cylinder_between(
        frame,
        (0.0, -0.040, 0.340),
        (0.0, -0.015, 0.340),
        0.016,
        material=dark_steel,
        name="gauge_neck",
    )
    frame.visual(
        Cylinder(radius=0.073, length=0.030),
        origin=Origin(xyz=(0.0, -0.055, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="gauge_body",
    )
    frame.visual(
        Cylinder(radius=0.061, length=0.004),
        origin=Origin(xyz=(0.0, -0.072, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_dial",
    )
    for i, angle_deg in enumerate((-120, -80, -40, 0, 40, 80, 120)):
        angle = math.radians(angle_deg)
        x = 0.047 * math.sin(angle)
        z = 0.340 + 0.047 * math.cos(angle)
        frame.visual(
            Box((0.004, 0.001, 0.014)),
            origin=Origin(xyz=(x, -0.0745, z), rpy=(0.0, angle, 0.0)),
            material=dark_steel,
            name=f"gauge_tick_{i}",
        )
    frame.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.075, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="needle_axle",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, -0.055, 0.413)),
        material=dark_steel,
        name="button_socket",
    )

    handle = model.part("handle")
    _cylinder_between(
        handle,
        (0.0, 0.0, 0.000),
        (0.0, 0.0, 0.370),
        0.006,
        material=satin_steel,
        name="piston_rod",
    )
    _cylinder_between(
        handle,
        (-0.145, 0.0, 0.372),
        (0.145, 0.0, 0.372),
        0.018,
        material=rubber,
        name="t_grip",
    )
    handle.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.372)),
        material=dark_steel,
        name="handle_boss",
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="needle_hub",
    )
    needle.visual(
        Box((0.005, 0.002, 0.048)),
        origin=Origin(xyz=(0.0, -0.003, 0.024)),
        material=needle_red,
        name="needle_pointer",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=needle_red,
        name="button_cap",
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.02, 0.775)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.7, lower=0.0, upper=0.18),
    )
    model.articulation(
        "gauge_to_needle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=needle,
        origin=Origin(xyz=(0.0, -0.080, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=4.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "gauge_to_button",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=release_button,
        origin=Origin(xyz=(0.0, -0.055, 0.417)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    needle = object_model.get_part("needle")
    release_button = object_model.get_part("release_button")
    handle_slide = object_model.get_articulation("frame_to_handle")
    needle_sweep = object_model.get_articulation("gauge_to_needle")
    button_push = object_model.get_articulation("gauge_to_button")

    ctx.check(
        "primary mechanisms are articulated",
        handle_slide.articulation_type == ArticulationType.PRISMATIC
        and needle_sweep.articulation_type == ArticulationType.REVOLUTE
        and button_push.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.expect_gap(
        handle,
        frame,
        axis="z",
        positive_elem="piston_rod",
        negative_elem="top_collar",
        max_gap=0.001,
        max_penetration=0.0005,
        name="piston rod sits on barrel axis collar",
    )
    ctx.expect_gap(
        release_button,
        frame,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_socket",
        max_gap=0.001,
        max_penetration=0.0005,
        name="release button is separate on gauge housing",
    )
    ctx.expect_gap(
        frame,
        needle,
        axis="y",
        positive_elem="needle_axle",
        negative_elem="needle_hub",
        max_gap=0.001,
        max_penetration=0.0005,
        name="gauge needle rides on central axle",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.18}):
        raised_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "T-handle slides upward on barrel axis",
        rest_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] > rest_handle_pos[2] + 0.17,
        details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
    )

    rest_pointer = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    with ctx.pose({needle_sweep: 1.0}):
        swept_pointer = ctx.part_element_world_aabb(needle, elem="needle_pointer")
    ctx.check(
        "gauge needle visibly rotates",
        rest_pointer is not None
        and swept_pointer is not None
        and swept_pointer[1][0] > rest_pointer[1][0] + 0.025,
        details=f"rest={rest_pointer}, swept={swept_pointer}",
    )

    rest_button_pos = ctx.part_world_position(release_button)
    with ctx.pose({button_push: 0.006}):
        pressed_button_pos = ctx.part_world_position(release_button)
    ctx.check(
        "release button pushes into gauge housing",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.005,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
