from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.118
BODY_HEIGHT = 0.082
PLATE_DEPTH = 0.002
SHELL_DEPTH = 0.020
FACE_DEPTH = 0.002
FRONT_Z = PLATE_DEPTH + SHELL_DEPTH + FACE_DEPTH

SLIDER_X = -0.034
DIAL_X = 0.028


def _rounded_panel_mesh(width: float, height: float, depth: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, height, radius),
            depth,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    housing = model.material("housing", rgba=(0.92, 0.92, 0.89, 1.0))
    trim = model.material("trim", rgba=(0.84, 0.84, 0.82, 1.0))
    wall_plate = model.material("wall_plate", rgba=(0.88, 0.88, 0.86, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    slot_grey = model.material("slot_grey", rgba=(0.40, 0.41, 0.43, 1.0))
    pointer = model.material("pointer", rgba=(0.74, 0.18, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.126, 0.090, PLATE_DEPTH)),
        origin=Origin(xyz=(0.0, 0.0, PLATE_DEPTH * 0.5)),
        material=wall_plate,
        name="mount_plate",
    )
    body.visual(
        _rounded_panel_mesh(BODY_WIDTH, BODY_HEIGHT, SHELL_DEPTH, 0.012, "housing_shell"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_DEPTH)),
        material=housing,
        name="housing_shell",
    )
    body.visual(
        _rounded_panel_mesh(0.112, 0.076, FACE_DEPTH, 0.010, "faceplate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_DEPTH + SHELL_DEPTH)),
        material=trim,
        name="faceplate",
    )
    body.visual(
        Box((0.013, 0.040, 0.0012)),
        origin=Origin(xyz=(SLIDER_X, 0.0, FRONT_Z - 0.0006)),
        material=slot_grey,
        name="slot_track",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.0016),
        origin=Origin(xyz=(DIAL_X, 0.0, FRONT_Z - 0.0008)),
        material=slot_grey,
        name="dial_boss",
    )
    body.visual(
        Box((0.024, 0.006, 0.001)),
        origin=Origin(xyz=(DIAL_X, 0.027, FRONT_Z - 0.0005)),
        material=graphite,
        name="mode_label_strip",
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        Box((0.011, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=graphite,
        name="slider_shoe",
    )
    mode_slider.visual(
        Box((0.014, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="slider_thumb",
    )
    mode_slider.visual(
        Box((0.004, 0.018, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim,
        name="slider_rib",
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.040,
            0.012,
            body_style="tapered",
            top_diameter=0.034,
            edge_radius=0.001,
            side_draft_deg=8.0,
            center=False,
        ),
        "temperature_dial",
    )

    temperature_dial = model.part("temperature_dial")
    temperature_dial.visual(
        Cylinder(radius=0.0035, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=graphite,
        name="shaft",
    )
    temperature_dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=graphite,
        name="dial_shell",
    )
    temperature_dial.visual(
        Box((0.003, 0.014, 0.0014)),
        origin=Origin(xyz=(0.0, 0.006, 0.0163)),
        material=pointer,
        name="dial_pointer",
    )

    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(SLIDER_X, 0.0, FRONT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "body_to_temperature_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=temperature_dial,
        origin=Origin(xyz=(DIAL_X, 0.0, FRONT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    mode_slider = object_model.get_part("mode_slider")
    temperature_dial = object_model.get_part("temperature_dial")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    dial_joint = object_model.get_articulation("body_to_temperature_dial")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected thermostat body bounds.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "body_home_wall_scale",
            0.115 <= size[0] <= 0.130 and 0.080 <= size[1] <= 0.095 and 0.023 <= size[2] <= 0.026,
            details=f"size={size!r}",
        )

    ctx.expect_gap(
        mode_slider,
        body,
        axis="z",
        positive_elem="slider_shoe",
        negative_elem="faceplate",
        max_gap=0.0005,
        max_penetration=0.0,
        name="slider shoe sits flush on the faceplate",
    )
    ctx.expect_within(
        mode_slider,
        body,
        axes="xy",
        inner_elem="slider_shoe",
        outer_elem="slot_track",
        margin=0.001,
        name="slider shoe stays within the guide at rest",
    )

    slider_rest = ctx.part_world_position(mode_slider)
    slider_lower = None
    slider_upper = None
    limits = slider_joint.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({slider_joint: limits.lower}):
            ctx.expect_within(
                mode_slider,
                body,
                axes="xy",
                inner_elem="slider_shoe",
                outer_elem="slot_track",
                margin=0.001,
                name="slider shoe stays within the guide at lower travel",
            )
            slider_lower = ctx.part_world_position(mode_slider)
        with ctx.pose({slider_joint: limits.upper}):
            ctx.expect_within(
                mode_slider,
                body,
                axes="xy",
                inner_elem="slider_shoe",
                outer_elem="slot_track",
                margin=0.001,
                name="slider shoe stays within the guide at upper travel",
            )
            slider_upper = ctx.part_world_position(mode_slider)

    ctx.check(
        "slider translates through short vertical travel",
        slider_rest is not None
        and slider_lower is not None
        and slider_upper is not None
        and slider_lower[1] < slider_rest[1] - 0.005
        and slider_upper[1] > slider_rest[1] + 0.005
        and slider_upper[1] - slider_lower[1] > 0.020,
        details=f"rest={slider_rest}, lower={slider_lower}, upper={slider_upper}",
    )

    ctx.expect_gap(
        temperature_dial,
        body,
        axis="z",
        positive_elem="shaft",
        negative_elem="dial_boss",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial shaft starts at the boss face",
    )
    ctx.expect_within(
        temperature_dial,
        body,
        axes="xy",
        inner_elem="shaft",
        outer_elem="dial_boss",
        margin=0.0002,
        name="dial shaft stays centered on the boss",
    )

    dial_rest = ctx.part_world_position(temperature_dial)
    with ctx.pose({dial_joint: math.pi * 0.5}):
        ctx.expect_gap(
            temperature_dial,
            body,
            axis="z",
            positive_elem="shaft",
            negative_elem="dial_boss",
            max_gap=0.0005,
            max_penetration=0.0,
            name="dial remains seated while rotated",
        )
        dial_quarter_turn = ctx.part_world_position(temperature_dial)

    ctx.check(
        "dial rotates in place",
        dial_rest is not None
        and dial_quarter_turn is not None
        and max(abs(dial_rest[i] - dial_quarter_turn[i]) for i in range(3)) < 1e-6,
        details=f"rest={dial_rest}, rotated={dial_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
