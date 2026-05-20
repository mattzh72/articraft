from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)
import cadquery as cq


HANDLE_LENGTH = 0.170
HANDLE_RADIUS = 0.018
CHANNEL_SPACING = 0.030
CHANNEL_XS = tuple((index - 2) * CHANNEL_SPACING for index in range(5))
CHANNEL_RADIUS = 0.0052
ROD_RADIUS = 0.0032
ROD_TRAVEL = 0.010


def _build_handle_body() -> cq.Workplane:
    """Round handle with five bored vertical channels and raised guide collars."""
    body = (
        cq.Workplane("YZ")
        .circle(HANDLE_RADIUS)
        .extrude(HANDLE_LENGTH)
        .translate((-HANDLE_LENGTH * 0.5, 0.0, 0.0))
    )

    collar_outer_radius = 0.0084
    collar_height = 0.0032
    collar_embed = 0.0020
    for x in CHANNEL_XS:
        collar = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(collar_outer_radius)
            .extrude(collar_height + collar_embed)
            .translate((0.0, 0.0, HANDLE_RADIUS - collar_embed))
        )
        body = body.union(collar)

    cut_height = 2.0 * HANDLE_RADIUS + collar_height + 0.010
    for x in CHANNEL_XS:
        cutter = (
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(CHANNEL_RADIUS)
            .extrude(cut_height)
            .translate((0.0, 0.0, -HANDLE_RADIUS - 0.005))
        )
        body = body.cut(cutter)

    return body


def _build_return_spring() -> object:
    """Small exposed compression spring that sits around each button stem."""
    points: list[tuple[float, float, float]] = []
    turns = 4.5
    samples = 88
    spring_radius = 0.0060
    spring_height = 0.0096
    for index in range(samples + 1):
        t = index / samples
        angle = 2.0 * math.pi * turns * t
        points.append(
            (
                spring_radius * math.cos(angle),
                spring_radius * math.sin(angle),
                spring_height * t,
            )
        )
    return tube_from_spline_points(
        points,
        radius=0.00045,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_click_fidget_stick")

    handle_plastic = model.material("satin_teal_plastic", rgba=(0.04, 0.44, 0.53, 1.0))
    rubber_dark = model.material("dark_rubber", rgba=(0.035, 0.038, 0.042, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    cap_materials: tuple[Material, ...] = (
        model.material("button_red", rgba=(0.86, 0.12, 0.10, 1.0)),
        model.material("button_orange", rgba=(0.96, 0.46, 0.08, 1.0)),
        model.material("button_yellow", rgba=(0.95, 0.78, 0.12, 1.0)),
        model.material("button_green", rgba=(0.15, 0.70, 0.28, 1.0)),
        model.material("button_purple", rgba=(0.45, 0.24, 0.78, 1.0)),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_body(), "handle_body", tolerance=0.00035),
        material=handle_plastic,
        name="handle_shell",
    )
    handle.visual(
        Cylinder(radius=HANDLE_RADIUS * 0.92, length=0.006),
        origin=Origin(xyz=(-HANDLE_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_dark,
        name="end_cap_0",
    )
    handle.visual(
        Cylinder(radius=HANDLE_RADIUS * 0.92, length=0.006),
        origin=Origin(xyz=(HANDLE_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_dark,
        name="end_cap_1",
    )

    spring_mesh = mesh_from_geometry(_build_return_spring(), "button_return_spring")
    for index, x in enumerate(CHANNEL_XS):
        handle.visual(
            spring_mesh,
            origin=Origin(xyz=(x, 0.0, HANDLE_RADIUS + 0.0030)),
            material=steel,
            name=f"spring_{index}",
        )

    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=HANDLE_RADIUS, length=HANDLE_LENGTH),
        mass=0.11,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    for index, x in enumerate(CHANNEL_XS):
        rod = model.part(f"rod_{index}")
        rod.visual(
            Cylinder(radius=ROD_RADIUS, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name="stem",
        )
        rod.visual(
            Cylinder(radius=0.0070, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.016), rpy=(0.0, 0.0, 0.0)),
            material=cap_materials[index],
            name="cap",
        )
        rod.inertial = Inertial.from_geometry(
            Box((0.014, 0.014, 0.032)),
            mass=0.009,
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
        )
        model.articulation(
            f"handle_to_rod_{index}",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=rod,
            origin=Origin(xyz=(x, 0.0, HANDLE_RADIUS)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.18, lower=0.0, upper=ROD_TRAVEL),
            motion_properties=MotionProperties(damping=0.08, friction=0.01),
            meta={"spring_return": True, "rest_position": 0.0},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(f"handle_to_rod_{index}") for index in range(5)]
    ctx.check(
        "five spring return prismatic rods",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 0.0, -1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and abs(joint.motion_limits.upper - ROD_TRAVEL) < 1e-9
            and joint.meta.get("spring_return") is True
            for joint in joints
        ),
        details="Each button rod should slide inward on a spring-return prismatic joint.",
    )

    joint_xs = [joint.origin.xyz[0] for joint in joints]
    spacings = [joint_xs[index + 1] - joint_xs[index] for index in range(4)]
    ctx.check(
        "channels equally spaced",
        all(abs(spacing - CHANNEL_SPACING) < 1e-9 for spacing in spacings),
        details=f"x positions={joint_xs}, spacings={spacings}",
    )

    for index in range(5):
        rod = object_model.get_part(f"rod_{index}")
        ctx.expect_within(
            rod,
            "handle",
            axes="xy",
            inner_elem="stem",
            outer_elem="handle_shell",
            margin=0.001,
            name=f"rod_{index} stem stays in its bored handle channel footprint",
        )

    rest_positions = [ctx.part_world_position(object_model.get_part(f"rod_{index}")) for index in range(5)]
    with ctx.pose({joint: ROD_TRAVEL for joint in joints}):
        pressed_positions = [
            ctx.part_world_position(object_model.get_part(f"rod_{index}")) for index in range(5)
        ]
    ctx.check(
        "buttons retract inward when pressed",
        all(
            rest is not None
            and pressed is not None
            and pressed[2] < rest[2] - (ROD_TRAVEL * 0.9)
            for rest, pressed in zip(rest_positions, pressed_positions)
        ),
        details=f"rest={rest_positions}, pressed={pressed_positions}",
    )

    return ctx.report()


object_model = build_object_model()
