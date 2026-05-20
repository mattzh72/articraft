from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BUTTON_POSITIONS = (
    (0.0, 0.0110),
    (0.0110, 0.0),
    (0.0, -0.0110),
    (-0.0110, 0.0),
)


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").circle(0.044).extrude(0.0065)
    shell = shell.faces(">Z").workplane().circle(0.032).extrude(0.0115)
    shell = (
        shell.faces(">Z")
        .workplane()
        .pushPoints(BUTTON_POSITIONS)
        .circle(0.0056)
        .cutBlind(-0.0018)
    )
    shell = (
        shell.faces(">Z")
        .workplane()
        .pushPoints(BUTTON_POSITIONS)
        .circle(0.0042)
        .cutBlind(-0.0060)
    )
    return shell


def _build_ring_shell() -> cq.Workplane:
    lower_band = (
        cq.Workplane("XY")
        .circle(0.0455)
        .circle(0.0338)
        .extrude(0.0080)
        .translate((0.0, 0.0, 0.0065))
    )
    upper_band = (
        cq.Workplane("XY")
        .circle(0.0445)
        .circle(0.0348)
        .extrude(0.0035)
        .translate((0.0, 0.0, 0.0145))
    )
    return lower_band.union(upper_band)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="audio_remote")

    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    ring_satin = model.material("ring_satin", rgba=(0.64, 0.66, 0.69, 1.0))
    button_dark = model.material("button_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "audio_remote_body"),
        material=shell_dark,
        name="body_shell",
    )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_build_ring_shell(), "audio_remote_ring"),
        material=ring_satin,
        name="ring_shell",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=10.0),
    )

    for index, (x_pos, y_pos) in enumerate(BUTTON_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.0052, length=0.0024),
            origin=Origin(xyz=(0.0, 0.0, 0.0012)),
            material=button_dark,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0040, length=0.0054),
            origin=Origin(xyz=(0.0, 0.0, -0.0027)),
            material=button_dark,
            name="button_stem",
        )

        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, 0.0180)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0016,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    ring_joint = object_model.get_articulation("body_to_ring")

    ctx.allow_overlap(
        body,
        ring,
        elem_a="body_shell",
        elem_b="ring_shell",
        reason="The rotating outer ring is intentionally represented as riding on a concealed bearing track in the body shell.",
    )

    ring_limits = ring_joint.motion_limits
    ctx.check(
        "ring_joint_is_continuous",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS
        and ring_limits is not None
        and ring_limits.lower is None
        and ring_limits.upper is None,
        details=f"joint_type={ring_joint.articulation_type}, limits={ring_limits!r}",
    )
    ctx.expect_origin_distance(
        ring,
        body,
        axes="xy",
        max_dist=1e-6,
        name="ring stays centered on the body axis",
    )
    ctx.expect_overlap(
        ring,
        body,
        axes="xy",
        elem_a="ring_shell",
        elem_b="body_shell",
        min_overlap=0.060,
        name="ring footprint wraps around the body",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits

        ctx.allow_isolated_part(
            button,
            reason="Each push button is intentionally retained inside a clearanced guide bore that stands in for the hidden switch mechanism.",
        )
        ctx.check(
            f"button_{index}_is_prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and button_limits is not None
            and button_limits.lower == 0.0
            and button_limits.upper is not None
            and 0.0012 <= button_limits.upper <= 0.0020,
            details=f"joint_type={button_joint.articulation_type}, limits={button_limits!r}",
        )

        rest_position = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper if button_limits is not None else 0.0}):
            pressed_position = ctx.part_world_position(button)
            ctx.check(
                f"button_{index}_depresses_downward",
                rest_position is not None
                and pressed_position is not None
                and pressed_position[2] < rest_position[2] - 0.001,
                details=f"rest={rest_position}, pressed={pressed_position}",
            )
            ctx.expect_overlap(
                button,
                body,
                axes="xy",
                elem_a="button_cap",
                elem_b="body_shell",
                min_overlap=0.009,
                name=f"button_{index} stays nested over the center cluster",
            )

    return ctx.report()


object_model = build_object_model()
