from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_lamp")

    brass_color = Material(name="brass", rgba=(0.85, 0.75, 0.3, 1.0))
    black_metal = Material(name="black_metal", rgba=(0.1, 0.1, 0.1, 1.0))
    bulb_color = Material(name="bulb", rgba=(1.0, 0.95, 0.8, 1.0))
    shade_color = Material(name="shade", rgba=(0.9, 0.9, 0.85, 1.0))

    # CAD models
    base_profile = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(0.18, 0)
        .lineTo(0.18, 0.01)
        .lineTo(0.15, 0.03)
        .lineTo(0.02, 0.05)
        .lineTo(0, 0.05)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )

    shade_cq = (
        cq.Workplane("XZ")
        .moveTo(0, 0)
        .lineTo(0.02, 0)
        .lineTo(0.03, 0.02)
        .lineTo(0.12, 0.18)
        .lineTo(0, 0.18)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )
    shade_hollow = shade_cq.faces(">Z").shell(-0.002)

    # Base part
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_profile, "base_profile"),
        origin=Origin(xyz=(0, 0, 0)),
        name="floor_base",
        material=black_metal,
    )
    base.visual(
        Cylinder(radius=0.025, length=0.1),
        origin=Origin(xyz=(0, 0, 0.05)),
        name="stem_base",
        material=brass_color,
    )
    base.visual(
        Cylinder(radius=0.012, length=1.36),
        origin=Origin(xyz=(0, 0, 0.77)),
        name="stem",
        material=black_metal,
    )
    base.visual(
        Cylinder(radius=0.016, length=0.02),
        origin=Origin(xyz=(0, 0, 0.77)),
        name="stem_collar",
        material=brass_color,
    )
    base.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0, 0, 1.45)),
        name="stem_cap",
        material=brass_color,
    )
    base.visual(
        Cylinder(radius=0.015, length=0.005),
        origin=Origin(xyz=(0.12, 0, 0.035)),
        name="switch_housing",
        material=black_metal,
    )

    foot_switch = model.part("foot_switch")
    model.articulation(
        "switch_joint",
        ArticulationType.PRISMATIC,
        parent=base,
        child=foot_switch,
        origin=Origin(xyz=(0.12, 0, 0.035)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.005, upper=0.0),
    )
    foot_switch.visual(
        Cylinder(radius=0.01, length=0.01),
        origin=Origin(xyz=(0, 0, 0.005)),
        name="button",
        material=brass_color,
    )

    # Head mount (yaw)
    head_mount = model.part("head_mount")
    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head_mount,
        origin=Origin(xyz=(0, 0, 1.47)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-3.14, upper=3.14),
    )
    head_mount.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(0, 0, 0.02)),
        name="swivel_base",
        material=black_metal,
    )
    head_mount.visual(
        Cylinder(radius=0.008, length=0.2),
        origin=Origin(xyz=(0.1, 0, 0.025), rpy=(0, 1.5708, 0)),
        name="arm",
        material=brass_color,
    )
    head_mount.visual(
        Box((0.02, 0.036, 0.02)),
        origin=Origin(xyz=(0.2, 0, 0.025)),
        name="arm_end",
        material=black_metal,
    )
    head_mount.visual(
        Box((0.02, 0.005, 0.04)),
        origin=Origin(xyz=(0.2, -0.0175, 0.01)),
        name="left_bracket",
        material=black_metal,
    )
    head_mount.visual(
        Box((0.02, 0.005, 0.04)),
        origin=Origin(xyz=(0.2, 0.0175, 0.01)),
        name="right_bracket",
        material=black_metal,
    )

    # Head (pitch)
    head = model.part("head")
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=head_mount,
        child=head,
        origin=Origin(xyz=(0.2, 0, 0.0)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.5, upper=0.5),
    )
    head.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0, 0, 0), rpy=(1.5708, 0, 0)),
        name="hinge_pin",
        material=brass_color,
    )
    head.visual(
        Cylinder(radius=0.01, length=0.03),
        origin=Origin(xyz=(0, 0, -0.015)),
        name="neck",
        material=brass_color,
    )
    head.visual(
        mesh_from_cadquery(shade_hollow, "shade_hollow"),
        origin=Origin(xyz=(0, 0, -0.029), rpy=(3.14159, 0, 0)),
        name="shade",
        material=shade_color,
    )
    head.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0, 0, -0.055)),
        name="socket",
        material=black_metal,
    )
    head.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0, 0, -0.10)),
        name="bulb",
        material=bulb_color,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head_mount = object_model.get_part("head_mount")
    head = object_model.get_part("head")

    yaw = object_model.get_articulation("yaw_joint")
    pitch = object_model.get_articulation("pitch_joint")
    foot_switch = object_model.get_part("foot_switch")
    switch_joint = object_model.get_articulation("switch_joint")

    ctx.allow_overlap(
        foot_switch, base,
        elem_a="button", elem_b="floor_base",
        reason="The switch button is partially embedded in the base."
    )
    ctx.allow_overlap(
        foot_switch, base,
        elem_a="button", elem_b="switch_housing",
        reason="The switch button is inside the switch housing."
    )

    ctx.allow_overlap(head, head_mount, elem_a="hinge_pin", elem_b="left_bracket", reason="Hinge pin goes through bracket")
    ctx.allow_overlap(head, head_mount, elem_a="hinge_pin", elem_b="right_bracket", reason="Hinge pin goes through bracket")

    # Exact seating checks
    ctx.expect_gap(
        head_mount, base, axis="z", min_gap=0.0, max_gap=0.001,
        positive_elem="swivel_base", negative_elem="stem_cap",
        name="swivel base sits exactly on stem cap"
    )

    ctx.expect_overlap(
        head, head_mount, axes="y",
        elem_a="hinge_pin", elem_b="left_bracket",
        min_overlap=0.004,
        name="hinge pin overlaps left bracket"
    )
    ctx.expect_overlap(
        head, head_mount, axes="y",
        elem_a="hinge_pin", elem_b="right_bracket",
        min_overlap=0.004,
        name="hinge pin overlaps right bracket"
    )

    ctx.expect_within(
        head, head_mount, axes="xz",
        inner_elem="hinge_pin", outer_elem="left_bracket",
        name="hinge pin is contained within left bracket profile"
    )

    ctx.expect_within(
        foot_switch, base, axes="xy",
        inner_elem="button", outer_elem="switch_housing",
        name="switch button is within switch housing"
    )
    with ctx.pose({switch_joint: -0.005}):
        ctx.expect_within(
            foot_switch, base, axes="xy",
            inner_elem="button", outer_elem="switch_housing",
            name="pressed switch button is within switch housing"
        )

    with ctx.pose({pitch: 0.5}):
        ctx.expect_gap(
            head_mount, base, axis="z", min_gap=0.0, max_gap=0.001,
            positive_elem="swivel_base", negative_elem="stem_cap",
            name="swivel base remains seated when pitched"
        )

    return ctx.report()


object_model = build_object_model()