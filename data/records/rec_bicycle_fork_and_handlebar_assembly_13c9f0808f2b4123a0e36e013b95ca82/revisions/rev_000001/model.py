from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _tube_z(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Centered hollow tube with its axis on local Z."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner).translate((0.0, 0.0, -0.5 * length))


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dirt_jump_fork_stem")

    alloy = Material("brushed_alloy", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_alloy = Material("satin_black_alloy", rgba=(0.035, 0.038, 0.040, 1.0))
    frame_paint = Material("matte_frame_grey", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    bolt_steel = Material("dark_bolt_steel", rgba=(0.18, 0.18, 0.17, 1.0))

    # Stationary bicycle head tube and small frame stubs.  This is the root part
    # that carries the headset bearing stack and defines the steering axis.
    head_tube = model.part("head_tube")
    head_tube.visual(
        mesh_from_cadquery(_tube_z(0.034, 0.018, 0.165), "head_tube_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=frame_paint,
        name="head_tube_shell",
    )
    head_tube.visual(
        mesh_from_cadquery(_tube_z(0.040, 0.0165, 0.018), "lower_headset_cup"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=bolt_steel,
        name="lower_headset_cup",
    )
    head_tube.visual(
        mesh_from_cadquery(_tube_z(0.040, 0.0165, 0.018), "upper_headset_cup"),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=bolt_steel,
        name="upper_headset_cup",
    )
    head_tube.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(-0.125, 0.0, 0.148), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="top_tube_stub",
    )
    head_tube.visual(
        Cylinder(radius=0.021, length=0.24),
        origin=Origin(xyz=(-0.152, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="down_tube_stub",
    )

    # Rotating fork/steerer: one-piece crown, long threadless steerer tube, and
    # two straight, round dirt-jump legs.
    fork = model.part("fork")
    crown_shape = _rounded_box((0.092, 0.205, 0.055), 0.010)
    fork.visual(
        mesh_from_cadquery(crown_shape, "wide_alloy_crown"),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=alloy,
        name="wide_alloy_crown",
    )
    fork.visual(
        Cylinder(radius=0.026, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=alloy,
        name="steerer_boss",
    )
    fork.visual(
        mesh_from_cadquery(_tube_z(0.0143, 0.0095, 0.450), "threadless_steerer"),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=alloy,
        name="threadless_steerer",
    )
    fork.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=bolt_steel,
        name="crown_race",
    )
    for idx, y in enumerate((-0.060, 0.060)):
        fork.visual(
            Cylinder(radius=0.023, length=0.070),
            origin=Origin(xyz=(0.0, y, -0.055)),
            material=alloy,
            name=f"leg_socket_{idx}",
        )
        fork.visual(
            Cylinder(radius=0.018, length=0.540),
            origin=Origin(xyz=(0.0, y, -0.335)),
            material=dark_alloy,
            name=f"straight_leg_{idx}",
        )
        fork.visual(
            Box((0.032, 0.026, 0.070)),
            origin=Origin(xyz=(0.0, y, -0.612)),
            material=dark_alloy,
            name=f"dropout_tab_{idx}",
        )
        fork.visual(
            Cylinder(radius=0.011, length=0.032),
            origin=Origin(xyz=(0.0, y, -0.595), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"axle_boss_{idx}",
        )
    fork.visual(
        Cylinder(radius=0.0075, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.595), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="thru_axle",
    )

    steering_joint = model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0, lower=-1.25, upper=1.25),
    )

    # The stem and bars slide together on the threadless steerer, representing
    # clamp-height repositioning before the steerer clamp is tightened.
    stem_bar = model.part("stem_bar")
    stem_bar.visual(
        mesh_from_cadquery(_tube_z(0.026, 0.0140, 0.076), "steerer_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_alloy,
        name="steerer_collar",
    )
    stem_bar.visual(
        Box((0.088, 0.034, 0.028)),
        origin=Origin(xyz=(0.064, 0.0, 0.052), rpy=(0.0, -0.12, 0.0)),
        material=dark_alloy,
        name="midrise_stem_body",
    )
    stem_bar.visual(
        mesh_from_cadquery(_tube_z(0.026, 0.0100, 0.070), "bar_clamp_collar"),
        origin=Origin(xyz=(0.118, 0.0, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="bar_clamp_collar",
    )
    stem_bar.visual(
        Box((0.015, 0.080, 0.065)),
        origin=Origin(xyz=(0.149, 0.0, 0.062)),
        material=dark_alloy,
        name="faceplate",
    )
    for idx, (y, z) in enumerate(((-0.028, 0.039), (0.028, 0.039), (-0.028, 0.085), (0.028, 0.085))):
        stem_bar.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.160, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bolt_steel,
            name=f"faceplate_bolt_{idx}",
        )
    stem_bar.visual(
        Cylinder(radius=0.0105, length=0.720),
        origin=Origin(xyz=(0.118, 0.0, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="flat_handlebar",
    )
    for idx, y in enumerate((-0.330, 0.330)):
        stem_bar.visual(
            Cylinder(radius=0.013, length=0.105),
            origin=Origin(xyz=(0.118, y, 0.062), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rubber_grip_{idx}",
        )
    stem_bar.visual(
        Box((0.012, 0.020, 0.060)),
        origin=Origin(xyz=(0.026, 0.0, 0.038)),
        material=dark_alloy,
        name="steerer_clamp_bridge",
    )

    model.articulation(
        "stem_height",
        ArticulationType.PRISMATIC,
        parent=fork,
        child=stem_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.10, lower=0.0, upper=0.045),
    )

    # Keep the joint variable named so static analyzers see it as intentionally
    # constructed before returning.
    _ = steering_joint
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    stem_bar = object_model.get_part("stem_bar")
    steering = object_model.get_articulation("steering_axis")
    stem_height = object_model.get_articulation("stem_height")

    ctx.allow_overlap(
        fork,
        stem_bar,
        elem_a="threadless_steerer",
        elem_b="steerer_collar",
        reason="The stem collar is shown lightly clamped around the threadless steerer for retained height adjustment.",
    )

    ctx.expect_within(
        fork,
        head_tube,
        axes="xy",
        inner_elem="threadless_steerer",
        outer_elem="head_tube_shell",
        margin=0.001,
        name="steerer runs concentrically through head tube",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="z",
        elem_a="threadless_steerer",
        elem_b="head_tube_shell",
        min_overlap=0.12,
        name="steerer is retained through headset length",
    )
    ctx.expect_within(
        fork,
        stem_bar,
        axes="xy",
        inner_elem="threadless_steerer",
        outer_elem="steerer_collar",
        margin=0.001,
        name="stem collar is centered on steerer",
    )
    ctx.expect_overlap(
        stem_bar,
        fork,
        axes="z",
        elem_a="steerer_collar",
        elem_b="threadless_steerer",
        min_overlap=0.070,
        name="stem collar remains clamped over steerer",
    )

    rest_pos = ctx.part_world_position(stem_bar)
    with ctx.pose({stem_height: 0.045}):
        raised_pos = ctx.part_world_position(stem_bar)
        ctx.expect_overlap(
            stem_bar,
            fork,
            axes="z",
            elem_a="steerer_collar",
            elem_b="threadless_steerer",
            min_overlap=0.070,
            name="raised stem still surrounds steerer",
        )
    ctx.check(
        "stem height joint moves assembly upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.040,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    fork_rest = ctx.part_world_position(fork)
    with ctx.pose({steering: 0.75}):
        fork_turned = ctx.part_world_position(fork)
        ctx.expect_within(
            fork,
            head_tube,
            axes="xy",
            inner_elem="threadless_steerer",
            outer_elem="head_tube_shell",
            margin=0.001,
            name="steering rotates about head tube axis",
        )
    ctx.check(
        "steering joint keeps fork origin on axis",
        fork_rest is not None and fork_turned is not None and abs(fork_turned[0]) < 1e-6 and abs(fork_turned[1]) < 1e-6,
        details=f"rest={fork_rest}, turned={fork_turned}",
    )

    return ctx.report()


object_model = build_object_model()
