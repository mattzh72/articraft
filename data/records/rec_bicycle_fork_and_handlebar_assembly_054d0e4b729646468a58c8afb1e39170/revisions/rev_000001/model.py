from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_tube_z(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """CadQuery tube with a real open bore, centered on local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _tube_mesh(points, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fork_quill_stem")

    carbon = model.material("satin_carbon", rgba=(0.015, 0.017, 0.018, 1.0))
    carbon_edge = model.material("carbon_edge", rgba=(0.04, 0.045, 0.05, 1.0))
    alloy = model.material("brushed_alloy", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.08, 0.085, 0.09, 1.0))
    steel = model.material("bolt_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    tape = model.material("cloth_bar_tape", rgba=(0.68, 0.49, 0.31, 1.0))

    # Root: a carbon track fork, with a true hollow round steerer for the quill.
    fork = model.part("fork")
    fork.visual(
        mesh_from_cadquery(_annular_tube_z(0.0185, 0.0140, 0.460), "steerer_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=carbon,
        name="steerer_shell",
    )
    crown_shape = (
        cq.Workplane("XY")
        .box(0.090, 0.140, 0.065)
        .edges()
        .fillet(0.010)
    )
    fork.visual(
        mesh_from_cadquery(crown_shape, "fork_crown"),
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        material=carbon_edge,
        name="crown",
    )
    fork.visual(
        _tube_mesh(
            [(0.000, -0.050, 0.510), (0.014, -0.053, 0.360), (0.032, -0.054, 0.190), (0.043, -0.055, 0.075)],
            0.014,
            "blade_0_mesh",
        ),
        material=carbon,
        name="blade_0",
    )
    fork.visual(
        _tube_mesh(
            [(0.000, 0.050, 0.510), (0.014, 0.053, 0.360), (0.032, 0.054, 0.190), (0.043, 0.055, 0.075)],
            0.014,
            "blade_1_mesh",
        ),
        material=carbon,
        name="blade_1",
    )
    fork.visual(
        Box((0.055, 0.014, 0.050)),
        origin=Origin(xyz=(0.048, -0.055, 0.055)),
        material=dark_alloy,
        name="dropout_0",
    )
    fork.visual(
        Box((0.055, 0.014, 0.050)),
        origin=Origin(xyz=(0.048, 0.055, 0.055)),
        material=dark_alloy,
        name="dropout_1",
    )
    fork.visual(
        Cylinder(radius=0.006, length=0.155),
        origin=Origin(xyz=(0.060, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_reference",
    )

    # Quill stem: the vertical quill is the prismatic child and remains inserted
    # in the open steerer.  Its frame is at the steerer lip at minimum height.
    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.0105, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=alloy,
        name="quill_shaft",
    )
    stem.visual(
        Cylinder(radius=0.0145, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.225)),
        material=dark_alloy,
        name="expander_wedge",
    )
    stem.visual(
        _tube_mesh(
            [(0.000, 0.0, 0.075), (0.043, 0.0, 0.115), (0.084, 0.0, 0.105), (0.110, 0.0, 0.090)],
            0.014,
            "stem_neck_mesh",
        ),
        material=alloy,
        name="stem_neck",
    )
    stem.visual(
        mesh_from_cadquery(
            _annular_tube_z(0.030, 0.0105, 0.056).rotate((0, 0, 0), (1, 0, 0), 90),
            "bar_clamp_shell",
        ),
        origin=Origin(xyz=(0.140, 0.0, 0.090)),
        material=alloy,
        name="clamp_shell",
    )
    stem.visual(
        Box((0.028, 0.052, 0.010)),
        origin=Origin(xyz=(0.166, 0.0, 0.108)),
        material=alloy,
        name="clamp_ear_upper",
    )
    stem.visual(
        Box((0.028, 0.052, 0.010)),
        origin=Origin(xyz=(0.166, 0.0, 0.072)),
        material=alloy,
        name="clamp_ear_lower",
    )
    stem.visual(
        Cylinder(radius=0.0040, length=0.036),
        origin=Origin(xyz=(0.173, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_bolt_upper",
    )
    stem.visual(
        Cylinder(radius=0.0040, length=0.036),
        origin=Origin(xyz=(0.173, 0.0, 0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_bolt_lower",
    )
    stem.visual(
        Cylinder(radius=0.0060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=steel,
        name="quill_bolt_head",
    )

    # Deep-drop track bars.  The part frame is the clamp axis, so the stem can
    # rotate the whole bar about its round center section for angle adjustment.
    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0110, length=0.260),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="center_bar",
    )
    handlebar.visual(
        _tube_mesh(
            [
                (0.000, -0.125, 0.000),
                (0.030, -0.165, 0.004),
                (0.075, -0.197, -0.030),
                (0.108, -0.207, -0.105),
                (0.135, -0.205, -0.180),
                (0.175, -0.195, -0.245),
            ],
            0.011,
            "drop_0_mesh",
        ),
        material=dark_alloy,
        name="drop_0",
    )
    handlebar.visual(
        _tube_mesh(
            [
                (0.000, 0.125, 0.000),
                (0.030, 0.165, 0.004),
                (0.075, 0.197, -0.030),
                (0.108, 0.207, -0.105),
                (0.135, 0.205, -0.180),
                (0.175, 0.195, -0.245),
            ],
            0.011,
            "drop_1_mesh",
        ),
        material=dark_alloy,
        name="drop_1",
    )
    handlebar.visual(
        _tube_mesh(
            [(0.082, -0.205, -0.080), (0.115, -0.207, -0.150), (0.172, -0.196, -0.240)],
            0.0125,
            "grip_0_mesh",
        ),
        material=tape,
        name="grip_0",
    )
    handlebar.visual(
        _tube_mesh(
            [(0.082, 0.205, -0.080), (0.115, 0.207, -0.150), (0.172, 0.196, -0.240)],
            0.0125,
            "grip_1_mesh",
        ),
        material=tape,
        name="grip_1",
    )

    model.articulation(
        "steerer_to_stem",
        ArticulationType.PRISMATIC,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 1.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.075, effort=120.0, velocity=0.12),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.140, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=20.0, velocity=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    height = object_model.get_articulation("steerer_to_stem")
    angle = object_model.get_articulation("stem_to_handlebar")

    ctx.allow_overlap(
        fork,
        stem,
        elem_a="steerer_shell",
        elem_b="expander_wedge",
        reason="The quill expander wedge is intentionally compressed against the inside of the carbon steerer to lock the height-adjustable stem.",
    )
    ctx.allow_overlap(
        stem,
        handlebar,
        elem_a="clamp_shell",
        elem_b="center_bar",
        reason="The handlebar clamp bore is modeled with slight local compression around the round bar so the revolute clamp is visibly captured.",
    )

    ctx.expect_within(
        stem,
        fork,
        axes="xy",
        inner_elem="quill_shaft",
        outer_elem="steerer_shell",
        margin=0.002,
        name="quill shaft stays centered in round steerer",
    )
    ctx.expect_overlap(
        stem,
        fork,
        axes="z",
        elem_a="quill_shaft",
        elem_b="steerer_shell",
        min_overlap=0.20,
        name="quill has deep insertion at minimum height",
    )
    ctx.expect_within(
        stem,
        fork,
        axes="xy",
        inner_elem="expander_wedge",
        outer_elem="steerer_shell",
        margin=0.002,
        name="expander wedge sits inside steerer bore",
    )
    ctx.expect_overlap(
        stem,
        fork,
        axes="z",
        elem_a="expander_wedge",
        elem_b="steerer_shell",
        min_overlap=0.040,
        name="expander wedge bears inside steerer length",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="xz",
        inner_elem="center_bar",
        outer_elem="clamp_shell",
        margin=0.001,
        name="round bar section is captured by clamp bore",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="y",
        elem_a="center_bar",
        elem_b="clamp_shell",
        min_overlap=0.050,
        name="handlebar center spans the stem clamp",
    )

    rest_pos = ctx.part_world_position(stem)
    with ctx.pose({height: 0.075}):
        ctx.expect_overlap(
            stem,
            fork,
            axes="z",
            elem_a="quill_shaft",
            elem_b="steerer_shell",
            min_overlap=0.13,
            name="raised quill remains safely inserted",
        )
        raised_pos = ctx.part_world_position(stem)
    ctx.check(
        "prismatic height adjustment raises stem",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.070,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({angle: -0.30}):
        tilted_drop = ctx.part_element_world_aabb(handlebar, elem="drop_0")
    with ctx.pose({angle: 0.30}):
        dropped_drop = ctx.part_element_world_aabb(handlebar, elem="drop_0")
    ctx.check(
        "revolute clamp changes bar drop angle",
        tilted_drop is not None
        and dropped_drop is not None
        and tilted_drop[0][2] > dropped_drop[0][2] + 0.04,
        details=f"tilted={tilted_drop}, dropped={dropped_drop}",
    )

    return ctx.report()


object_model = build_object_model()
