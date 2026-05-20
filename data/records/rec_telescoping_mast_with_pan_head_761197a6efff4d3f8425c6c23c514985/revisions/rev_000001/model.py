from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _tube_mesh(name: str, outer: tuple[float, float], inner: tuple[float, float], length: float):
    """Rounded rectangular hollow mast/sleeve section, centered on local Z."""
    outer_profile = rounded_rect_profile(outer[0], outer[1], min(outer) * 0.16, corner_segments=6)
    inner_profile = rounded_rect_profile(inner[0], inner[1], min(inner) * 0.14, corner_segments=6)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer_profile, [inner_profile], length, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_survey_mast")

    dark = model.material("matte_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.012, 0.013, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.72, 0.72, 1.0))
    light_aluminum = model.material("light_anodized_aluminum", rgba=(0.82, 0.84, 0.82, 1.0))
    blue = model.material("blue_glass", rgba=(0.05, 0.18, 0.32, 0.82))
    safety = model.material("survey_orange", rgba=(0.95, 0.40, 0.08, 1.0))

    # Root: a weighted electronics/base housing with a fixed outer mast sleeve.
    base = model.part("base_housing")
    base.visual(
        Box((0.58, 0.42, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark,
        name="base_shell",
    )
    base.visual(
        Box((0.50, 0.035, 0.010)),
        origin=Origin(xyz=(0.0, -0.197, 0.155)),
        material=safety,
        name="front_accent",
    )
    for idx, (x, y) in enumerate(((-0.22, -0.16), (0.22, -0.16), (-0.22, 0.16), (0.22, 0.16))):
        base.visual(
            Box((0.12, 0.08, 0.025)),
            origin=Origin(xyz=(x, y, 0.012)),
            material=black,
            name=f"foot_pad_{idx}",
        )
    base.visual(
        Box((0.21, 0.17, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark,
        name="raised_socket",
    )
    base.visual(
        _tube_mesh("outer_sleeve_mesh", (0.108, 0.108), (0.078, 0.078), 0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=aluminum,
        name="outer_sleeve",
    )
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.057, 0.0, 0.032, 0.145),
            (-0.057, 0.0, 0.032, 0.145),
            (0.0, 0.057, 0.145, 0.032),
            (0.0, -0.057, 0.145, 0.032),
        )
    ):
        base.visual(
            Box((sx, sy, 0.085)),
            origin=Origin(xyz=(x, y, 0.885)),
            material=dark,
            name=f"outer_collar_{idx}",
        )
    base.visual(
        Box((0.040, 0.150, 0.34)),
        origin=Origin(xyz=(0.086, 0.0, 0.36), rpy=(0.0, 0.25, 0.0)),
        material=dark,
        name="side_gusset_0",
    )
    base.visual(
        Box((0.040, 0.150, 0.34)),
        origin=Origin(xyz=(-0.086, 0.0, 0.36), rpy=(0.0, -0.25, 0.0)),
        material=dark,
        name="side_gusset_1",
    )

    # First sliding stage. Its frame sits at the outer sleeve entry plane.
    mid = model.part("mid_section")
    mid.visual(
        _tube_mesh("mid_tube_mesh", (0.066, 0.066), (0.048, 0.048), 0.95),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=light_aluminum,
        name="mid_tube",
    )
    for idx, (x, y, sx, sy) in enumerate(
        (
            (0.037, 0.0, 0.022, 0.096),
            (-0.037, 0.0, 0.022, 0.096),
            (0.0, 0.037, 0.096, 0.022),
            (0.0, -0.037, 0.096, 0.022),
        )
    ):
        mid.visual(
            Box((sx, sy, 0.075)),
            origin=Origin(xyz=(x, y, 0.335)),
            material=dark,
            name=f"mid_collar_{idx}",
        )
    mid.visual(
        Box((0.042, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, -0.036, 0.20)),
        material=safety,
        name="mid_scale_mark",
    )

    # Second sliding stage, carrying the pan head.
    inner = model.part("inner_section")
    inner.visual(
        _tube_mesh("inner_tube_mesh", (0.040, 0.040), (0.026, 0.026), 0.82),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=aluminum,
        name="inner_tube",
    )
    inner.visual(
        Box((0.055, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=dark,
        name="head_mount_plate",
    )

    outer_lock = model.part("outer_lock")
    outer_lock.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lock_stem",
    )
    outer_lock.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lock_knob",
    )

    mid_lock = model.part("mid_lock")
    mid_lock.visual(
        Cylinder(radius=0.009, length=0.043),
        origin=Origin(xyz=(0.0215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lock_stem",
    )
    mid_lock.visual(
        Cylinder(radius=0.023, length=0.022),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lock_knob",
    )

    # Compact yawing pan head / inspection payload.
    pan = model.part("pan_head")
    pan.visual(
        Cylinder(radius=0.054, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark,
        name="pan_turntable",
    )
    pan.visual(
        Box((0.155, 0.095, 0.066)),
        origin=Origin(xyz=(0.025, 0.0, 0.067)),
        material=dark,
        name="head_body",
    )
    pan.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.1115, 0.0, 0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="front_optic",
    )
    pan.visual(
        Box((0.090, 0.010, 0.030)),
        origin=Origin(xyz=(0.010, 0.052, 0.070)),
        material=safety,
        name="side_label_0",
    )
    pan.visual(
        Box((0.090, 0.010, 0.030)),
        origin=Origin(xyz=(0.010, -0.052, 0.070)),
        material=safety,
        name="side_label_1",
    )
    pan.visual(
        Box((0.060, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=black,
        name="top_cap",
    )

    model.articulation(
        "base_to_mid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mid,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.25, lower=0.0, upper=0.42),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.22, lower=0.0, upper=0.35),
    )
    model.articulation(
        "inner_to_pan",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "base_to_outer_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_lock,
        origin=Origin(xyz=(0.073, 0.0, 0.885)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "mid_to_mid_lock",
        ArticulationType.REVOLUTE,
        parent=mid,
        child=mid_lock,
        origin=Origin(xyz=(0.048, 0.0, 0.335)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    mid = object_model.get_part("mid_section")
    inner = object_model.get_part("inner_section")
    pan = object_model.get_part("pan_head")
    base_to_mid = object_model.get_articulation("base_to_mid")
    mid_to_inner = object_model.get_articulation("mid_to_inner")
    inner_to_pan = object_model.get_articulation("inner_to_pan")

    ctx.allow_overlap(
        base,
        mid,
        elem_a="outer_sleeve",
        elem_b="mid_tube",
        reason="The mid stage is intentionally represented as sliding inside the hollow outer sleeve.",
    )
    ctx.allow_overlap(
        mid,
        inner,
        elem_a="mid_tube",
        elem_b="inner_tube",
        reason="The inner stage is intentionally represented as sliding inside the hollow mid tube.",
    )
    ctx.allow_overlap(
        base,
        inner,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="In the collapsed pose the nested inner stage passes down through the lower sleeve clearance.",
    )

    ctx.expect_within(
        mid,
        base,
        axes="xy",
        inner_elem="mid_tube",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="mid stage is centered in the outer sleeve",
    )
    ctx.expect_overlap(
        mid,
        base,
        axes="z",
        elem_a="mid_tube",
        elem_b="outer_sleeve",
        min_overlap=0.35,
        name="collapsed mid stage remains deeply inserted",
    )
    ctx.expect_within(
        inner,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="collapsed inner stage fits within the outer sleeve envelope",
    )
    ctx.expect_overlap(
        inner,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.15,
        name="collapsed inner stage remains inside the lower nest",
    )
    ctx.expect_within(
        inner,
        mid,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="mid_tube",
        margin=0.001,
        name="inner stage is centered in the mid tube",
    )
    ctx.expect_overlap(
        inner,
        mid,
        axes="z",
        elem_a="inner_tube",
        elem_b="mid_tube",
        min_overlap=0.40,
        name="collapsed inner stage remains inserted",
    )
    ctx.expect_contact(
        pan,
        inner,
        elem_a="pan_turntable",
        elem_b="head_mount_plate",
        contact_tol=0.004,
        name="pan head is seated on the mast top plate",
    )

    rest_pan_position = ctx.part_world_position(pan)
    with ctx.pose({base_to_mid: 0.42, mid_to_inner: 0.35}):
        extended_pan_position = ctx.part_world_position(pan)
        ctx.expect_overlap(
            mid,
            base,
            axes="z",
            elem_a="mid_tube",
            elem_b="outer_sleeve",
            min_overlap=0.17,
            name="extended mid stage retains insertion",
        )
        ctx.expect_overlap(
            inner,
            mid,
            axes="z",
            elem_a="inner_tube",
            elem_b="mid_tube",
            min_overlap=0.16,
            name="extended inner stage retains insertion",
        )
    ctx.check(
        "mast extension raises the pan head",
        rest_pan_position is not None
        and extended_pan_position is not None
        and extended_pan_position[2] > rest_pan_position[2] + 0.70,
        details=f"rest={rest_pan_position}, extended={extended_pan_position}",
    )

    with ctx.pose({inner_to_pan: math.pi / 2.0}):
        yawed_aabb = ctx.part_element_world_aabb(pan, elem="head_body")
    rest_aabb = ctx.part_element_world_aabb(pan, elem="head_body")
    if yawed_aabb is not None and rest_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        yawed_dx = yawed_aabb[1][0] - yawed_aabb[0][0]
        ctx.check(
            "pan joint yaws the compact head",
            yawed_dx < rest_dx * 0.85,
            details=f"rest_dx={rest_dx}, yawed_dx={yawed_dx}",
        )
    else:
        ctx.fail("pan joint yaws the compact head", "missing head_body AABB")

    return ctx.report()


object_model = build_object_model()
