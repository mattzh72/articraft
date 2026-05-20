from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobRelief,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _z_cylinder(radius: float, z0: float, z1: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is world Z and whose ends sit at z0/z1."""
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((0.0, 0.0, z0))


def _rounded_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.34, 0.24, 0.012)
        .edges("|Z")
        .fillet(0.030)
        .translate((0.0, 0.0, 0.006))
    )


def _lower_tube_shell() -> cq.Workplane:
    tube = _z_cylinder(0.022, 0.012, 0.620).cut(_z_cylinder(0.0155, 0.010, 0.626))
    collar = _z_cylinder(0.033, 0.560, 0.625).cut(_z_cylinder(0.0155, 0.555, 0.630))
    return tube.union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_desk_orchestra_stand")

    black = model.material("satin_black", rgba=(0.005, 0.005, 0.004, 1.0))
    dark = model.material("dark_hardware", rgba=(0.025, 0.023, 0.020, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("dull_rubber", rgba=(0.010, 0.010, 0.010, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_plate(), "flat_plate_base", tolerance=0.001),
        material=black,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_lower_tube_shell(), "lower_tube_shell", tolerance=0.001),
        material=black,
        name="lower_tube_shell",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black,
        name="tube_flange",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.102),
        origin=Origin(xyz=(0.081, 0.0, 0.595), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="clamp_shaft",
    )
    for x, y, name in (
        (-0.135, -0.090, "foot_0"),
        (0.135, -0.090, "foot_1"),
        (-0.135, 0.090, "foot_2"),
        (0.135, 0.090, "foot_3"),
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(x, y, 0.002)),
            material=rubber,
            name=name,
        )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0157, length=0.950),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="inner_post_tube",
    )
    inner_post.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        material=dark,
        name="upper_socket",
    )
    inner_post.visual(
        Box((0.200, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.481)),
        material=dark,
        name="hinge_bridge",
    )
    for x, name in ((-0.092, "hinge_cheek_0"), (0.092, "hinge_cheek_1")):
        inner_post.visual(
            Box((0.014, 0.046, 0.076)),
            origin=Origin(xyz=(x, 0.0, 0.523)),
            material=dark,
            name=name,
        )
    inner_post.visual(
        Cylinder(radius=0.006, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.525), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.012, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    for x, name in ((-0.065, "hinge_tab_0"), (0.065, "hinge_tab_1")):
        desk.visual(
            Box((0.032, 0.027, 0.014)),
            origin=Origin(xyz=(x, -0.0245, 0.007)),
            material=dark,
            name=name,
        )
    desk.visual(
        Box((0.500, 0.010, 0.320)),
        origin=Origin(xyz=(0.0, -0.030, 0.176)),
        material=black,
        name="score_panel",
    )
    desk.visual(
        Box((0.500, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, -0.055, 0.012)),
        material=black,
        name="score_shelf",
    )
    desk.visual(
        Box((0.500, 0.012, 0.042)),
        origin=Origin(xyz=(0.0, -0.085, 0.036)),
        material=black,
        name="front_lip",
    )
    for x, name in ((-0.253, "side_flange_0"), (0.253, "side_flange_1")):
        desk.visual(
            Box((0.012, 0.014, 0.310)),
            origin=Origin(xyz=(x, -0.028, 0.177)),
            material=black,
            name=name,
        )

    clamp_knob = model.part("clamp_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.026,
            body_style="lobed",
            base_diameter=0.030,
            top_diameter=0.041,
            crown_radius=0.0015,
            edge_radius=0.0008,
            bore=KnobBore(style="round", diameter=0.009),
            body_reliefs=(KnobRelief(style="top_recess", width=0.018, depth=0.0016),),
            center=False,
        ),
        "lobed_clamp_knob",
    )
    clamp_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knob_cap",
    )
    clamp_knob.visual(
        Cylinder(radius=0.0047, length=0.020),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="threaded_insert",
    )

    height_slide = model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.280),
    )
    desk_hinge = model.articulation(
        "desk_hinge",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.55, upper=0.55),
    )
    knob_spin = model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=clamp_knob,
        origin=Origin(xyz=(0.118, 0.0, 0.595)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    # Keep joint variables live enough for linters while preserving semantic names above.
    assert height_slide and desk_hinge and knob_spin
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner_post = object_model.get_part("inner_post")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")
    height_slide = object_model.get_articulation("height_slide")
    desk_hinge = object_model.get_articulation("desk_hinge")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.allow_overlap(
        inner_post,
        desk,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The hinge pin is intentionally captured inside the rotating desk barrel.",
    )
    ctx.allow_overlap(
        base,
        inner_post,
        elem_a="lower_tube_shell",
        elem_b="inner_post_tube",
        reason="The telescoping post is represented as a close sliding fit inside the lower tube sleeve.",
    )
    ctx.allow_overlap(
        base,
        clamp_knob,
        elem_a="clamp_shaft",
        elem_b="threaded_insert",
        reason="The fixed threaded shaft is intentionally seated in the knob's internal threaded insert.",
    )

    ctx.expect_within(
        inner_post,
        base,
        axes="xy",
        inner_elem="inner_post_tube",
        outer_elem="lower_tube_shell",
        margin=0.001,
        name="inner post stays centered in the lower tube",
    )
    ctx.expect_overlap(
        inner_post,
        base,
        axes="z",
        elem_a="inner_post_tube",
        elem_b="lower_tube_shell",
        min_overlap=0.30,
        name="collapsed post remains retained in the lower tube",
    )
    rest_post = ctx.part_world_position(inner_post)
    with ctx.pose({height_slide: 0.280}):
        ctx.expect_within(
            inner_post,
            base,
            axes="xy",
            inner_elem="inner_post_tube",
            outer_elem="lower_tube_shell",
            margin=0.001,
            name="extended post remains centered in the lower tube",
        )
        ctx.expect_overlap(
            inner_post,
            base,
            axes="z",
            elem_a="inner_post_tube",
            elem_b="lower_tube_shell",
            min_overlap=0.18,
            name="extended post keeps retained insertion",
        )
        extended_post = ctx.part_world_position(inner_post)
    ctx.check(
        "height slide raises the desk post",
        rest_post is not None and extended_post is not None and extended_post[2] > rest_post[2] + 0.25,
        details=f"rest={rest_post}, extended={extended_post}",
    )

    ctx.expect_overlap(
        desk,
        inner_post,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="hinge_pin",
        min_overlap=0.10,
        name="desk barrel is carried by the horizontal hinge pin",
    )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[1] + hi[1])

    rest_panel = ctx.part_element_world_aabb(desk, elem="score_panel")
    with ctx.pose({desk_hinge: 0.45}):
        tilted_panel = ctx.part_element_world_aabb(desk, elem="score_panel")
        ctx.expect_overlap(
            desk,
            inner_post,
            axes="x",
            elem_a="hinge_barrel",
            elem_b="hinge_pin",
            min_overlap=0.10,
            name="desk remains on hinge pin while tilted",
        )
    rest_y = _aabb_center_y(rest_panel)
    tilted_y = _aabb_center_y(tilted_panel)
    ctx.check(
        "desk hinge tilts the score shelf",
        rest_y is not None and tilted_y is not None and tilted_y < rest_y - 0.04,
        details=f"rest_center_y={rest_y}, tilted_center_y={tilted_y}",
    )

    ctx.expect_overlap(
        base,
        clamp_knob,
        axes="x",
        elem_a="clamp_shaft",
        elem_b="threaded_insert",
        min_overlap=0.008,
        name="clamp knob threaded insert sits on the shaft",
    )
    ctx.check(
        "clamp knob joint is horizontal and continuous",
        tuple(knob_spin.axis) == (1.0, 0.0, 0.0)
        and str(knob_spin.articulation_type).lower().endswith("continuous"),
        details=f"axis={knob_spin.axis}, type={knob_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
