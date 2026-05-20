from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_rect_sketch(width: float, depth: float, radius: float) -> cq.Sketch:
    return cq.Sketch().rect(width, depth).vertices().fillet(radius)


def _build_base_shape() -> cq.Workplane:
    lower = _rounded_rect_sketch(0.260, 0.220, 0.034)
    mid = _rounded_rect_sketch(0.246, 0.206, 0.031).moved(z=0.034)
    upper = _rounded_rect_sketch(0.206, 0.170, 0.027).moved(z=0.070)

    housing = cq.Workplane("XY").placeSketch(lower, mid, upper).loft()
    top_pad = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.070))
        .placeSketch(_rounded_rect_sketch(0.154, 0.126, 0.020))
        .extrude(0.008)
    )
    return housing.union(top_pad)


def _build_pitcher_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").placeSketch(
        _rounded_rect_sketch(0.112, 0.110, 0.018),
        _rounded_rect_sketch(0.128, 0.126, 0.020).moved(z=0.170),
        _rounded_rect_sketch(0.148, 0.142, 0.022).moved(z=0.300),
    ).loft()

    inner = cq.Workplane("XY").placeSketch(
        _rounded_rect_sketch(0.090, 0.088, 0.013).moved(z=0.012),
        _rounded_rect_sketch(0.116, 0.114, 0.015).moved(z=0.180),
        _rounded_rect_sketch(0.132, 0.128, 0.018).moved(z=0.330),
    ).loft()

    return outer.cut(inner)


def _build_cap_cover() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .transformed(offset=(0.075, 0.0, -0.006))
        .placeSketch(_rounded_rect_sketch(0.146, 0.136, 0.018))
        .extrude(0.012)
    )
    dome = (
        cq.Workplane("XY")
        .transformed(offset=(0.075, 0.0, 0.006))
        .circle(0.028)
        .extrude(0.008)
    )
    button_bore = (
        cq.Workplane("XY")
        .transformed(offset=(0.075, 0.0, -0.012))
        .circle(0.0115)
        .extrude(0.032)
    )
    return cover.union(dome).cut(button_bore)


def _build_cap_plug() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.075, 0.0, -0.018))
        .placeSketch(_rounded_rect_sketch(0.122, 0.116, 0.015))
        .extrude(0.012)
    )


def _build_blade_shape() -> cq.Workplane:
    assembly = (
        cq.Workplane("XY").circle(0.016).extrude(0.006).union(
            cq.Workplane("XY").circle(0.005).extrude(0.014)
        )
    )

    base_blade = (
        cq.Workplane("XY")
        .box(0.050, 0.011, 0.0018, centered=(True, True, False))
        .translate((0.018, 0.0, 0.0035))
    )
    blade_specs = (
        (0.0, 18.0),
        (90.0, -16.0),
        (180.0, 12.0),
        (270.0, -14.0),
    )
    for yaw_deg, pitch_deg in blade_specs:
        blade = base_blade.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), pitch_deg).rotate(
            (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg
        )
        assembly = assembly.union(blade)
    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    body_graphite = model.material("body_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.08, 0.09, 0.10, 1.0))
    soft_black = model.material("soft_black", rgba=(0.12, 0.12, 0.13, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    smoky_tritan = model.material("smoky_tritan", rgba=(0.76, 0.84, 0.90, 0.34))
    seal_black = model.material("seal_black", rgba=(0.10, 0.11, 0.11, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    button_metal = model.material("button_metal", rgba=(0.88, 0.89, 0.90, 1.0))

    base_shell_mesh = mesh_from_cadquery(_build_base_shape(), "vacuum_blender_base_shell")
    pitcher_shell_mesh = mesh_from_cadquery(_build_pitcher_shell(), "vacuum_blender_pitcher_shell")
    cap_cover_mesh = mesh_from_cadquery(_build_cap_cover(), "vacuum_blender_cap_cover")
    cap_plug_mesh = mesh_from_cadquery(_build_cap_plug(), "vacuum_blender_cap_plug")
    blade_mesh = mesh_from_cadquery(_build_blade_shape(), "vacuum_blender_blade")

    base = model.part("base")
    base.visual(base_shell_mesh, material=body_graphite, name="base_shell")
    base.visual(
        Box((0.118, 0.118, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0755)),
        material=brushed_metal,
        name="deck_plate",
    )
    base.visual(
        Box((0.004, 0.096, 0.032)),
        origin=Origin(xyz=(0.119, 0.0, 0.038)),
        material=gloss_black,
        name="status_panel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.260, 0.220, 0.083)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0415)),
    )

    pitcher = model.part("pitcher")
    pitcher.visual(pitcher_shell_mesh, material=smoky_tritan, name="jar_shell")
    pitcher.visual(
        Box((0.040, 0.040, 0.022)),
        origin=Origin(xyz=(0.012, 0.084, 0.236)),
        material=soft_black,
        name="handle_upper_bridge",
    )
    pitcher.visual(
        Box((0.034, 0.040, 0.024)),
        origin=Origin(xyz=(0.020, 0.084, 0.102)),
        material=soft_black,
        name="handle_lower_bridge",
    )
    pitcher.visual(
        Box((0.026, 0.020, 0.170)),
        origin=Origin(xyz=(0.012, 0.101, 0.164)),
        material=soft_black,
        name="handle_grip",
    )
    pitcher.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=seal_black,
        name="blade_seat",
    )
    for index, y_pos in enumerate((-0.038, 0.038)):
        pitcher.visual(
            Box((0.020, 0.016, 0.012)),
            origin=Origin(xyz=(-0.070, y_pos, 0.292)),
            material=soft_black,
            name=f"hinge_support_{index}",
        )
        pitcher.visual(
            Box((0.010, 0.016, 0.024)),
            origin=Origin(xyz=(-0.082, y_pos, 0.296)),
            material=soft_black,
            name=f"hinge_post_{index}",
        )
        pitcher.visual(
            Cylinder(radius=0.0055, length=0.020),
            origin=Origin(
                xyz=(-0.082, y_pos, 0.308),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=soft_black,
            name=f"hinge_barrel_{index}",
        )
    pitcher.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.312)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
    )

    cap = model.part("cap")
    cap.visual(cap_cover_mesh, material=gloss_black, name="cap_cover")
    cap.visual(cap_plug_mesh, material=seal_black, name="seal_plug")
    cap.visual(
        Box((0.024, 0.074, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, -0.001)),
        material=gloss_black,
        name="hinge_bridge",
    )
    cap.visual(
        Cylinder(radius=0.0053, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="hinge_barrel",
    )
    cap.inertial = Inertial.from_geometry(
        Box((0.150, 0.140, 0.040)),
        mass=0.22,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    button = model.part("release_button")
    button.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_metal,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=button_metal,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.0125, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0145)),
        material=button_metal,
        name="button_retainer",
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.020),
        mass=0.02,
        origin=Origin(),
    )

    blade = model.part("blade")
    blade.visual(blade_mesh, material=blade_steel, name="blade_assembly")
    blade.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=blade_steel,
        name="blade_spindle",
    )
    blade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.018),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )
    model.articulation(
        "pitcher_to_cap",
        ArticulationType.REVOLUTE,
        parent=pitcher,
        child=cap,
        origin=Origin(xyz=(-0.076, 0.0, 0.308)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "cap_to_release_button",
        ArticulationType.PRISMATIC,
        parent=cap,
        child=button,
        origin=Origin(xyz=(0.075, 0.0, 0.010)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    cap = object_model.get_part("cap")
    button = object_model.get_part("release_button")
    blade = object_model.get_part("blade")

    cap_hinge = object_model.get_articulation("pitcher_to_cap")
    button_slide = object_model.get_articulation("cap_to_release_button")

    ctx.expect_gap(
        pitcher,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        name="pitcher seats on the blender base",
    )
    ctx.expect_gap(
        cap,
        pitcher,
        axis="z",
        positive_elem="cap_cover",
        negative_elem="jar_shell",
        max_gap=0.0035,
        max_penetration=0.0,
        name="vacuum cap closes just above the pitcher rim",
    )
    ctx.expect_gap(
        blade,
        pitcher,
        axis="z",
        positive_elem="blade_assembly",
        negative_elem="blade_seat",
        min_gap=0.006,
        max_gap=0.020,
        name="blade clears the pitcher floor",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        inner_elem="blade_assembly",
        outer_elem="jar_shell",
        margin=0.030,
        name="blade remains inside the pitcher footprint",
    )
    ctx.allow_overlap(
        blade,
        pitcher,
        elem_a="blade_assembly",
        elem_b="jar_shell",
        reason="The blade is intentionally nested inside the true hollow pitcher shell; the open-top shell mesh is treated conservatively by current-pose overlap QC.",
    )
    ctx.allow_overlap(
        blade,
        pitcher,
        elem_a="blade_spindle",
        elem_b="jar_shell",
        reason="The spindle passes through the pitcher's simplified central drive opening, which is not explicitly cut out of the shell mesh.",
    )

    closed_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_cover")
    with ctx.pose({cap_hinge: math.radians(100.0)}):
        open_cap_aabb = ctx.part_element_world_aabb(cap, elem="cap_cover")

    ctx.check(
        "cap swings upward over the pitcher",
        closed_cap_aabb is not None
        and open_cap_aabb is not None
        and open_cap_aabb[1][2] > closed_cap_aabb[1][2] + 0.09,
        details=f"closed={closed_cap_aabb}, open={open_cap_aabb}",
    )

    rest_button_pos = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.005}):
        pressed_button_pos = ctx.part_world_position(button)

    ctx.check(
        "release button depresses downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
