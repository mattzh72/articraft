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
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder with its axis along the local/world Y axis."""
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _hollow_cylinder_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    outer = _cylinder_y(outer_radius, length, center)
    cutter = _cylinder_y(inner_radius, length + 0.006, center)
    return outer.cut(cutter)


def _rect_tube_x(
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    center_x: float,
) -> cq.Workplane:
    outer = _box((length, outer_y, outer_z), (center_x, 0.0, 0.0))
    cutter = _box((length + 0.014, inner_y, inner_z), (center_x, 0.0, 0.0))
    return outer.cut(cutter)


def _base_clevis_shape() -> cq.Workplane:
    base = _box((0.120, 0.112, 0.014), (0.0, 0.0, -0.073))
    cheek_a = _box((0.046, 0.014, 0.106), (0.0, 0.039, -0.023))
    cheek_b = _box((0.046, 0.014, 0.106), (0.0, -0.039, -0.023))
    bore = _cylinder_y(0.0075, 0.140, (0.0, 0.0, 0.0))
    return base.union(cheek_a).union(cheek_b).cut(bore)


def _slider_body_shape() -> cq.Workplane:
    sleeve = _rect_tube_x(
        length=0.150,
        outer_y=0.044,
        outer_z=0.032,
        inner_y=0.030,
        inner_z=0.020,
        center_x=0.110,
    )
    pivot_tube = _hollow_cylinder_y(0.0175, 0.0054, 0.050, (0.0, 0.0, 0.0))

    # Two tangential webs connect the root pivot tube to the rectangular sleeve
    # without blocking the through-pin bore.
    upper_web = _box((0.047, 0.032, 0.006), (0.027, 0.0, 0.013))
    lower_web = _box((0.047, 0.032, 0.006), (0.027, 0.0, -0.013))
    return sleeve.union(pivot_tube).union(upper_web).union(lower_web)


def _output_tab_shape() -> cq.Workplane:
    eye = _hollow_cylinder_y(0.012, 0.0042, 0.012, (0.0, 0.0, 0.0))
    blade = _box((0.052, 0.012, 0.016), (0.033, 0.0, 0.0))
    rounded_tip = _cylinder_y(0.008, 0.012, (0.059, 0.0, 0.0))
    pin_clearance = _cylinder_y(0.0042, 0.020, (0.0, 0.0, 0.0))
    return eye.union(blade).union(rounded_tip).cut(pin_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hinge_slide_hinge")

    dark_anodized = model.material("dark_anodized", rgba=(0.07, 0.08, 0.09, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.20, 0.55, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    orange_tab = model.material("orange_tab", rgba=(0.95, 0.42, 0.08, 1.0))

    base = model.part("base_clevis")
    base.visual(
        mesh_from_cadquery(_base_clevis_shape(), "base_clevis"),
        material=dark_anodized,
        name="clevis",
    )
    base.visual(
        Cylinder(radius=0.0054, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="root_pin",
    )
    for side, y in (("pos", 0.049), ("neg", -0.049)):
        base.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name=f"pin_washer_{side}",
        )

    slider_body = model.part("slider_body")
    slider_body.visual(
        Cylinder(radius=0.0175, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue_anodized,
        name="pivot_barrel",
    )
    slider_body.visual(
        Box((0.047, 0.032, 0.006)),
        origin=Origin(xyz=(0.027, 0.0, 0.013)),
        material=blue_anodized,
        name="upper_web",
    )
    slider_body.visual(
        Box((0.047, 0.032, 0.006)),
        origin=Origin(xyz=(0.027, 0.0, -0.013)),
        material=blue_anodized,
        name="lower_web",
    )
    slider_body.visual(
        Box((0.150, 0.044, 0.006)),
        origin=Origin(xyz=(0.110, 0.0, 0.013)),
        material=blue_anodized,
        name="upper_wall",
    )
    slider_body.visual(
        Box((0.150, 0.044, 0.006)),
        origin=Origin(xyz=(0.110, 0.0, -0.013)),
        material=blue_anodized,
        name="lower_wall",
    )
    for side, y in (("pos", 0.0185), ("neg", -0.0185)):
        slider_body.visual(
            Box((0.150, 0.007, 0.020)),
            origin=Origin(xyz=(0.110, y, 0.0)),
            material=blue_anodized,
            name=f"side_wall_{side}",
        )
    slide = model.part("slide")
    slide.visual(
        Box((0.180, 0.020, 0.020)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=polished_steel,
        name="rail",
    )
    slide.visual(
        Box((0.020, 0.038, 0.014)),
        origin=Origin(xyz=(0.142, 0.0, 0.0)),
        material=polished_steel,
        name="fork_bridge",
    )
    for side, y in (("pos", 0.016), ("neg", -0.016)):
        slide.visual(
            Box((0.038, 0.006, 0.034)),
            origin=Origin(xyz=(0.170, y, 0.0)),
            material=polished_steel,
            name=f"fork_cheek_{side}",
        )
    slide.visual(
        Cylinder(radius=0.0042, length=0.046),
        origin=Origin(xyz=(0.170, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="distal_pin",
    )

    output_tab = model.part("output_tab")
    output_tab.visual(
        mesh_from_cadquery(_output_tab_shape(), "output_tab"),
        material=orange_tab,
        name="tab_plate",
    )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=slider_body,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.85),
    )
    model.articulation(
        "slider_stage",
        ArticulationType.PRISMATIC,
        parent=slider_body,
        child=slide,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.055),
    )
    model.articulation(
        "distal_hinge",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=output_tab,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-1.15, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_clevis")
    slider_body = object_model.get_part("slider_body")
    slide = object_model.get_part("slide")
    output_tab = object_model.get_part("output_tab")
    root_hinge = object_model.get_articulation("root_hinge")
    slider_stage = object_model.get_articulation("slider_stage")
    distal_hinge = object_model.get_articulation("distal_hinge")

    ctx.check(
        "chain is revolute-prismatic-revolute",
        root_hinge.articulation_type == ArticulationType.REVOLUTE
        and slider_stage.articulation_type == ArticulationType.PRISMATIC
        and distal_hinge.articulation_type == ArticulationType.REVOLUTE,
    )

    ctx.allow_overlap(
        base,
        slider_body,
        elem_a="root_pin",
        elem_b="pivot_barrel",
        reason="The root pin is intentionally represented as captured through the slider body's pivot barrel.",
    )
    ctx.allow_overlap(
        output_tab,
        slide,
        elem_a="tab_plate",
        elem_b="distal_pin",
        reason="The distal hinge pin is intentionally represented as captured through the output tab eye.",
    )

    ctx.expect_overlap(
        slider_body,
        base,
        axes="y",
        elem_a="pivot_barrel",
        elem_b="root_pin",
        min_overlap=0.040,
        name="root pivot bushing wraps the base pin",
    )
    ctx.expect_within(
        base,
        slider_body,
        axes="xz",
        inner_elem="root_pin",
        outer_elem="pivot_barrel",
        margin=0.001,
        name="root pin is centered in the pivot barrel",
    )
    ctx.expect_contact(
        slide,
        slider_body,
        elem_a="rail",
        elem_b="upper_wall",
        contact_tol=0.001,
        name="rail bears on the upper sleeve wall",
    )
    ctx.expect_contact(
        slide,
        slider_body,
        elem_a="rail",
        elem_b="lower_wall",
        contact_tol=0.001,
        name="rail bears on the lower sleeve wall",
    )
    ctx.expect_within(
        slide,
        slider_body,
        axes="yz",
        inner_elem="rail",
        margin=0.0,
        name="slide rail is captured inside the sleeve cross section",
    )
    ctx.expect_overlap(
        slide,
        slider_body,
        axes="x",
        elem_a="rail",
        elem_b="upper_wall",
        min_overlap=0.100,
        name="collapsed slide remains inserted in the sleeve",
    )
    ctx.expect_overlap(
        output_tab,
        slide,
        axes="yz",
        elem_a="tab_plate",
        elem_b="distal_pin",
        min_overlap=0.008,
        name="output tab eye is carried on the distal pin",
    )
    ctx.expect_within(
        slide,
        output_tab,
        axes="xz",
        inner_elem="distal_pin",
        outer_elem="tab_plate",
        margin=0.001,
        name="distal pin is centered in the tab eye",
    )

    rest_slide_pos = ctx.part_world_position(slide)
    rest_slider_aabb = ctx.part_world_aabb(slider_body)
    rest_tab_aabb = ctx.part_element_world_aabb(output_tab, elem="tab_plate")

    with ctx.pose({slider_stage: 0.055}):
        ctx.expect_within(
            slide,
            slider_body,
            axes="yz",
            inner_elem="rail",
            margin=0.0,
            name="extended rail stays in the sleeve cross section",
        )
        ctx.expect_overlap(
            slide,
            slider_body,
            axes="x",
            elem_a="rail",
            elem_b="upper_wall",
            min_overlap=0.085,
            name="extended slide retains insertion in the sleeve",
        )
        extended_slide_pos = ctx.part_world_position(slide)

    ctx.check(
        "prismatic stage extends along the slider body",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.045,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    with ctx.pose({root_hinge: 0.65}):
        raised_slider_aabb = ctx.part_world_aabb(slider_body)

    ctx.check(
        "root hinge lifts the carried slider body",
        rest_slider_aabb is not None
        and raised_slider_aabb is not None
        and raised_slider_aabb[1][2] > rest_slider_aabb[1][2] + 0.08,
        details=f"rest={rest_slider_aabb}, raised={raised_slider_aabb}",
    )

    with ctx.pose({distal_hinge: 0.90}):
        raised_tab_aabb = ctx.part_element_world_aabb(output_tab, elem="tab_plate")

    ctx.check(
        "distal hinge swings the output tab",
        rest_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][2] > rest_tab_aabb[1][2] + 0.035,
        details=f"rest={rest_tab_aabb}, raised={raised_tab_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
