from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_four_joint_chain")

    bracket_mat = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    link_mat = model.material("blue_anodized_links", rgba=(0.05, 0.20, 0.42, 1.0))
    tab_mat = model.material("matte_black_end_tab", rgba=(0.02, 0.02, 0.018, 1.0))
    pin_mat = model.material("brushed_stainless_pins", rgba=(0.72, 0.72, 0.68, 1.0))

    hinge_rpy = (math.pi / 2.0, 0.0, 0.0)
    link_radius = 0.030
    link_thickness = 0.012
    link_width = 0.030
    pin_radius = 0.008
    pin_span = 0.060
    layer_near = -0.011
    layer_far = 0.011

    support = model.part("support")
    support.visual(
        Box((0.170, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=bracket_mat,
        name="top_plate",
    )
    support.visual(
        Box((0.070, 0.062, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=bracket_mat,
        name="mounting_block",
    )
    for y, name in ((-0.034, "cheek_0"), (0.034, "cheek_1")):
        support.visual(
            Box((0.052, 0.012, 0.105)),
            origin=Origin(xyz=(0.0, y, -0.055)),
            material=bracket_mat,
            name=name,
        )
    support.visual(
        Cylinder(radius=pin_radius, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.090), rpy=hinge_rpy),
        material=pin_mat,
        name="joint_0_pin",
    )

    def add_hanging_link(part_name: str, length: float, y_layer: float, material) -> object:
        part = model.part(part_name)
        web_length = length - 2.0 * link_radius + 0.010
        part.visual(
            Cylinder(radius=link_radius, length=link_thickness),
            origin=Origin(xyz=(0.0, y_layer, 0.0), rpy=hinge_rpy),
            material=material,
            name="proximal_eye",
        )
        part.visual(
            Box((link_width, link_thickness, web_length)),
            origin=Origin(xyz=(0.0, y_layer, -length / 2.0)),
            material=material,
            name="web",
        )
        part.visual(
            Cylinder(radius=link_radius, length=link_thickness),
            origin=Origin(xyz=(0.0, y_layer, -length), rpy=hinge_rpy),
            material=material,
            name="distal_eye",
        )
        part.visual(
            Cylinder(radius=pin_radius, length=pin_span),
            origin=Origin(xyz=(0.0, 0.0, -length), rpy=hinge_rpy),
            material=pin_mat,
            name="distal_pin",
        )
        return part

    link_0 = add_hanging_link("link_0", 0.220, layer_near, link_mat)
    link_1 = add_hanging_link("link_1", 0.195, layer_far, link_mat)
    link_2 = add_hanging_link("link_2", 0.175, layer_near, link_mat)

    end_tab = model.part("end_tab")
    end_tab.visual(
        Cylinder(radius=link_radius, length=link_thickness),
        origin=Origin(xyz=(0.0, layer_far, 0.0), rpy=hinge_rpy),
        material=tab_mat,
        name="proximal_eye",
    )
    end_tab.visual(
        Box((0.026, link_thickness, 0.070)),
        origin=Origin(xyz=(0.0, layer_far, -0.048)),
        material=tab_mat,
        name="neck",
    )
    end_tab.visual(
        Box((0.064, link_thickness, 0.044)),
        origin=Origin(xyz=(0.0, layer_far, -0.096)),
        material=tab_mat,
        name="tab_plate",
    )

    swing_limits = MotionLimits(effort=8.0, velocity=3.5, lower=-1.45, upper=1.45)
    model.articulation(
        "support_to_link_0",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "link_2_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_tab = object_model.get_part("end_tab")
    joints = [
        object_model.get_articulation("support_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_end_tab"),
    ]

    ctx.check(
        "four parallel revolute hinges",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(j.axis or ()) == (0.0, 1.0, 0.0) for j in joints),
        details=f"articulations={object_model.articulations}",
    )

    pin_pairs = [
        (support, link_0, "joint_0_pin", "top bracket pin captures the first link eye"),
        (link_0, link_1, "distal_pin", "first link pin captures the second link eye"),
        (link_1, link_2, "distal_pin", "second link pin captures the third link eye"),
        (link_2, end_tab, "distal_pin", "third link pin captures the compact end tab eye"),
    ]
    for parent, child, pin_name, reason in pin_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a=pin_name,
            elem_b="proximal_eye",
            reason=reason,
        )
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem=pin_name,
            outer_elem="proximal_eye",
            margin=0.002,
            name=f"{parent.name} pin centered in {child.name} eye",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin_name,
            elem_b="proximal_eye",
            min_overlap=0.010,
            name=f"{parent.name} pin spans {child.name} eye thickness",
        )

    ctx.expect_origin_gap(support, link_0, axis="z", min_gap=0.085, max_gap=0.095)
    ctx.expect_origin_gap(link_0, link_1, axis="z", min_gap=0.210, max_gap=0.230)
    ctx.expect_origin_gap(link_1, link_2, axis="z", min_gap=0.185, max_gap=0.205)
    ctx.expect_origin_gap(link_2, end_tab, axis="z", min_gap=0.165, max_gap=0.185)

    rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose({"support_to_link_0": 0.65}):
        swung_tip = ctx.part_world_position(end_tab)
    ctx.check(
        "upper hinge swings chain out of vertical plane",
        rest_tip is not None
        and swung_tip is not None
        and abs(swung_tip[0] - rest_tip[0]) > 0.08,
        details=f"rest={rest_tip}, swung={swung_tip}",
    )

    return ctx.report()


object_model = build_object_model()
