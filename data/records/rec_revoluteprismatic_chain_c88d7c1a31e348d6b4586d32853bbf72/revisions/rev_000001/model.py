from __future__ import annotations

import math

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
)


def _box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_y(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_linear_service_arm")

    base_mat = Material("powder_coated_charcoal", color=(0.08, 0.09, 0.10, 1.0))
    rubber_mat = Material("black_rubber_feet", color=(0.015, 0.015, 0.014, 1.0))
    steel_mat = Material("brushed_steel", color=(0.66, 0.67, 0.64, 1.0))
    link_mat = Material("safety_blue_link", color=(0.05, 0.25, 0.72, 1.0))
    slider_mat = Material("hard_anodized_nose", color=(0.15, 0.17, 0.19, 1.0))
    tip_mat = Material("orange_soft_tip", color=(1.0, 0.42, 0.06, 1.0))

    base = model.part("base")
    _box(base, "ground_plate", (0.56, 0.38, 0.055), (0.0, 0.0, 0.0275), base_mat)
    _box(base, "front_foot", (0.46, 0.040, 0.018), (0.0, -0.170, 0.009), rubber_mat)
    _box(base, "rear_foot", (0.46, 0.040, 0.018), (0.0, 0.170, 0.009), rubber_mat)
    _box(base, "square_plinth", (0.24, 0.24, 0.155), (0.0, 0.0, 0.1325), base_mat)
    base.visual(
        Cylinder(radius=0.070, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.3325)),
        material=steel_mat,
        name="round_column",
    )
    _box(base, "yoke_bridge", (0.170, 0.235, 0.045), (0.0, 0.0, 0.4775), base_mat)
    _box(base, "yoke_cheek_0", (0.135, 0.035, 0.150), (0.0, -0.095, 0.575), base_mat)
    _box(base, "yoke_cheek_1", (0.135, 0.035, 0.150), (0.0, 0.095, 0.575), base_mat)
    _cylinder_y(base, "pin_cap_0", 0.042, 0.018, (0.0, -0.1215, 0.575), steel_mat)
    _cylinder_y(base, "pin_cap_1", 0.042, 0.018, (0.0, 0.1215, 0.575), steel_mat)

    link = model.part("link")
    _cylinder_y(link, "pivot_hub", 0.055, 0.155, (0.0, 0.0, 0.0), steel_mat)
    _box(link, "main_beam", (0.285, 0.100, 0.070), (0.1725, 0.0, 0.0), link_mat)
    _box(link, "upper_sleeve_rail", (0.360, 0.130, 0.020), (0.440, 0.0, 0.036), link_mat)
    _box(link, "lower_sleeve_rail", (0.360, 0.130, 0.020), (0.440, 0.0, -0.036), link_mat)
    _box(link, "side_sleeve_rail_0", (0.360, 0.024, 0.072), (0.440, -0.053, 0.0), link_mat)
    _box(link, "side_sleeve_rail_1", (0.360, 0.024, 0.072), (0.440, 0.053, 0.0), link_mat)
    _box(link, "rear_gusset_0", (0.110, 0.020, 0.104), (0.082, -0.060, 0.0), steel_mat)
    _box(link, "rear_gusset_1", (0.110, 0.020, 0.104), (0.082, 0.060, 0.0), steel_mat)

    slider = model.part("tip_slider")
    _box(slider, "slide_bar", (0.440, 0.064, 0.040), (-0.040, 0.0, 0.0), slider_mat)
    _box(slider, "top_wear_strip", (0.250, 0.050, 0.006), (-0.090, 0.0, 0.023), steel_mat)
    _box(slider, "bottom_wear_strip", (0.250, 0.050, 0.006), (-0.090, 0.0, -0.023), steel_mat)
    _cylinder_x(slider, "rounded_tip", 0.026, 0.040, (0.200, 0.0, 0.0), tip_mat)

    model.articulation(
        "base_to_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link,
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.3, lower=-0.35, upper=0.95),
    )
    model.articulation(
        "link_to_tip_slider",
        ArticulationType.PRISMATIC,
        parent=link,
        child=slider,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link = object_model.get_part("link")
    slider = object_model.get_part("tip_slider")
    shoulder = object_model.get_articulation("base_to_link")
    slide = object_model.get_articulation("link_to_tip_slider")

    ctx.check(
        "one revolute followed by one prismatic",
        len(object_model.articulations) == 2
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and slide.parent == link.name
        and shoulder.child == link.name,
        details=f"articulations={object_model.articulations}",
    )

    ctx.expect_gap(
        link,
        base,
        axis="z",
        min_gap=-0.150,
        max_gap=0.020,
        name="link is captured inside the base yoke",
    )
    ctx.expect_overlap(
        slider,
        link,
        axes="x",
        elem_a="slide_bar",
        elem_b="upper_sleeve_rail",
        min_overlap=0.250,
        name="slider remains deeply inserted at rest",
    )
    ctx.expect_within(
        slider,
        link,
        axes="yz",
        margin=0.010,
        name="tip slider is smaller than the link sleeve aperture",
    )

    base_aabb = ctx.part_world_aabb(base)
    link_aabb = ctx.part_world_aabb(link)
    slider_bar_aabb = ctx.part_element_world_aabb(slider, elem="slide_bar")

    def span(aabb, axis_index: int) -> float:
        if aabb is None:
            return 0.0
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.check(
        "base is broad and grounded",
        base_aabb is not None
        and span(base_aabb, 0) > 0.50
        and span(base_aabb, 1) > 0.34
        and abs(base_aabb[0][2]) < 1e-6,
        details=f"base_aabb={base_aabb}",
    )
    ctx.check(
        "tip slider stays clearly smaller than the rigid link",
        link_aabb is not None
        and slider_bar_aabb is not None
        and span(slider_bar_aabb, 1) < 0.70 * span(link_aabb, 1)
        and span(slider_bar_aabb, 2) < 0.45 * span(link_aabb, 2),
        details=f"link_aabb={link_aabb}, slider_bar_aabb={slider_bar_aabb}",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.140}):
        extended_slider_pos = ctx.part_world_position(slider)
        ctx.expect_overlap(
            slider,
            link,
            axes="x",
            elem_a="slide_bar",
            elem_b="upper_sleeve_rail",
            min_overlap=0.110,
            name="extended slider retains insertion",
        )
        ctx.expect_within(
            slider,
            link,
            axes="yz",
            margin=0.010,
            name="extended slider remains centered in sleeve",
        )

    ctx.check(
        "prismatic joint drives the linear nose outward",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.12,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    return ctx.report()


object_model = build_object_model()
