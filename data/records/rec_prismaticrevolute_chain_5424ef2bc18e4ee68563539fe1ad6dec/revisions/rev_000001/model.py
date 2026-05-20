from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _output_link_profile(
    *, length: float = 0.36, width: float = 0.055, eye_radius: float = 0.040
) -> list[tuple[float, float]]:
    """Single closed outline for a flat link with a round pin eye at its root."""
    half_w = width / 2.0
    theta = math.asin(half_w / eye_radius)
    x_tangent = math.sqrt(eye_radius * eye_radius - half_w * half_w)
    profile: list[tuple[float, float]] = [
        (x_tangent, half_w),
        (length, half_w),
        (length, -half_w),
        (x_tangent, -half_w),
    ]
    arc_segments = 40
    for i in range(arc_segments + 1):
        angle = -theta - (2.0 * math.pi - 2.0 * theta) * i / arc_segments
        profile.append((eye_radius * math.cos(angle), eye_radius * math.sin(angle)))
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_and_hinge_chain")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.22, 0.45, 1.0))
    link_orange = model.material("output_orange", rgba=(0.92, 0.43, 0.10, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.78, 0.76, 0.70, 1.0))

    base = model.part("base_guide")
    base.visual(
        Box((0.86, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.82, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, 0.090, 0.075)),
        material=rail_steel,
        name="right_rail",
    )
    base.visual(
        Box((0.82, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, 0.090, 0.116)),
        material=rail_steel,
        name="right_rail_lip",
    )
    base.visual(
        Box((0.82, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.090, 0.075)),
        material=rail_steel,
        name="left_rail",
    )
    base.visual(
        Box((0.82, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, -0.090, 0.116)),
        material=rail_steel,
        name="left_rail_lip",
    )
    for x, name in ((-0.420, "rear_stop"), (0.420, "front_stop")):
        base.visual(
            Box((0.025, 0.22, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.0775)),
            material=dark_steel,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.132, 0.075)),
        origin=Origin(),
        material=carriage_blue,
        name="slide_block",
    )
    carriage.visual(
        Box((0.180, 0.150, 0.014)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0445)),
        material=carriage_blue,
        name="top_cap",
    )
    # Two horizontal clevis plates project from the carriage front face and
    # carry the vertical revolute pin for the output link.
    carriage.visual(
        Box((0.080, 0.090, 0.012)),
        origin=Origin(xyz=(0.150, 0.0, 0.029)),
        material=carriage_blue,
        name="upper_clevis",
    )
    carriage.visual(
        Box((0.080, 0.090, 0.012)),
        origin=Origin(xyz=(0.150, 0.0, -0.029)),
        material=carriage_blue,
        name="lower_clevis",
    )
    carriage.visual(
        Cylinder(radius=0.011, length=0.095),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=pin_metal,
        name="hinge_pin",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.155, 0.0, 0.050)),
        material=pin_metal,
        name="pin_head",
    )

    output = model.part("output_link")
    link_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _output_link_profile(),
            [_circle_profile(0.014, 48)],
            0.028,
            center=True,
        ),
        "output_link_plate",
    )
    output.visual(link_mesh, origin=Origin(), material=link_orange, name="link_plate")
    output.visual(
        Box((0.080, 0.030, 0.018)),
        origin=Origin(xyz=(0.330, 0.0, 0.0)),
        material=link_orange,
        name="output_pad",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.220, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.340),
    )
    model.articulation(
        "output_pin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-1.25, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    output = object_model.get_part("output_link")
    slide = object_model.get_articulation("guide_slide")
    hinge = object_model.get_articulation("output_pin")

    ctx.allow_overlap(
        carriage,
        output,
        elem_a="hinge_pin",
        elem_b="link_plate",
        reason=(
            "The vertical hinge pin is intentionally represented as a captured "
            "shaft through the output link's pin eye."
        ),
    )

    ctx.check(
        "root joint is prismatic along guide axis",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "carried joint is vertical revolute pin",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="y",
            positive_elem="slide_block",
            negative_elem="left_rail_lip",
            min_gap=-0.001,
            max_gap=0.001,
            name="carriage bears against left rail lip",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="y",
            positive_elem="right_rail_lip",
            negative_elem="slide_block",
            min_gap=-0.001,
            max_gap=0.001,
            name="carriage bears against right rail lip",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="y",
            positive_elem="right_rail",
            negative_elem="slide_block",
            min_gap=0.005,
            max_gap=0.010,
            name="right rail side clearance",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="y",
            positive_elem="slide_block",
            negative_elem="left_rail",
            min_gap=0.005,
            max_gap=0.010,
            name="left rail side clearance",
        )
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem="slide_block",
            outer_elem="base_plate",
            margin=0.0,
            name="carriage remains on the guide at retracted pose",
        )
        ctx.expect_overlap(
            carriage,
            output,
            axes="z",
            elem_a="hinge_pin",
            elem_b="link_plate",
            min_overlap=0.020,
            name="hinge pin spans the output link thickness",
        )
        ctx.expect_gap(
            carriage,
            output,
            axis="z",
            positive_elem="upper_clevis",
            negative_elem="link_plate",
            min_gap=0.006,
            max_gap=0.012,
            name="upper clevis clears the swinging link",
        )
        ctx.expect_gap(
            output,
            carriage,
            axis="z",
            positive_elem="link_plate",
            negative_elem="lower_clevis",
            min_gap=0.006,
            max_gap=0.012,
            name="lower clevis clears the swinging link",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.340, hinge: 0.0}):
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem="slide_block",
            outer_elem="base_plate",
            margin=0.0,
            name="carriage remains on the guide at extended pose",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides forward on the guide",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        rest_link_aabb = ctx.part_element_world_aabb(output, elem="link_plate")
    with ctx.pose({slide: 0.0, hinge: 1.0}):
        swung_link_aabb = ctx.part_element_world_aabb(output, elem="link_plate")
    rest_center_y = None if rest_link_aabb is None else (rest_link_aabb[0][1] + rest_link_aabb[1][1]) / 2.0
    swung_center_y = None if swung_link_aabb is None else (swung_link_aabb[0][1] + swung_link_aabb[1][1]) / 2.0
    ctx.check(
        "output link swings about the carried pin",
        rest_center_y is not None and swung_center_y is not None and swung_center_y > rest_center_y + 0.10,
        details=f"rest_center_y={rest_center_y}, swung_center_y={swung_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
