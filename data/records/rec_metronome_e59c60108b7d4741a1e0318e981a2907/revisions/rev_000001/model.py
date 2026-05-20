from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_metronome")

    body_mat = Material("deep_burgundy_enamel", rgba=(0.23, 0.035, 0.045, 1.0))
    base_mat = Material("dark_walnut_base", rgba=(0.18, 0.105, 0.055, 1.0))
    brass = Material("brushed_brass", rgba=(0.92, 0.66, 0.28, 1.0))
    steel = Material("polished_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    shadow = Material("dark_slot_interior", rgba=(0.005, 0.004, 0.003, 1.0))

    base_radius = 0.160
    base_height = 0.035
    body_radius = 0.115
    body_height = 0.290
    body_z0 = base_height - 0.003
    body_top = body_z0 + body_height

    slot_width = 0.086
    slot_depth = 0.110
    slot_center_y = body_radius - slot_depth / 2.0 + 0.010
    slot_back_y = slot_center_y - slot_depth / 2.0
    slot_bottom = body_z0 + 0.035
    slot_height = 0.238
    slot_center_z = slot_bottom + slot_height / 2.0

    housing_shape = (
        cq.Workplane("XY")
        .circle(body_radius)
        .extrude(body_height)
        .translate((0.0, 0.0, body_z0))
    )
    slot_cutter = (
        cq.Workplane("XY")
        .box(slot_width, slot_depth, slot_height)
        .translate((0.0, slot_center_y, slot_center_z))
    )
    housing_shape = housing_shape.cut(slot_cutter)

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=base_radius, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=base_mat,
        name="disc_base",
    )
    housing.visual(
        Cylinder(radius=base_radius * 1.01, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=brass,
        name="lower_brass_rim",
    )
    housing.visual(
        Cylinder(radius=0.128, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, base_height + 0.002)),
        material=brass,
        name="upper_brass_rim",
    )
    housing.visual(
        mesh_from_cadquery(housing_shape, "slotted_round_housing", tolerance=0.0008),
        material=body_mat,
        name="slotted_round_housing",
    )
    housing.visual(
        Box((slot_width * 0.92, 0.005, slot_height * 0.92)),
        origin=Origin(xyz=(0.0, slot_back_y - 0.0015, slot_center_z)),
        material=shadow,
        name="slot_shadow",
    )

    pivot_y = body_radius - 0.033
    pivot_z = body_z0 + 0.073
    rod_y = 0.045
    rod_length = 0.320

    bearing_y0 = slot_back_y - 0.002
    bearing_y1 = pivot_y - 0.015 + 0.002
    housing.visual(
        Cylinder(radius=0.010, length=bearing_y1 - bearing_y0),
        origin=Origin(
            xyz=(0.0, (bearing_y0 + bearing_y1) / 2.0, pivot_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="bearing_socket",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0085, length=0.030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.0038, length=rod_y + 0.004),
        origin=Origin(xyz=(0.0, rod_y / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shaft_standoff",
    )
    pendulum.visual(
        Cylinder(radius=0.0034, length=rod_length),
        origin=Origin(xyz=(0.0, rod_y, rod_length / 2.0)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.0, rod_y, rod_length)),
        material=brass,
        name="tip_cap",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=-0.12, upper=0.12),
    )

    weight_outer = 0.024
    weight_inner = 0.007
    weight_height = 0.034
    weight_shape = (
        cq.Workplane("XY")
        .circle(weight_outer)
        .circle(weight_inner)
        .extrude(weight_height)
        .translate((0.0, 0.0, -weight_height / 2.0))
    )

    weight = model.part("pendulum_weight")
    weight.visual(
        mesh_from_cadquery(weight_shape, "sliding_ring_weight", tolerance=0.0006),
        material=brass,
        name="ring_weight",
    )
    weight.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="thumb_screw",
    )

    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, rod_y, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.25, lower=0.0, upper=0.105),
    )

    crown = model.part("crown_knob")
    crown.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brass,
        name="crown_stem",
    )
    crown.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.026,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="ribbed", count=22, depth=0.0018),
                center=False,
            ),
            "ribbed_winding_crown",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=brass,
        name="ribbed_crown",
    )

    model.articulation(
        "housing_to_crown",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crown,
        origin=Origin(xyz=(0.0, 0.0, body_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("pendulum_weight")
    crown = object_model.get_part("crown_knob")
    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_slide = object_model.get_articulation("pendulum_to_weight")
    crown_joint = object_model.get_articulation("housing_to_crown")

    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="bearing_socket",
        elem_b="pivot_hub",
        reason="The pendulum hub is intentionally captured by the brass bearing socket at the central shaft.",
    )
    ctx.allow_overlap(
        weight,
        pendulum,
        elem_a="thumb_screw",
        elem_b="rod",
        reason="The sliding ring weight has a small thumb screw pressing into the rod so it is retained while sliding.",
    )

    def _coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    ctx.check(
        "pendulum has limited swing",
        pendulum_joint.articulation_type == ArticulationType.REVOLUTE
        and pendulum_joint.motion_limits is not None
        and pendulum_joint.motion_limits.lower < 0.0
        and pendulum_joint.motion_limits.upper > 0.0,
        details=f"joint={pendulum_joint}",
    )
    ctx.check(
        "ring weight slides along rod",
        weight_slide.articulation_type == ArticulationType.PRISMATIC
        and weight_slide.axis == (0.0, 0.0, 1.0)
        and weight_slide.motion_limits is not None
        and weight_slide.motion_limits.upper >= 0.10,
        details=f"joint={weight_slide}",
    )
    ctx.check(
        "crown is continuous rotary knob",
        crown_joint.articulation_type == ArticulationType.CONTINUOUS
        and crown_joint.axis == (0.0, 0.0, 1.0),
        details=f"joint={crown_joint}",
    )

    ctx.expect_gap(
        crown,
        housing,
        axis="z",
        max_gap=0.002,
        max_penetration=0.000001,
        name="crown sits on the top face",
    )
    ctx.expect_overlap(
        housing,
        pendulum,
        axes="xyz",
        elem_a="bearing_socket",
        elem_b="pivot_hub",
        min_overlap=0.001,
        name="pivot hub is captured by bearing socket",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="xyz",
        elem_a="thumb_screw",
        elem_b="rod",
        min_overlap=0.002,
        name="thumb screw reaches the pendulum rod",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    if housing_aabb is not None and rod_aabb is not None:
        ctx.check(
            "rod protrudes above housing",
            _coord(rod_aabb[1], 2) > _coord(housing_aabb[1], 2) + 0.08,
            details=f"housing={housing_aabb}, rod={rod_aabb}",
        )
    else:
        ctx.fail("rod protrudes above housing", "missing AABB for housing or rod")

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({weight_slide: 0.105}):
        raised_weight = ctx.part_world_position(weight)
    ctx.check(
        "weight raises along pendulum rod",
        rest_weight is not None
        and raised_weight is not None
        and raised_weight[2] > rest_weight[2] + 0.095,
        details=f"rest={rest_weight}, raised={raised_weight}",
    )

    rest_rod = ctx.part_element_world_aabb(pendulum, elem="rod")
    with ctx.pose({pendulum_joint: 0.12}):
        swung_rod = ctx.part_element_world_aabb(pendulum, elem="rod")
    if rest_rod is not None and swung_rod is not None:
        rest_center_x = (_coord(rest_rod[0], 0) + _coord(rest_rod[1], 0)) / 2.0
        swung_center_x = (_coord(swung_rod[0], 0) + _coord(swung_rod[1], 0)) / 2.0
        ctx.check(
            "pendulum swings sideways through front slot",
            swung_center_x > rest_center_x + 0.015,
            details=f"rest_x={rest_center_x}, swung_x={swung_center_x}",
        )
    else:
        ctx.fail("pendulum swings sideways through front slot", "missing rod AABB")

    return ctx.report()


object_model = build_object_model()
