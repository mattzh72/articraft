from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _beveled_annulus(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    chamfer: float = 0.004,
    segments: int = 96,
) -> LatheGeometry:
    """A low, hollow ring with small chamfers on the visible edges."""
    c = min(chamfer, (outer_radius - inner_radius) * 0.35, (z_max - z_min) * 0.35)
    profile = [
        (inner_radius + c, z_min),
        (outer_radius - c, z_min),
        (outer_radius, z_min + c),
        (outer_radius, z_max - c),
        (outer_radius - c, z_max),
        (inner_radius + c, z_max),
        (inner_radius, z_max - c),
        (inner_radius, z_min + c),
    ]
    return LatheGeometry(profile, segments=segments)


def _base_housing() -> LatheGeometry:
    """Squat cast housing with softened edges and a recessed upper deck."""
    profile = [
        (0.000, 0.000),
        (0.350, 0.000),
        (0.390, 0.010),
        (0.405, 0.026),
        (0.405, 0.058),
        (0.386, 0.073),
        (0.330, 0.079),
        (0.070, 0.079),
        (0.045, 0.086),
        (0.000, 0.086),
    ]
    return LatheGeometry(profile, segments=112)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_coaxial_stack")

    model.material("matte_base", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("dark_bearing", rgba=(0.05, 0.055, 0.060, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.61, 0.64, 1.0))
    model.material("anodized_blue", rgba=(0.05, 0.26, 0.55, 1.0))
    model.material("warm_titanium", rgba=(0.70, 0.58, 0.42, 1.0))
    model.material("pale_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("white_index", rgba=(0.92, 0.94, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(_base_housing(), "base_housing"),
        material="matte_base",
        name="base_housing",
    )
    base.visual(
        Cylinder(radius=0.220, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0885)),
        material="dark_bearing",
        name="base_bearing",
    )

    lower_tier = model.part("lower_tier")
    lower_tier.visual(
        mesh_from_geometry(
            _beveled_annulus(
                outer_radius=0.360,
                inner_radius=0.220,
                z_min=0.000,
                z_max=0.040,
                chamfer=0.006,
            ),
            "lower_annular_ring",
        ),
        material="anodized_blue",
        name="lower_ring",
    )
    lower_tier.visual(
        Cylinder(radius=0.118, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="brushed_steel",
        name="lower_hub",
    )
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        lower_tier.visual(
            Box((0.150, 0.040, 0.018)),
            origin=Origin(xyz=(0.170 * math.cos(yaw), 0.170 * math.sin(yaw), 0.021), rpy=(0.0, 0.0, yaw)),
            material="brushed_steel",
            name=f"lower_spoke_{index}",
        )
    lower_tier.visual(
        Cylinder(radius=0.105, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material="dark_bearing",
        name="lower_bearing",
    )
    lower_tier.visual(
        Box((0.088, 0.020, 0.006)),
        origin=Origin(xyz=(0.292, 0.0, 0.0425)),
        material="white_index",
        name="lower_index",
    )

    middle_tier = model.part("middle_tier")
    middle_tier.visual(
        mesh_from_geometry(
            _beveled_annulus(
                outer_radius=0.245,
                inner_radius=0.145,
                z_min=0.000,
                z_max=0.036,
                chamfer=0.005,
            ),
            "middle_annular_stage",
        ),
        material="warm_titanium",
        name="middle_ring",
    )
    middle_tier.visual(
        Cylinder(radius=0.088, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="brushed_steel",
        name="middle_hub",
    )
    for index, yaw in enumerate((math.pi / 4.0, 5.0 * math.pi / 4.0)):
        middle_tier.visual(
            Box((0.122, 0.034, 0.016)),
            origin=Origin(xyz=(0.122 * math.cos(yaw), 0.122 * math.sin(yaw), 0.020), rpy=(0.0, 0.0, yaw)),
            material="brushed_steel",
            name=f"middle_spoke_{index}",
        )
    middle_tier.visual(
        Cylinder(radius=0.074, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="dark_bearing",
        name="middle_bearing",
    )
    middle_tier.visual(
        Box((0.064, 0.018, 0.006)),
        origin=Origin(xyz=(0.191, 0.0, 0.0385), rpy=(0.0, 0.0, 0.0)),
        material="white_index",
        name="middle_index",
    )

    upper_tier = model.part("upper_tier")
    upper_tier.visual(
        mesh_from_geometry(
            _beveled_annulus(
                outer_radius=0.150,
                inner_radius=0.050,
                z_min=0.000,
                z_max=0.028,
                chamfer=0.004,
            ),
            "upper_compact_flange",
        ),
        material="pale_aluminum",
        name="upper_flange",
    )
    upper_tier.visual(
        Cylinder(radius=0.070, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material="brushed_steel",
        name="upper_cap",
    )
    upper_tier.visual(
        Box((0.046, 0.014, 0.005)),
        origin=Origin(xyz=(0.105, 0.0, 0.0305)),
        material="white_index",
        name="upper_index",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_tier,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_tier,
        child=middle_tier,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.REVOLUTE,
        parent=middle_tier,
        child=upper_tier,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_tier = object_model.get_part("lower_tier")
    middle_tier = object_model.get_part("middle_tier")
    upper_tier = object_model.get_part("upper_tier")
    lower_joint = object_model.get_articulation("base_to_lower")
    middle_joint = object_model.get_articulation("lower_to_middle")
    upper_joint = object_model.get_articulation("middle_to_upper")

    joints = (lower_joint, middle_joint, upper_joint)
    ctx.check(
        "three independent vertical revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(j.axis == (0.0, 0.0, 1.0) for j in joints)
        and all(j.mimic is None for j in joints),
        details=f"joints={[(j.name, j.articulation_type, j.axis, j.mimic) for j in joints]}",
    )

    ctx.expect_contact(
        lower_tier,
        base,
        elem_a="lower_hub",
        elem_b="base_bearing",
        name="lower tier bears on base housing",
    )
    ctx.expect_contact(
        middle_tier,
        lower_tier,
        elem_a="middle_hub",
        elem_b="lower_bearing",
        name="middle tier bears on lower coaxial race",
    )
    ctx.expect_contact(
        upper_tier,
        middle_tier,
        elem_a="upper_flange",
        elem_b="middle_bearing",
        name="upper tier bears on middle coaxial race",
    )

    ctx.expect_gap(
        middle_tier,
        lower_tier,
        axis="z",
        min_gap=0.018,
        positive_elem="middle_ring",
        negative_elem="lower_ring",
        name="visible gap between lower and middle rings",
    )
    ctx.expect_gap(
        upper_tier,
        middle_tier,
        axis="z",
        min_gap=0.018,
        positive_elem="upper_flange",
        negative_elem="middle_ring",
        name="visible gap between middle and upper rings",
    )

    with ctx.pose({lower_joint: 1.1, middle_joint: -0.8, upper_joint: 0.55}):
        lower_pos = ctx.part_world_position(lower_tier)
        middle_pos = ctx.part_world_position(middle_tier)
        upper_pos = ctx.part_world_position(upper_tier)
    ctx.check(
        "posed tiers remain coaxial",
        lower_pos is not None
        and middle_pos is not None
        and upper_pos is not None
        and max(abs(lower_pos[0]), abs(lower_pos[1]), abs(middle_pos[0]), abs(middle_pos[1]), abs(upper_pos[0]), abs(upper_pos[1])) < 1e-6
        and lower_pos[2] < middle_pos[2] < upper_pos[2],
        details=f"lower={lower_pos}, middle={middle_pos}, upper={upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
