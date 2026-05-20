from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _make_hub_sleeve():
    """Stepped hollow sleeve: one low-cost molded/cast rotor core with bearing bore."""
    outer_profile = [
        (0.160, -0.670),
        (0.160, -0.585),
        (0.145, -0.555),
        (0.145, -0.230),
        (0.166, -0.190),
        (0.166, -0.090),
        (0.145, -0.050),
        (0.145, 0.330),
        (0.166, 0.370),
        (0.166, 0.470),
        (0.145, 0.510),
        (0.145, 0.670),
    ]
    inner_profile = [
        (0.075, -0.670),
        (0.075, -0.400),
        (0.075, 0.000),
        (0.075, 0.400),
        (0.075, 0.670),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_turnstile_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.60, 0.62, 0.60, 1.0))
    dark_bearing = model.material("dark_bearing_core", rgba=(0.12, 0.13, 0.14, 1.0))
    molded_black = model.material("molded_black", rgba=(0.05, 0.055, 0.06, 1.0))
    yellow_caps = model.material("safety_yellow_caps", rgba=(0.98, 0.70, 0.08, 1.0))
    floor_plate = model.material("powder_coated_base", rgba=(0.26, 0.28, 0.30, 1.0))
    bolt_zinc = model.material("zinc_bolts", rgba=(0.72, 0.73, 0.70, 1.0))

    fixed_support = model.part("fixed_support")
    fixed_support.visual(
        Box((1.26, 1.26, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=floor_plate,
        name="base_plate",
    )
    fixed_support.visual(
        Cylinder(radius=0.230, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=floor_plate,
        name="pedestal_casting",
    )
    fixed_support.visual(
        Cylinder(radius=0.052, length=1.900),
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        material=dark_bearing,
        name="supported_spindle",
    )
    fixed_support.visual(
        Cylinder(radius=0.165, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.3475)),
        material=dark_bearing,
        name="lower_bearing_race",
    )
    fixed_support.visual(
        Cylinder(radius=0.185, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.790)),
        material=dark_bearing,
        name="upper_bearing_race",
    )
    fixed_support.visual(
        Cylinder(radius=0.130, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 1.885)),
        material=bolt_zinc,
        name="retaining_nut",
    )

    # Split clamp ears and bolt bosses are deliberately simple stampings welded to
    # the bearing races: easy assembly order, few tools, and no orphaned details.
    for z_center, suffix in ((0.325, "lower"), (1.790, "upper")):
        fixed_support.visual(
            Box((0.125, 0.040, 0.075)),
            origin=Origin(xyz=(0.205, 0.0, z_center)),
            material=floor_plate,
            name=f"{suffix}_clamp_ear",
        )
        fixed_support.visual(
            Cylinder(radius=0.021, length=0.060),
            origin=Origin(xyz=(0.255, 0.0, z_center), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt_zinc,
            name=f"{suffix}_clamp_bolt",
        )

    anchor_positions = [
        (0.46, 0.46),
        (-0.46, 0.46),
        (-0.46, -0.46),
        (0.46, -0.46),
    ]
    for index, (x_pos, y_pos) in enumerate(anchor_positions):
        fixed_support.visual(
            Cylinder(radius=0.035, length=0.018),
            origin=Origin(xyz=(x_pos, y_pos, 0.082)),
            material=bolt_zinc,
            name=f"anchor_bolt_{index}",
        )
        fixed_support.visual(
            Box((0.110, 0.030, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.090), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=bolt_zinc,
            name=f"anchor_slot_{index}",
        )

    fixed_support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.65, length=1.95),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(_make_hub_sleeve(), "turnstile_hub_sleeve"),
        material=galvanized,
        name="hub_sleeve",
    )
    rotor.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.147, tube=0.018, radial_segments=18, tubular_segments=56),
            "turnstile_lower_bearing_core",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.600)),
        material=dark_bearing,
        name="lower_bearing_core",
    )
    rotor.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.147, tube=0.018, radial_segments=18, tubular_segments=56),
            "turnstile_upper_bearing_core",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=dark_bearing,
        name="upper_bearing_core",
    )

    rail_levels = [(-0.420, "low"), (0.000, "mid"), (0.420, "high")]
    rail_inner = 0.135
    rail_outer = 1.180
    rail_length = rail_outer - rail_inner
    rail_center = 0.5 * (rail_inner + rail_outer)
    for wing_index in range(4):
        angle = wing_index * math.pi / 2.0
        radial_x = math.cos(angle)
        radial_y = math.sin(angle)
        for z_level, level_name in rail_levels:
            rotor.visual(
                Cylinder(radius=0.026, length=rail_length),
                origin=Origin(
                    xyz=(rail_center * radial_x, rail_center * radial_y, z_level),
                    rpy=(0.0, math.pi / 2.0, angle),
                ),
                material=galvanized,
                name=f"rail_{wing_index}_{level_name}",
            )
            rotor.visual(
                Cylinder(radius=0.044, length=0.180),
                origin=Origin(
                    xyz=(0.218 * radial_x, 0.218 * radial_y, z_level),
                    rpy=(0.0, math.pi / 2.0, angle),
                ),
                material=molded_black,
                name=f"socket_{wing_index}_{level_name}",
            )
        rotor.visual(
            Cylinder(radius=0.028, length=0.935),
            origin=Origin(xyz=(rail_outer * radial_x, rail_outer * radial_y, 0.0)),
            material=galvanized,
            name=f"outer_post_{wing_index}",
        )
        rotor.visual(
            Cylinder(radius=0.040, length=0.045),
            origin=Origin(
                xyz=(rail_outer * radial_x, rail_outer * radial_y, 0.490),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=yellow_caps,
            name=f"top_snap_cap_{wing_index}",
        )
        rotor.visual(
            Cylinder(radius=0.040, length=0.045),
            origin=Origin(
                xyz=(rail_outer * radial_x, rail_outer * radial_y, -0.490),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=yellow_caps,
            name=f"bottom_snap_cap_{wing_index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.20, length=1.05),
        mass=24.0,
        origin=Origin(),
    )

    model.articulation(
        "spindle_rotation",
        ArticulationType.CONTINUOUS,
        parent=fixed_support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8),
        motion_properties=MotionProperties(damping=0.18, friction=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_support = object_model.get_part("fixed_support")
    rotor = object_model.get_part("rotor")
    spindle_joint = object_model.get_articulation("spindle_rotation")

    ctx.check(
        "single rotating rotor on supported spindle",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spindle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={spindle_joint.articulation_type}, axis={spindle_joint.axis}",
    )
    ctx.expect_within(
        fixed_support,
        rotor,
        axes="xy",
        inner_elem="supported_spindle",
        outer_elem="hub_sleeve",
        margin=0.0,
        name="spindle centered inside bearing sleeve footprint",
    )
    ctx.expect_overlap(
        fixed_support,
        rotor,
        axes="z",
        elem_a="supported_spindle",
        elem_b="hub_sleeve",
        min_overlap=1.20,
        name="spindle supports most of the rotor sleeve height",
    )
    ctx.expect_gap(
        rotor,
        fixed_support,
        axis="z",
        positive_elem="hub_sleeve",
        negative_elem="lower_bearing_race",
        min_gap=0.0,
        max_gap=0.003,
        name="lower thrust race seats the rotating sleeve",
    )
    ctx.expect_gap(
        fixed_support,
        rotor,
        axis="z",
        positive_elem="upper_bearing_race",
        negative_elem="hub_sleeve",
        min_gap=0.018,
        max_gap=0.090,
        name="upper retaining race clears rotating sleeve",
    )

    rest_position = ctx.part_world_position(rotor)
    rest_aabb = ctx.part_element_world_aabb(rotor, elem="rail_0_mid")
    with ctx.pose({spindle_joint: math.pi / 2.0}):
        spun_position = ctx.part_world_position(rotor)
        spun_aabb = ctx.part_element_world_aabb(rotor, elem="rail_0_mid")
    ctx.check(
        "rotor spins around fixed vertical spindle without translation",
        rest_position is not None
        and spun_position is not None
        and max(abs(a - b) for a, b in zip(rest_position, spun_position)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )
    ctx.check(
        "radial arm visibly sweeps around spindle",
        rest_aabb is not None
        and spun_aabb is not None
        and abs(rest_aabb[1][0] - spun_aabb[1][1]) < 0.08,
        details=f"rest_aabb={rest_aabb}, spun_aabb={spun_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
