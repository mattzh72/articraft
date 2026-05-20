from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_live_axle_beam")

    beam_span = 1.20
    beam_depth = 0.12
    beam_height = 0.18
    end_pad_length = 0.08
    end_pad_depth = 0.16
    end_pad_height = 0.22

    kingpin_radius = 0.035
    kingpin_length = 0.24
    knuckle_body_length = 0.13
    knuckle_body_depth = 0.14
    knuckle_body_height = 0.14
    spindle_radius = 0.050
    spindle_length = 0.040

    hub_radius = 0.120
    hub_length = 0.110
    flange_radius = 0.145
    flange_length = 0.030
    cap_radius = 0.050
    cap_length = 0.028

    beam_outer_x = beam_span * 0.5 + end_pad_length * 0.5
    kingpin_mount_x = beam_outer_x + end_pad_length * 0.5
    body_inner_x = kingpin_radius * 2.0
    body_center_x = body_inner_x + knuckle_body_length * 0.5
    spindle_center_x = body_inner_x + knuckle_body_length + spindle_length * 0.5
    hub_joint_offset_x = body_inner_x + knuckle_body_length + spindle_length
    steering_limit = 0.60

    beam_steel = model.material("beam_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    housing_steel = model.material("housing_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    machined_hub = model.material("machined_hub", rgba=(0.58, 0.60, 0.64, 1.0))

    beam = model.part("beam")
    beam.visual(
        Box((beam_span, beam_depth, beam_height)),
        material=beam_steel,
        name="main_beam",
    )
    beam.visual(
        Box((end_pad_length, end_pad_depth, end_pad_height)),
        origin=Origin(xyz=(-beam_outer_x, 0.0, 0.0)),
        material=beam_steel,
        name="left_end_pad",
    )
    beam.visual(
        Box((end_pad_length, end_pad_depth, end_pad_height)),
        origin=Origin(xyz=(beam_outer_x, 0.0, 0.0)),
        material=beam_steel,
        name="right_end_pad",
    )
    beam.inertial = Inertial.from_geometry(
        Box((beam_span + 2.0 * end_pad_length, end_pad_depth, end_pad_height)),
        mass=95.0,
        origin=Origin(),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        knuckle = model.part(f"{side_name}_knuckle")
        knuckle.visual(
            Cylinder(radius=kingpin_radius, length=kingpin_length),
            origin=Origin(xyz=(side_sign * kingpin_radius, 0.0, 0.0)),
            material=housing_steel,
            name="kingpin_barrel",
        )
        knuckle.visual(
            Box((knuckle_body_length, knuckle_body_depth, knuckle_body_height)),
            origin=Origin(xyz=(side_sign * body_center_x, 0.0, 0.0)),
            material=housing_steel,
            name="housing_block",
        )
        knuckle.visual(
            Cylinder(radius=spindle_radius, length=spindle_length),
            origin=Origin(
                xyz=(side_sign * spindle_center_x, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=housing_steel,
            name="spindle",
        )
        knuckle.inertial = Inertial.from_geometry(
            Box(
                (
                    hub_joint_offset_x,
                    knuckle_body_depth,
                    kingpin_length,
                )
            ),
            mass=18.0,
            origin=Origin(xyz=(side_sign * hub_joint_offset_x * 0.5, 0.0, 0.0)),
        )

        hub = model.part(f"{side_name}_hub")
        hub.visual(
            Cylinder(radius=hub_radius, length=hub_length),
            origin=Origin(
                xyz=(side_sign * hub_length * 0.5, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=machined_hub,
            name="hub_drum",
        )
        hub.visual(
            Cylinder(radius=flange_radius, length=flange_length),
            origin=Origin(
                xyz=(side_sign * (hub_length + flange_length * 0.5), 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=machined_hub,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=cap_radius, length=cap_length),
            origin=Origin(
                xyz=(side_sign * (hub_length + cap_length * 0.5), 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=machined_hub,
            name="dust_cap",
        )
        hub.inertial = Inertial.from_geometry(
            Box((hub_length + flange_length, flange_radius * 2.0, flange_radius * 2.0)),
            mass=12.0,
            origin=Origin(xyz=(side_sign * (hub_length + flange_length) * 0.5, 0.0, 0.0)),
        )

        model.articulation(
            f"beam_to_{side_name}_knuckle",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=knuckle,
            origin=Origin(xyz=(side_sign * kingpin_mount_x, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3500.0,
                velocity=1.8,
                lower=-steering_limit,
                upper=steering_limit,
            ),
        )
        model.articulation(
            f"{side_name}_knuckle_to_{side_name}_hub",
            ArticulationType.CONTINUOUS,
            parent=knuckle,
            child=hub,
            origin=Origin(xyz=(side_sign * hub_joint_offset_x, 0.0, 0.0)),
            axis=(side_sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=800.0, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    left_knuckle = object_model.get_part("left_knuckle")
    right_knuckle = object_model.get_part("right_knuckle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_kingpin = object_model.get_articulation("beam_to_left_knuckle")
    right_kingpin = object_model.get_articulation("beam_to_right_knuckle")
    left_axle = object_model.get_articulation("left_knuckle_to_left_hub")
    right_axle = object_model.get_articulation("right_knuckle_to_right_hub")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        left_knuckle,
        beam,
        elem_a="kingpin_barrel",
        elem_b="left_end_pad",
        name="left kingpin barrel is mounted against the left beam pad",
    )
    ctx.expect_contact(
        right_knuckle,
        beam,
        elem_a="kingpin_barrel",
        elem_b="right_end_pad",
        name="right kingpin barrel is mounted against the right beam pad",
    )
    ctx.expect_contact(
        left_hub,
        left_knuckle,
        elem_a="hub_drum",
        elem_b="spindle",
        name="left hub seats on the left spindle face",
    )
    ctx.expect_contact(
        right_hub,
        right_knuckle,
        elem_a="hub_drum",
        elem_b="spindle",
        name="right hub seats on the right spindle face",
    )

    ctx.check(
        "kingpin joints use vertical revolute axes",
        left_kingpin.articulation_type == ArticulationType.REVOLUTE
        and right_kingpin.articulation_type == ArticulationType.REVOLUTE
        and tuple(left_kingpin.axis) == (0.0, 0.0, 1.0)
        and tuple(right_kingpin.axis) == (0.0, 0.0, 1.0),
        details=(
            f"left={left_kingpin.articulation_type}:{left_kingpin.axis}, "
            f"right={right_kingpin.articulation_type}:{right_kingpin.axis}"
        ),
    )
    ctx.check(
        "hub axles are lateral continuous joints",
        left_axle.articulation_type == ArticulationType.CONTINUOUS
        and right_axle.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_axle.axis) == (-1.0, 0.0, 0.0)
        and tuple(right_axle.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_axle.axis}, right={right_axle.axis}",
    )

    left_rest = ctx.part_world_position(left_hub)
    right_rest = ctx.part_world_position(right_hub)
    steering_pose = {left_kingpin: 0.50, right_kingpin: 0.50}
    with ctx.pose(steering_pose):
        left_steered = ctx.part_world_position(left_hub)
        right_steered = ctx.part_world_position(right_hub)
        ctx.expect_gap(
            beam,
            left_hub,
            axis="x",
            min_gap=0.08,
            name="left hub stays outboard of the beam at steering lock",
        )
        ctx.expect_gap(
            right_hub,
            beam,
            axis="x",
            min_gap=0.08,
            name="right hub stays outboard of the beam at steering lock",
        )

    ctx.check(
        "left steering joint swings the hub around the kingpin without lifting it",
        left_rest is not None
        and left_steered is not None
        and abs(left_steered[1] - left_rest[1]) > 0.04
        and abs(left_steered[2] - left_rest[2]) < 0.005,
        details=f"rest={left_rest}, steered={left_steered}",
    )
    ctx.check(
        "right steering joint swings the hub around the kingpin without lifting it",
        right_rest is not None
        and right_steered is not None
        and abs(right_steered[1] - right_rest[1]) > 0.04
        and abs(right_steered[2] - right_rest[2]) < 0.005,
        details=f"rest={right_rest}, steered={right_steered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
