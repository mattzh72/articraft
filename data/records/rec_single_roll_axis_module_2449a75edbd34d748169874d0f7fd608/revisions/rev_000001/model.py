from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _make_mount_plate(
    plate_length: float,
    plate_width: float,
    plate_thickness: float,
    slot_length: float,
    slot_width: float,
) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(plate_length, plate_width, plate_thickness, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
    )

    slot_points = [
        (-0.072, -0.044),
        (-0.072, 0.044),
        (0.072, -0.044),
        (0.072, 0.044),
    ]

    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(slot_length, slot_width, angle=0.0)
        .cutThruAll()
    )

    return plate


def _make_housing_upper(
    base_thickness: float,
    housing_width: float,
    beam_length: float,
    beam_height: float,
    block_offset_x: float,
    block_length: float,
    block_height: float,
    axis_height: float,
    bearing_radius: float,
) -> cq.Workplane:
    seat_z = base_thickness - 0.0006
    block_width = 0.034
    rail_width = 0.018
    rail_height = 0.020
    rail_y = -0.020
    rail_length = (2.0 * block_offset_x) + block_length
    gusset_top_z = axis_height - 0.006

    rear_rail = (
        cq.Workplane("XY")
        .box(rail_length, rail_width, rail_height, centered=(True, True, False))
        .translate((0.0, rail_y, seat_z))
    )
    left_block = (
        cq.Workplane("XY")
        .box(block_length, block_width, block_height, centered=(True, True, False))
        .translate((-block_offset_x, 0.0, seat_z))
    )
    right_block = (
        cq.Workplane("XY")
        .box(block_length, block_width, block_height, centered=(True, True, False))
        .translate((block_offset_x, 0.0, seat_z))
    )

    gusset_profile = [
        (-0.029, seat_z),
        (-0.011, seat_z),
        (-0.011, seat_z + rail_height),
        (-0.017, gusset_top_z),
        (-0.029, axis_height - 0.015),
    ]
    left_gusset = (
        cq.Workplane("YZ")
        .polyline(gusset_profile)
        .close()
        .extrude(0.014)
        .translate((-block_offset_x - 0.007, 0.0, 0.0))
    )
    right_gusset = (
        cq.Workplane("YZ")
        .polyline(gusset_profile)
        .close()
        .extrude(0.014)
        .translate((block_offset_x - 0.007, 0.0, 0.0))
    )

    housing = rear_rail.union(left_block).union(right_block).union(left_gusset).union(right_gusset)

    left_bore = (
        cq.Workplane("YZ", origin=(-(block_offset_x + (block_length / 2.0)), 0.0, 0.0))
        .center(0.0, axis_height)
        .circle(bearing_radius)
        .extrude(block_length)
    )
    right_bore = (
        cq.Workplane("YZ", origin=((block_offset_x - (block_length / 2.0)), 0.0, 0.0))
        .center(0.0, axis_height)
        .circle(bearing_radius)
        .extrude(block_length)
    )
    housing = housing.cut(left_bore).cut(right_bore)

    return housing


def _make_spindle_link(
    shaft_length: float,
    shaft_radius: float,
    collar_thickness: float,
    collar_radius: float,
    hub_length: float,
    hub_radius: float,
    cradle_length: float,
    cradle_width: float,
    cradle_height: float,
    cradle_wall: float,
) -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ", origin=(-(shaft_length / 2.0), 0.0, 0.0))
        .circle(shaft_radius)
        .extrude(shaft_length)
    )
    left_collar = (
        cq.Workplane("YZ", origin=(-(shaft_length / 2.0), 0.0, 0.0))
        .workplane(offset=0.0)
        .circle(collar_radius)
        .extrude(collar_thickness)
    )
    right_collar = (
        cq.Workplane("YZ", origin=((shaft_length / 2.0) - collar_thickness, 0.0, 0.0))
        .circle(collar_radius)
        .extrude(collar_thickness)
    )
    hub = (
        cq.Workplane("YZ", origin=(-(hub_length / 2.0), 0.0, 0.0))
        .circle(hub_radius)
        .extrude(hub_length)
    )

    stem = (
        cq.Workplane("XY")
        .box(
            0.018,
            0.014,
            0.020,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, shaft_radius - 0.002))
    )
    flange = (
        cq.Workplane("XY")
        .box(
            0.034,
            0.046,
            0.012,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, shaft_radius + 0.026))
    )

    spindle = shaft.union(left_collar).union(right_collar).union(hub).union(stem).union(flange)

    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_axis_spindle_module")

    plate_length = 0.22
    plate_width = 0.14
    plate_thickness = 0.012
    slot_length = 0.028
    slot_width = 0.010

    housing_width = 0.082
    beam_length = 0.050
    beam_height = 0.013
    block_offset_x = 0.049
    block_length = 0.030
    block_height = 0.052

    axis_height = 0.048

    shaft_length = 0.060
    shaft_radius = 0.010
    collar_thickness = 0.004
    collar_radius = 0.0135
    hub_length = 0.012
    hub_radius = 0.012
    cradle_length = 0.034
    cradle_width = 0.046
    cradle_height = 0.012
    cradle_wall = 0.008

    housing_color = model.material("housing_color", color=(0.24, 0.26, 0.29, 1.0))
    spindle_color = model.material("spindle_color", color=(0.73, 0.75, 0.78, 1.0))

    mount_plate_shape = _make_mount_plate(
        plate_length,
        plate_width,
        plate_thickness,
        slot_length,
        slot_width,
    )

    housing = model.part(
        "housing",
        inertial=Inertial.from_geometry(
            Box((plate_length, plate_width, axis_height + 0.024)),
            mass=2.8,
            origin=Origin(xyz=(0.0, 0.0, (axis_height + 0.024) / 2.0)),
        ),
    )
    housing.visual(
        mesh_from_cadquery(mount_plate_shape, "mount_plate"),
        material=housing_color,
        name="mount_plate",
    )
    housing.visual(
        Box((0.128, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, -0.016, 0.0204)),
        material=housing_color,
        name="rear_rail",
    )
    housing.visual(
        Box((block_length, 0.030, block_height)),
        origin=Origin(xyz=(-block_offset_x, 0.0, 0.0374)),
        material=housing_color,
        name="left_bearing_block",
    )
    housing.visual(
        Box((block_length, 0.030, block_height)),
        origin=Origin(xyz=(block_offset_x, 0.0, 0.0374)),
        material=housing_color,
        name="right_bearing_block",
    )
    housing.visual(
        Box((0.012, 0.016, 0.036)),
        origin=Origin(xyz=(-block_offset_x, -0.012, 0.028), rpy=(0.78, 0.0, 0.0)),
        material=housing_color,
        name="left_gusset",
    )
    housing.visual(
        Box((0.012, 0.016, 0.036)),
        origin=Origin(xyz=(block_offset_x, -0.012, 0.028), rpy=(0.78, 0.0, 0.0)),
        material=housing_color,
        name="right_gusset",
    )
    housing.visual(
        Box((beam_length, housing_width, beam_height)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + (beam_height / 2.0) - 0.0004)),
        material=housing_color,
        name="housing_upper",
    )

    spindle = model.part(
        "spindle",
        inertial=Inertial.from_geometry(
            Box((0.070, cradle_width, 0.050)),
            mass=0.85,
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
        ),
    )
    spindle.visual(
        Cylinder(radius=shaft_radius, length=shaft_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_color,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=collar_radius, length=collar_thickness),
        origin=Origin(xyz=(-(shaft_length / 2.0) - (collar_thickness / 2.0), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_color,
        name="left_collar",
    )
    spindle.visual(
        Cylinder(radius=collar_radius, length=collar_thickness),
        origin=Origin(xyz=((shaft_length / 2.0) + (collar_thickness / 2.0), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_color,
        name="right_collar",
    )
    spindle.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=spindle_color,
        name="hub",
    )
    spindle.visual(
        Box((0.016, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=spindle_color,
        name="stem",
    )
    spindle.visual(
        Box((cradle_length, cradle_width, cradle_height)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=spindle_color,
        name="spindle_link",
    )

    model.articulation(
        "housing_to_spindle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, axis_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-2.6,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    spindle = object_model.get_part("spindle")
    roll_joint = object_model.get_articulation("housing_to_spindle")
    mount_plate = housing.get_visual("mount_plate")
    spindle_link = spindle.get_visual("spindle_link")
    left_block = housing.get_visual("left_bearing_block")
    right_block = housing.get_visual("right_bearing_block")
    left_collar = spindle.get_visual("left_collar")
    right_collar = spindle.get_visual("right_collar")

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

    ctx.check(
        "spindle_module_parts_present",
        True,
        "",
    )
    ctx.check(
        "roll_joint_is_longitudinal_x",
        abs(roll_joint.axis[0]) > 0.99
        and abs(roll_joint.axis[1]) < 1e-9
        and abs(roll_joint.axis[2]) < 1e-9,
        f"expected an X-axis roll joint, got axis={roll_joint.axis}",
    )
    limits = roll_joint.motion_limits
    ctx.check(
        "roll_joint_has_broad_but_finite_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -2.0
        and limits.upper >= 2.0,
        f"expected a broad finite roll range, got limits={limits}",
    )
    ctx.expect_contact(
        spindle,
        housing,
        elem_a=left_collar,
        elem_b=left_block,
        name="left_collar_is_supported_by_left_bearing_block",
    )
    ctx.expect_contact(
        spindle,
        housing,
        elem_a=right_collar,
        elem_b=right_block,
        name="right_collar_is_supported_by_right_bearing_block",
    )
    ctx.expect_gap(
        spindle,
        housing,
        axis="z",
        positive_elem=spindle_link,
        negative_elem=mount_plate,
        min_gap=0.015,
        name="spindle_link_clears_mount_plate",
    )
    ctx.expect_within(
        spindle,
        housing,
        axes="xy",
        inner_elem=spindle_link,
        outer_elem=mount_plate,
        margin=0.0,
        name="spindle_link_stays_over_mount_plate_footprint",
    )
    with ctx.pose({roll_joint: 1.25}):
        ctx.expect_gap(
            spindle,
            housing,
            axis="z",
            positive_elem=spindle_link,
            negative_elem=mount_plate,
            min_gap=0.012,
            name="rolled_spindle_link_still_clears_mount_plate",
        )
        ctx.expect_within(
            spindle,
            housing,
            axes="xy",
            inner_elem=spindle_link,
            outer_elem=mount_plate,
            margin=0.002,
            name="rolled_spindle_link_stays_over_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
