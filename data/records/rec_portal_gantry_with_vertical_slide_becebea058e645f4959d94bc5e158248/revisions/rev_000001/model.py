from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


X_TRAVEL = 0.75
Z_TRAVEL = 0.55


def _add_box(part, size, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_robot_axis")

    model.material("powder_aluminum", rgba=(0.66, 0.68, 0.70, 1.0))
    model.material("dark_anodized", rgba=(0.10, 0.11, 0.13, 1.0))
    model.material("rail_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    model.material("brushed_plate", rgba=(0.78, 0.79, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.015, 0.014, 1.0))
    model.material("safety_yellow", rgba=(0.95, 0.72, 0.12, 1.0))
    model.material("cable_black", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("blue_cover", rgba=(0.12, 0.26, 0.52, 1.0))

    frame = model.part("portal_frame")

    # Grounded side frames: each side has front/rear uprights, a floor skid,
    # a top crosshead, and X braces, all tied into the tall top beam.
    _add_box(frame, (2.80, 0.28, 0.26), (0.0, 0.0, 2.16), "powder_aluminum", "top_beam")
    _add_box(frame, (2.72, 0.06, 0.06), (0.0, -0.17, 2.31), "dark_anodized", "beam_front_lip")

    for side, x in (("neg", -1.25), ("pos", 1.25)):
        _add_box(frame, (0.30, 0.78, 0.08), (x, 0.0, 0.04), "dark_anodized", f"{side}_floor_skid")
        _add_box(frame, (0.14, 0.14, 2.04), (x, -0.30, 1.06), "powder_aluminum", f"{side}_front_leg")
        _add_box(frame, (0.14, 0.14, 2.04), (x, 0.30, 1.06), "powder_aluminum", f"{side}_rear_leg")
        _add_box(frame, (0.22, 0.78, 0.14), (x, 0.0, 2.02), "powder_aluminum", f"{side}_top_crosshead")
        _add_box(frame, (0.16, 0.30, 0.035), (x, -0.30, 0.105), "rail_steel", f"{side}_front_foot_plate")
        _add_box(frame, (0.16, 0.30, 0.035), (x, 0.30, 0.105), "rail_steel", f"{side}_rear_foot_plate")
        _add_box(
            frame,
            (0.065, 1.16, 0.055),
            (x, 0.0, 0.92),
            "powder_aluminum",
            f"{side}_brace_a",
            rpy=(1.06, 0.0, 0.0),
        )
        _add_box(
            frame,
            (0.065, 1.16, 0.055),
            (x, 0.0, 0.92),
            "powder_aluminum",
            f"{side}_brace_b",
            rpy=(-1.06, 0.0, 0.0),
        )

    # Fixed machined guide interface on the beam front.  The rider bearing shoes
    # run just in front of these raised rails with a small inspection gap.
    _add_box(frame, (2.20, 0.024, 0.035), (0.0, -0.152, 2.235), "rail_steel", "upper_rail")
    _add_box(frame, (2.20, 0.024, 0.035), (0.0, -0.152, 2.065), "rail_steel", "lower_rail")
    _add_box(frame, (2.18, 0.018, 0.020), (0.0, -0.181, 2.150), "rail_steel", "rack_strip")

    # End buffers are mounted to the rail face and keep the X carriage well
    # inside the side frames.
    _add_box(frame, (0.070, 0.080, 0.180), (-1.05, -0.188, 2.150), "rubber_black", "end_buffer_neg")
    _add_box(frame, (0.070, 0.080, 0.180), (1.05, -0.188, 2.150), "rubber_black", "end_buffer_pos")
    _add_box(frame, (0.045, 0.055, 0.110), (-1.080, -0.172, 2.150), "safety_yellow", "stop_flag_neg")
    _add_box(frame, (0.045, 0.055, 0.110), (1.080, -0.172, 2.150), "safety_yellow", "stop_flag_pos")

    # Bracketed cable tray on the rear top of the portal.  Small brackets touch
    # both the beam and tray, so the tray reads as bolted on rather than floating.
    _add_box(frame, (2.30, 0.10, 0.080), (0.0, 0.205, 2.365), "cable_black", "rear_cable_channel")
    for i, x in enumerate((-0.90, -0.45, 0.0, 0.45, 0.90)):
        _add_box(frame, (0.070, 0.055, 0.140), (x, 0.155, 2.305), "dark_anodized", f"tray_bracket_{i}")
        _add_box(frame, (0.050, 0.115, 0.018), (x, 0.190, 2.320), "rail_steel", f"tray_strap_{i}")

    rider = model.part("beam_rider")
    # Compact horizontal stage: a saddle housing, bearing shoes close to the beam
    # rails, a front Z-guide face, side cheeks, and a moving-cable pickup.
    _add_box(rider, (0.38, 0.160, 0.380), (0.0, 0.0, 0.0), "dark_anodized", "rider_housing")
    _add_box(rider, (0.34, 0.030, 0.310), (0.0, -0.091, 0.0), "blue_cover", "front_cover")
    _add_box(rider, (0.16, 0.034, 0.060), (-0.09, 0.095, 0.075), "brushed_plate", "upper_bearing_neg")
    _add_box(rider, (0.16, 0.034, 0.060), (0.09, 0.095, 0.075), "brushed_plate", "upper_bearing_pos")
    _add_box(rider, (0.16, 0.034, 0.060), (-0.09, 0.095, -0.095), "brushed_plate", "lower_bearing_neg")
    _add_box(rider, (0.16, 0.034, 0.060), (0.09, 0.095, -0.095), "brushed_plate", "lower_bearing_pos")
    _add_box(rider, (0.080, 0.020, 0.032), (-0.09, 0.106, 0.075), "rail_steel", "upper_roller_neg")
    _add_box(rider, (0.080, 0.020, 0.032), (0.09, 0.106, 0.075), "rail_steel", "upper_roller_pos")
    _add_box(rider, (0.080, 0.020, 0.032), (-0.09, 0.106, -0.095), "rail_steel", "lower_roller_neg")
    _add_box(rider, (0.080, 0.020, 0.032), (0.09, 0.106, -0.095), "rail_steel", "lower_roller_pos")
    _add_box(rider, (0.080, 0.027, 0.040), (0.0, 0.072, -0.004), "rail_steel", "pinion_window")

    _add_box(rider, (0.035, 0.055, 0.650), (-0.105, -0.105, -0.105), "rail_steel", "z_guide_neg")
    _add_box(rider, (0.035, 0.055, 0.650), (0.105, -0.105, -0.105), "rail_steel", "z_guide_pos")
    _add_box(rider, (0.285, 0.030, 0.035), (0.0, -0.108, 0.225), "dark_anodized", "top_guide_tie")
    _add_box(rider, (0.285, 0.045, 0.035), (0.0, -0.060, -0.440), "dark_anodized", "bottom_guide_tie")
    _add_box(rider, (0.050, 0.035, 0.050), (-0.135, -0.128, 0.170), "rubber_black", "z_stop_top_neg")
    _add_box(rider, (0.050, 0.035, 0.050), (0.135, -0.128, 0.170), "rubber_black", "z_stop_top_pos")
    _add_box(rider, (0.050, 0.035, 0.050), (-0.135, -0.128, -0.410), "rubber_black", "z_stop_bottom_neg")
    _add_box(rider, (0.050, 0.035, 0.050), (0.135, -0.128, -0.410), "rubber_black", "z_stop_bottom_pos")

    _add_box(rider, (0.25, 0.070, 0.045), (0.0, -0.015, 0.235), "cable_black", "moving_cable_channel")
    for i, x in enumerate((-0.105, 0.0, 0.105)):
        _add_box(rider, (0.018, 0.080, 0.053), (x, -0.015, 0.235), "rail_steel", f"cable_channel_band_{i}")
    _add_box(rider, (0.070, 0.070, 0.090), (0.155, -0.020, 0.185), "dark_anodized", "cable_pickup_bracket")

    z_carriage = model.part("z_carriage")
    # The Z stage is a separate nested slide riding in front of the rider face.
    # It remains retained in the guide at full stroke and carries a lower tool
    # mount to make the vertical stage unambiguous.
    _add_box(z_carriage, (0.135, 0.028, 0.600), (0.0, 0.0, -0.250), "brushed_plate", "slide_plate")
    _add_box(z_carriage, (0.020, 0.058, 0.085), (-0.0775, 0.014, -0.080), "rail_steel", "z_bearing_neg_top")
    _add_box(z_carriage, (0.020, 0.058, 0.085), (0.0775, 0.014, -0.080), "rail_steel", "z_bearing_pos_top")
    _add_box(z_carriage, (0.020, 0.058, 0.085), (-0.0775, 0.014, -0.330), "rail_steel", "z_bearing_neg_bottom")
    _add_box(z_carriage, (0.020, 0.058, 0.085), (0.0775, 0.014, -0.330), "rail_steel", "z_bearing_pos_bottom")
    _add_box(z_carriage, (0.070, 0.032, 0.560), (0.0, -0.021, -0.255), "rail_steel", "center_stiffener")
    _add_box(z_carriage, (0.170, 0.070, 0.105), (0.0, -0.019, -0.580), "dark_anodized", "tool_mount")
    _add_box(z_carriage, (0.140, 0.040, 0.035), (0.0, -0.041, -0.515), "safety_yellow", "tool_clamp")
    _add_cylinder(
        z_carriage,
        radius=0.026,
        length=0.100,
        xyz=(0.0, -0.075, -0.580),
        material="rail_steel",
        name="tool_boss",
        rpy=(pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "portal_to_rider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rider,
        origin=Origin(xyz=(0.0, -0.280, 2.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.75, lower=-X_TRAVEL, upper=X_TRAVEL),
        motion_properties=MotionProperties(damping=8.0, friction=1.2),
    )
    model.articulation(
        "rider_to_z",
        ArticulationType.PRISMATIC,
        parent=rider,
        child=z_carriage,
        origin=Origin(xyz=(0.0, -0.150, 0.170)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.45, lower=0.0, upper=Z_TRAVEL),
        motion_properties=MotionProperties(damping=6.0, friction=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("portal_frame")
    rider = object_model.get_part("beam_rider")
    z_carriage = object_model.get_part("z_carriage")
    x_joint = object_model.get_articulation("portal_to_rider")
    z_joint = object_model.get_articulation("rider_to_z")

    ctx.check(
        "two ordered prismatic stages",
        x_joint.articulation_type == ArticulationType.PRISMATIC
        and z_joint.articulation_type == ArticulationType.PRISMATIC
        and x_joint.parent == "portal_frame"
        and x_joint.child == "beam_rider"
        and z_joint.parent == "beam_rider"
        and z_joint.child == "z_carriage",
        details="Expected fixed portal -> horizontal rider -> vertical Z carriage.",
    )

    ctx.expect_gap(
        frame,
        rider,
        axis="y",
        positive_elem="upper_rail",
        negative_elem="upper_bearing_neg",
        min_gap=0.002,
        max_gap=0.012,
        name="upper bearing clears beam rail",
    )
    ctx.expect_gap(
        rider,
        z_carriage,
        axis="y",
        positive_elem="z_guide_neg",
        negative_elem="slide_plate",
        min_gap=0.002,
        max_gap=0.010,
        name="nested z slide clears guide face",
    )
    ctx.expect_overlap(
        z_carriage,
        rider,
        axes="z",
        elem_a="slide_plate",
        elem_b="z_guide_neg",
        min_overlap=0.45,
        name="z slide retained in guides at rest",
    )

    rest_rider = ctx.part_world_position(rider)
    rest_z = ctx.part_world_position(z_carriage)

    with ctx.pose({x_joint: X_TRAVEL}):
        ctx.expect_gap(
            frame,
            rider,
            axis="x",
            positive_elem="end_buffer_pos",
            negative_elem="rider_housing",
            min_gap=0.040,
            name="right travel stops before buffer",
        )
        ctx.expect_overlap(
            rider,
            frame,
            axes="x",
            elem_a="upper_bearing_pos",
            elem_b="upper_rail",
            min_overlap=0.12,
            name="right end keeps rider on rail",
        )
        right_rider = ctx.part_world_position(rider)

    with ctx.pose({x_joint: -X_TRAVEL}):
        ctx.expect_gap(
            rider,
            frame,
            axis="x",
            positive_elem="rider_housing",
            negative_elem="end_buffer_neg",
            min_gap=0.040,
            name="left travel stops before buffer",
        )
        ctx.expect_overlap(
            rider,
            frame,
            axes="x",
            elem_a="upper_bearing_neg",
            elem_b="upper_rail",
            min_overlap=0.12,
            name="left end keeps rider on rail",
        )
        left_rider = ctx.part_world_position(rider)

    with ctx.pose({z_joint: Z_TRAVEL}):
        ctx.expect_gap(
            rider,
            z_carriage,
            axis="y",
            positive_elem="z_guide_pos",
            negative_elem="slide_plate",
            min_gap=0.002,
            max_gap=0.010,
            name="lowered z slide clears opposite guide",
        )
        ctx.expect_overlap(
            z_carriage,
            rider,
            axes="z",
            elem_a="slide_plate",
            elem_b="z_guide_pos",
            min_overlap=0.08,
            name="lowered z slide remains retained",
        )
        ctx.expect_gap(
            rider,
            z_carriage,
            axis="y",
            positive_elem="bottom_guide_tie",
            negative_elem="z_bearing_neg_top",
            min_gap=0.015,
            name="lowered bearing clears guide tie",
        )
        lowered_z = ctx.part_world_position(z_carriage)
        lowered_aabb = ctx.part_world_aabb(z_carriage)

    ctx.check(
        "rider translates horizontally",
        rest_rider is not None
        and right_rider is not None
        and left_rider is not None
        and right_rider[0] > rest_rider[0] + 0.70
        and left_rider[0] < rest_rider[0] - 0.70,
        details=f"rest={rest_rider}, right={right_rider}, left={left_rider}",
    )
    ctx.check(
        "z carriage translates downward",
        rest_z is not None and lowered_z is not None and lowered_z[2] < rest_z[2] - 0.50,
        details=f"rest={rest_z}, lowered={lowered_z}",
    )
    ctx.check(
        "lowered tool remains above floor",
        lowered_aabb is not None and lowered_aabb[0][2] > 0.90,
        details=f"lowered_aabb={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
