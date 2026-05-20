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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave_oven")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.14, 0.15, 0.17, 1.0))
    door_black = model.material("door_black", rgba=(0.09, 0.10, 0.11, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.52, 0.61, 0.67, 0.28))
    knob_silver = model.material("knob_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    ring_gray = model.material("ring_gray", rgba=(0.56, 0.58, 0.60, 1.0))

    outer_w = 0.520
    outer_d = 0.420
    outer_h = 0.310
    shell_t = 0.018
    inner_h = outer_h - (2.0 * shell_t)
    front_trim_d = 0.012
    partition_t = 0.014

    left_opening_edge = -0.224
    partition_center_x = 0.134
    partition_left_face = partition_center_x - (partition_t * 0.5)
    partition_right_face = partition_center_x + (partition_t * 0.5)
    opening_right_edge = partition_left_face
    opening_bottom = 0.052
    opening_top = 0.258
    opening_center_z = (opening_bottom + opening_top) * 0.5

    control_fascia_w = (outer_w * 0.5 - shell_t) - partition_right_face
    control_fascia_center_x = partition_right_face + (control_fascia_w * 0.5)

    door_axis_x = left_opening_edge + 0.004
    door_axis_y = (outer_d * 0.5) + 0.004
    door_w = 0.350
    door_h = 0.212
    door_t = 0.028
    door_stile_w = 0.026
    door_rail_h = 0.030
    hinge_pin_r = 0.004

    dial_x = control_fascia_center_x
    upper_dial_z = 0.205
    lower_dial_z = 0.125
    dial_rpy = (-math.pi / 2.0, 0.0, 0.0)

    turntable_x = (left_opening_edge + opening_right_edge) * 0.5
    turntable_y = 0.000
    support_ring_center_z = shell_t + 0.004
    turntable_axis_z = shell_t + 0.008

    support_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.104,
            tube=0.004,
            radial_segments=16,
            tubular_segments=48,
        ),
        "microwave_support_ring",
    )
    front_bezel_outer = rounded_rect_profile(outer_w - 0.008, outer_h - 0.012, 0.014, corner_segments=8)
    front_bezel_hole = [
        (x + turntable_x, y)
        for x, y in rounded_rect_profile(
            (opening_right_edge - left_opening_edge) + 0.010,
            (opening_top - opening_bottom) + 0.010,
            0.010,
            corner_segments=8,
        )
    ]
    front_bezel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            front_bezel_outer,
            [front_bezel_hole],
            0.004,
            center=True,
        ),
        "microwave_front_bezel",
    )
    door_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(door_w, door_h, 0.015, corner_segments=8),
            [
                rounded_rect_profile(
                    door_w - 0.090,
                    door_h - 0.078,
                    0.010,
                    corner_segments=8,
                )
            ],
            0.004,
            center=True,
        ),
        "microwave_door_panel",
    )

    housing = model.part("housing")
    housing.visual(
        Box((outer_w, outer_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t * 0.5)),
        material=shell_white,
        name="shell_bottom",
    )
    housing.visual(
        Box((outer_w, outer_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, outer_h - (shell_t * 0.5))),
        material=shell_white,
        name="shell_top",
    )
    housing.visual(
        Box((shell_t, outer_d, inner_h)),
        origin=Origin(xyz=(-(outer_w * 0.5) + (shell_t * 0.5), 0.0, outer_h * 0.5)),
        material=shell_white,
        name="shell_left",
    )
    housing.visual(
        Box((shell_t, outer_d, inner_h)),
        origin=Origin(xyz=((outer_w * 0.5) - (shell_t * 0.5), 0.0, outer_h * 0.5)),
        material=shell_white,
        name="shell_right",
    )
    housing.visual(
        Box((outer_w - (2.0 * shell_t), shell_t, inner_h)),
        origin=Origin(xyz=(0.0, -(outer_d * 0.5) + (shell_t * 0.5), outer_h * 0.5)),
        material=shell_white,
        name="shell_back",
    )
    housing.visual(
        Box((partition_t, outer_d - shell_t, inner_h)),
        origin=Origin(
            xyz=(
                partition_center_x,
                ((-(outer_d * 0.5) + shell_t) + (outer_d * 0.5)) * 0.5,
                outer_h * 0.5,
            )
        ),
        material=shell_white,
        name="control_partition",
    )
    housing.visual(
        Box((opening_right_edge - (-(outer_w * 0.5) + shell_t), front_trim_d, outer_h - opening_top)),
        origin=Origin(
            xyz=(
                ((-(outer_w * 0.5) + shell_t) + opening_right_edge) * 0.5,
                (outer_d * 0.5) - (front_trim_d * 0.5),
                (opening_top + (outer_h - shell_t)) * 0.5,
            )
        ),
        material=shell_white,
        name="front_header",
    )
    housing.visual(
        Box((opening_right_edge - (-(outer_w * 0.5) + shell_t), front_trim_d, opening_bottom - shell_t)),
        origin=Origin(
            xyz=(
                ((-(outer_w * 0.5) + shell_t) + opening_right_edge) * 0.5,
                (outer_d * 0.5) - (front_trim_d * 0.5),
                (shell_t + opening_bottom) * 0.5,
            )
        ),
        material=shell_white,
        name="front_sill",
    )
    housing.visual(
        Box((left_opening_edge - (-(outer_w * 0.5) + shell_t), front_trim_d, opening_top - opening_bottom)),
        origin=Origin(
            xyz=(
                ((-(outer_w * 0.5) + shell_t) + left_opening_edge) * 0.5,
                (outer_d * 0.5) - (front_trim_d * 0.5),
                opening_center_z,
            )
        ),
        material=shell_white,
        name="left_jamb",
    )
    housing.visual(
        Box((control_fascia_w, front_trim_d, inner_h)),
        origin=Origin(
            xyz=(
                control_fascia_center_x,
                (outer_d * 0.5) - (front_trim_d * 0.5),
                outer_h * 0.5,
            )
        ),
        material=fascia_black,
        name="control_fascia",
    )
    housing.visual(
        front_bezel_mesh,
        origin=Origin(
            xyz=(0.0, (outer_d * 0.5) - 0.002, outer_h * 0.5),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell_white,
        name="front_bezel",
    )
    housing.visual(
        support_ring_mesh,
        origin=Origin(xyz=(turntable_x, turntable_y, support_ring_center_z)),
        material=ring_gray,
        name="support_ring",
    )
    housing.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(turntable_x, turntable_y, support_ring_center_z)),
        material=ring_gray,
        name="drive_post",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.175, -0.145),
            (-0.175, 0.145),
            (0.175, -0.145),
            (0.175, 0.145),
        )
    ):
        housing.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(foot_x, foot_y, -0.004)),
            material=knob_dark,
            name=f"foot_{foot_index}",
        )
    for vent_index in range(6):
        housing.visual(
            Box((0.170, 0.003, 0.008)),
            origin=Origin(
                xyz=(
                    -0.010,
                    -(outer_d * 0.5) + 0.0015,
                    0.086 + (vent_index * 0.026),
                )
            ),
            material=fascia_black,
            name=f"rear_vent_{vent_index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h)),
        mass=15.5,
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5)),
    )

    door = model.part("door")
    door.visual(
        Cylinder(radius=hinge_pin_r, length=inner_h),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=knob_dark,
        name="hinge_pin",
    )
    door.visual(
        Box((door_stile_w, door_t, door_h)),
        origin=Origin(xyz=(door_stile_w * 0.5, 0.010, 0.0)),
        material=door_black,
        name="left_stile",
    )
    door.visual(
        Box((door_stile_w, door_t, door_h)),
        origin=Origin(xyz=(door_w - (door_stile_w * 0.5), 0.010, 0.0)),
        material=door_black,
        name="right_stile",
    )
    door.visual(
        Box((door_w - (2.0 * door_stile_w), door_t, door_rail_h)),
        origin=Origin(
            xyz=(
                door_w * 0.5,
                0.010,
                (door_h * 0.5) - (door_rail_h * 0.5),
            )
        ),
        material=door_black,
        name="top_rail",
    )
    door.visual(
        Box((door_w - (2.0 * door_stile_w), door_t, door_rail_h)),
        origin=Origin(
            xyz=(
                door_w * 0.5,
                0.010,
                -(door_h * 0.5) + (door_rail_h * 0.5),
            )
        ),
        material=door_black,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w - (2.0 * door_stile_w), 0.004, door_h - (2.0 * door_rail_h))),
        origin=Origin(xyz=(door_w * 0.5, 0.006, 0.0)),
        material=glass_tint,
        name="window_glass",
    )
    door.visual(
        door_panel_mesh,
        origin=Origin(
            xyz=(door_w * 0.5, 0.022, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=door_black,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(
            xyz=(door_w - 0.028, 0.031, 0.050),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_silver,
        name="handle_upper_post",
    )
    door.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=Origin(
            xyz=(door_w - 0.028, 0.031, -0.050),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_silver,
        name="handle_lower_post",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.138),
        origin=Origin(xyz=(door_w - 0.018, 0.041, 0.0)),
        material=knob_silver,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=2.1,
        origin=Origin(xyz=(door_w * 0.5, 0.010, 0.0)),
    )

    upper_dial = model.part("upper_dial")
    upper_dial.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=dial_rpy),
        material=knob_silver,
        name="knob_body",
    )
    upper_dial.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=dial_rpy),
        material=knob_dark,
        name="knob_cap",
    )
    upper_dial.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.019, 0.012)),
        material=shell_white,
        name="pointer_mark",
    )
    upper_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.018),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=dial_rpy),
    )

    lower_dial = model.part("lower_dial")
    lower_dial.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=dial_rpy),
        material=knob_silver,
        name="knob_body",
    )
    lower_dial.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=dial_rpy),
        material=knob_dark,
        name="knob_cap",
    )
    lower_dial.visual(
        Box((0.004, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.019, 0.012)),
        material=shell_white,
        name="pointer_mark",
    )
    lower_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.018),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=dial_rpy),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.135, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass_tint,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=glass_tint,
        name="center_boss",
    )
    turntable.visual(
        Box((0.016, 0.034, 0.002)),
        origin=Origin(xyz=(0.052, 0.0, 0.005)),
        material=glass_tint,
        name="plate_marker",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.135, length=0.006),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(door_axis_x, door_axis_y, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.92,
        ),
    )
    model.articulation(
        "upper_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=upper_dial,
        origin=Origin(xyz=(dial_x, outer_d * 0.5, upper_dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "lower_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=lower_dial,
        origin=Origin(xyz=(dial_x, outer_d * 0.5, lower_dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(turntable_x, turntable_y, turntable_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    upper_dial = object_model.get_part("upper_dial")
    lower_dial = object_model.get_part("lower_dial")
    turntable = object_model.get_part("turntable")

    door_hinge = object_model.get_articulation("door_hinge")
    upper_dial_spin = object_model.get_articulation("upper_dial_spin")
    lower_dial_spin = object_model.get_articulation("lower_dial_spin")
    turntable_spin = object_model.get_articulation("turntable_spin")

    shell_top = housing.get_visual("shell_top")
    shell_bottom = housing.get_visual("shell_bottom")
    front_header = housing.get_visual("front_header")
    front_sill = housing.get_visual("front_sill")
    control_fascia = housing.get_visual("control_fascia")
    support_ring = housing.get_visual("support_ring")
    drive_post = housing.get_visual("drive_post")

    hinge_pin = door.get_visual("hinge_pin")
    top_rail = door.get_visual("top_rail")
    bottom_rail = door.get_visual("bottom_rail")
    knob_body_upper = upper_dial.get_visual("knob_body")
    knob_body_lower = lower_dial.get_visual("knob_body")
    glass_plate = turntable.get_visual("glass_plate")

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
        "door_hinge_is_vertical_revolute",
        door_hinge.articulation_type == ArticulationType.REVOLUTE and door_hinge.axis == (0.0, 0.0, 1.0),
        details=f"door hinge type/axis was {door_hinge.articulation_type} / {door_hinge.axis}",
    )
    ctx.check(
        "upper_dial_axis_front_to_back",
        upper_dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and upper_dial_spin.axis == (0.0, 1.0, 0.0),
        details=f"upper dial type/axis was {upper_dial_spin.articulation_type} / {upper_dial_spin.axis}",
    )
    ctx.check(
        "lower_dial_axis_front_to_back",
        lower_dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and lower_dial_spin.axis == (0.0, 1.0, 0.0),
        details=f"lower dial type/axis was {lower_dial_spin.articulation_type} / {lower_dial_spin.axis}",
    )
    ctx.check(
        "turntable_axis_vertical_continuous",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS
        and turntable_spin.axis == (0.0, 0.0, 1.0),
        details=f"turntable type/axis was {turntable_spin.articulation_type} / {turntable_spin.axis}",
    )
    ctx.check(
        "door_opening_range_realistic",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.7 <= door_hinge.motion_limits.upper <= 2.05,
        details=f"door limits were {door_hinge.motion_limits}",
    )

    ctx.expect_origin_gap(
        upper_dial,
        lower_dial,
        axis="z",
        min_gap=0.06,
        max_gap=0.10,
        name="dials_are_stacked_vertically",
    )
    ctx.expect_within(
        upper_dial,
        housing,
        axes="xz",
        inner_elem=knob_body_upper,
        outer_elem=control_fascia,
        margin=0.0,
        name="upper_dial_within_control_panel",
    )
    ctx.expect_within(
        lower_dial,
        housing,
        axes="xz",
        inner_elem=knob_body_lower,
        outer_elem=control_fascia,
        margin=0.0,
        name="lower_dial_within_control_panel",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(
            door,
            housing,
            elem_a=hinge_pin,
            elem_b=shell_top,
            name="door_closed_hinge_pin_contacts_top_shell",
        )
        ctx.expect_contact(
            door,
            housing,
            elem_a=hinge_pin,
            elem_b=shell_bottom,
            name="door_closed_hinge_pin_contacts_bottom_shell",
        )
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=top_rail,
            negative_elem=front_header,
            name="door_closed_against_header",
        )
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=bottom_rail,
            negative_elem=front_sill,
            name="door_closed_against_sill",
        )
        ctx.expect_gap(
            upper_dial,
            housing,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=knob_body_upper,
            negative_elem=control_fascia,
            name="upper_dial_seated_on_fascia",
        )
        ctx.expect_gap(
            lower_dial,
            housing,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=knob_body_lower,
            negative_elem=control_fascia,
            name="lower_dial_seated_on_fascia",
        )
        ctx.expect_contact(
            turntable,
            housing,
            elem_a=glass_plate,
            elem_b=drive_post,
            name="turntable_contacts_drive_post",
        )
        ctx.expect_gap(
            turntable,
            housing,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=glass_plate,
            negative_elem=support_ring,
            name="turntable_sits_on_support_ring",
        )

    turntable_pos = ctx.part_world_position(turntable)
    if turntable_pos is not None:
        ctx.check(
            "turntable_centered_in_cavity",
            abs(turntable_pos[0] + 0.0485) <= 0.002 and abs(turntable_pos[1]) <= 0.002,
            details=f"turntable origin was at {turntable_pos}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose(
            {
                door_hinge: door_limits.upper,
                upper_dial_spin: math.pi / 2.0,
                lower_dial_spin: -math.pi / 2.0,
                turntable_spin: math.pi / 3.0,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="operating_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="operating_pose_no_floating")
            ctx.expect_contact(
                door,
                housing,
                elem_a=hinge_pin,
                elem_b=shell_top,
                name="door_open_hinge_pin_contacts_top_shell",
            )
            ctx.expect_contact(
                door,
                housing,
                elem_a=hinge_pin,
                elem_b=shell_bottom,
                name="door_open_hinge_pin_contacts_bottom_shell",
            )
            ctx.expect_gap(
                upper_dial,
                housing,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=knob_body_upper,
                negative_elem=control_fascia,
                name="upper_dial_rotated_stays_seated",
            )
            ctx.expect_gap(
                lower_dial,
                housing,
                axis="y",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=knob_body_lower,
                negative_elem=control_fascia,
                name="lower_dial_rotated_stays_seated",
            )
            ctx.expect_contact(
                turntable,
                housing,
                elem_a=glass_plate,
                elem_b=drive_post,
                name="turntable_rotated_stays_supported",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
