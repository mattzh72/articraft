from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_cooker_with_rotisserie")

    stainless = model.material("stainless", rgba=(0.77, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.17, 0.18, 1.0))
    glass = model.material("oven_glass", rgba=(0.13, 0.15, 0.17, 0.34))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.20, 0.20, 0.21, 1.0))
    burner_black = model.material("burner_black", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_metal = model.material("knob_metal", rgba=(0.64, 0.66, 0.69, 1.0))

    body_w = 0.90
    body_d = 0.64
    body_h = 0.92
    side_t = 0.025
    back_t = 0.020
    top_t = 0.030
    frame_t = 0.035
    toe_h = 0.10
    control_h = 0.11
    control_t = 0.050
    front_y = -body_d / 2.0

    door_w = 0.71
    door_h = 0.56
    door_t = 0.035
    hinge_y = front_y + frame_t
    hinge_z = 0.12

    opening_w = 0.73
    opening_h = 0.59
    side_rail_w = (body_w - opening_w) / 2.0
    upper_trim_h = 0.045
    front_section_h = 0.72
    front_section_z = front_section_h / 2.0
    upper_trim_z = hinge_z + opening_h + upper_trim_h / 2.0
    control_z = body_h - top_t - control_h / 2.0 - 0.01

    cavity_w = 0.72
    cavity_h = 0.46
    cavity_d = 0.51
    liner_t = 0.015
    cavity_center_y = 0.015
    cavity_bottom_z = 0.18
    cavity_center_z = cavity_bottom_z + cavity_h / 2.0
    cavity_back_inner_y = cavity_center_y + cavity_d / 2.0

    spit_len = 0.68
    spit_r = 0.006
    spit_z = cavity_center_z
    spit_support_t = 0.020

    burner_r = 0.035
    burner_h = 0.012
    cooktop_z = body_h - top_t / 2.0
    cooktop_top_z = body_h

    body = model.part("body")

    _add_box(
        body,
        "left_side_panel",
        (side_t, body_d, body_h - top_t + 0.004),
        (-body_w / 2.0 + side_t / 2.0, 0.0, (body_h - top_t + 0.004) / 2.0),
        stainless,
    )
    _add_box(
        body,
        "right_side_panel",
        (side_t, body_d, body_h - top_t + 0.004),
        (body_w / 2.0 - side_t / 2.0, 0.0, (body_h - top_t + 0.004) / 2.0),
        stainless,
    )
    _add_box(
        body,
        "rear_panel",
        (body_w - side_t + 0.010, back_t, body_h - top_t + 0.004),
        (0.0, body_d / 2.0 - back_t / 2.0, (body_h - top_t + 0.004) / 2.0),
        stainless,
    )
    _add_box(
        body,
        "cooktop",
        (body_w, body_d, top_t),
        (0.0, 0.0, cooktop_z),
        stainless,
    )
    _add_box(
        body,
        "backsplash",
        (body_w, 0.050, 0.080),
        (0.0, body_d / 2.0 - 0.025, body_h + 0.040),
        stainless,
    )
    _add_box(
        body,
        "toe_kick",
        (body_w, frame_t, toe_h),
        (0.0, front_y + frame_t / 2.0, toe_h / 2.0),
        dark_trim,
    )
    _add_box(
        body,
        "front_left_rail",
        (side_rail_w, frame_t, front_section_h),
        (-(opening_w / 2.0 + side_rail_w / 2.0), front_y + frame_t / 2.0, front_section_z),
        stainless,
    )
    _add_box(
        body,
        "front_right_rail",
        (side_rail_w, frame_t, front_section_h),
        ((opening_w / 2.0 + side_rail_w / 2.0), front_y + frame_t / 2.0, front_section_z),
        stainless,
    )
    _add_box(
        body,
        "front_upper_trim",
        (opening_w + 2.0 * side_rail_w, frame_t, upper_trim_h),
        (0.0, front_y + frame_t / 2.0, upper_trim_z),
        stainless,
    )
    _add_box(
        body,
        "control_panel",
        (body_w - 2.0 * side_t + 0.010, control_t, control_h),
        (0.0, front_y + control_t / 2.0 + 0.004, control_z),
        stainless,
    )

    for index, x_pos in enumerate((-0.28, -0.14, 0.0, 0.14, 0.28), start=1):
        _add_cylinder(
            body,
            f"control_knob_{index}",
            radius=0.021,
            length=0.032,
            xyz=(x_pos, front_y - 0.012, control_z + 0.010),
            material=knob_metal,
            rpy=(pi / 2.0, 0.0, 0.0),
        )

    for index, (x_pos, y_pos, radius) in enumerate(
        (
            (-0.24, -0.11, 0.032),
            (0.00, -0.11, 0.040),
            (0.24, -0.11, 0.032),
            (-0.18, 0.12, 0.032),
            (0.18, 0.12, 0.032),
        ),
        start=1,
    ):
        _add_cylinder(
            body,
            f"burner_cap_{index}",
            radius=radius,
            length=burner_h,
            xyz=(x_pos, y_pos, cooktop_top_z + burner_h / 2.0 - 0.001),
            material=burner_black,
        )

    for index, (x_pos, y_pos) in enumerate(((-0.18, -0.11), (0.18, -0.11), (0.0, 0.12)), start=1):
        _add_box(
            body,
            f"grate_longitudinal_{index}",
            (0.18, 0.010, 0.012),
            (x_pos, y_pos, cooktop_top_z + 0.006),
            dark_trim,
        )
        _add_box(
            body,
            f"grate_transverse_{index}",
            (0.010, 0.15, 0.012),
            (x_pos, y_pos, cooktop_top_z + 0.006),
            dark_trim,
        )

    _add_box(
        body,
        "liner_left",
        (liner_t, cavity_d, cavity_h + 2.0 * liner_t),
        (-(cavity_w / 2.0 + liner_t / 2.0), cavity_center_y, cavity_center_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_right",
        (liner_t, cavity_d, cavity_h + 2.0 * liner_t),
        ((cavity_w / 2.0 + liner_t / 2.0), cavity_center_y, cavity_center_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_floor",
        (cavity_w + 2.0 * liner_t, cavity_d, liner_t),
        (0.0, cavity_center_y, cavity_bottom_z - liner_t / 2.0),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_ceiling",
        (cavity_w + 2.0 * liner_t, cavity_d, liner_t),
        (0.0, cavity_center_y, cavity_bottom_z + cavity_h + liner_t / 2.0),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_back",
        (cavity_w + 2.0 * liner_t, liner_t, cavity_h + 2.0 * liner_t),
        (0.0, cavity_back_inner_y + liner_t / 2.0, cavity_center_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "spit_support_left",
        (spit_support_t, 0.040, 0.040),
        (-(spit_len / 2.0 + spit_support_t / 2.0), cavity_center_y, spit_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "spit_support_right",
        (spit_support_t, 0.040, 0.040),
        ((spit_len / 2.0 + spit_support_t / 2.0), cavity_center_y, spit_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_bridge_left",
        (0.050, 0.180, 0.100),
        (-0.400, cavity_center_y, cavity_center_z),
        cavity_enamel,
    )
    _add_box(
        body,
        "liner_bridge_right",
        (0.050, 0.180, 0.100),
        (0.400, cavity_center_y, cavity_center_z),
        cavity_enamel,
    )

    door = model.part("door")
    door_side_w = 0.090
    door_bottom_h = 0.110
    door_top_h = 0.095
    glass_w = door_w - 0.140
    glass_h = door_h - 0.180
    handle_z = door_h - 0.090

    _add_box(
        door,
        "door_left_frame",
        (door_side_w, door_t, door_h),
        (-(door_w / 2.0 - door_side_w / 2.0), -door_t / 2.0, door_h / 2.0),
        stainless,
    )
    _add_box(
        door,
        "door_right_frame",
        (door_side_w, door_t, door_h),
        ((door_w / 2.0 - door_side_w / 2.0), -door_t / 2.0, door_h / 2.0),
        stainless,
    )
    _add_box(
        door,
        "door_bottom_frame",
        (door_w - 2.0 * door_side_w + 0.020, door_t, door_bottom_h),
        (0.0, -door_t / 2.0, door_bottom_h / 2.0),
        stainless,
    )
    _add_box(
        door,
        "door_top_frame",
        (door_w - 2.0 * door_side_w + 0.020, door_t, door_top_h),
        (0.0, -door_t / 2.0, door_h - door_top_h / 2.0),
        stainless,
    )
    _add_box(
        door,
        "door_glass",
        (glass_w, 0.006, glass_h),
        (0.0, -door_t + 0.007, door_h / 2.0),
        glass,
    )
    _add_cylinder(
        door,
        "door_hinge_barrel",
        radius=0.009,
        length=opening_w,
        xyz=(0.0, -0.007, 0.0),
        material=dark_trim,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    _add_cylinder(
        door,
        "handle_left_standoff",
        radius=0.012,
        length=0.040,
        xyz=(-0.180, -0.035, handle_z),
        material=stainless,
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        door,
        "handle_right_standoff",
        radius=0.012,
        length=0.040,
        xyz=(0.180, -0.035, handle_z),
        material=stainless,
        rpy=(pi / 2.0, 0.0, 0.0),
    )
    _add_cylinder(
        door,
        "handle_bar",
        radius=0.013,
        length=0.440,
        xyz=(0.0, -0.058, handle_z),
        material=stainless,
        rpy=(0.0, pi / 2.0, 0.0),
    )

    spit = model.part("rotisserie_spit")
    _add_cylinder(
        spit,
        "spit_rod",
        radius=spit_r,
        length=spit_len,
        xyz=(0.0, 0.0, 0.0),
        material=knob_metal,
        rpy=(0.0, pi / 2.0, 0.0),
    )
    for index, x_pos in enumerate((-0.120, 0.120), start=1):
        _add_box(
            spit,
            f"fork_block_{index}",
            (0.020, 0.024, 0.024),
            (x_pos, 0.0, 0.0),
            knob_metal,
        )
        _add_box(
            spit,
            f"fork_upper_prong_{index}",
            (0.006, 0.010, 0.085),
            (x_pos, 0.014, -0.026),
            knob_metal,
        )
        _add_box(
            spit,
            f"fork_lower_prong_{index}",
            (0.006, 0.010, 0.085),
            (x_pos, -0.014, -0.026),
            knob_metal,
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "spit_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spit,
        origin=Origin(xyz=(0.0, cavity_center_y, spit_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    spit = object_model.get_part("rotisserie_spit")
    door_hinge = object_model.get_articulation("door_hinge")
    spit_joint = object_model.get_articulation("spit_joint")

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
        "door_hinge_axis_is_widthwise",
        tuple(round(value, 3) for value in door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"door hinge axis was {door_hinge.axis!r}",
    )
    ctx.check(
        "door_hinge_is_drop_down",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.4,
        details=f"door hinge limits were {door_hinge.motion_limits!r}",
    )
    ctx.check(
        "spit_axis_is_widthwise",
        tuple(round(value, 3) for value in spit_joint.axis) == (1.0, 0.0, 0.0),
        details=f"spit joint axis was {spit_joint.axis!r}",
    )
    ctx.check(
        "spit_has_rotation_range",
        spit_joint.motion_limits is not None
        and spit_joint.motion_limits.lower is not None
        and spit_joint.motion_limits.upper is not None
        and spit_joint.motion_limits.lower <= -3.0
        and spit_joint.motion_limits.upper >= 3.0,
        details=f"spit joint limits were {spit_joint.motion_limits!r}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.50,
            name="door_covers_front_opening_projection",
        )
        ctx.expect_origin_distance(
            door,
            body,
            axes="x",
            max_dist=0.001,
            name="door_is_centered_in_width",
        )
        ctx.expect_within(
            spit,
            body,
            axes="xyz",
            margin=0.0,
            name="spit_stays_inside_oven_envelope",
        )
        ctx.expect_contact(
            spit,
            body,
            elem_a="spit_rod",
            elem_b="spit_support_left",
            name="spit_contacts_left_support",
        )
        ctx.expect_contact(
            spit,
            body,
            elem_a="spit_rod",
            elem_b="spit_support_right",
            name="spit_contacts_right_support",
        )

    with ctx.pose({door_hinge: 1.35}):
        door_aabb = ctx.part_world_aabb(door)
        body_aabb = ctx.part_world_aabb(body)
        door_drops_forward = (
            door_aabb is not None
            and body_aabb is not None
            and door_aabb[0][1] < body_aabb[0][1] - 0.18
            and door_aabb[1][2] < 0.35
        )
        ctx.check(
            "door_drops_forward_when_open",
            door_drops_forward,
            details=f"door_aabb={door_aabb!r} body_aabb={body_aabb!r}",
        )

    with ctx.pose({spit_joint: 1.2}):
        ctx.expect_within(
            spit,
            body,
            axes="xyz",
            margin=0.0,
            name="spit_remains_inside_oven_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
