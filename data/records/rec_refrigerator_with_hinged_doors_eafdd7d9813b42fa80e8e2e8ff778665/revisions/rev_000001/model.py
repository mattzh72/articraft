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


def _add_box(part, size, xyz, *, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(part, radius, length, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_refrigerator")

    appliance_white = model.material("appliance_white", rgba=(0.92, 0.93, 0.94, 1.0))
    door_white = model.material("door_white", rgba=(0.97, 0.97, 0.96, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.84, 0.86, 0.88, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.67, 0.69, 0.72, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.93, 0.31, 0.18, 1.0))

    body_w = 0.55
    body_d = 0.53
    body_h = 1.42
    side_t = 0.028
    top_t = 0.03
    bottom_t = 0.04
    back_t = 0.012
    divider_t = 0.032

    top_margin = 0.02
    bottom_margin = 0.02
    door_gap = 0.012
    upper_door_h = 0.315
    lower_door_h = body_h - top_margin - bottom_margin - door_gap - upper_door_h

    upper_center_z = body_h - top_margin - (upper_door_h / 2.0)
    lower_center_z = bottom_margin + (lower_door_h / 2.0)
    upper_bottom_z = upper_center_z - (upper_door_h / 2.0)
    upper_top_z = upper_center_z + (upper_door_h / 2.0)
    lower_bottom_z = lower_center_z - (lower_door_h / 2.0)
    lower_top_z = lower_center_z + (lower_door_h / 2.0)

    divider_bottom_z = 1.065
    hinge_x = (body_w / 2.0) + 0.008
    hinge_y = (body_d / 2.0) + 0.006
    hinge_radius = 0.007
    pin_len = 0.03
    knuckle_len = 0.045

    cabinet = model.part("cabinet_body")
    _add_box(
        cabinet,
        (side_t, body_d, body_h),
        (-(body_w / 2.0) + (side_t / 2.0), 0.0, body_h / 2.0),
        material=appliance_white,
        name="left_wall",
    )
    _add_box(
        cabinet,
        (side_t, body_d, body_h),
        ((body_w / 2.0) - (side_t / 2.0), 0.0, body_h / 2.0),
        material=appliance_white,
        name="right_wall",
    )
    _add_box(
        cabinet,
        (body_w - (2.0 * side_t), body_d - back_t, top_t),
        (0.0, back_t / 2.0, body_h - (top_t / 2.0)),
        material=appliance_white,
        name="top_shell",
    )
    _add_box(
        cabinet,
        (body_w - (2.0 * side_t), body_d - back_t, bottom_t),
        (0.0, back_t / 2.0, bottom_t / 2.0),
        material=appliance_white,
        name="bottom_shell",
    )
    _add_box(
        cabinet,
        (body_w - (2.0 * side_t), back_t, body_h),
        (0.0, -(body_d / 2.0) + (back_t / 2.0), body_h / 2.0),
        material=appliance_white,
        name="back_panel",
    )
    _add_box(
        cabinet,
        (body_w - (2.0 * side_t), body_d - back_t, divider_t),
        (0.0, back_t / 2.0, divider_bottom_z + (divider_t / 2.0)),
        material=liner_gray,
        name="divider_shelf",
    )
    _add_box(
        cabinet,
        (body_w - (2.0 * side_t), 0.018, 0.024),
        (0.0, (body_d / 2.0) - 0.009, divider_bottom_z + (divider_t / 2.0) - 0.001),
        material=appliance_white,
        name="center_face_rail",
    )

    cabinet_pin_specs = (
        ("upper_top_hinge_pin", upper_top_z - (pin_len / 2.0)),
        ("upper_center_hinge_pin", upper_bottom_z + (pin_len / 2.0)),
        ("lower_center_hinge_pin", lower_top_z - (pin_len / 2.0)),
        ("lower_bottom_hinge_pin", lower_bottom_z + (pin_len / 2.0)),
    )
    for pin_name, z_center in cabinet_pin_specs:
        _add_box(
            cabinet,
            (0.022, 0.018, pin_len),
            ((body_w / 2.0) + 0.0005, (body_d / 2.0) - 0.0005, z_center),
            material=hinge_metal,
            name=f"{pin_name}_bracket",
        )
        _add_cylinder(
            cabinet,
            hinge_radius,
            pin_len,
            (hinge_x, hinge_y, z_center),
            material=hinge_metal,
            name=pin_name,
        )

    door_w = 0.575
    door_t = 0.055
    door_panel_center_x = -(door_w / 2.0) + 0.014
    door_panel_center_y = 0.0345
    gasket_center_x = -hinge_x
    gasket_center_y = 0.001
    liner_center_x = -hinge_x
    liner_center_y = 0.019
    handle_x = door_panel_center_x - (door_w / 2.0) + 0.046
    handle_y = door_panel_center_y + (door_t / 2.0) + 0.005
    flange_x = 0.011
    flange_y = 0.020

    def add_door(name: str, door_height: float, handle_height: float, handle_z: float):
        door = model.part(name)
        _add_box(
            door,
            (door_w, door_t, door_height),
            (door_panel_center_x, door_panel_center_y, 0.0),
            material=door_white,
            name="outer_panel",
        )
        _add_box(
            door,
            (body_w - (2.0 * side_t) - 0.03, 0.01, door_height - 0.035),
            (gasket_center_x, gasket_center_y, 0.0),
            material=gasket_gray,
            name="door_gasket",
        )
        _add_box(
            door,
            (body_w - (2.0 * side_t) - 0.045, 0.028, door_height - 0.075),
            (liner_center_x, liner_center_y, 0.0),
            material=liner_gray,
            name="inner_liner",
        )
        _add_box(
            door,
            (0.014, 0.020, door_height - 0.10),
            (flange_x, flange_y, 0.0),
            material=door_white,
            name="hinge_flange",
        )
        _add_box(
            door,
            (0.022, 0.020, handle_height),
            (handle_x, handle_y, handle_z),
            material=trim_dark,
            name="handle",
        )

        top_knuckle_center = (door_height / 2.0) - 0.0525
        bottom_knuckle_center = -(door_height / 2.0) + 0.0525
        top_name = f"{name}_top_knuckle"
        bottom_name = f"{name}_bottom_knuckle"
        _add_cylinder(
            door,
            hinge_radius,
            knuckle_len,
            (0.0, 0.0, top_knuckle_center),
            material=hinge_metal,
            name=top_name,
        )
        _add_cylinder(
            door,
            hinge_radius,
            knuckle_len,
            (0.0, 0.0, bottom_knuckle_center),
            material=hinge_metal,
            name=bottom_name,
        )
        return door, top_name, bottom_name

    upper_door, upper_top_knuckle_name, upper_bottom_knuckle_name = add_door(
        "upper_freezer_door",
        upper_door_h,
        handle_height=0.14,
        handle_z=-0.055,
    )
    lower_door, lower_top_knuckle_name, lower_bottom_knuckle_name = add_door(
        "lower_main_door",
        lower_door_h,
        handle_height=0.24,
        handle_z=0.22,
    )

    control_housing = model.part("control_housing")
    housing_center = (0.09, 0.10, 1.03)
    housing_size = (0.16, 0.10, 0.05)
    _add_box(
        control_housing,
        housing_size,
        housing_center,
        material=housing_gray,
        name="control_box",
    )
    _add_box(
        control_housing,
        (0.08, 0.06, 0.01),
        (housing_center[0], housing_center[1], housing_center[2] + 0.03),
        material=housing_gray,
        name="mount_tab",
    )
    dial_origin_local = (0.035, (housing_size[1] / 2.0) + 0.006, -0.004)
    _add_cylinder(
        control_housing,
        0.022,
        0.006,
        (
            housing_center[0] + dial_origin_local[0],
            housing_center[1] + (housing_size[1] / 2.0) + 0.003,
            housing_center[2] + dial_origin_local[2],
        ),
        material=trim_dark,
        name="dial_bezel",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )

    thermostat_dial = model.part("thermostat_dial")
    _add_cylinder(
        thermostat_dial,
        0.018,
        0.012,
        (0.0, 0.006, 0.0),
        material=dial_dark,
        name="dial_body",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    _add_box(
        thermostat_dial,
        (0.004, 0.003, 0.012),
        (0.0, 0.0125, 0.013),
        material=dial_mark,
        name="dial_indicator",
    )

    upper_hinge = model.articulation(
        "upper_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=upper_door,
        origin=Origin(xyz=(hinge_x, hinge_y, upper_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-1.95,
            upper=0.0,
        ),
    )
    lower_hinge = model.articulation(
        "lower_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lower_door,
        origin=Origin(xyz=(hinge_x, hinge_y, lower_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=-1.95,
            upper=0.0,
        ),
    )
    model.articulation(
        "cabinet_to_control_housing",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_housing,
        origin=Origin(),
    )
    thermostat_joint = model.articulation(
        "thermostat_dial_joint",
        ArticulationType.REVOLUTE,
        parent=control_housing,
        child=thermostat_dial,
        origin=Origin(
            xyz=(
                housing_center[0] + dial_origin_local[0],
                housing_center[1] + dial_origin_local[1],
                housing_center[2] + dial_origin_local[2],
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-2.2,
            upper=0.6,
        ),
    )

    model.meta["named_contacts"] = {
        "upper_top_knuckle": upper_top_knuckle_name,
        "upper_bottom_knuckle": upper_bottom_knuckle_name,
        "lower_top_knuckle": lower_top_knuckle_name,
        "lower_bottom_knuckle": lower_bottom_knuckle_name,
        "upper_hinge": upper_hinge.name,
        "lower_hinge": lower_hinge.name,
        "thermostat_joint": thermostat_joint.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_body")
    upper_door = object_model.get_part("upper_freezer_door")
    lower_door = object_model.get_part("lower_main_door")
    control_housing = object_model.get_part("control_housing")
    thermostat_dial = object_model.get_part("thermostat_dial")
    upper_hinge = object_model.get_articulation("upper_door_hinge")
    lower_hinge = object_model.get_articulation("lower_door_hinge")
    thermostat_joint = object_model.get_articulation("thermostat_dial_joint")

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
        "vertical door hinge axes",
        upper_hinge.axis == (0.0, 0.0, 1.0) and lower_hinge.axis == (0.0, 0.0, 1.0),
        f"upper={upper_hinge.axis}, lower={lower_hinge.axis}",
    )
    ctx.check(
        "doors share one hinge side",
        abs(upper_hinge.origin.xyz[0] - lower_hinge.origin.xyz[0]) < 1e-9
        and abs(upper_hinge.origin.xyz[1] - lower_hinge.origin.xyz[1]) < 1e-9,
        f"upper origin={upper_hinge.origin.xyz}, lower origin={lower_hinge.origin.xyz}",
    )
    ctx.check(
        "thermostat dial axis is local control axis",
        thermostat_joint.axis == (0.0, 1.0, 0.0),
        f"dial axis={thermostat_joint.axis}",
    )

    ctx.expect_contact(
        control_housing,
        cabinet,
        elem_a="mount_tab",
        elem_b="divider_shelf",
        name="control housing mounts under divider",
    )
    ctx.expect_contact(
        thermostat_dial,
        control_housing,
        elem_a="dial_body",
        elem_b="dial_bezel",
        name="thermostat dial seats in housing bezel",
    )
    ctx.expect_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.010,
        max_gap=0.014,
        positive_elem="outer_panel",
        negative_elem="outer_panel",
        name="freezer and main door gap",
    )

    with ctx.pose({upper_hinge: -1.2, lower_hinge: -1.2}):
        ctx.expect_contact(
            upper_door,
            cabinet,
            elem_a="upper_freezer_door_top_knuckle",
            elem_b="upper_top_hinge_pin",
            name="upper door stays clipped at top hinge",
        )
        ctx.expect_contact(
            upper_door,
            cabinet,
            elem_a="upper_freezer_door_bottom_knuckle",
            elem_b="upper_center_hinge_pin",
            name="upper door stays clipped at center hinge",
        )
        ctx.expect_contact(
            lower_door,
            cabinet,
            elem_a="lower_main_door_top_knuckle",
            elem_b="lower_center_hinge_pin",
            name="lower door stays clipped at center hinge",
        )
        ctx.expect_contact(
            lower_door,
            cabinet,
            elem_a="lower_main_door_bottom_knuckle",
            elem_b="lower_bottom_hinge_pin",
            name="lower door stays clipped at bottom hinge",
        )

    with ctx.pose({thermostat_joint: -1.6}):
        ctx.expect_contact(
            thermostat_dial,
            control_housing,
            elem_a="dial_body",
            elem_b="dial_bezel",
            name="thermostat dial remains mounted while turning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
