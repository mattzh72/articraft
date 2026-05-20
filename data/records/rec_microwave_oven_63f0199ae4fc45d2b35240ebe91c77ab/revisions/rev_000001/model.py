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
    model = ArticulatedObject(name="office_microwave")

    body_white = model.material("body_white", rgba=(0.88, 0.89, 0.86, 1.0))
    cavity_gray = model.material("cavity_gray", rgba=(0.76, 0.77, 0.78, 1.0))
    glass_black = model.material("glass_black", rgba=(0.12, 0.14, 0.15, 0.55))
    knob_gray = model.material("knob_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.30, 0.32, 0.35, 1.0))
    button_gray = model.material("button_gray", rgba=(0.80, 0.81, 0.83, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.82, 0.90, 0.95, 0.35))
    spindle_dark = model.material("spindle_dark", rgba=(0.20, 0.20, 0.22, 1.0))

    total_width = 0.48
    total_depth = 0.37
    total_height = 0.285

    shell_bottom = 0.012
    shell_top = 0.008
    shell_side = 0.008
    shell_back = 0.008
    front_face_depth = 0.010

    opening_x_min = -0.214
    opening_x_max = 0.113
    opening_width = opening_x_max - opening_x_min
    opening_center_x = (opening_x_min + opening_x_max) / 2.0

    door_z_min = 0.033
    door_height = 0.212
    door_z_max = door_z_min + door_height
    door_center_z = door_z_min + door_height / 2.0

    control_x_min = opening_x_max
    control_x_max = total_width / 2.0
    control_width = control_x_max - control_x_min
    control_center_x = (control_x_min + control_x_max) / 2.0
    dial_center_x = control_x_min + 0.048

    chamber_center_y = 0.178
    chamber_center_x = opening_center_x

    body = model.part("body")
    body.visual(
        Box((total_width, total_depth, shell_bottom)),
        origin=Origin(xyz=(0.0, total_depth / 2.0, shell_bottom / 2.0)),
        material=body_white,
        name="bottom_shell",
    )
    body.visual(
        Box((total_width, total_depth, shell_top)),
        origin=Origin(
            xyz=(0.0, total_depth / 2.0, total_height - shell_top / 2.0)
        ),
        material=body_white,
        name="top_shell",
    )
    body.visual(
        Box((opening_x_min + total_width / 2.0, total_depth, total_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                (-total_width / 2.0 + opening_x_min) / 2.0,
                total_depth / 2.0,
                shell_bottom + (total_height - shell_bottom - shell_top) / 2.0,
            )
        ),
        material=body_white,
        name="left_shell",
    )
    body.visual(
        Box((shell_side, total_depth, total_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                total_width / 2.0 - shell_side / 2.0,
                total_depth / 2.0,
                shell_bottom + (total_height - shell_bottom - shell_top) / 2.0,
            )
        ),
        material=body_white,
        name="right_shell",
    )
    body.visual(
        Box((total_width, shell_back, total_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                0.0,
                total_depth - shell_back / 2.0,
                shell_bottom + (total_height - shell_bottom - shell_top) / 2.0,
            )
        ),
        material=body_white,
        name="back_shell",
    )
    body.visual(
        Box((opening_x_min - (-total_width / 2.0) + opening_width, front_face_depth, door_z_min)),
        origin=Origin(
            xyz=(
                (-total_width / 2.0 + opening_x_max) / 2.0,
                front_face_depth / 2.0,
                door_z_min / 2.0,
            )
        ),
        material=body_white,
        name="front_sill",
    )
    body.visual(
        Box((opening_x_min - (-total_width / 2.0) + opening_width, front_face_depth, total_height - door_z_max)),
        origin=Origin(
            xyz=(
                (-total_width / 2.0 + opening_x_max) / 2.0,
                front_face_depth / 2.0,
                door_z_max + (total_height - door_z_max) / 2.0,
            )
        ),
        material=body_white,
        name="front_header",
    )
    body.visual(
        Box((shell_side, total_depth - shell_back, total_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                control_x_min + shell_side / 2.0,
                (total_depth - shell_back) / 2.0,
                shell_bottom + (total_height - shell_bottom - shell_top) / 2.0,
            )
        ),
        material=body_white,
        name="cavity_partition",
    )

    slot_x_center = control_x_max - 0.019
    slot_width = 0.018
    slot_z_center = 0.065
    slot_height = 0.070
    slot_z_min = slot_z_center - slot_height / 2.0
    slot_z_max = slot_z_center + slot_height / 2.0

    body.visual(
        Box((control_width, front_face_depth, total_height - slot_z_max)),
        origin=Origin(
            xyz=(
                control_center_x,
                front_face_depth / 2.0,
                slot_z_max + (total_height - slot_z_max) / 2.0,
            )
        ),
        material=body_white,
        name="panel_upper",
    )
    body.visual(
        Box((control_width, front_face_depth, slot_z_min)),
        origin=Origin(
            xyz=(control_center_x, front_face_depth / 2.0, slot_z_min / 2.0)
        ),
        material=body_white,
        name="panel_lower",
    )
    left_strip_width = (slot_x_center - slot_width / 2.0) - control_x_min
    right_strip_width = control_x_max - (slot_x_center + slot_width / 2.0)
    body.visual(
        Box((left_strip_width, front_face_depth, slot_height)),
        origin=Origin(
            xyz=(
                control_x_min + left_strip_width / 2.0,
                front_face_depth / 2.0,
                slot_z_center,
            )
        ),
        material=body_white,
        name="panel_slot_left",
    )
    body.visual(
        Box((right_strip_width, front_face_depth, slot_height)),
        origin=Origin(
            xyz=(
                slot_x_center + slot_width / 2.0 + right_strip_width / 2.0,
                front_face_depth / 2.0,
                slot_z_center,
            )
        ),
        material=body_white,
        name="panel_slot_right",
    )

    for dial_name, dial_z in (("upper", 0.193), ("lower", 0.118)):
        body.visual(
            Cylinder(radius=0.033, length=0.004),
            origin=Origin(
                xyz=(dial_center_x, -0.002, dial_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_dark,
            name=f"{dial_name}_bezel",
        )

    liner_clearance = 0.002
    chamber_width = opening_width - shell_side
    chamber_depth = total_depth - shell_back - 0.030
    chamber_height = total_height - shell_bottom - shell_top - 0.010
    body.visual(
        Box((chamber_width, chamber_depth, liner_clearance)),
        origin=Origin(
            xyz=(chamber_center_x, chamber_depth / 2.0, shell_bottom + liner_clearance / 2.0)
        ),
        material=cavity_gray,
        name="floor_liner",
    )
    body.visual(
        Box((chamber_width, chamber_depth, liner_clearance)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                chamber_depth / 2.0,
                total_height - shell_top - liner_clearance / 2.0,
            )
        ),
        material=cavity_gray,
        name="ceiling_liner",
    )
    body.visual(
        Box((liner_clearance, chamber_depth, chamber_height)),
        origin=Origin(
            xyz=(
                opening_x_min + liner_clearance / 2.0,
                chamber_depth / 2.0,
                shell_bottom + chamber_height / 2.0,
            )
        ),
        material=cavity_gray,
        name="left_liner",
    )
    body.visual(
        Box((liner_clearance, chamber_depth, chamber_height)),
        origin=Origin(
            xyz=(
                control_x_min - liner_clearance / 2.0,
                chamber_depth / 2.0,
                shell_bottom + chamber_height / 2.0,
            )
        ),
        material=cavity_gray,
        name="right_liner",
    )
    body.visual(
        Box((chamber_width, liner_clearance, chamber_height)),
        origin=Origin(
            xyz=(
                chamber_center_x,
                total_depth - shell_back - liner_clearance / 2.0,
                shell_bottom + chamber_height / 2.0,
            )
        ),
        material=cavity_gray,
        name="back_liner",
    )
    body.inertial = Inertial.from_geometry(
        Box((total_width, total_depth, total_height)),
        mass=10.0,
        origin=Origin(xyz=(0.0, total_depth / 2.0, total_height / 2.0)),
    )

    door = model.part("door")
    door_width = opening_width + 0.009
    door_thickness = 0.030
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)
        ),
        material=body_white,
        name="outer_frame",
    )
    door.visual(
        Box((0.278, 0.004, 0.156)),
        origin=Origin(
            xyz=(door_width / 2.0 + 0.006, -door_thickness + 0.002, 0.110)
        ),
        material=glass_black,
        name="window_glass",
    )
    door.visual(
        Box((0.260, 0.006, 0.018)),
        origin=Origin(
            xyz=(door_width / 2.0 + 0.006, -0.003, 0.189)
        ),
        material=body_white,
        name="inner_top_trim",
    )
    door.visual(
        Box((0.260, 0.006, 0.026)),
        origin=Origin(
            xyz=(door_width / 2.0 + 0.006, -0.003, 0.030)
        ),
        material=body_white,
        name="inner_bottom_trim",
    )
    door.visual(
        Box((0.020, 0.006, 0.150)),
        origin=Origin(
            xyz=(0.030, -0.003, 0.109)
        ),
        material=body_white,
        name="inner_hinge_trim",
    )
    door.visual(
        Box((0.020, 0.006, 0.150)),
        origin=Origin(
            xyz=(door_width - 0.018, -0.003, 0.109)
        ),
        material=body_white,
        name="inner_latch_trim",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.5,
        origin=Origin(
            xyz=(door_width / 2.0, -door_thickness / 2.0, door_height / 2.0)
        ),
    )

    upper_dial = model.part("upper_dial")
    upper_dial.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(
            xyz=(0.0, -0.011, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_gray,
        name="knob_body",
    )
    upper_dial.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.022, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_gray,
        name="knob_face",
    )
    upper_dial.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.024, 0.018)),
        material=knob_dark,
        name="pointer",
    )
    upper_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.022),
        mass=0.08,
        origin=Origin(
            xyz=(0.0, -0.011, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    lower_dial = model.part("lower_dial")
    lower_dial.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(
            xyz=(0.0, -0.011, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_gray,
        name="knob_body",
    )
    lower_dial.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.022, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_gray,
        name="knob_face",
    )
    lower_dial.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.024, 0.018)),
        material=knob_dark,
        name="pointer",
    )
    lower_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.022),
        mass=0.08,
        origin=Origin(
            xyz=(0.0, -0.011, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
    )

    door_open_key = model.part("door_open_key")
    door_open_key.visual(
        Box((0.016, 0.007, 0.062)),
        origin=Origin(xyz=(0.0, -0.0035, 0.0)),
        material=button_gray,
        name="key_cap",
    )
    door_open_key.visual(
        Box((0.010, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=button_gray,
        name="key_stem",
    )
    door_open_key.visual(
        Box((0.028, 0.006, 0.074)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=button_gray,
        name="retainer_plate",
    )
    door_open_key.inertial = Inertial.from_geometry(
        Box((0.028, 0.026, 0.074)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
    )

    spindle = model.part("turntable_spindle")
    spindle.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=spindle_dark,
        name="spindle_pin",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.006),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.115, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=glass_clear,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=glass_clear,
        name="hub",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.012),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_x_min - 0.005, 0.0, door_z_min)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "body_to_upper_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_dial,
        origin=Origin(xyz=(dial_center_x, 0.0, 0.193)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )
    model.articulation(
        "body_to_lower_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_dial,
        origin=Origin(xyz=(dial_center_x, 0.0, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )
    model.articulation(
        "body_to_door_open_key",
        ArticulationType.PRISMATIC,
        parent=body,
        child=door_open_key,
        origin=Origin(xyz=(slot_x_center, 0.0, slot_z_center)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "body_to_turntable_spindle",
        ArticulationType.FIXED,
        parent=body,
        child=spindle,
        origin=Origin(xyz=(chamber_center_x, chamber_center_y, 0.012)),
    )
    model.articulation(
        "spindle_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    upper_dial = object_model.get_part("upper_dial")
    lower_dial = object_model.get_part("lower_dial")
    door_open_key = object_model.get_part("door_open_key")
    spindle = object_model.get_part("turntable_spindle")
    turntable = object_model.get_part("turntable")

    door_joint = object_model.get_articulation("body_to_door")
    upper_dial_joint = object_model.get_articulation("body_to_upper_dial")
    lower_dial_joint = object_model.get_articulation("body_to_lower_dial")
    key_joint = object_model.get_articulation("body_to_door_open_key")
    turntable_joint = object_model.get_articulation("spindle_to_turntable")

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

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        positive_elem="front_sill",
        name="door_sits_flush_to_front_opening",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="xz",
        min_overlap=0.18,
        name="door_covers_front_opening",
    )
    ctx.expect_contact(
        upper_dial,
        body,
        name="upper_dial_is_mounted_to_panel",
    )
    ctx.expect_contact(
        lower_dial,
        body,
        name="lower_dial_is_mounted_to_panel",
    )
    ctx.expect_contact(
        door_open_key,
        body,
        name="door_open_key_is_captured_by_slot",
    )
    ctx.expect_contact(
        spindle,
        body,
        name="turntable_spindle_is_seated_on_cavity_floor",
    )
    ctx.expect_contact(
        turntable,
        spindle,
        name="turntable_is_supported_on_spindle",
    )
    ctx.expect_origin_distance(
        upper_dial,
        lower_dial,
        axes="x",
        max_dist=0.001,
        name="stacked_dials_share_panel_column",
    )
    ctx.expect_origin_gap(
        upper_dial,
        lower_dial,
        axis="z",
        min_gap=0.060,
        max_gap=0.090,
        name="stacked_dials_are_vertically_separated",
    )

    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "dial_axes_face_forward",
        tuple(upper_dial_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(lower_dial_joint.axis) == (0.0, 1.0, 0.0),
        details=(
            f"upper={upper_dial_joint.axis}, lower={lower_dial_joint.axis}"
        ),
    )
    ctx.check(
        "door_open_key_axis_is_panel_normal",
        tuple(key_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={key_joint.axis}",
    )
    ctx.check(
        "turntable_axis_is_vertical",
        tuple(turntable_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={turntable_joint.axis}",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    assert closed_door_aabb is not None
    with ctx.pose({door_joint: math.radians(100.0)}):
        open_door_aabb = ctx.part_world_aabb(door)
        assert open_door_aabb is not None
        ctx.check(
            "door_swings_outward_on_hinge",
            open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
            details=f"closed_min_y={closed_door_aabb[0][1]:.4f}, open_min_y={open_door_aabb[0][1]:.4f}",
        )

    key_rest = ctx.part_world_position(door_open_key)
    assert key_rest is not None
    with ctx.pose({key_joint: 0.006}):
        key_pressed = ctx.part_world_position(door_open_key)
        assert key_pressed is not None
        ctx.check(
            "door_open_key_translates_inward",
            key_pressed[1] > key_rest[1] + 0.005,
            details=f"rest_y={key_rest[1]:.4f}, pressed_y={key_pressed[1]:.4f}",
        )

    turntable_pos = ctx.part_world_position(turntable)
    assert turntable_pos is not None
    ctx.check(
        "turntable_sits_low_in_cavity",
        0.015 <= turntable_pos[2] <= 0.030,
        details=f"turntable_z={turntable_pos[2]:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
