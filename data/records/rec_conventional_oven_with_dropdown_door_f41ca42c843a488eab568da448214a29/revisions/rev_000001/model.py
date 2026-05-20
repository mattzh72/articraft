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
    model = ArticulatedObject(name="combo_microwave_oven")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.12, 0.15, 0.55))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    enamel = model.material("enamel", rgba=(0.93, 0.94, 0.95, 1.0))
    display_black = model.material("display_black", rgba=(0.03, 0.05, 0.06, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.72, 0.84, 0.90, 0.35))
    amber = model.material("amber", rgba=(0.95, 0.62, 0.18, 1.0))

    body_w = 0.64
    body_d = 0.53
    body_h = 0.40
    shell_t = 0.018

    cavity_w = 0.43
    cavity_d = 0.36
    cavity_h = 0.28
    cavity_x = -0.07
    cavity_y = -0.06
    floor_top = shell_t + 0.006

    door_w = 0.48
    door_h = 0.31
    door_t = 0.03
    door_bottom = 0.045
    door_center_y = -(body_d / 2.0) - 0.002 - (door_t / 2.0)
    door_hinge_x = -0.302

    housing = model.part("housing")
    housing.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2.0)),
        material=stainless,
        name="outer_bottom_shell",
    )
    housing.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - (shell_t / 2.0))),
        material=stainless,
        name="outer_top_shell",
    )
    housing.visual(
        Box((shell_t, body_d, body_h - (2.0 * shell_t))),
        origin=Origin(xyz=(-(body_w / 2.0) + (shell_t / 2.0), 0.0, body_h / 2.0)),
        material=stainless,
        name="outer_left_shell",
    )
    housing.visual(
        Box((shell_t, body_d, body_h - (2.0 * shell_t))),
        origin=Origin(xyz=((body_w / 2.0) - (shell_t / 2.0), 0.0, body_h / 2.0)),
        material=stainless,
        name="outer_right_shell",
    )
    housing.visual(
        Box((body_w - (2.0 * shell_t), shell_t, body_h - (2.0 * shell_t))),
        origin=Origin(xyz=(0.0, (body_d / 2.0) - (shell_t / 2.0), body_h / 2.0)),
        material=stainless,
        name="outer_back_shell",
    )
    housing.visual(
        Box((0.12, 0.14, body_h - (2.0 * shell_t))),
        origin=Origin(xyz=(0.225, -(body_d / 2.0) + 0.07, body_h / 2.0)),
        material=black_plastic,
        name="control_bay_block",
    )
    housing.visual(
        Box((0.14, 0.03, body_h - (2.0 * shell_t))),
        origin=Origin(xyz=(0.24, -(body_d / 2.0) + 0.015, body_h / 2.0)),
        material=black_plastic,
        name="control_panel_fascia",
    )
    housing.visual(
        Box((0.010, 0.034, 0.092)),
        origin=Origin(xyz=(door_hinge_x - 0.005, door_center_y + 0.010, 0.105)),
        material=black_plastic,
        name="hinge_knuckle_upper",
    )
    housing.visual(
        Box((0.010, 0.034, 0.092)),
        origin=Origin(xyz=(door_hinge_x - 0.005, door_center_y + 0.010, 0.295)),
        material=black_plastic,
        name="hinge_knuckle_lower",
    )
    housing.visual(
        Box((cavity_w, cavity_d, 0.006)),
        origin=Origin(xyz=(cavity_x, cavity_y, floor_top - 0.003)),
        material=enamel,
        name="cavity_floor_liner",
    )
    housing.visual(
        Box((cavity_w, cavity_d, 0.006)),
        origin=Origin(xyz=(cavity_x, cavity_y, body_h - shell_t - 0.003)),
        material=enamel,
        name="cavity_ceiling_liner",
    )
    housing.visual(
        Box((0.006, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x - (cavity_w / 2.0) + 0.003,
                cavity_y,
                floor_top + (cavity_h / 2.0),
            )
        ),
        material=enamel,
        name="cavity_left_liner",
    )
    housing.visual(
        Box((0.020, cavity_d, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x + (cavity_w / 2.0) + 0.010,
                cavity_y,
                floor_top + (cavity_h / 2.0),
            )
        ),
        material=enamel,
        name="cavity_right_divider",
    )
    housing.visual(
        Box((cavity_w, 0.006, cavity_h)),
        origin=Origin(
            xyz=(
                cavity_x,
                cavity_y + (cavity_d / 2.0) - 0.003,
                floor_top + (cavity_h / 2.0),
            )
        ),
        material=enamel,
        name="cavity_back_liner",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(cavity_x, cavity_y, floor_top + 0.005)),
        material=black_plastic,
        name="turntable_motor_cap",
    )
    housing.visual(
        Box((0.085, 0.004, 0.032)),
        origin=Origin(xyz=(0.24, -(body_d / 2.0) - 0.001, 0.300)),
        material=display_black,
        name="display_window",
    )
    housing.visual(
        Box((0.070, 0.004, 0.014)),
        origin=Origin(xyz=(0.24, -(body_d / 2.0) - 0.001, 0.258)),
        material=amber,
        name="display_strip",
    )
    housing.visual(
        Cylinder(
            radius=0.022,
            length=0.018,
        ),
        origin=Origin(
            xyz=(0.24, -(body_d / 2.0) + 0.007, 0.108),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_plastic,
        name="control_knob",
    )
    housing.visual(
        Box((0.050, 0.008, 0.016)),
        origin=Origin(xyz=(0.24, -(body_d / 2.0) + 0.003, 0.175)),
        material=black_plastic,
        name="start_button",
    )
    housing.visual(
        Box((0.050, 0.008, 0.016)),
        origin=Origin(xyz=(0.24, -(body_d / 2.0) + 0.003, 0.145)),
        material=black_plastic,
        name="stop_button",
    )
    housing.visual(
        Box((0.070, 0.050, 0.015)),
        origin=Origin(
            xyz=(
                -(body_w / 2.0) + 0.085,
                (body_d / 2.0) - 0.065,
                body_h - 0.020,
            )
        ),
        material=black_plastic,
        name="top_vent_left",
    )
    housing.visual(
        Box((0.070, 0.050, 0.015)),
        origin=Origin(
            xyz=(
                (body_w / 2.0) - 0.085,
                (body_d / 2.0) - 0.065,
                body_h - 0.020,
            )
        ),
        material=black_plastic,
        name="top_vent_right",
    )
    housing.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(
            xyz=(
                -(body_w / 2.0) + 0.070,
                -(body_d / 2.0) + 0.070,
                0.009,
            )
        ),
        material=black_plastic,
        name="front_left_foot",
    )
    housing.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(
            xyz=(
                (body_w / 2.0) - 0.070,
                -(body_d / 2.0) + 0.070,
                0.009,
            )
        ),
        material=black_plastic,
        name="front_right_foot",
    )
    housing.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(
            xyz=(
                -(body_w / 2.0) + 0.070,
                (body_d / 2.0) - 0.070,
                0.009,
            )
        ),
        material=black_plastic,
        name="rear_left_foot",
    )
    housing.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(
            xyz=(
                (body_w / 2.0) - 0.070,
                (body_d / 2.0) - 0.070,
                0.009,
            )
        ),
        material=black_plastic,
        name="rear_right_foot",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.050, door_t, door_h)),
        origin=Origin(xyz=(0.025, 0.0, door_h / 2.0)),
        material=black_plastic,
        name="hinge_stile",
    )
    door.visual(
        Box((0.050, door_t, door_h)),
        origin=Origin(xyz=(door_w - 0.025, 0.0, door_h / 2.0)),
        material=black_plastic,
        name="latch_stile",
    )
    door.visual(
        Box((door_w, door_t, 0.040)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h - 0.020)),
        material=black_plastic,
        name="top_rail",
    )
    door.visual(
        Box((door_w, door_t, 0.048)),
        origin=Origin(xyz=(door_w / 2.0, 0.0, 0.024)),
        material=black_plastic,
        name="bottom_rail",
    )
    door.visual(
        Box((0.012, 0.012, 0.040)),
        origin=Origin(xyz=(door_w - 0.054, -0.010, 0.245)),
        material=stainless,
        name="handle_post_top",
    )
    door.visual(
        Box((0.012, 0.012, 0.040)),
        origin=Origin(xyz=(door_w - 0.054, -0.010, 0.095)),
        material=stainless,
        name="handle_post_bottom",
    )
    door.visual(
        Box((0.018, 0.022, 0.190)),
        origin=Origin(xyz=(door_w - 0.042, -0.016, 0.170)),
        material=stainless,
        name="door_handle_bar",
    )
    door.visual(
        Box((0.340, 0.008, 0.180)),
        origin=Origin(xyz=(0.240, 0.004, 0.175)),
        material=dark_glass,
        name="door_window",
    )
    door.visual(
        Box((0.360, 0.012, 0.205)),
        origin=Origin(xyz=(0.235, 0.009, 0.175)),
        material=black_plastic,
        name="inner_window_border",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=3.4,
        origin=Origin(xyz=(door_w / 2.0, 0.0, door_h / 2.0)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.150, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=glass_clear,
        name="turntable_glass",
    )
    turntable.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=glass_clear,
        name="turntable_hub",
    )
    turntable.visual(
        Box((0.030, 0.018, 0.003)),
        origin=Origin(xyz=(0.100, 0.0, 0.0155)),
        material=glass_clear,
        name="turntable_marker",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.016),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_center_y, door_bottom)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "turntable_bearing",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(cavity_x, cavity_y, floor_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("door_hinge")
    turntable_bearing = object_model.get_articulation("turntable_bearing")

    ctx.check(
        "door hinge axis opens outward",
        door_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "turntable bearing is continuous about vertical axis",
        turntable_bearing.joint_type == ArticulationType.CONTINUOUS
        and turntable_bearing.axis == (0.0, 0.0, 1.0),
        details=f"type={turntable_bearing.joint_type}, axis={turntable_bearing.axis}",
    )

    ctx.expect_gap(
        housing,
        door,
        axis="y",
        positive_elem="control_panel_fascia",
        negative_elem="door_window",
        min_gap=0.002,
        max_gap=0.010,
        name="closed door sits just ahead of the front face",
    )
    ctx.expect_contact(
        door,
        housing,
        elem_a="hinge_stile",
        elem_b="hinge_knuckle_upper",
        contact_tol=1e-6,
        name="door is physically mounted on the hinge knuckle",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        min_overlap=0.25,
        name="closed door covers the cooking cavity opening",
    )
    ctx.expect_overlap(
        turntable,
        housing,
        axes="xy",
        elem_a="turntable_glass",
        elem_b="cavity_floor_liner",
        min_overlap=0.20,
        name="turntable remains centered over the cavity floor",
    )
    ctx.expect_gap(
        turntable,
        housing,
        axis="z",
        positive_elem="turntable_glass",
        negative_elem="cavity_floor_liner",
        min_gap=0.006,
        max_gap=0.012,
        name="glass tray sits slightly above the cavity floor",
    )
    ctx.expect_contact(
        turntable,
        housing,
        elem_a="turntable_hub",
        elem_b="turntable_motor_cap",
        contact_tol=1e-6,
        name="turntable is seated on the center drive bearing",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle_bar")
    with ctx.pose({door_hinge: 1.15}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle_bar")
    handle_opens_out = (
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.12
    )
    ctx.check(
        "door swings outward on its left hinge",
        handle_opens_out,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    marker_rest_aabb = ctx.part_element_world_aabb(turntable, elem="turntable_marker")
    with ctx.pose({turntable_bearing: math.pi / 2.0}):
        marker_quarter_aabb = ctx.part_element_world_aabb(turntable, elem="turntable_marker")
    marker_rotates = False
    if marker_rest_aabb is not None and marker_quarter_aabb is not None:
        rest_center = (
            (marker_rest_aabb[0][0] + marker_rest_aabb[1][0]) / 2.0,
            (marker_rest_aabb[0][1] + marker_rest_aabb[1][1]) / 2.0,
        )
        quarter_center = (
            (marker_quarter_aabb[0][0] + marker_quarter_aabb[1][0]) / 2.0,
            (marker_quarter_aabb[0][1] + marker_quarter_aabb[1][1]) / 2.0,
        )
        marker_rotates = (
            abs(rest_center[0] - quarter_center[0]) > 0.05
            and abs(rest_center[1] - quarter_center[1]) > 0.05
        )
    ctx.check(
        "turntable rotates about the center bearing",
        marker_rotates,
        details=f"rest={marker_rest_aabb}, quarter={marker_quarter_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
