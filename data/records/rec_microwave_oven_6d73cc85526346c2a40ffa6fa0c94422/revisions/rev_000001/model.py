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
    model = ArticulatedObject(name="built_in_microwave_oven")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.22, 0.26, 0.55))
    handle_metal = model.material("handle_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.68, 0.69, 0.71, 1.0))

    body_w = 0.600
    body_h = 0.460
    body_d = 0.540
    shell_t = 0.018
    front_t = 0.030

    door_w = 0.448
    door_h = 0.265
    door_t = 0.028

    left_trim = 0.030
    center_mullion = 0.010
    control_strip_w = 0.080
    right_trim = body_w - left_trim - door_w - center_mullion - control_strip_w

    door_left = -body_w * 0.5 + left_trim
    door_right = door_left + door_w
    control_strip_x = door_right + center_mullion + control_strip_w * 0.5
    right_trim_x = body_w * 0.5 - right_trim * 0.5

    cavity_bottom = 0.108
    cavity_h = 0.230
    cavity_top = cavity_bottom + cavity_h
    cavity_left = -body_w * 0.5 + shell_t
    cavity_right = door_right - 0.016
    cavity_w = cavity_right - cavity_left
    cavity_depth = 0.395
    cavity_center_x = (cavity_left + cavity_right) * 0.5
    cavity_center_y = front_t + cavity_depth * 0.5
    cavity_back_y = front_t + cavity_depth + shell_t * 0.5
    cavity_divider_x = cavity_right + shell_t * 0.5
    door_center_z = cavity_bottom + door_h * 0.5

    body = model.part("oven_body")

    shell_depth = body_d - front_t
    shell_center_y = front_t + shell_depth * 0.5

    body.visual(
        Box((body_w, shell_depth, shell_t)),
        origin=Origin(xyz=(0.0, shell_center_y, shell_t * 0.5)),
        material=dark_trim,
        name="outer_bottom_shell",
    )
    body.visual(
        Box((body_w, shell_depth, shell_t)),
        origin=Origin(xyz=(0.0, shell_center_y, body_h - shell_t * 0.5)),
        material=dark_trim,
        name="outer_top_shell",
    )
    body.visual(
        Box((shell_t, shell_depth, body_h)),
        origin=Origin(xyz=(-body_w * 0.5 + shell_t * 0.5, shell_center_y, body_h * 0.5)),
        material=dark_trim,
        name="outer_left_shell",
    )
    body.visual(
        Box((shell_t, shell_depth, body_h)),
        origin=Origin(xyz=(body_w * 0.5 - shell_t * 0.5, shell_center_y, body_h * 0.5)),
        material=dark_trim,
        name="outer_right_shell",
    )
    body.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(0.0, body_d - shell_t * 0.5, body_h * 0.5)),
        material=dark_trim,
        name="outer_back_shell",
    )

    body.visual(
        Box((cavity_w, cavity_depth, shell_t)),
        origin=Origin(xyz=(cavity_center_x, cavity_center_y, cavity_bottom - shell_t * 0.5)),
        material=cavity_dark,
        name="cavity_floor",
    )
    body.visual(
        Box((cavity_w, cavity_depth, shell_t)),
        origin=Origin(xyz=(cavity_center_x, cavity_center_y, cavity_top + shell_t * 0.5)),
        material=cavity_dark,
        name="cavity_ceiling",
    )
    body.visual(
        Box((shell_t, cavity_depth, cavity_h)),
        origin=Origin(xyz=(cavity_divider_x, cavity_center_y, cavity_bottom + cavity_h * 0.5)),
        material=cavity_dark,
        name="cavity_right_wall",
    )
    body.visual(
        Box((cavity_w, shell_t, cavity_h)),
        origin=Origin(xyz=(cavity_center_x, cavity_back_y, cavity_bottom + cavity_h * 0.5)),
        material=cavity_dark,
        name="cavity_back_wall",
    )

    body.visual(
        Box((body_w, front_t, body_h - cavity_top)),
        origin=Origin(xyz=(0.0, front_t * 0.5, cavity_top + (body_h - cavity_top) * 0.5)),
        material=stainless,
        name="front_top_trim",
    )
    body.visual(
        Box((body_w, front_t, cavity_bottom)),
        origin=Origin(xyz=(0.0, front_t * 0.5, cavity_bottom * 0.5)),
        material=stainless,
        name="front_bottom_trim",
    )
    body.visual(
        Box((left_trim, front_t, cavity_h)),
        origin=Origin(
            xyz=(-body_w * 0.5 + left_trim * 0.5, front_t * 0.5, cavity_bottom + cavity_h * 0.5)
        ),
        material=stainless,
        name="front_left_trim",
    )
    body.visual(
        Box((center_mullion, front_t, cavity_h)),
        origin=Origin(
            xyz=(door_right + center_mullion * 0.5, front_t * 0.5, cavity_bottom + cavity_h * 0.5)
        ),
        material=stainless,
        name="front_mullion",
    )
    body.visual(
        Box((control_strip_w, front_t, body_h)),
        origin=Origin(xyz=(control_strip_x, front_t * 0.5, body_h * 0.5)),
        material=dark_trim,
        name="front_control_strip",
    )
    body.visual(
        Box((right_trim, front_t, body_h)),
        origin=Origin(xyz=(right_trim_x, front_t * 0.5, body_h * 0.5)),
        material=stainless,
        name="front_right_trim",
    )
    body.visual(
        Box((0.052, 0.004, 0.060)),
        origin=Origin(xyz=(control_strip_x, 0.002, 0.312)),
        material=smoked_glass,
        name="control_window",
    )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=29.0,
        origin=Origin(xyz=(0.0, body_d * 0.5, body_h * 0.5)),
    )

    door = model.part("access_panel")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5, -door_t * 0.5, 0.0)),
        material=dark_trim,
        name="door_panel",
    )
    door.visual(
        Box((door_w * 0.70, 0.006, door_h * 0.58)),
        origin=Origin(xyz=(door_w * 0.5, -door_t + 0.003, 0.0)),
        material=smoked_glass,
        name="door_window",
    )
    door.visual(
        Box((door_w, 0.006, 0.038)),
        origin=Origin(xyz=(door_w * 0.5, -door_t + 0.003, door_h * 0.5 - 0.019)),
        material=stainless,
        name="door_top_rail",
    )
    door.visual(
        Box((door_w, 0.006, 0.038)),
        origin=Origin(xyz=(door_w * 0.5, -door_t + 0.003, -door_h * 0.5 + 0.019)),
        material=stainless,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.044, 0.006, door_h)),
        origin=Origin(xyz=(0.022, -door_t + 0.003, 0.0)),
        material=stainless,
        name="door_hinge_stile",
    )
    door.visual(
        Box((0.044, 0.006, door_h)),
        origin=Origin(xyz=(door_w - 0.022, -door_t + 0.003, 0.0)),
        material=stainless,
        name="door_handle_stile",
    )
    for index, barrel_z in enumerate((-0.083, 0.0, 0.083)):
        door.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(0.010, -door_t * 0.5, barrel_z)),
            material=handle_metal,
            name=f"hinge_barrel_{index}",
        )
    door.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(
            xyz=(door_w - 0.030, -door_t - 0.011, 0.055),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=handle_metal,
        name="handle_upper_post",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(
            xyz=(door_w - 0.030, -door_t - 0.011, -0.055),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=handle_metal,
        name="handle_lower_post",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(door_w - 0.030, -door_t - 0.024, 0.0)),
        material=handle_metal,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + 0.030, door_h)),
        mass=6.2,
        origin=Origin(xyz=(door_w * 0.5, -0.020, 0.0)),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="knob_base",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_silver,
        name="knob_body",
    )
    knob.visual(
        Box((0.003, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.014)),
        material=dark_trim,
        name="knob_marker",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.026, 0.040)),
        mass=0.09,
        origin=Origin(xyz=(0.0, -0.013, 0.0)),
    )

    model.articulation(
        "body_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_left, 0.0, door_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(control_strip_x, 0.0, 0.156)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("access_panel")
    knob = object_model.get_part("selector_knob")
    door_hinge = object_model.get_articulation("body_to_access_panel")
    knob_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.check(
        "door hinge uses a vertical side-hinge axis",
        door_hinge.axis == (0.0, 0.0, -1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.7,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "selector knob spins on the control strip normal",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(abs(knob_joint.axis[1]) - 1.0) < 1e-9,
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name="door closes flush to the front trim",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="xz",
            min_overlap=0.200,
            name="door sits over the microwave opening region",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_base",
            elem_b="front_control_strip",
            name="selector knob is mounted to the control strip",
        )

    with ctx.pose({door_hinge: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: math.radians(95.0)}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "door swings outward from the cavity",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.18,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
