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
    model = ArticulatedObject(name="countertop_toaster_oven")

    body_color = model.material("body_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    trim_color = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    glass_color = model.material("glass_smoke", rgba=(0.18, 0.22, 0.26, 0.40))
    metal_color = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    knob_color = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body_w = 0.50
    body_d = 0.36
    body_h = 0.30
    wall_t = 0.014

    opening_x_min = -0.214
    opening_x_max = 0.110
    opening_z_min = 0.035
    opening_z_max = 0.229
    opening_w = opening_x_max - opening_x_min
    opening_h = opening_z_max - opening_z_min

    divider_w = 0.030
    control_x_min = opening_x_max + divider_w
    control_x_max = body_w * 0.5 - wall_t
    control_w = control_x_max - control_x_min

    front_structure_d = 0.050

    body = model.part("body")
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, body_d * 0.5, body_h * 0.5)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, body_d * 0.5, body_h * 0.5)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d, wall_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5, wall_t * 0.5)),
        material=body_color,
        name="bottom_shell",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d, wall_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5, body_h - wall_t * 0.5)),
        material=body_color,
        name="top_shell",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h - 2.0 * wall_t)),
        origin=Origin(xyz=(0.0, body_d - wall_t * 0.5, body_h * 0.5)),
        material=body_color,
        name="back_panel",
    )
    body.visual(
        Box((opening_w, front_structure_d, opening_z_min - wall_t)),
        origin=Origin(
            xyz=(
                (opening_x_min + opening_x_max) * 0.5,
                front_structure_d * 0.5,
                wall_t + (opening_z_min - wall_t) * 0.5,
            )
        ),
        material=trim_color,
        name="door_sill",
    )
    body.visual(
        Box((opening_w, front_structure_d, body_h - wall_t - opening_z_max)),
        origin=Origin(
            xyz=(
                (opening_x_min + opening_x_max) * 0.5,
                front_structure_d * 0.5,
                opening_z_max + (body_h - wall_t - opening_z_max) * 0.5,
            )
        ),
        material=trim_color,
        name="top_brow",
    )
    body.visual(
        Box((divider_w, front_structure_d, body_h - 2.0 * wall_t)),
        origin=Origin(
            xyz=(
                opening_x_max + divider_w * 0.5,
                front_structure_d * 0.5,
                body_h * 0.5,
            )
        ),
        material=trim_color,
        name="control_divider",
    )
    body.visual(
        Box((control_w, front_structure_d, body_h - 2.0 * wall_t)),
        origin=Origin(
            xyz=(
                (control_x_min + control_x_max) * 0.5,
                front_structure_d * 0.5,
                body_h * 0.5,
            )
        ),
        material=trim_color,
        name="control_housing",
    )
    body.visual(
        Box((opening_w + 0.012, 0.008, 0.018)),
        origin=Origin(
            xyz=(
                (opening_x_min + opening_x_max) * 0.5,
                0.004,
                opening_z_max + 0.009,
            )
        ),
        material=metal_color,
        name="top_trim_lip",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=8.5,
        origin=Origin(xyz=(0.0, body_d * 0.5, body_h * 0.5)),
    )

    door = model.part("door")
    door_w = opening_w - 0.004
    door_h = opening_h - 0.004
    door_t = 0.026
    rail_w = 0.036
    glass_w = door_w - 2.0 * rail_w
    glass_h = door_h - 2.0 * rail_w

    door.visual(
        Box((door_w, door_t, rail_w)),
        origin=Origin(xyz=(0.0, -door_t * 0.5, rail_w * 0.5)),
        material=trim_color,
        name="bottom_rail",
    )
    door.visual(
        Box((door_w, door_t, rail_w)),
        origin=Origin(xyz=(0.0, -door_t * 0.5, door_h - rail_w * 0.5)),
        material=trim_color,
        name="top_rail",
    )
    door.visual(
        Box((rail_w, door_t, door_h)),
        origin=Origin(xyz=(-door_w * 0.5 + rail_w * 0.5, -door_t * 0.5, door_h * 0.5)),
        material=trim_color,
        name="left_rail",
    )
    door.visual(
        Box((rail_w, door_t, door_h)),
        origin=Origin(xyz=(door_w * 0.5 - rail_w * 0.5, -door_t * 0.5, door_h * 0.5)),
        material=trim_color,
        name="right_rail",
    )
    door.visual(
        Box((glass_w, 0.004, glass_h)),
        origin=Origin(
            xyz=(
                0.0,
                -door_t * 0.5 + 0.011,
                rail_w + glass_h * 0.5,
            )
        ),
        material=glass_color,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.007, length=door_w - 0.056),
        origin=Origin(
            xyz=(0.0, -door_t - 0.016, door_h * 0.62),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal_color,
        name="handle_grip",
    )
    door.visual(
        Box((0.014, 0.024, 0.038)),
        origin=Origin(
            xyz=(-door_w * 0.5 + 0.028, -door_t - 0.012, door_h * 0.62),
        ),
        material=metal_color,
        name="handle_left_post",
    )
    door.visual(
        Box((0.014, 0.024, 0.038)),
        origin=Origin(
            xyz=(door_w * 0.5 - 0.028, -door_t - 0.012, door_h * 0.62),
        ),
        material=metal_color,
        name="handle_right_post",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + 0.020, door_h)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.023, door_h * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=((opening_x_min + opening_x_max) * 0.5, 0.0, opening_z_min)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    knob_x = (control_x_min + control_x_max) * 0.5
    knob_positions = (
        ("function_knob", 0.222),
        ("temperature_knob", 0.155),
        ("timer_knob", 0.088),
    )
    for label_name, knob_z in knob_positions:
        body.visual(
            Box((0.060, 0.004, 0.016)),
            origin=Origin(xyz=(knob_x, -0.002, knob_z + 0.035)),
            material=metal_color,
            name=f"{label_name}_label",
        )

        knob = model.part(label_name)
        knob.visual(
            Cylinder(radius=0.0045, length=0.010),
            origin=Origin(
                xyz=(0.0, -0.005, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=metal_color,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(
                xyz=(0.0, -0.003, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=metal_color,
            name="hub",
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.024),
            origin=Origin(
                xyz=(0.0, -0.022, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_color,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(
                xyz=(0.0, -0.036, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=metal_color,
            name="front_cap",
        )
        knob.visual(
            Box((0.004, 0.005, 0.014)),
            origin=Origin(xyz=(0.016, -0.036, 0.0)),
            material=metal_color,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.044, 0.040, 0.044)),
            mass=0.12,
            origin=Origin(xyz=(0.0, -0.020, 0.0)),
        )
        model.articulation(
            f"body_to_{label_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, 0.0, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.25,
                velocity=5.0,
                lower=-2.35,
                upper=2.35,
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")
    knob_names = ("function_knob", "temperature_knob", "timer_knob")

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="z",
            min_gap=-0.001,
            max_gap=0.003,
            positive_elem="bottom_rail",
            negative_elem="door_sill",
            name="closed door seats near sill",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            min_overlap=0.30,
            elem_a="bottom_rail",
            elem_b="door_sill",
            name="door spans main oven opening width",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="handle_grip")
    with ctx.pose({door_joint: 1.20}):
        open_aabb = ctx.part_element_world_aabb(door, elem="handle_grip")

    ctx.check(
        "door opens downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.08
        and open_aabb[1][2] < closed_aabb[1][2] - 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for knob_name in knob_names:
        knob = object_model.get_part(knob_name)
        knob_joint = object_model.get_articulation(f"body_to_{knob_name}")
        label_name = f"{knob_name}_label"

        with ctx.pose({knob_joint: 0.0}):
            ctx.expect_gap(
                body,
                knob,
                axis="y",
                min_gap=0.0,
                max_gap=0.001,
                positive_elem="control_housing",
                negative_elem="shaft",
                name=f"{knob_name} shaft seats against control panel",
            )

        closed_indicator = ctx.part_element_world_aabb(knob, elem="indicator")
        with ctx.pose({knob_joint: 1.1}):
            turned_indicator = ctx.part_element_world_aabb(knob, elem="indicator")

        ctx.check(
            f"{knob_name} indicator rotates about shaft",
            closed_indicator is not None
            and turned_indicator is not None
            and abs(turned_indicator[0][2] - closed_indicator[0][2]) > 0.008,
            details=f"closed={closed_indicator}, turned={turned_indicator}, label={label_name}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
