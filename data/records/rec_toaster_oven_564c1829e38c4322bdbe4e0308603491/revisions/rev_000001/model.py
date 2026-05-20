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

    stainless = model.material("stainless", rgba=(0.73, 0.74, 0.75, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.13, 0.17, 0.21, 0.45))
    black_trim = model.material("black_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.85, 0.29, 0.17, 1.0))
    foot_black = model.material("foot_black", rgba=(0.07, 0.07, 0.08, 1.0))

    width = 0.44
    depth = 0.37
    height = 0.335
    shell_t = 0.018
    back_t = 0.012
    front_frame_d = 0.060
    bottom_frame_d = 0.082

    opening_center_x = -0.052
    opening_w = 0.272
    opening_h = 0.190
    opening_bottom_z = 0.070
    opening_left_x = opening_center_x - opening_w / 2.0
    opening_right_x = opening_center_x + opening_w / 2.0

    body = model.part("oven_body")

    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(-(width - shell_t) / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=((width - shell_t) / 2.0, 0.0, height / 2.0)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2.0)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((width, back_t, height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 - back_t / 2.0,
                shell_t + (height - 2.0 * shell_t) / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )

    left_jamb_w = abs(-width / 2.0 - opening_left_x)
    divider_w = 0.020
    control_column_w = width / 2.0 - opening_right_x

    body.visual(
        Box((left_jamb_w, front_frame_d, 0.255)),
        origin=Origin(
            xyz=(
                (-width / 2.0 + opening_left_x) / 2.0,
                -depth / 2.0 + front_frame_d / 2.0,
                0.1525,
            )
        ),
        material=stainless,
        name="left_front_jamb",
    )
    body.visual(
        Box((divider_w, front_frame_d, 0.255)),
        origin=Origin(
            xyz=(
                opening_right_x + divider_w / 2.0,
                -depth / 2.0 + front_frame_d / 2.0,
                0.1525,
            )
        ),
        material=stainless,
        name="door_divider_jamb",
    )
    body.visual(
        Box((opening_w + left_jamb_w + divider_w, front_frame_d, height - opening_bottom_z - opening_h)),
        origin=Origin(
            xyz=(
                (opening_left_x - left_jamb_w / 2.0 + opening_right_x + divider_w / 2.0) / 2.0,
                -depth / 2.0 + front_frame_d / 2.0,
                opening_bottom_z + opening_h + (height - opening_bottom_z - opening_h) / 2.0,
            )
        ),
        material=stainless,
        name="top_front_header",
    )
    body.visual(
        Box((opening_w + left_jamb_w + divider_w, bottom_frame_d, opening_bottom_z)),
        origin=Origin(
            xyz=(
                (opening_left_x - left_jamb_w / 2.0 + opening_right_x + divider_w / 2.0) / 2.0,
                -depth / 2.0 + bottom_frame_d / 2.0,
                opening_bottom_z / 2.0,
            )
        ),
        material=stainless,
        name="deep_hinge_frame",
    )
    body.visual(
        Box((control_column_w, front_frame_d, height - 0.030)),
        origin=Origin(
            xyz=(
                opening_right_x + divider_w + control_column_w / 2.0 - divider_w / 2.0,
                -depth / 2.0 + front_frame_d / 2.0,
                (height - 0.030) / 2.0 + 0.015,
            )
        ),
        material=stainless,
        name="control_column",
    )
    body.visual(
        Box((control_column_w - 0.018, 0.006, 0.248)),
        origin=Origin(
            xyz=(
                opening_right_x + divider_w + control_column_w / 2.0 - divider_w / 2.0,
                -depth / 2.0 + 0.003,
                0.174,
            )
        ),
        material=charcoal,
        name="control_panel_face",
    )
    body.visual(
        Box((opening_w - 0.008, depth - front_frame_d - back_t, 0.004)),
        origin=Origin(
            xyz=(
                opening_center_x,
                -depth / 2.0 + front_frame_d + (depth - front_frame_d - back_t) / 2.0,
                0.054,
            )
        ),
        material=charcoal,
        name="oven_floor",
    )
    body.visual(
        Box((opening_w - 0.030, 0.004, opening_h - 0.012)),
        origin=Origin(
            xyz=(
                opening_center_x,
                depth / 2.0 - back_t - 0.002,
                opening_bottom_z + opening_h / 2.0,
            )
        ),
        material=charcoal,
        name="oven_rear_liner",
    )

    foot_offsets = (
        (-0.150, -0.120),
        (0.150, -0.120),
        (-0.150, 0.120),
        (0.150, 0.120),
    )
    for foot_index, (foot_x, foot_y) in enumerate(foot_offsets):
        body.visual(
            Box((0.030, 0.030, 0.010)),
            origin=Origin(xyz=(foot_x, foot_y, -0.005)),
            material=foot_black,
            name=f"foot_{foot_index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("door")
    door_w = 0.300
    door_h = 0.220
    door_t = 0.022
    stile_w = 0.032
    top_rail_h = 0.032
    bottom_rail_h = 0.045

    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(xyz=(-(door_w - stile_w) / 2.0, -door_t / 2.0, door_h / 2.0)),
        material=black_trim,
        name="left_stile",
    )
    door.visual(
        Box((stile_w, door_t, door_h)),
        origin=Origin(xyz=((door_w - stile_w) / 2.0, -door_t / 2.0, door_h / 2.0)),
        material=black_trim,
        name="right_stile",
    )
    door.visual(
        Box((door_w, door_t, top_rail_h)),
        origin=Origin(xyz=(0.0, -door_t / 2.0, door_h - top_rail_h / 2.0)),
        material=black_trim,
        name="door_top_rail",
    )
    door.visual(
        Box((door_w, door_t, bottom_rail_h)),
        origin=Origin(xyz=(0.0, -door_t / 2.0, bottom_rail_h / 2.0)),
        material=black_trim,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.238, 0.006, 0.150)),
        origin=Origin(xyz=(0.0, -0.006, 0.117)),
        material=dark_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.030, 0.022, 0.016)),
        origin=Origin(xyz=(-0.105, -0.025, 0.176)),
        material=stainless,
        name="left_handle_bracket",
    )
    door.visual(
        Box((0.030, 0.022, 0.016)),
        origin=Origin(xyz=(0.105, -0.025, 0.176)),
        material=stainless,
        name="right_handle_bracket",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.188),
        origin=Origin(xyz=(0.0, -0.034, 0.176), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -door_t / 2.0, door_h / 2.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_center_x, -depth / 2.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    knob_x = 0.155
    knob_zs = (0.245, 0.178, 0.111)
    for knob_index, knob_z in enumerate(knob_zs):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_trim,
            name="collar",
        )
        knob.visual(
            Cylinder(radius=0.021, length=0.028),
            origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="dial",
        )
        knob.visual(
            Box((0.004, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, -0.047, 0.017,)),
            material=knob_marker,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.050, 0.050, 0.050)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.026, 0.0)),
        )
        model.articulation(
            f"body_to_knob_{knob_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -depth / 2.0, knob_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=4.0,
                lower=-2.6,
                upper=2.6,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            name="closed door sits just ahead of the front frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.18,
            name="closed door covers the oven opening footprint",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        opened_top = ctx.part_element_world_aabb(door, elem="door_top_rail")

    ctx.check(
        "door opens downward and outward",
        closed_top is not None
        and opened_top is not None
        and opened_top[1][2] < closed_top[1][2] - 0.12
        and opened_top[0][1] < closed_top[0][1] - 0.12,
        details=f"closed_top={closed_top}, opened_top={opened_top}",
    )

    for knob_index in range(3):
        knob = object_model.get_part(f"knob_{knob_index}")
        ctx.expect_contact(
            knob,
            body,
            name=f"knob {knob_index + 1} remains shaft-mounted to the control panel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
