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

    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.22, 0.24, 0.35))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    indicator = model.material("indicator", rgba=(0.86, 0.23, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    width = 0.48
    depth = 0.36
    height = 0.30
    wall = 0.008
    control_panel_width = 0.108
    front_frame_left = 0.016
    center_mullion = 0.014
    bottom_sill = 0.024
    top_lintel = 0.038

    control_panel_left_x = width / 2.0 - wall - control_panel_width
    control_panel_center_x = control_panel_left_x + control_panel_width / 2.0
    opening_left_x = -width / 2.0 + wall + front_frame_left
    opening_right_x = control_panel_left_x - center_mullion
    opening_width = opening_right_x - opening_left_x
    opening_bottom_z = bottom_sill
    opening_top_z = height - wall - top_lintel
    opening_height = opening_top_z - opening_bottom_z
    door_center_x = (opening_left_x + opening_right_x) / 2.0
    door_width = opening_width - 0.004
    door_height = opening_height - 0.004
    door_thickness = 0.018

    body = model.part("body")
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=steel,
        name="bottom_panel",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=steel,
        name="top_panel",
    )
    body.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(
            xyz=(-width / 2.0 + wall / 2.0, 0.0, wall + (height - 2.0 * wall) / 2.0)
        ),
        material=steel,
        name="left_wall",
    )
    body.visual(
        Box((wall, depth, height - 2.0 * wall)),
        origin=Origin(
            xyz=(width / 2.0 - wall / 2.0, 0.0, wall + (height - 2.0 * wall) / 2.0)
        ),
        material=steel,
        name="right_wall",
    )
    body.visual(
        Box((width - 2.0 * wall, wall, height - 2.0 * wall)),
        origin=Origin(
            xyz=(0.0, -depth / 2.0 + wall / 2.0, wall + (height - 2.0 * wall) / 2.0)
        ),
        material=steel,
        name="back_panel",
    )
    body.visual(
        Box((opening_left_x - (-width / 2.0 + wall), wall, opening_height)),
        origin=Origin(
            xyz=(
                (-width / 2.0 + wall + opening_left_x) / 2.0,
                depth / 2.0 - wall / 2.0,
                opening_bottom_z + opening_height / 2.0,
            )
        ),
        material=dark_trim,
        name="front_left_frame",
    )
    body.visual(
        Box((opening_width + front_frame_left, wall, top_lintel)),
        origin=Origin(
            xyz=(
                opening_left_x - front_frame_left / 2.0 + opening_width / 2.0,
                depth / 2.0 - wall / 2.0,
                opening_top_z + top_lintel / 2.0,
            )
        ),
        material=dark_trim,
        name="front_top_frame",
    )
    body.visual(
        Box((opening_width + front_frame_left, wall, bottom_sill)),
        origin=Origin(
            xyz=(
                opening_left_x - front_frame_left / 2.0 + opening_width / 2.0,
                depth / 2.0 - wall / 2.0,
                bottom_sill / 2.0,
            )
        ),
        material=dark_trim,
        name="front_bottom_frame",
    )
    body.visual(
        Box((center_mullion, wall, height - 2.0 * wall)),
        origin=Origin(
            xyz=(
                control_panel_left_x - center_mullion / 2.0,
                depth / 2.0 - wall / 2.0,
                wall + (height - 2.0 * wall) / 2.0,
            )
        ),
        material=dark_trim,
        name="center_mullion",
    )
    body.visual(
        Box((control_panel_width, wall, height - 2.0 * wall)),
        origin=Origin(
            xyz=(
                control_panel_center_x,
                depth / 2.0 - wall / 2.0,
                wall + (height - 2.0 * wall) / 2.0,
            )
        ),
        material=dark_trim,
        name="control_panel_face",
    )
    body.visual(
        Box((control_panel_width - 0.022, 0.003, 0.052)),
        origin=Origin(
            xyz=(control_panel_center_x, depth / 2.0 + 0.0015, height - 0.064)
        ),
        material=glass,
        name="status_window",
    )
    hinge_bracket_x_offset = door_width / 2.0 - 0.030
    for side, sign in (("left", -1.0), ("right", 1.0)):
        body.visual(
            Box((0.028, 0.010, 0.016)),
            origin=Origin(
                xyz=(
                    door_center_x + sign * hinge_bracket_x_offset,
                    depth / 2.0 - 0.005,
                    opening_bottom_z + 0.007,
                )
            ),
            material=dark_trim,
            name=f"hinge_bracket_{side}",
        )

    knob_z_positions = (height - 0.098, height - 0.162, height - 0.226)
    for index, knob_z in enumerate(knob_z_positions):
        body.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(
                xyz=(control_panel_center_x, depth / 2.0 + 0.002, knob_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"knob_collar_{index}",
        )
        body.visual(
            Box((0.0025, 0.004, 0.012)),
            origin=Origin(
                xyz=(control_panel_center_x, depth / 2.0 + 0.002, knob_z + 0.032)
            ),
            material=indicator,
            name=f"knob_tick_{index}",
        )
    body.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(
            xyz=(control_panel_center_x - 0.030, depth / 2.0 + 0.002, height - 0.160),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=indicator,
        name="power_light",
    )
    for foot_x in (-width / 2.0 + 0.055, width / 2.0 - 0.055):
        for foot_y in (-depth / 2.0 + 0.060, depth / 2.0 - 0.060):
            body.visual(
                Cylinder(radius=0.015, length=0.010),
                origin=Origin(
                    xyz=(foot_x, foot_y, -0.005),
                ),
                material=rubber,
                name=f"foot_{'l' if foot_x < 0 else 'r'}_{'rear' if foot_y < 0 else 'front'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    door = model.part("door")
    side_rail = 0.026
    top_rail = 0.030
    bottom_rail = 0.036
    door.visual(
        Box((side_rail, door_thickness, door_height)),
        origin=Origin(
            xyz=(-door_width / 2.0 + side_rail / 2.0, door_thickness / 2.0, door_height / 2.0)
        ),
        material=dark_trim,
        name="left_rail",
    )
    door.visual(
        Box((side_rail, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_width / 2.0 - side_rail / 2.0, door_thickness / 2.0, door_height / 2.0)
        ),
        material=dark_trim,
        name="right_rail",
    )
    door.visual(
        Box((door_width - 2.0 * side_rail, door_thickness, top_rail)),
        origin=Origin(
            xyz=(0.0, door_thickness / 2.0, door_height - top_rail / 2.0)
        ),
        material=dark_trim,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 2.0 * side_rail, door_thickness, bottom_rail)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0, bottom_rail / 2.0)),
        material=dark_trim,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width - 2.0 * side_rail + 0.004, 0.004, door_height - top_rail - bottom_rail + 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                0.010,
                bottom_rail + (door_height - top_rail - bottom_rail) / 2.0,
            )
        ),
        material=glass,
        name="glass_panel",
    )
    handle_z = door_height - 0.018
    for side, sign in (("left", -1.0), ("right", 1.0)):
        door.visual(
            Cylinder(radius=0.005, length=0.016),
            origin=Origin(
                xyz=(sign * 0.065, 0.026, handle_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"handle_post_{side}",
        )
    door.visual(
        Cylinder(radius=0.006, length=0.170),
        origin=Origin(
            xyz=(0.0, 0.039, handle_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="handle_grip",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        door.visual(
            Cylinder(radius=0.005, length=0.028),
            origin=Origin(
                xyz=(sign * (door_width / 2.0 - 0.030), 0.006, 0.008),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_trim,
            name=f"hinge_barrel_{side}",
        )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.045, door_height)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0225, door_height / 2.0)),
    )
    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, depth / 2.0, opening_bottom_z + 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=1.45,
        ),
    )

    knob_joint_names: list[str] = []
    for index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.025, length=0.006),
            origin=Origin(
                xyz=(0.0, 0.003, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_trim,
            name="rear_flange",
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(
                xyz=(0.0, 0.009, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(
                xyz=(0.0, 0.022, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_black,
            name="knob_cap",
        )
        knob.visual(
            Box((0.003, 0.011, 0.010)),
            origin=Origin(xyz=(0.0, 0.024, 0.0115)),
            material=indicator,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.052, 0.032, 0.052)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.016, 0.0)),
        )
        knob_joint = model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(control_panel_center_x, depth / 2.0 + 0.004, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=6.0,
                lower=-2.45,
                upper=2.45,
            ),
        )
        knob_joint_names.append(knob_joint.name)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.004,
        max_penetration=0.0,
        negative_elem="front_bottom_frame",
        name="door sits just in front of the oven face",
    )
    ctx.expect_gap(
        body,
        door,
        axis="x",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="center_mullion",
        negative_elem="right_rail",
        name="door stops before the control panel mullion",
    )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.30}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward and outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > rest_aabb[1][1] + 0.10
        and open_aabb[1][2] < rest_aabb[1][2] - 0.14,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    for index in range(3):
        knob = object_model.get_part(f"knob_{index}")
        joint = object_model.get_articulation(f"body_to_knob_{index}")
        ctx.expect_gap(
            knob,
            body,
            axis="y",
            max_gap=0.003,
            max_penetration=0.0,
            negative_elem=f"knob_collar_{index}",
            name=f"knob {index + 1} seats on its control collar",
        )
        limits = joint.motion_limits
        ctx.check(
            f"knob {index + 1} has broad rotary travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and (limits.upper - limits.lower) > 4.5,
            details=f"limits={limits}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
