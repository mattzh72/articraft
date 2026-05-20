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

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.15, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.23, 0.27, 0.42))
    knob_metal = model.material("knob_metal", rgba=(0.69, 0.70, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))

    overall_width = 0.480
    overall_depth = 0.390
    body_height = 0.305
    side_thickness = 0.010
    top_thickness = 0.010
    bottom_thickness = 0.014
    front_face_y = -(overall_depth * 0.5 + 0.005)

    door_width = 0.346
    door_center_x = -0.053
    hinge_z = 0.030
    left_hinge_x = door_center_x - 0.146
    right_hinge_x = door_center_x + 0.146

    body = model.part("body")
    body.visual(
        Box((overall_width, overall_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((overall_width, overall_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_thickness * 0.5)),
        material=stainless,
        name="top_shell",
    )
    side_height = body_height - (0.017 + bottom_thickness * 0.5) - top_thickness
    side_center_z = 0.024 + side_height * 0.5
    body.visual(
        Box((side_thickness, overall_depth, side_height)),
        origin=Origin(xyz=(-(overall_width * 0.5) + side_thickness * 0.5, 0.0, side_center_z)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((side_thickness, overall_depth, side_height)),
        origin=Origin(xyz=((overall_width * 0.5) - side_thickness * 0.5, 0.0, side_center_z)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((overall_width - 2.0 * side_thickness, side_thickness, side_height)),
        origin=Origin(xyz=(0.0, overall_depth * 0.5 - side_thickness * 0.5, side_center_z)),
        material=stainless,
        name="back_wall",
    )
    body.visual(
        Box((overall_width - 2.0 * side_thickness, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, front_face_y, 0.292)),
        material=dark_trim,
        name="front_top_rail",
    )
    body.visual(
        Box((overall_width - 2.0 * side_thickness, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, front_face_y + 0.001, 0.039)),
        material=dark_trim,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.014, 0.010, 0.240)),
        origin=Origin(xyz=(0.111, front_face_y, 0.159)),
        material=dark_trim,
        name="door_mullion",
    )
    body.visual(
        Box((0.114, 0.010, 0.252)),
        origin=Origin(xyz=(0.173, front_face_y, 0.165)),
        material=dark_trim,
        name="control_fascia",
    )
    body.visual(
        Box((0.104, 0.052, 0.226)),
        origin=Origin(xyz=(0.173, -0.169, 0.156)),
        material=dark_trim,
        name="control_housing",
    )
    body.visual(
        Box((0.016, 0.017, 0.223)),
        origin=Origin(xyz=(door_center_x - door_width * 0.5 + 0.020, -0.2165, 0.1655)),
        material=dark_trim,
        name="door_stop_left",
    )
    body.visual(
        Box((0.016, 0.017, 0.223)),
        origin=Origin(xyz=(door_center_x + door_width * 0.5 - 0.020, -0.2165, 0.1655)),
        material=dark_trim,
        name="door_stop_right",
    )

    for hinge_x, name_prefix in ((left_hinge_x, "left"), (right_hinge_x, "right")):
        body.visual(
            Box((0.034, 0.020, 0.020)),
            origin=Origin(xyz=(hinge_x, -0.205, hinge_z)),
            material=dark_trim,
            name=f"{name_prefix}_hinge_bracket",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(hinge_x, -0.214, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=knob_metal,
            name=f"{name_prefix}_hinge_sleeve",
        )

    for foot_index, foot_pos in enumerate(
        (
            (-0.180, -0.130),
            (0.180, -0.130),
            (-0.180, 0.130),
            (0.180, 0.130),
        )
    ):
        body.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(foot_pos[0], foot_pos[1], 0.005)),
            material=rubber,
            name=f"foot_{foot_index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((overall_width, overall_depth, body_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    knob_specs = (
        ("function_knob", 0.247),
        ("temperature_knob", 0.166),
        ("timer_knob", 0.087),
    )
    for knob_name, knob_z in knob_specs:
        body.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.173, -0.201, knob_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"{knob_name}_bushing",
        )

        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_metal,
            name="knob_collar",
        )
        knob.visual(
            Cylinder(radius=0.024, length=0.024),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_metal,
            name="knob_grip",
        )
        knob.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_metal,
            name="knob_face",
        )
        knob.visual(
            Box((0.003, 0.005, 0.010)),
            origin=Origin(xyz=(0.0, -0.029, 0.018,)),
            material=dark_trim,
            name="pointer_mark",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.052, 0.034, 0.052)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
        )

        model.articulation(
            f"body_to_{knob_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.173, -0.205, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=4.0,
                lower=-2.6,
                upper=2.6,
            ),
        )

    door = model.part("door")
    door_height = 0.242
    door_thickness = 0.020
    door.visual(
        Box((door_width, door_thickness, 0.032)),
        origin=Origin(xyz=(0.0, -0.006, 0.016)),
        material=dark_trim,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, 0.038)),
        origin=Origin(xyz=(0.0, -0.006, 0.223)),
        material=dark_trim,
        name="door_top_rail",
    )
    door.visual(
        Box((0.034, door_thickness, 0.172)),
        origin=Origin(xyz=(-(door_width * 0.5) + 0.017, -0.006, 0.118)),
        material=dark_trim,
        name="door_left_stile",
    )
    door.visual(
        Box((0.034, door_thickness, 0.172)),
        origin=Origin(xyz=((door_width * 0.5) - 0.017, -0.006, 0.118)),
        material=dark_trim,
        name="door_right_stile",
    )
    door.visual(
        Box((door_width - 0.068, 0.004, 0.148)),
        origin=Origin(xyz=(0.0, 0.002, 0.118)),
        material=smoked_glass,
        name="door_glass",
    )
    for post_x, post_name in ((-0.105, "left"), (0.105, "right")):
        door.visual(
            Cylinder(radius=0.006, length=0.060),
            origin=Origin(xyz=(post_x, -0.024, 0.183), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_metal,
            name=f"handle_post_{post_name}",
        )
    door.visual(
        Cylinder(radius=0.010, length=0.224),
        origin=Origin(xyz=(0.0, -0.055, 0.183), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_metal,
        name="door_handle",
    )
    for hinge_x, name_prefix in ((-0.146, "left"), (0.146, "right")):
        door.visual(
            Cylinder(radius=0.007, length=0.034),
            origin=Origin(xyz=(hinge_x, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=knob_metal,
            name=f"door_hinge_barrel_{name_prefix}",
        )

    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.070, door_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.012, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_center_x, -0.229, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_contact(
        door,
        body,
        elem_a="door_left_stile",
        elem_b="door_stop_left",
        name="door closes against left stop",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="door_right_stile",
        elem_b="door_stop_right",
        name="door closes against right stop",
    )

    closed_aabb = ctx.part_world_aabb(door)
    open_angle = 1.20
    with ctx.pose({door_hinge: open_angle}):
        open_aabb = ctx.part_world_aabb(door)

    door_opens_downward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.12
        and open_aabb[1][2] < closed_aabb[1][2] - 0.12
    )
    ctx.check(
        "door swings downward and outward",
        door_opens_downward,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    for knob_name in ("function_knob", "temperature_knob", "timer_knob"):
        knob = object_model.get_part(knob_name)
        knob_joint = object_model.get_articulation(f"body_to_{knob_name}")

        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_collar",
            elem_b=f"{knob_name}_bushing",
            name=f"{knob_name} stays seated on its shaft bushing",
        )

        rest_pointer = ctx.part_element_world_aabb(knob, elem="pointer_mark")
        with ctx.pose({knob_joint: 1.20}):
            turned_pointer = ctx.part_element_world_aabb(knob, elem="pointer_mark")

        pointer_moves = (
            rest_pointer is not None
            and turned_pointer is not None
            and (
                abs(turned_pointer[0][0] - rest_pointer[0][0]) > 0.008
                or abs(turned_pointer[0][2] - rest_pointer[0][2]) > 0.008
            )
        )
        ctx.check(
            f"{knob_name} pointer rotates with the knob",
            pointer_moves,
            details=f"rest_pointer={rest_pointer}, turned_pointer={turned_pointer}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
