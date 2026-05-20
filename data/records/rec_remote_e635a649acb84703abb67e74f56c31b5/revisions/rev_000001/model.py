from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arcade_fight_stick")

    case_black = model.material("case_black", rgba=(0.10, 0.10, 0.11, 1.0))
    panel_black = model.material("panel_black", rgba=(0.16, 0.16, 0.17, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_ivory = model.material("button_ivory", rgba=(0.94, 0.94, 0.90, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    ball_red = model.material("ball_red", rgba=(0.74, 0.10, 0.12, 1.0))

    case_width = 0.42
    case_depth = 0.27
    case_height = 0.06
    top_z = case_height

    case = model.part("case")
    case.visual(
        Box((case_width, case_depth, case_height)),
        origin=Origin(xyz=(0.0, 0.0, case_height * 0.5)),
        material=case_black,
        name="case_shell",
    )
    case.visual(
        Box((0.388, 0.238, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, case_height - 0.0015)),
        material=panel_black,
        name="top_panel",
    )
    for index, foot_x in enumerate((-0.145, 0.145)):
        for foot_row, foot_y in enumerate((-0.090, 0.090)):
            case.visual(
                Box((0.040, 0.018, 0.006)),
                origin=Origin(xyz=(foot_x, foot_y, -0.003)),
                material=bezel_black,
                name=f"foot_{index}_{foot_row}",
            )
    case.inertial = Inertial.from_geometry(
        Box((case_width, case_depth, case_height)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, case_height * 0.5)),
    )

    joystick_collar = model.part("joystick_collar")
    joystick_collar.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=bezel_black,
        name="collar_housing",
    )
    joystick_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.012),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    model.articulation(
        "case_to_joystick_collar",
        ArticulationType.FIXED,
        parent=case,
        child=joystick_collar,
        origin=Origin(xyz=(-0.105, 0.015, top_z)),
    )

    joystick_stick = model.part("joystick_stick")
    joystick_stick.visual(
        Cylinder(radius=0.0055, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=steel,
        name="shaft",
    )
    joystick_stick.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="base_sleeve",
    )
    joystick_stick.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=ball_red,
        name="stick_ball",
    )
    joystick_stick.inertial = Inertial.from_geometry(
        Box((0.036, 0.036, 0.131)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.0655)),
    )

    model.articulation(
        "joystick_pivot",
        ArticulationType.REVOLUTE,
        parent=joystick_collar,
        child=joystick_stick,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-0.42,
            upper=0.42,
        ),
    )

    button_positions = (
        (0.055, 0.048),
        (0.108, 0.036),
        (0.161, 0.024),
        (0.063, -0.004),
        (0.116, -0.016),
        (0.169, -0.028),
    )
    for index, (button_x, button_y) in enumerate(button_positions, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.021, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=bezel_black,
            name="button_housing",
        )
        button.visual(
            Cylinder(radius=0.0185, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=button_ivory,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=0.021, length=0.016),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
        )
        model.articulation(
            f"case_to_button_{index}",
            ArticulationType.FIXED,
            parent=case,
            child=button,
            origin=Origin(xyz=(button_x, button_y, top_z)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    joystick_collar = object_model.get_part("joystick_collar")
    joystick_stick = object_model.get_part("joystick_stick")
    joystick_pivot = object_model.get_articulation("joystick_pivot")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 7)]

    axis = joystick_pivot.axis
    ctx.check(
        "joystick pivots on a horizontal axis",
        axis is not None
        and abs(axis[0]) < 1e-6
        and abs(abs(axis[1]) - 1.0) < 1e-6
        and abs(axis[2]) < 1e-6,
        details=f"axis={axis}",
    )

    ctx.expect_contact(
        joystick_collar,
        case,
        elem_a="collar_housing",
        elem_b="case_shell",
        name="joystick collar seats on the case top",
    )
    ctx.expect_contact(
        joystick_stick,
        joystick_collar,
        elem_a="base_sleeve",
        elem_b="collar_housing",
        name="shaft sleeve meets the base collar",
    )

    for index, button in enumerate(buttons, start=1):
        ctx.expect_contact(
            button,
            case,
            elem_a="button_housing",
            elem_b="case_shell",
            name=f"button {index} housing is mounted to the case top",
        )
        ctx.expect_within(
            button,
            case,
            axes="xy",
            inner_elem="button_housing",
            outer_elem="case_shell",
            name=f"button {index} stays within the case planform",
        )
        button_position = ctx.part_world_position(button)
        ctx.check(
            f"button {index} sits on the right half",
            button_position is not None and button_position[0] > 0.0,
            details=f"button_position={button_position}",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            (lower[0] + upper[0]) * 0.5,
            (lower[1] + upper[1]) * 0.5,
            (lower[2] + upper[2]) * 0.5,
        )

    rest_ball_aabb = ctx.part_element_world_aabb(joystick_stick, elem="stick_ball")
    with ctx.pose({joystick_pivot: 0.35}):
        tilted_ball_aabb = ctx.part_element_world_aabb(joystick_stick, elem="stick_ball")

    rest_ball_center = _aabb_center(rest_ball_aabb)
    tilted_ball_center = _aabb_center(tilted_ball_aabb)
    ctx.check(
        "positive joystick motion leans the ball toward +x",
        rest_ball_center is not None
        and tilted_ball_center is not None
        and tilted_ball_center[0] > rest_ball_center[0] + 0.02,
        details=f"rest_ball_center={rest_ball_center}, tilted_ball_center={tilted_ball_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
