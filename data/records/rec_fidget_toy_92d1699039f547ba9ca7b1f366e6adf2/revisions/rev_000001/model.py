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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


HANDLE_LENGTH = 0.190
HANDLE_RADIUS = 0.022
BUTTON_COUNT = 5
BUTTON_PITCH = 0.024
BUTTON_TRAVEL = 0.0045
BUTTON_HEAD_RADIUS = 0.0082
BUTTON_HEAD_LENGTH = 0.0055
BUTTON_STEM_RADIUS = 0.0054
BUTTON_STEM_LENGTH = 0.0165
BUTTON_FLANGE_RADIUS = 0.0074
BUTTON_FLANGE_LENGTH = 0.004
BUTTON_HEAD_PROUD = 0.005
GUIDE_OUTER_RADIUS = 0.0102
GUIDE_INNER_RADIUS = 0.0080
GUIDE_LENGTH = 0.022
GUIDE_TOP_Z = 0.039
GUIDE_CENTER_Z = GUIDE_TOP_Z - (GUIDE_LENGTH * 0.5)
UPPER_STOP_OUTER_RADIUS = 0.0084
UPPER_STOP_INNER_RADIUS = 0.0060
UPPER_STOP_THICKNESS = 0.002
UPPER_STOP_CENTER_REL = -0.0105
LOWER_STOP_OUTER_RADIUS = 0.0086
LOWER_STOP_INNER_RADIUS = 0.0060
LOWER_STOP_THICKNESS = 0.002
LOWER_STOP_CENTER_REL = -0.021
BUTTON_JOINT_Z = GUIDE_TOP_Z


def _button_positions() -> list[float]:
    half_span = BUTTON_PITCH * (BUTTON_COUNT - 1) * 0.5
    return [(-half_span + (index * BUTTON_PITCH)) for index in range(BUTTON_COUNT)]


def _build_guide_barrel():
    return boolean_difference(
        CylinderGeometry(
            radius=GUIDE_OUTER_RADIUS,
            height=GUIDE_LENGTH,
            radial_segments=36,
        ),
        CylinderGeometry(
            radius=GUIDE_INNER_RADIUS,
            height=GUIDE_LENGTH + 0.002,
            radial_segments=36,
        ),
    )


def _build_stop_ring(*, outer_radius: float, inner_radius: float, thickness: float):
    return boolean_difference(
        CylinderGeometry(radius=outer_radius, height=thickness, radial_segments=32),
        CylinderGeometry(radius=inner_radius, height=thickness + 0.002, radial_segments=32),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_click_fidget_stick")

    graphite = model.material("graphite", rgba=(0.16, 0.18, 0.20, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    stem_steel = model.material("stem_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    button_materials = [
        model.material("button_red", rgba=(0.83, 0.25, 0.22, 1.0)),
        model.material("button_orange", rgba=(0.90, 0.49, 0.17, 1.0)),
        model.material("button_yellow", rgba=(0.92, 0.78, 0.17, 1.0)),
        model.material("button_teal", rgba=(0.18, 0.67, 0.65, 1.0)),
        model.material("button_blue", rgba=(0.22, 0.46, 0.86, 1.0)),
    ]

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=HANDLE_RADIUS, length=HANDLE_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="handle_shell",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(-0.067, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip_sleeve",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip_sleeve",
    )
    handle.visual(
        Cylinder(radius=0.0245, length=0.008),
        origin=Origin(xyz=(-0.091, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_end_bumper",
    )
    handle.visual(
        Cylinder(radius=0.0245, length=0.008),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_end_bumper",
    )
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=HANDLE_RADIUS, length=HANDLE_LENGTH),
        mass=0.32,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    guide_barrel_mesh = mesh_from_geometry(_build_guide_barrel(), "button_guide_barrel")
    upper_stop_mesh = mesh_from_geometry(
        _build_stop_ring(
            outer_radius=UPPER_STOP_OUTER_RADIUS,
            inner_radius=UPPER_STOP_INNER_RADIUS,
            thickness=UPPER_STOP_THICKNESS,
        ),
        "button_upper_stop",
    )
    lower_stop_mesh = mesh_from_geometry(
        _build_stop_ring(
            outer_radius=LOWER_STOP_OUTER_RADIUS,
            inner_radius=LOWER_STOP_INNER_RADIUS,
            thickness=LOWER_STOP_THICKNESS,
        ),
        "button_lower_stop",
    )

    for index, x_pos in enumerate(_button_positions()):
        handle.visual(
            guide_barrel_mesh,
            origin=Origin(xyz=(x_pos, 0.0, GUIDE_CENTER_Z)),
            material=graphite,
            name=f"guide_barrel_{index}",
        )
        handle.visual(
            upper_stop_mesh,
            origin=Origin(xyz=(x_pos, 0.0, BUTTON_JOINT_Z + UPPER_STOP_CENTER_REL)),
            material=stem_steel,
            name=f"upper_stop_{index}",
        )
        handle.visual(
            lower_stop_mesh,
            origin=Origin(xyz=(x_pos, 0.0, BUTTON_JOINT_Z + LOWER_STOP_CENTER_REL)),
            material=stem_steel,
            name=f"lower_stop_{index}",
        )

    for index, x_pos in enumerate(_button_positions()):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=BUTTON_HEAD_RADIUS, length=BUTTON_HEAD_LENGTH),
            origin=Origin(
                xyz=(0.0, 0.0, BUTTON_HEAD_PROUD + (BUTTON_HEAD_LENGTH * 0.5))
            ),
            material=button_materials[index],
            name="button_head",
        )
        button.visual(
            Cylinder(radius=BUTTON_STEM_RADIUS, length=BUTTON_STEM_LENGTH),
            origin=Origin(
                xyz=(0.0, 0.0, BUTTON_HEAD_PROUD - (BUTTON_STEM_LENGTH * 0.5))
            ),
            material=stem_steel,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=BUTTON_FLANGE_RADIUS, length=BUTTON_FLANGE_LENGTH),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    BUTTON_HEAD_PROUD
                    - BUTTON_STEM_LENGTH
                    - (BUTTON_FLANGE_LENGTH * 0.5),
                )
            ),
            material=stem_steel,
            name="retention_flange",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.020, 0.020, 0.024)),
            mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
        )

        model.articulation(
            f"handle_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=handle,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, BUTTON_JOINT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.12,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
            motion_properties=MotionProperties(damping=0.28, friction=0.12),
            meta={"spring_return": True},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    buttons = [object_model.get_part(f"button_{index}") for index in range(BUTTON_COUNT)]
    button_joints = [
        object_model.get_articulation(f"handle_to_button_{index}") for index in range(BUTTON_COUNT)
    ]

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

    joint_details: list[str] = []
    joints_ok = True
    for index, button_joint in enumerate(button_joints):
        limits = button_joint.motion_limits
        joint_ok = (
            button_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(round(value, 6) for value in button_joint.axis) == (0.0, 0.0, -1.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and abs(limits.upper - BUTTON_TRAVEL) < 1e-9
        )
        joints_ok = joints_ok and joint_ok
        joint_details.append(
            f"{button_joint.name}: type={button_joint.articulation_type}, axis={button_joint.axis}, "
            f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})"
        )
    ctx.check(
        "five matching prismatic button joints",
        joints_ok,
        details="; ".join(joint_details),
    )

    for index, button in enumerate(buttons):
        ctx.expect_contact(
            button,
            handle,
            elem_a="retention_flange",
            elem_b=f"upper_stop_{index}",
            name=f"button {index + 1} is retained against its spring stop",
        )
        ctx.expect_gap(
            button,
            handle,
            axis="z",
            positive_elem="button_head",
            negative_elem=f"guide_barrel_{index}",
            min_gap=0.0045,
            max_gap=0.0055,
            name=f"button {index + 1} sits proud above its guide",
        )

    for index in range(1, BUTTON_COUNT):
        ctx.expect_origin_gap(
            buttons[index],
            buttons[index - 1],
            axis="x",
            min_gap=BUTTON_PITCH - 0.0005,
            max_gap=BUTTON_PITCH + 0.0005,
            name=f"button {index + 1} spacing stays even",
        )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    pressed_pose = {
        button_joint: BUTTON_TRAVEL
        for button_joint in button_joints
    }
    with ctx.pose(pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="buttons clear the handle when fully pressed"
        )
        for index, button in enumerate(buttons):
            ctx.expect_contact(
                button,
                handle,
                elem_a="retention_flange",
                elem_b=f"lower_stop_{index}",
                name=f"button {index + 1} reaches its lower travel stop",
            )
            ctx.expect_gap(
                button,
                handle,
                axis="z",
                positive_elem="button_head",
                negative_elem=f"guide_barrel_{index}",
                min_gap=0.0003,
                max_gap=0.0010,
                name=f"button {index + 1} remains slightly proud when pressed",
            )
        pressed_positions = [ctx.part_world_position(button) for button in buttons]

    buttons_retract = True
    retract_details: list[str] = []
    for index, (rest_pos, pressed_pos) in enumerate(zip(rest_positions, pressed_positions)):
        moved = (
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] < rest_pos[2] - 0.004
        )
        buttons_retract = buttons_retract and moved
        retract_details.append(f"button_{index}: rest={rest_pos}, pressed={pressed_pos}")
    ctx.check(
        "buttons retract inward when pressed",
        buttons_retract,
        details="; ".join(retract_details),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
