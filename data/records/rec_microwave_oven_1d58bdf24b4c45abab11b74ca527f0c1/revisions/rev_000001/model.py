from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave")

    body_enamel = model.material("body_enamel", rgba=(0.90, 0.91, 0.92, 1.0))
    cavity_grey = model.material("cavity_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.20, 0.31, 0.36, 0.30))
    button_grey = model.material("button_grey", rgba=(0.78, 0.80, 0.82, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    width = 0.530
    depth = 0.420
    height = 0.310
    shell_t = 0.010

    control_strip_height = 0.070
    control_strip_depth = 0.018
    control_strip_width = width - 2.0 * shell_t

    door_width = width - 0.038
    door_height = 0.205
    door_thickness = 0.022
    door_joint_z = 0.287

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, shell_t * 0.5)),
        material=body_enamel,
        name="bottom_shell",
    )
    housing.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, depth * 0.5, height - shell_t * 0.5)),
        material=body_enamel,
        name="top_shell",
    )
    housing.visual(
        Box((shell_t, depth, height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(-width * 0.5 + shell_t * 0.5, depth * 0.5, height * 0.5),
        ),
        material=body_enamel,
        name="left_shell",
    )
    housing.visual(
        Box((shell_t, depth, height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(width * 0.5 - shell_t * 0.5, depth * 0.5, height * 0.5),
        ),
        material=body_enamel,
        name="right_shell",
    )
    housing.visual(
        Box((width - 2.0 * shell_t, shell_t, height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(0.0, depth - shell_t * 0.5, height * 0.5),
        ),
        material=body_enamel,
        name="back_shell",
    )
    housing.visual(
        Box((control_strip_width, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.009, door_joint_z + 0.008)),
        material=body_enamel,
        name="front_header",
    )
    housing.visual(
        Box((width - 0.050, depth - 0.040, 0.004)),
        origin=Origin(xyz=(0.0, depth * 0.5, shell_t + 0.002)),
        material=cavity_grey,
        name="cavity_floor",
    )

    keypad_outer = rounded_rect_profile(
        control_strip_width,
        control_strip_height,
        radius=0.010,
        corner_segments=8,
    )
    button_pitch_x = 0.110
    button_pitch_z = 0.027
    button_hole_w = 0.050
    button_hole_h = 0.021
    button_centers = [
        (-1.5 * button_pitch_x, control_strip_height * 0.5 - 0.018),
        (-0.5 * button_pitch_x, control_strip_height * 0.5 - 0.018),
        (0.5 * button_pitch_x, control_strip_height * 0.5 - 0.018),
        (1.5 * button_pitch_x, control_strip_height * 0.5 - 0.018),
        (-1.5 * button_pitch_x, -control_strip_height * 0.5 + 0.018),
        (-0.5 * button_pitch_x, -control_strip_height * 0.5 + 0.018),
        (0.5 * button_pitch_x, -control_strip_height * 0.5 + 0.018),
        (1.5 * button_pitch_x, -control_strip_height * 0.5 + 0.018),
    ]
    keypad_holes = [
        _offset_profile(
            rounded_rect_profile(button_hole_w, button_hole_h, radius=0.004, corner_segments=6),
            center_x,
            center_z,
        )
        for center_x, center_z in button_centers
    ]
    keypad_bezel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            keypad_outer,
            keypad_holes,
            control_strip_depth,
            cap=True,
            center=True,
            closed=True,
        ),
        "microwave_keypad_bezel",
    )
    housing.visual(
        keypad_bezel,
        origin=Origin(
            xyz=(0.0, -control_strip_depth * 0.5, control_strip_height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="keypad_bezel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=11.0,
        origin=Origin(xyz=(0.0, depth * 0.5, height * 0.5)),
    )

    door = model.part("door")
    outer_door_profile = rounded_rect_profile(
        door_width,
        door_height,
        radius=0.016,
        corner_segments=10,
    )
    inner_door_profile = rounded_rect_profile(
        door_width - 0.072,
        door_height - 0.060,
        radius=0.010,
        corner_segments=10,
    )
    door_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_door_profile,
            [inner_door_profile],
            door_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        "microwave_door_frame",
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(
            xyz=(0.0, -door_thickness * 0.5, -door_height * 0.5),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=charcoal,
        name="door_frame",
    )
    door.visual(
        Box((door_width - 0.064, 0.006, door_height - 0.052)),
        origin=Origin(
            xyz=(0.0, -0.011, -door_height * 0.5),
        ),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.200, 0.018, 0.020)),
        origin=Origin(
            xyz=(0.0, -door_thickness - 0.005, -door_height + 0.035),
        ),
        material=trim_dark,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness + 0.018, door_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.011, -door_height * 0.5)),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, door_joint_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    button_part_centers = [
        (center_x, 0.0, control_strip_height * 0.5 + center_z)
        for center_x, center_z in button_centers
    ]
    for index, (button_x, button_y, button_z) in enumerate(button_part_centers):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.046, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, -0.0135, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.026, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=trim_dark,
            name="button_plunger",
        )
        button.visual(
            Box((0.060, 0.004, 0.028)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=trim_dark,
            name="button_retainer",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.060, 0.024, 0.028)),
            mass=0.030,
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(button_x, button_y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("housing_to_door")
    first_button = object_model.get_part("button_0")
    first_button_joint = object_model.get_articulation("housing_to_button_0")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            positive_elem="front_header",
            negative_elem="door_frame",
            max_gap=0.001,
            max_penetration=1e-6,
            name="door closes against the upper front frame",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="x",
            min_overlap=0.470,
            elem_a="front_header",
            elem_b="door_frame",
            name="door spans the broad front opening",
        )

    closed_aabb = ctx.part_world_aabb(door)
    open_limit = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else None
    with ctx.pose({door_hinge: open_limit}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings upward from the top hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > closed_aabb[0][2] + 0.10
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_button_pos = ctx.part_world_position(first_button)
    press_limit = (
        first_button_joint.motion_limits.upper
        if first_button_joint.motion_limits is not None
        else None
    )
    with ctx.pose({first_button_joint: press_limit}):
        pressed_button_pos = ctx.part_world_position(first_button)

    ctx.check(
        "button plunges inward into the keypad strip",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.0025,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    ctx.expect_overlap(
        first_button,
        housing,
        axes="xz",
        min_overlap=0.015,
        elem_a="button_cap",
        elem_b="keypad_bezel",
        name="button stays registered within the lower keypad strip",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
