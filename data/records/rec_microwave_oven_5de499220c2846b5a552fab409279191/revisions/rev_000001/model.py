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
    model = ArticulatedObject(name="over_range_microwave")

    body_w = 0.76
    body_d = 0.41
    body_h = 0.43
    shell_t = 0.018
    front_frame_t = 0.012

    door_w = 0.64
    door_h = 0.30
    door_t = 0.034

    cavity_w = 0.64
    cavity_d = 0.34
    cavity_h = 0.31
    cavity_wall = 0.010

    strip_w = 0.64
    strip_h = 0.078
    strip_t = 0.028

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.12, 0.14, 0.16, 0.55))
    cavity_grey = model.material("cavity_grey", rgba=(0.86, 0.87, 0.88, 1.0))
    button_grey = model.material("button_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.86, 0.87, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0 - shell_t / 2.0)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + shell_t / 2.0, 0.0, 0.0)),
        material=stainless,
        name="left_shell",
    )
    body.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - shell_t / 2.0, 0.0, 0.0)),
        material=stainless,
        name="right_shell",
    )
    body.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + shell_t / 2.0, 0.0)),
        material=stainless,
        name="back_shell",
    )
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, -body_h / 2.0 + shell_t / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((0.060, front_frame_t, 0.355)),
        origin=Origin(xyz=(-0.350, body_d / 2.0 - front_frame_t / 2.0, -0.020)),
        material=stainless,
        name="front_left_stile",
    )
    body.visual(
        Box((0.060, front_frame_t, 0.355)),
        origin=Origin(xyz=(0.350, body_d / 2.0 - front_frame_t / 2.0, -0.020)),
        material=stainless,
        name="front_right_stile",
    )
    body.visual(
        Box((body_w, front_frame_t, 0.030)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - front_frame_t / 2.0, -0.200)),
        material=dark_trim,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.46, 0.16, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, -body_h / 2.0 + shell_t + 0.003)),
        material=dark_trim,
        name="bottom_vent_panel",
    )
    body.visual(
        Box((0.18, 0.08, 0.010)),
        origin=Origin(xyz=(-0.20, 0.020, body_h / 2.0 + 0.005)),
        material=dark_trim,
        name="left_mount_rail",
    )
    body.visual(
        Box((0.18, 0.08, 0.010)),
        origin=Origin(xyz=(0.20, 0.020, body_h / 2.0 + 0.005)),
        material=dark_trim,
        name="right_mount_rail",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h + 0.010)),
        mass=18.0,
    )

    cavity = model.part("cavity")
    cavity.visual(
        Box((cavity_wall, cavity_d, cavity_h)),
        origin=Origin(xyz=(-cavity_w / 2.0 + cavity_wall / 2.0, 0.0, 0.0)),
        material=cavity_grey,
        name="cavity_left_wall",
    )
    cavity.visual(
        Box((cavity_wall, cavity_d, cavity_h)),
        origin=Origin(xyz=(cavity_w / 2.0 - cavity_wall / 2.0, 0.0, 0.0)),
        material=cavity_grey,
        name="cavity_right_wall",
    )
    cavity.visual(
        Box((cavity_w, cavity_d, cavity_wall)),
        origin=Origin(xyz=(0.0, 0.0, cavity_h / 2.0 - cavity_wall / 2.0)),
        material=cavity_grey,
        name="cavity_ceiling",
    )
    cavity.visual(
        Box((cavity_w, cavity_d, cavity_wall)),
        origin=Origin(xyz=(0.0, 0.0, -cavity_h / 2.0 + cavity_wall / 2.0)),
        material=cavity_grey,
        name="cavity_floor",
    )
    cavity.visual(
        Box((cavity_w, cavity_wall, cavity_h)),
        origin=Origin(xyz=(0.0, -cavity_d / 2.0 + cavity_wall / 2.0, 0.0)),
        material=cavity_grey,
        name="cavity_back_wall",
    )
    cavity.visual(
        Box((0.020, 0.022, cavity_h)),
        origin=Origin(xyz=(-0.310, 0.167, 0.0)),
        material=cavity_grey,
        name="front_left_flange",
    )
    cavity.visual(
        Box((0.020, 0.022, cavity_h)),
        origin=Origin(xyz=(0.310, 0.167, 0.0)),
        material=cavity_grey,
        name="front_right_flange",
    )
    cavity.visual(
        Box((cavity_w, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.167, -0.145)),
        material=cavity_grey,
        name="front_bottom_flange",
    )
    cavity.visual(
        Box((0.600, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.167, 0.110)),
        material=cavity_grey,
        name="front_top_flange",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((cavity_w, cavity_d, cavity_h)),
        mass=3.4,
    )
    model.articulation(
        "body_to_cavity",
        ArticulationType.FIXED,
        parent=body,
        child=cavity,
        origin=Origin(xyz=(0.0, 0.015, -0.030)),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((strip_w, strip_t, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, strip_h / 2.0 - 0.007)),
        material=panel_black,
        name="strip_top_rail",
    )
    control_strip.visual(
        Box((strip_w, strip_t, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -strip_h / 2.0 + 0.007)),
        material=panel_black,
        name="strip_bottom_rail",
    )
    control_strip.visual(
        Box((0.014, strip_t, strip_h - 0.028)),
        origin=Origin(xyz=(-strip_w / 2.0 + 0.007, 0.0, 0.0)),
        material=panel_black,
        name="strip_left_rail",
    )
    control_strip.visual(
        Box((0.014, strip_t, strip_h - 0.028)),
        origin=Origin(xyz=(strip_w / 2.0 - 0.007, 0.0, 0.0)),
        material=panel_black,
        name="strip_right_rail",
    )
    control_strip.visual(
        Box((0.538, 0.020, 0.062)),
        origin=Origin(xyz=(-0.045, 0.003, 0.0)),
        material=smoked_glass,
        name="display_window",
    )
    control_strip.visual(
        Box((0.012, strip_t, 0.062)),
        origin=Origin(xyz=(0.230, 0.0, 0.0)),
        material=panel_black,
        name="button_column_divider",
    )
    control_strip.visual(
        Box((0.070, strip_t, 0.007)),
        origin=Origin(xyz=(0.271, 0.0, 0.0115)),
        material=panel_black,
        name="button_separator_upper",
    )
    control_strip.visual(
        Box((0.070, strip_t, 0.007)),
        origin=Origin(xyz=(0.271, 0.0, -0.0115)),
        material=panel_black,
        name="button_separator_lower",
    )

    button_centers = (
        ("button_top", 0.0235),
        ("button_middle", 0.0),
        ("button_bottom", -0.0235),
    )
    for index, (_, z_center) in enumerate(button_centers):
        control_strip.visual(
            Box((0.046, 0.017, 0.003)),
            origin=Origin(xyz=(0.271, -0.0015, z_center + 0.0065)),
            material=panel_black,
            name=f"guide_top_{index}",
        )
        control_strip.visual(
            Box((0.046, 0.017, 0.003)),
            origin=Origin(xyz=(0.271, -0.0015, z_center - 0.0065)),
            material=panel_black,
            name=f"guide_bottom_{index}",
        )
        control_strip.visual(
            Box((0.003, 0.017, 0.010)),
            origin=Origin(xyz=(0.271 - 0.023, -0.0015, z_center)),
            material=panel_black,
            name=f"guide_left_{index}",
        )
        control_strip.visual(
            Box((0.003, 0.017, 0.010)),
            origin=Origin(xyz=(0.271 + 0.023, -0.0015, z_center)),
            material=panel_black,
            name=f"guide_right_{index}",
        )
    control_strip.inertial = Inertial.from_geometry(
        Box((strip_w, strip_t, strip_h)),
        mass=1.1,
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(0.0, 0.206, 0.157)),
    )

    for button_name, z_center in button_centers:
        button_part = model.part(button_name)
        button_part.visual(
            Box((0.054, 0.006, 0.012)),
            material=button_grey,
            name="button_cap",
        )
        button_part.visual(
            Box((0.026, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_grey,
            name="button_neck",
        )
        button_part.visual(
            Box((0.040, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
            material=button_grey,
            name="button_stem",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((0.054, 0.022, 0.012)),
            mass=0.025,
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
        )
        model.articulation(
            f"control_strip_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button_part,
            origin=Origin(xyz=(0.271, 0.015, z_center)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.04,
                lower=0.0,
                upper=0.004,
            ),
        )

    door = model.part("door")
    door.visual(
        Box((0.044, door_t, door_h)),
        origin=Origin(xyz=(-0.298, 0.0, door_h / 2.0)),
        material=stainless,
        name="door_left_rail",
    )
    door.visual(
        Box((0.044, door_t, door_h)),
        origin=Origin(xyz=(0.298, 0.0, door_h / 2.0)),
        material=stainless,
        name="door_right_rail",
    )
    door.visual(
        Box((door_w, door_t, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        material=stainless,
        name="door_top_rail",
    )
    door.visual(
        Box((door_w, door_t, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=stainless,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.556, 0.008, 0.210)),
        origin=Origin(xyz=(0.0, 0.006, 0.157)),
        material=smoked_glass,
        name="door_window",
    )
    door.visual(
        Box((0.560, 0.010, 0.230)),
        origin=Origin(xyz=(0.0, -0.010, 0.149)),
        material=cavity_grey,
        name="door_inner_liner",
    )
    door.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(-0.240, -0.007, 0.010)),
        material=dark_trim,
        name="hinge_lug_left",
    )
    door.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(0.240, -0.007, 0.010)),
        material=dark_trim,
        name="hinge_lug_right",
    )
    door.visual(
        Box((0.022, 0.030, 0.030)),
        origin=Origin(xyz=(-0.200, 0.017, 0.244)),
        material=handle_metal,
        name="handle_left_mount",
    )
    door.visual(
        Box((0.022, 0.030, 0.030)),
        origin=Origin(xyz=(0.200, 0.017, 0.244)),
        material=handle_metal,
        name="handle_right_mount",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.42),
        origin=Origin(xyz=(0.0, 0.032, 0.244), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.050, door_h)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, door_h / 2.0)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, body_d / 2.0 + door_t / 2.0 + 0.002, -0.185)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(95.0),
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

    body = object_model.get_part("body")
    cavity = object_model.get_part("cavity")
    control_strip = object_model.get_part("control_strip")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_contact(
        control_strip,
        body,
        contact_tol=0.002,
        name="control strip is mounted to the body face",
    )
    ctx.expect_contact(
        cavity,
        body,
        contact_tol=0.002,
        name="cavity liner is supported by the body frame",
    )
    ctx.expect_within(
        cavity,
        body,
        axes="xz",
        margin=0.0,
        name="cavity liner stays inside the microwave envelope",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            control_strip,
            door,
            axis="z",
            min_gap=0.003,
            max_gap=0.012,
            name="closed door sits just below the control strip",
        )
        ctx.expect_overlap(
            door,
            cavity,
            axes="xz",
            min_overlap=0.25,
            name="closed door covers the cooking cavity opening",
        )
        closed_top_rail = ctx.part_element_world_aabb(door, elem="door_top_rail")

    with ctx.pose({door_hinge: math.radians(90.0)}):
        open_top_rail = ctx.part_element_world_aabb(door, elem="door_top_rail")

    ctx.check(
        "door swings down and outward from the lower hinge",
        closed_top_rail is not None
        and open_top_rail is not None
        and open_top_rail[1][2] < closed_top_rail[1][2] - 0.20
        and open_top_rail[1][1] > closed_top_rail[1][1] + 0.20,
        details=f"closed_top_rail={closed_top_rail}, open_top_rail={open_top_rail}",
    )

    for button_name in ("button_top", "button_middle", "button_bottom"):
        button = object_model.get_part(button_name)
        button_joint = object_model.get_articulation(f"control_strip_to_{button_name}")

        with ctx.pose({button_joint: 0.0}):
            rest_cap = ctx.part_element_world_aabb(button, elem="button_cap")
            ctx.expect_overlap(
                button,
                control_strip,
                axes="xz",
                min_overlap=0.010,
                name=f"{button_name} stays aligned with the control panel opening",
            )

        with ctx.pose({button_joint: 0.004}):
            pressed_cap = ctx.part_element_world_aabb(button, elem="button_cap")

        rest_center = (
            None
            if rest_cap is None
            else tuple((rest_cap[0][axis] + rest_cap[1][axis]) * 0.5 for axis in range(3))
        )
        pressed_center = (
            None
            if pressed_cap is None
            else tuple((pressed_cap[0][axis] + pressed_cap[1][axis]) * 0.5 for axis in range(3))
        )
        ctx.check(
            f"{button_name} travels straight inward on its guide",
            rest_center is not None
            and pressed_center is not None
            and pressed_center[1] < rest_center[1] - 0.003
            and abs(pressed_center[0] - rest_center[0]) < 1e-6
            and abs(pressed_center[2] - rest_center[2]) < 1e-6,
            details=f"rest_center={rest_center}, pressed_center={pressed_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
