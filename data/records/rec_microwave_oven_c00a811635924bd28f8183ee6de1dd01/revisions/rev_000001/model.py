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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grill_combo_microwave")

    cabinet_metal = model.material("cabinet_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    cabinet_dark = model.material("cabinet_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    cavity_steel = model.material("cavity_steel", rgba=(0.80, 0.81, 0.82, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.10, 0.12, 0.14, 0.36))
    display_tint = model.material("display_tint", rgba=(0.16, 0.32, 0.38, 0.55))
    button_dark = model.material("button_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.35, 0.36, 0.38, 1.0))
    heater_metal = model.material("heater_metal", rgba=(0.52, 0.35, 0.20, 1.0))
    tray_glass = model.material("tray_glass", rgba=(0.74, 0.84, 0.92, 0.72))
    foot_rubber = model.material("foot_rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    cabinet_w = 0.540
    cabinet_d = 0.430
    cabinet_h = 0.330
    shell_t = 0.014
    divider_w = 0.010
    divider_x = 0.153

    cavity_w = 0.404
    cavity_d = cabinet_d - 2.0 * shell_t
    cavity_h = cabinet_h - 2.0 * shell_t
    cavity_x = -0.054
    cavity_y = 0.0
    cavity_z = 0.0
    liner_t = 0.003

    control_w = 0.098
    control_x = 0.207
    control_h = 0.288
    control_t = 0.010

    door_w = 0.407
    door_h = 0.287
    door_t = 0.034
    door_hinge_x = -0.259
    door_side_frame = 0.036
    door_top_frame = 0.030
    door_bottom_frame = 0.044
    window_w = 0.332
    window_h = 0.196

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_w, cabinet_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, -cabinet_h / 2.0 + shell_t / 2.0)),
        material=cabinet_metal,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((cabinet_w, cabinet_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h / 2.0 - shell_t / 2.0)),
        material=cabinet_metal,
        name="top_shell",
    )
    cabinet.visual(
        Box((shell_t, cabinet_d, cabinet_h - 2.0 * shell_t)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + shell_t / 2.0, 0.0, 0.0)),
        material=cabinet_metal,
        name="left_shell",
    )
    cabinet.visual(
        Box((shell_t, cabinet_d, cabinet_h - 2.0 * shell_t)),
        origin=Origin(xyz=(cabinet_w / 2.0 - shell_t / 2.0, 0.0, 0.0)),
        material=cabinet_metal,
        name="right_shell",
    )
    cabinet.visual(
        Box((cabinet_w - 2.0 * shell_t, shell_t, cabinet_h - 2.0 * shell_t)),
        origin=Origin(xyz=(0.0, cabinet_d / 2.0 - shell_t / 2.0, 0.0)),
        material=cabinet_metal,
        name="rear_shell",
    )
    cabinet.visual(
        Box((divider_w, cabinet_d, cabinet_h - 2.0 * shell_t)),
        origin=Origin(xyz=(divider_x, 0.0, 0.0)),
        material=cabinet_dark,
        name="control_divider",
    )
    cabinet.visual(
        Box((0.070, 0.010, 0.030)),
        origin=Origin(xyz=(0.212, -cabinet_d / 2.0 + 0.015, -0.136)),
        material=cabinet_dark,
        name="control_pedestal",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            cabinet.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(
                    xyz=(
                        x_sign * 0.210,
                        y_sign * 0.150,
                        -cabinet_h / 2.0 - 0.005,
                    )
                ),
                material=foot_rubber,
                name=f"foot_{'r' if x_sign > 0 else 'l'}_{'b' if y_sign > 0 else 'f'}",
            )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_w, cabinet_d, cabinet_h)),
        mass=12.0,
    )

    cavity_liner = model.part("cavity_liner")
    cavity_liner.visual(
        Box((cavity_w, cavity_d, liner_t)),
        origin=Origin(xyz=(0.0, 0.0, -cavity_h / 2.0 + liner_t / 2.0)),
        material=cavity_steel,
        name="cavity_floor",
    )
    cavity_liner.visual(
        Box((cavity_w, cavity_d, liner_t)),
        origin=Origin(xyz=(0.0, 0.0, cavity_h / 2.0 - liner_t / 2.0)),
        material=cavity_steel,
        name="cavity_roof",
    )
    cavity_liner.visual(
        Box((liner_t, cavity_d, cavity_h - 2.0 * liner_t)),
        origin=Origin(xyz=(-cavity_w / 2.0 + liner_t / 2.0, 0.0, 0.0)),
        material=cavity_steel,
        name="cavity_left_wall",
    )
    cavity_liner.visual(
        Box((liner_t, cavity_d, cavity_h - 2.0 * liner_t)),
        origin=Origin(xyz=(cavity_w / 2.0 - liner_t / 2.0, 0.0, 0.0)),
        material=cavity_steel,
        name="cavity_right_wall",
    )
    cavity_liner.visual(
        Box((cavity_w - 2.0 * liner_t, liner_t, cavity_h - 2.0 * liner_t)),
        origin=Origin(xyz=(0.0, cavity_d / 2.0 - liner_t / 2.0, 0.0)),
        material=cavity_steel,
        name="cavity_back_wall",
    )
    cavity_liner.inertial = Inertial.from_geometry(
        Box((cavity_w, cavity_d, cavity_h)),
        mass=2.8,
    )
    model.articulation(
        "cabinet_to_cavity_liner",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cavity_liner,
        origin=Origin(xyz=(cavity_x, cavity_y, cavity_z)),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((control_w, control_t, control_h)),
        origin=Origin(xyz=(0.0, control_t / 2.0, 0.0)),
        material=cabinet_dark,
        name="control_face",
    )
    control_strip.visual(
        Box((0.072, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.001, 0.105)),
        material=display_tint,
        name="display_window",
    )
    control_strip.visual(
        Box((0.082, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.0015, 0.073)),
        material=knob_cap,
        name="display_trim",
    )
    control_strip.inertial = Inertial.from_geometry(
        Box((control_w, control_t, control_h)),
        mass=0.45,
        origin=Origin(xyz=(0.0, control_t / 2.0, 0.0)),
    )
    model.articulation(
        "cabinet_to_control_strip",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_strip,
        origin=Origin(xyz=(control_x, -cabinet_d / 2.0, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_side_frame, door_t, door_h)),
        origin=Origin(xyz=(door_side_frame / 2.0, -door_t / 2.0, 0.0)),
        material=cabinet_dark,
        name="hinge_stile",
    )
    door.visual(
        Box((door_side_frame, door_t, door_h)),
        origin=Origin(xyz=(door_w - door_side_frame / 2.0, -door_t / 2.0, 0.0)),
        material=cabinet_dark,
        name="latch_stile",
    )
    door.visual(
        Box((door_w - 2.0 * door_side_frame, door_t, door_top_frame)),
        origin=Origin(
            xyz=(
                door_w / 2.0,
                -door_t / 2.0,
                door_h / 2.0 - door_top_frame / 2.0,
            )
        ),
        material=cabinet_dark,
        name="top_rail",
    )
    door.visual(
        Box((door_w - 2.0 * door_side_frame, door_t, door_bottom_frame)),
        origin=Origin(
            xyz=(
                door_w / 2.0,
                -door_t / 2.0,
                -door_h / 2.0 + door_bottom_frame / 2.0,
            )
        ),
        material=cabinet_dark,
        name="bottom_rail",
    )
    door.visual(
        Box((0.360, 0.006, 0.235)),
        origin=Origin(xyz=(door_w / 2.0, -0.015, 0.002)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((window_w - 0.018, 0.002, window_h - 0.018)),
        origin=Origin(xyz=(door_w / 2.0, -0.012, 0.002)),
        material=knob_cap,
        name="screen_shadow",
    )
    for z_center in (-0.056, 0.056):
        door.visual(
            Box((0.014, 0.020, 0.020)),
            origin=Origin(xyz=(door_w - 0.020, -door_t - 0.010, z_center)),
            material=knob_cap,
            name=f"handle_post_{'u' if z_center > 0 else 'l'}",
        )
    door.visual(
        Box((0.018, 0.018, 0.140)),
        origin=Origin(xyz=(door_w - 0.020, -door_t - 0.029, 0.0)),
        material=knob_cap,
        name="handle_grip",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.060, door_h)),
        mass=2.5,
        origin=Origin(xyz=(door_w / 2.0, -0.020, 0.0)),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_hinge_x, -cabinet_d / 2.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.75,
        ),
    )

    grill_geom = wire_from_points(
        [
            (-0.110, 0.015, 0.0),
            (-0.110, -0.110, 0.0),
            (0.110, -0.110, 0.0),
            (0.110, 0.015, 0.0),
        ],
        radius=0.0028,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=10,
    )
    grill = model.part("grill_element")
    grill.visual(
        mesh_from_geometry(grill_geom, "microwave_grill_element"),
        material=heater_metal,
        name="heater_loop",
    )
    grill.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(-0.110, 0.015, 0.006)),
        material=heater_metal,
        name="left_mount",
    )
    grill.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.110, 0.015, 0.006)),
        material=heater_metal,
        name="right_mount",
    )
    grill.inertial = Inertial.from_geometry(
        Box((0.230, 0.130, 0.014)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.045, 0.003)),
    )
    model.articulation(
        "cavity_liner_to_grill_element",
        ArticulationType.FIXED,
        parent=cavity_liner,
        child=grill,
        origin=Origin(xyz=(0.0, 0.0, cavity_h / 2.0 - 0.014)),
    )

    support = model.part("turntable_support")
    support.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=knob_cap,
        name="hub_base",
    )
    support.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=knob_cap,
        name="spindle",
    )
    support.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=knob_cap,
        name="capture_head",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        pad_x = 0.092 * math.cos(angle)
        pad_y = 0.092 * math.sin(angle)
        spoke_x = 0.058 * math.cos(angle)
        spoke_y = 0.058 * math.sin(angle)
        support.visual(
            Box((0.068, 0.010, 0.004)),
            origin=Origin(xyz=(spoke_x, spoke_y, 0.004), rpy=(0.0, 0.0, angle)),
            material=knob_cap,
            name=f"spoke_{index}",
        )
        support.visual(
            Cylinder(radius=0.005, length=0.013),
            origin=Origin(xyz=(pad_x, pad_y, 0.0125)),
            material=knob_cap,
            name=f"support_post_{index}",
        )
        support.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(pad_x, pad_y, 0.022)),
            material=knob_cap,
            name=f"support_pad_{index}",
        )
    support.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.030)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "cavity_liner_to_turntable_support",
        ArticulationType.FIXED,
        parent=cavity_liner,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, -cavity_h / 2.0 + liner_t)),
    )

    tray_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.145, segments=64),
            [_circle_profile(0.018, segments=40)],
            0.006,
        ),
        "turntable_tray_plate",
    )
    tray_rim = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.145, segments=64),
            [_circle_profile(0.131, segments=48)],
            0.006,
        ),
        "turntable_tray_rim",
    )
    tray_hub = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.028, segments=40),
            [_circle_profile(0.0125, segments=32)],
            0.010,
        ),
        "turntable_tray_hub",
    )
    tray = model.part("turntable_tray")
    tray.visual(tray_plate, material=tray_glass, name="tray_plate")
    tray.visual(
        tray_rim,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=tray_glass,
        name="tray_rim",
    )
    tray.visual(
        tray_hub,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=tray_glass,
        name="tray_hub",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.016),
        mass=0.55,
    )
    model.articulation(
        "turntable_support_to_tray",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    for index, z_pos in enumerate((0.075, 0.040, 0.005)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.066, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=button_dark,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.066, 0.008, 0.018)),
            mass=0.03,
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
        )
        model.articulation(
            f"control_strip_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.003,
            ),
        )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_cap,
        name="knob_face",
    )
    knob.visual(
        Box((0.006, 0.004, 0.018)),
        origin=Origin(xyz=(0.019, -0.029, 0.0)),
        material=glass_tint,
        name="pointer_ridge",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.070, 0.034, 0.070)),
        mass=0.14,
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
    )
    model.articulation(
        "control_strip_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=control_strip,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, -0.087)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=5.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    cavity_liner = object_model.get_part("cavity_liner")
    control_strip = object_model.get_part("control_strip")
    door = object_model.get_part("door")
    grill = object_model.get_part("grill_element")
    support = object_model.get_part("turntable_support")
    tray = object_model.get_part("turntable_tray")
    knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    tray_spin = object_model.get_articulation("turntable_support_to_tray")
    knob_turn = object_model.get_articulation("control_strip_to_selector_knob")
    button_joint_0 = object_model.get_articulation("control_strip_to_button_0")
    button_joint_1 = object_model.get_articulation("control_strip_to_button_1")
    button_joint_2 = object_model.get_articulation("control_strip_to_button_2")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cavity_liner, cabinet)
    ctx.expect_contact(control_strip, cabinet)
    ctx.expect_contact(door, cabinet)
    ctx.expect_contact(grill, cavity_liner)
    ctx.expect_contact(support, cavity_liner)
    ctx.expect_contact(tray, support)
    ctx.expect_contact(knob, control_strip)
    ctx.expect_contact(button_0, control_strip)
    ctx.expect_contact(button_1, control_strip)
    ctx.expect_contact(button_2, control_strip)

    ctx.expect_within(tray, cavity_liner, axes="xy", margin=0.002)
    ctx.expect_overlap(door, cabinet, axes="z", min_overlap=0.24)

    ctx.check(
        "door hinge uses vertical side axis",
        tuple(round(value, 4) for value in door_hinge.axis) == (0.0, 0.0, -1.0),
        f"Unexpected door hinge axis: {door_hinge.axis}",
    )
    ctx.check(
        "turntable rotates about vertical spindle",
        tuple(round(value, 4) for value in tray_spin.axis) == (0.0, 0.0, 1.0),
        f"Unexpected tray axis: {tray_spin.axis}",
    )
    ctx.check(
        "selector knob rotates front-to-back",
        tuple(round(value, 4) for value in knob_turn.axis) == (0.0, 1.0, 0.0),
        f"Unexpected knob axis: {knob_turn.axis}",
    )
    for index, joint in enumerate((button_joint_0, button_joint_1, button_joint_2)):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index} presses inward",
            tuple(round(value, 4) for value in joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.upper is not None
            and limits.upper <= 0.0031,
            f"Unexpected button articulation for {joint.name}: axis={joint.axis}, limits={limits}",
        )

    rest_door_aabb = ctx.part_world_aabb(door)
    assert rest_door_aabb is not None
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
        assert open_door_aabb is not None
        ctx.check(
            "door swings outward",
            open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.10,
            f"Door did not swing outward enough: rest={rest_door_aabb}, open={open_door_aabb}",
        )

    rest_button_pos = ctx.part_world_position(button_0)
    assert rest_button_pos is not None
    with ctx.pose({button_joint_0: 0.0025}):
        pressed_button_pos = ctx.part_world_position(button_0)
        assert pressed_button_pos is not None
        ctx.check(
            "button translates inward under press",
            pressed_button_pos[1] > rest_button_pos[1] + 0.002,
            f"Button did not move inward enough: rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    with ctx.pose({tray_spin: 1.8, knob_turn: 1.3}):
        ctx.expect_contact(tray, support)
        ctx.expect_contact(knob, control_strip)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
