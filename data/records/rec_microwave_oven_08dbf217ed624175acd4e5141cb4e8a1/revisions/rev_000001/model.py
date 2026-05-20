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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_microwave")

    # Overall envelope for a medium commercial microwave.
    body_w = 0.54
    body_d = 0.43
    body_h = 0.32
    shell_t = 0.014

    # Front-layout regions.
    panel_w = 0.118
    panel_h = body_h - 2.0 * shell_t
    panel_t = 0.032
    panel_x0 = body_w - shell_t - panel_w
    panel_z0 = shell_t

    door_x0 = 0.012
    door_z0 = 0.026
    door_w = panel_x0 - 0.006 - door_x0
    door_h = 0.262
    door_t = 0.040

    # Interior cooking cavity.
    cavity_x0 = 0.035
    cavity_y0 = -0.012
    cavity_z0 = shell_t
    cavity_w = 0.332
    cavity_d = 0.330
    cavity_h = 0.206
    cavity_wall_t = 0.006
    cavity_floor_t = 0.025

    body_metal = model.material("body_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    control_grey = model.material("control_grey", rgba=(0.28, 0.30, 0.32, 1.0))
    cavity_grey = model.material("cavity_grey", rgba=(0.84, 0.85, 0.86, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.12, 0.16, 0.18, 0.45))
    glass_clear = model.material("glass_clear", rgba=(0.72, 0.80, 0.86, 0.28))
    keypad_dark = model.material("keypad_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    keypad_accent = model.material("keypad_accent", rgba=(0.66, 0.70, 0.74, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(body_w / 2.0, -body_d / 2.0, shell_t / 2.0)),
        material=body_metal,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(body_w / 2.0, -body_d / 2.0, body_h - shell_t / 2.0)),
        material=body_metal,
        name="top_shell",
    )
    cabinet.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=(shell_t / 2.0, -body_d / 2.0, body_h / 2.0)),
        material=body_metal,
        name="left_shell",
    )
    cabinet.visual(
        Box((shell_t, body_d, body_h)),
        origin=Origin(xyz=(body_w - shell_t / 2.0, -body_d / 2.0, body_h / 2.0)),
        material=body_metal,
        name="right_shell",
    )
    cabinet.visual(
        Box((body_w, shell_t, body_h)),
        origin=Origin(xyz=(body_w / 2.0, -body_d + shell_t / 2.0, body_h / 2.0)),
        material=body_metal,
        name="rear_shell",
    )
    cabinet.visual(
        Box((0.010, 0.020, body_h)),
        origin=Origin(xyz=(panel_x0 - 0.006, -0.010, body_h / 2.0)),
        material=trim_dark,
        name="panel_divider",
    )
    cabinet.visual(
        Box((0.500, 0.018, 0.010)),
        origin=Origin(xyz=(body_w / 2.0, -body_d + 0.009, 0.044)),
        material=trim_dark,
        name="rear_bumper_band",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=18.0,
        origin=Origin(xyz=(body_w / 2.0, -body_d / 2.0, body_h / 2.0)),
    )

    cavity = model.part("cavity_liner")
    cavity.visual(
        Box((cavity_w, cavity_d, cavity_floor_t)),
        origin=Origin(xyz=(cavity_w / 2.0, -cavity_d / 2.0, cavity_floor_t / 2.0)),
        material=cavity_grey,
        name="cavity_floor",
    )
    cavity.visual(
        Box((cavity_wall_t, cavity_d, cavity_h + cavity_wall_t)),
        origin=Origin(
            xyz=(
                cavity_wall_t / 2.0,
                -cavity_d / 2.0,
                cavity_floor_t + (cavity_h + cavity_wall_t) / 2.0,
            )
        ),
        material=cavity_grey,
        name="cavity_left_wall",
    )
    cavity.visual(
        Box((cavity_wall_t, cavity_d, cavity_h + cavity_wall_t)),
        origin=Origin(
            xyz=(
                cavity_w - cavity_wall_t / 2.0,
                -cavity_d / 2.0,
                cavity_floor_t + (cavity_h + cavity_wall_t) / 2.0,
            )
        ),
        material=cavity_grey,
        name="cavity_right_wall",
    )
    cavity.visual(
        Box((cavity_w, cavity_wall_t, cavity_h + cavity_wall_t)),
        origin=Origin(
            xyz=(
                cavity_w / 2.0,
                -cavity_d + cavity_wall_t / 2.0,
                cavity_floor_t + (cavity_h + cavity_wall_t) / 2.0,
            )
        ),
        material=cavity_grey,
        name="cavity_back_wall",
    )
    cavity.visual(
        Box((cavity_w, cavity_d, cavity_wall_t)),
        origin=Origin(
            xyz=(
                cavity_w / 2.0,
                -cavity_d / 2.0,
                cavity_floor_t + cavity_h + cavity_wall_t / 2.0,
            )
        ),
        material=cavity_grey,
        name="cavity_ceiling",
    )
    cavity.visual(
        Box((0.032, 0.070, 0.045)),
        origin=Origin(
            xyz=(
                cavity_w - 0.016,
                -cavity_d + 0.045,
                cavity_floor_t + cavity_h * 0.55,
            )
        ),
        material=cavity_grey,
        name="waveguide_cover",
    )
    cavity.inertial = Inertial.from_geometry(
        Box((cavity_w, cavity_d, cavity_floor_t + cavity_h + cavity_wall_t)),
        mass=6.0,
        origin=Origin(
            xyz=(
                cavity_w / 2.0,
                -cavity_d / 2.0,
                (cavity_floor_t + cavity_h + cavity_wall_t) / 2.0,
            )
        ),
    )
    model.articulation(
        "cabinet_to_cavity",
        ArticulationType.FIXED,
        parent=cabinet,
        child=cavity,
        origin=Origin(xyz=(cavity_x0, cavity_y0, cavity_z0)),
    )

    tray = model.part("turntable_tray")
    tray.visual(
        Cylinder(radius=0.125, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=glass_clear,
        name="tray_glass",
    )
    tray.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=keypad_accent,
        name="tray_hub",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.008),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "cavity_to_turntable",
        ArticulationType.FIXED,
        parent=cavity,
        child=tray,
        origin=Origin(
            xyz=(
                cavity_w / 2.0,
                -cavity_d / 2.0,
                cavity_floor_t,
            )
        ),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2.0, door_t / 2.0, door_h / 2.0)),
        material=trim_dark,
        name="door_outer_panel",
    )
    door.visual(
        Box((door_w - 0.058, 0.006, door_h - 0.082)),
        origin=Origin(
            xyz=(
                (door_w - 0.010) / 2.0,
                door_t - 0.003,
                door_h / 2.0 + 0.002,
            )
        ),
        material=glass_dark,
        name="door_window_glass",
    )
    door.visual(
        Box((door_w - 0.090, 0.004, door_h - 0.118)),
        origin=Origin(
            xyz=(
                (door_w - 0.040) / 2.0,
                0.002,
                door_h / 2.0 + 0.002,
            )
        ),
        material=glass_clear,
        name="door_inner_glass",
    )
    door.visual(
        Cylinder(radius=0.010, length=door_t + 0.018),
        origin=Origin(
            xyz=(door_w - 0.030, door_t * 0.5 + 0.009, door_h * 0.76),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=keypad_accent,
        name="handle_upper_post",
    )
    door.visual(
        Cylinder(radius=0.010, length=door_t + 0.018),
        origin=Origin(
            xyz=(door_w - 0.030, door_t * 0.5 + 0.009, door_h * 0.24),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=keypad_accent,
        name="handle_lower_post",
    )
    handle_path = [
        (door_w - 0.030, door_t + 0.018, door_h * 0.24),
        (door_w - 0.023, door_t + 0.022, door_h * 0.32),
        (door_w - 0.020, door_t + 0.024, door_h * 0.50),
        (door_w - 0.023, door_t + 0.022, door_h * 0.68),
        (door_w - 0.030, door_t + 0.018, door_h * 0.76),
    ]
    door.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                handle_path,
                radius=0.008,
                samples_per_segment=10,
                radial_segments=18,
                cap_ends=True,
            ),
            "microwave_pull_handle",
        ),
        material=keypad_accent,
        name="handle_grip",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.000, door_t / 2.0, door_h * 0.15)),
        material=trim_dark,
        name="hinge_barrel_lower",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(xyz=(0.000, door_t / 2.0, door_h * 0.52)),
        material=trim_dark,
        name="hinge_barrel_upper",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t + 0.050, door_h)),
        mass=5.0,
        origin=Origin(xyz=(door_w / 2.0, (door_t + 0.020) / 2.0, door_h / 2.0)),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_x0, 0.0, door_z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    button_w = 0.020
    button_h = 0.022
    button_t = 0.006
    plunger_t = 0.012
    col_centers = [0.0175, 0.0435, 0.0695, 0.0955]
    row_centers = [0.196, 0.164, 0.132, 0.100, 0.068]

    hole_profiles: list[list[tuple[float, float]]] = []
    for zc in row_centers:
        for xc in col_centers:
            half_w = button_w * 0.42
            half_h = button_h * 0.42
            hole_profiles.append(
                [
                    (xc - half_w, zc - half_h),
                    (xc + half_w, zc - half_h),
                    (xc + half_w, zc + half_h),
                    (xc - half_w, zc + half_h),
                ]
            )
    display_hole = [
        (0.018, 0.247),
        (panel_w - 0.018, 0.247),
        (panel_w - 0.018, 0.279),
        (0.018, 0.279),
    ]
    panel_geom = ExtrudeWithHolesGeometry(
        outer_profile=[(0.0, 0.0), (panel_w, 0.0), (panel_w, panel_h), (0.0, panel_h)],
        hole_profiles=[display_hole, *hole_profiles],
        height=panel_t,
        center=True,
        closed=True,
    )
    panel_geom.rotate_x(math.pi / 2.0)
    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_geometry(panel_geom, "microwave_control_panel"),
        origin=Origin(xyz=(0.0, -panel_t / 2.0, 0.0)),
        material=control_grey,
        name="panel_face",
    )
    control_panel.visual(
        Box((panel_w, 0.006, panel_h)),
        origin=Origin(xyz=(panel_w / 2.0, -panel_t + 0.003, panel_h / 2.0)),
        material=control_grey,
        name="panel_backing",
    )
    control_panel.visual(
        Box((panel_w - 0.038, 0.026, 0.028)),
        origin=Origin(xyz=(panel_w / 2.0, -0.013, 0.263)),
        material=glass_dark,
        name="display_window",
    )
    control_panel.visual(
        Box((panel_w - 0.050, 0.016, 0.012)),
        origin=Origin(xyz=(panel_w / 2.0, -0.018, 0.232)),
        material=keypad_accent,
        name="legend_strip",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((panel_w, panel_t, panel_h)),
        mass=1.3,
        origin=Origin(xyz=(panel_w / 2.0, -panel_t / 2.0, panel_h / 2.0)),
    )
    model.articulation(
        "cabinet_to_control_panel",
        ArticulationType.FIXED,
        parent=cabinet,
        child=control_panel,
        origin=Origin(xyz=(panel_x0, 0.0, panel_z0)),
    )

    for row_index, zc in enumerate(row_centers):
        for col_index, xc in enumerate(col_centers):
            key_part = model.part(f"key_{row_index}_{col_index}")
            key_part.visual(
                Box((button_w, button_t, button_h)),
                origin=Origin(xyz=(0.0, button_t / 2.0, 0.0)),
                material=keypad_dark,
                name="key_face",
            )
            key_part.visual(
                Box((button_w * 0.62, plunger_t, button_h * 0.62)),
                origin=Origin(xyz=(0.0, -plunger_t / 2.0, 0.0)),
                material=keypad_accent,
                name="key_plunger",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((button_w, button_t + plunger_t, button_h)),
                mass=0.012,
                origin=Origin(xyz=(0.0, (button_t - plunger_t) / 2.0, 0.0)),
            )
            model.articulation(
                f"control_panel_to_key_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=control_panel,
                child=key_part,
                origin=Origin(xyz=(xc, 0.0, zc)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=3.0,
                    velocity=0.06,
                    lower=0.0,
                    upper=0.0035,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    cavity = object_model.get_part("cavity_liner")
    door = object_model.get_part("door")
    control_panel = object_model.get_part("control_panel")
    tray = object_model.get_part("turntable_tray")
    sample_key = object_model.get_part("key_2_1")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    sample_key_joint = object_model.get_articulation("control_panel_to_key_2_1")

    ctx.expect_contact(
        cavity,
        cabinet,
        contact_tol=0.002,
        name="cavity liner is supported by the cabinet shell",
    )
    ctx.expect_contact(
        tray,
        cavity,
        contact_tol=0.001,
        name="turntable tray rests on the cavity floor",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.015,
        name="door sits just in front of the cabinet face",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.20,
        name="closed door spans the microwave front opening",
    )
    ctx.expect_within(
        sample_key,
        control_panel,
        axes="xz",
        margin=0.006,
        name="sample key stays within the keypad panel footprint",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_grip")
    with ctx.pose({door_hinge: math.radians(95.0)}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_grip")
    ctx.check(
        "door swings outward on the side hinge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][0] < closed_handle_aabb[1][0] - 0.15
        and open_handle_aabb[1][1] > closed_handle_aabb[1][1] + 0.16,
        details=f"closed_handle={closed_handle_aabb}, open_handle={open_handle_aabb}",
    )

    rest_key_aabb = ctx.part_element_world_aabb(sample_key, elem="key_face")
    with ctx.pose({sample_key_joint: 0.0035}):
        pressed_key_aabb = ctx.part_element_world_aabb(sample_key, elem="key_face")
    ctx.check(
        "sample keypad button plunges into the control panel",
        rest_key_aabb is not None
        and pressed_key_aabb is not None
        and pressed_key_aabb[1][1] < rest_key_aabb[1][1] - 0.0025,
        details=f"rest_key={rest_key_aabb}, pressed_key={pressed_key_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
