from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_microwave_oven")

    stainless = model.material("stainless", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.11, 0.12, 1.0))
    cavity_gray = model.material("cavity_gray", rgba=(0.79, 0.80, 0.82, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.23, 0.28, 0.45))
    clear_glass = model.material("clear_glass", rgba=(0.74, 0.85, 0.92, 0.45))
    knob_marker = model.material("knob_marker", rgba=(0.86, 0.18, 0.12, 1.0))

    width = 0.56
    depth = 0.47
    height = 0.37
    shell_t = 0.02
    front_face_y = -depth * 0.5

    cavity_left = -0.26
    cavity_right = 0.08
    cavity_center_x = (cavity_left + cavity_right) * 0.5
    cavity_width = cavity_right - cavity_left
    cavity_front = -0.18
    cavity_back = 0.207
    cavity_depth = cavity_back - cavity_front
    cavity_floor_top = 0.023
    cavity_ceiling_bottom = 0.347
    cavity_height = cavity_ceiling_bottom - cavity_floor_top
    cavity_center_z = (cavity_floor_top + cavity_ceiling_bottom) * 0.5

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t * 0.5)),
        material=stainless,
        name="bottom_shell",
    )
    housing.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t * 0.5)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((shell_t, depth, height - shell_t)),
        origin=Origin(xyz=(-0.27, 0.0, height * 0.5)),
        material=stainless,
        name="left_shell",
    )
    housing.visual(
        Box((shell_t, depth, height - shell_t)),
        origin=Origin(xyz=(0.27, 0.0, height * 0.5)),
        material=stainless,
        name="right_shell",
    )
    housing.visual(
        Box((width - 2.0 * shell_t, 0.025, height - shell_t)),
        origin=Origin(xyz=(0.0, 0.2225, height * 0.5)),
        material=stainless,
        name="back_shell",
    )
    housing.visual(
        Box((0.03, 0.44, height - shell_t)),
        origin=Origin(xyz=(0.095, 0.0, height * 0.5)),
        material=stainless,
        name="divider",
    )
    housing.visual(
        Box((cavity_width, 0.03, 0.07)),
        origin=Origin(xyz=(cavity_center_x, -0.22, 0.035)),
        material=dark_trim,
        name="door_sill",
    )
    housing.visual(
        Box((cavity_width, 0.03, 0.055)),
        origin=Origin(xyz=(cavity_center_x, -0.22, 0.3425)),
        material=dark_trim,
        name="door_header",
    )

    panel_outer = _rect_profile(0.16, 0.28)
    panel_hole = _circle_profile(0.033, center=(0.0, 0.0), segments=32)
    panel_front_geom = ExtrudeWithHolesGeometry(panel_outer, [panel_hole], height=0.03, center=True)
    panel_front_geom.rotate_x(pi * 0.5)
    housing.visual(
        _mesh("control_panel_front", panel_front_geom),
        origin=Origin(xyz=(0.19, -0.22, 0.19)),
        material=control_black,
        name="panel_front",
    )
    housing.visual(
        Box((0.112, 0.003, 0.19)),
        origin=Origin(xyz=(0.19, -0.2315, 0.19)),
        material=dark_trim,
        name="panel_trim",
    )

    housing.visual(
        Box((cavity_width - 0.006, cavity_depth, 0.003)),
        origin=Origin(xyz=(cavity_center_x, (cavity_front + cavity_back) * 0.5, cavity_floor_top - 0.0015)),
        material=cavity_gray,
        name="cavity_floor",
    )
    housing.visual(
        Box((cavity_width - 0.006, cavity_depth, 0.003)),
        origin=Origin(xyz=(cavity_center_x, (cavity_front + cavity_back) * 0.5, cavity_ceiling_bottom + 0.0015)),
        material=cavity_gray,
        name="cavity_ceiling",
    )
    housing.visual(
        Box((0.003, cavity_depth, cavity_height)),
        origin=Origin(xyz=(cavity_left + 0.0015, (cavity_front + cavity_back) * 0.5, cavity_center_z)),
        material=cavity_gray,
        name="cavity_left_wall",
    )
    housing.visual(
        Box((0.003, cavity_depth, cavity_height)),
        origin=Origin(xyz=(cavity_right - 0.0015, (cavity_front + cavity_back) * 0.5, cavity_center_z)),
        material=cavity_gray,
        name="cavity_right_wall",
    )
    housing.visual(
        Box((cavity_width - 0.006, 0.003, cavity_height)),
        origin=Origin(xyz=(cavity_center_x, cavity_back + 0.0015, cavity_center_z)),
        material=cavity_gray,
        name="cavity_back_wall",
    )

    door = model.part("door")
    door_outer_w = 0.37
    door_outer_h = 0.275
    door_thickness = 0.022
    door_window_w = 0.24
    door_window_h = 0.15
    door_frame_geom = ExtrudeWithHolesGeometry(
        _rect_profile(door_outer_w, door_outer_h),
        [_rect_profile(door_window_w, door_window_h)],
        height=door_thickness,
        center=True,
    )
    door_frame_geom.rotate_x(pi * 0.5)
    door.visual(
        _mesh("door_frame", door_frame_geom),
        origin=Origin(xyz=(0.0, -door_thickness * 0.5, door_outer_h * 0.5)),
        material=dark_trim,
        name="door_frame",
    )
    door.visual(
        Box((door_window_w + 0.03, 0.004, door_window_h + 0.02)),
        origin=Origin(xyz=(0.0, -0.011, 0.145)),
        material=smoked_glass,
        name="door_window",
    )
    door.visual(
        Box((0.18, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, -0.031, 0.225)),
        material=stainless,
        name="door_handle",
    )

    timer_dial = model.part("timer_dial")
    dial_rot = Origin(rpy=(pi * 0.5, 0.0, 0.0))
    timer_dial.visual(
        Cylinder(radius=0.04, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="dial_flange",
    )
    timer_dial.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="dial_body",
    )
    timer_dial.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_trim,
        name="dial_shaft",
    )
    timer_dial.visual(
        Box((0.005, 0.004, 0.016)),
        origin=Origin(xyz=(0.017, -0.030, 0.0)),
        material=knob_marker,
        name="dial_pointer",
    )

    glass_tray = model.part("glass_tray")
    glass_tray.visual(
        Cylinder(radius=0.145, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=clear_glass,
        name="tray_disk",
    )
    glass_tray.visual(
        Cylinder(radius=0.035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=clear_glass,
        name="tray_hub",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(cavity_center_x, front_face_y, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "timer_dial_joint",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=timer_dial,
        origin=Origin(xyz=(0.19, front_face_y, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0, lower=0.0, upper=6.0),
    )
    model.articulation(
        "turntable_joint",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=glass_tray,
        origin=Origin(xyz=(cavity_center_x, 0.0135, cavity_floor_top + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    timer_dial = object_model.get_part("timer_dial")
    glass_tray = object_model.get_part("glass_tray")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_joint = object_model.get_articulation("timer_dial_joint")
    tray_joint = object_model.get_articulation("turntable_joint")

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

    ctx.check(
        "parts_present",
        all(part is not None for part in (housing, door, timer_dial, glass_tray)),
        "expected housing, door, timer_dial, and glass_tray parts",
    )
    ctx.check(
        "door_hinge_axis",
        door_hinge.axis == (1.0, 0.0, 0.0) and door_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"expected lower-edge revolute hinge on x axis, got {door_hinge.axis}",
    )
    ctx.check(
        "timer_dial_axis",
        dial_joint.axis == (0.0, 1.0, 0.0) and dial_joint.articulation_type == ArticulationType.REVOLUTE,
        f"expected front-to-back dial axis, got {dial_joint.axis}",
    )
    ctx.check(
        "turntable_axis",
        tray_joint.axis == (0.0, 0.0, 1.0) and tray_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"expected vertical continuous turntable axis, got {tray_joint.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(door, housing, name="door_closed_contacts_housing")
    ctx.expect_contact(timer_dial, housing, elem_a="dial_flange", elem_b="panel_front", name="dial_mounted_to_panel")
    ctx.expect_gap(
        glass_tray,
        housing,
        axis="z",
        negative_elem="cavity_floor",
        min_gap=0.0,
        max_gap=0.0,
        name="tray_seated_on_cavity_floor",
    )
    ctx.expect_within(
        glass_tray,
        housing,
        axes="xy",
        outer_elem="cavity_floor",
        margin=0.0,
        name="tray_within_cavity_footprint",
    )

    with ctx.pose({door_hinge: 1.2}):
        door_aabb = ctx.part_world_aabb(door)
        opens_down = (
            door_aabb is not None
            and door_aabb[1][2] < 0.20
            and door_aabb[0][1] < -0.30
        )
        ctx.check(
            "door_opens_downward",
            opens_down,
            f"expected opened door to swing forward and down, got aabb={door_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
