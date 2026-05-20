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
    mesh_from_geometry,
    wire_from_points,
)


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _member_rpy(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float]:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.hypot(vx, vy), vz)
    return (0.0, pitch, yaw)


def _add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    length = math.dist(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=_member_rpy(start, end),
        ),
        material=material,
        name=name,
    )


def _add_box_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    thickness: float,
    width: float,
    material,
    name: str | None = None,
) -> None:
    length = math.dist(start, end)
    part.visual(
        Box((thickness, width, length)),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=_member_rpy(start, end),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_tripod_device")

    steel_dark = model.material("steel_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.38, 0.40, 0.43, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.91, 0.74, 0.10, 1.0))
    device_charcoal = model.material("device_charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    lens_dark = model.material("lens_dark", rgba=(0.08, 0.11, 0.14, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.inertial = Inertial.from_geometry(
        Box((1.36, 1.36, 1.34)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )
    tripod_base.visual(
        Cylinder(radius=0.13, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 1.285)),
        material=steel_dark,
        name="head_plate",
    )
    tripod_base.visual(
        Cylinder(radius=0.10, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 1.235)),
        material=steel_mid,
        name="crown_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.055, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 1.045)),
        material=steel_dark,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.080, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=steel_mid,
        name="spreader_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.060, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=steel_dark,
        name="hub_sleeve",
    )
    tripod_base.visual(
        Box((0.03, 0.02, 0.035)),
        origin=Origin(xyz=(-0.145, 0.052, 1.3175)),
        material=safety_yellow,
        name="pan_stop_tab_left",
    )
    tripod_base.visual(
        Box((0.03, 0.02, 0.035)),
        origin=Origin(xyz=(-0.145, -0.052, 1.3175)),
        material=safety_yellow,
        name="pan_stop_tab_right",
    )
    tripod_base.visual(
        Box((0.05, 0.124, 0.015)),
        origin=Origin(xyz=(-0.145, 0.0, 1.3075)),
        material=steel_mid,
        name="pan_stop_bridge",
    )

    for leg_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        top_pt = _polar(0.12, angle, 1.24)
        knee_pt = _polar(0.38, angle, 0.58)
        brace_pt = _polar(0.29, angle, 0.83)
        foot_pt = _polar(0.63, angle, 0.05)
        hinge_center = _polar(0.15, angle, 1.225)
        knee_collar = _polar(0.38, angle, 0.58)
        brace_collar = _polar(0.29, angle, 0.83)
        inner_gusset = _polar(0.05, angle, 1.17)
        outer_gusset = _polar(0.13, angle, 1.215)
        arm_root = _polar(0.09, angle, 0.845)
        tangent = (-math.sin(angle), math.cos(angle), 0.0)

        tripod_base.visual(
            Box((0.09, 0.06, 0.10)),
            origin=Origin(xyz=hinge_center, rpy=(0.0, 0.0, angle)),
            material=steel_mid,
            name=f"leg{leg_index}_hinge_block",
        )
        tripod_base.visual(
            Box((0.085, 0.06, 0.055)),
            origin=Origin(xyz=knee_collar, rpy=(0.0, 0.0, angle)),
            material=steel_mid,
            name=f"leg{leg_index}_lock_collar",
        )
        tripod_base.visual(
            Box((0.07, 0.05, 0.05)),
            origin=Origin(xyz=brace_collar, rpy=(0.0, 0.0, angle)),
            material=steel_mid,
            name=f"leg{leg_index}_brace_collar",
        )
        tripod_base.visual(
            Box((0.14, 0.085, 0.05)),
            origin=Origin(xyz=(foot_pt[0], foot_pt[1], 0.025), rpy=(0.0, 0.0, angle)),
            material=rubber_black,
            name=f"leg{leg_index}_foot",
        )
        _add_cylinder_between(
            tripod_base,
            foot_pt,
            knee_pt,
            radius=0.024,
            material=steel_dark,
            name=f"leg{leg_index}_lower_tube",
        )
        _add_cylinder_between(
            tripod_base,
            knee_pt,
            top_pt,
            radius=0.030,
            material=steel_dark,
            name=f"leg{leg_index}_upper_tube",
        )
        _add_cylinder_between(
            tripod_base,
            arm_root,
            brace_pt,
            radius=0.012,
            material=steel_mid,
            name=f"leg{leg_index}_spreader_arm",
        )
        _add_box_between(
            tripod_base,
            inner_gusset,
            outer_gusset,
            thickness=0.012,
            width=0.08,
            material=steel_mid,
            name=f"leg{leg_index}_crown_gusset",
        )
        _add_cylinder_between(
            tripod_base,
            (
                hinge_center[0] - 0.04 * tangent[0],
                hinge_center[1] - 0.04 * tangent[1],
                hinge_center[2],
            ),
            (
                hinge_center[0] + 0.04 * tangent[0],
                hinge_center[1] + 0.04 * tangent[1],
                hinge_center[2],
            ),
            radius=0.008,
            material=fastener_steel,
            name=f"leg{leg_index}_hinge_bolt",
        )
        _add_cylinder_between(
            tripod_base,
            (
                brace_collar[0] - 0.028 * tangent[0],
                brace_collar[1] - 0.028 * tangent[1],
                brace_collar[2],
            ),
            (
                brace_collar[0] + 0.028 * tangent[0],
                brace_collar[1] + 0.028 * tangent[1],
                brace_collar[2],
            ),
            radius=0.006,
            material=fastener_steel,
            name=f"leg{leg_index}_brace_bolt",
        )

    pan_head = model.part("pan_head")
    pan_head.inertial = Inertial.from_geometry(
        Box((0.26, 0.30, 0.26)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )
    pan_head.visual(
        Cylinder(radius=0.11, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=steel_dark,
        name="rotation_base",
    )
    pan_head.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=steel_mid,
        name="bearing_skirt",
    )
    pan_head.visual(
        Box((0.11, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=steel_dark,
        name="head_pedestal",
    )
    pan_head.visual(
        Box((0.05, 0.03, 0.20)),
        origin=Origin(xyz=(0.0, 0.12, 0.15)),
        material=steel_mid,
        name="left_tower",
    )
    pan_head.visual(
        Box((0.05, 0.03, 0.20)),
        origin=Origin(xyz=(0.0, -0.12, 0.15)),
        material=steel_mid,
        name="right_tower",
    )
    pan_head.visual(
        Box((0.05, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.095, 0.09)),
        material=steel_mid,
        name="left_tower_web",
    )
    pan_head.visual(
        Box((0.05, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, -0.095, 0.09)),
        material=steel_mid,
        name="right_tower_web",
    )
    _add_cylinder_between(
        pan_head,
        (0.0, 0.10, 0.185),
        (0.0, 0.14, 0.185),
        radius=0.018,
        material=fastener_steel,
        name="left_trunnion",
    )
    _add_cylinder_between(
        pan_head,
        (0.0, -0.10, 0.185),
        (0.0, -0.14, 0.185),
        radius=0.018,
        material=fastener_steel,
        name="right_trunnion",
    )
    for stop_index, x_center in enumerate((-0.03, 0.005), start=1):
        pan_head.visual(
            Box((0.04, 0.03, 0.04)),
            origin=Origin(xyz=(x_center, 0.11, 0.175)),
            material=safety_yellow,
            name=f"left_tilt_stop_{stop_index}",
        )
        pan_head.visual(
            Box((0.04, 0.03, 0.04)),
            origin=Origin(xyz=(x_center, -0.11, 0.175)),
            material=safety_yellow,
            name=f"right_tilt_stop_{stop_index}",
        )
    pan_head.visual(
        Box((0.025, 0.018, 0.04)),
        origin=Origin(xyz=(-0.115, 0.0, 0.045)),
        material=safety_yellow,
        name="pan_stop_lug",
    )
    pan_head.visual(
        Box((0.045, 0.055, 0.045)),
        origin=Origin(xyz=(-0.080, -0.105, 0.045)),
        material=steel_mid,
        name="pan_lock_body",
    )
    _add_cylinder_between(
        pan_head,
        (-0.1025, -0.105, 0.045),
        (-0.1425, -0.105, 0.045),
        radius=0.018,
        material=fastener_steel,
        name="pan_lock_knob",
    )
    pan_head.visual(
        Box((0.02, 0.03, 0.07)),
        origin=Origin(xyz=(-0.080, -0.132, 0.052)),
        material=safety_yellow,
        name="pan_lock_guard_outer",
    )
    pan_head.visual(
        Box((0.02, 0.03, 0.07)),
        origin=Origin(xyz=(-0.080, -0.078, 0.052)),
        material=safety_yellow,
        name="pan_lock_guard_inner",
    )

    tilt_yoke = model.part("tilt_yoke")
    tilt_yoke.inertial = Inertial.from_geometry(
        Box((0.44, 0.40, 0.36)),
        mass=7.5,
        origin=Origin(xyz=(0.18, 0.0, 0.01)),
    )
    tilt_yoke.visual(
        Box((0.28, 0.02, 0.30)),
        origin=Origin(xyz=(0.14, 0.15, 0.015)),
        material=steel_mid,
        name="left_side_plate",
    )
    tilt_yoke.visual(
        Box((0.28, 0.02, 0.30)),
        origin=Origin(xyz=(0.14, -0.15, 0.015)),
        material=steel_mid,
        name="right_side_plate",
    )
    tilt_yoke.visual(
        Box((0.06, 0.04, 0.08)),
        origin=Origin(xyz=(0.07, 0.13, 0.0)),
        material=steel_dark,
        name="left_bearing_cheek",
    )
    tilt_yoke.visual(
        Box((0.06, 0.04, 0.08)),
        origin=Origin(xyz=(0.07, -0.13, 0.0)),
        material=steel_dark,
        name="right_bearing_cheek",
    )
    tilt_yoke.visual(
        Box((0.08, 0.28, 0.03)),
        origin=Origin(xyz=(0.08, 0.0, 0.065)),
        material=steel_dark,
        name="lower_bridge",
    )
    tilt_yoke.visual(
        Box((0.08, 0.30, 0.03)),
        origin=Origin(xyz=(0.12, 0.0, 0.155)),
        material=steel_dark,
        name="upper_bridge",
    )
    tilt_yoke.visual(
        Box((0.16, 0.255, 0.015)),
        origin=Origin(xyz=(0.20, 0.0, -0.1375)),
        material=steel_dark,
        name="tray_plate",
    )
    tilt_yoke.visual(
        Box((0.02, 0.20, 0.08)),
        origin=Origin(xyz=(0.11, 0.0, 0.045)),
        material=steel_mid,
        name="back_plate",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.185, 0.175, -0.015)),
        material=steel_mid,
        name="left_slide_rail",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.185, -0.175, -0.015)),
        material=steel_mid,
        name="right_slide_rail",
    )
    tilt_yoke.visual(
        Box((0.025, 0.045, 0.03)),
        origin=Origin(xyz=(0.215, 0.175, 0.075)),
        material=safety_yellow,
        name="left_overtravel_ear",
    )
    tilt_yoke.visual(
        Box((0.025, 0.045, 0.03)),
        origin=Origin(xyz=(0.215, -0.175, 0.075)),
        material=safety_yellow,
        name="right_overtravel_ear",
    )
    _add_cylinder_between(
        tilt_yoke,
        (0.015, 0.17, 0.015),
        (0.055, 0.17, 0.015),
        radius=0.016,
        material=fastener_steel,
        name="left_tilt_lock_knob",
    )
    _add_cylinder_between(
        tilt_yoke,
        (0.015, -0.17, 0.015),
        (0.055, -0.17, 0.015),
        radius=0.016,
        material=fastener_steel,
        name="right_tilt_lock_knob",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(0.03, 0.17, 0.065)),
        material=safety_yellow,
        name="left_lock_guard_front",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(0.03, 0.17, -0.015)),
        material=safety_yellow,
        name="left_lock_guard_rear",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(0.03, -0.17, 0.065)),
        material=safety_yellow,
        name="right_lock_guard_front",
    )
    tilt_yoke.visual(
        Box((0.03, 0.03, 0.07)),
        origin=Origin(xyz=(0.03, -0.17, -0.015)),
        material=safety_yellow,
        name="right_lock_guard_rear",
    )
    tilt_yoke.visual(
        Box((0.02, 0.02, 0.24)),
        origin=Origin(xyz=(0.31, 0.15, -0.03)),
        material=safety_yellow,
        name="left_guard_post",
    )
    tilt_yoke.visual(
        Box((0.02, 0.02, 0.24)),
        origin=Origin(xyz=(0.31, -0.15, -0.03)),
        material=safety_yellow,
        name="right_guard_post",
    )
    tilt_yoke.visual(
        Box((0.02, 0.30, 0.02)),
        origin=Origin(xyz=(0.31, 0.0, 0.07)),
        material=safety_yellow,
        name="top_guard_bar",
    )
    tilt_yoke.visual(
        Box((0.02, 0.30, 0.02)),
        origin=Origin(xyz=(0.31, 0.0, -0.14)),
        material=safety_yellow,
        name="bottom_guard_bar",
    )
    tilt_yoke.visual(
        Box((0.25, 0.02, 0.02)),
        origin=Origin(xyz=(0.185, 0.15, 0.07)),
        material=steel_mid,
        name="left_guard_upper_standoff",
    )
    tilt_yoke.visual(
        Box((0.25, 0.02, 0.02)),
        origin=Origin(xyz=(0.185, -0.15, 0.07)),
        material=steel_mid,
        name="right_guard_upper_standoff",
    )
    tilt_yoke.visual(
        Box((0.25, 0.02, 0.02)),
        origin=Origin(xyz=(0.185, 0.15, -0.14)),
        material=steel_mid,
        name="left_guard_lower_standoff",
    )
    tilt_yoke.visual(
        Box((0.25, 0.02, 0.02)),
        origin=Origin(xyz=(0.185, -0.15, -0.14)),
        material=steel_mid,
        name="right_guard_lower_standoff",
    )
    _add_box_between(
        tilt_yoke,
        (0.10, 0.145, 0.02),
        (0.16, 0.145, -0.1375),
        thickness=0.012,
        width=0.035,
        material=steel_mid,
        name="left_tray_brace",
    )
    _add_box_between(
        tilt_yoke,
        (0.10, -0.145, 0.02),
        (0.16, -0.145, -0.1375),
        thickness=0.012,
        width=0.035,
        material=steel_mid,
        name="right_tray_brace",
    )

    device_body = model.part("device_body")
    device_body.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.22)),
        mass=9.0,
        origin=Origin(xyz=(0.10, 0.0, 0.09)),
    )
    device_body.visual(
        Box((0.14, 0.18, 0.01)),
        origin=Origin(xyz=(0.07, 0.0, 0.005)),
        material=steel_mid,
        name="mount_rail",
    )
    device_body.visual(
        Box((0.18, 0.22, 0.15)),
        origin=Origin(xyz=(0.11, 0.0, 0.085)),
        material=device_charcoal,
        name="housing_shell",
    )
    device_body.visual(
        Box((0.012, 0.18, 0.12)),
        origin=Origin(xyz=(0.206, 0.0, 0.085)),
        material=steel_mid,
        name="front_bezel",
    )
    device_body.visual(
        Box((0.002, 0.15, 0.09)),
        origin=Origin(xyz=(0.200, 0.0, 0.085)),
        material=lens_dark,
        name="lens_panel",
    )
    device_body.visual(
        Box((0.08, 0.12, 0.02)),
        origin=Origin(xyz=(0.07, 0.0, 0.16)),
        material=steel_mid,
        name="service_cover",
    )
    device_body.visual(
        Box((0.02, 0.03, 0.14)),
        origin=Origin(xyz=(0.10, 0.125, 0.085)),
        material=steel_mid,
        name="left_side_rib",
    )
    device_body.visual(
        Box((0.02, 0.03, 0.14)),
        origin=Origin(xyz=(0.10, -0.125, 0.085)),
        material=steel_mid,
        name="right_side_rib",
    )
    for bolt_index, y_pos, z_pos in (
        (1, 0.07, 0.15),
        (2, -0.07, 0.15),
        (3, 0.07, 0.05),
        (4, -0.07, 0.05),
    ):
        _add_cylinder_between(
            device_body,
            (0.212, y_pos, z_pos - 0.015),
            (0.218, y_pos, z_pos - 0.015),
            radius=0.006,
            material=fastener_steel,
            name=f"front_fastener_{bolt_index}",
        )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.18)),
        mass=1.8,
        origin=Origin(xyz=(0.10, 0.22, -0.015)),
    )
    clamp_jaw.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(0.03, 0.175, -0.04)),
        material=steel_mid,
        name="slider_plate",
    )
    clamp_jaw.visual(
        Box((0.03, 0.03, 0.04)),
        origin=Origin(xyz=(0.055, 0.235, -0.04)),
        material=steel_mid,
        name="right_slider_plate",
    )
    clamp_jaw.visual(
        Box((0.06, 0.04, 0.08)),
        origin=Origin(xyz=(0.055, 0.21, -0.015)),
        material=steel_mid,
        name="jaw_carriage",
    )
    clamp_jaw.visual(
        Box((0.08, 0.04, 0.08)),
        origin=Origin(xyz=(0.10, 0.19, -0.015)),
        material=steel_mid,
        name="jaw_plate",
    )
    clamp_jaw.visual(
        Box((0.12, 0.04, 0.05)),
        origin=Origin(xyz=(0.145, 0.19, -0.015)),
        material=steel_mid,
        name="pressure_arm",
    )
    clamp_jaw.visual(
        Box((0.008, 0.06, 0.05)),
        origin=Origin(xyz=(0.188, 0.19, -0.015)),
        material=rubber_black,
        name="pressure_pad",
    )
    clamp_jaw.visual(
        Box((0.05, 0.06, 0.06)),
        origin=Origin(xyz=(0.055, 0.255, -0.015)),
        material=steel_mid,
        name="screw_block",
    )
    _add_cylinder_between(
        clamp_jaw,
        (0.055, 0.27, -0.015),
        (0.055, 0.33, -0.015),
        radius=0.012,
        material=fastener_steel,
        name="clamp_handwheel",
    )
    clamp_jaw.visual(
        Box((0.03, 0.01, 0.03)),
        origin=Origin(xyz=(0.055, 0.30, -0.015)),
        material=safety_yellow,
        name="clamp_lock_tab",
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.REVOLUTE,
        parent=tripod_base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=-math.radians(160.0),
            upper=math.radians(160.0),
        ),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.7,
            lower=-math.radians(20.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "tilt_to_device",
        ArticulationType.FIXED,
        parent=tilt_yoke,
        child=device_body,
        origin=Origin(xyz=(0.12, 0.0, -0.13)),
    )
    model.articulation(
        "tilt_to_clamp",
        ArticulationType.PRISMATIC,
        parent=tilt_yoke,
        child=clamp_jaw,
        origin=Origin(xyz=(0.157, 0.0, -0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.03,
            lower=0.0,
            upper=0.007,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    pan_head = object_model.get_part("pan_head")
    tilt_yoke = object_model.get_part("tilt_yoke")
    device_body = object_model.get_part("device_body")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan_joint = object_model.get_articulation("tripod_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_tilt")
    clamp_joint = object_model.get_articulation("tilt_to_clamp")

    head_plate = tripod_base.get_visual("head_plate")
    rotation_base = pan_head.get_visual("rotation_base")
    left_trunnion = pan_head.get_visual("left_trunnion")
    left_side_plate = tilt_yoke.get_visual("left_side_plate")
    tray_plate = tilt_yoke.get_visual("tray_plate")
    left_slide_rail = tilt_yoke.get_visual("left_slide_rail")
    mount_rail = device_body.get_visual("mount_rail")
    front_bezel = device_body.get_visual("front_bezel")
    slider_plate = clamp_jaw.get_visual("slider_plate")
    pressure_pad = clamp_jaw.get_visual("pressure_pad")

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

    ctx.expect_contact(
        pan_head,
        tripod_base,
        elem_a=rotation_base,
        elem_b=head_plate,
        name="pan head seats on tripod head plate",
    )
    ctx.expect_contact(
        tilt_yoke,
        pan_head,
        elem_a=left_side_plate,
        elem_b=left_trunnion,
        name="tilt yoke is carried by trunnion support",
    )
    ctx.expect_contact(
        device_body,
        tilt_yoke,
        elem_a=mount_rail,
        elem_b=tray_plate,
        name="device body is mounted on tray plate",
    )
    ctx.expect_contact(
        clamp_jaw,
        tilt_yoke,
        elem_a=slider_plate,
        elem_b=left_slide_rail,
        name="clamp carriage is supported on slide rail",
    )
    ctx.expect_gap(
        clamp_jaw,
        device_body,
        axis="x",
        min_gap=0.006,
        max_gap=0.012,
        positive_elem=pressure_pad,
        negative_elem=front_bezel,
        name="clamp starts slightly open ahead of device face",
    )
    with ctx.pose({clamp_joint: clamp_joint.motion_limits.upper}):
        ctx.expect_gap(
            clamp_jaw,
            device_body,
            axis="x",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem=pressure_pad,
            negative_elem=front_bezel,
            name="clamp closes without penetrating device face",
        )

    rest_device_pos = ctx.part_world_position(device_body)
    with ctx.pose({tilt_joint: math.radians(45.0)}):
        raised_device_pos = ctx.part_world_position(device_body)
    ctx.check(
        "positive tilt raises device",
        rest_device_pos is not None
        and raised_device_pos is not None
        and raised_device_pos[2] > rest_device_pos[2] + 0.06,
        details=f"rest={rest_device_pos}, raised={raised_device_pos}",
    )

    rest_pan_pos = ctx.part_world_position(device_body)
    with ctx.pose({pan_joint: math.radians(45.0)}):
        panned_device_pos = ctx.part_world_position(device_body)
    ctx.check(
        "positive pan swings device toward positive y",
        rest_pan_pos is not None
        and panned_device_pos is not None
        and panned_device_pos[1] > rest_pan_pos[1] + 0.06,
        details=f"rest={rest_pan_pos}, panned={panned_device_pos}",
    )
    ctx.check(
        "joint axes match industrial head intent",
        pan_joint.axis == (0.0, 0.0, 1.0)
        and tilt_joint.axis == (0.0, -1.0, 0.0)
        and clamp_joint.axis == (-1.0, 0.0, 0.0),
        details=f"pan={pan_joint.axis}, tilt={tilt_joint.axis}, clamp={clamp_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
