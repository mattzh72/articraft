from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

TRAY_TOP = 0.008
PIVOT_Z = 0.052
MOTOR_AXIS = (0.0, -0.120, 0.052)
LEFT_PIVOT_X = -0.22
RIGHT_PIVOT_X = 0.22

LEVER_RADIUS = 0.085
LEFT_LEVER_YAW = -0.65
RIGHT_LEVER_YAW = pi + 0.65

ARM_RADIUS = 0.232
LEFT_ARM_YAW = 0.70
RIGHT_ARM_YAW = pi - 0.70

CRANK_RADIUS = 0.040
CRANK_PARK_YAW = -2.65

LEFT_PIN_LOCAL = (
    LEVER_RADIUS * cos(LEFT_LEVER_YAW),
    LEVER_RADIUS * sin(LEFT_LEVER_YAW),
)
RIGHT_PIN_LOCAL = (
    LEVER_RADIUS * cos(RIGHT_LEVER_YAW),
    LEVER_RADIUS * sin(RIGHT_LEVER_YAW),
)
LEFT_PIN_WORLD = (LEFT_PIVOT_X + LEFT_PIN_LOCAL[0], LEFT_PIN_LOCAL[1])
RIGHT_PIN_WORLD = (RIGHT_PIVOT_X + RIGHT_PIN_LOCAL[0], RIGHT_PIN_LOCAL[1])
CRANK_PIN_LOCAL = (
    CRANK_RADIUS * cos(CRANK_PARK_YAW),
    CRANK_RADIUS * sin(CRANK_PARK_YAW),
)
CRANK_PIN_WORLD = (MOTOR_AXIS[0] + CRANK_PIN_LOCAL[0], MOTOR_AXIS[1] + CRANK_PIN_LOCAL[1])

DRAG_VECTOR = (
    LEFT_PIN_WORLD[0] - CRANK_PIN_WORLD[0],
    LEFT_PIN_WORLD[1] - CRANK_PIN_WORLD[1],
)
DRAG_LINK_LENGTH = hypot(DRAG_VECTOR[0], DRAG_VECTOR[1])
DRAG_LINK_YAW = 0.0 if DRAG_LINK_LENGTH == 0.0 else atan2(DRAG_VECTOR[1], DRAG_VECTOR[0])

CROSS_VECTOR = (
    RIGHT_PIN_WORLD[0] - LEFT_PIN_WORLD[0],
    RIGHT_PIN_WORLD[1] - LEFT_PIN_WORLD[1],
)
CROSS_LINK_LENGTH = hypot(CROSS_VECTOR[0], CROSS_VECTOR[1])
CROSS_LINK_YAW = 0.0 if CROSS_LINK_LENGTH == 0.0 else atan2(CROSS_VECTOR[1], CROSS_VECTOR[0])


def _add_bolt_head(
    part,
    *,
    xyz: tuple[float, float, float],
    radius: float = 0.005,
    height: float = 0.0035,
    axis: str = "z",
    material: str = "fastener_metal",
    name: str | None = None,
) -> None:
    rotations = {
        "x": (0.0, pi / 2.0, 0.0),
        "y": (pi / 2.0, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=xyz, rpy=rotations[axis]),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_section(width: float, height: float, radius: float, x_pos: float, *, z_center: float = 0.0):
    return [(x_pos, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]


def _build_tapered_arm_shell_mesh(name: str):
    return _save_mesh(
        name,
        section_loft(
            [
                _yz_section(0.040, 0.014, 0.004, 0.000, z_center=0.000),
                _yz_section(0.032, 0.012, 0.0035, 0.070, z_center=0.001),
                _yz_section(0.022, 0.010, 0.0030, 0.150, z_center=-0.001),
                _yz_section(0.016, 0.008, 0.0025, 0.214, z_center=-0.002),
            ]
        ),
    )


def _build_blade_carrier_shell_mesh(name: str):
    return _save_mesh(
        name,
        section_loft(
            [
                _yz_section(0.014, 0.006, 0.0018, -0.120, z_center=0.000),
                _yz_section(0.018, 0.008, 0.0025, -0.040, z_center=0.001),
                _yz_section(0.024, 0.010, 0.0030, 0.000, z_center=0.002),
                _yz_section(0.018, 0.008, 0.0025, 0.040, z_center=0.001),
                _yz_section(0.014, 0.006, 0.0018, 0.120, z_center=0.000),
            ]
        ),
    )


def _build_pivot_tower(part, *, x_center: float, prefix: str) -> None:
    part.visual(
        Box((0.072, 0.060, 0.010)),
        origin=Origin(xyz=(x_center, 0.0, TRAY_TOP + 0.005)),
        material="frame_steel",
        name=f"{prefix}_base_pad",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Box((0.016, 0.034, 0.032)),
            origin=Origin(xyz=(x_center + side * 0.026, 0.010, 0.034)),
            material="housing_paint",
            name=f"{prefix}_cheek_{'left' if side < 0.0 else 'right'}",
        )
    part.visual(
        Box((0.050, 0.016, 0.016)),
        origin=Origin(xyz=(x_center, 0.022, 0.033)),
        material="housing_paint",
        name=f"{prefix}_front_gusset",
    )
    part.visual(
        Box((0.040, 0.010, 0.008)),
        origin=Origin(xyz=(x_center, 0.024, 0.038)),
        material="housing_paint",
        name=f"{prefix}_bridge",
    )
    for dx in (-0.022, 0.022):
        for dy in (-0.018, 0.018):
            _add_bolt_head(
                part,
                xyz=(x_center + dx, dy, 0.0185),
                radius=0.0045,
                height=0.003,
                name=f"{prefix}_bolt_{'n' if dy > 0.0 else 's'}_{'l' if dx < 0.0 else 'r'}",
            )


def _build_wiper_blade(part, *, prefix: str) -> None:
    part.visual(
        Box((0.295, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="arm_paint",
        name=f"{prefix}_blade_backbone",
    )
    part.visual(
        Box((0.275, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material="rubber",
        name=f"{prefix}_squeegee",
    )
    part.visual(
        Box((0.046, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material="arm_paint",
        name=f"{prefix}_saddle",
    )
    part.visual(
        Box((0.210, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.010, -0.009)),
        material="arm_paint",
        name=f"{prefix}_spoiler",
    )
    part.visual(
        _build_blade_carrier_shell_mesh(f"{prefix}_blade_carrier_shell.obj"),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material="arm_paint",
        name=f"{prefix}_carrier_shell",
    )
    for x_mid, tag in ((-0.082, "inner_claw"), (0.082, "outer_claw")):
        part.visual(
            Box((0.012, 0.020, 0.006)),
            origin=Origin(xyz=(x_mid, 0.0, -0.014)),
            material="arm_paint",
            name=f"{prefix}_{tag}",
        )
    for x_end, tag in ((-0.140, "inboard"), (0.140, "outboard")):
        part.visual(
            Box((0.012, 0.020, 0.010)),
            origin=Origin(xyz=(x_end, 0.0, -0.012)),
            material="arm_paint",
            name=f"{prefix}_{tag}_endcap",
        )


def _build_spindle_module(part, *, side: str) -> None:
    is_left = side == "left"
    arm_yaw = LEFT_ARM_YAW if is_left else RIGHT_ARM_YAW
    lever_yaw = LEFT_LEVER_YAW if is_left else RIGHT_LEVER_YAW
    lever_x = LEVER_RADIUS * cos(lever_yaw)
    lever_y = LEVER_RADIUS * sin(lever_yaw)
    arm_tip_x = ARM_RADIUS * cos(arm_yaw)
    arm_tip_y = ARM_RADIUS * sin(arm_yaw)

    part.visual(
        Cylinder(radius=0.010, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="linkage_steel",
        name=f"{side}_shaft",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material="fastener_metal",
        name=f"{side}_collar",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material="fastener_metal",
        name=f"{side}_pivot_nut",
    )

    part.visual(
        Box((0.082, 0.024, 0.006)),
        origin=Origin(
            xyz=(0.041 * cos(lever_yaw), 0.041 * sin(lever_yaw), 0.009),
            rpy=(0.0, 0.0, lever_yaw),
        ),
        material="linkage_steel",
        name=f"{side}_drive_lever",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(lever_x, lever_y, 0.009)),
        material="linkage_steel",
        name=f"{side}_lever_eye",
    )
    part.visual(
        Box((0.028, 0.020, 0.012)),
        origin=Origin(
            xyz=(0.018 * cos(lever_yaw), 0.018 * sin(lever_yaw), 0.011),
            rpy=(0.0, 0.0, lever_yaw),
        ),
        material="linkage_steel",
        name=f"{side}_lever_root",
    )

    part.visual(
        Box((0.074, 0.036, 0.012)),
        origin=Origin(
            xyz=(0.034 * cos(arm_yaw), 0.034 * sin(arm_yaw), 0.038),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material="arm_paint",
        name=f"{side}_arm_head",
    )
    part.visual(
        Box((0.148, 0.024, 0.010)),
        origin=Origin(
            xyz=(0.084 * cos(arm_yaw), 0.084 * sin(arm_yaw), 0.040),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material="arm_paint",
        name=f"{side}_arm_primary",
    )
    part.visual(
        Box((0.102, 0.020, 0.009)),
        origin=Origin(
            xyz=(0.182 * cos(arm_yaw), 0.182 * sin(arm_yaw), 0.038),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material="arm_paint",
        name=f"{side}_arm_tip",
    )
    part.visual(
        _build_tapered_arm_shell_mesh(f"{side}_arm_shell.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.040), rpy=(0.0, 0.0, arm_yaw)),
        material="arm_paint",
        name=f"{side}_arm_shell",
    )
    part.visual(
        Box((0.118, 0.010, 0.008)),
        origin=Origin(
            xyz=(0.132 * cos(arm_yaw), 0.132 * sin(arm_yaw), 0.045),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material="arm_paint",
        name=f"{side}_arm_brace",
    )
    part.visual(
        Box((0.048, 0.026, 0.012)),
        origin=Origin(xyz=(arm_tip_x, arm_tip_y, 0.040)),
        material="arm_paint",
        name=f"{side}_blade_adapter",
    )
    _add_bolt_head(
        part,
        xyz=(0.0, 0.0, 0.032),
        radius=0.0065,
        height=0.004,
        name=f"{side}_pivot_cap",
    )


def _build_planar_link(part, *, length: float, prefix: str, material: str = "linkage_steel") -> None:
    eye_radius = 0.014 if prefix == "drag" else 0.015
    bar_length = max(length - 0.028, 0.030)
    part.visual(
        Cylinder(radius=eye_radius, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=material,
        name=f"{prefix}_near_eye",
    )
    part.visual(
        Box((bar_length, 0.018, 0.006)),
        origin=Origin(xyz=(eye_radius + bar_length / 2.0, 0.0, 0.003)),
        material=material,
        name=f"{prefix}_bar",
    )
    part.visual(
        Cylinder(radius=eye_radius + 0.001, length=0.006),
        origin=Origin(xyz=(length, 0.0, 0.003)),
        material=material,
        name=f"{prefix}_far_eye",
    )
    part.visual(
        Box((max(bar_length - 0.030, 0.020), 0.010, 0.004)),
        origin=Origin(xyz=(eye_radius + bar_length / 2.0, 0.0, 0.008)),
        material=material,
        name=f"{prefix}_stiffener",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_windshield_wiper_assembly", assets=ASSETS)

    model.material("frame_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("housing_paint", rgba=(0.26, 0.30, 0.28, 1.0))
    model.material("linkage_steel", rgba=(0.36, 0.39, 0.42, 1.0))
    model.material("arm_paint", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("fastener_metal", rgba=(0.62, 0.64, 0.66, 1.0))

    mounting_tray = model.part("mounting_tray")
    mounting_tray.visual(
        Box((0.880, 0.260, 0.008)),
        origin=Origin(xyz=(0.0, -0.050, 0.004)),
        material="frame_steel",
        name="tray_plate",
    )
    mounting_tray.visual(
        Box((0.880, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, -0.176, 0.022)),
        material="frame_steel",
        name="rear_flange",
    )
    mounting_tray.visual(
        Box((0.880, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.076, 0.018)),
        material="frame_steel",
        name="front_flange",
    )
    mounting_tray.visual(
        Box((0.180, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.030, 0.018)),
        material="frame_steel",
        name="center_rib",
    )
    for x in (-0.360, -0.290, 0.290, 0.360):
        _add_bolt_head(
            mounting_tray,
            xyz=(x, -0.050, TRAY_TOP + 0.0015),
            radius=0.0045,
            height=0.003,
            name=f"tray_mount_bolt_{'neg' if x < 0.0 else 'pos'}_{abs(int(x * 1000))}",
        )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Box((0.180, 0.075, 0.012)),
        origin=Origin(xyz=(0.040, -0.134, 0.014)),
        material="frame_steel",
        name="motor_mount_pad",
    )
    motor_housing.visual(
        Cylinder(radius=0.055, length=0.032),
        origin=Origin(xyz=(0.0, -0.120, 0.036)),
        material="housing_paint",
        name="gearbox_cover",
    )
    motor_housing.visual(
        Cylinder(radius=0.034, length=0.160),
        origin=Origin(xyz=(0.120, -0.141, 0.052), rpy=(0.0, pi / 2.0, 0.0)),
        material="housing_paint",
        name="motor_can",
    )
    motor_housing.visual(
        Box((0.095, 0.052, 0.038)),
        origin=Origin(xyz=(0.166, -0.141, 0.052)),
        material="housing_paint",
        name="brush_service_cover",
    )
    motor_housing.visual(
        Box((0.040, 0.028, 0.012)),
        origin=Origin(xyz=(-0.025, -0.140, 0.022)),
        material="housing_paint",
        name="gearbox_neck",
    )
    for dx, dy, tag in (
        (-0.028, -0.020, "nw"),
        (0.028, -0.020, "ne"),
        (-0.028, 0.020, "sw"),
        (0.028, 0.020, "se"),
    ):
        _add_bolt_head(
            motor_housing,
            xyz=(dx, -0.120 + dy, 0.0535),
            radius=0.0045,
            height=0.003,
            name=f"gearbox_fastener_{tag}",
        )

    left_tower = model.part("left_tower")
    _build_pivot_tower(left_tower, x_center=LEFT_PIVOT_X, prefix="left")

    right_tower = model.part("right_tower")
    _build_pivot_tower(right_tower, x_center=RIGHT_PIVOT_X, prefix="right")

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="linkage_steel",
        name="crank_hub",
    )
    motor_crank.visual(
        Box((0.048, 0.022, 0.006)),
        origin=Origin(
            xyz=(0.024 * cos(CRANK_PARK_YAW), 0.024 * sin(CRANK_PARK_YAW), 0.009),
            rpy=(0.0, 0.0, CRANK_PARK_YAW),
        ),
        material="linkage_steel",
        name="crank_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(CRANK_PIN_LOCAL[0], CRANK_PIN_LOCAL[1], 0.009)),
        material="linkage_steel",
        name="crank_eye",
    )
    motor_crank.visual(
        Box((0.026, 0.020, 0.008)),
        origin=Origin(
            xyz=(0.012 * cos(CRANK_PARK_YAW), 0.012 * sin(CRANK_PARK_YAW), 0.012),
            rpy=(0.0, 0.0, CRANK_PARK_YAW),
        ),
        material="linkage_steel",
        name="crank_reinforcement",
    )

    drag_link = model.part("drag_link")
    _build_planar_link(drag_link, length=DRAG_LINK_LENGTH, prefix="drag")

    left_spindle = model.part("left_spindle")
    _build_spindle_module(left_spindle, side="left")

    cross_link = model.part("cross_link")
    _build_planar_link(cross_link, length=CROSS_LINK_LENGTH, prefix="cross")

    right_spindle = model.part("right_spindle")
    _build_spindle_module(right_spindle, side="right")

    left_blade = model.part("left_blade")
    _build_wiper_blade(left_blade, prefix="left")

    right_blade = model.part("right_blade")
    _build_wiper_blade(right_blade, prefix="right")

    model.articulation(
        "tray_to_motor_housing",
        ArticulationType.FIXED,
        parent=mounting_tray,
        child=motor_housing,
    )
    model.articulation(
        "tray_to_left_tower",
        ArticulationType.FIXED,
        parent=mounting_tray,
        child=left_tower,
    )
    model.articulation(
        "tray_to_right_tower",
        ArticulationType.FIXED,
        parent=mounting_tray,
        child=right_tower,
    )
    model.articulation(
        "motor_output",
        ArticulationType.REVOLUTE,
        parent=mounting_tray,
        child=motor_crank,
        origin=Origin(xyz=MOTOR_AXIS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "crank_to_drag_link",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drag_link,
        origin=Origin(xyz=(CRANK_PIN_LOCAL[0], CRANK_PIN_LOCAL[1], 0.012), rpy=(0.0, 0.0, DRAG_LINK_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "left_spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=mounting_tray,
        child=left_spindle,
        origin=Origin(xyz=(LEFT_PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.85, upper=0.55),
    )
    model.articulation(
        "left_pin_to_cross_link",
        ArticulationType.REVOLUTE,
        parent=left_spindle,
        child=cross_link,
        origin=Origin(xyz=(LEFT_PIN_LOCAL[0], LEFT_PIN_LOCAL[1], 0.000), rpy=(0.0, 0.0, CROSS_LINK_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-1.30, upper=1.30),
    )
    model.articulation(
        "right_spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=mounting_tray,
        child=right_spindle,
        origin=Origin(xyz=(RIGHT_PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "left_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=left_spindle,
        child=left_blade,
        origin=Origin(
            xyz=(ARM_RADIUS * cos(LEFT_ARM_YAW), ARM_RADIUS * sin(LEFT_ARM_YAW), 0.040),
            rpy=(0.0, 0.0, pi / 2.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "right_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=right_spindle,
        child=right_blade,
        origin=Origin(
            xyz=(ARM_RADIUS * cos(RIGHT_ARM_YAW), ARM_RADIUS * sin(RIGHT_ARM_YAW), 0.040),
            rpy=(0.0, 0.0, pi / 2.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    def _elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    def _check_xy_stack(
        name: str,
        upper_part: str,
        upper_elem: str,
        lower_part: str,
        lower_elem: str,
        *,
        xy_tol: float,
        min_z_delta: float,
        max_z_delta: float,
    ) -> bool:
        upper = _elem_center(upper_part, upper_elem)
        lower = _elem_center(lower_part, lower_elem)
        if upper is None or lower is None:
            return ctx.fail(name, "missing element center for stacked-linkage check")
        dx = abs(upper[0] - lower[0])
        dy = abs(upper[1] - lower[1])
        dz = upper[2] - lower[2]
        return ctx.check(
            name,
            dx <= xy_tol and dy <= xy_tol and min_z_delta <= dz <= max_z_delta,
            details=f"dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}",
        )

    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mounting_tray = object_model.get_part("mounting_tray")
    motor_housing = object_model.get_part("motor_housing")
    left_tower = object_model.get_part("left_tower")
    right_tower = object_model.get_part("right_tower")
    motor_crank = object_model.get_part("motor_crank")
    drag_link = object_model.get_part("drag_link")
    left_spindle = object_model.get_part("left_spindle")
    cross_link = object_model.get_part("cross_link")
    right_spindle = object_model.get_part("right_spindle")
    left_blade = object_model.get_part("left_blade")
    right_blade = object_model.get_part("right_blade")

    motor_output = object_model.get_articulation("motor_output")
    crank_to_drag = object_model.get_articulation("crank_to_drag_link")
    left_spindle_sweep = object_model.get_articulation("left_spindle_sweep")
    left_pin_to_cross = object_model.get_articulation("left_pin_to_cross_link")
    right_spindle_sweep = object_model.get_articulation("right_spindle_sweep")
    left_blade_pitch = object_model.get_articulation("left_blade_pitch")
    right_blade_pitch = object_model.get_articulation("right_blade_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        motor_housing,
        mounting_tray,
        elem_a="motor_mount_pad",
        elem_b="tray_plate",
        name="motor_mounts_on_tray",
    )
    ctx.expect_contact(
        left_tower,
        mounting_tray,
        elem_a="left_base_pad",
        elem_b="tray_plate",
        name="left_tower_mounts_on_tray",
    )
    ctx.expect_contact(
        right_tower,
        mounting_tray,
        elem_a="right_base_pad",
        elem_b="tray_plate",
        name="right_tower_mounts_on_tray",
    )
    ctx.expect_gap(left_blade, mounting_tray, axis="z", min_gap=0.030, name="left_blade_above_tray")
    ctx.expect_gap(right_blade, mounting_tray, axis="z", min_gap=0.030, name="right_blade_above_tray")
    left_pad_center = _elem_center("left_tower", "left_base_pad")
    right_pad_center = _elem_center("right_tower", "right_base_pad")
    motor_cover_center = _elem_center("motor_housing", "gearbox_cover")
    if left_pad_center is None or right_pad_center is None:
        ctx.fail("pivot_spacing_utility_width", "missing tower pad centers")
    else:
        pivot_dx = right_pad_center[0] - left_pad_center[0]
        ctx.check(
            "pivot_spacing_utility_width",
            0.40 <= pivot_dx <= 0.48,
            details=f"pivot_dx={pivot_dx:.4f}",
        )
    if left_pad_center is None or motor_cover_center is None:
        ctx.fail("motor_near_left_pivot_cluster", "missing motor or pivot reference center")
    else:
        motor_dx = abs(left_pad_center[0] - motor_cover_center[0])
        ctx.check(
            "motor_near_left_pivot_cluster",
            motor_dx <= 0.30,
            details=f"motor_dx={motor_dx:.4f}",
        )

    left_shaft_center = _elem_center("left_spindle", "left_shaft")
    right_shaft_center = _elem_center("right_spindle", "right_shaft")
    if left_pad_center is None or left_shaft_center is None:
        ctx.fail("left_spindle_centered_on_tower", "missing left spindle or tower center")
    else:
        ctx.check(
            "left_spindle_centered_on_tower",
            abs(left_shaft_center[0] - left_pad_center[0]) <= 0.005
            and abs(left_shaft_center[1] - left_pad_center[1]) <= 0.005
            and left_shaft_center[2] > left_pad_center[2],
            details=f"tower={left_pad_center}, shaft={left_shaft_center}",
        )
    if right_pad_center is None or right_shaft_center is None:
        ctx.fail("right_spindle_centered_on_tower", "missing right spindle or tower center")
    else:
        ctx.check(
            "right_spindle_centered_on_tower",
            abs(right_shaft_center[0] - right_pad_center[0]) <= 0.005
            and abs(right_shaft_center[1] - right_pad_center[1]) <= 0.005
            and right_shaft_center[2] > right_pad_center[2],
            details=f"tower={right_pad_center}, shaft={right_shaft_center}",
        )

    ctx.check("motor_output_axis_is_vertical", tuple(motor_output.axis) == (0.0, 0.0, 1.0), details=str(motor_output.axis))
    ctx.check("crank_to_drag_axis_is_vertical", tuple(crank_to_drag.axis) == (0.0, 0.0, 1.0), details=str(crank_to_drag.axis))
    ctx.check("left_spindle_axis_is_vertical", tuple(left_spindle_sweep.axis) == (0.0, 0.0, 1.0), details=str(left_spindle_sweep.axis))
    ctx.check("cross_link_axis_is_vertical", tuple(left_pin_to_cross.axis) == (0.0, 0.0, 1.0), details=str(left_pin_to_cross.axis))
    ctx.check("right_spindle_axis_is_vertical", tuple(right_spindle_sweep.axis) == (0.0, 0.0, 1.0), details=str(right_spindle_sweep.axis))
    ctx.check("left_blade_pitch_axis_is_longitudinal", tuple(left_blade_pitch.axis) == (1.0, 0.0, 0.0), details=str(left_blade_pitch.axis))
    ctx.check("right_blade_pitch_axis_is_longitudinal", tuple(right_blade_pitch.axis) == (1.0, 0.0, 0.0), details=str(right_blade_pitch.axis))

    ctx.expect_contact(
        drag_link,
        motor_crank,
        elem_a="drag_near_eye",
        elem_b="crank_eye",
        name="drag_link_contacts_motor_crank_pin",
    )
    ctx.expect_contact(
        drag_link,
        left_spindle,
        elem_a="drag_far_eye",
        elem_b="left_lever_eye",
        name="drag_link_contacts_left_spindle_pin",
    )
    ctx.expect_contact(
        cross_link,
        left_spindle,
        elem_a="cross_near_eye",
        elem_b="left_lever_eye",
        name="cross_link_contacts_left_spindle_pin",
    )
    ctx.expect_contact(
        cross_link,
        right_spindle,
        elem_a="cross_far_eye",
        elem_b="right_lever_eye",
        name="cross_link_contacts_right_spindle_pin",
    )

    parked_left_blade = ctx.part_world_position(left_blade)
    parked_right_blade = ctx.part_world_position(right_blade)

    with ctx.pose(
        {
            left_spindle_sweep: 0.40,
            right_spindle_sweep: -0.40,
            left_blade_pitch: 0.12,
            right_blade_pitch: -0.12,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="swept_wiper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="swept_wiper_pose_no_floating")
        ctx.expect_gap(left_blade, mounting_tray, axis="z", min_gap=0.025, name="left_blade_clears_tray_in_sweep")
        ctx.expect_gap(right_blade, mounting_tray, axis="z", min_gap=0.025, name="right_blade_clears_tray_in_sweep")
        swept_left_blade = ctx.part_world_position(left_blade)
        swept_right_blade = ctx.part_world_position(right_blade)

    with ctx.pose({motor_output: 0.50, crank_to_drag: 0.32, left_pin_to_cross: 0.18}):
        ctx.fail_if_parts_overlap_in_current_pose(name="service_linkage_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="service_linkage_pose_no_floating")

    with ctx.pose({left_blade_pitch: 0.25, right_blade_pitch: -0.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="blade_pitch_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="blade_pitch_pose_no_floating")

    if parked_left_blade is not None and parked_right_blade is not None and swept_left_blade is not None and swept_right_blade is not None:
        ctx.check(
            "left_blade_sweeps_laterally",
            abs(swept_left_blade[0] - parked_left_blade[0]) >= 0.060,
            details=f"parked={parked_left_blade}, swept={swept_left_blade}",
        )
        ctx.check(
            "right_blade_sweeps_laterally",
            abs(swept_right_blade[0] - parked_right_blade[0]) >= 0.060,
            details=f"parked={parked_right_blade}, swept={swept_right_blade}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
