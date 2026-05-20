from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(
    z: float,
    size_x: float,
    size_y: float,
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + cx, y + cy, z) for x, y in rounded_rect_profile(size_x, size_y, radius)]


def _yz_section(
    x: float,
    size_y: float,
    size_z: float,
    radius: float,
    *,
    cy: float = 0.0,
    cz: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + cy, z + cz) for y, z in rounded_rect_profile(size_y, size_z, radius)]


def _z_loft_mesh(name: str, specs: list[tuple[float, float, float, float, float, float]]):
    sections = [_xy_section(z, sx, sy, r, cx=cx, cy=cy) for z, sx, sy, r, cx, cy in specs]
    return _save_mesh(name, section_loft(sections))


def _x_loft_mesh(name: str, specs: list[tuple[float, float, float, float, float, float]]):
    sections = [_yz_section(x, sy, sz, r, cy=cy, cz=cz) for x, sy, sz, r, cy, cz in specs]
    return _save_mesh(name, section_loft(sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_arm", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.36, 0.38, 0.41, 1.0))
    satin_titanium = model.material("satin_titanium", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_composite = model.material("dark_composite", rgba=(0.12, 0.13, 0.14, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.46, 0.48, 0.50, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.28, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=matte_graphite,
        name="foot_plate",
    )
    base.visual(
        Box((0.13, 0.16, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.029)),
        material=dark_composite,
        name="upper_pad",
    )
    base.visual(
        _z_loft_mesh(
            "base_pedestal.obj",
            [
                (0.026, 0.22, 0.18, 0.040, 0.0, 0.0),
                (0.090, 0.19, 0.155, 0.036, 0.0, 0.0),
                (0.160, 0.16, 0.135, 0.030, 0.0, 0.0),
            ],
        ),
        material=satin_graphite,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.074, 0.094, 0.046)),
        origin=Origin(xyz=(-0.070, 0.0, 0.053)),
        material=dark_composite,
        name="service_cover",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=satin_titanium,
        name="slew_ring",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(0.118 * x_sign, 0.092 * y_sign, 0.028)),
                material=hardware_steel,
                name=f"anchor_bolt_{'n' if x_sign < 0 else 'p'}x_{'n' if y_sign < 0 else 'p'}y",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.28, 0.18)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.067, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_titanium,
        name="slew_ring",
    )
    turret.visual(
        _x_loft_mesh(
            "turret_body.obj",
            [
                (-0.070, 0.118, 0.096, 0.024, 0.0, 0.056),
                (0.000, 0.180, 0.170, 0.036, 0.0, 0.078),
                (0.046, 0.132, 0.134, 0.024, 0.0, 0.084),
            ],
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=satin_graphite,
        name="body_shell",
    )
    turret.visual(
        _x_loft_mesh(
            "turret_drive.obj",
            [
                (-0.045, 0.088, 0.066, 0.016, 0.0, 0.146),
                (0.000, 0.102, 0.080, 0.018, 0.0, 0.154),
                (0.048, 0.082, 0.068, 0.015, 0.0, 0.150),
            ],
        ),
        material=dark_composite,
        name="shoulder_drive",
    )
    turret.visual(
        Box((0.052, 0.024, 0.070)),
        origin=Origin(xyz=(0.066, -0.052, 0.118)),
        material=matte_graphite,
        name="shoulder_left_support",
    )
    turret.visual(
        Box((0.052, 0.024, 0.070)),
        origin=Origin(xyz=(0.066, 0.052, 0.118)),
        material=matte_graphite,
        name="shoulder_right_support",
    )
    turret.visual(
        Box((0.040, 0.088, 0.082)),
        origin=Origin(xyz=(-0.036, 0.0, 0.111)),
        material=matte_graphite,
        name="counterweight",
    )
    turret.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.095, -0.049, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="shoulder_left_collar",
    )
    turret.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.095, 0.049, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="shoulder_right_collar",
    )
    turret.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.095, -0.064, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="shoulder_left_axis_cap",
    )
    turret.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.095, 0.064, 0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="shoulder_right_axis_cap",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.18, 0.20, 0.18)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="shoulder_left_seal",
    )
    upper_arm.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="shoulder_right_seal",
    )
    upper_arm.visual(
        _x_loft_mesh(
            "upper_arm_shell.obj",
            [
                (0.060, 0.074, 0.084, 0.018, 0.0, -0.002),
                (0.135, 0.074, 0.086, 0.018, 0.0, -0.003),
                (0.238, 0.084, 0.072, 0.014, 0.0, -0.004),
            ],
        ),
        material=satin_graphite,
        name="link_shell",
    )
    upper_arm.visual(
        Box((0.056, 0.016, 0.054)),
        origin=Origin(xyz=(0.060, -0.018, 0.0)),
        material=matte_graphite,
        name="shoulder_left_bridge",
    )
    upper_arm.visual(
        Box((0.056, 0.016, 0.054)),
        origin=Origin(xyz=(0.060, 0.018, 0.0)),
        material=matte_graphite,
        name="shoulder_right_bridge",
    )
    upper_arm.visual(
        _x_loft_mesh(
            "upper_arm_actuator.obj",
            [
                (0.096, 0.050, 0.052, 0.012, 0.0, -0.046),
                (0.162, 0.058, 0.060, 0.014, 0.0, -0.040),
                (0.212, 0.048, 0.050, 0.011, 0.0, -0.038),
            ],
        ),
        material=dark_composite,
        name="actuator_housing",
    )
    upper_arm.visual(
        Box((0.108, 0.064, 0.004)),
        origin=Origin(xyz=(0.148, 0.0, 0.037)),
        material=satin_titanium,
        name="top_break",
    )
    upper_arm.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.265, -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="elbow_left_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.265, 0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="elbow_right_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.265, -0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="elbow_left_axis_cap",
    )
    upper_arm.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.265, 0.053, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="elbow_right_axis_cap",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.10, 0.12)),
        mass=7.5,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.025, length=0.056),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="elbow_hub",
    )
    forearm.visual(
        _x_loft_mesh(
            "forearm_shell.obj",
            [
                (0.022, 0.064, 0.074, 0.016, 0.0, 0.000),
                (0.118, 0.056, 0.068, 0.014, 0.0, 0.001),
                (0.194, 0.050, 0.058, 0.012, 0.0, 0.002),
            ],
        ),
        material=satin_graphite,
        name="link_shell",
    )
    forearm.visual(
        _x_loft_mesh(
            "forearm_actuator.obj",
            [
                (0.060, 0.046, 0.046, 0.011, 0.0, 0.030),
                (0.122, 0.052, 0.054, 0.012, 0.0, 0.034),
                (0.170, 0.044, 0.044, 0.010, 0.0, 0.028),
            ],
        ),
        material=dark_composite,
        name="actuator_housing",
    )
    forearm.visual(
        Box((0.084, 0.046, 0.004)),
        origin=Origin(xyz=(0.118, 0.0, 0.032)),
        material=satin_titanium,
        name="top_break",
    )
    forearm.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.215, -0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="wrist_left_collar",
    )
    forearm.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.215, 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_titanium,
        name="wrist_right_collar",
    )
    forearm.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.215, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="wrist_left_axis_cap",
    )
    forearm.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.215, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="wrist_right_axis_cap",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.24, 0.08, 0.09)),
        mass=5.0,
        origin=Origin(xyz=(0.12, 0.0, 0.01)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.021, length=0.046),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="wrist_hub",
    )
    wrist.visual(
        _x_loft_mesh(
            "wrist_shell.obj",
            [
                (0.012, 0.050, 0.050, 0.012, 0.0, 0.000),
                (0.050, 0.044, 0.044, 0.010, 0.0, 0.000),
                (0.088, 0.034, 0.032, 0.008, 0.0, 0.000),
            ],
        ),
        material=satin_titanium,
        name="body_shell",
    )
    wrist.visual(
        Box((0.040, 0.036, 0.004)),
        origin=Origin(xyz=(0.045, 0.0, 0.022)),
        material=dark_composite,
        name="top_break",
    )
    wrist.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_titanium,
        name="flange_mount",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.10, 0.06, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
    )

    tool_flange = model.part("tool_flange")
    tool_flange.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_titanium,
        name="flange_collar",
    )
    tool_flange.visual(
        Box((0.010, 0.052, 0.052)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=matte_graphite,
        name="mount_plate",
    )
    tool_flange.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.026, -0.016, 0.0)),
        material=soft_black,
        name="left_fixture",
    )
    tool_flange.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.026, 0.016, 0.0)),
        material=soft_black,
        name="right_fixture",
    )
    tool_flange.inertial = Inertial.from_geometry(
        Box((0.04, 0.06, 0.06)),
        mass=0.8,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
    )

    model.articulation(
        "base_slew",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.6,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.095, 0.0, 0.118)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.4,
            lower=-0.15,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.7,
            lower=-1.15,
            upper=1.20,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=2.2,
            lower=-1.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "wrist_to_tool_flange",
        ArticulationType.FIXED,
        parent=wrist,
        child=tool_flange,
        origin=Origin(xyz=(0.089, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    expected_parts = (
        "base",
        "turret",
        "upper_arm",
        "forearm",
        "wrist",
        "tool_flange",
    )
    expected_joints = (
        "base_slew",
        "shoulder_pitch",
        "elbow_pitch",
        "wrist_pitch",
        "wrist_to_tool_flange",
    )
    parts_by_name = {part.name: part for part in object_model.parts}
    joints_by_name = {joint.name: joint for joint in object_model.articulations}

    for part_name in expected_parts:
        ctx.check(f"part_{part_name}_present", part_name in parts_by_name, f"Missing part: {part_name}")
    for joint_name in expected_joints:
        ctx.check(f"joint_{joint_name}_present", joint_name in joints_by_name, f"Missing joint: {joint_name}")

    base = parts_by_name.get("base")
    turret = parts_by_name.get("turret")
    upper_arm = parts_by_name.get("upper_arm")
    forearm = parts_by_name.get("forearm")
    wrist = parts_by_name.get("wrist")
    tool_flange = parts_by_name.get("tool_flange")

    base_slew = joints_by_name.get("base_slew")
    shoulder_pitch = joints_by_name.get("shoulder_pitch")
    elbow_pitch = joints_by_name.get("elbow_pitch")
    wrist_pitch = joints_by_name.get("wrist_pitch")

    def require_visual(part, visual_name: str):
        if part is None:
            return None
        try:
            visual = part.get_visual(visual_name)
        except Exception as exc:  # pragma: no cover - defensive recording path
            ctx.fail(f"visual_{part.name}_{visual_name}_present", f"{exc}")
            return None
        ctx.check(
            f"visual_{part.name}_{visual_name}_present",
            visual is not None,
            f"Missing visual {visual_name} on {part.name}",
        )
        return visual

    base_ring = require_visual(base, "slew_ring")
    turret_ring = require_visual(turret, "slew_ring")
    shoulder_left = require_visual(turret, "shoulder_left_collar")
    shoulder_right = require_visual(turret, "shoulder_right_collar")
    upper_hub = require_visual(upper_arm, "shoulder_hub")
    elbow_left = require_visual(upper_arm, "elbow_left_collar")
    elbow_right = require_visual(upper_arm, "elbow_right_collar")
    forearm_hub = require_visual(forearm, "elbow_hub")
    wrist_left = require_visual(forearm, "wrist_left_collar")
    wrist_right = require_visual(forearm, "wrist_right_collar")
    wrist_hub = require_visual(wrist, "wrist_hub")
    wrist_mount = require_visual(wrist, "flange_mount")
    flange_collar = require_visual(tool_flange, "flange_collar")

    if all(part is not None for part in (base, turret, upper_arm, forearm, wrist, tool_flange)):
        ctx.expect_contact(
            turret,
            base,
            elem_a=turret_ring,
            elem_b=base_ring,
            name="turret_bearing_contact",
        )
        ctx.expect_overlap(
            turret,
            base,
            axes="xy",
            elem_a=turret_ring,
            elem_b=base_ring,
            min_overlap=0.13,
            name="turret_bearing_xy_overlap",
        )
        ctx.expect_within(
            turret,
            base,
            axes="xy",
            inner_elem=turret_ring,
            outer_elem=base_ring,
            margin=0.008,
            name="turret_ring_within_base_ring",
        )

        ctx.expect_gap(
            upper_arm,
            turret,
            axis="y",
            positive_elem=upper_hub,
            negative_elem=shoulder_left,
            max_gap=0.0005,
            max_penetration=0.0,
            name="shoulder_left_face_seat",
        )
        ctx.expect_gap(
            turret,
            upper_arm,
            axis="y",
            positive_elem=shoulder_right,
            negative_elem=upper_hub,
            max_gap=0.0005,
            max_penetration=0.0,
            name="shoulder_right_face_seat",
        )
        ctx.expect_overlap(
            upper_arm,
            turret,
            axes="xz",
            elem_a=upper_hub,
            elem_b=shoulder_left,
            min_overlap=0.055,
            name="shoulder_axis_registration",
        )

        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="y",
            positive_elem=forearm_hub,
            negative_elem=elbow_left,
            max_gap=0.0005,
            max_penetration=0.0,
            name="elbow_left_face_seat",
        )
        ctx.expect_gap(
            upper_arm,
            forearm,
            axis="y",
            positive_elem=elbow_right,
            negative_elem=forearm_hub,
            max_gap=0.0005,
            max_penetration=0.0,
            name="elbow_right_face_seat",
        )
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="xz",
            elem_a=forearm_hub,
            elem_b=elbow_left,
            min_overlap=0.045,
            name="elbow_axis_registration",
        )

        ctx.expect_gap(
            wrist,
            forearm,
            axis="y",
            positive_elem=wrist_hub,
            negative_elem=wrist_left,
            max_gap=0.0005,
            max_penetration=0.0,
            name="wrist_left_face_seat",
        )
        ctx.expect_gap(
            forearm,
            wrist,
            axis="y",
            positive_elem=wrist_right,
            negative_elem=wrist_hub,
            max_gap=0.0005,
            max_penetration=0.0,
            name="wrist_right_face_seat",
        )
        ctx.expect_overlap(
            wrist,
            forearm,
            axes="xz",
            elem_a=wrist_hub,
            elem_b=wrist_left,
            min_overlap=0.038,
            name="wrist_axis_registration",
        )

        ctx.expect_contact(
            tool_flange,
            wrist,
            elem_a=flange_collar,
            elem_b=wrist_mount,
            name="tool_flange_mount_contact",
        )
        ctx.expect_overlap(
            tool_flange,
            wrist,
            axes="yz",
            elem_a=flange_collar,
            elem_b=wrist_mount,
            min_overlap=0.036,
            name="tool_flange_mount_registration",
        )

        base_aabb = ctx.part_world_aabb(base)
        tool_rest = ctx.part_world_position(tool_flange)
        if base_aabb is not None:
            base_dx = base_aabb[1][0] - base_aabb[0][0]
            base_dy = base_aabb[1][1] - base_aabb[0][1]
            base_dz = base_aabb[1][2] - base_aabb[0][2]
            ctx.check(
                "base_has_stable_footprint",
                base_dx >= 0.33 and base_dy >= 0.27 and base_dz <= 0.19,
                f"Unexpected base dimensions: {(base_dx, base_dy, base_dz)}",
            )
        else:
            ctx.fail("base_has_stable_footprint", "Base AABB unavailable")

        if tool_rest is not None:
            ctx.check(
                "rest_pose_reach_balanced",
                0.62 <= tool_rest[0] <= 0.68 and abs(tool_rest[1]) <= 0.01 and 0.27 <= tool_rest[2] <= 0.31,
                f"Unexpected rest tool position: {tool_rest}",
            )
        else:
            ctx.fail("rest_pose_reach_balanced", "Tool flange world position unavailable")

        if tool_rest is not None and base_slew is not None:
            with ctx.pose({base_slew: 0.70}):
                swung = ctx.part_world_position(tool_flange)
                ctx.check(
                    "base_slew_moves_planar_reach",
                    swung is not None
                    and abs(swung[1]) >= 0.38
                    and swung[0] <= tool_rest[0] - 0.14
                    and abs(swung[2] - tool_rest[2]) <= 0.02,
                    f"Unexpected slew pose: {swung}",
                )

        if tool_rest is not None and shoulder_pitch is not None:
            with ctx.pose({shoulder_pitch: 0.68}):
                lifted = ctx.part_world_position(tool_flange)
                ctx.check(
                    "shoulder_pitch_lifts_arm",
                    lifted is not None
                    and lifted[2] >= tool_rest[2] + 0.32
                    and lifted[0] <= tool_rest[0] - 0.10,
                    f"Unexpected shoulder pose: {lifted}",
                )
                ctx.expect_gap(
                    tool_flange,
                    base,
                    axis="z",
                    min_gap=0.25,
                    name="raised_tool_clears_base",
                )

        if tool_rest is not None and elbow_pitch is not None:
            with ctx.pose({elbow_pitch: 0.95}):
                bent = ctx.part_world_position(tool_flange)
                ctx.check(
                    "elbow_pitch_folds_forearm",
                    bent is not None
                    and bent[2] >= tool_rest[2] + 0.17
                    and bent[0] <= tool_rest[0] - 0.09,
                    f"Unexpected elbow pose: {bent}",
                )

        if tool_rest is not None and wrist_pitch is not None:
            with ctx.pose({wrist_pitch: 0.80}):
                reoriented = ctx.part_world_position(tool_flange)
                ctx.check(
                    "wrist_pitch_reorients_tool",
                    reoriented is not None
                    and reoriented[2] >= tool_rest[2] + 0.05
                    and reoriented[0] <= tool_rest[0] - 0.02,
                    f"Unexpected wrist pose: {reoriented}",
                )

        if shoulder_pitch is not None and elbow_pitch is not None and wrist_pitch is not None:
            with ctx.pose({shoulder_pitch: 0.52, elbow_pitch: 0.88, wrist_pitch: -0.42}):
                ctx.expect_gap(
                    tool_flange,
                    base,
                    axis="z",
                    min_gap=0.14,
                    name="folded_pose_tool_clearance",
                )
                ctx.expect_gap(
                    wrist,
                    base,
                    axis="z",
                    min_gap=0.16,
                    name="folded_pose_wrist_clearance",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
