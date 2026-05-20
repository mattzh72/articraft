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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _beam_mesh(name: str, *, length: float, width: float, height: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            length,
            cap=True,
            closed=True,
        ).rotate_y(math.pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mass_manufacturable_robotic_arm")

    painted_steel = model.material("painted_steel", rgba=(0.79, 0.34, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.10, 0.11, 0.12, 1.0))

    upper_arm_beam = _beam_mesh(
        "upper_arm_beam",
        length=0.39,
        width=0.14,
        height=0.10,
        radius=0.022,
    )
    forearm_beam = _beam_mesh(
        "forearm_beam",
        length=0.31,
        width=0.12,
        height=0.085,
        radius=0.018,
    )

    base_pedestal = model.part("base_pedestal")
    base_pedestal.visual(
        Box((0.50, 0.46, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="floor_plate",
    )
    base_pedestal.visual(
        Box((0.24, 0.22, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=cast_gray,
        name="column_shell",
    )
    base_pedestal.visual(
        Box((0.16, 0.006, 0.22)),
        origin=Origin(xyz=(0.0, -0.113, 0.24)),
        material=polymer_black,
        name="service_door",
    )
    base_pedestal.visual(
        Box((0.11, 0.08, 0.08)),
        origin=Origin(xyz=(-0.035, -0.07, 0.10)),
        material=polymer_black,
        name="junction_box",
    )
    base_pedestal.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=dark_steel,
        name="slew_base_ring",
    )
    base_pedestal.visual(
        Cylinder(radius=0.20, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=cast_gray,
        name="top_clamp_plate",
    )
    base_pedestal.inertial = Inertial.from_geometry(
        Box((0.50, 0.46, 0.58)),
        mass=88.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    shoulder_module = model.part("shoulder_module")
    shoulder_module.visual(
        Cylinder(radius=0.155, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_steel,
        name="yaw_cartridge",
    )
    shoulder_module.visual(
        Box((0.18, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=cast_gray,
        name="turret_core",
    )
    shoulder_module.visual(
        Box((0.12, 0.12, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, 0.23)),
        material=polymer_black,
        name="yaw_drive_cover",
    )
    shoulder_module.visual(
        Cylinder(radius=0.085, length=0.22),
        origin=Origin(xyz=(0.055, 0.0, 0.31), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_cartridge",
    )
    shoulder_module.visual(
        Box((0.08, 0.03, 0.08)),
        origin=Origin(xyz=(0.06, 0.095, 0.31)),
        material=painted_steel,
        name="shoulder_clamp_left",
    )
    shoulder_module.visual(
        Box((0.08, 0.03, 0.08)),
        origin=Origin(xyz=(0.06, -0.095, 0.31)),
        material=painted_steel,
        name="shoulder_clamp_right",
    )
    shoulder_module.inertial = Inertial.from_geometry(
        Box((0.30, 0.26, 0.40)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.08, 0.18, 0.12)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=painted_steel,
        name="shoulder_clevis",
    )
    upper_arm.visual(
        upper_arm_beam,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=painted_steel,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Box((0.20, 0.16, 0.025)),
        origin=Origin(xyz=(0.30, 0.0, 0.0625)),
        material=cast_gray,
        name="upper_arm_cover",
    )
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.18),
        origin=Origin(xyz=(0.58, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_cartridge",
    )
    upper_arm.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(0.49, 0.0, 0.075)),
        material=polymer_black,
        name="elbow_motor_cover",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.66, 0.20, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.07, 0.16, 0.10)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=painted_steel,
        name="elbow_clevis",
    )
    forearm.visual(
        forearm_beam,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=painted_steel,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.17, 0.13, 0.02)),
        origin=Origin(xyz=(0.28, 0.0, 0.0525)),
        material=cast_gray,
        name="forearm_cover",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.15),
        origin=Origin(xyz=(0.48, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_pitch_cartridge",
    )
    forearm.visual(
        Box((0.09, 0.09, 0.07)),
        origin=Origin(xyz=(0.40, 0.0, -0.06)),
        material=polymer_black,
        name="wrist_pitch_drive_cover",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.56, 0.18, 0.16)),
        mass=10.0,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
    )

    wrist_pitch_module = model.part("wrist_pitch_module")
    wrist_pitch_module.visual(
        Box((0.055, 0.12, 0.08)),
        origin=Origin(xyz=(0.0825, 0.0, 0.0)),
        material=painted_steel,
        name="pitch_clevis",
    )
    wrist_pitch_module.visual(
        Box((0.12, 0.11, 0.08)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=cast_gray,
        name="wrist_body",
    )
    wrist_pitch_module.visual(
        Box((0.06, 0.11, 0.025)),
        origin=Origin(xyz=(0.14, 0.0, 0.0525)),
        material=polymer_black,
        name="wrist_body_cover",
    )
    wrist_pitch_module.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_collar",
    )
    wrist_pitch_module.inertial = Inertial.from_geometry(
        Box((0.28, 0.14, 0.12)),
        mass=5.0,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
    )

    wrist_roll_module = model.part("wrist_roll_module")
    wrist_roll_module.visual(
        Cylinder(radius=0.045, length=0.11),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_output_can",
    )
    wrist_roll_module.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        material=painted_steel,
        name="tool_body",
    )
    wrist_roll_module.visual(
        Cylinder(radius=0.065, length=0.018),
        origin=Origin(xyz=(0.219, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_gray,
        name="tool_flange",
    )
    wrist_roll_module.visual(
        Cylinder(radius=0.025, length=0.02),
        origin=Origin(xyz=(0.238, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="locating_nose",
    )
    wrist_roll_module.inertial = Inertial.from_geometry(
        Box((0.26, 0.14, 0.14)),
        mass=3.0,
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=base_pedestal,
        child=shoulder_module,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_module,
        child=upper_arm,
        origin=Origin(xyz=(0.055, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=-1.45,
            upper=1.35,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=95.0,
            velocity=1.5,
            lower=-2.3,
            upper=2.3,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch_module,
        origin=Origin(xyz=(0.48, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=48.0,
            velocity=2.2,
            lower=-2.0,
            upper=2.0,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent=wrist_pitch_module,
        child=wrist_roll_module,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_pedestal = object_model.get_part("base_pedestal")
    shoulder_module = object_model.get_part("shoulder_module")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch_module = object_model.get_part("wrist_pitch_module")
    wrist_roll_module = object_model.get_part("wrist_roll_module")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")
    wrist_roll = object_model.get_articulation("wrist_roll")

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
        "axis_order_is_readable",
        base_yaw.axis == (0.0, 0.0, 1.0)
        and shoulder_pitch.axis == (0.0, -1.0, 0.0)
        and elbow_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        details="Expected yaw -> shoulder pitch -> elbow pitch -> wrist pitch -> wrist roll axis sequence.",
    )

    with ctx.pose(
        {
            base_yaw: 0.0,
            shoulder_pitch: 0.0,
            elbow_pitch: 0.0,
            wrist_pitch: 0.0,
            wrist_roll: 0.0,
        }
    ):
        ctx.expect_gap(
            shoulder_module,
            base_pedestal,
            axis="z",
            positive_elem="yaw_cartridge",
            negative_elem="top_clamp_plate",
            max_gap=0.0015,
            max_penetration=0.0,
            name="yaw_cartridge_seats_on_pedestal",
        )
        ctx.expect_overlap(
            shoulder_module,
            base_pedestal,
            axes="xy",
            elem_a="yaw_cartridge",
            elem_b="top_clamp_plate",
            min_overlap=0.22,
            name="yaw_stack_has_shared_footprint",
        )
        ctx.expect_gap(
            upper_arm,
            shoulder_module,
            axis="x",
            positive_elem="shoulder_clevis",
            negative_elem="shoulder_cartridge",
            max_gap=0.0015,
            max_penetration=1e-5,
            name="upper_arm_clamps_to_shoulder_cartridge",
        )
        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="x",
            positive_elem="elbow_clevis",
            negative_elem="elbow_cartridge",
            max_gap=0.0015,
            max_penetration=0.0,
            name="forearm_clamps_to_elbow_cartridge",
        )
        ctx.expect_gap(
            wrist_pitch_module,
            forearm,
            axis="x",
            positive_elem="pitch_clevis",
            negative_elem="wrist_pitch_cartridge",
            max_gap=0.0015,
            max_penetration=0.0,
            name="wrist_pitch_module_clamps_to_forearm_cartridge",
        )
        ctx.expect_gap(
            wrist_roll_module,
            wrist_pitch_module,
            axis="x",
            positive_elem="roll_output_can",
            negative_elem="roll_collar",
            max_gap=0.0015,
            max_penetration=0.0,
            name="roll_module_seats_in_roll_collar",
        )

        forearm_home = ctx.part_world_position(forearm)
        wrist_home = ctx.part_world_position(wrist_roll_module)
        upper_home_aabb = ctx.part_world_aabb(upper_arm)

    with ctx.pose({shoulder_pitch: 0.7, elbow_pitch: 0.95, wrist_pitch: 0.5}):
        forearm_lifted = ctx.part_world_position(forearm)
        wrist_lifted = ctx.part_world_position(wrist_roll_module)
        upper_lifted_aabb = ctx.part_world_aabb(upper_arm)
        ctx.check(
            "positive_pitch_pose_lifts_chain",
            forearm_home is not None
            and wrist_home is not None
            and forearm_lifted is not None
            and wrist_lifted is not None
            and upper_home_aabb is not None
            and upper_lifted_aabb is not None
            and upper_lifted_aabb[1][2] > upper_home_aabb[1][2] + 0.22
            and forearm_lifted[2] > forearm_home[2] + 0.20
            and wrist_lifted[2] > wrist_home[2] + 0.22,
            details="Positive pitch motion should clearly raise the outboard chain.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
