from __future__ import annotations

from math import pi

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

LOWER_ARM_LENGTH = 0.34
UPPER_ARM_LENGTH = 0.32


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y - (length / 2.0), z))


def _tapered_beam(
    *,
    start_x: float,
    end_x: float,
    start_size: tuple[float, float],
    end_size: tuple[float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .rect(*start_size)
        .workplane(offset=end_x - start_x)
        .rect(*end_size)
        .loft(combine=True)
    )


def _x_ring(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    x_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((x_center - (thickness / 2.0), 0.0, 0.0))
    )


def _make_base_shape() -> cq.Workplane:
    spine = cq.Workplane("XY").box(0.022, 0.078, 0.074).translate((-0.019, 0.0, -0.047))
    upper_jaw = cq.Workplane("XY").box(0.050, 0.078, 0.014).translate((0.004, 0.0, -0.015))
    lower_jaw = cq.Workplane("XY").box(0.038, 0.060, 0.014).translate((-0.002, 0.0, -0.078))
    screw = cq.Workplane("XY").circle(0.006).extrude(0.056).translate((0.012, 0.0, -0.078))
    pressure_pad = cq.Workplane("XY").circle(0.015).extrude(0.004).translate((0.012, 0.0, -0.082))
    handle = _y_cylinder(0.004, 0.050, (0.012, 0.0, -0.103))
    swivel_flange = cq.Workplane("XY").circle(0.026).extrude(0.010).translate((0.0, 0.0, -0.010))
    swivel_neck = cq.Workplane("XY").circle(0.016).extrude(0.014).translate((0.0, 0.0, -0.014))
    lock_knob = _y_cylinder(0.005, 0.060, (-0.012, 0.0, -0.003))
    return (
        spine.union(upper_jaw)
        .union(lower_jaw)
        .union(screw)
        .union(pressure_pad)
        .union(handle)
        .union(swivel_flange)
        .union(swivel_neck)
        .union(lock_knob)
    )


def _make_lower_arm_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").circle(0.024).extrude(0.014)
    turret_cap = cq.Workplane("XY").circle(0.017).extrude(0.010).translate((0.0, 0.0, 0.014))
    beam = _tapered_beam(
        start_x=0.030,
        end_x=LOWER_ARM_LENGTH - 0.038,
        start_size=(0.028, 0.016),
        end_size=(0.022, 0.012),
    )
    cable_channel = cq.Workplane("XY").box(0.065, 0.012, 0.008).translate((0.170, 0.0, -0.011))
    clevis_block = (
        cq.Workplane("XY").box(0.022, 0.036, 0.020).translate((LOWER_ARM_LENGTH - 0.010, 0.0, 0.0))
    )
    lug_left = _y_cylinder(0.020, 0.008, (LOWER_ARM_LENGTH, 0.012, 0.0))
    lug_right = _y_cylinder(0.020, 0.008, (LOWER_ARM_LENGTH, -0.012, 0.0))
    return (
        base_plate.union(turret_cap)
        .union(beam)
        .union(cable_channel)
        .union(clevis_block)
        .union(lug_left)
        .union(lug_right)
    )


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _y_cylinder(0.020, 0.017, (0.0, 0.0, 0.0))
    shoulder_block = cq.Workplane("XY").box(0.024, 0.024, 0.018).translate((0.012, 0.0, 0.0))
    beam = _tapered_beam(
        start_x=0.028,
        end_x=UPPER_ARM_LENGTH - 0.032,
        start_size=(0.024, 0.014),
        end_size=(0.019, 0.011),
    )
    cable_channel = cq.Workplane("XY").box(0.055, 0.010, 0.007).translate((0.175, 0.0, -0.009))
    clevis_block = (
        cq.Workplane("XY").box(0.018, 0.032, 0.018).translate((UPPER_ARM_LENGTH - 0.009, 0.0, 0.0))
    )
    lug_left = _y_cylinder(0.018, 0.008, (UPPER_ARM_LENGTH, 0.011, 0.0))
    lug_right = _y_cylinder(0.018, 0.008, (UPPER_ARM_LENGTH, -0.011, 0.0))
    return (
        shoulder_hub.union(shoulder_block)
        .union(beam)
        .union(cable_channel)
        .union(clevis_block)
        .union(lug_left)
        .union(lug_right)
    )


def _make_microphone_mount_shape() -> cq.Workplane:
    pivot_hub = _y_cylinder(0.018, 0.0148, (0.0, 0.0, 0.0))
    bracket = cq.Workplane("XY").box(0.028, 0.016, 0.020).translate((0.014, 0.0, 0.0))
    bridge = cq.Workplane("XY").box(0.014, 0.014, 0.016).translate((0.032, 0.0, 0.0))
    rear_collar = cq.Workplane("YZ").circle(0.018).extrude(0.018).translate((0.028, 0.0, 0.0))
    shock_ring = _x_ring(0.030, 0.024, 0.004, 0.041)
    return pivot_hub.union(bracket).union(bridge).union(rear_collar).union(shock_ring)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_mounted_microphone_boom", assets=ASSETS)

    model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("satin_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("dark_polymer", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("mic_body", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("charcoal_grille", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_clamp.obj", assets=ASSETS),
        material="matte_black",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.070, 0.080, 0.110)),
        mass=1.6,
        origin=Origin(xyz=(-0.006, 0.0, -0.048)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_make_lower_arm_shape(), "lower_arm.obj", assets=ASSETS),
        material="satin_silver",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.355, 0.040, 0.045)),
        mass=0.45,
        origin=Origin(xyz=(0.175, 0.0, 0.006)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_shape(), "upper_arm.obj", assets=ASSETS),
        material="satin_silver",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.330, 0.036, 0.040)),
        mass=0.36,
        origin=Origin(xyz=(0.165, 0.0, 0.004)),
    )

    microphone = model.part("microphone")
    microphone.visual(
        mesh_from_cadquery(_make_microphone_mount_shape(), "microphone_mount.obj", assets=ASSETS),
        material="dark_polymer",
    )
    microphone.visual(
        Cylinder(radius=0.021, length=0.082),
        origin=Origin(xyz=(0.084, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="mic_body",
    )
    microphone.visual(
        Cylinder(radius=0.0185, length=0.028),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="charcoal_grille",
    )
    microphone.visual(
        Sphere(radius=0.0185),
        origin=Origin(xyz=(0.153, 0.0, 0.0)),
        material="charcoal_grille",
    )
    microphone.inertial = Inertial.from_geometry(
        Box((0.170, 0.050, 0.050)),
        mass=0.24,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.75, upper=1.75, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.15, effort=8.0, velocity=2.5),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=microphone,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=0.75, effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("base", "lower_arm")
    ctx.expect_aabb_contact("lower_arm", "upper_arm")
    ctx.expect_aabb_contact("upper_arm", "microphone")
    ctx.expect_aabb_overlap("base", "lower_arm", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("lower_arm", "upper_arm", axes="yz", min_overlap=0.015)
    ctx.expect_aabb_overlap("upper_arm", "microphone", axes="yz", min_overlap=0.014)
    ctx.expect_origin_gap("microphone", "base", axis="x", min_gap=0.60)

    ctx.expect_joint_motion_axis(
        "base_yaw",
        "lower_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "microphone",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(base_yaw=1.20):
        ctx.expect_aabb_contact("base", "lower_arm")
        ctx.expect_origin_gap("microphone", "base", axis="y", min_gap=0.45)

    with ctx.pose(elbow_pitch=1.00):
        ctx.expect_aabb_contact("lower_arm", "upper_arm")
        ctx.expect_origin_gap("microphone", "base", axis="z", min_gap=0.22)

    with ctx.pose(elbow_pitch=0.85, wrist_pitch=0.60):
        ctx.expect_aabb_contact("upper_arm", "microphone")
        ctx.expect_origin_gap("microphone", "base", axis="z", min_gap=0.18)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
