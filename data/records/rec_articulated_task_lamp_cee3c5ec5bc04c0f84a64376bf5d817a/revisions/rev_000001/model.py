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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jewelers_bench_lamp")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    ivory_enamel = model.material("ivory_enamel", rgba=(0.88, 0.88, 0.84, 1.0))
    warm_white = model.material("warm_white", rgba=(0.96, 0.97, 0.90, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.90, 0.98, 0.35))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))

    lens_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.060, tube=0.007, radial_segments=18, tubular_segments=54),
        "lens_bezel",
    )
    fluorescent_tube_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.047, tube=0.005, radial_segments=16, tubular_segments=48),
        "fluorescent_tube",
    )

    base = model.part("base")
    base.visual(
        Box((0.026, 0.064, 0.190)),
        origin=Origin(xyz=(-0.030, 0.0, 0.095)),
        material=painted_steel,
        name="clamp_spine",
    )
    base.visual(
        Box((0.090, 0.064, 0.014)),
        origin=Origin(xyz=(0.002, 0.0, 0.173)),
        material=painted_steel,
        name="upper_jaw",
    )
    base.visual(
        Box((0.038, 0.064, 0.060)),
        origin=Origin(xyz=(-0.012, 0.0, 0.145)),
        material=painted_steel,
        name="jaw_gusset",
    )
    base.visual(
        Box((0.066, 0.054, 0.020)),
        origin=Origin(xyz=(-0.002, 0.0, 0.018)),
        material=painted_steel,
        name="lower_frame",
    )
    base.visual(
        Box((0.030, 0.040, 0.038)),
        origin=Origin(xyz=(0.010, 0.0, 0.045)),
        material=painted_steel,
        name="screw_boss",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.154),
        origin=Origin(xyz=(0.010, 0.0, 0.083)),
        material=satin_aluminum,
        name="clamp_screw",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.010, 0.0, 0.158)),
        material=satin_aluminum,
        name="pressure_pad",
    )
    base.visual(
        Cylinder(radius=0.0045, length=0.086),
        origin=Origin(xyz=(0.010, 0.0, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="handle_bar",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.190),
        origin=Origin(xyz=(-0.010, 0.0, 0.265)),
        material=painted_steel,
        name="post_socket",
    )
    base.visual(
        Cylinder(radius=0.0125, length=0.190),
        origin=Origin(xyz=(-0.010, 0.0, 0.325)),
        material=satin_aluminum,
        name="main_post",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(-0.010, 0.0, 0.415)),
        material=painted_steel,
        name="top_collar",
    )
    base.visual(
        Box((0.028, 0.014, 0.060)),
        origin=Origin(xyz=(-0.010, 0.025, 0.452)),
        material=painted_steel,
        name="left_clevis_ear",
    )
    base.visual(
        Box((0.028, 0.014, 0.060)),
        origin=Origin(xyz=(-0.010, -0.025, 0.452)),
        material=painted_steel,
        name="right_clevis_ear",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.500)),
        mass=2.7,
        origin=Origin(xyz=(-0.004, 0.0, 0.250)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="shoulder_hub",
    )
    lower_arm.visual(
        Box((0.268, 0.010, 0.012)),
        origin=Origin(xyz=(0.134, 0.013, 0.0)),
        material=satin_aluminum,
        name="main_bar_left",
    )
    lower_arm.visual(
        Box((0.268, 0.010, 0.012)),
        origin=Origin(xyz=(0.134, -0.013, 0.0)),
        material=satin_aluminum,
        name="main_bar_right",
    )
    lower_arm.visual(
        Box((0.042, 0.036, 0.010)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=painted_steel,
        name="mid_brace",
    )
    lower_arm.visual(
        Box((0.038, 0.036, 0.010)),
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        material=painted_steel,
        name="rear_brace",
    )
    lower_arm.visual(
        Box((0.034, 0.010, 0.050)),
        origin=Origin(xyz=(0.283, 0.021, 0.0)),
        material=painted_steel,
        name="end_fork_left",
    )
    lower_arm.visual(
        Box((0.034, 0.010, 0.050)),
        origin=Origin(xyz=(0.283, -0.021, 0.0)),
        material=painted_steel,
        name="end_fork_right",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.315, 0.070, 0.060)),
        mass=0.8,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.242, 0.010, 0.012)),
        origin=Origin(xyz=(0.121, 0.013, 0.0)),
        material=satin_aluminum,
        name="main_bar_left",
    )
    upper_arm.visual(
        Box((0.242, 0.010, 0.012)),
        origin=Origin(xyz=(0.121, -0.013, 0.0)),
        material=satin_aluminum,
        name="main_bar_right",
    )
    upper_arm.visual(
        Box((0.036, 0.034, 0.010)),
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        material=painted_steel,
        name="mid_brace",
    )
    upper_arm.visual(
        Box((0.034, 0.034, 0.010)),
        origin=Origin(xyz=(0.184, 0.0, 0.0)),
        material=painted_steel,
        name="rear_brace",
    )
    upper_arm.visual(
        Box((0.030, 0.010, 0.046)),
        origin=Origin(xyz=(0.255, 0.019, 0.0)),
        material=painted_steel,
        name="end_fork_left",
    )
    upper_arm.visual(
        Box((0.030, 0.010, 0.046)),
        origin=Origin(xyz=(0.255, -0.019, 0.0)),
        material=painted_steel,
        name="end_fork_right",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.286, 0.066, 0.056)),
        mass=0.65,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="tilt_hub",
    )
    lamp_head.visual(
        Box((0.050, 0.018, 0.018)),
        origin=Origin(xyz=(0.025, 0.0, -0.004)),
        material=painted_steel,
        name="neck_arm",
    )
    lamp_head.visual(
        Box((0.024, 0.050, 0.034)),
        origin=Origin(xyz=(0.032, 0.0, -0.010)),
        material=ivory_enamel,
        name="neck_block",
    )
    lamp_head.visual(
        Box((0.040, 0.108, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, -0.004)),
        material=ivory_enamel,
        name="top_bridge",
    )
    lamp_head.visual(
        lens_bezel_mesh,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=ivory_enamel,
        name="lens_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=lens_glass,
        name="magnifier_lens",
    )
    lamp_head.visual(
        fluorescent_tube_mesh,
        origin=Origin(xyz=(0.085, 0.0, -0.018)),
        material=warm_white,
        name="fluorescent_tube",
    )
    lamp_head.visual(
        Box((0.082, 0.104, 0.018)),
        origin=Origin(xyz=(0.074, 0.0, -0.030)),
        material=ivory_enamel,
        name="lamp_tray",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.170, 0.140, 0.080)),
        mass=0.95,
        origin=Origin(xyz=(0.085, 0.0, -0.018)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(-0.010, 0.0, 0.444)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.20,
        ),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.283, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lamp_head,
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.85,
            upper=0.75,
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

    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lamp_head = object_model.get_part("lamp_head")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    head_tilt = object_model.get_articulation("upper_arm_to_head")

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "all primary parts exist",
        all(part is not None for part in (base, lower_arm, upper_arm, lamp_head)),
        details="Expected base, lower arm, upper arm, and lamp head parts.",
    )
    ctx.check(
        "all primary articulations exist",
        all(joint is not None for joint in (shoulder, elbow, head_tilt)),
        details="Expected shoulder, elbow, and head tilt articulations.",
    )

    lens_aabb = ctx.part_element_world_aabb(lamp_head, elem="lens_bezel")
    tube_aabb = ctx.part_element_world_aabb(lamp_head, elem="fluorescent_tube")
    lens_above_tube = (
        lens_aabb is not None
        and tube_aabb is not None
        and lens_aabb[0][2] > tube_aabb[1][2] + 0.002
    )
    ctx.check(
        "magnifier ring sits above fluorescent tube",
        lens_above_tube,
        details=f"lens_aabb={lens_aabb}, tube_aabb={tube_aabb}",
    )

    lower_rest = _center_z(ctx.part_element_world_aabb(lower_arm, elem="main_bar_left"))
    with ctx.pose({shoulder: 0.75}):
        lower_raised = _center_z(ctx.part_element_world_aabb(lower_arm, elem="main_bar_left"))
    ctx.check(
        "shoulder joint lifts first arm with positive rotation",
        lower_rest is not None and lower_raised is not None and lower_raised > lower_rest + 0.05,
        details=f"rest_z={lower_rest}, raised_z={lower_raised}",
    )

    with ctx.pose({shoulder: 0.45, elbow: 0.0}):
        upper_rest = _center_z(ctx.part_element_world_aabb(upper_arm, elem="main_bar_left"))
    with ctx.pose({shoulder: 0.45, elbow: 0.70}):
        upper_raised = _center_z(ctx.part_element_world_aabb(upper_arm, elem="main_bar_left"))
    ctx.check(
        "elbow joint lifts second arm with positive rotation",
        upper_rest is not None and upper_raised is not None and upper_raised > upper_rest + 0.04,
        details=f"rest_z={upper_rest}, raised_z={upper_raised}",
    )

    with ctx.pose({shoulder: 0.40, elbow: 0.30, head_tilt: 0.0}):
        head_rest = _center_z(ctx.part_element_world_aabb(lamp_head, elem="lens_bezel"))
    with ctx.pose({shoulder: 0.40, elbow: 0.30, head_tilt: 0.45}):
        head_raised = _center_z(ctx.part_element_world_aabb(lamp_head, elem="lens_bezel"))
    ctx.check(
        "head tilt raises lens ring with positive rotation",
        head_rest is not None and head_raised is not None and head_raised > head_rest + 0.01,
        details=f"rest_z={head_rest}, raised_z={head_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
