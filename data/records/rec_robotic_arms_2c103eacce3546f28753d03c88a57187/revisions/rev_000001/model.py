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
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_place_robotic_arm")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.57, 0.60, 0.64, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.92, 0.48, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.15, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))

    upper_arm_shell = _mesh(
        "upper_arm_shell",
        sweep_profile_along_spline(
            [
                (0.08, 0.0, 0.11),
                (0.20, 0.0, 0.16),
                (0.40, 0.0, 0.18),
                (0.56, 0.0, 0.16),
            ],
            profile=rounded_rect_profile(0.17, 0.12, radius=0.028, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )
    forearm_shell = _mesh(
        "forearm_shell",
        sweep_profile_along_spline(
            [
                (0.10, 0.0, 0.00),
                (0.26, 0.0, 0.00),
                (0.44, 0.0, 0.00),
                (0.58, 0.0, 0.00),
            ],
            profile=rounded_rect_profile(0.13, 0.10, radius=0.020, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=pedestal_gray,
        name="floor_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.14, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=pedestal_gray,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=steel,
        name="top_mount_flange",
    )
    pedestal_base.visual(
        Box((0.18, 0.22, 0.16)),
        origin=Origin(xyz=(-0.05, -0.12, 0.17)),
        material=dark_metal,
        name="service_box",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.39)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Cylinder(radius=0.19, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=steel,
        name="lower_mount_flange",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.145, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=housing_gray,
        name="main_bearing_housing",
    )
    shoulder_housing.visual(
        Box((0.34, 0.24, 0.11)),
        origin=Origin(xyz=(0.05, 0.0, 0.20)),
        material=housing_gray,
        name="drive_gearbox",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.165, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=steel,
        name="upper_bearing_cap",
    )
    shoulder_housing.visual(
        Box((0.10, 0.16, 0.10)),
        origin=Origin(xyz=(-0.13, 0.0, 0.17)),
        material=dark_metal,
        name="encoder_pod",
    )
    shoulder_housing.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.28)),
        mass=34.0,
        origin=Origin(xyz=(0.02, 0.0, 0.14)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.155, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel,
        name="shoulder_rotor_flange",
    )
    upper_arm.visual(
        Cylinder(radius=0.13, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_metal,
        name="shoulder_rotor_drum",
    )
    upper_arm.visual(
        upper_arm_shell,
        material=arm_orange,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.08, 0.18, 0.10)),
        origin=Origin(xyz=(0.50, 0.0, 0.16)),
        material=arm_orange,
        name="elbow_mount_block",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.18)),
        origin=Origin(xyz=(0.62, 0.074, 0.16)),
        material=dark_metal,
        name="elbow_cheek_left",
    )
    upper_arm.visual(
        Box((0.12, 0.04, 0.18)),
        origin=Origin(xyz=(0.62, -0.074, 0.16)),
        material=dark_metal,
        name="elbow_cheek_right",
    )
    upper_arm.visual(
        Box((0.10, 0.188, 0.04)),
        origin=Origin(xyz=(0.60, 0.0, 0.24)),
        material=steel,
        name="elbow_bridge_top",
    )
    upper_arm.visual(
        Box((0.06, 0.188, 0.08)),
        origin=Origin(xyz=(0.53, 0.0, 0.16)),
        material=steel,
        name="elbow_bridge_rear",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.72, 0.22, 0.28)),
        mass=26.0,
        origin=Origin(xyz=(0.33, 0.0, 0.13)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.042, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_trunnion",
    )
    forearm.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_gearcase",
    )
    forearm.visual(
        forearm_shell,
        material=arm_orange,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.064, length=0.018),
        origin=Origin(xyz=(0.565, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_mount_flange",
    )
    forearm.visual(
        Cylinder(radius=0.050, length=0.066),
        origin=Origin(xyz=(0.589, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="wrist_neck_tube",
    )
    forearm.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(xyz=(0.611, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="wrist_support_ring",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.68, 0.18, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.31, 0.0, 0.0)),
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_rotor_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.044, length=0.060),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="wrist_rotor_drum",
    )
    wrist_head.visual(
        Box((0.12, 0.11, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=housing_gray,
        name="wrist_housing",
    )
    wrist_head.visual(
        Box((0.08, 0.06, 0.06)),
        origin=Origin(xyz=(0.11, 0.0, 0.075)),
        material=dark_metal,
        name="wrist_cable_pod",
    )
    wrist_head.visual(
        Cylinder(radius=0.042, length=0.016),
        origin=Origin(xyz=(0.178, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.05, 0.09, 0.04)),
        origin=Origin(xyz=(0.155, 0.0, -0.055)),
        material=dark_metal,
        name="tool_interface_block",
    )
    wrist_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.18)),
        mass=8.0,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_shoulder",
        ArticulationType.FIXED,
        parent=pedestal_base,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
    )
    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.261)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=2.0,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.62, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=2.0,
            lower=-0.15,
            upper=2.2,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.622, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=3.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal_base")
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder_joint = object_model.get_articulation("shoulder_yaw")
    elbow_joint = object_model.get_articulation("elbow_pitch")
    wrist_joint = object_model.get_articulation("wrist_roll")

    ctx.expect_contact(
        shoulder_housing,
        pedestal,
        elem_a="lower_mount_flange",
        elem_b="top_mount_flange",
        name="shoulder housing bolts to pedestal flange",
    )
    ctx.expect_overlap(
        shoulder_housing,
        pedestal,
        axes="xy",
        elem_a="lower_mount_flange",
        elem_b="top_mount_flange",
        min_overlap=0.30,
        name="pedestal and shoulder flanges share a broad footprint",
    )
    ctx.expect_contact(
        upper_arm,
        shoulder_housing,
        elem_a="shoulder_rotor_flange",
        elem_b="upper_bearing_cap",
        name="upper arm shoulder rotor seats on housing cap",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_cheek_left",
        elem_b="elbow_trunnion",
        name="left elbow cheek supports forearm trunnion",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_cheek_right",
        elem_b="elbow_trunnion",
        name="right elbow cheek supports forearm trunnion",
    )
    ctx.expect_contact(
        forearm,
        wrist_head,
        elem_a="wrist_support_ring",
        elem_b="wrist_rotor_flange",
        name="wrist head mounts against forearm support ring",
    )

    rest_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder_joint: 1.2}):
        swung_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings arm in plan while holding height",
        rest_wrist_pos is not None
        and swung_wrist_pos is not None
        and abs(rest_wrist_pos[2] - swung_wrist_pos[2]) < 1e-6
        and math.hypot(
            swung_wrist_pos[0] - rest_wrist_pos[0],
            swung_wrist_pos[1] - rest_wrist_pos[1],
        )
        > 0.25,
        details=f"rest={rest_wrist_pos}, swung={swung_wrist_pos}",
    )

    with ctx.pose({elbow_joint: 0.0}):
        straight_wrist_pos = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_joint: 1.0}):
        bent_wrist_pos = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch lifts wrist head upward",
        straight_wrist_pos is not None
        and bent_wrist_pos is not None
        and bent_wrist_pos[2] > straight_wrist_pos[2] + 0.20,
        details=f"straight={straight_wrist_pos}, bent={bent_wrist_pos}",
    )

    wrist_origin_rest = ctx.part_world_position(wrist_head)
    pod_rest = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="wrist_cable_pod"))
    with ctx.pose({wrist_joint: math.pi / 2.0}):
        wrist_origin_rolled = ctx.part_world_position(wrist_head)
        pod_rolled = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="wrist_cable_pod"))
    ctx.check(
        "wrist roll spins off-axis cable pod around forearm axis",
        wrist_origin_rest is not None
        and wrist_origin_rolled is not None
        and pod_rest is not None
        and pod_rolled is not None
        and abs(wrist_origin_rest[0] - wrist_origin_rolled[0]) < 1e-6
        and abs(wrist_origin_rest[1] - wrist_origin_rolled[1]) < 1e-6
        and abs(wrist_origin_rest[2] - wrist_origin_rolled[2]) < 1e-6
        and abs(pod_rest[1] - pod_rolled[1]) > 0.04
        and abs(pod_rest[2] - pod_rolled[2]) > 0.04,
        details=f"origin_rest={wrist_origin_rest}, origin_rolled={wrist_origin_rolled}, pod_rest={pod_rest}, pod_rolled={pod_rolled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
