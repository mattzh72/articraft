from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_ground_corner_floodlight")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    housing_black = model.material("lamp_housing", rgba=(0.18, 0.19, 0.21, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.75, 0.82, 0.90, 0.55))

    azimuth_ring_geom = BoxGeometry((0.012, 0.112, 0.060)).translate(-0.050, 0.0, 0.0)
    azimuth_ring_geom.merge(BoxGeometry((0.042, 0.012, 0.060)).translate(-0.029, 0.050, 0.0))
    azimuth_ring_geom.merge(BoxGeometry((0.042, 0.012, 0.060)).translate(-0.029, -0.050, 0.0))
    azimuth_ring_mesh = mesh_from_geometry(azimuth_ring_geom, "floodlight_azimuth_ring")

    mast = model.part("mast")

    pole_height = 16.0
    pole_radius = 0.18
    arm_radius = 0.085
    arm_length = 1.90
    arm_z = 15.74
    hinge_x = 1.86
    hinge_z = 15.58

    mast.visual(
        Cylinder(radius=pole_radius, length=pole_height),
        origin=Origin(xyz=(0.0, 0.0, pole_height * 0.5)),
        material=galvanized,
        name="pole_shaft",
    )
    mast.visual(
        Cylinder(radius=0.28, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=galvanized,
        name="base_flange",
    )
    mast.visual(
        Box((0.015, 0.18, 0.70)),
        origin=Origin(xyz=(pole_radius - 0.0075, 0.0, 1.25)),
        material=painted_steel,
        name="access_door",
    )
    mast.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(xyz=(arm_length * 0.5, 0.0, arm_z), rpy=(0.0, 1.57079632679, 0.0)),
        material=galvanized,
        name="outreach_arm",
    )

    brace_origin, brace_length = _cylinder_origin_between(
        (0.0, 0.0, arm_z - 0.62),
        (1.22, 0.0, arm_z - 0.06),
    )
    mast.visual(
        Cylinder(radius=0.05, length=brace_length),
        origin=brace_origin,
        material=galvanized,
        name="arm_brace",
    )
    mast.visual(
        Box((0.22, 0.16, 0.12)),
        origin=Origin(xyz=(1.73, 0.0, hinge_z + 0.10)),
        material=galvanized,
        name="arm_end_block",
    )
    mast.visual(
        Box((0.12, 0.016, 0.24)),
        origin=Origin(xyz=(hinge_x, 0.065, hinge_z)),
        material=galvanized,
        name="clevis_plate_left",
    )
    mast.visual(
        Box((0.12, 0.016, 0.24)),
        origin=Origin(xyz=(hinge_x, -0.065, hinge_z)),
        material=galvanized,
        name="clevis_plate_right",
    )

    cluster = model.part("cluster_frame")
    cluster.visual(
        Cylinder(radius=0.028, length=0.114),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    cluster.visual(
        Box((0.09, 0.10, 0.18)),
        origin=Origin(xyz=(-0.035, 0.0, -0.09)),
        material=galvanized,
        name="hinge_neck",
    )
    cluster.visual(
        Box((0.05, 0.08, 0.70)),
        origin=Origin(xyz=(-0.03, 0.0, -0.44)),
        material=galvanized,
        name="center_spine",
    )
    cluster.visual(
        Box((0.05, 0.08, 0.70)),
        origin=Origin(xyz=(-0.03, 0.28, -0.44)),
        material=galvanized,
        name="left_rail",
    )
    cluster.visual(
        Box((0.05, 0.08, 0.70)),
        origin=Origin(xyz=(-0.03, -0.28, -0.44)),
        material=galvanized,
        name="right_rail",
    )
    cluster.visual(
        Box((0.05, 0.24, 0.05)),
        origin=Origin(xyz=(-0.03, 0.20, -0.20)),
        material=galvanized,
        name="upper_left_crossbar",
    )
    cluster.visual(
        Box((0.05, 0.24, 0.05)),
        origin=Origin(xyz=(-0.03, -0.20, -0.20)),
        material=galvanized,
        name="upper_right_crossbar",
    )
    cluster.visual(
        Box((0.05, 0.64, 0.05)),
        origin=Origin(xyz=(-0.03, 0.0, -0.405)),
        material=galvanized,
        name="mid_rail",
    )
    cluster.visual(
        Box((0.05, 0.64, 0.06)),
        origin=Origin(xyz=(-0.03, 0.0, -0.67)),
        material=galvanized,
        name="bottom_rail",
    )

    lamp_mounts = {
        "lamp_upper_left": (-0.112, 0.28, -0.24, 0.12),
        "lamp_upper_right": (-0.112, -0.28, -0.24, -0.12),
        "lamp_lower_left": (-0.112, 0.28, -0.59, 0.06),
        "lamp_lower_right": (-0.112, -0.28, -0.59, -0.06),
    }

    for lamp_name, (mx, my, mz, _) in lamp_mounts.items():
        mount_id = lamp_name.replace("lamp_", "")
        cluster.visual(
            Cylinder(radius=0.012, length=0.060),
            origin=Origin(
                xyz=(mx + 0.030, my, mz),
                rpy=(0.0, 1.57079632679, 0.0),
            ),
            material=galvanized,
            name=f"mount_stalk_{mount_id}",
        )
        cluster.visual(
            Cylinder(radius=0.032, length=0.072),
            origin=Origin(xyz=(mx, my, mz)),
            material=painted_steel,
            name=f"mount_spindle_{mount_id}",
        )
    model.articulation(
        "mast_to_cluster_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=cluster,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(0.0, 0.42, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.7,
            lower=-0.18,
            upper=0.40,
        ),
    )

    for lamp_name, (mx, my, mz, _) in lamp_mounts.items():
        lamp = model.part(lamp_name)
        lamp.visual(
            azimuth_ring_mesh,
            origin=Origin(),
            material=painted_steel,
            name="azimuth_ring",
        )
        lamp.visual(
            Box((0.050, 0.040, 0.060)),
            origin=Origin(xyz=(-0.075, 0.0, 0.0)),
            material=painted_steel,
            name="ring_web",
        )
        lamp.visual(
            Cylinder(radius=0.007, length=0.072),
            origin=Origin(xyz=(-0.039, 0.0, 0.0)),
            material=painted_steel,
            name="azimuth_contact_roller",
        )
        lamp.visual(
            Box((0.020, 0.032, 0.072)),
            origin=Origin(xyz=(-0.053, 0.0, 0.0)),
            material=painted_steel,
            name="azimuth_bridge",
        )
        lamp.visual(
            Box((0.036, 0.018, 0.080)),
            origin=Origin(xyz=(-0.096, 0.046, 0.0)),
            material=painted_steel,
            name="side_yoke_left",
        )
        lamp.visual(
            Box((0.036, 0.018, 0.080)),
            origin=Origin(xyz=(-0.096, -0.046, 0.0)),
            material=painted_steel,
            name="side_yoke_right",
        )
        lamp.visual(
            Box((0.040, 0.090, 0.060)),
            origin=Origin(xyz=(-0.108, 0.0, 0.0)),
            material=painted_steel,
            name="rear_knuckle",
        )
        lamp.visual(
            Box((0.18, 0.24, 0.14)),
            origin=Origin(xyz=(-0.190, 0.0, 0.0)),
            material=housing_black,
            name="lamp_body",
        )
        lamp.visual(
            Box((0.055, 0.16, 0.18)),
            origin=Origin(xyz=(-0.116, 0.0, 0.0)),
            material=painted_steel,
            name="rear_heat_sink",
        )
        lamp.visual(
            Box((0.018, 0.26, 0.18)),
            origin=Origin(xyz=(-0.289, 0.0, 0.0)),
            material=painted_steel,
            name="front_bezel",
        )
        lamp.visual(
            Box((0.006, 0.22, 0.16)),
            origin=Origin(xyz=(-0.301, 0.0, 0.0)),
            material=lens_glass,
            name="front_glass",
        )
        lamp.visual(
            Box((0.045, 0.24, 0.02)),
            origin=Origin(xyz=(-0.264, 0.0, 0.090)),
            material=painted_steel,
            name="visor",
        )

        model.articulation(
            f"cluster_to_{lamp_name}",
            ArticulationType.REVOLUTE,
            parent=cluster,
            child=lamp,
            origin=Origin(xyz=(mx, my, mz)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=1.2,
                lower=-0.55,
                upper=0.55,
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

    mast = object_model.get_part("mast")
    cluster = object_model.get_part("cluster_frame")
    tilt_joint = object_model.get_articulation("mast_to_cluster_tilt")

    lamp_names = (
        "lamp_upper_left",
        "lamp_upper_right",
        "lamp_lower_left",
        "lamp_lower_right",
    )

    for name in ("mast", "cluster_frame", *lamp_names):
        ctx.check(f"{name} exists", object_model.get_part(name) is not None)

    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "cluster tilt hinge is a pitched revolute mount",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.axis == (0.0, -1.0, 0.0)
        and tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None,
        details=f"axis={tilt_joint.axis}, limits={tilt_limits}",
    )

    for lamp_name in lamp_names:
        lamp_joint = object_model.get_articulation(f"cluster_to_{lamp_name}")
        joint_limits = lamp_joint.motion_limits
        ctx.check(
            f"{lamp_name} has azimuth adjustment",
            lamp_joint.articulation_type == ArticulationType.REVOLUTE
            and lamp_joint.axis == (0.0, 0.0, 1.0)
            and joint_limits is not None
            and joint_limits.lower is not None
            and joint_limits.upper is not None,
            details=f"axis={lamp_joint.axis}, limits={joint_limits}",
        )

    for lamp_name in lamp_names:
        mount_id = lamp_name.replace("lamp_", "")
        ctx.expect_contact(
            lamp_name,
            cluster,
            elem_a="azimuth_contact_roller",
            elem_b=f"mount_spindle_{mount_id}",
            name=f"{lamp_name} azimuth collar bears on its mount spindle",
        )
        ctx.expect_overlap(
            lamp_name,
            cluster,
            axes="yz",
            elem_a="azimuth_ring",
            elem_b=f"mount_spindle_{mount_id}",
            min_overlap=0.05,
            name=f"{lamp_name} azimuth ring stays centered around its mount spindle",
        )

    tilt_down_aabb = None
    tilt_up_aabb = None
    with ctx.pose({tilt_joint: -0.18}):
        tilt_down_aabb = ctx.part_element_world_aabb(cluster, elem="mount_spindle_lower_left")
    with ctx.pose({tilt_joint: 0.40}):
        tilt_up_aabb = ctx.part_element_world_aabb(cluster, elem="mount_spindle_lower_left")

    tilt_down_center = _aabb_center(tilt_down_aabb)
    tilt_up_center = _aabb_center(tilt_up_aabb)
    ctx.check(
        "positive tilt pulls the lower lamp row back toward the pole",
        tilt_down_center is not None
        and tilt_up_center is not None
        and tilt_up_center[0] > tilt_down_center[0] + 0.12,
        details=f"tilt_down_center={tilt_down_center}, tilt_up_center={tilt_up_center}",
    )

    sample_lamp = object_model.get_part("lamp_upper_left")
    sample_joint = object_model.get_articulation("cluster_to_lamp_upper_left")
    azimuth_left_aabb = None
    azimuth_right_aabb = None
    with ctx.pose({sample_joint: -0.45}):
        azimuth_left_aabb = ctx.part_element_world_aabb(sample_lamp, elem="lamp_body")
    with ctx.pose({sample_joint: 0.45}):
        azimuth_right_aabb = ctx.part_element_world_aabb(sample_lamp, elem="lamp_body")

    azimuth_left_center = _aabb_center(azimuth_left_aabb)
    azimuth_right_center = _aabb_center(azimuth_right_aabb)
    ctx.check(
        "lamp azimuth joint swings the housing laterally",
        azimuth_left_center is not None
        and azimuth_right_center is not None
        and abs(azimuth_right_center[1] - azimuth_left_center[1]) > 0.12,
        details=f"left={azimuth_left_center}, right={azimuth_right_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
