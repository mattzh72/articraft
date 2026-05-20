from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


BASE_RADIUS = 0.18
BASE_THICKNESS = 0.045
POST_RADIUS = 0.021
POST_TOP_Z = 1.61
COLLAR_RADIUS = 0.0312
COLLAR_CENTER_Z = 1.40
COLLAR_LENGTH = 0.31

RING_MAJOR = 0.039
RING_TUBE = 0.0085
ARM_RADIUS = 0.010
ARM_TIP = (0.60, 0.0, 0.205)
SHADE_AXIS_PITCH = math.radians(18.0)
SHADE_RADIUS = 0.076
SHADE_CENTER_X = 0.124


def _arm_mesh(name: str):
    """A gently rising tubular branch, authored in the branch local frame."""
    arm_path = [
        (0.044, 0.0, 0.000),
        (0.155, 0.0, 0.040),
        (0.365, 0.0, 0.135),
        ARM_TIP,
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            arm_path,
            radius=ARM_RADIUS,
            samples_per_segment=12,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def _globe_shell_mesh(name: str):
    """Thin frosted glass globe with a small neck opening toward the arm."""
    radial_segments = 36
    theta_steps = 22
    shell_thickness = 0.0035
    neck_opening_radius = 0.030
    inner_radius = SHADE_RADIUS - shell_thickness
    opening_half_angle = math.asin(neck_opening_radius / SHADE_RADIUS)
    theta_max = math.pi - opening_half_angle

    geom = MeshGeometry()

    def add_ring(radius: float, theta: float):
        ring = []
        x = SHADE_CENTER_X + radius * math.cos(theta)
        ring_radius = radius * math.sin(theta)
        for seg in range(radial_segments):
            phi = 2.0 * math.pi * seg / radial_segments
            ring.append(
                geom.add_vertex(
                    x,
                    ring_radius * math.cos(phi),
                    ring_radius * math.sin(phi),
                )
            )
        return ring

    outer_pole = geom.add_vertex(SHADE_CENTER_X + SHADE_RADIUS, 0.0, 0.0)
    inner_pole = geom.add_vertex(SHADE_CENTER_X + inner_radius, 0.0, 0.0)
    theta_values = [theta_max * step / theta_steps for step in range(1, theta_steps + 1)]
    outer_rings = [add_ring(SHADE_RADIUS, theta) for theta in theta_values]
    inner_rings = [add_ring(inner_radius, theta) for theta in theta_values]

    for seg in range(radial_segments):
        nxt = (seg + 1) % radial_segments
        geom.add_face(outer_pole, outer_rings[0][seg], outer_rings[0][nxt])
        geom.add_face(inner_pole, inner_rings[0][nxt], inner_rings[0][seg])

    for ring_idx in range(len(outer_rings) - 1):
        outer_a = outer_rings[ring_idx]
        outer_b = outer_rings[ring_idx + 1]
        inner_a = inner_rings[ring_idx]
        inner_b = inner_rings[ring_idx + 1]
        for seg in range(radial_segments):
            nxt = (seg + 1) % radial_segments
            geom.add_face(outer_a[seg], outer_b[seg], outer_b[nxt])
            geom.add_face(outer_a[seg], outer_b[nxt], outer_a[nxt])
            geom.add_face(inner_a[seg], inner_b[nxt], inner_b[seg])
            geom.add_face(inner_a[seg], inner_a[nxt], inner_b[nxt])

    outer_lip = outer_rings[-1]
    inner_lip = inner_rings[-1]
    for seg in range(radial_segments):
        nxt = (seg + 1) % radial_segments
        geom.add_face(outer_lip[seg], inner_lip[seg], inner_lip[nxt])
        geom.add_face(outer_lip[seg], inner_lip[nxt], outer_lip[nxt])

    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_branch_tree_floor_lamp")

    model.material("brushed_bronze", rgba=(0.50, 0.39, 0.20, 1.0))
    model.material("dark_weighted_base", rgba=(0.08, 0.075, 0.065, 1.0))
    model.material("warm_glass", rgba=(1.0, 0.88, 0.58, 0.48))
    model.material("soft_bulb", rgba=(1.0, 0.94, 0.70, 0.72))

    base_post = model.part("base_post")
    base_post.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="dark_weighted_base",
        name="base_disc",
    )
    base_post.visual(
        Cylinder(radius=BASE_RADIUS * 0.83, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.005)),
        material="brushed_bronze",
        name="base_cap",
    )
    base_post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_TOP_Z - BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, (POST_TOP_Z + BASE_THICKNESS) / 2.0)),
        material="brushed_bronze",
        name="post",
    )
    base_post.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_CENTER_Z)),
        material="brushed_bronze",
        name="collar_core",
    )
    base_post.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z + 0.032)),
        material="brushed_bronze",
        name="top_finial",
    )

    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=RING_MAJOR, tube=RING_TUBE, radial_segments=16, tubular_segments=36),
        "swivel_ring",
    )

    branch_heights = [1.28, 1.34, 1.40, 1.46, 1.52]
    branch_yaws = [math.radians(90.0 + 72.0 * idx) for idx in range(5)]

    for idx, (height, yaw) in enumerate(zip(branch_heights, branch_yaws)):
        branch = model.part(f"branch_{idx}")
        branch.visual(
            ring_mesh,
            material="brushed_bronze",
            name="collar_ring",
        )
        branch.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.052, 0.0, 0.0)),
            material="brushed_bronze",
            name="root_knuckle",
        )
        branch.visual(
            _arm_mesh(f"branch_tube_{idx}"),
            material="brushed_bronze",
            name="arm_tube",
        )
        branch.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=ARM_TIP),
            material="brushed_bronze",
            name="tip_ball",
        )

        shade = model.part(f"shade_{idx}")
        shade.visual(
            Cylinder(radius=0.009, length=0.030),
            origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="brushed_bronze",
            name="pivot_socket",
        )
        shade.visual(
            Cylinder(radius=0.011, length=0.060),
            origin=Origin(xyz=(0.057, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_bronze",
            name="neck",
        )
        shade.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="soft_bulb",
            name="bulb_stem",
        )
        shade.visual(
            _globe_shell_mesh(f"globe_shell_{idx}"),
            material="warm_glass",
            name="globe",
        )
        shade.visual(
            Sphere(radius=0.034),
            origin=Origin(xyz=(0.124, 0.0, 0.0)),
            material="soft_bulb",
            name="bulb",
        )
        shade.visual(
            Cylinder(radius=0.031, length=0.010),
            origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_bronze",
            name="shade_cap",
        )
        shade.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.029, tube=0.004, radial_segments=10, tubular_segments=28),
                f"shade_retainer_{idx}",
            ),
            origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_bronze",
            name="retainer_ring",
        )

        model.articulation(
            f"post_to_branch_{idx}",
            ArticulationType.REVOLUTE,
            parent=base_post,
            child=branch,
            origin=Origin(xyz=(0.0, 0.0, height), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.45),
        )
        model.articulation(
            f"branch_to_shade_{idx}",
            ArticulationType.REVOLUTE,
            parent=branch,
            child=shade,
            origin=Origin(xyz=ARM_TIP, rpy=(0.0, -SHADE_AXIS_PITCH, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.70, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    branch_joints = [object_model.get_articulation(f"post_to_branch_{idx}") for idx in range(5)]
    shade_joints = [object_model.get_articulation(f"branch_to_shade_{idx}") for idx in range(5)]
    ctx.check(
        "five independent branch swivels",
        all(joint is not None for joint in branch_joints),
        details="Expected five post-to-branch revolute joints.",
    )
    ctx.check(
        "five independent shade tilts",
        all(joint is not None for joint in shade_joints),
        details="Expected five branch-to-shade revolute joints.",
    )

    base_post = object_model.get_part("base_post")
    for idx in range(5):
        branch = object_model.get_part(f"branch_{idx}")
        shade = object_model.get_part(f"shade_{idx}")
        ctx.allow_overlap(
            base_post,
            branch,
            elem_a="collar_core",
            elem_b="collar_ring",
            reason="The rotating branch ring is modeled with a tiny bearing-seat embed around the post collar.",
        )
        ctx.allow_overlap(
            branch,
            shade,
            elem_a="tip_ball",
            elem_b="pivot_socket",
            reason="The shade socket intentionally captures the branch-tip ball at the revolute pivot.",
        )
        ctx.expect_overlap(
            branch,
            base_post,
            axes="xy",
            min_overlap=0.040,
            elem_a="collar_ring",
            elem_b="collar_core",
            name=f"branch {idx} ring surrounds the post collar",
        )
        ctx.expect_overlap(
            shade,
            branch,
            axes="xyz",
            min_overlap=0.002,
            elem_a="pivot_socket",
            elem_b="tip_ball",
            name=f"shade {idx} socket captures the branch-tip ball",
        )
        ctx.expect_origin_distance(
            shade,
            branch,
            axes="xy",
            min_dist=0.55,
            max_dist=0.64,
            name=f"shade {idx} joint is at the branch tip",
        )

    branch_joint = branch_joints[0]
    shade_joint = shade_joints[0]
    shade = object_model.get_part("shade_0")
    rest_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({branch_joint: 0.35}):
        swept_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({shade_joint: 0.50}):
        tilted_aabb = ctx.part_world_aabb(shade)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(rest_aabb)
    swept_center = _aabb_center(swept_aabb)
    tilted_center = _aabb_center(tilted_aabb)
    ctx.check(
        "branch swivel moves its shade around the post",
        rest_center is not None
        and swept_center is not None
        and math.hypot(swept_center[0] - rest_center[0], swept_center[1] - rest_center[1]) > 0.12,
        details=f"rest={rest_center}, swept={swept_center}",
    )
    ctx.check(
        "shade tilt changes globe aim",
        rest_center is not None
        and tilted_center is not None
        and abs(tilted_center[2] - rest_center[2]) > 0.035,
        details=f"rest={rest_center}, tilted={tilted_center}",
    )

    return ctx.report()


object_model = build_object_model()
