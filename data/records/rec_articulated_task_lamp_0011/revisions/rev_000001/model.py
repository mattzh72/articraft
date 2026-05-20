from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_task_lamp", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.08, 0.08, 0.09, 1.0))
    shade_white = model.material("shade_white", rgba=(0.91, 0.90, 0.85, 1.0))
    diffuser_metal = model.material("diffuser_metal", rgba=(0.82, 0.82, 0.80, 1.0))

    shade_shell_mesh = _save_mesh(
        "task_lamp_shade_shell.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.018, -0.033),
                (0.028, -0.024),
                (0.032, 0.006),
                (0.033, 0.032),
            ],
            inner_profile=[
                (0.015, -0.032),
                (0.025, -0.023),
                (0.029, 0.005),
                (0.030, 0.030),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    front_rim_mesh = _save_mesh(
        "task_lamp_front_rim.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.036, -0.004),
                (0.039, 0.0),
                (0.039, 0.004),
            ],
            inner_profile=[
                (0.032, -0.004),
                (0.035, 0.0),
                (0.035, 0.004),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.016, 0.090, 0.220)),
        origin=Origin(xyz=(0.008, 0.0, 0.170)),
        material=painted_steel,
        name="backplate",
    )
    wall_mount.visual(
        Box((0.018, 0.034, 0.100)),
        origin=Origin(xyz=(0.025, 0.0, 0.220)),
        material=painted_steel,
        name="spine_rib",
    )
    wall_mount.visual(
        Box((0.012, 0.028, 0.016)),
        origin=Origin(xyz=(0.028, 0.0, 0.220)),
        material=hinge_black,
        name="shoulder_bridge",
    )
    wall_mount.visual(
        Box((0.016, 0.006, 0.048)),
        origin=Origin(xyz=(0.042, 0.010, 0.220)),
        material=hinge_black,
        name="shoulder_upper_cheek",
    )
    wall_mount.visual(
        Box((0.016, 0.006, 0.048)),
        origin=Origin(xyz=(0.042, -0.010, 0.220)),
        material=hinge_black,
        name="shoulder_lower_cheek",
    )
    wall_mount.inertial = Inertial.from_geometry(
        Box((0.050, 0.090, 0.220)),
        mass=1.6,
        origin=Origin(xyz=(0.025, 0.0, 0.170)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="rear_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.006, length=0.210),
        origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="beam",
    )
    upper_arm.visual(
        Box((0.018, 0.028, 0.012)),
        origin=Origin(xyz=(0.211, 0.0, 0.0)),
        material=hinge_black,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(0.229, 0.010, 0.0)),
        material=hinge_black,
        name="elbow_upper_cheek",
    )
    upper_arm.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(0.229, -0.010, 0.0)),
        material=hinge_black,
        name="elbow_lower_cheek",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.244, 0.028, 0.038)),
        mass=0.68,
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="rear_hub",
    )
    forearm.visual(
        Cylinder(radius=0.006, length=0.170),
        origin=Origin(xyz=(0.091, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="beam",
    )
    forearm.visual(
        Box((0.020, 0.028, 0.012)),
        origin=Origin(xyz=(0.173, 0.0, 0.0)),
        material=hinge_black,
        name="tip_bridge",
    )
    forearm.visual(
        Box((0.016, 0.006, 0.034)),
        origin=Origin(xyz=(0.191, 0.010, 0.0)),
        material=hinge_black,
        name="pivot_upper_cheek",
    )
    forearm.visual(
        Box((0.016, 0.006, 0.034)),
        origin=Origin(xyz=(0.191, -0.010, 0.0)),
        material=hinge_black,
        name="pivot_lower_cheek",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.205, 0.028, 0.034)),
        mass=0.54,
        origin=Origin(xyz=(0.103, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="pivot_hub",
    )
    shade.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_black,
        name="neck",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.049, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_black,
        name="shade_collar",
    )
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.089, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shade_white,
        name="shade_shell",
    )
    shade.visual(
        front_rim_mesh,
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser_metal,
        name="front_rim",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.135, 0.078, 0.078)),
        mass=0.36,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=upper_arm,
        origin=Origin(xyz=(0.042, 0.0, 0.220)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.229, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-1.05,
            upper=0.20,
        ),
    )
    model.articulation(
        "shade_pivot",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=shade,
        origin=Origin(xyz=(0.191, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-0.85,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    wall_mount = object_model.get_part("wall_mount")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shade = object_model.get_part("shade")
    shoulder_hinge = object_model.get_articulation("shoulder_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    shade_pivot = object_model.get_articulation("shade_pivot")

    shoulder_upper = wall_mount.get_visual("shoulder_upper_cheek")
    shoulder_lower = wall_mount.get_visual("shoulder_lower_cheek")
    upper_rear = upper_arm.get_visual("rear_hub")
    upper_front_upper = upper_arm.get_visual("elbow_upper_cheek")
    upper_front_lower = upper_arm.get_visual("elbow_lower_cheek")
    forearm_rear = forearm.get_visual("rear_hub")
    upper_cheek = forearm.get_visual("pivot_upper_cheek")
    lower_cheek = forearm.get_visual("pivot_lower_cheek")
    neck = shade.get_visual("neck")
    shade_collar = shade.get_visual("shade_collar")
    shade_shell = shade.get_visual("shade_shell")
    front_rim = shade.get_visual("front_rim")
    pivot_hub = shade.get_visual("pivot_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.check(
        "root_is_wall_mount",
        {part.name for part in object_model.root_parts()} == {"wall_mount"},
        details=f"roots={[part.name for part in object_model.root_parts()]}",
    )
    ctx.check(
        "hinges_rotate_about_y",
        tuple(shoulder_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(elbow_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(shade_pivot.axis) == (0.0, 1.0, 0.0),
        details=(
            f"shoulder={shoulder_hinge.axis}, elbow={elbow_hinge.axis}, "
            f"shade={shade_pivot.axis}"
        ),
    )

    ctx.expect_contact(
        upper_arm,
        wall_mount,
        elem_a=upper_rear,
        elem_b=shoulder_upper,
        name="upper_arm_contacts_upper_wall_cheek",
    )
    ctx.expect_contact(
        upper_arm,
        wall_mount,
        elem_a=upper_rear,
        elem_b=shoulder_lower,
        name="upper_arm_contacts_lower_wall_cheek",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="z",
        min_overlap=0.010,
        elem_a=forearm_rear,
        elem_b=upper_front_upper,
        name="forearm_hub_aligned_with_elbow_fork",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=forearm_rear,
        elem_b=upper_front_upper,
        name="forearm_hub_contacts_upper_elbow_cheek",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a=forearm_rear,
        elem_b=upper_front_lower,
        name="forearm_hub_contacts_lower_elbow_cheek",
    )
    ctx.expect_overlap(
        shade,
        forearm,
        axes="z",
        min_overlap=0.010,
        elem_a=pivot_hub,
        elem_b=upper_cheek,
        name="shade_hub_aligned_with_forearm_cheeks",
    )
    ctx.expect_contact(
        shade,
        forearm,
        elem_a=pivot_hub,
        elem_b=upper_cheek,
        name="shade_hub_contacts_upper_cheek",
    )
    ctx.expect_contact(
        shade,
        forearm,
        elem_a=pivot_hub,
        elem_b=lower_cheek,
        name="shade_hub_contacts_lower_cheek",
    )
    ctx.expect_gap(
        shade,
        shade,
        axis="x",
        min_gap=0.112,
        positive_elem=front_rim,
        negative_elem=pivot_hub,
        name="shade_projects_forward_from_pivot",
    )
    ctx.expect_overlap(
        shade,
        shade,
        axes="yz",
        min_overlap=0.004,
        elem_a=shade_collar,
        elem_b=shade_shell,
        name="collar_feeds_cylindrical_shade_body",
    )
    ctx.expect_gap(
        shade,
        forearm,
        axis="x",
        min_gap=0.0015,
        positive_elem=shade_shell,
        negative_elem=forearm.get_visual("tip_bridge"),
        name="shade_shell_clears_tip_bridge",
    )

    with ctx.pose({shoulder_hinge: 0.95, elbow_hinge: -0.95}):
        ctx.expect_gap(
            forearm,
            wall_mount,
            axis="x",
            min_gap=0.015,
            name="folded_forearm_stays_proud_of_wall",
        )
        ctx.expect_gap(
            shade,
            wall_mount,
            axis="x",
            min_gap=0.030,
            name="folded_shade_stays_proud_of_wall",
        )
        ctx.expect_contact(
            shade,
            forearm,
            elem_a=pivot_hub,
            elem_b=upper_cheek,
            name="shade_remains_supported_in_folded_pose",
        )

    def _center_of_aabb(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    rim_rest = ctx.part_element_world_aabb(shade, elem="front_rim")
    ctx.check("front_rim_has_world_aabb", rim_rest is not None, details="front rim AABB missing")

    if rim_rest is not None:
        rim_rest_center = _center_of_aabb(rim_rest)
        with ctx.pose({shade_pivot: shade_pivot.motion_limits.lower}):
            rim_up = ctx.part_element_world_aabb(shade, elem="front_rim")
            ctx.check("shade_pivot_lower_has_aabb", rim_up is not None, details="lower-pose AABB missing")
            if rim_up is not None:
                rim_up_center = _center_of_aabb(rim_up)
                ctx.check(
                    "shade_pivot_lower_aims_up",
                    rim_up_center[2] > rim_rest_center[2] + 0.025,
                    details=f"rest={rim_rest_center}, lower={rim_up_center}",
                )
        with ctx.pose({shade_pivot: shade_pivot.motion_limits.upper}):
            rim_down = ctx.part_element_world_aabb(shade, elem="front_rim")
            ctx.check("shade_pivot_upper_has_aabb", rim_down is not None, details="upper-pose AABB missing")
            if rim_down is not None:
                rim_down_center = _center_of_aabb(rim_down)
                ctx.check(
                    "shade_pivot_upper_aims_down",
                    rim_down_center[2] < rim_rest_center[2] - 0.020,
                    details=f"rest={rim_rest_center}, upper={rim_down_center}",
                )

    with ctx.pose({shade_pivot: -0.55}):
        ctx.expect_contact(
            shade,
            forearm,
            elem_a=pivot_hub,
            elem_b=lower_cheek,
            name="shade_pivot_rotates_about_tip_mount",
        )

    rest_shade_pos = ctx.part_world_position(shade)
    ctx.check("shade_has_rest_position", rest_shade_pos is not None, details="shade world position missing")
    if rest_shade_pos is not None:
        with ctx.pose({shoulder_hinge: shoulder_hinge.motion_limits.upper, elbow_hinge: elbow_hinge.motion_limits.lower}):
            folded_shade_pos = ctx.part_world_position(shade)
            ctx.check(
                "folded_pose_has_shade_position",
                folded_shade_pos is not None,
                details="folded-pose shade world position missing",
            )
            if folded_shade_pos is not None:
                ctx.check(
                    "arm_fold_retracts_and_repositions_shade",
                    folded_shade_pos[0] < rest_shade_pos[0] - 0.10
                    and abs(folded_shade_pos[2] - rest_shade_pos[2]) > 0.15,
                    details=f"rest={rest_shade_pos}, folded={folded_shade_pos}",
                )

    for articulation in (shoulder_hinge, elbow_hinge, shade_pivot):
        limits = articulation.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulation.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulation.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
