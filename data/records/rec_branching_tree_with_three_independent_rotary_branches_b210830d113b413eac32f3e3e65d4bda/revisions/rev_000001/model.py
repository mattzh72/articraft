from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


MAST_RADIUS = 0.024
MAST_HEIGHT = 0.320
BASE_RADIUS = 0.045
BASE_HEIGHT = 0.052
TOP_CAP_RADIUS = 0.031
TOP_CAP_HEIGHT = 0.016

HUB_HEIGHT = 0.040
HUB_BLOCK_THICKNESS = 0.030
HUB_BLOCK_WIDTH = 0.064
HUB_BLOCK_CENTER_X = MAST_RADIUS + HUB_BLOCK_THICKNESS / 2.0
PIVOT_BOSS_RADIUS = 0.015
PIVOT_BOSS_LENGTH = 0.052
PIVOT_X = 0.061
ARM_ROOT_LENGTH = 0.030
ARM_ROOT_WIDTH = 0.028
ARM_ROOT_HEIGHT = 0.026
ARM_BEAM_WIDTH = 0.020
ARM_BEAM_HEIGHT = 0.018
ARM_BEAM_START_X = PIVOT_BOSS_RADIUS + ARM_ROOT_LENGTH
PAD_THICKNESS = 0.010
PAD_WIDTH = 0.042
PAD_HEIGHT = 0.030


def make_pad_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        PAD_THICKNESS, PAD_WIDTH, PAD_HEIGHT, centered=(True, True, True)
    )
    pad = (
        pad.faces(">X")
        .workplane()
        .pushPoints([(0.0, -0.010), (0.0, 0.010)])
        .hole(0.006)
    )
    return pad


BRANCH_CONFIGS = (
    {
        "name": "lower",
        "hub_name": "hub_lower",
        "arm_name": "arm_lower",
        "mount_name": "mast_to_hub_lower",
        "joint_name": "hub_lower_to_arm_lower",
        "height": 0.108,
        "yaw": 0.0,
        "pad_center_x": 0.165,
        "limits": (-0.75, 0.85),
    },
    {
        "name": "middle",
        "hub_name": "hub_middle",
        "arm_name": "arm_middle",
        "mount_name": "mast_to_hub_middle",
        "joint_name": "hub_middle_to_arm_middle",
        "height": 0.188,
        "yaw": math.radians(122.0),
        "pad_center_x": 0.152,
        "limits": (-0.70, 0.90),
    },
    {
        "name": "upper",
        "hub_name": "hub_upper",
        "arm_name": "arm_upper",
        "mount_name": "mast_to_hub_upper",
        "joint_name": "hub_upper_to_arm_upper",
        "height": 0.268,
        "yaw": math.radians(246.0),
        "pad_center_x": 0.170,
        "limits": (-0.80, 0.78),
    },
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_fixture_head")

    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.48, 0.50, 0.53, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=dark_steel,
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=MAST_RADIUS, length=MAST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + MAST_HEIGHT / 2.0)),
        material=machine_gray,
        name="mast_shaft",
    )
    mast.visual(
        Cylinder(radius=TOP_CAP_RADIUS, length=TOP_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + MAST_HEIGHT + TOP_CAP_HEIGHT / 2.0)),
        material=dark_steel,
        name="top_cap",
    )
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT + MAST_HEIGHT + TOP_CAP_HEIGHT),
        mass=5.6,
        origin=Origin(
            xyz=(0.0, 0.0, (BASE_HEIGHT + MAST_HEIGHT + TOP_CAP_HEIGHT) / 2.0)
        ),
    )

    pad_shape = make_pad_shape()

    for branch in BRANCH_CONFIGS:
        hub = model.part(branch["hub_name"])
        hub.visual(
            Box((HUB_BLOCK_THICKNESS, HUB_BLOCK_WIDTH, HUB_HEIGHT)),
            origin=Origin(xyz=(HUB_BLOCK_CENTER_X, 0.0, 0.0)),
            material=machine_gray,
            name="hub_body",
        )
        hub.visual(
            Cylinder(radius=PIVOT_BOSS_RADIUS, length=PIVOT_BOSS_LENGTH),
            origin=Origin(xyz=(PIVOT_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_boss",
        )
        hub.inertial = Inertial.from_geometry(
            Box((0.080, 0.070, 0.050)),
            mass=0.9,
            origin=Origin(xyz=(0.040, 0.0, 0.0)),
        )

        model.articulation(
            branch["mount_name"],
            ArticulationType.FIXED,
            parent=mast,
            child=hub,
            origin=Origin(xyz=(0.0, 0.0, branch["height"]), rpy=(0.0, 0.0, branch["yaw"])),
        )

        arm = model.part(branch["arm_name"])
        arm.visual(
            Box((ARM_ROOT_LENGTH, ARM_ROOT_WIDTH, ARM_ROOT_HEIGHT)),
            origin=Origin(
                xyz=(PIVOT_BOSS_RADIUS + ARM_ROOT_LENGTH / 2.0, 0.0, 0.0)
            ),
            material=aluminum,
            name="root_lug",
        )
        arm.visual(
            Box(
                (
                    branch["pad_center_x"] - PAD_THICKNESS / 2.0 - ARM_BEAM_START_X,
                    ARM_BEAM_WIDTH,
                    ARM_BEAM_HEIGHT,
                )
            ),
            origin=Origin(
                xyz=(
                    (
                        ARM_BEAM_START_X
                        + branch["pad_center_x"]
                        - PAD_THICKNESS / 2.0
                    )
                    / 2.0,
                    0.0,
                    0.0,
                )
            ),
            material=aluminum,
            name="arm_body",
        )
        arm.visual(
            mesh_from_cadquery(pad_shape, f"{branch['name']}_mount_pad"),
            origin=Origin(xyz=(branch["pad_center_x"], 0.0, 0.0)),
            material=pad_gray,
            name="pad",
        )
        arm.inertial = Inertial.from_geometry(
            Box((branch["pad_center_x"], 0.050, 0.040)),
            mass=0.65,
            origin=Origin(xyz=(branch["pad_center_x"] / 2.0, 0.0, 0.0)),
        )

        lower, upper = branch["limits"]
        model.articulation(
            branch["joint_name"],
            ArticulationType.REVOLUTE,
            parent=hub,
            child=arm,
            origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.6,
                lower=lower,
                upper=upper,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    hubs = [object_model.get_part(branch["hub_name"]) for branch in BRANCH_CONFIGS]
    arms = [object_model.get_part(branch["arm_name"]) for branch in BRANCH_CONFIGS]
    hub_mounts = [object_model.get_articulation(branch["mount_name"]) for branch in BRANCH_CONFIGS]
    arm_joints = [object_model.get_articulation(branch["joint_name"]) for branch in BRANCH_CONFIGS]

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
        "fixture_part_count",
        len(object_model.parts) == 7,
        f"expected 7 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "fixture_articulation_count",
        len(object_model.articulations) == 6,
        f"expected 6 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "three_revolute_branches",
        sum(
            1 for articulation in object_model.articulations
            if articulation.articulation_type == ArticulationType.REVOLUTE
        )
        == 3,
        "fixture head should expose exactly three independently pivoting branches",
    )

    for branch, hub, arm, hub_mount, arm_joint in zip(
        BRANCH_CONFIGS, hubs, arms, hub_mounts, arm_joints
    ):
        ctx.expect_contact(hub, mast, contact_tol=0.0025, name=f"{hub.name}_mounted_to_mast")
        ctx.expect_contact(arm, hub, contact_tol=0.0025, name=f"{arm.name}_captured_in_hub")
        ctx.check(
            f"{arm_joint.name}_axis_orientation",
            tuple(round(value, 6) for value in arm_joint.axis) == (0.0, 1.0, 0.0),
            f"expected local hub-axis revolute motion for {arm_joint.name}",
        )
        limits = arm_joint.motion_limits
        ctx.check(
            f"{arm_joint.name}_motion_range",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and (limits.upper - limits.lower) >= 1.45,
            f"unexpected motion limits for {arm_joint.name}: {limits}",
        )
        ctx.check(
            f"{hub_mount.name}_yaw_set",
            hub_mount.origin is not None
            and abs(hub_mount.origin.rpy[2] - branch["yaw"]) < 1e-9,
            f"hub yaw for {hub_mount.name} does not match branch layout",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    hub_positions = [ctx.part_world_position(hub) for hub in hubs]
    if all(position is not None for position in hub_positions):
        hub_z = [position[2] for position in hub_positions]
        ctx.check(
            "hub_heights_staggered",
            hub_z[0] < hub_z[1] < hub_z[2],
            f"hub heights should rise along the mast, got {hub_z}",
        )
    else:
        ctx.fail("hub_positions_available", "could not resolve all hub world positions")

    def element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) / 2.0 for axis in range(3))

    pad_centers = [element_center(branch["arm_name"], "pad") for branch in BRANCH_CONFIGS]
    if all(center is not None for center in pad_centers):
        pad_z = [center[2] for center in pad_centers]
        pad_radii = [math.hypot(center[0], center[1]) for center in pad_centers]
        pairwise_xy = []
        for index, first in enumerate(pad_centers):
            for second in pad_centers[index + 1:]:
                pairwise_xy.append(math.hypot(first[0] - second[0], first[1] - second[1]))
        ctx.check(
            "pad_heights_staggered",
            pad_z[0] < pad_z[1] < pad_z[2],
            f"pad heights should reflect the staggered branch stack, got {pad_z}",
        )
        ctx.check(
            "pads_reach_outward",
            min(pad_radii) > 0.14,
            f"pads should sit well away from the mast, got radial distances {pad_radii}",
        )
        ctx.check(
            "pads_sector_separation",
            min(pairwise_xy) > 0.19,
            f"pad centers should occupy distinct radial sectors, got separations {pairwise_xy}",
        )
    else:
        ctx.fail("pad_centers_available", "could not resolve one or more mounting pad positions")

    for arm_joint in arm_joints:
        limits = arm_joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{arm_joint.name}_limits_available", "bounded revolute joint missing limits")
            continue
        with ctx.pose({arm_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{arm_joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{arm_joint.name}_lower_no_floating")
        with ctx.pose({arm_joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{arm_joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{arm_joint.name}_upper_no_floating")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=42,
        ignore_adjacent=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
