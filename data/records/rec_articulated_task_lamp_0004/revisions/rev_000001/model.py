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
)

ASSETS = AssetContext.from_script(__file__)

ARM_PIVOT_Z = 0.202
HEAD_PIVOT_X = 0.436


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_task_lamp", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.14, 0.15, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminium = model.material("aluminium", rgba=(0.76, 0.78, 0.80, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.94, 0.95, 1.0))
    diffuser = model.material("diffuser", rgba=(0.98, 0.98, 0.95, 0.82))

    mount = model.part("mount")
    mount.visual(
        Box((0.065, 0.078, 0.010)),
        origin=Origin(xyz=(0.010, 0.000, 0.055)),
        material=matte_black,
        name="clamp_top",
    )
    mount.visual(
        Box((0.016, 0.078, 0.104)),
        origin=Origin(xyz=(-0.022, 0.000, 0.008)),
        material=matte_black,
        name="clamp_back",
    )
    mount.visual(
        Box((0.026, 0.046, 0.022)),
        origin=Origin(xyz=(-0.014, 0.000, -0.025)),
        material=graphite,
        name="screw_collar",
    )
    mount.visual(
        Cylinder(radius=0.0065, length=0.052),
        origin=Origin(xyz=(-0.014, 0.000, -0.062)),
        material=graphite,
        name="screw_shaft",
    )
    mount.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.014, 0.000, -0.090)),
        material=matte_black,
        name="screw_pad",
    )
    mount.visual(
        Box((0.028, 0.034, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.068)),
        material=graphite,
        name="column_base",
    )
    mount.visual(
        Cylinder(radius=0.008, length=0.102),
        origin=Origin(xyz=(0.000, 0.000, 0.127)),
        material=graphite,
        name="column",
    )
    mount.visual(
        Box((0.012, 0.032, 0.016)),
        origin=Origin(xyz=(-0.012, 0.000, 0.184)),
        material=graphite,
        name="mount_bridge",
    )
    mount.visual(
        Box((0.016, 0.006, 0.032)),
        origin=Origin(xyz=(0.000, -0.013, ARM_PIVOT_Z)),
        material=graphite,
        name="mount_left_cheek",
    )
    mount.visual(
        Box((0.016, 0.006, 0.032)),
        origin=Origin(xyz=(0.000, 0.013, ARM_PIVOT_Z)),
        material=graphite,
        name="mount_right_cheek",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.090, 0.085, 0.290)),
        mass=1.8,
        origin=Origin(xyz=(-0.005, 0.000, 0.045)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(
            xyz=(0.000, 0.000, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=aluminium,
        name="arm_hub",
    )
    arm.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(0.024, 0.000, 0.000)),
        material=aluminium,
        name="rear_knuckle",
    )
    arm.visual(
        Box((0.388, 0.020, 0.012)),
        origin=Origin(xyz=(0.231, 0.000, 0.000)),
        material=aluminium,
        name="boom",
    )
    arm.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(0.433, -0.011, 0.000)),
        material=aluminium,
        name="head_left_cheek",
    )
    arm.visual(
        Box((0.016, 0.006, 0.024)),
        origin=Origin(xyz=(0.433, 0.011, 0.000)),
        material=aluminium,
        name="head_right_cheek",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.450, 0.026, 0.026)),
        mass=0.55,
        origin=Origin(xyz=(0.225, 0.000, 0.000)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.009, length=0.016),
        origin=Origin(
            xyz=(0.000, 0.000, 0.000),
            rpy=(math.pi / 2.0, 0.000, 0.000),
        ),
        material=graphite,
        name="head_hinge",
    )
    head.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(0.018, 0.000, 0.000)),
        material=graphite,
        name="neck",
    )
    head.visual(
        Box((0.156, 0.070, 0.018)),
        origin=Origin(xyz=(0.105, 0.000, 0.000)),
        material=warm_white,
        name="shade_shell",
    )
    head.visual(
        Box((0.122, 0.046, 0.004)),
        origin=Origin(xyz=(0.109, 0.000, -0.011)),
        material=diffuser,
        name="diffuser",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.190, 0.080, 0.026)),
        mass=0.34,
        origin=Origin(xyz=(0.095, 0.000, -0.001)),
    )

    arm_pitch = model.articulation(
        "mount_to_arm",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=arm,
        origin=Origin(xyz=(0.000, 0.000, ARM_PIVOT_Z)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.55,
            upper=1.00,
        ),
    )
    model.articulation(
        "arm_to_head",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=head,
        origin=Origin(xyz=(HEAD_PIVOT_X, 0.000, 0.000)),
        axis=(0.000, 1.000, 0.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.5,
            lower=-0.65,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    mount = object_model.get_part("mount")
    arm = object_model.get_part("arm")
    head = object_model.get_part("head")
    arm_pitch = object_model.get_articulation("mount_to_arm")
    head_pitch = object_model.get_articulation("arm_to_head")

    clamp_top = mount.get_visual("clamp_top")
    screw_pad = mount.get_visual("screw_pad")
    column = mount.get_visual("column")
    mount_left_cheek = mount.get_visual("mount_left_cheek")
    mount_right_cheek = mount.get_visual("mount_right_cheek")
    arm_hub = arm.get_visual("arm_hub")
    boom = arm.get_visual("boom")
    head_left_cheek = arm.get_visual("head_left_cheek")
    head_right_cheek = arm.get_visual("head_right_cheek")
    head_hinge = head.get_visual("head_hinge")
    shade_shell = head.get_visual("shade_shell")
    diffuser = head.get_visual("diffuser")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(arm, mount, elem_a=arm_hub, elem_b=mount_left_cheek)
    ctx.expect_contact(arm, mount, elem_a=arm_hub, elem_b=mount_right_cheek)
    ctx.expect_contact(head, arm, elem_a=head_hinge, elem_b=head_left_cheek)
    ctx.expect_contact(head, arm, elem_a=head_hinge, elem_b=head_right_cheek)
    ctx.expect_gap(
        mount,
        mount,
        axis="z",
        min_gap=0.12,
        positive_elem=clamp_top,
        negative_elem=screw_pad,
    )
    ctx.expect_gap(
        arm,
        mount,
        axis="z",
        min_gap=0.10,
        positive_elem=boom,
        negative_elem=clamp_top,
    )
    ctx.expect_gap(
        head,
        mount,
        axis="x",
        min_gap=0.34,
        positive_elem=shade_shell,
        negative_elem=column,
    )
    ctx.expect_within(
        head,
        head,
        axes="xy",
        inner_elem=diffuser,
        outer_elem=shade_shell,
    )
    ctx.expect_origin_distance(head, arm, axes="y", max_dist=0.002)

    with ctx.pose({arm_pitch: -0.20, head_pitch: 0.00}):
        ctx.expect_contact(head, arm, elem_a=head_hinge, elem_b=head_left_cheek)
        ctx.expect_contact(head, arm, elem_a=head_hinge, elem_b=head_right_cheek)
        ctx.expect_gap(
            head,
            mount,
            axis="x",
            min_gap=0.40,
            positive_elem=shade_shell,
            negative_elem=column,
        )
        ctx.expect_gap(
            head,
            mount,
            axis="z",
            min_gap=0.20,
            positive_elem=diffuser,
            negative_elem=clamp_top,
        )

    with ctx.pose({arm_pitch: -0.20, head_pitch: 0.25}):
        ctx.expect_gap(
            head,
            mount,
            axis="x",
            min_gap=0.30,
            positive_elem=shade_shell,
            negative_elem=column,
        )
        ctx.expect_gap(
            head,
            mount,
            axis="z",
            min_gap=0.03,
            positive_elem=diffuser,
            negative_elem=clamp_top,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
