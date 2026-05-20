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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestrian_barrier_gate", assets=ASSETS)

    cabinet_gray = model.material("cabinet_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_white = model.material("arm_white", rgba=(0.94, 0.95, 0.96, 1.0))
    alert_red = model.material("alert_red", rgba=(0.78, 0.12, 0.12, 1.0))
    hinge_black = model.material("hinge_black", rgba=(0.10, 0.10, 0.11, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.46, 0.30, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cabinet_gray,
        name="plinth",
    )
    cabinet.visual(
        Box((0.40, 0.24, 0.74)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=cabinet_gray,
        name="cabinet_body",
    )
    cabinet.visual(
        Box((0.42, 0.26, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=cabinet_gray,
        name="top_cap",
    )
    cabinet.visual(
        Box((0.28, 0.012, 0.54)),
        origin=Origin(xyz=(0.0, 0.126, 0.41)),
        material=panel_gray,
        name="service_panel",
    )
    cabinet.visual(
        Box((0.125, 0.05, 0.18)),
        origin=Origin(xyz=(0.2005, 0.0, 0.71)),
        material=cabinet_gray,
        name="hinge_support",
    )
    hinge_axis_origin = Origin(xyz=(0.285, 0.0, 0.75))
    cabinet.inertial = Inertial.from_geometry(
        Box((0.40, 0.24, 0.74)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_black,
        name="arm_knuckle",
    )
    arm.visual(
        Box((0.12, 0.018, 0.045)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=hinge_black,
        name="arm_root_block",
    )
    arm.visual(
        Box((1.00, 0.07, 0.028)),
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        material=arm_white,
        name="arm_bar",
    )
    arm.visual(
        Box((0.20, 0.072, 0.004)),
        origin=Origin(xyz=(0.34, 0.0, 0.016)),
        material=alert_red,
        name="stripe_front",
    )
    arm.visual(
        Box((0.20, 0.072, 0.004)),
        origin=Origin(xyz=(0.74, 0.0, 0.016)),
        material=alert_red,
        name="stripe_rear",
    )
    arm.visual(
        Box((0.085, 0.085, 0.035)),
        origin=Origin(xyz=(1.08, 0.0, 0.0)),
        material=hinge_black,
        name="tip_cap",
    )
    arm.inertial = Inertial.from_geometry(
        Box((1.00, 0.07, 0.028)),
        mass=4.5,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
    )

    model.articulation(
        "cabinet_to_arm",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=arm,
        origin=hinge_axis_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    arm = object_model.get_part("arm")
    hinge = object_model.get_articulation("cabinet_to_arm")
    cabinet_body = cabinet.get_visual("cabinet_body")
    top_cap = cabinet.get_visual("top_cap")
    hinge_support = cabinet.get_visual("hinge_support")
    arm_knuckle = arm.get_visual("arm_knuckle")
    arm_root_block = arm.get_visual("arm_root_block")
    arm_bar = arm.get_visual("arm_bar")
    tip_cap = arm.get_visual("tip_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.03)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(arm, cabinet, elem_a=arm_knuckle, elem_b=hinge_support)
    ctx.expect_contact(arm, arm, elem_a=arm_knuckle, elem_b=arm_root_block)
    ctx.expect_overlap(arm, cabinet, axes="yz", elem_a=arm_knuckle, elem_b=hinge_support, min_overlap=0.03)
    ctx.expect_gap(
        arm,
        cabinet,
        axis="x",
        positive_elem=tip_cap,
        negative_elem=cabinet_body,
        min_gap=0.90,
    )
    ctx.expect_gap(
        arm,
        cabinet,
        axis="x",
        positive_elem=arm_bar,
        negative_elem=hinge_support,
        min_gap=0.05,
    )
    ctx.expect_gap(
        cabinet,
        arm,
        axis="z",
        positive_elem=top_cap,
        negative_elem=arm_bar,
        min_gap=0.015,
        max_gap=0.04,
    )

    with ctx.pose({hinge: math.pi / 2.0}):
        ctx.expect_contact(arm, cabinet, elem_a=arm_knuckle, elem_b=hinge_support)
        ctx.expect_contact(arm, arm, elem_a=arm_knuckle, elem_b=arm_root_block)
        ctx.expect_gap(
            arm,
            cabinet,
            axis="z",
            positive_elem=tip_cap,
            negative_elem=top_cap,
            min_gap=0.90,
        )
        ctx.expect_overlap(arm, cabinet, axes="yz", elem_a=arm_knuckle, elem_b=hinge_support, min_overlap=0.03)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
