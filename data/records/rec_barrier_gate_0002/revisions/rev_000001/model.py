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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_lot_gate", assets=ASSETS)

    steel = model.material("steel", rgba=(0.38, 0.40, 0.43, 1.0))
    painted_white = model.material("painted_white", rgba=(0.93, 0.94, 0.95, 1.0))
    reflective_red = model.material("reflective_red", rgba=(0.84, 0.12, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    counterweight_dark = model.material("counterweight_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    post = model.part("post")
    post.visual(
        Box((0.24, 0.22, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=steel,
        name="housing",
    )
    post.visual(
        Box((0.34, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=steel,
        name="base_plate",
    )
    post.visual(
        Box((0.20, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=steel,
        name="head_cap",
    )
    post.visual(
        Box((0.08, 0.10, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 1.15)),
        material=steel,
        name="hinge_block",
    )
    post.visual(
        Box((0.21, 0.05, 0.16)),
        origin=Origin(xyz=(0.1925, -0.085, 1.23)),
        material=steel,
        name="yoke_support_left",
    )
    post.visual(
        Box((0.21, 0.05, 0.16)),
        origin=Origin(xyz=(0.1925, 0.085, 1.23)),
        material=steel,
        name="yoke_support_right",
    )
    post.visual(
        Box((0.08, 0.02, 0.14)),
        origin=Origin(xyz=(0.29, -0.055, 1.29)),
        material=steel,
        name="hinge_cheek_left",
    )
    post.visual(
        Box((0.08, 0.02, 0.14)),
        origin=Origin(xyz=(0.29, 0.055, 1.29)),
        material=steel,
        name="hinge_cheek_right",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.29, -0.11, 1.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pin_head",
    )
    post.visual(
        Cylinder(radius=0.012, length=0.230),
        origin=Origin(xyz=(0.29, 0.0, 1.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    post.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.29, 0.107, 1.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=counterweight_dark,
        name="lock_nut",
    )

    arm = model.part("arm")
    arm.visual(
        Box((4.05, 0.08, 0.08)),
        origin=Origin(xyz=(2.07, 0.0, 0.0)),
        material=painted_white,
        name="boom_tube",
    )
    arm.visual(
        Box((0.09, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_lug",
    )
    arm.visual(
        Box((0.14, 0.04, 0.04)),
        origin=Origin(xyz=(-0.10, 0.0, -0.02)),
        material=painted_white,
        name="tail_tube",
    )
    arm.visual(
        Box((0.06, 0.04, 0.10)),
        origin=Origin(xyz=(-0.11, 0.0, -0.085)),
        material=steel,
        name="counterweight_bracket",
    )
    arm.visual(
        Box((0.04, 0.04, 0.05)),
        origin=Origin(xyz=(-0.11, 0.0, -0.155)),
        material=steel,
        name="counterweight_hanger",
    )
    arm.visual(
        Box((0.14, 0.10, 0.12)),
        origin=Origin(xyz=(-0.09, 0.0, -0.235)),
        material=counterweight_dark,
        name="counterweight_block",
    )
    arm.visual(
        Box((3.70, 0.012, 0.04)),
        origin=Origin(xyz=(2.05, 0.0, -0.06)),
        material=reflective_red,
        name="reflective_strip",
    )
    arm.visual(
        Box((0.10, 0.086, 0.086)),
        origin=Origin(xyz=(4.145, 0.0, 0.0)),
        material=rubber_black,
        name="tip_cap",
    )

    model.articulation(
        "boom_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=arm,
        origin=Origin(xyz=(0.29, 0.0, 1.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    post = object_model.get_part("post")
    arm = object_model.get_part("arm")
    boom_hinge = object_model.get_articulation("boom_hinge")

    head_cap = post.get_visual("head_cap")
    hinge_block = post.get_visual("hinge_block")
    hinge_cheek_left = post.get_visual("hinge_cheek_left")
    hinge_cheek_right = post.get_visual("hinge_cheek_right")
    housing = post.get_visual("housing")
    pin_head = post.get_visual("pin_head")
    pivot_pin = post.get_visual("pivot_pin")
    lock_nut = post.get_visual("lock_nut")

    boom_tube = arm.get_visual("boom_tube")
    pivot_lug = arm.get_visual("pivot_lug")
    tail_tube = arm.get_visual("tail_tube")
    counterweight_bracket = arm.get_visual("counterweight_bracket")
    counterweight_hanger = arm.get_visual("counterweight_hanger")
    reflective_strip = arm.get_visual("reflective_strip")
    tip_cap = arm.get_visual("tip_cap")
    counterweight_block = arm.get_visual("counterweight_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        post,
        arm,
        elem_a=pivot_pin,
        elem_b=pivot_lug,
        reason="the visible hinge pin passes through the boom pivot lug",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        post,
        arm,
        axis="y",
        max_gap=0.022,
        max_penetration=0.0,
        positive_elem=hinge_cheek_right,
        negative_elem=pivot_lug,
    )
    ctx.expect_gap(
        arm,
        post,
        axis="y",
        max_gap=0.022,
        max_penetration=0.0,
        positive_elem=pivot_lug,
        negative_elem=hinge_cheek_left,
    )
    ctx.expect_overlap(post, arm, axes="xz", min_overlap=0.02, elem_a=pivot_pin, elem_b=pivot_lug)
    ctx.expect_overlap(post, arm, axes="xz", min_overlap=0.015, elem_a=pin_head, elem_b=pivot_lug)
    ctx.expect_overlap(post, arm, axes="xz", min_overlap=0.015, elem_a=lock_nut, elem_b=pivot_lug)
    ctx.expect_gap(
        arm,
        arm,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=tip_cap,
        negative_elem=boom_tube,
    )
    ctx.expect_within(arm, arm, axes="xy", inner_elem=reflective_strip, outer_elem=boom_tube)
    ctx.expect_gap(
        arm,
        arm,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=boom_tube,
        negative_elem=reflective_strip,
    )
    ctx.expect_gap(
        arm,
        arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.01,
        positive_elem=tail_tube,
        negative_elem=counterweight_bracket,
    )
    ctx.expect_gap(
        arm,
        arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.01,
        positive_elem=counterweight_bracket,
        negative_elem=counterweight_hanger,
    )
    ctx.expect_gap(
        arm,
        arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.01,
        positive_elem=counterweight_hanger,
        negative_elem=counterweight_block,
    )
    ctx.expect_within(arm, arm, axes="xy", inner_elem=counterweight_hanger, outer_elem=counterweight_block)

    with ctx.pose({boom_hinge: 1.0}):
        ctx.expect_gap(
            arm,
            post,
            axis="z",
            min_gap=2.1,
            positive_elem=tip_cap,
            negative_elem=head_cap,
        )
        ctx.expect_gap(
            arm,
            post,
            axis="x",
            min_gap=0.01,
            positive_elem=counterweight_block,
            negative_elem=housing,
        )
        ctx.expect_gap(
            arm,
            post,
            axis="z",
            min_gap=0.05,
            positive_elem=boom_tube,
            negative_elem=head_cap,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
