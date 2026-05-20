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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _counterweight_fin_mesh():
    fin_profile = [
        (-0.10, 0.00),
        (0.12, 0.00),
        (0.04, 0.18),
        (-0.04, 0.15),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry.centered(fin_profile, 0.014).rotate_x(-math.pi / 2.0),
        "counterweight_fin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toll_highway_gate_arm")

    booth_gray = model.material("booth_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    concrete = model.material("concrete", rgba=(0.60, 0.61, 0.62, 1.0))
    arm_white = model.material("arm_white", rgba=(0.94, 0.95, 0.96, 1.0))
    stripe_red = model.material("stripe_red", rgba=(0.80, 0.14, 0.11, 1.0))
    fin_dark = model.material("fin_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    hardware = model.material("hardware", rgba=(0.41, 0.43, 0.45, 1.0))

    post = model.part("post")
    post.visual(
        Box((0.50, 0.48, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="base_plinth",
    )
    post.visual(
        Box((0.34, 0.30, 1.00)),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=booth_gray,
        name="control_post",
    )
    post.visual(
        Box((0.36, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=dark_trim,
        name="top_cap",
    )
    post.visual(
        Box((0.14, 0.18, 0.16)),
        origin=Origin(xyz=(0.22, 0.0, 1.02)),
        material=dark_trim,
        name="pivot_housing",
    )
    post.visual(
        Box((0.28, 0.02, 0.76)),
        origin=Origin(xyz=(0.001, -0.141, 0.64)),
        material=dark_trim,
        name="door_reveal",
    )
    post.inertial = Inertial.from_geometry(
        Box((0.50, 0.48, 1.18)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.04, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_hub",
    )
    boom.visual(
        Box((4.15, 0.10, 0.05)),
        origin=Origin(xyz=(2.075, 0.0, 0.0)),
        material=arm_white,
        name="boom_body",
    )
    boom.visual(
        Box((0.04, 0.10, 0.055)),
        origin=Origin(xyz=(4.17, 0.0, 0.0)),
        material=stripe_red,
        name="tip_cap",
    )
    boom.visual(
        _counterweight_fin_mesh(),
        origin=Origin(xyz=(0.10, 0.0, -0.015)),
        material=fin_dark,
        name="counterweight_fin",
    )
    boom.visual(
        Box((0.16, 0.12, 0.012)),
        origin=Origin(xyz=(0.10, 0.0, -0.030)),
        material=fin_dark,
        name="counterweight_root",
    )

    stripe_x_positions = (0.58, 1.34, 2.10, 2.86, 3.62)
    for index, x_pos in enumerate(stripe_x_positions):
        boom.visual(
            Box((0.22, 0.008, 0.09)),
            origin=Origin(
                xyz=(x_pos, 0.047, 0.0),
                rpy=(0.0, 0.58, 0.0),
            ),
            material=stripe_red,
            name=f"stripe_right_{index}",
        )
        boom.visual(
            Box((0.22, 0.008, 0.09)),
            origin=Origin(
                xyz=(x_pos, -0.047, 0.0),
                rpy=(0.0, -0.58, 0.0),
            ),
            material=stripe_red,
            name=f"stripe_left_{index}",
        )
    boom.inertial = Inertial.from_geometry(
        Box((4.24, 0.18, 0.32)),
        mass=22.0,
        origin=Origin(xyz=(2.02, 0.0, -0.01)),
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(0.33, 0.0, 1.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=0.0,
            upper=1.45,
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

    post = object_model.get_part("post")
    boom = object_model.get_part("boom")
    boom_joint = object_model.get_articulation("post_to_boom")

    ctx.check(
        "boom pivot uses horizontal lift axis",
        tuple(round(value, 6) for value in boom_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={boom_joint.axis}",
    )

    with ctx.pose({boom_joint: 0.0}):
        ctx.expect_gap(
            boom,
            post,
            axis="x",
            positive_elem="pivot_hub",
            negative_elem="pivot_housing",
            max_gap=0.001,
            max_penetration=0.0,
            name="pivot hub seats against housing face",
        )
        ctx.expect_overlap(
            boom,
            post,
            axes="yz",
            elem_a="pivot_hub",
            elem_b="pivot_housing",
            min_overlap=0.08,
            name="pivot hub is aligned with housing bracket",
        )

    closed_tip = ctx.part_element_world_aabb(boom, elem="tip_cap")
    with ctx.pose({boom_joint: boom_joint.motion_limits.upper}):
        open_tip = ctx.part_element_world_aabb(boom, elem="tip_cap")

    tip_lifts = (
        closed_tip is not None
        and open_tip is not None
        and open_tip[1][2] > closed_tip[1][2] + 3.2
    )
    ctx.check(
        "boom raises well above the post",
        tip_lifts,
        details=f"closed_tip={closed_tip}, open_tip={open_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
