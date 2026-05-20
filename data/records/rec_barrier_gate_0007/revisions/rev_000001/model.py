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
    model = ArticulatedObject(name="security_barrier_gate", assets=ASSETS)

    housing_gray = model.material("housing_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.17, 1.0))
    boom_white = model.material("boom_white", rgba=(0.95, 0.96, 0.94, 1.0))
    stripe_red = model.material("stripe_red", rgba=(0.78, 0.16, 0.16, 1.0))
    beacon_amber = model.material("beacon_amber", rgba=(0.95, 0.66, 0.15, 0.95))

    housing = model.part("housing")
    housing.visual(
        Box((0.14, 0.14, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=housing_gray,
        name="pedestal",
    )
    housing.visual(
        Box((0.38, 0.18, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material=housing_gray,
        name="housing_shell",
    )
    housing.visual(
        Box((0.32, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.554)),
        material=dark_trim,
        name="service_lid",
    )
    housing.visual(
        Box((0.026, 0.104, 0.11)),
        origin=Origin(xyz=(0.178, 0.0, 0.49)),
        material=dark_trim,
        name="pivot_backplate",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(-0.11, 0.0, 0.575)),
        material=beacon_amber,
        name="warning_beacon",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.38, 0.18, 0.56)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    inner_boom = model.part("inner_boom")
    inner_boom.visual(
        Cylinder(radius=0.027, length=0.052),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="pivot_hub",
    )
    inner_boom.visual(
        Box((0.09, 0.08, 0.062)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=dark_trim,
        name="root_collar",
    )
    inner_boom.visual(
        Box((1.20, 0.08, 0.048)),
        origin=Origin(xyz=(0.67, 0.0, 0.0)),
        material=boom_white,
        name="inner_beam",
    )
    for idx, x_pos in enumerate((0.30, 0.58, 0.86, 1.12), start=1):
        inner_boom.visual(
            Box((0.17, 0.082, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0225)),
            material=stripe_red,
            name=f"inner_stripe_{idx}",
        )
    inner_boom.visual(
        Box((0.088, 0.084, 0.078)),
        origin=Origin(xyz=(1.294, 0.0, 0.0)),
        material=dark_trim,
        name="fold_backplate",
    )
    inner_boom.inertial = Inertial.from_geometry(
        Box((1.36, 0.10, 0.10)),
        mass=6.5,
        origin=Origin(xyz=(0.68, 0.0, 0.0)),
    )

    outer_boom = model.part("outer_boom")
    outer_boom.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="mid_hub",
    )
    outer_boom.visual(
        Box((1.12, 0.075, 0.043)),
        origin=Origin(xyz=(0.56, 0.0, 0.0)),
        material=boom_white,
        name="outer_beam",
    )
    for idx, x_pos in enumerate((0.25, 0.52, 0.79), start=1):
        outer_boom.visual(
            Box((0.15, 0.077, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0205)),
            material=stripe_red,
            name=f"outer_stripe_{idx}",
        )
    outer_boom.visual(
        Box((0.10, 0.085, 0.055)),
        origin=Origin(xyz=(1.17, 0.0, 0.0)),
        material=stripe_red,
        name="tip_pad",
    )
    outer_boom.inertial = Inertial.from_geometry(
        Box((1.20, 0.10, 0.08)),
        mass=5.0,
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
    )

    model.articulation(
        "housing_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inner_boom,
        origin=Origin(xyz=(0.218, 0.0, 0.49)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "mid_fold",
        ArticulationType.REVOLUTE,
        parent=inner_boom,
        child=outer_boom,
        origin=Origin(xyz=(1.36, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=-1.15,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    inner_boom = object_model.get_part("inner_boom")
    outer_boom = object_model.get_part("outer_boom")
    housing_pivot = object_model.get_articulation("housing_pivot")
    mid_fold = object_model.get_articulation("mid_fold")

    housing_shell = housing.get_visual("housing_shell")
    pivot_backplate = housing.get_visual("pivot_backplate")
    pivot_hub = inner_boom.get_visual("pivot_hub")
    inner_beam = inner_boom.get_visual("inner_beam")
    fold_backplate = inner_boom.get_visual("fold_backplate")
    mid_hub = outer_boom.get_visual("mid_hub")
    outer_beam = outer_boom.get_visual("outer_beam")
    tip_pad = outer_boom.get_visual("tip_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        inner_boom,
        housing,
        axis="x",
        min_gap=0.02,
        positive_elem=inner_beam,
        negative_elem=housing_shell,
        name="inner_boom_projects_outboard_of_flat_housing",
    )
    ctx.expect_within(
        inner_boom,
        housing,
        axes="yz",
        inner_elem=pivot_hub,
        outer_elem=pivot_backplate,
        name="housing_pivot_hub_is_centered_in_mount",
    )
    ctx.expect_gap(
        inner_boom,
        housing,
        axis="x",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=pivot_hub,
        negative_elem=pivot_backplate,
        name="housing_pivot_hub_is_seated_against_backplate",
    )
    ctx.expect_within(
        outer_boom,
        inner_boom,
        axes="yz",
        inner_elem=mid_hub,
        outer_elem=fold_backplate,
        name="midpoint_hub_is_centered_in_fold_mount",
    )
    ctx.expect_gap(
        outer_boom,
        inner_boom,
        axis="x",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=mid_hub,
        negative_elem=fold_backplate,
        name="midpoint_hub_is_seated_against_fold_backplate",
    )
    ctx.expect_overlap(
        inner_boom,
        outer_boom,
        axes="yz",
        min_overlap=0.003,
        elem_a=inner_beam,
        elem_b=outer_beam,
        name="boom_segments_align_into_one_barrier_in_rest_pose",
    )
    ctx.expect_gap(
        outer_boom,
        housing,
        axis="x",
        min_gap=2.3,
        positive_elem=tip_pad,
        negative_elem=housing_shell,
        name="full_barrier_reaches_far_beyond_housing",
    )

    with ctx.pose({housing_pivot: 1.18, mid_fold: -0.75}):
        ctx.expect_gap(
            inner_boom,
            housing,
            axis="z",
            min_gap=0.45,
            positive_elem=fold_backplate,
            negative_elem=housing_shell,
            name="inner_segment_lifts_clear_when_gate_raises",
        )
        ctx.expect_gap(
            outer_boom,
            housing,
            axis="z",
            min_gap=1.0,
            positive_elem=tip_pad,
            negative_elem=housing_shell,
            name="outer_segment_stays_high_in_folded_raised_pose",
        )
        ctx.expect_gap(
            outer_boom,
            housing,
            axis="x",
            min_gap=1.2,
            max_gap=1.9,
            positive_elem=tip_pad,
            negative_elem=housing_shell,
            name="folded_outer_segment_draws_in_toward_pivot_when_raised",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
