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
    model = ArticulatedObject(name="utility_sewing_box", assets=ASSETS)

    painted_body = model.material("painted_body", rgba=(0.30, 0.35, 0.28, 1.0))
    molded_trim = model.material("molded_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    hardware = model.material("hardware", rgba=(0.66, 0.68, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body_w = 0.24
    body_d = 0.16
    body_h = 0.09
    bottom_t = 0.012
    wall_t = 0.008
    rim_t = 0.010
    rim_h = 0.008
    top_z = body_h

    hinge_y = -0.082
    hinge_z = 0.102

    lid_w = 0.236
    lid_d = 0.156
    lid_t = 0.018
    lid_z = top_z + (lid_t / 2.0) - hinge_z

    def x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))

    base = model.part("base")
    base.visual(
        Box((body_w, body_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=painted_body,
        name="bottom",
    )
    base.visual(
        Box((body_w, wall_t, body_h - bottom_t)),
        origin=Origin(xyz=(0.0, (body_d / 2.0) - (wall_t / 2.0), bottom_t + (body_h - bottom_t) / 2.0)),
        material=painted_body,
        name="front_wall",
    )
    base.visual(
        Box((body_w, wall_t, 0.066)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0) + (wall_t / 2.0), bottom_t + 0.033)),
        material=painted_body,
        name="rear_wall",
    )
    base.visual(
        Box((wall_t, body_d - (2.0 * wall_t), body_h - bottom_t)),
        origin=Origin(xyz=((body_w / 2.0) - (wall_t / 2.0), 0.0, bottom_t + (body_h - bottom_t) / 2.0)),
        material=painted_body,
        name="right_wall",
    )
    base.visual(
        Box((wall_t, body_d - (2.0 * wall_t), body_h - bottom_t)),
        origin=Origin(xyz=(-(body_w / 2.0) + (wall_t / 2.0), 0.0, bottom_t + (body_h - bottom_t) / 2.0)),
        material=painted_body,
        name="left_wall",
    )

    base.visual(
        Box((0.216, rim_t, rim_h)),
        origin=Origin(xyz=(0.0, 0.066, top_z - (rim_h / 2.0))),
        material=molded_trim,
        name="front_rim",
    )
    base.visual(
        Box((0.200, rim_t, rim_h)),
        origin=Origin(xyz=(0.0, -0.066, top_z - (rim_h / 2.0))),
        material=molded_trim,
        name="rear_rim",
    )
    base.visual(
        Box((rim_t, 0.132, rim_h)),
        origin=Origin(xyz=(0.106, 0.0, top_z - (rim_h / 2.0))),
        material=molded_trim,
        name="right_rim",
    )
    base.visual(
        Box((rim_t, 0.132, rim_h)),
        origin=Origin(xyz=(-0.106, 0.0, top_z - (rim_h / 2.0))),
        material=molded_trim,
        name="left_rim",
    )

    for idx, (x_pos, y_pos) in enumerate(
        (
            (0.103, 0.063),
            (-0.103, 0.063),
            (0.103, -0.063),
            (-0.103, -0.063),
        ),
        start=1,
    ):
        base.visual(
            Box((0.014, 0.014, 0.066)),
            origin=Origin(xyz=(x_pos, y_pos, 0.045)),
            material=molded_trim,
            name=f"corner_reinforcement_{idx}",
        )

    for name, x_pos in (
        ("hinge_support_left", -0.078),
        ("hinge_support_center", 0.0),
        ("hinge_support_right", 0.078),
    ):
        base.visual(
            Box((0.018, 0.016, 0.024)),
            origin=Origin(xyz=(x_pos, hinge_y, hinge_z)),
            material=painted_body,
            name=name,
        )
        base.visual(
            Box((0.030, 0.020, 0.010)),
            origin=Origin(xyz=(x_pos, -0.078, top_z - 0.005)),
            material=molded_trim,
            name=f"{name}_pad",
        )

    base.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=x_axis_origin((0.119, hinge_y, hinge_z)),
        material=hardware,
        name="pin_cap_right",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=x_axis_origin((-0.119, hinge_y, hinge_z)),
        material=hardware,
        name="pin_cap_left",
    )

    for idx, (x_pos, y_pos) in enumerate(
        (
            (0.094, 0.058),
            (-0.094, 0.058),
            (0.094, -0.058),
            (-0.094, -0.058),
        ),
        start=1,
    ):
        base.visual(
            Box((0.020, 0.020, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.002)),
            material=rubber,
            name=f"foot_{idx}",
        )

    base.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, 0.018, lid_t)),
        origin=Origin(xyz=(0.0, 0.147, lid_z)),
        material=painted_body,
        name="front_frame",
    )
    lid.visual(
        Box((lid_w, 0.014, lid_t)),
        origin=Origin(xyz=(0.0, 0.022, lid_z)),
        material=painted_body,
        name="rear_frame",
    )
    lid.visual(
        Box((0.018, 0.120, lid_t)),
        origin=Origin(xyz=(0.109, 0.084, lid_z)),
        material=painted_body,
        name="right_frame",
    )
    lid.visual(
        Box((0.018, 0.120, lid_t)),
        origin=Origin(xyz=(-0.109, 0.084, lid_z)),
        material=painted_body,
        name="left_frame",
    )
    lid.visual(
        Box((0.196, 0.116, 0.008)),
        origin=Origin(xyz=(0.0, 0.084, -0.004)),
        material=molded_trim,
        name="inner_panel",
    )

    lid.visual(
        Box((0.206, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.136, -0.021)),
        material=molded_trim,
        name="front_lip",
    )
    lid.visual(
        Box((0.006, 0.108, 0.010)),
        origin=Origin(xyz=(0.095, 0.082, -0.021)),
        material=molded_trim,
        name="right_lip",
    )
    lid.visual(
        Box((0.006, 0.108, 0.010)),
        origin=Origin(xyz=(-0.095, 0.082, -0.021)),
        material=molded_trim,
        name="left_lip",
    )
    lid.visual(
        Box((0.176, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, -0.021)),
        material=molded_trim,
        name="rear_lip",
    )

    for name, x_pos in (("left_hinge_barrel", -0.039), ("right_hinge_barrel", 0.039)):
        lid.visual(
            Cylinder(radius=0.006, length=0.050),
            origin=x_axis_origin((x_pos, 0.0, 0.0)),
            material=hardware,
            name=name,
        )
        lid.visual(
            Box((0.050, 0.028, 0.010)),
            origin=Origin(xyz=(x_pos, 0.014, -0.004)),
            material=painted_body,
            name=f"{name}_strap",
        )

    for idx, (x_pos, y_pos, z_pos) in enumerate(
        (
            (-0.039, 0.014, 0.0),
            (0.039, 0.014, 0.0),
            (-0.082, 0.147, 0.004),
            (0.082, 0.147, 0.004),
            (-0.109, 0.084, 0.004),
            (0.109, 0.084, 0.004),
        ),
        start=1,
    ):
        lid.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=hardware,
            name=f"lid_fastener_{idx}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=0.75,
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_z)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_contact("lid", "base")
    ctx.expect_aabb_gap(
        "lid",
        "base",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="front_frame",
        negative_elem="front_rim",
        name="front frame seats cleanly on the front rim",
    )
    ctx.expect_aabb_gap(
        "lid",
        "base",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="left_frame",
        negative_elem="left_rim",
        name="left frame rail sits on the left rim",
    )
    ctx.expect_aabb_gap(
        "lid",
        "base",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="right_frame",
        negative_elem="right_rim",
        name="right frame rail sits on the right rim",
    )
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(lid_hinge=1.35):
        ctx.expect_aabb_gap(
            "lid",
            "base",
            axis="z",
            min_gap=0.025,
            positive_elem="front_frame",
            negative_elem="front_rim",
            name="opened lid front edge clears the storage opening",
        )
        ctx.expect_aabb_overlap("lid", "base", axes="x", min_overlap=0.18)

    with ctx.pose(lid_hinge=2.0):
        ctx.expect_aabb_gap(
            "lid",
            "base",
            axis="z",
            min_gap=0.06,
            positive_elem="front_frame",
            negative_elem="front_rim",
            name="full open pose keeps the front edge high above the body",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
