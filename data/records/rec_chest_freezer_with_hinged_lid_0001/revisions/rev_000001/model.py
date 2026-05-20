from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _add_box(part, size, xyz, material, rpy=(0.0, 0.0, 0.0), name=None):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0), name=None):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chest_freezer", assets=ASSETS)

    white_enamel = model.material("white_enamel", rgba=(0.94, 0.95, 0.96, 1.0))
    lid_enamel = model.material("lid_enamel", rgba=(0.96, 0.97, 0.98, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.84, 0.86, 0.88, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.27, 0.29, 0.32, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))

    body_w = 1.28
    body_d = 0.72
    body_h = 0.88
    base_h = 0.12
    shell_t = 0.055

    opening_w = 1.10
    opening_d = 0.54
    liner_t = 0.018
    liner_floor_z = 0.145
    rim_t = 0.03

    lid_w = 1.30
    lid_d = 0.76
    lid_t = 0.065
    hinge_y = -(body_d / 2.0) + 0.03

    body = model.part("body")
    _add_box(
        body, (body_w, body_d, base_h), (0.0, 0.0, base_h / 2.0), white_enamel, name="base_pan"
    )
    _add_box(
        body,
        (shell_t, body_d, body_h - base_h + 0.005),
        (-(body_w / 2.0) + shell_t / 2.0, 0.0, (base_h + body_h) / 2.0 - 0.0025),
        white_enamel,
        name="left_outer_wall",
    )
    _add_box(
        body,
        (shell_t, body_d, body_h - base_h + 0.005),
        ((body_w / 2.0) - shell_t / 2.0, 0.0, (base_h + body_h) / 2.0 - 0.0025),
        white_enamel,
        name="right_outer_wall",
    )
    _add_box(
        body,
        (body_w, shell_t, body_h - base_h + 0.005),
        (0.0, (body_d / 2.0) - shell_t / 2.0, (base_h + body_h) / 2.0 - 0.0025),
        white_enamel,
        name="front_outer_wall",
    )
    _add_box(
        body,
        (body_w, shell_t, body_h - base_h + 0.005),
        (0.0, -(body_d / 2.0) + shell_t / 2.0, (base_h + body_h) / 2.0 - 0.0025),
        white_enamel,
        name="rear_outer_wall",
    )

    inner_wall_h = body_h - liner_floor_z - 0.015
    inner_wall_z = liner_floor_z + inner_wall_h / 2.0
    _add_box(
        body,
        (liner_t, opening_d, inner_wall_h),
        (-(opening_w / 2.0) - liner_t / 2.0, 0.0, inner_wall_z),
        liner_gray,
        name="left_liner",
    )
    _add_box(
        body,
        (liner_t, opening_d, inner_wall_h),
        ((opening_w / 2.0) + liner_t / 2.0, 0.0, inner_wall_z),
        liner_gray,
        name="right_liner",
    )
    _add_box(
        body,
        (opening_w + 2.0 * liner_t, liner_t, inner_wall_h),
        (0.0, (opening_d / 2.0) + liner_t / 2.0, inner_wall_z),
        liner_gray,
        name="front_liner",
    )
    _add_box(
        body,
        (opening_w + 2.0 * liner_t, liner_t, inner_wall_h),
        (0.0, -(opening_d / 2.0) - liner_t / 2.0, inner_wall_z),
        liner_gray,
        name="rear_liner",
    )
    _add_box(
        body,
        (opening_w + 2.0 * liner_t, opening_d + 2.0 * liner_t, 0.03),
        (0.0, 0.0, liner_floor_z - 0.015),
        liner_gray,
        name="liner_floor",
    )
    _add_box(
        body,
        (0.24, 0.50, 0.18),
        (0.36, 0.0, liner_floor_z + 0.09),
        liner_gray,
        name="compressor_hump",
    )

    side_rim_w = (body_w - opening_w) / 2.0
    front_rim_d = (body_d - opening_d) / 2.0
    _add_box(
        body,
        (side_rim_w, body_d, rim_t),
        (-(opening_w / 2.0) - side_rim_w / 2.0, 0.0, body_h - rim_t / 2.0),
        trim_gray,
        name="left_top_rim",
    )
    _add_box(
        body,
        (side_rim_w, body_d, rim_t),
        ((opening_w / 2.0) + side_rim_w / 2.0, 0.0, body_h - rim_t / 2.0),
        trim_gray,
        name="right_top_rim",
    )
    _add_box(
        body,
        (opening_w, front_rim_d, rim_t),
        (0.0, (opening_d / 2.0) + front_rim_d / 2.0, body_h - rim_t / 2.0),
        trim_gray,
        name="front_top_rim",
    )
    _add_box(
        body,
        (opening_w, front_rim_d, rim_t),
        (0.0, -(opening_d / 2.0) - front_rim_d / 2.0, body_h - rim_t / 2.0),
        trim_gray,
        name="rear_top_rim",
    )

    _add_box(
        body,
        (0.18, 0.045, 0.12),
        (0.39, body_d / 2.0 + 0.0225, 0.22),
        dark_plastic,
        name="control_housing",
    )
    _add_box(
        body,
        (0.11, 0.006, 0.05),
        (0.39, body_d / 2.0 + 0.048, 0.245),
        trim_gray,
        name="control_face",
    )
    _add_cylinder(
        body,
        radius=0.026,
        length=0.024,
        xyz=(0.325, body_d / 2.0 + 0.060, 0.22),
        material=charcoal,
        rpy=(pi / 2.0, 0.0, 0.0),
        name="thermostat_knob",
    )
    _add_box(
        body,
        (0.008, 0.01, 0.028),
        (0.325, body_d / 2.0 + 0.073, 0.238),
        steel,
        name="knob_indicator",
    )

    _add_box(
        body,
        (0.014, 0.28, 0.16),
        (body_w / 2.0 + 0.007, 0.09, 0.23),
        charcoal,
        name="condenser_vent_panel",
    )
    for i in range(6):
        z = 0.165 + 0.025 * i
        _add_box(
            body,
            (0.018, 0.24, 0.010),
            (body_w / 2.0 + 0.009, 0.09, z),
            trim_gray,
            name=f"vent_slat_{i}",
        )

    _add_box(
        body,
        (0.62, 0.008, 0.24),
        (0.0, -(body_d / 2.0) - 0.004, 0.24),
        trim_gray,
        name="rear_service_panel",
    )

    for idx, (fx, fy) in enumerate(
        (
            (0.53, 0.26),
            (-0.53, 0.26),
            (0.53, -0.26),
            (-0.53, -0.26),
        )
    ):
        _add_cylinder(
            body,
            radius=0.03,
            length=0.06,
            xyz=(fx, fy, 0.03),
            material=black_rubber,
            name=f"foot_{idx}",
        )

    for side, x in (("left", -0.43), ("right", 0.43)):
        _add_cylinder(
            body,
            radius=0.015,
            length=0.10,
            xyz=(x, hinge_y, body_h - 0.015),
            material=dark_plastic,
            rpy=(0.0, pi / 2.0, 0.0),
            name=f"{side}_hinge_barrel",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    _add_box(
        lid,
        (lid_w, lid_d, lid_t),
        (0.0, lid_d / 2.0 - 0.03, lid_t / 2.0),
        lid_enamel,
        name="lid_shell",
    )
    _add_box(
        lid,
        (1.02, 0.48, 0.008),
        (0.0, lid_d / 2.0 - 0.03, lid_t + 0.004),
        trim_gray,
        name="top_reinforcement_panel",
    )
    _add_box(
        lid,
        (1.18, 0.60, 0.026),
        (0.0, 0.315, 0.010),
        liner_gray,
        name="inner_lid_panel",
    )

    _add_box(lid, (0.022, 0.60, 0.014), (-0.569, 0.315, -0.007), black_rubber, name="gasket_left")
    _add_box(lid, (0.022, 0.60, 0.014), (0.569, 0.315, -0.007), black_rubber, name="gasket_right")
    _add_box(lid, (1.116, 0.022, 0.014), (0.0, 0.026, -0.007), black_rubber, name="gasket_rear")
    _add_box(lid, (1.116, 0.022, 0.014), (0.0, 0.604, -0.007), black_rubber, name="gasket_front")

    for side, x in (("left", -0.43), ("right", 0.43)):
        _add_box(
            lid,
            (0.10, 0.045, 0.020),
            (x, -0.005, 0.015),
            dark_plastic,
            name=f"{side}_hinge_leaf",
        )

    _add_box(
        lid, (0.045, 0.032, 0.030), (-0.185, 0.725, 0.032), dark_plastic, name="handle_mount_left"
    )
    _add_box(
        lid, (0.045, 0.032, 0.030), (0.185, 0.725, 0.032), dark_plastic, name="handle_mount_right"
    )

    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.185, 0.744, 0.032),
                (-0.110, 0.758, 0.046),
                (0.110, 0.758, 0.046),
                (0.185, 0.744, 0.032),
            ],
            radius=0.011,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("freezer_lid_handle.obj"),
    )
    lid.visual(handle_mesh, material=steel, name="handle_bar")

    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_t)),
        mass=12.0,
        origin=Origin(xyz=(0.0, lid_d / 2.0 - 0.03, lid_t / 2.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.0, hinge_y, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("lid", "body")
    ctx.expect_aabb_overlap("lid", "body", axes="x", min_overlap=1.20)
    ctx.expect_aabb_overlap("lid", "body", axes="y", min_overlap=0.71)
    ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.003, max_penetration=0.016)
    ctx.expect_origin_distance("lid", "body", axes="x", max_dist=0.02)
    ctx.expect_joint_motion_axis(
        "lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.08
    )

    with ctx.pose(lid_hinge=0.70):
        ctx.expect_aabb_overlap("lid", "body", axes="x", min_overlap=1.20)
        ctx.expect_aabb_overlap("lid", "body", axes="y", min_overlap=0.45)

    with ctx.pose(lid_hinge=1.35):
        ctx.expect_aabb_overlap("lid", "body", axes="x", min_overlap=1.20)
        ctx.expect_aabb_overlap("lid", "body", axes="y", min_overlap=0.16)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
