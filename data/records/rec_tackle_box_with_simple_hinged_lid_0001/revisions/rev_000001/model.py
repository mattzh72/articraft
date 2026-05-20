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
    ExtrudeGeometry,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _profile_at_z(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _rounded_shell(
    width_bottom: float,
    depth_bottom: float,
    width_top: float,
    depth_top: float,
    height: float,
    radius_bottom: float,
    radius_top: float,
) -> LoftGeometry:
    bottom = rounded_rect_profile(width_bottom, depth_bottom, radius_bottom, corner_segments=12)
    top = rounded_rect_profile(width_top, depth_top, radius_top, corner_segments=12)
    return LoftGeometry(
        [_profile_at_z(bottom, 0.0), _profile_at_z(top, height)],
        cap=True,
        closed=True,
    )


def _rounded_panel(width: float, depth: float, radius: float, height: float) -> ExtrudeGeometry:
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, radius, corner_segments=12),
        height,
        cap=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_tackle_box", assets=ASSETS)

    olive_polymer = Material("olive_polymer", (0.30, 0.34, 0.23, 1.0))
    charcoal_trim = Material("charcoal_trim", (0.13, 0.14, 0.15, 1.0))
    rubber_black = Material("rubber_black", (0.05, 0.05, 0.05, 1.0))
    safety_orange = Material("safety_orange", (0.86, 0.43, 0.13, 1.0))
    brushed_steel = Material("brushed_steel", (0.74, 0.75, 0.77, 1.0))
    smoky_polycarbonate = Material("smoky_polycarbonate", (0.24, 0.28, 0.31, 0.45))
    model.materials.extend(
        [
            olive_polymer,
            charcoal_trim,
            rubber_black,
            safety_orange,
            brushed_steel,
            smoky_polycarbonate,
        ]
    )

    body_width_bottom = 0.344
    body_depth_bottom = 0.206
    body_width_top = 0.336
    body_depth_top = 0.200
    body_height = 0.112

    lid_width_bottom = 0.332
    lid_depth_bottom = 0.196
    lid_width_top = 0.326
    lid_depth_top = 0.190
    lid_height = 0.044
    hinge_y = -0.099
    hinge_z = 0.113

    body = model.part("body")
    body_shell_mesh = mesh_from_geometry(
        _rounded_shell(
            body_width_bottom,
            body_depth_bottom,
            body_width_top,
            body_depth_top,
            body_height,
            radius_bottom=0.022,
            radius_top=0.018,
        ),
        ASSETS.mesh_path("tackle_box_body_shell.obj"),
    )
    body.visual(body_shell_mesh, material=olive_polymer, name="body_shell")
    body.visual(
        Box((0.228, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.101, 0.060)),
        material=charcoal_trim,
        name="right_side_rib",
    )
    body.visual(
        Box((0.228, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.101, 0.060)),
        material=charcoal_trim,
        name="left_side_rib",
    )
    body.visual(
        Box((0.112, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.052, 0.006)),
        material=rubber_black,
        name="front_skid_rail",
    )
    body.visual(
        Box((0.112, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, -0.052, 0.006)),
        material=rubber_black,
        name="rear_skid_rail",
    )
    for index, x_sign in enumerate((-1.0, 1.0), start=1):
        for side, y_center in (("rear", -0.082), ("front", 0.082)):
            body.visual(
                Box((0.030, 0.022, 0.022)),
                origin=Origin(xyz=(x_sign * 0.148, y_center, 0.038)),
                material=rubber_black,
                name=f"{side}_corner_bumper_{index}",
            )
    body.visual(
        Box((0.190, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.052)),
        material=charcoal_trim,
        name="front_reinforcement",
    )
    body.visual(
        Box((0.050, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.111, 0.081)),
        material=charcoal_trim,
        name="lower_latch_body",
    )
    body.visual(
        Box((0.028, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.120, 0.082)),
        material=safety_orange,
        name="latch_insert",
    )
    for index, x_center in enumerate((-0.118, 0.118), start=1):
        body.visual(
            Box((0.050, 0.010, 0.016)),
            origin=Origin(xyz=(x_center, -0.102, 0.104)),
            material=charcoal_trim,
            name=f"hinge_pad_{index}",
        )
        body.visual(
            Cylinder(radius=0.004, length=0.030),
            origin=Origin(xyz=(x_center, -0.100, 0.106), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"hinge_pin_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.344, 0.206, 0.112)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    lid = model.part("lid")
    lid_shell_mesh = mesh_from_geometry(
        _rounded_shell(
            lid_width_bottom,
            lid_depth_bottom,
            lid_width_top,
            lid_depth_top,
            lid_height,
            radius_bottom=0.018,
            radius_top=0.015,
        ),
        ASSETS.mesh_path("tackle_box_lid_shell.obj"),
    )
    lid_frame_mesh = mesh_from_geometry(
        _rounded_panel(0.256, 0.122, 0.014, 0.006),
        ASSETS.mesh_path("tackle_box_lid_frame.obj"),
    )
    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.084, 0.100, 0.068),
                (-0.058, 0.100, 0.095),
                (0.058, 0.100, 0.095),
                (0.084, 0.100, 0.068),
            ],
            radius=0.007,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_path("tackle_box_handle.obj"),
    )
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.0, lid_depth_bottom / 2.0, 0.002)),
        material=olive_polymer,
        name="lid_shell",
    )
    lid.visual(
        lid_frame_mesh,
        origin=Origin(xyz=(0.0, 0.100, 0.046)),
        material=charcoal_trim,
        name="lid_top_frame",
    )
    lid.visual(
        Box((0.202, 0.086, 0.004)),
        origin=Origin(xyz=(0.0, 0.100, 0.049)),
        material=smoky_polycarbonate,
        name="organizer_window",
    )
    lid.visual(
        Box((0.020, 0.032, 0.022)),
        origin=Origin(xyz=(-0.084, 0.100, 0.057)),
        material=charcoal_trim,
        name="left_handle_foot",
    )
    lid.visual(
        Box((0.020, 0.032, 0.022)),
        origin=Origin(xyz=(0.084, 0.100, 0.057)),
        material=charcoal_trim,
        name="right_handle_foot",
    )
    lid.visual(handle_mesh, material=rubber_black, name="carry_handle")
    lid.visual(
        Box((0.250, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.192, 0.017)),
        material=charcoal_trim,
        name="front_lip",
    )
    lid.visual(
        Box((0.050, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, 0.198, 0.018)),
        material=safety_orange,
        name="upper_latch_release",
    )
    lid.visual(
        Box((0.046, 0.010, 0.014)),
        origin=Origin(xyz=(-0.114, 0.008, 0.016)),
        material=charcoal_trim,
        name="left_hinge_cover",
    )
    lid.visual(
        Box((0.046, 0.010, 0.014)),
        origin=Origin(xyz=(0.114, 0.008, 0.016)),
        material=charcoal_trim,
        name="right_hinge_cover",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.332, 0.196, 0.050)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.098, 0.027)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.005, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )

    with ctx.pose(lid_hinge=0.85):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.07)
        ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.12, max_penetration=0.0)

    with ctx.pose(lid_hinge=1.35):
        ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.18, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
