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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _yz_rounded_section(
    x: float,
    width: float,
    height: float,
    z_shift: float = 0.0,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    radius = min(width, height) * 0.24
    return [
        (x, y, z + z_shift)
        for y, z in rounded_rect_profile(
            width,
            height,
            radius=radius,
            corner_segments=corner_segments,
        )
    ]


def _build_handle_shell_mesh():
    sections = [
        (-0.084, -0.0105, 0.0075, 0.026),
        (-0.067, -0.0115, 0.0095, 0.031),
        (-0.040, -0.0122, 0.0118, 0.036),
        (-0.006, -0.0115, 0.0135, 0.037),
        (0.026, -0.0104, 0.0136, 0.035),
        (0.050, -0.0068, 0.0112, 0.024),
    ]
    shell_geom = superellipse_side_loft(
        sections,
        exponents=(2.8, 3.0, 3.2, 3.2, 3.0, 2.7),
        segments=56,
        cap=True,
        closed=True,
    ).rotate_z(-pi / 2.0)
    return mesh_from_geometry(shell_geom, ASSETS.mesh_path("utility_knife_handle_shell.obj"))


def _build_side_grip_mesh():
    panel_profile = [
        (-0.058, -0.0065),
        (-0.026, -0.0092),
        (0.022, -0.0058),
        (0.030, 0.0008),
        (0.006, 0.0085),
        (-0.045, 0.0073),
    ]
    panel_geom = ExtrudeGeometry.centered(
        panel_profile,
        0.0022,
        cap=True,
        closed=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(panel_geom, ASSETS.mesh_path("utility_knife_side_grip.obj"))


def _build_blade_mesh():
    outer_profile = [
        (0.002, 0.0038),
        (0.040, 0.0038),
        (0.055, 0.0012),
        (0.051, -0.0054),
        (0.006, -0.0054),
    ]
    slot_profile = [
        (0.018, 0.0014),
        (0.032, 0.0014),
        (0.032, -0.0012),
        (0.018, -0.0012),
    ]
    blade_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [slot_profile],
        height=0.0009,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(blade_geom, ASSETS.mesh_path("utility_knife_blade.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife", assets=ASSETS)

    body_yellow = model.material("body_yellow", rgba=(0.90, 0.75, 0.16, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))
    slider_black = model.material("slider_black", rgba=(0.14, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.44, 0.47, 0.51, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.84, 0.86, 0.88, 1.0))

    handle_shell_mesh = _build_handle_shell_mesh()
    side_grip_mesh = _build_side_grip_mesh()
    blade_mesh = _build_blade_mesh()

    handle = model.part("handle")
    handle.visual(handle_shell_mesh, material=body_yellow)
    handle.visual(
        Box((0.010, 0.022, 0.016)),
        origin=Origin(xyz=(-0.082, 0.0, -0.0015)),
        material=steel,
        name="rear_cap",
    )
    handle.visual(
        Box((0.097, 0.010, 0.003)),
        origin=Origin(xyz=(0.000, 0.0, 0.0115)),
        material=grip_black,
        name="track_insert",
    )
    handle.visual(
        Box((0.096, 0.003, 0.003)),
        origin=Origin(xyz=(0.000, 0.0060, 0.0136)),
        material=steel,
        name="left_track_rail",
    )
    handle.visual(
        Box((0.096, 0.003, 0.003)),
        origin=Origin(xyz=(0.000, -0.0060, 0.0136)),
        material=steel,
        name="right_track_rail",
    )
    handle.visual(
        Box((0.034, 0.020, 0.006)),
        origin=Origin(xyz=(0.069, 0.0, 0.0020)),
        material=steel,
        name="nose_shoe",
    )
    handle.visual(
        Box((0.030, 0.003, 0.012)),
        origin=Origin(xyz=(0.068, 0.0085, 0.0080)),
        material=steel,
        name="nose_left_cheek",
    )
    handle.visual(
        Box((0.030, 0.003, 0.012)),
        origin=Origin(xyz=(0.068, -0.0085, 0.0080)),
        material=steel,
        name="nose_right_cheek",
    )
    handle.visual(
        Box((0.030, 0.003, 0.003)),
        origin=Origin(xyz=(0.069, 0.0060, 0.0136)),
        material=steel,
        name="nose_left_top_guide",
    )
    handle.visual(
        Box((0.030, 0.003, 0.003)),
        origin=Origin(xyz=(0.069, -0.0060, 0.0136)),
        material=steel,
        name="nose_right_top_guide",
    )
    handle.visual(
        side_grip_mesh,
        origin=Origin(xyz=(0.0, 0.0167, 0.0)),
        material=grip_black,
        name="left_grip_panel",
    )
    handle.visual(
        side_grip_mesh,
        origin=Origin(xyz=(0.0, -0.0167, 0.0)),
        material=grip_black,
        name="right_grip_panel",
    )
    handle.visual(
        Box((0.032, 0.012, 0.0025)),
        origin=Origin(xyz=(-0.043, 0.0, 0.0117)),
        material=grip_black,
        name="rear_spine_pad",
    )
    for x_pos, z_pos in [(-0.040, 0.0020), (0.006, -0.0012)]:
        handle.visual(
            Cylinder(radius=0.0024, length=0.0018),
            origin=Origin(xyz=(x_pos, 0.0171, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"left_fastener_{x_pos:.3f}",
        )
        handle.visual(
            Cylinder(radius=0.0024, length=0.0018),
            origin=Origin(xyz=(x_pos, -0.0171, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"right_fastener_{x_pos:.3f}",
        )
    handle.inertial = Inertial.from_geometry(
        Box((0.172, 0.038, 0.026)),
        mass=0.24,
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.072, 0.014, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
        material=dark_steel,
        name="carrier_sled",
    )
    blade_carrier.visual(
        Box((0.036, 0.0032, 0.004)),
        origin=Origin(xyz=(-0.002, 0.0052, 0.0030)),
        material=steel,
        name="carrier_left_runner",
    )
    blade_carrier.visual(
        Box((0.036, 0.0032, 0.004)),
        origin=Origin(xyz=(-0.002, -0.0052, 0.0030)),
        material=steel,
        name="carrier_right_runner",
    )
    blade_carrier.visual(
        Box((0.026, 0.016, 0.008)),
        origin=Origin(xyz=(0.029, 0.0, -0.0010)),
        material=steel,
        name="blade_clamp",
    )
    blade_carrier.visual(
        Box((0.018, 0.0025, 0.006)),
        origin=Origin(xyz=(0.021, 0.0055, 0.0005)),
        material=steel,
        name="clamp_left_cheek",
    )
    blade_carrier.visual(
        Box((0.018, 0.0025, 0.006)),
        origin=Origin(xyz=(0.021, -0.0055, 0.0005)),
        material=steel,
        name="clamp_right_cheek",
    )
    blade_carrier.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0075)),
        material=slider_black,
        name="thumb_slider_base",
    )
    blade_carrier.visual(
        Box((0.016, 0.008, 0.003)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0110)),
        material=slider_black,
        name="thumb_slider_top",
    )
    for x_pos in (-0.011, -0.006, -0.001):
        blade_carrier.visual(
            Box((0.0022, 0.0084, 0.0012)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0130)),
            material=steel,
            name=f"thumb_grip_rib_{x_pos:.3f}",
        )
    blade_carrier.visual(blade_mesh, material=blade_steel, name="utility_blade")
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.090, 0.018, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.010, 0.0, 0.000)),
    )

    model.articulation(
        "blade_slide",
        ArticulationType.PRISMATIC,
        parent="handle",
        child="blade_carrier",
        origin=Origin(xyz=(0.000, 0.0, 0.0080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.25,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(
        object_model,
        asset_root=HERE,
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "blade_slide",
        "blade_carrier",
        world_axis="x",
        direction="positive",
        min_delta=0.025,
    )
    ctx.expect_aabb_contact("handle", "blade_carrier")
    ctx.expect_aabb_overlap("blade_carrier", "handle", axes="yz", min_overlap=0.010)
    ctx.expect_origin_distance("blade_carrier", "handle", axes="y", max_dist=0.001)
    ctx.expect_origin_distance("blade_carrier", "handle", axes="z", max_dist=0.009)

    with ctx.pose(blade_slide=0.0):
        ctx.expect_aabb_contact("handle", "blade_carrier")
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="x", min_overlap=0.085)
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="yz", min_overlap=0.010)
        ctx.expect_origin_distance("blade_carrier", "handle", axes="x", max_dist=0.005)

    with ctx.pose(blade_slide=0.028):
        ctx.expect_aabb_contact("handle", "blade_carrier")
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="x", min_overlap=0.080)
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="yz", min_overlap=0.010)
        ctx.expect_origin_distance("blade_carrier", "handle", axes="y", max_dist=0.001)

    with ctx.pose(blade_slide=0.055):
        ctx.expect_aabb_contact("handle", "blade_carrier")
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="x", min_overlap=0.065)
        ctx.expect_aabb_overlap("blade_carrier", "handle", axes="yz", min_overlap=0.010)
        ctx.expect_origin_distance("blade_carrier", "handle", axes="y", max_dist=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
