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
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _rect_section(width: float, depth: float, z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=7)]


def _rear_anchored_section(
    width: float,
    depth: float,
    z: float,
    radius: float,
    *,
    rear_x: float,
) -> list[tuple[float, float, float]]:
    x_shift = rear_x - (width * 0.5)
    return [
        (x + x_shift, y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=7)
    ]


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    lo, hi = aabb
    return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_roof_vent_tower", assets=ASSETS)

    body_matte = model.material("body_matte", rgba=(0.24, 0.25, 0.27, 1.0))
    frame_satin = model.material("frame_satin", rgba=(0.66, 0.68, 0.70, 1.0))
    hardware_satin = model.material("hardware_satin", rgba=(0.47, 0.49, 0.52, 1.0))
    seal_dark = model.material("seal_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    tower_body = model.part("tower_body")

    outer_shell = section_loft(
        [
            _rect_section(0.280, 0.204, 0.000, 0.014),
            _rect_section(0.255, 0.186, 0.118, 0.012),
            _rect_section(0.230, 0.170, 0.232, 0.010),
        ]
    )
    inner_void = section_loft(
        [
            _rect_section(0.252, 0.176, -0.010, 0.008),
            _rect_section(0.228, 0.158, 0.118, 0.007),
            _rect_section(0.204, 0.142, 0.248, 0.006),
        ]
    )
    shell_geom = boolean_difference(outer_shell, inner_void)
    outlet_cutter = BoxGeometry((0.165, 0.118, 0.080)).translate(-0.108, 0.0, 0.162)
    shell_geom = boolean_difference(shell_geom, outlet_cutter)
    tower_shell_mesh = mesh_from_geometry(shell_geom, ASSETS.mesh_path("vent_tower_shell.obj"))
    tower_body.visual(tower_shell_mesh, material=body_matte, name="tower_shell")

    flange_specs = (
        ("base_flange_front", (-0.151, 0.0, 0.003), (0.022, 0.252, 0.006)),
        ("base_flange_rear", (0.151, 0.0, 0.003), (0.022, 0.252, 0.006)),
        ("base_flange_left", (0.0, 0.119, 0.003), (0.300, 0.022, 0.006)),
        ("base_flange_right", (0.0, -0.119, 0.003), (0.300, 0.022, 0.006)),
    )
    for name, xyz, size in flange_specs:
        tower_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_satin,
            name=name,
        )

    top_frame_specs = (
        ("top_frame_front", (-0.103, 0.0, 0.235), (0.010, 0.156, 0.008)),
        ("top_frame_rear", (0.103, 0.0, 0.235), (0.010, 0.156, 0.008)),
        ("top_frame_left", (0.0, 0.077, 0.235), (0.192, 0.010, 0.008)),
        ("top_frame_right", (0.0, -0.077, 0.235), (0.192, 0.010, 0.008)),
    )
    for name, xyz, size in top_frame_specs:
        tower_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_satin,
            name=name,
        )

    outlet_frame_specs = (
        ("outlet_frame_top", (-0.120, 0.0, 0.206), (0.012, 0.134, 0.010)),
        ("outlet_frame_bottom", (-0.118, 0.0, 0.117), (0.014, 0.134, 0.012)),
        ("outlet_frame_left", (-0.119, 0.066, 0.161), (0.014, 0.012, 0.078)),
        ("outlet_frame_right", (-0.119, -0.066, 0.161), (0.014, 0.012, 0.078)),
    )
    for name, xyz, size in outlet_frame_specs:
        tower_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_satin,
            name=name,
        )

    seam_specs = (
        ("rear_seam_left", (0.123, 0.076, 0.116), (0.010, 0.008, 0.188)),
        ("rear_seam_right", (0.123, -0.076, 0.116), (0.010, 0.008, 0.188)),
    )
    for name, xyz, size in seam_specs:
        tower_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=hardware_satin,
            name=name,
        )

    hinge_mount_specs = (
        ("hinge_mount_left", (0.121, 0.056, 0.237), (0.030, 0.024, 0.016)),
        ("hinge_mount_right", (0.121, -0.056, 0.237), (0.030, 0.024, 0.016)),
    )
    for name, xyz, size in hinge_mount_specs:
        tower_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=hardware_satin,
            name=name,
        )

    hinge_barrel_specs = (
        ("hinge_barrel_left", (0.127, 0.058, 0.246)),
        ("hinge_barrel_right", (0.127, -0.058, 0.246)),
    )
    for name, xyz in hinge_barrel_specs:
        tower_body.visual(
            Cylinder(radius=0.006, length=0.040),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware_satin,
            name=name,
        )

    tower_body.inertial = Inertial.from_geometry(
        Box((0.330, 0.252, 0.240)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    weather_flap = model.part("weather_flap")

    flap_shell_geom = section_loft(
        [
            _rear_anchored_section(0.252, 0.188, -0.006, 0.010, rear_x=-0.024),
            _rear_anchored_section(0.244, 0.182, -0.001, 0.010, rear_x=-0.024),
            _rear_anchored_section(0.236, 0.172, 0.004, 0.009, rear_x=-0.024),
        ]
    )
    flap_shell_mesh = mesh_from_geometry(flap_shell_geom, ASSETS.mesh_path("weather_flap_shell.obj"))
    weather_flap.visual(flap_shell_mesh, material=body_matte, name="flap_shell")

    weather_flap.visual(
        Box((0.012, 0.178, 0.018)),
        origin=Origin(xyz=(-0.250, 0.0, -0.008)),
        material=frame_satin,
        name="front_drip",
    )
    for name, xyz in (
        ("side_return_left", (-0.126, 0.094, -0.006)),
        ("side_return_right", (-0.126, -0.094, -0.006)),
    ):
        weather_flap.visual(
            Box((0.212, 0.008, 0.014)),
            origin=Origin(xyz=xyz),
            material=frame_satin,
            name=name,
        )

    for name, xyz in (
        ("hinge_strap_left", (-0.014, 0.024, 0.000)),
        ("hinge_strap_right", (-0.014, -0.024, 0.000)),
    ):
        weather_flap.visual(
            Box((0.022, 0.018, 0.006)),
            origin=Origin(xyz=xyz),
            material=hardware_satin,
            name=name,
        )

    weather_flap.visual(
        Cylinder(radius=0.006, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_satin,
        name="center_knuckle",
    )
    weather_flap.inertial = Inertial.from_geometry(
        Box((0.266, 0.194, 0.030)),
        mass=1.1,
        origin=Origin(xyz=(-0.132, 0.0, -0.001)),
    )

    model.articulation(
        "tower_to_flap",
        ArticulationType.REVOLUTE,
        parent=tower_body,
        child=weather_flap,
        origin=Origin(xyz=(0.127, 0.0, 0.246)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower_body = object_model.get_part("tower_body")
    weather_flap = object_model.get_part("weather_flap")
    tower_to_flap = object_model.get_articulation("tower_to_flap")

    top_frame_front = tower_body.get_visual("top_frame_front")
    outlet_frame_top = tower_body.get_visual("outlet_frame_top")
    outlet_frame_left = tower_body.get_visual("outlet_frame_left")
    outlet_frame_right = tower_body.get_visual("outlet_frame_right")
    hinge_barrel_left = tower_body.get_visual("hinge_barrel_left")
    hinge_barrel_right = tower_body.get_visual("hinge_barrel_right")
    flap_shell = weather_flap.get_visual("flap_shell")
    front_drip = weather_flap.get_visual("front_drip")
    center_knuckle = weather_flap.get_visual("center_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=False, ignore_fixed=True)
    ctx.expect_contact(
        tower_body,
        weather_flap,
        elem_a=hinge_barrel_left,
        elem_b=center_knuckle,
        name="left_hinge_barrel_contacts_center_knuckle",
    )
    ctx.expect_contact(
        tower_body,
        weather_flap,
        elem_a=hinge_barrel_right,
        elem_b=center_knuckle,
        name="right_hinge_barrel_contacts_center_knuckle",
    )

    ctx.expect_gap(
        weather_flap,
        tower_body,
        axis="z",
        positive_elem=front_drip,
        negative_elem=outlet_frame_top,
        min_gap=0.001,
        max_gap=0.020,
        name="closed_front_drip_sits_just_above_outlet_frame",
    )
    ctx.expect_overlap(
        weather_flap,
        tower_body,
        axes="y",
        elem_a=front_drip,
        elem_b=outlet_frame_top,
        min_overlap=0.120,
        name="front_drip_covers_outlet_width",
    )
    ctx.expect_overlap(
        weather_flap,
        tower_body,
        axes="xy",
        elem_a=flap_shell,
        elem_b=top_frame_front,
        min_overlap=0.008,
        name="flap_reads_as_seated_over_top_frame",
    )

    tower_aabb = ctx.part_world_aabb(tower_body)
    flap_aabb = ctx.part_world_aabb(weather_flap)
    if tower_aabb is None or flap_aabb is None:
        ctx.fail("world_aabbs_available", "Expected measurable AABBs for tower body and weather flap.")
        return ctx.report()

    tower_dx, tower_dy, tower_dz = _aabb_dims(tower_aabb)
    flap_dx, flap_dy, flap_dz = _aabb_dims(flap_aabb)
    ctx.check(
        "tower_body_has_premium_vent_proportions",
        tower_dx > 0.26 and tower_dy > 0.18 and 0.22 < tower_dz < 0.27,
        (
            f"Unexpected tower proportions: dx={tower_dx:.4f}, "
            f"dy={tower_dy:.4f}, dz={tower_dz:.4f}."
        ),
    )
    ctx.check(
        "flap_projects_beyond_outlet_for_weather_cover",
        flap_dx > 0.24 and flap_dy > 0.17 and flap_dz > 0.015,
        (
            f"Unexpected flap envelope: dx={flap_dx:.4f}, "
            f"dy={flap_dy:.4f}, dz={flap_dz:.4f}."
        ),
    )

    left_barrel_aabb = ctx.part_element_world_aabb(tower_body, elem=hinge_barrel_left)
    right_barrel_aabb = ctx.part_element_world_aabb(tower_body, elem=hinge_barrel_right)
    center_knuckle_aabb = ctx.part_element_world_aabb(weather_flap, elem=center_knuckle)
    outlet_left_aabb = ctx.part_element_world_aabb(tower_body, elem=outlet_frame_left)
    outlet_right_aabb = ctx.part_element_world_aabb(tower_body, elem=outlet_frame_right)
    if (
        left_barrel_aabb is None
        or right_barrel_aabb is None
        or center_knuckle_aabb is None
        or outlet_left_aabb is None
        or outlet_right_aabb is None
    ):
        ctx.fail("named_feature_aabbs_available", "Expected all named hinge and outlet frame features to be measurable.")
        return ctx.report()

    left_center = _aabb_center(left_barrel_aabb)
    right_center = _aabb_center(right_barrel_aabb)
    knuckle_center = _aabb_center(center_knuckle_aabb)
    left_gap = left_barrel_aabb[0][1] - center_knuckle_aabb[1][1]
    right_gap = center_knuckle_aabb[0][1] - right_barrel_aabb[1][1]
    ctx.check(
        "hinge_hardware_is_axially_aligned",
        (
            abs(left_center[0] - knuckle_center[0]) < 0.0025
            and abs(left_center[2] - knuckle_center[2]) < 0.0025
            and abs(right_center[0] - knuckle_center[0]) < 0.0025
            and abs(right_center[2] - knuckle_center[2]) < 0.0025
        ),
        (
            "Body hinge barrels and flap knuckle do not share the same pivot axis: "
            f"left={left_center}, right={right_center}, knuckle={knuckle_center}."
        ),
    )
    ctx.check(
        "hinge_knuckle_sits_cleanly_between_body_barrels",
        0.0 <= left_gap <= 0.010 and 0.0 <= right_gap <= 0.010,
        (
            "Expected restrained gaps between the center knuckle and outer barrels, got "
            f"left_gap={left_gap:.4f}, right_gap={right_gap:.4f}."
        ),
    )

    outlet_left_center = _aabb_center(outlet_left_aabb)
    outlet_right_center = _aabb_center(outlet_right_aabb)
    ctx.check(
        "outlet_frame_is_balanced_around_centerline",
        abs(outlet_left_center[1] + outlet_right_center[1]) < 0.003,
        (
            "Outlet jambs are not balanced about the tower centerline: "
            f"left_y={outlet_left_center[1]:.4f}, right_y={outlet_right_center[1]:.4f}."
        ),
    )

    rest_front_drip_aabb = ctx.part_element_world_aabb(weather_flap, elem=front_drip)
    if rest_front_drip_aabb is None:
        ctx.fail("front_drip_aabb_available", "Expected front drip AABB at rest pose.")
        return ctx.report()
    rest_front_center = _aabb_center(rest_front_drip_aabb)

    with ctx.pose({tower_to_flap: math.radians(42.0)}):
        ctx.expect_gap(
            weather_flap,
            tower_body,
            axis="z",
            positive_elem=front_drip,
            negative_elem=outlet_frame_top,
            min_gap=0.070,
            name="open_flap_lifts_clear_of_outlet_frame",
        )
        open_knuckle_aabb = ctx.part_element_world_aabb(weather_flap, elem=center_knuckle)
        open_front_drip_aabb = ctx.part_element_world_aabb(weather_flap, elem=front_drip)
        if open_knuckle_aabb is None or open_front_drip_aabb is None:
            ctx.fail("posed_flap_feature_aabbs_available", "Expected posed hinge and flap features to remain measurable.")
            return ctx.report()
        open_knuckle_center = _aabb_center(open_knuckle_aabb)
        open_front_center = _aabb_center(open_front_drip_aabb)
        ctx.check(
            "pivot_stays_fixed_as_flap_rotates",
            abs(open_knuckle_center[0] - knuckle_center[0]) < 0.001
            and abs(open_knuckle_center[1] - knuckle_center[1]) < 0.001
            and abs(open_knuckle_center[2] - knuckle_center[2]) < 0.001,
            (
                "Center knuckle drifted away from the articulation origin during rotation: "
                f"rest={knuckle_center}, open={open_knuckle_center}."
            ),
        )
        ctx.check(
            "front_edge_rises_in_open_pose",
            open_front_center[2] > rest_front_center[2] + 0.070,
            (
                "Expected the front drip to lift significantly in the open pose, got "
                f"rest_z={rest_front_center[2]:.4f}, open_z={open_front_center[2]:.4f}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
