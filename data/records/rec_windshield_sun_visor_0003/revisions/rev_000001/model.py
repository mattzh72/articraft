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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_windshield_sun_visor", assets=ASSETS)

    headliner = model.material("headliner", rgba=(0.77, 0.78, 0.76, 1.0))
    visor_skin = model.material("visor_skin", rgba=(0.19, 0.20, 0.21, 1.0))
    visor_back = model.material("visor_back", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_trim = model.material("satin_trim", rgba=(0.42, 0.44, 0.46, 1.0))
    pivot_metal = model.material("pivot_metal", rgba=(0.33, 0.35, 0.37, 1.0))
    lens = model.material("lens", rgba=(0.92, 0.92, 0.88, 0.85))
    mirror_glass = model.material("mirror_glass", rgba=(0.58, 0.65, 0.69, 0.32))
    bezel_dark = model.material("bezel_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    visor_width = 0.365
    visor_height = 0.166
    visor_outer_radius = 0.020
    visor_attach_x = 0.170
    visor_attach_y = -0.072

    mirror_center_x = 0.218
    mirror_center_y = -0.082

    outer_profile = _shift_profile(
        rounded_rect_profile(visor_width, visor_height, visor_outer_radius),
        dx=visor_attach_x,
        dy=visor_attach_y,
    )
    front_outer_profile = _shift_profile(
        rounded_rect_profile(0.356, 0.158, 0.017),
        dx=visor_attach_x,
        dy=visor_attach_y,
    )
    rear_outer_profile = _shift_profile(
        rounded_rect_profile(0.350, 0.152, 0.016),
        dx=visor_attach_x,
        dy=visor_attach_y,
    )
    core_hole_profile = _shift_profile(
        rounded_rect_profile(0.224, 0.096, 0.010),
        dx=mirror_center_x,
        dy=mirror_center_y,
    )
    front_hole_profile = _shift_profile(
        rounded_rect_profile(0.206, 0.084, 0.009),
        dx=mirror_center_x,
        dy=mirror_center_y,
    )

    visor_body_mesh = _save_mesh(
        "visor_body.obj",
        ExtrudeWithHolesGeometry(
            outer_profile,
            [core_hole_profile],
            0.016,
            center=True,
        ),
    )
    visor_front_mesh = _save_mesh(
        "visor_front_skin.obj",
        ExtrudeWithHolesGeometry(
            front_outer_profile,
            [front_hole_profile],
            0.003,
            center=True,
        ),
    )
    visor_rear_mesh = _save_mesh(
        "visor_rear_skin.obj",
        ExtrudeGeometry.centered(rear_outer_profile, 0.003),
    )
    vanity_cover_mesh = _save_mesh(
        "vanity_cover_panel.obj",
        ExtrudeGeometry.centered(
            _shift_profile(rounded_rect_profile(0.202, 0.080, 0.008), dy=-0.040),
            0.003,
        ),
    )

    roof_header = model.part("roof_header")
    roof_header.visual(
        Box((0.500, 0.180, 0.024)),
        origin=Origin(xyz=(0.220, -0.030, 0.012)),
        material=headliner,
        name="header_trim",
    )
    roof_header.visual(
        Box((0.084, 0.050, 0.004)),
        origin=Origin(xyz=(0.012, 0.000, -0.002)),
        material=satin_trim,
        name="mount_reinforcement",
    )
    roof_header.visual(
        Box((0.050, 0.030, 0.003)),
        origin=Origin(xyz=(0.442, -0.098, -0.0015)),
        material=satin_trim,
        name="clip_reinforcement",
    )
    roof_header.inertial = Inertial.from_geometry(
        Box((0.500, 0.180, 0.024)),
        mass=1.2,
        origin=Origin(xyz=(0.220, -0.030, 0.012)),
    )

    mount_bracket = model.part("mount_bracket")
    mount_bracket.visual(
        Box((0.074, 0.040, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, -0.003)),
        material=satin_trim,
        name="cap_plate",
    )
    mount_bracket.visual(
        Box((0.010, 0.028, 0.018)),
        origin=Origin(xyz=(-0.017, 0.000, -0.012)),
        material=visor_skin,
        name="left_cheek",
    )
    mount_bracket.visual(
        Box((0.010, 0.028, 0.018)),
        origin=Origin(xyz=(0.017, 0.000, -0.012)),
        material=visor_skin,
        name="right_cheek",
    )
    mount_bracket.visual(
        Box((0.034, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, -0.011, -0.012)),
        material=visor_skin,
        name="rear_frame",
    )
    mount_bracket.visual(
        Box((0.034, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.011, -0.016)),
        material=visor_skin,
        name="front_frame",
    )
    mount_bracket.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, -0.008)),
        material=pivot_metal,
        name="pivot_socket",
    )
    mount_bracket.inertial = Inertial.from_geometry(
        Box((0.074, 0.040, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.000, -0.011)),
    )

    retainer_clip = model.part("retainer_clip")
    retainer_clip.visual(
        Box((0.036, 0.020, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, -0.002)),
        material=satin_trim,
        name="clip_cap",
    )
    retainer_clip.visual(
        Box((0.010, 0.012, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, -0.011)),
        material=visor_skin,
        name="clip_stem",
    )
    retainer_clip.visual(
        Box((0.024, 0.012, 0.004)),
        origin=Origin(xyz=(0.000, -0.002, -0.020)),
        material=visor_skin,
        name="clip_saddle",
    )
    retainer_clip.visual(
        Box((0.018, 0.004, 0.010)),
        origin=Origin(xyz=(0.000, 0.008, -0.011)),
        material=visor_skin,
        name="clip_guard",
    )
    retainer_clip.inertial = Inertial.from_geometry(
        Box((0.036, 0.020, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
    )

    swivel_knuckle = model.part("swivel_knuckle")
    swivel_knuckle.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, -0.009)),
        material=pivot_metal,
        name="pivot_stem",
    )
    swivel_knuckle.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.018)),
        material=pivot_metal,
        name="pivot_body",
    )
    swivel_knuckle.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=visor_skin,
        name="fold_boss",
    )
    swivel_knuckle.inertial = Inertial.from_geometry(
        Box((0.018, 0.014, 0.030)),
        mass=0.03,
        origin=Origin(xyz=(0.000, 0.000, -0.015)),
    )

    hinge_arm = model.part("hinge_arm")
    hinge_arm.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.016, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="fold_tube",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0042, length=0.070),
        origin=Origin(xyz=(0.053, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="support_rod",
    )
    hinge_arm.visual(
        Cylinder(radius=0.0050, length=0.016),
        origin=Origin(xyz=(0.096, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_trim,
        name="transition_collar",
    )
    hinge_arm.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(0.106, 0.000, 0.000)),
        material=visor_skin,
        name="panel_spigot",
    )
    hinge_arm.inertial = Inertial.from_geometry(
        Box((0.122, 0.016, 0.010)),
        mass=0.04,
        origin=Origin(xyz=(0.061, 0.000, 0.000)),
    )

    visor_panel = model.part("visor_panel")
    visor_panel.visual(
        visor_body_mesh,
        material=visor_back,
        name="panel_body",
    )
    visor_panel.visual(
        visor_front_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.0095)),
        material=visor_skin,
        name="front_skin",
    )
    visor_panel.visual(
        visor_rear_mesh,
        origin=Origin(xyz=(0.000, 0.000, -0.0095)),
        material=visor_back,
        name="rear_skin",
    )
    visor_panel.visual(
        Box((0.196, 0.008, 0.003)),
        origin=Origin(xyz=(0.218, -0.038, 0.0095)),
        material=bezel_dark,
        name="upper_seam_rail",
    )
    visor_panel.visual(
        Box((0.014, 0.014, 0.006)),
        origin=Origin(xyz=(-0.0055, -0.004, 0.000)),
        material=bezel_dark,
        name="arm_receiver",
    )
    visor_panel.visual(
        Box((0.022, 0.010, 0.003)),
        origin=Origin(xyz=(0.320, -0.100, 0.0150)),
        material=bezel_dark,
        name="clip_landing",
    )
    visor_panel.visual(
        Box((0.012, 0.018, 0.0055)),
        origin=Origin(xyz=(0.320, -0.100, 0.01075)),
        material=bezel_dark,
        name="clip_landing_pedestal",
    )
    visor_panel.visual(
        Box((0.194, 0.010, 0.003)),
        origin=Origin(xyz=(0.218, -0.126, 0.0095)),
        material=satin_trim,
        name="frame_lower_rail",
    )
    visor_panel.visual(
        Box((0.010, 0.078, 0.003)),
        origin=Origin(xyz=(0.120, -0.082, 0.0095)),
        material=satin_trim,
        name="frame_left_rail",
    )
    visor_panel.visual(
        Box((0.010, 0.078, 0.003)),
        origin=Origin(xyz=(0.316, -0.082, 0.0095)),
        material=satin_trim,
        name="frame_right_rail",
    )
    visor_panel.visual(
        Box((0.206, 0.084, 0.002)),
        origin=Origin(xyz=(0.218, -0.082, 0.0090)),
        material=bezel_dark,
        name="mirror_tray",
    )
    visor_panel.visual(
        Box((0.016, 0.060, 0.0015)),
        origin=Origin(xyz=(0.134, -0.082, 0.0090)),
        material=lens,
        name="left_map_light",
    )
    visor_panel.visual(
        Box((0.140, 0.056, 0.001)),
        origin=Origin(xyz=(0.218, -0.082, 0.0090)),
        material=mirror_glass,
        name="mirror_glass",
    )
    visor_panel.visual(
        Box((0.016, 0.060, 0.0015)),
        origin=Origin(xyz=(0.302, -0.082, 0.0090)),
        material=lens,
        name="right_map_light",
    )
    visor_panel.visual(
        Box((0.130, 0.006, 0.004)),
        origin=Origin(xyz=(0.218, -0.040, 0.0110)),
        material=bezel_dark,
        name="cover_hinge_rail",
    )
    visor_panel.inertial = Inertial.from_geometry(
        Box((0.365, 0.166, 0.020)),
        mass=0.62,
        origin=Origin(xyz=(0.170, -0.072, 0.000)),
    )

    vanity_cover = model.part("vanity_cover")
    vanity_cover.visual(
        vanity_cover_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=visor_skin,
        name="cover_panel",
    )
    vanity_cover.visual(
        Box((0.040, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, -0.078, 0.0035)),
        material=satin_trim,
        name="pull_lip",
    )
    vanity_cover.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(-0.072, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="left_cover_hinge",
    )
    vanity_cover.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.072, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_metal,
        name="right_cover_hinge",
    )
    vanity_cover.inertial = Inertial.from_geometry(
        Box((0.202, 0.084, 0.005)),
        mass=0.03,
        origin=Origin(xyz=(0.000, -0.040, 0.0015)),
    )

    model.articulation(
        "roof_to_mount_bracket",
        ArticulationType.FIXED,
        parent=roof_header,
        child=mount_bracket,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
    )
    model.articulation(
        "roof_to_retainer_clip",
        ArticulationType.FIXED,
        parent=roof_header,
        child=retainer_clip,
        origin=Origin(xyz=(0.442, -0.098, 0.000)),
    )
    model.articulation(
        "bracket_to_swivel",
        ArticulationType.REVOLUTE,
        parent=mount_bracket,
        child=swivel_knuckle,
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "swivel_to_hinge_arm",
        ArticulationType.REVOLUTE,
        parent=swivel_knuckle,
        child=hinge_arm,
        origin=Origin(xyz=(0.000, 0.000, -0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "arm_to_visor_panel",
        ArticulationType.FIXED,
        parent=hinge_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.1225, 0.000, -0.0045)),
    )
    model.articulation(
        "visor_to_vanity_cover",
        ArticulationType.REVOLUTE,
        parent=visor_panel,
        child=vanity_cover,
        origin=Origin(xyz=(0.218, -0.040, 0.0110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    roof_header = object_model.get_part("roof_header")
    mount_bracket = object_model.get_part("mount_bracket")
    retainer_clip = object_model.get_part("retainer_clip")
    swivel_knuckle = object_model.get_part("swivel_knuckle")
    hinge_arm = object_model.get_part("hinge_arm")
    visor_panel = object_model.get_part("visor_panel")
    vanity_cover = object_model.get_part("vanity_cover")

    pivot_body = swivel_knuckle.get_visual("pivot_body")
    fold_tube = hinge_arm.get_visual("fold_tube")
    panel_spigot = hinge_arm.get_visual("panel_spigot")
    arm_receiver = visor_panel.get_visual("arm_receiver")
    panel_body = visor_panel.get_visual("panel_body")
    mirror_tray = visor_panel.get_visual("mirror_tray")
    mirror_glass = visor_panel.get_visual("mirror_glass")
    left_map_light = visor_panel.get_visual("left_map_light")
    right_map_light = visor_panel.get_visual("right_map_light")
    cover_panel = vanity_cover.get_visual("cover_panel")

    bracket_to_swivel = object_model.get_articulation("bracket_to_swivel")
    swivel_to_hinge_arm = object_model.get_articulation("swivel_to_hinge_arm")
    visor_to_vanity_cover = object_model.get_articulation("visor_to_vanity_cover")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        roof_header,
        mount_bracket,
        reason="The roof bracket cap is embedded flush into the reinforced headliner mount pad.",
    )
    ctx.allow_overlap(
        swivel_knuckle,
        hinge_arm,
        reason="The folding hinge tube runs through the knuckle body as a captured pivot.",
        elem_a=pivot_body,
        elem_b=fold_tube,
    )
    ctx.allow_overlap(
        hinge_arm,
        visor_panel,
        reason="The hinge arm spigot is keyed into the visor receiver pocket.",
        elem_a=panel_spigot,
        elem_b=arm_receiver,
    )
    ctx.allow_overlap(
        vanity_cover,
        visor_panel,
        reason="The vanity cover nests inside the recessed vanity opening and shares the hinge rail envelope.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "secondary_pivot_axis_vertical",
        tuple(bracket_to_swivel.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical side-swing axis, got {bracket_to_swivel.axis}",
    )
    ctx.check(
        "main_fold_axis_longitudinal",
        tuple(swivel_to_hinge_arm.axis) == (1.0, 0.0, 0.0),
        f"Expected visor fold axis along x, got {swivel_to_hinge_arm.axis}",
    )
    ctx.check(
        "cover_hinge_axis_longitudinal",
        tuple(visor_to_vanity_cover.axis) == (1.0, 0.0, 0.0),
        f"Expected vanity cover hinge axis along x, got {visor_to_vanity_cover.axis}",
    )

    ctx.expect_contact(mount_bracket, roof_header, name="mount_bracket_attached")
    ctx.expect_contact(retainer_clip, roof_header, name="retainer_clip_attached")
    ctx.expect_overlap(
        retainer_clip,
        roof_header,
        axes="xy",
        min_overlap=0.018,
        name="retainer_clip_on_header_pad",
    )
    ctx.expect_origin_distance(
        swivel_knuckle,
        mount_bracket,
        axes="xy",
        max_dist=0.002,
        name="swivel_centered_in_bracket",
    )
    ctx.expect_origin_gap(
        mount_bracket,
        swivel_knuckle,
        axis="z",
        min_gap=0.006,
        max_gap=0.012,
        name="swivel_below_bracket",
    )
    ctx.expect_contact(
        hinge_arm,
        swivel_knuckle,
        elem_a=fold_tube,
        elem_b=pivot_body,
        name="hinge_arm_captured",
    )
    ctx.expect_contact(
        hinge_arm,
        visor_panel,
        elem_a=panel_spigot,
        elem_b=arm_receiver,
        name="visor_panel_supported",
    )
    ctx.expect_within(
        visor_panel,
        visor_panel,
        axes="xy",
        inner_elem=mirror_tray,
        outer_elem=panel_body,
        margin=0.012,
        name="mirror_tray_within_panel",
    )
    ctx.expect_within(
        visor_panel,
        visor_panel,
        axes="xy",
        inner_elem=mirror_glass,
        outer_elem=mirror_tray,
        margin=0.004,
        name="mirror_glass_within_tray",
    )
    ctx.expect_within(
        visor_panel,
        visor_panel,
        axes="xy",
        inner_elem=left_map_light,
        outer_elem=mirror_tray,
        margin=0.004,
        name="left_map_light_within_tray",
    )
    ctx.expect_within(
        visor_panel,
        visor_panel,
        axes="xy",
        inner_elem=right_map_light,
        outer_elem=mirror_tray,
        margin=0.004,
        name="right_map_light_within_tray",
    )
    ctx.expect_gap(
        vanity_cover,
        visor_panel,
        axis="z",
        positive_elem=cover_panel,
        negative_elem=panel_body,
        min_gap=0.002,
        max_gap=0.004,
        name="cover_closed_clearance_to_body",
    )
    ctx.expect_within(
        vanity_cover,
        visor_panel,
        axes="xy",
        inner_elem=cover_panel,
        outer_elem=panel_body,
        margin=0.012,
        name="cover_within_vanity_frame",
    )
    ctx.expect_gap(
        retainer_clip,
        visor_panel,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="retainer_clip_stowed_gap",
    )
    ctx.expect_overlap(
        retainer_clip,
        visor_panel,
        axes="xy",
        min_overlap=0.018,
        name="retainer_clip_aligned_to_landing",
    )
    ctx.expect_gap(
        roof_header,
        visor_panel,
        axis="z",
        min_gap=0.018,
        max_gap=0.026,
        name="stowed_clearance_below_header",
    )
    ctx.expect_overlap(
        visor_panel,
        roof_header,
        axes="xy",
        min_overlap=0.030,
        name="stowed_panel_under_header",
    )

    fold_upper = swivel_to_hinge_arm.motion_limits.upper
    swing_upper = bracket_to_swivel.motion_limits.upper
    cover_upper = visor_to_vanity_cover.motion_limits.upper
    assert fold_upper is not None
    assert swing_upper is not None
    assert cover_upper is not None

    with ctx.pose({swivel_to_hinge_arm: fold_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="visor_down_no_overlap")
        ctx.fail_if_isolated_parts(name="visor_down_no_floating")
        down_aabb = ctx.part_world_aabb(visor_panel)
        assert down_aabb is not None
        down_dx = down_aabb[1][0] - down_aabb[0][0]
        down_dy = down_aabb[1][1] - down_aabb[0][1]
        down_dz = down_aabb[1][2] - down_aabb[0][2]
        ctx.check(
            "visor_down_reads_vertical",
            down_dz > 0.14 and down_dy < 0.07 and down_dx > 0.30,
            f"Unexpected down-pose extents dx={down_dx:.3f}, dy={down_dy:.3f}, dz={down_dz:.3f}",
        )
        ctx.expect_gap(
            roof_header,
            visor_panel,
            axis="z",
            min_gap=0.015,
            name="visor_down_still_below_header",
        )

    with ctx.pose({swivel_to_hinge_arm: fold_upper, bracket_to_swivel: swing_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="visor_side_swing_no_overlap")
        ctx.fail_if_isolated_parts(name="visor_side_swing_no_floating")
        side_aabb = ctx.part_world_aabb(visor_panel)
        assert side_aabb is not None
        side_dx = side_aabb[1][0] - side_aabb[0][0]
        side_dy = side_aabb[1][1] - side_aabb[0][1]
        side_dz = side_aabb[1][2] - side_aabb[0][2]
        ctx.check(
            "visor_side_window_orientation",
            side_dy > 0.30 and side_dx < 0.08 and side_dz > 0.14,
            f"Unexpected side-swing extents dx={side_dx:.3f}, dy={side_dy:.3f}, dz={side_dz:.3f}",
        )

    with ctx.pose({swivel_to_hinge_arm: fold_upper}):
        cover_closed_aabb = ctx.part_world_aabb(vanity_cover)
        assert cover_closed_aabb is not None
        cover_lower = visor_to_vanity_cover.motion_limits.lower
        assert cover_lower is not None
        with ctx.pose({visor_to_vanity_cover: cover_lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="cover_open_no_overlap")
            ctx.fail_if_isolated_parts(name="cover_open_no_floating")
            cover_open_aabb = ctx.part_world_aabb(vanity_cover)
            assert cover_open_aabb is not None
            open_dy = cover_open_aabb[1][1] - cover_open_aabb[0][1]
            ctx.check(
                "cover_opens_forward",
                cover_open_aabb[0][1] < cover_closed_aabb[0][1] - 0.040 and open_dy > 0.070,
                (
                    "Vanity cover did not swing out of the recessed opening enough: "
                    f"closed_min_y={cover_closed_aabb[0][1]:.3f}, "
                    f"open_min_y={cover_open_aabb[0][1]:.3f}, "
                    f"open_dy={open_dy:.3f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
