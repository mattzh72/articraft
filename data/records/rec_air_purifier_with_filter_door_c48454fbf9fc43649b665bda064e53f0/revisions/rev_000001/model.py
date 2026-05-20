from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cut_opening_on_face,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_housing_shell_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    wall: float,
    outer_radius: float,
    grille_width: float,
    grille_height: float,
    grille_bottom: float,
    tray_slot_width: float,
    tray_slot_height: float,
    tray_slot_bottom: float,
):
    outer_profile = rounded_rect_profile(
        width,
        depth,
        outer_radius,
        corner_segments=10,
    )
    inner_profile = rounded_rect_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        max(outer_radius - wall, wall * 1.2),
        corner_segments=10,
    )
    shell = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        height,
        cap=False,
        center=True,
        closed=True,
    )
    shell = cut_opening_on_face(
        shell,
        face="+y",
        opening_profile=rounded_rect_profile(
            grille_width,
            grille_height,
            min(0.014, grille_height * 0.18, grille_width * 0.12),
            corner_segments=8,
        ),
        depth=wall * 2.2,
        offset=(0.0, grille_bottom + grille_height * 0.5 - height * 0.5),
    )
    shell = cut_opening_on_face(
        shell,
        face="+y",
        opening_profile=rounded_rect_profile(
            tray_slot_width,
            tray_slot_height,
            min(0.009, tray_slot_height * 0.35, tray_slot_width * 0.08),
            corner_segments=8,
        ),
        depth=wall * 2.2,
        offset=(0.0, tray_slot_bottom + tray_slot_height * 0.5 - height * 0.5),
    )
    return mesh_from_geometry(shell, "bedside_purifier_shell")


def _build_slotted_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    outer_radius: float,
    frame_margin_x: float,
    frame_margin_y: float,
    slot_count: int,
    slot_gap: float,
    mesh_name: str,
):
    outer_profile = rounded_rect_profile(
        width,
        height,
        outer_radius,
        corner_segments=8,
    )
    usable_height = height - 2.0 * frame_margin_y
    slot_height = (usable_height - slot_gap * (slot_count - 1)) / slot_count
    slot_width = width - 2.0 * frame_margin_x
    slot_radius = min(slot_height * 0.42, 0.0038)
    hole_profiles: list[list[tuple[float, float]]] = []
    first_center = -0.5 * usable_height + 0.5 * slot_height
    for slot_index in range(slot_count):
        center_y = first_center + slot_index * (slot_height + slot_gap)
        hole_profiles.append(
            _shift_profile(
                rounded_rect_profile(
                    slot_width,
                    slot_height,
                    slot_radius,
                    corner_segments=6,
                ),
                dy=center_y,
            )
        )

    grille = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    grille.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(grille, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_bedside_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.94, 1.0))
    grille_grey = model.material("grille_grey", rgba=(0.84, 0.86, 0.86, 1.0))
    interior_charcoal = model.material("interior_charcoal", rgba=(0.16, 0.18, 0.19, 1.0))
    tray_grey = model.material("tray_grey", rgba=(0.82, 0.84, 0.85, 1.0))
    filter_green = model.material("filter_green", rgba=(0.72, 0.80, 0.72, 1.0))
    filter_shadow = model.material("filter_shadow", rgba=(0.56, 0.63, 0.56, 1.0))

    housing_width = 0.230
    housing_depth = 0.225
    housing_height = 0.305
    shell_wall = 0.006
    shell_radius = 0.024
    top_cap_thickness = 0.006

    grille_width = 0.176
    grille_height = 0.180
    grille_bottom = 0.072
    tray_slot_width = 0.194
    tray_slot_height = 0.042
    tray_slot_bottom = 0.000

    tray_width = 0.198
    tray_depth = 0.168
    tray_wall = 0.003
    tray_front_lip_width = 0.188
    tray_front_lip_thickness = 0.004
    tray_front_lip_height = 0.038
    tray_travel = 0.125

    housing = model.part("housing")
    shell_body_height = housing_height - top_cap_thickness
    front_plane_y = housing_depth * 0.5 - shell_wall * 0.5
    side_panel_depth = housing_depth - shell_wall
    front_post_width = 0.018
    housing.visual(
        Box((shell_wall, side_panel_depth, shell_body_height)),
        origin=Origin(
            xyz=(
                -(housing_width * 0.5 - shell_wall * 0.5),
                -shell_wall * 0.5,
                shell_body_height * 0.5,
            )
        ),
        material=shell_white,
        name="left_shell_wall",
    )
    housing.visual(
        Box((shell_wall, side_panel_depth, shell_body_height)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - shell_wall * 0.5,
                -shell_wall * 0.5,
                shell_body_height * 0.5,
            )
        ),
        material=shell_white,
        name="right_shell_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * shell_wall, shell_wall, shell_body_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_depth * 0.5 - shell_wall * 0.5),
                shell_body_height * 0.5,
            )
        ),
        material=shell_white,
        name="back_shell_wall",
    )
    housing.visual(
        Box((housing_width, housing_depth, top_cap_thickness)),
        origin=Origin(xyz=(0.0, 0.0, housing_height - top_cap_thickness * 0.5)),
        material=shell_white,
        name="top_cap",
    )
    housing.visual(
        Box((front_post_width, shell_wall, shell_body_height)),
        origin=Origin(
            xyz=(
                -(housing_width * 0.5 - front_post_width * 0.5),
                front_plane_y,
                shell_body_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_left_post",
    )
    housing.visual(
        Box((front_post_width, shell_wall, shell_body_height)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - front_post_width * 0.5,
                front_plane_y,
                shell_body_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_right_post",
    )
    housing.visual(
        Box((0.194, shell_wall, tray_slot_bottom + tray_slot_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_plane_y,
                (tray_slot_bottom + tray_slot_height) * 0.5,
            )
        ),
        material=shell_white,
        name="front_lower_band",
    )
    mid_rail_height = grille_bottom - (tray_slot_bottom + tray_slot_height)
    housing.visual(
        Box((0.194, shell_wall, mid_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_plane_y,
                tray_slot_bottom + tray_slot_height + mid_rail_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_mid_rail",
    )
    top_rail_height = shell_body_height - (grille_bottom + grille_height)
    housing.visual(
        Box((0.194, shell_wall, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_plane_y,
                grille_bottom + grille_height + top_rail_height * 0.5,
            )
        ),
        material=shell_white,
        name="front_top_rail",
    )

    left_runner_center_x = -(housing_width * 0.5 - shell_wall - 0.015)
    right_runner_center_x = -left_runner_center_x
    housing.visual(
        Box((0.030, 0.180, 0.008)),
        origin=Origin(xyz=(left_runner_center_x, 0.011, 0.004)),
        material=interior_charcoal,
        name="left_tray_runner",
    )
    housing.visual(
        Box((0.030, 0.180, 0.008)),
        origin=Origin(xyz=(right_runner_center_x, 0.011, 0.004)),
        material=interior_charcoal,
        name="right_tray_runner",
    )
    housing.visual(
        Box((0.206, 0.048, 0.006)),
        origin=Origin(xyz=(0.0, -0.0885, 0.003)),
        material=interior_charcoal,
        name="rear_floor",
    )
    grille_depth = 0.004
    grille_outer_width = 0.194
    grille_side_rail_width = 0.010
    grille_frame_height = 0.010
    housing.visual(
        Box((grille_side_rail_width, grille_depth, grille_height)),
        origin=Origin(
            xyz=(
                -(grille_outer_width * 0.5 - grille_side_rail_width * 0.5),
                housing_depth * 0.5 - grille_depth * 0.5,
                grille_bottom + grille_height * 0.5,
            )
        ),
        material=grille_grey,
        name="grille_left_rail",
    )
    housing.visual(
        Box((grille_side_rail_width, grille_depth, grille_height)),
        origin=Origin(
            xyz=(
                grille_outer_width * 0.5 - grille_side_rail_width * 0.5,
                housing_depth * 0.5 - grille_depth * 0.5,
                grille_bottom + grille_height * 0.5,
            )
        ),
        material=grille_grey,
        name="grille_right_rail",
    )
    housing.visual(
        Box((grille_outer_width, grille_depth, grille_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - grille_depth * 0.5,
                grille_bottom + grille_frame_height * 0.5,
            )
        ),
        material=grille_grey,
        name="grille_lower_rail",
    )
    housing.visual(
        Box((grille_outer_width, grille_depth, grille_frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - grille_depth * 0.5,
                grille_bottom + grille_height - grille_frame_height * 0.5,
            )
        ),
        material=grille_grey,
        name="grille_upper_rail",
    )
    slat_count = 9
    slat_height = 0.006
    slat_width = grille_outer_width - 2.0 * grille_side_rail_width
    slat_gap = (grille_height - 2.0 * grille_frame_height - slat_count * slat_height) / (
        slat_count - 1
    )
    for slat_index in range(slat_count):
        z_center = (
            grille_bottom
            + grille_frame_height
            + slat_height * 0.5
            + slat_index * (slat_height + slat_gap)
        )
        housing.visual(
            Box((slat_width, grille_depth, slat_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    housing_depth * 0.5 - grille_depth * 0.5,
                    z_center,
                )
            ),
            material=grille_grey,
            name=f"grille_slat_{slat_index}",
        )
    housing.visual(
        Box((0.168, 0.014, 0.172)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - 0.0105,
                grille_bottom + grille_height * 0.5,
            )
        ),
        material=interior_charcoal,
        name="intake_shadow",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    filter_tray = model.part("filter_tray")
    filter_tray.visual(
        Box((tray_width, tray_depth, tray_wall)),
        origin=Origin(xyz=(0.0, 0.0, tray_wall * 0.5)),
        material=tray_grey,
        name="tray_base",
    )
    filter_tray.visual(
        Box((tray_wall, tray_depth, 0.024)),
        origin=Origin(xyz=(-(tray_width - tray_wall) * 0.5, 0.0, 0.012)),
        material=tray_grey,
        name="tray_left_wall",
    )
    filter_tray.visual(
        Box((tray_wall, tray_depth, 0.024)),
        origin=Origin(xyz=(((tray_width - tray_wall) * 0.5), 0.0, 0.012)),
        material=tray_grey,
        name="tray_right_wall",
    )
    filter_tray.visual(
        Box((tray_width, tray_wall, 0.024)),
        origin=Origin(xyz=(0.0, -(tray_depth - tray_wall) * 0.5, 0.012)),
        material=tray_grey,
        name="tray_rear_wall",
    )
    filter_tray.visual(
        Box((tray_front_lip_width, tray_front_lip_thickness, tray_front_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                tray_depth * 0.5 - tray_front_lip_thickness * 0.5,
                tray_front_lip_height * 0.5,
            )
        ),
        material=tray_grey,
        name="tray_front_lip",
    )
    filter_tray.visual(
        Box((0.060, 0.008, 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                tray_depth * 0.5 - tray_front_lip_thickness - 0.003,
                tray_front_lip_height - 0.006,
            )
        ),
        material=tray_grey,
        name="tray_pull_ridge",
    )
    filter_tray.visual(
        Box((0.184, 0.142, 0.032)),
        origin=Origin(xyz=(0.0, -0.002, 0.019)),
        material=filter_green,
        name="filter_block",
    )
    for pleat_index, x_pos in enumerate(
        (-0.074, -0.053, -0.032, -0.011, 0.011, 0.032, 0.053, 0.074)
    ):
        filter_tray.visual(
            Box((0.004, 0.142, 0.007)),
            origin=Origin(xyz=(x_pos, -0.002, 0.0325)),
            material=filter_shadow,
            name=f"filter_pleat_{pleat_index}",
        )
    filter_tray.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth, 0.041)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
    )

    model.articulation(
        "housing_to_filter_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_tray,
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - tray_depth * 0.5,
                0.008,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=tray_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    filter_tray = object_model.get_part("filter_tray")
    tray_slide = object_model.get_articulation("housing_to_filter_tray")
    grille_lower_rail = housing.get_visual("grille_lower_rail")
    tray_lip = filter_tray.get_visual("tray_front_lip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    limits = tray_slide.motion_limits
    ctx.check(
        "tray_joint_is_prismatic",
        tray_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint_type={tray_slide.articulation_type}",
    )
    ctx.check(
        "tray_joint_slides_forward",
        tuple(tray_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={tray_slide.axis}",
    )
    ctx.check(
        "tray_joint_limits_match_portable_drawer_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and 0.10 <= limits.upper <= 0.14,
        details=f"limits={limits}",
    )

    ctx.expect_contact(
        filter_tray,
        housing,
        name="tray_is_supported_by_housing_guides",
    )
    ctx.expect_gap(
        housing,
        filter_tray,
        axis="z",
        positive_elem=grille_lower_rail,
        negative_elem=tray_lip,
        min_gap=0.020,
        max_gap=0.050,
        name="front_grille_sits_above_tray_opening",
    )
    ctx.expect_overlap(
        housing,
        filter_tray,
        axes="x",
        min_overlap=0.180,
        name="tray_stays_centered_in_housing_width",
    )

    with ctx.pose({tray_slide: limits.upper if limits is not None and limits.upper is not None else 0.125}):
        ctx.expect_origin_gap(
            filter_tray,
            housing,
            axis="y",
            min_gap=0.12,
            max_gap=0.17,
            name="tray_pulls_forward_when_opened",
        )
        ctx.expect_contact(
            filter_tray,
            housing,
            name="tray_remains_guided_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
