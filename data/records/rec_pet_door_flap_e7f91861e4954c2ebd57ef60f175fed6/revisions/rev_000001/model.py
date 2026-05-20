from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_insert")

    frame_cream = model.material("frame_cream", rgba=(0.87, 0.85, 0.81, 1.0))
    frame_shadow = model.material("frame_shadow", rgba=(0.64, 0.63, 0.60, 1.0))
    smoked_flap = model.material("smoked_flap", rgba=(0.18, 0.23, 0.27, 0.42))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.77, 0.79, 0.81, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.22, 0.24, 0.26, 1.0))

    opening_w = 0.18
    opening_h = 0.24
    frame_margin = 0.04
    side_pocket_w = 0.20
    outer_w = opening_w + 2.0 * frame_margin + side_pocket_w
    outer_h = opening_h + 2.0 * frame_margin
    frame_t = 0.038
    bezel_t = 0.006
    tunnel_t = frame_t - 2.0 * bezel_t
    opening_center_x = -side_pocket_w * 0.5
    opening_bottom_z = frame_margin
    opening_top_z = opening_bottom_z + opening_h
    opening_center_z = outer_h * 0.5
    front_face_y = frame_t * 0.5

    cover_w = opening_w + 0.008
    cover_h = opening_h + 0.008
    cover_t = 0.0035
    cover_store_x = 0.10
    cover_travel = cover_store_x - opening_center_x
    cover_center_y = front_face_y + cover_t * 0.5

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_w, frame_t, outer_h)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, outer_h * 0.5)),
    )

    bezel_profile = rounded_rect_profile(outer_w, outer_h, radius=0.028, corner_segments=10)
    opening_profile = _shift_profile(
        rounded_rect_profile(opening_w, opening_h, radius=0.018, corner_segments=10),
        dx=opening_center_x,
    )
    bezel_geom = ExtrudeWithHolesGeometry(
        bezel_profile,
        [opening_profile],
        height=bezel_t,
        center=True,
    ).rotate_x(pi / 2.0)
    bezel_mesh = mesh_from_geometry(bezel_geom, "pet_door_bezel")

    frame.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, front_face_y - bezel_t * 0.5, opening_center_z)),
        material=frame_cream,
        name="front_bezel",
    )
    frame.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, -front_face_y + bezel_t * 0.5, opening_center_z)),
        material=frame_cream,
        name="rear_bezel",
    )

    main_frame_w = opening_w + 2.0 * frame_margin
    frame.visual(
        Box((frame_margin, tunnel_t, opening_h)),
        origin=Origin(
            xyz=(
                opening_center_x - opening_w * 0.5 - frame_margin * 0.5,
                0.0,
                opening_center_z,
            )
        ),
        material=frame_shadow,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((frame_margin, tunnel_t, opening_h)),
        origin=Origin(
            xyz=(
                opening_center_x + opening_w * 0.5 + frame_margin * 0.5,
                0.0,
                opening_center_z,
            )
        ),
        material=frame_shadow,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((main_frame_w, tunnel_t, frame_margin)),
        origin=Origin(xyz=(opening_center_x, 0.0, outer_h - frame_margin * 0.5)),
        material=frame_shadow,
        name="header_tunnel_wall",
    )
    frame.visual(
        Box((main_frame_w, tunnel_t, frame_margin)),
        origin=Origin(xyz=(opening_center_x, 0.0, frame_margin * 0.5)),
        material=frame_shadow,
        name="threshold_tunnel_wall",
    )
    frame.visual(
        Box((opening_w + 0.032, 0.020, 0.020)),
        origin=Origin(xyz=(opening_center_x, 0.008, opening_top_z + 0.010)),
        material=frame_shadow,
        name="hinge_shroud",
    )

    track_span = 0.395
    lip_depth = 0.003
    lip_height = 0.012
    lip_center_y = front_face_y + 0.0065
    top_lip_z = opening_center_z + cover_h * 0.5 + lip_height * 0.5 - 0.004
    bottom_lip_z = opening_center_z - cover_h * 0.5 - lip_height * 0.5 + 0.004

    frame.visual(
        Box((track_span, lip_depth, lip_height)),
        origin=Origin(xyz=(0.0, lip_center_y, top_lip_z)),
        material=frame_shadow,
        name="top_track_lip",
    )
    frame.visual(
        Box((track_span, lip_depth, lip_height)),
        origin=Origin(xyz=(0.0, lip_center_y, bottom_lip_z)),
        material=frame_shadow,
        name="bottom_track_lip",
    )

    for support_x in (-0.196, 0.196):
        frame.visual(
            Box((0.014, lip_center_y - front_face_y + lip_depth * 0.5, lip_height)),
            origin=Origin(
                xyz=(
                    support_x,
                    (front_face_y + lip_center_y + lip_depth * 0.5) * 0.5,
                    top_lip_z,
                )
            ),
            material=frame_shadow,
        )
        frame.visual(
            Box((0.014, lip_center_y - front_face_y + lip_depth * 0.5, lip_height)),
            origin=Origin(
                xyz=(
                    support_x,
                    (front_face_y + lip_center_y + lip_depth * 0.5) * 0.5,
                    bottom_lip_z,
                )
            ),
            material=frame_shadow,
        )

    frame.visual(
        Box((0.012, 0.008, cover_h + 0.014)),
        origin=Origin(xyz=(outer_w * 0.5 - 0.008, front_face_y + 0.004, opening_center_z)),
        material=frame_shadow,
        name="pocket_end_stop",
    )

    flap = model.part("flap")
    flap.inertial = Inertial.from_geometry(
        Box((opening_w - 0.006, 0.014, opening_h - 0.004)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
    )
    flap_w = opening_w - 0.006
    flap.visual(
        Cylinder(radius=0.004, length=flap_w - 0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_shadow,
        name="hinge_tube",
    )
    flap.visual(
        Box((flap_w, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=frame_shadow,
        name="flap_cap",
    )
    flap.visual(
        Box((flap_w - 0.006, 0.0028, 0.206)),
        origin=Origin(xyz=(0.0, 0.0, -0.123)),
        material=smoked_flap,
        name="flap_panel",
    )
    flap.visual(
        Box((flap_w - 0.010, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.231)),
        material=gasket_black,
        name="bottom_seal",
    )

    security_cover = model.part("security_cover")
    security_cover.inertial = Inertial.from_geometry(
        Box((cover_w, cover_t, cover_h)),
        mass=0.34,
    )
    security_cover.visual(
        Box((cover_w, cover_t, cover_h)),
        material=cover_gray,
        name="cover_panel",
    )
    security_cover.visual(
        Box((0.030, 0.010, 0.052)),
        origin=Origin(xyz=(cover_w * 0.5 - 0.026, 0.006, 0.0)),
        material=handle_dark,
        name="pull_handle",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(opening_center_x, 0.0, opening_top_z - 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "security_cover_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=security_cover,
        origin=Origin(xyz=(cover_store_x, cover_center_y, opening_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.25,
            lower=0.0,
            upper=cover_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    security_cover = object_model.get_part("security_cover")
    flap_hinge = object_model.get_articulation("flap_hinge")
    cover_slide = object_model.get_articulation("security_cover_slide")

    ctx.check(
        "flap_hinge_axis_is_horizontal",
        flap_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"expected (-1, 0, 0), got {flap_hinge.axis}",
    )
    ctx.check(
        "cover_slide_axis_is_horizontal",
        cover_slide.axis == (-1.0, 0.0, 0.0),
        details=f"expected (-1, 0, 0), got {cover_slide.axis}",
    )

    flap_limits = flap_hinge.motion_limits
    cover_limits = cover_slide.motion_limits
    ctx.check(
        "flap_hinge_limits_are_pet_door_like",
        flap_limits is not None
        and flap_limits.lower == 0.0
        and flap_limits.upper is not None
        and 1.0 <= flap_limits.upper <= 1.4,
        details=f"unexpected flap limits: {flap_limits}",
    )
    ctx.check(
        "cover_slide_limits_span_the_opening",
        cover_limits is not None
        and cover_limits.lower == 0.0
        and cover_limits.upper is not None
        and cover_limits.upper >= 0.19,
        details=f"unexpected cover limits: {cover_limits}",
    )

    ctx.expect_contact(flap, frame, name="flap_rests_on_threshold")
    ctx.expect_contact(security_cover, frame, name="stored_cover_is_supported_by_frame")
    ctx.expect_gap(
        security_cover,
        flap,
        axis="x",
        min_gap=0.015,
        name="stored_cover_clears_the_opening",
    )

    flap_rest_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 1.05}):
        flap_open_panel = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.check(
            "flap_swings_backward_from_top_hinge",
            flap_rest_panel is not None
            and flap_open_panel is not None
            and flap_open_panel[0][1] < flap_rest_panel[0][1] - 0.10,
            details=f"rest={flap_rest_panel}, open={flap_open_panel}",
        )

    cover_rest_pos = ctx.part_world_position(security_cover)
    with ctx.pose({cover_slide: 0.20}):
        cover_block_pos = ctx.part_world_position(security_cover)
        ctx.check(
            "security_cover_slides_across_the_opening",
            cover_rest_pos is not None
            and cover_block_pos is not None
            and cover_block_pos[0] < cover_rest_pos[0] - 0.18,
            details=f"rest={cover_rest_pos}, blocked={cover_block_pos}",
        )
        ctx.expect_contact(security_cover, frame, name="cover_stays_captured_in_track")
        ctx.expect_overlap(
            security_cover,
            flap,
            axes="xz",
            min_overlap=0.15,
            name="cover_blocks_the_flap_footprint",
        )
        ctx.expect_gap(
            security_cover,
            flap,
            axis="y",
            min_gap=0.010,
            max_gap=0.020,
            name="cover_stays_in_front_of_flap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
