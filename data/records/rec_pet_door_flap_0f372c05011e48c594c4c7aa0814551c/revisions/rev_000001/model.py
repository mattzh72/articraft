from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_W = 0.340
OUTER_H = 0.400
FRAME_DEPTH = 0.052
OPEN_W = 0.230
OPEN_H = 0.290
SIDE_BORDER = (OUTER_W - OPEN_W) * 0.5
TOP_BORDER = (OUTER_H - OPEN_H) * 0.5
BEZEL_T = 0.006
FRONT_Y = -(FRAME_DEPTH * 0.5 - BEZEL_T * 0.5)
REAR_Y = FRAME_DEPTH * 0.5 - BEZEL_T * 0.5
MID_DEPTH = FRAME_DEPTH - 2.0 * BEZEL_T
OPEN_TOP = OPEN_H * 0.5
OPEN_BOTTOM = -OPEN_H * 0.5


def _add_box(part, size, xyz, material, *, name=None) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_flap_with_security_cover")

    frame_plastic = model.material("frame_plastic", rgba=(0.87, 0.88, 0.90, 1.0))
    frame_shadow = model.material("frame_shadow", rgba=(0.69, 0.71, 0.74, 1.0))
    seal_black = model.material("seal_black", rgba=(0.10, 0.10, 0.11, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.48, 0.54, 0.60, 0.35))
    flap_frame = model.material("flap_frame", rgba=(0.18, 0.19, 0.20, 1.0))
    cover_white = model.material("cover_white", rgba=(0.95, 0.96, 0.97, 1.0))
    cover_grey = model.material("cover_grey", rgba=(0.55, 0.58, 0.61, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((OUTER_W, FRAME_DEPTH, OUTER_H)),
        mass=1.8,
        origin=Origin(),
    )

    side_x = OPEN_W * 0.5 + SIDE_BORDER * 0.5
    top_z = OPEN_H * 0.5 + TOP_BORDER * 0.5
    side_connector_x = OUTER_W * 0.5 - 0.019
    top_connector_z = OUTER_H * 0.5 - 0.009

    for y_pos in (FRONT_Y, REAR_Y):
        suffix = "front" if y_pos < 0.0 else "rear"
        _add_box(
            frame,
            (SIDE_BORDER, BEZEL_T, OUTER_H),
            (-side_x, y_pos, 0.0),
            frame_plastic,
            name=f"{suffix}_left_bezel",
        )
        _add_box(
            frame,
            (SIDE_BORDER, BEZEL_T, OUTER_H),
            (side_x, y_pos, 0.0),
            frame_plastic,
            name=f"{suffix}_right_bezel",
        )
        _add_box(
            frame,
            (OPEN_W, BEZEL_T, TOP_BORDER),
            (0.0, y_pos, top_z),
            frame_plastic,
            name=f"{suffix}_top_bezel",
        )
        _add_box(
            frame,
            (OPEN_W, BEZEL_T, TOP_BORDER),
            (0.0, y_pos, -top_z),
            frame_plastic,
            name=f"{suffix}_bottom_bezel",
        )

    _add_box(
        frame,
        (0.018, MID_DEPTH, OUTER_H),
        (-side_connector_x, 0.0, 0.0),
        frame_shadow,
        name="left_body_rail",
    )
    _add_box(
        frame,
        (0.018, MID_DEPTH, OUTER_H),
        (side_connector_x, 0.0, 0.0),
        frame_shadow,
        name="right_body_rail",
    )
    _add_box(
        frame,
        (OUTER_W, MID_DEPTH, 0.018),
        (0.0, 0.0, top_connector_z),
        frame_shadow,
        name="top_body_rail",
    )
    _add_box(
        frame,
        (OUTER_W, MID_DEPTH, 0.018),
        (0.0, 0.0, -top_connector_z),
        frame_shadow,
        name="bottom_body_rail",
    )

    return_depth = 0.020
    return_y = FRONT_Y + (BEZEL_T + return_depth) * 0.5
    inner_side_x = OPEN_W * 0.5 + 0.004
    inner_top_z = OPEN_H * 0.5 + 0.004
    _add_box(frame, (0.008, return_depth, OPEN_H), (-inner_side_x, return_y, 0.0), frame_shadow, name="left_front_return")
    _add_box(frame, (0.008, return_depth, OPEN_H), (inner_side_x, return_y, 0.0), frame_shadow, name="right_front_return")
    _add_box(frame, (OPEN_W + 0.016, return_depth, 0.008), (0.0, return_y, inner_top_z), frame_shadow, name="top_front_return")
    _add_box(frame, (OPEN_W + 0.016, return_depth, 0.008), (0.0, return_y, -inner_top_z), frame_shadow, name="bottom_front_return")
    _add_box(frame, (0.224, 0.012, 0.004), (0.0, -0.008, -0.147), seal_black, name="flap_stop")

    channel_height = 0.345
    channel_center_z = 0.0275
    lip_y_center = 0.00775
    outer_wall_y_center = 0.0225
    lip_x = 0.1125
    wall_x = 0.123
    _add_box(frame, (0.005, 0.0155, channel_height), (-lip_x, lip_y_center, channel_center_z), frame_shadow, name="left_channel_lip")
    _add_box(frame, (0.005, 0.0155, channel_height), (lip_x, lip_y_center, channel_center_z), frame_shadow, name="right_channel_lip")
    _add_box(frame, (0.006, 0.007, channel_height), (-wall_x, outer_wall_y_center, channel_center_z), frame_shadow, name="left_channel_wall")
    _add_box(frame, (0.006, 0.007, channel_height), (wall_x, outer_wall_y_center, channel_center_z), frame_shadow, name="right_channel_wall")
    _add_box(frame, (0.246, 0.006, 0.010), (0.0, 0.020, 0.162), frame_shadow, name="top_cover_slot")

    flap = model.part("flap")
    flap.inertial = Inertial.from_geometry(
        Box((0.222, 0.014, 0.286)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.007, -0.143)),
    )
    _add_box(flap, (0.222, 0.010, 0.018), (0.0, -0.007, -0.009), flap_frame, name="top_rail")
    _add_box(flap, (0.220, 0.004, 0.260), (0.0, -0.007, -0.148), flap_tint, name="flap_panel")
    _add_box(flap, (0.222, 0.008, 0.008), (0.0, -0.007, -0.282), seal_black, name="bottom_sweep")
    _add_box(flap, (0.006, 0.008, 0.250), (-0.108, -0.007, -0.148), flap_frame, name="left_stile")
    _add_box(flap, (0.006, 0.008, 0.250), (0.108, -0.007, -0.148), flap_frame, name="right_stile")

    security_cover = model.part("security_cover")
    security_cover.inertial = Inertial.from_geometry(
        Box((0.238, 0.010, 0.314)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.002, -0.145)),
    )
    _add_box(security_cover, (0.238, 0.003, 0.300), (0.0, 0.0, -0.150), cover_white, name="cover_panel")
    _add_box(security_cover, (0.080, 0.010, 0.012), (0.0, 0.003, 0.006), cover_grey, name="cover_grip")
    _add_box(security_cover, (0.190, 0.004, 0.010), (0.0, 0.0, -0.292), cover_grey, name="cover_bottom_rib")

    flap_hinge = model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, -0.007, 0.141)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-1.15, upper=1.15),
    )
    cover_slide = model.articulation(
        "frame_to_security_cover",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=security_cover,
        origin=Origin(xyz=(0.0, 0.013, 0.454)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.25, lower=-0.297, upper=0.0),
    )

    frame.meta["primary_joints"] = [flap_hinge.name, cover_slide.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    security_cover = object_model.get_part("security_cover")
    flap_hinge = object_model.get_articulation("frame_to_flap")
    cover_slide = object_model.get_articulation("frame_to_security_cover")

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

    ctx.check(
        "flap_hinge_is_horizontal",
        flap_hinge.axis == (1.0, 0.0, 0.0),
        details=f"expected flap hinge axis (1,0,0), got {flap_hinge.axis}",
    )
    ctx.check(
        "security_cover_slides_vertically",
        cover_slide.axis == (0.0, 0.0, 1.0),
        details=f"expected cover slide axis (0,0,1), got {cover_slide.axis}",
    )
    ctx.check(
        "security_cover_travel_direction",
        cover_slide.motion_limits is not None
        and cover_slide.motion_limits.lower is not None
        and cover_slide.motion_limits.upper is not None
        and cover_slide.motion_limits.lower < 0.0 <= cover_slide.motion_limits.upper
        and isclose(cover_slide.motion_limits.upper, 0.0, abs_tol=1e-9),
        details="expected the security cover to park upward at the default pose and slide downward to close",
    )

    with ctx.pose({flap_hinge: 0.0, cover_slide: 0.0}):
        ctx.expect_contact(flap, frame, name="flap_hangs_on_frame_in_rest_pose")
        ctx.expect_contact(security_cover, frame, name="cover_remains_captured_in_open_channels")

    with ctx.pose({flap_hinge: 0.0, cover_slide: -0.297}):
        ctx.expect_gap(
            security_cover,
            flap,
            axis="y",
            min_gap=0.016,
            max_gap=0.022,
            name="closed_cover_stays_behind_flap",
        )
        ctx.expect_overlap(
            security_cover,
            flap,
            axes="xz",
            min_overlap=0.205,
            name="closed_cover_blocks_flap_opening",
        )
        ctx.expect_contact(
            security_cover,
            frame,
            name="closed_cover_stays_in_frame_tracks",
        )

    flap_rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 0.80, cover_slide: 0.0}):
        flap_open_aabb = ctx.part_world_aabb(flap)
        moved_up_and_in = False
        if flap_rest_aabb is not None and flap_open_aabb is not None:
            moved_up_and_in = (
                flap_open_aabb[0][2] > flap_rest_aabb[0][2] + 0.05
                and flap_open_aabb[1][1] > flap_rest_aabb[1][1] + 0.04
            )
        ctx.check(
            "flap_rotates_inward_about_top_axis",
            moved_up_and_in,
            details="expected the swung flap to lift its lower edge and move toward +y",
        )
        ctx.expect_gap(
            security_cover,
            flap,
            axis="z",
            min_gap=0.006,
            name="raised_cover_clears_flap_swing_path",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
