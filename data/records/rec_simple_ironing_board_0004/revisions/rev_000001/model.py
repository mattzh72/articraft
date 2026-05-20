from __future__ import annotations

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
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_ironing_board", assets=ASSETS)

    deck_paint = model.material("deck_paint", rgba=(0.43, 0.48, 0.44, 1.0))
    cover_fabric = model.material("cover_fabric", rgba=(0.20, 0.24, 0.28, 1.0))
    leg_paint = model.material("leg_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    zinc = model.material("zinc", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def yz_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y, z)
            for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
        ]

    def u_frame_mesh(
        name: str,
        points: list[tuple[float, float, float]],
        *,
        radius: float,
        corner_radius: float,
    ):
        return save_mesh(
            name,
            wire_from_points(
                points,
                radius=radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=corner_radius,
                corner_segments=10,
            ),
        )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((1.26, 0.42, 0.10)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    deck_shell = save_mesh(
        "ironing_board_deck_shell.obj",
        section_loft(
            [
                yz_section(-0.60, 0.405, 0.028, 0.008),
                yz_section(-0.34, 0.398, 0.028, 0.008),
                yz_section(-0.02, 0.350, 0.028, 0.008),
                yz_section(0.24, 0.265, 0.026, 0.007),
                yz_section(0.47, 0.175, 0.022, 0.006),
                yz_section(0.61, 0.088, 0.018, 0.004),
            ]
        ),
    )
    deck.visual(deck_shell, material=deck_paint, name="deck_shell")

    cover_pad = save_mesh(
        "ironing_board_cover_pad.obj",
        section_loft(
            [
                yz_section(-0.57, 0.370, 0.010, 0.0035),
                yz_section(-0.31, 0.364, 0.010, 0.0035),
                yz_section(-0.02, 0.320, 0.010, 0.0035),
                yz_section(0.22, 0.240, 0.009, 0.0030),
                yz_section(0.44, 0.155, 0.008, 0.0025),
                yz_section(0.57, 0.078, 0.006, 0.0020),
            ]
        ),
        )
    deck.visual(
        cover_pad,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=cover_fabric,
        name="cover_pad",
    )

    deck.visual(
        Box((0.28, 0.046, 0.024)),
        origin=Origin(xyz=(-0.33, -0.105, -0.027)),
        material=deck_paint,
        name="left_reinforcement_rail_rear",
    )
    deck.visual(
        Box((0.24, 0.046, 0.024)),
        origin=Origin(xyz=(0.02, -0.105, -0.027)),
        material=deck_paint,
        name="left_reinforcement_rail_mid",
    )
    deck.visual(
        Box((0.16, 0.046, 0.024)),
        origin=Origin(xyz=(0.30, -0.105, -0.027)),
        material=deck_paint,
        name="left_reinforcement_rail_front",
    )
    deck.visual(
        Box((0.28, 0.046, 0.024)),
        origin=Origin(xyz=(-0.33, 0.105, -0.027)),
        material=deck_paint,
        name="right_reinforcement_rail_rear",
    )
    deck.visual(
        Box((0.24, 0.046, 0.024)),
        origin=Origin(xyz=(0.02, 0.105, -0.027)),
        material=deck_paint,
        name="right_reinforcement_rail_mid",
    )
    deck.visual(
        Box((0.16, 0.046, 0.024)),
        origin=Origin(xyz=(0.30, 0.105, -0.027)),
        material=deck_paint,
        name="right_reinforcement_rail_front",
    )
    deck.visual(
        Box((0.090, 0.046, 0.024)),
        origin=Origin(xyz=(-0.145, -0.105, -0.027)),
        material=deck_paint,
        name="left_reinforcement_coupler_rear",
    )
    deck.visual(
        Box((0.080, 0.046, 0.024)),
        origin=Origin(xyz=(0.180, -0.105, -0.027)),
        material=deck_paint,
        name="left_reinforcement_coupler_front",
    )
    deck.visual(
        Box((0.090, 0.046, 0.024)),
        origin=Origin(xyz=(-0.145, 0.105, -0.027)),
        material=deck_paint,
        name="right_reinforcement_coupler_rear",
    )
    deck.visual(
        Box((0.080, 0.046, 0.024)),
        origin=Origin(xyz=(0.180, 0.105, -0.027)),
        material=deck_paint,
        name="right_reinforcement_coupler_front",
    )
    deck.visual(
        Box((0.26, 0.090, 0.026)),
        origin=Origin(xyz=(0.02, 0.0, -0.027)),
        material=deck_paint,
        name="center_bridge",
    )
    deck.visual(
        Box((0.090, 0.246, 0.024)),
        origin=Origin(xyz=(0.165, 0.0, -0.028)),
        material=deck_paint,
        name="rear_crossmember",
    )
    deck.visual(
        Box((0.14, 0.300, 0.016)),
        origin=Origin(xyz=(-0.42, 0.0, -0.022)),
        material=deck_paint,
        name="rear_tie_plate",
    )
    deck.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(-0.12, -0.140, -0.025)),
        material=deck_paint,
        name="front_mount_left",
    )
    deck.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(-0.12, 0.140, -0.025)),
        material=deck_paint,
        name="front_mount_right",
    )
    deck.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(0.18, -0.180, -0.025)),
        material=deck_paint,
        name="rear_mount_left",
    )
    deck.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(0.18, 0.180, -0.025)),
        material=deck_paint,
        name="rear_mount_right",
    )
    deck.visual(
        Box((0.040, 0.078, 0.022)),
        origin=Origin(xyz=(0.180, -0.155, -0.025)),
        material=deck_paint,
        name="rear_mount_left_connector",
    )
    deck.visual(
        Box((0.040, 0.078, 0.022)),
        origin=Origin(xyz=(0.180, 0.155, -0.025)),
        material=deck_paint,
        name="rear_mount_right_connector",
    )
    deck.visual(
        Box((0.008, 0.016, 0.120)),
        origin=Origin(xyz=(-0.148, 0.140, -0.096)),
        material=deck_paint,
        name="lock_side_bracket",
    )
    deck.visual(
        Box((0.036, 0.016, 0.024)),
        origin=Origin(xyz=(-0.134, 0.140, -0.150)),
        material=deck_paint,
        name="lock_drop_bracket",
    )
    deck.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(-0.124, 0.140, -0.150)),
        material=zinc,
        name="lock_strike",
    )

    for index, (x_pos, y_pos) in enumerate(
        [
            (-0.18, -0.105),
            (-0.18, 0.105),
            (0.12, -0.105),
            (0.12, 0.105),
            (-0.30, -0.105),
            (-0.30, 0.105),
            (0.30, -0.105),
            (0.30, 0.105),
        ],
        start=1,
    ):
        deck.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(xyz=(x_pos, y_pos, -0.026)),
            material=zinc,
            name=f"fastener_{index}",
        )

    front_leg_frame = model.part("front_leg_frame")
    front_leg_frame.inertial = Inertial.from_geometry(
        Box((0.28, 0.33, 0.78)),
        mass=2.4,
        origin=Origin(xyz=(0.08, 0.0, -0.39)),
    )
    front_leg_frame.visual(
        u_frame_mesh(
            "front_leg_frame.obj",
            [
                (0.0, -0.140, 0.0),
                (0.030, -0.140, -0.110),
                (0.160, -0.140, -0.640),
                (0.200, -0.105, -0.750),
                (0.200, 0.105, -0.750),
                (0.160, 0.140, -0.640),
                (0.030, 0.140, -0.110),
                (0.0, 0.140, 0.0),
            ],
            radius=0.011,
            corner_radius=0.045,
        ),
        material=leg_paint,
        name="frame_tube",
    )
    front_leg_frame.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, -0.140, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_left",
    )
    front_leg_frame.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_right",
    )
    front_leg_frame.visual(
        Cylinder(radius=0.009, length=0.286),
        origin=Origin(xyz=(0.095, 0.0, -0.385), rpy=(pi / 2.0, 0.0, 0.0)),
        material=leg_paint,
        name="mid_spreader",
    )
    front_leg_frame.visual(
        Box((0.060, 0.036, 0.028)),
        origin=Origin(xyz=(0.198, -0.105, -0.750)),
        material=rubber,
        name="left_foot_pad",
    )
    front_leg_frame.visual(
        Box((0.060, 0.036, 0.028)),
        origin=Origin(xyz=(0.198, 0.105, -0.750)),
        material=rubber,
        name="right_foot_pad",
    )
    front_leg_frame.visual(
        Box((0.018, 0.020, 0.116)),
        origin=Origin(xyz=(0.012, 0.140, -0.096)),
        material=leg_paint,
        name="lock_lug_plate",
    )
    front_leg_frame.visual(
        Box((0.036, 0.018, 0.024)),
        origin=Origin(xyz=(0.022, 0.140, -0.100)),
        material=zinc,
        name="lock_lug",
    )

    rear_leg_frame = model.part("rear_leg_frame")
    rear_leg_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.40, 0.76)),
        mass=2.6,
        origin=Origin(xyz=(-0.12, 0.0, -0.37)),
    )
    rear_leg_frame.visual(
        u_frame_mesh(
            "rear_leg_frame.obj",
            [
                (0.0, -0.180, 0.0),
                (-0.038, -0.180, -0.130),
                (-0.190, -0.180, -0.600),
                (-0.255, -0.135, -0.715),
                (-0.255, 0.135, -0.715),
                (-0.190, 0.180, -0.600),
                (-0.038, 0.180, -0.130),
                (0.0, 0.180, 0.0),
            ],
            radius=0.0105,
            corner_radius=0.048,
        ),
        material=leg_paint,
        name="frame_tube",
    )
    rear_leg_frame.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, -0.180, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_left",
    )
    rear_leg_frame.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.0, 0.180, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_sleeve_right",
    )
    rear_leg_frame.visual(
        Box((0.050, 0.366, 0.022)),
        origin=Origin(xyz=(-0.145, 0.0, -0.392)),
        material=leg_paint,
        name="mid_spreader",
    )
    rear_leg_frame.visual(
        Box((0.054, 0.032, 0.062)),
        origin=Origin(xyz=(-0.142, -0.166, -0.408)),
        material=leg_paint,
        name="left_spreader_bracket",
    )
    rear_leg_frame.visual(
        Box((0.054, 0.032, 0.062)),
        origin=Origin(xyz=(-0.142, 0.166, -0.408)),
        material=leg_paint,
        name="right_spreader_bracket",
    )
    rear_leg_frame.visual(
        Box((0.062, 0.040, 0.028)),
        origin=Origin(xyz=(-0.255, -0.135, -0.715)),
        material=rubber,
        name="left_foot_pad",
    )
    rear_leg_frame.visual(
        Box((0.062, 0.040, 0.028)),
        origin=Origin(xyz=(-0.255, 0.135, -0.715)),
        material=rubber,
        name="right_foot_pad",
    )
    model.articulation(
        "deck_to_front_leg_frame",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg_frame,
        origin=Origin(xyz=(-0.12, 0.0, -0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=-1.22,
            upper=0.0,
        ),
    )
    model.articulation(
        "deck_to_rear_leg_frame",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg_frame,
        origin=Origin(xyz=(0.18, 0.0, -0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    front_leg_frame = object_model.get_part("front_leg_frame")
    rear_leg_frame = object_model.get_part("rear_leg_frame")
    front_hinge = object_model.get_articulation("deck_to_front_leg_frame")
    rear_hinge = object_model.get_articulation("deck_to_rear_leg_frame")

    deck_shell = deck.get_visual("deck_shell")
    cover_pad = deck.get_visual("cover_pad")
    front_mount_left = deck.get_visual("front_mount_left")
    front_mount_right = deck.get_visual("front_mount_right")
    rear_mount_left = deck.get_visual("rear_mount_left")
    rear_mount_right = deck.get_visual("rear_mount_right")
    lock_strike = deck.get_visual("lock_strike")

    front_frame_tube = front_leg_frame.get_visual("frame_tube")
    front_sleeve_left = front_leg_frame.get_visual("hinge_sleeve_left")
    front_sleeve_right = front_leg_frame.get_visual("hinge_sleeve_right")
    lock_lug = front_leg_frame.get_visual("lock_lug")
    front_left_foot = front_leg_frame.get_visual("left_foot_pad")

    rear_frame_tube = rear_leg_frame.get_visual("frame_tube")
    rear_sleeve_left = rear_leg_frame.get_visual("hinge_sleeve_left")
    rear_sleeve_right = rear_leg_frame.get_visual("hinge_sleeve_right")
    rear_left_foot = rear_leg_frame.get_visual("left_foot_pad")

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

    ctx.expect_contact(
        front_leg_frame,
        deck,
        elem_a=front_sleeve_left,
        elem_b=front_mount_left,
        name="front_left_hinge_bearing_contacts_mount",
    )
    ctx.expect_contact(
        front_leg_frame,
        deck,
        elem_a=front_sleeve_right,
        elem_b=front_mount_right,
        name="front_right_hinge_bearing_contacts_mount",
    )
    ctx.expect_contact(
        rear_leg_frame,
        deck,
        elem_a=rear_sleeve_left,
        elem_b=rear_mount_left,
        name="rear_left_hinge_bearing_contacts_mount",
    )
    ctx.expect_contact(
        rear_leg_frame,
        deck,
        elem_a=rear_sleeve_right,
        elem_b=rear_mount_right,
        name="rear_right_hinge_bearing_contacts_mount",
    )
    ctx.expect_contact(
        front_leg_frame,
        deck,
        elem_a=lock_lug,
        elem_b=lock_strike,
        name="open_position_lock_lug_seats_on_strike",
    )
    ctx.expect_overlap(
        front_leg_frame,
        rear_leg_frame,
        axes="xz",
        min_overlap=0.12,
        name="scissor_frames_overlap_in_side_profile",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    cover_aabb = ctx.part_element_world_aabb(deck, elem=cover_pad)
    front_aabb = ctx.part_world_aabb(front_leg_frame)
    rear_aabb = ctx.part_world_aabb(rear_leg_frame)
    front_left_foot_open = ctx.part_element_world_aabb(front_leg_frame, elem=front_left_foot)
    rear_left_foot_open = ctx.part_element_world_aabb(rear_leg_frame, elem=rear_left_foot)

    ctx.check(
        "open_stance_has_useful_height",
        cover_aabb is not None
        and front_aabb is not None
        and rear_aabb is not None
        and front_left_foot_open is not None
        and rear_left_foot_open is not None
        and cover_aabb[1][2] - front_left_foot_open[0][2] > 0.80
        and cover_aabb[1][2] - rear_left_foot_open[0][2] > 0.76,
        details="Both leg frames should drop well below the deck to form a working-height stance.",
    )
    ctx.check(
        "open_stance_spreads_feet_fore_and_aft",
        front_aabb is not None
        and rear_aabb is not None
        and front_aabb[1][0] > 0.04
        and rear_aabb[0][0] < -0.04,
        details="The front frame should plant ahead of center and the rear frame behind it.",
    )

    with ctx.pose({front_hinge: -1.02, rear_hinge: 1.02}):
        ctx.expect_gap(
            front_leg_frame,
            deck,
            axis="x",
            positive_elem=lock_lug,
            negative_elem=lock_strike,
            min_gap=0.050,
            name="lock_lug_clears_strike_when_folded",
        )
        ctx.expect_overlap(
            front_leg_frame,
            deck,
            axes="xy",
            min_overlap=0.18,
            name="front_frame_stows_under_deck",
        )
        ctx.expect_overlap(
            rear_leg_frame,
            deck,
            axes="xy",
            min_overlap=0.18,
            name="rear_frame_stows_under_deck",
        )

        folded_front_foot = ctx.part_element_world_aabb(front_leg_frame, elem=front_left_foot)
        folded_rear_foot = ctx.part_element_world_aabb(rear_leg_frame, elem=rear_left_foot)

        ctx.check(
            "front_frame_lifts_when_folded",
            front_left_foot_open is not None
            and folded_front_foot is not None
            and folded_front_foot[0][2] > front_left_foot_open[0][2] + 0.48,
            details="The front foot should rise substantially toward the underside in the folded pose.",
        )
        ctx.check(
            "rear_frame_lifts_when_folded",
            rear_left_foot_open is not None
            and folded_rear_foot is not None
            and folded_rear_foot[0][2] > rear_left_foot_open[0][2] + 0.50,
            details="The rear foot should rise substantially toward the underside in the folded pose.",
        )
        ctx.expect_gap(
            deck,
            front_leg_frame,
            axis="z",
            positive_elem=deck_shell,
            negative_elem=front_frame_tube,
            min_gap=0.001,
            name="front_frame_clears_deck_shell_in_storage_pose",
        )
        ctx.expect_gap(
            deck,
            rear_leg_frame,
            axis="z",
            positive_elem=deck_shell,
            negative_elem=rear_frame_tube,
            min_gap=0.020,
            name="rear_frame_clears_deck_shell_in_storage_pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
