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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_y_pin(part, *, xyz, radius, length, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _leg_frame_mesh(
    name: str,
    *,
    foot_x: float,
    foot_half_width: float,
    top_half_width: float,
    top_z: float,
    foot_z: float,
    tube_radius: float,
):
    return _mesh(
        name,
        wire_from_points(
            [
                (0.0, -top_half_width, top_z),
                (foot_x, -foot_half_width, foot_z),
                (foot_x, foot_half_width, foot_z),
                (0.0, top_half_width, top_z),
            ],
            radius=tube_radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.07,
            corner_segments=10,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_ironing_board")

    steel = model.material("steel", rgba=(0.26, 0.28, 0.30, 1.0))
    zinc = model.material("zinc", rgba=(0.72, 0.74, 0.77, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.62, 0.64, 0.66, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.85, 0.73, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((1.42, 0.44, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    # Deck shell and pad: long rear rectangle plus narrowed nose.
    deck.visual(
        Box((0.96, 0.42, 0.010)),
        origin=Origin(xyz=(-0.19, 0.0, 0.915)),
        material=steel,
        name="deck_skin_rear",
    )
    deck.visual(
        Box((0.31, 0.28, 0.010)),
        origin=Origin(xyz=(0.425, 0.0, 0.915)),
        material=steel,
        name="deck_skin_shoulder",
    )
    deck.visual(
        Cylinder(radius=0.14, length=0.010),
        origin=Origin(xyz=(0.57, 0.0, 0.915)),
        material=steel,
        name="deck_skin_nose",
    )
    deck.visual(
        Box((0.95, 0.40, 0.012)),
        origin=Origin(xyz=(-0.19, 0.0, 0.926)),
        material=pad_gray,
        name="pad_rear",
    )
    deck.visual(
        Box((0.30, 0.26, 0.012)),
        origin=Origin(xyz=(0.425, 0.0, 0.926)),
        material=pad_gray,
        name="pad_shoulder",
    )
    deck.visual(
        Cylinder(radius=0.13, length=0.012),
        origin=Origin(xyz=(0.57, 0.0, 0.926)),
        material=pad_gray,
        name="pad_nose",
    )

    # Under-deck load path: side channels into a central chassis plate.
    deck.visual(
        Box((0.94, 0.03, 0.07)),
        origin=Origin(xyz=(-0.18, 0.165, 0.875)),
        material=steel,
        name="left_side_channel",
    )
    deck.visual(
        Box((0.94, 0.03, 0.07)),
        origin=Origin(xyz=(-0.18, -0.165, 0.875)),
        material=steel,
        name="right_side_channel",
    )
    deck.visual(
        Box((0.76, 0.22, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.820)),
        material=steel,
        name="center_chassis_plate",
    )
    deck.visual(
        Box((0.18, 0.36, 0.08)),
        origin=Origin(xyz=(-0.48, 0.0, 0.840)),
        material=steel,
        name="rear_stiffener",
    )
    deck.visual(
        Box((0.20, 0.08, 0.10)),
        origin=Origin(xyz=(0.37, 0.0, 0.860)),
        material=steel,
        name="nose_rib",
    )

    # Hinge reinforcement and visible bracket logic.
    for name, x, y in (
        ("left_hinge_plate_left", -0.45, 0.135),
        ("left_hinge_plate_right", -0.45, -0.135),
        ("right_hinge_plate_left", 0.34, 0.095),
        ("right_hinge_plate_right", 0.34, -0.095),
    ):
        deck.visual(
            Box((0.06, 0.025, 0.07)),
            origin=Origin(xyz=(x, y, 0.830)),
            material=zinc,
            name=name,
        )

    # Over-travel stops under the board.
    deck.visual(
        Box((0.08, 0.16, 0.06)),
        origin=Origin(xyz=(-0.20, 0.0, 0.825)),
        material=safety_yellow,
        name="left_stop_block",
    )
    deck.visual(
        Box((0.08, 0.16, 0.06)),
        origin=Origin(xyz=(0.20, 0.0, 0.825)),
        material=safety_yellow,
        name="right_stop_block",
    )

    # Guarded side-mounted lock receiver on the outboard side of the right stance.
    deck.visual(
        Box((0.03, 0.03, 0.20)),
        origin=Origin(xyz=(0.05, 0.170, 0.745)),
        material=steel,
        name="lock_spine",
    )
    deck.visual(
        Box((0.14, 0.03, 0.03)),
        origin=Origin(xyz=(0.10, 0.170, 0.665)),
        material=steel,
        name="lock_mount_arm",
    )
    deck.visual(
        Box((0.06, 0.09, 0.02)),
        origin=Origin(xyz=(0.12, 0.190, 0.645)),
        material=safety_yellow,
        name="lock_catch",
    )
    deck.visual(
        Box((0.04, 0.008, 0.07)),
        origin=Origin(xyz=(0.14, 0.231, 0.680)),
        material=safety_yellow,
        name="lock_guard_left",
    )
    deck.visual(
        Box((0.04, 0.008, 0.07)),
        origin=Origin(xyz=(0.14, 0.149, 0.680)),
        material=safety_yellow,
        name="lock_guard_right",
    )
    deck.visual(
        Box((0.06, 0.09, 0.02)),
        origin=Origin(xyz=(0.14, 0.190, 0.715)),
        material=safety_yellow,
        name="lock_guard_top",
    )

    # Visible fastener heads on guarded and reinforced areas.
    for x, y, z in (
        (-0.20, 0.084, 0.825),
        (-0.20, -0.084, 0.825),
        (0.20, 0.084, 0.825),
        (0.20, -0.084, 0.825),
    ):
        _add_y_pin(deck, xyz=(x, y, z), radius=0.008, length=0.008, material=zinc)

    left_leg = model.part("left_leg_frame")
    left_leg.inertial = Inertial.from_geometry(
        Box((0.58, 0.42, 0.94)),
        mass=5.2,
        origin=Origin(xyz=(0.24, 0.0, -0.44)),
    )
    left_leg.visual(
        _leg_frame_mesh(
            "left_leg_frame_tube",
            foot_x=0.48,
            foot_half_width=0.19,
            top_half_width=0.10,
            top_z=-0.10,
            foot_z=-0.88,
            tube_radius=0.015,
        ),
        material=steel,
        name="left_leg_tube",
    )
    left_leg.visual(
        Box((0.03, 0.26, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=zinc,
        name="left_top_bridge",
    )
    left_leg.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(0.03, 0.125, -0.14)),
        material=zinc,
        name="left_stop_link_left",
    )
    left_leg.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(0.03, -0.125, -0.14)),
        material=zinc,
        name="left_stop_link_right",
    )
    left_leg.visual(
        Box((0.12, 0.38, 0.03)),
        origin=Origin(xyz=(0.08, 0.0, -0.10)),
        material=zinc,
        name="open_stop_pad",
    )
    left_leg.visual(
        Box((0.07, 0.05, 0.02)),
        origin=Origin(xyz=(0.48, -0.19, -0.905)),
        material=rubber,
        name="left_foot_pad_left",
    )
    left_leg.visual(
        Box((0.07, 0.05, 0.02)),
        origin=Origin(xyz=(0.48, 0.19, -0.905)),
        material=rubber,
        name="left_foot_pad_right",
    )
    left_leg.visual(
        Box((0.04, 0.04, 0.03)),
        origin=Origin(xyz=(0.48, -0.19, -0.885)),
        material=steel,
        name="left_foot_saddle_left",
    )
    left_leg.visual(
        Box((0.04, 0.04, 0.03)),
        origin=Origin(xyz=(0.48, 0.19, -0.885)),
        material=steel,
        name="left_foot_saddle_right",
    )
    _add_y_pin(left_leg, xyz=(0.23, 0.145, -0.47), radius=0.034, length=0.032, material=zinc, name="left_pivot_boss_outer")
    _add_y_pin(left_leg, xyz=(0.23, -0.145, -0.47), radius=0.034, length=0.032, material=zinc, name="left_pivot_boss_inner")

    right_leg = model.part("right_leg_frame")
    right_leg.inertial = Inertial.from_geometry(
        Box((0.54, 0.28, 0.92)),
        mass=4.8,
        origin=Origin(xyz=(-0.22, 0.0, -0.43)),
    )
    right_leg.visual(
        _leg_frame_mesh(
            "right_leg_frame_tube",
            foot_x=-0.44,
            foot_half_width=0.12,
            top_half_width=0.08,
            top_z=-0.10,
            foot_z=-0.86,
            tube_radius=0.015,
        ),
        material=steel,
        name="right_leg_tube",
    )
    right_leg.visual(
        Box((0.03, 0.20, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=zinc,
        name="right_top_bridge",
    )
    right_leg.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(-0.03, 0.090, -0.14)),
        material=zinc,
        name="right_stop_link_left",
    )
    right_leg.visual(
        Box((0.10, 0.02, 0.08)),
        origin=Origin(xyz=(-0.03, -0.090, -0.14)),
        material=zinc,
        name="right_stop_link_right",
    )
    right_leg.visual(
        Box((0.12, 0.24, 0.03)),
        origin=Origin(xyz=(-0.08, 0.0, -0.10)),
        material=zinc,
        name="open_stop_pad",
    )
    right_leg.visual(
        Box((0.08, 0.08, 0.10)),
        origin=Origin(xyz=(-0.08, 0.140, -0.30)),
        material=zinc,
        name="lock_web",
    )
    right_leg.visual(
        Box((0.04, 0.04, 0.04)),
        origin=Origin(xyz=(-0.12, 0.180, -0.275)),
        material=zinc,
        name="lock_link",
    )
    right_leg.visual(
        Box((0.06, 0.06, 0.02)),
        origin=Origin(xyz=(-0.16, 0.190, -0.255)),
        material=safety_yellow,
        name="lock_hook",
    )
    right_leg.visual(
        Box((0.07, 0.05, 0.02)),
        origin=Origin(xyz=(-0.44, -0.12, -0.885)),
        material=rubber,
        name="right_foot_pad_left",
    )
    right_leg.visual(
        Box((0.07, 0.05, 0.02)),
        origin=Origin(xyz=(-0.44, 0.12, -0.885)),
        material=rubber,
        name="right_foot_pad_right",
    )
    right_leg.visual(
        Box((0.04, 0.04, 0.03)),
        origin=Origin(xyz=(-0.44, -0.12, -0.865)),
        material=steel,
        name="right_foot_saddle_left",
    )
    right_leg.visual(
        Box((0.04, 0.04, 0.03)),
        origin=Origin(xyz=(-0.44, 0.12, -0.865)),
        material=steel,
        name="right_foot_saddle_right",
    )
    _add_y_pin(right_leg, xyz=(-0.22, 0.10, -0.455), radius=0.030, length=0.028, material=zinc, name="right_pivot_boss_outer")
    _add_y_pin(right_leg, xyz=(-0.22, -0.10, -0.455), radius=0.030, length=0.028, material=zinc, name="right_pivot_boss_inner")

    left_joint = model.articulation(
        "deck_to_left_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_leg,
        origin=Origin(xyz=(-0.28, 0.0, 0.880)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.1, lower=-0.08, upper=0.95),
    )
    right_joint = model.articulation(
        "deck_to_right_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_leg,
        origin=Origin(xyz=(0.28, 0.0, 0.880)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.1, lower=-0.08, upper=0.95),
    )

    # Store open-pose articulations for tests and debugging readability.
    deck.meta["primary_articulations"] = [left_joint.name, right_joint.name]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    left_leg = object_model.get_part("left_leg_frame")
    right_leg = object_model.get_part("right_leg_frame")
    left_joint = object_model.get_articulation("deck_to_left_leg")
    right_joint = object_model.get_articulation("deck_to_right_leg")

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

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_contact(
            left_leg,
            deck,
            elem_a="open_stop_pad",
            elem_b="left_stop_block",
            name="left_leg_seats_on_overtravel_stop",
        )
        ctx.expect_contact(
            right_leg,
            deck,
            elem_a="open_stop_pad",
            elem_b="right_stop_block",
            name="right_leg_seats_on_overtravel_stop",
        )
        ctx.expect_contact(
            right_leg,
            deck,
            elem_a="lock_hook",
            elem_b="lock_catch",
            name="lock_hook_engages_catch_in_open_pose",
        )
        ctx.expect_overlap(
            right_leg,
            deck,
            elem_a="lock_hook",
            elem_b="lock_catch",
            axes="xy",
            min_overlap=0.045,
            name="lock_hook_is_laterally_captured",
        )
        ctx.expect_gap(
            deck,
            left_leg,
            axis="z",
            positive_elem="deck_skin_rear",
            negative_elem="left_foot_pad_left",
            min_gap=0.88,
            max_gap=0.97,
            name="open_stance_left_foot_drop",
        )
        ctx.expect_gap(
            deck,
            right_leg,
            axis="z",
            positive_elem="deck_skin_rear",
            negative_elem="right_foot_pad_right",
            min_gap=0.88,
            max_gap=0.98,
            name="open_stance_right_foot_drop",
        )

    with ctx.pose({left_joint: 0.78, right_joint: 0.78}):
        ctx.expect_gap(
            deck,
            left_leg,
            axis="z",
            positive_elem="deck_skin_rear",
            negative_elem="left_foot_pad_left",
            min_gap=0.20,
            max_gap=0.45,
            name="left_foot_rises_toward_board_when_folded",
        )
        ctx.expect_gap(
            deck,
            right_leg,
            axis="z",
            positive_elem="deck_skin_rear",
            negative_elem="right_foot_pad_right",
            min_gap=0.18,
            max_gap=0.45,
            name="right_foot_rises_toward_board_when_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
