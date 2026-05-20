from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _horizontal_cushion_mesh(name: str, size: tuple[float, float, float], radius: float):
    sx, sy, sz = size
    geom = ExtrudeGeometry(
        rounded_rect_profile(sx, sy, radius, corner_segments=8),
        sz,
        cap=True,
        center=True,
    )
    return _mesh(name, geom)


def _upright_cushion_mesh(name: str, size: tuple[float, float, float], radius: float):
    sx, sy, sz = size
    geom = ExtrudeGeometry(
        rounded_rect_profile(sx, sz, radius, corner_segments=8),
        sy,
        cap=True,
        center=True,
    ).rotate_x(pi / 2.0)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recliner_lounge_chair")

    frame_wood = model.material("frame_wood", rgba=(0.30, 0.22, 0.16, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.21, 0.23, 1.0))
    upholstery = model.material("upholstery", rgba=(0.63, 0.57, 0.50, 1.0))
    upholstery_shadow = model.material("upholstery_shadow", rgba=(0.56, 0.50, 0.45, 1.0))
    leg_pad = model.material("leg_pad", rgba=(0.10, 0.10, 0.11, 1.0))

    arm_mesh = _upright_cushion_mesh("left_right_arm_body", (0.76, 0.14, 0.46), 0.055)
    seat_mesh = _horizontal_cushion_mesh("seat_cushion_body", (0.52, 0.48, 0.10), 0.05)
    back_mesh = _upright_cushion_mesh("backrest_body", (0.12, 0.48, 0.70), 0.045)
    footrest_mesh = _upright_cushion_mesh("footrest_panel_body", (0.075, 0.50, 0.22), 0.028)

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.68, 0.05, 0.16)),
        origin=Origin(xyz=(0.0, 0.24, 0.24)),
        material=frame_wood,
        name="left_side_rail",
    )
    base_frame.visual(
        Box((0.68, 0.05, 0.16)),
        origin=Origin(xyz=(0.0, -0.24, 0.24)),
        material=frame_wood,
        name="right_side_rail",
    )
    base_frame.visual(
        Box((0.04, 0.48, 0.16)),
        origin=Origin(xyz=(0.32, 0.0, 0.24)),
        material=frame_wood,
        name="front_rail",
    )
    base_frame.visual(
        Box((0.04, 0.48, 0.16)),
        origin=Origin(xyz=(-0.32, 0.0, 0.24)),
        material=frame_wood,
        name="rear_rail",
    )
    base_frame.visual(
        Box((0.48, 0.44, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=frame_wood,
        name="seat_deck",
    )

    for x_sign, y_sign, name in (
        (1.0, 1.0, "front_left_leg"),
        (1.0, -1.0, "front_right_leg"),
        (-1.0, 1.0, "rear_left_leg"),
        (-1.0, -1.0, "rear_right_leg"),
    ):
        base_frame.visual(
            Box((0.06, 0.06, 0.16)),
            origin=Origin(xyz=(0.29 * x_sign, 0.24 * y_sign, 0.08)),
            material=frame_wood,
            name=name,
        )
        base_frame.visual(
            Box((0.065, 0.065, 0.01)),
            origin=Origin(xyz=(0.29 * x_sign, 0.24 * y_sign, 0.005)),
            material=leg_pad,
            name=f"{name}_pad",
        )

    base_frame.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(-0.325, 0.255, 0.52)),
        material=frame_wood,
        name="left_rear_cheek",
    )
    base_frame.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(-0.325, -0.255, 0.52)),
        material=frame_wood,
        name="right_rear_cheek",
    )
    base_frame.visual(
        Box((0.06, 0.05, 0.17)),
        origin=Origin(xyz=(-0.29, 0.24, 0.385)),
        material=frame_wood,
        name="left_rear_stanchion",
    )
    base_frame.visual(
        Box((0.06, 0.05, 0.17)),
        origin=Origin(xyz=(-0.29, -0.24, 0.385)),
        material=frame_wood,
        name="right_rear_stanchion",
    )
    base_frame.visual(
        Box((0.48, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.23, 0.33)),
        material=frame_wood,
        name="left_seat_cleat",
    )
    base_frame.visual(
        Box((0.48, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, -0.23, 0.33)),
        material=frame_wood,
        name="right_seat_cleat",
    )
    base_frame.visual(
        Box((0.56, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.32, 0.12)),
        material=frame_wood,
        name="left_arm_support",
    )
    base_frame.visual(
        Box((0.58, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.32, 0.12)),
        material=frame_wood,
        name="right_arm_support",
    )
    base_frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.315, 0.185, 0.20)),
        material=dark_metal,
        name="left_footrest_bracket",
    )
    base_frame.visual(
        Box((0.03, 0.03, 0.10)),
        origin=Origin(xyz=(0.315, -0.185, 0.20)),
        material=dark_metal,
        name="right_footrest_bracket",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.84, 0.60, 0.62)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    left_arm = model.part("left_arm")
    left_arm.visual(arm_mesh, material=upholstery, name="arm_body")
    left_arm.inertial = Inertial.from_geometry(
        Box((0.76, 0.14, 0.46)),
        mass=9.0,
    )

    right_arm = model.part("right_arm")
    right_arm.visual(arm_mesh, material=upholstery, name="arm_body")
    right_arm.inertial = Inertial.from_geometry(
        Box((0.76, 0.14, 0.46)),
        mass=9.0,
    )

    seat_cushion = model.part("seat_cushion")
    seat_cushion.visual(seat_mesh, material=upholstery, name="seat_pad")
    seat_cushion.visual(
        Box((0.46, 0.42, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=upholstery_shadow,
        name="seat_base_panel",
    )
    seat_cushion.inertial = Inertial.from_geometry(
        Box((0.52, 0.48, 0.10)),
        mass=8.0,
    )

    backrest = model.part("backrest")
    backrest.visual(
        back_mesh,
        origin=Origin(xyz=(-0.09, 0.0, 0.35)),
        material=upholstery,
        name="back_pad",
    )
    backrest.visual(
        Box((0.03, 0.50, 0.62)),
        origin=Origin(xyz=(-0.125, 0.0, 0.35)),
        material=upholstery_shadow,
        name="back_shell",
    )
    backrest.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(0.015, 0.25, 0.05)),
        material=dark_metal,
        name="left_hinge_knuckle",
    )
    backrest.visual(
        Box((0.03, 0.02, 0.10)),
        origin=Origin(xyz=(0.015, -0.25, 0.05)),
        material=dark_metal,
        name="right_hinge_knuckle",
    )
    backrest.visual(
        Box((0.125, 0.02, 0.08)),
        origin=Origin(xyz=(-0.0575, 0.235, 0.09)),
        material=dark_metal,
        name="left_side_spine",
    )
    backrest.visual(
        Box((0.125, 0.02, 0.08)),
        origin=Origin(xyz=(-0.0575, -0.235, 0.09)),
        material=dark_metal,
        name="right_side_spine",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.16, 0.50, 0.72)),
        mass=11.5,
        origin=Origin(xyz=(-0.07, 0.0, 0.36)),
    )

    footrest_panel = model.part("footrest_panel")
    footrest_panel.visual(
        footrest_mesh,
        origin=Origin(xyz=(0.11, 0.0, -0.11)),
        material=upholstery,
        name="panel_pad",
    )
    footrest_panel.visual(
        Box((0.01, 0.03, 0.10)),
        origin=Origin(xyz=(0.005, 0.185, -0.05)),
        material=dark_metal,
        name="left_clip_ear",
    )
    footrest_panel.visual(
        Box((0.01, 0.03, 0.10)),
        origin=Origin(xyz=(0.005, -0.185, -0.05)),
        material=dark_metal,
        name="right_clip_ear",
    )
    footrest_panel.visual(
        Box((0.075, 0.03, 0.09)),
        origin=Origin(xyz=(0.0375, 0.185, -0.145)),
        material=dark_metal,
        name="left_clip_bridge",
    )
    footrest_panel.visual(
        Box((0.075, 0.03, 0.09)),
        origin=Origin(xyz=(0.0375, -0.185, -0.145)),
        material=dark_metal,
        name="right_clip_bridge",
    )
    footrest_panel.visual(
        Box((0.08, 0.40, 0.06)),
        origin=Origin(xyz=(0.07, 0.0, -0.14)),
        material=dark_metal,
        name="clip_backer",
    )
    footrest_panel.inertial = Inertial.from_geometry(
        Box((0.075, 0.50, 0.22)),
        mass=5.0,
        origin=Origin(xyz=(0.11, 0.0, -0.11)),
    )

    model.articulation(
        "left_arm_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=left_arm,
        origin=Origin(xyz=(0.0, 0.35, 0.39)),
    )
    model.articulation(
        "right_arm_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=right_arm,
        origin=Origin(xyz=(0.0, -0.35, 0.39)),
    )
    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=base_frame,
        child=seat_cushion,
        origin=Origin(xyz=(0.02, 0.0, 0.42)),
    )
    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=backrest,
        origin=Origin(xyz=(-0.31, 0.0, 0.48)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=0.85,
        ),
    )
    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=footrest_panel,
        origin=Origin(xyz=(0.29, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    seat_cushion = object_model.get_part("seat_cushion")
    backrest = object_model.get_part("backrest")
    footrest_panel = object_model.get_part("footrest_panel")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    footrest_hinge = object_model.get_articulation("footrest_hinge")

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
        "backrest_hinges_from_rear_side_cheeks",
        tuple(backrest_hinge.axis) == (0.0, -1.0, 0.0)
        and object_model.get_part(backrest_hinge.parent).name == base_frame.name
        and object_model.get_part(backrest_hinge.child).name == backrest.name,
        "Backrest should hinge directly from the base frame on a rear horizontal axis.",
    )
    ctx.check(
        "footrest_uses_its_own_front_hinge",
        tuple(footrest_hinge.axis) == (0.0, -1.0, 0.0)
        and object_model.get_part(footrest_hinge.parent).name == base_frame.name
        and object_model.get_part(footrest_hinge.child).name == footrest_panel.name,
        "Footrest should hinge directly from the seat frame instead of following the backrest.",
    )

    ctx.expect_contact(left_arm, base_frame, name="left_arm_is_mounted_to_frame")
    ctx.expect_contact(right_arm, base_frame, name="right_arm_is_mounted_to_frame")
    ctx.expect_contact(
        seat_cushion,
        base_frame,
        elem_b="seat_deck",
        name="seat_cushion_is_supported_by_seat_deck",
    )
    ctx.expect_gap(
        seat_cushion,
        backrest,
        axis="x",
        min_gap=0.025,
        max_gap=0.12,
        name="backrest_sits_just_behind_seat",
    )
    ctx.expect_within(
        backrest,
        base_frame,
        axes="y",
        margin=0.02,
        name="backrest_fits_between_side_cheeks",
    )
    ctx.expect_contact(
        base_frame,
        footrest_panel,
        elem_a="left_footrest_bracket",
        elem_b="left_clip_ear",
        name="left_footrest_clip_contacts_bracket",
    )
    ctx.expect_contact(
        base_frame,
        footrest_panel,
        elem_a="right_footrest_bracket",
        elem_b="right_clip_ear",
        name="right_footrest_clip_contacts_bracket",
    )
    ctx.expect_within(
        footrest_panel,
        base_frame,
        axes="y",
        margin=0.05,
        name="footrest_panel_stays_between_front_brackets",
    )
    ctx.expect_gap(
        seat_cushion,
        footrest_panel,
        axis="z",
        min_gap=0.09,
        max_gap=0.14,
        name="footrest_stows_below_seat",
    )
    ctx.expect_contact(
        base_frame,
        backrest,
        elem_a="left_rear_cheek",
        elem_b="left_hinge_knuckle",
        name="left_backrest_pivot_knuckle_contacts_cheek",
    )
    ctx.expect_contact(
        base_frame,
        backrest,
        elem_a="right_rear_cheek",
        elem_b="right_hinge_knuckle",
        name="right_backrest_pivot_knuckle_contacts_cheek",
    )

    with ctx.pose({backrest_hinge: 0.72, footrest_hinge: 0.0}):
        ctx.expect_contact(
            base_frame,
            footrest_panel,
            elem_a="left_footrest_bracket",
            elem_b="left_clip_ear",
            name="left_footrest_remains_supported_when_back_reclines",
        )
        ctx.expect_contact(
            base_frame,
            footrest_panel,
            elem_a="right_footrest_bracket",
            elem_b="right_clip_ear",
            name="right_footrest_remains_supported_when_back_reclines",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
