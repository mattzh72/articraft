from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


BEAM_HALF_SPAN = 0.96
SIDE_PIVOT_X = 0.58
FRONT_PIVOT_Y = -0.17
REAR_PIVOT_Y = 0.17
PIVOT_SPACING = REAR_PIVOT_Y - FRONT_PIVOT_Y
UPPER_PIVOT_Z = 1.85
LINK_LENGTH = 1.30
BENCH_HALF_SPAN = 0.56


def _add_support_clevis(
    support,
    *,
    x: float,
    y: float,
    z: float,
    material,
) -> None:
    plate_gap = 0.05
    plate_thickness = 0.014
    offset = plate_gap * 0.5 + plate_thickness * 0.5
    for sign in (-1.0, 1.0):
        support.visual(
            Box((plate_thickness, 0.052, 0.10)),
            origin=Origin(xyz=(x + sign * offset, y, z)),
            material=material,
        )


def _add_link_visuals(part, *, material, pin_material) -> None:
    part.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_material,
        name="upper_boss",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_material,
        name="lower_boss",
    )
    part.visual(
        Box((0.020, 0.045, LINK_LENGTH - 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
        material=material,
        name="hanger_bar",
    )


def _inward_x(side_sign: float, distance: float) -> float:
    return -side_sign * distance


def _add_bench_half(
    part,
    *,
    side_sign: float,
    frame_material,
    wood_material,
    pin_material,
) -> None:
    inner_center_x = _inward_x(side_sign, 0.54)
    slat_center_x = _inward_x(side_sign, BENCH_HALF_SPAN * 0.5)
    main_side_x = _inward_x(side_sign, 0.15)
    brace_center_x = _inward_x(side_sign, 0.085)

    for clip_name, clip_y in (("front", 0.0), ("rear", PIVOT_SPACING)):
        for sign in (-1.0, 1.0):
            part.visual(
                Box((0.014, 0.056, 0.140)),
                origin=Origin(xyz=(sign * 0.032, clip_y, 0.030)),
                material=frame_material,
                name=f"{clip_name}_clip_{'left' if sign < 0.0 else 'right'}",
            )

    part.visual(
        Box((0.23, 0.044, 0.040)),
        origin=Origin(xyz=(brace_center_x, 0.050, 0.090)),
        material=frame_material,
        name="front_clip_brace",
    )
    part.visual(
        Box((0.23, 0.044, 0.040)),
        origin=Origin(xyz=(brace_center_x, PIVOT_SPACING - 0.050, 0.090)),
        material=frame_material,
        name="rear_clip_brace",
    )
    part.visual(
        Box((0.050, 0.24, 0.050)),
        origin=Origin(xyz=(main_side_x, PIVOT_SPACING * 0.5, 0.085)),
        material=frame_material,
        name="outer_side_rail",
    )
    part.visual(
        Box((0.036, 0.29, 0.050)),
        origin=Origin(xyz=(inner_center_x, 0.16, 0.050)),
        material=frame_material,
        name="inner_side_rail",
    )
    part.visual(
        Box((0.070, 0.036, 0.040)),
        origin=Origin(xyz=(slat_center_x * 0.45, 0.026, 0.058)),
        material=frame_material,
        name="front_seat_mount",
    )
    part.visual(
        Box((0.070, 0.036, 0.040)),
        origin=Origin(xyz=(slat_center_x * 0.45, 0.244, 0.058)),
        material=frame_material,
        name="rear_seat_mount",
    )

    seat_slat_y = (0.045, 0.110, 0.175, 0.240)
    for index, slat_y in enumerate(seat_slat_y):
        slat_name = None
        if index == 0:
            slat_name = "seat_front_slat"
        elif index == len(seat_slat_y) - 1:
            slat_name = "seat_rear_slat"
        part.visual(
            Box((BENCH_HALF_SPAN, 0.045, 0.018)),
            origin=Origin(xyz=(slat_center_x, slat_y, 0.075)),
            material=wood_material,
            name=slat_name,
        )

    part.visual(
        Box((0.036, 0.045, 0.34)),
        origin=Origin(xyz=(_inward_x(side_sign, 0.050), 0.312, 0.190)),
        material=frame_material,
        name="outer_back_post",
    )
    part.visual(
        Box((0.036, 0.045, 0.31)),
        origin=Origin(xyz=(inner_center_x, 0.300, 0.175)),
        material=frame_material,
        name="inner_back_post",
    )

    for index, slat_z in enumerate((0.170, 0.245, 0.320)):
        slat_name = "backrest_top_slat" if index == 2 else None
        part.visual(
            Box((BENCH_HALF_SPAN, 0.028, 0.018)),
            origin=Origin(xyz=(slat_center_x, 0.300, slat_z)),
            material=wood_material,
            name=slat_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_bench_swing")

    support_blue = model.material("support_blue", rgba=(0.17, 0.41, 0.61, 1.0))
    hanger_steel = model.material("hanger_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    frame_green = model.material("frame_green", rgba=(0.22, 0.36, 0.26, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    wood_slats = model.material("wood_slats", rgba=(0.57, 0.39, 0.20, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((2.10, 0.70, 2.05)),
        mass=230.0,
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
    )
    support_frame.visual(
        Box((BEAM_HALF_SPAN * 2.0, 0.42, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.96)),
        material=support_blue,
        name="top_beam",
    )
    for x in (-0.80, 0.80):
        for y in (-0.16, 0.16):
            support_frame.visual(
                Box((0.10, 0.08, 1.92)),
                origin=Origin(xyz=(x, y, 0.96)),
                material=support_blue,
            )
        support_frame.visual(
            Box((0.08, 0.40, 0.08)),
            origin=Origin(xyz=(x, 0.0, 1.70)),
            material=support_blue,
        )
        support_frame.visual(
            Box((0.08, 0.40, 0.08)),
            origin=Origin(xyz=(x, 0.0, 0.28)),
            material=support_blue,
        )
    for x in (-SIDE_PIVOT_X, SIDE_PIVOT_X):
        for y in (FRONT_PIVOT_Y, REAR_PIVOT_Y):
            _add_support_clevis(
                support_frame,
                x=x,
                y=y,
                z=UPPER_PIVOT_Z,
                material=support_blue,
            )

    left_front_link = model.part("left_front_link")
    _add_link_visuals(left_front_link, material=hanger_steel, pin_material=pin_steel)
    left_front_link.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, LINK_LENGTH)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )

    left_rear_link = model.part("left_rear_link")
    _add_link_visuals(left_rear_link, material=hanger_steel, pin_material=pin_steel)
    left_rear_link.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, LINK_LENGTH)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )

    right_front_link = model.part("right_front_link")
    _add_link_visuals(right_front_link, material=hanger_steel, pin_material=pin_steel)
    right_front_link.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, LINK_LENGTH)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )

    right_rear_link = model.part("right_rear_link")
    _add_link_visuals(right_rear_link, material=hanger_steel, pin_material=pin_steel)
    right_rear_link.inertial = Inertial.from_geometry(
        Box((0.06, 0.06, LINK_LENGTH)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )

    left_bench_half = model.part("left_bench_half")
    _add_bench_half(
        left_bench_half,
        side_sign=1.0,
        frame_material=frame_green,
        wood_material=wood_slats,
        pin_material=pin_steel,
    )
    left_bench_half.inertial = Inertial.from_geometry(
        Box((0.64, 0.44, 0.40)),
        mass=11.0,
        origin=Origin(xyz=(-0.28, 0.17, 0.18)),
    )

    right_bench_half = model.part("right_bench_half")
    _add_bench_half(
        right_bench_half,
        side_sign=-1.0,
        frame_material=frame_green,
        wood_material=wood_slats,
        pin_material=pin_steel,
    )
    right_bench_half.inertial = Inertial.from_geometry(
        Box((0.64, 0.44, 0.40)),
        mass=11.0,
        origin=Origin(xyz=(0.28, 0.17, 0.18)),
    )

    upper_limits = MotionLimits(
        effort=180.0,
        velocity=1.5,
        lower=-0.75,
        upper=0.75,
    )
    lower_limits = MotionLimits(
        effort=160.0,
        velocity=2.0,
        lower=-0.75,
        upper=0.75,
    )

    model.articulation(
        "support_to_left_front_link",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=left_front_link,
        origin=Origin(xyz=(SIDE_PIVOT_X, FRONT_PIVOT_Y, UPPER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=upper_limits,
    )
    model.articulation(
        "support_to_left_rear_link",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=left_rear_link,
        origin=Origin(xyz=(SIDE_PIVOT_X, REAR_PIVOT_Y, UPPER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=upper_limits,
    )
    model.articulation(
        "support_to_right_front_link",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=right_front_link,
        origin=Origin(xyz=(-SIDE_PIVOT_X, FRONT_PIVOT_Y, UPPER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=upper_limits,
    )
    model.articulation(
        "support_to_right_rear_link",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=right_rear_link,
        origin=Origin(xyz=(-SIDE_PIVOT_X, REAR_PIVOT_Y, UPPER_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=upper_limits,
    )
    model.articulation(
        "left_front_link_to_left_bench_half",
        ArticulationType.REVOLUTE,
        parent=left_front_link,
        child=left_bench_half,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=lower_limits,
    )
    model.articulation(
        "right_front_link_to_right_bench_half",
        ArticulationType.REVOLUTE,
        parent=right_front_link,
        child=right_bench_half,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=lower_limits,
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

    support_frame = object_model.get_part("support_frame")
    left_front_link = object_model.get_part("left_front_link")
    left_rear_link = object_model.get_part("left_rear_link")
    right_front_link = object_model.get_part("right_front_link")
    right_rear_link = object_model.get_part("right_rear_link")
    left_bench_half = object_model.get_part("left_bench_half")
    right_bench_half = object_model.get_part("right_bench_half")

    support_to_left_front_link = object_model.get_articulation("support_to_left_front_link")
    support_to_left_rear_link = object_model.get_articulation("support_to_left_rear_link")
    support_to_right_front_link = object_model.get_articulation("support_to_right_front_link")
    support_to_right_rear_link = object_model.get_articulation("support_to_right_rear_link")
    left_front_link_to_left_bench_half = object_model.get_articulation("left_front_link_to_left_bench_half")
    right_front_link_to_right_bench_half = object_model.get_articulation("right_front_link_to_right_bench_half")

    for joint in (
        support_to_left_front_link,
        support_to_left_rear_link,
        support_to_right_front_link,
        support_to_right_rear_link,
        left_front_link_to_left_bench_half,
        right_front_link_to_right_bench_half,
    ):
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"{joint.name} should rotate about the beam-width x axis, got {joint.axis!r}",
        )

    ctx.expect_contact(left_front_link, support_frame)
    ctx.expect_contact(left_rear_link, support_frame)
    ctx.expect_contact(right_front_link, support_frame)
    ctx.expect_contact(right_rear_link, support_frame)
    ctx.expect_contact(left_bench_half, left_front_link)
    ctx.expect_contact(left_bench_half, left_rear_link)
    ctx.expect_contact(right_bench_half, right_front_link)
    ctx.expect_contact(right_bench_half, right_rear_link)
    ctx.expect_gap(
        left_bench_half,
        right_bench_half,
        axis="x",
        min_gap=0.015,
        max_gap=0.05,
        name="bench_halves_leave_small_center_gap",
    )

    def elem_center_z(part, elem: str) -> float:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        assert aabb is not None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    left_rest_position = ctx.part_world_position(left_bench_half)
    assert left_rest_position is not None

    swing_angle = 0.35
    with ctx.pose(
        {
            support_to_left_front_link: swing_angle,
            support_to_left_rear_link: swing_angle,
            support_to_right_front_link: swing_angle,
            support_to_right_rear_link: swing_angle,
            left_front_link_to_left_bench_half: -swing_angle,
            right_front_link_to_right_bench_half: -swing_angle,
        }
    ):
        swung_left_position = ctx.part_world_position(left_bench_half)
        assert swung_left_position is not None
        ctx.check(
            "bench_moves_when_swing_posed",
            abs(swung_left_position[1] - left_rest_position[1]) > 0.10
            and swung_left_position[2] > left_rest_position[2] + 0.03,
            details=(
                "Coordinated upper-link swing and lower-bench counter-rotation should move "
                "the bench on an arc below the beam."
            ),
        )

        ctx.expect_contact(left_bench_half, left_front_link)
        ctx.expect_contact(left_bench_half, left_rear_link)
        ctx.expect_contact(right_bench_half, right_front_link)
        ctx.expect_contact(right_bench_half, right_rear_link)
        ctx.expect_gap(
            left_bench_half,
            right_bench_half,
            axis="x",
            min_gap=0.015,
            max_gap=0.05,
            name="bench_halves_keep_center_gap_in_pose",
        )

        left_front_z = elem_center_z(left_bench_half, "seat_front_slat")
        left_rear_z = elem_center_z(left_bench_half, "seat_rear_slat")
        right_front_z = elem_center_z(right_bench_half, "seat_front_slat")
        right_rear_z = elem_center_z(right_bench_half, "seat_rear_slat")
        ctx.check(
            "left_bench_half_stays_level_in_pose",
            abs(left_front_z - left_rear_z) < 0.002,
            details=f"left half front/rear slat z mismatch: {left_front_z:.4f} vs {left_rear_z:.4f}",
        )
        ctx.check(
            "right_bench_half_stays_level_in_pose",
            abs(right_front_z - right_rear_z) < 0.002,
            details=f"right half front/rear slat z mismatch: {right_front_z:.4f} vs {right_rear_z:.4f}",
        )
        ctx.check(
            "bench_halves_match_height_in_pose",
            abs(left_front_z - right_front_z) < 0.002 and abs(left_rear_z - right_rear_z) < 0.002,
            details="Both bench halves should stay aligned at the same seat height in the swung pose.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
