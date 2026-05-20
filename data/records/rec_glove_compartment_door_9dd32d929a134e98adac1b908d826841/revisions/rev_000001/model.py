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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PANEL_WIDTH = 0.72
PANEL_HEIGHT = 0.40
PANEL_THICKNESS = 0.008
OPENING_WIDTH = 0.50
OPENING_HEIGHT = 0.24

BOX_OUTER_WIDTH = 0.488
BOX_OUTER_HEIGHT = 0.230
BOX_DEPTH = 0.180
BOX_WALL = 0.006
BOX_FLANGE_THICKNESS = 0.006
BOX_FLANGE_WIDTH = 0.024

HINGE_Y = 0.002
HINGE_Z = 0.114
HINGE_RADIUS = 0.0055
HINGE_BOX_KNUCKLE_LENGTH = 0.080
HINGE_LID_KNUCKLE_LENGTH = 0.212
HINGE_KNUCKLE_CENTER_X = 0.146

LID_WIDTH = 0.492
LID_HEIGHT = 0.228
LID_FACE_THICKNESS = 0.006
LID_INNER_FRAME_DEPTH = 0.012
LID_INNER_FRAME_WIDTH = 0.470
LID_INNER_FRAME_HEIGHT = 0.212
LID_INNER_FRAME_BAND = 0.014
LID_LATCH_RADIUS = 0.013

STAY_SIDE_X = 0.194
STAY_CONTACT_OFFSET_X = 0.004
STAY_HUB_RADIUS = 0.005
STAY_HUB_LENGTH = 0.008
STAY_PIVOT_Y = HINGE_Y
STAY_PIVOT_Z = HINGE_Z - 0.008
LID_STAY_PIN_Y = -0.024
LID_STAY_PIN_Z = -0.039

LID_OPEN_ANGLE = 1.10
LATCH_TRAVEL = 0.010
LATCH_HOLE_RADIUS = 0.013
LATCH_CAP_RADIUS = 0.015
LATCH_STEM_RADIUS = 0.0065


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _front_mesh_from_profile(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
    name: str,
):
    geom = ExtrudeGeometry(profile, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _front_mesh_from_profile_with_holes(
    outer_profile: list[tuple[float, float]],
    hole_profiles: list[list[tuple[float, float]]],
    *,
    thickness: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        thickness,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _stay_vector_for_lid_angle(lid_angle: float) -> tuple[float, float]:
    pin_y = LID_STAY_PIN_Y * math.cos(lid_angle) - LID_STAY_PIN_Z * math.sin(lid_angle)
    pin_z = LID_STAY_PIN_Y * math.sin(lid_angle) + LID_STAY_PIN_Z * math.cos(lid_angle)
    return pin_y - (STAY_PIVOT_Y - HINGE_Y), pin_z - (STAY_PIVOT_Z - HINGE_Z)


def _stay_sweep_angle(dy: float, dz: float) -> float:
    return math.atan2(dy, dz)


def _signed_yz_angle(
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    sy, sz = start
    ey, ez = end
    return math.atan2(sy * ez - sz * ey, sy * ey + sz * ez)


STAY_CLOSED_DY, STAY_CLOSED_DZ = _stay_vector_for_lid_angle(0.0)
STAY_LENGTH = math.hypot(STAY_CLOSED_DY, STAY_CLOSED_DZ)
STAY_BAR_PITCH = math.atan2(-STAY_CLOSED_DY, STAY_CLOSED_DZ)
STAY_OPEN_DY, STAY_OPEN_DZ = _stay_vector_for_lid_angle(LID_OPEN_ANGLE)
STAY_OPEN_ANGLE = _signed_yz_angle(
    (STAY_CLOSED_DY, STAY_CLOSED_DZ),
    (STAY_OPEN_DY, STAY_OPEN_DZ),
)


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-9 for a, b in zip(axis, expected))


def _build_stay_part(part) -> None:
    part.visual(
        Cylinder(radius=STAY_HUB_RADIUS, length=STAY_HUB_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        name="lower_hub",
    )
    part.visual(
        Box((STAY_HUB_LENGTH * 0.8, 0.004, STAY_LENGTH + 0.002)),
        origin=Origin(
            xyz=(0.0, STAY_CLOSED_DY * 0.5, STAY_CLOSED_DZ * 0.5),
            rpy=(STAY_BAR_PITCH, 0.0, 0.0),
        ),
        name="stay_bar",
    )
    part.visual(
        Cylinder(radius=STAY_HUB_RADIUS, length=STAY_HUB_LENGTH),
        origin=Origin(
            xyz=(0.0, STAY_CLOSED_DY, STAY_CLOSED_DZ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        name="upper_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_glove_compartment")

    dash_charcoal = model.material("dash_charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    marine_white = model.material("marine_white", rgba=(0.93, 0.95, 0.96, 1.0))
    liner_gray = model.material("liner_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.65, 0.68, 0.72, 1.0))
    latch_black = model.material("latch_black", rgba=(0.12, 0.13, 0.14, 1.0))

    dash_panel = model.part("dash_panel")
    dash_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, (PANEL_HEIGHT - OPENING_HEIGHT) * 0.5)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT * 0.5 + (PANEL_HEIGHT - OPENING_HEIGHT) * 0.25)),
        material=dash_charcoal,
        name="panel_top",
    )
    dash_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, (PANEL_HEIGHT - OPENING_HEIGHT) * 0.5)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_HEIGHT * 0.5 - (PANEL_HEIGHT - OPENING_HEIGHT) * 0.25)),
        material=dash_charcoal,
        name="panel_bottom",
    )
    dash_panel.visual(
        Box(((PANEL_WIDTH - OPENING_WIDTH) * 0.5, PANEL_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH * 0.5 + (PANEL_WIDTH - OPENING_WIDTH) * 0.25, 0.0, 0.0)),
        material=dash_charcoal,
        name="panel_right",
    )
    dash_panel.visual(
        Box(((PANEL_WIDTH - OPENING_WIDTH) * 0.5, PANEL_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH * 0.5 - (PANEL_WIDTH - OPENING_WIDTH) * 0.25, 0.0, 0.0)),
        material=dash_charcoal,
        name="panel_left",
    )
    dash_panel.visual(
        Box((OPENING_WIDTH + 0.042, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.0055, OPENING_HEIGHT * 0.5 + 0.009)),
        material=trim_black,
        name="bezel_top",
    )
    dash_panel.visual(
        Box((OPENING_WIDTH + 0.042, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.0055, -OPENING_HEIGHT * 0.5 - 0.009)),
        material=trim_black,
        name="bezel_bottom",
    )
    dash_panel.visual(
        Box((0.018, 0.003, OPENING_HEIGHT + 0.018)),
        origin=Origin(xyz=(OPENING_WIDTH * 0.5 + 0.009, 0.0055, 0.0)),
        material=trim_black,
        name="bezel_right",
    )
    dash_panel.visual(
        Box((0.018, 0.003, OPENING_HEIGHT + 0.018)),
        origin=Origin(xyz=(-OPENING_WIDTH * 0.5 - 0.009, 0.0055, 0.0)),
        material=trim_black,
        name="bezel_left",
    )

    storage_box = model.part("storage_box")
    flange_y = -PANEL_THICKNESS * 0.5 - BOX_FLANGE_THICKNESS * 0.5
    flange_outer_w = OPENING_WIDTH + 2.0 * BOX_FLANGE_WIDTH
    flange_outer_h = OPENING_HEIGHT + 2.0 * BOX_FLANGE_WIDTH

    storage_box.visual(
        Box((flange_outer_w, BOX_FLANGE_THICKNESS, BOX_FLANGE_WIDTH)),
        origin=Origin(xyz=(0.0, flange_y, OPENING_HEIGHT * 0.5 + BOX_FLANGE_WIDTH * 0.5)),
        material=marine_white,
        name="front_flange_top",
    )
    storage_box.visual(
        Box((flange_outer_w, BOX_FLANGE_THICKNESS, BOX_FLANGE_WIDTH)),
        origin=Origin(xyz=(0.0, flange_y, -OPENING_HEIGHT * 0.5 - BOX_FLANGE_WIDTH * 0.5)),
        material=marine_white,
        name="front_flange_bottom",
    )
    storage_box.visual(
        Box((BOX_FLANGE_WIDTH, BOX_FLANGE_THICKNESS, flange_outer_h)),
        origin=Origin(xyz=(OPENING_WIDTH * 0.5 + BOX_FLANGE_WIDTH * 0.5, flange_y, 0.0)),
        material=marine_white,
        name="front_flange_right",
    )
    storage_box.visual(
        Box((BOX_FLANGE_WIDTH, BOX_FLANGE_THICKNESS, flange_outer_h)),
        origin=Origin(xyz=(-OPENING_WIDTH * 0.5 - BOX_FLANGE_WIDTH * 0.5, flange_y, 0.0)),
        material=marine_white,
        name="front_flange_left",
    )

    shell_center_y = -PANEL_THICKNESS * 0.5 - BOX_DEPTH * 0.5
    shell_top_z = BOX_OUTER_HEIGHT * 0.5 - BOX_WALL * 0.5
    shell_side_x = BOX_OUTER_WIDTH * 0.5 - BOX_WALL * 0.5

    storage_box.visual(
        Box((BOX_OUTER_WIDTH - 2.0 * BOX_WALL, BOX_DEPTH, BOX_WALL)),
        origin=Origin(xyz=(0.0, shell_center_y, shell_top_z)),
        material=liner_gray,
        name="top_wall",
    )
    storage_box.visual(
        Box((BOX_OUTER_WIDTH - 2.0 * BOX_WALL, BOX_DEPTH, BOX_WALL)),
        origin=Origin(xyz=(0.0, shell_center_y, -shell_top_z)),
        material=liner_gray,
        name="bottom_wall",
    )
    storage_box.visual(
        Box((BOX_WALL, BOX_DEPTH, BOX_OUTER_HEIGHT)),
        origin=Origin(xyz=(shell_side_x, shell_center_y, 0.0)),
        material=liner_gray,
        name="right_wall",
    )
    storage_box.visual(
        Box((BOX_WALL, BOX_DEPTH, BOX_OUTER_HEIGHT)),
        origin=Origin(xyz=(-shell_side_x, shell_center_y, 0.0)),
        material=liner_gray,
        name="left_wall",
    )
    storage_box.visual(
        Box((BOX_OUTER_WIDTH - 2.0 * BOX_WALL, BOX_WALL, BOX_OUTER_HEIGHT - 2.0 * BOX_WALL)),
        origin=Origin(
            xyz=(
                0.0,
                -PANEL_THICKNESS * 0.5 - BOX_DEPTH + BOX_WALL * 0.5,
                0.0,
            )
        ),
        material=liner_gray,
        name="back_wall",
    )
    storage_box.visual(
        Box((OPENING_WIDTH + 0.008, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, HINGE_Z)),
        material=marine_white,
        name="hinge_rail",
    )
    storage_box.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_BOX_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(-HINGE_KNUCKLE_CENTER_X, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_gray,
        name="left_hinge_knuckle",
    )
    storage_box.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(-HINGE_KNUCKLE_CENTER_X, -0.011, 0.103)),
        material=marine_white,
        name="left_hinge_web",
    )
    storage_box.visual(
        Box((0.020, 0.0015, 0.009)),
        origin=Origin(xyz=(-HINGE_KNUCKLE_CENTER_X, -0.00425, 0.1105)),
        material=marine_white,
        name="left_hinge_link",
    )
    storage_box.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_BOX_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(HINGE_KNUCKLE_CENTER_X, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_gray,
        name="right_hinge_knuckle",
    )
    storage_box.visual(
        Box((0.020, 0.012, 0.020)),
        origin=Origin(xyz=(HINGE_KNUCKLE_CENTER_X, -0.011, 0.103)),
        material=marine_white,
        name="right_hinge_web",
    )
    storage_box.visual(
        Box((0.020, 0.0015, 0.009)),
        origin=Origin(xyz=(HINGE_KNUCKLE_CENTER_X, -0.00425, 0.1105)),
        material=marine_white,
        name="right_hinge_link",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        pivot_pin_x = sign * (STAY_SIDE_X + STAY_CONTACT_OFFSET_X)
        storage_box.visual(
            Box((0.016, 0.022, 0.022)),
            origin=Origin(
                xyz=(
                    sign * (STAY_SIDE_X + 0.010),
                    STAY_PIVOT_Y - 0.015,
                    STAY_PIVOT_Z - 0.011,
                )
            ),
            material=marine_white,
            name=f"{side_name}_stay_bracket",
        )
        storage_box.visual(
            Box((0.038, 0.012, 0.022)),
            origin=Origin(
                xyz=(
                    sign * 0.223,
                    -0.013,
                    0.095,
                )
            ),
            material=marine_white,
            name=f"{side_name}_stay_brace",
        )
        storage_box.visual(
            Cylinder(radius=STAY_HUB_RADIUS, length=STAY_HUB_LENGTH),
            origin=Origin(
                xyz=(pivot_pin_x, STAY_PIVOT_Y, STAY_PIVOT_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_gray,
            name=f"{side_name}_stay_pivot",
        )

    lid = model.part("lid")
    lid_outer_profile = rounded_rect_profile(LID_WIDTH, LID_HEIGHT, 0.018)
    lid.visual(
        _front_mesh_from_profile_with_holes(
            lid_outer_profile,
            [_circle_profile(LATCH_HOLE_RADIUS)],
            thickness=LID_FACE_THICKNESS,
            name="lid_outer_panel",
        ),
        origin=Origin(xyz=(0.0, 0.006, -LID_HEIGHT * 0.5)),
        material=marine_white,
        name="outer_panel",
    )
    frame_y = -0.003
    lid.visual(
        Box((0.180, LID_INNER_FRAME_DEPTH, LID_INNER_FRAME_BAND)),
        origin=Origin(xyz=(0.0, frame_y, -LID_INNER_FRAME_BAND * 0.5)),
        material=liner_gray,
        name="inner_frame_top",
    )
    lid.visual(
        Box((LID_INNER_FRAME_WIDTH, LID_INNER_FRAME_DEPTH, LID_INNER_FRAME_BAND)),
        origin=Origin(
            xyz=(0.0, frame_y, -(LID_INNER_FRAME_HEIGHT - LID_INNER_FRAME_BAND * 0.5)),
        ),
        material=liner_gray,
        name="inner_frame_bottom",
    )
    inner_side_height = LID_INNER_FRAME_HEIGHT - 2.0 * LID_INNER_FRAME_BAND
    lid.visual(
        Box((LID_INNER_FRAME_BAND, LID_INNER_FRAME_DEPTH, inner_side_height)),
        origin=Origin(
            xyz=(
                LID_INNER_FRAME_WIDTH * 0.5 - LID_INNER_FRAME_BAND * 0.5,
                frame_y,
                -(LID_INNER_FRAME_HEIGHT * 0.5),
            )
        ),
        material=liner_gray,
        name="inner_frame_right",
    )
    lid.visual(
        Box((LID_INNER_FRAME_BAND, LID_INNER_FRAME_DEPTH, inner_side_height)),
        origin=Origin(
            xyz=(
                -LID_INNER_FRAME_WIDTH * 0.5 + LID_INNER_FRAME_BAND * 0.5,
                frame_y,
                -(LID_INNER_FRAME_HEIGHT * 0.5),
            )
        ),
        material=liner_gray,
        name="inner_frame_left",
    )
    lid.visual(
        Box((0.280, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, frame_y, -0.012)),
        material=liner_gray,
        name="hinge_backer",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LID_KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(0.0, HINGE_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_gray,
        name="hinge_knuckle",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        pin_x = sign * (STAY_SIDE_X + STAY_CONTACT_OFFSET_X)
        lid.visual(
            Box((0.032, 0.012, 0.022)),
            origin=Origin(
                xyz=(pin_x + sign * 0.008, -0.016, LID_STAY_PIN_Z),
            ),
            material=liner_gray,
            name=f"{side_name}_stay_mount",
        )
        lid.visual(
            Box((0.010, 0.005, 0.026)),
            origin=Origin(
                xyz=(sign * 0.221, -0.0095, LID_STAY_PIN_Z),
            ),
            material=liner_gray,
            name=f"{side_name}_stay_mount_link",
        )
        lid.visual(
            Cylinder(radius=STAY_HUB_RADIUS, length=STAY_HUB_LENGTH),
            origin=Origin(
                xyz=(pin_x, LID_STAY_PIN_Y, LID_STAY_PIN_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_gray,
            name=f"{side_name}_stay_pin",
        )

    push_latch = model.part("push_latch")
    push_latch.visual(
        Cylinder(radius=LATCH_CAP_RADIUS, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.011, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=latch_black,
        name="button",
    )
    push_latch.visual(
        Cylinder(radius=LATCH_STEM_RADIUS, length=0.016),
        origin=Origin(
            xyz=(0.0, 0.001, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_gray,
        name="button_core",
    )
    push_latch.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_gray,
        name="latch_body",
    )

    left_stay = model.part("left_stay")
    _build_stay_part(left_stay)
    for visual in left_stay.visuals:
        visual.material = hardware_gray

    right_stay = model.part("right_stay")
    _build_stay_part(right_stay)
    for visual in right_stay.visuals:
        visual.material = hardware_gray

    model.articulation(
        "panel_to_box",
        ArticulationType.FIXED,
        parent=dash_panel,
        child=storage_box,
        origin=Origin(),
    )
    model.articulation(
        "box_to_lid",
        ArticulationType.REVOLUTE,
        parent=storage_box,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "lid_to_latch",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=push_latch,
        origin=Origin(xyz=(0.0, 0.0, -LID_HEIGHT * 0.5)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=LATCH_TRAVEL,
        ),
    )
    model.articulation(
        "box_to_left_stay",
        ArticulationType.REVOLUTE,
        parent=storage_box,
        child=left_stay,
        origin=Origin(
            xyz=(-STAY_SIDE_X + STAY_CONTACT_OFFSET_X, STAY_PIVOT_Y, STAY_PIVOT_Z),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-1.40,
            upper=1.40,
        ),
    )
    model.articulation(
        "box_to_right_stay",
        ArticulationType.REVOLUTE,
        parent=storage_box,
        child=right_stay,
        origin=Origin(
            xyz=(STAY_SIDE_X - STAY_CONTACT_OFFSET_X, STAY_PIVOT_Y, STAY_PIVOT_Z),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-1.40,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dash_panel = object_model.get_part("dash_panel")
    storage_box = object_model.get_part("storage_box")
    lid = object_model.get_part("lid")
    push_latch = object_model.get_part("push_latch")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")

    box_to_lid = object_model.get_articulation("box_to_lid")
    lid_to_latch = object_model.get_articulation("lid_to_latch")
    box_to_left_stay = object_model.get_articulation("box_to_left_stay")
    box_to_right_stay = object_model.get_articulation("box_to_right_stay")

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

    ctx.expect_contact(storage_box, dash_panel, name="box_is_mounted_to_dash")
    ctx.check(
        "lid_hinge_axis_is_horizontal",
        _axis_matches(box_to_lid.axis, (1.0, 0.0, 0.0)),
        f"Expected lid hinge axis (1, 0, 0), got {box_to_lid.axis}",
    )
    ctx.check(
        "latch_axis_points_inward",
        _axis_matches(lid_to_latch.axis, (0.0, -1.0, 0.0)),
        f"Expected latch axis (0, -1, 0), got {lid_to_latch.axis}",
    )
    ctx.check(
        "stay_axes_match_hinge_axis",
        _axis_matches(box_to_left_stay.axis, (1.0, 0.0, 0.0))
        and _axis_matches(box_to_right_stay.axis, (1.0, 0.0, 0.0)),
        f"Stay axes were {box_to_left_stay.axis} and {box_to_right_stay.axis}",
    )
    ctx.check(
        "lid_motion_limits_open_upward",
        box_to_lid.motion_limits is not None
        and box_to_lid.motion_limits.lower is not None
        and box_to_lid.motion_limits.upper is not None
        and abs(box_to_lid.motion_limits.lower - 0.0) < 1e-9
        and box_to_lid.motion_limits.upper >= 1.2,
        f"Unexpected lid limits: {box_to_lid.motion_limits}",
    )

    with ctx.pose(
        {
            box_to_lid: 0.0,
            lid_to_latch: 0.0,
            box_to_left_stay: 0.0,
            box_to_right_stay: 0.0,
        }
    ):
        ctx.expect_contact(
            lid,
            storage_box,
            elem_a="hinge_knuckle",
            elem_b="left_hinge_knuckle",
            name="left_hinge_knuckle_contact_closed",
        )
        ctx.expect_contact(
            lid,
            storage_box,
            elem_a="hinge_knuckle",
            elem_b="right_hinge_knuckle",
            name="right_hinge_knuckle_contact_closed",
        )
        ctx.expect_contact(
            left_stay,
            storage_box,
            elem_a="lower_hub",
            elem_b="left_stay_pivot",
            name="left_stay_lower_pivot_contact_closed",
        )
        ctx.expect_contact(
            right_stay,
            storage_box,
            elem_a="lower_hub",
            elem_b="right_stay_pivot",
            name="right_stay_lower_pivot_contact_closed",
        )
        ctx.expect_contact(
            left_stay,
            lid,
            elem_a="upper_hub",
            elem_b="left_stay_pin",
            name="left_stay_upper_pivot_contact_closed",
        )
        ctx.expect_contact(
            right_stay,
            lid,
            elem_a="upper_hub",
            elem_b="right_stay_pin",
            name="right_stay_upper_pivot_contact_closed",
        )
        ctx.expect_contact(
            push_latch,
            lid,
            elem_a="button",
            elem_b="outer_panel",
            name="latch_button_is_guided_in_lid",
        )
        ctx.expect_overlap(
            lid,
            storage_box,
            axes="xz",
            min_overlap=0.18,
            name="lid_covers_storage_opening_when_closed",
        )

    latch_rest_aabb = ctx.part_world_aabb(push_latch)
    assert latch_rest_aabb is not None
    with ctx.pose({lid_to_latch: LATCH_TRAVEL}):
        latch_pressed_aabb = ctx.part_world_aabb(push_latch)
        assert latch_pressed_aabb is not None
        ctx.check(
            "latch_translates_inward",
            latch_pressed_aabb[1][1] < latch_rest_aabb[1][1] - 0.009,
            f"Rest latch max y {latch_rest_aabb[1][1]:.4f}, pressed max y {latch_pressed_aabb[1][1]:.4f}",
        )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    assert lid_closed_aabb is not None
    with ctx.pose(
        {
            box_to_lid: LID_OPEN_ANGLE,
            box_to_left_stay: STAY_OPEN_ANGLE,
            box_to_right_stay: STAY_OPEN_ANGLE,
            lid_to_latch: LATCH_TRAVEL * 0.5,
        }
    ):
        ctx.expect_contact(
            lid,
            storage_box,
            elem_a="hinge_knuckle",
            elem_b="left_hinge_knuckle",
            name="left_hinge_knuckle_contact_open",
        )
        ctx.expect_contact(
            lid,
            storage_box,
            elem_a="hinge_knuckle",
            elem_b="right_hinge_knuckle",
            name="right_hinge_knuckle_contact_open",
        )
        ctx.expect_contact(
            left_stay,
            lid,
            elem_a="upper_hub",
            elem_b="left_stay_pin",
            name="left_stay_upper_pivot_contact_open",
        )
        ctx.expect_contact(
            right_stay,
            lid,
            elem_a="upper_hub",
            elem_b="right_stay_pin",
            name="right_stay_upper_pivot_contact_open",
        )
        ctx.expect_contact(
            push_latch,
            lid,
            elem_a="button",
            elem_b="outer_panel",
            name="latch_remains_captured_when_pressed",
        )
        lid_open_aabb = ctx.part_world_aabb(lid)
        assert lid_open_aabb is not None
        ctx.check(
            "lid_swings_upward_and_outward",
            lid_open_aabb[1][1] > lid_closed_aabb[1][1] + 0.09
            and lid_open_aabb[0][2] > lid_closed_aabb[0][2] + 0.08,
            (
                f"Closed lid aabb={lid_closed_aabb}, "
                f"open lid aabb={lid_open_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
