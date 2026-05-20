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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cut_opening_on_face,
    mesh_from_geometry,
    rounded_rect_profile,
)


DOOR_OPEN_ANGLE = 0.95
STAY_TRACK_ANGLE = 0.75
BUTTON_TRAVEL = 0.006


def _door_outer_panel_mesh():
    panel = ExtrudeGeometry.centered(
        rounded_rect_profile(0.344, 0.158, 0.018, corner_segments=10),
        0.012,
        cap=True,
        closed=True,
    )
    panel = cut_opening_on_face(
        panel,
        face="+z",
        opening_profile=rounded_rect_profile(0.040, 0.022, 0.004, corner_segments=8),
        depth=0.016,
        offset=(0.0, 0.021),
    )
    return mesh_from_geometry(panel, "glovebox_door_outer_panel")


def _button_bezel_mesh():
    bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.062, 0.032, 0.006, corner_segments=8),
        [rounded_rect_profile(0.040, 0.022, 0.004, corner_segments=8)],
        0.004,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(bezel, "glovebox_button_bezel")


def _axis_matches(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) <= 1e-6 for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_car_glove_compartment")

    dash_skin = model.material("dash_skin", rgba=(0.18, 0.18, 0.19, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    door_skin = model.material("door_skin", rgba=(0.22, 0.23, 0.24, 1.0))
    inner_trim = model.material("inner_trim", rgba=(0.14, 0.15, 0.16, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    latch_trim = model.material("latch_trim", rgba=(0.73, 0.75, 0.78, 1.0))
    button_finish = model.material("button_finish", rgba=(0.83, 0.85, 0.87, 1.0))
    stay_finish = model.material("stay_finish", rgba=(0.61, 0.63, 0.66, 1.0))

    dashboard_bin = model.part("dashboard_bin")
    dashboard_bin.visual(
        Box((0.620, 0.070, 0.095)),
        origin=Origin(xyz=(0.0, -0.040, 0.223)),
        material=dash_skin,
        name="dash_upper_pad",
    )
    dashboard_bin.visual(
        Box((0.620, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, -0.045, 0.008)),
        material=dash_skin,
        name="dash_lower_rail",
    )
    dashboard_bin.visual(
        Box((0.130, 0.060, 0.190)),
        origin=Origin(xyz=(-0.246, -0.032, 0.110)),
        material=dash_skin,
        name="dash_left_cheek",
    )
    dashboard_bin.visual(
        Box((0.130, 0.060, 0.190)),
        origin=Origin(xyz=(0.246, -0.032, 0.110)),
        material=dash_skin,
        name="dash_right_cheek",
    )
    dashboard_bin.visual(
        Box((0.366, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, 0.186)),
        material=dash_skin,
        name="opening_top_frame",
    )
    dashboard_bin.visual(
        Box((0.366, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.023, 0.014)),
        material=dash_skin,
        name="opening_lower_sill",
    )
    dashboard_bin.visual(
        Box((0.012, 0.018, 0.160)),
        origin=Origin(xyz=(-0.179, -0.010, 0.102)),
        material=dash_skin,
        name="left_jamb",
    )
    dashboard_bin.visual(
        Box((0.012, 0.018, 0.160)),
        origin=Origin(xyz=(0.179, -0.010, 0.102)),
        material=dash_skin,
        name="right_jamb",
    )
    dashboard_bin.visual(
        Box((0.342, 0.200, 0.004)),
        origin=Origin(xyz=(0.0, -0.106, 0.186)),
        material=bin_plastic,
        name="bin_roof",
    )
    dashboard_bin.visual(
        Box((0.342, 0.210, 0.004)),
        origin=Origin(xyz=(0.0, -0.110, 0.018)),
        material=bin_plastic,
        name="bin_floor",
    )
    dashboard_bin.visual(
        Box((0.004, 0.206, 0.164)),
        origin=Origin(xyz=(-0.171, -0.105, 0.102)),
        material=bin_plastic,
        name="left_bin_wall",
    )
    dashboard_bin.visual(
        Box((0.004, 0.206, 0.164)),
        origin=Origin(xyz=(0.171, -0.105, 0.102)),
        material=bin_plastic,
        name="right_bin_wall",
    )
    dashboard_bin.visual(
        Box((0.338, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, -0.208, 0.102)),
        material=bin_plastic,
        name="bin_back_wall",
    )
    dashboard_bin.visual(
        Box((0.042, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.028, 0.177)),
        material=inner_trim,
        name="striker_housing",
    )
    dashboard_bin.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(-0.132, -0.004, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_hinge_knuckle",
    )
    dashboard_bin.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(0.132, -0.004, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_hinge_knuckle",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        dashboard_bin.visual(
            Box((0.010, 0.090, 0.004)),
            origin=Origin(xyz=(sign * 0.166, -0.034, 0.071)),
            material=inner_trim,
            name=f"{side}_guide_lower",
        )
        dashboard_bin.visual(
            Box((0.010, 0.090, 0.004)),
            origin=Origin(xyz=(sign * 0.166, -0.034, 0.101)),
            material=inner_trim,
            name=f"{side}_guide_upper",
        )
        dashboard_bin.visual(
            Box((0.010, 0.004, 0.032)),
            origin=Origin(xyz=(sign * 0.166, -0.079, 0.086)),
            material=inner_trim,
            name=f"{side}_guide_back",
        )
        dashboard_bin.visual(
            Box((0.010, 0.004, 0.032)),
            origin=Origin(xyz=(sign * 0.166, 0.011, 0.086)),
            material=inner_trim,
            name=f"{side}_guide_front",
        )

    dashboard_bin.inertial = Inertial.from_geometry(
        Box((0.620, 0.220, 0.270)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.075, 0.135)),
    )

    door_panel = model.part("door_panel")
    door_panel.visual(
        _door_outer_panel_mesh(),
        origin=Origin(xyz=(0.0, 0.006, 0.083), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=door_skin,
        name="outer_panel",
    )
    door_panel.visual(
        Box((0.312, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.148)),
        material=inner_trim,
        name="inner_top_rib",
    )
    door_panel.visual(
        Box((0.312, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, 0.009)),
        material=inner_trim,
        name="inner_bottom_rib",
    )
    door_panel.visual(
        Box((0.008, 0.018, 0.118)),
        origin=Origin(xyz=(-0.146, -0.004, 0.082)),
        material=inner_trim,
        name="inner_left_rib",
    )
    door_panel.visual(
        Box((0.008, 0.018, 0.118)),
        origin=Origin(xyz=(0.146, -0.004, 0.082)),
        material=inner_trim,
        name="inner_right_rib",
    )
    door_panel.visual(
        _button_bezel_mesh(),
        origin=Origin(xyz=(0.0, 0.013, 0.104), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_trim,
        name="latch_bezel",
    )
    door_panel.visual(
        Box((0.004, 0.016, 0.022)),
        origin=Origin(xyz=(-0.016, 0.014, 0.104)),
        material=latch_trim,
        name="latch_guide_left",
    )
    door_panel.visual(
        Box((0.004, 0.016, 0.022)),
        origin=Origin(xyz=(0.016, 0.014, 0.104)),
        material=latch_trim,
        name="latch_guide_right",
    )
    door_panel.visual(
        Box((0.028, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.014, 0.117)),
        material=latch_trim,
        name="latch_guide_top",
    )
    door_panel.visual(
        Box((0.028, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.014, 0.091)),
        material=latch_trim,
        name="latch_guide_bottom",
    )
    door_panel.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(-0.016, -0.008, 0.104)),
        material=inner_trim,
        name="latch_left_wall",
    )
    door_panel.visual(
        Box((0.006, 0.018, 0.024)),
        origin=Origin(xyz=(0.016, -0.008, 0.104)),
        material=inner_trim,
        name="latch_right_wall",
    )
    door_panel.visual(
        Box((0.038, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.008, 0.116)),
        material=inner_trim,
        name="latch_upper_wall",
    )
    door_panel.visual(
        Box((0.038, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.008, 0.092)),
        material=inner_trim,
        name="latch_lower_wall",
    )
    door_panel.visual(
        Box((0.038, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, -0.017, 0.104)),
        material=inner_trim,
        name="latch_back_wall",
    )
    door_panel.visual(
        Cylinder(radius=0.006, length=0.186),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="door_hinge_knuckle",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        door_panel.visual(
            Box((0.004, 0.014, 0.020)),
            origin=Origin(xyz=(sign * 0.149, -0.006, 0.078)),
            material=hinge_metal,
            name=f"{side}_stay_bracket_inner",
        )
        door_panel.visual(
            Box((0.004, 0.014, 0.020)),
            origin=Origin(xyz=(sign * 0.163, -0.006, 0.078)),
            material=hinge_metal,
            name=f"{side}_stay_bracket_outer",
        )
    door_panel.inertial = Inertial.from_geometry(
        Box((0.344, 0.034, 0.162)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.000, 0.081)),
    )

    model.articulation(
        "dashboard_to_door",
        ArticulationType.REVOLUTE,
        parent=dashboard_bin,
        child=door_panel,
        origin=Origin(xyz=(0.0, -0.004, 0.016)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    push_button = model.part("push_button")
    push_button.visual(
        Box((0.028, 0.008, 0.018)),
        material=button_finish,
        name="button_cap",
    )
    push_button.inertial = Inertial.from_geometry(
        Box((0.028, 0.008, 0.018)),
        mass=0.03,
        origin=Origin(),
    )
    model.articulation(
        "door_to_push_button",
        ArticulationType.PRISMATIC,
        parent=door_panel,
        child=push_button,
        origin=Origin(xyz=(0.0, 0.018, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        stay_link = model.part(f"{side}_stay_link")
        stay_link.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name="pivot_eye",
        )
        stay_link.visual(
            Box((0.006, 0.058, 0.004)),
            origin=Origin(xyz=(0.0, -0.029, 0.0)),
            material=stay_finish,
            name="strap",
        )
        stay_link.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name="tip",
        )
        stay_link.inertial = Inertial.from_geometry(
            Box((0.008, 0.060, 0.010)),
            mass=0.04,
            origin=Origin(xyz=(0.0, -0.028, 0.0)),
        )
        model.articulation(
            f"door_to_{side}_stay",
            ArticulationType.REVOLUTE,
            parent=door_panel,
            child=stay_link,
            origin=Origin(xyz=(sign * 0.156, -0.006, 0.078)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.2,
                lower=0.0,
                upper=0.85,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard_bin = object_model.get_part("dashboard_bin")
    door_panel = object_model.get_part("door_panel")
    push_button = object_model.get_part("push_button")
    left_stay = object_model.get_part("left_stay_link")
    right_stay = object_model.get_part("right_stay_link")

    door_hinge = object_model.get_articulation("dashboard_to_door")
    button_slide = object_model.get_articulation("door_to_push_button")
    left_stay_joint = object_model.get_articulation("door_to_left_stay")
    right_stay_joint = object_model.get_articulation("door_to_right_stay")

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

    door_limits = door_hinge.motion_limits
    button_limits = button_slide.motion_limits
    left_limits = left_stay_joint.motion_limits
    right_limits = right_stay_joint.motion_limits

    ctx.check(
        "door_hinge_definition",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(door_hinge.axis, (-1.0, 0.0, 0.0))
        and door_limits is not None
        and door_limits.lower is not None
        and door_limits.upper is not None
        and abs(door_limits.lower - 0.0) <= 1e-6
        and door_limits.upper >= DOOR_OPEN_ANGLE,
        f"axis={door_hinge.axis}, limits={door_limits}",
    )
    ctx.check(
        "button_slide_definition",
        button_slide.articulation_type == ArticulationType.PRISMATIC
        and _axis_matches(button_slide.axis, (0.0, -1.0, 0.0))
        and button_limits is not None
        and button_limits.lower is not None
        and button_limits.upper is not None
        and abs(button_limits.lower - 0.0) <= 1e-6
        and button_limits.upper >= BUTTON_TRAVEL,
        f"axis={button_slide.axis}, limits={button_limits}",
    )
    ctx.check(
        "stay_link_definitions",
        left_stay_joint.articulation_type == ArticulationType.REVOLUTE
        and right_stay_joint.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(left_stay_joint.axis, (1.0, 0.0, 0.0))
        and _axis_matches(right_stay_joint.axis, (1.0, 0.0, 0.0))
        and left_limits is not None
        and right_limits is not None
        and left_limits.upper is not None
        and right_limits.upper is not None
        and left_limits.upper >= STAY_TRACK_ANGLE
        and right_limits.upper >= STAY_TRACK_ANGLE,
        (
            f"left_axis={left_stay_joint.axis}, left_limits={left_limits}, "
            f"right_axis={right_stay_joint.axis}, right_limits={right_limits}"
        ),
    )

    with ctx.pose(
        {
            door_hinge: 0.0,
            button_slide: 0.0,
            left_stay_joint: 0.0,
            right_stay_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            door_panel,
            dashboard_bin,
            axis="x",
            positive_elem="outer_panel",
            negative_elem="left_jamb",
            min_gap=0.0005,
            max_gap=0.012,
            name="door_left_clearance_closed",
        )
        ctx.expect_gap(
            dashboard_bin,
            door_panel,
            axis="x",
            positive_elem="right_jamb",
            negative_elem="outer_panel",
            min_gap=0.0005,
            max_gap=0.012,
            name="door_right_clearance_closed",
        )
        ctx.expect_gap(
            dashboard_bin,
            door_panel,
            axis="z",
            positive_elem="opening_top_frame",
            negative_elem="outer_panel",
            min_gap=0.001,
            max_gap=0.012,
            name="door_top_clearance_closed",
        )
        ctx.expect_within(
            push_button,
            door_panel,
            axes="xz",
            inner_elem="button_cap",
            outer_elem="latch_bezel",
            margin=0.0,
            name="button_cap_centered_in_bezel_closed",
        )
        for side, stay_link in (("left", left_stay), ("right", right_stay)):
            ctx.expect_gap(
                stay_link,
                dashboard_bin,
                axis="z",
                positive_elem="tip",
                negative_elem=f"{side}_guide_lower",
                min_gap=0.010,
                max_gap=0.024,
                name=f"{side}_tip_above_lower_guide_closed",
            )
            ctx.expect_gap(
                dashboard_bin,
                stay_link,
                axis="z",
                positive_elem=f"{side}_guide_upper",
                negative_elem="tip",
                min_gap=0.0,
                max_gap=0.010,
                name=f"{side}_tip_below_upper_guide_closed",
            )
            ctx.expect_gap(
                stay_link,
                dashboard_bin,
                axis="y",
                positive_elem="tip",
                negative_elem=f"{side}_guide_back",
                min_gap=0.004,
                max_gap=0.022,
                name=f"{side}_tip_ahead_of_back_stop_closed",
            )
            ctx.expect_gap(
                dashboard_bin,
                stay_link,
                axis="y",
                positive_elem=f"{side}_guide_front",
                negative_elem="tip",
                min_gap=0.055,
                max_gap=0.080,
                name=f"{side}_tip_behind_front_stop_closed",
            )

    with ctx.pose(
        {
            door_hinge: DOOR_OPEN_ANGLE,
            button_slide: BUTTON_TRAVEL,
            left_stay_joint: STAY_TRACK_ANGLE,
            right_stay_joint: STAY_TRACK_ANGLE,
        }
    ):
        for side, stay_link in (("left", left_stay), ("right", right_stay)):
            ctx.expect_gap(
                stay_link,
                dashboard_bin,
                axis="z",
                positive_elem="tip",
                negative_elem=f"{side}_guide_lower",
                min_gap=0.0,
                max_gap=0.008,
                name=f"{side}_tip_above_lower_guide_open",
            )
            ctx.expect_gap(
                dashboard_bin,
                stay_link,
                axis="z",
                positive_elem=f"{side}_guide_upper",
                negative_elem="tip",
                min_gap=0.012,
                max_gap=0.032,
                name=f"{side}_tip_below_upper_guide_open",
            )
            ctx.expect_gap(
                stay_link,
                dashboard_bin,
                axis="y",
                positive_elem="tip",
                negative_elem=f"{side}_guide_back",
                min_gap=0.070,
                max_gap=0.086,
                name=f"{side}_tip_ahead_of_back_stop_open",
            )
            ctx.expect_gap(
                dashboard_bin,
                stay_link,
                axis="y",
                positive_elem=f"{side}_guide_front",
                negative_elem="tip",
                min_gap=0.002,
                max_gap=0.012,
                name=f"{side}_tip_behind_front_stop_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
