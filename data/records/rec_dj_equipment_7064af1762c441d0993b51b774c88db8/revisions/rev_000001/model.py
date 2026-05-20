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


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_controller_with_laptop_stand")

    body_charcoal = model.material("body_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    deck_black = model.material("deck_black", rgba=(0.08, 0.09, 0.10, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.32, 0.34, 0.36, 1.0))
    platter_black = model.material("platter_black", rgba=(0.11, 0.11, 0.12, 1.0))
    platter_ring = model.material("platter_ring", rgba=(0.63, 0.65, 0.68, 1.0))
    pad_grey = model.material("pad_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    cap_black = model.material("cap_black", rgba=(0.13, 0.13, 0.14, 1.0))
    marker_orange = model.material("marker_orange", rgba=(0.92, 0.48, 0.10, 1.0))
    stand_metal = model.material("stand_metal", rgba=(0.46, 0.48, 0.51, 1.0))
    stand_pad = model.material("stand_pad", rgba=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("housing")

    body_w = 0.72
    body_d = 0.34
    body_h = 0.055
    wall_t = 0.008
    bottom_t = 0.004
    deck_t = 0.006
    top_z = body_h

    housing.visual(
        Box((body_w, body_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=body_charcoal,
        name="bottom_pan",
    )
    housing.visual(
        Box((wall_t, body_d, body_h - bottom_t)),
        origin=Origin(
            xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, bottom_t + (body_h - bottom_t) * 0.5)
        ),
        material=body_charcoal,
        name="left_wall",
    )
    housing.visual(
        Box((wall_t, body_d, body_h - bottom_t)),
        origin=Origin(
            xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, bottom_t + (body_h - bottom_t) * 0.5)
        ),
        material=body_charcoal,
        name="right_wall",
    )
    housing.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h - bottom_t)),
        origin=Origin(
            xyz=(0.0, -body_d * 0.5 + wall_t * 0.5, bottom_t + (body_h - bottom_t) * 0.5)
        ),
        material=body_charcoal,
        name="front_wall",
    )
    housing.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h - bottom_t)),
        origin=Origin(
            xyz=(0.0, body_d * 0.5 - wall_t * 0.5, bottom_t + (body_h - bottom_t) * 0.5)
        ),
        material=body_charcoal,
        name="rear_wall",
    )

    housing.visual(
        Box((0.27, 0.32, deck_t)),
        origin=Origin(xyz=(-0.225, 0.0, top_z - deck_t * 0.5)),
        material=deck_black,
        name="left_deck",
    )
    housing.visual(
        Box((0.27, 0.32, deck_t)),
        origin=Origin(xyz=(0.225, 0.0, top_z - deck_t * 0.5)),
        material=deck_black,
        name="right_deck",
    )
    housing.visual(
        Box((0.18, 0.254, deck_t)),
        origin=Origin(xyz=(0.0, 0.031, top_z - deck_t * 0.5)),
        material=deck_black,
        name="mixer_deck",
    )
    housing.visual(
        Box((0.18, 0.042, deck_t)),
        origin=Origin(xyz=(0.0, -0.149, top_z - deck_t * 0.5)),
        material=deck_black,
        name="front_center_deck",
    )

    housing.visual(
        Box((0.18, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.100, top_z + 0.002)),
        material=trim_grey,
        name="crossfader_rail_upper",
    )
    housing.visual(
        Box((0.18, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.130, top_z + 0.002)),
        material=trim_grey,
        name="crossfader_rail_lower",
    )
    housing.visual(
        Box((0.12, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, -0.115, top_z + 0.002)),
        material=trim_grey,
        name="crossfader_slot_bezel",
    )

    left_center = (-0.22, 0.03)
    right_center = (0.22, 0.03)
    for side_name, cx, cy in (
        ("left", left_center[0], left_center[1]),
        ("right", right_center[0], right_center[1]),
    ):
        housing.visual(
            Cylinder(radius=0.082, length=0.002),
            origin=Origin(xyz=(cx, cy, top_z + 0.001)),
            material=trim_grey,
            name=f"{side_name}_platter_bezel",
        )
        housing.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(cx, cy, top_z + 0.002)),
            material=trim_grey,
            name=f"{side_name}_bearing_post",
        )
        housing.visual(
            Cylinder(radius=0.060, length=0.0015),
            origin=Origin(xyz=(cx, cy, top_z + 0.00075)),
            material=body_charcoal,
            name=f"{side_name}_platter_recess",
        )

    housing.visual(
        Box((0.64, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.163, 0.012)),
        material=trim_grey,
        name="front_trim",
    )

    for x_pos in (-0.315, 0.315):
        housing.visual(
            Box((0.014, 0.145, 0.003)),
            origin=Origin(xyz=(x_pos, 0.005, top_z + 0.0015)),
            material=trim_grey,
            name=f"pitch_strip_{'left' if x_pos < 0.0 else 'right'}",
        )

    pad_offsets = (-0.028, 0.028)
    for side_name, base_x in (("left", -0.220), ("right", 0.220)):
        for row, y_pos in enumerate((-0.115, -0.080)):
            for col, x_delta in enumerate(pad_offsets):
                housing.visual(
                    Box((0.024, 0.024, 0.004)),
                    origin=Origin(xyz=(base_x + x_delta, y_pos, top_z + 0.002)),
                    material=pad_grey,
                    name=f"{side_name}_pad_{row}_{col}",
                )
        housing.visual(
            Box((0.050, 0.014, 0.004)),
            origin=Origin(xyz=(base_x, -0.030, top_z + 0.002)),
            material=trim_grey,
            name=f"{side_name}_transport_bar",
        )
        housing.visual(
            Box((0.036, 0.010, 0.003)),
            origin=Origin(xyz=(base_x, -0.002, top_z + 0.0015)),
            material=trim_grey,
            name=f"{side_name}_load_button",
        )

    for channel_x in (-0.030, 0.030):
        for knob_y in (0.102, 0.066, 0.030):
            housing.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(channel_x, knob_y, top_z + 0.005)),
                material=cap_black,
                name=f"eq_knob_{'l' if channel_x < 0.0 else 'r'}_{int((knob_y + 0.001) * 1000)}",
            )
        housing.visual(
            Box((0.012, 0.060, 0.004)),
            origin=Origin(xyz=(channel_x, -0.015, top_z + 0.002)),
            material=trim_grey,
            name=f"channel_fader_slot_{'l' if channel_x < 0.0 else 'r'}",
        )
        housing.visual(
            Box((0.018, 0.014, 0.014)),
            origin=Origin(xyz=(channel_x, -0.028, top_z + 0.007)),
            material=cap_black,
            name=f"channel_fader_cap_{'l' if channel_x < 0.0 else 'r'}",
        )

    housing.visual(
        Box((0.054, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.137, top_z + 0.003)),
        material=trim_grey,
        name="rear_hinge_bridge",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(
            xyz=(-0.125, 0.158, top_z + 0.012),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_grey,
        name="left_hinge_barrel",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(
            xyz=(0.125, 0.158, top_z + 0.012),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_grey,
        name="right_hinge_barrel",
    )
    housing.inertial = Inertial.from_geometry(
        Box((body_w, body_d, 0.075)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
    )

    def add_platter(part_name: str) -> None:
        platter = model.part(part_name)
        platter.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=platter_ring,
            name="spindle",
        )
        platter.visual(
            Cylinder(radius=0.074, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=platter_black,
            name="platter_disc",
        )
        platter.visual(
            Cylinder(radius=0.056, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0195)),
            material=platter_ring,
            name="top_ring",
        )
        platter.visual(
            Box((0.016, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, 0.056, 0.0225)),
            material=marker_orange,
            name="cue_marker",
        )
        platter.inertial = Inertial.from_geometry(
            Cylinder(radius=0.074, length=0.022),
            mass=0.28,
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
        )

    add_platter("left_platter")
    add_platter("right_platter")

    model.articulation(
        "housing_to_left_platter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="left_platter",
        origin=Origin(xyz=(left_center[0], left_center[1], top_z + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=8.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "housing_to_right_platter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="right_platter",
        origin=Origin(xyz=(right_center[0], right_center[1], top_z + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=8.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.030, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=platter_ring,
        name="carriage",
    )
    crossfader.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cap_black,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.004, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=cap_black,
        name="fader_stem",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.030, 0.034, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.115, top_z + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.20,
            lower=-0.070,
            upper=0.070,
        ),
    )

    laptop_stand = model.part("laptop_stand")
    stand_open_angle = math.radians(58.0)
    stand_cos = math.cos(stand_open_angle)
    stand_sin = math.sin(stand_open_angle)
    support_len = 0.170
    support_half = support_len * 0.5
    support_center_y = stand_cos * support_half
    support_center_z = stand_sin * support_half
    tray_depth = 0.170
    tray_half = tray_depth * 0.5
    tray_center_y = stand_cos * (support_len + tray_half)
    tray_center_z = stand_sin * (support_len + tray_half)
    lip_offset_y = stand_cos * (support_len + 0.010)
    lip_offset_z = stand_sin * (support_len + 0.010)
    laptop_stand.visual(
        Cylinder(radius=0.012, length=0.160),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=stand_metal,
        name="center_hinge_barrel",
    )
    laptop_stand.visual(
        Box((0.080, support_len, 0.014)),
        origin=Origin(
            xyz=(0.0, support_center_y, support_center_z),
            rpy=(stand_open_angle, 0.0, 0.0),
        ),
        material=stand_metal,
        name="support_arm",
    )
    laptop_stand.visual(
        Box((0.300, tray_depth, 0.008)),
        origin=Origin(
            xyz=(0.0, tray_center_y, tray_center_z),
            rpy=(stand_open_angle, 0.0, 0.0),
        ),
        material=stand_metal,
        name="tray_panel",
    )
    laptop_stand.visual(
        Box((0.260, 0.014, 0.020)),
        origin=Origin(
            xyz=(0.0, lip_offset_y, lip_offset_z),
            rpy=(stand_open_angle, 0.0, 0.0),
        ),
        material=stand_pad,
        name="tray_lip",
    )
    laptop_stand.inertial = Inertial.from_geometry(
        Box((0.320, 0.260, 0.090)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.175, 0.110)),
    )

    model.articulation(
        "housing_to_laptop_stand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=laptop_stand,
        origin=Origin(xyz=(0.0, 0.158, top_z + 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.6,
            lower=-0.90,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_platter = object_model.get_part("left_platter")
    right_platter = object_model.get_part("right_platter")
    crossfader = object_model.get_part("crossfader")
    laptop_stand = object_model.get_part("laptop_stand")

    left_joint = object_model.get_articulation("housing_to_left_platter")
    right_joint = object_model.get_articulation("housing_to_right_platter")
    crossfader_joint = object_model.get_articulation("housing_to_crossfader")
    stand_joint = object_model.get_articulation("housing_to_laptop_stand")

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

    ctx.expect_contact(left_platter, housing, name="left_platter_contacts_bearing")
    ctx.expect_contact(right_platter, housing, name="right_platter_contacts_bearing")
    ctx.expect_contact(crossfader, housing, name="crossfader_contacts_rail")
    ctx.expect_contact(laptop_stand, housing, name="stand_contacts_hinge")

    ctx.expect_gap(
        left_platter,
        housing,
        axis="z",
        min_gap=0.009,
        max_gap=0.013,
        positive_elem="platter_disc",
        negative_elem="left_deck",
        name="left_platter_disc_sits_proud_of_deck",
    )
    ctx.expect_gap(
        right_platter,
        housing,
        axis="z",
        min_gap=0.009,
        max_gap=0.013,
        positive_elem="platter_disc",
        negative_elem="right_deck",
        name="right_platter_disc_sits_proud_of_deck",
    )
    ctx.expect_overlap(
        left_platter,
        housing,
        axes="xy",
        min_overlap=0.12,
        elem_a="platter_disc",
        name="left_platter_centered_over_left_deck",
    )
    ctx.expect_overlap(
        right_platter,
        housing,
        axes="xy",
        min_overlap=0.12,
        elem_a="platter_disc",
        name="right_platter_centered_over_right_deck",
    )
    ctx.expect_overlap(
        crossfader,
        housing,
        axes="x",
        min_overlap=0.030,
        elem_a="carriage",
        elem_b="crossfader_slot_bezel",
        name="crossfader_stays_over_slot_region",
    )

    axis_tuple = lambda joint: tuple(round(float(v), 6) for v in joint.axis)
    ctx.check(
        "platter_axes_are_vertical",
        axis_tuple(left_joint) == (0.0, 0.0, 1.0) and axis_tuple(right_joint) == (0.0, 0.0, 1.0),
        details=f"Left axis={left_joint.axis}, right axis={right_joint.axis}",
    )
    ctx.check(
        "crossfader_axis_is_horizontal",
        axis_tuple(crossfader_joint) == (1.0, 0.0, 0.0),
        details=f"Crossfader axis={crossfader_joint.axis}",
    )
    ctx.check(
        "stand_axis_is_rear_hinge",
        axis_tuple(stand_joint) == (1.0, 0.0, 0.0),
        details=f"Stand axis={stand_joint.axis}",
    )

    left_marker_rest = ctx.part_element_world_aabb(left_platter, elem="cue_marker")
    assert left_marker_rest is not None
    left_marker_rest_center = _aabb_center(left_marker_rest)
    with ctx.pose({left_joint: math.pi * 0.5}):
        left_marker_turned = ctx.part_element_world_aabb(left_platter, elem="cue_marker")
        assert left_marker_turned is not None
        left_marker_turned_center = _aabb_center(left_marker_turned)
        ctx.check(
            "left_platter_marker_rotates",
            abs(left_marker_turned_center[0] - left_marker_rest_center[0]) > 0.045
            and abs(left_marker_turned_center[1] - left_marker_rest_center[1]) > 0.045,
            details=f"Rest={left_marker_rest_center}, turned={left_marker_turned_center}",
        )
        ctx.expect_contact(left_platter, housing)

    crossfader_rest = ctx.part_world_position(crossfader)
    assert crossfader_rest is not None
    with ctx.pose({crossfader_joint: 0.060}):
        crossfader_high = ctx.part_world_position(crossfader)
        assert crossfader_high is not None
        ctx.check(
            "crossfader_slides_right",
            crossfader_high[0] > crossfader_rest[0] + 0.05,
            details=f"Rest={crossfader_rest}, shifted={crossfader_high}",
        )
        ctx.expect_contact(crossfader, housing)

    stand_rest = ctx.part_element_world_aabb(laptop_stand, elem="tray_panel")
    assert stand_rest is not None
    stand_rest_center = _aabb_center(stand_rest)
    with ctx.pose({stand_joint: -0.75}):
        stand_lowered = ctx.part_element_world_aabb(laptop_stand, elem="tray_panel")
        assert stand_lowered is not None
        stand_lowered_center = _aabb_center(stand_lowered)
        ctx.check(
            "stand_folds_down_toward_closed_pose",
            stand_lowered_center[2] < stand_rest_center[2] - 0.09
            and stand_lowered_center[1] > stand_rest_center[1] + 0.08,
            details=f"Rest={stand_rest_center}, lowered={stand_lowered_center}",
        )
        ctx.expect_contact(laptop_stand, housing)

    with ctx.pose({stand_joint: 0.25}):
        stand_open = ctx.part_element_world_aabb(laptop_stand, elem="tray_panel")
        assert stand_open is not None
        stand_open_center = _aabb_center(stand_open)
        ctx.check(
            "stand_opens_higher_than_rest_pose",
            stand_open_center[2] > stand_rest_center[2] + 0.02,
            details=f"Rest={stand_rest_center}, open={stand_open_center}",
        )
        ctx.expect_contact(laptop_stand, housing)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
