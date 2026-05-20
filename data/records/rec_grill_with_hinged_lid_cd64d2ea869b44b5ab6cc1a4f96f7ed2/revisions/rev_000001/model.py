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
    ExtrudeGeometry,
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


def _lid_profile() -> list[tuple[float, float]]:
    return [
        (0.000, 0.000),
        (0.018, 0.070),
        (0.085, 0.145),
        (0.180, 0.170),
        (0.275, 0.152),
        (0.340, 0.105),
        (0.360, 0.020),
        (0.340, 0.032),
        (0.322, 0.092),
        (0.265, 0.132),
        (0.180, 0.146),
        (0.092, 0.128),
        (0.032, 0.070),
        (0.000, 0.012),
    ]


def _lid_shell_mesh(width: float):
    return _mesh(
        "grill_lid_shell",
        ExtrudeGeometry(_lid_profile(), width, center=True).rotate_x(pi / 2.0).rotate_z(pi / 2.0),
    )


def _leg_frame_mesh(name: str, depth: float, drop: float, radius: float):
    half_depth = depth * 0.5
    loop = wire_from_points(
        [
            (0.0, -half_depth, -0.018),
            (0.0, -half_depth, -drop),
            (0.0, half_depth, -drop),
            (0.0, half_depth, -0.018),
        ],
        radius=radius,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=8,
    )
    return _mesh(name, loop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_suitcase_grill")

    enamel_black = model.material("enamel_black", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.74, 1.0))
    grate_steel = model.material("grate_steel", rgba=(0.63, 0.65, 0.67, 1.0))
    handle_black = model.material("handle_black", rgba=(0.18, 0.18, 0.17, 1.0))

    body_w = 0.56
    body_d = 0.34
    body_h = 0.10
    wall_t = 0.012
    bottom_t = 0.006
    hinge_block_y = 0.090
    leg_joint_z = -0.0245
    leg_joint_x = body_w * 0.5 + 0.010
    leg_depth = 0.22
    leg_drop = 0.23
    leg_radius = 0.0085

    firebox = model.part("firebox")
    firebox.inertial = Inertial.from_geometry(
        Box((body_w, body_d, 0.20)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    firebox.visual(
        Box((body_w - (2.0 * wall_t), body_d - (2.0 * wall_t), bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=enamel_black,
        name="bottom_plate",
    )
    firebox.visual(
        Box((body_w, wall_t, body_h - bottom_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall_t * 0.5, bottom_t + (body_h - bottom_t) * 0.5)),
        material=enamel_black,
        name="front_wall",
    )
    firebox.visual(
        Box((body_w, wall_t, body_h - bottom_t)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall_t * 0.5, bottom_t + (body_h - bottom_t) * 0.5)),
        material=enamel_black,
        name="rear_wall",
    )
    firebox.visual(
        Box((wall_t, body_d - (2.0 * wall_t), body_h - bottom_t)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, bottom_t + (body_h - bottom_t) * 0.5)),
        material=enamel_black,
        name="left_wall",
    )
    firebox.visual(
        Box((wall_t, body_d - (2.0 * wall_t), body_h - bottom_t)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, bottom_t + (body_h - bottom_t) * 0.5)),
        material=enamel_black,
        name="right_wall",
    )

    firebox.visual(
        Box((body_w + 0.008, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - 0.007, body_h - 0.004)),
        material=dark_steel,
        name="front_rim",
    )
    firebox.visual(
        Box((body_w + 0.008, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + 0.007, body_h - 0.004)),
        material=dark_steel,
        name="rear_rim",
    )
    firebox.visual(
        Box((0.014, body_d - 0.020, 0.008)),
        origin=Origin(xyz=(-body_w * 0.5 + 0.007, 0.0, body_h - 0.004)),
        material=dark_steel,
        name="left_rim",
    )
    firebox.visual(
        Box((0.014, body_d - 0.020, 0.008)),
        origin=Origin(xyz=(body_w * 0.5 - 0.007, 0.0, body_h - 0.004)),
        material=dark_steel,
        name="right_rim",
    )

    grate_w = body_w - 0.100
    grate_d = body_d - 0.090
    grate_z = 0.074
    firebox.visual(
        Box((0.008, grate_d, 0.004)),
        origin=Origin(xyz=(-grate_w * 0.5, 0.0, grate_z)),
        material=grate_steel,
        name="grate_left_rail",
    )
    firebox.visual(
        Box((0.008, grate_d, 0.004)),
        origin=Origin(xyz=(grate_w * 0.5, 0.0, grate_z)),
        material=grate_steel,
        name="grate_right_rail",
    )
    firebox.visual(
        Box((grate_w + 0.008, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -grate_d * 0.5, grate_z)),
        material=grate_steel,
        name="grate_rear_bar",
    )
    firebox.visual(
        Box((grate_w + 0.008, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, grate_d * 0.5, grate_z)),
        material=grate_steel,
        name="grate_front_bar",
    )
    for idx, y_pos in enumerate((-0.080, -0.048, -0.016, 0.016, 0.048, 0.080), start=1):
        firebox.visual(
            Box((grate_w + 0.008, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, y_pos, grate_z)),
            material=grate_steel,
            name=f"grate_slat_{idx}",
        )
    firebox.visual(
        Box((0.042, grate_d + 0.016, 0.008)),
        origin=Origin(xyz=(-(body_w * 0.5 - 0.033), 0.0, grate_z - 0.004)),
        material=dark_steel,
        name="grate_left_support",
    )
    firebox.visual(
        Box((0.042, grate_d + 0.016, 0.008)),
        origin=Origin(xyz=((body_w * 0.5 - 0.033), 0.0, grate_z - 0.004)),
        material=dark_steel,
        name="grate_right_support",
    )

    firebox.visual(
        Box((0.32, 0.055, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="burner_cover",
    )
    firebox.visual(
        Box((0.14, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.010, 0.055)),
        material=dark_steel,
        name="control_panel",
    )
    firebox.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, body_d * 0.5 + 0.034, 0.055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="control_knob",
    )

    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        for idx, y_pos in enumerate((-0.110, 0.110), start=1):
            firebox.visual(
                Box((0.013, 0.040, 0.025)),
                origin=Origin(xyz=(sign * (body_w * 0.5 - 0.005), y_pos, -0.030)),
                material=dark_steel,
                name=f"{side_name}_leg_clip_{idx}",
            )
    firebox.visual(
        Box((0.024, 0.260, 0.040)),
        origin=Origin(xyz=(-(body_w * 0.5 - 0.008), 0.0, -0.014)),
        material=dark_steel,
        name="left_leg_bracket",
    )
    firebox.visual(
        Box((0.024, 0.260, 0.040)),
        origin=Origin(xyz=((body_w * 0.5 - 0.008), 0.0, -0.014)),
        material=dark_steel,
        name="right_leg_bracket",
    )
    firebox.visual(
        Cylinder(radius=0.010, length=body_w - 0.090),
        origin=Origin(xyz=(0.0, -body_d * 0.5 - 0.004, body_h + 0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="lid_hinge_axle",
    )
    firebox.visual(
        Cylinder(radius=0.0095, length=0.180),
        origin=Origin(xyz=(-leg_joint_x, 0.0, leg_joint_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="left_leg_axle",
    )
    firebox.visual(
        Cylinder(radius=0.0095, length=0.180),
        origin=Origin(xyz=(leg_joint_x, 0.0, leg_joint_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="right_leg_axle",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((body_w + 0.030, body_d + 0.040, 0.18)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.18, 0.08)),
    )
    lid.visual(
        _lid_shell_mesh(body_w + 0.030),
        origin=Origin(xyz=(0.0, 0.004, 0.010)),
        material=enamel_black,
        name="lid_shell",
    )
    lid.visual(
        Box((0.420, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.337, 0.011)),
        material=enamel_black,
        name="lid_front_flange",
    )
    lid.visual(
        Box((0.460, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.353, 0.023)),
        material=enamel_black,
        name="lid_flange_bridge",
    )
    lid.visual(
        Box((0.014, 0.026, 0.030)),
        origin=Origin(xyz=(-0.080, 0.348, 0.074)),
        material=dark_steel,
        name="handle_post_left",
    )
    lid.visual(
        Box((0.014, 0.026, 0.030)),
        origin=Origin(xyz=(0.080, 0.348, 0.074)),
        material=dark_steel,
        name="handle_post_right",
    )
    lid.visual(
        Box((0.190, 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.366, 0.090)),
        material=handle_black,
        name="front_handle",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.199, 0.170)),
        material=brushed_steel,
        name="thermometer_bezel",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.0, 0.199, 0.180)),
        material=dark_steel,
        name="thermometer_cap",
    )

    left_leg = model.part("left_leg")
    left_leg.inertial = Inertial.from_geometry(
        Box((0.03, leg_depth, leg_drop)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -leg_drop * 0.5)),
    )
    left_leg.visual(
        _leg_frame_mesh("left_leg_frame", leg_depth, leg_drop, leg_radius),
        material=brushed_steel,
        name="left_leg_frame",
    )

    right_leg = model.part("right_leg")
    right_leg.inertial = Inertial.from_geometry(
        Box((0.03, leg_depth, leg_drop)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -leg_drop * 0.5)),
    )
    right_leg.visual(
        _leg_frame_mesh("right_leg_frame", leg_depth, leg_drop, leg_radius),
        material=brushed_steel,
        name="right_leg_frame",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=lid,
        origin=Origin(xyz=(0.0, -body_d * 0.5 - 0.004, body_h + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=2.10,
        ),
    )
    model.articulation(
        "left_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=left_leg,
        origin=Origin(xyz=(-leg_joint_x, 0.0, leg_joint_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.38,
        ),
    )
    model.articulation(
        "right_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=right_leg,
        origin=Origin(xyz=(leg_joint_x, 0.0, leg_joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    firebox = object_model.get_part("firebox")
    lid = object_model.get_part("lid")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_leg_hinge = object_model.get_articulation("left_leg_hinge")
    right_leg_hinge = object_model.get_articulation("right_leg_hinge")

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

    ctx.check("grill_has_four_parts", len(object_model.parts) == 4, f"Expected 4 parts, found {len(object_model.parts)}")
    ctx.check("lid_axis_is_rear_horizontal", lid_hinge.axis == (1.0, 0.0, 0.0), f"Unexpected lid axis: {lid_hinge.axis}")
    ctx.check(
        "left_leg_axis_is_side_pivot",
        left_leg_hinge.axis == (0.0, -1.0, 0.0),
        f"Unexpected left leg axis: {left_leg_hinge.axis}",
    )
    ctx.check(
        "right_leg_axis_is_side_pivot",
        right_leg_hinge.axis == (0.0, 1.0, 0.0),
        f"Unexpected right leg axis: {right_leg_hinge.axis}",
    )

    ctx.expect_contact(left_leg, firebox, name="left_leg_remains_clipped_to_body")
    ctx.expect_contact(right_leg, firebox, name="right_leg_remains_clipped_to_body")
    ctx.expect_contact(
        lid,
        firebox,
        elem_a="lid_front_flange",
        elem_b="front_rim",
        name="lid_remains_seated_on_firebox",
    )
    ctx.expect_gap(
        lid,
        firebox,
        axis="z",
        min_gap=0.0,
        max_gap=0.035,
        positive_elem="lid_shell",
        negative_elem="front_rim",
        name="closed_lid_sits_just_above_firebox",
    )
    ctx.expect_overlap(lid, firebox, axes="xy", min_overlap=0.28, name="lid_covers_cooking_area")

    firebox_aabb = ctx.part_world_aabb(firebox)
    left_leg_aabb = ctx.part_world_aabb(left_leg)
    right_leg_aabb = ctx.part_world_aabb(right_leg)
    lid_closed_aabb = ctx.part_world_aabb(lid)

    assert firebox_aabb is not None
    assert left_leg_aabb is not None
    assert right_leg_aabb is not None
    assert lid_closed_aabb is not None

    ctx.check(
        "left_leg_supports_below_body",
        left_leg_aabb[0][2] < firebox_aabb[0][2] - 0.18,
        f"Left leg does not extend far enough below the body: firebox min z={firebox_aabb[0][2]:.4f}, leg min z={left_leg_aabb[0][2]:.4f}",
    )
    ctx.check(
        "right_leg_supports_below_body",
        right_leg_aabb[0][2] < firebox_aabb[0][2] - 0.18,
        f"Right leg does not extend far enough below the body: firebox min z={firebox_aabb[0][2]:.4f}, leg min z={right_leg_aabb[0][2]:.4f}",
    )

    with ctx.pose({lid_hinge: 1.85}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        assert lid_open_aabb is not None
        ctx.check(
            "lid_opens_upward_and_rearward",
            lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.12 and lid_open_aabb[0][1] < lid_closed_aabb[0][1] - 0.08,
            (
                "Lid did not move like a rear clamshell: "
                f"closed max z={lid_closed_aabb[1][2]:.4f}, open max z={lid_open_aabb[1][2]:.4f}, "
                f"closed min y={lid_closed_aabb[0][1]:.4f}, open min y={lid_open_aabb[0][1]:.4f}"
            ),
        )

    with ctx.pose({left_leg_hinge: 1.35, right_leg_hinge: 1.35}):
        left_folded_aabb = ctx.part_world_aabb(left_leg)
        right_folded_aabb = ctx.part_world_aabb(right_leg)
        assert left_folded_aabb is not None
        assert right_folded_aabb is not None

        ctx.expect_within(left_leg, firebox, axes="y", margin=0.03, name="left_leg_folds_within_body_depth")
        ctx.expect_within(right_leg, firebox, axes="y", margin=0.03, name="right_leg_folds_within_body_depth")
        ctx.expect_gap(
            firebox,
            left_leg,
            axis="z",
            min_gap=0.004,
            positive_elem="bottom_plate",
            name="left_leg_stays_below_firebox_floor_when_folded",
        )
        ctx.expect_gap(
            firebox,
            right_leg,
            axis="z",
            min_gap=0.004,
            positive_elem="bottom_plate",
            name="right_leg_stays_below_firebox_floor_when_folded",
        )
        ctx.check(
            "left_leg_folds_inward_under_body",
            left_folded_aabb[1][0] > left_leg_aabb[1][0] + 0.12,
            (
                "Left leg did not swing inward under the body: "
                f"deployed max x={left_leg_aabb[1][0]:.4f}, folded max x={left_folded_aabb[1][0]:.4f}, "
                f"folded max z={left_folded_aabb[1][2]:.4f}"
            ),
        )
        ctx.check(
            "right_leg_folds_inward_under_body",
            right_folded_aabb[0][0] < right_leg_aabb[0][0] - 0.12,
            (
                "Right leg did not swing inward under the body: "
                f"deployed min x={right_leg_aabb[0][0]:.4f}, folded min x={right_folded_aabb[0][0]:.4f}, "
                f"folded max z={right_folded_aabb[1][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
