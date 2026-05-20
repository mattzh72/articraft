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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_watch_winder_box")

    shell_black = model.material("shell_black", rgba=(0.13, 0.13, 0.14, 1.0))
    liner_black = model.material("liner_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    box_w = 0.270
    box_d = 0.190
    box_h = 0.128
    floor_t = 0.012
    wall_t = 0.012
    front_t = 0.016
    rear_t = 0.014

    cradle_axis_y = -0.008
    cradle_axis_z = 0.076
    hinge_y = 0.107
    hinge_z = 0.138

    base = model.part("base")
    base.visual(
        Box((box_w, box_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=shell_black,
        name="floor_plate",
    )
    base.visual(
        Box((wall_t, box_d, box_h)),
        origin=Origin(xyz=(-(box_w * 0.5) + (wall_t * 0.5), 0.0, box_h * 0.5)),
        material=shell_black,
        name="left_wall",
    )
    base.visual(
        Box((wall_t, box_d, box_h)),
        origin=Origin(xyz=((box_w * 0.5) - (wall_t * 0.5), 0.0, box_h * 0.5)),
        material=shell_black,
        name="right_wall",
    )
    base.visual(
        Box((box_w - (2.0 * wall_t), front_t, box_h)),
        origin=Origin(xyz=(0.0, -(box_d * 0.5) + (front_t * 0.5), box_h * 0.5)),
        material=shell_black,
        name="front_wall",
    )
    base.visual(
        Box((box_w - (2.0 * wall_t), rear_t, box_h)),
        origin=Origin(xyz=(0.0, (box_d * 0.5) - (rear_t * 0.5), box_h * 0.5)),
        material=shell_black,
        name="rear_wall",
    )
    base.visual(
        Box((0.190, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.052, 0.021)),
        material=dark_steel,
        name="front_service_rail",
    )
    base.visual(
        Box((0.190, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.032, 0.021)),
        material=dark_steel,
        name="rear_service_rail",
    )
    base.visual(
        Box((0.108, 0.046, 0.040)),
        origin=Origin(xyz=(0.0, 0.058, 0.032)),
        material=dark_steel,
        name="drive_module",
    )
    base.visual(
        Box((0.140, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, 0.098, 0.058)),
        material=steel,
        name="rear_service_cover",
    )
    for screw_index, screw_x in enumerate((-0.050, 0.050)):
        for screw_row, screw_z in enumerate((0.036, 0.080)):
            base.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(
                    xyz=(screw_x, 0.103, screw_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark_steel,
                name=f"service_screw_{screw_index}_{screw_row}",
            )

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        base.visual(
            Box((0.030, 0.058, 0.056)),
            origin=Origin(xyz=(sign * 0.110, cradle_axis_y, 0.028)),
            material=dark_steel,
            name=f"{side_name}_support_block",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(
                xyz=(sign * 0.094, cradle_axis_y, cradle_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{side_name}_bearing_face",
        )
        base.visual(
            Box((0.060, 0.018, 0.006)),
            origin=Origin(xyz=(sign * 0.086, 0.101, 0.129)),
            material=steel,
            name=f"base_hinge_leaf_{side_name}",
        )

    base.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(-0.106, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="base_hinge_outer_left",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(-0.066, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="base_hinge_inner_left",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.066, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="base_hinge_inner_right",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.106, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="base_hinge_outer_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((box_w, box_d, box_h)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, box_h * 0.5)),
    )

    lid = model.part("lid")
    lid_outer_w = 0.294
    lid_outer_d = 0.214
    lid_wall_t = 0.010
    lid_drop = 0.034
    lid_top_d = 0.208
    lid.visual(
        Box((lid_outer_w, lid_top_d, 0.010)),
        origin=Origin(xyz=(0.0, -0.110, -0.004)),
        material=shell_black,
        name="lid_top_panel",
    )
    lid.visual(
        Box((lid_wall_t, lid_outer_d, lid_drop)),
        origin=Origin(
            xyz=((lid_outer_w * 0.5) - (lid_wall_t * 0.5), -(lid_outer_d * 0.5), -0.026)
        ),
        material=shell_black,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_wall_t, lid_outer_d, lid_drop)),
        origin=Origin(
            xyz=(-(lid_outer_w * 0.5) + (lid_wall_t * 0.5), -(lid_outer_d * 0.5), -0.026)
        ),
        material=shell_black,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((lid_outer_w - (2.0 * lid_wall_t), lid_wall_t, lid_drop)),
        origin=Origin(xyz=(0.0, -lid_outer_d + (lid_wall_t * 0.5), -0.026)),
        material=shell_black,
        name="lid_front_skirt",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(-0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lid_hinge_left",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lid_hinge_right",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(0.0, -0.233, -0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="grab_bar",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(
            xyz=(-0.028, -0.220, -0.019),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="grab_bar_post_left",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(
            xyz=(0.028, -0.220, -0.019),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="grab_bar_post_right",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_outer_w, lid_outer_d, 0.042)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -(lid_outer_d * 0.5), -0.021)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0055, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    cradle.visual(
        Box((0.084, 0.056, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=liner_black,
        name="pillow_block",
    )
    cradle.visual(
        Cylinder(radius=0.024, length=0.084),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="front_cushion_roll",
    )
    cradle.visual(
        Box((0.070, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.020, 0.008)),
        material=rubber,
        name="rear_keeper_pad",
    )
    cradle.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_side_flange",
    )
    cradle.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_side_flange",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(-0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_hub",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.059, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_hub",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.023),
        origin=Origin(xyz=(-0.0785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_wear_collar",
    )
    cradle.visual(
        Cylinder(radius=0.013, length=0.023),
        origin=Origin(xyz=(0.0785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_wear_collar",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.180, 0.060, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.3,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "base_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=cradle,
        origin=Origin(xyz=(0.0, cradle_axis_y, cradle_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("base_to_lid")
    cradle_spin = object_model.get_articulation("base_to_cradle")

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
        "expected_primary_parts",
        len(object_model.parts) == 3,
        details=f"expected 3 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "expected_primary_articulations",
        len(object_model.articulations) == 2,
        details=f"expected 2 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "articulation_types_match_prompt",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and cradle_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="lid should be revolute and cradle should be continuous",
    )

    ctx.expect_contact(
        base,
        lid,
        elem_a="base_hinge_inner_left",
        elem_b="lid_hinge_left",
        name="left_hinge_support_contact",
    )
    ctx.expect_contact(
        base,
        lid,
        elem_a="base_hinge_inner_right",
        elem_b="lid_hinge_right",
        name="right_hinge_support_contact",
    )

    with ctx.pose({cradle_spin: math.pi / 3.0}):
        ctx.expect_contact(
            base,
            cradle,
            elem_a="left_bearing_face",
            elem_b="left_wear_collar",
            name="left_cradle_support_contact",
        )
        ctx.expect_contact(
            base,
            cradle,
            elem_a="right_bearing_face",
            elem_b="right_wear_collar",
            name="right_cradle_support_contact",
        )
        ctx.expect_within(
            cradle,
            base,
            axes="xyz",
            margin=0.0,
            name="cradle_stays_inside_box_envelope",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_top_panel",
            negative_elem="rear_wall",
            min_gap=0.0005,
            max_gap=0.0040,
            name="closed_lid_sits_just_above_shell",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="lid_top_panel",
            elem_b="floor_plate",
            min_overlap=0.16,
            name="closed_lid_covers_presentation_box",
        )

    with ctx.pose({lid_hinge: 1.10}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_front_skirt",
            negative_elem="rear_wall",
            min_gap=0.10,
            name="opened_lid_front_edge_lifts_clear",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
