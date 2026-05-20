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
    model = ArticulatedObject(name="pedalboard_case")

    shell_black = model.material("shell_black", rgba=(0.18, 0.18, 0.19, 1.0))
    liner_black = model.material("liner_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hardware = model.material("hardware", rgba=(0.73, 0.75, 0.78, 1.0))
    trim = model.material("trim", rgba=(0.58, 0.60, 0.64, 1.0))

    tray_width = 0.780
    tray_depth = 0.440
    tray_height = 0.090
    tray_wall = 0.012
    tray_bottom = 0.010

    lid_width = 0.800
    lid_depth = 0.460
    lid_height = 0.065
    lid_wall = 0.009
    lid_top = 0.008

    hinge_radius = 0.0055
    hinge_y = -(tray_depth * 0.5) - hinge_radius
    hinge_z = tray_height - hinge_radius

    lid_bottom_world = 0.062
    lid_local_bottom = lid_bottom_world - hinge_z
    lid_skirt_height = lid_height - lid_top
    lid_top_underside = lid_local_bottom + lid_skirt_height
    lid_top_center_z = lid_local_bottom + lid_height - lid_top * 0.5
    lid_wall_center_z = lid_local_bottom + lid_skirt_height * 0.5

    lid_rear_outer = -0.004
    lid_front_outer = lid_rear_outer + lid_depth
    lid_top_center_y = lid_rear_outer + lid_depth * 0.5
    lid_side_depth = lid_depth - 2.0 * lid_wall
    lid_side_center_y = lid_rear_outer + lid_wall + lid_side_depth * 0.5
    lid_front_center_y = lid_front_outer - lid_wall * 0.5
    lid_rear_center_y = lid_rear_outer + lid_wall * 0.5
    lid_inner_side_x = (lid_width * 0.5) - lid_wall
    lid_seat_center_z = (0.090 - hinge_z + lid_top_underside) * 0.5
    lid_seat_height = lid_top_underside - (0.090 - hinge_z)

    latch_x_positions = (-0.220, 0.220)

    tray = model.part("tray")
    tray.visual(
        Box((tray_width, tray_depth, tray_bottom)),
        origin=Origin(xyz=(0.0, 0.0, tray_bottom * 0.5)),
        material=shell_black,
        name="tray_bottom",
    )
    tray.visual(
        Box((tray_wall, tray_depth, tray_height - tray_bottom)),
        origin=Origin(
            xyz=((tray_width * 0.5) - (tray_wall * 0.5), 0.0, tray_bottom + (tray_height - tray_bottom) * 0.5)
        ),
        material=shell_black,
        name="tray_right_wall",
    )
    tray.visual(
        Box((tray_wall, tray_depth, tray_height - tray_bottom)),
        origin=Origin(
            xyz=(-(tray_width * 0.5) + (tray_wall * 0.5), 0.0, tray_bottom + (tray_height - tray_bottom) * 0.5)
        ),
        material=shell_black,
        name="tray_left_wall",
    )
    tray.visual(
        Box((tray_width - 2.0 * tray_wall, tray_wall, tray_height - tray_bottom)),
        origin=Origin(
            xyz=(0.0, (tray_depth * 0.5) - (tray_wall * 0.5), tray_bottom + (tray_height - tray_bottom) * 0.5)
        ),
        material=shell_black,
        name="tray_front_wall",
    )
    tray.visual(
        Box((tray_width - 2.0 * tray_wall, tray_wall, tray_height - tray_bottom)),
        origin=Origin(
            xyz=(0.0, -(tray_depth * 0.5) + (tray_wall * 0.5), tray_bottom + (tray_height - tray_bottom) * 0.5)
        ),
        material=shell_black,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((tray_width - 0.036, tray_depth - 0.036, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, tray_bottom + 0.002)),
        material=liner_black,
        name="equipment_deck",
    )
    tray.visual(
        Box((tray_width - 2.0 * tray_wall, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, (tray_depth * 0.5) - 0.005, tray_height - 0.005)),
        material=trim,
        name="front_trim_bar",
    )

    tray_knuckles = [
        ("tray_hinge_left", -0.265, 0.150),
        ("tray_hinge_center", 0.0, 0.120),
        ("tray_hinge_right", 0.265, 0.150),
    ]
    for name, x_center, length in tray_knuckles:
        tray.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(x_center, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=name,
        )

    for side_name, x_center in (("left", latch_x_positions[0]), ("right", latch_x_positions[1])):
        tray.visual(
            Box((0.066, 0.008, 0.034)),
            origin=Origin(xyz=(x_center, (tray_depth * 0.5) + 0.004, 0.045)),
            material=trim,
            name=f"{side_name}_latch_plate",
        )
        tray.visual(
            Box((0.052, 0.006, 0.014)),
            origin=Origin(xyz=(x_center, (tray_depth * 0.5) + 0.003, 0.064)),
            material=hardware,
            name=f"{side_name}_latch_strike",
        )

    tray.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth, tray_height)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, tray_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_top)),
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_top_center_z)),
        material=shell_black,
        name="top_panel",
    )
    lid.visual(
        Box((lid_wall, lid_side_depth, lid_skirt_height)),
        origin=Origin(xyz=((lid_width * 0.5) - (lid_wall * 0.5), lid_side_center_y, lid_wall_center_z)),
        material=shell_black,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_wall, lid_side_depth, lid_skirt_height)),
        origin=Origin(xyz=(-(lid_width * 0.5) + (lid_wall * 0.5), lid_side_center_y, lid_wall_center_z)),
        material=shell_black,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, lid_skirt_height)),
        origin=Origin(xyz=(0.0, lid_front_center_y, lid_wall_center_z)),
        material=shell_black,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, 0.022)),
        origin=Origin(xyz=(0.0, lid_rear_center_y, 0.0235)),
        material=shell_black,
        name="rear_fascia",
    )
    lid.visual(
        Box((0.720, 0.010, lid_seat_height)),
        origin=Origin(xyz=(0.0, 0.213, lid_seat_center_z)),
        material=trim,
        name="front_seat_rail",
    )
    lid.visual(
        Box((0.013, 0.390, lid_seat_height)),
        origin=Origin(xyz=(0.3815, 0.110, lid_seat_center_z)),
        material=trim,
        name="right_seat_rail",
    )
    lid.visual(
        Box((0.013, 0.390, lid_seat_height)),
        origin=Origin(xyz=(-0.3815, 0.110, lid_seat_center_z)),
        material=trim,
        name="left_seat_rail",
    )
    lid.visual(
        Box((0.018, 0.090, lid_seat_height)),
        origin=Origin(xyz=(0.391, -0.040, lid_seat_center_z)),
        material=trim,
        name="rear_right_seat_pad",
    )
    lid.visual(
        Box((0.018, 0.090, lid_seat_height)),
        origin=Origin(xyz=(-0.391, -0.040, lid_seat_center_z)),
        material=trim,
        name="rear_left_seat_pad",
    )

    lid_knuckles = [
        ("lid_hinge_left", -0.125, 0.110),
        ("lid_hinge_right", 0.125, 0.110),
    ]
    for name, x_center, length in lid_knuckles:
        lid.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=name,
        )
        lid.visual(
            Box((0.050, 0.009, 0.007)),
            origin=Origin(xyz=(x_center, lid_rear_center_y, 0.009)),
            material=hardware,
            name=f"{name}_leaf",
        )

    for side_name, x_center in (("left", latch_x_positions[0]), ("right", latch_x_positions[1])):
        lid.visual(
            Box((0.050, 0.004, 0.016)),
            origin=Origin(xyz=(x_center, lid_front_outer + 0.002, -0.006)),
            material=hardware,
            name=f"{side_name}_keeper",
        )

    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_height)),
        mass=3.6,
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_local_bottom + lid_height * 0.5)),
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        Cylinder(radius=0.003, length=0.034),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    left_latch.visual(
        Box((0.020, 0.004, 0.028)),
        origin=Origin(xyz=(-0.012, 0.002, -0.014), rpy=(0.0, 0.0, 0.35)),
        material=hardware,
        name="latch_leaf_left",
    )
    left_latch.visual(
        Box((0.020, 0.004, 0.028)),
        origin=Origin(xyz=(0.012, 0.002, -0.014), rpy=(0.0, 0.0, -0.35)),
        material=hardware,
        name="latch_leaf_right",
    )
    left_latch.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, -0.028)),
        material=hardware,
        name="latch_leaf",
    )
    left_latch.inertial = Inertial.from_geometry(
        Box((0.050, 0.018, 0.040)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.005, -0.018)),
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Cylinder(radius=0.003, length=0.034),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    right_latch.visual(
        Box((0.020, 0.004, 0.028)),
        origin=Origin(xyz=(-0.012, 0.002, -0.014), rpy=(0.0, 0.0, 0.35)),
        material=hardware,
        name="latch_leaf_left",
    )
    right_latch.visual(
        Box((0.020, 0.004, 0.028)),
        origin=Origin(xyz=(0.012, 0.002, -0.014), rpy=(0.0, 0.0, -0.35)),
        material=hardware,
        name="latch_leaf_right",
    )
    right_latch.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, -0.028)),
        material=hardware,
        name="latch_leaf",
    )
    right_latch.inertial = Inertial.from_geometry(
        Box((0.050, 0.018, 0.040)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.005, -0.018)),
    )

    model.articulation(
        "tray_to_lid",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "tray_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=left_latch,
        origin=Origin(xyz=(latch_x_positions[0], (tray_depth * 0.5) + 0.008, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "tray_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=right_latch,
        origin=Origin(xyz=(latch_x_positions[1], (tray_depth * 0.5) + 0.008, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    lid_hinge = object_model.get_articulation("tray_to_lid")
    left_latch_hinge = object_model.get_articulation("tray_to_left_latch")
    right_latch_hinge = object_model.get_articulation("tray_to_right_latch")

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
        "lid hinge axis is horizontal",
        lid_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected rear hinge axis (1, 0, 0), got {lid_hinge.axis!r}.",
    )
    ctx.check(
        "left latch axis is horizontal",
        left_latch_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected left latch axis (1, 0, 0), got {left_latch_hinge.axis!r}.",
    )
    ctx.check(
        "right latch axis is horizontal",
        right_latch_hinge.axis == (1.0, 0.0, 0.0),
        f"Expected right latch axis (1, 0, 0), got {right_latch_hinge.axis!r}.",
    )

    with ctx.pose({lid_hinge: 0.0, left_latch_hinge: 0.0, right_latch_hinge: 0.0}):
        ctx.expect_contact(lid, tray, name="lid_seats_on_tray")
        ctx.expect_overlap(lid, tray, axes="xy", min_overlap=0.35, name="lid_covers_tray_opening")
        ctx.expect_contact(left_latch, tray, name="left_latch_is_mounted")
        ctx.expect_contact(right_latch, tray, name="right_latch_is_mounted")

    closed_top = ctx.part_element_world_aabb(lid, elem="top_panel")
    assert closed_top is not None
    with ctx.pose({lid_hinge: math.radians(62.0)}):
        open_top = ctx.part_element_world_aabb(lid, elem="top_panel")
        assert open_top is not None
        ctx.check(
            "lid opens upward from rear hinge",
            open_top[1][2] > closed_top[1][2] + 0.18,
            f"Expected lid top to rise by at least 0.18 m, got closed={closed_top} open={open_top}.",
        )
        ctx.expect_overlap(lid, tray, axes="x", min_overlap=0.70, name="lid_stays_registered_to_hinge_line")

    with ctx.pose({left_latch_hinge: 1.0}):
        ctx.expect_gap(
            left_latch,
            tray,
            axis="y",
            positive_elem="latch_leaf",
            negative_elem="left_latch_plate",
            min_gap=0.010,
            name="left_latch_swings_clear_of_front_plate",
        )

    with ctx.pose({right_latch_hinge: 1.0}):
        ctx.expect_gap(
            right_latch,
            tray,
            axis="y",
            positive_elem="latch_leaf",
            negative_elem="right_latch_plate",
            min_gap=0.010,
            name="right_latch_swings_clear_of_front_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
