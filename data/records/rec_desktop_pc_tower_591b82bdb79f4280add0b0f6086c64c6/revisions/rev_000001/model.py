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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rackmount_4u_desktop_chassis")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    bezel_finish = model.material("bezel_finish", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.05, 0.05, 0.05, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.55, 0.57, 0.60, 1.0))

    rack_width = 0.4826
    chassis_width = 0.4320
    chassis_depth = 0.5300
    chassis_height = 0.1778

    sheet = 0.0022
    frame_thickness = 0.0025
    bezel_thickness = 0.0060
    bezel_width = chassis_width - 0.0080
    bezel_height = chassis_height - 0.0160
    bezel_hinge_radius = 0.0045
    bezel_hinge_clearance = 0.0015
    bezel_body_knuckle_len = 0.0320
    bezel_center_knuckle_len = bezel_height - 2.0 * bezel_body_knuckle_len

    lid_width = chassis_width + 0.0060
    lid_depth = 0.5200
    lid_skin_thickness = 0.0022
    lid_skirt_thickness = 0.0020
    lid_front_lip_height = 0.0150
    lid_rear_flange_height = 0.0160
    lid_side_skirt_height = 0.0200
    lid_hinge_radius = 0.0040
    lid_clearance = 0.0008
    lid_axis_z = chassis_height - lid_hinge_radius + lid_clearance
    lid_axis_y = chassis_depth * 0.5 + 0.0015
    lid_body_knuckle_len = 0.0180
    lid_panel_knuckle_len = 0.0260

    body = model.part("body")

    body.visual(
        Box((chassis_width, chassis_depth, sheet)),
        origin=Origin(xyz=(0.0, 0.0, sheet * 0.5)),
        material=body_finish,
        name="base_tray",
    )
    body.visual(
        Box((sheet, chassis_depth, chassis_height - sheet)),
        origin=Origin(
            xyz=(-chassis_width * 0.5 + sheet * 0.5, 0.0, sheet + (chassis_height - sheet) * 0.5)
        ),
        material=body_finish,
        name="left_wall",
    )
    body.visual(
        Box((sheet, chassis_depth, chassis_height - sheet)),
        origin=Origin(
            xyz=(chassis_width * 0.5 - sheet * 0.5, 0.0, sheet + (chassis_height - sheet) * 0.5)
        ),
        material=body_finish,
        name="right_wall",
    )
    body.visual(
        Box((chassis_width - 2.0 * sheet, sheet, chassis_height - sheet)),
        origin=Origin(
            xyz=(0.0, chassis_depth * 0.5 - sheet * 0.5, sheet + (chassis_height - sheet) * 0.5)
        ),
        material=body_finish,
        name="rear_wall",
    )
    body.visual(
        Box((sheet, frame_thickness, chassis_height - sheet)),
        origin=Origin(
            xyz=(-chassis_width * 0.5 + sheet * 0.5, -chassis_depth * 0.5 + frame_thickness * 0.5, sheet + (chassis_height - sheet) * 0.5)
        ),
        material=body_finish,
        name="front_left_post",
    )
    body.visual(
        Box((sheet, frame_thickness, chassis_height - sheet)),
        origin=Origin(
            xyz=(chassis_width * 0.5 - sheet * 0.5, -chassis_depth * 0.5 + frame_thickness * 0.5, sheet + (chassis_height - sheet) * 0.5)
        ),
        material=body_finish,
        name="front_right_post",
    )
    body.visual(
        Box((chassis_width - 2.0 * sheet, frame_thickness, 0.0220)),
        origin=Origin(
            xyz=(0.0, -chassis_depth * 0.5 + frame_thickness * 0.5, sheet + 0.0110)
        ),
        material=body_finish,
        name="front_bottom_rail",
    )
    body.visual(
        Box((chassis_width - 2.0 * sheet, frame_thickness, 0.0180)),
        origin=Origin(
            xyz=(0.0, -chassis_depth * 0.5 + frame_thickness * 0.5, chassis_height - 0.0090)
        ),
        material=body_finish,
        name="front_top_rail",
    )

    rack_ear_width = rack_width * 0.5 - chassis_width * 0.5
    body.visual(
        Box((rack_ear_width, 0.0050, chassis_height - 0.0240)),
        origin=Origin(
            xyz=(-(rack_width * 0.5) + rack_ear_width * 0.5, -chassis_depth * 0.5 + 0.0025, chassis_height * 0.5)
        ),
        material=steel_finish,
        name="left_rack_ear",
    )
    body.visual(
        Box((rack_ear_width, 0.0050, chassis_height - 0.0240)),
        origin=Origin(
            xyz=((rack_width * 0.5) - rack_ear_width * 0.5, -chassis_depth * 0.5 + 0.0025, chassis_height * 0.5)
        ),
        material=steel_finish,
        name="right_rack_ear",
    )

    foot_size = (0.0260, 0.0180, 0.0060)
    foot_z = -foot_size[2] * 0.5
    for name, x_pos, y_pos in (
        ("front_left_foot", -0.1500, -0.1800),
        ("front_right_foot", 0.1500, -0.1800),
        ("rear_left_foot", -0.1500, 0.1800),
        ("rear_right_foot", 0.1500, 0.1800),
    ):
        body.visual(
            Box(foot_size),
            origin=Origin(xyz=(x_pos, y_pos, foot_z)),
            material=rubber_finish,
            name=name,
        )

    bezel_axis_x = -bezel_width * 0.5
    bezel_axis_y = -chassis_depth * 0.5 - bezel_hinge_clearance
    lower_bezel_knuckle_z = chassis_height * 0.5 - bezel_height * 0.5 + bezel_body_knuckle_len * 0.5
    upper_bezel_knuckle_z = chassis_height * 0.5 + bezel_height * 0.5 - bezel_body_knuckle_len * 0.5

    body.visual(
        Cylinder(radius=bezel_hinge_radius, length=bezel_body_knuckle_len),
        origin=Origin(xyz=(bezel_axis_x, bezel_axis_y, lower_bezel_knuckle_z)),
        material=body_finish,
        name="front_hinge_lower",
    )
    body.visual(
        Cylinder(radius=bezel_hinge_radius, length=bezel_body_knuckle_len),
        origin=Origin(xyz=(bezel_axis_x, bezel_axis_y, upper_bezel_knuckle_z)),
        material=body_finish,
        name="front_hinge_upper",
    )

    left_body_knuckle_x = -0.2050
    right_body_knuckle_x = 0.2050
    body.visual(
        Cylinder(radius=lid_hinge_radius, length=lid_body_knuckle_len),
        origin=Origin(xyz=(left_body_knuckle_x, lid_axis_y, lid_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_finish,
        name="rear_hinge_left_body",
    )
    body.visual(
        Cylinder(radius=lid_hinge_radius, length=lid_body_knuckle_len),
        origin=Origin(xyz=(right_body_knuckle_x, lid_axis_y, lid_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_finish,
        name="rear_hinge_right_body",
    )

    front_bezel = model.part("front_bezel")
    bezel_frame_width = 0.0260
    bezel_cross_rail = 0.0200

    front_bezel.visual(
        Box((bezel_frame_width, bezel_thickness, bezel_height)),
        origin=Origin(xyz=(bezel_frame_width * 0.5, -bezel_thickness * 0.5, 0.0)),
        material=bezel_finish,
        name="left_stile",
    )
    front_bezel.visual(
        Box((bezel_frame_width, bezel_thickness, bezel_height)),
        origin=Origin(xyz=(bezel_width - bezel_frame_width * 0.5, -bezel_thickness * 0.5, 0.0)),
        material=bezel_finish,
        name="right_stile",
    )
    front_bezel.visual(
        Box((bezel_width, bezel_thickness, bezel_cross_rail)),
        origin=Origin(xyz=(bezel_width * 0.5, -bezel_thickness * 0.5, bezel_height * 0.5 - bezel_cross_rail * 0.5)),
        material=bezel_finish,
        name="top_rail",
    )
    front_bezel.visual(
        Box((bezel_width, bezel_thickness, bezel_cross_rail)),
        origin=Origin(xyz=(bezel_width * 0.5, -bezel_thickness * 0.5, -bezel_height * 0.5 + bezel_cross_rail * 0.5)),
        material=bezel_finish,
        name="bottom_rail",
    )
    front_bezel.visual(
        Box((bezel_width - 0.0520, bezel_thickness * 0.55, bezel_height - 0.0520)),
        origin=Origin(xyz=(bezel_width * 0.5, -bezel_thickness * 0.275, 0.0)),
        material=bezel_finish,
        name="center_panel",
    )
    front_bezel.visual(
        Cylinder(radius=bezel_hinge_radius, length=bezel_center_knuckle_len),
        origin=Origin(),
        material=bezel_finish,
        name="bezel_hinge_barrel",
    )
    front_bezel.visual(
        Box((0.0180, 0.0080, 0.0500)),
        origin=Origin(xyz=(bezel_width - 0.0200, -bezel_thickness - 0.0040, 0.0)),
        material=steel_finish,
        name="latch_handle",
    )

    top_access_panel = model.part("top_access_panel")
    top_access_panel.visual(
        Box((lid_width, lid_depth, lid_skin_thickness)),
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, lid_hinge_radius + lid_skin_thickness * 0.5)),
        material=body_finish,
        name="top_skin",
    )
    top_access_panel.visual(
        Box((lid_width - 0.0060, 0.0100, lid_front_lip_height)),
        origin=Origin(
            xyz=(0.0, -lid_depth + 0.0050, lid_hinge_radius - lid_front_lip_height * 0.5)
        ),
        material=body_finish,
        name="front_lip",
    )
    top_access_panel.visual(
        Box((lid_width, 0.0080, lid_rear_flange_height)),
        origin=Origin(
            xyz=(0.0, -0.0040, lid_hinge_radius - lid_rear_flange_height * 0.5)
        ),
        material=body_finish,
        name="rear_flange",
    )
    top_access_panel.visual(
        Box((lid_skirt_thickness, lid_depth - 0.0180, lid_side_skirt_height)),
        origin=Origin(
            xyz=(-lid_width * 0.5 + lid_skirt_thickness * 0.5, -lid_depth * 0.5 + 0.0010, lid_hinge_radius - lid_side_skirt_height * 0.5)
        ),
        material=body_finish,
        name="left_skirt",
    )
    top_access_panel.visual(
        Box((lid_skirt_thickness, lid_depth - 0.0180, lid_side_skirt_height)),
        origin=Origin(
            xyz=(lid_width * 0.5 - lid_skirt_thickness * 0.5, -lid_depth * 0.5 + 0.0010, lid_hinge_radius - lid_side_skirt_height * 0.5)
        ),
        material=body_finish,
        name="right_skirt",
    )

    left_lid_knuckle_x = -0.1830
    right_lid_knuckle_x = 0.1830
    top_access_panel.visual(
        Cylinder(radius=lid_hinge_radius, length=lid_panel_knuckle_len),
        origin=Origin(xyz=(left_lid_knuckle_x, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_finish,
        name="rear_hinge_left_lid",
    )
    top_access_panel.visual(
        Cylinder(radius=lid_hinge_radius, length=lid_panel_knuckle_len),
        origin=Origin(xyz=(right_lid_knuckle_x, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_finish,
        name="rear_hinge_right_lid",
    )

    model.articulation(
        "body_to_front_bezel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_bezel,
        origin=Origin(xyz=(bezel_axis_x, bezel_axis_y, chassis_height * 0.5)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )
    model.articulation(
        "body_to_top_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_access_panel,
        origin=Origin(xyz=(0.0, lid_axis_y, lid_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_bezel = object_model.get_part("front_bezel")
    top_access_panel = object_model.get_part("top_access_panel")
    front_hinge = object_model.get_articulation("body_to_front_bezel")
    top_hinge = object_model.get_articulation("body_to_top_access_panel")

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
        "front bezel hinge is vertical on the left edge",
        front_hinge.axis == (0.0, 0.0, -1.0),
        details=f"axis={front_hinge.axis}",
    )
    ctx.check(
        "top access panel hinge is transverse at the rear edge",
        top_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={top_hinge.axis}",
    )

    ctx.expect_contact(
        body,
        front_bezel,
        elem_a="front_hinge_lower",
        elem_b="bezel_hinge_barrel",
        contact_tol=0.0005,
        name="front bezel is mounted on the body hinge knuckles",
    )
    ctx.expect_contact(
        body,
        top_access_panel,
        elem_a="rear_hinge_left_body",
        elem_b="rear_hinge_left_lid",
        contact_tol=0.0005,
        name="top access panel is mounted on the rear hinge pins",
    )
    ctx.expect_gap(
        body,
        front_bezel,
        axis="y",
        positive_elem="front_bottom_rail",
        negative_elem="center_panel",
        max_gap=0.0030,
        max_penetration=0.0,
        name="front bezel closes just ahead of the front frame",
    )
    ctx.expect_gap(
        top_access_panel,
        body,
        axis="z",
        positive_elem="top_skin",
        max_gap=0.0020,
        max_penetration=0.0,
        name="top access panel rests just above the chassis shell",
    )
    ctx.expect_overlap(
        front_bezel,
        body,
        axes="xz",
        min_overlap=0.1400,
        name="front bezel covers the front opening footprint",
    )
    ctx.expect_overlap(
        top_access_panel,
        body,
        axes="xy",
        elem_a="top_skin",
        min_overlap=0.2000,
        name="top access panel spans the top opening footprint",
    )

    closed_handle = _aabb_center(ctx.part_element_world_aabb(front_bezel, elem="latch_handle"))
    with ctx.pose({front_hinge: 1.20}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(front_bezel, elem="latch_handle"))
    ctx.check(
        "front bezel swings outward from the chassis face",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1] < closed_handle[1] - 0.0800,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    closed_front_lip = _aabb_center(ctx.part_element_world_aabb(top_access_panel, elem="front_lip"))
    with ctx.pose({top_hinge: 1.00}):
        open_front_lip = _aabb_center(ctx.part_element_world_aabb(top_access_panel, elem="front_lip"))
    ctx.check(
        "top access panel lifts upward for service access",
        closed_front_lip is not None
        and open_front_lip is not None
        and open_front_lip[2] > closed_front_lip[2] + 0.0800,
        details=f"closed_front_lip={closed_front_lip}, open_front_lip={open_front_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
