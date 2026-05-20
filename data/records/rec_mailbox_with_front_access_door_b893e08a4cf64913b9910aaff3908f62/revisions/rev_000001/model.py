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
    model = ArticulatedObject(name="european_slot_mailbox")

    body_color = model.material("graphite_body", rgba=(0.20, 0.22, 0.25, 1.0))
    metal_dark = model.material("dark_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    body_w = 0.36
    body_h = 0.48
    body_d = 0.14
    wall_t = 0.004
    front_frame_d = 0.012
    side_frame_w = 0.018
    bottom_frame_h = 0.018
    top_cap_h = 0.030
    slot_h = 0.026
    slot_sill_h = 0.016
    slot_w = 0.266
    half_w = body_w * 0.5
    half_d = body_d * 0.5
    front_y = half_d

    door_t = 0.010
    door_h = body_h - top_cap_h - slot_h - slot_sill_h - bottom_frame_h - 0.018
    door_w = body_w - 2.0 * side_frame_w - 0.004
    door_bottom_z = bottom_frame_h + 0.002
    door_mid_z = door_bottom_z + door_h * 0.5
    door_y = front_y - door_t * 0.5
    door_hinge_x = -door_w * 0.5
    door_hinge_leaf_w = 0.008

    slot_top_z = body_h - top_cap_h
    slot_axis_z = slot_top_z + 0.003
    slot_lid_w = slot_w + 0.020
    slot_lid_h = 0.055
    slot_lid_t = 0.008
    slot_axis_y = front_y + slot_lid_t * 0.5

    body = model.part("body")
    body.visual(
        Box((body_w, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -half_d + wall_t * 0.5, body_h * 0.5)),
        material=body_color,
        name="back_panel",
    )
    body.visual(
        Box((wall_t, body_d - wall_t, body_h)),
        origin=Origin(xyz=(-half_w + wall_t * 0.5, wall_t * 0.5, body_h * 0.5)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d - wall_t, body_h)),
        origin=Origin(xyz=(half_w - wall_t * 0.5, wall_t * 0.5, body_h * 0.5)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d - wall_t, wall_t)),
        origin=Origin(xyz=(0.0, wall_t * 0.5, wall_t * 0.5)),
        material=body_color,
        name="bottom_shell",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d - wall_t, wall_t)),
        origin=Origin(xyz=(0.0, wall_t * 0.5, body_h - wall_t * 0.5)),
        material=body_color,
        name="top_shell",
    )
    body.visual(
        Box((side_frame_w, front_frame_d, body_h)),
        origin=Origin(xyz=(-half_w + side_frame_w * 0.5, front_y - front_frame_d * 0.5, body_h * 0.5)),
        material=body_color,
        name="left_stile",
    )
    body.visual(
        Box((side_frame_w, front_frame_d, body_h)),
        origin=Origin(xyz=(half_w - side_frame_w * 0.5, front_y - front_frame_d * 0.5, body_h * 0.5)),
        material=body_color,
        name="right_stile",
    )
    inner_frame_w = body_w - 2.0 * side_frame_w
    body.visual(
        Box((inner_frame_w, front_frame_d, bottom_frame_h)),
        origin=Origin(xyz=(0.0, front_y - front_frame_d * 0.5, bottom_frame_h * 0.5)),
        material=body_color,
        name="bottom_frame",
    )
    body.visual(
        Box((inner_frame_w, front_frame_d, top_cap_h)),
        origin=Origin(xyz=(0.0, front_y - front_frame_d * 0.5, body_h - top_cap_h * 0.5)),
        material=body_color,
        name="top_cap",
    )
    body.visual(
        Box((inner_frame_w, front_frame_d, slot_sill_h)),
        origin=Origin(
            xyz=(
                0.0,
                front_y - front_frame_d * 0.5,
                body_h - top_cap_h - slot_h - slot_sill_h * 0.5,
            )
        ),
        material=body_color,
        name="slot_sill",
    )
    slot_cheek_w = (inner_frame_w - slot_w) * 0.5
    slot_cheek_x = slot_w * 0.5 + slot_cheek_w * 0.5
    slot_mid_z = body_h - top_cap_h - slot_h * 0.5
    body.visual(
        Box((slot_cheek_w, front_frame_d, slot_h)),
        origin=Origin(xyz=(-slot_cheek_x, front_y - front_frame_d * 0.5, slot_mid_z)),
        material=body_color,
        name="slot_cheek_left",
    )
    body.visual(
        Box((slot_cheek_w, front_frame_d, slot_h)),
        origin=Origin(xyz=(slot_cheek_x, front_y - front_frame_d * 0.5, slot_mid_z)),
        material=body_color,
        name="slot_cheek_right",
    )
    body.visual(
        Box((slot_lid_w + 0.006, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, front_y - 0.011, slot_top_z + 0.008)),
        material=body_color,
        name="slot_hood",
    )
    body.visual(
        Box((0.010, 0.006, 0.006)),
        origin=Origin(xyz=(-0.108, 0.059, body_h - top_cap_h - slot_h + 0.001)),
        material=metal_dark,
        name="slot_stop_left",
    )
    body.visual(
        Box((0.010, 0.006, 0.006)),
        origin=Origin(xyz=(0.108, 0.059, body_h - top_cap_h - slot_h + 0.001)),
        material=metal_dark,
        name="slot_stop_right",
    )
    body.visual(
        Box((0.006, 0.004, 0.040)),
        origin=Origin(xyz=(0.159, 0.058, door_mid_z)),
        material=metal_dark,
        name="door_strike",
    )
    for index, barrel_z in enumerate((0.075, door_mid_z, 0.351)):
        body.visual(
            Cylinder(radius=0.005, length=0.055),
            origin=Origin(xyz=(-0.165, 0.063, barrel_z)),
            material=steel,
            name=f"door_hinge_barrel_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    door = model.part("main_door")
    door.visual(
        Box((door_hinge_leaf_w, door_t, door_h)),
        origin=Origin(xyz=(door_hinge_leaf_w * 0.5, 0.0, 0.0)),
        material=body_color,
        name="hinge_leaf",
    )
    door.visual(
        Box((door_w - door_hinge_leaf_w, door_t, door_h)),
        origin=Origin(xyz=(door_hinge_leaf_w + (door_w - door_hinge_leaf_w) * 0.5, 0.0, 0.0)),
        material=body_color,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.030, 0.006, door_h - 0.026)),
        origin=Origin(xyz=(door_w * 0.5 + 0.001, -0.008, 0.0)),
        material=metal_dark,
        name="inner_return",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(door_w - 0.044, 0.011, -0.020), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="lock_cylinder",
    )
    door.visual(
        Box((0.020, 0.006, 0.010)),
        origin=Origin(xyz=(door_w - 0.044, 0.008, -0.028)),
        material=steel,
        name="lock_cover",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, 0.016, door_h)),
        mass=2.0,
        origin=Origin(xyz=(door_w * 0.5, -0.003, 0.0)),
    )

    slot_lid = model.part("slot_lid")
    slot_lid.visual(
        Box((slot_lid_w, slot_lid_t, slot_lid_h)),
        origin=Origin(xyz=(0.0, 0.0, -slot_lid_h * 0.5)),
        material=body_color,
        name="lid_panel",
    )
    slot_lid.visual(
        Box((slot_lid_w - 0.044, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.003, -0.004)),
        material=body_color,
        name="clip_return",
    )
    slot_lid.visual(
        Box((0.012, 0.006, 0.014)),
        origin=Origin(xyz=(-slot_w * 0.5 + 0.012, 0.003, -0.008)),
        material=body_color,
        name="left_clip",
    )
    slot_lid.visual(
        Box((0.012, 0.006, 0.014)),
        origin=Origin(xyz=(slot_w * 0.5 - 0.012, 0.003, -0.008)),
        material=body_color,
        name="right_clip",
    )
    slot_lid.visual(
        Box((slot_lid_w, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, -slot_lid_h + 0.003)),
        material=body_color,
        name="drip_edge",
    )
    slot_lid.inertial = Inertial.from_geometry(
        Box((slot_lid_w, 0.012, 0.060)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.002, -0.026)),
    )

    model.articulation(
        "body_to_main_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_y, door_mid_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_slot_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=slot_lid,
        origin=Origin(xyz=(0.0, slot_axis_y, slot_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(80.0),
        ),
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

    body = object_model.get_part("body")
    main_door = object_model.get_part("main_door")
    slot_lid = object_model.get_part("slot_lid")
    door_hinge = object_model.get_articulation("body_to_main_door")
    slot_hinge = object_model.get_articulation("body_to_slot_lid")

    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical hinge axis, got {door_hinge.axis}",
    )
    ctx.check(
        "slot_lid_hinge_axis_is_horizontal",
        tuple(slot_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected top-edge horizontal hinge axis, got {slot_hinge.axis}",
    )
    ctx.check(
        "door_hinge_limits_open_outward",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.6,
        details="door should open outward from the closed pose on a side hinge",
    )
    ctx.check(
        "slot_lid_limits_lift_upward",
        slot_hinge.motion_limits is not None
        and slot_hinge.motion_limits.lower == 0.0
        and slot_hinge.motion_limits.upper is not None
        and slot_hinge.motion_limits.upper > 1.2,
        details="slot lid should rotate upward from the closed pose",
    )

    ctx.expect_contact(main_door, body, name="door_contacts_body_when_closed")
    ctx.expect_contact(slot_lid, body, name="slot_lid_contacts_body_when_closed")
    ctx.expect_overlap(main_door, body, axes="xz", min_overlap=0.30, name="door_covers_front_opening")
    ctx.expect_overlap(slot_lid, body, axes="x", min_overlap=0.24, name="slot_lid_spans_mail_slot")
    ctx.expect_gap(
        slot_lid,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.01,
        name="slot_lid_sits_on_front_face",
    )

    door_rest_aabb = ctx.part_world_aabb(main_door)
    lid_rest_aabb = ctx.part_world_aabb(slot_lid)
    assert door_rest_aabb is not None
    assert lid_rest_aabb is not None

    with ctx.pose({door_hinge: math.radians(90.0)}):
        door_open_aabb = ctx.part_world_aabb(main_door)
        assert door_open_aabb is not None
        ctx.check(
            "door_swings_forward_on_side_hinge",
            door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.24,
            details=(
                "expected main door to swing out in +y, "
                f"rest_aabb={door_rest_aabb}, open_aabb={door_open_aabb}"
            ),
        )

    with ctx.pose({slot_hinge: math.radians(70.0)}):
        lid_open_aabb = ctx.part_world_aabb(slot_lid)
        assert lid_open_aabb is not None
        ctx.check(
            "slot_lid_lifts_for_mail_insertion",
            lid_open_aabb[1][1] > lid_rest_aabb[1][1] + 0.03
            and lid_open_aabb[0][2] > lid_rest_aabb[0][2] + 0.02,
            details=(
                "expected slot lid to rise and tilt outward, "
                f"rest_aabb={lid_rest_aabb}, open_aabb={lid_open_aabb}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
