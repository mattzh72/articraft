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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    # Realistic compact USB-A swivel drive dimensions.
    body_len = 0.048
    body_w = 0.0172
    body_h = 0.0086

    connector_len = 0.012
    connector_w = 0.0122
    connector_h = 0.0046
    shell_t = 0.0005

    tongue_len = 0.008
    tongue_w = 0.0100
    tongue_h = 0.0022
    tongue_x = 0.0048
    tongue_support_len = 0.0022
    tongue_support_h = (connector_h * 0.5) - shell_t - (-tongue_h * 0.5)
    tongue_support_z = -((connector_h * 0.5) - shell_t) + (tongue_support_h * 0.5)

    pivot_x = -0.015
    pin_shaft_r = 0.00135
    pin_head_r = 0.0027
    pin_head_t = 0.0007

    body_slot_half_x = pin_shaft_r
    body_slot_half_z = pin_shaft_r
    body_half_len = body_len * 0.5
    body_half_h = body_h * 0.5

    cover_inner_w = 0.0188
    cheek_t = 0.0008
    cover_outer_w = cover_inner_w + 2.0 * cheek_t
    cover_front_reach = 0.051
    cover_rear_overhang = 0.004
    cheek_h = 0.0122
    bridge_t = 0.0008
    bridge_len = 0.049
    bridge_center_x = 0.0245
    bridge_z = (cheek_h * 0.5) - (bridge_t * 0.5)
    cover_hole_half_x = 0.00175
    cover_hole_half_z = 0.00195
    cover_frame_x = 0.0034
    cheek_y = (cover_inner_w * 0.5) + (cheek_t * 0.5)

    cover_front_strip_len = cover_front_reach - cover_hole_half_x
    cover_front_strip_x = cover_hole_half_x + (cover_front_strip_len * 0.5)
    cover_rear_strip_len = cover_rear_overhang - cover_hole_half_x
    cover_rear_strip_x = -cover_hole_half_x - (cover_rear_strip_len * 0.5)
    cover_bridge_frame_h = (cheek_h * 0.5) - cover_hole_half_z
    cover_upper_frame_z = cover_hole_half_z + (cover_bridge_frame_h * 0.5)
    cover_lower_frame_z = -cover_upper_frame_z

    plastic = model.material("plastic_body", rgba=(0.10, 0.11, 0.13, 1.0))
    tongue_plastic = model.material("tongue_black", rgba=(0.04, 0.04, 0.05, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("pivot_steel", rgba=(0.46, 0.48, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        Box((pivot_x - body_slot_half_x + body_half_len, body_w, body_h)),
        origin=Origin(
            xyz=(
                (-body_half_len + (pivot_x - body_slot_half_x)) * 0.5,
                0.0,
                body_half_h,
            )
        ),
        material=plastic,
        name="rear_block",
    )
    body.visual(
        Box((body_half_len - (pivot_x + body_slot_half_x), body_w, body_h)),
        origin=Origin(
            xyz=(
                ((pivot_x + body_slot_half_x) + body_half_len) * 0.5,
                0.0,
                body_half_h,
            )
        ),
        material=plastic,
        name="front_block",
    )
    body.visual(
        Box((2.0 * body_slot_half_x, body_w, body_half_h - body_slot_half_z)),
        origin=Origin(
            xyz=(
                pivot_x,
                0.0,
                body_half_h + ((body_half_h - body_slot_half_z) * 0.5),
            )
        ),
        material=plastic,
        name="upper_pivot_bridge",
    )
    body.visual(
        Box((2.0 * body_slot_half_x, body_w, body_half_h - body_slot_half_z)),
        origin=Origin(
            xyz=(
                pivot_x,
                0.0,
                ((body_half_h - body_slot_half_z) * 0.5),
            )
        ),
        material=plastic,
        name="lower_pivot_bridge",
    )

    connector = model.part("connector")
    connector.visual(
        Box((connector_len, connector_w, shell_t)),
        origin=Origin(xyz=(connector_len * 0.5, 0.0, (connector_h * 0.5) - (shell_t * 0.5))),
        material=steel,
        name="top_wall",
    )
    connector.visual(
        Box((connector_len, connector_w, shell_t)),
        origin=Origin(xyz=(connector_len * 0.5, 0.0, -(connector_h * 0.5) + (shell_t * 0.5))),
        material=steel,
        name="bottom_wall",
    )
    connector.visual(
        Box((connector_len, shell_t, connector_h - (2.0 * shell_t))),
        origin=Origin(
            xyz=(connector_len * 0.5, (connector_w * 0.5) - (shell_t * 0.5), 0.0)
        ),
        material=steel,
        name="left_wall",
    )
    connector.visual(
        Box((connector_len, shell_t, connector_h - (2.0 * shell_t))),
        origin=Origin(
            xyz=(connector_len * 0.5, -(connector_w * 0.5) + (shell_t * 0.5), 0.0)
        ),
        material=steel,
        name="right_wall",
    )
    connector.visual(
        Box((tongue_len, tongue_w, tongue_h)),
        origin=Origin(xyz=(tongue_x, 0.0, 0.0)),
        material=tongue_plastic,
        name="tongue",
    )
    connector.visual(
        Box((tongue_support_len, tongue_w, tongue_support_h)),
        origin=Origin(xyz=(tongue_support_len * 0.5, 0.0, tongue_support_z)),
        material=tongue_plastic,
        name="tongue_support",
    )

    cover = model.part("swivel_cover")
    for side_name, side_y in (("left", cheek_y), ("right", -cheek_y)):
        cover.visual(
            Box((cover_front_strip_len, cheek_t, cheek_h)),
            origin=Origin(xyz=(cover_front_strip_x, side_y, 0.0)),
            material=steel,
            name=f"{side_name}_front_strip",
        )
        cover.visual(
            Box((cover_rear_strip_len, cheek_t, cheek_h)),
            origin=Origin(xyz=(cover_rear_strip_x, side_y, 0.0)),
            material=steel,
            name=f"{side_name}_rear_strip",
        )
        cover.visual(
            Box((2.0 * cover_frame_x, cheek_t, cover_bridge_frame_h)),
            origin=Origin(xyz=(0.0, side_y, cover_upper_frame_z)),
            material=steel,
            name=f"{side_name}_upper_frame",
        )
        cover.visual(
            Box((2.0 * cover_frame_x, cheek_t, cover_bridge_frame_h)),
            origin=Origin(xyz=(0.0, side_y, cover_lower_frame_z)),
            material=steel,
            name=f"{side_name}_lower_frame",
        )
    cover.visual(
        Box((bridge_len, cover_outer_w, bridge_t)),
        origin=Origin(xyz=(bridge_center_x, 0.0, bridge_z)),
        material=steel,
        name="bridge",
    )

    pin = model.part("pivot_pin")
    pin.visual(
        Cylinder(radius=pin_shaft_r, length=cover_outer_w),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    pin.visual(
        Cylinder(radius=pin_head_r, length=pin_head_t),
        origin=Origin(
            xyz=(0.0, (cover_outer_w * 0.5) + (pin_head_t * 0.5), 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="left_head",
    )
    pin.visual(
        Cylinder(radius=pin_head_r, length=pin_head_t),
        origin=Origin(
            xyz=(0.0, -((cover_outer_w * 0.5) + (pin_head_t * 0.5)), 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="right_head",
    )

    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=body,
        child=connector,
        origin=Origin(xyz=(body_len * 0.5, 0.0, body_h * 0.5)),
    )
    model.articulation(
        "body_to_pin",
        ArticulationType.FIXED,
        parent=body,
        child=pin,
        origin=Origin(xyz=(pivot_x, 0.0, body_h * 0.5)),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pivot_x, 0.0, body_h * 0.5)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
            lower=0.0,
            upper=3.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    connector = object_model.get_part("connector")
    cover = object_model.get_part("swivel_cover")
    pin = object_model.get_part("pivot_pin")
    swivel = object_model.get_articulation("body_to_cover")

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

    ctx.expect_gap(
        connector,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        name="connector_seats_flush_to_body_front",
    )
    ctx.expect_contact(pin, cover, name="pivot_pin_clamps_cover_cheeks")
    ctx.expect_contact(pin, body, name="pivot_pin_carries_in_body_bore")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_overlap(
            cover,
            connector,
            axes="xy",
            elem_a="bridge",
            min_overlap=0.010,
            name="closed_cover_spans_plug_footprint",
        )
        ctx.expect_gap(
            cover,
            connector,
            axis="z",
            positive_elem="bridge",
            min_gap=0.0008,
            max_gap=0.0035,
            name="closed_cover_clears_connector_shell",
        )

    with ctx.pose({swivel: swivel.motion_limits.upper}):
        bridge_aabb = ctx.part_element_world_aabb(cover, elem="bridge")
        connector_aabb = ctx.part_world_aabb(connector)
        ctx.check(
            "opened_cover_swings_behind_connector",
            bridge_aabb is not None
            and connector_aabb is not None
            and bridge_aabb[1][0] < connector_aabb[0][0],
            details=(
                f"bridge_aabb={bridge_aabb}, connector_aabb={connector_aabb}, "
                f"joint_upper={swivel.motion_limits.upper if swivel.motion_limits else None}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
