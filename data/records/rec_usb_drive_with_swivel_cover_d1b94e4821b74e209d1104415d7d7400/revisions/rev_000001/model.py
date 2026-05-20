from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / segments),
            radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _side_plate_mesh(thickness: float, hole_radius: float):
    outer = [
        (-0.0095, -0.0085),
        (0.0070, -0.0085),
        (0.0180, -0.0075),
        (0.0550, -0.0075),
        (0.0660, -0.0045),
        (0.0680, 0.0045),
        (0.0600, 0.0095),
        (0.0180, 0.0095),
        (0.0105, 0.0115),
        (-0.0070, 0.0115),
        (-0.0110, 0.0065),
        (-0.0110, -0.0045),
    ]
    return (
        ExtrudeWithHolesGeometry(
            outer,
            [_circle_profile(hole_radius, segments=28)],
            height=thickness,
            center=True,
        )
        .rotate_x(pi / 2.0)
    )


def _pivot_doubler_mesh(thickness: float, outer_radius: float, inner_radius: float):
    return (
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=32),
            [_circle_profile(inner_radius, segments=28)],
            height=thickness,
            center=True,
        )
        .rotate_x(pi / 2.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_swivel_usb_drive")

    dark_elastomer = model.material("dark_elastomer", rgba=(0.14, 0.15, 0.16, 1.0))
    reinforced_black = model.material("reinforced_black", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.80, 0.69, 0.14, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.37, 1.0))
    usb_blue = model.material("usb_blue", rgba=(0.22, 0.45, 0.82, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.062, 0.026, 0.018)),
        mass=0.060,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    body.visual(
        Box((0.048, 0.022, 0.011)),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=dark_elastomer,
        name="housing_shell",
    )
    body.visual(
        Box((0.034, 0.016, 0.002)),
        origin=Origin(xyz=(0.001, 0.0, 0.0110)),
        material=reinforced_black,
        name="top_armor_plate",
    )
    body.visual(
        Box((0.028, 0.0025, 0.006)),
        origin=Origin(xyz=(0.002, 0.0098, 0.0075)),
        material=reinforced_black,
        name="right_side_rail",
    )
    body.visual(
        Box((0.028, 0.0025, 0.006)),
        origin=Origin(xyz=(0.002, -0.0098, 0.0075)),
        material=reinforced_black,
        name="left_side_rail",
    )
    body.visual(
        Box((0.012, 0.018, 0.014)),
        origin=Origin(xyz=(-0.023, 0.0, 0.007)),
        material=reinforced_black,
        name="rear_bumper_block",
    )
    body.visual(
        Box((0.012, 0.010, 0.004)),
        origin=Origin(xyz=(-0.030, 0.0, 0.015)),
        material=dark_steel,
        name="rear_open_stop",
    )
    body.visual(
        Box((0.010, 0.0025, 0.016)),
        origin=Origin(xyz=(-0.016, 0.00975, 0.0065)),
        material=dark_steel,
        name="right_pivot_cheek",
    )
    body.visual(
        Box((0.010, 0.0025, 0.016)),
        origin=Origin(xyz=(-0.016, -0.00975, 0.0065)),
        material=dark_steel,
        name="left_pivot_cheek",
    )
    body.visual(
        Box((0.014, 0.010, 0.0015)),
        origin=Origin(xyz=(0.002, 0.0, 0.01175)),
        material=dark_steel,
        name="closed_stop_pad",
    )
    body.visual(
        Box((0.006, 0.002, 0.009)),
        origin=Origin(xyz=(0.018, 0.0085, 0.0085)),
        material=dark_steel,
        name="right_lockout_lug",
    )
    body.visual(
        Box((0.006, 0.002, 0.009)),
        origin=Origin(xyz=(0.018, -0.0085, 0.0085)),
        material=dark_steel,
        name="left_lockout_lug",
    )
    body.visual(
        Cylinder(radius=0.0024, length=0.036),
        origin=Origin(xyz=(-0.020, 0.0, 0.0055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0036, length=0.0018),
        origin=Origin(xyz=(-0.020, 0.0174, 0.0055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="right_pin_head",
    )
    body.visual(
        Cylinder(radius=0.0036, length=0.0018),
        origin=Origin(xyz=(-0.020, -0.0174, 0.0055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc_steel,
        name="left_pin_head",
    )
    body.visual(
        Box((0.012, 0.014, 0.007)),
        origin=Origin(xyz=(0.028, 0.0, 0.0050)),
        material=reinforced_black,
        name="connector_neck",
    )
    body.visual(
        Box((0.012, 0.0105, 0.0016)),
        origin=Origin(xyz=(0.034, 0.0, 0.0050)),
        material=usb_blue,
        name="usb_tongue",
    )
    body.visual(
        Box((0.015, 0.012, 0.0010)),
        origin=Origin(xyz=(0.0340, 0.0, 0.0075)),
        material=zinc_steel,
        name="usb_shell_top",
    )
    body.visual(
        Box((0.015, 0.012, 0.0010)),
        origin=Origin(xyz=(0.0340, 0.0, 0.0025)),
        material=zinc_steel,
        name="usb_shell_bottom",
    )
    body.visual(
        Box((0.015, 0.0010, 0.0060)),
        origin=Origin(xyz=(0.0340, 0.0055, 0.0050)),
        material=zinc_steel,
        name="usb_shell_right",
    )
    body.visual(
        Box((0.015, 0.0010, 0.0060)),
        origin=Origin(xyz=(0.0340, -0.0055, 0.0050)),
        material=zinc_steel,
        name="usb_shell_left",
    )

    cover = model.part("cover")
    cover.inertial = Inertial.from_geometry(
        Box((0.078, 0.032, 0.022)),
        mass=0.038,
        origin=Origin(xyz=(0.030, 0.0, 0.001)),
    )
    side_plate_mesh = mesh_from_geometry(_side_plate_mesh(0.0030, 0.0034), "usb_cover_side_plate")
    doubler_mesh = mesh_from_geometry(
        _pivot_doubler_mesh(0.0015, 0.0065, 0.0034),
        "usb_cover_pivot_doubler",
    )
    cover.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0140, 0.0)),
        material=safety_yellow,
        name="right_side_plate",
    )
    cover.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.0140, 0.0)),
        material=safety_yellow,
        name="left_side_plate",
    )
    cover.visual(
        doubler_mesh,
        origin=Origin(xyz=(0.0, 0.01575, 0.0)),
        material=dark_steel,
        name="right_pivot_doubler",
    )
    cover.visual(
        doubler_mesh,
        origin=Origin(xyz=(0.0, -0.01575, 0.0)),
        material=dark_steel,
        name="left_pivot_doubler",
    )
    cover.visual(
        Box((0.058, 0.026, 0.003)),
        origin=Origin(xyz=(0.037, 0.0, 0.0085)),
        material=safety_yellow,
        name="top_plate",
    )
    cover.visual(
        Box((0.020, 0.026, 0.002)),
        origin=Origin(xyz=(0.006, 0.0, 0.0105)),
        material=dark_steel,
        name="top_reinforcement_strap",
    )
    cover.visual(
        Box((0.010, 0.032, 0.011)),
        origin=Origin(xyz=(0.068, 0.0, 0.0015)),
        material=safety_yellow,
        name="front_bridge",
    )
    cover.visual(
        Box((0.012, 0.020, 0.0025)),
        origin=Origin(xyz=(0.060, 0.0, -0.00525)),
        material=dark_steel,
        name="lower_guard_skid",
    )
    cover.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.007, 0.0085, 0.0065)),
        material=dark_steel,
        name="right_stop_arm",
    )
    cover.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.007, -0.0085, 0.0065)),
        material=dark_steel,
        name="left_stop_arm",
    )

    for index, x_pos in enumerate((0.014, 0.028, 0.046)):
        cover.visual(
            Cylinder(radius=0.0014, length=0.0012),
            origin=Origin(xyz=(x_pos, 0.0, 0.0106)),
            material=zinc_steel,
            name=f"top_fastener_{index + 1}",
        )
    for side_name, y_pos in (("right", 0.0162), ("left", -0.0162)):
        for index, (x_pos, z_pos) in enumerate(((0.018, 0.0080), (0.050, 0.0055))):
            cover.visual(
                Cylinder(radius=0.0013, length=0.0014),
                origin=Origin(xyz=(x_pos, y_pos, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
                material=zinc_steel,
                name=f"{side_name}_plate_fastener_{index + 1}",
            )

    hinge = model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.020, 0.0, 0.0055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=4.0, lower=0.0, upper=3.05),
    )
    hinge.meta["intent"] = "industrial swivel cover with visible side pin and stop-limited travel"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("body_to_cover")

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

    limits = hinge.motion_limits
    ctx.check(
        "swivel_axis_and_limits",
        hinge.axis == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 2.9 <= limits.upper <= 3.1,
        "Cover hinge should rotate about the visible side pin and stop just short of a full 180-degree fold-back.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            cover,
            body,
            elem_a="top_plate",
            elem_b="closed_stop_pad",
            contact_tol=1e-4,
            name="cover_seats_on_closed_stop_pad",
        )
        ctx.expect_overlap(
            cover,
            body,
            elem_a="top_plate",
            elem_b="closed_stop_pad",
            axes="xy",
            min_overlap=0.008,
            name="closed_stop_pad_sits_under_top_plate",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="front_bridge",
            negative_elem="usb_shell_top",
            min_gap=0.0015,
            max_gap=0.010,
            name="front_bridge_guards_connector_face",
        )

    with ctx.pose({hinge: pi / 2.0}):
        aabb = ctx.part_element_world_aabb(cover, elem="top_plate")
        center_z = None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])
        ctx.check(
            "positive_rotation_opens_cover_upward",
            center_z is not None and center_z > 0.030,
            "Positive hinge motion should swing the cover upward before it folds rearward.",
        )

    with ctx.pose({hinge: 3.0}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="housing_shell",
            negative_elem="top_plate",
            min_gap=0.002,
            name="opened_cover_moves_behind_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
