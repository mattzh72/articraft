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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=7)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_swivel_usb_drive")

    body_polymer = model.material("body_polymer", rgba=(0.16, 0.18, 0.19, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.09, 0.10, 1.0))
    cover_metal = model.material("cover_metal", rgba=(0.66, 0.69, 0.72, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.83, 1.0))
    connector_insert = model.material("connector_insert", rgba=(0.13, 0.14, 0.15, 1.0))

    body = model.part("body")
    body_shell = section_loft(
        [
            _yz_section(0.0198, 0.0104, 0.0042, -0.0290),
            _yz_section(0.0218, 0.0110, 0.0047, -0.0100),
            _yz_section(0.0212, 0.0106, 0.0044, 0.0140),
            _yz_section(0.0196, 0.0094, 0.0039, 0.0290),
        ]
    )
    body.visual(_mesh("usb_body_shell", body_shell), material=body_polymer, name="body_shell")
    body.visual(
        Box((0.0050, 0.0150, 0.0072)),
        origin=Origin(xyz=(0.0265, 0.0, 0.0)),
        material=gasket_black,
        name="seal_block",
    )
    body.visual(
        Box((0.0150, 0.0225, 0.0018)),
        origin=Origin(xyz=(0.0335, 0.0, 0.0048)),
        material=body_polymer,
        name="drip_hood",
    )
    body.visual(
        Box((0.0110, 0.0204, 0.0016)),
        origin=Origin(xyz=(0.0310, 0.0, -0.0043)),
        material=body_polymer,
        name="lower_shelf",
    )
    body.visual(
        Box((0.0100, 0.0018, 0.0074)),
        origin=Origin(xyz=(0.0310, 0.0073, 0.0)),
        material=body_polymer,
        name="right_cheek",
    )
    body.visual(
        Box((0.0100, 0.0018, 0.0074)),
        origin=Origin(xyz=(0.0310, -0.0073, 0.0)),
        material=body_polymer,
        name="left_cheek",
    )
    body.visual(
        Box((0.0060, 0.0048, 0.0096)),
        origin=Origin(xyz=(-0.0255, -0.0082, 0.0)),
        material=body_polymer,
        name="pivot_guard",
    )
    body.visual(
        Cylinder(radius=0.00155, length=0.0129),
        origin=Origin(xyz=(-0.0205, -0.0082, 0.0)),
        material=stainless,
        name="pivot_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.066, 0.024, 0.014)),
        mass=0.025,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    connector = model.part("connector")
    connector.visual(
        Box((0.0132, 0.0124, 0.0046)),
        origin=Origin(xyz=(0.0066, 0.0, 0.0)),
        material=stainless,
        name="plug_shell",
    )
    connector.visual(
        Box((0.0044, 0.0116, 0.0041)),
        origin=Origin(xyz=(0.0022, 0.0, 0.0)),
        material=connector_insert,
        name="plug_insert",
    )
    connector.visual(
        Box((0.0088, 0.0086, 0.0016)),
        origin=Origin(xyz=(0.0088, 0.0, 0.0)),
        material=connector_insert,
        name="plug_tongue",
    )
    connector.inertial = Inertial.from_geometry(
        Box((0.0132, 0.0124, 0.0046)),
        mass=0.004,
        origin=Origin(xyz=(0.0066, 0.0, 0.0)),
    )

    cover = model.part("swivel_cover")
    boss_ring = _mesh(
        "usb_cover_boss_ring",
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0052, segments=48),
            [_circle_profile(0.00155, segments=48)],
            height=0.0007,
            center=True,
        ),
    )
    cover.visual(
        Box((0.0625, 0.0242, 0.0011)),
        origin=Origin(xyz=(0.03425, 0.0082, 0.0069)),
        material=cover_metal,
        name="top_plate",
    )
    cover.visual(
        Box((0.0625, 0.0242, 0.0011)),
        origin=Origin(xyz=(0.03425, 0.0082, -0.0069)),
        material=cover_metal,
        name="bottom_plate",
    )
    cover.visual(
        Box((0.0030, 0.0242, 0.0132)),
        origin=Origin(xyz=(0.0660, 0.0082, 0.0)),
        material=cover_metal,
        name="front_bridge",
    )
    cover.visual(
        boss_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0061)),
        material=cover_metal,
        name="top_boss",
    )
    cover.visual(
        boss_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.0061)),
        material=cover_metal,
        name="bottom_boss",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.069, 0.026, 0.014)),
        mass=0.009,
        origin=Origin(xyz=(0.034, 0.008, 0.0)),
    )

    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=body,
        child=connector,
        origin=Origin(xyz=(0.0290, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0205, -0.0082, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=5.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    connector = object_model.get_part("connector")
    cover = object_model.get_part("swivel_cover")
    cover_joint = object_model.get_articulation("body_to_cover")

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
        "cover_joint_axis_is_vertical_swivel",
        tuple(cover_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected swivel axis (0, 0, 1), got {cover_joint.axis}",
    )
    ctx.check(
        "cover_joint_has_180_degree_travel",
        cover_joint.motion_limits is not None
        and cover_joint.motion_limits.lower == 0.0
        and abs((cover_joint.motion_limits.upper or 0.0) - math.pi) < 1e-6,
        details="swivel cover should open from closed at 0 rad to a full 180-degree open stop",
    )

    ctx.expect_contact(
        body,
        connector,
        elem_a="seal_block",
        elem_b="plug_shell",
        name="connector_is_seated_in_sealed_body_interface",
    )
    ctx.expect_contact(
        cover,
        body,
        elem_a="top_boss",
        elem_b="pivot_pin",
        name="cover_is_captured_by_visible_pivot_pin",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_overlap(
            cover,
            connector,
            axes="yz",
            elem_a="front_bridge",
            elem_b="plug_shell",
            min_overlap=0.004,
            name="closed_cover_shrouds_connector_face",
        )
        ctx.expect_gap(
            cover,
            connector,
            axis="x",
            positive_elem="front_bridge",
            negative_elem="plug_shell",
            min_gap=0.001,
            max_gap=0.004,
            name="closed_cover_keeps_drip_gap_ahead_of_plug",
        )

    with ctx.pose({cover_joint: math.pi}):
        ctx.expect_gap(
            connector,
            cover,
            axis="x",
            positive_elem="plug_shell",
            negative_elem="front_bridge",
            min_gap=0.060,
            name="open_cover_swings_clear_of_connector",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
