from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _yz_section(x: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_swivel_usb_drive")

    composite_olive = model.material("composite_olive", rgba=(0.34, 0.38, 0.27, 1.0))
    bakelite_black = model.material("bakelite_black", rgba=(0.12, 0.11, 0.10, 1.0))
    service_gray = model.material("service_gray", rgba=(0.48, 0.50, 0.52, 1.0))
    stamped_steel = model.material("stamped_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.33, 1.0))

    body_length = 0.042
    body_width = 0.019
    body_thickness = 0.0088
    body_front_x = body_length * 0.5
    body_rear_x = -body_length * 0.5

    connector_length = 0.0124
    connector_width = 0.0122
    connector_height = 0.0046
    adapter_length = 0.0062
    adapter_block_length = 0.0068

    pivot_x = body_rear_x + 0.0047
    pivot_y = -body_width * 0.5 + 0.0024
    pin_radius = 0.00165
    pin_head_radius = 0.00315

    drive_body = model.part("drive_body")
    drive_body.inertial = Inertial.from_geometry(
        Box((0.062, 0.024, 0.012)),
        mass=0.030,
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
    )

    body_shell = section_loft(
        [
            _yz_section(body_rear_x, 0.0182, 0.0086, 0.0022),
            _yz_section(-0.008, 0.0190, 0.0092, 0.0025),
            _yz_section(0.010, 0.0192, 0.0091, 0.0026),
            _yz_section(body_front_x, 0.0178, 0.0084, 0.0021),
        ]
    )
    drive_body.visual(
        _save_mesh("usb_drive_body_shell", body_shell),
        material=composite_olive,
        name="body_shell",
    )

    drive_body.visual(
        Box((0.0175, 0.0112, 0.0012)),
        origin=Origin(xyz=(-0.001, 0.0008, body_thickness * 0.5 + 0.0005)),
        material=service_gray,
        name="service_hatch",
    )
    drive_body.visual(
        Box((0.0105, 0.0016, 0.0058)),
        origin=Origin(xyz=(pivot_x + 0.0062, -body_width * 0.5 - 0.0008, 0.0)),
        material=dark_steel,
        name="side_reinforcement",
    )
    drive_body.visual(
        Box((0.0140, 0.0034, 0.0042)),
        origin=Origin(xyz=(body_front_x - 0.0005, 0.0066, 0.0031)),
        material=service_gray,
        name="upper_adapter_gusset",
    )
    drive_body.visual(
        Box((0.0140, 0.0034, 0.0042)),
        origin=Origin(xyz=(body_front_x - 0.0005, -0.0066, 0.0031)),
        material=service_gray,
        name="lower_adapter_gusset",
    )
    drive_body.visual(
        Box((adapter_block_length, body_width * 0.90, body_thickness * 0.78)),
        origin=Origin(xyz=(body_front_x + adapter_block_length * 0.5 - 0.0003, 0.0, 0.0)),
        material=stamped_steel,
        name="adapter_block",
    )
    drive_body.visual(
        Box((connector_length, connector_width, connector_height)),
        origin=Origin(
            xyz=(body_front_x + adapter_length + connector_length * 0.5 - 0.0002, 0.0, 0.0)
        ),
        material=stamped_steel,
        name="connector_shell",
    )
    drive_body.visual(
        Box((connector_length * 0.82, connector_width * 0.60, 0.0024)),
        origin=Origin(
            xyz=(body_front_x + adapter_length + connector_length * 0.39, 0.0, -0.0011)
        ),
        material=bakelite_black,
        name="connector_core",
    )

    for name, x, y in (
        ("hatch_bolt_left", -0.0062, -0.0030),
        ("hatch_bolt_right", 0.0042, 0.0030),
        ("adapter_bolt_upper", body_front_x + 0.0010, 0.0066),
        ("adapter_bolt_lower", body_front_x + 0.0010, -0.0066),
    ):
        drive_body.visual(
            Cylinder(radius=0.0011, length=0.0010),
            origin=Origin(xyz=(x, y, body_thickness * 0.5 + 0.0010)),
            material=brass,
            name=name,
        )

    drive_body.visual(
        Cylinder(radius=pin_radius, length=body_thickness + 0.0034),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        material=dark_steel,
        name="pivot_pin_shaft",
    )
    drive_body.visual(
        Cylinder(radius=pin_head_radius, length=0.0012),
        origin=Origin(xyz=(pivot_x, pivot_y, body_thickness * 0.5 + 0.0006)),
        material=brass,
        name="pivot_pin_head_top",
    )
    drive_body.visual(
        Cylinder(radius=pin_head_radius * 0.92, length=0.0012),
        origin=Origin(xyz=(pivot_x, pivot_y, -body_thickness * 0.5 - 0.0006)),
        material=brass,
        name="pivot_pin_head_bottom",
    )

    swivel_cover = model.part("swivel_cover")
    swivel_cover.inertial = Inertial.from_geometry(
        Box((0.062, 0.025, 0.010)),
        mass=0.012,
        origin=Origin(xyz=(0.028, 0.001, 0.003)),
    )

    cover_length = 0.060
    cover_width = 0.023
    cover_plate_t = 0.0014
    cover_side_depth = 0.0045
    cover_side_t = 0.0012
    cover_plate_z = body_thickness * 0.5 + 0.00045 + cover_plate_t * 0.5

    strap_profile = _shift_profile(
        rounded_rect_profile(cover_length, cover_width, 0.0028, corner_segments=8),
        dx=0.026,
        dy=0.0010,
    )
    pivot_hole = _shift_profile(
        superellipse_profile(0.0044, 0.0044, exponent=2.0, segments=24),
        dx=0.0,
        dy=0.0,
    )
    top_strap_geom = ExtrudeWithHolesGeometry(
        strap_profile,
        [pivot_hole],
        height=cover_plate_t,
        center=True,
    ).translate(0.0, 0.0, cover_plate_z)

    doubler_profile = _shift_profile(
        rounded_rect_profile(0.0115, 0.0090, 0.0020, corner_segments=6),
        dx=0.0030,
        dy=0.0003,
    )
    doubler_geom = ExtrudeWithHolesGeometry(
        doubler_profile,
        [pivot_hole],
        height=0.0009,
        center=True,
    ).translate(0.0, 0.0, cover_plate_z + cover_plate_t * 0.5 + 0.00045)
    top_strap_geom.merge(doubler_geom)

    side_frame_geom = BoxGeometry((0.0510, cover_side_t, cover_side_depth)).translate(
        0.0280,
        cover_width * 0.5 - cover_side_t * 0.5 - 0.0003,
        cover_plate_z - cover_plate_t * 0.5 - cover_side_depth * 0.5 + 0.00015,
    )
    side_frame_geom.merge(
        BoxGeometry((0.0510, cover_side_t, cover_side_depth)).translate(
            0.0280,
            -cover_width * 0.5 + cover_side_t * 0.5 + 0.0003,
            cover_plate_z - cover_plate_t * 0.5 - cover_side_depth * 0.5 + 0.00015,
        )
    )
    side_frame_geom.merge(
        BoxGeometry((0.0052, cover_width - 0.0030, 0.0028)).translate(
            0.0535,
            0.0010,
            cover_plate_z - cover_plate_t * 0.5 - 0.00135,
        )
    )
    side_frame_geom.merge(
        BoxGeometry((0.0080, cover_width - 0.0040, 0.0018)).translate(
            0.0060,
            0.0010,
            cover_plate_z - cover_plate_t * 0.5 - 0.00075,
        )
    )

    swivel_cover.visual(
        _save_mesh("usb_swivel_cover_top", top_strap_geom),
        material=stamped_steel,
        name="top_strap",
    )
    swivel_cover.visual(
        _save_mesh("usb_swivel_cover_frame", side_frame_geom),
        material=dark_steel,
        name="side_frame",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=drive_body,
        child=swivel_cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=0.0, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drive_body = object_model.get_part("drive_body")
    swivel_cover = object_model.get_part("swivel_cover")
    hinge = object_model.get_articulation("body_to_cover")

    drive_body.get_visual("body_shell")
    drive_body.get_visual("service_hatch")
    drive_body.get_visual("adapter_block")
    drive_body.get_visual("connector_shell")
    drive_body.get_visual("pivot_pin_shaft")
    drive_body.get_visual("pivot_pin_head_top")
    swivel_cover.get_visual("top_strap")
    swivel_cover.get_visual("side_frame")

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
        "cover_joint_axis_is_vertical_pin",
        hinge.axis == (0.0, 0.0, -1.0),
        details=f"expected swivel axis (0, 0, -1), got {hinge.axis}",
    )
    ctx.check(
        "cover_joint_limits_span_swivel_arc",
        limits is not None and limits.lower == 0.0 and abs((limits.upper or 0.0) - pi) < 1e-6,
        details=f"expected 0..pi swivel motion, got {limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            swivel_cover,
            drive_body,
            axis="z",
            positive_elem="top_strap",
            negative_elem="body_shell",
            min_gap=0.0002,
            max_gap=0.0016,
            name="closed_cover_stands_proud_of_body",
        )
        ctx.expect_overlap(
            swivel_cover,
            drive_body,
            axes="xy",
            elem_a="top_strap",
            elem_b="body_shell",
            min_overlap=0.010,
            name="closed_cover_overlaps_body_plan",
        )
        ctx.expect_overlap(
            swivel_cover,
            drive_body,
            axes="x",
            elem_a="top_strap",
            elem_b="connector_shell",
            min_overlap=0.010,
            name="closed_cover_reaches_over_connector",
        )

    with ctx.pose({hinge: pi}):
        ctx.expect_gap(
            drive_body,
            swivel_cover,
            axis="x",
            positive_elem="connector_shell",
            negative_elem="top_strap",
            min_gap=0.008,
            name="open_cover_clears_connector_front",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
