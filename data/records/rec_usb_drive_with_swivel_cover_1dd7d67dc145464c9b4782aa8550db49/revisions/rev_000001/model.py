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


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_val + z_shift)
        for z_val, y_val in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=8,
        )
    ]


def _circle_profile(
    *,
    radius: float,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 20,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_usb_drive")

    painted_metal = model.material("painted_metal", rgba=(0.44, 0.46, 0.49, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.12, 0.12, 0.13, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.06, 0.06, 0.07, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    tongue_black = model.material("tongue_black", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")

    body_shell_mesh = _mesh(
        "usb_body_shell",
        section_loft(
            [
                _yz_section(-0.0220, width=0.0166, height=0.0080, radius=0.0030),
                _yz_section(-0.0090, width=0.0180, height=0.0088, radius=0.0034),
                _yz_section(0.0080, width=0.0180, height=0.0086, radius=0.0032),
                _yz_section(0.0210, width=0.0160, height=0.0076, radius=0.0027),
            ]
        ),
    )
    body.visual(body_shell_mesh, material=polymer_black, name="body_shell")
    body.visual(
        Box((0.029, 0.0105, 0.0012)),
        origin=Origin(xyz=(-0.0015, 0.0, -0.00455)),
        material=elastomer_black,
        name="grip_pad",
    )
    body.visual(
        Box((0.0046, 0.0136, 0.0056)),
        origin=Origin(xyz=(0.0200, 0.0, 0.0)),
        material=painted_metal,
        name="connector_collar",
    )
    body.visual(
        Box((0.0010, 0.0122, 0.0045)),
        origin=Origin(xyz=(0.0227, 0.0, 0.0)),
        material=connector_metal,
        name="connector_rear_wall",
    )
    body.visual(
        Box((0.0124, 0.0122, 0.00042)),
        origin=Origin(xyz=(0.0284, 0.0, 0.00204)),
        material=connector_metal,
        name="connector_top",
    )
    body.visual(
        Box((0.0124, 0.0122, 0.00042)),
        origin=Origin(xyz=(0.0284, 0.0, -0.00204)),
        material=connector_metal,
        name="connector_bottom",
    )
    body.visual(
        Box((0.0124, 0.00042, 0.00408)),
        origin=Origin(xyz=(0.0284, 0.00589, 0.0)),
        material=connector_metal,
        name="connector_side_left",
    )
    body.visual(
        Box((0.0124, 0.00042, 0.00408)),
        origin=Origin(xyz=(0.0284, -0.00589, 0.0)),
        material=connector_metal,
        name="connector_side_right",
    )
    body.visual(
        Box((0.0102, 0.0084, 0.0011)),
        origin=Origin(xyz=(0.0281, 0.0, -0.00065)),
        material=tongue_black,
        name="connector_tongue",
    )
    body.visual(
        Cylinder(radius=0.00125, length=0.0202),
        origin=Origin(xyz=(-0.0180, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=connector_metal,
        name="pivot_pin_shaft",
    )
    body.visual(
        Cylinder(radius=0.00235, length=0.0007),
        origin=Origin(xyz=(-0.0180, 0.01035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=connector_metal,
        name="pivot_pin_head_left",
    )
    body.visual(
        Cylinder(radius=0.00235, length=0.0007),
        origin=Origin(xyz=(-0.0180, -0.01035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=connector_metal,
        name="pivot_pin_head_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.058, 0.020, 0.010)),
        mass=0.020,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    cover = model.part("swivel_cover")

    cover_side_plate_mesh = _mesh(
        "usb_cover_side_plate",
        ExtrudeWithHolesGeometry(
            _translate_profile(
                rounded_rect_profile(0.0550, 0.0114, 0.0021, corner_segments=8),
                dx=0.0265,
                dy=0.0006,
            ),
            [_circle_profile(radius=0.00175, center=(0.0, 0.0), segments=20)],
            height=0.0008,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )

    cover.visual(
        cover_side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0096, 0.0)),
        material=painted_metal,
        name="left_plate",
    )
    cover.visual(
        cover_side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.0096, 0.0)),
        material=painted_metal,
        name="right_plate",
    )
    cover.visual(
        Box((0.0490, 0.0200, 0.00082)),
        origin=Origin(xyz=(0.0260, 0.0, 0.00566)),
        material=painted_metal,
        name="top_skin",
    )
    cover.visual(
        Box((0.0014, 0.0200, 0.0093)),
        origin=Origin(xyz=(0.0522, 0.0, 0.00035)),
        material=painted_metal,
        name="front_cap",
    )
    cover.visual(
        Cylinder(radius=0.00115, length=0.0200),
        origin=Origin(xyz=(0.0525, 0.0, 0.00515), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_metal,
        name="front_nose",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.055, 0.020, 0.012)),
        mass=0.010,
        origin=Origin(xyz=(0.026, 0.0, 0.0005)),
    )

    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0180, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("swivel_cover")
    swivel = object_model.get_articulation("body_to_swivel_cover")

    body.get_visual("body_shell")
    body.get_visual("pivot_pin_shaft")
    body.get_visual("connector_top")
    cover.get_visual("top_skin")
    cover.get_visual("left_plate")
    cover.get_visual("right_plate")
    cover.get_visual("front_cap")

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

    limits = swivel.motion_limits
    ctx.check(
        "swivel_axis_is_side_pin_pitch",
        tuple(swivel.axis) == (0.0, -1.0, 0.0),
        details=f"expected axis (0, -1, 0), got {swivel.axis}",
    )
    ctx.check(
        "swivel_range_reaches_full_open",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= math.radians(175.0),
        details=f"unexpected motion limits: {limits}",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_skin",
            negative_elem="body_shell",
            min_gap=0.0003,
            max_gap=0.0018,
            name="closed_cover_sits_tightly_over_body",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_skin",
            elem_b="body_shell",
            min_overlap=0.014,
            name="closed_cover_tracks_body_planform",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="front_cap",
            negative_elem="connector_rear_wall",
            min_gap=0.0100,
            max_gap=0.0135,
            name="closed_cover_reaches_connector_tip_region",
        )

    with ctx.pose({swivel: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="body_shell",
            negative_elem="top_skin",
            min_gap=0.0002,
            max_gap=0.0022,
            name="open_cover_clears_body_underside",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="body_shell",
            negative_elem="front_cap",
            min_gap=0.020,
            name="open_cover_swings_behind_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
