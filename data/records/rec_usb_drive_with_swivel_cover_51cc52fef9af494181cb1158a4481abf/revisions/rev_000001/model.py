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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_swivel_usb_drive")

    body_graphite = model.material("body_graphite", rgba=(0.16, 0.18, 0.20, 1.0))
    cover_steel = model.material("cover_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.77, 0.79, 0.81, 1.0))
    insulator_white = model.material("insulator_white", rgba=(0.92, 0.93, 0.90, 1.0))
    index_paint = model.material("index_paint", rgba=(0.88, 0.89, 0.90, 1.0))
    adjuster_bronze = model.material("adjuster_bronze", rgba=(0.63, 0.52, 0.35, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.066, 0.021, 0.016)),
        mass=0.020,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    body_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            _shift_profile(
                rounded_rect_profile(0.0515, 0.0190, 0.0023, corner_segments=8),
                dx=0.02425,
            ),
            0.0096,
            center=True,
        ),
        "usb_body_shell",
    )
    body.visual(
        body_shell_mesh,
        material=body_graphite,
        name="body_shell",
    )
    connector_neck_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            _shift_profile(
                rounded_rect_profile(0.0060, 0.0162, 0.0013, corner_segments=6),
                dx=0.05075,
            ),
            0.0066,
            center=True,
        ),
        "usb_connector_neck",
    )
    body.visual(
        connector_neck_mesh,
        material=body_graphite,
        name="connector_neck",
    )
    body.visual(
        Box((0.0120, 0.0124, 0.0046)),
        origin=Origin(xyz=(0.05955, 0.0, 0.0)),
        material=connector_metal,
        name="connector_shell",
    )
    body.visual(
        Box((0.0080, 0.0100, 0.0011)),
        origin=Origin(xyz=(0.05905, 0.0, -0.0006)),
        material=insulator_white,
        name="connector_tongue",
    )
    body.visual(
        Box((0.0240, 0.0135, 0.0008)),
        origin=Origin(xyz=(0.0190, 0.0, 0.00495)),
        material=index_paint,
        name="top_datum_pad",
    )
    body.visual(
        Box((0.0240, 0.0135, 0.0008)),
        origin=Origin(xyz=(0.0190, 0.0, -0.00495)),
        material=index_paint,
        name="bottom_datum_pad",
    )
    body.visual(
        Box((0.0040, 0.0010, 0.00055)),
        origin=Origin(xyz=(0.0040, 0.0040, 0.00503)),
        material=index_paint,
        name="index_mark_0",
    )
    body.visual(
        Box((0.0052, 0.0010, 0.00055)),
        origin=Origin(xyz=(0.0055, 0.0060, 0.00503)),
        material=index_paint,
        name="index_mark_1",
    )
    body.visual(
        Box((0.0064, 0.0010, 0.00055)),
        origin=Origin(xyz=(0.0072, 0.0080, 0.00503)),
        material=index_paint,
        name="index_mark_2",
    )
    body.visual(
        Cylinder(radius=0.0018, length=0.0022),
        origin=Origin(xyz=(0.018, 0.0106, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=adjuster_bronze,
        name="adjuster_0",
    )
    body.visual(
        Box((0.0034, 0.0018, 0.0038)),
        origin=Origin(xyz=(0.018, 0.0097, 0.0)),
        material=body_graphite,
        name="adjuster_0_boss",
    )
    body.visual(
        Box((0.0030, 0.00055, 0.00055)),
        origin=Origin(xyz=(0.018, 0.01142, 0.0)),
        material=index_paint,
        name="adjuster_0_slot",
    )
    body.visual(
        Cylinder(radius=0.0018, length=0.0022),
        origin=Origin(xyz=(0.033, 0.0106, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=adjuster_bronze,
        name="adjuster_1",
    )
    body.visual(
        Box((0.0034, 0.0018, 0.0038)),
        origin=Origin(xyz=(0.033, 0.0097, 0.0)),
        material=body_graphite,
        name="adjuster_1_boss",
    )
    body.visual(
        Box((0.0030, 0.00055, 0.00055)),
        origin=Origin(xyz=(0.033, 0.01142, 0.0)),
        material=index_paint,
        name="adjuster_1_slot",
    )
    body.visual(
        Cylinder(radius=0.00170, length=0.0144),
        origin=Origin(),
        material=connector_metal,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.00270, length=0.0009),
        origin=Origin(xyz=(0.0, 0.0, 0.00765)),
        material=connector_metal,
        name="pivot_head_top",
    )
    body.visual(
        Cylinder(radius=0.00270, length=0.0009),
        origin=Origin(xyz=(0.0, 0.0, -0.00765)),
        material=connector_metal,
        name="pivot_head_bottom",
    )

    cover = model.part("swivel_cover")
    cover.inertial = Inertial.from_geometry(
        Box((0.071, 0.021, 0.016)),
        mass=0.008,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    plate_outer = _shift_profile(
        rounded_rect_profile(0.0712, 0.0208, 0.0018, corner_segments=8),
        dx=0.0324,
    )
    plate_hole = _shift_profile(
        superellipse_profile(0.0044, 0.0044, exponent=2.0, segments=28),
        dx=0.0,
        dy=0.0,
    )
    plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            plate_outer,
            [plate_hole],
            height=0.0011,
            center=True,
        ),
        "usb_swivel_cover_plate",
    )

    cover.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.00665)),
        material=cover_steel,
        name="top_plate",
    )
    cover.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.00665)),
        material=cover_steel,
        name="bottom_plate",
    )
    cover.visual(
        Box((0.0700, 0.0012, 0.0150)),
        origin=Origin(xyz=(0.0324, -0.0105, 0.0)),
        material=cover_steel,
        name="cover_web",
    )
    cover.visual(
        Box((0.0045, 0.0010, 0.0008)),
        origin=Origin(xyz=(0.0060, 0.0069, 0.00755)),
        material=index_paint,
        name="index_pointer",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("swivel_cover")
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

    limits = swivel.motion_limits
    ctx.check(
        "swivel_axis_is_explicit",
        tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"expected +Z swivel axis, got {swivel.axis}",
    )
    ctx.check(
        "swivel_limits_match_closed_to_open_sweep",
        limits is not None and limits.lower == 0.0 and limits.upper == pi,
        details=f"expected limits [0, pi], got {None if limits is None else (limits.lower, limits.upper)}",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_contact(
            cover,
            body,
            elem_a="top_plate",
            elem_b="pivot_head_top",
            name="top_pin_head_captures_cover",
        )
        ctx.expect_contact(
            cover,
            body,
            elem_a="bottom_plate",
            elem_b="pivot_head_bottom",
            name="bottom_pin_head_captures_cover",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="body_shell",
            min_gap=0.0010,
            max_gap=0.0016,
            name="top_plate_running_gap",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="body_shell",
            negative_elem="bottom_plate",
            min_gap=0.0010,
            max_gap=0.0016,
            name="bottom_plate_running_gap",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="y",
            positive_elem="body_shell",
            negative_elem="cover_web",
            min_gap=0.0003,
            max_gap=0.0007,
            name="left_side_controlled_gap",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            elem_a="top_plate",
            elem_b="body_shell",
            min_overlap=0.045,
            name="closed_cover_tracks_body_length",
        )

    with ctx.pose({swivel: pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_when_fully_open")
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector_shell",
            min_gap=0.040,
            name="connector_exposed_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
