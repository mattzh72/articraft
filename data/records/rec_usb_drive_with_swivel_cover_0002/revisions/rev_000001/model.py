from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


BODY_LENGTH = 0.044
BODY_WIDTH = 0.021
BODY_THICKNESS = 0.0108
BODY_TOP = BODY_THICKNESS / 2.0

CONNECTOR_LENGTH = 0.012
CONNECTOR_WIDTH = 0.0122
CONNECTOR_THICKNESS = 0.0046

PIVOT_X = 0.0065
PIVOT_Y = -0.0065
PIVOT_RADIUS = 0.0017

COVER_LENGTH = 0.035
COVER_WIDTH = 0.0230
COVER_PLATE_THICKNESS = 0.0016
COVER_CLEARANCE = 0.0010
COVER_TOP_Z = BODY_TOP + COVER_CLEARANCE + COVER_PLATE_THICKNESS / 2.0
COVER_CHANNEL_HEIGHT = BODY_THICKNESS + 2.0 * COVER_CLEARANCE + 2.0 * COVER_PLATE_THICKNESS


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circumscribed_hole_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    polygon_radius = radius / math.cos(math.pi / segments)
    return [
        (
            polygon_radius * math.cos(2.0 * math.pi * i / segments),
            polygon_radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _yz_section(x: float, width: float, thickness: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, thickness, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_swivel_usb_drive", assets=ASSETS)

    body_polymer = model.material("body_polymer", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    painted_metal = model.material("painted_metal", rgba=(0.84, 0.40, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    connector_insulator = model.material("connector_insulator", rgba=(0.08, 0.09, 0.10, 1.0))

    body = model.part("body")
    body_sections = [
        _yz_section(0.0000, 0.0228, 0.0122, 0.0048),
        _yz_section(0.0100, 0.0220, 0.0118, 0.0046),
        _yz_section(0.0310, 0.0212, 0.0110, 0.0041),
        _yz_section(BODY_LENGTH, 0.0190, 0.0098, 0.0035),
    ]
    body_shell = mesh_from_geometry(
        section_loft(body_sections),
        ASSETS.mesh_path("body_shell.obj"),
    )
    body.visual(body_shell, material=body_polymer, name="body_shell")
    body.visual(
        Box((0.0075, 0.0232, 0.0124)),
        origin=Origin(xyz=(BODY_LENGTH - 0.0038, 0.0, 0.0)),
        material=rubber_black,
        name="nose_guard",
    )
    body.visual(
        Box((0.026, 0.0030, 0.0068)),
        origin=Origin(xyz=(0.024, BODY_WIDTH / 2.0 + 0.00125, 0.0)),
        material=rubber_black,
        name="side_grip_pad",
    )
    body.visual(
        Box((0.026, 0.0030, 0.0068)),
        origin=Origin(xyz=(0.024, -(BODY_WIDTH / 2.0 + 0.00125), 0.0)),
        material=rubber_black,
        name="side_grip_pad_opposite",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0006),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, BODY_TOP - 0.0003)),
        material=body_polymer,
        name="pivot_boss_top",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0006),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, -BODY_TOP + 0.0003)),
        material=body_polymer,
        name="pivot_boss_bottom",
    )
    body.visual(
        Cylinder(radius=PIVOT_RADIUS, length=COVER_CHANNEL_HEIGHT),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    for idx, x_pos in enumerate((0.018, 0.031), start=1):
        body.visual(
            Cylinder(radius=0.00175, length=0.0009),
            origin=Origin(
                xyz=(x_pos, BODY_WIDTH / 2.0 + 0.0020, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"service_fastener_{idx}",
        )

    connector = model.part("connector")
    connector.visual(
        Box((CONNECTOR_LENGTH, CONNECTOR_WIDTH, 0.00045)),
        origin=Origin(
            xyz=(CONNECTOR_LENGTH / 2.0, 0.0, CONNECTOR_THICKNESS / 2.0 - 0.000225),
        ),
        material=steel,
        name="shell_top",
    )
    connector.visual(
        Box((CONNECTOR_LENGTH, CONNECTOR_WIDTH, 0.00045)),
        origin=Origin(
            xyz=(CONNECTOR_LENGTH / 2.0, 0.0, -CONNECTOR_THICKNESS / 2.0 + 0.000225),
        ),
        material=steel,
        name="shell_bottom",
    )
    connector.visual(
        Box((CONNECTOR_LENGTH, 0.00055, CONNECTOR_THICKNESS)),
        origin=Origin(xyz=(CONNECTOR_LENGTH / 2.0, CONNECTOR_WIDTH / 2.0 - 0.000275, 0.0)),
        material=steel,
        name="shell_right",
    )
    connector.visual(
        Box((CONNECTOR_LENGTH, 0.00055, CONNECTOR_THICKNESS)),
        origin=Origin(xyz=(CONNECTOR_LENGTH / 2.0, -CONNECTOR_WIDTH / 2.0 + 0.000275, 0.0)),
        material=steel,
        name="shell_left",
    )
    connector.visual(
        Box((0.0018, CONNECTOR_WIDTH - 0.0012, CONNECTOR_THICKNESS - 0.0009)),
        origin=Origin(xyz=(0.0009, 0.0, 0.0)),
        material=connector_insulator,
        name="rear_insert",
    )
    connector.visual(
        Box((CONNECTOR_LENGTH - 0.0018, 0.0086, 0.0022)),
        origin=Origin(xyz=(0.0018 + (CONNECTOR_LENGTH - 0.0018) / 2.0, 0.0, -0.00015)),
        material=connector_insulator,
        name="contact_tongue",
    )
    for idx, y_pos in enumerate((-0.0032, 0.0032), start=1):
        connector.visual(
            Box((0.0015, 0.0013, 0.00045)),
            origin=Origin(
                xyz=(0.0065, y_pos, CONNECTOR_THICKNESS / 2.0 + 0.000225),
            ),
            material=steel,
            name=f"shell_latch_dimple_{idx}",
        )

    cover = model.part("cover")
    plate_center = (0.0135, 0.0007)
    plate_profile = _translate_profile(
        rounded_rect_profile(COVER_LENGTH, COVER_WIDTH, 0.0026),
        dx=plate_center[0],
        dy=plate_center[1],
    )
    pivot_hole_profile = _circumscribed_hole_profile(PIVOT_RADIUS, segments=24)
    slot_hole_profile = _translate_profile(
        rounded_rect_profile(0.0140, 0.0058, 0.0022),
        dx=0.0215,
        dy=plate_center[1],
    )
    plate_geometry = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            plate_profile,
            [pivot_hole_profile, slot_hole_profile],
            COVER_PLATE_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("cover_plate.obj"),
    )
    cover.visual(
        plate_geometry,
        origin=Origin(xyz=(0.0, 0.0, COVER_TOP_Z)),
        material=painted_metal,
        name="cover_top_plate",
    )
    cover.visual(
        plate_geometry,
        origin=Origin(xyz=(0.0, 0.0, -COVER_TOP_Z)),
        material=painted_metal,
        name="cover_bottom_plate",
    )
    cover.visual(
        Box((COVER_LENGTH, 0.0016, COVER_CHANNEL_HEIGHT)),
        origin=Origin(xyz=(plate_center[0], -0.0110, 0.0)),
        material=painted_metal,
        name="cover_bridge",
    )
    cover.visual(
        Box((0.010, 0.009, 0.0009)),
        origin=Origin(xyz=(0.0055, -0.0016, COVER_TOP_Z + 0.00035)),
        material=painted_metal,
        name="joint_reinforce_top",
    )
    cover.visual(
        Box((0.010, 0.009, 0.0009)),
        origin=Origin(xyz=(0.0055, -0.0016, -COVER_TOP_Z - 0.00035)),
        material=painted_metal,
        name="joint_reinforce_bottom",
    )
    cover.visual(
        Cylinder(radius=0.0036, length=0.0005),
        origin=Origin(xyz=(0.0, 0.0, COVER_TOP_Z + 0.00025)),
        material=steel,
        name="pivot_washer_top",
    )
    cover.visual(
        Cylinder(radius=0.0036, length=0.0005),
        origin=Origin(xyz=(0.0, 0.0, -COVER_TOP_Z - 0.00025)),
        material=steel,
        name="pivot_washer_bottom",
    )
    cover.visual(
        Cylinder(radius=0.0018, length=0.0009),
        origin=Origin(xyz=(0.024, 0.0045, COVER_TOP_Z + 0.00045)),
        material=steel,
        name="cover_fastener_top",
    )
    cover.visual(
        Cylinder(radius=0.0018, length=0.0009),
        origin=Origin(xyz=(0.024, 0.0045, -COVER_TOP_Z - 0.00045)),
        material=steel,
        name="cover_fastener_bottom",
    )

    model.articulation(
        "body_to_connector",
        ArticulationType.FIXED,
        parent=body,
        child=connector,
        origin=Origin(xyz=(BODY_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=6.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    body = object_model.get_part("body")
    connector = object_model.get_part("connector")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    cover_top = cover.get_visual("cover_top_plate")
    cover_bottom = cover.get_visual("cover_bottom_plate")

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    limits = swivel.motion_limits
    axis_ok = swivel.axis == (0.0, 0.0, 1.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, 0.0, abs_tol=1e-9)
        and math.isclose(limits.upper, math.pi, rel_tol=0.0, abs_tol=1e-6)
    )
    ctx.check("swivel_axis_vertical", axis_ok, f"Expected swivel axis (0, 0, 1), got {swivel.axis}.")
    ctx.check(
        "swivel_has_half_turn_limits",
        limits_ok,
        "Swivel cover should rotate through a realistic 180 degree range.",
    )

    ctx.expect_contact(connector, body, contact_tol=1e-5, name="connector_mounted_to_body")
    ctx.expect_contact(cover, body, contact_tol=1e-5, name="cover_captured_on_pivot")
    ctx.expect_gap(
        connector,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.0002,
        name="connector_seats_against_body_front",
    )
    ctx.expect_overlap(
        connector,
        body,
        axes="yz",
        min_overlap=0.004,
        name="connector_aligned_with_body_centerline",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem=cover_top,
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.0012,
        name="cover_top_plate_stands_off_body",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="body_shell",
        negative_elem=cover_bottom,
        min_gap=0.0002,
        max_gap=0.0012,
        name="cover_bottom_plate_stands_off_body",
    )

    body_aabb = ctx.part_world_aabb(body)
    connector_aabb = ctx.part_world_aabb(connector)
    cover_top_aabb = ctx.part_element_world_aabb(cover, elem="cover_top_plate")
    ctx.check(
        "connector_protrudes_from_front",
        body_aabb is not None
        and connector_aabb is not None
        and connector_aabb[1][0] > body_aabb[1][0] + 0.010,
        "The exposed USB-A shell should project clearly beyond the drive body.",
    )
    ctx.check(
        "rest_pose_keeps_connector_exposed",
        connector_aabb is not None
        and cover_top_aabb is not None
        and cover_top_aabb[1][0] < connector_aabb[0][0] - 0.0015,
        "The swivel cover should stop short of the exposed connector in the default pose.",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({swivel: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="swivel_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="swivel_lower_no_floating")
            ctx.expect_contact(cover, body, contact_tol=1e-5, name="swivel_lower_stays_captured")

        with ctx.pose({swivel: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="swivel_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="swivel_upper_no_floating")
            ctx.expect_contact(cover, body, contact_tol=1e-5, name="swivel_upper_stays_captured")

            body_upper_aabb = ctx.part_world_aabb(body)
            cover_upper_aabb = ctx.part_element_world_aabb(cover, elem="cover_top_plate")
            ctx.check(
                "cover_swings_behind_body",
                body_upper_aabb is not None
                and cover_upper_aabb is not None
                and cover_upper_aabb[1][0] < body_upper_aabb[0][0] + 0.014,
                "At full travel the cover should fold behind the body, not hover beside the connector.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
