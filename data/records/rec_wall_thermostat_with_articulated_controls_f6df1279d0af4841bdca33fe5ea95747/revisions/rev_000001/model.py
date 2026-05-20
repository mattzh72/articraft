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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _disc_mesh(name: str, *, radius: float, thickness: float, segments: int = 64):
    return mesh_from_geometry(
        ExtrudeGeometry(
            _circle_profile(radius, segments=segments),
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 64,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wall_thermostat")

    wall_white = model.material("wall_white", rgba=(0.94, 0.95, 0.96, 1.0))
    shell_offwhite = model.material("shell_offwhite", rgba=(0.88, 0.89, 0.87, 1.0))
    dial_satin = model.material("dial_satin", rgba=(0.74, 0.76, 0.78, 1.0))
    marks_dark = model.material("marks_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    fastener_dark = model.material("fastener_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    housing_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.132, 0.132, 0.018),
            0.014,
            cap=True,
            center=True,
            closed=True,
        ),
        "thermostat_housing_shell",
    )
    bezel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.124, 0.124, 0.015),
            0.003,
            cap=True,
            center=True,
            closed=True,
        ),
        "thermostat_face_bezel",
    )
    shaft_mesh = _disc_mesh("thermostat_center_shaft", radius=0.0072, thickness=0.0073)

    face_annulus_mesh = _annulus_mesh(
        "thermostat_dial_face_annulus",
        outer_radius=0.041,
        inner_radius=0.0124,
        thickness=0.005,
    )
    grip_ring_mesh = _annulus_mesh(
        "thermostat_dial_grip_ring",
        outer_radius=0.045,
        inner_radius=0.0385,
        thickness=0.013,
    )
    retainer_cap_mesh = _disc_mesh("thermostat_retainer_cap", radius=0.0104, thickness=0.002)
    retainer_head_mesh = _disc_mesh("thermostat_retainer_head", radius=0.0054, thickness=0.0014)

    body = model.part("thermostat_body")
    body.visual(
        Box((0.146, 0.146, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=wall_white,
        name="wall_plate",
    )
    body.visual(
        housing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=shell_offwhite,
        name="housing_shell",
    )
    body.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=shell_offwhite,
        name="face_bezel",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=shell_offwhite,
        name="dial_seat",
    )
    body.visual(
        Box((0.034, 0.006, 0.0012)),
        origin=Origin(xyz=(0.0, 0.048, 0.0224)),
        material=marks_dark,
        name="top_datum_flat",
    )
    body.visual(
        Box((0.026, 0.005, 0.0012)),
        origin=Origin(xyz=(0.0, -0.047, 0.0224)),
        material=marks_dark,
        name="bottom_datum_flat",
    )
    body.visual(
        Box((0.0045, 0.010, 0.0014)),
        origin=Origin(xyz=(0.0, 0.042, 0.0223)),
        material=marks_dark,
        name="primary_index",
    )
    for tick_index, angle_deg in enumerate(range(-120, 121, 30)):
        if angle_deg == 90:
            continue
        angle = math.radians(float(angle_deg))
        tick_x, tick_y = _polar_xy(0.0465, angle)
        body.visual(
            Box((0.0024, 0.008, 0.0012)),
            origin=Origin(
                xyz=(tick_x, tick_y, 0.0224),
                rpy=(0.0, 0.0, angle),
            ),
            material=marks_dark,
            name=f"tick_{tick_index}",
        )
    body.visual(
        Cylinder(radius=0.011, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0242)),
        material=fastener_dark,
        name="shaft_shoulder",
    )
    body.visual(
        shaft_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.02665)),
        material=fastener_dark,
        name="shaft",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.146, 0.146, 0.023)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )

    dial = model.part("dial")
    dial.visual(
        face_annulus_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=dial_satin,
        name="face_annulus",
    )
    dial.visual(
        grip_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=dial_satin,
        name="grip_ring",
    )
    dial.visual(
        Box((0.0032, 0.016, 0.0018)),
        origin=Origin(xyz=(0.0, 0.033, 0.0129)),
        material=marks_dark,
        name="pointer",
    )
    hub_cap_mesh = _annulus_mesh(
        "thermostat_dial_hub_cap",
        outer_radius=0.0102,
        inner_radius=0.0062,
        thickness=0.0012,
    )
    dial.visual(
        hub_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0101)),
        material=dial_satin,
        name="hub_cap",
    )
    for spoke_index, angle in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        spoke_x, spoke_y = _polar_xy(0.024, angle)
        dial.visual(
            Box((0.032, 0.006, 0.0075)),
            origin=Origin(
                xyz=(spoke_x, spoke_y, 0.0073),
                rpy=(0.0, 0.0, angle),
            ),
            material=dial_satin,
            name=f"hub_spoke_{spoke_index}",
        )
    for rib_index in range(18):
        angle = (2.0 * math.pi * rib_index) / 18.0
        rib_x, rib_y = _polar_xy(0.0436, angle)
        dial.visual(
            Box((0.0028, 0.0050, 0.0085)),
            origin=Origin(
                xyz=(rib_x, rib_y, 0.0076),
                rpy=(0.0, 0.0, angle),
            ),
            material=dial_satin,
            name=f"grip_rib_{rib_index}",
        )
    dial.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.014)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    retainer = model.part("retainer")
    retainer.visual(
        retainer_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0083)),
        material=fastener_dark,
        name="retainer_cap",
    )
    retainer.visual(
        retainer_head_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
        material=fastener_dark,
        name="retainer_head",
    )
    retainer.visual(
        Box((0.008, 0.0016, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0109)),
        material=marks_dark,
        name="retainer_slot",
    )
    retainer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0104, length=0.004),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.0090)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.0,
            lower=-math.radians(140.0),
            upper=math.radians(140.0),
        ),
    )
    model.articulation(
        "body_to_retainer",
        ArticulationType.FIXED,
        parent=body,
        child=retainer,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("thermostat_body")
    dial = object_model.get_part("dial")
    retainer = object_model.get_part("retainer")
    dial_joint = object_model.get_articulation("body_to_dial")

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

    ctx.expect_contact(
        body,
        retainer,
        elem_a="shaft",
        elem_b="retainer_cap",
        name="retainer_is_mounted_to_shaft",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="grip_ring",
        negative_elem="dial_seat",
        min_gap=0.0009,
        max_gap=0.0011,
        name="dial_face_gap_is_controlled",
    )
    ctx.expect_gap(
        dial,
        retainer,
        axis="z",
        positive_elem="hub_cap",
        negative_elem="retainer_cap",
        min_gap=0.00015,
        max_gap=0.00045,
        name="retainer_capture_gap_is_controlled",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="grip_ring",
        elem_b="dial_seat",
        min_overlap=0.080,
        name="dial_is_centered_on_datum_seat",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="hub_cap",
        negative_elem="shaft",
        min_gap=0.0021,
        max_gap=0.0023,
        name="shaft_remains_visibly_clear_below_dial_hub",
    )
    ctx.expect_overlap(
        dial,
        retainer,
        axes="xy",
        elem_a="hub_cap",
        elem_b="retainer_cap",
        min_overlap=0.015,
        name="retainer_is_concentric_with_dial_hub",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xy",
        elem_a="pointer",
        elem_b="primary_index",
        min_overlap=0.003,
        name="dial_pointer_is_zero_aligned",
    )

    closed_origin = ctx.part_world_position(dial)
    closed_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))
    with ctx.pose({dial_joint: math.radians(60.0)}):
        turned_origin = ctx.part_world_position(dial)
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(dial, elem="pointer"))

    origin_ok = (
        closed_origin is not None
        and turned_origin is not None
        and abs(closed_origin[0] - turned_origin[0]) <= 1e-6
        and abs(closed_origin[1] - turned_origin[1]) <= 1e-6
        and abs(closed_origin[2] - turned_origin[2]) <= 1e-6
    )
    ctx.check(
        "dial_rotates_about_fixed_center_axis",
        origin_ok,
        details=f"closed_origin={closed_origin}, turned_origin={turned_origin}",
    )

    pointer_ok = (
        closed_pointer is not None
        and turned_pointer is not None
        and abs(closed_pointer[0]) <= 0.002
        and turned_pointer[0] < -0.020
        and turned_pointer[1] > 0.012
    )
    ctx.check(
        "positive_dial_rotation_moves_pointer_counterclockwise",
        pointer_ok,
        details=f"closed_pointer={closed_pointer}, turned_pointer={turned_pointer}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
