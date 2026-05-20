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
    LatheGeometry,
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


def _rounded_rect_section(width: float, height: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, height, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_wall_thermostat")

    enclosure_offwhite = model.material("enclosure_offwhite", rgba=(0.84, 0.85, 0.82, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.18, 0.20, 0.22, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.09, 0.10, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.95, 0.96, 0.96, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.155, 0.185, 0.066)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    body.visual(
        Box((0.155, 0.185, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=enclosure_offwhite,
        name="wall_backplate",
    )

    housing_geom = section_loft(
        [
            _rounded_rect_section(0.132, 0.168, 0.018, 0.006),
            _rounded_rect_section(0.126, 0.162, 0.019, 0.026),
            _rounded_rect_section(0.118, 0.152, 0.020, 0.048),
        ]
    )
    body.visual(
        _mesh("thermostat_housing_shell", housing_geom),
        material=enclosure_offwhite,
        name="front_housing",
    )

    body.visual(
        Box((0.142, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.077, 0.048)),
        material=enclosure_offwhite,
        name="rain_hood",
    )
    body.visual(
        Box((0.136, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.059, 0.047)),
        material=enclosure_offwhite,
        name="drip_lip",
    )

    perimeter_gasket_geom = section_loft(
        [
            _rounded_rect_section(0.138, 0.174, 0.019, 0.006),
            _rounded_rect_section(0.134, 0.170, 0.020, 0.008),
        ]
    )
    body.visual(
        _mesh("thermostat_perimeter_gasket", perimeter_gasket_geom),
        material=gasket_black,
        name="perimeter_gasket",
    )

    bezel_geom = LatheGeometry.from_shell_profiles(
        [
            (0.0415, -0.0020),
            (0.0445, 0.0000),
            (0.0490, 0.0035),
            (0.0515, 0.0085),
            (0.0515, 0.0130),
            (0.0485, 0.0150),
        ],
        [
            (0.0410, -0.0020),
            (0.0425, 0.0010),
            (0.0445, 0.0040),
            (0.0445, 0.0115),
            (0.0420, 0.0150),
        ],
        segments=56,
    )
    body.visual(
        _mesh("thermostat_bezel_ring", bezel_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=enclosure_offwhite,
        name="bezel_ring",
    )

    body.visual(
        Cylinder(radius=0.0102, length=0.013),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=stainless,
        name="center_shaft",
    )
    body.visual(
        Cylinder(radius=0.0130, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0605)),
        material=stainless,
        name="center_retainer",
    )

    for suffix, y_pos in (("upper", 0.056), ("lower", -0.056)):
        body.visual(
            Cylinder(radius=0.0035, length=0.028),
            origin=Origin(xyz=(0.0, y_pos, 0.020)),
            material=stainless,
            name=f"mount_shank_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.0068, length=0.003),
            origin=Origin(xyz=(0.0, y_pos, 0.0355)),
            material=stainless,
            name=f"mount_head_{suffix}",
        )

    dial = model.part("dial")
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.015),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
    )

    dial_geom = LatheGeometry.from_shell_profiles(
        [
            (0.0320, 0.0000),
            (0.0345, 0.0020),
            (0.0360, 0.0060),
            (0.0360, 0.0100),
            (0.0340, 0.0130),
            (0.0310, 0.0150),
        ],
        [
            (0.0108, 0.0000),
            (0.0108, 0.0098),
            (0.0134, 0.0124),
            (0.0134, 0.0150),
        ],
        segments=64,
    )
    dial.visual(
        _mesh("thermostat_dial_shell", dial_geom),
        material=dark_polymer,
        name="dial_shell",
    )
    dial.visual(
        Box((0.006, 0.016, 0.0024)),
        origin=Origin(xyz=(0.0, 0.027, 0.0150)),
        material=indicator_white,
        name="dial_indicator",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-2.35,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
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

    limits = dial_joint.motion_limits
    ctx.check("body_present", body is not None, "Expected thermostat body part.")
    ctx.check("dial_present", dial is not None, "Expected rotating thermostat dial part.")
    ctx.check(
        "dial_axis_is_wall_normal",
        tuple(round(v, 6) for v in dial_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected dial axis along +Z, got {dial_joint.axis!r}.",
    )
    ctx.check(
        "dial_has_realistic_rotation_limits",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
        "Dial should have bounded rotary travel around the wall-normal axis.",
    )
    ctx.expect_origin_distance(dial, body, axes="xy", max_dist=0.0005, name="dial_is_centered_on_body")

    with ctx.pose({dial_joint: 0.0}):
        ctx.expect_overlap(
            dial,
            body,
            axes="xy",
            min_overlap=0.060,
            name="dial_projects_within_body_footprint",
        )
        ctx.expect_within(
            dial,
            body,
            axes="xy",
            outer_elem="bezel_ring",
            margin=0.0025,
            name="dial_stays_inside_protective_bezel",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="z",
            positive_elem="dial_shell",
            negative_elem="front_housing",
            min_gap=0.0,
            max_gap=0.003,
            name="dial_seats_proud_of_front_housing",
        )
        ctx.expect_overlap(
            dial,
            body,
            elem_a="dial_shell",
            elem_b="center_shaft",
            axes="xy",
            min_overlap=0.018,
            name="dial_is_coaxial_with_center_shaft",
        )
        ctx.expect_overlap(
            dial,
            body,
            elem_a="dial_shell",
            elem_b="center_retainer",
            axes="xy",
            min_overlap=0.024,
            name="dial_is_coaxial_with_retainer",
        )

    with ctx.pose({dial_joint: math.radians(110.0)}):
        ctx.expect_origin_distance(
            dial,
            body,
            axes="xy",
            max_dist=0.0005,
            name="dial_remains_centered_when_rotated",
        )
        ctx.expect_within(
            dial,
            body,
            axes="xy",
            outer_elem="bezel_ring",
            margin=0.0025,
            name="rotated_dial_stays_inside_bezel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
