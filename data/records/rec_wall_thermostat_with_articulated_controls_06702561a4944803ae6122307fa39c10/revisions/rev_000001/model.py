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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(width: float, height: float, corner_radius: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            height,
            corner_radius,
            corner_segments=8,
        )
    ]


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / segments),
            radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wall_thermostat")

    painted_metal = model.material("painted_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    machined_metal = model.material("machined_metal", rgba=(0.60, 0.63, 0.67, 1.0))
    polymer_shell = model.material("polymer_shell", rgba=(0.93, 0.94, 0.95, 1.0))
    polymer_bezel = model.material("polymer_bezel", rgba=(0.86, 0.88, 0.90, 1.0))
    elastomer_dark = model.material("elastomer_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    smoked_lens = model.material("smoked_lens", rgba=(0.18, 0.24, 0.27, 0.62))

    body = model.part("body")
    body.visual(
        Box((0.090, 0.090, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=painted_metal,
        name="wall_plate",
    )

    housing_shell = section_loft(
        [
            _xy_section(0.078, 0.078, 0.015, 0.0015),
            _xy_section(0.080, 0.080, 0.017, 0.0065),
            _xy_section(0.076, 0.076, 0.015, 0.0110),
        ]
    )
    body.visual(
        _save_mesh("thermostat_housing_shell", housing_shell),
        material=polymer_shell,
        name="housing_shell",
    )

    front_frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.072, 0.072, 0.012, corner_segments=8),
        [_circle_profile(0.0310)],
        height=0.0025,
        center=True,
    )
    body.visual(
        _save_mesh("thermostat_front_frame", front_frame),
        origin=Origin(xyz=(0.0, 0.0, 0.01225)),
        material=polymer_bezel,
        name="front_frame",
    )

    display_lens = ExtrudeGeometry(
        rounded_rect_profile(0.020, 0.0065, 0.0020, corner_segments=6),
        0.0016,
        center=True,
    )
    body.visual(
        _save_mesh("thermostat_display_lens", display_lens),
        origin=Origin(xyz=(0.0, 0.020, 0.0143)),
        material=smoked_lens,
        name="display_lens",
    )
    body.visual(
        Box((0.014, 0.004, 0.0025)),
        origin=Origin(xyz=(0.0, 0.020, 0.01225)),
        material=elastomer_dark,
        name="display_mount",
    )

    body.visual(
        Cylinder(radius=0.0278, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0119)),
        material=polymer_bezel,
        name="dial_seat",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0115),
        origin=Origin(xyz=(0.0, 0.0, 0.01855)),
        material=machined_metal,
        name="shaft_post",
    )

    retainer_profile = [
        (0.0, 0.0),
        (0.0065, 0.0),
        (0.0090, 0.00035),
        (0.0102, 0.0010),
        (0.0102, 0.0016),
        (0.0, 0.0016),
    ]
    body.visual(
        _save_mesh("thermostat_retainer_cap", LatheGeometry(retainer_profile, segments=64)),
        origin=Origin(xyz=(0.0, 0.0, 0.0243)),
        material=machined_metal,
        name="retainer_cap",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.026)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    dial = model.part("dial")
    dial_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0282, 0.0000),
            (0.0292, 0.0018),
            (0.0300, 0.0058),
            (0.0296, 0.0098),
            (0.0286, 0.0113),
        ],
        [
            (0.0108, 0.0000),
            (0.0108, 0.0108),
            (0.0106, 0.0113),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    dial.visual(
        _save_mesh("thermostat_dial_shell", dial_shell),
        material=painted_metal,
        name="dial_shell",
    )

    dial_grip = LatheGeometry.from_shell_profiles(
        [
            (0.0310, 0.0022),
            (0.0312, 0.0060),
            (0.0310, 0.0094),
        ],
        [
            (0.0290, 0.0022),
            (0.0291, 0.0094),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    dial.visual(
        _save_mesh("thermostat_dial_grip", dial_grip),
        material=elastomer_dark,
        name="dial_grip",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0312, length=0.0113),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.00565)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.0128)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")
    dial_shell = dial.get_visual("dial_shell")
    dial_seat = body.get_visual("dial_seat")
    retainer_cap = body.get_visual("retainer_cap")

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
        "dial articulation axis is front-facing",
        tuple(round(value, 6) for value in dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={dial_joint.axis}",
    )
    ctx.expect_origin_distance(
        dial,
        body,
        axes="xy",
        max_dist=0.0005,
        name="dial is centered on the thermostat body",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a=dial_shell,
        elem_b=dial_seat,
        name="dial shell is supported by the seat ring",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="z",
        positive_elem=retainer_cap,
        negative_elem=dial_shell,
        min_gap=0.00005,
        max_gap=0.0015,
        name="retainer cap captures dial with slight axial clearance",
    )

    with ctx.pose({dial_joint: 1.7}):
        ctx.expect_contact(
            dial,
            body,
            elem_a=dial_shell,
            elem_b=dial_seat,
            name="dial remains seated while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
