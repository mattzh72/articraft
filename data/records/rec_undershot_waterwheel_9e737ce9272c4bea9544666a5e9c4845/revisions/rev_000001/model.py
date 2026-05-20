from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sin, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _yz_brace_geometry(
    *,
    x: float,
    y0: float,
    z0: float,
    y1: float,
    z1: float,
    thickness_x: float,
    thickness_z: float,
) -> tuple[Box, Origin]:
    dy = y1 - y0
    dz = z1 - z0
    length = sqrt(dy * dy + dz * dz)
    center_y = (y0 + y1) * 0.5
    center_z = (z0 + z1) * 0.5
    angle = atan2(dz, dy)
    return (
        Box((thickness_x, length, thickness_z)),
        Origin(xyz=(x, center_y, center_z), rpy=(angle, 0.0, 0.0)),
    )


def _bearing_plate_mesh(*, outer_z: float, outer_y: float, hole_radius: float, thickness: float):
    plate_profile = rounded_rect_profile(outer_z, outer_y, radius=0.022, corner_segments=8)
    hole_profile = superellipse_profile(hole_radius * 2.0, hole_radius * 2.0, exponent=2.0, segments=36)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            plate_profile,
            [hole_profile],
            height=thickness,
            center=True,
        ).rotate_y(pi / 2.0),
        "bearing_plate",
    )


def _rim_mesh(*, radius: float, tube_radius: float):
    return mesh_from_geometry(
        TorusGeometry(radius, tube_radius, radial_segments=18, tubular_segments=56).rotate_y(pi / 2.0),
        "rolled_rim",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    galvanized = model.material("galvanized", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    composite_paddle = model.material("composite_paddle", rgba=(0.33, 0.37, 0.40, 1.0))

    axle_z = 1.20
    wheel_radius = 1.05
    rim_center_x = 0.340
    rim_radius = 0.920
    rim_tube_radius = 0.030
    axle_radius = 0.055
    collar_radius = 0.095
    plate_thickness = 0.028
    side_plate_x = 0.416

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.40, 2.30, 1.62)),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    skid_size = (0.140, 2.300, 0.120)
    for skid_x in (-0.620, 0.620):
        frame.visual(
            Box(skid_size),
            origin=Origin(xyz=(skid_x, 0.0, skid_size[2] * 0.5)),
            material=galvanized,
        )

    for tie_y in (-1.090, 1.090):
        frame.visual(
            Box((1.100, 0.120, 0.120)),
            origin=Origin(xyz=(0.0, tie_y, 0.180)),
            material=galvanized,
        )

    post_x = 0.520
    post_y = 0.520
    post_size = (0.100, 0.120, 1.420)
    for side_x in (-post_x, post_x):
        for y_sign in (-1.0, 1.0):
            frame.visual(
                Box(post_size),
                origin=Origin(xyz=(side_x, y_sign * post_y, 0.770)),
                material=galvanized,
            )

    side_rail_x = 0.470
    for side_x in (-side_rail_x, side_rail_x):
        for rail_z in (0.260, axle_z - 0.190, axle_z + 0.190, 1.500):
            frame.visual(
                Box((0.080, 1.040, 0.080)),
                origin=Origin(xyz=(side_x, 0.0, rail_z)),
                material=galvanized,
            )

        brace_geom, brace_origin = _yz_brace_geometry(
            x=side_x,
            y0=0.820,
            z0=0.180,
            y1=0.520,
            z1=0.720,
            thickness_x=0.080,
            thickness_z=0.080,
        )
        frame.visual(brace_geom, origin=brace_origin, material=galvanized)
        brace_geom, brace_origin = _yz_brace_geometry(
            x=side_x,
            y0=-0.820,
            z0=0.180,
            y1=-0.520,
            z1=0.720,
            thickness_x=0.080,
            thickness_z=0.080,
        )
        frame.visual(brace_geom, origin=brace_origin, material=galvanized)

    bearing_plate = _bearing_plate_mesh(outer_z=0.340, outer_y=0.240, hole_radius=0.066, thickness=plate_thickness)
    frame.visual(
        bearing_plate,
        origin=Origin(xyz=(-side_plate_x, 0.0, axle_z)),
        material=dark_steel,
        name="left_bearing_plate",
    )
    frame.visual(
        bearing_plate,
        origin=Origin(xyz=(side_plate_x, 0.0, axle_z)),
        material=dark_steel,
        name="right_bearing_plate",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=1.120),
        mass=165.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    wheel.visual(
        Cylinder(radius=axle_radius, length=1.120),
        origin=spin_origin,
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.112, length=0.720),
        origin=spin_origin,
        material=galvanized,
        name="hub_tube",
    )

    for hub_x in (-rim_center_x, rim_center_x):
        wheel.visual(
            Cylinder(radius=0.188, length=0.024),
            origin=Origin(xyz=(hub_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
        )

    wheel.visual(
        Cylinder(radius=collar_radius, length=0.056),
        origin=Origin(xyz=(-0.458, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_collar",
    )
    wheel.visual(
        Cylinder(radius=collar_radius, length=0.056),
        origin=Origin(xyz=(0.458, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_collar",
    )

    for stub_x in (-0.548, 0.548):
        wheel.visual(
            Cylinder(radius=0.074, length=0.018),
            origin=Origin(xyz=(stub_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
        )

    rim_mesh = _rim_mesh(radius=rim_radius, tube_radius=rim_tube_radius)
    wheel.visual(rim_mesh, origin=Origin(xyz=(-rim_center_x, 0.0, 0.0)), material=galvanized, name="left_rim")
    wheel.visual(rim_mesh, origin=Origin(xyz=(rim_center_x, 0.0, 0.0)), material=galvanized, name="right_rim")

    spoke_inner_radius = 0.176
    spoke_length = 0.790
    spoke_center_radius = spoke_inner_radius + spoke_length * 0.5
    paddle_radius = 0.960
    paddle_span = 0.760
    paddle_chord = 0.120
    paddle_depth = 0.180
    strap_thickness = 0.040

    for index in range(8):
        angle = index * (pi / 4.0)
        y = sin(angle) * paddle_radius
        z = sin(angle + pi / 2.0) * paddle_radius

        wheel.visual(
            Box((paddle_span, paddle_chord, paddle_depth)),
            origin=Origin(xyz=(0.0, y, z), rpy=(angle, 0.0, 0.0)),
            material=composite_paddle,
            name=f"paddle_{index}",
        )

        for strap_x in (-rim_center_x, rim_center_x):
            wheel.visual(
                Box((strap_thickness, paddle_chord, 0.034)),
                origin=Origin(xyz=(strap_x, y, z), rpy=(angle, 0.0, 0.0)),
                material=dark_steel,
            )

        for spoke_x in (-rim_center_x, rim_center_x):
            wheel.visual(
                Cylinder(radius=0.024, length=spoke_length),
                origin=Origin(
                    xyz=(spoke_x, sin(angle) * spoke_center_radius, sin(angle + pi / 2.0) * spoke_center_radius),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=galvanized,
            )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("frame_to_wheel")

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
        "frame_present",
        frame.name == "frame",
        "Frame part is missing.",
    )
    ctx.check(
        "wheel_present",
        wheel.name == "wheel",
        "Wheel part is missing.",
    )
    ctx.check(
        "wheel_joint_axis",
        tuple(spin.axis) == (1.0, 0.0, 0.0),
        f"Wheel should spin around the axle on +X, got axis={spin.axis}.",
    )
    ctx.check(
        "wheel_joint_type",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        f"Wheel joint should be continuous, got {spin.articulation_type}.",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="left_collar",
        elem_b="left_bearing_plate",
        contact_tol=0.0005,
        name="left_bearing_contact",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="right_collar",
        elem_b="right_bearing_plate",
        contact_tol=0.0005,
        name="right_bearing_contact",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="left_rim",
        negative_elem="left_bearing_plate",
        min_gap=0.010,
        max_gap=0.050,
        name="left_rim_clear_of_bearing_plate",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="right_bearing_plate",
        negative_elem="right_rim",
        min_gap=0.010,
        max_gap=0.050,
        name="right_rim_clear_of_bearing_plate",
    )
    with ctx.pose({spin: pi / 4.0}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="left_collar",
            elem_b="left_bearing_plate",
            contact_tol=0.0005,
            name="left_bearing_contact_rotated",
        )
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="right_collar",
            elem_b="right_bearing_plate",
            contact_tol=0.0005,
            name="right_bearing_contact_rotated",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="wheel_clearance_in_rotation")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
