from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _wheel_point(x: float, radius: float, angle: float) -> tuple[float, float, float]:
    return (x, radius * cos(angle), radius * sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")

    frame_paint = model.material("frame_paint", rgba=(0.26, 0.31, 0.24, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.53, 0.55, 0.57, 1.0))
    bronze = model.material("bronze", rgba=(0.56, 0.42, 0.23, 1.0))
    wear_oak = model.material("wear_oak", rgba=(0.60, 0.50, 0.32, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.95, 2.55, 1.45)),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )

    frame.visual(
        Box((0.18, 2.45, 0.14)),
        origin=Origin(xyz=(-0.82, 0.0, 0.07)),
        material=frame_paint,
        name="left_skid",
    )
    frame.visual(
        Box((0.18, 2.45, 0.14)),
        origin=Origin(xyz=(0.82, 0.0, 0.07)),
        material=frame_paint,
        name="right_skid",
    )
    frame.visual(
        Box((1.46, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, -0.78, 0.12)),
        material=frame_paint,
        name="front_cross_sill",
    )
    frame.visual(
        Box((1.46, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, 0.78, 0.12)),
        material=frame_paint,
        name="rear_cross_sill",
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side < 0.0 else "right"
        x_tower = side * 0.74
        x_saddle = side * 0.63

        front_base = (x_tower, -0.88, 0.14)
        front_top = (x_tower, -0.24, 0.82)
        rear_base = (x_tower, 0.88, 0.14)
        rear_top = (x_tower, 0.24, 0.82)
        top_center = (x_tower, 0.0, 0.82)
        pedestal_top = (x_tower, 0.0, 0.94)
        saddle_center = (x_saddle, 0.0, 0.93)

        _add_box_member(
            frame,
            front_base,
            front_top,
            width=0.10,
            depth=0.14,
            material=frame_paint,
            name=f"{side_name}_front_leg",
        )
        _add_box_member(
            frame,
            rear_base,
            rear_top,
            width=0.10,
            depth=0.14,
            material=frame_paint,
            name=f"{side_name}_rear_leg",
        )
        _add_box_member(
            frame,
            front_top,
            rear_top,
            width=0.10,
            depth=0.16,
            material=frame_paint,
            name=f"{side_name}_top_tie",
        )
        _add_box_member(
            frame,
            top_center,
            pedestal_top,
            width=0.10,
            depth=0.12,
            material=frame_paint,
            name=f"{side_name}_pedestal_post",
        )
        _add_box_member(
            frame,
            (x_tower, -0.50, 0.34),
            top_center,
            width=0.07,
            depth=0.10,
            material=steel_dark,
        )
        _add_box_member(
            frame,
            (x_tower, 0.50, 0.34),
            top_center,
            width=0.07,
            depth=0.10,
            material=steel_dark,
        )
        frame.visual(
            Box((0.11, 0.18, 0.09)),
            origin=Origin(xyz=(side * 0.685, 0.0, 0.88)),
            material=frame_paint,
            name=f"{side_name}_bearing_arm",
        )

        frame.visual(
            Box((0.06, 1.20, 0.06)),
            origin=Origin(xyz=(x_tower, 0.0, 0.52)),
            material=frame_paint,
            name=f"{side_name}_service_rail",
        )
        frame.visual(
            Box((0.10, 0.22, 0.03)),
            origin=Origin(xyz=saddle_center),
            material=bronze,
            name=f"{side_name}_bearing_saddle",
        )
        frame.visual(
            Box((0.10, 0.06, 0.19)),
            origin=Origin(xyz=(x_saddle, -0.09, 1.005)),
            material=frame_paint,
            name=f"{side_name}_bearing_cheek_front",
        )
        frame.visual(
            Box((0.10, 0.06, 0.19)),
            origin=Origin(xyz=(x_saddle, 0.09, 1.005)),
            material=frame_paint,
            name=f"{side_name}_bearing_cheek_rear",
        )
        frame.visual(
            Box((0.12, 0.24, 0.03)),
            origin=Origin(xyz=(x_saddle + side * 0.015, 0.0, 1.10)),
            material=frame_paint,
            name=f"{side_name}_bearing_cap",
        )
        frame.visual(
            Box((0.12, 0.08, 0.12)),
            origin=Origin(xyz=(side * 0.69, 0.0, 0.88)),
            material=steel_dark,
            name=f"{side_name}_pedestal_gusset",
        )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.76, length=1.44),
        mass=420.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    rotor.visual(
        Cylinder(radius=0.055, length=1.44),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="axle",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(-0.44, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="left_hub",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.10),
        origin=Origin(xyz=(0.44, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_dark,
        name="right_hub",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.06),
        origin=Origin(xyz=(-0.55, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mid,
        name="left_collar",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.06),
        origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mid,
        name="right_collar",
    )

    paddle_count = 8
    inner_radius = 0.09
    spoke_radius = 0.60
    rim_radius = 0.64
    carrier_radius = 0.61
    wear_radius = 0.70
    wheel_half_width = 0.44

    for x_side in (-wheel_half_width, wheel_half_width):
        outer_points = [
            _wheel_point(x_side, rim_radius, 2.0 * pi * index / paddle_count)
            for index in range(paddle_count)
        ]
        for index, outer_point in enumerate(outer_points):
            angle = 2.0 * pi * index / paddle_count
            _add_box_member(
                rotor,
                _wheel_point(x_side, inner_radius, angle),
                _wheel_point(x_side, spoke_radius, angle),
                width=0.05,
                depth=0.08,
                material=steel_dark,
            )
            _add_box_member(
                rotor,
                outer_point,
                outer_points[(index + 1) % paddle_count],
                width=0.06,
                depth=0.08,
                material=steel_mid,
            )

    for index in range(paddle_count):
        angle = 2.0 * pi * index / paddle_count
        roll = angle - pi / 2.0
        y_dir = cos(angle)
        z_dir = sin(angle)

        rotor.visual(
            Box((0.88, 0.08, 0.09)),
            origin=Origin(
                xyz=(0.0, carrier_radius * y_dir, carrier_radius * z_dir),
                rpy=(roll, 0.0, 0.0),
            ),
            material=steel_mid,
            name=f"carrier_{index}",
        )
        rotor.visual(
            Box((0.92, 0.045, 0.18)),
            origin=Origin(
                xyz=(0.0, wear_radius * y_dir, wear_radius * z_dir),
                rpy=(roll, 0.0, 0.0),
            ),
            material=wear_oak,
            name=f"wear_paddle_{index}",
        )
        rotor.visual(
            Box((0.92, 0.015, 0.04)),
            origin=Origin(
                xyz=(0.0, (wear_radius - 0.05) * y_dir, (wear_radius - 0.05) * z_dir),
                rpy=(roll, 0.0, 0.0),
            ),
            material=steel_dark,
        )
        rotor.visual(
            Box((0.92, 0.015, 0.04)),
            origin=Origin(
                xyz=(0.0, (wear_radius + 0.05) * y_dir, (wear_radius + 0.05) * z_dir),
                rpy=(roll, 0.0, 0.0),
            ),
            material=steel_dark,
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1600.0, velocity=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    spin_origin = spin.origin.xyz
    axis = spin.axis
    limits = spin.motion_limits
    ctx.check(
        "wheel_spin_runs_on_centered_axle",
        abs(axis[0] - 1.0) < 1e-9
        and abs(axis[1]) < 1e-9
        and abs(axis[2]) < 1e-9
        and abs(spin_origin[0]) < 1e-9
        and abs(spin_origin[1]) < 1e-9
        and abs(spin_origin[2] - 1.0) < 1e-9,
        details=(
            f"axis={axis}, origin={spin_origin}; expected centered axle at "
            "(0.0, 0.0, 1.0) spinning about +X"
        ),
    )
    ctx.check(
        "wheel_spin_is_continuous",
        limits is not None and limits.lower is None and limits.upper is None,
        details="Waterwheel should spin continuously around the axle without hard stops.",
    )

    ctx.expect_contact(
        rotor,
        frame,
        elem_a="axle",
        elem_b="left_bearing_saddle",
        contact_tol=0.0015,
        name="left_saddle_supports_axle",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="axle",
        elem_b="right_bearing_saddle",
        contact_tol=0.0015,
        name="right_saddle_supports_axle",
    )

    with ctx.pose({spin: pi / 4.0}):
        ctx.expect_contact(
            rotor,
            frame,
            elem_a="axle",
            elem_b="left_bearing_saddle",
            contact_tol=0.0015,
            name="left_saddle_supports_axle_quarter_turn",
        )
        ctx.expect_contact(
            rotor,
            frame,
            elem_a="axle",
            elem_b="right_bearing_saddle",
            contact_tol=0.0015,
            name="right_saddle_supports_axle_quarter_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
