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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * cos((2.0 * pi * index) / segments),
            radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_face_bolts(
    part,
    *,
    prefix: str,
    x: float,
    y_positions: tuple[float, float],
    z_positions: tuple[float, float],
    radius: float,
    length: float,
    material,
) -> None:
    for yi, y in enumerate(y_positions):
        for zi, z in enumerate(z_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=material,
                name=f"{prefix}_bolt_{yi}_{zi}",
            )


def _add_ring_bolts(
    part,
    *,
    prefix: str,
    x: float,
    bolt_circle_radius: float,
    bolt_radius: float,
    bolt_length: float,
    count: int,
    material,
) -> None:
    for index in range(count):
        angle = (2.0 * pi * index) / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_length),
            origin=Origin(
                xyz=(
                    x,
                    bolt_circle_radius * sin(angle),
                    bolt_circle_radius * cos(angle),
                ),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_bolt_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_undershot_waterwheel")

    timber = model.material("timber", rgba=(0.43, 0.30, 0.18, 1.0))
    aged_timber = model.material("aged_timber", rgba=(0.54, 0.38, 0.22, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.20, 0.24, 0.23, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.70, 0.72, 1.0))
    hatch_red = model.material("hatch_red", rgba=(0.47, 0.16, 0.10, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    bearing_housing_mesh = _save_mesh(
        "bearing_housing",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.30, 0.36, 0.055, corner_segments=8),
            [_circle_profile(0.074, segments=40)],
            height=0.14,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    bearing_cartridge_mesh = _save_mesh(
        "bearing_cartridge",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.22, 0.22, 0.030, corner_segments=6),
            [_circle_profile(0.072, segments=40)],
            height=0.035,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    rim_mesh = _save_mesh(
        "wheel_rim",
        TorusGeometry(radius=0.94, tube=0.045, radial_segments=18, tubular_segments=84).rotate_y(pi / 2.0),
    )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.62, 1.96, 2.42)),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
    )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        frame.visual(
            Box((0.18, 1.96, 0.18)),
            origin=Origin(xyz=(side_sign * 0.72, 0.0, -1.18)),
            material=aged_timber,
            name=f"{side_name}_base_sill",
        )
        for fore_sign, fore_name in ((-1.0, "rear"), (1.0, "front")):
            frame.visual(
                Box((0.16, 0.18, 1.48)),
                origin=Origin(xyz=(side_sign * 0.62, fore_sign * 0.46, -0.44)),
                material=timber,
                name=f"{side_name}_{fore_name}_post",
            )
            frame.visual(
                Box((0.03, 0.24, 0.24)),
                origin=Origin(xyz=(side_sign * 0.54, fore_sign * 0.46, -0.18)),
                material=galvanized,
                name=f"{side_name}_{fore_name}_gusset",
            )
            brace_angle = -fore_sign * 0.86
            frame.visual(
                Box((0.08, 0.14, 0.72)),
                origin=Origin(xyz=(side_sign * 0.62, fore_sign * 0.25, -0.44), rpy=(brace_angle, 0.0, 0.0)),
                material=painted_steel,
                name=f"{side_name}_{fore_name}_knee_brace",
            )

        frame.visual(
            Box((0.18, 1.12, 0.16)),
            origin=Origin(xyz=(side_sign * 0.62, 0.0, 0.26)),
            material=timber,
            name=f"{side_name}_cap_beam",
        )
        frame.visual(
            Box((0.26, 0.34, 0.30)),
            origin=Origin(xyz=(side_sign * 0.62, 0.0, -0.31)),
            material=painted_steel,
            name=f"{side_name}_pedestal",
        )
        frame.visual(
            Box((0.22, 0.28, 0.05)),
            origin=Origin(xyz=(side_sign * 0.62, 0.0, -0.135)),
            material=galvanized,
            name=f"{side_name}_adapter_block",
        )
        frame.visual(
            bearing_housing_mesh,
            origin=Origin(xyz=(side_sign * 0.62, 0.0, 0.0)),
            material=painted_steel,
            name=f"{side_name}_bearing_housing",
        )
        frame.visual(
            bearing_cartridge_mesh,
            origin=Origin(xyz=(side_sign * 0.5325, 0.0, 0.0)),
            material=galvanized,
            name=f"{side_name}_bearing_cartridge",
        )
        frame.visual(
            Box((0.02, 0.22, 0.24)),
            origin=Origin(xyz=(side_sign * 0.70, 0.0, 0.0)),
            material=hatch_red,
            name=f"{side_name}_service_hatch",
        )
        _add_face_bolts(
            frame,
            prefix=f"{side_name}_service_hatch",
            x=side_sign * 0.711,
            y_positions=(-0.075, 0.075),
            z_positions=(-0.08, 0.08),
            radius=0.012,
            length=0.018,
            material=bolt_steel,
        )

    frame.visual(
        Box((1.62, 0.18, 0.22)),
        origin=Origin(xyz=(0.0, 0.74, -1.14)),
        material=aged_timber,
        name="front_tie_beam",
    )
    frame.visual(
        Box((1.62, 0.18, 0.22)),
        origin=Origin(xyz=(0.0, -0.74, -1.14)),
        material=aged_timber,
        name="rear_tie_beam",
    )
    frame.visual(
        Box((1.54, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -1.18)),
        material=timber,
        name="center_sill_tie",
    )
    frame.visual(
        Box((1.28, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.34, -0.58)),
        material=painted_steel,
        name="front_mid_tie_rod",
    )
    frame.visual(
        Box((1.28, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.34, -0.58)),
        material=painted_steel,
        name="rear_mid_tie_rod",
    )
    frame.visual(
        Box((1.22, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
        material=painted_steel,
        name="pedestal_tie_rod",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.03, length=0.92),
        mass=980.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.065, length=1.24),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.098, length=0.38),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="hub_barrel",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        wheel.visual(
            Cylinder(radius=0.24, length=0.06),
            origin=Origin(xyz=(side_sign * 0.33, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"{side_name}_adapter_flange",
        )
        wheel.visual(
            Cylinder(radius=0.08, length=0.03),
            origin=Origin(xyz=(side_sign * 0.50, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"{side_name}_seal_sleeve",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(side_sign * 0.42, 0.0, 0.0)),
            material=painted_steel,
            name=f"{side_name}_rim",
        )
        _add_ring_bolts(
            wheel,
            prefix=f"{side_name}_adapter_flange",
            x=side_sign * 0.355,
            bolt_circle_radius=0.155,
            bolt_radius=0.011,
            bolt_length=0.020,
            count=8,
            material=bolt_steel,
        )

    spoke_count = 8
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        for index in range(spoke_count):
            angle = (2.0 * pi * index) / spoke_count + (pi / spoke_count)
            wheel.visual(
                Box((0.08, 0.06, 0.76)),
                origin=Origin(
                    xyz=(side_sign * 0.39, 0.55 * sin(angle), 0.55 * cos(angle)),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=galvanized,
                name=f"{side_name}_spoke_{index:02d}",
            )

    paddle_count = 12
    for index in range(paddle_count):
        angle = (2.0 * pi * index) / paddle_count
        wheel.visual(
            Box((0.92, 0.032, 0.18)),
            origin=Origin(
                xyz=(0.0, 0.94 * sin(angle), 0.94 * cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=timber,
            name=f"paddle_{index:02d}",
        )
        for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
            wheel.visual(
                Box((0.10, 0.05, 0.20)),
                origin=Origin(
                    xyz=(side_sign * 0.38, 0.94 * sin(angle), 0.94 * cos(angle)),
                    rpy=(-angle, 0.0, 0.0),
                ),
                material=galvanized,
                name=f"{side_name}_paddle_strap_{index:02d}",
            )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

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
        "wheel_joint_is_continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        f"joint type was {wheel_spin.articulation_type}",
    )
    ctx.check(
        "wheel_joint_axis_runs_along_axle",
        tuple(round(value, 3) for value in wheel_spin.axis) == (1.0, 0.0, 0.0),
        f"axis was {wheel_spin.axis}",
    )
    ctx.expect_origin_distance(
        wheel,
        frame,
        axes="yz",
        min_dist=0.0,
        max_dist=0.001,
        name="wheel_centered_between_side_frames",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="yz",
        min_overlap=0.12,
        elem_a="axle_shaft",
        elem_b="left_bearing_housing",
        name="left_bearing_envelopes_axle",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="yz",
        min_overlap=0.12,
        elem_a="axle_shaft",
        elem_b="right_bearing_housing",
        name="right_bearing_envelopes_axle",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        min_gap=0.05,
        max_gap=0.12,
        positive_elem="left_rim",
        negative_elem="left_bearing_housing",
        name="left_rim_clears_left_support",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        min_gap=0.05,
        max_gap=0.12,
        positive_elem="right_bearing_housing",
        negative_elem="right_rim",
        name="right_rim_clears_right_support",
    )
    ctx.expect_contact(
        frame,
        frame,
        elem_a="left_service_hatch",
        elem_b="left_bearing_housing",
        name="left_service_hatch_is_mounted",
    )
    ctx.expect_contact(
        frame,
        frame,
        elem_a="right_service_hatch",
        elem_b="right_bearing_housing",
        name="right_service_hatch_is_mounted",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="left_bearing_cartridge",
        elem_b="left_seal_sleeve",
        name="left_bearing_support_contacts_wheel",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="right_bearing_cartridge",
        elem_b="right_seal_sleeve",
        name="right_bearing_support_contacts_wheel",
    )
    ctx.expect_contact(
        wheel,
        wheel,
        elem_a="left_adapter_flange",
        elem_b="axle_shaft",
        name="left_adapter_flange_is_supported",
    )
    ctx.expect_contact(
        wheel,
        wheel,
        elem_a="right_adapter_flange",
        elem_b="axle_shaft",
        name="right_adapter_flange_is_supported",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
