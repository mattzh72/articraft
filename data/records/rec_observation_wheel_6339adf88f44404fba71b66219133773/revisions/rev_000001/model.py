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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    def _polar_yz(radius: float, angle: float) -> tuple[float, float]:
        return (radius * math.sin(angle), radius * math.cos(angle))

    def _add_bar(
        part,
        *,
        name: str,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        width: float,
        depth: float,
        material,
    ) -> None:
        sx, sy, sz = start
        ex, ey, ez = end
        vx = ex - sx
        vy = ey - sy
        vz = ez - sz
        length = math.sqrt(vx * vx + vy * vy + vz * vz)
        yaw = math.atan2(vy, vx)
        pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
        part.visual(
            Box((width, depth, length)),
            origin=Origin(
                xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
                rpy=(0.0, pitch, yaw),
            ),
            material=material,
            name=name,
        )

    def _add_ring_segment(
        part,
        *,
        name: str,
        x: float,
        radius: float,
        angle: float,
        tangential: float,
        radial: float,
        thickness_x: float,
        material,
    ) -> None:
        y, z = _polar_yz(radius, angle)
        part.visual(
            Box((thickness_x, tangential, radial)),
            origin=Origin(xyz=(x, y, z), rpy=(angle, 0.0, 0.0)),
            material=material,
            name=name,
        )

    model = ArticulatedObject(name="observation_wheel")

    concrete = model.material("concrete", rgba=(0.72, 0.73, 0.74, 1.0))
    white_steel = model.material("white_steel", rgba=(0.93, 0.94, 0.95, 1.0))
    silver_steel = model.material("silver_steel", rgba=(0.70, 0.74, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    cabin_shell = model.material("cabin_shell", rgba=(0.92, 0.94, 0.97, 1.0))
    cabin_glass = model.material("cabin_glass", rgba=(0.52, 0.73, 0.86, 0.35))

    wheel_center_z = 11.0
    rim_radius = 8.0
    inner_ring_radius = 6.9
    cabin_pivot_radius = 6.2
    rim_x = 0.65
    inner_x = 0.45
    cabin_count = 10

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((10.0, 7.2, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        material=concrete,
        name="foundation",
    )
    base_frame.visual(
        Box((4.6, 1.9, 0.42)),
        origin=Origin(xyz=(0.0, -3.10, 1.01)),
        material=concrete,
        name="boarding_plinth",
    )
    base_frame.visual(
        Box((2.7, 1.25, 0.12)),
        origin=Origin(xyz=(0.0, -3.10, 1.28)),
        material=silver_steel,
        name="boarding_deck",
    )
    base_frame.visual(
        Box((0.30, 1.00, 1.00)),
        origin=Origin(xyz=(-1.02, 0.0, wheel_center_z)),
        material=dark_steel,
        name="left_bearing_housing",
    )
    base_frame.visual(
        Box((0.30, 1.00, 1.00)),
        origin=Origin(xyz=(1.02, 0.0, wheel_center_z)),
        material=dark_steel,
        name="right_bearing_housing",
    )
    for side_name, leg_x, housing_x in (("left", -1.80, -1.17), ("right", 1.80, 1.17)):
        _add_bar(
            base_frame,
            name=f"{side_name}_front_leg",
            start=(leg_x, -3.25, 0.8),
            end=(housing_x, -0.10, wheel_center_z),
            width=0.24,
            depth=0.20,
            material=white_steel,
        )
        _add_bar(
            base_frame,
            name=f"{side_name}_rear_leg",
            start=(leg_x, 3.25, 0.8),
            end=(housing_x, 0.10, wheel_center_z),
            width=0.24,
            depth=0.20,
            material=white_steel,
        )
        _add_bar(
            base_frame,
            name=f"{side_name}_mid_tie",
            start=(leg_x, -1.60, 6.00),
            end=(leg_x, 1.60, 6.00),
            width=0.12,
            depth=0.12,
            material=silver_steel,
        )
        _add_bar(
            base_frame,
            name=f"{side_name}_front_brace",
            start=(leg_x, -1.60, 6.00),
            end=(housing_x, -0.10, wheel_center_z),
            width=0.10,
            depth=0.10,
            material=silver_steel,
        )
        _add_bar(
            base_frame,
            name=f"{side_name}_rear_brace",
            start=(leg_x, 1.60, 6.00),
            end=(housing_x, 0.10, wheel_center_z),
            width=0.10,
            depth=0.10,
            material=silver_steel,
        )

    base_frame.inertial = Inertial.from_geometry(
        Box((10.0, 7.2, 11.8)),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, 5.9)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.55, length=1.08),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.95, length=0.08),
        origin=Origin(xyz=(-0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hub_disc",
    )
    wheel.visual(
        Cylinder(radius=0.95, length=0.08),
        origin=Origin(xyz=(0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hub_disc",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.25),
        origin=Origin(xyz=(-0.745, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.25),
        origin=Origin(xyz=(0.745, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_shaft",
    )

    rim_segments = 24
    rim_arc = (2.0 * math.pi * rim_radius / rim_segments) * 1.35
    for index in range(rim_segments):
        angle = 2.0 * math.pi * index / rim_segments
        _add_ring_segment(
            wheel,
            name=f"front_rim_seg_{index:02d}",
            x=rim_x,
            radius=rim_radius,
            angle=angle,
            tangential=rim_arc,
            radial=0.28,
            thickness_x=0.16,
            material=white_steel,
        )
        _add_ring_segment(
            wheel,
            name=f"rear_rim_seg_{index:02d}",
            x=-rim_x,
            radius=rim_radius,
            angle=angle,
            tangential=rim_arc,
            radial=0.28,
            thickness_x=0.16,
            material=white_steel,
        )

    inner_segments = 20
    inner_arc = (2.0 * math.pi * inner_ring_radius / inner_segments) * 1.35
    for index in range(inner_segments):
        angle = 2.0 * math.pi * index / inner_segments
        _add_ring_segment(
            wheel,
            name=f"front_inner_seg_{index:02d}",
            x=inner_x,
            radius=inner_ring_radius,
            angle=angle,
            tangential=inner_arc,
            radial=0.18,
            thickness_x=0.10,
            material=silver_steel,
        )
        _add_ring_segment(
            wheel,
            name=f"rear_inner_seg_{index:02d}",
            x=-inner_x,
            radius=inner_ring_radius,
            angle=angle,
            tangential=inner_arc,
            radial=0.18,
            thickness_x=0.10,
            material=silver_steel,
        )

    spoke_angles = [2.0 * math.pi * index / 12.0 for index in range(12)]
    for index, angle in enumerate(spoke_angles):
        start_y, start_z = _polar_yz(0.95, angle)
        end_y, end_z = _polar_yz(rim_radius - 0.12, angle)
        _add_bar(
            wheel,
            name=f"front_spoke_{index:02d}",
            start=(0.58, start_y, start_z),
            end=(rim_x, end_y, end_z),
            width=0.10,
            depth=0.10,
            material=silver_steel,
        )
        _add_bar(
            wheel,
            name=f"rear_spoke_{index:02d}",
            start=(-0.58, start_y, start_z),
            end=(-rim_x, end_y, end_z),
            width=0.10,
            depth=0.10,
            material=silver_steel,
        )

    for index in range(cabin_count):
        angle = 2.0 * math.pi * index / cabin_count
        inner_y, inner_z = _polar_yz(inner_ring_radius - 0.09, angle)
        pivot_y, pivot_z = _polar_yz(cabin_pivot_radius, angle)
        _add_bar(
            wheel,
            name=f"front_inner_spoke_{index:02d}",
            start=(0.58, *_polar_yz(0.95, angle)),
            end=(inner_x, inner_y, inner_z),
            width=0.08,
            depth=0.08,
            material=silver_steel,
        )
        _add_bar(
            wheel,
            name=f"rear_inner_spoke_{index:02d}",
            start=(-0.58, *_polar_yz(0.95, angle)),
            end=(-inner_x, inner_y, inner_z),
            width=0.08,
            depth=0.08,
            material=silver_steel,
        )
        _add_bar(
            wheel,
            name=f"front_radial_bracket_{index:02d}",
            start=(inner_x, inner_y, inner_z),
            end=(inner_x, pivot_y, pivot_z),
            width=0.08,
            depth=0.08,
            material=dark_steel,
        )
        _add_bar(
            wheel,
            name=f"rear_radial_bracket_{index:02d}",
            start=(-inner_x, inner_y, inner_z),
            end=(-inner_x, pivot_y, pivot_z),
            width=0.08,
            depth=0.08,
            material=dark_steel,
        )
        _add_bar(
            wheel,
            name=f"front_pivot_link_{index:02d}",
            start=(inner_x, pivot_y, pivot_z),
            end=(0.18, pivot_y, pivot_z),
            width=0.08,
            depth=0.08,
            material=dark_steel,
        )
        _add_bar(
            wheel,
            name=f"rear_pivot_link_{index:02d}",
            start=(-inner_x, pivot_y, pivot_z),
            end=(-0.18, pivot_y, pivot_z),
            width=0.08,
            depth=0.08,
            material=dark_steel,
        )
        wheel.visual(
            Box((0.08, 0.12, 0.12)),
            origin=Origin(xyz=(0.18, pivot_y, pivot_z)),
            material=dark_steel,
            name=f"right_clamp_{index:02d}",
        )
        wheel.visual(
            Box((0.08, 0.12, 0.12)),
            origin=Origin(xyz=(-0.18, pivot_y, pivot_z)),
            material=dark_steel,
            name=f"left_clamp_{index:02d}",
        )

    wheel.inertial = Inertial.from_geometry(
        Box((1.60, 16.4, 16.4)),
        mass=8200.0,
        origin=Origin(),
    )

    for index in range(cabin_count):
        angle = 2.0 * math.pi * index / cabin_count
        cabin = model.part(f"cabin_{index:02d}")
        cabin.visual(
            Cylinder(radius=0.05, length=0.28),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="top_axle",
        )
        cabin.visual(
            Box((0.06, 0.08, 0.64)),
            origin=Origin(xyz=(-0.08, 0.0, -0.36)),
            material=dark_steel,
            name="left_hanger",
        )
        cabin.visual(
            Box((0.06, 0.08, 0.64)),
            origin=Origin(xyz=(0.08, 0.0, -0.36)),
            material=dark_steel,
            name="right_hanger",
        )
        cabin.visual(
            Box((0.74, 1.02, 0.08)),
            origin=Origin(xyz=(0.0, 0.0, -0.70)),
            material=cabin_shell,
            name="roof",
        )
        cabin.visual(
            Box((0.68, 0.96, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -1.48)),
            material=cabin_shell,
            name="floor",
        )
        for x_sign in (-1.0, 1.0):
            for y_sign in (-1.0, 1.0):
                cabin.visual(
                    Box((0.06, 0.06, 0.74)),
                    origin=Origin(xyz=(0.31 * x_sign, 0.45 * y_sign, -1.09)),
                    material=cabin_shell,
                    name=f"post_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign < 0 else 'b'}",
                )
        cabin.visual(
            Box((0.56, 0.04, 0.74)),
            origin=Origin(xyz=(0.0, -0.48, -1.09)),
            material=cabin_glass,
            name="front_glass",
        )
        cabin.visual(
            Box((0.56, 0.04, 0.74)),
            origin=Origin(xyz=(0.0, 0.48, -1.09)),
            material=cabin_glass,
            name="rear_glass",
        )
        cabin.visual(
            Box((0.04, 0.84, 0.74)),
            origin=Origin(xyz=(-0.34, 0.0, -1.09)),
            material=cabin_glass,
            name="left_glass",
        )
        cabin.visual(
            Box((0.04, 0.84, 0.74)),
            origin=Origin(xyz=(0.34, 0.0, -1.09)),
            material=cabin_glass,
            name="right_glass",
        )
        cabin.visual(
            Box((0.42, 0.70, 0.12)),
            origin=Origin(xyz=(0.0, 0.0, -1.59)),
            material=cabin_shell,
            name="seat_plinth",
        )
        cabin.inertial = Inertial.from_geometry(
            Box((0.74, 1.02, 1.70)),
            mass=320.0,
            origin=Origin(xyz=(0.0, 0.0, -0.85)),
        )

        model.articulation(
            f"wheel_to_cabin_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=cabin,
            origin=Origin(
                xyz=(
                    0.0,
                    cabin_pivot_radius * math.sin(angle),
                    cabin_pivot_radius * math.cos(angle),
                )
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1500.0,
                velocity=1.5,
                lower=-6.3,
                upper=6.3,
            ),
        )

    model.articulation(
        "base_to_wheel",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120000.0,
            velocity=0.3,
            lower=-6.3,
            upper=6.3,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    wheel = object_model.get_part("wheel")
    cabin_00 = object_model.get_part("cabin_00")
    wheel_joint = object_model.get_articulation("base_to_wheel")
    cabin_joint = object_model.get_articulation("wheel_to_cabin_00")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wheel_rotates_on_horizontal_x_axis",
        tuple(round(value, 6) for value in wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected wheel axis (1, 0, 0), got {wheel_joint.axis}",
    )
    ctx.check(
        "cabin_hangs_on_horizontal_x_axis",
        tuple(round(value, 6) for value in cabin_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected cabin axis (1, 0, 0), got {cabin_joint.axis}",
    )
    ctx.expect_contact(
        wheel,
        base_frame,
        elem_a="left_shaft",
        elem_b="left_bearing_housing",
        name="left_shaft_contacts_left_bearing",
    )
    ctx.expect_contact(
        wheel,
        base_frame,
        elem_a="right_shaft",
        elem_b="right_bearing_housing",
        name="right_shaft_contacts_right_bearing",
    )
    ctx.expect_contact(
        cabin_00,
        wheel,
        elem_a="top_axle",
        elem_b="left_clamp_00",
        name="cabin_00_axle_contacts_left_clamp",
    )
    ctx.expect_contact(
        cabin_00,
        wheel,
        elem_a="top_axle",
        elem_b="right_clamp_00",
        name="cabin_00_axle_contacts_right_clamp",
    )
    ctx.expect_within(
        cabin_00,
        wheel,
        axes="x",
        margin=0.0,
        name="cabin_00_stays_between_rims",
    )

    with ctx.pose({wheel_joint: math.pi / 2.0, cabin_joint: -math.pi / 2.0}):
        ctx.expect_gap(
            cabin_00,
            base_frame,
            axis="z",
            min_gap=7.8,
            negative_elem="foundation",
            name="side_cabin_clears_foundation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
