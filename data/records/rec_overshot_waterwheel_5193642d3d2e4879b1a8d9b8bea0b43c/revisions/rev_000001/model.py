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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_shell_mesh(
    *,
    name: str,
    inner_radius: float,
    outer_radius: float,
    length: float,
):
    half = length * 0.5
    geom = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half),
            (outer_radius, half),
        ],
        [
            (inner_radius, -half),
            (inner_radius, half),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geom)


def _hub_shell_mesh(name: str):
    geom = LatheGeometry.from_shell_profiles(
        [
            (0.22, -0.30),
            (0.19, -0.24),
            (0.19, 0.24),
            (0.22, 0.30),
        ],
        [
            (0.056, -0.30),
            (0.056, 0.30),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geom)


def _pivot_bushing_mesh(name: str):
    return _ring_shell_mesh(
        name=name,
        inner_radius=0.045,
        outer_radius=0.075,
        length=0.08,
    )


def _pivot_bushing_vertical_mesh(name: str):
    geom = LatheGeometry.from_shell_profiles(
        [
            (0.072, -0.09),
            (0.072, 0.09),
        ],
        [
            (0.050, -0.09),
            (0.050, 0.09),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mill_waterwheel")

    stone = model.material("stone", rgba=(0.58, 0.58, 0.56, 1.0))
    frame_timber = model.material("frame_timber", rgba=(0.43, 0.31, 0.18, 1.0))
    wheel_timber = model.material("wheel_timber", rgba=(0.36, 0.25, 0.13, 1.0))
    weathered_oak = model.material("weathered_oak", rgba=(0.48, 0.35, 0.22, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.23, 0.24, 0.25, 1.0))
    brake_lining = model.material("brake_lining", rgba=(0.20, 0.16, 0.12, 1.0))

    shroud_mesh = _ring_shell_mesh(
        name="wheel_shroud_ring",
        inner_radius=0.94,
        outer_radius=1.22,
        length=0.035,
    )
    inner_ring_mesh = _ring_shell_mesh(
        name="wheel_inner_ring",
        inner_radius=0.72,
        outer_radius=0.86,
        length=0.12,
    )
    hub_mesh = _hub_shell_mesh("wheel_hub_shell")
    bushing_mesh = _pivot_bushing_vertical_mesh("brake_pivot_bushing")

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((3.20, 1.90, 2.55)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.275)),
    )

    for side, y_pos in (("left", -0.72), ("right", 0.72)):
        frame.visual(
            Box((2.30, 0.28, 0.20)),
            origin=Origin(xyz=(0.0, y_pos, 0.10)),
            material=stone,
            name=f"{side}_stone_sill",
        )
        frame.visual(
            Box((0.16, 0.14, 1.72)),
            origin=Origin(xyz=(-0.42, y_pos, 0.99), rpy=(0.0, 0.38, 0.0)),
            material=frame_timber,
            name=f"{side}_front_leg",
        )
        frame.visual(
            Box((0.16, 0.14, 1.72)),
            origin=Origin(xyz=(0.42, y_pos, 0.99), rpy=(0.0, -0.38, 0.0)),
            material=frame_timber,
            name=f"{side}_rear_leg",
        )
        frame.visual(
            Box((1.08, 0.14, 0.16)),
            origin=Origin(xyz=(0.0, y_pos, 1.74)),
            material=frame_timber,
            name=f"{side}_head_beam",
        )
        frame.visual(
            Box((0.92, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, y_pos, 0.90), rpy=(0.0, 0.77, 0.0)),
            material=frame_timber,
            name=f"{side}_brace_rising",
        )
        frame.visual(
            Box((0.92, 0.10, 0.12)),
            origin=Origin(xyz=(0.0, y_pos, 0.90), rpy=(0.0, -0.77, 0.0)),
            material=frame_timber,
            name=f"{side}_brace_falling",
        )
        frame.visual(
            Box((0.34, 0.22, 0.26)),
            origin=Origin(xyz=(0.0, y_pos * 0.80, 1.40)),
            material=frame_timber,
            name=f"{side}_bearing_block",
        )

    for name, x_pos, size_y, size_z, z_pos in (
        ("front_tie", -0.84, 1.44, 0.14, 0.20),
        ("rear_tie", 0.84, 1.44, 0.14, 0.20),
    ):
        frame.visual(
            Box((0.22, size_y, size_z)),
            origin=Origin(xyz=(x_pos, 0.0, z_pos)),
            material=frame_timber,
            name=name,
        )
    frame.visual(
        Box((0.20, 1.20, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=frame_timber,
        name="center_tie",
    )

    frame.visual(
        Cylinder(radius=0.05, length=1.60),
        origin=Origin(xyz=(0.0, 0.0, 1.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    frame.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.0, -0.315, 1.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="left_thrust_collar",
    )
    frame.visual(
        Cylinder(radius=0.10, length=0.03),
        origin=Origin(xyz=(0.0, 0.315, 1.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="right_thrust_collar",
    )

    frame.visual(
        Box((1.34, 0.12, 0.14)),
        origin=Origin(xyz=(1.09, 0.72, 1.78)),
        material=frame_timber,
        name="brake_support_beam",
    )
    frame.visual(
        Box((0.10, 0.06, 0.34)),
        origin=Origin(xyz=(1.72, 0.75, 1.61)),
        material=frame_timber,
        name="brake_support_hanger",
    )
    frame.visual(
        Box((0.08, 0.12, 0.10)),
        origin=Origin(xyz=(1.72, 0.72, 1.652)),
        material=dark_iron,
        name="brake_retainer_bracket",
    )
    frame.visual(
        Box((0.16, 0.08, 0.08)),
        origin=Origin(xyz=(1.72, 0.69, 1.70)),
        material=dark_iron,
        name="brake_support_bridge",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.18),
        origin=Origin(xyz=(1.72, 0.60, 1.50)),
        material=dark_iron,
        name="brake_pivot_pin",
    )
    frame.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(1.72, 0.60, 1.596)),
        material=dark_iron,
        name="brake_pin_retain_upper",
    )
    frame.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(1.72, 0.60, 1.404)),
        material=dark_iron,
        name="brake_pin_retain_lower",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=1.22, length=0.56),
        mass=420.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    wheel.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, -0.262, 0.0)),
        material=wheel_timber,
        name="left_shroud",
    )
    wheel.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, 0.262, 0.0)),
        material=wheel_timber,
        name="right_shroud",
    )
    wheel.visual(
        inner_ring_mesh,
        material=weathered_oak,
        name="inner_ring",
    )
    wheel.visual(
        hub_mesh,
        material=dark_iron,
        name="hub_shell",
    )

    spoke_count = 8
    for index in range(spoke_count):
        angle = (2.0 * pi * index) / spoke_count
        wheel.visual(
            Box((0.10, 0.12, 0.82)),
            origin=Origin(
                xyz=(sin(angle) * 0.51, 0.0, cos(angle) * 0.51),
                rpy=(0.0, angle, 0.0),
            ),
            material=weathered_oak,
            name=f"spoke_{index:02d}",
        )

    bucket_count = 16
    step = (2.0 * pi) / bucket_count
    for index in range(bucket_count):
        divider_angle = index * step
        rim_angle = divider_angle + (0.5 * step)
        floor_angle = divider_angle + (0.34 * step)

        wheel.visual(
            Box((0.46, 0.54, 0.10)),
            origin=Origin(
                xyz=(sin(rim_angle) * 1.17, 0.0, cos(rim_angle) * 1.17),
                rpy=(0.0, rim_angle, 0.0),
            ),
            material=wheel_timber,
            name=f"outer_rim_band_{index:02d}",
        )
        wheel.visual(
            Box((0.11, 0.52, 0.30)),
            origin=Origin(
                xyz=(sin(divider_angle) * 1.00, 0.0, cos(divider_angle) * 1.00),
                rpy=(0.0, divider_angle, 0.0),
            ),
            material=wheel_timber,
            name=f"bucket_divider_{index:02d}",
        )
        wheel.visual(
            Box((0.24, 0.52, 0.08)),
            origin=Origin(
                xyz=(sin(floor_angle) * 0.89, 0.0, cos(floor_angle) * 0.89),
                rpy=(0.0, floor_angle, 0.0),
            ),
            material=weathered_oak,
            name=f"bucket_floor_{index:02d}",
        )

    brake_lever = model.part("brake_lever")
    brake_lever.inertial = Inertial.from_geometry(
        Box((1.08, 0.34, 0.34)),
        mass=22.0,
        origin=Origin(xyz=(0.00, -0.08, -0.04)),
    )
    brake_lever.visual(
        bushing_mesh,
        material=dark_iron,
        name="pivot_bushing",
    )
    brake_lever.visual(
        Box((0.12, 0.12, 0.12)),
        origin=Origin(xyz=(0.13, -0.03, -0.02)),
        material=dark_iron,
        name="lever_hub",
    )
    brake_lever.visual(
        Box((0.72, 0.08, 0.08)),
        origin=Origin(xyz=(0.46, 0.07, 0.06)),
        material=frame_timber,
        name="handle_beam",
    )
    brake_lever.visual(
        Box((0.22, 0.06, 0.08)),
        origin=Origin(xyz=(-0.02, -0.12, -0.08)),
        material=weathered_oak,
        name="shoe_link",
    )
    brake_lever.visual(
        Box((0.42, 0.10, 0.10)),
        origin=Origin(xyz=(-0.25, -0.15, -0.10)),
        material=weathered_oak,
        name="shoe_arm",
    )
    brake_lever.visual(
        Box((0.16, 0.12, 0.16)),
        origin=Origin(xyz=(-0.44, -0.22, -0.18)),
        material=weathered_oak,
        name="shoe_block",
    )
    brake_lever.visual(
        Box((0.14, 0.04, 0.12)),
        origin=Origin(xyz=(-0.52, -0.22, -0.18)),
        material=brake_lining,
        name="shoe_lining",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=1.8),
    )
    model.articulation(
        "brake_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake_lever,
        origin=Origin(xyz=(1.72, 0.60, 1.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.0,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    brake_lever = object_model.get_part("brake_lever")
    wheel_spin = object_model.get_articulation("wheel_spin")
    brake_pivot = object_model.get_articulation("brake_pivot")

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
        "wheel_spin_joint_is_continuous_y",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"Expected continuous Y-axis wheel joint, got {wheel_spin.articulation_type} axis={wheel_spin.axis}",
    )
    ctx.check(
        "brake_pivot_joint_is_revolute_z",
        brake_pivot.articulation_type == ArticulationType.REVOLUTE and brake_pivot.axis == (0.0, 0.0, 1.0),
        details=f"Expected revolute Z-axis brake pivot, got {brake_pivot.articulation_type} axis={brake_pivot.axis}",
    )

    wheel_pos = ctx.part_world_position(wheel)
    brake_pos = ctx.part_world_position(brake_lever)
    ctx.check(
        "wheel_is_raised_between_frames",
        wheel_pos is not None and 1.25 <= wheel_pos[2] <= 1.55,
        details=f"Unexpected wheel world position: {wheel_pos}",
    )
    ctx.check(
        "brake_pivot_sits_outboard_of_wheel",
        wheel_pos is not None
        and brake_pos is not None
        and brake_pos[0] > wheel_pos[0] + 1.2
        and brake_pos[1] > wheel_pos[1] + 0.45
        and brake_pos[2] > wheel_pos[2] + 0.05,
        details=f"Wheel pos={wheel_pos}, brake pos={brake_pos}",
    )

    ctx.expect_contact(wheel, frame, name="wheel_carried_on_axle")
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="hub_shell",
        elem_b="right_thrust_collar",
        name="hub_shell_bears_against_right_collar",
    )
    ctx.expect_overlap(
        brake_lever,
        frame,
        elem_a="pivot_bushing",
        elem_b="brake_pivot_pin",
        axes="xy",
        min_overlap=0.08,
        name="brake_bushing_is_coaxial_with_pivot_pin",
    )
    ctx.expect_gap(
        frame,
        brake_lever,
        axis="z",
        min_gap=0.0,
        max_gap=0.01,
        positive_elem="brake_pin_retain_upper",
        negative_elem="pivot_bushing",
        name="upper_retainer_clips_brake_bushing",
    )
    ctx.expect_gap(
        brake_lever,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.01,
        positive_elem="pivot_bushing",
        negative_elem="brake_pin_retain_lower",
        name="lower_retainer_clips_brake_bushing",
    )
    ctx.expect_gap(
        brake_lever,
        wheel,
        axis="y",
        min_gap=0.05,
        positive_elem="shoe_lining",
        negative_elem="right_shroud",
        name="released_brake_shoe_clears_rim",
    )

    with ctx.pose({wheel_spin: 1.1}):
        ctx.expect_contact(wheel, frame, name="wheel_stays_carried_through_spin")

    with ctx.pose({brake_pivot: 0.14}):
        ctx.expect_gap(
            brake_lever,
            wheel,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="shoe_lining",
            negative_elem="right_shroud",
            name="brake_shoe_can_bite_rim",
        )
        ctx.expect_overlap(
            brake_lever,
            wheel,
            axes="xz",
            min_overlap=0.05,
            elem_a="shoe_lining",
            elem_b="right_shroud",
            name="brake_shoe_aligns_with_broad_rim",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
