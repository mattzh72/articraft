from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

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
    ValidationError,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


AXLE_HEIGHT = 4.65
RIM_RADIUS = 3.15
PIVOT_RADIUS = 3.80
WHEEL_HALF_WIDTH = 0.24
GONDOLA_COUNT = 8


def _circle_points(radius: float, y: float, samples: int, phase: float = 0.0) -> list[tuple[float, float, float]]:
    return [
        (
            radius * cos(phase + (2.0 * pi * index / samples)),
            y,
            radius * sin(phase + (2.0 * pi * index / samples)),
        )
        for index in range(samples)
    ]


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    closed: bool = False,
    radial_segments: int = 18,
):
    if closed:
        geom = tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=8,
            closed_spline=True,
            radial_segments=radial_segments,
            cap_ends=False,
            up_hint=(0.0, 1.0, 0.0),
        )
    else:
        geom = wire_from_points(
            points,
            radius=radius,
            radial_segments=radial_segments,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=max(radius * 2.0, 0.04),
            corner_segments=10,
            up_hint=(0.0, 1.0, 0.0),
        )
    return mesh_from_geometry(geom, name)


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_xz_segment_cylinder(
    part,
    *,
    start: tuple[float, float],
    end: tuple[float, float],
    y: float,
    radius: float,
    material,
    name: str | None = None,
    overlap: float = 0.0,
) -> None:
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    length = sqrt(dx * dx + dz * dz) + overlap
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((start[0] + end[0]) * 0.5, y, (start[1] + end[1]) * 0.5),
            rpy=(0.0, atan2(dx, dz), 0.0),
        ),
        material=material,
        name=name,
    )


def _has_part(model: ArticulatedObject, name: str) -> bool:
    try:
        model.get_part(name)
    except ValidationError:
        return False
    return True


def _has_joint(model: ArticulatedObject, name: str) -> bool:
    try:
        model.get_articulation(name)
    except ValidationError:
        return False
    return True


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fairground_observation_wheel")

    support_white = model.material("support_white", rgba=(0.93, 0.94, 0.95, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    wheel_blue = model.material("wheel_blue", rgba=(0.14, 0.33, 0.63, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.84, 0.87, 0.91, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.32, 0.34, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.63, 0.67, 1.0))
    gondola_red = model.material("gondola_red", rgba=(0.77, 0.18, 0.18, 1.0))
    gondola_yellow = model.material("gondola_yellow", rgba=(0.94, 0.74, 0.18, 1.0))
    gondola_teal = model.material("gondola_teal", rgba=(0.16, 0.60, 0.58, 1.0))
    gondola_orange = model.material("gondola_orange", rgba=(0.89, 0.43, 0.17, 1.0))
    gondola_colors = [gondola_red, gondola_yellow, gondola_teal, gondola_orange]

    support = model.part("support")
    support.inertial = Inertial.from_geometry(
        Box((4.20, 1.70, 7.10)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, 3.55)),
    )
    support.visual(
        Box((4.20, 1.70, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=deck_gray,
        name="base_platform",
    )
    support.visual(
        Box((1.65, 1.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=wheel_white,
        name="loading_deck",
    )
    for side_name, y_side in (("front", -0.72), ("rear", 0.72)):
        support.visual(
            _tube_mesh(
                f"{side_name}_a_frame",
                [(-1.62, y_side, 0.12), (0.0, y_side, AXLE_HEIGHT), (1.62, y_side, 0.12)],
                radius=0.10,
            ),
            material=support_white,
            name=f"{side_name}_a_frame",
        )
        support.visual(
            _tube_mesh(
                f"{side_name}_mid_tie",
                [(-0.82, y_side, 2.20), (0.82, y_side, 2.20)],
                radius=0.055,
            ),
            material=support_white,
            name=f"{side_name}_mid_tie",
        )
    support.visual(
        Box((0.24, 0.08, 0.34)),
        origin=Origin(xyz=(0.0, -0.72, AXLE_HEIGHT + 0.17)),
        material=support_white,
        name="top_crossbeam",
    )
    support.visual(
        Box((0.24, 0.08, 0.34)),
        origin=Origin(xyz=(0.0, 0.72, AXLE_HEIGHT + 0.17)),
        material=support_white,
        name="rear_top_crossbeam",
    )
    support.visual(
        Box((2.08, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, -0.72, 1.90)),
        material=support_white,
        name="lower_crossbeam",
    )
    support.visual(
        Box((2.08, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.72, 1.90)),
        material=support_white,
        name="rear_lower_crossbeam",
    )
    support.visual(
        Box((0.30, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, -0.46, AXLE_HEIGHT)),
        material=hub_dark,
        name="front_bearing",
    )
    support.visual(
        Box((0.30, 0.06, 0.28)),
        origin=Origin(xyz=(0.0, 0.46, AXLE_HEIGHT)),
        material=hub_dark,
        name="rear_bearing",
    )
    support.visual(
        Box((0.12, 0.26, 0.12)),
        origin=Origin(xyz=(0.0, -0.59, AXLE_HEIGHT + 0.07)),
        material=support_white,
        name="front_bearing_mount",
    )
    support.visual(
        Box((0.12, 0.26, 0.12)),
        origin=Origin(xyz=(0.0, 0.59, AXLE_HEIGHT + 0.07)),
        material=support_white,
        name="rear_bearing_mount",
    )
    support.visual(
        Box((0.62, 0.24, 0.16)),
        origin=Origin(xyz=(0.95, 0.0, 0.20)),
        material=deck_gray,
        name="operator_plinth",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Box((6.40, 0.90, 6.40)),
        mass=950.0,
        origin=Origin(),
    )
    wheel.visual(
        Cylinder(radius=0.22, length=0.86),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="hub_shell",
    )
    wheel.visual(
        Cylinder(radius=0.38, length=0.06),
        origin=Origin(xyz=(0.0, -0.22, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_white,
        name="front_flange",
    )
    wheel.visual(
        Cylinder(radius=0.38, length=0.06),
        origin=Origin(xyz=(0.0, 0.22, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_white,
        name="rear_flange",
    )
    rim_segments = 24
    for rim_index in range(rim_segments):
        angle0 = 2.0 * pi * rim_index / rim_segments
        angle1 = 2.0 * pi * (rim_index + 1) / rim_segments
        start = (RIM_RADIUS * cos(angle0), RIM_RADIUS * sin(angle0))
        end = (RIM_RADIUS * cos(angle1), RIM_RADIUS * sin(angle1))
        _add_xz_segment_cylinder(
            wheel,
            start=start,
            end=end,
            y=-WHEEL_HALF_WIDTH,
            radius=0.065,
            overlap=0.08,
            material=wheel_blue,
            name="front_rim" if rim_index == 0 else f"front_rim_seg_{rim_index}",
        )
        _add_xz_segment_cylinder(
            wheel,
            start=start,
            end=end,
            y=WHEEL_HALF_WIDTH,
            radius=0.065,
            overlap=0.08,
            material=wheel_blue,
            name="rear_rim" if rim_index == 0 else f"rear_rim_seg_{rim_index}",
        )
    for brace_index in range(rim_segments):
        if brace_index % 3 == 0:
            continue
        angle = 2.0 * pi * brace_index / rim_segments
        x = RIM_RADIUS * cos(angle)
        z = RIM_RADIUS * sin(angle)
        _add_y_cylinder(
            wheel,
            radius=0.028,
            length=WHEEL_HALF_WIDTH * 2.0 + 0.03,
            xyz=(x, 0.0, z),
            material=wheel_white,
            name=f"rim_tie_{brace_index}",
        )
    spoke_count = 16
    for spoke_index in range(spoke_count):
        angle = 2.0 * pi * spoke_index / spoke_count
        offset = pi / spoke_count
        _add_xz_segment_cylinder(
            wheel,
            start=(0.34 * cos(angle), 0.34 * sin(angle)),
            end=((RIM_RADIUS - 0.02) * cos(angle + offset), (RIM_RADIUS - 0.02) * sin(angle + offset)),
            y=-0.205,
            radius=0.024,
            overlap=0.06,
            material=wheel_white,
            name=f"front_spoke_{spoke_index}",
        )
        _add_xz_segment_cylinder(
            wheel,
            start=(0.34 * cos(angle + offset), 0.34 * sin(angle + offset)),
            end=((RIM_RADIUS - 0.02) * cos(angle), (RIM_RADIUS - 0.02) * sin(angle)),
            y=0.205,
            radius=0.024,
            overlap=0.06,
            material=wheel_white,
            name=f"rear_spoke_{spoke_index}",
        )

    for gondola_index in range(GONDOLA_COUNT):
        angle = pi / 2.0 - (2.0 * pi * gondola_index / GONDOLA_COUNT)
        pivot_x = PIVOT_RADIUS * cos(angle)
        pivot_z = PIVOT_RADIUS * sin(angle)

        wheel.visual(
            Box((0.10, 0.05, 0.16)),
            origin=Origin(xyz=(pivot_x, -0.245, pivot_z)),
            material=hub_dark,
            name=f"hanger_front_{gondola_index}",
        )
        wheel.visual(
            Box((0.10, 0.05, 0.16)),
            origin=Origin(xyz=(pivot_x, 0.245, pivot_z)),
            material=hub_dark,
            name=f"hanger_back_{gondola_index}",
        )
        _add_xz_segment_cylinder(
            wheel,
            start=(RIM_RADIUS * cos(angle), RIM_RADIUS * sin(angle)),
            end=((PIVOT_RADIUS - 0.04) * cos(angle), (PIVOT_RADIUS - 0.04) * sin(angle)),
            y=-0.285,
            radius=0.028,
            overlap=0.05,
            material=wheel_blue,
            name=f"hanger_front_strut_{gondola_index}",
        )
        _add_xz_segment_cylinder(
            wheel,
            start=(RIM_RADIUS * cos(angle), RIM_RADIUS * sin(angle)),
            end=((PIVOT_RADIUS - 0.04) * cos(angle), (PIVOT_RADIUS - 0.04) * sin(angle)),
            y=0.285,
            radius=0.028,
            overlap=0.05,
            material=wheel_blue,
            name=f"hanger_back_strut_{gondola_index}",
        )

        gondola = model.part(f"gondola_{gondola_index}")
        gondola.inertial = Inertial.from_geometry(
            Box((0.38, 0.30, 0.74)),
            mass=90.0,
            origin=Origin(xyz=(0.0, 0.0, -0.37)),
        )
        color = gondola_colors[gondola_index % len(gondola_colors)]

        gondola.visual(
            Cylinder(radius=0.045, length=0.04),
            origin=Origin(xyz=(0.0, -0.20, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_boss_front",
        )
        gondola.visual(
            Cylinder(radius=0.045, length=0.04),
            origin=Origin(xyz=(0.0, 0.20, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_boss_back",
        )
        gondola.visual(
            Box((0.05, 0.08, 0.30)),
            origin=Origin(xyz=(0.0, -0.16, -0.15)),
            material=wheel_white,
            name="front_hanger_plate",
        )
        gondola.visual(
            Box((0.05, 0.08, 0.30)),
            origin=Origin(xyz=(0.0, 0.16, -0.15)),
            material=wheel_white,
            name="rear_hanger_plate",
        )
        gondola.visual(
            Box((0.34, 0.30, 0.06)),
            origin=Origin(xyz=(0.0, 0.0, -0.24)),
            material=color,
            name="canopy",
        )
        gondola.visual(
            Box((0.04, 0.04, 0.42)),
            origin=Origin(xyz=(0.14, -0.13, -0.43)),
            material=wheel_white,
            name="front_right_post",
        )
        gondola.visual(
            Box((0.04, 0.04, 0.42)),
            origin=Origin(xyz=(-0.14, -0.13, -0.43)),
            material=wheel_white,
            name="front_left_post",
        )
        gondola.visual(
            Box((0.04, 0.04, 0.42)),
            origin=Origin(xyz=(0.14, 0.13, -0.43)),
            material=wheel_white,
            name="rear_right_post",
        )
        gondola.visual(
            Box((0.04, 0.04, 0.42)),
            origin=Origin(xyz=(-0.14, 0.13, -0.43)),
            material=wheel_white,
            name="rear_left_post",
        )
        _add_y_cylinder(gondola, radius=0.016, length=0.30, xyz=(0.14, 0.0, -0.44), material=wheel_white, name="guard_bar")
        gondola.visual(
            Box((0.30, 0.30, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, -0.62)),
            material=deck_gray,
            name="floor",
        )
        gondola.visual(
            Box((0.10, 0.18, 0.12)),
            origin=Origin(xyz=(-0.03, 0.0, -0.54)),
            material=deck_gray,
            name="seat_pedestal",
        )
        gondola.visual(
            Box((0.18, 0.26, 0.05)),
            origin=Origin(xyz=(-0.01, 0.0, -0.47)),
            material=color,
            name="seat",
        )
        gondola.visual(
            Box((0.04, 0.26, 0.23)),
            origin=Origin(xyz=(-0.10, 0.0, -0.40)),
            material=color,
            name="backrest",
        )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=0.8),
    )
    for gondola_index in range(GONDOLA_COUNT):
        angle = pi / 2.0 - (2.0 * pi * gondola_index / GONDOLA_COUNT)
        model.articulation(
            f"wheel_to_gondola_{gondola_index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=f"gondola_{gondola_index}",
            origin=Origin(xyz=(PIVOT_RADIUS * cos(angle), 0.0, PIVOT_RADIUS * sin(angle))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=150.0, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    support = object_model.get_part("support")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("support_to_wheel")

    expected_parts = ["support", "wheel"] + [f"gondola_{index}" for index in range(GONDOLA_COUNT)]
    expected_joints = ["support_to_wheel"] + [f"wheel_to_gondola_{index}" for index in range(GONDOLA_COUNT)]
    for part_name in expected_parts:
        ctx.check(f"{part_name}_present", _has_part(object_model, part_name), f"missing part: {part_name}")
    for joint_name in expected_joints:
        ctx.check(f"{joint_name}_present", _has_joint(object_model, joint_name), f"missing articulation: {joint_name}")

    ctx.check(
        "wheel_joint_is_continuous",
        wheel_joint.joint_type == ArticulationType.CONTINUOUS,
        f"expected continuous wheel joint, got {wheel_joint.joint_type}",
    )
    ctx.check(
        "wheel_axis_is_horizontal",
        tuple(round(value, 6) for value in wheel_joint.axis) == (0.0, 1.0, 0.0),
        f"unexpected wheel axis: {wheel_joint.axis}",
    )
    ctx.expect_contact(wheel, support, elem_a="hub_shell", elem_b="front_bearing", name="wheel_contacts_front_bearing")
    ctx.expect_contact(wheel, support, elem_a="hub_shell", elem_b="rear_bearing", name="wheel_contacts_rear_bearing")

    for gondola_index in range(GONDOLA_COUNT):
        gondola = object_model.get_part(f"gondola_{gondola_index}")
        joint = object_model.get_articulation(f"wheel_to_gondola_{gondola_index}")
        ctx.check(
            f"gondola_{gondola_index}_pivot_is_continuous",
            joint.joint_type == ArticulationType.CONTINUOUS,
            f"expected continuous gondola pivot, got {joint.joint_type}",
        )
        ctx.check(
            f"gondola_{gondola_index}_pivot_axis_parallel_to_axle",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0),
            f"unexpected gondola axis: {joint.axis}",
        )
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a="pivot_boss_front",
            elem_b=f"hanger_front_{gondola_index}",
            name=f"gondola_{gondola_index}_captured_front",
        )
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a="pivot_boss_back",
            elem_b=f"hanger_back_{gondola_index}",
            name=f"gondola_{gondola_index}_captured_back",
        )

    sample_gondola = object_model.get_part("gondola_0")
    sample_joint = object_model.get_articulation("wheel_to_gondola_0")
    with ctx.pose({wheel_joint: pi / 2.0, sample_joint: -pi / 2.0}):
        ctx.expect_contact(
            sample_gondola,
            wheel,
            elem_a="pivot_boss_front",
            elem_b="hanger_front_0",
            name="gondola_0_stays_captured_after_quarter_turn_front",
        )
        ctx.expect_contact(
            sample_gondola,
            wheel,
            elem_a="pivot_boss_back",
            elem_b="hanger_back_0",
            name="gondola_0_stays_captured_after_quarter_turn_back",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
