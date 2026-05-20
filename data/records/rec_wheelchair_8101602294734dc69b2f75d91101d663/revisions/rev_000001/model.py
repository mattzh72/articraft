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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_rear_wheel_visuals(part, *, side_sign: float, rubber, spoke_metal, frame_metal) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    tire_width = 0.035
    rim_width = 0.022
    pushrim_offset = 0.030 * side_sign

    part.visual(
        Cylinder(radius=0.305, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.270, length=rim_width),
        origin=spin_origin,
        material=spoke_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=spin_origin,
        material=frame_metal,
        name="hub",
    )
    for index, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0)):
        part.visual(
            Box((0.47, 0.006, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, angle, 0.0)),
            material=spoke_metal,
            name=f"spoke_{index}",
        )
    part.visual(
        Cylinder(radius=0.288, length=0.006),
        origin=Origin(xyz=(0.0, pushrim_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="pushrim",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        x = 0.205 * cos(angle)
        z = 0.205 * sin(angle)
        part.visual(
            Box((0.050, 0.006, 0.006)),
            origin=Origin(xyz=(x, pushrim_offset * 0.5, z), rpy=(0.0, angle, 0.0)),
            material=frame_metal,
            name=f"pushrim_standoff_{index}",
        )


def _add_caster_wheel_visuals(part, *, rubber, spoke_metal, frame_metal) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=spin_origin,
        material=spoke_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=spin_origin,
        material=frame_metal,
        name="hub",
    )
    part.visual(Box((0.090, 0.006, 0.010)), material=spoke_metal, name="spoke_bar")


def _add_caster_fork_visuals(part, *, frame_metal, footplate_black) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=frame_metal,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=frame_metal,
        name="crown",
    )
    part.visual(
        Box((0.055, 0.040, 0.012)),
        origin=Origin(xyz=(-0.028, 0.0, -0.105)),
        material=frame_metal,
        name="fork_bridge",
    )
    part.visual(
        Box((0.012, 0.010, 0.078)),
        origin=Origin(xyz=(-0.028, 0.020, -0.150)),
        material=frame_metal,
        name="outer_leg",
    )
    part.visual(
        Box((0.012, 0.010, 0.078)),
        origin=Origin(xyz=(-0.028, -0.020, -0.150)),
        material=frame_metal,
        name="inner_leg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_metal = model.material("frame_metal", rgba=(0.73, 0.76, 0.80, 1.0))
    spoke_metal = model.material("spoke_metal", rgba=(0.82, 0.84, 0.87, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    fabric = model.material("fabric", rgba=(0.10, 0.12, 0.16, 1.0))
    footplate_black = model.material("footplate_black", rgba=(0.14, 0.15, 0.16, 1.0))

    tube_r = 0.012
    side_y = 0.23

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.62, 0.96)),
        mass=13.0,
        origin=Origin(xyz=(0.04, 0.0, 0.48)),
    )

    left_side_points = [
        (0.34, side_y, 0.93),
        (0.26, side_y, 0.82),
        (-0.16, side_y, 0.80),
        (-0.16, side_y, 0.54),
        (0.08, side_y, 0.54),
        (0.28, side_y, 0.46),
    ]
    right_side_points = [(x, -y, z) for x, y, z in left_side_points]

    left_side_frame = tube_from_spline_points(
        left_side_points,
        radius=tube_r,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_side_frame = tube_from_spline_points(
        right_side_points,
        radius=tube_r,
        samples_per_segment=12,
        radial_segments=16,
    )
    frame.visual(mesh_from_geometry(left_side_frame, "left_side_frame"), material=frame_metal, name="left_side_frame")
    frame.visual(mesh_from_geometry(right_side_frame, "right_side_frame"), material=frame_metal, name="right_side_frame")

    frame.visual(
        Cylinder(radius=tube_r, length=0.46),
        origin=Origin(xyz=(0.03, 0.0, 0.54), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="front_seat_crossbar",
    )
    frame.visual(
        Cylinder(radius=tube_r, length=0.46),
        origin=Origin(xyz=(-0.12, 0.0, 0.54), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="rear_seat_crossbar",
    )
    frame.visual(
        Cylinder(radius=tube_r, length=0.50),
        origin=Origin(xyz=(0.28, 0.0, 0.46), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="front_lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=tube_r, length=0.50),
        origin=Origin(xyz=(0.32, 0.0, 0.93), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="push_handle_bar",
    )
    frame.visual(
        Cylinder(radius=tube_r, length=0.48),
        origin=Origin(xyz=(-0.08, 0.0, 0.54), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="rear_lower_crossbar",
    )
    left_axle_support = tube_from_spline_points(
        [(-0.16, side_y, 0.54), (-0.12, 0.245, 0.44), (-0.09, 0.245, 0.305)],
        radius=tube_r,
        samples_per_segment=10,
        radial_segments=14,
    )
    right_axle_support = tube_from_spline_points(
        [(-0.16, -side_y, 0.54), (-0.12, -0.245, 0.44), (-0.09, -0.245, 0.305)],
        radius=tube_r,
        samples_per_segment=10,
        radial_segments=14,
    )
    frame.visual(mesh_from_geometry(left_axle_support, "left_axle_support"), material=frame_metal, name="left_axle_support")
    frame.visual(mesh_from_geometry(right_axle_support, "right_axle_support"), material=frame_metal, name="right_axle_support")
    frame.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.09, 0.260, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(-0.09, -0.260, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_metal,
        name="right_axle_stub",
    )
    frame.visual(
        Box((0.36, 0.40, 0.014)),
        origin=Origin(xyz=(0.10, 0.0, 0.521)),
        material=fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.030, 0.44, 0.34)),
        origin=Origin(xyz=(-0.165, 0.0, 0.71)),
        material=fabric,
        name="backrest_sling",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.22),
        origin=Origin(xyz=(-0.16, 0.22, 0.65)),
        material=frame_metal,
        name="left_backrest_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.22),
        origin=Origin(xyz=(-0.16, -0.22, 0.65)),
        material=frame_metal,
        name="right_backrest_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.26),
        origin=Origin(xyz=(0.30, 0.12, 0.33)),
        material=frame_metal,
        name="left_footrest_hanger",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(0.34, 0.12, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_metal,
        name="left_footrest_support",
    )
    frame.visual(
        Box((0.12, 0.10, 0.010)),
        origin=Origin(xyz=(0.40, 0.12, 0.23)),
        material=footplate_black,
        name="left_footplate",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.26),
        origin=Origin(xyz=(0.30, -0.12, 0.33)),
        material=frame_metal,
        name="right_footrest_hanger",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.08),
        origin=Origin(xyz=(0.34, -0.12, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_metal,
        name="right_footrest_support",
    )
    frame.visual(
        Box((0.12, 0.10, 0.010)),
        origin=Origin(xyz=(0.40, -0.12, 0.23)),
        material=footplate_black,
        name="right_footplate",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(xyz=(0.28, 0.20, 0.420)),
        material=frame_metal,
        name="left_caster_receiver",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.080),
        origin=Origin(xyz=(0.28, -0.20, 0.420)),
        material=frame_metal,
        name="right_caster_receiver",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.035),
        mass=2.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        left_rear_wheel,
        side_sign=1.0,
        rubber=rubber,
        spoke_metal=spoke_metal,
        frame_metal=frame_metal,
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.035),
        mass=2.2,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        right_rear_wheel,
        side_sign=-1.0,
        rubber=rubber,
        spoke_metal=spoke_metal,
        frame_metal=frame_metal,
    )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.09, 0.305, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.09, -0.305, 0.305)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )

    left_caster_swivel = model.part("left_caster_swivel")
    left_caster_swivel.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.22)),
        mass=0.45,
        origin=Origin(xyz=(-0.02, 0.0, -0.11)),
    )
    _add_caster_fork_visuals(left_caster_swivel, frame_metal=frame_metal, footplate_black=footplate_black)

    right_caster_swivel = model.part("right_caster_swivel")
    right_caster_swivel.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 0.22)),
        mass=0.45,
        origin=Origin(xyz=(-0.02, 0.0, -0.11)),
    )
    _add_caster_fork_visuals(right_caster_swivel, frame_metal=frame_metal, footplate_black=footplate_black)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.030),
        mass=0.35,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(left_caster_wheel, rubber=rubber, spoke_metal=spoke_metal, frame_metal=frame_metal)

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.030),
        mass=0.35,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(right_caster_wheel, rubber=rubber, spoke_metal=spoke_metal, frame_metal=frame_metal)

    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_swivel,
        origin=Origin(xyz=(0.28, 0.20, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_swivel,
        origin=Origin(xyz=(0.28, -0.20, 0.380)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_swivel,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.028, 0.0, -0.190)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_swivel,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.028, 0.0, -0.190)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")

    ctx.expect_gap(
        left_rear_wheel,
        frame,
        axis="y",
        min_gap=0.0,
        max_gap=0.04,
        name="left rear wheel sits tightly beside the frame",
    )
    ctx.expect_gap(
        frame,
        right_rear_wheel,
        axis="y",
        min_gap=0.0,
        max_gap=0.04,
        name="right rear wheel sits tightly beside the frame",
    )
    ctx.expect_gap(
        frame,
        left_caster_wheel,
        axis="z",
        min_gap=0.12,
        max_gap=0.24,
        positive_elem="front_lower_crossbar",
        name="left caster wheel hangs below the front frame rail",
    )
    ctx.expect_gap(
        frame,
        right_caster_wheel,
        axis="z",
        min_gap=0.12,
        max_gap=0.24,
        positive_elem="front_lower_crossbar",
        name="right caster wheel hangs below the front frame rail",
    )

    rest_pos = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({left_caster_swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(left_caster_wheel)
    ctx.check(
        "left caster swivel changes wheel heading position",
        rest_pos is not None
        and turned_pos is not None
        and abs(turned_pos[1] - rest_pos[1]) > 0.02,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
