from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.34, 0.46, 0.22, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    rim_metal = model.material("rim_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    def _mesh(name: str, geom):
        return mesh_from_geometry(geom, name)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.55, 0.70, 0.82)),
        mass=18.0,
        origin=Origin(xyz=(0.45, 0.0, 0.42)),
    )

    left_handle = tube_from_spline_points(
        [
            (-0.30, 0.23, 0.68),
            (-0.02, 0.22, 0.65),
            (0.26, 0.22, 0.56),
            (0.62, 0.18, 0.42),
            (0.90, 0.10, 0.28),
            (0.95, 0.10, 0.24),
        ],
        radius=0.022,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_handle = tube_from_spline_points(
        [
            (-0.30, -0.23, 0.68),
            (-0.02, -0.22, 0.65),
            (0.26, -0.22, 0.56),
            (0.62, -0.18, 0.42),
            (0.90, -0.10, 0.28),
            (0.95, -0.10, 0.24),
        ],
        radius=0.022,
        samples_per_segment=16,
        radial_segments=18,
    )
    left_fork_arm = tube_from_spline_points(
        [
            (0.95, 0.10, 0.24),
            (1.00, 0.085, 0.225),
            (1.032, 0.072, 0.215),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_fork_arm = tube_from_spline_points(
        [
            (0.95, -0.10, 0.24),
            (1.00, -0.085, 0.225),
            (1.032, -0.072, 0.215),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=16,
    )
    left_leg = tube_from_spline_points(
        [
            (0.12, 0.20, 0.55),
            (0.06, 0.19, 0.36),
            (0.01, 0.17, 0.06),
        ],
        radius=0.018,
        samples_per_segment=14,
        radial_segments=16,
    )
    right_leg = tube_from_spline_points(
        [
            (0.12, -0.20, 0.55),
            (0.06, -0.19, 0.36),
            (0.01, -0.17, 0.06),
        ],
        radius=0.018,
        samples_per_segment=14,
        radial_segments=16,
    )
    frame.visual(_mesh("left_handle", left_handle), material=frame_steel, name="left_handle")
    frame.visual(_mesh("right_handle", right_handle), material=frame_steel, name="right_handle")
    frame.visual(_mesh("left_fork_arm", left_fork_arm), material=frame_steel, name="left_fork_arm")
    frame.visual(_mesh("right_fork_arm", right_fork_arm), material=frame_steel, name="right_fork_arm")
    frame.visual(_mesh("left_leg", left_leg), material=frame_steel, name="left_leg")
    frame.visual(_mesh("right_leg", right_leg), material=frame_steel, name="right_leg")

    frame.visual(
        Box((0.82, 0.50, 0.024)),
        origin=Origin(xyz=(0.44, 0.0, 0.55)),
        material=tray_green,
        name="tray_floor",
    )
    frame.visual(
        Box((0.78, 0.022, 0.18)),
        origin=Origin(xyz=(0.42, 0.255, 0.64)),
        material=tray_green,
        name="left_tray_wall",
    )
    frame.visual(
        Box((0.78, 0.022, 0.18)),
        origin=Origin(xyz=(0.42, -0.255, 0.64)),
        material=tray_green,
        name="right_tray_wall",
    )
    frame.visual(
        Box((0.022, 0.52, 0.17)),
        origin=Origin(xyz=(0.83, 0.0, 0.64)),
        material=tray_green,
        name="front_tray_wall",
    )
    frame.visual(
        Box((0.020, 0.46, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.60)),
        material=tray_green,
        name="rear_tray_wall",
    )

    frame.visual(
        Box((0.10, 0.40, 0.030)),
        origin=Origin(xyz=(0.28, 0.0, 0.53)),
        material=frame_steel,
        name="undertray_crossbrace",
    )
    frame.visual(
        Box((0.10, 0.28, 0.036)),
        origin=Origin(xyz=(0.70, 0.0, 0.38)),
        material=frame_steel,
        name="front_crossbrace",
    )
    frame.visual(
        Box((0.10, 0.32, 0.040)),
        origin=Origin(xyz=(0.05, 0.0, 0.57)),
        material=frame_steel,
        name="rear_crossbrace",
    )
    frame.visual(
        Box((0.04, 0.18, 0.05)),
        origin=Origin(xyz=(0.85, 0.0, 0.30)),
        material=frame_steel,
        name="fork_bridge",
    )
    frame.visual(
        Box((0.09, 0.06, 0.08)),
        origin=Origin(xyz=(0.01, 0.17, 0.04)),
        material=frame_steel,
        name="left_foot",
    )
    frame.visual(
        Box((0.09, 0.06, 0.08)),
        origin=Origin(xyz=(0.01, -0.17, 0.04)),
        material=frame_steel,
        name="right_foot",
    )
    frame.visual(
        Box((0.035, 0.012, 0.08)),
        origin=Origin(xyz=(1.05, 0.067, 0.18)),
        material=frame_steel,
        name="left_fork_plate",
    )
    frame.visual(
        Box((0.035, 0.012, 0.08)),
        origin=Origin(xyz=(1.05, -0.067, 0.18)),
        material=frame_steel,
        name="right_fork_plate",
    )

    front_wheel = model.part("front_wheel")
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.09),
        mass=2.8,
        origin=spin_origin,
    )
    front_wheel.visual(
        Cylinder(radius=0.18, length=0.09),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.12, length=0.055),
        origin=spin_origin,
        material=rim_metal,
        name="rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.040, length=0.075),
        origin=spin_origin,
        material=frame_steel,
        name="hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.018, length=0.122),
        origin=spin_origin,
        material=frame_steel,
        name="axle_spindle",
    )

    model.articulation(
        "frame_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(1.05, 0.0, 0.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("front_wheel")
    wheel_joint = object_model.get_articulation("frame_to_front_wheel")

    ctx.check(
        "front wheel uses continuous axle joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and wheel_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="tire",
        min_gap=0.16,
        name="tray sits clearly above the front wheel",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem="left_fork_plate",
        negative_elem="tire",
        min_gap=0.012,
        max_gap=0.03,
        name="left fork plate clears the tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="right_fork_plate",
        min_gap=0.012,
        max_gap=0.03,
        name="right fork plate clears the tire",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
