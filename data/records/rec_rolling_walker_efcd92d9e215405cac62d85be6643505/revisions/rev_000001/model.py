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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_caster_walker")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.65, 0.67, 0.70, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_tip = model.material("rubber_tip", rgba=(0.16, 0.16, 0.17, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.70, 0.72, 0.75, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.58, 0.94)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.01, 0.47)),
    )

    tube_radius = 0.011
    side_path = [
        (0.255, -0.205, 0.020),
        (0.255, -0.205, 0.520),
        (0.255, -0.095, 0.635),
        (0.255, -0.060, 0.895),
        (0.255, 0.085, 0.895),
        (0.255, 0.085, 0.640),
        (0.255, 0.155, 0.185),
    ]
    frame.visual(
        _mesh(
            "walker_left_side_frame",
            tube_from_spline_points(
                side_path,
                radius=tube_radius,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_aluminum,
        name="left_side_frame",
    )
    frame.visual(
        _mesh(
            "walker_right_side_frame",
            tube_from_spline_points(
                _mirror_x(side_path),
                radius=tube_radius,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_aluminum,
        name="right_side_frame",
    )

    frame.visual(
        Cylinder(radius=0.017, length=0.085),
        origin=Origin(xyz=(0.255, -0.020, 0.895), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_hand_grip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.085),
        origin=Origin(xyz=(-0.255, -0.020, 0.895), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_hand_grip",
    )

    for name, center in (
        ("front_crossbar", (0.0, 0.085, 0.640)),
        ("rear_crossbar", (0.0, -0.090, 0.635)),
    ):
        frame.visual(
            Cylinder(radius=0.010, length=0.490),
            origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_aluminum,
            name=name,
        )

    tray_profile = rounded_rect_profile(0.345, 0.235, 0.020, corner_segments=8)
    frame.visual(
        _mesh("walker_tray_floor", ExtrudeGeometry.from_z0(tray_profile, 0.003)),
        origin=Origin(xyz=(0.0, -0.002, 0.628)),
        material=tray_gray,
        name="tray_floor",
    )
    frame.visual(
        Box((0.345, 0.004, 0.045)),
        origin=Origin(xyz=(0.0, 0.1135, 0.6505)),
        material=tray_gray,
        name="tray_front_wall",
    )
    frame.visual(
        Box((0.345, 0.004, 0.045)),
        origin=Origin(xyz=(0.0, -0.1175, 0.6505)),
        material=tray_gray,
        name="tray_rear_wall",
    )
    frame.visual(
        Box((0.004, 0.227, 0.045)),
        origin=Origin(xyz=(0.1705, -0.002, 0.6505)),
        material=tray_gray,
        name="tray_left_wall",
    )
    frame.visual(
        Box((0.004, 0.227, 0.045)),
        origin=Origin(xyz=(-0.1705, -0.002, 0.6505)),
        material=tray_gray,
        name="tray_right_wall",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.255, -0.205, 0.021)),
        material=rubber_tip,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(-0.255, -0.205, 0.021)),
        material=rubber_tip,
        name="right_rear_tip",
    )
    frame.visual(
        Box((0.022, 0.038, 0.012)),
        origin=Origin(xyz=(0.255, 0.170, 0.181)),
        material=frame_aluminum,
        name="left_caster_mount",
    )
    frame.visual(
        Box((0.022, 0.038, 0.012)),
        origin=Origin(xyz=(-0.255, 0.170, 0.181)),
        material=frame_aluminum,
        name="right_caster_mount",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        caster_swivel = model.part(f"{side_name}_caster_swivel")
        caster_swivel.inertial = Inertial.from_geometry(
            Box((0.040, 0.070, 0.130)),
            mass=0.35,
            origin=Origin(xyz=(0.0, -0.018, -0.065)),
        )
        caster_swivel.visual(
            Cylinder(radius=0.008, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, -0.032)),
            material=caster_steel,
            name="stem",
        )
        caster_swivel.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=caster_steel,
            name="swivel_housing",
        )
        caster_swivel.visual(
            Box((0.028, 0.022, 0.012)),
            origin=Origin(xyz=(0.0, -0.010, -0.044)),
            material=caster_steel,
            name="fork_bridge",
        )
        caster_swivel.visual(
            Box((0.012, 0.038, 0.022)),
            origin=Origin(xyz=(0.0, -0.024, -0.059)),
            material=caster_steel,
            name="fork_spine",
        )
        caster_swivel.visual(
            Box((0.006, 0.016, 0.084)),
            origin=Origin(xyz=(0.0135 * side_sign, -0.028, -0.084)),
            material=caster_steel,
            name="outer_tine",
        )
        caster_swivel.visual(
            Box((0.006, 0.016, 0.084)),
            origin=Origin(xyz=(-0.0135 * side_sign, -0.028, -0.084)),
            material=caster_steel,
            name="inner_tine",
        )

        caster_wheel = model.part(f"{side_name}_caster_wheel")
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.055, length=0.018),
            mass=0.22,
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        )
        caster_wheel.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        caster_wheel.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_gray,
            name="rim",
        )
        caster_wheel.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=caster_steel,
            name="hub",
        )

        model.articulation(
            f"{side_name}_caster_swivel",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=caster_swivel,
            origin=Origin(xyz=(0.255 * side_sign, 0.185, 0.175)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=-pi, upper=pi),
        )
        model.articulation(
            f"{side_name}_caster_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster_swivel,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, -0.028, -0.120)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_caster_wheel")
    right_wheel = object_model.get_part("right_caster_wheel")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    left_spin = object_model.get_articulation("left_caster_wheel_spin")

    tray_aabb = ctx.part_element_world_aabb(frame, elem="tray_floor")
    left_wheel_aabb = ctx.part_world_aabb(left_wheel)
    right_wheel_aabb = ctx.part_world_aabb(right_wheel)
    left_tip_aabb = ctx.part_element_world_aabb(frame, elem="left_rear_tip")
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem="right_rear_tip")

    tray_width = None if tray_aabb is None else tray_aabb[1][0] - tray_aabb[0][0]
    wheel_diameter = None if left_wheel_aabb is None else left_wheel_aabb[1][2] - left_wheel_aabb[0][2]
    ctx.check(
        "caster wheels stay small relative to tray",
        tray_width is not None and wheel_diameter is not None and wheel_diameter < tray_width * 0.40,
        details=f"tray_width={tray_width}, wheel_diameter={wheel_diameter}",
    )

    left_ground = None if left_wheel_aabb is None else left_wheel_aabb[0][2]
    right_ground = None if right_wheel_aabb is None else right_wheel_aabb[0][2]
    left_tip_ground = None if left_tip_aabb is None else left_tip_aabb[0][2]
    right_tip_ground = None if right_tip_aabb is None else right_tip_aabb[0][2]
    ctx.check(
        "front casters and rear tips share a plausible ground plane",
        left_ground is not None
        and right_ground is not None
        and left_tip_ground is not None
        and right_tip_ground is not None
        and abs(left_ground - left_tip_ground) <= 0.008
        and abs(right_ground - right_tip_ground) <= 0.008,
        details=(
            f"left_wheel_zmin={left_ground}, right_wheel_zmin={right_ground}, "
            f"left_tip_zmin={left_tip_ground}, right_tip_zmin={right_tip_ground}"
        ),
    )

    left_rest = ctx.part_world_position(left_wheel)
    with ctx.pose({left_swivel: pi / 2.0}):
        left_swiveled = ctx.part_world_position(left_wheel)
    ctx.check(
        "caster swivel redirects the wheel trail",
        left_rest is not None
        and left_swiveled is not None
        and abs(left_swiveled[0] - left_rest[0]) > 0.020
        and abs(left_swiveled[1] - left_rest[1]) > 0.020,
        details=f"rest={left_rest}, swiveled={left_swiveled}",
    )

    spin_rest = ctx.part_world_position(left_wheel)
    with ctx.pose({left_spin: 1.30}):
        spin_pose = ctx.part_world_position(left_wheel)
    ctx.check(
        "wheel spin stays centered on its axle",
        spin_rest is not None
        and spin_pose is not None
        and max(abs(a - b) for a, b in zip(spin_rest, spin_pose)) <= 1e-6,
        details=f"rest={spin_rest}, spun={spin_pose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
