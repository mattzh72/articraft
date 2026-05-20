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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _frame_member(name: str, points: list[tuple[float, float, float]], *, width: float, height: float):
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.28,
        corner_segments=6,
    )
    return _save_mesh(
        name,
        sweep_profile_along_spline(
            points,
            profile=profile,
            samples_per_segment=14,
            cap_profile=True,
        ),
    )


def _wheel_visuals(part, *, tire_outer_radius: float, tire_width: float, steel, dark_steel, rubber) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    tire_mesh = _save_mesh(
        "wheelbarrow_tire",
        TorusGeometry(
            radius=tire_outer_radius - 0.045,
            tube=0.045,
            radial_segments=20,
            tubular_segments=40,
        ).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=tire_outer_radius * 0.63, length=tire_width * 0.54),
        origin=spin_origin,
        material=steel,
        name="rim_drum",
    )
    part.visual(
        Cylinder(radius=tire_outer_radius * 0.46, length=tire_width * 0.30),
        origin=Origin(xyz=(tire_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_rim_face",
    )
    part.visual(
        Cylinder(radius=tire_outer_radius * 0.46, length=tire_width * 0.30),
        origin=Origin(xyz=(-tire_width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_rim_face",
    )
    part.visual(
        Cylinder(radius=tire_outer_radius * 0.18, length=tire_width * 1.15),
        origin=spin_origin,
        material=dark_steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=tire_outer_radius * 0.10, length=tire_width * 1.40),
        origin=spin_origin,
        material=steel,
        name="axle_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.27, 0.39, 0.18, 1.0))
    frame_green = model.material("frame_green", rgba=(0.22, 0.33, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.08, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.88, 1.72, 0.72)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.03, 0.33)),
    )

    frame.visual(
        Box((0.72, 0.68, 0.028)),
        origin=Origin(xyz=(0.0, 0.04, 0.35)),
        material=tray_green,
        name="tray_floor",
    )
    frame.visual(
        Box((0.024, 0.80, 0.24)),
        origin=Origin(xyz=(0.304, 0.02, 0.455), rpy=(0.0, -0.18, 0.0)),
        material=tray_green,
        name="left_tray_wall",
    )
    frame.visual(
        Box((0.024, 0.80, 0.24)),
        origin=Origin(xyz=(-0.304, 0.02, 0.455), rpy=(0.0, 0.18, 0.0)),
        material=tray_green,
        name="right_tray_wall",
    )
    frame.visual(
        Box((0.66, 0.024, 0.25)),
        origin=Origin(xyz=(0.0, 0.40, 0.47), rpy=(0.12, 0.0, 0.0)),
        material=tray_green,
        name="tray_front_wall",
    )
    frame.visual(
        Box((0.62, 0.040, 0.10)),
        origin=Origin(xyz=(0.0, -0.29, 0.39), rpy=(-0.05, 0.0, 0.0)),
        material=tray_green,
        name="tray_rear_lip",
    )
    frame.visual(
        Box((0.30, 0.34, 0.12)),
        origin=Origin(xyz=(0.0, 0.20, 0.28)),
        material=frame_green,
        name="tray_support_block",
    )
    frame.visual(
        Box((0.14, 0.22, 0.09)),
        origin=Origin(xyz=(0.0, 0.29, 0.23)),
        material=frame_green,
        name="fork_spine",
    )
    frame.visual(
        _frame_member(
            "left_handle_rail",
            [
                (0.28, 0.36, 0.26),
                (0.31, 0.12, 0.28),
                (0.33, -0.28, 0.36),
                (0.34, -0.77, 0.63),
            ],
            width=0.060,
            height=0.040,
        ),
        material=frame_green,
        name="left_handle",
    )
    frame.visual(
        _frame_member(
            "right_handle_rail",
            [
                (-0.28, 0.36, 0.26),
                (-0.31, 0.12, 0.28),
                (-0.33, -0.28, 0.36),
                (-0.34, -0.77, 0.63),
            ],
            width=0.060,
            height=0.040,
        ),
        material=frame_green,
        name="right_handle",
    )
    frame.visual(
        _frame_member(
            "left_rear_leg",
            [
                (0.28, -0.26, 0.37),
                (0.27, -0.40, 0.23),
                (0.26, -0.52, 0.10),
                (0.25, -0.59, 0.012),
            ],
            width=0.050,
            height=0.035,
        ),
        material=frame_green,
        name="left_leg",
    )
    frame.visual(
        _frame_member(
            "right_rear_leg",
            [
                (-0.28, -0.26, 0.37),
                (-0.27, -0.40, 0.23),
                (-0.26, -0.52, 0.10),
                (-0.25, -0.59, 0.012),
            ],
            width=0.050,
            height=0.035,
        ),
        material=frame_green,
        name="right_leg",
    )
    frame.visual(
        Box((0.50, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.44, 0.20)),
        material=frame_green,
        name="rear_leg_crossbrace",
    )
    frame.visual(
        Box((0.16, 0.18, 0.08)),
        origin=Origin(xyz=(0.27, 0.03, 0.29)),
        material=frame_green,
        name="left_tray_mount",
    )
    frame.visual(
        Box((0.16, 0.18, 0.08)),
        origin=Origin(xyz=(-0.27, 0.03, 0.29)),
        material=frame_green,
        name="right_tray_mount",
    )
    frame.visual(
        Box((0.090, 0.30, 0.095)),
        origin=Origin(xyz=(0.11, 0.49, 0.235), rpy=(-0.23, 0.0, 0.0)),
        material=frame_green,
        name="left_fork_arm",
    )
    frame.visual(
        Box((0.090, 0.30, 0.095)),
        origin=Origin(xyz=(-0.11, 0.49, 0.235), rpy=(-0.23, 0.0, 0.0)),
        material=frame_green,
        name="right_fork_arm",
    )
    frame.visual(
        Box((0.080, 0.12, 0.11)),
        origin=Origin(xyz=(0.11, 0.62, 0.19)),
        material=dark_steel,
        name="left_axle_block",
    )
    frame.visual(
        Box((0.080, 0.12, 0.11)),
        origin=Origin(xyz=(-0.11, 0.62, 0.19)),
        material=dark_steel,
        name="right_axle_block",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(0.34, -0.76, 0.63), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(-0.34, -0.76, 0.63), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.10),
        mass=4.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        wheel,
        tire_outer_radius=0.19,
        tire_width=0.10,
        steel=steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.62, 0.19)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="left_axle_block",
        negative_elem="tire",
        min_gap=0.020,
        max_gap=0.040,
        name="left axle block clears the tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="tire",
        negative_elem="right_axle_block",
        min_gap=0.020,
        max_gap=0.040,
        name="right axle block clears the tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="tray_front_wall",
        min_gap=0.0,
        max_gap=0.012,
        name="wheel tucks tightly under the tray nose",
    )

    wheel_aabb = ctx.part_world_aabb(wheel)
    left_leg_aabb = ctx.part_element_world_aabb(frame, elem="left_leg")
    right_leg_aabb = ctx.part_element_world_aabb(frame, elem="right_leg")
    ctx.check(
        "wheel and rear legs rest on the same ground plane",
        wheel_aabb is not None
        and left_leg_aabb is not None
        and right_leg_aabb is not None
        and abs(wheel_aabb[0][2]) <= 0.001
        and abs(left_leg_aabb[0][2] - wheel_aabb[0][2]) <= 0.01
        and abs(right_leg_aabb[0][2] - wheel_aabb[0][2]) <= 0.01,
        details=f"wheel_min_z={None if wheel_aabb is None else wheel_aabb[0][2]}, "
        f"left_leg_min_z={None if left_leg_aabb is None else left_leg_aabb[0][2]}, "
        f"right_leg_min_z={None if right_leg_aabb is None else right_leg_aabb[0][2]}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: pi / 2.0}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins in place about a fixed axle",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) <= 1e-6,
        details=f"rest_pos={rest_pos}, spun_pos={spun_pos}",
    )

    limits = wheel_spin.motion_limits
    ctx.check(
        "front wheel uses a continuous x-axis spin joint",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}, limits={limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
