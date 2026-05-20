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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _tube(points: list[tuple[float, float, float]], *, radius: float, name: str, samples: int = 12):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=18,
        ),
        name,
    )


def _union_aabb(bounds):
    valid = [bound for bound in bounds if bound is not None]
    if not valid:
        return None
    mins = [min(bound[0][axis] for bound in valid) for axis in range(3)]
    maxs = [max(bound[1][axis] for bound in valid) for axis in range(3)]
    return (tuple(mins), tuple(maxs))


def _extent(aabb, axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def _mid(aabb, axis: int) -> float:
    return 0.5 * (aabb[0][axis] + aabb[1][axis])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_wheelbarrow")

    tray_finish = model.material("tray_finish", rgba=(0.42, 0.49, 0.36, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.18, 0.19, 0.20, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    foot_pad = model.material("foot_pad", rgba=(0.13, 0.13, 0.14, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 0.10)),
        mass=4.2,
        origin=Origin(xyz=(0.00, 0.00, 0.00)),
    )
    frame.visual(
        Box((0.24, 0.10, 0.030)),
        origin=Origin(xyz=(0.02, 0.00, -0.010)),
        material=frame_finish,
        name="center_spine",
    )
    frame.visual(
        Box((0.100, 0.180, 0.018)),
        origin=Origin(xyz=(-0.100, 0.000, -0.014)),
        material=frame_finish,
        name="rear_bridge",
    )
    frame.visual(
        _tube(
            [(-0.100, 0.065, -0.014), (-0.015, 0.090, 0.000), (0.120, 0.050, -0.008)],
            radius=0.009,
            name="frame_left_rail",
            samples=10,
        ),
        material=frame_finish,
        name="left_rail",
    )
    frame.visual(
        _tube(
            _mirror_y([(-0.100, 0.065, -0.014), (-0.015, 0.090, 0.000), (0.120, 0.050, -0.008)]),
            radius=0.009,
            name="frame_right_rail",
            samples=10,
        ),
        material=frame_finish,
        name="right_rail",
    )
    frame.visual(
        Box((0.110, 0.052, 0.020)),
        origin=Origin(xyz=(-0.055, 0.085, 0.010)),
        material=frame_finish,
        name="left_tray_arm",
    )
    frame.visual(
        Box((0.110, 0.052, 0.020)),
        origin=Origin(xyz=(-0.055, -0.085, 0.010)),
        material=frame_finish,
        name="right_tray_arm",
    )
    frame.visual(
        Box((0.064, 0.090, 0.020)),
        origin=Origin(xyz=(-0.092, 0.132, 0.000)),
        material=frame_finish,
        name="left_handle_strut",
    )
    frame.visual(
        Box((0.064, 0.090, 0.020)),
        origin=Origin(xyz=(-0.092, -0.132, 0.000)),
        material=frame_finish,
        name="right_handle_strut",
    )
    frame.visual(
        Box((0.030, 0.100, 0.040)),
        origin=Origin(xyz=(0.130, 0.000, 0.000)),
        material=frame_finish,
        name="fork_mount",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.36, 0.26, 0.11)),
        mass=2.4,
        origin=Origin(xyz=(-0.170, 0.000, 0.055)),
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(xyz=(0.000, 0.123, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_hinge_sleeve",
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(xyz=(0.000, -0.123, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_hinge_sleeve",
    )
    tray.visual(
        Box((0.026, 0.020, 0.032)),
        origin=Origin(xyz=(-0.018, 0.102, 0.016)),
        material=frame_finish,
        name="left_hinge_cheek",
    )
    tray.visual(
        Box((0.026, 0.020, 0.032)),
        origin=Origin(xyz=(-0.018, -0.102, 0.016)),
        material=frame_finish,
        name="right_hinge_cheek",
    )
    tray.visual(
        Box((0.330, 0.210, 0.005)),
        origin=Origin(xyz=(-0.185, 0.000, 0.028)),
        material=tray_finish,
        name="tray_floor",
    )
    tray.visual(
        Box((0.008, 0.150, 0.070)),
        origin=Origin(xyz=(-0.022, 0.000, 0.061)),
        material=tray_finish,
        name="front_wall",
    )
    tray.visual(
        Box((0.008, 0.210, 0.062)),
        origin=Origin(xyz=(-0.346, 0.000, 0.056)),
        material=tray_finish,
        name="rear_wall",
    )
    tray.visual(
        Box((0.330, 0.008, 0.062)),
        origin=Origin(xyz=(-0.185, 0.089, 0.056)),
        material=tray_finish,
        name="left_wall",
    )
    tray.visual(
        Box((0.330, 0.008, 0.062)),
        origin=Origin(xyz=(-0.185, -0.089, 0.056)),
        material=tray_finish,
        name="right_wall",
    )
    tray.visual(
        Box((0.330, 0.010, 0.008)),
        origin=Origin(xyz=(-0.185, 0.089, 0.088)),
        material=tray_finish,
        name="left_rim",
    )
    tray.visual(
        Box((0.330, 0.010, 0.008)),
        origin=Origin(xyz=(-0.185, -0.089, 0.088)),
        material=tray_finish,
        name="right_rim",
    )
    tray.visual(
        Box((0.010, 0.210, 0.008)),
        origin=Origin(xyz=(-0.020, 0.000, 0.088)),
        material=tray_finish,
        name="front_rim",
    )
    tray.visual(
        Box((0.022, 0.220, 0.010)),
        origin=Origin(xyz=(-0.348, 0.000, 0.081)),
        material=tray_finish,
        name="rear_dump_lip",
    )

    handles = model.part("handles")
    handles.inertial = Inertial.from_geometry(
        Box((0.42, 0.38, 0.18)),
        mass=1.8,
        origin=Origin(xyz=(-0.180, 0.000, 0.070)),
    )
    handles.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, 0.189, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_pivot_sleeve",
    )
    handles.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, -0.189, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_pivot_sleeve",
    )
    handles.visual(
        _tube(
            [(0.000, 0.189, 0.000), (-0.090, 0.186, 0.028), (-0.180, 0.190, 0.052), (-0.275, 0.205, 0.088), (-0.370, 0.215, 0.120)],
            radius=0.010,
            name="left_handle_tube",
            samples=14,
        ),
        material=frame_finish,
        name="left_handle_tube",
    )
    handles.visual(
        _tube(
            _mirror_y([(0.000, 0.189, 0.000), (-0.090, 0.186, 0.028), (-0.180, 0.190, 0.052), (-0.275, 0.205, 0.088), (-0.370, 0.215, 0.120)]),
            radius=0.010,
            name="right_handle_tube",
            samples=14,
        ),
        material=frame_finish,
        name="right_handle_tube",
    )
    handles.visual(
        _tube(
            [(-0.370, -0.215, 0.120), (-0.370, 0.215, 0.120)],
            radius=0.010,
            name="rear_grip_bridge",
            samples=2,
        ),
        material=frame_finish,
        name="rear_grip_bridge",
    )
    handles.visual(
        _tube(
            [(-0.120, 0.176, 0.050), (-0.165, 0.137, 0.042)],
            radius=0.008,
            name="left_leg_link",
            samples=2,
        ),
        material=frame_finish,
        name="left_leg_link",
    )
    handles.visual(
        _tube(
            _mirror_y([(-0.120, 0.176, 0.050), (-0.165, 0.137, 0.042)]),
            radius=0.008,
            name="right_leg_link",
            samples=2,
        ),
        material=frame_finish,
        name="right_leg_link",
    )
    handles.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(-0.370, 0.215, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    handles.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(xyz=(-0.370, -0.215, 0.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    rear_legs = model.part("rear_legs")
    rear_legs.inertial = Inertial.from_geometry(
        Box((0.18, 0.28, 0.22)),
        mass=1.1,
        origin=Origin(xyz=(-0.060, 0.000, -0.070)),
    )
    rear_legs.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, 0.113, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_leg_sleeve",
    )
    rear_legs.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.000, -0.113, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_leg_sleeve",
    )
    rear_legs.visual(
        _tube(
            [(0.000, 0.113, 0.000), (-0.030, 0.122, -0.090), (-0.078, 0.140, -0.165)],
            radius=0.009,
            name="left_leg_tube",
            samples=10,
        ),
        material=frame_finish,
        name="left_leg_tube",
    )
    rear_legs.visual(
        _tube(
            _mirror_y([(0.000, 0.113, 0.000), (-0.030, 0.122, -0.090), (-0.078, 0.140, -0.165)]),
            radius=0.009,
            name="right_leg_tube",
            samples=10,
        ),
        material=frame_finish,
        name="right_leg_tube",
    )
    rear_legs.visual(
        _tube(
            [(-0.062, -0.135, -0.145), (-0.062, 0.135, -0.145)],
            radius=0.008,
            name="leg_spreader",
            samples=2,
        ),
        material=frame_finish,
        name="leg_spreader",
    )
    rear_legs.visual(
        Box((0.035, 0.018, 0.012)),
        origin=Origin(xyz=(-0.078, 0.140, -0.165)),
        material=foot_pad,
        name="left_foot",
    )
    rear_legs.visual(
        Box((0.035, 0.018, 0.012)),
        origin=Origin(xyz=(-0.078, -0.140, -0.165)),
        material=foot_pad,
        name="right_foot",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.16, 0.12, 0.08)),
        mass=1.3,
        origin=Origin(xyz=(0.080, 0.000, -0.012)),
    )
    fork.visual(
        Box((0.024, 0.080, 0.035)),
        origin=Origin(xyz=(0.012, 0.000, 0.000)),
        material=frame_finish,
        name="fork_crown",
    )
    fork.visual(
        _tube(
            [(0.015, 0.048, 0.000), (0.070, 0.048, -0.006), (0.105, 0.048, -0.015)],
            radius=0.008,
            name="left_fork_tine",
            samples=8,
        ),
        material=frame_finish,
        name="left_fork_tine",
    )
    fork.visual(
        _tube(
            _mirror_y([(0.015, 0.048, 0.000), (0.070, 0.048, -0.006), (0.105, 0.048, -0.015)]),
            radius=0.008,
            name="right_fork_tine",
            samples=8,
        ),
        material=frame_finish,
        name="right_fork_tine",
    )
    fork.visual(
        Box((0.018, 0.016, 0.022)),
        origin=Origin(xyz=(0.105, 0.058, -0.015)),
        material=frame_finish,
        name="left_dropout",
    )
    fork.visual(
        Box((0.018, 0.016, 0.022)),
        origin=Origin(xyz=(0.105, -0.058, -0.015)),
        material=frame_finish,
        name="right_dropout",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.058),
        mass=1.5,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.085, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.062, length=0.046),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="axle",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_handles",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handles,
        origin=Origin(xyz=(-0.060, 0.000, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "handles_to_rear_legs",
        ArticulationType.REVOLUTE,
        parent=handles,
        child=rear_legs,
        origin=Origin(xyz=(-0.160, 0.000, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "frame_to_fork",
        ArticulationType.FIXED,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(0.145, 0.000, 0.000)),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.105, 0.000, -0.015)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    handles = object_model.get_part("handles")
    rear_legs = object_model.get_part("rear_legs")
    fork = object_model.get_part("fork")
    wheel = object_model.get_part("wheel")

    tray_hinge = object_model.get_articulation("frame_to_tray")
    handle_hinge = object_model.get_articulation("frame_to_handles")
    leg_hinge = object_model.get_articulation("handles_to_rear_legs")
    wheel_spin = object_model.get_articulation("fork_to_wheel")

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

    ctx.expect_contact(tray, frame, name="tray remains physically mounted to frame")
    ctx.expect_contact(handles, frame, name="handles remain physically mounted to frame")
    ctx.expect_contact(rear_legs, handles, name="rear legs remain pinned to handles")
    ctx.expect_contact(fork, frame, name="fork remains mounted to front frame")
    ctx.expect_contact(wheel, fork, name="wheel remains captured by fork dropouts")
    ctx.expect_gap(
        wheel,
        tray,
        axis="x",
        min_gap=0.10,
        name="front wheel clears tray in working pose",
    )

    frame_pos = ctx.part_world_position(frame)
    tray_rest = ctx.part_element_world_aabb(tray, elem="rear_dump_lip")
    handles_rest = ctx.part_element_world_aabb(handles, elem="left_grip")
    tray_upper = tray_hinge.motion_limits.upper if tray_hinge.motion_limits else None
    handle_upper = handle_hinge.motion_limits.upper if handle_hinge.motion_limits else None
    leg_upper = leg_hinge.motion_limits.upper if leg_hinge.motion_limits else None

    with ctx.pose({tray_hinge: tray_upper}):
        tray_folded = ctx.part_element_world_aabb(tray, elem="rear_dump_lip")
    ctx.check(
        "tray folds upward for dump or storage",
        tray_rest is not None
        and tray_folded is not None
        and _mid(tray_folded, 2) > _mid(tray_rest, 2) + 0.18,
        details=f"tray z rest={tray_rest} folded={tray_folded}",
    )

    with ctx.pose({handle_hinge: handle_upper}):
        handles_folded = ctx.part_element_world_aabb(handles, elem="left_grip")
    ctx.check(
        "handles fold forward and upward",
        handles_rest is not None
        and handles_folded is not None
        and _mid(handles_folded, 2) > _mid(handles_rest, 2) + 0.14
        and _mid(handles_folded, 0) > _mid(handles_rest, 0) + 0.08,
        details=f"handles rest={handles_rest} folded={handles_folded}",
    )

    left_foot_rest = ctx.part_element_world_aabb(rear_legs, elem="left_foot")
    tire_rest = ctx.part_element_world_aabb(wheel, elem="tire")
    stance_ok = (
        left_foot_rest is not None
        and tire_rest is not None
        and abs(left_foot_rest[0][2] - tire_rest[0][2]) <= 0.015
    )
    ctx.check(
        "rear feet and front wheel share a stable stance plane",
        stance_ok,
        details=f"left_foot_bottom={left_foot_rest} tire_bottom={tire_rest}",
    )

    with ctx.pose({wheel_spin: 1.4}):
        ctx.expect_contact(wheel, fork, name="wheel stays captured while spinning")

    with ctx.pose({handle_hinge: handle_upper, leg_hinge: leg_upper}):
        left_foot_stowed = ctx.part_element_world_aabb(rear_legs, elem="left_foot")
    ctx.check(
        "rear legs lift when stowed",
        left_foot_rest is not None
        and left_foot_stowed is not None
        and left_foot_stowed[0][2] > left_foot_rest[0][2] + 0.09
        and _mid(left_foot_stowed, 0) > _mid(left_foot_rest, 0) + 0.15,
        details=f"rest={left_foot_rest} stowed={left_foot_stowed}",
    )

    working_aabb = _union_aabb(
        [
            ctx.part_world_aabb(frame),
            ctx.part_world_aabb(tray),
            ctx.part_world_aabb(handles),
            ctx.part_world_aabb(rear_legs),
            ctx.part_world_aabb(fork),
            ctx.part_world_aabb(wheel),
        ]
    )
    with ctx.pose({tray_hinge: tray_upper, handle_hinge: handle_upper, leg_hinge: leg_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_stowed_pose")
        ctx.expect_gap(
            wheel,
            rear_legs,
            axis="x",
            min_gap=0.02,
            name="folded rear legs stay clear of front wheel",
        )
        stowed_aabb = _union_aabb(
            [
                ctx.part_world_aabb(frame),
                ctx.part_world_aabb(tray),
                ctx.part_world_aabb(handles),
                ctx.part_world_aabb(rear_legs),
                ctx.part_world_aabb(fork),
                ctx.part_world_aabb(wheel),
            ]
        )
    ctx.check(
        "folded pose shortens overall length",
        working_aabb is not None
        and stowed_aabb is not None
        and _extent(stowed_aabb, 0) < _extent(working_aabb, 0) - 0.18,
        details=f"working={working_aabb} stowed={stowed_aabb}",
    )
    ctx.check(
        "primary hinge axes stay lateral to the tray",
        tray_hinge.axis == (0.0, 1.0, 0.0)
        and handle_hinge.axis == (0.0, 1.0, 0.0)
        and leg_hinge.axis == (0.0, -1.0, 0.0)
        and wheel_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"tray_axis={tray_hinge.axis}, handle_axis={handle_hinge.axis}, "
            f"leg_axis={leg_hinge.axis}, wheel_axis={wheel_spin.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
