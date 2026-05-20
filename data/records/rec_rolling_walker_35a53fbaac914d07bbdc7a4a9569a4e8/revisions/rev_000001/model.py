from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


TUBE_RADIUS = 0.012
BRACE_RADIUS = 0.009
GUSSET_RADIUS = 0.008
CASTER_SWIVEL_Z = 0.18
CASTER_AXLE_DROP = 0.125


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _bolt_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_bolts(
    part,
    *,
    prefix: str,
    positions: list[tuple[float, float, float]],
    axis: str,
    radius: float,
    length: float,
    material,
) -> None:
    for index, position in enumerate(positions):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=position, rpy=_bolt_rpy(axis)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _build_side_tubing_mesh(
    name: str,
    *,
    loop_points: list[tuple[float, float, float]],
):
    geom = tube_from_spline_points(
        loop_points,
        radius=TUBE_RADIUS,
        samples_per_segment=18,
        radial_segments=18,
    )
    return _save_mesh(name, geom)


def _build_brace_mesh(name: str, points: list[tuple[float, float, float]], radius: float = BRACE_RADIUS):
    return _save_mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=16,
        ),
    )


def _build_caster_fork_mesh(name: str):
    arm_center_x = 0.0155
    geom = CylinderGeometry(radius=0.009, height=0.060).translate(0.0, 0.0, -0.030)
    geom.merge(BoxGeometry((0.040, 0.030, 0.020)).translate(0.0, 0.0, -0.070))
    geom.merge(BoxGeometry((0.006, 0.030, 0.110)).translate(arm_center_x, 0.0, -0.115))
    geom.merge(BoxGeometry((0.006, 0.030, 0.110)).translate(-arm_center_x, 0.0, -0.115))
    geom.merge(
        CylinderGeometry(radius=0.010, height=0.006)
        .rotate_y(pi / 2.0)
        .translate(0.0215, 0.0, -0.125)
    )
    geom.merge(
        CylinderGeometry(radius=0.010, height=0.006)
        .rotate_y(pi / 2.0)
        .translate(-0.0215, 0.0, -0.125)
    )
    return _save_mesh(name, geom)


def _build_caster_wheel_mesh(name: str):
    geom = TorusGeometry(radius=0.043, tube=0.012, radial_segments=16, tubular_segments=40).rotate_y(
        pi / 2.0
    )
    geom.merge(CylinderGeometry(radius=0.031, height=0.016).rotate_y(pi / 2.0))
    geom.merge(CylinderGeometry(radius=0.018, height=0.025).rotate_y(pi / 2.0))
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_rolling_walker")

    frame_paint = model.material("frame_paint", rgba=(0.66, 0.69, 0.63, 1.0))
    heritage_red = model.material("heritage_red", rgba=(0.56, 0.19, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.05, 1.0))

    left_loop = [
        (-0.272, -0.185, 0.030),
        (-0.272, -0.158, 0.190),
        (-0.270, -0.105, 0.595),
        (-0.265, -0.025, 0.888),
        (-0.248, 0.080, 0.888),
        (-0.230, 0.180, 0.848),
        (-0.220, 0.220, 0.220),
    ]
    left_lower_bridge = [
        (-0.279, -0.138, 0.360),
        (-0.160, -0.094, 0.374),
        (-0.008, -0.030, 0.395),
    ]
    left_front_gusset = [
        (-0.220, 0.205, 0.248),
        (-0.145, 0.130, 0.330),
        (-0.004, 0.020, 0.430),
    ]

    right_loop = _mirror_x(left_loop)
    right_lower_bridge = [(0.008, -0.030, 0.395), (0.160, -0.094, 0.374), (0.279, -0.138, 0.360)]
    right_front_gusset = [(0.004, 0.020, 0.430), (0.145, 0.130, 0.330), (0.220, 0.205, 0.248)]

    frame_core = model.part("frame_core")
    frame_core.visual(
        _build_side_tubing_mesh(
            "frame_core_tubing",
            loop_points=left_loop,
        ),
        material=frame_paint,
        name="left_side_tubing",
    )
    frame_core.visual(
        Box((0.244, 0.040, 0.040)),
        origin=Origin(xyz=(-0.126, 0.070, 0.850)),
        material=dark_steel,
        name="left_upper_bridge_bar",
    )
    frame_core.visual(
        _build_brace_mesh("left_lower_brace_mesh", left_lower_bridge),
        material=frame_paint,
        name="left_lower_brace",
    )
    frame_core.visual(
        _build_brace_mesh("left_front_gusset_mesh", left_front_gusset, GUSSET_RADIUS),
        material=frame_paint,
        name="left_front_gusset_brace",
    )
    frame_core.visual(
        Box((0.032, 0.075, 0.460)),
        origin=Origin(xyz=(-0.016, 0.000, 0.450)),
        material=dark_steel,
        name="hinge_spine",
    )
    frame_core.visual(
        Box((0.055, 0.110, 0.160)),
        origin=Origin(xyz=(-0.0275, 0.060, 0.760)),
        material=dark_steel,
        name="upper_hinge_housing",
    )
    frame_core.visual(
        Box((0.050, 0.090, 0.100)),
        origin=Origin(xyz=(-0.025, -0.030, 0.395)),
        material=dark_steel,
        name="lower_lock_housing",
    )
    frame_core.visual(
        Box((0.004, 0.070, 0.100)),
        origin=Origin(xyz=(-0.052, -0.030, 0.395)),
        material=heritage_red,
        name="left_service_hatch",
    )
    _add_bolts(
        frame_core,
        prefix="left_hatch_bolt",
        positions=[
            (-0.057, -0.055, 0.432),
            (-0.057, -0.005, 0.432),
            (-0.057, -0.055, 0.358),
            (-0.057, -0.005, 0.358),
        ],
        axis="x",
        radius=0.004,
        length=0.006,
        material=steel,
    )
    frame_core.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(-0.265, -0.025, 0.892), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame_core.visual(
        Box((0.030, 0.024, 0.034)),
        origin=Origin(xyz=(-0.249, -0.020, 0.892)),
        material=dark_steel,
        name="left_brake_adapter",
    )
    frame_core.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.274, -0.020, 0.892), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_brake_boss",
    )
    _add_bolts(
        frame_core,
        prefix="left_adapter_bolt",
        positions=[(-0.249, -0.031, 0.913), (-0.249, -0.009, 0.913)],
        axis="z",
        radius=0.0035,
        length=0.008,
        material=steel,
    )
    frame_core.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(-0.220, 0.220, 0.200)),
        material=dark_steel,
        name="left_caster_socket",
    )
    frame_core.visual(
        Box((0.024, 0.055, 0.016)),
        origin=Origin(xyz=(-0.220, 0.192, 0.228)),
        material=dark_steel,
        name="left_caster_adapter",
    )
    frame_core.visual(
        Box((0.018, 0.055, 0.090)),
        origin=Origin(xyz=(-0.210, 0.190, 0.268)),
        material=dark_steel,
        name="left_front_reinforcement",
    )
    frame_core.visual(
        Box((0.026, 0.045, 0.026)),
        origin=Origin(xyz=(-0.264, -0.128, 0.368)),
        material=dark_steel,
        name="left_lower_adapter_plate",
    )
    frame_core.visual(
        Box((0.276, 0.030, 0.032)),
        origin=Origin(xyz=(-0.138, -0.055, 0.374)),
        material=dark_steel,
        name="left_lower_reinforcement_strap",
    )
    frame_core.visual(
        Box((0.026, 0.052, 0.042)),
        origin=Origin(xyz=(-0.264, -0.112, 0.375)),
        material=dark_steel,
        name="left_lower_clamp_block",
    )
    frame_core.visual(
        Box((0.024, 0.118, 0.026)),
        origin=Origin(xyz=(-0.262, -0.142, 0.372)),
        material=dark_steel,
        name="left_lower_saddle_bracket",
    )
    _add_bolts(
        frame_core,
        prefix="left_caster_bolt",
        positions=[(-0.220, 0.176, 0.240), (-0.220, 0.208, 0.240)],
        axis="z",
        radius=0.0035,
        length=0.008,
        material=steel,
    )
    frame_core.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(-0.272, -0.185, 0.015)),
        material=rubber,
        name="left_rear_foot",
    )
    frame_core.inertial = Inertial.from_geometry(
        Box((0.320, 0.520, 0.940)),
        mass=3.9,
        origin=Origin(xyz=(-0.150, 0.000, 0.470)),
    )

    right_frame = model.part("right_frame")
    right_frame.visual(
        _build_side_tubing_mesh(
            "right_frame_tubing",
            loop_points=right_loop,
        ),
        material=frame_paint,
        name="right_side_tubing",
    )
    right_frame.visual(
        Box((0.244, 0.040, 0.040)),
        origin=Origin(xyz=(0.126, 0.070, 0.850)),
        material=dark_steel,
        name="right_upper_bridge_bar",
    )
    right_frame.visual(
        Box((0.022, 0.050, 0.240)),
        origin=Origin(xyz=(0.016, 0.060, 0.750)),
        material=dark_steel,
        name="right_upper_mount_block",
    )
    right_frame.visual(
        _build_brace_mesh("right_lower_brace_mesh", right_lower_bridge),
        material=frame_paint,
        name="right_lower_brace",
    )
    right_frame.visual(
        _build_brace_mesh("right_front_gusset_mesh", right_front_gusset, GUSSET_RADIUS),
        material=frame_paint,
        name="right_front_gusset_brace",
    )
    right_frame.visual(
        Box((0.010, 0.120, 0.460)),
        origin=Origin(xyz=(0.005, 0.000, 0.450)),
        material=dark_steel,
        name="hinge_leaf",
    )
    right_frame.visual(
        Box((0.004, 0.065, 0.120)),
        origin=Origin(xyz=(0.012, 0.000, 0.430)),
        material=heritage_red,
        name="right_service_hatch",
    )
    _add_bolts(
        right_frame,
        prefix="right_hatch_bolt",
        positions=[
            (0.017, -0.020, 0.472),
            (0.017, 0.020, 0.472),
            (0.017, -0.020, 0.388),
            (0.017, 0.020, 0.388),
        ],
        axis="x",
        radius=0.004,
        length=0.006,
        material=steel,
    )
    right_frame.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.265, -0.025, 0.892), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    right_frame.visual(
        Box((0.030, 0.024, 0.034)),
        origin=Origin(xyz=(0.249, -0.020, 0.892)),
        material=dark_steel,
        name="right_brake_adapter",
    )
    right_frame.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.274, -0.020, 0.892), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_brake_boss",
    )
    _add_bolts(
        right_frame,
        prefix="right_adapter_bolt",
        positions=[(0.249, -0.031, 0.913), (0.249, -0.009, 0.913)],
        axis="z",
        radius=0.0035,
        length=0.008,
        material=steel,
    )
    right_frame.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(0.220, 0.220, 0.200)),
        material=dark_steel,
        name="right_caster_socket",
    )
    right_frame.visual(
        Box((0.024, 0.055, 0.016)),
        origin=Origin(xyz=(0.220, 0.192, 0.228)),
        material=dark_steel,
        name="right_caster_adapter",
    )
    right_frame.visual(
        Box((0.018, 0.055, 0.090)),
        origin=Origin(xyz=(0.210, 0.190, 0.268)),
        material=dark_steel,
        name="right_front_reinforcement",
    )
    right_frame.visual(
        Box((0.026, 0.045, 0.026)),
        origin=Origin(xyz=(0.264, -0.128, 0.368)),
        material=dark_steel,
        name="right_lower_adapter_plate",
    )
    right_frame.visual(
        Box((0.276, 0.030, 0.032)),
        origin=Origin(xyz=(0.138, -0.055, 0.374)),
        material=dark_steel,
        name="right_lower_reinforcement_strap",
    )
    right_frame.visual(
        Box((0.026, 0.052, 0.042)),
        origin=Origin(xyz=(0.264, -0.112, 0.375)),
        material=dark_steel,
        name="right_lower_clamp_block",
    )
    right_frame.visual(
        Box((0.024, 0.118, 0.026)),
        origin=Origin(xyz=(0.262, -0.142, 0.372)),
        material=dark_steel,
        name="right_lower_saddle_bracket",
    )
    _add_bolts(
        right_frame,
        prefix="right_caster_bolt",
        positions=[(0.220, 0.176, 0.240), (0.220, 0.208, 0.240)],
        axis="z",
        radius=0.0035,
        length=0.008,
        material=steel,
    )
    right_frame.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.272, -0.185, 0.015)),
        material=rubber,
        name="right_rear_foot",
    )
    right_frame.inertial = Inertial.from_geometry(
        Box((0.320, 0.520, 0.940)),
        mass=3.8,
        origin=Origin(xyz=(0.150, 0.000, 0.470)),
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.visual(
        _build_caster_fork_mesh("left_caster_fork_mesh"),
        material=steel,
        name="left_caster_fork_shell",
    )
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.180)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, -0.090)),
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.visual(
        _build_caster_fork_mesh("right_caster_fork_mesh"),
        material=steel,
        name="right_caster_fork_shell",
    )
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.180)),
        mass=0.45,
        origin=Origin(xyz=(0.000, 0.000, -0.090)),
    )

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.visual(
        _build_caster_wheel_mesh("left_front_wheel_mesh"),
        material=tire_black,
        name="left_front_wheel_shell",
    )
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.028),
        mass=0.30,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.visual(
        _build_caster_wheel_mesh("right_front_wheel_mesh"),
        material=tire_black,
        name="right_front_wheel_shell",
    )
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.028),
        mass=0.30,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_fold",
        ArticulationType.REVOLUTE,
        parent=frame_core,
        child=right_frame,
        origin=Origin(),
        # Closed/opened right side extends along local +X from the hinge leaf.
        # +Z makes positive q swing the side inward toward the center for storage.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame_core,
        child=left_caster_fork,
        origin=Origin(xyz=(-0.220, 0.220, CASTER_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=right_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.220, 0.220, CASTER_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -CASTER_AXLE_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=18.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -CASTER_AXLE_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame_core = object_model.get_part("frame_core")
    right_frame = object_model.get_part("right_frame")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    frame_fold = object_model.get_articulation("frame_fold")
    left_swivel = object_model.get_articulation("left_caster_swivel")
    right_swivel = object_model.get_articulation("right_caster_swivel")
    left_roll = object_model.get_articulation("left_caster_roll")
    right_roll = object_model.get_articulation("right_caster_roll")

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
    ctx.allow_overlap(
        left_caster_fork,
        left_front_wheel,
        reason="Caster wheel rotates on a captured axle stub integrated into the fork shell.",
    )
    ctx.allow_overlap(
        right_caster_fork,
        right_front_wheel,
        reason="Caster wheel rotates on a captured axle stub integrated into the fork shell.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "primary_parts_present",
        all(
            part is not None
            for part in (
                frame_core,
                right_frame,
                left_caster_fork,
                right_caster_fork,
                left_front_wheel,
                right_front_wheel,
            )
        ),
        details="Expected walker frame, folding side, caster forks, and wheels.",
    )
    ctx.check(
        "fold_axis_and_caster_axes",
        frame_fold.axis == (0.0, 0.0, 1.0)
        and left_swivel.axis == (0.0, 0.0, 1.0)
        and right_swivel.axis == (0.0, 0.0, 1.0)
        and left_roll.axis == (1.0, 0.0, 0.0)
        and right_roll.axis == (1.0, 0.0, 0.0),
        details="Fold should swing on a vertical hinge and caster wheels should roll on X axes.",
    )
    ctx.expect_contact(frame_core, right_frame, contact_tol=0.0015, name="frame_hinge_contact")
    ctx.expect_contact(frame_core, left_caster_fork, contact_tol=0.0015, name="left_caster_socket_contact")
    ctx.expect_contact(right_frame, right_caster_fork, contact_tol=0.0015, name="right_caster_socket_contact")
    ctx.expect_contact(left_caster_fork, left_front_wheel, contact_tol=0.0015, name="left_wheel_supported")
    ctx.expect_contact(right_caster_fork, right_front_wheel, contact_tol=0.0015, name="right_wheel_supported")

    def _union_aabb(parts) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        mins = [None, None, None]
        maxs = [None, None, None]
        for part in parts:
            aabb = ctx.part_world_aabb(part)
            if aabb is None:
                continue
            lo, hi = aabb
            for axis in range(3):
                mins[axis] = lo[axis] if mins[axis] is None else min(mins[axis], lo[axis])
                maxs[axis] = hi[axis] if maxs[axis] is None else max(maxs[axis], hi[axis])
        if any(value is None for value in mins + maxs):
            return None
        return (
            (mins[0], mins[1], mins[2]),
            (maxs[0], maxs[1], maxs[2]),
        )

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({frame_fold: 0.0}):
        left_grip_aabb = ctx.part_element_world_aabb(frame_core, elem="left_grip")
        right_grip_aabb = ctx.part_element_world_aabb(right_frame, elem="right_grip")
        right_open_aabb = ctx.part_world_aabb(right_frame)
        open_union = _union_aabb(
            [frame_core, right_frame, left_caster_fork, right_caster_fork, left_front_wheel, right_front_wheel]
        )
        open_grip_gap = None
        open_width = None
        open_right_grip_x = None
        if left_grip_aabb is not None and right_grip_aabb is not None:
            open_grip_gap = _aabb_center(right_grip_aabb)[0] - _aabb_center(left_grip_aabb)[0]
            open_right_grip_x = _aabb_center(right_grip_aabb)[0]
        if open_union is not None:
            open_width = open_union[1][0] - open_union[0][0]

    with ctx.pose({frame_fold: 1.20}):
        folded_union = _union_aabb(
            [frame_core, right_frame, left_caster_fork, right_caster_fork, left_front_wheel, right_front_wheel]
        )
        folded_right_aabb = ctx.part_world_aabb(right_frame)
        folded_right_grip_aabb = ctx.part_element_world_aabb(right_frame, elem="right_grip")
        folded_width = None
        folded_grip_gap = None
        folded_right_grip_x = None
        if folded_union is not None:
            folded_width = folded_union[1][0] - folded_union[0][0]
        if left_grip_aabb is not None and folded_right_grip_aabb is not None:
            folded_grip_gap = _aabb_center(folded_right_grip_aabb)[0] - _aabb_center(left_grip_aabb)[0]
            folded_right_grip_x = _aabb_center(folded_right_grip_aabb)[0]

    ctx.check(
        "open_grip_spacing_realistic",
        open_grip_gap is not None and 0.48 <= open_grip_gap <= 0.60,
        details=f"Expected open grip spacing near a real walker width, got {open_grip_gap!r}.",
    )
    ctx.check(
        "folded_grip_draws_inboard",
        open_right_grip_x is not None
        and folded_right_grip_x is not None
        and folded_grip_gap is not None
        and folded_grip_gap is not None
        and folded_right_grip_x < open_right_grip_x - 0.12
        and folded_grip_gap <= 0.40,
        details=(
            "Expected right grip to swing clearly inward in folded pose, "
            f"got x_open={open_right_grip_x!r}, x_folded={folded_right_grip_x!r}, "
            f"gap_open={open_grip_gap!r}, gap_folded={folded_grip_gap!r}."
        ),
    )
    ctx.check(
        "folded_side_crosses_centerline",
        folded_right_aabb is not None and folded_right_aabb[0][0] < -0.10,
        details=f"Expected the folding side to swing across the centerline, got aabb={folded_right_aabb!r}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
