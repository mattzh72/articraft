from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
TUBE_RADIUS = 0.011
SIDE_OFFSET_Y = 0.245
WHEEL_RADIUS = 0.065
WHEEL_THICKNESS = 0.022
HINGE_ORIGIN = (0.02, 0.0, 0.61)
LEFT_WHEEL_CENTER = (0.205, SIDE_OFFSET_Y, WHEEL_RADIUS)


def _make_materials() -> dict[str, Material]:
    return {
        "frame": Material(name="walker_frame_satin", rgba=(0.82, 0.84, 0.86, 1.0)),
        "hub": Material(name="hub_gray", rgba=(0.59, 0.62, 0.66, 1.0)),
        "rubber": Material(name="rubber_black", rgba=(0.08, 0.08, 0.09, 1.0)),
        "plastic": Material(name="plastic_graphite", rgba=(0.16, 0.17, 0.19, 1.0)),
        "fabric": Material(name="fabric_charcoal", rgba=(0.18, 0.22, 0.25, 1.0)),
        "reflector": Material(name="reflector_red", rgba=(0.70, 0.12, 0.10, 1.0)),
    }


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(xyz=_midpoint(start, end), rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    origin, length = _segment_pose(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material)


def _localize(
    point: tuple[float, float, float],
    origin: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (point[0] - origin[0], point[1] - origin[1], point[2] - origin[2])


def _side_points(side_y: float) -> dict[str, tuple[float, float, float]]:
    return {
        "rear_bottom": (-0.19, side_y, 0.032),
        "rear_mid": (-0.145, side_y, 0.43),
        "rear_top": (-0.11, side_y, 0.83),
        "front_bottom": (0.205, side_y, 0.155),
        "front_mid": (0.175, side_y, 0.49),
        "front_top": (0.155, side_y, 0.82),
        "inner_upper": (0.0, side_y, 0.70),
        "inner_lower": (0.03, side_y, 0.53),
        "wheel_center": (0.205, side_y, WHEEL_RADIUS),
    }


def _add_side_frame(
    part,
    *,
    pts: dict[str, tuple[float, float, float]],
    materials: dict[str, Material],
    add_caddy: bool,
    hinge_plate_center: tuple[float, float, float] | None,
) -> None:
    _add_tube(part, pts["rear_bottom"], pts["rear_top"], TUBE_RADIUS, materials["frame"])
    _add_tube(part, pts["front_bottom"], pts["front_top"], TUBE_RADIUS, materials["frame"])
    _add_tube(part, pts["rear_top"], pts["front_top"], TUBE_RADIUS, materials["frame"])
    _add_tube(part, pts["rear_mid"], pts["front_mid"], 0.0095, materials["frame"])
    _add_tube(part, pts["rear_mid"], pts["front_top"], 0.0085, materials["frame"])
    _add_tube(part, pts["inner_upper"], pts["front_top"], 0.009, materials["frame"])
    _add_tube(part, pts["inner_lower"], pts["front_mid"], 0.009, materials["frame"])

    rear_top = pts["rear_top"]
    rear_bottom = pts["rear_bottom"]
    _add_tube(
        part,
        (rear_top[0] - 0.07, rear_top[1], rear_top[2] + 0.01),
        (rear_top[0] + 0.03, rear_top[1], rear_top[2] + 0.01),
        0.017,
        materials["rubber"],
    )
    _add_tube(
        part,
        (rear_top[0] + 0.024, rear_top[1], rear_top[2] - 0.03),
        (rear_top[0] + 0.005, rear_top[1], rear_top[2] - 0.085),
        0.0045,
        materials["plastic"],
    )
    part.visual(
        Box((0.018, 0.024, 0.036)),
        origin=Origin(xyz=(rear_top[0] + 0.024, rear_top[1], rear_top[2] - 0.038)),
        material=materials["plastic"],
    )

    part.visual(
        Cylinder(radius=0.017, length=0.032),
        origin=Origin(xyz=(rear_bottom[0], rear_bottom[1], rear_bottom[2] - 0.016)),
        material=materials["rubber"],
    )

    wheel_center = pts["wheel_center"]
    part.visual(
        Box((0.026, 0.032, 0.020)),
        origin=Origin(xyz=(wheel_center[0], wheel_center[1], wheel_center[2] + 0.097)),
        material=materials["frame"],
    )
    for y_offset in (-0.018, 0.018):
        part.visual(
            Box((0.010, 0.006, 0.110)),
            origin=Origin(
                xyz=(wheel_center[0], wheel_center[1] + y_offset, wheel_center[2] + 0.040)
            ),
            material=materials["frame"],
        )
    part.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=wheel_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["hub"],
    )
    part.visual(
        Box((0.012, 0.020, 0.010)),
        origin=Origin(xyz=(wheel_center[0] + 0.008, wheel_center[1], wheel_center[2] + 0.095)),
        material=materials["plastic"],
    )

    if hinge_plate_center is not None:
        part.visual(
            Box((0.045, 0.010, 0.200)),
            origin=Origin(xyz=hinge_plate_center),
            material=materials["frame"],
        )

    if add_caddy:
        part.visual(
            Box((0.16, 0.070, 0.16)),
            origin=Origin(xyz=(0.015, 0.182, 0.53)),
            material=materials["fabric"],
        )
        part.visual(
            Box((0.10, 0.050, 0.085)),
            origin=Origin(xyz=(0.075, 0.182, 0.475)),
            material=materials["fabric"],
        )
        _add_tube(
            part, (-0.03, 0.182, 0.61), (-0.02, SIDE_OFFSET_Y, 0.735), 0.003, materials["plastic"]
        )
        _add_tube(
            part, (0.08, 0.182, 0.61), (0.09, SIDE_OFFSET_Y, 0.705), 0.003, materials["plastic"]
        )
        part.visual(
            Box((0.060, 0.048, 0.090)),
            origin=Origin(xyz=(0.055, 0.062, 0.61)),
            material=materials["plastic"],
        )
        part.visual(
            Box((0.010, 0.030, 0.018)),
            origin=Origin(xyz=(0.080, 0.064, 0.648)),
            material=materials["reflector"],
        )


def _add_front_wheel(part, *, materials: dict[str, Material]) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["rubber"],
    )
    part.visual(
        Cylinder(radius=0.042, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["hub"],
    )
    part.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["plastic"],
    )
    for angle in (0.0, 0.628, 1.257, 1.885, 2.513):
        part.visual(
            Box((0.040, 0.0035, 0.006)),
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=materials["hub"],
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_rolling_walker", assets=ASSETS)
    materials = _make_materials()
    model.materials.extend(materials.values())

    left_pts = _side_points(SIDE_OFFSET_Y)
    right_global_pts = _side_points(-SIDE_OFFSET_Y)
    right_pts = {name: _localize(point, HINGE_ORIGIN) for name, point in right_global_pts.items()}

    main_frame = model.part("main_frame")
    _add_side_frame(
        main_frame,
        pts=left_pts,
        materials=materials,
        add_caddy=True,
        hinge_plate_center=(HINGE_ORIGIN[0], 0.007, HINGE_ORIGIN[2]),
    )
    _add_tube(
        main_frame, (0.0, SIDE_OFFSET_Y, 0.70), (0.02, 0.012, 0.70), 0.009, materials["frame"]
    )
    _add_tube(
        main_frame, (0.03, SIDE_OFFSET_Y, 0.53), (0.02, 0.012, 0.52), 0.009, materials["frame"]
    )
    _add_tube(
        main_frame, (0.12, SIDE_OFFSET_Y, 0.60), (0.06, 0.018, 0.61), 0.008, materials["frame"]
    )
    _add_tube(
        main_frame, (-0.01, SIDE_OFFSET_Y, 0.62), (0.05, 0.020, 0.565), 0.008, materials["frame"]
    )
    main_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 0.86)),
        mass=4.6,
        origin=Origin(xyz=(0.005, 0.132, 0.43)),
    )

    right_side = model.part("right_side")
    _add_side_frame(
        right_side,
        pts=right_pts,
        materials=materials,
        add_caddy=False,
        hinge_plate_center=(0.0, -0.007, 0.0),
    )
    _add_tube(right_side, right_pts["inner_upper"], (0.0, -0.012, 0.09), 0.009, materials["frame"])
    _add_tube(right_side, right_pts["inner_lower"], (0.0, -0.012, -0.09), 0.009, materials["frame"])
    _add_tube(right_side, right_pts["front_mid"], (0.04, -0.018, -0.01), 0.008, materials["frame"])
    right_side.inertial = Inertial.from_geometry(
        Box((0.42, 0.07, 0.86)),
        mass=2.0,
        origin=Origin(xyz=(-0.01, -0.245, -0.18)),
    )

    left_front_wheel = model.part("left_front_wheel")
    _add_front_wheel(left_front_wheel, materials=materials)
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        mass=0.34,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_front_wheel = model.part("right_front_wheel")
    _add_front_wheel(right_front_wheel, materials=materials)
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        mass=0.34,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent="main_frame",
        child="right_side",
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.6,
            lower=0.0,
            upper=0.58,
        ),
    )
    model.articulation(
        "left_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="left_front_wheel",
        origin=Origin(xyz=LEFT_WHEEL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "right_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="right_side",
        child="right_front_wheel",
        origin=Origin(xyz=right_pts["wheel_center"]),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "left_front_wheel",
        "main_frame",
        reason="The wheel spins on a small fixed axle that passes through the hub.",
    )
    ctx.allow_overlap(
        "right_front_wheel",
        "right_side",
        reason="The wheel spins on a small fixed axle that passes through the hub.",
    )
    ctx.allow_overlap(
        "main_frame",
        "right_side",
        reason="At the fully folded storage limit the hinge plates and nested side rails are conservatively reported as overlapping.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("left_front_wheel", "main_frame", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_overlap("right_front_wheel", "right_side", axes="xy", min_overlap=0.008)

    left_wheel_open = ctx.part_world_position("left_front_wheel")
    right_wheel_open = ctx.part_world_position("right_front_wheel")
    if left_wheel_open[1] < 0.20:
        raise AssertionError(
            "The left front wheel should remain well outboard for a stable stance."
        )
    if right_wheel_open[1] > -0.20:
        raise AssertionError(
            "The right front wheel should start well outboard on the folding side."
        )
    if abs(left_wheel_open[0] - right_wheel_open[0]) > 0.01:
        raise AssertionError("The front wheels should align along the walker's forward direction.")
    if abs(left_wheel_open[2] - right_wheel_open[2]) > 0.01:
        raise AssertionError("The front wheels should ride at the same height.")
    if left_wheel_open[1] - right_wheel_open[1] < 0.46:
        raise AssertionError("The open walker should have a broad front track for stability.")

    if left_wheel_open[0] < 0.16 or right_wheel_open[0] < 0.16:
        raise AssertionError(
            "The front wheels should sit forward of the user's stance, like a real rollator."
        )
    if left_wheel_open[2] < 0.05 or right_wheel_open[2] < 0.05:
        raise AssertionError(
            "The front casters should keep the frame lifted a realistic distance above the floor."
        )

    with ctx.pose(right_fold=0.55):
        folded_right_wheel = ctx.part_world_position("right_front_wheel")
        if folded_right_wheel[1] <= right_wheel_open[1] + 0.12:
            raise AssertionError("Folding should pull the right wheel decisively inward.")
        if folded_right_wheel[0] <= right_wheel_open[0] + 0.04:
            raise AssertionError(
                "The folded side should also swing slightly forward as on a real walker hinge."
            )
        if (
            left_wheel_open[1] - folded_right_wheel[1]
            >= left_wheel_open[1] - right_wheel_open[1] - 0.10
        ):
            raise AssertionError("Folding should noticeably narrow the walker's front track.")
        if abs(folded_right_wheel[2] - right_wheel_open[2]) > 0.015:
            raise AssertionError(
                "Folding should tuck the side inward without lifting the front wheel off its rolling height."
            )

    with ctx.pose(left_front_wheel_spin=1.6):
        left_wheel_spun = ctx.part_world_position("left_front_wheel")
        if any(abs(a - b) > 1e-5 for a, b in zip(left_wheel_open, left_wheel_spun)):
            raise AssertionError("Spinning the left wheel must not translate the wheel assembly.")

    with ctx.pose(right_front_wheel_spin=1.2):
        right_wheel_spun = ctx.part_world_position("right_front_wheel")
        if any(abs(a - b) > 1e-5 for a, b in zip(right_wheel_open, right_wheel_spun)):
            raise AssertionError("Spinning the right wheel must not translate the wheel assembly.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
