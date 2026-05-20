from __future__ import annotations

from math import atan2, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AXLE_Z = 1.35
WHEEL_RADIUS = 0.95
WHEEL_WIDTH = 0.72
FRAME_SIDE_Y = 0.62


def _beam_between(part, name, p0, p1, thickness, material) -> None:
    """Add one rectangular timber whose local Z axis runs from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = (dx * dx + dy * dy + dz * dz) ** 0.5 + 0.04
    if abs(dy) > 1e-6:
        # This model only needs XZ-plane angled timbers and axis-aligned cross ties.
        raise ValueError("angled helper expects a constant Y coordinate")
    theta = atan2(dx, dz)
    part.visual(
        Box((thickness, thickness, length)),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, p0[1], (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, theta, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_overshot_waterwheel")

    weathered_oak = model.material("weathered_oak", color=(0.53, 0.32, 0.16, 1.0))
    end_grain = model.material("dark_end_grain", color=(0.35, 0.20, 0.10, 1.0))
    wet_timber = model.material("wet_bucket_timber", color=(0.43, 0.24, 0.11, 1.0))
    black_iron = model.material("blackened_iron", color=(0.04, 0.04, 0.035, 1.0))
    water_dark = model.material("shadowed_trough", color=(0.13, 0.19, 0.19, 1.0))

    frame = model.part("frame")

    # Ground skids and transverse ties: a connected timber trestle foundation.
    for side, y in (("near", -FRAME_SIDE_Y), ("far", FRAME_SIDE_Y)):
        frame.visual(
            Box((2.45, 0.11, 0.09)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=weathered_oak,
            name=f"{side}_base_sill",
        )
    for x, label in ((-0.93, "rear"), (0.93, "front")):
        frame.visual(
            Box((0.13, 1.36, 0.09)),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=weathered_oak,
            name=f"{label}_ground_tie",
        )

    # Two A-frame side supports converge on the central axle bearings.
    for side, y in (("near", -FRAME_SIDE_Y), ("far", FRAME_SIDE_Y)):
        _beam_between(frame, f"{side}_rising_post_0", (-0.86, y, 0.08), (0.0, y, AXLE_Z), 0.09, weathered_oak)
        _beam_between(frame, f"{side}_rising_post_1", (0.86, y, 0.08), (0.0, y, AXLE_Z), 0.09, weathered_oak)
        _beam_between(frame, f"{side}_brace", (-0.74, y, 0.42), (0.74, y, 0.86), 0.055, weathered_oak)

    # Bearing saddles surround, but do not intersect, the rotating axle.
    bearing_ring = TorusGeometry(0.105, 0.018, radial_segments=12, tubular_segments=32)
    bearing_ring.rotate_x(pi / 2.0)
    bearing_mesh = mesh_from_geometry(bearing_ring, "iron_bearing_ring")
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, -0.54, AXLE_Z)),
        material=black_iron,
        name="near_bearing_ring",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, 0.54, AXLE_Z)),
        material=black_iron,
        name="far_bearing_ring",
    )
    for side, y in (("near", -0.54), ("far", 0.54)):
        frame.visual(
            Box((0.24, 0.10, 0.045)),
            origin=Origin(xyz=(0.0, y, AXLE_Z - 0.135)),
            material=end_grain,
            name=f"{side}_lower_bearing_cap",
        )
        frame.visual(
            Box((0.13, 0.09, 0.0485)),
            origin=Origin(xyz=(0.0, y, AXLE_Z - 0.08825)),
            material=black_iron,
            name=f"{side}_bearing_shoe",
        )
        frame.visual(
            Box((0.24, 0.10, 0.045)),
            origin=Origin(xyz=(0.0, y, AXLE_Z + 0.135)),
            material=end_grain,
            name=f"{side}_upper_bearing_cap",
        )
        for sx, x in (("left", -0.135), ("right", 0.135)):
            frame.visual(
                Box((0.045, 0.10, 0.24)),
                origin=Origin(xyz=(x, y, AXLE_Z)),
                material=end_grain,
                name=f"{side}_{sx}_bearing_cheek",
            )
        frame.visual(
            Box((0.18, 0.18, 0.07)),
            origin=Origin(xyz=(0.0, y + (0.075 if y < 0 else -0.075), AXLE_Z - 0.18)),
            material=weathered_oak,
            name=f"{side}_bearing_knee",
        )

    # A top feed trough, slightly pitched down toward the wheel crown.
    trough_angle = 0.06
    trough_center = (-0.65, 0.0, AXLE_Z + WHEEL_RADIUS + 0.36)
    trough_length = 1.45
    frame.visual(
        Box((trough_length, 0.56, 0.06)),
        origin=Origin(xyz=trough_center, rpy=(0.0, trough_angle, 0.0)),
        material=weathered_oak,
        name="trough_floor",
    )
    for label, y in (("near", -0.295), ("far", 0.295)):
        frame.visual(
            Box((trough_length, 0.055, 0.28)),
            origin=Origin(xyz=(trough_center[0], y, trough_center[2] + 0.145), rpy=(0.0, trough_angle, 0.0)),
            material=weathered_oak,
            name=f"{label}_trough_wall",
        )
    frame.visual(
        Box((1.30, 0.46, 0.012)),
        origin=Origin(xyz=(-0.72, 0.0, trough_center[2] + 0.035), rpy=(0.0, trough_angle, 0.0)),
        material=water_dark,
        name="trough_water_shadow",
    )

    # Trough trestle posts tie the elevated feed chute into the side frames.
    for x, label in ((-1.18, "rear"), (0.04, "mouth")):
        for y, side in ((-FRAME_SIDE_Y, "near"), (FRAME_SIDE_Y, "far")):
            frame.visual(
                Box((0.08, 0.08, 2.70)),
                origin=Origin(xyz=(x, y, 1.39)),
                material=weathered_oak,
                name=f"{side}_{label}_trough_post",
            )
        frame.visual(
            Box((0.12, 1.36, 0.08)),
            origin=Origin(xyz=(x, 0.0, trough_center[2] - 0.04)),
            material=weathered_oak,
            name=f"{label}_trough_cross_tie",
        )

    wheel = model.part("wheel")

    # Two timber side rings, a through axle, side spokes, and deep bucket pockets.
    ring_geom = TorusGeometry(WHEEL_RADIUS, 0.035, radial_segments=14, tubular_segments=72)
    ring_geom.rotate_x(pi / 2.0)
    ring_mesh = mesh_from_geometry(ring_geom, "timber_side_ring")
    for side, y in (("near", -0.32), ("far", 0.32)):
        wheel.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=wet_timber,
            name=f"{side}_side_ring",
        )
        for i in range(8):
            theta = i * 2.0 * pi / 8.0
            wheel.visual(
                Box((0.075, 0.075, 0.83)),
                origin=Origin(
                    xyz=(0.50 * sin(theta), y, 0.50 * cos(theta)),
                    rpy=(0.0, theta, 0.0),
                ),
                material=weathered_oak,
                name=f"{side}_spoke_{i}",
            )

    wheel.visual(
        Cylinder(radius=0.16, length=0.68),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wet_timber,
        name="wooden_hub",
    )
    wheel.visual(
        Cylinder(radius=0.064, length=1.13),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="iron_axle",
    )

    for i in range(16):
        theta = i * 2.0 * pi / 16.0
        radial = (sin(theta), 0.0, cos(theta))
        tangent = (cos(theta), 0.0, -sin(theta))

        def local_point(r_offset: float, t_offset: float) -> tuple[float, float, float]:
            return (
                radial[0] * r_offset + tangent[0] * t_offset,
                0.0,
                radial[2] * r_offset + tangent[2] * t_offset,
            )

        floor_pos = local_point(WHEEL_RADIUS + 0.005, 0.0)
        lip_pos = local_point(WHEEL_RADIUS + 0.105, 0.13)
        back_pos = local_point(WHEEL_RADIUS - 0.045, -0.13)
        wheel.visual(
            Box((0.29, WHEEL_WIDTH, 0.045)),
            origin=Origin(xyz=floor_pos, rpy=(0.0, theta, 0.0)),
            material=wet_timber,
            name=f"bucket_floor_{i}",
        )
        wheel.visual(
            Box((0.045, WHEEL_WIDTH, 0.22)),
            origin=Origin(xyz=lip_pos, rpy=(0.0, theta, 0.0)),
            material=wet_timber,
            name=f"bucket_lip_{i}",
        )
        wheel.visual(
            Box((0.040, WHEEL_WIDTH, 0.16)),
            origin=Origin(xyz=back_pos, rpy=(0.0, theta, 0.0)),
            material=wet_timber,
            name=f"bucket_back_{i}",
        )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4),
    )

    flap = model.part("flap")
    hinge_x = trough_center[0] + (trough_length * 0.5 * cos(trough_angle)) + 0.05
    hinge_z = trough_center[2] - (trough_length * 0.5 * sin(trough_angle)) + 0.31
    flap.visual(
        Box((0.042, 0.52, 0.30)),
        origin=Origin(xyz=(0.021, 0.0, -0.15)),
        material=weathered_oak,
        name="flap_board",
    )
    flap.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="flap_hinge_barrel",
    )
    for y, label in ((-0.18, "near"), (0.18, "far")):
        flap.visual(
            Box((0.018, 0.045, 0.22)),
            origin=Origin(xyz=(0.045, y, -0.115)),
            material=black_iron,
            name=f"{label}_flap_strap",
        )
    frame.visual(
        Box((0.11, 0.46, 0.018)),
        origin=Origin(xyz=(hinge_x - 0.045, 0.0, hinge_z + 0.018)),
        material=black_iron,
        name="trough_hinge_leaf",
    )
    for y, label in ((-0.265, "near"), (0.265, "far")):
        frame.visual(
            Box((0.075, 0.075, 0.18)),
            origin=Origin(xyz=(hinge_x - 0.055, y, hinge_z - 0.075)),
            material=black_iron,
            name=f"{label}_hinge_bracket",
        )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    wheel_joint = object_model.get_articulation("wheel_axle")
    flap_joint = object_model.get_articulation("flap_hinge")

    ctx.allow_overlap(
        flap,
        frame,
        elem_a="flap_hinge_barrel",
        elem_b="trough_hinge_leaf",
        reason="The movable hinge barrel is intentionally captured through the fixed trough hinge leaf.",
    )
    ctx.expect_gap(
        frame,
        flap,
        axis="z",
        max_penetration=0.012,
        positive_elem="trough_hinge_leaf",
        negative_elem="flap_hinge_barrel",
        name="hinge barrel is only lightly seated in the fixed leaf",
    )

    ctx.check(
        "wheel uses a continuous axle joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type}",
    )
    ctx.check(
        "inspection flap has a realistic limited hinge",
        flap_joint.motion_limits is not None
        and flap_joint.motion_limits.lower == 0.0
        and flap_joint.motion_limits.upper is not None
        and flap_joint.motion_limits.upper > 1.0,
        details=f"limits={flap_joint.motion_limits}",
    )

    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        min_gap=0.05,
        positive_elem="trough_floor",
        name="feed trough sits above the wheel crown",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="y",
        min_overlap=0.02,
        elem_a="iron_axle",
        elem_b="near_bearing_ring",
        name="axle is aligned through the side bearing line",
    )

    rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.1}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "inspection flap swings upward and outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > rest_aabb[0][2] + 0.10
        and open_aabb[1][0] > rest_aabb[1][0] + 0.08,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
