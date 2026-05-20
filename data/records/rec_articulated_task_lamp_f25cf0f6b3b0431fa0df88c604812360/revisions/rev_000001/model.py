from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    """Counter-clockwise 2-D circle profile for mesh annuli."""
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    *,
    material: Material | str | None = None,
    name: str | None = None,
) -> None:
    """Attach a cylinder whose local +Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnifying_work_lamp")

    cast_iron = model.material("blackened_cast_iron", rgba=(0.02, 0.022, 0.024, 1.0))
    satin_metal = model.material("satin_chrome", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_pivot = model.material("dark_pivot_bushings", rgba=(0.055, 0.058, 0.060, 1.0))
    white_enamel = model.material("white_enamel_shade", rgba=(0.93, 0.91, 0.84, 1.0))
    lens_glass = model.material("pale_magnifier_glass", rgba=(0.72, 0.90, 1.0, 0.38))
    warm_diffuser = model.material("warm_led_diffuser", rgba=(1.0, 0.82, 0.45, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.012, 0.012, 0.011, 1.0))

    # A heavy, shallow cast-iron base with a raised softened rim.
    base = model.part("base")
    base_profile = [
        (0.0, 0.000),
        (0.195, 0.000),
        (0.225, 0.010),
        (0.218, 0.036),
        (0.175, 0.054),
        (0.060, 0.058),
        (0.0, 0.058),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=96), "cast_iron_base"),
        material=cast_iron,
        name="cast_iron_base",
    )

    # Low rubber feet are slightly seated into the underside of the base.
    for idx, (x, y) in enumerate(
        ((0.135, 0.095), (-0.135, 0.095), (-0.135, -0.095), (0.135, -0.095))
    ):
        base.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )

    # Pedestal and shoulder yoke are part of the fixed base casting.
    base.visual(
        Cylinder(radius=0.030, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=cast_iron,
        name="shoulder_post",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=cast_iron,
        name="post_foot",
    )
    base.visual(
        Box((0.090, 0.125, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=cast_iron,
        name="shoulder_yoke_bridge",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        base.visual(
            Box((0.080, 0.014, 0.098)),
            origin=Origin(xyz=(0.0, y, 0.245)),
            material=cast_iron,
            name=f"shoulder_cheek_{idx}",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.01, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_pivot,
            name=f"shoulder_washer_{idx}",
        )

    # Lower arm: two slender rods with visible bushings, framed from shoulder to elbow.
    lower_arm = model.part("lower_arm")
    lower_elbow = (0.300, 0.0, 0.360)
    lower_arm.visual(
        Cylinder(radius=0.024, length=0.090),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_pivot,
        name="shoulder_barrel",
    )
    for idx, y in enumerate((-0.032, 0.032)):
        _cylinder_between(
            lower_arm,
            (0.000, y, 0.000),
            (lower_elbow[0] - 0.020, y, lower_elbow[2] - 0.025),
            0.0065,
            material=satin_metal,
            name=f"lower_rod_{idx}",
        )
        lower_arm.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(
                xyz=(lower_elbow[0], y * 1.42, lower_elbow[2]),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_pivot,
            name=f"elbow_side_bushing_{idx}",
        )

    # Upper arm starts in the elbow gap, then terminates in a small fork at the lamp head.
    upper_arm = model.part("upper_arm")
    tip = (0.360, 0.0, -0.180)
    tip_bridge = (0.307, 0.0, -0.153)
    upper_arm.visual(
        Cylinder(radius=0.023, length=0.073),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_pivot,
        name="elbow_barrel",
    )
    for idx, y in enumerate((-0.018, 0.018)):
        _cylinder_between(
            upper_arm,
            (0.020, y, -0.010),
            (tip_bridge[0], y, tip_bridge[2]),
            0.0065,
            material=satin_metal,
            name=f"upper_rod_{idx}",
        )
    upper_arm.visual(
        Cylinder(radius=0.014, length=0.104),
        origin=Origin(xyz=tip_bridge, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_pivot,
        name="tip_crossbar",
    )
    for idx, y in enumerate((-0.046, 0.046)):
        _cylinder_between(
            upper_arm,
            (tip_bridge[0], y, tip_bridge[2]),
            (tip[0], y, tip[2]),
            0.012,
            material=dark_pivot,
            name=f"tip_fork_{idx}",
        )

    # The head pivots at the arm tip.  Its circular shade surrounds a magnifying lens.
    lens_frame = model.part("lens_frame")
    ring_outer = _circle_profile(0.165, 96)
    ring_inner = _circle_profile(0.118, 96)
    lens_frame.visual(
        Cylinder(radius=0.022, length=0.068),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_pivot,
        name="tilt_barrel",
    )
    _cylinder_between(
        lens_frame,
        (0.000, 0.0, 0.000),
        (0.035, 0.0, -0.026),
        0.012,
        material=white_enamel,
        name="shade_neck",
    )
    lens_frame.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(ring_outer, [ring_inner], 0.046, center=True),
            "ring_shade",
        ),
        origin=Origin(xyz=(0.132, 0.0, -0.030)),
        material=white_enamel,
        name="ring_shade",
    )
    lens_frame.visual(
        Cylinder(radius=0.121, length=0.006),
        origin=Origin(xyz=(0.132, 0.0, -0.030)),
        material=lens_glass,
        name="lens_glass",
    )
    lens_frame.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(_circle_profile(0.116, 96), [_circle_profile(0.097, 96)], 0.005, center=True),
            "led_diffuser_ring",
        ),
        origin=Origin(xyz=(0.132, 0.0, -0.055)),
        material=warm_diffuser,
        name="led_diffuser",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=lower_elbow),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=2.0, lower=-1.25, upper=1.20),
    )
    model.articulation(
        "tip_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lens_frame,
        origin=Origin(xyz=tip),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.95, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lens_frame = object_model.get_part("lens_frame")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    tip_tilt = object_model.get_articulation("tip_tilt")

    ctx.check(
        "work lamp has the requested articulated chain",
        all(part is not None for part in (base, lower_arm, upper_arm, lens_frame))
        and all(joint is not None for joint in (shoulder, elbow, tip_tilt)),
    )
    ctx.check(
        "all primary joints are revolute",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and tip_tilt.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.check(
        "shoulder elbow and tilt have practical lamp limits",
        shoulder.motion_limits.lower < -0.5
        and shoulder.motion_limits.upper > 0.7
        and elbow.motion_limits.lower < -1.0
        and elbow.motion_limits.upper > 1.0
        and tip_tilt.motion_limits.lower < -0.8
        and tip_tilt.motion_limits.upper > 0.8,
    )

    ctx.expect_overlap(
        lens_frame,
        lens_frame,
        axes="xy",
        elem_a="lens_glass",
        elem_b="ring_shade",
        min_overlap=0.18,
        name="magnifier lens sits inside circular ring shade footprint",
    )
    ctx.expect_within(
        lens_frame,
        lens_frame,
        axes="xy",
        inner_elem="lens_glass",
        outer_elem="ring_shade",
        margin=0.004,
        name="lens is retained by the ring shade",
    )

    def _distance(a, b):
        if a is None or b is None:
            return None
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_elbow_position = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.55}):
        raised_elbow_position = ctx.part_world_position(upper_arm)
    shoulder_motion = _distance(rest_elbow_position, raised_elbow_position)
    ctx.check(
        "shoulder joint swings both arm segments",
        shoulder_motion is not None and shoulder_motion > 0.18,
        details=f"rest={rest_elbow_position}, posed={raised_elbow_position}",
    )

    rest_tip_position = ctx.part_world_position(lens_frame)
    with ctx.pose({elbow: 0.70}):
        bent_tip_position = ctx.part_world_position(lens_frame)
    elbow_motion = _distance(rest_tip_position, bent_tip_position)
    ctx.check(
        "elbow joint changes lamp head reach",
        elbow_motion is not None and elbow_motion > 0.20,
        details=f"rest={rest_tip_position}, posed={bent_tip_position}",
    )

    rest_lens_center = _aabb_center(ctx.part_element_world_aabb(lens_frame, elem="lens_glass"))
    with ctx.pose({tip_tilt: 0.65}):
        tilted_lens_center = _aabb_center(ctx.part_element_world_aabb(lens_frame, elem="lens_glass"))
    ctx.check(
        "tip tilt joint pitches the magnifier lens",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] < rest_lens_center[2] - 0.06,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    return ctx.report()


object_model = build_object_model()
