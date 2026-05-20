from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)
import cadquery as cq


def _cylinder_between_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an origin and length for a URDF-style Z-axis cylinder beam."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must differ")
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    yaw = math.atan2(dy, dx)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_tilt_searchlight_tower")

    dark_metal = model.material("dark_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    matte_black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    parkerized = model.material("parkerized_steel", rgba=(0.20, 0.22, 0.22, 1.0))
    warm_metal = model.material("warm_brass", rgba=(0.78, 0.56, 0.22, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.35, 0.65, 0.95, 0.48))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.82, 0.82, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="ground_plate",
    )
    tower.visual(
        Cylinder(radius=0.25, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=parkerized,
        name="base_plinth",
    )
    tower.visual(
        Cylinder(radius=0.055, length=1.22),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=parkerized,
        name="mast",
    )
    for index, (sx, sy) in enumerate(
        ((0.34, 0.34), (-0.34, 0.34), (0.34, -0.34), (-0.34, -0.34))
    ):
        origin, length = _cylinder_between_origin(
            (sx, sy, 0.055), (0.034 if sx > 0 else -0.034, 0.034 if sy > 0 else -0.034, 0.56)
        )
        tower.visual(
            Cylinder(radius=0.017, length=length),
            origin=origin,
            material=parkerized,
            name=f"brace_{index}",
        )
    tower.visual(
        Cylinder(radius=0.125, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.305)),
        material=parkerized,
        name="top_collar",
    )
    tower.visual(
        Box((0.25, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.325)),
        material=dark_metal,
        name="bearing_plate",
    )
    for index, y in enumerate((-0.19, 0.19)):
        tower.visual(
            Box((0.18, 0.035, 0.18)),
            origin=Origin(xyz=(0.0, y, 1.255)),
            material=dark_metal,
            name=f"side_support_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.14, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="turntable",
    )
    yoke.visual(
        Box((0.18, 0.70, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_metal,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.09, 0.075, 0.44)),
        origin=Origin(xyz=(0.0, -0.315, 0.32)),
        material=dark_metal,
        name="yoke_arm_0",
    )
    yoke.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0, -0.364, 0.36), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="bearing_boss_0",
    )
    yoke.visual(
        Box((0.09, 0.075, 0.44)),
        origin=Origin(xyz=(0.0, 0.315, 0.32)),
        material=dark_metal,
        name="yoke_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0, 0.364, 0.36), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="bearing_boss_1",
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.205, length=0.48),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="body",
    )
    lamp.visual(
        Cylinder(radius=0.238, length=0.06),
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.178, length=0.018),
        origin=Origin(xyz=(0.342, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens",
    )
    lamp.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.333, 0.0, 0.0)),
        material=warm_metal,
        name="reflector_bulb",
    )
    lamp.visual(
        Cylinder(radius=0.185, length=0.045),
        origin=Origin(xyz=(-0.222, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.041, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=parkerized,
        name="trunnion_axle",
    )
    lamp.visual(
        Box((0.035, 0.05, 0.075)),
        origin=Origin(xyz=(-0.09, 0.0, 0.225)),
        material=rubber,
        name="handle_post_0",
    )
    lamp.visual(
        Box((0.035, 0.05, 0.075)),
        origin=Origin(xyz=(0.16, 0.0, 0.225)),
        material=rubber,
        name="handle_post_1",
    )
    lamp.visual(
        Box((0.30, 0.05, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, 0.278)),
        material=rubber,
        name="top_handle",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.345)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.9, lower=-0.55, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.allow_overlap(
        yoke,
        lamp,
        elem_a="yoke_arm_0",
        elem_b="trunnion_axle",
        reason="The tilt trunnion axle is intentionally captured in the yoke arm bushing.",
    )
    ctx.allow_overlap(
        yoke,
        lamp,
        elem_a="yoke_arm_1",
        elem_b="trunnion_axle",
        reason="The tilt trunnion axle is intentionally captured in the opposite yoke arm bushing.",
    )

    ctx.check(
        "pan joint is vertical revolute",
        pan.articulation_type == ArticulationType.REVOLUTE and tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )
    ctx.check(
        "tilt joint is horizontal revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    ctx.expect_gap(
        yoke,
        tower,
        axis="z",
        positive_elem="turntable",
        negative_elem="bearing_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable seats on tower bearing",
    )
    ctx.expect_overlap(
        yoke,
        tower,
        axes="xy",
        elem_a="turntable",
        elem_b="bearing_plate",
        min_overlap=0.18,
        name="pan bearing has broad footprint",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="yoke_arm_1",
        negative_elem="body",
        min_gap=0.045,
        max_gap=0.10,
        name="lamp body clears positive yoke arm",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="body",
        negative_elem="yoke_arm_0",
        min_gap=0.045,
        max_gap=0.10,
        name="lamp body clears negative yoke arm",
    )
    ctx.expect_gap(
        yoke,
        lamp,
        axis="y",
        positive_elem="yoke_arm_1",
        negative_elem="trunnion_axle",
        max_gap=0.001,
        max_penetration=0.004,
        name="positive trunnion is captured",
    )
    ctx.expect_gap(
        lamp,
        yoke,
        axis="y",
        positive_elem="trunnion_axle",
        negative_elem="yoke_arm_0",
        max_gap=0.001,
        max_penetration=0.004,
        name="negative trunnion is captured",
    )

    lens_rest = ctx.part_element_world_aabb(lamp, elem="lens")
    with ctx.pose({pan: 1.0}):
        lens_panned = ctx.part_element_world_aabb(lamp, elem="lens")
    with ctx.pose({tilt: 0.65}):
        lens_tilted = ctx.part_element_world_aabb(lamp, elem="lens")

    def _center(aabb, index: int) -> float:
        return (aabb[0][index] + aabb[1][index]) * 0.5

    ctx.check(
        "pan pose swings beam around mast",
        lens_rest is not None
        and lens_panned is not None
        and _center(lens_panned, 1) > _center(lens_rest, 1) + 0.20
        and _center(lens_panned, 0) < _center(lens_rest, 0),
        details=f"rest={lens_rest}, panned={lens_panned}",
    )
    ctx.check(
        "positive tilt raises lamp beam",
        lens_rest is not None
        and lens_tilted is not None
        and _center(lens_tilted, 2) > _center(lens_rest, 2) + 0.15,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
