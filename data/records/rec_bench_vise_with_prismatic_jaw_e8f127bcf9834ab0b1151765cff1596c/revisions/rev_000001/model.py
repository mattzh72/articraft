from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _x_hole(y: float, z: float, radius: float, length: float = 0.60):
    return cq.Workplane("YZ").center(y, z).circle(radius).extrude(length, both=True)


def _moving_jaw_casting():
    """Single sliding casting with clear through-holes for the two bars and screw."""
    carriage = _box_at((0.165, 0.172, 0.078), (0.020, 0.0, 0.002))
    jaw_upright = _box_at((0.080, 0.188, 0.158), (-0.018, 0.0, 0.058))
    top_cap = _box_at((0.100, 0.176, 0.026), (-0.020, 0.0, 0.146))
    screw_nose = _box_at((0.060, 0.096, 0.032), (0.090, 0.0, -0.026))
    throat_rib = _box_at((0.115, 0.045, 0.052), (0.055, 0.0, 0.075))

    shape = carriage.union(jaw_upright).union(top_cap).union(screw_nose).union(throat_rib)
    for y in (-0.055, 0.055):
        shape = shape.cut(_x_hole(y, 0.0, 0.0175))
    shape = shape.cut(_x_hole(0.0, -0.030, 0.0120))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_release_bench_vise")

    cast_iron = model.material("blue_cast_iron", rgba=(0.05, 0.12, 0.22, 1.0))
    dark_cast = model.material("dark_cast_iron", rgba=(0.025, 0.030, 0.035, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    jaw_steel = model.material("flat_ground_jaw_steel", rgba=(0.84, 0.84, 0.78, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = model.material("oiled_bronze_half_nut", rgba=(0.72, 0.48, 0.18, 1.0))
    release_red = model.material("red_release_lever", rgba=(0.72, 0.055, 0.035, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.54, 0.225, 0.035)),
        origin=Origin(xyz=(0.080, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    for xi, x in enumerate((-0.135, 0.295)):
        for yi, y in enumerate((-0.078, 0.078)):
            body.visual(
                Cylinder(radius=0.018, length=0.006),
                origin=Origin(xyz=(x, y, 0.038)),
                material=black_oxide,
                name=f"mount_bolt_{xi}_{yi}",
            )

    body.visual(
        Box((0.105, 0.190, 0.120)),
        origin=Origin(xyz=(-0.085, 0.0, 0.145)),
        material=cast_iron,
        name="fixed_jaw_upright",
    )
    for i, y in enumerate((-0.073, 0.073)):
        body.visual(
            Box((0.105, 0.040, 0.052)),
            origin=Origin(xyz=(-0.085, y, 0.061)),
            material=cast_iron,
            name=f"fixed_lower_cheek_{i}",
        )
    body.visual(
        Box((0.118, 0.150, 0.025)),
        origin=Origin(xyz=(-0.115, 0.0, 0.217)),
        material=dark_cast,
        name="small_anvil",
    )
    body.visual(
        Box((0.012, 0.165, 0.058)),
        origin=Origin(xyz=(-0.027, 0.0, 0.172)),
        material=jaw_steel,
        name="fixed_jaw_plate",
    )

    for i, y in enumerate((-0.075, 0.075)):
        body.visual(
            Box((0.165, 0.035, 0.072)),
            origin=Origin(xyz=(0.015, y, 0.066)),
            material=cast_iron,
            name=f"guide_side_web_{i}",
        )
    body.visual(
        Box((0.175, 0.160, 0.018)),
        origin=Origin(xyz=(0.035, 0.0, 0.108)),
        material=cast_iron,
        name="upper_guide_bridge",
    )
    for i, y in enumerate((-0.055, 0.055)):
        body.visual(
            Cylinder(radius=0.012, length=0.505),
            origin=Origin(xyz=(0.185, y, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name=f"guide_bar_{i}",
        )

    body.visual(
        Box((0.083, 0.080, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.0645)),
        material=brass,
        name="upper_half_nut",
    )
    body.visual(
        Box((0.083, 0.080, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.0355)),
        material=brass,
        name="lower_half_nut",
    )
    for i, y in enumerate((-0.040, 0.040)):
        body.visual(
            Box((0.083, 0.012, 0.050)),
            origin=Origin(xyz=(0.034, y, 0.050)),
            material=brass,
            name=f"half_nut_side_tie_{i}",
        )

    body.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(-0.025, -0.108, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_cast,
        name="release_pivot_boss",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        mesh_from_cadquery(_moving_jaw_casting(), "moving_jaw_casting", tolerance=0.0007),
        material=cast_iron,
        name="sliding_casting",
    )
    moving_jaw.visual(
        Box((0.012, 0.165, 0.058)),
        origin=Origin(xyz=(-0.059, 0.0, 0.105)),
        material=jaw_steel,
        name="jaw_plate",
    )
    moving_jaw.visual(
        Box((0.060, 0.065, 0.008)),
        origin=Origin(xyz=(0.092, 0.0, -0.010)),
        material=black_oxide,
        name="screw_bearing_upper",
    )
    moving_jaw.visual(
        Box((0.110, 0.052, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, -0.042)),
        material=dark_cast,
        name="bed_slide_shoe",
    )
    screw_handle = model.part("lead_screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.0075, length=0.580),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
        name="threaded_shaft",
    )
    for i in range(18):
        x = -0.425 + i * 0.025
        screw_handle.visual(
            Cylinder(radius=0.0092, length=0.0032),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_oxide,
            name=f"thread_crest_{i}",
        )
    screw_handle.visual(
        Cylinder(radius=0.006, length=0.145),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
        name="tommy_bar",
    )
    for i, y in enumerate((-0.078, 0.078)):
        screw_handle.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.067, y, 0.0)),
            material=black_oxide,
            name=f"handle_ball_{i}",
        )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="pivot_washer",
    )
    release_lever.visual(
        Box((0.052, 0.008, 0.026)),
        origin=Origin(xyz=(0.018, -0.006, -0.012), rpy=(0.0, 0.18, 0.0)),
        material=release_red,
        name="lever_root",
    )
    release_lever.visual(
        Box((0.024, 0.008, 0.104)),
        origin=Origin(xyz=(0.025, -0.006, -0.046), rpy=(0.0, 0.35, 0.0)),
        material=release_red,
        name="lever_blade",
    )
    release_lever.visual(
        Box((0.040, 0.010, 0.026)),
        origin=Origin(xyz=(0.046, -0.006, -0.083), rpy=(0.0, 0.18, 0.0)),
        material=release_red,
        name="lever_end_pad",
    )
    release_lever.visual(
        Sphere(radius=0.020),
        origin=Origin(xyz=(0.050, -0.006, -0.094)),
        material=release_red,
        name="finger_knob",
    )

    model.articulation(
        "body_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(0.190, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.22, lower=0.0, upper=0.120),
    )
    model.articulation(
        "jaw_to_lead_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(0.090, 0.0, -0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=9.0),
    )
    model.articulation(
        "body_to_release_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=release_lever,
        origin=Origin(xyz=(-0.025, -0.123, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.35, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    moving_jaw = object_model.get_part("moving_jaw")
    release_lever = object_model.get_part("release_lever")
    lead_screw_handle = object_model.get_part("lead_screw_handle")
    slide = object_model.get_articulation("body_to_moving_jaw")
    screw_turn = object_model.get_articulation("jaw_to_lead_screw_handle")
    release = object_model.get_articulation("body_to_release_lever")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        moving_jaw,
        body,
        axis="x",
        min_gap=0.13,
        max_gap=0.17,
        positive_elem="jaw_plate",
        negative_elem="fixed_jaw_plate",
        name="flat jaws start with a usable vise opening",
    )
    ctx.expect_overlap(
        moving_jaw,
        body,
        axes="yz",
        min_overlap=0.040,
        elem_a="jaw_plate",
        elem_b="fixed_jaw_plate",
        name="opposed flat jaws face each other squarely",
    )
    ctx.expect_contact(
        moving_jaw,
        body,
        elem_a="bed_slide_shoe",
        elem_b="base_plate",
        contact_tol=0.001,
        name="moving jaw has a supported slide shoe on the bed",
    )

    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({slide: 0.120}):
        ctx.expect_gap(
            moving_jaw,
            body,
            axis="x",
            min_gap=0.25,
            max_gap=0.30,
            positive_elem="jaw_plate",
            negative_elem="fixed_jaw_plate",
            name="prismatic slide opens the jaw gap",
        )
        extended_pos = ctx.part_world_position(moving_jaw)
    ctx.check(
        "moving jaw translates along the twin guide bar direction",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.11
        and abs(extended_pos[1] - rest_pos[1]) < 0.002
        and abs(extended_pos[2] - rest_pos[2]) < 0.002,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_contact(
        release_lever,
        body,
        elem_a="pivot_washer",
        elem_b="release_pivot_boss",
        contact_tol=0.001,
        name="release lever pivots on the body side boss",
    )
    knob_rest = aabb_center(ctx.part_element_world_aabb(release_lever, elem="finger_knob"))
    with ctx.pose({release: 0.70}):
        knob_released = aabb_center(ctx.part_element_world_aabb(release_lever, elem="finger_knob"))
    ctx.check(
        "release lever swings through a side-mounted arc",
        knob_rest is not None
        and knob_released is not None
        and abs(knob_released[0] - knob_rest[0]) > 0.045,
        details=f"rest={knob_rest}, released={knob_released}",
    )

    with ctx.pose({screw_turn: math.pi / 2.0}):
        ball_0 = aabb_center(ctx.part_element_world_aabb(lead_screw_handle, elem="handle_ball_0"))
        ball_1 = aabb_center(ctx.part_element_world_aabb(lead_screw_handle, elem="handle_ball_1"))
    ctx.check(
        "lead screw tommy handle rotates about the screw axis",
        ball_0 is not None and ball_1 is not None and abs(ball_1[2] - ball_0[2]) > 0.12,
        details=f"rotated handle ball centers={ball_0}, {ball_1}",
    )

    return ctx.report()


object_model = build_object_model()
