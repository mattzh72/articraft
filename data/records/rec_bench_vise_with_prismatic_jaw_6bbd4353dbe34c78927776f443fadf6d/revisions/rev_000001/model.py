from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

JAW_TRAVEL = 0.18
JAW_REST_OFFSET = 0.10


def _disk(radius: float, thickness: float, center_xyz: tuple[float, float, float]):
    cx, cy, cz = center_xyz
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(thickness)
        .translate((cx, cy, cz - thickness * 0.5))
    )


def _base_casting_shape():
    lower_plate = _disk(0.147, 0.022, (-0.055, 0.0, -0.146))
    upper_plate = _disk(0.140, 0.018, (-0.055, 0.0, -0.126))
    pedestal = cq.Workplane("XY").box(0.170, 0.215, 0.070).translate((-0.055, 0.0, -0.090))
    body = cq.Workplane("XY").box(0.118, 0.285, 0.138).translate((-0.061, 0.0, 0.008))
    anvil = cq.Workplane("XY").box(0.102, 0.116, 0.012).translate((-0.060, 0.0, 0.083))
    screw_housing = (
        cq.Workplane("YZ")
        .circle(0.032)
        .extrude(0.070)
        .translate((-0.074, 0.0, 0.0))
    )
    cheek_pad = cq.Workplane("XY").box(0.050, 0.200, 0.050).translate((-0.050, 0.0, -0.060))
    return lower_plate.union(upper_plate).union(pedestal).union(body).union(anvil).union(screw_housing).union(cheek_pad)


def _jaw_carriage_shape():
    carriage = cq.Workplane("XY").box(0.082, 0.174, 0.138).translate((-0.022, 0.0, -0.016))
    front_plate = cq.Workplane("XY").box(0.026, 0.286, 0.120).translate((0.013, 0.0, 0.000))
    anvil = cq.Workplane("XY").box(0.076, 0.112, 0.012).translate((0.006, 0.0, 0.059))
    boss = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(0.034)
        .translate((0.026, 0.0, 0.0))
    )
    guide_clearance = cq.Workplane("XY").box(0.125, 0.118, 0.060).translate((-0.0025, 0.0, -0.052))
    return carriage.union(front_plate).union(anvil).union(boss).cut(guide_clearance)


def _handwheel_shape():
    rim = cq.Workplane("YZ").circle(0.052).circle(0.041).extrude(0.010, both=True)
    hub = cq.Workplane("YZ").circle(0.017).extrude(0.014, both=True)
    spoke = cq.Workplane("XY").box(0.008, 0.070, 0.012).translate((0.0, 0.035, 0.0))
    spokes = spoke.union(
        spoke.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 120.0)
    ).union(
        spoke.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 240.0)
    )
    return rim.union(hub).union(spokes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_base_woodworking_vise")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.28, 0.34, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.71, 0.74, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    wood_pad = model.material("wood_pad", rgba=(0.59, 0.43, 0.24, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_casting_shape(), "vise_base_casting"),
        material=cast_iron,
        name="base_casting",
    )
    base.visual(
        Box((0.335, 0.100, 0.048)),
        origin=Origin(xyz=(0.1675, 0.0, -0.052)),
        material=machined_steel,
        name="guide_beam",
    )
    base.visual(
        Box((0.018, 0.240, 0.085)),
        origin=Origin(xyz=(-0.009, 0.0, 0.020)),
        material=wood_pad,
        name="fixed_pad",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(-0.030, 0.139, -0.122), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=machined_steel,
        name="lock_boss",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_jaw_carriage_shape(), "vise_jaw_carriage"),
        material=cast_iron,
        name="carriage",
    )
    jaw.visual(
        Box((0.018, 0.240, 0.085)),
        origin=Origin(xyz=(-0.009, 0.0, 0.020)),
        material=wood_pad,
        name="jaw_pad",
    )
    jaw.visual(
        Box((0.110, 0.108, 0.006)),
        origin=Origin(xyz=(-0.005, 0.0, -0.025)),
        material=machined_steel,
        name="guide_shoe",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "vise_handwheel"),
        material=dark_steel,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=machined_steel,
        name="spindle",
    )

    swivel_lock = model.part("swivel_lock")
    swivel_lock.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=machined_steel,
        name="hub",
    )
    swivel_lock.visual(
        Box((0.084, 0.020, 0.020)),
        origin=Origin(xyz=(0.042, 0.011, 0.0)),
        material=machined_steel,
        name="arm",
    )
    swivel_lock.visual(
        Cylinder(radius=0.014, length=0.038),
        origin=Origin(xyz=(0.084, 0.011, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=handle_grip,
        name="grip",
    )

    jaw_slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jaw,
        origin=Origin(xyz=(JAW_REST_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=jaw,
        child=handwheel,
        origin=Origin(xyz=(0.084, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=8.0,
        ),
    )
    swivel_lock_pivot = model.articulation(
        "swivel_lock_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swivel_lock,
        origin=Origin(xyz=(-0.030, 0.149, -0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.35,
        ),
    )

    base.meta["primary_motion"] = jaw_slide.name
    jaw.meta["role"] = "sliding_front_jaw"
    handwheel.meta["role"] = "front_drive_handwheel"
    swivel_lock.meta["role"] = "swivel_ring_lock"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jaw = object_model.get_part("jaw")
    handwheel = object_model.get_part("handwheel")
    swivel_lock = object_model.get_part("swivel_lock")
    jaw_slide = object_model.get_articulation("jaw_slide")
    swivel_lock_pivot = object_model.get_articulation("swivel_lock_pivot")

    ctx.expect_gap(
        jaw,
        base,
        axis="x",
        positive_elem="jaw_pad",
        negative_elem="fixed_pad",
        min_gap=0.080,
        max_gap=0.085,
        name="rest jaw opening is workshop-scale",
    )

    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            jaw,
            base,
            axis="x",
            positive_elem="jaw_pad",
            negative_elem="fixed_pad",
            min_gap=0.260,
            max_gap=0.265,
            name="front jaw opens wide at full travel",
        )
        ctx.expect_overlap(
            jaw,
            base,
            axes="x",
            elem_a="carriage",
            elem_b="guide_beam",
            min_overlap=0.100,
            name="jaw carriage stays retained on guide beam",
        )

    rest_wheel_pos = ctx.part_world_position(handwheel)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        extended_wheel_pos = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel translates with sliding jaw",
        rest_wheel_pos is not None
        and extended_wheel_pos is not None
        and extended_wheel_pos[0] > rest_wheel_pos[0] + 0.17,
        details=f"rest={rest_wheel_pos}, extended={extended_wheel_pos}",
    )

    rest_lock = ctx.part_element_world_aabb(swivel_lock, elem="grip")
    with ctx.pose({swivel_lock_pivot: 0.30}):
        raised_lock = ctx.part_element_world_aabb(swivel_lock, elem="grip")
    rest_grip_center_z = None if rest_lock is None else (rest_lock[0][2] + rest_lock[1][2]) * 0.5
    raised_grip_center_z = None if raised_lock is None else (raised_lock[0][2] + raised_lock[1][2]) * 0.5
    ctx.check(
        "swivel lock lever lifts on its pivot",
        rest_grip_center_z is not None
        and raised_grip_center_z is not None
        and raised_grip_center_z > rest_grip_center_z + 0.010,
        details=f"rest_z={rest_grip_center_z}, raised_z={raised_grip_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
