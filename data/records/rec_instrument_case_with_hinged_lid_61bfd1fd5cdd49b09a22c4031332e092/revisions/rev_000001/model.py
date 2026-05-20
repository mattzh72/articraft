from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_drum_hardware_case")

    case_plastic = Material("molded_black_plastic", rgba=(0.03, 0.035, 0.035, 1.0))
    dark_recess = Material("shadowed_recess", rgba=(0.005, 0.006, 0.006, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.64, 1.0))

    length = 1.25
    depth = 0.34
    wall_t = 0.022
    lower_h = 0.205
    rim_h = 0.027
    hinge_z = lower_h + rim_h - 0.004
    lid_h = 0.100
    latch_xs = (-0.52, -0.31, 0.31, 0.52)

    body = model.part("lower_shell")
    body.visual(
        Box((length, depth, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=case_plastic,
        name="bottom_pan",
    )
    body.visual(
        Box((length, wall_t, lower_h)),
        origin=Origin(xyz=(0.0, -depth / 2 + wall_t / 2, lower_h / 2)),
        material=case_plastic,
        name="front_wall",
    )
    body.visual(
        Box((length, wall_t, lower_h)),
        origin=Origin(xyz=(0.0, depth / 2 - wall_t / 2, lower_h / 2)),
        material=case_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, depth, lower_h)),
        origin=Origin(xyz=(-length / 2 + wall_t / 2, 0.0, lower_h / 2)),
        material=case_plastic,
        name="end_wall_0",
    )
    body.visual(
        Box((wall_t, depth, lower_h)),
        origin=Origin(xyz=(length / 2 - wall_t / 2, 0.0, lower_h / 2)),
        material=case_plastic,
        name="end_wall_1",
    )

    rim_z = hinge_z - rim_h / 2
    body.visual(
        Box((length + 0.030, wall_t, rim_h)),
        origin=Origin(xyz=(0.0, -depth / 2 + wall_t / 2, rim_z)),
        material=black_rubber,
        name="front_rim",
    )
    body.visual(
        Box((length + 0.030, wall_t, rim_h)),
        origin=Origin(xyz=(0.0, depth / 2 - wall_t / 2, rim_z)),
        material=black_rubber,
        name="rear_rim",
    )
    body.visual(
        Box((wall_t, depth, rim_h)),
        origin=Origin(xyz=(-length / 2 + wall_t / 2, 0.0, rim_z)),
        material=black_rubber,
        name="end_rim_0",
    )
    body.visual(
        Box((wall_t, depth, rim_h)),
        origin=Origin(xyz=(length / 2 - wall_t / 2, 0.0, rim_z)),
        material=black_rubber,
        name="end_rim_1",
    )
    body.visual(
        Cylinder(radius=0.008, length=length + 0.030),
        origin=Origin(
            xyz=(0.0, depth / 2 + 0.016, hinge_z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=steel,
        name="rear_hinge_pin",
    )
    body.visual(
        Box((length + 0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, depth / 2 + 0.006, hinge_z - 0.006)),
        material=steel,
        name="hinge_backing_strip",
    )

    # Recessed center carry handle molded into the front face.
    body.visual(
        Box((0.330, 0.008, 0.095)),
        origin=Origin(xyz=(0.0, -depth / 2 - 0.002, 0.142)),
        material=dark_recess,
        name="handle_recess",
    )
    body.visual(
        Box((0.360, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -depth / 2 - 0.008, 0.195)),
        material=case_plastic,
        name="handle_upper_lip",
    )
    body.visual(
        Box((0.360, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -depth / 2 - 0.008, 0.089)),
        material=case_plastic,
        name="handle_lower_lip",
    )
    body.visual(
        Box((0.026, 0.012, 0.110)),
        origin=Origin(xyz=(-0.180, -depth / 2 - 0.008, 0.142)),
        material=case_plastic,
        name="handle_side_0",
    )
    body.visual(
        Box((0.026, 0.012, 0.110)),
        origin=Origin(xyz=(0.180, -depth / 2 - 0.008, 0.142)),
        material=case_plastic,
        name="handle_side_1",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.250),
        origin=Origin(
            xyz=(0.0, -depth / 2 - 0.036, 0.142),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=black_rubber,
        name="handle_grip",
    )
    body.visual(
        Box((0.032, 0.040, 0.052)),
        origin=Origin(xyz=(-0.120, -depth / 2 - 0.023, 0.142)),
        material=steel,
        name="handle_boss_0",
    )
    body.visual(
        Box((0.032, 0.040, 0.052)),
        origin=Origin(xyz=(0.120, -depth / 2 - 0.023, 0.142)),
        material=steel,
        name="handle_boss_1",
    )

    for i, x in enumerate(latch_xs):
        body.visual(
            Box((0.082, 0.012, 0.040)),
            origin=Origin(xyz=(x, -depth / 2 - 0.003, hinge_z - 0.020)),
            material=steel,
            name=f"latch_keeper_{i}",
        )

    # Small metal corner guards and molded ribs make the trunk read as a load case.
    for i, x in enumerate((-length / 2 + 0.040, length / 2 - 0.040)):
        for j, y in enumerate((-depth / 2 + 0.010, depth / 2 - 0.010)):
            body.visual(
                Box((0.070, 0.028, 0.052)),
                origin=Origin(xyz=(x, y, 0.080)),
                material=steel,
                name=f"corner_guard_{i}_{j}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((length + 0.030, depth, 0.024)),
        origin=Origin(xyz=(0.0, -depth / 2, lid_h - 0.012)),
        material=case_plastic,
        name="top_panel",
    )
    lid.visual(
        Box((length + 0.030, wall_t, lid_h)),
        origin=Origin(xyz=(0.0, -depth + wall_t / 2, lid_h / 2)),
        material=case_plastic,
        name="front_wall",
    )
    lid.visual(
        Box((length + 0.030, wall_t, lid_h - 0.030)),
        origin=Origin(xyz=(0.0, -wall_t / 2, 0.030 + (lid_h - 0.030) / 2)),
        material=case_plastic,
        name="rear_wall",
    )
    lid.visual(
        Box((wall_t, depth - 0.060, lid_h)),
        origin=Origin(xyz=(-(length + 0.030) / 2 + wall_t / 2, -depth / 2 - 0.030, lid_h / 2)),
        material=case_plastic,
        name="end_wall_0",
    )
    lid.visual(
        Box((wall_t, depth - 0.060, lid_h)),
        origin=Origin(xyz=((length + 0.030) / 2 - wall_t / 2, -depth / 2 - 0.030, lid_h / 2)),
        material=case_plastic,
        name="end_wall_1",
    )
    lid.visual(
        Box((length + 0.015, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.005, 0.045)),
        material=steel,
        name="hinge_capture_strip",
    )
    for y in (-0.105, -0.235):
        lid.visual(
            Box((length * 0.86, 0.026, 0.014)),
            origin=Origin(xyz=(0.0, y, lid_h + 0.007)),
            material=black_rubber,
            name=f"top_rib_{0 if y > -0.17 else 1}",
        )
    for i, x in enumerate(latch_xs):
        lid.visual(
            Box((0.074, 0.012, 0.042)),
            origin=Origin(xyz=(x, -depth - 0.006, 0.036)),
            material=steel,
            name=f"lid_striker_{i}",
        )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, depth / 2, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.32),
    )

    for i, x in enumerate(latch_xs):
        latch = model.part(f"latch_{i}")
        latch.visual(
            Cylinder(radius=0.009, length=0.070),
            origin=Origin(rpy=(0.0, math.pi / 2, 0.0)),
            material=steel,
            name="pivot_bar",
        )
        latch.visual(
            Box((0.058, 0.010, 0.100)),
            origin=Origin(xyz=(0.0, -0.006, -0.047)),
            material=steel,
            name="latch_plate",
        )
        latch.visual(
            Box((0.045, 0.018, 0.015)),
            origin=Origin(xyz=(0.0, -0.011, -0.095)),
            material=steel,
            name="pull_lip",
        )
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(x, -depth / 2 - 0.018, hinge_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.35),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")

    def _center_y(aabb):
        return (aabb[0][1] + aabb[1][1]) / 2.0

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0

    with ctx.pose({rear_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_wall",
            negative_elem="front_rim",
            max_gap=0.003,
            max_penetration=0.0,
            name="closed lid front edge seats on the lower rim",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="hinge_capture_strip",
            negative_elem="rear_hinge_pin",
            max_gap=0.040,
            max_penetration=0.0,
            name="closed lid hinge strip rides just above the rear hinge pin",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a="rear_hinge_pin",
            elem_b="hinge_capture_strip",
            min_overlap=1.15,
            name="rear hinge captures nearly the full lid width",
        )

    front_closed = ctx.part_element_world_aabb(lid, elem="front_wall")
    with ctx.pose({rear_hinge: 1.15}):
        front_open = ctx.part_element_world_aabb(lid, elem="front_wall")
    ctx.check(
        "lid rotates upward around the rear long hinge",
        front_closed is not None
        and front_open is not None
        and _center_z(front_open) > _center_z(front_closed) + 0.18,
        details=f"closed={front_closed}, open={front_open}",
    )

    for i in range(4):
        latch = object_model.get_part(f"latch_{i}")
        pivot = object_model.get_articulation(f"latch_pivot_{i}")
        ctx.expect_contact(
            latch,
            body,
            elem_a="pivot_bar",
            elem_b=f"latch_keeper_{i}",
            contact_tol=0.002,
            name=f"latch {i} pivot is mounted on its front keeper",
        )
        closed_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")
        with ctx.pose({pivot: 1.05}):
            open_plate = ctx.part_element_world_aabb(latch, elem="latch_plate")
        ctx.check(
            f"latch {i} swings outward on its local rim pivot",
            closed_plate is not None
            and open_plate is not None
            and _center_y(open_plate) < _center_y(closed_plate) - 0.025,
            details=f"closed={closed_plate}, open={open_plate}",
        )

    return ctx.report()


object_model = build_object_model()
