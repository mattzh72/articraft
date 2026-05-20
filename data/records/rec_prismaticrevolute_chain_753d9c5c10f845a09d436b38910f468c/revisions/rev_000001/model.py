from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_slide_hinged_nose")

    rail_mat = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_mat = model.material("dark_oxide", rgba=(0.08, 0.09, 0.10, 1.0))
    truck_mat = model.material("blue_anodized_carriage", rgba=(0.05, 0.22, 0.50, 1.0))
    nose_mat = model.material("safety_orange_nose", rgba=(0.95, 0.38, 0.06, 1.0))
    rubber_mat = model.material("black_rubber_pad", rgba=(0.015, 0.015, 0.014, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((1.70, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_mat,
        name="ground_plate",
    )
    rail.visual(
        Box((1.52, 0.038, 0.082)),
        origin=Origin(xyz=(0.0, -0.067, 0.079)),
        material=rail_mat,
        name="guide_0",
    )
    rail.visual(
        Box((1.52, 0.038, 0.082)),
        origin=Origin(xyz=(0.0, 0.067, 0.079)),
        material=rail_mat,
        name="guide_1",
    )
    rail.visual(
        Box((1.46, 0.035, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=rail_mat,
        name="center_web",
    )
    for i, x in enumerate((-0.79, 0.79)):
        rail.visual(
            Box((0.050, 0.210, 0.130)),
            origin=Origin(xyz=(x, 0.0, 0.103)),
            material=dark_mat,
            name=f"end_stop_{i}",
        )
    for i, (x, y) in enumerate(
        ((-0.63, -0.095), (-0.63, 0.095), (0.63, -0.095), (0.63, 0.095))
    ):
        rail.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.044)),
            material=rail_mat,
            name=f"mount_bolt_{i}",
        )

    truck = model.part("truck")
    truck.visual(
        Box((0.310, 0.172, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=truck_mat,
        name="carriage_body",
    )
    truck.visual(
        Box((0.300, 0.046, 0.030)),
        origin=Origin(xyz=(0.0, -0.067, -0.0105)),
        material=dark_mat,
        name="runner_0",
    )
    truck.visual(
        Box((0.300, 0.046, 0.030)),
        origin=Origin(xyz=(0.0, 0.067, -0.0105)),
        material=dark_mat,
        name="runner_1",
    )
    for i, y in enumerate((-0.105, 0.105)):
        truck.visual(
            Box((0.260, 0.016, 0.066)),
            origin=Origin(xyz=(-0.005, -0.094 if y < 0.0 else 0.094, 0.010)),
            material=truck_mat,
            name=f"side_skirt_{i}",
        )
    truck.visual(
        Box((0.070, 0.025, 0.100)),
        origin=Origin(xyz=(0.170, -0.065, 0.080)),
        material=truck_mat,
        name="fork_cheek_neg",
    )
    truck.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.170, -0.065, 0.118), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_mat,
        name="hinge_boss_neg",
    )
    truck.visual(
        Box((0.070, 0.025, 0.100)),
        origin=Origin(xyz=(0.170, 0.065, 0.080)),
        material=truck_mat,
        name="fork_cheek_pos",
    )
    truck.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.170, 0.065, 0.118), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_mat,
        name="hinge_boss_pos",
    )
    truck.visual(
        Box((0.055, 0.105, 0.020)),
        origin=Origin(xyz=(0.143, 0.0, 0.066)),
        material=truck_mat,
        name="fork_root",
    )
    truck.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(0.170, 0.0, 0.118), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_mat,
        name="hinge_pin",
    )

    nose = model.part("nose")
    nose.visual(
        Cylinder(radius=0.032, length=0.080),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_mat,
        name="hinge_barrel",
    )
    nose.visual(
        Box((0.180, 0.076, 0.054)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=nose_mat,
        name="nose_body",
    )
    nose.visual(
        Cylinder(radius=0.036, length=0.076),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_mat,
        name="nose_cap",
    )
    nose.visual(
        Box((0.095, 0.050, 0.014)),
        origin=Origin(xyz=(0.107, 0.0, 0.034)),
        material=rubber_mat,
        name="top_grip",
    )

    model.articulation(
        "rail_to_truck",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=truck,
        origin=Origin(xyz=(-0.410, 0.0, 0.1455)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.720),
    )

    model.articulation(
        "truck_to_nose",
        ArticulationType.REVOLUTE,
        parent=truck,
        child=nose,
        origin=Origin(xyz=(0.170, 0.0, 0.118)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    truck = object_model.get_part("truck")
    nose = object_model.get_part("nose")
    slide = object_model.get_articulation("rail_to_truck")
    hinge = object_model.get_articulation("truck_to_nose")

    ctx.allow_overlap(
        truck,
        nose,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The steel hinge pin is intentionally captured through the nose barrel.",
    )

    ctx.expect_gap(
        truck,
        rail,
        axis="z",
        positive_elem="runner_0",
        negative_elem="guide_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="runner sits on guide rail",
    )
    ctx.expect_overlap(
        truck,
        rail,
        axes="xy",
        elem_a="runner_0",
        elem_b="guide_0",
        min_overlap=0.025,
        name="runner footprint remains over guide",
    )
    ctx.expect_gap(
        truck,
        nose,
        axis="y",
        positive_elem="fork_cheek_pos",
        negative_elem="hinge_barrel",
        min_gap=0.008,
        max_gap=0.020,
        name="positive fork cheek clears barrel",
    )
    ctx.expect_gap(
        nose,
        truck,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="fork_cheek_neg",
        min_gap=0.008,
        max_gap=0.020,
        name="negative fork cheek clears barrel",
    )
    ctx.expect_within(
        truck,
        nose,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin is concentric inside barrel",
    )
    ctx.expect_overlap(
        truck,
        nose,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.070,
        name="hinge pin spans barrel length",
    )

    rest_pos = ctx.part_world_position(truck)
    rest_cap_aabb = ctx.part_element_world_aabb(nose, elem="nose_cap")
    with ctx.pose({slide: 0.720}):
        ctx.expect_gap(
            truck,
            rail,
            axis="z",
            positive_elem="runner_1",
            negative_elem="guide_1",
            max_gap=0.001,
            max_penetration=0.00001,
            name="extended runner stays on rail",
        )
        ctx.expect_overlap(
            truck,
            rail,
            axes="xy",
            elem_a="runner_1",
            elem_b="guide_1",
            min_overlap=0.025,
            name="extended truck remains guided",
        )
        extended_pos = ctx.part_world_position(truck)

    ctx.check(
        "truck translates along rail",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.65
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({hinge: 1.10}):
        raised_cap_aabb = ctx.part_element_world_aabb(nose, elem="nose_cap")
    ctx.check(
        "nose hinge raises nose cap",
        rest_cap_aabb is not None
        and raised_cap_aabb is not None
        and raised_cap_aabb[0][2] > rest_cap_aabb[0][2] + 0.10,
        details=f"rest_cap={rest_cap_aabb}, raised_cap={raised_cap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
