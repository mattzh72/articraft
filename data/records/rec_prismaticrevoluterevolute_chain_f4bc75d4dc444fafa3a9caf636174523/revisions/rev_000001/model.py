from __future__ import annotations

import math

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
    model = ArticulatedObject(name="overhead_rail_inspection_shuttle")

    rail_mat = model.material("painted_rail_blue_gray", rgba=(0.28, 0.34, 0.38, 1.0))
    edge_mat = model.material("dark_steel_edges", rgba=(0.08, 0.09, 0.10, 1.0))
    shuttle_mat = model.material("safety_yellow_shuttle", rgba=(0.95, 0.66, 0.12, 1.0))
    arm_mat = model.material("blackened_arm_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    pin_mat = model.material("brushed_pin_metal", rgba=(0.64, 0.66, 0.68, 1.0))
    plate_mat = model.material("matte_inspection_plate", rgba=(0.12, 0.16, 0.18, 1.0))
    sensor_mat = model.material("dark_sensor_face", rgba=(0.01, 0.012, 0.014, 1.0))

    beam = model.part("beam")
    beam.visual(
        Box((2.40, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.7975)),
        material=rail_mat,
        name="lower_flange",
    )
    beam.visual(
        Box((2.40, 0.060, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, 1.875)),
        material=rail_mat,
        name="web",
    )
    beam.visual(
        Box((2.40, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.9525)),
        material=rail_mat,
        name="upper_flange",
    )
    for i, x in enumerate((-1.12, 1.12)):
        beam.visual(
            Box((0.055, 0.25, 0.105)),
            origin=Origin(xyz=(x, 0.0, 1.825)),
            material=edge_mat,
            name=f"end_stop_{i}",
        )
    for i, x in enumerate((-0.78, 0.78)):
        beam.visual(
            Box((0.045, 0.045, 0.185)),
            origin=Origin(xyz=(x, 0.0, 2.057)),
            material=edge_mat,
            name=f"ceiling_hanger_{i}",
        )
        beam.visual(
            Box((0.22, 0.17, 0.025)),
            origin=Origin(xyz=(x, 0.0, 2.155)),
            material=edge_mat,
            name=f"ceiling_pad_{i}",
        )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((0.30, 0.32, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=edge_mat,
        name="wear_shoe",
    )
    for i, y in enumerate((-0.1375, 0.1375)):
        shuttle.visual(
            Box((0.32, 0.035, 0.260)),
            origin=Origin(xyz=(0.0, y, 0.220)),
            material=shuttle_mat,
            name=f"side_plate_{i}",
        )
    shuttle.visual(
        Box((0.34, 0.31, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=shuttle_mat,
        name="lower_crosshead",
    )
    for i, y in enumerate((-0.055, 0.055)):
        shuttle.visual(
            Box((0.060, 0.025, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.100)),
            material=shuttle_mat,
            name=f"hanger_strap_{i}",
        )
        shuttle.visual(
            Box((0.080, 0.025, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=shuttle_mat,
            name=f"shoulder_yoke_{i}",
        )
        shuttle.visual(
            Cylinder(radius=0.045, length=0.025),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_mat,
            name=f"shoulder_pivot_disc_{i}",
        )

    first_link = model.part("first_link")
    first_link.visual(
        Box((0.080, 0.085, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_mat,
        name="shoulder_lug",
    )
    first_link.visual(
        Cylinder(radius=0.035, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="shoulder_bushing",
    )
    first_link.visual(
        Box((0.050, 0.040, 0.230)),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=arm_mat,
        name="short_strut",
    )
    first_link.visual(
        Box((0.085, 0.130, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=arm_mat,
        name="elbow_bridge",
    )
    for i, y in enumerate((-0.055, 0.055)):
        first_link.visual(
            Box((0.080, 0.025, 0.110)),
            origin=Origin(xyz=(0.0, y, -0.335)),
            material=arm_mat,
            name=f"elbow_yoke_{i}",
        )
        first_link.visual(
            Cylinder(radius=0.045, length=0.025),
            origin=Origin(xyz=(0.0, y, -0.335), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_mat,
            name=f"elbow_pivot_disc_{i}",
        )

    second_link = model.part("second_link")
    second_link.visual(
        Box((0.075, 0.085, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_mat,
        name="elbow_lug",
    )
    second_link.visual(
        Cylinder(radius=0.035, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_mat,
        name="elbow_bushing",
    )
    second_link.visual(
        Box((0.046, 0.036, 0.650)),
        origin=Origin(xyz=(0.0, 0.0, -0.3575)),
        material=arm_mat,
        name="long_strut",
    )
    second_link.visual(
        Box((0.280, 0.180, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.695)),
        material=plate_mat,
        name="rectangular_plate",
    )
    second_link.visual(
        Box((0.220, 0.135, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.7145)),
        material=sensor_mat,
        name="sensor_face",
    )

    model.articulation(
        "beam_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, 1.560)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "shuttle_to_first",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=first_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-1.40, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    shuttle = object_model.get_part("shuttle")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    slide = object_model.get_articulation("beam_to_shuttle")
    shoulder = object_model.get_articulation("shuttle_to_first")
    elbow = object_model.get_articulation("first_to_second")

    ctx.expect_gap(
        beam,
        shuttle,
        axis="z",
        positive_elem="lower_flange",
        negative_elem="wear_shoe",
        max_gap=0.001,
        max_penetration=0.0,
        name="shuttle wear shoe rides under the fixed rail",
    )
    ctx.expect_overlap(
        shuttle,
        beam,
        axes="xy",
        elem_a="wear_shoe",
        elem_b="lower_flange",
        min_overlap=0.10,
        name="shuttle carriage footprint remains on rail flange",
    )

    shoulder_parent = getattr(getattr(shoulder, "parent", None), "name", getattr(shoulder, "parent", None))
    slide_child = getattr(getattr(slide, "child", None), "name", getattr(slide, "child", None))
    ctx.check(
        "hanging arm is mounted to the traveling shuttle",
        shoulder_parent == "shuttle" and slide_child == "shuttle",
        details=f"shoulder_parent={shoulder_parent}, slide_child={slide_child}",
    )

    rest_shuttle = ctx.part_world_position(shuttle)
    with ctx.pose({slide: 0.50}):
        moved_shuttle = ctx.part_world_position(shuttle)
        ctx.expect_overlap(
            shuttle,
            beam,
            axes="xy",
            elem_a="wear_shoe",
            elem_b="lower_flange",
            min_overlap=0.10,
            name="translated shuttle still bears on rail flange",
        )
    ctx.check(
        "prismatic joint travels along the beam axis",
        rest_shuttle is not None
        and moved_shuttle is not None
        and moved_shuttle[0] > rest_shuttle[0] + 0.45
        and abs(moved_shuttle[1] - rest_shuttle[1]) < 1e-6,
        details=f"rest={rest_shuttle}, moved={moved_shuttle}",
    )

    rest_elbow = ctx.part_world_position(second_link)
    with ctx.pose({shoulder: 0.60}):
        swung_elbow = ctx.part_world_position(second_link)
    ctx.check(
        "first revolute joint swings the short link",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[0] < rest_elbow[0] - 0.15
        and swung_elbow[2] > rest_elbow[2] + 0.04,
        details=f"rest_elbow={rest_elbow}, swung_elbow={swung_elbow}",
    )

    rest_tip_aabb = ctx.part_world_aabb(second_link)
    with ctx.pose({elbow: -0.75}):
        bent_tip_aabb = ctx.part_world_aabb(second_link)
    ctx.check(
        "second revolute joint swings the long link and plate",
        rest_tip_aabb is not None
        and bent_tip_aabb is not None
        and bent_tip_aabb[1][0] > rest_tip_aabb[1][0] + 0.30,
        details=f"rest_aabb={rest_tip_aabb}, bent_aabb={bent_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
