from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.34
BASE_DEPTH = 0.18
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.028
PEDESTAL_HEIGHT = 0.145
BOWL_RADIUS = 0.075
BOWL_SHELL_THICKNESS = 0.003
BOWL_CENTER_Z = 0.260
BOWL_OFFSET_X = 0.096
CUP_CENTER = (0.0, 0.030, 0.108)


def _make_bowl_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").sphere(BOWL_RADIUS)
    inner = cq.Workplane("XY").sphere(BOWL_RADIUS - BOWL_SHELL_THICKNESS)
    shell = outer.cut(inner)

    top_cutter = cq.Workplane("XY").box(0.30, 0.30, 0.20).translate((0.0, 0.0, 0.152))
    bottom_cutter = cq.Workplane("XY").box(0.30, 0.30, 0.20).translate((0.0, 0.0, -0.170))
    return shell.cut(top_cutter).cut(bottom_cutter)


def _make_collection_cup_shell() -> cq.Workplane:
    outer_radius = 0.043
    inner_radius = 0.039
    height = 0.070
    bottom_thickness = 0.004

    body = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=bottom_thickness)
        .circle(inner_radius)
        .extrude(height - bottom_thickness)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=height - 0.006)
        .circle(outer_radius + 0.006)
        .circle(outer_radius)
        .extrude(0.006)
    )
    door_opening = cq.Workplane("XY").box(0.056, 0.060, 0.040).translate((0.0, outer_radius, 0.022))

    return body.cut(cavity).union(rim).cut(door_opening).translate((0.0, 0.0, -height / 2.0))


def _add_bowl_vendor(model: ArticulatedObject, base, index: int, x_pos: float) -> None:
    bowl = model.part(f"bowl_{index}")
    bowl.visual(
        mesh_from_cadquery(_make_bowl_shell(), f"bowl_{index}_shell"),
        material="clear_globe",
        name="globe_shell",
    )
    bowl.visual(
        Cylinder(radius=0.027, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, -0.0835)),
        material="chrome",
        name="neck",
    )
    bowl.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.061, -0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="dispense_housing",
    )
    bowl.visual(
        Box((0.026, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.049, -0.052)),
        material="chrome",
        name="chute",
    )
    bowl.visual(
        Box((0.100, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.028, 0.050)),
        material="chrome",
        name="lid_stop",
    )

    model.articulation(
        f"base_to_bowl_{index}",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(x_pos, 0.0, BOWL_CENTER_Z)),
    )

    knob = model.part(f"knob_{index}")
    knob.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="chrome",
        name="knob_bezel",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.036,
                0.018,
                body_style="skirted",
                top_diameter=0.029,
                skirt=KnobSkirt(0.041, 0.004, flare=0.08),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            f"knob_{index}_cap",
        ),
        material="black_plastic",
        name="knob_cap",
    )
    model.articulation(
        f"bowl_{index}_knob",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=knob,
        origin=Origin(xyz=(0.0, 0.075, -0.030), rpy=(-pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    lid = model.part(f"lid_{index}")
    lid.visual(
        Cylinder(radius=0.033, length=0.004),
        origin=Origin(xyz=(0.0, 0.030, 0.002)),
        material="chrome",
        name="lid_plate",
    )
    lid.visual(
        Cylinder(radius=0.003, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material="chrome",
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.042, 0.008)),
        material="black_plastic",
        name="lid_grip",
    )
    model.articulation(
        f"bowl_{index}_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, -0.028, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=1.0, velocity=2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_globe_candy_vendor")

    model.material("chrome", rgba=(0.80, 0.81, 0.83, 1.0))
    model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("coin_door", rgba=(0.63, 0.65, 0.68, 1.0))
    model.material("clear_globe", rgba=(0.83, 0.93, 0.98, 0.28))

    base = model.part("base")
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="dark_metal",
        name="platform",
    )
    for index, x_pos in enumerate((-BOWL_OFFSET_X, BOWL_OFFSET_X)):
        base.visual(
            Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
            origin=Origin(xyz=(x_pos, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
            material="chrome",
            name=f"pedestal_{index}",
        )
    _add_bowl_vendor(model, base, 0, -BOWL_OFFSET_X)
    _add_bowl_vendor(model, base, 1, BOWL_OFFSET_X)

    collection_cup = model.part("collection_cup")
    collection_cup.visual(
        mesh_from_cadquery(_make_collection_cup_shell(), "collection_cup_shell"),
        material="chrome",
        name="cup_shell",
    )
    collection_cup.visual(
        Box((0.024, 0.038, 0.055)),
        origin=Origin(xyz=(0.0, -0.030, -0.0625)),
        material="chrome",
        name="cup_support",
    )
    collection_cup.visual(
        Box((0.052, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.040, -0.030)),
        material="chrome",
        name="door_lip",
    )
    model.articulation(
        "base_to_collection_cup",
        ArticulationType.FIXED,
        parent=base,
        child=collection_cup,
        origin=Origin(xyz=CUP_CENTER),
    )

    collection_door = model.part("collection_door")
    collection_door.visual(
        Box((0.050, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, 0.0015, 0.019)),
        material="coin_door",
        name="door_panel",
    )
    collection_door.visual(
        Cylinder(radius=0.003, length=0.050),
        origin=Origin(xyz=(0.0, 0.0005, 0.0015), rpy=(0.0, pi / 2.0, 0.0)),
        material="chrome",
        name="door_hinge_barrel",
    )
    collection_door.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0045, 0.035)),
        material="chrome",
        name="door_pull",
    )
    model.articulation(
        "collection_door_hinge",
        ArticulationType.REVOLUTE,
        parent=collection_cup,
        child=collection_door,
        origin=Origin(xyz=(0.0, 0.044, -0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=1.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    collection_cup = object_model.get_part("collection_cup")
    collection_door = object_model.get_part("collection_door")
    door_hinge = object_model.get_articulation("collection_door_hinge")

    ctx.expect_gap(
        collection_cup,
        base,
        axis="z",
        positive_elem="cup_support",
        negative_elem="platform",
        max_gap=0.001,
        max_penetration=0.0,
        name="shared cup is supported by the base",
    )

    for index in (0, 1):
        bowl = object_model.get_part(f"bowl_{index}")
        lid = object_model.get_part(f"lid_{index}")
        knob_joint = object_model.get_articulation(f"bowl_{index}_knob")
        lid_hinge = object_model.get_articulation(f"bowl_{index}_lid_hinge")

        ctx.expect_gap(
            bowl,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"bowl_{index} is seated on its pedestal",
        )
        ctx.expect_overlap(
            lid,
            bowl,
            axes="x",
            elem_a="lid_plate",
            elem_b="globe_shell",
            min_overlap=0.050,
            name=f"lid_{index} covers the refill opening",
        )
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            positive_elem="lid_plate",
            negative_elem="globe_shell",
            min_gap=0.006,
            max_gap=0.015,
            name=f"lid_{index} sits above the bowl opening",
        )

        knob_limits = knob_joint.motion_limits
        ctx.check(
            f"knob_{index} uses a continuous rotary joint",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and knob_limits is not None
            and knob_limits.lower is None
            and knob_limits.upper is None,
            details=f"type={knob_joint.articulation_type}, limits={knob_limits}",
        )

        lid_limits = lid_hinge.motion_limits
        closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper if lid_limits is not None else 0.0}):
            opened_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            f"lid_{index} opens upward",
            closed_lid_aabb is not None
            and opened_lid_aabb is not None
            and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.020,
            details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
        )

    closed_door_aabb = ctx.part_world_aabb(collection_door)
    door_limits = door_hinge.motion_limits
    with ctx.pose({door_hinge: door_limits.upper if door_limits is not None else 0.0}):
        opened_door_aabb = ctx.part_world_aabb(collection_door)
    ctx.check(
        "collection door swings outward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.012,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
