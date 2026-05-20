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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PIN_RADIUS = 0.010
EYE_CLEARANCE_RADIUS = 0.014
EYE_OUTER_RADIUS = 0.026
EYE_WIDTH = 0.034
BODY_WIDTH = 0.044
BODY_HEIGHT = 0.035
CHEEK_THICKNESS = 0.019
CHEEK_Y = 0.031
CHEEK_HEIGHT = 0.058
CHEEK_LENGTH = 0.090
PIN_LENGTH = 0.094
LINK_LENGTHS = (0.310, 0.235, 0.165)


def _eye_mesh(name: str):
    """Washer-like proximal lug with a real clearance hole for the pin barrel."""
    eye = (
        cq.Workplane("XZ")
        .circle(EYE_OUTER_RADIUS)
        .circle(EYE_CLEARANCE_RADIUS)
        .extrude(EYE_WIDTH / 2.0, both=True)
    )
    return mesh_from_cadquery(eye, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_finger_chain")

    aluminum = Material("warm_anodized_aluminum", rgba=(0.86, 0.58, 0.22, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_steel = Material("dark_pin_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    base_mat = Material("matte_black_block", rgba=(0.025, 0.028, 0.030, 1.0))

    root = model.part("root_block")
    root.visual(
        Box((0.155, 0.110, 0.110)),
        origin=Origin(xyz=(-0.1125, 0.0, 0.0)),
        material=base_mat,
        name="root_block",
    )
    root.visual(
        Box((0.090, CHEEK_THICKNESS, 0.064)),
        origin=Origin(xyz=(-0.010, CHEEK_Y, 0.0)),
        material=steel,
        name="cheek_0",
    )
    root.visual(
        Box((0.090, CHEEK_THICKNESS, 0.064)),
        origin=Origin(xyz=(-0.010, -CHEEK_Y, 0.0)),
        material=steel,
        name="cheek_1",
    )
    root.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="root_pin",
    )
    root.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pin_head_0",
    )
    root.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pin_head_1",
    )

    links = []
    for index, length in enumerate(LINK_LENGTHS):
        link = model.part(f"finger_link_{index}")
        links.append(link)

        body_start = EYE_CLEARANCE_RADIUS + 0.004
        body_end = length - 0.055
        body_length = body_end - body_start
        link.visual(
            _eye_mesh(f"proximal_eye_{index}"),
            material=aluminum,
            name="proximal_eye",
        )
        link.visual(
            Box((body_length, BODY_WIDTH, BODY_HEIGHT)),
            origin=Origin(xyz=((body_start + body_end) / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="tapered_bar",
        )
        # The paired side cheeks overlap the bar very slightly at their inner
        # faces, making each moving link one rigid supported fork.
        for cheek_index, y in enumerate((CHEEK_Y, -CHEEK_Y)):
            link.visual(
                Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
                origin=Origin(xyz=(length - 0.022, y, 0.0)),
                material=steel,
                name=f"distal_cheek_{cheek_index}",
            )
        link.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(xyz=(length, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="distal_pin",
        )
        link.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(length, 0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pin_head_0",
        )
        link.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(length, -0.050, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pin_head_1",
        )

    limits = MotionLimits(effort=8.0, velocity=3.0, lower=-0.30, upper=1.25)
    model.articulation(
        "root_knuckle",
        ArticulationType.REVOLUTE,
        parent=root,
        child=links[0],
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=links[0],
        child=links[1],
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "tip_knuckle",
        ArticulationType.REVOLUTE,
        parent=links[1],
        child=links[2],
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_block")
    link_0 = object_model.get_part("finger_link_0")
    link_1 = object_model.get_part("finger_link_1")
    link_2 = object_model.get_part("finger_link_2")
    root_joint = object_model.get_articulation("root_knuckle")
    middle_joint = object_model.get_articulation("middle_knuckle")
    tip_joint = object_model.get_articulation("tip_knuckle")

    ctx.check(
        "three serial revolute knuckles",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "finger links decrease in length",
        LINK_LENGTHS[0] > LINK_LENGTHS[1] > LINK_LENGTHS[2],
        details=f"lengths={LINK_LENGTHS}",
    )

    ctx.expect_gap(
        link_0,
        root,
        axis="x",
        positive_elem="proximal_eye",
        negative_elem="root_block",
        min_gap=0.004,
        name="first eye clears root block",
    )
    ctx.expect_within(
        root,
        link_0,
        axes="xz",
        inner_elem="root_pin",
        outer_elem="proximal_eye",
        margin=0.0,
        name="root pin is centered in first eye",
    )
    ctx.expect_within(
        link_0,
        link_1,
        axes="xz",
        inner_elem="distal_pin",
        outer_elem="proximal_eye",
        margin=0.0,
        name="middle pin is centered in second eye",
    )
    ctx.expect_within(
        link_1,
        link_2,
        axes="xz",
        inner_elem="distal_pin",
        outer_elem="proximal_eye",
        margin=0.0,
        name="tip pin is centered in third eye",
    )
    ctx.expect_overlap(
        root,
        link_0,
        axes="y",
        elem_a="root_pin",
        elem_b="proximal_eye",
        min_overlap=0.030,
        name="root pin spans the first eye",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_eye",
        min_overlap=0.030,
        name="middle pin spans the second eye",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_eye",
        min_overlap=0.030,
        name="tip pin spans the third eye",
    )

    rest_tip_aabb = ctx.part_world_aabb(link_2)
    with ctx.pose({root_joint: 0.75, middle_joint: 0.55, tip_joint: 0.35}):
        flexed_tip_aabb = ctx.part_world_aabb(link_2)

    ctx.check(
        "positive knuckle motion curls in one plane",
        rest_tip_aabb is not None
        and flexed_tip_aabb is not None
        and flexed_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.16,
        details=f"rest_tip_aabb={rest_tip_aabb}, flexed_tip_aabb={flexed_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
