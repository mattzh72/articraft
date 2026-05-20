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
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_SIZE = 0.32
BASE_THICKNESS = 0.03
POST_RADIUS = 0.018
POST_HEIGHT = 1.52
TOP_COLLAR_RADIUS = 0.024
TOP_COLLAR_THICKNESS = 0.016
SWIVEL_HEIGHT = BASE_THICKNESS + POST_HEIGHT + TOP_COLLAR_THICKNESS + 0.015

ARM_LENGTH = 0.255
ARM_WIDTH = 0.028
ARM_THICKNESS = 0.018
YOKE_CENTER_Y = 0.286
YOKE_BASE_Z = -0.040
YOKE_AXIS_Z = YOKE_BASE_Z + 0.058


def _spotlight_housing() -> cq.Workplane:
    """Build a compact spotlight can with a hollow front and rear wall."""
    outer = cq.Workplane("XZ").circle(0.045).extrude(0.126).val()
    inner = (
        cq.Workplane("XZ")
        .workplane(offset=0.006)
        .circle(0.039)
        .extrude(0.114)
        .val()
    )
    bezel = (
        cq.Workplane("XZ")
        .workplane(offset=0.126)
        .circle(0.054)
        .circle(0.044)
        .extrude(0.012)
        .val()
    )
    rear_band = (
        cq.Workplane("XZ")
        .workplane(offset=0.012)
        .circle(0.041)
        .circle(0.037)
        .extrude(0.016)
        .val()
    )
    housing = cq.Workplane(obj=outer).cut(inner)
    housing = housing.union(cq.Workplane(obj=bezel))
    housing = housing.union(cq.Workplane(obj=rear_band))
    return housing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gallery_floor_lamp")

    powder_black = model.material("powder_black", rgba=(0.11, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.42, 0.44, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((BASE_SIZE, BASE_SIZE, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=powder_black,
        name="base_plate",
    )
    stand.visual(
        Box((0.19, 0.19, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=graphite,
        name="base_pad",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.006)),
        material=graphite,
        name="post_shoe",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + POST_HEIGHT / 2.0)),
        material=powder_black,
        name="post",
    )
    stand.visual(
        Cylinder(radius=TOP_COLLAR_RADIUS, length=TOP_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + POST_HEIGHT + TOP_COLLAR_THICKNESS / 2.0,
            )
        ),
        material=graphite,
        name="top_collar",
    )

    crossarm = model.part("crossarm")
    crossarm.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(),
        material=steel,
        name="pivot_barrel",
    )
    crossarm.visual(
        Box((0.060, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, -0.002)),
        material=graphite,
        name="pivot_block",
    )
    crossarm.visual(
        Box((ARM_WIDTH, ARM_LENGTH, ARM_THICKNESS)),
        origin=Origin(xyz=(0.0, ARM_LENGTH / 2.0, 0.0)),
        material=powder_black,
        name="arm_beam",
    )
    crossarm.visual(
        Box((0.026, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, 0.255, -0.022)),
        material=powder_black,
        name="yoke_drop",
    )
    crossarm.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.128, 0.046, 0.094),
                span_width=0.094,
                trunnion_diameter=0.0125,
                trunnion_center_z=0.058,
                base_thickness=0.012,
                corner_radius=0.004,
                center=False,
            ),
            "spotlight_yoke",
        ),
        origin=Origin(xyz=(0.0, YOKE_CENTER_Y, YOKE_BASE_Z)),
        material=powder_black,
        name="yoke",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_spotlight_housing(), "spotlight_housing"),
        origin=Origin(rpy=(0.0, 0.0, 3.141592653589793)),
        material=powder_black,
        name="housing",
    )
    head.visual(
        Cylinder(radius=0.00625, length=0.020),
        origin=Origin(xyz=(-0.046, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.00625, length=0.020),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="trunnion_1",
    )

    swivel = model.articulation(
        "stand_to_crossarm",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=crossarm,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-2.1,
            upper=2.1,
        ),
    )
    swivel.meta["qc_samples"] = [0.0, 1.2, -1.2]

    tilt = model.articulation(
        "crossarm_to_head",
        ArticulationType.REVOLUTE,
        parent=crossarm,
        child=head,
        origin=Origin(xyz=(0.0, YOKE_CENTER_Y, YOKE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-1.10,
            upper=0.95,
        ),
    )
    tilt.meta["qc_samples"] = [0.0, 0.7, -0.7]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    crossarm = object_model.get_part("crossarm")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("stand_to_crossarm")
    tilt = object_model.get_articulation("crossarm_to_head")

    ctx.allow_overlap(
        crossarm,
        head,
        elem_a="yoke",
        elem_b="trunnion_0",
        reason="The spotlight trunnion is intentionally represented as seated inside the yoke bearing at the tilt hinge.",
    )
    ctx.allow_overlap(
        crossarm,
        head,
        elem_a="yoke",
        elem_b="trunnion_1",
        reason="The opposite trunnion is intentionally represented as seated inside the matching yoke bearing.",
    )

    with ctx.pose({swivel: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            crossarm,
            stand,
            elem_a="pivot_barrel",
            elem_b="top_collar",
            name="crossarm rides on the top collar",
        )
        ctx.expect_gap(
            head,
            stand,
            axis="z",
            positive_elem="housing",
            negative_elem="base_plate",
            min_gap=1.45,
            name="spotlight sits well above the square base",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({swivel: 1.2, tilt: 0.0}):
        swiveled_head_pos = ctx.part_world_position(head)

    ctx.check(
        "crossarm swivel turns the head around the post",
        rest_head_pos is not None
        and swiveled_head_pos is not None
        and swiveled_head_pos[0] > rest_head_pos[0] + 0.20,
        details=f"rest={rest_head_pos}, swiveled={swiveled_head_pos}",
    )

    rest_housing_box = ctx.part_element_world_aabb(head, elem="housing")
    with ctx.pose({swivel: 0.0, tilt: 0.75}):
        tilted_housing_box = ctx.part_element_world_aabb(head, elem="housing")

    ctx.check(
        "spotlight tilts upward in the yoke",
        rest_housing_box is not None
        and tilted_housing_box is not None
        and tilted_housing_box[1][2] > rest_housing_box[1][2] + 0.05,
        details=f"rest={rest_housing_box}, tilted={tilted_housing_box}",
    )

    return ctx.report()


object_model = build_object_model()
