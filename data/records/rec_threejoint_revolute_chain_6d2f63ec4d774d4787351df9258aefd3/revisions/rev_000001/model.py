from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.28
FOOT_WIDTH = 0.12
FOOT_THICKNESS = 0.020
FOOT_CENTER_X = 0.020

LINK_LENGTH = 0.46
SECOND_LINK_LENGTH = 0.40
NOSE_LENGTH = 0.16
PLATE_WIDTH = 0.074
PLATE_THICKNESS = 0.012
PLATE_RAISE = 0.003
HOLE_RADIUS = 0.017
PIN_RADIUS = 0.010
LAYER_SEP = 0.026
FIRST_LAYER_Z = 0.036


def _capsule_plate_shape(
    length: float,
    width: float,
    thickness: float,
    *,
    holes: tuple[float, ...],
    boss_radius: float = 0.030,
    boss_raise: float = PLATE_RAISE,
) -> cq.Workplane:
    """Flat rounded plate whose joint centers lie at x=0 and x=length."""
    r = width / 2.0
    body = (
        cq.Workplane("XY")
        .moveTo(0.0, -r)
        .lineTo(length, -r)
        .threePointArc((length + r, 0.0), (length, r))
        .lineTo(0.0, r)
        .threePointArc((-r, 0.0), (0.0, -r))
        .close()
        .extrude(thickness / 2.0, both=True)
    )

    boss_height = thickness + 2.0 * boss_raise
    for x in holes:
        boss = cq.Workplane("XY").center(x, 0.0).circle(boss_radius).extrude(
            boss_height / 2.0, both=True
        )
        body = body.union(boss)

    for x in holes:
        cutter = cq.Workplane("XY").center(x, 0.0).circle(HOLE_RADIUS).extrude(
            (boss_height + 0.030) / 2.0, both=True
        )
        body = body.cut(cutter)

    return body


def _foot_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)
        .edges("|Z")
        .fillet(0.012)
        .translate((FOOT_CENTER_X, 0.0, FOOT_THICKNESS / 2.0))
    )


def _nose_hook_shape() -> cq.Workplane:
    """Short link with a fixed up-turned hook tab at its far end."""
    base = _capsule_plate_shape(
        NOSE_LENGTH,
        PLATE_WIDTH * 0.92,
        PLATE_THICKNESS,
        holes=(0.0,),
        boss_radius=0.028,
    )

    hook_thickness = PLATE_THICKNESS * 1.05
    neck = cq.Workplane("XY").box(0.090, 0.032, hook_thickness).translate(
        (NOSE_LENGTH + 0.034, -0.004, 0.0)
    )
    lip = cq.Workplane("XY").box(0.032, 0.102, hook_thickness).translate(
        (NOSE_LENGTH + 0.077, 0.036, 0.0)
    )
    rounded_toe = (
        cq.Workplane("XY")
        .center(NOSE_LENGTH + 0.077, 0.088)
        .circle(0.017)
        .extrude(hook_thickness / 2.0, both=True)
    )
    throat_marker = (
        cq.Workplane("XY")
        .center(NOSE_LENGTH + 0.053, 0.035)
        .slot2D(0.055, 0.018, 90.0)
        .extrude((hook_thickness + 0.010) / 2.0, both=True)
    )
    return base.union(neck).union(lip).union(rounded_toe).cut(throat_marker)


def _add_vertical_pin(
    part,
    *,
    x: float,
    child_layer_z: float,
    plate_top_z: float,
    pin_material: Material,
    name_prefix: str,
) -> None:
    """Parent-owned rivet, pin, and retainer washer for the next higher link."""
    part.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(x, 0.0, plate_top_z + 0.002)),
        material=pin_material,
        name=f"{name_prefix}_rivet",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=child_layer_z + 0.035),
        origin=Origin(xyz=(x, 0.0, (child_layer_z + 0.035) / 2.0 - 0.004)),
        material=pin_material,
        name=f"{name_prefix}_pin",
    )
    part.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(x, 0.0, child_layer_z + plate_top_z + 0.002)),
        material=pin_material,
        name=f"{name_prefix}_washer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_folding_bracket_chain")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    blue_steel = model.material("blue_steel", rgba=(0.08, 0.20, 0.36, 1.0))
    zinc = model.material("zinc_plated", rgba=(0.72, 0.74, 0.70, 1.0))
    orange = model.material("orange_tab", rgba=(0.92, 0.42, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    foot = model.part("foot")
    foot.visual(
        mesh_from_cadquery(_foot_plate_shape(), "foot_plate"),
        material=dark_steel,
        name="base_plate",
    )
    foot.visual(
        Cylinder(radius=0.032, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS + 0.0035)),
        material=zinc,
        name="pivot_boss",
    )
    foot.visual(
        Cylinder(radius=PIN_RADIUS, length=0.077),
        origin=Origin(xyz=(0.0, 0.0, FOOT_THICKNESS + 0.0385)),
        material=zinc,
        name="pivot_pin",
    )
    foot.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, FIRST_LAYER_Z + PLATE_THICKNESS / 2.0 + PLATE_RAISE + 0.004)),
        material=zinc,
        name="pivot_washer",
    )
    for i, (x, y) in enumerate(
        (
            (-0.080, -0.037),
            (-0.080, 0.037),
            (0.110, -0.037),
            (0.110, 0.037),
        )
    ):
        foot.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, FOOT_THICKNESS + 0.0015)),
            material=black,
            name=f"screw_head_{i}",
        )

    link_0 = model.part("long_link_0")
    link_0.visual(
        mesh_from_cadquery(
            _capsule_plate_shape(
                LINK_LENGTH,
                PLATE_WIDTH,
                PLATE_THICKNESS,
                holes=(0.0, LINK_LENGTH),
            ),
            "long_link_0_plate",
        ),
        material=blue_steel,
        name="plate",
    )
    _add_vertical_pin(
        link_0,
        x=LINK_LENGTH,
        child_layer_z=LAYER_SEP,
        plate_top_z=PLATE_THICKNESS / 2.0 + PLATE_RAISE,
        pin_material=zinc,
        name_prefix="elbow",
    )

    link_1 = model.part("long_link_1")
    link_1.visual(
        mesh_from_cadquery(
            _capsule_plate_shape(
                SECOND_LINK_LENGTH,
                PLATE_WIDTH,
                PLATE_THICKNESS,
                holes=(0.0, SECOND_LINK_LENGTH),
            ),
            "long_link_1_plate",
        ),
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=blue_steel,
        name="plate",
    )
    _add_vertical_pin(
        link_1,
        x=-SECOND_LINK_LENGTH,
        child_layer_z=LAYER_SEP,
        plate_top_z=PLATE_THICKNESS / 2.0 + PLATE_RAISE,
        pin_material=zinc,
        name_prefix="nose",
    )

    nose = model.part("nose_link")
    nose.visual(
        mesh_from_cadquery(_nose_hook_shape(), "nose_link_with_hook"),
        material=orange,
        name="hook_tab",
    )
    nose.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(NOSE_LENGTH + 0.077, 0.088, 0.005)),
        material=black,
        name="rubber_tip",
    )

    model.articulation(
        "foot_pivot",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, FIRST_LAYER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "plate_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, LAYER_SEP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.75, upper=0.0, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "nose_pivot",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=nose,
        origin=Origin(xyz=(-SECOND_LINK_LENGTH, 0.0, LAYER_SEP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.90, effort=10.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("foot_pivot"),
        object_model.get_articulation("plate_pivot"),
        object_model.get_articulation("nose_pivot"),
    ]
    link_0 = object_model.get_part("long_link_0")
    link_1 = object_model.get_part("long_link_1")
    nose = object_model.get_part("nose_link")

    ctx.check(
        "three parallel revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    with ctx.pose({"foot_pivot": 0.0, "plate_pivot": 0.0, "nose_pivot": 0.0}):
        ctx.expect_overlap(
            link_1,
            link_0,
            axes="x",
            min_overlap=0.35,
            name="folded long links pack over one another",
        )
        folded_nose = ctx.part_world_position(nose)

    with ctx.pose({"foot_pivot": 0.28, "plate_pivot": -2.70, "nose_pivot": 2.86}):
        extended_nose = ctx.part_world_position(nose)

    ctx.check(
        "arm extends outward in a shallow arc",
        folded_nose is not None
        and extended_nose is not None
        and extended_nose[0] > folded_nose[0] + 0.55
        and 0.15 < extended_nose[1] < 0.45,
        details=f"folded={folded_nose}, extended={extended_nose}",
    )

    return ctx.report()


object_model = build_object_model()
