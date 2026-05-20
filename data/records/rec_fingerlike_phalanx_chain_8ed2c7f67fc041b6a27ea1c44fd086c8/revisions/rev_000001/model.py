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


PIN_RADIUS = 0.0065
KNUCKLE_RADIUS = 0.023
CENTRAL_BARREL_LEN = 0.030
SIDE_LUG_LEN = 0.020
SIDE_LUG_Y = CENTRAL_BARREL_LEN / 2.0 + 0.003 + SIDE_LUG_LEN / 2.0
PIN_LEN = 0.092
PIN_HEAD_Y = PIN_LEN / 2.0

PROXIMAL_LEN = 0.145
MIDDLE_LEN = 0.115
DISTAL_LEN = 0.090


def _y_cylinder_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Origin for a cylinder whose local Z axis is turned into the linkage Y axis."""

    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_cross_pin(part, prefix: str, x: float, steel) -> None:
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LEN),
        origin=_y_cylinder_origin(x),
        material=steel,
        name=f"{prefix}_pin",
    )
    for suffix, y in (("neg", -PIN_HEAD_Y), ("pos", PIN_HEAD_Y)):
        part.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=_y_cylinder_origin(x, y),
            material=steel,
            name=f"{prefix}_pin_head_{suffix}",
        )


def _add_fork_lugs(part, prefix: str, x: float, metal, steel) -> None:
    for suffix, y in (("neg", -SIDE_LUG_Y), ("pos", SIDE_LUG_Y)):
        part.visual(
            Box((0.062, SIDE_LUG_LEN, 0.026)),
            origin=Origin(xyz=(x - 0.024, y, 0.0)),
            material=metal,
            name=f"{prefix}_fork_arm_{suffix}",
        )
        part.visual(
            Cylinder(radius=KNUCKLE_RADIUS, length=SIDE_LUG_LEN),
            origin=_y_cylinder_origin(x, y),
            material=metal,
            name=f"{prefix}_lug_{suffix}",
        )
    _add_cross_pin(part, prefix, x, steel)


def _add_link_body(
    part,
    prefix: str,
    length: float,
    metal,
    rubber,
    steel,
    *,
    distal_fork: bool,
    fingertip: bool = False,
) -> None:
    part.visual(
        Cylinder(radius=KNUCKLE_RADIUS, length=CENTRAL_BARREL_LEN),
        origin=_y_cylinder_origin(0.0),
        material=metal,
        name=f"{prefix}_barrel",
    )
    part.visual(
        Box((0.052, CENTRAL_BARREL_LEN, 0.022)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=metal,
        name=f"{prefix}_neck",
    )

    body_start = 0.045
    body_end = length - (0.050 if distal_fork else 0.018)
    if body_end > body_start:
        body_len = body_end - body_start
        body_x = (body_start + body_end) / 2.0
        body_width = 0.058 if distal_fork else 0.050
        part.visual(
            Box((body_len, body_width, 0.020)),
            origin=Origin(xyz=(body_x, 0.0, 0.0)),
            material=metal,
            name=f"{prefix}_body",
        )
        pad_len = max(0.018, body_len - 0.014)
        part.visual(
            Box((pad_len, body_width - 0.020, 0.004)),
            origin=Origin(xyz=(body_x, 0.0, 0.0115)),
            material=rubber,
            name=f"{prefix}_rubber_pad",
        )

    if distal_fork:
        _add_fork_lugs(part, prefix, length, metal, steel)

    if fingertip:
        part.visual(
            Sphere(radius=0.026),
            origin=Origin(xyz=(length, 0.0, 0.0)),
            material=rubber,
            name=f"{prefix}_rounded_tip",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_knuckle_finger_linkage")

    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.10, 0.12, 1.0))
    blue_aluminum = model.material("blue_gray_anodized_aluminum", rgba=(0.16, 0.22, 0.28, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    base_tab = model.part("base_tab")
    base_tab.visual(
        Box((0.105, 0.082, 0.018)),
        origin=Origin(xyz=(-0.073, 0.0, -0.036)),
        material=dark_aluminum,
        name="mount_plate",
    )
    for suffix, y in (("neg", -SIDE_LUG_Y), ("pos", SIDE_LUG_Y)):
        base_tab.visual(
            Box((0.066, SIDE_LUG_LEN, 0.060)),
            origin=Origin(xyz=(-0.012, y, -0.008)),
            material=dark_aluminum,
            name=f"base_cheek_{suffix}",
        )
        base_tab.visual(
            Cylinder(radius=KNUCKLE_RADIUS, length=SIDE_LUG_LEN),
            origin=_y_cylinder_origin(0.0, y, 0.0),
            material=dark_aluminum,
            name=f"base_lug_{suffix}",
        )
    _add_cross_pin(base_tab, "base", 0.0, brushed_steel)

    proximal_phalanx = model.part("proximal_phalanx")
    _add_link_body(
        proximal_phalanx,
        "proximal",
        PROXIMAL_LEN,
        blue_aluminum,
        black_rubber,
        brushed_steel,
        distal_fork=True,
    )

    middle_phalanx = model.part("middle_phalanx")
    _add_link_body(
        middle_phalanx,
        "middle",
        MIDDLE_LEN,
        blue_aluminum,
        black_rubber,
        brushed_steel,
        distal_fork=True,
    )

    distal_phalanx = model.part("distal_phalanx")
    _add_link_body(
        distal_phalanx,
        "distal",
        DISTAL_LEN,
        blue_aluminum,
        black_rubber,
        brushed_steel,
        distal_fork=False,
        fingertip=True,
    )

    flex_limits = MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35)
    model.articulation(
        "base_knuckle",
        ArticulationType.REVOLUTE,
        parent=base_tab,
        child=proximal_phalanx,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=flex_limits,
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal_phalanx,
        child=middle_phalanx,
        origin=Origin(xyz=(PROXIMAL_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "distal_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle_phalanx,
        child=distal_phalanx,
        origin=Origin(xyz=(MIDDLE_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_tab")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")
    base_knuckle = object_model.get_articulation("base_knuckle")
    middle_knuckle = object_model.get_articulation("middle_knuckle")
    distal_knuckle = object_model.get_articulation("distal_knuckle")

    for parent, child, parent_pin, child_barrel in (
        (base, proximal, "base_pin", "proximal_barrel"),
        (proximal, middle, "proximal_pin", "middle_barrel"),
        (middle, distal, "middle_pin", "distal_barrel"),
    ):
        ctx.allow_overlap(
            parent,
            child,
            elem_a=parent_pin,
            elem_b=child_barrel,
            reason="The steel hinge pin is intentionally modeled as captured through the child's broad knuckle barrel.",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="xyz",
            elem_a=parent_pin,
            elem_b=child_barrel,
            min_overlap=0.010,
            name=f"{parent_pin} is captured in {child_barrel}",
        )

    for part, lug_prefix, barrel_part, barrel_name in (
        (base, "base_lug", proximal, "proximal_barrel"),
        (proximal, "proximal_lug", middle, "middle_barrel"),
        (middle, "middle_lug", distal, "distal_barrel"),
    ):
        ctx.expect_gap(
            part,
            barrel_part,
            axis="y",
            positive_elem=f"{lug_prefix}_pos",
            negative_elem=barrel_name,
            min_gap=0.001,
            max_gap=0.010,
            name=f"{lug_prefix}_pos clears {barrel_name}",
        )
        ctx.expect_gap(
            barrel_part,
            part,
            axis="y",
            positive_elem=barrel_name,
            negative_elem=f"{lug_prefix}_neg",
            min_gap=0.001,
            max_gap=0.010,
            name=f"{barrel_name} clears {lug_prefix}_neg",
        )

    ctx.check(
        "three revolute knuckles",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (base_knuckle, middle_knuckle, distal_knuckle)),
        details="The linkage should contain exactly the three prompted revolute knuckle joints.",
    )
    ctx.check(
        "parallel knuckle axes",
        all(tuple(j.axis) == (0.0, -1.0, 0.0) for j in (base_knuckle, middle_knuckle, distal_knuckle)),
        details=f"axes={[j.axis for j in (base_knuckle, middle_knuckle, distal_knuckle)]}",
    )

    rest_aabb = ctx.part_world_aabb(distal)
    with ctx.pose({base_knuckle: 0.70, middle_knuckle: 0.85, distal_knuckle: 0.65}):
        flexed_aabb = ctx.part_world_aabb(distal)
    ctx.check(
        "distal phalanx flexes upward",
        rest_aabb is not None
        and flexed_aabb is not None
        and flexed_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest={rest_aabb}, flexed={flexed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
