from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.285
BODY_WIDTH = 0.250
BODY_HEIGHT = 0.040
PLATE_DEPTH = 0.206
PLATE_WIDTH = 0.188

BRIDGE_DEPTH = 0.018
BRIDGE_WIDTH = 0.074
BRIDGE_HEIGHT = 0.021
BRIDGE_X = -0.132
BRIDGE_POST_WIDTH = 0.012
BRIDGE_ROOF_THICKNESS = 0.004

HINGE_X = -0.126
HINGE_Z = 0.0515

LID_DEPTH = 0.255
LID_WIDTH = 0.242
LID_HEIGHT = 0.054
LID_REAR_CLEARANCE = 0.004
LID_BOTTOM_Z = -0.009

BARREL_RADIUS = 0.0055
BARREL_LENGTH = 0.050

DIAL_X = 0.092
DIAL_Z = 0.024
DIAL_SHAFT_LENGTH = 0.004
DIAL_DIAMETER = 0.030
DIAL_HEIGHT = 0.014


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _grid_cutter(
    *,
    center_x: float,
    center_y: float,
    span_x: float,
    span_y: float,
    plane_z: float,
    groove_depth: float,
    from_top: bool,
    count_x: int = 4,
    count_y: int = 4,
    groove_width: float = 0.003,
) -> cq.Workplane:
    half_x = span_x / 2.0
    half_y = span_y / 2.0
    z = plane_z - groove_depth / 2.0 if from_top else plane_z + groove_depth / 2.0
    cutter: cq.Workplane | None = None

    for index in range(count_x):
        x = center_x - half_x + span_x * (index + 1) / (count_x + 1)
        slot = _box((x, center_y, z), (groove_width, span_y, groove_depth))
        cutter = slot if cutter is None else cutter.union(slot)

    for index in range(count_y):
        y = center_y - half_y + span_y * (index + 1) / (count_y + 1)
        slot = _box((center_x, y, z), (span_x, groove_width, groove_depth))
        cutter = slot if cutter is None else cutter.union(slot)

    assert cutter is not None
    return cutter


def _build_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    base = base.cut(
        _box(
            (0.0, 0.0, BODY_HEIGHT - 0.0018),
            (PLATE_DEPTH, PLATE_WIDTH, 0.0036),
        )
    )
    base = base.cut(
        _grid_cutter(
            center_x=0.0,
            center_y=0.0,
            span_x=PLATE_DEPTH - 0.024,
            span_y=PLATE_WIDTH - 0.024,
            plane_z=BODY_HEIGHT - 0.0036,
            groove_depth=0.0018,
            from_top=True,
        )
    )
    return base


def _build_lid_shape() -> cq.Workplane:
    lid_center_x = LID_REAR_CLEARANCE + LID_DEPTH / 2.0
    lid = (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, LID_HEIGHT)
        .translate(
            (
                lid_center_x,
                0.0,
                LID_BOTTOM_Z + LID_HEIGHT / 2.0,
            )
        )
        .faces(">Z")
        .edges()
        .fillet(0.018)
    )

    crown = (
        cq.Workplane("XY")
        .rect(LID_DEPTH * 0.80, LID_WIDTH * 0.88)
        .workplane(offset=0.018)
        .rect(LID_DEPTH * 0.62, LID_WIDTH * 0.68)
        .loft(combine=False)
        .translate((lid_center_x, 0.0, 0.034))
    )
    lid = lid.union(crown)

    lid = lid.cut(
        _grid_cutter(
            center_x=lid_center_x,
            center_y=0.0,
            span_x=0.188,
            span_y=0.176,
            plane_z=LID_BOTTOM_Z,
            groove_depth=0.0018,
            from_top=False,
        )
    )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_waffle_press")

    housing_finish = model.material("housing_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.60, 0.61, 0.63, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_body"),
        material=housing_finish,
        name="base_body",
    )
    base.visual(
        Box((BRIDGE_DEPTH, BRIDGE_POST_WIDTH, BRIDGE_HEIGHT - BRIDGE_ROOF_THICKNESS)),
        material=housing_finish,
        origin=Origin(
            xyz=(
                BRIDGE_X,
                (BARREL_LENGTH + BRIDGE_POST_WIDTH) / 2.0,
                BODY_HEIGHT + (BRIDGE_HEIGHT - BRIDGE_ROOF_THICKNESS) / 2.0,
            )
        ),
        name="bridge_post_0",
    )
    base.visual(
        Box((BRIDGE_DEPTH, BRIDGE_POST_WIDTH, BRIDGE_HEIGHT - BRIDGE_ROOF_THICKNESS)),
        material=housing_finish,
        origin=Origin(
            xyz=(
                BRIDGE_X,
                -(BARREL_LENGTH + BRIDGE_POST_WIDTH) / 2.0,
                BODY_HEIGHT + (BRIDGE_HEIGHT - BRIDGE_ROOF_THICKNESS) / 2.0,
            )
        ),
        name="bridge_post_1",
    )
    base.visual(
        Box((BRIDGE_DEPTH, BRIDGE_WIDTH, BRIDGE_ROOF_THICKNESS)),
        material=housing_finish,
        origin=Origin(
            xyz=(
                BRIDGE_X,
                0.0,
                BODY_HEIGHT + BRIDGE_HEIGHT - BRIDGE_ROOF_THICKNESS / 2.0,
            )
        ),
        name="bridge_roof",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "lid_shell"),
        material=housing_finish,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        material=hardware_finish,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        name="hinge_barrel",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0032, length=DIAL_SHAFT_LENGTH),
        material=hardware_finish,
        origin=Origin(
            xyz=(0.0, DIAL_SHAFT_LENGTH / 2.0, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAL_DIAMETER,
                DIAL_HEIGHT,
                body_style="tapered",
                top_diameter=0.024,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=14, depth=0.0009),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "thermostat_dial",
        ),
        material=dial_finish,
        origin=Origin(xyz=(0.0, DIAL_SHAFT_LENGTH, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        name="dial_knob",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=0.0, upper=1.2),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(DIAL_X, BODY_WIDTH / 2.0, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    hinge = object_model.get_articulation("base_to_lid")
    dial_joint = object_model.get_articulation("base_to_dial")

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="base_body",
        max_gap=0.006,
        max_penetration=0.0,
        name="closed lid hovers just above lower body",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="base_body",
        min_overlap=0.180,
        name="closed lid covers lower cooking area",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({hinge: upper}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward visibly",
            closed_aabb is not None
            and open_aabb is not None
            and float(open_aabb[1][2]) > float(closed_aabb[1][2]) + 0.050,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    ctx.expect_gap(
        dial,
        base,
        axis="y",
        positive_elem="dial_shaft",
        negative_elem="base_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial shaft starts flush from the side wall",
    )
    ctx.expect_origin_gap(
        dial,
        base,
        axis="y",
        min_gap=0.120,
        max_gap=0.130,
        name="dial is mounted on the right wall",
    )
    limits = dial_joint.motion_limits
    ctx.check(
        "thermostat dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"articulation_type={dial_joint.articulation_type}, limits={limits}",
    )

    return ctx.report()


object_model = build_object_model()
