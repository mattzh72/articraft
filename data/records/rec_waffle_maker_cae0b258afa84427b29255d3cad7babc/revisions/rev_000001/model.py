from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


BASE_RADIUS = 0.145
BASE_HEIGHT = 0.062
PLATE_RADIUS = 0.118
PLATE_THICKNESS = 0.010
HINGE_RADIUS = 0.008
HINGE_X = -0.145
HINGE_Z = 0.070
BASE_KNUCKLE_LENGTH = 0.032
LID_KNUCKLE_LENGTH = 0.044
HINGE_Y_OFFSET = 0.044
LID_CENTER_X = 0.145


def make_base_shell() -> cq.Workplane:
    lower_body = cq.Workplane("XY").circle(BASE_RADIUS - 0.004).extrude(0.020)
    upper_body = (
        cq.Workplane("XY")
        .workplane(offset=0.020)
        .circle(BASE_RADIUS)
        .extrude(BASE_HEIGHT - 0.020)
    )
    shell = lower_body.union(upper_body)

    plate_recess = (
        cq.Workplane("XY")
        .circle(PLATE_RADIUS + 0.006)
        .extrude(0.010)
        .translate((0.0, 0.0, BASE_HEIGHT - 0.010))
    )
    return shell.cut(plate_recess)


def make_base_hinge_support() -> cq.Workplane:
    left_bridge = cq.Workplane("XY").box(0.028, 0.034, 0.016).translate((HINGE_X - 0.004, 0.043, 0.056))
    right_bridge = cq.Workplane("XY").box(0.028, 0.034, 0.016).translate((HINGE_X - 0.004, -0.043, 0.056))
    left_ear = (
        cq.Workplane("XY")
        .box(0.020, BASE_KNUCKLE_LENGTH, 0.024)
        .translate((HINGE_X, HINGE_Y_OFFSET, 0.060))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.020, BASE_KNUCKLE_LENGTH, 0.024)
        .translate((HINGE_X, -HINGE_Y_OFFSET, 0.060))
    )
    left_barrel = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(HINGE_RADIUS)
        .extrude(BASE_KNUCKLE_LENGTH / 2.0, both=True)
        .translate((0.0, HINGE_Y_OFFSET, 0.0))
    )
    right_barrel = (
        cq.Workplane("XZ")
        .center(HINGE_X, HINGE_Z)
        .circle(HINGE_RADIUS)
        .extrude(BASE_KNUCKLE_LENGTH / 2.0, both=True)
        .translate((0.0, -HINGE_Y_OFFSET, 0.0))
    )
    return left_bridge.union(right_bridge).union(left_ear).union(right_ear).union(left_barrel).union(right_barrel)


def make_lid_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .workplane(offset=0.026)
        .circle(BASE_RADIUS * 0.96)
        .workplane(offset=0.024)
        .circle(BASE_RADIUS * 0.72)
        .loft(combine=True)
        .translate((LID_CENTER_X, 0.0, -0.008))
    )

    inner_cavity = (
        cq.Workplane("XY")
        .circle(BASE_RADIUS - 0.009)
        .extrude(0.024)
        .translate((LID_CENTER_X, 0.0, -0.008))
    )
    rear_relief = cq.Workplane("XY").box(0.050, 0.126, 0.018).translate((0.010, 0.0, -0.001))
    return shell.cut(inner_cavity).cut(rear_relief)


def make_lid_hinge_bracket() -> cq.Workplane:
    rear_gusset = cq.Workplane("XY").box(0.032, 0.030, 0.014).translate((0.016, 0.0, 0.015))
    center_barrel = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HINGE_RADIUS)
        .extrude(LID_KNUCKLE_LENGTH / 2.0, both=True)
    )
    return rear_gusset.union(center_barrel)


def make_front_handle() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.056, 0.066, 0.018)
        .translate((0.286, 0.0, 0.010))
        .edges("|Z")
        .fillet(0.007)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belgian_waffle_maker")

    black_plastic = model.material("black_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.78, 0.79, 1.0))
    dark_plate = model.material("dark_plate", rgba=(0.18, 0.18, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shell(), "base_shell"),
        material=black_plastic,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(make_base_hinge_support(), "rear_hinge"),
        material=black_plastic,
        name="rear_hinge",
    )
    base.visual(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT - 0.014)),
        material=dark_plate,
        name="lower_plate",
    )
    base.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(0.028, 0.145, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="dial_boss",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_shell(), "lid_shell"),
        material=steel,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(make_lid_hinge_bracket(), "hinge_bracket"),
        material=black_plastic,
        name="hinge_bracket",
    )
    lid.visual(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, 0.004)),
        material=dark_plate,
        name="upper_plate",
    )
    lid.visual(
        mesh_from_cadquery(make_front_handle(), "front_handle"),
        material=black_plastic,
        name="front_handle",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plate,
        name="shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.018,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.045, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "thermostat_dial",
        ),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="dial_body",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.028, 0.151, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def _aabb_center_z(aabb):
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    dial_spin = object_model.get_articulation("dial_spin")

    limits = lid_hinge.motion_limits
    open_angle = 1.1
    if limits is not None and limits.upper is not None:
        open_angle = limits.upper

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.20,
            name="closed plates stay aligned",
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.006,
            max_gap=0.018,
            name="closed plates keep a small batter gap",
        )

    rest_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({lid_hinge: open_angle}):
        open_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="front_handle",
            negative_elem="base_shell",
            min_gap=0.080,
            name="opened handle lifts above the base",
        )
    ctx.check(
        "lid opens upward",
        _aabb_center_z(rest_handle) is not None
        and _aabb_center_z(open_handle) is not None
        and _aabb_center_z(open_handle) > _aabb_center_z(rest_handle) + 0.14,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    ctx.expect_origin_gap(
        dial,
        base,
        axis="y",
        min_gap=0.145,
        name="dial sits on the housing side",
    )
    ctx.expect_contact(
        dial,
        base,
        elem_a="shaft",
        elem_b="dial_boss",
        name="dial shaft mounts against the side boss",
    )
    with ctx.pose({dial_spin: 2.2}):
        ctx.expect_contact(
            dial,
            base,
            elem_a="shaft",
            elem_b="dial_boss",
            name="dial remains seated while rotating",
        )

    return ctx.report()


object_model = build_object_model()
