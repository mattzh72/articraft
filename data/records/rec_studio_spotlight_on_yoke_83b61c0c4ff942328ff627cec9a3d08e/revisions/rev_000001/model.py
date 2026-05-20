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
    KnobGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _build_spot_can_shell() -> object:
    outer_radius = 0.046
    shell_thickness = 0.0025
    body_length = 0.086
    front_bezel_length = 0.016
    rear_cap_length = 0.020
    front_bezel_radius = 0.051
    rear_cap_radius = 0.041
    trunnion_radius = 0.00625
    trunnion_length = 0.012
    inner_radius = outer_radius - shell_thickness

    shell = (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .extrude(body_length * 0.5, both=True)
        .cut(
            cq.Workplane("XZ")
            .center(0.0, -0.005)
            .circle(inner_radius)
            .extrude(body_length * 0.5 - 0.004, both=True)
        )
    )

    front_bezel = (
        cq.Workplane("XZ")
        .workplane(offset=body_length * 0.5)
        .circle(front_bezel_radius)
        .extrude(front_bezel_length)
        .cut(
            cq.Workplane("XZ")
            .workplane(offset=body_length * 0.5 - 0.0002)
            .circle(inner_radius)
            .extrude(front_bezel_length + 0.001)
        )
    )

    rear_cap = (
        cq.Workplane("XZ")
        .workplane(offset=-(body_length * 0.5 + rear_cap_length))
        .circle(rear_cap_radius)
        .extrude(rear_cap_length)
    )

    right_trunnion = (
        cq.Workplane("YZ")
        .workplane(offset=0.044)
        .circle(trunnion_radius)
        .extrude(trunnion_length)
    )
    left_trunnion = (
        cq.Workplane("YZ")
        .workplane(offset=-0.044)
        .circle(trunnion_radius)
        .extrude(-trunnion_length)
    )

    return (
        shell.union(front_bezel)
        .union(rear_cap)
        .union(right_trunnion)
        .union(left_trunnion)
    )


def _material(
    model: ArticulatedObject,
    name: str,
    rgba: tuple[float, float, float, float],
) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_table_spotlight")

    base_metal = _material(model, "base_metal", (0.12, 0.12, 0.13, 1.0))
    stem_metal = _material(model, "stem_metal", (0.20, 0.21, 0.23, 1.0))
    yoke_metal = _material(model, "yoke_metal", (0.15, 0.15, 0.16, 1.0))
    can_paint = _material(model, "can_paint", (0.07, 0.07, 0.08, 1.0))
    bezel_trim = _material(model, "bezel_trim", (0.18, 0.18, 0.19, 1.0))
    lens_glass = _material(model, "lens_glass", (0.70, 0.80, 0.88, 0.35))
    knob_finish = _material(model, "knob_finish", (0.11, 0.11, 0.12, 1.0))
    threaded_steel = _material(model, "threaded_steel", (0.45, 0.46, 0.48, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.080, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=base_metal,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=base_metal,
        name="base_riser",
    )
    stand.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=stem_metal,
        name="upright_stem",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=stem_metal,
        name="pan_head",
    )
    stand.visual(
        Box((0.050, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=stem_metal,
        name="head_block",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.124, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=yoke_metal,
        name="yoke_base",
    )
    yoke.visual(
        Box((0.008, 0.018, 0.060)),
        origin=Origin(xyz=(-0.060, 0.0, 0.042)),
        material=yoke_metal,
        name="arm_0",
    )
    yoke.visual(
        Box((0.008, 0.018, 0.060)),
        origin=Origin(xyz=(0.060, 0.0, 0.042)),
        material=yoke_metal,
        name="arm_1",
    )
    yoke.visual(
        Box((0.116, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=yoke_metal,
        name="rear_bridge",
    )
    yoke.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(
            xyz=(-0.059, 0.0, 0.052),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=yoke_metal,
        name="boss_0",
    )
    yoke.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(
            xyz=(0.059, 0.0, 0.052),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=yoke_metal,
        name="boss_1",
    )
    yoke.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=yoke_metal,
        name="pan_hub",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_build_spot_can_shell(), "spotlight_can"),
        material=can_paint,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.044, length=0.003),
        origin=Origin(
            xyz=(0.0, 0.0455, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=lens_glass,
        name="lens",
    )
    can.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(
            xyz=(0.0, 0.0465, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bezel_trim,
        name="lens_ring",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(
            xyz=(0.004, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=threaded_steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=Origin(
            xyz=(0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=threaded_steel,
        name="washer",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.016,
                body_style="mushroom",
                top_diameter=0.034,
                base_diameter=0.022,
                edge_radius=0.0015,
                center=False,
            ),
            "spotlight_tightening_knob",
        ),
        origin=Origin(
            xyz=(0.010, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_finish,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.006, 0.010)),
        origin=Origin(xyz=(0.022, 0.0, 0.010)),
        material=threaded_steel,
        name="pointer",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.3,
            upper=1.3,
        ),
    )
    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "yoke_to_knob",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=knob,
        origin=Origin(xyz=(0.064, 0.0, 0.052)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    knob = object_model.get_part("knob")

    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_can")
    knob_spin = object_model.get_articulation("yoke_to_knob")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))

    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="yoke_base",
        negative_elem="head_block",
        max_gap=0.0005,
        max_penetration=0.0,
        name="yoke seats on stand head",
    )
    ctx.expect_contact(
        can,
        yoke,
        elem_a="can_shell",
        name="can is retained by the yoke",
    )
    ctx.expect_gap(
        knob,
        yoke,
        axis="x",
        positive_elem="shaft",
        negative_elem="boss_1",
        max_gap=0.0005,
        max_penetration=0.0,
        name="tightening knob mounts against the yoke arm",
    )

    rest_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="lens_ring"))
    with ctx.pose({pan: 0.9}):
        panned_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="lens_ring"))
    ctx.check(
        "pan joint swings the can sideways",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[0] > rest_lens[0] + 0.03
        and panned_lens[1] < rest_lens[1] - 0.015,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt: 0.8}):
        tilted_lens = _aabb_center(ctx.part_element_world_aabb(can, elem="lens_ring"))
    ctx.check(
        "tilt joint raises the beam direction",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.025
        and tilted_lens[1] < rest_lens[1] - 0.01,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    rest_pointer = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    ctx.check(
        "tightening knob visibly rotates about its shaft",
        rest_pointer is not None
        and turned_pointer is not None
        and turned_pointer[1] < rest_pointer[1] - 0.006
        and turned_pointer[2] < rest_pointer[2] - 0.006,
        details=f"rest_pointer={rest_pointer}, turned_pointer={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
