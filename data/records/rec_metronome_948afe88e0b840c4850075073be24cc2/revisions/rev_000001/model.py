from __future__ import annotations

import math

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
    mesh_from_cadquery,
)


def _frustum_shell() -> cq.Workplane:
    """Tall hollow Maelzel case: tapered walnut body with an open front slot."""

    body_z0 = 0.035
    body_h = 0.315
    bottom_w, bottom_d = 0.122, 0.092
    top_w, top_d = 0.038, 0.034
    wall = 0.006

    outer = (
        cq.Workplane("XY")
        .workplane(offset=body_z0)
        .rect(bottom_w, bottom_d)
        .workplane(offset=body_h)
        .rect(top_w, top_d)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=body_z0 - 0.006)
        .rect(bottom_w - 2.0 * wall, bottom_d - 2.0 * wall)
        .workplane(offset=body_h + 0.020)
        .rect(max(top_w - 2.0 * wall, 0.012), max(top_d - 2.0 * wall, 0.012))
        .loft(combine=True)
    )

    # Front aperture and apex slot.  It cuts only the front wall, leaving a
    # visibly hollow, sloped rear and side shell around the moving pendulum.
    slot = cq.Workplane("XY").box(0.042, 0.080, 0.282).translate((0.0, -0.050, 0.201))
    apex_slot = cq.Workplane("XY").box(0.024, 0.070, 0.070).translate((0.0, -0.040, 0.315))
    shell = outer.cut(inner).cut(slot).cut(apex_slot)
    return shell


def _cone_weight_mesh() -> cq.Workplane:
    """Conical sliding weight with a true through-bore for the pendulum rod."""

    height = 0.070
    base_r = 0.026
    top_r = 0.012
    bore_r = 0.0065

    cone = (
        cq.Workplane("XY")
        .circle(base_r)
        .workplane(offset=height)
        .circle(top_r)
        .loft(combine=True)
    )
    bore = cq.Workplane("XY").circle(bore_r).extrude(height + 0.020).translate((0.0, 0.0, -0.010))
    return cone.cut(bore).translate((0.0, 0.0, -height / 2.0))


def _pivot_bearing_mesh() -> cq.Workplane:
    """Small annular bearing plate at the internal pendulum shaft."""

    outer_r = 0.030
    inner_r = 0.018
    thickness = 0.006
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _annular_collar_mesh() -> cq.Workplane:
    """Short brass collar with a clear center bore around the sliding rod."""

    return (
        cq.Workplane("XY")
        .circle(0.008)
        .circle(0.0035)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.004))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maelzel_pyramid_metronome")

    walnut = model.material("dark_walnut", rgba=(0.20, 0.095, 0.035, 1.0))
    edge_wood = model.material("polished_edges", rgba=(0.30, 0.15, 0.060, 1.0))
    brass = model.material("aged_brass", rgba=(0.82, 0.58, 0.22, 1.0))
    steel = model.material("blued_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    black = model.material("blackened_weight", rgba=(0.015, 0.014, 0.012, 1.0))
    ivory = model.material("ivory_scale", rgba=(0.88, 0.82, 0.62, 1.0))
    ink = model.material("engraved_ticks", rgba=(0.02, 0.018, 0.014, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.160, 0.112, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=edge_wood,
        name="base_plinth",
    )
    housing.visual(
        Box((0.138, 0.096, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=walnut,
        name="stepped_base",
    )
    housing.visual(
        mesh_from_cadquery(_frustum_shell(), "tapered_housing"),
        material=walnut,
        name="tapered_housing",
    )
    # Small feet, kept embedded in the plinth so the root part remains a single
    # supported assembly.
    for i, x in enumerate((-0.060, 0.060)):
        for j, y in enumerate((-0.040, 0.040)):
            housing.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x, y, 0.003)),
                material=black,
                name=f"rubber_foot_{i}_{j}",
            )

    # A narrow tempo scale strip on the front rail, with individual tick bars.
    housing.visual(
        Box((0.013, 0.003, 0.256)),
        origin=Origin(xyz=(0.033, -0.045, 0.169)),
        material=ivory,
        name="tempo_scale",
    )
    for idx, z in enumerate((0.105, 0.128, 0.151, 0.174, 0.197, 0.220, 0.243, 0.266, 0.289)):
        tick_w = 0.010 if idx % 2 == 0 else 0.007
        housing.visual(
            Box((tick_w, 0.0015, 0.0022)),
            origin=Origin(xyz=(0.033, -0.0465, z)),
            material=ink,
            name=f"scale_tick_{idx}",
        )

    pivot_z = 0.162
    pivot_y = -0.052
    housing.visual(
        mesh_from_cadquery(_pivot_bearing_mesh(), "pivot_bearing"),
        origin=Origin(xyz=(0.0, pivot_y + 0.013, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_bearing",
    )
    housing.visual(
        Box((0.074, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, pivot_y + 0.013, pivot_z)),
        material=brass,
        name="pivot_bridge",
    )
    housing.visual(
        Cylinder(radius=0.0032, length=0.024),
        origin=Origin(xyz=(0.0, pivot_y, pivot_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_pin",
    )
    housing.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(0.0, 0.052, 0.092), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_bushing",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0038, length=0.207),
        origin=Origin(xyz=(0.0, 0.0, 0.1095)),
        material=steel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0038, length=0.109),
        origin=Origin(xyz=(0.0, 0.0, -0.0605)),
        material=steel,
        name="lower_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=brass,
        name="counterweight",
    )

    cone_weight = model.part("cone_weight")
    cone_weight.visual(
        mesh_from_cadquery(_cone_weight_mesh(), "cone_weight_shell"),
        material=black,
        name="cone_weight_shell",
    )
    cone_weight.visual(
        mesh_from_cadquery(_annular_collar_mesh(), "lower_collar"),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=brass,
        name="lower_collar",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.083, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.054, 0.0)),
        material=brass,
        name="wing_plate",
    )
    for i, x in enumerate((-0.0415, 0.0415)):
        winding_key.visual(
            Cylinder(radius=0.014, length=0.011),
            origin=Origin(xyz=(x, 0.054, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"rounded_wing_{i}",
        )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "pendulum_to_cone_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=cone_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=0.0, upper=0.065),
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.065, 0.092)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    cone_weight = object_model.get_part("cone_weight")
    winding_key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("housing_to_pendulum")
    slide = object_model.get_articulation("pendulum_to_cone_weight")
    key_turn = object_model.get_articulation("housing_to_winding_key")

    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        reason="The fixed brass pivot pin is intentionally captured through the pendulum hub at the housed shaft.",
    )
    ctx.allow_overlap(
        pendulum,
        cone_weight,
        elem_a="pendulum_rod",
        elem_b="lower_collar",
        reason="The sliding weight's lower friction collar lightly grips the rod while still moving along it.",
    )

    ctx.check(
        "requested articulations present",
        swing.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and key_turn.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={swing.articulation_type}, {slide.articulation_type}, {key_turn.articulation_type}",
    )
    ctx.expect_overlap(
        housing,
        pendulum,
        axes="xyz",
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        min_overlap=0.006,
        name="pivot pin is captured in pendulum hub",
    )

    ctx.expect_overlap(
        pendulum,
        cone_weight,
        axes="z",
        elem_a="pendulum_rod",
        elem_b="cone_weight_shell",
        min_overlap=0.050,
        name="cone weight surrounds upper rod",
    )
    ctx.expect_overlap(
        pendulum,
        cone_weight,
        axes="xyz",
        elem_a="pendulum_rod",
        elem_b="lower_collar",
        min_overlap=0.004,
        name="friction collar is retained on rod",
    )
    ctx.expect_gap(
        cone_weight,
        pendulum,
        axis="z",
        positive_elem="cone_weight_shell",
        negative_elem="pivot_hub",
        min_gap=0.020,
        name="cone weight is above pivot hub",
    )
    ctx.expect_gap(
        housing,
        pendulum,
        axis="y",
        positive_elem="tapered_housing",
        negative_elem="pendulum_rod",
        max_penetration=0.0,
        name="pendulum rod sits in front slot without entering shell",
    )

    rest_cone = ctx.part_world_position(cone_weight)
    with ctx.pose({slide: 0.055}):
        raised_cone = ctx.part_world_position(cone_weight)
        ctx.expect_overlap(
            pendulum,
            cone_weight,
            axes="z",
            elem_a="pendulum_rod",
            elem_b="cone_weight_shell",
            min_overlap=0.050,
            name="raised cone remains on rod",
        )
    ctx.check(
        "cone weight slides upward along rod",
        rest_cone is not None and raised_cone is not None and raised_cone[2] > rest_cone[2] + 0.050,
        details=f"rest={rest_cone}, raised={raised_cone}",
    )

    with ctx.pose({swing: 0.25}):
        rod_box = ctx.part_element_world_aabb(pendulum, elem="pendulum_rod")
    ctx.check(
        "pendulum swings through front slot",
        rod_box is not None and rod_box[1][0] > 0.045,
        details=f"rod_box={rod_box}",
    )

    rest_wing = ctx.part_element_world_aabb(winding_key, elem="wing_plate")
    with ctx.pose({key_turn: math.pi / 2.0}):
        turned_wing = ctx.part_element_world_aabb(winding_key, elem="wing_plate")
    if rest_wing is not None and turned_wing is not None:
        rest_x = rest_wing[1][0] - rest_wing[0][0]
        turned_z = turned_wing[1][2] - turned_wing[0][2]
        key_ok = turned_z > rest_x * 0.80
    else:
        key_ok = False
    ctx.check(
        "rear winding key turns continuously",
        key_ok,
        details=f"rest_wing={rest_wing}, turned_wing={turned_wing}",
    )

    return ctx.report()


object_model = build_object_model()
