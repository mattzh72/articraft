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


def _bowl_shell() -> cq.Workplane:
    """Open, hollow processor bowl with a thick rim and solid lower floor."""
    wall = cq.Workplane("XY").circle(0.165).circle(0.145).extrude(0.300).translate(
        (0.0, 0.0, 0.040)
    )
    floor = cq.Workplane("XY").circle(0.145).extrude(0.022).translate(
        (0.0, 0.0, 0.018)
    )
    rim = cq.Workplane("XY").circle(0.173).circle(0.142).extrude(0.018).translate(
        (0.0, 0.0, 0.330)
    )
    return wall.union(floor).union(rim)


def _feed_tube() -> cq.Workplane:
    """Hollow cylindrical feed tube rising from the lid."""
    return cq.Workplane("XY").circle(0.038).circle(0.027).extrude(0.105)


def _blade_swept() -> cq.Workplane:
    """Two broad, flattened cutting wings joined at the central hub."""
    left = (
        cq.Workplane("XY")
        .polyline([(0.018, -0.018), (0.126, -0.032), (0.108, 0.020), (0.020, 0.026)])
        .close()
        .extrude(0.006)
        .translate((0.0, 0.0, 0.008))
    )
    right = (
        cq.Workplane("XY")
        .polyline([(-0.018, 0.018), (-0.126, 0.032), (-0.108, -0.020), (-0.020, -0.026)])
        .close()
        .extrude(0.006)
        .translate((0.0, 0.0, 0.018))
    )
    bridge = cq.Workplane("XY").circle(0.034).extrude(0.030)
    return bridge.union(left).union(right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("charcoal_trim", rgba=(0.07, 0.08, 0.09, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("clear_smoky_bowl", rgba=(0.62, 0.82, 0.95, 0.34))
    lid_plastic = model.material("smoky_lid", rgba=(0.55, 0.72, 0.86, 0.42))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    accent = model.material("red_lock_marks", rgba=(0.80, 0.08, 0.05, 1.0))

    bowl_mesh = mesh_from_cadquery(_bowl_shell(), "hollow_processor_bowl", tolerance=0.001)
    feed_tube_mesh = mesh_from_cadquery(_feed_tube(), "hollow_feed_tube", tolerance=0.001)
    blade_mesh = mesh_from_cadquery(_blade_swept(), "processor_blade_swept", tolerance=0.0005)

    base = model.part("base")
    base.visual(
        Box((0.58, 0.38, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=warm_white,
        name="base_body",
    )
    base.visual(
        Box((0.52, 0.30, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.150)),
        material=warm_white,
        name="raised_top",
    )
    base.visual(
        Cylinder(radius=0.205, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=dark,
        name="socket_plate",
    )
    base.visual(
        Cylinder(radius=0.126, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=dark,
        name="drive_well",
    )
    base.visual(
        Box((0.30, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.199, 0.078)),
        material=dark,
        name="front_panel",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, -0.205, 0.078), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dial_collar",
    )
    for x in (-0.210, 0.210):
        for y in (-0.130, 0.130):
            base.visual(
                Box((0.080, 0.055, 0.014)),
                origin=Origin(xyz=(x, y, -0.007)),
                material=rubber,
                name=f"rubber_foot_{x}_{y}",
            )

    bowl = model.part("bowl")
    bowl.visual(bowl_mesh, origin=Origin(), material=glass, name="bowl_shell")
    bowl.visual(
        Cylinder(radius=0.184, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark,
        name="bayonet_collar",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        bowl.visual(
            Box((0.072, 0.024, 0.014)),
            origin=Origin(
                xyz=(0.187 * math.cos(angle), 0.187 * math.sin(angle), 0.029),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=dark,
            name=f"bayonet_tab_{i}",
        )
    bowl.visual(
        Box((0.028, 0.058, 0.205)),
        origin=Origin(xyz=(0.232, 0.0, 0.205)),
        material=glass,
        name="handle_grip",
    )
    bowl.visual(
        Box((0.082, 0.052, 0.030)),
        origin=Origin(xyz=(0.197, 0.0, 0.122)),
        material=glass,
        name="lower_handle_root",
    )
    bowl.visual(
        Box((0.082, 0.052, 0.030)),
        origin=Origin(xyz=(0.197, 0.0, 0.290)),
        material=glass,
        name="upper_handle_root",
    )
    bowl.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark,
        name="center_boss",
    )
    for z in (0.120, 0.170, 0.220, 0.270):
        bowl.visual(
            Box((0.010, 0.045, 0.004)),
            origin=Origin(xyz=(-0.162, -0.018, z)),
            material=dark,
            name=f"measure_mark_{int(z * 1000)}",
        )
    for side, x in (("hinge_a", -0.070), ("hinge_b", 0.070)):
        bowl.visual(
            Box((0.030, 0.042, 0.024)),
            origin=Origin(xyz=(x, 0.184, 0.329)),
            material=dark,
            name=f"{side}_base",
        )
        bowl.visual(
            Box((0.030, 0.022, 0.052)),
            origin=Origin(xyz=(x, 0.190, 0.360)),
            material=dark,
            name=f"{side}_cheek",
        )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.168, length=0.018),
        origin=Origin(xyz=(0.0, -0.165, 0.009)),
        material=lid_plastic,
        name="lid_disc",
    )
    lid.visual(
        feed_tube_mesh,
        origin=Origin(xyz=(0.060, -0.165, 0.018)),
        material=lid_plastic,
        name="feed_tube",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.086),
        origin=Origin(xyz=(0.0, 0.008, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.050, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.322, 0.014)),
        material=dark,
        name="front_latch",
    )

    blade = model.part("blade")
    blade.visual(blade_mesh, origin=Origin(), material=steel, name="blade_swept")
    blade.visual(
        Cylinder(radius=0.040, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark,
        name="blade_hub",
    )
    blade.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=dark,
        name="hub_cap",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.041, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="dial_cap",
    )
    dial.visual(
        Box((0.007, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, -0.028, 0.010)),
        material=accent,
        name="dial_pointer",
    )

    model.articulation(
        "bowl_bayonet",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.165, 0.348)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=30.0),
    )
    model.articulation(
        "dial_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.210, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-2.2, upper=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")
    bayonet = object_model.get_articulation("bowl_bayonet")
    lid_hinge = object_model.get_articulation("lid_hinge")
    blade_spin = object_model.get_articulation("blade_spin")

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="bayonet_collar",
        negative_elem="socket_plate",
        name="bowl bayonet collar seats on base socket",
    )
    ctx.expect_gap(
        lid,
        bowl,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lid_disc",
        negative_elem="bowl_shell",
        name="closed lid rests on bowl rim",
    )
    ctx.expect_within(
        blade,
        bowl,
        axes="xy",
        margin=0.010,
        inner_elem="blade_swept",
        outer_elem="bowl_shell",
        name="blade swept diameter stays inside bowl",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.25}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge raises disc clear of bowl",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.080,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_handle = ctx.part_element_world_aabb(bowl, elem="handle_grip")
    with ctx.pose({bayonet: 0.40}):
        twisted_handle = ctx.part_element_world_aabb(bowl, elem="handle_grip")
    ctx.check(
        "bayonet twist moves bowl handle around vertical axis",
        rest_handle is not None
        and twisted_handle is not None
        and abs(twisted_handle[0][1] - rest_handle[0][1]) > 0.040,
        details=f"rest={rest_handle}, twisted={twisted_handle}",
    )

    rest_blade = ctx.part_element_world_aabb(blade, elem="blade_swept")
    with ctx.pose({blade_spin: math.pi / 2.0}):
        spun_blade = ctx.part_element_world_aabb(blade, elem="blade_swept")
    ctx.check(
        "continuous blade rotation changes cutting sweep orientation",
        rest_blade is not None
        and spun_blade is not None
        and (rest_blade[1][0] - rest_blade[0][0]) > (rest_blade[1][1] - rest_blade[0][1]) * 1.5
        and (spun_blade[1][1] - spun_blade[0][1]) > (spun_blade[1][0] - spun_blade[0][0]) * 1.5,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    return ctx.report()


object_model = build_object_model()
