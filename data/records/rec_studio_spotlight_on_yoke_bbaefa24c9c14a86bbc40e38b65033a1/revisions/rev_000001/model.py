from __future__ import annotations

from math import pi

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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.10, 0.105, 0.105, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    lens_glass = model.material("warm_lens_glass", rgba=(1.0, 0.78, 0.34, 0.55))

    base_thickness = 0.070
    sleeve_height = 0.780
    sleeve_outer = 0.100
    sleeve_clear = 0.066
    sleeve_top_z = base_thickness + sleeve_height

    base = model.part("base")
    base.visual(
        Box((0.620, 0.420, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=matte_black,
        name="weighted_base",
    )
    # Low rubber pads are bonded to the underside of the plate and make the
    # rectangular floor base read at full studio-light scale.
    for i, (x, y) in enumerate(
        ((-0.245, -0.155), (-0.245, 0.155), (0.245, -0.155), (0.245, 0.155))
    ):
        base.visual(
            Box((0.090, 0.055, 0.014)),
            origin=Origin(xyz=(x, y, 0.007)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    sleeve_wall = (sleeve_outer - sleeve_clear) / 2.0
    sleeve_z = base_thickness + sleeve_height / 2.0
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height + 0.004)),
        origin=Origin(xyz=(0.0, sleeve_clear / 2.0 + sleeve_wall / 2.0, sleeve_z)),
        material=dark_metal,
        name="sleeve_wall_front",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height + 0.004)),
        origin=Origin(xyz=(0.0, -sleeve_clear / 2.0 - sleeve_wall / 2.0, sleeve_z)),
        material=dark_metal,
        name="sleeve_wall_rear",
    )
    base.visual(
        Box((sleeve_wall, sleeve_clear, sleeve_height + 0.004)),
        origin=Origin(xyz=(-sleeve_clear / 2.0 - sleeve_wall / 2.0, 0.0, sleeve_z)),
        material=dark_metal,
        name="sleeve_wall_side_0",
    )
    base.visual(
        Box((sleeve_wall, sleeve_clear, sleeve_height + 0.004)),
        origin=Origin(xyz=(sleeve_clear / 2.0 + sleeve_wall / 2.0, 0.0, sleeve_z)),
        material=dark_metal,
        name="sleeve_wall_side_1",
    )

    collar_outer = 0.145
    collar_wall = (collar_outer - sleeve_clear) / 2.0
    collar_z = sleeve_top_z + 0.023
    for name, size, xyz in (
        ("collar_front", (collar_outer, collar_wall, 0.082), (0.0, sleeve_clear / 2.0 + collar_wall / 2.0, collar_z)),
        ("collar_rear", (collar_outer, collar_wall, 0.082), (0.0, -sleeve_clear / 2.0 - collar_wall / 2.0, collar_z)),
        ("collar_side_0", (collar_wall, sleeve_clear, 0.082), (-sleeve_clear / 2.0 - collar_wall / 2.0, 0.0, collar_z)),
        ("collar_side_1", (collar_wall, sleeve_clear, 0.082), (sleeve_clear / 2.0 + collar_wall / 2.0, 0.0, collar_z)),
    ):
        base.visual(Box(size), origin=Origin(xyz=xyz), material=dark_metal, name=name)
    for i, (x, y) in enumerate(((-0.125, -0.085), (-0.125, 0.085), (0.125, -0.085), (0.125, 0.085))):
        base.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, base_thickness + 0.005)),
            material=brushed_steel,
            name=f"base_bolt_{i}",
        )

    stand_column = model.part("stand_column")
    stand_column.visual(
        Box((0.052, 0.052, 1.370)),
        # The child frame sits at the sleeve mouth.  The column extends down
        # into the lower sleeve so it remains captured at full extension.
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=brushed_steel,
        name="inner_post",
    )
    # Low-friction guide shoes make physical sliding contact with the inside of
    # the lower sleeve so the telescoping member reads as supported, not
    # hovering inside a clearance envelope.
    stand_column.visual(
        Box((0.052, 0.007, 0.090)),
        origin=Origin(xyz=(0.0, 0.0295, -0.345)),
        material=dark_metal,
        name="front_guide_shoe",
    )
    stand_column.visual(
        Box((0.052, 0.007, 0.090)),
        origin=Origin(xyz=(0.0, -0.0295, -0.345)),
        material=dark_metal,
        name="rear_guide_shoe",
    )
    stand_column.visual(
        Cylinder(radius=0.074, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.938)),
        material=dark_metal,
        name="pan_bearing_cap",
    )
    stand_column.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=brushed_steel,
        name="pan_spigot",
    )

    yoke = model.part("yoke")
    yoke_body = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.520, 0.110, 0.380),
            span_width=0.380,
            trunnion_diameter=0.058,
            trunnion_center_z=0.260,
            base_thickness=0.050,
            corner_radius=0.015,
            center=False,
        ),
        "trunnion_yoke",
    )
    yoke.visual(yoke_body, material=matte_black, name="yoke_body")
    yoke.visual(
        Cylinder(radius=0.072, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_metal,
        name="pan_turntable",
    )

    lamp = model.part("lamp_can")
    can_length = 0.420
    can_radius = 0.160
    rear_overhang = 0.075
    can_shell_cq = (
        cq.Workplane("XY")
        .circle(can_radius)
        .extrude(can_length)
        .faces(">Z")
        .shell(-0.012)
        .translate((0.0, 0.0, -rear_overhang))
    )
    lamp.visual(
        mesh_from_cadquery(can_shell_cq, "lamp_can_shell", tolerance=0.0015),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="lamp_can",
    )
    front_ring_cq = (
        cq.Workplane("XY")
        .circle(can_radius + 0.018)
        .circle(can_radius - 0.020)
        .extrude(0.030)
        .translate((0.0, 0.0, can_length - rear_overhang - 0.018))
    )
    lamp.visual(
        mesh_from_cadquery(front_ring_cq, "front_bezel", tolerance=0.0015),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=can_radius - 0.015, length=0.010),
        origin=Origin(xyz=(0.0, can_length - rear_overhang - 0.022, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=can_radius * 0.72, length=0.018),
        origin=Origin(xyz=(0.0, -rear_overhang - 0.009, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    # Trunnion pins project from the lamp shell into the yoke cheek bores.
    for side, x in enumerate((-0.195, 0.195)):
        lamp.visual(
            Cylinder(radius=0.022, length=0.130),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"trunnion_pin_{side}",
        )
    for side, x in enumerate((-0.269, 0.269)):
        lamp.visual(
            Cylinder(radius=0.043, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"trunnion_washer_{side}",
        )
    # Small raised rear vent ribs avoid a featureless back cap without adding a
    # separate floating grille part.
    for i, z in enumerate((-0.060, 0.0, 0.060)):
        lamp.visual(
            Box((0.155, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, -rear_overhang - 0.022, z)),
            material=matte_black,
            name=f"rear_vent_rib_{i}",
        )

    model.articulation(
        "sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stand_column,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.320),
    )
    model.articulation(
        "column_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=stand_column,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.950)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("stand_column")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp_can")
    slide = object_model.get_articulation("sleeve_to_column")
    pan = object_model.get_articulation("column_to_yoke")
    tilt = object_model.get_articulation("yoke_to_lamp")

    ctx.expect_gap(
        base,
        column,
        axis="y",
        positive_elem="sleeve_wall_front",
        negative_elem="inner_post",
        min_gap=0.004,
        max_gap=0.010,
        name="front sleeve clearance around post",
    )
    ctx.expect_gap(
        column,
        base,
        axis="y",
        positive_elem="inner_post",
        negative_elem="sleeve_wall_rear",
        min_gap=0.004,
        max_gap=0.010,
        name="rear sleeve clearance around post",
    )
    ctx.expect_gap(
        base,
        column,
        axis="x",
        positive_elem="sleeve_wall_side_1",
        negative_elem="inner_post",
        min_gap=0.004,
        max_gap=0.010,
        name="side sleeve clearance around post",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="sleeve_wall_front",
        min_overlap=0.30,
        name="collapsed column remains deeply inserted",
    )

    with ctx.pose({slide: 0.320}):
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_wall_front",
            min_overlap=0.090,
            name="extended column retains sleeve insertion",
        )

    front_rest_aabb = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({tilt: 0.65}):
        front_tilt_aabb = ctx.part_element_world_aabb(lamp, elem="front_lens")
    if front_rest_aabb is not None and front_tilt_aabb is not None:
        rest_center_z = (front_rest_aabb[0][2] + front_rest_aabb[1][2]) / 2.0
        tilt_center_z = (front_tilt_aabb[0][2] + front_tilt_aabb[1][2]) / 2.0
    else:
        rest_center_z = tilt_center_z = None
    ctx.check(
        "positive lamp tilt raises beam",
        rest_center_z is not None and tilt_center_z is not None and tilt_center_z > rest_center_z + 0.08,
        details=f"rest_z={rest_center_z}, tilted_z={tilt_center_z}",
    )

    front_pan0 = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan: pi / 2.0}):
        front_pan90 = ctx.part_element_world_aabb(lamp, elem="front_lens")
    if front_pan0 is not None and front_pan90 is not None:
        pan0_center = tuple((front_pan0[0][i] + front_pan0[1][i]) / 2.0 for i in range(3))
        pan90_center = tuple((front_pan90[0][i] + front_pan90[1][i]) / 2.0 for i in range(3))
    else:
        pan0_center = pan90_center = None
    ctx.check(
        "vertical pan swings beam sideways",
        pan0_center is not None
        and pan90_center is not None
        and pan0_center[1] > 0.20
        and pan90_center[0] < -0.20,
        details=f"pan0={pan0_center}, pan90={pan90_center}",
    )

    return ctx.report()


object_model = build_object_model()
