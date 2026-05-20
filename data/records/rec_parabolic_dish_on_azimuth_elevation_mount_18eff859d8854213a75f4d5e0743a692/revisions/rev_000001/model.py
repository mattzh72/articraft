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
    mesh_from_cadquery,
)


def make_dish(diameter: float, depth: float, thickness: float) -> cq.Workplane:
    r = (diameter**2) / (8 * depth) + depth / 2
    
    # Outer sphere
    outer = cq.Workplane("XY").sphere(r)
    # Inner sphere
    inner = cq.Workplane("XY").sphere(r - thickness)
    
    dish = outer.cut(inner)
    
    # Move bottom to Z=0
    dish = dish.translate((0, 0, r))
    
    # Cut top (everything above Z=depth)
    box_size = r * 3
    dish = dish.cut(
        cq.Workplane("XY")
        .box(box_size, box_size, box_size)
        .translate((0, 0, depth + box_size / 2))
    )
    
    # Rotate so it opens towards +Y instead of +Z
    dish = dish.rotate((0, 0, 0), (1, 0, 0), -90)
    
    return dish


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_dish")

    # 1. Base
    base = model.part("base")
    base.visual(
        Cylinder(0.3, 0.1),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="base_body"
    )

    # 2. Pedestal (Azimuth)
    pedestal = model.part("pedestal")
    
    # Pedestal body
    pedestal.visual(
        Cylinder(0.2, 0.5),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        name="pedestal_body"
    )
    
    # Yoke base
    pedestal.visual(
        Box((0.5, 0.2, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        name="yoke_base"
    )
    
    # Yoke left arm
    pedestal.visual(
        Box((0.05, 0.2, 0.4)),
        origin=Origin(xyz=(-0.225, 0.0, 0.75)),
        name="yoke_left"
    )
    
    # Yoke right arm
    pedestal.visual(
        Box((0.05, 0.2, 0.4)),
        origin=Origin(xyz=(0.225, 0.0, 0.75)),
        name="yoke_right"
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14)
    )

    # 3. Reflector Assembly (Elevation)
    reflector = model.part("reflector")
    
    # Rear frame (connects to yoke)
    reflector.visual(
        Box((0.40, 0.5, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rear_frame"
    )
    
    # Dish
    dish_cq = make_dish(diameter=1.2, depth=0.2, thickness=0.02)
    reflector.visual(
        mesh_from_cadquery(dish_cq, "dish"),
        # Shift forward so the dish clears the pedestal body
        origin=Origin(xyz=(0.0, 0.24, 0.0)),
        name="dish_mesh"
    )
    
    # Feed arm (extends from rear frame to feed horn)
    reflector.visual(
        Box((0.04, 0.68, 0.04)),
        origin=Origin(xyz=(0.0, 0.57, 0.0)),
        name="feed_arm"
    )
    
    # Feed horn
    reflector.visual(
        Cylinder(0.05, 0.1),
        # Rotate Z to Y
        origin=Origin(xyz=(0.0, 0.9, 0.0), rpy=(-1.5708, 0.0, 0.0)),
        name="feed_horn"
    )

    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=reflector,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    reflector = object_model.get_part("reflector")
    
    ctx.expect_contact(pedestal, base, name="pedestal rests on base")
    
    ctx.expect_within(
        reflector, pedestal,
        axes="x",
        inner_elem="rear_frame",
        outer_elem="yoke_base",
        margin=0.0,
        name="rear_frame fits between yoke arms"
    )
    
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")
    
    with ctx.pose({azimuth: 1.57, elevation: 0.785}):
        ctx.check("pose_valid", True, details="Checks that mixed pose is collision-free")
        
    with ctx.pose({elevation: 1.57}):
        ctx.check("pose_up_valid", True, details="Checks that looking straight up is collision-free")

    return ctx.report()


object_model = build_object_model()