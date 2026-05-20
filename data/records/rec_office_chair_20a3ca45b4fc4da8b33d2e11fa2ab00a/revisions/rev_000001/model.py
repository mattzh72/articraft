import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_base_geom():
    # Hub: Z from 0.08 to 0.14 -> center Z=0.11, H=0.06
    hub = cq.Workplane("XY").cylinder(0.06, 0.05).translate((0, 0, 0.11))
    
    # Pedestal: Z from 0.14 to 0.42 -> center Z=0.28, H=0.28
    pedestal = cq.Workplane("XY").cylinder(0.28, 0.025).translate((0, 0, 0.28))
    
    base_geom = hub.union(pedestal)
    
    for i in range(5):
        angle = i * 360.0 / 5
        # Leg
        leg = cq.Workplane("XY").box(0.28, 0.03, 0.04).translate((0.18, 0, 0.10)).rotate((0,0,0), (0,0,1), angle)
        base_geom = base_geom.union(leg)
        
        # Boss with hole for caster pin
        boss = cq.Workplane("XY").cylinder(0.05, 0.015).translate((0.30, 0, 0.085)).rotate((0,0,0), (0,0,1), angle)
        base_geom = base_geom.union(boss)
        
        hole = cq.Workplane("XY").cylinder(0.055, 0.006).translate((0.30, 0, 0.085)).rotate((0,0,0), (0,0,1), angle)
        base_geom = base_geom.cut(hole)
        
    # Tilt mechanism bracket at the top of the pedestal
    bracket = cq.Workplane("XY").box(0.06, 0.118, 0.06).translate((0, 0, 0.45))
    pin_hole = cq.Workplane("XY").cylinder(0.12, 0.006).rotate((0,0,0), (1,0,0), 90).translate((0, 0, 0.465))
    bracket = bracket.cut(pin_hole)
    base_geom = base_geom.union(bracket)
        
    return base_geom


def build_seat_brackets_geom():
    # Top plate
    top_plate = cq.Workplane("XY").box(0.25, 0.14, 0.02).translate((-0.025, 0, 0.035))
    # Flanges
    flange1 = cq.Workplane("XY").box(0.08, 0.01, 0.055).translate((0, 0.066, 0.0075))
    flange2 = cq.Workplane("XY").box(0.08, 0.01, 0.055).translate((0, -0.066, 0.0075))
    # Rear bracket
    rear_bracket = cq.Workplane("XY").box(0.1, 0.1, 0.035).translate((-0.20, 0, 0.0425))
    # Tilt pin
    pin = cq.Workplane("XY").cylinder(0.16, 0.005).rotate((0,0,0), (1,0,0), 90)
    # Armrest supports
    supp1 = cq.Workplane("XY").box(0.04, 0.16, 0.02).translate((0, 0.15, 0.035))
    supp2 = cq.Workplane("XY").box(0.04, 0.16, 0.02).translate((0, -0.15, 0.035))
    
    return top_plate.union(flange1).union(flange2).union(rear_bracket).union(pin).union(supp1).union(supp2)


def build_seat_cushion_geom():
    return cq.Workplane("XY").box(0.4, 0.5, 0.05).edges("|Z").fillet(0.05)


def build_back_rest_geom():
    frame = cq.Workplane("XY").box(0.05, 0.44, 0.60).edges("|X").fillet(0.04)
    cutout = cq.Workplane("XY").box(0.06, 0.36, 0.52)
    frame = frame.cut(cutout)
    fabric = cq.Workplane("XY").box(0.005, 0.36, 0.52)
    return frame.union(fabric)


def build_armrest_geom():
    pad = cq.Workplane("XY").box(0.25, 0.06, 0.03).edges("|Z").fillet(0.02)
    stem = cq.Workplane("XY").box(0.04, 0.02, 0.23).translate((-0.025, 0, -0.115))
    return pad.union(stem)


def build_caster_swivel_geom():
    pin = cq.Workplane("XY").cylinder(0.03, 0.005).translate((0, 0, 0.075))
    arm = cq.Workplane("XY").box(0.035, 0.03, 0.01).translate((-0.015, 0, 0.055))
    fork1 = cq.Workplane("XY").box(0.015, 0.005, 0.035).translate((-0.025, 0.015, 0.0375))
    fork2 = cq.Workplane("XY").box(0.015, 0.005, 0.035).translate((-0.025, -0.015, 0.0375))
    axle = cq.Workplane("XY").cylinder(0.035, 0.005).rotate((0,0,0), (1,0,0), 90).translate((-0.025, 0, 0.025))
    return pin.union(arm).union(fork1).union(fork2).union(axle)


def build_wheel_geom():
    wheel = cq.Workplane("XY").cylinder(0.02, 0.02).rotate((0,0,0), (1,0,0), 90)
    hole = cq.Workplane("XY").cylinder(0.022, 0.006).rotate((0,0,0), (1,0,0), 90)
    return wheel.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_chair")

    base = model.part("base")
    base.visual(mesh_from_cadquery(build_base_geom(), "base_geom"))

    seat = model.part("seat")
    seat.visual(mesh_from_cadquery(build_seat_brackets_geom(), "seat_brackets"))
    seat.visual(mesh_from_cadquery(build_seat_cushion_geom(), "seat_cushion"), origin=Origin(xyz=(0.05, 0, 0.07)))
    seat.visual(mesh_from_cadquery(build_back_rest_geom(), "back_rest"), origin=Origin(xyz=(-0.255, 0, 0.36)))
    
    armrest_geom = build_armrest_geom()
    seat.visual(mesh_from_cadquery(armrest_geom, "armrest_left"), origin=Origin(xyz=(0.025, 0.23, 0.275)))
    seat.visual(mesh_from_cadquery(armrest_geom, "armrest_right"), origin=Origin(xyz=(0.025, -0.23, 0.275)))

    model.articulation(
        "base_to_seat",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0, 0, 0.465)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.26),
    )

    swivel_geom = build_caster_swivel_geom()
    wheel_geom = build_wheel_geom()

    for i in range(5):
        angle = i * 2 * math.pi / 5
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        
        swivel = model.part(f"caster_swivel_{i}")
        swivel.visual(mesh_from_cadquery(swivel_geom, f"swivel_geom_{i}"))
        
        model.articulation(
            f"base_to_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=swivel,
            origin=Origin(xyz=(0.30 * cos_a, 0.30 * sin_a, 0.0), rpy=(0, 0, angle)),
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=5.0, velocity=5.0),
        )
        
        wheel = model.part(f"wheel_{i}")
        wheel.visual(mesh_from_cadquery(wheel_geom, f"wheel_geom_{i}"))
        
        model.articulation(
            f"swivel_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(-0.025, 0, 0.025)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=5.0, velocity=5.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    tilt_joint = object_model.get_articulation("base_to_seat")
    
    ctx.allow_isolated_part(seat, reason="Seat is mounted via the tilt pin which has intentional clearance inside the base bracket.")
    
    for i in range(5):
        swivel = object_model.get_part(f"caster_swivel_{i}")
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(base, swivel, reason="Caster pins intentionally fit into base bosses.")
        ctx.allow_isolated_part(wheel, reason="Wheels are mounted on axles with intentional clearance.")
    
    with ctx.pose({tilt_joint: 0.26}):
        pass
        
    return ctx.report()


object_model = build_object_model()
