from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _torus(major_radius: float, tube_radius: float) -> cq.Workplane:
    """Return a torus centered on the Z axis, authored in meters."""
    return (
        cq.Workplane("XZ")
        .moveTo(major_radius, 0.0)
        .circle(tube_radius)
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )


def _trumpet_base_shell() -> cq.Workplane:
    """A flared chrome trumpet foot with a rolled outer lip."""
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.035, 0.012),
                (0.285, 0.012),
                (0.285, 0.025),
                (0.248, 0.038),
                (0.092, 0.138),
                (0.052, 0.138),
                (0.066, 0.082),
                (0.035, 0.040),
            ]
        )
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )


def _horizontal_cylinder_x(length: float, radius: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is local +X."""
    return cq.Workplane("XY").cylinder(length, radius).rotate(
        (0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0
    )


def _footrest_ring() -> cq.Workplane:
    """Continuous circular foot rail with four welded radial spokes."""
    z = 0.455
    major = 0.255
    tube = 0.017
    hub_radius = 0.062
    spoke_length = major - hub_radius
    spoke_mid = hub_radius + spoke_length / 2.0

    assembly = _torus(major, tube).translate((0.0, 0.0, z))
    hub = cq.Workplane("XY").cylinder(0.052, hub_radius).translate((0.0, 0.0, z))
    assembly = assembly.union(hub)

    for angle in (0.0, 90.0, 180.0, 270.0):
        spoke = _horizontal_cylinder_x(spoke_length, 0.012).translate(
            (spoke_mid, 0.0, z)
        )
        spoke = spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        assembly = assembly.union(spoke)

    return assembly


def _seat_cushion() -> cq.Workplane:
    """Rounded padded lounge-size round seat cushion."""
    return cq.Workplane("XY").cylinder(0.085, 0.245).edges().fillet(0.012)


def _back_pad() -> cq.Workplane:
    """Low rounded rectangular back pad."""
    return cq.Workplane("XY").box(0.430, 0.055, 0.240).edges().fillet(0.018)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool")

    chrome = Material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_leather = Material("charcoal_leather", rgba=(0.035, 0.032, 0.030, 1.0))
    seam = Material("raised_black_piping", rgba=(0.005, 0.005, 0.004, 1.0))
    black_plastic = Material("black_plastic", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = Material("matte_rubber", rgba=(0.010, 0.010, 0.009, 1.0))

    # Fixed floor assembly: rubber pad, flared trumpet foot, pedestal, and ring footrest.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.287, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="floor_pad",
    )
    base.visual(
        mesh_from_cadquery(_trumpet_base_shell(), "trumpet_base"),
        material=chrome,
        name="trumpet_base",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.510),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=chrome,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        material=chrome,
        name="lower_collar",
    )
    base.visual(
        mesh_from_cadquery(_footrest_ring(), "ring_footrest"),
        material=chrome,
        name="ring_footrest",
    )
    base.visual(
        Cylinder(radius=0.074, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.651)),
        material=chrome,
        name="top_collar",
    )

    # Rotating seat assembly.  The child frame is the swivel axis at the top of the pedestal.
    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.100, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=chrome,
        name="bearing_plate",
    )
    seat.visual(
        Cylinder(radius=0.180, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=chrome,
        name="under_pan",
    )
    seat.visual(
        mesh_from_cadquery(_seat_cushion(), "round_cushion"),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_leather,
        name="cushion",
    )
    seat.visual(
        mesh_from_cadquery(_torus(0.238, 0.006), "seat_piping"),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=seam,
        name="seat_piping",
    )
    seat.visual(
        Box((0.460, 0.045, 0.034)),
        origin=Origin(xyz=(0.0, 0.255, 0.040)),
        material=chrome,
        name="rear_yoke",
    )
    for index, x in enumerate((-0.215, 0.215)):
        seat.visual(
            Box((0.045, 0.044, 0.096)),
            origin=Origin(xyz=(x, 0.270, 0.095)),
            material=chrome,
            name=f"rear_lug_{index}",
        )
    seat.visual(
        Cylinder(radius=0.018, length=0.030),
        origin=Origin(
            xyz=(0.2525, 0.270, 0.120),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="knob_boss",
    )

    # Tilting low back: hinge barrel, two short chrome arms, and a padded pad.
    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.016, length=0.385),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    for index, x in enumerate((-0.150, 0.150)):
        backrest.visual(
            Box((0.030, 0.036, 0.138)),
            origin=Origin(xyz=(x, 0.016, 0.069)),
            material=chrome,
            name=f"back_arm_{index}",
        )
    backrest.visual(
        mesh_from_cadquery(_back_pad(), "low_back_pad"),
        origin=Origin(xyz=(0.0, 0.038, 0.198)),
        material=dark_leather,
        name="back_pad",
    )

    # Side-mounted hand knob on a short shaft, rotating about the shaft axis.
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.075),
        origin=Origin(xyz=(0.0375, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="hub_collar",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.064,
                0.038,
                body_style="lobed",
                base_diameter=0.048,
                top_diameter=0.060,
                crown_radius=0.002,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0015),
                bore=KnobBore(style="round", diameter=0.010),
            ),
            "tension_knob",
        ),
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.682)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=4.5),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.270, 0.130)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.12, upper=0.45),
        motion_properties=MotionProperties(damping=0.7, friction=0.25),
    )
    model.articulation(
        "seat_to_knob",
        ArticulationType.CONTINUOUS,
        parent=seat,
        child=knob,
        origin=Origin(xyz=(0.2675, 0.270, 0.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    knob = object_model.get_part("knob")
    swivel = object_model.get_articulation("base_to_seat")
    back_hinge = object_model.get_articulation("seat_to_backrest")
    knob_joint = object_model.get_articulation("seat_to_knob")

    ctx.check(
        "seat swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel.articulation_type}",
    )
    ctx.check(
        "side knob is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.check(
        "backrest has realistic tilt limits",
        back_hinge.motion_limits is not None
        and back_hinge.motion_limits.lower is not None
        and back_hinge.motion_limits.upper is not None
        and back_hinge.motion_limits.lower < 0.0
        and back_hinge.motion_limits.upper > 0.35,
        details=f"limits={back_hinge.motion_limits}",
    )

    ctx.expect_gap(
        seat,
        base,
        axis="z",
        positive_elem="bearing_plate",
        negative_elem="top_collar",
        min_gap=0.001,
        max_gap=0.006,
        name="swivel bearing sits just above pedestal collar",
    )
    ctx.expect_gap(
        backrest,
        seat,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="cushion",
        min_gap=0.004,
        name="rear hinge sits behind cushion",
    )
    ctx.expect_gap(
        knob,
        seat,
        axis="x",
        positive_elem="shaft",
        negative_elem="knob_boss",
        min_gap=0.0,
        max_gap=0.002,
        name="knob shaft seats on side boss",
    )

    closed_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    with ctx.pose({back_hinge: 0.45}):
        reclined_aabb = ctx.part_element_world_aabb(backrest, elem="back_pad")
    if closed_aabb is not None and reclined_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
        reclined_center_y = (reclined_aabb[0][1] + reclined_aabb[1][1]) / 2.0
        ctx.check(
            "backrest tilts rearward",
            reclined_center_y > closed_center_y + 0.040,
            details=f"closed_y={closed_center_y:.3f}, reclined_y={reclined_center_y:.3f}",
        )
    else:
        ctx.fail("backrest tilts rearward", "could not measure back pad AABBs")

    return ctx.report()


object_model = build_object_model()
