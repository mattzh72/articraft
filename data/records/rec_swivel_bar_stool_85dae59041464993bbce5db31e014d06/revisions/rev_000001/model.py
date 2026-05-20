from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_cylinder(radius: float, height: float, z_center: float, fillet: float = 0.0) -> cq.Workplane:
    shape = cq.Workplane("XY").cylinder(height, radius).translate((0.0, 0.0, z_center))
    if fillet > 0.0:
        shape = shape.edges().fillet(fillet)
    return shape


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    fillet: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(height, outer_radius)
    inner = cq.Workplane("XY").cylinder(height * 1.8, inner_radius)
    shape = outer.cut(inner).translate((0.0, 0.0, z_center))
    if fillet > 0.0:
        shape = shape.edges().fillet(fillet)
    return shape


def _strut(angle_deg: float, *, length: float, width: float, height: float, radius: float, z: float) -> cq.Workplane:
    """A softened rectangular radial support bar centered on the stool axis."""
    shape = (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(min(width, height) * 0.28)
        .translate((radius, 0.0, z))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    )
    return shape


def _build_base_metal() -> cq.Workplane:
    """One continuous painted-metal pedestal frame with a supported bearing race."""
    frame = _rounded_cylinder(0.238, 0.034, 0.017, 0.006)

    # Low collars make the pedestal look like a manufactured assembly while
    # still keeping all metal surfaces continuous and supported.
    frame = frame.union(_rounded_cylinder(0.070, 0.040, 0.048, 0.004))
    frame = frame.union(_rounded_cylinder(0.040, 0.500, 0.288, 0.006))
    frame = frame.union(_rounded_cylinder(0.057, 0.050, 0.533, 0.004))

    # Foot hoop and four forged-looking radial webs; all overlap the central
    # sleeve and hoop slightly so the footrest reads as one welded metal part.
    frame = frame.union(_annular_cylinder(0.292, 0.260, 0.032, 0.305, 0.005))
    for angle in (0.0, 90.0, 180.0, 270.0):
        frame = frame.union(
            _strut(angle, length=0.242, width=0.030, height=0.024, radius=0.161, z=0.305)
        )

    # Bearing pedestal: a stepped lower turntable race carried by the column.
    frame = frame.union(_rounded_cylinder(0.074, 0.036, 0.569, 0.004))
    frame = frame.union(_annular_cylinder(0.108, 0.045, 0.016, 0.586, 0.002))
    return frame


def _build_floor_gasket() -> cq.Workplane:
    # A continuous elastomer ring is bonded under the heavy metal base to keep
    # the stool planted and protect the floor.
    return _annular_cylinder(0.231, 0.195, 0.006, -0.003, 0.0015)


def _build_seat_metal() -> cq.Workplane:
    """Rotating upper bearing race and under-seat spider plate."""
    stage = _annular_cylinder(0.108, 0.046, 0.018, 0.009, 0.002)
    stage = stage.union(_rounded_cylinder(0.058, 0.035, 0.0355, 0.003))
    stage = stage.union(_rounded_cylinder(0.158, 0.020, 0.063, 0.003))
    for angle in (45.0, 135.0, 225.0, 315.0):
        stage = stage.union(
            _strut(angle, length=0.205, width=0.038, height=0.018, radius=0.072, z=0.071)
        )
    return stage


def _build_polymer_pan() -> cq.Workplane:
    """A shallow molded dish that transitions from metal swivel plate to cushion."""
    pan = _rounded_cylinder(0.222, 0.030, 0.091, 0.012)
    center_relief = cq.Workplane("XY").cylinder(0.034, 0.138).translate((0.0, 0.0, 0.103))
    pan = pan.cut(center_relief)
    pan = pan.union(_rounded_cylinder(0.160, 0.018, 0.076, 0.006))
    return pan


def _build_cushion() -> cq.Workplane:
    # Rounded circular cushion with a light crown and tight perimeter radius.
    cushion = (
        cq.Workplane("XY")
        .cylinder(0.078, 0.238)
        .edges()
        .fillet(0.022)
        .translate((0.0, 0.0, 0.145))
    )
    return cushion


def _build_cushion_seam() -> cq.Workplane:
    # Subtle raised upholstery cord just inside the seating edge.
    return _annular_cylinder(0.205, 0.198, 0.004, 0.186, 0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_swivel_bar_stool")

    painted_metal = model.material("satin_graphite_painted_metal", rgba=(0.055, 0.058, 0.060, 1.0))
    brushed_steel = model.material("brushed_steel_bearing", rgba=(0.62, 0.64, 0.63, 1.0))
    black_polymer = model.material("black_molded_polymer", rgba=(0.018, 0.019, 0.020, 1.0))
    warm_leather = model.material("warm_black_leather", rgba=(0.045, 0.036, 0.030, 1.0))
    seam_leather = model.material("subtle_leather_seam", rgba=(0.025, 0.020, 0.017, 1.0))
    elastomer = model.material("soft_floor_elastomer", rgba=(0.012, 0.012, 0.011, 1.0))

    base = model.part("pedestal")
    base.visual(
        mesh_from_cadquery(_build_base_metal(), "pedestal_metal", tolerance=0.0008, angular_tolerance=0.08),
        material=painted_metal,
        name="painted_frame",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.107, 0.047, 0.010, 0.599, 0.0015), "lower_bearing_race"),
        material=brushed_steel,
        name="lower_race",
    )

    floor_gasket = model.part("floor_gasket")
    floor_gasket.visual(
        mesh_from_cadquery(_build_floor_gasket(), "floor_gasket", tolerance=0.0008, angular_tolerance=0.08),
        material=elastomer,
        name="rubber_ring",
    )

    model.articulation(
        "pedestal_to_floor_gasket",
        ArticulationType.FIXED,
        parent=base,
        child=floor_gasket,
        origin=Origin(),
    )

    seat = model.part("seat_stage")
    seat.visual(
        mesh_from_cadquery(_build_seat_metal(), "seat_swivel_metal", tolerance=0.0008, angular_tolerance=0.08),
        material=painted_metal,
        name="upper_race",
    )
    seat.visual(
        mesh_from_cadquery(_build_polymer_pan(), "molded_seat_pan", tolerance=0.0008, angular_tolerance=0.08),
        material=black_polymer,
        name="polymer_pan",
    )
    seat.visual(
        mesh_from_cadquery(_build_cushion(), "rounded_cushion", tolerance=0.0008, angular_tolerance=0.08),
        material=warm_leather,
        name="cushion",
    )
    seat.visual(
        mesh_from_cadquery(_build_cushion_seam(), "cushion_seam", tolerance=0.0008, angular_tolerance=0.08),
        material=seam_leather,
        name="seam_cord",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        # The child frame lies at the interface between the fixed lower race and
        # rotating upper race, so the cushion swivels about the supported column.
        origin=Origin(xyz=(0.0, 0.0, 0.604)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat_stage")
    swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper race sits on fixed lower race",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="upper_race",
        elem_b="lower_race",
        min_overlap=0.080,
        name="bearing races are concentrically supported",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="upper_race",
            negative_elem="lower_race",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel keeps bearing stack seated at quarter turn",
        )

    return ctx.report()


object_model = build_object_model()
