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


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _truck_body() -> cq.Workplane:
    """One connected carriage, saddle, and bored bearing housing."""

    carriage = _cq_box((0.240, 0.170, 0.045), (0.0, 0.0, 0.0225))
    pedestal = _cq_box((0.100, 0.085, 0.050), (0.0, 0.0, 0.070))
    front_rib = _cq_box((0.030, 0.130, 0.070), (0.040, 0.0, 0.080))
    rear_rib = _cq_box((0.030, 0.130, 0.070), (-0.040, 0.0, 0.080))

    # Annular bearing block: extruded on the YZ plane so the bore and rotary
    # axis run along local +X.  The bore is intentionally slightly larger than
    # the spindle body so the cartridge reads as captured in a real bearing.
    bearing_ring = (
        cq.Workplane("YZ")
        .circle(0.070)
        .circle(0.048)
        .extrude(0.078)
        .translate((-0.039, 0.0, 0.160))
    )

    # Small clamp ears on top make the truck look like a serviceable spindle
    # saddle while staying outside the bearing bore.
    upper_clamp = _cq_box((0.080, 0.082, 0.020), (0.0, 0.0, 0.234))
    lower_neck = _cq_box((0.082, 0.070, 0.015), (0.0, 0.0, 0.096))

    return (
        carriage.union(pedestal)
        .union(front_rib)
        .union(rear_rib)
        .union(bearing_ring)
        .union(upper_clamp)
        .union(lower_neck)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guideway_tool_slide")

    cast_iron = model.material("oiled_cast_iron", color=(0.12, 0.15, 0.17, 1.0))
    ground_steel = model.material("ground_steel", color=(0.72, 0.74, 0.72, 1.0))
    carriage_paint = model.material("carriage_blue_gray", color=(0.22, 0.31, 0.36, 1.0))
    spindle_steel = model.material("brushed_spindle_steel", color=(0.62, 0.64, 0.63, 1.0))
    bronze = model.material("bearing_bronze", color=(0.72, 0.48, 0.18, 1.0))
    dark_tooling = model.material("dark_tooling_face", color=(0.05, 0.055, 0.06, 1.0))
    fastener_black = model.material("black_oxide_fasteners", color=(0.015, 0.015, 0.014, 1.0))

    guideway = model.part("guideway")
    guideway.visual(
        Box((0.860, 0.260, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="bed",
    )
    guideway.visual(
        Box((0.780, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, -0.065, 0.0725)),
        material=ground_steel,
        name="rail_0",
    )
    guideway.visual(
        Box((0.780, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.065, 0.0725)),
        material=ground_steel,
        name="rail_1",
    )
    guideway.visual(
        Box((0.780, 0.026, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=cast_iron,
        name="center_web",
    )
    for idx, x in enumerate((-0.420, 0.420)):
        guideway.visual(
            Box((0.030, 0.225, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=cast_iron,
            name=f"travel_stop_{idx}",
        )
    for idx, x in enumerate((-0.300, -0.100, 0.100, 0.300)):
        guideway.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.105, 0.063), rpy=(0.0, 0.0, 0.0)),
            material=fastener_black,
            name=f"way_screw_{idx}",
        )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_truck_body(), "truck_body", tolerance=0.0008, angular_tolerance=0.08),
        material=carriage_paint,
        name="truck_body",
    )
    truck.visual(
        Box((0.070, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=bronze,
        name="bearing_pad_top",
    )
    truck.visual(
        Box((0.070, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=bronze,
        name="bearing_pad_bottom",
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        Cylinder(radius=0.041, length=0.230),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="main_spindle",
    )
    cartridge.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(-0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="rear_collar",
    )
    cartridge.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="faceplate",
    )
    cartridge.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.151, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="tooling_nose",
    )
    cartridge.visual(
        Box((0.008, 0.055, 0.055)),
        origin=Origin(xyz=(0.173, 0.0, 0.0)),
        material=dark_tooling,
        name="tooling_face",
    )
    for idx, (y, z) in enumerate(((-0.030, -0.030), (-0.030, 0.030), (0.030, -0.030), (0.030, 0.030))):
        cartridge.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(xyz=(0.137, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener_black,
            name=f"face_bolt_{idx}",
        )

    model.articulation(
        "guideway_to_truck",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=truck,
        origin=Origin(xyz=(-0.220, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.20, lower=0.0, upper=0.440),
    )
    model.articulation(
        "truck_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=truck,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guideway = object_model.get_part("guideway")
    truck = object_model.get_part("truck")
    cartridge = object_model.get_part("cartridge")
    slide = object_model.get_articulation("guideway_to_truck")
    spindle = object_model.get_articulation("truck_to_cartridge")

    ctx.expect_gap(
        truck,
        guideway,
        axis="z",
        positive_elem="truck_body",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="truck rides on hardened guide rail",
    )
    ctx.expect_overlap(
        truck,
        guideway,
        axes="xy",
        elem_a="truck_body",
        elem_b="rail_0",
        min_overlap=0.025,
        name="truck remains seated over rail footprint",
    )
    ctx.expect_within(
        cartridge,
        truck,
        axes="yz",
        inner_elem="main_spindle",
        outer_elem="truck_body",
        margin=0.002,
        name="spindle axis is captured by the truck bearing housing",
    )
    ctx.expect_overlap(
        cartridge,
        truck,
        axes="x",
        elem_a="main_spindle",
        elem_b="truck_body",
        min_overlap=0.060,
        name="spindle body passes through bearing span",
    )
    ctx.expect_gap(
        cartridge,
        truck,
        axis="x",
        positive_elem="tooling_face",
        negative_elem="truck_body",
        min_gap=0.040,
        name="tooling face projects in front of the bearing",
    )
    ctx.expect_contact(
        cartridge,
        truck,
        elem_a="main_spindle",
        elem_b="bearing_pad_top",
        contact_tol=0.0015,
        name="spindle is carried by bearing pads",
    )

    rest_pos = ctx.part_world_position(truck)
    with ctx.pose({slide: 0.440}):
        extended_pos = ctx.part_world_position(truck)
        ctx.expect_overlap(
            truck,
            guideway,
            axes="xy",
            elem_a="truck_body",
            elem_b="rail_0",
            min_overlap=0.025,
            name="extended truck remains on the guide rail",
        )
    ctx.check(
        "prismatic truck translates along the guideway",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    face_rest = ctx.part_element_world_aabb(cartridge, elem="tooling_face")
    with ctx.pose({spindle: math.pi / 4.0}):
        face_rotated = ctx.part_element_world_aabb(cartridge, elem="tooling_face")

    def _span_yz(aabb: tuple[object, object] | None) -> tuple[float, float] | None:
        if aabb is None:
            return None
        return (aabb[1][1] - aabb[0][1], aabb[1][2] - aabb[0][2])

    rest_span = _span_yz(face_rest)
    rotated_span = _span_yz(face_rotated)
    ctx.check(
        "tooling face visibly rotates about spindle axis",
        rest_span is not None
        and rotated_span is not None
        and rotated_span[0] > rest_span[0] + 0.010
        and rotated_span[1] > rest_span[1] + 0.010,
        details=f"rest_span={rest_span}, rotated_span={rotated_span}",
    )

    return ctx.report()


object_model = build_object_model()
