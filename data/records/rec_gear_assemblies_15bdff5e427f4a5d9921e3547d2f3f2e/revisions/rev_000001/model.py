from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _bevel_gear_mesh(
    *,
    teeth: int,
    face_width: float,
    large_radius: float,
    small_radius: float,
    tooth_depth: float,
) -> MeshGeometry:
    """A visible conical bevel gear body with alternating raised teeth."""
    geom = MeshGeometry()
    count = teeth * 2
    bottom = []
    top = []
    for i in range(count):
        a = 2.0 * math.pi * i / count
        crest = (i % 2) == 0
        rb = large_radius + (tooth_depth if crest else 0.0)
        rt = small_radius + (tooth_depth * 0.45 if crest else 0.0)
        bottom.append(geom.add_vertex(rb * math.cos(a), rb * math.sin(a), -face_width / 2.0))
        top.append(geom.add_vertex(rt * math.cos(a), rt * math.sin(a), face_width / 2.0))

    for i in range(count):
        j = (i + 1) % count
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])

    bottom_center = geom.add_vertex(0.0, 0.0, -face_width / 2.0)
    top_center = geom.add_vertex(0.0, 0.0, face_width / 2.0)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_gear_assembly")

    dark_cast = Material("dark_cast_iron", color=(0.12, 0.13, 0.14, 1.0))
    machined = Material("machined_steel", color=(0.66, 0.68, 0.70, 1.0))
    bronze = Material("bearing_bronze", color=(0.72, 0.46, 0.18, 1.0))
    brass = Material("brass_gears", color=(0.88, 0.66, 0.24, 1.0))
    red = Material("red_oil_caps", color=(0.65, 0.05, 0.03, 1.0))

    base = model.part("base_bracket")
    base.visual(Box((0.62, 0.30, 0.025)), origin=Origin(xyz=(0.17, 0.04, 0.0125)), material=dark_cast, name="base_plate")

    # Vertical input-shaft gantry and bearing.
    for y in (-0.085, 0.085):
        base.visual(Box((0.035, 0.035, 0.255)), origin=Origin(xyz=(0.0, y, 0.1525)), material=dark_cast, name=f"vertical_post_{'neg' if y < 0 else 'pos'}")
    base.visual(Box((0.13, 0.070, 0.035)), origin=Origin(xyz=(0.0, -0.070, 0.285)), material=dark_cast, name="top_bridge_neg")
    base.visual(Box((0.13, 0.070, 0.035)), origin=Origin(xyz=(0.0, 0.070, 0.285)), material=dark_cast, name="top_bridge_pos")
    base.visual(Cylinder(0.040, 0.040), origin=Origin(xyz=(0.0, 0.0, 0.285)), material=bronze, name="input_bearing_bush")
    base.visual(Cylinder(0.018, 0.035), origin=Origin(xyz=(0.030, 0.070, 0.315)), material=red, name="input_oil_cap")

    # Horizontal output-shaft and reduction-shaft bearing blocks.
    for x, label in ((0.145, "inner"), (0.360, "outer")):
        base.visual(Box((0.060, 0.060, 0.155)), origin=Origin(xyz=(x, 0.0, 0.1025)), material=dark_cast, name=f"output_pedestal_{label}")
        base.visual(Cylinder(0.036, 0.058), origin=Origin(xyz=(x, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)), material=bronze, name=f"output_bearing_{label}")
    for x, label in ((0.135, "inner"), (0.355, "outer")):
        base.visual(Box((0.055, 0.055, 0.155)), origin=Origin(xyz=(x, 0.094, 0.1025)), material=dark_cast, name=f"reduction_pedestal_{label}")
        base.visual(Cylinder(0.033, 0.052), origin=Origin(xyz=(x, 0.094, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)), material=bronze, name=f"reduction_bearing_{label}")

    input_shaft = model.part("input_shaft")
    input_shaft.visual(Cylinder(0.010, 0.275), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=machined, name="vertical_shaft")
    input_shaft.visual(
        mesh_from_geometry(_bevel_gear_mesh(teeth=20, face_width=0.052, large_radius=0.052, small_radius=0.020, tooth_depth=0.006), "input_bevel_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=brass,
        name="input_bevel_gear",
    )
    input_shaft.visual(Cylinder(0.020, 0.030), origin=Origin(xyz=(0.0, 0.0, -0.078)), material=brass, name="input_gear_hub")

    output_shaft = model.part("output_shaft")
    output_shaft.visual(Cylinder(0.010, 0.360), origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=machined, name="horizontal_shaft")
    output_shaft.visual(
        mesh_from_geometry(_bevel_gear_mesh(teeth=20, face_width=0.052, large_radius=0.052, small_radius=0.020, tooth_depth=0.006), "output_bevel_mesh"),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=brass,
        name="output_bevel_gear",
    )
    output_shaft.visual(Cylinder(0.021, 0.034), origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="output_gear_hub")
    output_shaft.visual(
        mesh_from_cadquery(SpurGear(3.0, 18, 28.0).build(bore_d=18.0, hub_d=32.0, hub_length=38.0, recess_d=44.0, recess=4.0, chamfer=1.0), "spur_pinion", unit_scale=0.001),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="spur_pinion",
    )

    reduction_shaft = model.part("reduction_shaft")
    reduction_shaft.visual(Cylinder(0.011, 0.340), origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=machined, name="reduction_shaft_bar")
    reduction_shaft.visual(Cylinder(0.024, 0.052), origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=machined, name="reduction_gear_hub")
    reduction_shaft.visual(
        mesh_from_cadquery(SpurGear(3.0, 42, 34.0).build(bore_d=20.0, hub_d=42.0, hub_length=46.0, recess_d=100.0, recess=5.0, chamfer=1.0), "large_spur_gear", unit_scale=0.001),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="large_spur_gear",
    )

    model.articulation(
        "input_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=input_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "output_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=output_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=25.0),
    )
    model.articulation(
        "reduction_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=reduction_shaft,
        origin=Origin(xyz=(0.0, 0.094, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=12.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    input_shaft = object_model.get_part("input_shaft")
    output_shaft = object_model.get_part("output_shaft")
    reduction_shaft = object_model.get_part("reduction_shaft")

    ctx.allow_overlap(
        base,
        input_shaft,
        elem_a="input_bearing_bush",
        elem_b="vertical_shaft",
        reason="The visible bronze bearing bush intentionally surrounds the rotating vertical shaft.",
    )
    ctx.expect_within(input_shaft, base, axes="xy", inner_elem="vertical_shaft", outer_elem="input_bearing_bush", margin=0.001, name="input shaft is centered in its bearing")
    ctx.expect_overlap(input_shaft, base, axes="z", elem_a="vertical_shaft", elem_b="input_bearing_bush", min_overlap=0.030, name="input bearing captures the vertical shaft")

    for bearing_name in ("output_bearing_inner", "output_bearing_outer"):
        ctx.allow_overlap(
            base,
            output_shaft,
            elem_a=bearing_name,
            elem_b="horizontal_shaft",
            reason="The horizontal shaft is intentionally shown passing through a solid bronze bearing-bush proxy.",
        )
        ctx.expect_within(output_shaft, base, axes="yz", inner_elem="horizontal_shaft", outer_elem=bearing_name, margin=0.001, name=f"{bearing_name} centers the output shaft")
        ctx.expect_overlap(output_shaft, base, axes="x", elem_a="horizontal_shaft", elem_b=bearing_name, min_overlap=0.045, name=f"{bearing_name} captures the output shaft")

    for bearing_name in ("reduction_bearing_inner", "reduction_bearing_outer"):
        ctx.allow_overlap(
            base,
            reduction_shaft,
            elem_a=bearing_name,
            elem_b="reduction_shaft_bar",
            reason="The reduction shaft is intentionally seated through a bronze bearing-bush proxy.",
        )
        ctx.expect_within(reduction_shaft, base, axes="yz", inner_elem="reduction_shaft_bar", outer_elem=bearing_name, margin=0.001, name=f"{bearing_name} centers the reduction shaft")
        ctx.expect_overlap(reduction_shaft, base, axes="x", elem_a="reduction_shaft_bar", elem_b=bearing_name, min_overlap=0.040, name=f"{bearing_name} captures the reduction shaft")

    ctx.expect_overlap(input_shaft, output_shaft, axes="xy", elem_a="input_bevel_gear", elem_b="output_bevel_gear", min_overlap=0.001, name="right angle bevel gears meet near the corner")
    ctx.expect_overlap(output_shaft, reduction_shaft, axes="yz", elem_a="spur_pinion", elem_b="large_spur_gear", min_overlap=0.001, name="secondary spur gears mesh on parallel shafts")

    return ctx.report()


object_model = build_object_model()
