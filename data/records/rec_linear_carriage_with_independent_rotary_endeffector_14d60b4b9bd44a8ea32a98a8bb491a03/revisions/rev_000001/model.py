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
    mesh_from_cadquery,
)


def _linear_bearing_mesh(name: str):
    """Rectangular carriage bearing block with a clear through-bore for a rail."""
    block = cq.Workplane("XY").box(0.096, 0.050, 0.052)
    rail_clearance = cq.Workplane("YZ").circle(0.017).extrude(0.120, both=True)
    bearing = block.cut(rail_clearance).edges("|X").fillet(0.002)
    return mesh_from_cadquery(bearing, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_linear_axis_turning_head")

    dark_plate = Material("powder_coated_charcoal", rgba=(0.05, 0.055, 0.06, 1.0))
    black = Material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    blue = Material("anodized_blue", rgba=(0.08, 0.23, 0.58, 1.0))
    white = Material("painted_index", rgba=(0.92, 0.93, 0.90, 1.0))
    amber = Material("amber_stop", rgba=(1.0, 0.58, 0.08, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.48, 0.035, 0.82)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=dark_plate,
        name="back_plate",
    )
    backplate.visual(
        Box((0.41, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.0225, 0.45)),
        material=black,
        name="center_slot",
    )

    # Two polished horizontal guide rails stand off the front of the wall plate.
    for z, rail_name in ((0.57, "upper_rail"), (0.33, "lower_rail")):
        backplate.visual(
            Cylinder(radius=0.012, length=0.420),
            origin=Origin(xyz=(0.0, -0.040, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=rail_name,
        )
        for x, mount_name in ((-0.215, f"{rail_name}_mount_0"), (0.215, f"{rail_name}_mount_1")):
            backplate.visual(
                Box((0.045, 0.050, 0.055)),
                origin=Origin(xyz=(x, -0.025, z)),
                material=black,
                name=mount_name,
            )

    for x, z, screw_name in (
        (-0.195, 0.78, "screw_0"),
        (0.195, 0.78, "screw_1"),
        (-0.195, 0.12, "screw_2"),
        (0.195, 0.12, "screw_3"),
    ):
        backplate.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -0.0205, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=screw_name,
        )

    for x, stop_name in ((-0.235, "end_stop_0"), (0.235, "end_stop_1")):
        backplate.visual(
            Box((0.018, 0.036, 0.34)),
            origin=Origin(xyz=(x, -0.044, 0.45)),
            material=amber,
            name=stop_name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.115, 0.040, 0.350)),
        origin=Origin(xyz=(0.0, -0.076, 0.0)),
        material=black,
        name="main_plate",
    )
    bearing_mesh = _linear_bearing_mesh("linear_bearing")
    carriage.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.120)),
        material=steel,
        name="upper_bearing",
    )
    carriage.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, -0.040, -0.120)),
        material=steel,
        name="lower_bearing",
    )
    carriage.visual(
        Box((0.080, 0.028, 0.285)),
        origin=Origin(xyz=(0.0, -0.080, 0.0)),
        material=dark_plate,
        name="bearing_bridge",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, -0.106, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rotary_boss",
    )

    slide = model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(-0.140, 0.0, 0.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.280),
    )
    slide.meta["description"] = "Horizontal carriage travel on the wall-backed guide rails."

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=blue,
        name="disk",
    )
    faceplate.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=steel,
        name="front_hub",
    )
    faceplate.visual(
        Box((0.010, 0.052, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.022)),
        material=white,
        name="indicator",
    )

    spin = model.articulation(
        "face_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=faceplate,
        # Rotate the joint frame so its local +Z is the front normal (-Y).
        origin=Origin(xyz=(0.0, -0.116, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    spin.meta["description"] = "Independent rotary face carried by the moving carriage."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    faceplate = object_model.get_part("faceplate")
    slide = object_model.get_articulation("guide_slide")
    spin = object_model.get_articulation("face_spin")

    ctx.check(
        "linear guide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "face is revolute",
        spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={spin.articulation_type}",
    )

    ctx.expect_overlap(
        carriage,
        backplate,
        axes="x",
        elem_a="upper_bearing",
        elem_b="upper_rail",
        min_overlap=0.080,
        name="upper bearing remains on upper rail",
    )
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="x",
        elem_a="lower_bearing",
        elem_b="lower_rail",
        min_overlap=0.080,
        name="lower bearing remains on lower rail",
    )
    ctx.expect_contact(
        faceplate,
        carriage,
        elem_a="disk",
        elem_b="rotary_boss",
        contact_tol=0.001,
        name="faceplate seats against rotary boss",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_face = ctx.part_world_position(faceplate)
    with ctx.pose({slide: 0.280}):
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="x",
            elem_a="upper_bearing",
            elem_b="upper_rail",
            min_overlap=0.080,
            name="extended upper bearing still retained",
        )
        ctx.expect_overlap(
            carriage,
            backplate,
            axes="x",
            elem_a="lower_bearing",
            elem_b="lower_rail",
            min_overlap=0.080,
            name="extended lower bearing still retained",
        )
        extended_carriage = ctx.part_world_position(carriage)
        extended_face = ctx.part_world_position(faceplate)

    ctx.check(
        "carriage translates along guide",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.25,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "face is carried by carriage",
        rest_face is not None
        and extended_face is not None
        and extended_face[0] > rest_face[0] + 0.25,
        details=f"rest={rest_face}, extended={extended_face}",
    )

    indicator_rest = ctx.part_element_world_aabb(faceplate, elem="indicator")
    with ctx.pose({spin: math.pi / 2.0}):
        indicator_quarter = ctx.part_element_world_aabb(faceplate, elem="indicator")

    def _extent(aabb, idx: int) -> float:
        return float(aabb[1][idx] - aabb[0][idx])

    ctx.check(
        "face indicator rotates in plane",
        indicator_rest is not None
        and indicator_quarter is not None
        and _extent(indicator_rest, 2) > _extent(indicator_rest, 0) * 2.5
        and _extent(indicator_quarter, 0) > _extent(indicator_quarter, 2) * 2.5,
        details=f"rest={indicator_rest}, quarter={indicator_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
