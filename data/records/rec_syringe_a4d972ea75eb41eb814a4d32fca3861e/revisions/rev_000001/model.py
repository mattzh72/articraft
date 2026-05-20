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


BARREL_LENGTH = 0.110
BARREL_OUTER_R = 0.0130
BARREL_INNER_R = 0.0100
PLUNGER_TRAVEL = 0.055


def _tube_along_x(length: float, outer_r: float, inner_r: float, *, x0: float = 0.0):
    """CadQuery annular tube with a true central bore along the syringe datum axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def _hollow_nozzle():
    outer = (
        cq.Workplane("YZ", origin=(BARREL_LENGTH - 0.001, 0.0, 0.0))
        .circle(0.0060)
        .workplane(offset=0.028)
        .circle(0.0024)
        .loft(combine=True)
    )
    bore = (
        cq.Workplane("YZ", origin=(BARREL_LENGTH - 0.003, 0.0, 0.0))
        .circle(0.0011)
        .extrude(0.034)
    )
    return outer.cut(bore)


def _centered_ring(length: float, outer_r: float, inner_r: float):
    return _tube_along_x(length, outer_r, inner_r, x0=-length * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_syringe")

    clear_poly = model.material("clear_polycarbonate", rgba=(0.62, 0.88, 1.0, 0.34))
    frosted_poly = model.material("frosted_polymer", rgba=(0.86, 0.91, 0.96, 0.58))
    black_ink = model.material("black_graduation_ink", rgba=(0.02, 0.025, 0.030, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_anodized", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.08, 0.22, 0.70, 1.0))
    amber = model.material("adjustment_amber", rgba=(0.95, 0.56, 0.12, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_tube_along_x(BARREL_LENGTH, BARREL_OUTER_R, BARREL_INNER_R), "barrel_shell"),
        material=clear_poly,
        name="barrel_shell",
    )
    barrel.visual(
        mesh_from_cadquery(_hollow_nozzle(), "nozzle_tip"),
        material=frosted_poly,
        name="nozzle_tip",
    )
    barrel.visual(
        mesh_from_cadquery(_tube_along_x(0.010, BARREL_OUTER_R, 0.0013, x0=0.103), "front_socket"),
        material=frosted_poly,
        name="front_socket",
    )
    barrel.visual(
        mesh_from_cadquery(_tube_along_x(0.016, 0.0170, 0.0042, x0=-0.012), "rear_guide"),
        material=dark_steel,
        name="rear_guide",
    )

    # Datum-friendly flats and a rear calibration fixture are fixed to the barrel.
    barrel.visual(
        Box((0.112, 0.0040, 0.0040)),
        origin=Origin(xyz=(0.055, 0.0143, 0.0)),
        material=datum_blue,
        name="side_datum_flat_0",
    )
    barrel.visual(
        Box((0.112, 0.0040, 0.0040)),
        origin=Origin(xyz=(0.055, -0.0143, 0.0)),
        material=datum_blue,
        name="side_datum_flat_1",
    )
    for i, y in enumerate((-0.025, 0.025)):
        barrel.visual(
            Box((0.007, 0.020, 0.027)),
            origin=Origin(xyz=(-0.008, y, 0.0)),
            material=dark_steel,
            name=f"finger_flange_{i}",
        )
        barrel.visual(
            Box((0.128, 0.004, 0.008)),
            origin=Origin(xyz=(-0.074, y * 0.82, 0.0)),
            material=dark_steel,
            name=f"stop_rail_{i}",
        )
    barrel.visual(
        Box((0.006, 0.052, 0.019)),
        origin=Origin(xyz=(-0.139, 0.0, 0.0)),
        material=dark_steel,
        name="rear_stop_plate",
    )
    barrel.visual(
        Box((0.0020, 0.018, 0.0030)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.0178)),
        material=datum_blue,
        name="zero_index",
    )

    # Printed graduations are slightly embedded into the transparent tube surface.
    for tick in range(12):
        x = 0.010 + tick * 0.008
        is_major = tick % 2 == 0
        width = 0.014 if is_major else 0.008
        barrel.visual(
            Box((0.00075, width, 0.00055)),
            origin=Origin(xyz=(x, 0.0, BARREL_OUTER_R + 0.00005)),
            material=black_ink,
            name=f"graduation_{tick}",
        )
        if is_major:
            barrel.visual(
                Box((0.0010, 0.0010, 0.0006)),
                origin=Origin(xyz=(x + 0.0022, 0.0095, 0.00895)),
                material=black_ink,
                name=f"index_dot_{tick}",
            )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00945, length=0.0080),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="piston_seal",
    )
    plunger.visual(
        Cylinder(radius=0.0030, length=0.164),
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0062, length=0.0040),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="forward_stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0120, length=0.0060),
        origin=Origin(xyz=(-0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=datum_blue,
        name="thumb_pad",
    )
    plunger.visual(
        Box((0.007, 0.029, 0.004)),
        origin=Origin(xyz=(-0.078, 0.0, 0.0)),
        material=datum_blue,
        name="thumb_flat",
    )
    plunger.visual(
        Box((0.002, 0.006, 0.0005)),
        origin=Origin(xyz=(-0.028, 0.0, 0.00305)),
        material=black_ink,
        name="moving_index",
    )

    fine_adjuster = model.part("fine_adjuster")
    fine_adjuster.visual(
        mesh_from_cadquery(_centered_ring(0.011, 0.0072, 0.0030), "fine_adjuster_ring"),
        material=amber,
        name="adjuster_ring",
    )
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        y = math.cos(angle) * 0.0077
        z = math.sin(angle) * 0.0077
        fine_adjuster.visual(
            Box((0.010, 0.0026 if abs(z) > abs(y) else 0.0014, 0.0026 if abs(y) > abs(z) else 0.0014)),
            origin=Origin(xyz=(0.0, y, z)),
            material=dark_steel,
            name=f"knurl_pad_{i}",
        )
    fine_adjuster.visual(
        Box((0.0015, 0.0014, 0.0030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=black_ink,
        name="adjuster_index",
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.12, lower=0.0, upper=PLUNGER_TRAVEL),
    )
    model.articulation(
        "adjuster_spin",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=fine_adjuster,
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    adjuster = object_model.get_part("fine_adjuster")
    slide = object_model.get_articulation("plunger_slide")
    spin = object_model.get_articulation("adjuster_spin")

    with ctx.pose({slide: 0.0, spin: 0.0}):
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="rear_guide",
            negative_elem="forward_stop_collar",
            max_gap=0.0008,
            max_penetration=0.0001,
            name="forward hard stop is seated",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.0,
            name="piston is coaxial inside barrel bore envelope",
        )
        ctx.expect_overlap(
            adjuster,
            plunger,
            axes="x",
            elem_a="adjuster_ring",
            elem_b="plunger_rod",
            min_overlap=0.008,
            name="adjuster surrounds the plunger rod",
        )
        rest_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: PLUNGER_TRAVEL, spin: math.pi / 2.0}):
        ctx.expect_gap(
            plunger,
            barrel,
            axis="x",
            positive_elem="thumb_pad",
            negative_elem="rear_stop_plate",
            max_gap=0.0008,
            max_penetration=0.0001,
            name="rear hard stop bounds retraction",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="piston_seal",
            outer_elem="barrel_shell",
            margin=0.0,
            name="extended piston remains coaxial",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="piston_seal",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="extended piston remains retained in the barrel",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive slide retracts plunger along the datum axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.050,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
