from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_Z = 0.36
SUPPORT_X = (-0.25, 0.25)


def _x_cylinder(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    """CadQuery cylinder whose axis is the global X axis."""
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((x - length / 2.0, y, z))
    )


def _pedestal_frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.86, 0.36, 0.055).translate((0.0, 0.0, 0.0275))

    frame = base
    for x in SUPPORT_X:
        foot = cq.Workplane("XY").box(0.17, 0.30, 0.060).translate((x, 0.0, 0.085))
        web = cq.Workplane("XY").box(0.095, 0.18, 0.245).translate((x, 0.0, 0.2375))
        bearing = _x_cylinder(0.112, 0.115, x=x, z=SHAFT_Z)
        # The bore is sized to the shaft journal so the rotating spindle is
        # physically supported by the two plain bearing blocks rather than
        # floating inside an oversized visual clearance.
        bore = _x_cylinder(0.045, 0.150, x=x, z=SHAFT_Z)
        stand = foot.union(web).union(bearing).cut(bore)
        frame = frame.union(stand)

    # Low tie rail between pedestals, visible webbing, and bolt bosses make the
    # support read as a grounded spindle module rather than two loose brackets.
    tie_rail = cq.Workplane("XY").box(0.61, 0.070, 0.060).translate((0.0, 0.0, 0.100))
    frame = frame.union(tie_rail)
    for x in SUPPORT_X:
        for y in (-0.115, 0.115):
            boss = (
                cq.Workplane("XY")
                .circle(0.020)
                .extrude(0.012)
                .translate((x, y, 0.113))
            )
            frame = frame.union(boss)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_spindle_shaft_module")

    painted_iron = model.material("painted_iron", rgba=(0.12, 0.22, 0.30, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.69, 0.66, 1.0))
    black_mark = model.material("black_index_mark", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("pedestal_frame")
    frame.visual(
        Box((0.86, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=painted_iron,
        name="base_plate",
    )
    frame.visual(
        Box((0.61, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=painted_iron,
        name="tie_rail",
    )
    for index, x in enumerate(SUPPORT_X):
        frame.visual(
            Box((0.17, 0.30, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.083)),
            material=painted_iron,
            name=f"support_foot_{index}",
        )
        frame.visual(
            Box((0.090, 0.085, 0.180)),
            origin=Origin(xyz=(x, 0.0, 0.200)),
            material=painted_iron,
            name=f"center_web_{index}",
        )
        frame.visual(
            Box((0.112, 0.035, 0.345)),
            origin=Origin(xyz=(x, -0.088, 0.248)),
            material=painted_iron,
            name=f"bearing_cheek_{index}_0",
        )
        frame.visual(
            Box((0.112, 0.035, 0.345)),
            origin=Origin(xyz=(x, 0.088, 0.248)),
            material=painted_iron,
            name=f"bearing_cheek_{index}_1",
        )
        frame.visual(
            Box((0.112, 0.118, 0.040)),
            origin=Origin(xyz=(x, 0.0, SHAFT_Z - 0.045 - 0.020)),
            material=dark_steel,
            name=f"support_saddle_{index}",
        )
        frame.visual(
            Box((0.112, 0.160, 0.045)),
            origin=Origin(xyz=(x, 0.0, SHAFT_Z + 0.045 + 0.0225)),
            material=dark_steel,
            name=f"bearing_cap_{index}",
        )
        for y in (-0.115, 0.115):
            frame.visual(
                Cylinder(radius=0.020, length=0.014),
                origin=Origin(xyz=(x, y, 0.120)),
                material=dark_steel,
                name=f"bolt_head_{index}_{0 if y < 0.0 else 1}",
            )

    # The spindle part frame is on the shaft axis.  All cylindrical visuals are
    # rotated from their native +Z axis to the spindle's +X axis.
    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.045, length=0.78),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="end_flange",
    )
    spindle.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=Origin(xyz=(0.365, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="flange_hub",
    )
    spindle.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.457, -0.030, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_mark,
        name="flange_index",
    )

    model.articulation(
        "shaft_axis",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("pedestal_frame")
    spindle = object_model.get_part("spindle")
    joint = object_model.get_articulation("shaft_axis")

    ctx.check(
        "spindle has one continuous shaft joint",
        joint.articulation_type == ArticulationType.CONTINUOUS
        and joint.parent == frame.name
        and joint.child == spindle.name
        and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, parent={joint.parent}, child={joint.child}, axis={joint.axis}",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        min_overlap=0.50,
        name="shaft spans both support stations",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="y",
        inner_elem="shaft",
        margin=0.0,
        name="shaft is centered between bearing cheeks",
    )
    for index in range(2):
        ctx.expect_contact(
            spindle,
            frame,
            elem_a="shaft",
            elem_b=f"support_saddle_{index}",
            contact_tol=0.001,
            name=f"shaft seats on lower bearing {index}",
        )
        ctx.expect_contact(
            spindle,
            frame,
            elem_a="shaft",
            elem_b=f"bearing_cap_{index}",
            contact_tol=0.001,
            name=f"shaft is captured by upper bearing {index}",
        )

    def _elem_center_z(elem_name: str) -> float | None:
        bounds = ctx.part_element_world_aabb(spindle, elem=elem_name)
        if bounds is None:
            return None
        lo, hi = bounds
        return (lo[2] + hi[2]) / 2.0

    with ctx.pose({joint: 0.0}):
        index_z_at_zero = _elem_center_z("flange_index")
    with ctx.pose({joint: math.pi}):
        index_z_at_half_turn = _elem_center_z("flange_index")

    ctx.check(
        "continuous joint rotates visible flange index",
        index_z_at_zero is not None
        and index_z_at_half_turn is not None
        and index_z_at_zero > SHAFT_Z + 0.035
        and index_z_at_half_turn < SHAFT_Z - 0.035,
        details=f"index_z_at_zero={index_z_at_zero}, index_z_at_half_turn={index_z_at_half_turn}",
    )

    return ctx.report()


object_model = build_object_model()
