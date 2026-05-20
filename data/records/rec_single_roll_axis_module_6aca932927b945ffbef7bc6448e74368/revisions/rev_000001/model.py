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


ROLL_AXIS_Z = 0.230
SUPPORT_X = 0.380
SUPPORT_THICKNESS = 0.050
SUPPORT_WIDTH = 0.190
SUPPORT_HEIGHT = 0.285
SUPPORT_AXIS_FROM_BOTTOM = 0.140
SHAFT_RADIUS = 0.023
BEARING_BORE_RADIUS = 0.035
TUBE_RADIUS = 0.070
TUBE_INNER_RADIUS = 0.052
TUBE_LENGTH = 0.620
SHAFT_LENGTH = 0.980


def _cylinder_x(radius: float, length: float, center_x: float = 0.0) -> cq.Solid:
    """CadQuery solid cylinder whose axis is local +X."""
    return cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(center_x - length / 2.0, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )


def _annular_cylinder_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center_x: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").add(_cylinder_x(outer_radius, length, center_x))
    cutter = cq.Workplane("XY").add(_cylinder_x(inner_radius, length + 0.006, center_x))
    return outer.cut(cutter)


def _support_plate_mesh() -> cq.Workplane:
    """Bearing plate with a true through-bore on the spindle axis."""
    plate = (
        cq.Workplane("XY")
        .box(SUPPORT_THICKNESS, SUPPORT_WIDTH, SUPPORT_HEIGHT)
        .translate((0.0, 0.0, SUPPORT_HEIGHT / 2.0 - SUPPORT_AXIS_FROM_BOTTOM))
    )
    bore = cq.Workplane("XY").add(
        _cylinder_x(BEARING_BORE_RADIUS, SUPPORT_THICKNESS + 0.030)
    )
    return plate.cut(bore)


def _spindle_tube_mesh() -> cq.Workplane:
    """Hollow roll tube with raised clamp bands and end webs tied to the shaft."""
    tube = _annular_cylinder_x(TUBE_RADIUS, TUBE_INNER_RADIUS, TUBE_LENGTH)

    # Raised bands make the otherwise cylindrical roll visibly rotate.
    for band_x in (-0.245, 0.245):
        tube = tube.union(
            cq.Workplane("XY").add(_cylinder_x(TUBE_RADIUS + 0.005, 0.038, band_x))
        )

    # Annular end webs overlap the shaft slightly, so the hollow tube is a single
    # mechanically credible rotating member instead of a shell floating around it.
    for web_x in (-TUBE_LENGTH / 2.0 + 0.010, TUBE_LENGTH / 2.0 - 0.010):
        tube = tube.union(
            _annular_cylinder_x(TUBE_RADIUS, SHAFT_RADIUS - 0.003, 0.020, center_x=web_x)
        )

    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roll_stage")

    frame_mat = model.material("black_powder_coat", rgba=(0.03, 0.035, 0.04, 1.0))
    rail_mat = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    screw_mat = model.material("black_oxide", rgba=(0.005, 0.005, 0.006, 1.0))
    tube_mat = model.material("blue_anodized_tube", rgba=(0.05, 0.20, 0.55, 1.0))
    shaft_mat = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    bearing_mat = model.material("oil_bronze", rgba=(0.78, 0.55, 0.24, 1.0))
    marker_mat = model.material("dark_index_mark", rgba=(0.01, 0.02, 0.06, 1.0))

    base = model.part("base")

    # Rectangular floor frame: two long rails and two cross rails overlap at the
    # corners so the support path is visibly continuous.
    for y, name in ((-0.160, "side_rail_0"), (0.160, "side_rail_1")):
        base.visual(
            Box((1.080, 0.050, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=rail_mat,
            name=name,
        )
    for x, name in ((-0.470, "cross_rail_0"), (0.470, "cross_rail_1")):
        base.visual(
            Box((0.070, 0.360, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=rail_mat,
            name=name,
        )

    # Four low feet with visible fasteners tie the frame to the bench/fixture.
    for i, (x, y) in enumerate(
        ((-0.430, -0.160), (-0.430, 0.160), (0.430, -0.160), (0.430, 0.160))
    ):
        base.visual(
            Box((0.155, 0.090, 0.024)),
            origin=Origin(xyz=(x, y, 0.012)),
            material=frame_mat,
            name=f"foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(x, y, 0.067)),
            material=screw_mat,
            name=f"foot_screw_{i}",
        )

    # Pedestal pads under each end support bridge into both side rails.
    base.visual(
        Box((0.125, 0.335, 0.036)),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.078)),
        material=frame_mat,
        name="pedestal_0",
    )
    base.visual(
        Box((0.125, 0.335, 0.036)),
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.078)),
        material=frame_mat,
        name="pedestal_1",
    )

    support_mesh = mesh_from_cadquery(_support_plate_mesh(), "bearing_plate")
    for side, sx in enumerate((-SUPPORT_X, SUPPORT_X)):
        base.visual(
            support_mesh,
            origin=Origin(xyz=(sx, 0.0, ROLL_AXIS_Z)),
            material=frame_mat,
            name=f"support_plate_{side}",
        )

        # Side ribs are simple welded gussets; they touch both plate and pedestal.
        for rib, y in enumerate((-0.099, 0.099)):
            base.visual(
                Box((0.058, 0.014, 0.160)),
                origin=Origin(xyz=(sx, y, 0.164)),
                material=frame_mat,
                name=f"support_rib_{side}_{rib}",
            )

    collar_mesh = mesh_from_cadquery(
        _annular_cylinder_x(0.052, 0.032, 0.018), "bearing_collar"
    )
    left_inner_x = -SUPPORT_X + SUPPORT_THICKNESS / 2.0 + 0.007
    left_outer_x = -SUPPORT_X - SUPPORT_THICKNESS / 2.0 - 0.007
    right_inner_x = SUPPORT_X - SUPPORT_THICKNESS / 2.0 - 0.007
    right_outer_x = SUPPORT_X + SUPPORT_THICKNESS / 2.0 + 0.007

    base.visual(
        collar_mesh,
        origin=Origin(xyz=(left_inner_x, 0.0, ROLL_AXIS_Z)),
        material=rail_mat,
        name="inner_collar_0",
    )
    base.visual(
        collar_mesh,
        origin=Origin(xyz=(left_outer_x, 0.0, ROLL_AXIS_Z)),
        material=rail_mat,
        name="outer_collar_0",
    )
    base.visual(
        collar_mesh,
        origin=Origin(xyz=(right_inner_x, 0.0, ROLL_AXIS_Z)),
        material=rail_mat,
        name="inner_collar_1",
    )
    base.visual(
        collar_mesh,
        origin=Origin(xyz=(right_outer_x, 0.0, ROLL_AXIS_Z)),
        material=rail_mat,
        name="outer_collar_1",
    )

    # Paired cap screws on each split collar, embedded slightly into the curved
    # collar so they are grounded rather than floating details.
    for collar_name, cx in (
        ("inner_collar_0", left_inner_x),
        ("outer_collar_0", left_outer_x),
        ("inner_collar_1", right_inner_x),
        ("outer_collar_1", right_outer_x),
    ):
        for screw_i, y in enumerate((-0.026, 0.026)):
            base.visual(
                Cylinder(radius=0.006, length=0.007),
                origin=Origin(xyz=(cx, y, ROLL_AXIS_Z + 0.0495)),
                material=screw_mat,
                name=f"{collar_name}_screw_{screw_i}",
            )

    # Lower bearing shoes give the shaft exact physical support without filling
    # the roll clearance envelope; their upper faces are tangent to the shaft.
    base.visual(
        Box((0.056, 0.052, 0.014)),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, ROLL_AXIS_Z - SHAFT_RADIUS - 0.007)),
        material=bearing_mat,
        name="bearing_shoe_0",
    )
    base.visual(
        Box((0.056, 0.052, 0.014)),
        origin=Origin(xyz=(SUPPORT_X, 0.0, ROLL_AXIS_Z - SHAFT_RADIUS - 0.007)),
        material=bearing_mat,
        name="bearing_shoe_1",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_mat,
        name="shaft",
    )
    spindle.visual(
        mesh_from_cadquery(_spindle_tube_mesh(), "spindle_tube"),
        material=tube_mat,
        name="tube_body",
    )
    spindle.visual(
        Box((0.460, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, TUBE_RADIUS + 0.003)),
        material=marker_mat,
        name="index_strip",
    )
    for i, x in enumerate((-0.492, 0.492)):
        spindle.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shaft_mat,
            name=f"end_cap_{i}",
        )

    model.articulation(
        "base_to_spindle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=12.0,
            lower=-math.pi,
            upper=math.pi,
        ),
        motion_properties=MotionProperties(damping=0.02, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("base_to_spindle")

    ctx.check(
        "single roll revolute",
        roll.articulation_type == ArticulationType.REVOLUTE and tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"type={roll.articulation_type}, axis={roll.axis}",
    )

    # The support plates and collars overlap the shaft in X projection while
    # their real bores provide clearance; this proves the roll is supported by
    # aligned end bearings rather than reading as a floating tube.
    for side in (0, 1):
        ctx.expect_overlap(
            spindle,
            base,
            axes="x",
            min_overlap=0.020,
            elem_a="shaft",
            elem_b=f"support_plate_{side}",
            name=f"shaft passes through support {side}",
        )
        ctx.expect_overlap(
            spindle,
            base,
            axes="x",
            min_overlap=0.010,
            elem_a="shaft",
            elem_b=f"inner_collar_{side}",
            name=f"shaft is captured by inner collar {side}",
        )
        ctx.expect_gap(
            spindle,
            base,
            axis="z",
            max_gap=0.00001,
            max_penetration=0.000001,
            positive_elem="shaft",
            negative_elem=f"bearing_shoe_{side}",
            name=f"shaft rests on bearing shoe {side}",
        )

    ctx.expect_gap(
        spindle,
        base,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem="tube_body",
        negative_elem="inner_collar_0",
        name="tube clears left bearing collar",
    )
    ctx.expect_gap(
        base,
        spindle,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem="inner_collar_1",
        negative_elem="tube_body",
        name="tube clears right bearing collar",
    )

    rest_strip = ctx.part_element_world_aabb(spindle, elem="index_strip")
    with ctx.pose({roll: math.pi / 2.0}):
        turned_strip = ctx.part_element_world_aabb(spindle, elem="index_strip")
        ctx.expect_gap(
            spindle,
            base,
            axis="z",
            min_gap=0.045,
            positive_elem="tube_body",
            negative_elem="pedestal_0",
            name="tube clears pedestal while rolled",
        )

    if rest_strip is not None and turned_strip is not None:
        rest_y = (rest_strip[0][1] + rest_strip[1][1]) / 2.0
        turned_y = (turned_strip[0][1] + turned_strip[1][1]) / 2.0
        ctx.check(
            "spindle visibly rotates",
            abs(turned_y - rest_y) > 0.055,
            details=f"rest_y={rest_y:.4f}, turned_y={turned_y:.4f}",
        )
    else:
        ctx.fail("spindle visibly rotates", "index strip AABBs were unavailable")

    return ctx.report()


object_model = build_object_model()
