from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_bar_stool")

    polished_steel = model.material("polished_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    graphite = model.material("graphite", rgba=(0.15, 0.15, 0.16, 1.0))
    upholstery = model.material("upholstery", rgba=(0.21, 0.18, 0.16, 1.0))
    backing = model.material("backing", rgba=(0.12, 0.12, 0.13, 1.0))

    base_shell_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.010),
                (0.090, 0.004),
                (0.190, 0.000),
                (0.225, 0.004),
                (0.232, 0.010),
                (0.215, 0.022),
                (0.160, 0.031),
                (0.060, 0.034),
                (0.0, 0.034),
            ],
            segments=72,
        ),
        "stool_base_shell",
    )
    foot_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.155,
            tube=0.012,
            radial_segments=20,
            tubular_segments=84,
        ),
        "stool_foot_ring",
    )

    seat_pad_geom = superellipse_side_loft(
        [
            (-0.190, 0.000, 0.050, 0.385),
            (-0.080, 0.000, 0.066, 0.405),
            (0.060, 0.000, 0.074, 0.418),
            (0.195, 0.000, 0.052, 0.382),
        ],
        exponents=3.0,
        segments=56,
    )
    seat_pad_geom.translate(0.0, 0.014, 0.076)
    seat_pad_mesh = mesh_from_geometry(seat_pad_geom, "seat_pad")

    back_pad_geom = superellipse_side_loft(
        [
            (0.000, -0.020, 0.020, 0.285),
            (0.100, -0.022, 0.022, 0.292),
            (0.210, -0.020, 0.020, 0.272),
            (0.300, -0.016, 0.016, 0.228),
        ],
        exponents=3.0,
        segments=52,
    )
    back_pad_geom.rotate_x(math.pi / 2.0 + 0.14).translate(0.0, -0.032, 0.070)
    back_pad_mesh = mesh_from_geometry(back_pad_geom, "back_pad")

    pedestal = model.part("pedestal")
    pedestal.visual(base_shell_mesh, material=polished_steel, name="base_shell")
    pedestal.visual(
        Cylinder(radius=0.160, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=graphite,
        name="floor_glide",
    )
    pedestal.visual(
        Cylinder(radius=0.036, length=0.568),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=polished_steel,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.632)),
        material=polished_steel,
        name="bearing_cap",
    )
    pedestal.visual(
        Cylinder(radius=0.047, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=polished_steel,
        name="foot_ring_collar",
    )
    pedestal.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=polished_steel,
        name="foot_ring",
    )
    arm_length = 0.102
    arm_center = 0.098
    for sign in (-1.0, 1.0):
        pedestal.visual(
            Cylinder(radius=0.009, length=arm_length),
            origin=Origin(xyz=(sign * arm_center, 0.0, 0.290), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name=f"ring_arm_x_{int(sign > 0)}",
        )
        pedestal.visual(
            Cylinder(radius=0.009, length=arm_length),
            origin=Origin(xyz=(0.0, sign * arm_center, 0.290), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name=f"ring_arm_y_{int(sign > 0)}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.054, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_steel,
        name="swivel_hub",
    )
    seat.visual(
        Cylinder(radius=0.097, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark_steel,
        name="turntable_plate",
    )
    seat.visual(
        Box((0.300, 0.250, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=backing,
        name="seat_frame",
    )
    seat.visual(
        Box((0.344, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.158, 0.070)),
        material=backing,
        name="front_apron",
    )
    for sign in (-1.0, 1.0):
        seat.visual(
            Box((0.030, 0.300, 0.050)),
            origin=Origin(xyz=(sign * 0.157, 0.0, 0.070)),
            material=backing,
            name=f"side_apron_{int(sign > 0)}",
        )
    seat.visual(
        Box((0.214, 0.030, 0.034)),
        origin=Origin(xyz=(0.0, -0.152, 0.060)),
        material=backing,
        name="rear_brace",
    )
    seat.visual(seat_pad_mesh, material=upholstery, name="seat_pad")
    seat.visual(
        Box((0.070, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, -0.178, 0.082)),
        material=dark_steel,
        name="hinge_bridge",
    )
    seat.visual(
        Cylinder(radius=0.016, length=0.071),
        origin=Origin(xyz=(0.0, -0.195, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_knuckle",
    )
    for sign in (-1.0, 1.0):
        seat.visual(
            Box((0.032, 0.030, 0.060)),
            origin=Origin(xyz=(sign * 0.106, -0.172, 0.078)),
            material=dark_steel,
            name=f"hinge_cheek_{int(sign > 0)}",
        )

    back = model.part("back")
    for x_pos, name in ((-0.058, "outer_knuckle_0"), (0.058, "outer_knuckle_1")):
        back.visual(
            Cylinder(radius=0.018, length=0.045),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=name,
        )
    for sign in (-1.0, 1.0):
        back.visual(
            Box((0.038, 0.022, 0.074)),
            origin=Origin(xyz=(sign * 0.082, -0.010, 0.034), rpy=(0.14, 0.0, 0.0)),
            material=dark_steel,
            name=f"lower_cheek_{int(sign > 0)}",
        )
        back.visual(
            Cylinder(radius=0.015, length=0.188),
            origin=Origin(xyz=(sign * 0.112, -0.022, 0.122), rpy=(0.14, 0.0, 0.0)),
            material=dark_steel,
            name=f"side_rail_{int(sign > 0)}",
        )
    back.visual(
        Box((0.274, 0.024, 0.228)),
        origin=Origin(xyz=(0.0, -0.040, 0.184), rpy=(0.14, 0.0, 0.0)),
        material=backing,
        name="back_shell",
    )
    back.visual(back_pad_mesh, material=upholstery, name="back_pad")
    back.visual(
        Box((0.240, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.048, 0.262), rpy=(0.14, 0.0, 0.0)),
        material=backing,
        name="top_rail",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.662)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0),
    )
    model.articulation(
        "back_tilt",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(0.0, -0.195, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=0.38,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_tilt = object_model.get_articulation("back_tilt")

    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_hub",
        elem_b="bearing_cap",
        min_overlap=0.090,
        name="seat swivel stays centered on pedestal bearing",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_hub",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.003,
        name="seat hub sits just above pedestal cap",
    )
    ctx.expect_overlap(
        back,
        seat,
        axes="x",
        elem_a="back_pad",
        elem_b="seat_pad",
        min_overlap=0.180,
        name="back stays aligned across seat width",
    )

    def aabb_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    with ctx.pose({seat_swivel: 0.0}):
        rest_back_center = aabb_center(back, "back_pad")
    with ctx.pose({seat_swivel: math.pi / 2.0}):
        quarter_turn_back_center = aabb_center(back, "back_pad")

    ctx.check(
        "seat swivel moves the back around the pedestal axis",
        rest_back_center is not None
        and quarter_turn_back_center is not None
        and quarter_turn_back_center[0] > rest_back_center[0] + 0.14
        and abs(quarter_turn_back_center[1]) < 0.07,
        details=f"rest={rest_back_center}, quarter_turn={quarter_turn_back_center}",
    )

    with ctx.pose({back_tilt: 0.0}):
        back_rest_center = aabb_center(back, "back_pad")
    with ctx.pose({back_tilt: 0.32}):
        back_reclined_center = aabb_center(back, "back_pad")

    ctx.check(
        "back tilt reclines rearward",
        back_rest_center is not None
        and back_reclined_center is not None
        and back_reclined_center[1] < back_rest_center[1] - 0.025,
        details=f"rest={back_rest_center}, reclined={back_reclined_center}",
    )

    return ctx.report()


object_model = build_object_model()
