from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_pickup_truck")

    red = Material("glossy_toy_red", color=(0.86, 0.08, 0.04, 1.0))
    dark_red = Material("dark_red_inner_bed", color=(0.48, 0.03, 0.02, 1.0))
    black = Material("soft_black_rubber", color=(0.01, 0.01, 0.01, 1.0))
    silver = Material("bright_silver_plastic", color=(0.78, 0.78, 0.72, 1.0))
    blue = Material("smoky_blue_windows", color=(0.12, 0.35, 0.58, 0.72))
    amber = Material("amber_lamps", color=(1.0, 0.58, 0.05, 1.0))
    white = Material("white_headlights", color=(0.95, 0.95, 0.86, 1.0))

    body = model.part("body")

    # One connected molded toy body: low chassis, boxy cab, open cargo bed,
    # fenders, bumper, and visible axle stubs.
    body.visual(
        Box((0.60, 0.18, 0.045)),
        origin=Origin(xyz=(-0.005, 0.0, 0.092)),
        material=red,
        name="chassis",
    )
    body.visual(
        Box((0.17, 0.225, 0.135)),
        origin=Origin(xyz=(-0.005, 0.0, 0.1775)),
        material=red,
        name="boxy_cab",
    )
    body.visual(
        Box((0.19, 0.235, 0.018)),
        origin=Origin(xyz=(-0.005, 0.0, 0.253)),
        material=red,
        name="flat_roof",
    )
    body.visual(
        Box((0.225, 0.205, 0.012)),
        origin=Origin(xyz=(-0.1975, 0.0, 0.124)),
        material=dark_red,
        name="bed_floor",
    )
    body.visual(
        Box((0.235, 0.016, 0.082)),
        origin=Origin(xyz=(-0.1975, 0.1105, 0.165)),
        material=red,
        name="bed_side_0",
    )
    body.visual(
        Box((0.235, 0.016, 0.082)),
        origin=Origin(xyz=(-0.1975, -0.1105, 0.165)),
        material=red,
        name="bed_side_1",
    )
    body.visual(
        Box((0.014, 0.225, 0.086)),
        origin=Origin(xyz=(-0.082, 0.0, 0.166)),
        material=red,
        name="bed_front",
    )
    body.visual(
        Box((0.020, 0.210, 0.060)),
        origin=Origin(xyz=(0.305, 0.0, 0.115)),
        material=silver,
        name="front_grille",
    )
    body.visual(
        Box((0.020, 0.055, 0.022)),
        origin=Origin(xyz=(0.317, 0.055, 0.127)),
        material=white,
        name="headlight_0",
    )
    body.visual(
        Box((0.020, 0.055, 0.022)),
        origin=Origin(xyz=(0.317, -0.055, 0.127)),
        material=white,
        name="headlight_1",
    )
    body.visual(
        Box((0.028, 0.245, 0.030)),
        origin=Origin(xyz=(0.325, 0.0, 0.079)),
        material=silver,
        name="front_bumper",
    )
    body.visual(
        Box((0.026, 0.245, 0.030)),
        origin=Origin(xyz=(-0.318, 0.0, 0.083)),
        material=silver,
        name="rear_bumper",
    )

    # Rectangular toy fenders hover just above the tires and overlap the body
    # sides so they are supported molded features rather than loose blocks.
    for name, x, y in (
        ("front_fender_0", 0.195, 0.145),
        ("front_fender_1", 0.195, -0.145),
        ("rear_fender_0", -0.205, 0.145),
        ("rear_fender_1", -0.205, -0.145),
    ):
        body.visual(
            Box((0.135, 0.082, 0.024)),
            origin=Origin(xyz=(x, y, 0.165)),
            material=red,
            name=name,
        )
    for name, y in (("front_side_rail_0", 0.101), ("front_side_rail_1", -0.101)):
        body.visual(
            Box((0.220, 0.024, 0.030)),
            origin=Origin(xyz=(0.190, y, 0.122)),
            material=red,
            name=name,
        )
    for name, y in (("front_fender_support_0", 0.115), ("front_fender_support_1", -0.115)):
        body.visual(
            Box((0.128, 0.014, 0.060)),
            origin=Origin(xyz=(0.195, y, 0.136)),
            material=red,
            name=name,
        )

    # Axle pins pass through the wheel bores.  The slight captured fit is
    # declared in tests at the axle/rim elements only.
    for name, x in (("front_axle", 0.195), ("rear_axle", -0.205)):
        body.visual(
            Cylinder(radius=0.0065, length=0.350),
            origin=Origin(xyz=(x, 0.0, 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=name,
        )

    # Dark blue inset window pieces on the boxy cab.
    body.visual(
        Box((0.004, 0.148, 0.058)),
        origin=Origin(xyz=(0.082, 0.0, 0.199)),
        material=blue,
        name="windshield",
    )
    body.visual(
        Box((0.072, 0.004, 0.052)),
        origin=Origin(xyz=(-0.010, 0.114, 0.203)),
        material=blue,
        name="side_window_0",
    )
    body.visual(
        Box((0.072, 0.004, 0.052)),
        origin=Origin(xyz=(-0.010, -0.114, 0.203)),
        material=blue,
        name="side_window_1",
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.225, 0.210, 0.018)),
        # The hood part frame sits on the cowl hinge line; the panel extends
        # forward along local +X.
        origin=Origin(xyz=(0.1125, 0.0, -0.006)),
        material=red,
        name="hood_panel",
    )
    hood.visual(
        Cylinder(radius=0.006, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, -0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hood_hinge_barrel",
    )
    hood.visual(
        Box((0.035, 0.018, 0.009)),
        origin=Origin(xyz=(0.165, 0.095, 0.006)),
        material=amber,
        name="marker_0",
    )
    hood.visual(
        Box((0.035, 0.018, 0.009)),
        origin=Origin(xyz=(0.165, -0.095, 0.006)),
        material=amber,
        name="marker_1",
    )
    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.082, 0.0, 0.156)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.012, 0.215, 0.082)),
        # The tailgate frame is on the lower hinge.  In the closed pose the
        # panel stands upward and sits just outside the bed opening.
        origin=Origin(xyz=(-0.006, 0.0, 0.041)),
        material=red,
        name="tailgate_panel",
    )
    tailgate.visual(
        Cylinder(radius=0.006, length=0.205),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="tailgate_hinge_barrel",
    )
    tailgate.visual(
        Box((0.006, 0.038, 0.018)),
        origin=Origin(xyz=(-0.012, 0.083, 0.062)),
        material=amber,
        name="taillight_0",
    )
    tailgate.visual(
        Box((0.006, 0.038, 0.018)),
        origin=Origin(xyz=(-0.012, -0.083, 0.062)),
        material=amber,
        name="taillight_1",
    )
    model.articulation(
        "body_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tailgate,
        origin=Origin(xyz=(-0.312, 0.0, 0.109)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.044,
            rim=WheelRim(inner_radius=0.034, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.017,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.026, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.012),
        ),
        "toy_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.072,
            0.058,
            inner_radius=0.053,
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "toy_tire",
    )

    wheel_specs = (
        ("front_left_wheel", "body_to_front_left_wheel", 0.195, 0.148),
        ("front_right_wheel", "body_to_front_right_wheel", 0.195, -0.148),
        ("rear_left_wheel", "body_to_rear_left_wheel", -0.205, 0.148),
        ("rear_right_wheel", "body_to_rear_right_wheel", -0.205, -0.148),
    )
    for part_name, joint_name, x, y in wheel_specs:
        wheel = model.part(part_name)
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=silver, name="rim")
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.075), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    tailgate = object_model.get_part("tailgate")
    hood_hinge = object_model.get_articulation("body_to_hood")
    tailgate_hinge = object_model.get_articulation("body_to_tailgate")

    wheel_mounts = (
        ("front_left_wheel", "front_axle"),
        ("front_right_wheel", "front_axle"),
        ("rear_left_wheel", "rear_axle"),
        ("rear_right_wheel", "rear_axle"),
    )
    for wheel_name, axle_elem in wheel_mounts:
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=axle_elem,
            elem_b="rim",
            reason="The toy wheel rim bore is intentionally captured on the axle pin so it can spin.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=axle_elem,
            elem_b="rim",
            min_overlap=0.020,
            name=f"{wheel_name} remains on its axle",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xz",
            elem_a=axle_elem,
            elem_b="rim",
            min_overlap=0.005,
            name=f"{wheel_name} is centered on the axle",
        )

    ctx.expect_overlap(
        hood,
        body,
        axes="y",
        elem_a="hood_hinge_barrel",
        elem_b="boxy_cab",
        min_overlap=0.12,
        name="hood hinge spans the cowl width",
    )
    ctx.expect_overlap(
        tailgate,
        body,
        axes="y",
        elem_a="tailgate_panel",
        elem_b="bed_side_0",
        min_overlap=0.002,
        name="tailgate closes across the bed width",
    )

    with ctx.pose({hood_hinge: 0.9}):
        ctx.expect_gap(
            hood,
            body,
            axis="z",
            positive_elem="marker_0",
            negative_elem="front_grille",
            min_gap=0.05,
            name="hood rotates upward at the cowl",
        )

    with ctx.pose({tailgate_hinge: 1.2}):
        ctx.expect_gap(
            body,
            tailgate,
            axis="x",
            positive_elem="rear_bumper",
            negative_elem="taillight_0",
            min_gap=0.015,
            name="tailgate rotates rearward and downward",
        )

    for joint_name in (
        "body_to_front_left_wheel",
        "body_to_front_right_wheel",
        "body_to_rear_left_wheel",
        "body_to_rear_right_wheel",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    return ctx.report()


object_model = build_object_model()
