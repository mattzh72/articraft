from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    model = ArticulatedObject(name="toy_utility_wagon_car")

    toy_red = model.material("painted_red", rgba=(0.78, 0.06, 0.04, 1.0))
    dark = model.material("dark_chassis", rgba=(0.03, 0.035, 0.04, 1.0))
    black = model.material("rubber_black", rgba=(0.01, 0.01, 0.012, 1.0))
    cream = model.material("cream_wheel", rgba=(0.93, 0.84, 0.62, 1.0))
    silver = model.material("axle_silver", rgba=(0.58, 0.60, 0.62, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.78, 0.92, 0.42))

    body = model.part("body")

    # A single sturdy toy body: low chassis, squared cabin, front engine bay,
    # and an open-topped cargo tub behind the cabin.
    body.visual(
        Box((0.78, 0.38, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark,
        name="chassis",
    )
    body.visual(
        Box((0.28, 0.025, 0.13)),
        origin=Origin(xyz=(0.255, 0.177, 0.255)),
        material=toy_red,
        name="engine_side_0",
    )
    body.visual(
        Box((0.28, 0.025, 0.13)),
        origin=Origin(xyz=(0.255, -0.177, 0.255)),
        material=toy_red,
        name="engine_side_1",
    )
    body.visual(
        Box((0.035, 0.36, 0.11)),
        origin=Origin(xyz=(0.39, 0.0, 0.235)),
        material=toy_red,
        name="front_nose",
    )
    body.visual(
        Box((0.19, 0.36, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.265)),
        material=toy_red,
        name="cabin_base",
    )
    body.visual(
        Box((0.22, 0.35, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=toy_red,
        name="cabin_roof",
    )
    for i, (x, y) in enumerate(
        ((0.105, 0.155), (0.105, -0.155), (-0.105, 0.155), (-0.105, -0.155))
    ):
        body.visual(
            Box((0.025, 0.035, 0.22)),
            origin=Origin(xyz=(x, y, 0.405)),
            material=toy_red,
            name=f"cabin_pillar_{i}",
        )
    body.visual(
        Box((0.010, 0.285, 0.155)),
        origin=Origin(xyz=(0.116, 0.0, 0.405)),
        material=glass,
        name="windshield",
    )
    body.visual(
        Box((0.010, 0.285, 0.150)),
        origin=Origin(xyz=(-0.116, 0.0, 0.405)),
        material=glass,
        name="rear_window",
    )
    body.visual(
        Box((0.31, 0.34, 0.035)),
        origin=Origin(xyz=(-0.245, 0.0, 0.1875)),
        material=toy_red,
        name="cargo_floor",
    )
    body.visual(
        Box((0.30, 0.035, 0.16)),
        origin=Origin(xyz=(-0.245, 0.185, 0.27)),
        material=toy_red,
        name="cargo_side_0",
    )
    body.visual(
        Box((0.30, 0.035, 0.16)),
        origin=Origin(xyz=(-0.245, -0.185, 0.27)),
        material=toy_red,
        name="cargo_side_1",
    )
    body.visual(
        Box((0.025, 0.36, 0.18)),
        origin=Origin(xyz=(-0.087, 0.0, 0.28)),
        material=toy_red,
        name="cargo_bulkhead",
    )

    # Straight toy axle rods sit below the body and meet the wheel inner faces.
    for i, x in enumerate((0.28, -0.28)):
        body.visual(
            Cylinder(radius=0.015, length=0.430),
            origin=Origin(xyz=(x, 0.0, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=f"axle_{i}",
        )

    # Four simple fenders, one piece with the main body via inner brackets.
    for i, (x, y) in enumerate(
        ((0.28, 0.255), (0.28, -0.255), (-0.28, 0.255), (-0.28, -0.255))
    ):
        side = 1.0 if y > 0.0 else -1.0
        body.visual(
            Box((0.18, 0.085, 0.025)),
            origin=Origin(xyz=(x, y, 0.21)),
            material=toy_red,
            name=f"fender_cap_{i}",
        )
        body.visual(
            Box((0.18, 0.030, 0.075)),
            origin=Origin(xyz=(x, side * 0.202, 0.18)),
            material=toy_red,
            name=f"fender_bracket_{i}",
        )

    body.visual(
        Cylinder(radius=0.0065, length=0.34),
        origin=Origin(xyz=(-0.402, 0.0, 0.1885), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="gate_hinge_pin",
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.27, 0.35, 0.025)),
        # The hood part frame is on the rear hinge line; the panel extends forward.
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=toy_red,
        name="hood_panel",
    )
    hood.visual(
        Box((0.020, 0.24, 0.012)),
        origin=Origin(xyz=(0.245, 0.0, 0.0185)),
        material=silver,
        name="hood_handle",
    )

    rear_gate = model.part("rear_gate")
    rear_gate.visual(
        Box((0.025, 0.35, 0.16)),
        # The gate part frame is on the lower hinge line; the panel rises upward.
        origin=Origin(xyz=(-0.0125, 0.0, 0.08)),
        material=toy_red,
        name="gate_panel",
    )
    rear_gate.visual(
        Box((0.010, 0.22, 0.018)),
        origin=Origin(xyz=(-0.028, 0.0, 0.135)),
        material=silver,
        name="gate_latch_bar",
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.090,
            0.055,
            inner_radius=0.060,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "utility_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.064,
            0.056,
            rim=WheelRim(inner_radius=0.042, flange_height=0.005, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.020,
                width=0.050,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.028, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "cream_wheel",
    )

    wheel_locations = {
        "wheel_front_left": (0.28, 0.255, 0.105),
        "wheel_front_right": (0.28, -0.255, 0.105),
        "wheel_rear_left": (-0.28, 0.255, 0.105),
        "wheel_rear_right": (-0.28, -0.255, 0.105),
    }
    for name, loc in wheel_locations.items():
        wheel = model.part(name)
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(wheel_mesh, material=cream, name="wheel_center")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=loc, rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.12, 0.0, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_rear_gate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_gate,
        origin=Origin(xyz=(-0.402, 0.0, 0.195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood = object_model.get_part("hood")
    gate = object_model.get_part("rear_gate")
    body = object_model.get_part("body")
    hood_joint = object_model.get_articulation("body_to_hood")
    gate_joint = object_model.get_articulation("body_to_rear_gate")

    wheel_names = (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    )
    for wheel_name in wheel_names:
        joint = object_model.get_articulation(f"body_to_{wheel_name}")
        ctx.check(
            f"{wheel_name} has continuous axle rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{wheel_name} joint type={joint.articulation_type}",
        )

    ctx.expect_gap(
        hood,
        body,
        axis="z",
        positive_elem="hood_panel",
        negative_elem="engine_side_0",
        min_gap=0.0,
        max_gap=0.03,
        name="hood sits just over engine bay",
    )
    ctx.expect_overlap(
        hood,
        body,
        axes="x",
        elem_a="hood_panel",
        elem_b="engine_side_0",
        min_overlap=0.24,
        name="hood length covers front compartment",
    )
    ctx.expect_overlap(
        gate,
        body,
        axes="y",
        elem_a="gate_panel",
        elem_b="cargo_floor",
        min_overlap=0.30,
        name="rear gate spans cargo bay width",
    )

    hood_closed = ctx.part_world_aabb(hood)
    gate_closed = ctx.part_world_aabb(gate)
    with ctx.pose({hood_joint: 0.90, gate_joint: 1.20}):
        hood_open = ctx.part_world_aabb(hood)
        gate_open = ctx.part_world_aabb(gate)

    ctx.check(
        "hood opens upward",
        hood_closed is not None
        and hood_open is not None
        and hood_open[1][2] > hood_closed[1][2] + 0.08,
        details=f"closed={hood_closed}, open={hood_open}",
    )
    ctx.check(
        "rear gate folds downward and rearward",
        gate_closed is not None
        and gate_open is not None
        and gate_open[1][2] < gate_closed[1][2] - 0.03
        and gate_open[0][0] < gate_closed[0][0] - 0.05,
        details=f"closed={gate_closed}, open={gate_open}",
    )

    return ctx.report()


object_model = build_object_model()
