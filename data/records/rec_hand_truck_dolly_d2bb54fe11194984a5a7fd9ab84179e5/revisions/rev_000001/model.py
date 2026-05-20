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
    model = ArticulatedObject(name="convertible_hand_truck")

    red = model.material("red_powder_coat", rgba=(0.82, 0.05, 0.03, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    galvanized = model.material("galvanized_metal", rgba=(0.62, 0.66, 0.66, 1.0))

    frame = model.part("frame")

    # Primary load plate and its stamped anti-slip ribs.
    frame.visual(
        Box((0.49, 0.46, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, 0.060)),
        material=dark_steel,
        name="primary_plate",
    )
    for i, x in enumerate((-0.015, 0.075, 0.165, 0.255, 0.345)):
        frame.visual(
            Box((0.012, 0.39, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.078)),
            material=galvanized,
            name=f"plate_rib_{i}",
        )

    # Upright tubular frame, crossbars, and axle carrier.
    for y, name in ((-0.18, "rail_0"), (0.18, "rail_1")):
        frame.visual(
            Box((0.040, 0.040, 1.18)),
            origin=Origin(xyz=(-0.075, y, 0.660)),
            material=red,
            name=name,
        )
    frame.visual(
        Box((0.050, 0.50, 0.045)),
        origin=Origin(xyz=(-0.075, 0.0, 1.245)),
        material=red,
        name="handle_bar",
    )
    frame.visual(
        Box((0.038, 0.43, 0.035)),
        origin=Origin(xyz=(-0.075, 0.0, 0.820)),
        material=red,
        name="upper_crossbar",
    )
    frame.visual(
        Box((0.050, 0.43, 0.040)),
        origin=Origin(xyz=(-0.075, 0.0, 0.265)),
        material=red,
        name="lower_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.0125, length=0.70),
        origin=Origin(xyz=(-0.180, 0.0, 0.170), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="wheel_axle",
    )
    for y, name in ((-0.18, "axle_mount_0"), (0.18, "axle_mount_1")):
        frame.visual(
            Box((0.140, 0.050, 0.130)),
            origin=Origin(xyz=(-0.130, y, 0.210)),
            material=red,
            name=name,
        )
    for y, name in ((-0.18, "axle_strut_0"), (0.18, "axle_strut_1")):
        frame.visual(
            Box((0.260, 0.026, 0.028)),
            origin=Origin(xyz=(-0.060, y, 0.155), rpy=(0.0, -0.28, 0.0)),
            material=red,
            name=name,
        )

    # Side hinge ears and the transverse pin sit just under the front edge of
    # the main nose plate.  The fold-out toe plate rotates around this line.
    hinge_x = 0.405
    hinge_z = 0.036
    for y, name in ((-0.235, "hinge_ear_0"), (0.235, "hinge_ear_1")):
        frame.visual(
            Box((0.045, 0.035, 0.070)),
            origin=Origin(xyz=(hinge_x, y, hinge_z)),
            material=red,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.007, length=0.54),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.123,
            0.058,
            rim=WheelRim(inner_radius=0.082, flange_height=0.010, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.034,
                width=0.042,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.044, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.007, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.024),
        ),
        "hand_truck_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.160,
            0.074,
            inner_radius=0.112,
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.035),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "hand_truck_tire",
    )

    for y, name, joint_name in (
        (-0.315, "wheel_0", "frame_to_wheel_0"),
        (0.315, "wheel_1", "frame_to_wheel_1"),
    ):
        wheel = model.part(name)
        wheel.visual(wheel_mesh, material=galvanized, name="rim")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            # Wheel helpers spin around local X; rotate the joint frame so local
            # X coincides with the hand truck's transverse axle.
            origin=Origin(xyz=(-0.180, y, 0.170), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=20.0),
        )

    toe = model.part("toe_section")
    toe.visual(
        Cylinder(radius=0.015, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_tube",
    )
    toe.visual(
        Box((0.370, 0.400, 0.018)),
        origin=Origin(xyz=(-0.200, 0.0, -0.021)),
        material=dark_steel,
        name="toe_plate",
    )
    for i, x in enumerate((-0.315, -0.235, -0.155, -0.075)):
        toe.visual(
            Box((0.010, 0.335, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.032)),
            material=galvanized,
            name=f"toe_rib_{i}",
        )
    for y, name in ((-0.170, "toe_tab_0"), (0.170, "toe_tab_1")):
        toe.visual(
            Box((0.050, 0.035, 0.026)),
            origin=Origin(xyz=(-0.030, y, -0.010)),
            material=galvanized,
            name=name,
        )

    model.articulation(
        "frame_to_toe",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=toe,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=0.0, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    toe = object_model.get_part("toe_section")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    toe_hinge = object_model.get_articulation("frame_to_toe")

    ctx.allow_overlap(
        frame,
        toe,
        elem_a="hinge_pin",
        elem_b="hinge_tube",
        reason="The transverse hinge pin is intentionally captured inside the fold-out toe tube.",
    )
    ctx.expect_within(
        frame,
        toe,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_tube",
        margin=0.002,
        name="hinge pin centered in toe tube",
    )
    ctx.expect_overlap(
        frame,
        toe,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_tube",
        min_overlap=0.38,
        name="hinge pin spans toe tube",
    )

    ctx.expect_within(
        toe,
        frame,
        axes="xy",
        inner_elem="toe_plate",
        outer_elem="primary_plate",
        margin=0.004,
        name="folded toe section nests under main plate footprint",
    )
    ctx.expect_gap(
        frame,
        toe,
        axis="z",
        positive_elem="primary_plate",
        negative_elem="toe_plate",
        min_gap=0.010,
        max_gap=0.030,
        name="folded toe section sits below main plate",
    )

    with ctx.pose({toe_hinge: math.pi}):
        ctx.expect_gap(
            toe,
            frame,
            axis="x",
            positive_elem="toe_plate",
            negative_elem="primary_plate",
            min_gap=0.020,
            max_gap=0.070,
            name="deployed toe plate folds forward",
        )

    for wheel, label in ((wheel_0, "wheel_0"), (wheel_1, "wheel_1")):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="wheel_axle",
            elem_b="rim",
            reason="The steel axle is intentionally represented as a tight captured fit through the wheel hub bore.",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="rim",
            elem_b="wheel_axle",
            min_overlap=0.030,
            name=f"{label} carried on transverse axle",
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="wheel_axle",
            outer_elem="rim",
            margin=0.0,
            name=f"{label} hub surrounds axle line",
        )

    return ctx.report()


object_model = build_object_model()
