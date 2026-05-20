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
    TireCarcass,
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
    model = ArticulatedObject(name="stair_climbing_hand_truck")

    blue = Material("powder_coated_blue", color=(0.05, 0.19, 0.58, 1.0))
    steel = Material("brushed_steel", color=(0.63, 0.66, 0.68, 1.0))
    dark_steel = Material("dark_bearing_steel", color=(0.20, 0.22, 0.24, 1.0))
    rubber = Material("matte_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    plate_mat = Material("scuffed_toe_plate", color=(0.38, 0.40, 0.42, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.058,
            0.050,
            rim=WheelRim(
                inner_radius=0.036,
                flange_height=0.004,
                flange_thickness=0.0025,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.020,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=4,
                    circle_diameter=0.030,
                    hole_diameter=0.0035,
                ),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=4, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "small_aluminum_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.084,
            0.056,
            inner_radius=0.059,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "small_block_tire",
    )

    frame = model.part("frame")

    rail_len = math.hypot(0.20, 1.10)
    rail_angle = math.atan2(-0.20, 1.10)
    for index, y in enumerate((-0.19, 0.19)):
        frame.visual(
            Box((0.038, 0.036, rail_len)),
            origin=Origin(xyz=(-0.10, y, 0.70), rpy=(0.0, rail_angle, 0.0)),
            material=blue,
            name=f"side_rail_{index}",
        )

    for name, z in (("lower_crossbar", 0.165), ("middle_crossbar", 0.58), ("upper_crossbar", 1.02)):
        x = (z - 0.15) * (-0.20 / 1.10)
        frame.visual(
            Box((0.052, 0.47, 0.038)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=blue,
            name=name,
        )

    frame.visual(
        Box((0.050, 0.56, 0.040)),
        origin=Origin(xyz=(-0.205, 0.0, 1.255)),
        material=blue,
        name="top_handle_bar",
    )
    frame.visual(
        Cylinder(radius=0.027, length=0.50),
        origin=Origin(xyz=(-0.205, 0.0, 1.260), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_grip",
    )

    frame.visual(
        Box((0.350, 0.370, 0.030)),
        origin=Origin(xyz=(0.190, 0.0, 0.020)),
        material=plate_mat,
        name="toe_plate",
    )
    frame.visual(
        Box((0.045, 0.425, 0.150)),
        origin=Origin(xyz=(0.000, 0.0, 0.085)),
        material=plate_mat,
        name="toe_plate_lip",
    )
    for index, y in enumerate((-0.155, 0.155)):
        frame.visual(
            Box((0.280, 0.035, 0.032)),
            origin=Origin(xyz=(0.070, y, 0.083), rpy=(0.0, -0.55, 0.0)),
            material=blue,
            name=f"toe_brace_{index}",
        )

    hub_z = 0.220
    hub_y = 0.290
    frame_stub_len = 0.0975
    for index, y in enumerate((-0.22375, 0.22375)):
        frame.visual(
            Cylinder(radius=0.027, length=frame_stub_len),
            origin=Origin(xyz=(0.0, y, hub_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bogie_stub_{index}",
        )
        frame.visual(
            Box((0.055, 0.065, 0.100)),
            origin=Origin(xyz=(0.0, -0.19 if y < 0 else 0.19, hub_z - 0.005)),
            material=blue,
            name=f"stub_boss_{index}",
        )

    cluster_radius = 0.135
    wheel_radius = 0.084
    wheel_width = 0.056
    wheel_angles = (-math.pi / 2.0, math.radians(30.0), math.radians(150.0))
    wheel_offsets = [
        (cluster_radius * math.cos(angle), cluster_radius * math.sin(angle))
        for angle in wheel_angles
    ]

    for side_index, side_sign in enumerate((-1.0, 1.0)):
        bogie = model.part(f"bogie_{side_index}")
        wheel_y = side_sign * 0.055

        bogie.visual(
            Cylinder(radius=0.038, length=0.035),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="central_hub",
        )
        bogie.visual(
            Cylinder(radius=0.052, length=0.010),
            origin=Origin(xyz=(0.0, side_sign * 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hub_cover",
        )

        for wheel_index, (x, z) in enumerate(wheel_offsets):
            arm_angle = math.atan2(x, z)
            bogie.visual(
                Box((0.030, 0.024, cluster_radius + 0.020)),
                origin=Origin(xyz=(x / 2.0, 0.0, z / 2.0), rpy=(0.0, arm_angle, 0.0)),
                material=blue,
                name=f"spoke_{wheel_index}",
            )
            bogie.visual(
                Cylinder(radius=0.027, length=0.030),
                origin=Origin(xyz=(x, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"axle_boss_{wheel_index}",
            )

            pin_inner = -side_sign * 0.010
            pin_outer = side_sign * (abs(wheel_y) - 0.033)
            pin_center = (pin_inner + pin_outer) / 2.0
            pin_len = abs(pin_outer - pin_inner)
            bogie.visual(
                Cylinder(radius=0.011, length=pin_len),
                origin=Origin(xyz=(x, pin_center, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"axle_pin_{wheel_index}",
            )

        model.articulation(
            f"frame_to_bogie_{side_index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=bogie,
            origin=Origin(xyz=(0.0, side_sign * hub_y, hub_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=5.0),
        )

        for wheel_index, (x, z) in enumerate(wheel_offsets):
            wheel = model.part(f"wheel_{side_index}_{wheel_index}")
            wheel.visual(
                tire_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=rubber,
                name="tire",
            )
            wheel.visual(
                wheel_mesh,
                origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
                material=steel,
                name="rim",
            )
            wheel.visual(
                Cylinder(radius=0.018, length=0.066),
                origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name="bearing_sleeve",
            )

            model.articulation(
                f"bogie_{side_index}_to_wheel_{wheel_index}",
                ArticulationType.CONTINUOUS,
                parent=bogie,
                child=wheel,
                origin=Origin(xyz=(x, wheel_y, z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=8.0, velocity=20.0),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    bogies = [object_model.get_part(f"bogie_{i}") for i in range(2)]
    wheels = [object_model.get_part(f"wheel_{side}_{i}") for side in range(2) for i in range(3)]

    ctx.check("one tri-wheel bogie per side", len(bogies) == 2, details=f"bogies={bogies}")
    ctx.check("six individually authored wheels", len(wheels) == 6, details=f"wheels={wheels}")

    toe_aabb = ctx.part_element_world_aabb(frame, elem="toe_plate")
    lip_aabb = ctx.part_element_world_aabb(frame, elem="toe_plate_lip")
    ctx.check(
        "toe plate projects forward from frame lip",
        toe_aabb is not None and lip_aabb is not None and toe_aabb[1][0] > lip_aabb[1][0] + 0.25,
        details=f"toe={toe_aabb}, lip={lip_aabb}",
    )

    for side in range(2):
        bogie_joint = object_model.get_articulation(f"frame_to_bogie_{side}")
        ctx.check(
            f"bogie_{side} rotates about central hub",
            bogie_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in bogie_joint.axis) == (0.0, 1.0, 0.0),
            details=f"type={bogie_joint.articulation_type}, axis={bogie_joint.axis}",
        )
        ctx.expect_contact(
            frame,
            object_model.get_part(f"bogie_{side}"),
            elem_a=f"bogie_stub_{side}",
            elem_b="central_hub",
            contact_tol=0.001,
            name=f"bogie_{side} hub is carried on frame stub",
        )

        before = ctx.part_world_position(object_model.get_part(f"wheel_{side}_0"))
        with ctx.pose({bogie_joint: math.radians(60.0)}):
            after = ctx.part_world_position(object_model.get_part(f"wheel_{side}_0"))
        ctx.check(
            f"bogie_{side} motion carries its wheel cluster",
            before is not None
            and after is not None
            and abs(after[0] - before[0]) + abs(after[2] - before[2]) > 0.05,
            details=f"before={before}, after={after}",
        )

        for wheel_index in range(3):
            wheel_joint = object_model.get_articulation(f"bogie_{side}_to_wheel_{wheel_index}")
            ctx.check(
                f"wheel_{side}_{wheel_index} has spin axle",
                wheel_joint.articulation_type == ArticulationType.CONTINUOUS
                and tuple(round(v, 6) for v in wheel_joint.axis) == (0.0, 1.0, 0.0),
                details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
            )

    return ctx.report()


object_model = build_object_model()
