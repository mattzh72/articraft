from __future__ import annotations

import math

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
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
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
    model = ArticulatedObject(name="toy_delivery_van")

    yellow = model.material("warm_yellow_plastic", color=(1.0, 0.73, 0.12, 1.0))
    darker_yellow = model.material("darker_yellow_panel", color=(0.92, 0.55, 0.05, 1.0))
    black = model.material("soft_black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    dark_blue = model.material("smoky_blue_window", color=(0.05, 0.16, 0.28, 1.0))
    grey = model.material("matte_grey_plastic", color=(0.58, 0.58, 0.55, 1.0))
    silver = model.material("brushed_axle_metal", color=(0.78, 0.76, 0.70, 1.0))
    red = model.material("red_tail_lens", color=(0.8, 0.02, 0.02, 1.0))
    amber = model.material("amber_lens", color=(1.0, 0.55, 0.05, 1.0))

    body = model.part("body")

    # One-piece toy base with axles, a tall cargo shell, and a short front nose.
    body.visual(Box((0.42, 0.155, 0.022)), origin=Origin(xyz=(0.015, 0.0, 0.040)), material=yellow, name="base_plate")
    body.visual(Box((0.235, 0.162, 0.022)), origin=Origin(xyz=(-0.075, 0.0, 0.071)), material=yellow, name="cargo_floor")
    body.visual(Box((0.240, 0.162, 0.024)), origin=Origin(xyz=(-0.075, 0.0, 0.184)), material=yellow, name="cargo_roof")
    body.visual(Box((0.240, 0.007, 0.116)), origin=Origin(xyz=(-0.075, -0.0815, 0.126)), material=yellow, name="plain_side_wall")

    # Framed cargo opening on the articulated side, so the moving door reads as
    # covering a real hole rather than painted-on trim.
    body.visual(Box((0.104, 0.007, 0.022)), origin=Origin(xyz=(-0.055, 0.0815, 0.083)), material=yellow, name="side_door_sill")
    body.visual(Box((0.104, 0.007, 0.024)), origin=Origin(xyz=(-0.055, 0.0815, 0.173)), material=yellow, name="side_door_header")
    body.visual(Box((0.012, 0.009, 0.112)), origin=Origin(xyz=(-0.002, 0.0825, 0.120)), material=yellow, name="side_hinge_pillar")
    body.visual(Box((0.012, 0.007, 0.102)), origin=Origin(xyz=(-0.108, 0.0815, 0.128)), material=yellow, name="side_rear_jamb")

    # Rear cargo opening frame for the upward hatch.
    body.visual(Box((0.012, 0.158, 0.020)), origin=Origin(xyz=(-0.196, 0.0, 0.070)), material=yellow, name="rear_sill")
    body.visual(Box((0.012, 0.158, 0.022)), origin=Origin(xyz=(-0.196, 0.0, 0.181)), material=yellow, name="rear_header")
    body.visual(Box((0.012, 0.014, 0.110)), origin=Origin(xyz=(-0.196, -0.073, 0.123)), material=yellow, name="rear_jamb_0")
    body.visual(Box((0.012, 0.014, 0.110)), origin=Origin(xyz=(-0.196, 0.073, 0.123)), material=yellow, name="rear_jamb_1")

    # Cab and short nose.
    body.visual(Box((0.138, 0.154, 0.118)), origin=Origin(xyz=(0.075, 0.0, 0.118)), material=yellow, name="cab_box")
    body.visual(Box((0.118, 0.146, 0.060)), origin=Origin(xyz=(0.183, 0.0, 0.079)), material=yellow, name="short_nose")
    body.visual(Box((0.074, 0.003, 0.041)), origin=Origin(xyz=(0.103, 0.0785, 0.142)), material=dark_blue, name="side_window")
    body.visual(Box((0.004, 0.100, 0.045)), origin=Origin(xyz=(0.145, 0.0, 0.145)), material=dark_blue, name="windshield")
    body.visual(Box((0.069, 0.003, 0.082)), origin=Origin(xyz=(0.087, 0.0795, 0.113)), material=darker_yellow, name="cab_side_door")
    body.visual(Box((0.019, 0.004, 0.007)), origin=Origin(xyz=(0.062, 0.083, 0.112)), material=black, name="cab_handle")

    # Toy bumpers, lamps, and axle rods.
    body.visual(Box((0.012, 0.154, 0.023)), origin=Origin(xyz=(0.246, 0.0, 0.060)), material=grey, name="front_bumper")
    body.visual(Box((0.012, 0.154, 0.023)), origin=Origin(xyz=(-0.205, 0.0, 0.050)), material=grey, name="rear_bumper")
    body.visual(Box((0.006, 0.026, 0.012)), origin=Origin(xyz=(0.241, -0.047, 0.082)), material=amber, name="headlamp_0")
    body.visual(Box((0.006, 0.026, 0.012)), origin=Origin(xyz=(0.241, 0.047, 0.082)), material=amber, name="headlamp_1")
    body.visual(Box((0.006, 0.010, 0.018)), origin=Origin(xyz=(-0.202, -0.084, 0.115)), material=red, name="tail_lamp_0")
    body.visual(Box((0.006, 0.010, 0.018)), origin=Origin(xyz=(-0.202, 0.084, 0.115)), material=red, name="tail_lamp_1")
    axle_origin = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    body.visual(Cylinder(radius=0.0024, length=0.212), origin=Origin(xyz=(0.137, 0.0, 0.043), rpy=axle_origin.rpy), material=silver, name="front_axle")
    body.visual(Cylinder(radius=0.0024, length=0.212), origin=Origin(xyz=(-0.139, 0.0, 0.043), rpy=axle_origin.rpy), material=silver, name="rear_axle")

    # Mesh-backed wheels use realistic tire and rim profiles but remain small toy scale.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.037,
            0.024,
            inner_radius=0.026,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="circumferential", depth=0.0022, count=3, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            shoulder=TireShoulder(width=0.0035, radius=0.002),
        ),
        "toy_van_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.026,
            0.026,
            rim=WheelRim(inner_radius=0.017, flange_height=0.0025, flange_thickness=0.0015),
            hub=WheelHub(
                radius=0.008,
                width=0.018,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.012, hole_diameter=0.0018),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.0015, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0018, window_radius=0.0045),
            bore=WheelBore(style="round", diameter=0.004),
        ),
        "toy_van_rim",
    )

    wheel_positions = {
        "front_wheel_0": (0.137, -0.104, 0.043),
        "front_wheel_1": (0.137, 0.104, 0.043),
        "rear_wheel_0": (-0.139, -0.104, 0.043),
        "rear_wheel_1": (-0.139, 0.104, 0.043),
    }
    wheel_visual_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    for wheel_name, xyz in wheel_positions.items():
        wheel = model.part(wheel_name)
        wheel.visual(tire_mesh, origin=wheel_visual_origin, material=black, name="tire")
        wheel.visual(rim_mesh, origin=wheel_visual_origin, material=grey, name="rim")
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=18.0),
        )

    side_door = model.part("side_cargo_door")
    side_door.visual(Box((0.096, 0.006, 0.090)), origin=Origin(xyz=(-0.048, 0.004, 0.0)), material=darker_yellow, name="door_panel")
    side_door.visual(Cylinder(radius=0.0035, length=0.100), origin=Origin(xyz=(0.0, 0.005, 0.0)), material=silver, name="hinge_barrel")
    side_door.visual(Box((0.020, 0.004, 0.008)), origin=Origin(xyz=(-0.073, 0.009, 0.004)), material=black, name="door_handle")
    model.articulation(
        "body_to_side_cargo_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_door,
        origin=Origin(xyz=(-0.004, 0.086, 0.128)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.65),
    )

    rear_hatch = model.part("rear_hatch")
    rear_hatch.visual(Box((0.010, 0.148, 0.118)), origin=Origin(xyz=(-0.006, 0.0, -0.064)), material=darker_yellow, name="hatch_panel")
    rear_hatch.visual(Box((0.003, 0.096, 0.034)), origin=Origin(xyz=(-0.012, 0.0, -0.042)), material=dark_blue, name="hatch_window")
    rear_hatch.visual(Box((0.004, 0.036, 0.007)), origin=Origin(xyz=(-0.012, 0.0, -0.096)), material=black, name="hatch_handle")
    rear_hatch.visual(Cylinder(radius=0.003, length=0.136), origin=Origin(xyz=(-0.010, 0.0, -0.003), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=silver, name="top_hinge_barrel")
    model.articulation(
        "body_to_rear_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_hatch,
        origin=Origin(xyz=(-0.198, 0.0, 0.187)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    side_door = object_model.get_part("side_cargo_door")
    rear_hatch = object_model.get_part("rear_hatch")
    side_joint = object_model.get_articulation("body_to_side_cargo_door")
    hatch_joint = object_model.get_articulation("body_to_rear_hatch")
    wheel_joints = [
        object_model.get_articulation("body_to_front_wheel_0"),
        object_model.get_articulation("body_to_front_wheel_1"),
        object_model.get_articulation("body_to_rear_wheel_0"),
        object_model.get_articulation("body_to_rear_wheel_1"),
    ]

    ctx.check(
        "four wheels use continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=", ".join(f"{j.name}:{j.articulation_type.value}" for j in wheel_joints),
    )
    ctx.expect_gap(
        side_door,
        body,
        axis="y",
        max_gap=0.0025,
        max_penetration=0.0002,
        positive_elem="door_panel",
        negative_elem="side_hinge_pillar",
        name="side cargo door is seated just outside the side frame",
    )
    ctx.expect_gap(
        body,
        rear_hatch,
        axis="x",
        max_penetration=0.004,
        positive_elem="rear_header",
        negative_elem="hatch_panel",
        name="rear hatch closes against the rear cargo header",
    )

    closed_side_aabb = ctx.part_world_aabb(side_door)
    with ctx.pose({side_joint: 1.20}):
        open_side_aabb = ctx.part_world_aabb(side_door)
    ctx.check(
        "side cargo door swings outward",
        closed_side_aabb is not None
        and open_side_aabb is not None
        and open_side_aabb[1][1] > closed_side_aabb[1][1] + 0.045,
        details=f"closed={closed_side_aabb}, open={open_side_aabb}",
    )

    closed_hatch_aabb = ctx.part_world_aabb(rear_hatch)
    with ctx.pose({hatch_joint: 1.20}):
        open_hatch_aabb = ctx.part_world_aabb(rear_hatch)
    ctx.check(
        "rear hatch rotates upward from top hinge",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][2] > closed_hatch_aabb[0][2] + 0.035
        and open_hatch_aabb[0][0] < closed_hatch_aabb[0][0] - 0.030,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
