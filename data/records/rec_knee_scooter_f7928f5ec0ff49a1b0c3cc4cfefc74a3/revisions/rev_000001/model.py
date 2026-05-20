from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _hollow_tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _knee_pad_mesh():
    profile = rounded_rect_profile(0.46, 0.24, 0.055, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, 0.070, center=True), "rounded_knee_pad")


def _small_tire_mesh(name: str):
    tire = TireGeometry(
        0.075,
        0.044,
        inner_radius=0.056,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.005, radius=0.0025),
    )
    return mesh_from_geometry(tire, name)


def _small_rim_mesh(name: str):
    wheel = WheelGeometry(
        0.057,
        0.038,
        rim=WheelRim(
            inner_radius=0.038,
            flange_height=0.004,
            flange_thickness=0.0025,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.020,
            width=0.032,
            cap_style="flat",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=0.026,
                hole_diameter=0.003,
            ),
        ),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.018),
    )
    return mesh_from_geometry(wheel, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    blue = model.material("powder_coated_blue", rgba=(0.08, 0.24, 0.55, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark = model.material("dark_frame_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    silver = model.material("brushed_aluminum", rgba=(0.74, 0.73, 0.68, 1.0))
    pad_mat = model.material("soft_vinyl_pad", rgba=(0.02, 0.025, 0.03, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.72, 0.060, 0.035)),
        origin=Origin(xyz=(-0.010, 0.0, 0.180)),
        material=blue,
        name="lower_spine",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.31),
        origin=Origin(xyz=(-0.325, 0.0, 0.166), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="rear_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.34),
        origin=Origin(xyz=(-0.325, 0.0, 0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle",
    )
    for idx, y in enumerate((-0.155, 0.155)):
        frame.visual(
            Cylinder(radius=0.010, length=0.090),
            origin=Origin(xyz=(-0.325, y, 0.120)),
            material=blue,
            name=f"rear_dropout_{idx}",
        )
    frame.visual(
        Box((0.148, 0.055, 0.038)),
        origin=Origin(xyz=(0.300, 0.0, 0.215)),
        material=blue,
        name="front_neck",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        frame.visual(
            Box((0.052, 0.016, 0.200)),
            origin=Origin(xyz=(0.398, y, 0.330)),
            material=dark,
            name=f"head_tube_cheek_{idx}",
        )
    frame.visual(
        Cylinder(radius=0.024, length=0.260),
        origin=Origin(xyz=(-0.125, 0.0, 0.320)),
        material=blue,
        name="knee_post",
    )
    frame.visual(
        Box((0.42, 0.20, 0.014)),
        origin=Origin(xyz=(-0.125, 0.0, 0.450)),
        material=dark,
        name="pad_support_plate",
    )
    frame.visual(
        _knee_pad_mesh(),
        origin=Origin(xyz=(-0.125, 0.0, 0.492)),
        material=pad_mat,
        name="knee_pad",
    )

    front_steerer = model.part("front_steerer")
    front_steerer.visual(
        Cylinder(radius=0.022, length=0.970),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=silver,
        name="steerer_tube",
    )
    front_steerer.visual(
        Box((0.072, 0.340, 0.030)),
        origin=Origin(xyz=(0.025, 0.0, -0.145)),
        material=blue,
        name="fork_crown",
    )
    for idx, y in enumerate((-0.155, 0.155)):
        front_steerer.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(0.025, y, -0.207)),
            material=blue,
            name=f"fork_leg_{idx}",
        )
    front_steerer.visual(
        Cylinder(radius=0.009, length=0.335),
        origin=Origin(xyz=(0.025, 0.0, -0.255), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_axle",
    )
    front_steerer.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(xyz=(-0.085, 0.0, 0.635), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="handlebar_stem",
    )
    front_steerer.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(-0.170, 0.0, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handlebar_bar",
    )
    for idx, y in enumerate((-0.265, 0.265)):
        front_steerer.visual(
            Cylinder(radius=0.018, length=0.105),
            origin=Origin(xyz=(-0.170, y, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"rubber_grip_{idx}",
        )

    steer_joint = model.articulation(
        "head_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_steerer,
        origin=Origin(xyz=(0.400, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-0.78, upper=0.78),
    )

    tire_mesh = _small_tire_mesh("small_block_tire")
    rim_mesh = _small_rim_mesh("small_silver_wheel")

    wheel_specs = (
        ("front_wheel_0", front_steerer, (0.025, -0.095, -0.255)),
        ("front_wheel_1", front_steerer, (0.025, 0.095, -0.255)),
        ("rear_wheel_0", frame, (-0.325, -0.115, 0.075)),
        ("rear_wheel_1", frame, (-0.325, 0.115, 0.075)),
    )
    for wheel_name, parent, xyz in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=silver,
            name="rim",
        )
        model.articulation(
            f"{wheel_name}_roll",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=18.0),
        )

    # Keep a stable reference name in model metadata for downstream inspection.
    model.meta["primary_steering_joint"] = steer_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    steer = object_model.get_articulation("head_yaw")
    front_0 = object_model.get_part("front_wheel_0")
    front_1 = object_model.get_part("front_wheel_1")
    rear_0 = object_model.get_part("rear_wheel_0")
    rear_1 = object_model.get_part("rear_wheel_1")

    roll_names = (
        "front_wheel_0_roll",
        "front_wheel_1_roll",
        "rear_wheel_0_roll",
        "rear_wheel_1_roll",
    )
    ctx.check(
        "four independent wheel roll joints",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in roll_names
        ),
        details="Each tire should be its own continuous axle joint.",
    )
    ctx.check(
        "steering yaw is limited",
        steer.motion_limits is not None
        and steer.motion_limits.lower < -0.5
        and steer.motion_limits.upper > 0.5,
        details=f"limits={steer.motion_limits}",
    )

    axle_interfaces = (
        ("frame", "rear_wheel_0", "rear_axle"),
        ("frame", "rear_wheel_1", "rear_axle"),
        ("front_steerer", "front_wheel_0", "front_axle"),
        ("front_steerer", "front_wheel_1", "front_axle"),
    )
    for parent_name, wheel_name, axle_elem in axle_interfaces:
        ctx.allow_overlap(
            parent_name,
            wheel_name,
            elem_a=axle_elem,
            elem_b="rim",
            reason=(
                "The visible metal axle is intentionally captured through the wheel hub bore "
                "so the wheel has a real rotating support."
            ),
        )
        ctx.expect_within(
            parent_name,
            wheel_name,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="rim",
            margin=0.001,
            name=f"{wheel_name} axle lies inside hub bore projection",
        )
        ctx.expect_overlap(
            wheel_name,
            parent_name,
            axes="y",
            elem_a="rim",
            elem_b=axle_elem,
            min_overlap=0.030,
            name=f"{wheel_name} rim remains on captured axle",
        )

    ctx.expect_origin_gap(
        front_1,
        front_0,
        axis="y",
        min_gap=0.18,
        max_gap=0.21,
        name="front wheels are a laterally spaced pair",
    )
    ctx.expect_origin_gap(
        rear_1,
        rear_0,
        axis="y",
        min_gap=0.22,
        max_gap=0.24,
        name="rear wheels are a laterally spaced pair",
    )

    rest_front = ctx.part_world_position(front_1)
    with ctx.pose({steer: 0.55}):
        turned_front = ctx.part_world_position(front_1)
    ctx.check(
        "front assembly yaws around head tube",
        rest_front is not None
        and turned_front is not None
        and abs(turned_front[0] - rest_front[0]) > 0.015,
        details=f"rest={rest_front}, turned={turned_front}",
    )

    return ctx.report()


object_model = build_object_model()
