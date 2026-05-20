from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
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
    rounded_rect_profile,
    tube_from_spline_points,
)


def _tube_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must differ")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
) -> None:
    origin, length = _tube_origin_between(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    satin_black = Material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    tube_blue = Material("anodized_blue_frame", rgba=(0.05, 0.20, 0.62, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    soft_pad = Material("charcoal_vinyl_pad", rgba=(0.035, 0.038, 0.042, 1.0))
    pad_stitch = Material("slightly_worn_pad_insert", rgba=(0.075, 0.080, 0.088, 1.0))
    brushed_metal = Material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    hardware = Material("black_hardware", rgba=(0.015, 0.014, 0.013, 1.0))
    reflector_red = Material("red_reflector", rgba=(0.9, 0.05, 0.035, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.095,
            0.046,
            inner_radius=0.066,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.004, count=24, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.003),
        ),
        "knee_scooter_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.068,
            0.050,
            rim=WheelRim(
                inner_radius=0.043,
                flange_height=0.005,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.024,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.032, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.009),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "knee_scooter_rim",
    )
    knee_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.44, 0.245, 0.055, corner_segments=10),
            0.070,
        ),
        "rounded_knee_pad",
    )
    knee_insert_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.315, 0.120, 0.030, corner_segments=8),
            0.008,
        ),
        "knee_pad_recess_insert",
    )
    lever_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (-0.026, 0.0, -0.040), (-0.070, 0.0, -0.077), (-0.105, 0.0, -0.092)],
            radius=0.0065,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
        "curved_brake_lever",
    )
    cable_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [(0.0, 0.0, 0.0), (-0.050, 0.0, -0.055), (-0.040, 0.0, -0.300), (0.028, 0.0, -0.660)],
            radius=0.0038,
            samples_per_segment=12,
            radial_segments=12,
            cap_ends=True,
        ),
        "brake_cable_housing",
    )
    clamp_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.026,
            body_style="lobed",
            base_diameter=0.035,
            top_diameter=0.050,
            crown_radius=0.0015,
            center=False,
        ),
        "lobed_height_clamp_knob",
    )

    frame = model.part("frame")
    _add_tube(frame, "main_spine", (-0.56, 0.0, 0.205), (0.470, 0.0, 0.205), 0.024, tube_blue)
    _add_tube(frame, "side_rail_0", (-0.470, 0.145, 0.225), (0.345, 0.145, 0.225), 0.018, tube_blue)
    _add_tube(frame, "side_rail_1", (-0.470, -0.145, 0.225), (0.345, -0.145, 0.225), 0.018, tube_blue)
    _add_tube(frame, "front_crossbar", (0.345, -0.190, 0.225), (0.345, 0.190, 0.225), 0.018, tube_blue)
    _add_tube(frame, "rear_crossbar", (-0.470, -0.190, 0.225), (-0.470, 0.190, 0.225), 0.018, tube_blue)
    _add_tube(frame, "rear_axle", (-0.485, -0.305, 0.105), (-0.485, 0.305, 0.105), 0.012, brushed_metal)
    _add_tube(frame, "rear_strut_0", (-0.485, 0.178, 0.105), (-0.385, 0.145, 0.225), 0.014, tube_blue)
    _add_tube(frame, "rear_strut_1", (-0.485, -0.178, 0.105), (-0.385, -0.145, 0.225), 0.014, tube_blue)
    _add_tube(frame, "head_brace_0", (0.345, 0.070, 0.225), (0.520, 0.045, 0.335), 0.017, tube_blue)
    _add_tube(frame, "head_brace_1", (0.345, -0.070, 0.225), (0.520, -0.045, 0.335), 0.017, tube_blue)
    _add_tube(frame, "knee_post", (-0.195, 0.0, 0.215), (-0.195, 0.0, 0.505), 0.026, tube_blue)
    frame.visual(
        Cylinder(radius=0.036, length=0.220),
        origin=Origin(xyz=(0.520, 0.0, 0.260)),
        material=brushed_metal,
        name="head_tube",
    )
    frame.visual(
        Box((0.285, 0.175, 0.030)),
        origin=Origin(xyz=(-0.195, 0.0, 0.515)),
        material=brushed_metal,
        name="pad_support_plate",
    )
    frame.visual(knee_pad_mesh, origin=Origin(xyz=(-0.230, 0.0, 0.555)), material=soft_pad, name="knee_pad")
    frame.visual(
        knee_insert_mesh,
        origin=Origin(xyz=(-0.230, 0.0, 0.590)),
        material=pad_stitch,
        name="knee_pad_recess",
    )
    _add_tube(frame, "pad_bolster_0", (-0.410, 0.095, 0.595), (-0.050, 0.095, 0.595), 0.022, soft_pad)
    _add_tube(frame, "pad_bolster_1", (-0.410, -0.095, 0.595), (-0.050, -0.095, 0.595), 0.022, soft_pad)
    frame.visual(
        Box((0.026, 0.120, 0.030)),
        origin=Origin(xyz=(-0.494, 0.0, 0.235)),
        material=reflector_red,
        name="rear_reflector",
    )

    steering = model.part("steering_column")
    _add_tube(steering, "steering_shaft", (0.0, 0.0, -0.120), (0.0, 0.0, 0.840), 0.021, brushed_metal)
    _add_tube(steering, "handlebar", (0.0, -0.300, 0.840), (0.0, 0.300, 0.840), 0.018, satin_black)
    _add_tube(steering, "grip_0", (0.0, 0.300, 0.840), (0.0, 0.405, 0.840), 0.023, dark_rubber)
    _add_tube(steering, "grip_1", (0.0, -0.300, 0.840), (0.0, -0.405, 0.840), 0.023, dark_rubber)
    steering.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=hardware,
        name="height_collar",
    )
    steering.visual(
        Cylinder(radius=0.040, length=0.038),
        origin=Origin(xyz=(0.010, 0.0, -0.102)),
        material=satin_black,
        name="fork_crown",
    )
    _add_tube(steering, "front_axle", (0.045, -0.270, -0.125), (0.045, 0.270, -0.125), 0.012, brushed_metal)
    _add_tube(steering, "fork_leg_0", (0.004, 0.060, -0.010), (0.045, 0.150, -0.125), 0.014, satin_black)
    _add_tube(steering, "fork_leg_1", (0.004, -0.060, -0.010), (0.045, -0.150, -0.125), 0.014, satin_black)
    _add_tube(steering, "fork_bridge", (0.010, -0.095, -0.100), (0.010, 0.095, -0.100), 0.014, satin_black)
    steering.visual(cable_mesh, origin=Origin(xyz=(0.0, 0.270, 0.823)), material=dark_rubber, name="brake_cable_0")
    steering.visual(cable_mesh, origin=Origin(xyz=(0.0, -0.270, 0.823)), material=dark_rubber, name="brake_cable_1")

    model.articulation(
        "frame_to_steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(xyz=(0.520, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.75, upper=0.75),
    )

    for index, y in enumerate((0.240, -0.240)):
        wheel = model.part(f"rear_wheel_{index}")
        wheel.visual(tire_mesh, material=dark_rubber, name="tire")
        wheel.visual(rim_mesh, material=brushed_metal, name="rim")
        model.articulation(
            f"rear_wheel_{index}_roll",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.485, y, 0.105), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    for index, y in enumerate((0.205, -0.205)):
        wheel = model.part(f"front_wheel_{index}")
        wheel.visual(tire_mesh, material=dark_rubber, name="tire")
        wheel.visual(rim_mesh, material=brushed_metal, name="rim")
        model.articulation(
            f"front_wheel_{index}_roll",
            ArticulationType.CONTINUOUS,
            parent=steering,
            child=wheel,
            origin=Origin(xyz=(0.045, y, -0.125), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=40.0),
        )

    for index, y in enumerate((0.230, -0.230)):
        lever = model.part(f"brake_lever_{index}")
        lever.visual(
            Cylinder(radius=0.014, length=0.036),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="pivot_boss",
        )
        lever.visual(lever_mesh, material=hardware, name="lever_blade")
        model.articulation(
            f"brake_lever_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=steering,
            child=lever,
            origin=Origin(xyz=(-0.004, y, 0.817)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=0.55),
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(clamp_knob_mesh, material=hardware, name="knob_cap")
    clamp_knob.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=brushed_metal,
        name="threaded_stud",
    )
    model.articulation(
        "collar_to_knob",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.039, 0.405), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    steering = object_model.get_part("steering_column")
    steer_joint = object_model.get_articulation("frame_to_steering")

    ctx.allow_overlap(
        frame,
        steering,
        elem_a="head_tube",
        elem_b="steering_shaft",
        reason="The steering shaft is intentionally captured through the head-tube bearing sleeve.",
    )
    ctx.expect_within(
        steering,
        frame,
        axes="xy",
        inner_elem="steering_shaft",
        outer_elem="head_tube",
        margin=0.001,
        name="steering shaft is centered in the head tube",
    )
    ctx.expect_overlap(
        steering,
        frame,
        axes="z",
        elem_a="steering_shaft",
        elem_b="head_tube",
        min_overlap=0.18,
        name="steering shaft remains captured through the bearing sleeve",
    )

    for index in (0, 1):
        rear_wheel = object_model.get_part(f"rear_wheel_{index}")
        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The rear axle is intentionally seated through the wheel hub/bearing proxy.",
        )
        ctx.expect_overlap(
            rear_wheel,
            frame,
            axes="xyz",
            elem_a="rim",
            elem_b="rear_axle",
            min_overlap=0.018,
            name=f"rear wheel {index} is retained on the axle",
        )

        front_wheel = object_model.get_part(f"front_wheel_{index}")
        ctx.allow_overlap(
            steering,
            front_wheel,
            elem_a="front_axle",
            elem_b="rim",
            reason="The front axle is intentionally seated through the wheel hub/bearing proxy.",
        )
        ctx.expect_overlap(
            front_wheel,
            steering,
            axes="xyz",
            elem_a="rim",
            elem_b="front_axle",
            min_overlap=0.018,
            name=f"front wheel {index} is retained on the steering axle",
        )

        lever = object_model.get_part(f"brake_lever_{index}")
        ctx.allow_overlap(
            steering,
            lever,
            elem_a="handlebar",
            elem_b="pivot_boss",
            reason="The brake-lever pivot boss wraps around the handlebar as a clamp-mounted control.",
        )
        ctx.expect_overlap(
            lever,
            steering,
            axes="xyz",
            elem_a="pivot_boss",
            elem_b="handlebar",
            min_overlap=0.006,
            name=f"brake lever {index} clamp is mounted to the handlebar",
        )

    clamp_knob = object_model.get_part("clamp_knob")
    ctx.allow_overlap(
        steering,
        clamp_knob,
        elem_a="height_collar",
        elem_b="threaded_stud",
        reason="The height-adjustment knob's threaded stud enters the collar socket.",
    )
    ctx.allow_overlap(
        steering,
        clamp_knob,
        elem_a="steering_shaft",
        elem_b="threaded_stud",
        reason="The clamp stud intentionally presses into the telescoping steering shaft to lock height.",
    )
    ctx.expect_overlap(
        clamp_knob,
        steering,
        axes="xyz",
        elem_a="threaded_stud",
        elem_b="height_collar",
        min_overlap=0.006,
        name="height clamp knob stud engages the collar",
    )
    ctx.expect_overlap(
        clamp_knob,
        steering,
        axes="xyz",
        elem_a="threaded_stud",
        elem_b="steering_shaft",
        min_overlap=0.006,
        name="height clamp stud reaches the steering shaft",
    )

    roll_joints = [j for j in object_model.articulations if j.name.endswith("_roll")]
    ctx.check(
        "four independent rolling wheels",
        len(roll_joints) == 4 and all(j.articulation_type == ArticulationType.CONTINUOUS for j in roll_joints),
        details=f"rolling joints={[j.name for j in roll_joints]}",
    )

    rest_front = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    with ctx.pose({steer_joint: 0.55}):
        turned_front = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    ctx.check(
        "steering yaw swings the front wheel assembly",
        rest_front is not None
        and turned_front is not None
        and abs(turned_front[0] - rest_front[0]) > 0.040,
        details=f"rest={rest_front}, turned={turned_front}",
    )

    lever_0 = object_model.get_part("brake_lever_0")
    lever_joint_0 = object_model.get_articulation("brake_lever_0_pivot")
    lever_rest_box = ctx.part_world_aabb(lever_0)
    with ctx.pose({lever_joint_0: 0.55}):
        lever_squeezed_box = ctx.part_world_aabb(lever_0)
    ctx.check(
        "brake lever squeezes upward toward the grip",
        lever_rest_box is not None
        and lever_squeezed_box is not None
        and lever_squeezed_box[1][2] > lever_rest_box[1][2] + 0.035,
        details=f"rest_aabb={lever_rest_box}, squeezed_aabb={lever_squeezed_box}",
    )

    wheel_boxes = [ctx.part_world_aabb(object_model.get_part(name)) for name in (
        "rear_wheel_0",
        "rear_wheel_1",
        "front_wheel_0",
        "front_wheel_1",
    )]
    ctx.check(
        "all four tires sit at the same floor height",
        all(box is not None and -0.002 <= box[0][2] <= 0.015 for box in wheel_boxes),
        details=f"wheel_aabbs={wheel_boxes}",
    )

    return ctx.report()


object_model = build_object_model()
