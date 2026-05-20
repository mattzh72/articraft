from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
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


def _cyl_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _cyl_z(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_knee_scooter")

    frame_mat = model.material("powder_coated_black", rgba=(0.015, 0.017, 0.018, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    axle_mat = model.material("dark_hardware", rgba=(0.05, 0.05, 0.055, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    pad_mat = model.material("blue_vinyl_pad", rgba=(0.02, 0.10, 0.28, 1.0))
    lever_mat = model.material("black_composite_lever", rgba=(0.01, 0.012, 0.013, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.056,
            0.034,
            rim=WheelRim(inner_radius=0.036, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.030,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "scooter_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.040,
            inner_radius=0.052,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.045),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "scooter_wheel_tire",
    )

    frame = model.part("central_frame")
    frame.visual(Cylinder(radius=0.024, length=0.68), origin=_cyl_x((0.03, 0.0, 0.280)), material=frame_mat, name="main_tube")
    frame.visual(Cylinder(radius=0.018, length=0.42), origin=_cyl_y((-0.30, 0.0, 0.130)), material=frame_mat, name="rear_cross_tube")
    frame.visual(Cylinder(radius=0.018, length=0.22), origin=_cyl_z((-0.30, 0.0, 0.190)), material=frame_mat, name="rear_down_post")
    frame.visual(Cylinder(radius=0.022, length=0.22), origin=_cyl_z((-0.08, 0.0, 0.390)), material=frame_mat, name="pad_post")
    frame.visual(Box((0.22, 0.055, 0.020)), origin=Origin(xyz=(-0.08, 0.0, 0.490)), material=metal_mat, name="pad_mount_plate")

    for index, y_sign in enumerate((1.0, -1.0)):
        y = y_sign * 0.210
        pin_y = y_sign * 0.2213
        frame.visual(
            Box((0.070, 0.030, 0.125)),
            origin=Origin(xyz=(-0.300, y, 0.095)),
            material=frame_mat,
            name=f"rear_dropout_{index}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=_cyl_y((-0.300, pin_y, 0.075)),
            material=axle_mat,
            name=f"rear_axle_pin_{index}",
        )

    # A squared-off head tube frame leaves a visible clearance hole for the steering shaft.
    frame.visual(Box((0.032, 0.032, 0.040)), origin=Origin(xyz=(0.375, 0.0, 0.290)), material=frame_mat, name="head_bridge")
    frame.visual(Box((0.012, 0.086, 0.160)), origin=Origin(xyz=(0.386, 0.0, 0.360)), material=frame_mat, name="head_front_plate")
    frame.visual(Box((0.012, 0.086, 0.160)), origin=Origin(xyz=(0.474, 0.0, 0.360)), material=frame_mat, name="head_rear_plate")
    frame.visual(Box((0.104, 0.012, 0.160)), origin=Origin(xyz=(0.430, 0.044, 0.360)), material=frame_mat, name="head_side_plate_0")
    frame.visual(Box((0.104, 0.012, 0.160)), origin=Origin(xyz=(0.430, -0.044, 0.360)), material=frame_mat, name="head_side_plate_1")

    knee_pad = model.part("knee_pad")
    knee_pad.visual(Box((0.420, 0.230, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=metal_mat, name="platform_plate")
    knee_pad.visual(Box((0.395, 0.205, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=pad_mat, name="cushion")
    model.articulation(
        "frame_to_knee_pad",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_pad,
        origin=Origin(xyz=(-0.08, 0.0, 0.500)),
    )

    fork = model.part("steering_fork")
    fork.visual(Cylinder(radius=0.038, length=0.050), origin=_cyl_z((0.0, 0.0, 0.0)), material=axle_mat, name="steering_bearing")
    fork.visual(Cylinder(radius=0.017, length=0.620), origin=_cyl_z((0.0, 0.0, 0.310)), material=metal_mat, name="steering_column")
    fork.visual(Cylinder(radius=0.017, length=0.260), origin=_cyl_z((0.0, 0.0, -0.130)), material=metal_mat, name="lower_steerer")
    fork.visual(Cylinder(radius=0.018, length=0.500), origin=_cyl_y((0.0, 0.0, 0.620)), material=metal_mat, name="handlebar")
    fork.visual(Cylinder(radius=0.023, length=0.085), origin=_cyl_y((0.0, 0.285, 0.620)), material=rubber_mat, name="grip_0")
    fork.visual(Cylinder(radius=0.023, length=0.085), origin=_cyl_y((0.0, -0.285, 0.620)), material=rubber_mat, name="grip_1")
    fork.visual(Box((0.110, 0.430, 0.034)), origin=Origin(xyz=(0.045, 0.0, -0.222)), material=frame_mat, name="fork_crown")

    for index, y_sign in enumerate((1.0, -1.0)):
        y = y_sign * 0.210
        pin_y = y_sign * 0.2213
        fork.visual(
            Box((0.052, 0.030, 0.170)),
            origin=Origin(xyz=(0.060, y, -0.280)),
            material=frame_mat,
            name=f"front_dropout_{index}",
        )
        fork.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=_cyl_y((0.060, pin_y, -0.285)),
            material=axle_mat,
            name=f"front_axle_pin_{index}",
        )

        pivot_y = y_sign * 0.140
        fork.visual(
            Box((0.026, 0.006, 0.046)),
            origin=Origin(xyz=(-0.030, pivot_y - y_sign * 0.014, 0.592)),
            material=axle_mat,
            name=f"brake_clevis_inner_{index}",
        )
        fork.visual(
            Box((0.026, 0.006, 0.046)),
            origin=Origin(xyz=(-0.030, pivot_y + y_sign * 0.014, 0.592)),
            material=axle_mat,
            name=f"brake_clevis_outer_{index}",
        )
        fork.visual(
            Box((0.022, 0.034, 0.018)),
            origin=Origin(xyz=(-0.018, pivot_y, 0.616)),
            material=axle_mat,
            name=f"brake_clamp_{index}",
        )

    model.articulation(
        "fork_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(0.430, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.65, upper=0.65),
    )

    wheel_specs = (
        ("rear_wheel_0", frame, (-0.300, 0.250, 0.075), "rear_0"),
        ("rear_wheel_1", frame, (-0.300, -0.250, 0.075), "rear_1"),
        ("front_wheel_0", fork, (0.060, 0.250, -0.285), "front_0"),
        ("front_wheel_1", fork, (0.060, -0.250, -0.285), "front_1"),
    )
    for part_name, parent, xyz, suffix in wheel_specs:
        wheel = model.part(part_name)
        wheel.visual(tire_mesh, material=rubber_mat, name="tire")
        wheel.visual(wheel_mesh, material=metal_mat, name="rim")
        inner_sign = -1.0 if xyz[1] > 0.0 else 1.0
        wheel.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=_cyl_x((inner_sign * 0.010, 0.0, 0.0)),
            material=metal_mat,
            name="inner_hub_spacer",
        )
        model.articulation(
            f"{part_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    for index, y_sign in enumerate((1.0, -1.0)):
        lever = model.part(f"brake_lever_{index}")
        lever.visual(Cylinder(radius=0.010, length=0.022), origin=_cyl_y((0.0, 0.0, 0.0)), material=lever_mat, name="pivot_hub")
        lever.visual(Box((0.040, 0.010, 0.012)), origin=Origin(xyz=(-0.015, 0.0, -0.006)), material=lever_mat, name="lever_neck")
        lever.visual(Box((0.017, 0.010, 0.125)), origin=Origin(xyz=(-0.030, 0.0, -0.065)), material=lever_mat, name="lever_blade")
        lever.visual(Sphere(radius=0.014), origin=Origin(xyz=(-0.030, 0.0, -0.130)), material=lever_mat, name="rounded_tip")
        model.articulation(
            f"brake_lever_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=fork,
            child=lever,
            origin=Origin(xyz=(-0.030, y_sign * 0.140, 0.592)),
            axis=(0.0, y_sign, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("central_frame")
    knee_pad = object_model.get_part("knee_pad")
    fork = object_model.get_part("steering_fork")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")

    steer = object_model.get_articulation("fork_steer")
    ctx.check("fork steers on a limited revolute joint", steer.articulation_type == ArticulationType.REVOLUTE and steer.motion_limits is not None and steer.motion_limits.lower < 0 < steer.motion_limits.upper)

    for joint_name in (
        "front_wheel_0_spin",
        "front_wheel_1_spin",
        "rear_wheel_0_spin",
        "rear_wheel_1_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS)

    for joint_name in ("brake_lever_0_pivot", "brake_lever_1_pivot"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} has squeeze travel",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.5,
        )

    ctx.expect_contact(
        knee_pad,
        frame,
        elem_a="platform_plate",
        elem_b="pad_mount_plate",
        contact_tol=0.001,
        name="knee platform is seated on the frame mount",
    )
    ctx.expect_contact(
        fork,
        frame,
        elem_a="steering_bearing",
        elem_b="head_front_plate",
        contact_tol=0.001,
        name="steering bearing is captured in the head frame",
    )

    ctx.expect_gap(rear_wheel_0, frame, axis="y", positive_elem="inner_hub_spacer", negative_elem="rear_axle_pin_0", max_gap=0.002, max_penetration=0.001, name="rear wheel 0 is on its axle support")
    ctx.expect_gap(frame, rear_wheel_1, axis="y", positive_elem="rear_axle_pin_1", negative_elem="inner_hub_spacer", max_gap=0.002, max_penetration=0.001, name="rear wheel 1 is on its axle support")
    ctx.expect_gap(front_wheel_0, fork, axis="y", positive_elem="inner_hub_spacer", negative_elem="front_axle_pin_0", max_gap=0.002, max_penetration=0.001, name="front wheel 0 is on the steering fork")
    ctx.expect_gap(fork, front_wheel_1, axis="y", positive_elem="front_axle_pin_1", negative_elem="inner_hub_spacer", max_gap=0.002, max_penetration=0.001, name="front wheel 1 is on the steering fork")

    rest_front = ctx.part_world_position(front_wheel_0)
    with ctx.pose({steer: 0.5}):
        turned_front = ctx.part_world_position(front_wheel_0)
    ctx.check(
        "front wheel follows steering yaw",
        rest_front is not None
        and turned_front is not None
        and ((turned_front[0] - rest_front[0]) ** 2 + (turned_front[1] - rest_front[1]) ** 2) ** 0.5 > 0.05,
        details=f"rest={rest_front}, turned={turned_front}",
    )

    def _vec_coord(vec, index: int) -> float:
        try:
            return float(vec[index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[index])

    lever_joint = object_model.get_articulation("brake_lever_0_pivot")
    lever = object_model.get_part("brake_lever_0")
    rest_aabb = ctx.part_element_world_aabb(lever, elem="lever_blade")
    with ctx.pose({lever_joint: 0.55}):
        squeezed_aabb = ctx.part_element_world_aabb(lever, elem="lever_blade")
    rest_blade_x = None if rest_aabb is None else (_vec_coord(rest_aabb[0], 0) + _vec_coord(rest_aabb[1], 0)) / 2.0
    squeezed_blade_x = None if squeezed_aabb is None else (_vec_coord(squeezed_aabb[0], 0) + _vec_coord(squeezed_aabb[1], 0)) / 2.0
    ctx.check(
        "brake lever blade rotates about its pivot",
        rest_blade_x is not None and squeezed_blade_x is not None and abs(squeezed_blade_x - rest_blade_x) > 0.010,
        details=f"rest_x={rest_blade_x}, squeezed_x={squeezed_blade_x}",
    )

    return ctx.report()


object_model = build_object_model()
