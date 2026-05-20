from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="micro_indoor_quadcopter")

    frame_plastic = model.material("frame_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_plastic = model.material("arm_plastic", rgba=(0.21, 0.22, 0.24, 1.0))
    prop_black = model.material("prop_black", rgba=(0.07, 0.07, 0.08, 1.0))
    motor_metal = model.material("motor_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    door_plastic = model.material("door_plastic", rgba=(0.24, 0.25, 0.27, 1.0))
    accent = model.material("accent", rgba=(0.80, 0.37, 0.10, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.118, 0.118, 0.038)),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    frame_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.0185, -0.0100),
            (0.0245, -0.0080),
            (0.0290, -0.0030),
            (0.0280, 0.0030),
            (0.0210, 0.0100),
            (0.0100, 0.0135),
            (0.0000, 0.0148),
        ],
        [
            (0.0150, -0.0090),
            (0.0215, -0.0070),
            (0.0257, -0.0026),
            (0.0244, 0.0020),
            (0.0177, 0.0080),
            (0.0078, 0.0115),
            (0.0000, 0.0132),
        ],
        segments=64,
    )
    frame.visual(
        mesh_from_geometry(frame_shell_geom, "frame_shell"),
        material=frame_plastic,
        name="center_frame_shell",
    )
    frame.visual(
        Cylinder(radius=0.0065, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0152)),
        material=accent,
        name="top_button",
    )

    motor_radius = 0.054
    arm_inner_radius = 0.020
    arm_outer_radius = 0.050
    arm_length = arm_outer_radius - arm_inner_radius
    arm_width = 0.010
    arm_thickness = 0.0036
    motor_pod_radius = 0.0065
    motor_pod_length = 0.017
    motor_pod_center_z = 0.0005
    motor_joint_z = motor_pod_center_z + motor_pod_length * 0.5
    arm_z = -0.0005

    rotor_specs = [
        ("front_left", math.radians(45.0)),
        ("front_right", math.radians(-45.0)),
        ("rear_left", math.radians(135.0)),
        ("rear_right", math.radians(-135.0)),
    ]

    for rotor_name, angle in rotor_specs:
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        arm_center_r = arm_inner_radius + arm_length * 0.5
        frame.visual(
            Box((arm_length, arm_width, arm_thickness)),
            origin=Origin(
                xyz=(arm_center_r * cos_a, arm_center_r * sin_a, arm_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=arm_plastic,
            name=f"arm_{rotor_name}",
        )
        frame.visual(
            Cylinder(radius=motor_pod_radius, length=motor_pod_length),
            origin=Origin(
                xyz=(motor_radius * cos_a, motor_radius * sin_a, motor_pod_center_z)
            ),
            material=motor_metal,
            name=f"motor_pod_{rotor_name}",
        )
        frame.visual(
            Cylinder(radius=0.0032, length=0.0045),
            origin=Origin(
                xyz=(motor_radius * cos_a, motor_radius * sin_a, -0.0102)
            ),
            material=arm_plastic,
            name=f"landing_foot_{rotor_name}",
        )

    frame.visual(
        Box((0.006, 0.044, 0.0022)),
        origin=Origin(xyz=(-0.0165, 0.0, -0.0091)),
        material=frame_plastic,
        name="door_hinge_rail",
    )
    frame.visual(
        Cylinder(radius=0.0024, length=0.0085),
        origin=Origin(
            xyz=(-0.0160, -0.0130, -0.0086),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_plastic,
        name="door_hinge_lug_left",
    )
    frame.visual(
        Cylinder(radius=0.0024, length=0.0085),
        origin=Origin(
            xyz=(-0.0160, 0.0130, -0.0086),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_plastic,
        name="door_hinge_lug_right",
    )
    frame.visual(
        Box((0.004, 0.012, 0.003)),
        origin=Origin(xyz=(0.016, 0.0, -0.0090)),
        material=frame_plastic,
        name="door_latch_catch",
    )

    battery_door = model.part("battery_door")
    battery_door.inertial = Inertial.from_geometry(
        Box((0.034, 0.038, 0.005)),
        mass=0.003,
        origin=Origin(xyz=(0.017, 0.0, -0.0025)),
    )
    battery_door.visual(
        Box((0.034, 0.038, 0.0022)),
        origin=Origin(xyz=(0.017, 0.0, -0.0027)),
        material=door_plastic,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0022, length=0.0150),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=door_plastic,
        name="door_hinge_barrel",
    )
    battery_door.visual(
        Box((0.004, 0.010, 0.0032)),
        origin=Origin(xyz=(0.032, 0.0, -0.0026)),
        material=door_plastic,
        name="door_latch_tab",
    )

    model.articulation(
        "frame_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=battery_door,
        origin=Origin(xyz=(-0.0160, 0.0, -0.0086)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    prop_radius = 0.024
    blade_length = 0.018
    blade_width = 0.0052
    blade_thickness = 0.0012
    blade_profile = [
        (-0.0015, -blade_width * 0.28),
        (0.0018, -blade_width * 0.48),
        (0.0085, -blade_width * 0.46),
        (0.0145, -blade_width * 0.34),
        (blade_length, -blade_width * 0.10),
        (blade_length, blade_width * 0.10),
        (0.0145, blade_width * 0.34),
        (0.0085, blade_width * 0.46),
        (0.0018, blade_width * 0.48),
        (-0.0015, blade_width * 0.28),
    ]
    prop_blade_mesh = mesh_from_geometry(
        ExtrudeGeometry(blade_profile, blade_thickness, center=True),
        "prop_blade",
    )
    for rotor_name, angle in rotor_specs:
        rotor = model.part(f"{rotor_name}_rotor")
        rotor.inertial = Inertial.from_geometry(
            Cylinder(radius=prop_radius, length=0.008),
            mass=0.0012,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )
        rotor.visual(
            Cylinder(radius=0.0054, length=0.0048),
            origin=Origin(xyz=(0.0, 0.0, 0.0024)),
            material=motor_metal,
            name="rotor_bell",
        )
        rotor.visual(
            Cylinder(radius=0.0017, length=0.0028),
            origin=Origin(xyz=(0.0, 0.0, 0.0047)),
            material=motor_metal,
            name="rotor_shaft",
        )
        rotor.visual(
            Cylinder(radius=0.0048, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, 0.0058)),
            material=prop_black,
            name="prop_hub",
        )
        for blade_index in range(3):
            blade_angle = blade_index * (2.0 * math.pi / 3.0)
            rotor.visual(
                prop_blade_mesh,
                origin=Origin(
                    xyz=(0.0, 0.0, 0.0059),
                    rpy=(0.0, 0.0, blade_angle),
                ),
                material=prop_black,
                name=f"blade_{blade_index + 1}",
            )
        model.articulation(
            f"frame_to_{rotor_name}_rotor",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=rotor,
            origin=Origin(
                xyz=(
                    motor_radius * math.cos(angle),
                    motor_radius * math.sin(angle),
                    motor_joint_z,
                )
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=60.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    battery_door = object_model.get_part("battery_door")
    door_hinge = object_model.get_articulation("frame_to_battery_door")
    shell = frame.get_visual("center_frame_shell")
    door_panel = battery_door.get_visual("door_panel")

    ctx.check(
        "battery door hinge opens downward on a y-axis revolute joint",
        door_hinge.joint_type == ArticulationType.REVOLUTE
        and door_hinge.axis == (0.0, 1.0, 0.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.5,
        details=(
            f"type={door_hinge.joint_type}, axis={door_hinge.axis}, "
            f"limits={door_hinge.motion_limits}"
        ),
    )

    rotor_names = [
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    ]

    for rotor_name in rotor_names:
        rotor = object_model.get_part(f"{rotor_name}_rotor")
        rotor_joint = object_model.get_articulation(f"frame_to_{rotor_name}_rotor")
        motor_pod = frame.get_visual(f"motor_pod_{rotor_name}")
        rotor_bell = rotor.get_visual("rotor_bell")

        ctx.check(
            f"{rotor_name} rotor articulation is continuous",
            rotor_joint.joint_type == ArticulationType.CONTINUOUS
            and rotor_joint.axis == (0.0, 0.0, 1.0)
            and rotor_joint.motion_limits is not None
            and rotor_joint.motion_limits.lower is None
            and rotor_joint.motion_limits.upper is None,
            details=(
                f"type={rotor_joint.joint_type}, axis={rotor_joint.axis}, "
                f"limits={rotor_joint.motion_limits}"
            ),
        )
        ctx.expect_gap(
            rotor,
            frame,
            axis="z",
            positive_elem=rotor_bell,
            negative_elem=motor_pod,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{rotor_name} rotor sits on its motor pod",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            frame,
            battery_door,
            axis="z",
            positive_elem=shell,
            negative_elem=door_panel,
            max_gap=0.003,
            max_penetration=0.0,
            name="battery door closes just beneath the frame shell",
        )
        ctx.expect_overlap(
            battery_door,
            frame,
            axes="xy",
            elem_a=door_panel,
            elem_b=shell,
            min_overlap=0.020,
            name="battery door covers the underside bay opening",
        )

    closed_door_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        closed_door_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")
    with ctx.pose({door_hinge: math.radians(100.0)}):
        open_door_aabb = ctx.part_element_world_aabb(battery_door, elem="door_panel")
    ctx.check(
        "battery door swings downward when opened",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.010,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rotor_positions = {
        rotor_name: ctx.part_world_position(f"{rotor_name}_rotor") for rotor_name in rotor_names
    }
    radii = []
    for pos in rotor_positions.values():
        if pos is not None:
            radii.append(math.hypot(pos[0], pos[1]))
    symmetry_ok = (
        len(radii) == 4
        and max(radii) - min(radii) < 0.002
        and rotor_positions["front_left"] is not None
        and rotor_positions["front_right"] is not None
        and rotor_positions["rear_left"] is not None
        and rotor_positions["rear_right"] is not None
        and abs(rotor_positions["front_left"][0] + rotor_positions["rear_right"][0]) < 0.002
        and abs(rotor_positions["front_left"][1] + rotor_positions["rear_right"][1]) < 0.002
        and abs(rotor_positions["front_right"][0] + rotor_positions["rear_left"][0]) < 0.002
        and abs(rotor_positions["front_right"][1] + rotor_positions["rear_left"][1]) < 0.002
    )
    ctx.check(
        "rotors are placed symmetrically at equal radius",
        symmetry_ok,
        details=str(rotor_positions),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
