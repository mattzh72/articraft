from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_quadcopter")

    carbon = Material("matte_carbon", rgba=(0.035, 0.038, 0.042, 1.0))
    dark = Material("soft_black", rgba=(0.005, 0.006, 0.007, 1.0))
    graphite = Material("graphite_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    metal = Material("dark_anodized_metal", rgba=(0.25, 0.26, 0.27, 1.0))
    lens = Material("blue_glass", rgba=(0.03, 0.08, 0.14, 0.78))
    amber = Material("front_status_amber", rgba=(1.0, 0.55, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.28, 0.18, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=carbon,
        name="main_shell",
    )
    body.visual(
        Box((0.19, 0.105, 0.020)),
        origin=Origin(xyz=(-0.018, 0.0, 0.164)),
        material=graphite,
        name="battery_hatch",
    )
    body.visual(
        Box((0.045, 0.070, 0.010)),
        origin=Origin(xyz=(0.120, 0.0, 0.158)),
        material=amber,
        name="nose_light",
    )

    # Landing gear: two tubular skids with four compression struts, all tied
    # into the central body as one rigid manufactured frame.
    for y in (-0.080, 0.080):
        side = "0" if y < 0 else "1"
        body.visual(
            Cylinder(radius=0.009, length=0.330),
            origin=Origin(xyz=(0.0, y, 0.030), rpy=(0.0, pi / 2, 0.0)),
            material=dark,
            name=f"skid_rail_{side}",
        )
        for x in (-0.090, 0.090):
            idx = "0" if x < 0 else "1"
            body.visual(
                Cylinder(radius=0.007, length=0.082),
                origin=Origin(xyz=(x, y, 0.068)),
                material=dark,
                name=f"skid_strut_{side}_{idx}",
            )

    # Camera yoke on the nose; the tilting camera lives between the ears.
    body.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(0.151, -0.041, 0.105)),
        material=graphite,
        name="camera_yoke_0",
    )
    body.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(0.151, 0.041, 0.105)),
        material=graphite,
        name="camera_yoke_1",
    )

    # Four hinge sockets at the corners for the folding arms.  The arm root
    # barrel is intentionally coaxial with each socket, like a compact drone
    # folding knuckle.
    arm_specs = [
        ("front_left_arm", 1.0, 1.0, pi / 4, (0.166, 0.106, 0.120)),
        ("front_right_arm", 1.0, -1.0, -pi / 4, (0.166, -0.106, 0.120)),
        ("rear_left_arm", -1.0, 1.0, 3 * pi / 4, (-0.166, 0.106, 0.120)),
        ("rear_right_arm", -1.0, -1.0, -3 * pi / 4, (-0.166, -0.106, 0.120)),
    ]
    for arm_name, _sx, _sy, yaw, hinge_xyz in arm_specs:
        hx, hy, hz = hinge_xyz
        body.visual(
            Box((0.052, 0.030, 0.034)),
            origin=Origin(xyz=(hx - 0.046 if hx > 0 else hx + 0.046, hy - 0.008 if hy > 0 else hy + 0.008, hz)),
            material=graphite,
            name=f"{arm_name}_hinge_block",
        )
        body.visual(
            Cylinder(radius=0.024, length=0.030),
            origin=Origin(xyz=hinge_xyz),
            material=metal,
            name=f"{arm_name}_socket",
        )

    rotor_meshes = {}
    for arm_name, _sx, _sy, _yaw, _hinge_xyz in arm_specs:
        rotor_meshes[arm_name] = mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.105,
                hub_radius=0.022,
                blade_count=3,
                thickness=0.010,
                blade_pitch_deg=30.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
                hub=FanRotorHub(style="spinner", bore_diameter=0.006),
            ),
            f"{arm_name}_rotor_mesh",
        )

    for arm_name, _sx, _sy, yaw, hinge_xyz in arm_specs:
        arm = model.part(arm_name)
        arm.visual(
            Cylinder(radius=0.018, length=0.042),
            origin=Origin(),
            material=metal,
            name="root_barrel",
        )
        arm.visual(
            Box((0.282, 0.034, 0.024)),
            origin=Origin(xyz=(0.159, 0.0, 0.0)),
            material=dark,
            name="carbon_arm",
        )
        arm.visual(
            Box((0.080, 0.050, 0.018)),
            origin=Origin(xyz=(0.260, 0.0, -0.004)),
            material=graphite,
            name="motor_mount",
        )
        arm.visual(
            Cylinder(radius=0.053, length=0.054),
            origin=Origin(xyz=(0.315, 0.0, 0.000)),
            material=metal,
            name="motor_pod",
        )
        arm.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(0.315, 0.0, 0.035)),
            material=metal,
            name="motor_cap",
        )

        model.articulation(
            f"body_to_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-1.55, upper=1.55),
        )

        rotor = model.part(arm_name.replace("_arm", "_rotor"))
        rotor.visual(
            rotor_meshes[arm_name],
            origin=Origin(),
            material=dark,
            name="propeller",
        )
        rotor.visual(
            Cylinder(radius=0.006, length=0.058),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=metal,
            name="rotor_shaft",
        )
        model.articulation(
            f"{arm_name}_to_rotor",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=rotor,
            origin=Origin(xyz=(0.315, 0.0, 0.052)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.45, velocity=80.0),
        )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=metal,
        name="tilt_axle",
    )
    camera.visual(
        Box((0.058, 0.052, 0.044)),
        origin=Origin(xyz=(0.035, 0.0, -0.004)),
        material=graphite,
        name="camera_housing",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.075, 0.0, -0.004), rpy=(0.0, pi / 2, 0.0)),
        material=lens,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.087, 0.0, -0.004), rpy=(0.0, pi / 2, 0.0)),
        material=lens,
        name="front_glass",
    )
    model.articulation(
        "body_to_camera",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera,
        origin=Origin(xyz=(0.156, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.5, lower=-0.65, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    camera = object_model.get_part("camera")
    camera_joint = object_model.get_articulation("body_to_camera")

    arm_names = ("front_left_arm", "front_right_arm", "rear_left_arm", "rear_right_arm")
    for arm_name in arm_names:
        arm = object_model.get_part(arm_name)
        rotor = object_model.get_part(arm_name.replace("_arm", "_rotor"))
        arm_joint = object_model.get_articulation(f"body_to_{arm_name}")
        rotor_joint = object_model.get_articulation(f"{arm_name}_to_rotor")

        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{arm_name}_socket",
            elem_b="root_barrel",
            reason="The folding arm root barrel is intentionally captured in the body hinge socket.",
        )
        ctx.allow_overlap(
            body,
            arm,
            elem_a=f"{arm_name}_socket",
            elem_b="carbon_arm",
            reason="The inboard carbon spar is lightly seated into the hinge socket for a mechanically plausible folding knuckle.",
        )
        ctx.expect_overlap(
            body,
            arm,
            axes="xy",
            elem_a=f"{arm_name}_socket",
            elem_b="root_barrel",
            min_overlap=0.010,
            name=f"{arm_name} hinge barrel is seated in socket",
        )
        ctx.allow_overlap(
            arm,
            rotor,
            elem_a="motor_pod",
            elem_b="rotor_shaft",
            reason="The rotor shaft intentionally enters the motor pod to show the spinning propeller drive connection.",
        )
        ctx.allow_overlap(
            arm,
            rotor,
            elem_a="motor_cap",
            elem_b="rotor_shaft",
            reason="The rotor shaft intentionally passes through the motor cap on the spinning axis.",
        )
        ctx.expect_within(
            rotor,
            arm,
            axes="xy",
            inner_elem="rotor_shaft",
            outer_elem="motor_pod",
            margin=0.002,
            name=f"{arm_name} rotor shaft is centered in motor pod",
        )
        ctx.expect_within(
            rotor,
            arm,
            axes="xy",
            inner_elem="rotor_shaft",
            outer_elem="motor_cap",
            margin=0.002,
            name=f"{arm_name} rotor shaft is centered in motor cap",
        )
        with ctx.pose({rotor_joint: 1.2}):
            ctx.expect_within(
                rotor,
                arm,
                axes="xy",
                inner_elem="rotor_shaft",
                outer_elem="motor_pod",
                margin=0.002,
                name=f"{arm_name} rotor spins about motor axis",
            )
        rest_pos = ctx.part_world_position(arm)
        with ctx.pose({arm_joint: 0.75}):
            folded_pos = ctx.part_world_position(arm)
        ctx.check(
            f"{arm_name} hinge moves arm in plan",
            rest_pos is not None
            and folded_pos is not None
            and abs(rest_pos[0] - folded_pos[0]) + abs(rest_pos[1] - folded_pos[1]) < 1e-6,
            details=f"hinge root should stay fixed: rest={rest_pos}, posed={folded_pos}",
        )

    ctx.allow_overlap(
        body,
        camera,
        elem_a="camera_yoke_0",
        elem_b="tilt_axle",
        reason="The camera tilt axle is intentionally captured by the yoke ear.",
    )
    ctx.allow_overlap(
        body,
        camera,
        elem_a="camera_yoke_1",
        elem_b="tilt_axle",
        reason="The camera tilt axle is intentionally captured by the yoke ear.",
    )
    ctx.expect_overlap(
        body,
        camera,
        axes="y",
        elem_a="camera_yoke_0",
        elem_b="tilt_axle",
        min_overlap=0.001,
        name="camera axle reaches one yoke ear",
    )
    with ctx.pose({camera_joint: -0.35}):
        ctx.expect_overlap(
            camera,
            body,
            axes="y",
            elem_a="tilt_axle",
            elem_b="camera_yoke_1",
            min_overlap=0.001,
            name="camera tilts about horizontal axle",
        )

    return ctx.report()


object_model = build_object_model()
