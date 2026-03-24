from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, str(ASSETS.mesh_dir / name))


def _spring_mesh(length: float, radius: float, coils: int, name: str):
    z0 = -length / 2.0
    z1 = length / 2.0
    profile = [(0.0, z0), (radius * 0.92, z0)]
    step = length / (coils * 2 + 1)
    for idx in range(coils * 2):
        z = z0 + (idx + 1) * step
        corrugation = radius if idx % 2 == 0 else radius * 0.68
        profile.append((corrugation, z))
    profile.extend([(radius * 0.92, z1), (0.0, z1)])
    spring_geom = LatheGeometry(profile, segments=30).rotate_y(math.pi / 2.0)
    return _save_mesh(name, spring_geom)


def _shade_shell_mesh(name: str):
    shell = LatheGeometry(
        [
            (0.0, -0.030),
            (0.018, -0.030),
            (0.023, -0.022),
            (0.027, -0.010),
            (0.031, 0.010),
            (0.036, 0.027),
            (0.039, 0.040),
            (0.033, 0.0385),
            (0.027, 0.018),
            (0.022, -0.002),
            (0.018, -0.015),
            (0.014, -0.021),
            (0.014, -0.025),
            (0.0, -0.025),
        ],
        segments=44,
    )
    shell.rotate_y(math.pi / 2.0)
    return _save_mesh(name, shell)


def _cable_loop_mesh(points, radius: float, name: str):
    cable = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=18,
        radial_segments=12,
        cap_ends=True,
    )
    return _save_mesh(name, cable)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anglepoise_desk_lamp", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.77, 0.78, 0.76, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.96, 0.90, 0.72, 0.38))
    off_white = model.material("off_white", rgba=(0.86, 0.84, 0.79, 1.0))

    lower_spring_mesh = _spring_mesh(0.166, 0.0048, 12, "lower_arm_spring.obj")
    upper_spring_mesh = _spring_mesh(0.145, 0.0046, 11, "upper_arm_spring.obj")
    shade_shell_mesh = _shade_shell_mesh("lamp_head_shell.obj")
    lower_cable_loop_mesh = _cable_loop_mesh(
        [(0.022, 0.0, -0.010), (0.030, 0.0, -0.018), (0.038, 0.0, -0.021), (0.046, 0.0, -0.019)],
        radius=0.0028,
        name="lower_arm_cable_loop.obj",
    )
    upper_cable_loop_mesh = _cable_loop_mesh(
        [(0.180, 0.0, -0.019), (0.188, 0.0, -0.026), (0.196, 0.0, -0.021), (0.204, 0.0, -0.010)],
        radius=0.0028,
        name="upper_arm_cable_loop.obj",
    )
    head_cable_loop_mesh = _cable_loop_mesh(
        [(-0.008, 0.0, 0.010), (0.004, 0.0, 0.017), (0.021, 0.0, 0.016), (0.038, 0.0, 0.010)],
        radius=0.0028,
        name="lamp_head_cable_loop.obj",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.118, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=painted_steel,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.108, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=black_rubber,
        name="base_pad",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=painted_steel,
        name="base_collar",
    )
    base.visual(
        Cylinder(radius=0.0135, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=painted_steel,
        name="stem",
    )
    base.visual(
        Box((0.019, 0.052, 0.014)),
        origin=Origin(xyz=(-0.019, 0.0, 0.104)),
        material=painted_steel,
        name="shoulder_bridge",
    )
    base.visual(
        Box((0.015, 0.006, 0.034)),
        origin=Origin(xyz=(-0.0025, 0.0245, 0.117)),
        material=painted_steel,
        name="shoulder_cheek_left",
    )
    base.visual(
        Box((0.015, 0.006, 0.034)),
        origin=Origin(xyz=(-0.0025, -0.0245, 0.117)),
        material=painted_steel,
        name="shoulder_cheek_right",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.0015),
        origin=Origin(xyz=(0.0, 0.021, 0.117), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="shoulder_washer_left",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.0015),
        origin=Origin(xyz=(0.0, -0.021, 0.117), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="shoulder_washer_right",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.024),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.0088, length=0.0410),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rear_knuckle",
    )
    lower_arm.visual(
        Box((0.030, 0.0055, 0.010)),
        origin=Origin(xyz=(0.013, 0.018, 0.0)),
        material=satin_aluminum,
        name="rear_link_left",
    )
    lower_arm.visual(
        Box((0.030, 0.0055, 0.010)),
        origin=Origin(xyz=(0.013, -0.018, 0.0)),
        material=satin_aluminum,
        name="rear_link_right",
    )
    lower_arm.visual(
        Box((0.206, 0.0055, 0.012)),
        origin=Origin(xyz=(0.130, 0.018, 0.0)),
        material=satin_aluminum,
        name="bar_left",
    )
    lower_arm.visual(
        Box((0.206, 0.0055, 0.012)),
        origin=Origin(xyz=(0.130, -0.018, 0.0)),
        material=satin_aluminum,
        name="bar_right",
    )
    lower_arm.visual(
        Box((0.028, 0.006, 0.028)),
        origin=Origin(xyz=(0.246, 0.0215, 0.0)),
        material=black_rubber,
        name="elbow_cheek_left",
    )
    lower_arm.visual(
        Box((0.028, 0.006, 0.028)),
        origin=Origin(xyz=(0.246, -0.0215, 0.0)),
        material=black_rubber,
        name="elbow_cheek_right",
    )
    lower_arm.visual(
        Cylinder(radius=0.0095, length=0.0015),
        origin=Origin(xyz=(0.246, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="elbow_washer_left",
    )
    lower_arm.visual(
        Cylinder(radius=0.0095, length=0.0015),
        origin=Origin(xyz=(0.246, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="elbow_washer_right",
    )
    lower_arm.visual(
        Box((0.022, 0.034, 0.008)),
        origin=Origin(xyz=(0.214, 0.0, -0.010)),
        material=black_rubber,
        name="front_cross_brace",
    )
    lower_arm.visual(
        lower_spring_mesh,
        origin=Origin(xyz=(0.128, 0.012, -0.013)),
        material=spring_steel,
        name="spring_left",
    )
    lower_arm.visual(
        lower_spring_mesh,
        origin=Origin(xyz=(0.128, -0.012, -0.013)),
        material=spring_steel,
        name="spring_right",
    )
    lower_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.050, 0.014, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_rear_left",
    )
    lower_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.050, -0.014, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_rear_right",
    )
    lower_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.206, 0.014, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_front_left",
    )
    lower_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.206, -0.014, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_front_right",
    )
    lower_arm.visual(
        Cylinder(radius=0.0028, length=0.185),
        origin=Origin(xyz=(0.132, 0.0, -0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="cable_run",
    )
    lower_arm.visual(
        lower_cable_loop_mesh,
        origin=Origin(),
        material=black_rubber,
        name="rear_cable_loop",
    )
    lower_arm.visual(
        Box((0.008, 0.022, 0.012)),
        origin=Origin(xyz=(0.074, 0.009, -0.011)),
        material=black_rubber,
        name="cable_clip_rear",
    )
    lower_arm.visual(
        Box((0.008, 0.022, 0.012)),
        origin=Origin(xyz=(0.182, 0.009, -0.011)),
        material=black_rubber,
        name="cable_clip_front",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.270, 0.060, 0.040)),
        mass=0.72,
        origin=Origin(xyz=(0.135, 0.0, -0.004)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.0084, length=0.0355),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rear_knuckle",
    )
    upper_arm.visual(
        Box((0.030, 0.0055, 0.010)),
        origin=Origin(xyz=(0.013, 0.015, 0.0)),
        material=satin_aluminum,
        name="rear_link_left",
    )
    upper_arm.visual(
        Box((0.030, 0.0055, 0.010)),
        origin=Origin(xyz=(0.013, -0.015, 0.0)),
        material=satin_aluminum,
        name="rear_link_right",
    )
    upper_arm.visual(
        Box((0.184, 0.0055, 0.011)),
        origin=Origin(xyz=(0.119, 0.015, 0.0)),
        material=satin_aluminum,
        name="bar_left",
    )
    upper_arm.visual(
        Box((0.184, 0.0055, 0.011)),
        origin=Origin(xyz=(0.119, -0.015, 0.0)),
        material=satin_aluminum,
        name="bar_right",
    )
    upper_arm.visual(
        Box((0.022, 0.006, 0.026)),
        origin=Origin(xyz=(0.222, 0.0195, 0.0)),
        material=black_rubber,
        name="head_cheek_left",
    )
    upper_arm.visual(
        Box((0.022, 0.006, 0.026)),
        origin=Origin(xyz=(0.222, -0.0195, 0.0)),
        material=black_rubber,
        name="head_cheek_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.0092, length=0.0015),
        origin=Origin(xyz=(0.223, 0.0165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="head_washer_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.0092, length=0.0015),
        origin=Origin(xyz=(0.223, -0.0165, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="head_washer_right",
    )
    upper_arm.visual(
        Box((0.020, 0.032, 0.007)),
        origin=Origin(xyz=(0.192, 0.0, -0.009)),
        material=black_rubber,
        name="front_cross_brace",
    )
    upper_arm.visual(
        upper_spring_mesh,
        origin=Origin(xyz=(0.113, 0.011, -0.013)),
        material=spring_steel,
        name="spring_left",
    )
    upper_arm.visual(
        upper_spring_mesh,
        origin=Origin(xyz=(0.113, -0.011, -0.013)),
        material=spring_steel,
        name="spring_right",
    )
    upper_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.046, 0.013, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_rear_left",
    )
    upper_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.046, -0.013, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_rear_right",
    )
    upper_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.180, 0.013, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_front_left",
    )
    upper_arm.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.180, -0.013, -0.007)),
        material=satin_aluminum,
        name="spring_anchor_front_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.0028, length=0.170),
        origin=Origin(xyz=(0.112, 0.0, -0.019), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="cable_run",
    )
    upper_arm.visual(
        upper_cable_loop_mesh,
        origin=Origin(),
        material=black_rubber,
        name="front_cable_loop",
    )
    upper_arm.visual(
        Box((0.008, 0.020, 0.012)),
        origin=Origin(xyz=(0.070, 0.0085, -0.011)),
        material=black_rubber,
        name="cable_clip_rear",
    )
    upper_arm.visual(
        Box((0.008, 0.020, 0.012)),
        origin=Origin(xyz=(0.172, 0.0085, -0.011)),
        material=black_rubber,
        name="cable_clip_front",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.240, 0.050, 0.040)),
        mass=0.56,
        origin=Origin(xyz=(0.118, 0.0, -0.004)),
    )

    head = model.part("lamp_head")
    head.visual(
        Cylinder(radius=0.0082, length=0.0335),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="rear_knuckle",
    )
    head.visual(
        Box((0.018, 0.022, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=black_rubber,
        name="yoke_block",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="neck_tube",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=off_white,
        name="rear_cap",
    )
    head.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=off_white,
        name="shade_shell",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="socket_housing",
    )
    head.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.096, 0.0, 0.0)),
        material=warm_glass,
        name="bulb_globe",
    )
    head.visual(
        Box((0.010, 0.014, 0.004)),
        origin=Origin(xyz=(0.074, 0.0, 0.028)),
        material=painted_steel,
        name="vent_fin_center",
    )
    head.visual(
        Box((0.010, 0.012, 0.004)),
        origin=Origin(xyz=(0.088, 0.0, 0.027)),
        material=painted_steel,
        name="vent_fin_front",
    )
    head.visual(
        head_cable_loop_mesh,
        origin=Origin(),
        material=black_rubber,
        name="head_cable_loop",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.150, 0.090, 0.090)),
        mass=0.46,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.117), rpy=(0.0, -0.35, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.78, upper=0.52),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.246, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.70, upper=0.92),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(0.223, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.8, lower=-0.75, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head = object_model.get_part("lamp_head")
    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    head_tilt = object_model.get_articulation("upper_to_head")

    base_disk = base.get_visual("base_disk")
    shoulder_washer_left = base.get_visual("shoulder_washer_left")
    lower_rear_knuckle = lower_arm.get_visual("rear_knuckle")
    lower_bar_left = lower_arm.get_visual("bar_left")
    lower_spring_left = lower_arm.get_visual("spring_left")
    lower_cable = lower_arm.get_visual("cable_run")
    lower_spring_anchor_rear_left = lower_arm.get_visual("spring_anchor_rear_left")
    lower_spring_anchor_front_left = lower_arm.get_visual("spring_anchor_front_left")
    lower_cable_clip_rear = lower_arm.get_visual("cable_clip_rear")
    lower_cable_clip_front = lower_arm.get_visual("cable_clip_front")
    elbow_washer_left = lower_arm.get_visual("elbow_washer_left")
    upper_rear_knuckle = upper_arm.get_visual("rear_knuckle")
    upper_bar_left = upper_arm.get_visual("bar_left")
    upper_spring_left = upper_arm.get_visual("spring_left")
    upper_cable = upper_arm.get_visual("cable_run")
    upper_spring_anchor_rear_left = upper_arm.get_visual("spring_anchor_rear_left")
    upper_spring_anchor_front_left = upper_arm.get_visual("spring_anchor_front_left")
    upper_cable_clip_rear = upper_arm.get_visual("cable_clip_rear")
    upper_cable_clip_front = upper_arm.get_visual("cable_clip_front")
    head_washer_left = upper_arm.get_visual("head_washer_left")
    head_rear_knuckle = head.get_visual("rear_knuckle")
    shade_shell = head.get_visual("shade_shell")
    bulb_globe = head.get_visual("bulb_globe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(lower_arm, base, elem_a=lower_rear_knuckle, elem_b=shoulder_washer_left)
    ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_knuckle, elem_b=elbow_washer_left)
    ctx.expect_contact(head, upper_arm, elem_a=head_rear_knuckle, elem_b=head_washer_left)
    ctx.expect_contact(
        lower_arm,
        lower_arm,
        elem_a=lower_spring_left,
        elem_b=lower_spring_anchor_rear_left,
        name="lower spring is hooked to the rear anchor",
    )
    ctx.expect_contact(
        lower_arm,
        lower_arm,
        elem_a=lower_spring_left,
        elem_b=lower_spring_anchor_front_left,
        name="lower spring is hooked to the front anchor",
    )
    ctx.expect_contact(
        upper_arm,
        upper_arm,
        elem_a=upper_spring_left,
        elem_b=upper_spring_anchor_rear_left,
        name="upper spring is hooked to the rear anchor",
    )
    ctx.expect_contact(
        upper_arm,
        upper_arm,
        elem_a=upper_spring_left,
        elem_b=upper_spring_anchor_front_left,
        name="upper spring is hooked to the front anchor",
    )
    ctx.expect_contact(
        lower_arm,
        lower_arm,
        elem_a=lower_cable,
        elem_b=lower_cable_clip_rear,
        name="lower arm cable is retained by the rear clip",
    )
    ctx.expect_contact(
        lower_arm,
        lower_arm,
        elem_a=lower_cable,
        elem_b=lower_cable_clip_front,
        name="lower arm cable is retained by the front clip",
    )
    ctx.expect_contact(
        upper_arm,
        upper_arm,
        elem_a=upper_cable,
        elem_b=upper_cable_clip_rear,
        name="upper arm cable is retained by the rear clip",
    )
    ctx.expect_contact(
        upper_arm,
        upper_arm,
        elem_a=upper_cable,
        elem_b=upper_cable_clip_front,
        name="upper arm cable is retained by the front clip",
    )
    ctx.expect_within(
        head,
        head,
        axes="yz",
        inner_elem=bulb_globe,
        outer_elem=shade_shell,
        name="bulb sits inside the lamp shade opening",
    )
    ctx.expect_gap(head, base, axis="x", min_gap=0.18, positive_elem=shade_shell, negative_elem=base_disk)
    ctx.expect_gap(head, base, axis="z", min_gap=0.09, positive_elem=shade_shell, negative_elem=base_disk)
    ctx.expect_within(head, base, axes="y", inner_elem=shade_shell, outer_elem=base_disk)

    with ctx.pose({shoulder: -0.55, elbow: -0.25, head_tilt: 0.30}):
        ctx.expect_contact(lower_arm, base, elem_a=lower_rear_knuckle, elem_b=shoulder_washer_left)
        ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_knuckle, elem_b=elbow_washer_left)
        ctx.expect_contact(head, upper_arm, elem_a=head_rear_knuckle, elem_b=head_washer_left)
        ctx.expect_gap(head, base, axis="x", min_gap=0.14, positive_elem=shade_shell, negative_elem=base_disk)
        ctx.expect_gap(head, base, axis="z", min_gap=0.05, positive_elem=shade_shell, negative_elem=base_disk)
        ctx.expect_within(head, base, axes="y", inner_elem=shade_shell, outer_elem=base_disk)

    with ctx.pose({shoulder: -0.32, elbow: -0.38, head_tilt: 0.12}):
        ctx.expect_contact(lower_arm, base, elem_a=lower_rear_knuckle, elem_b=shoulder_washer_left)
        ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_knuckle, elem_b=elbow_washer_left)
        ctx.expect_contact(head, upper_arm, elem_a=head_rear_knuckle, elem_b=head_washer_left)
        ctx.expect_gap(head, base, axis="x", min_gap=0.12, positive_elem=shade_shell, negative_elem=base_disk)
        ctx.expect_gap(head, base, axis="z", min_gap=0.22, positive_elem=shade_shell, negative_elem=base_disk)
        ctx.expect_within(head, base, axes="y", inner_elem=shade_shell, outer_elem=base_disk)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
