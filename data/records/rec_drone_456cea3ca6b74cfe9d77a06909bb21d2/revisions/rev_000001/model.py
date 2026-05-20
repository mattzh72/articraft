from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_propeller_mesh():
    blade_profile = [
        (0.012, -0.0055),
        (0.026, -0.0085),
        (0.052, -0.0095),
        (0.076, -0.0062),
        (0.086, -0.0022),
        (0.086, 0.0022),
        (0.076, 0.0062),
        (0.052, 0.0095),
        (0.026, 0.0085),
        (0.012, 0.0055),
    ]
    blade = ExtrudeGeometry(blade_profile, 0.003, center=True)
    blade.translate(0.0, 0.0, 0.0090)
    blade.rotate((0.0, 1.0, 0.0), 0.10, origin=(0.014, 0.0, 0.0090))

    propeller = blade
    propeller.merge(blade.copy().rotate_z(pi))
    return propeller


def _add_arm_visuals(part, *, arm_material, motor_material, accent_material) -> None:
    part.visual(
        Box((0.022, 0.022, 0.014)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=arm_material,
        name="root_knuckle",
    )
    part.visual(
        Box((0.014, 0.018, 0.005)),
        origin=Origin(xyz=(0.007, 0.0, 0.0065)),
        material=arm_material,
        name="hinge_fork_upper",
    )
    part.visual(
        Box((0.014, 0.018, 0.005)),
        origin=Origin(xyz=(0.007, 0.0, -0.0065)),
        material=arm_material,
        name="hinge_fork_lower",
    )
    part.visual(
        Cylinder(radius=0.0072, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=motor_material,
        name="hinge_barrel_upper",
    )
    part.visual(
        Cylinder(radius=0.0072, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0065)),
        material=motor_material,
        name="hinge_barrel_lower",
    )
    part.visual(
        Box((0.072, 0.018, 0.013)),
        origin=Origin(xyz=(0.050, 0.0, 0.001)),
        material=arm_material,
        name="inner_beam",
    )
    part.visual(
        Box((0.048, 0.014, 0.011)),
        origin=Origin(xyz=(0.096, 0.0, 0.003)),
        material=arm_material,
        name="outer_beam",
    )
    part.visual(
        Box((0.030, 0.016, 0.005)),
        origin=Origin(xyz=(0.090, 0.0, 0.0085)),
        material=accent_material,
        name="wire_cover",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.128, 0.0, 0.006)),
        material=motor_material,
        name="motor_mount",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.128, 0.0, -0.005)),
        material=motor_material,
        name="motor_can",
    )
    part.visual(
        Box((0.014, 0.020, 0.012)),
        origin=Origin(xyz=(0.124, 0.0, -0.021)),
        material=accent_material,
        name="landing_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_travel_quadcopter")

    body_gray = model.material("body_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_carbon = model.material("arm_carbon", rgba=(0.13, 0.13, 0.14, 1.0))
    motor_metal = model.material("motor_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.05, 0.06, 0.07, 1.0))

    body_shell_mesh = _save_mesh(
        "quadcopter_body_shell",
        ExtrudeGeometry(
            rounded_rect_profile(0.082, 0.190, 0.015, corner_segments=8),
            0.040,
            center=True,
        ),
    )
    propeller_blade_mesh = _save_mesh("travel_quadcopter_propeller_blades", _build_propeller_mesh())

    body = model.part("body")
    body.visual(body_shell_mesh, material=body_gray, name="body_shell")
    body.visual(
        Box((0.056, 0.104, 0.016)),
        origin=Origin(xyz=(0.0, 0.002, 0.020)),
        material=shell_dark,
        name="top_deck",
    )
    body.visual(
        Box((0.064, 0.096, 0.014)),
        origin=Origin(xyz=(0.0, -0.004, -0.020)),
        material=shell_dark,
        name="belly_pack",
    )
    body.visual(
        Box((0.030, 0.028, 0.015)),
        origin=Origin(xyz=(0.0, 0.078, -0.016)),
        material=shell_dark,
        name="nose_chin",
    )
    body.visual(
        Box((0.040, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.098, -0.002)),
        material=sensor_black,
        name="forward_sensor_bar",
    )
    body.visual(
        Box((0.034, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.098, 0.000)),
        material=shell_dark,
        name="rear_vent",
    )
    for pod_name, x, y, z, sx, sy, sz in [
        ("front_left_pod", -0.053, 0.042, 0.015, 0.030, 0.028, 0.020),
        ("front_right_pod", 0.053, 0.042, 0.015, 0.030, 0.028, 0.020),
        ("rear_left_pod", -0.053, -0.040, -0.012, 0.030, 0.026, 0.018),
        ("rear_right_pod", 0.053, -0.040, -0.012, 0.030, 0.026, 0.018),
    ]:
        body.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(x, y, z)),
            material=arm_carbon,
            name=pod_name,
        )
    for support_name, barrel_name, x, y, z, sx, sy in [
        ("front_left_hinge_support", "front_left_hinge_barrel", -0.072, 0.042, 0.015, 0.012, 0.018),
        ("front_right_hinge_support", "front_right_hinge_barrel", 0.072, 0.042, 0.015, 0.012, 0.018),
        ("rear_left_hinge_support", "rear_left_hinge_barrel", -0.072, -0.040, -0.012, 0.012, 0.017),
        ("rear_right_hinge_support", "rear_right_hinge_barrel", 0.072, -0.040, -0.012, 0.012, 0.017),
    ]:
        body.visual(
            Box((sx, sy, 0.0075)),
            origin=Origin(xyz=(x, y, z)),
            material=arm_carbon,
            name=support_name,
        )
        body.visual(
            Cylinder(radius=0.0072, length=0.008),
            origin=Origin(xyz=(x - 0.006 if x < 0.0 else x + 0.006, y, z)),
            material=motor_metal,
            name=barrel_name,
        )
    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.220, 0.055)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    arm_specs = [
        ("front_left_arm", "front_left_propeller"),
        ("front_right_arm", "front_right_propeller"),
        ("rear_left_arm", "rear_left_propeller"),
        ("rear_right_arm", "rear_right_propeller"),
    ]
    for arm_name, prop_name in arm_specs:
        arm = model.part(arm_name)
        _add_arm_visuals(
            arm,
            arm_material=arm_carbon,
            motor_material=motor_metal,
            accent_material=shell_dark,
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.150, 0.032, 0.045)),
            mass=0.09,
            origin=Origin(xyz=(0.075, 0.0, 0.0)),
        )

        propeller = model.part(prop_name)
        propeller.visual(propeller_blade_mesh, material=prop_black, name="blades")
        propeller.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=prop_black,
            name="prop_hub",
        )
        propeller.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=prop_black,
            name="hub_cap",
        )
        propeller.visual(
            Cylinder(radius=0.018, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.0085)),
            material=prop_black,
            name="blade_root_plate",
        )
        propeller.inertial = Inertial.from_geometry(
            Cylinder(radius=0.046, length=0.010),
            mass=0.014,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
        )

    front_left_arm = model.get_part("front_left_arm")
    front_right_arm = model.get_part("front_right_arm")
    rear_left_arm = model.get_part("rear_left_arm")
    rear_right_arm = model.get_part("rear_right_arm")

    front_left_propeller = model.get_part("front_left_propeller")
    front_right_propeller = model.get_part("front_right_propeller")
    rear_left_propeller = model.get_part("rear_left_propeller")
    rear_right_propeller = model.get_part("rear_right_propeller")

    fold_limits_front = MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=3.0 * pi / 4.0)
    fold_limits_rear = MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=pi / 4.0)

    model.articulation(
        "front_left_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_left_arm,
        origin=Origin(xyz=(-0.078, 0.042, 0.015), rpy=(0.0, 0.0, 3.0 * pi / 4.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=fold_limits_front,
    )
    model.articulation(
        "front_right_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_right_arm,
        origin=Origin(xyz=(0.078, 0.042, 0.015), rpy=(0.0, 0.0, pi / 4.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=fold_limits_front,
    )
    model.articulation(
        "rear_left_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_left_arm,
        origin=Origin(xyz=(-0.078, -0.040, -0.012), rpy=(0.0, 0.0, -3.0 * pi / 4.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=fold_limits_rear,
    )
    model.articulation(
        "rear_right_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_right_arm,
        origin=Origin(xyz=(0.078, -0.040, -0.012), rpy=(0.0, 0.0, -pi / 4.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=fold_limits_rear,
    )

    for joint_name, parent_name, child_name in [
        ("front_left_spin", "front_left_arm", "front_left_propeller"),
        ("front_right_spin", "front_right_arm", "front_right_propeller"),
        ("rear_left_spin", "rear_left_arm", "rear_left_propeller"),
        ("rear_right_spin", "rear_right_arm", "rear_right_propeller"),
    ]:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent_name,
            child=child_name,
            origin=Origin(xyz=(0.128, 0.0, 0.012)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=140.0),
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

    body = object_model.get_part("body")

    front_left_arm = object_model.get_part("front_left_arm")
    front_right_arm = object_model.get_part("front_right_arm")
    rear_left_arm = object_model.get_part("rear_left_arm")
    rear_right_arm = object_model.get_part("rear_right_arm")

    front_left_propeller = object_model.get_part("front_left_propeller")
    front_right_propeller = object_model.get_part("front_right_propeller")
    rear_left_propeller = object_model.get_part("rear_left_propeller")
    rear_right_propeller = object_model.get_part("rear_right_propeller")

    front_left_fold = object_model.get_articulation("front_left_fold")
    front_right_fold = object_model.get_articulation("front_right_fold")
    rear_left_fold = object_model.get_articulation("rear_left_fold")
    rear_right_fold = object_model.get_articulation("rear_right_fold")

    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")

    for joint_name, joint in [
        ("front_left_fold", front_left_fold),
        ("front_right_fold", front_right_fold),
        ("rear_left_fold", rear_left_fold),
        ("rear_right_fold", rear_right_fold),
    ]:
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is a vertical folding hinge",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper > 0.6
            and abs(abs(joint.axis[2]) - 1.0) < 1e-9,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    for joint_name, joint in [
        ("front_left_spin", front_left_spin),
        ("front_right_spin", front_right_spin),
        ("rear_left_spin", rear_left_spin),
        ("rear_right_spin", rear_right_spin),
    ]:
        ctx.check(
            f"{joint_name} is a vertical propeller spin joint",
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    for propeller, arm, label in [
        (front_left_propeller, front_left_arm, "front left"),
        (front_right_propeller, front_right_arm, "front right"),
        (rear_left_propeller, rear_left_arm, "rear left"),
        (rear_right_propeller, rear_right_arm, "rear right"),
    ]:
        ctx.expect_contact(
            propeller,
            arm,
            elem_a="prop_hub",
            elem_b="motor_mount",
            contact_tol=0.0002,
            name=f"{label} propeller hub seats on its motor",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    open_motor_centers = {}
    for name, arm in [
        ("front_left", front_left_arm),
        ("front_right", front_right_arm),
        ("rear_left", rear_left_arm),
        ("rear_right", rear_right_arm),
    ]:
        open_motor_centers[name] = _aabb_center(ctx.part_element_world_aabb(arm, elem="motor_mount"))

    folded_pose = {
        front_left_fold: front_left_fold.motion_limits.upper,
        front_right_fold: front_right_fold.motion_limits.upper,
        rear_left_fold: rear_left_fold.motion_limits.upper,
        rear_right_fold: rear_right_fold.motion_limits.upper,
    }

    with ctx.pose(folded_pose):
        folded_motor_centers = {}
        for name, arm in [
            ("front_left", front_left_arm),
            ("front_right", front_right_arm),
            ("rear_left", rear_left_arm),
            ("rear_right", rear_right_arm),
        ]:
            folded_motor_centers[name] = _aabb_center(ctx.part_element_world_aabb(arm, elem="motor_mount"))

        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            ctx.check(
                f"{name} arm folds rearward",
                open_motor_centers[name] is not None
                and folded_motor_centers[name] is not None
                and folded_motor_centers[name][1] < open_motor_centers[name][1] - 0.02,
                details=f"open={open_motor_centers[name]}, folded={folded_motor_centers[name]}",
            )

        ctx.expect_gap(
            body,
            front_left_arm,
            axis="x",
            positive_elem="body_shell",
            negative_elem="outer_beam",
            min_gap=0.002,
            name="front left folded arm clears the left body side",
        )
        ctx.expect_gap(
            body,
            rear_left_arm,
            axis="x",
            positive_elem="body_shell",
            negative_elem="outer_beam",
            min_gap=0.002,
            name="rear left folded arm clears the left body side",
        )
        ctx.expect_gap(
            front_right_arm,
            body,
            axis="x",
            positive_elem="outer_beam",
            negative_elem="body_shell",
            min_gap=0.002,
            name="front right folded arm clears the right body side",
        )
        ctx.expect_gap(
            rear_right_arm,
            body,
            axis="x",
            positive_elem="outer_beam",
            negative_elem="body_shell",
            min_gap=0.002,
            name="rear right folded arm clears the right body side",
        )
        ctx.expect_gap(
            front_left_arm,
            rear_left_arm,
            axis="z",
            positive_elem="motor_mount",
            negative_elem="motor_mount",
            min_gap=0.008,
            name="left folded motor pair stays vertically staggered",
        )
        ctx.expect_gap(
            front_right_arm,
            rear_right_arm,
            axis="z",
            positive_elem="motor_mount",
            negative_elem="motor_mount",
            min_gap=0.008,
            name="right folded motor pair stays vertically staggered",
        )
        ctx.expect_gap(
            front_left_propeller,
            rear_left_propeller,
            axis="z",
            positive_elem="prop_hub",
            negative_elem="prop_hub",
            min_gap=0.020,
            name="left folded propeller hubs are vertically separated",
        )
        ctx.expect_gap(
            front_right_propeller,
            rear_right_propeller,
            axis="z",
            positive_elem="prop_hub",
            negative_elem="prop_hub",
            min_gap=0.020,
            name="right folded propeller hubs are vertically separated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
