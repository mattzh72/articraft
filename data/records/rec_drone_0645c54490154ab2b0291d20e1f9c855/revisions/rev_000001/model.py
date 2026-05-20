from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            height,
            center=True,
        ),
        name,
    )


def _yaw_offset(origin_xyz: tuple[float, float, float], local_xyz: tuple[float, float, float], yaw: float):
    lx, ly, lz = local_xyz
    ox, oy, oz = origin_xyz
    return (
        ox + lx * cos(yaw) - ly * sin(yaw),
        oy + lx * sin(yaw) + ly * cos(yaw),
        oz + lz,
    )


def _prop_blade_mesh(name: str):
    blade_profile = [
        (-0.145, -0.003),
        (-0.102, -0.006),
        (-0.038, -0.011),
        (0.038, -0.011),
        (0.102, -0.006),
        (0.145, -0.003),
        (0.145, 0.003),
        (0.102, 0.006),
        (0.038, 0.011),
        (-0.038, 0.011),
        (-0.102, 0.006),
        (-0.145, 0.003),
    ]
    return mesh_from_geometry(
        ExtrudeGeometry(
            blade_profile,
            0.003,
            center=True,
        ),
        name,
    )


def _build_arm(part, *, arm_material, motor_material) -> None:
    part.visual(
        Box((0.038, 0.036, 0.022)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=arm_material,
        name="hinge_root",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=arm_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.220, 0.030, 0.020)),
        origin=Origin(xyz=(0.148, 0.0, 0.0)),
        material=arm_material,
        name="boom",
    )
    part.visual(
        Box((0.104, 0.020, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, -0.010)),
        material=arm_material,
        name="lower_brace",
    )
    part.visual(
        Box((0.086, 0.024, 0.016)),
        origin=Origin(xyz=(0.094, 0.0, 0.013)),
        material=arm_material,
        name="upper_fairing",
    )
    part.visual(
        Box((0.060, 0.032, 0.015)),
        origin=Origin(xyz=(0.222, 0.0, -0.003)),
        material=arm_material,
        name="end_plate",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.268, 0.0, 0.018)),
        material=motor_material,
        name="motor_can",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.268, 0.0, 0.002)),
        material=arm_material,
        name="motor_mount",
    )


def _build_propeller(part, *, hub_material, blade_material) -> None:
    part.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=hub_material,
        name="shaft",
    )
    part.visual(
        _prop_blade_mesh(f"{part.name}_blade_x_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(0.10, 0.03, 0.0)),
        material=blade_material,
        name="blade_x",
    )
    part.visual(
        _prop_blade_mesh(f"{part.name}_blade_y_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(0.10, -0.03, pi / 2.0)),
        material=blade_material,
        name="blade_y",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="delivery_drone")

    body_gray = model.material("body_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    body_dark = model.material("body_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    carbon = model.material("carbon", rgba=(0.10, 0.10, 0.11, 1.0))
    prop_black = model.material("prop_black", rgba=(0.05, 0.05, 0.06, 1.0))
    motor_silver = model.material("motor_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.86, 0.43, 0.10, 1.0))
    gear_gray = model.material("gear_gray", rgba=(0.24, 0.25, 0.27, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(Box((0.60, 0.44, 0.20)), mass=7.5)
    body.visual(
        _rounded_box_mesh(0.440, 0.300, 0.090, 0.050, "body_shell"),
        material=body_gray,
        name="body_shell",
    )
    body.visual(
        _rounded_box_mesh(0.300, 0.190, 0.040, 0.035, "top_cowl"),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=body_dark,
        name="top_cowl",
    )
    body.visual(
        _rounded_box_mesh(0.210, 0.170, 0.060, 0.020, "payload_belly"),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=body_dark,
        name="payload_belly",
    )
    body.visual(
        Box((0.140, 0.100, 0.020)),
        origin=Origin(xyz=(0.110, 0.0, 0.046)),
        material=accent_orange,
        name="nose_panel",
    )
    body.visual(
        Box((0.180, 0.220, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.073)),
        material=body_gray,
        name="gear_mount",
    )

    arm_specs = (
        {
            "arm_name": "front_left_arm",
            "prop_name": "front_left_propeller",
            "arm_joint": "body_to_front_left_arm",
            "prop_joint": "front_left_prop_spin",
            "hinge_xyz": (0.170, 0.160, 0.045),
            "hinge_yaw": 0.72,
            "arm_axis": (0.0, 0.0, 1.0),
            "arm_limits": MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.18),
        },
        {
            "arm_name": "front_right_arm",
            "prop_name": "front_right_propeller",
            "arm_joint": "body_to_front_right_arm",
            "prop_joint": "front_right_prop_spin",
            "hinge_xyz": (0.170, -0.160, 0.045),
            "hinge_yaw": -0.72,
            "arm_axis": (0.0, 0.0, 1.0),
            "arm_limits": MotionLimits(effort=8.0, velocity=2.5, lower=-1.18, upper=0.0),
        },
        {
            "arm_name": "rear_left_arm",
            "prop_name": "rear_left_propeller",
            "arm_joint": "body_to_rear_left_arm",
            "prop_joint": "rear_left_prop_spin",
            "hinge_xyz": (-0.170, 0.160, 0.045),
            "hinge_yaw": pi - 0.72,
            "arm_axis": (0.0, 0.0, 1.0),
            "arm_limits": MotionLimits(effort=8.0, velocity=2.5, lower=-1.18, upper=0.0),
        },
        {
            "arm_name": "rear_right_arm",
            "prop_name": "rear_right_propeller",
            "arm_joint": "body_to_rear_right_arm",
            "prop_joint": "rear_right_prop_spin",
            "hinge_xyz": (-0.170, -0.160, 0.045),
            "hinge_yaw": -pi + 0.72,
            "arm_axis": (0.0, 0.0, 1.0),
            "arm_limits": MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.18),
        },
    )

    for spec in arm_specs:
        body.visual(
            Box((0.040, 0.058, 0.020)),
            origin=Origin(
                xyz=_yaw_offset(spec["hinge_xyz"], (-0.030, 0.0, 0.0), spec["hinge_yaw"]),
                rpy=(0.0, 0.0, spec["hinge_yaw"]),
            ),
            material=body_gray,
            name=f'{spec["arm_name"]}_hinge_base',
        )
        for cheek_name, local_y in (("pos", 0.026), ("neg", -0.026)):
            body.visual(
                Box((0.016, 0.014, 0.030)),
                origin=Origin(
                    xyz=_yaw_offset(spec["hinge_xyz"], (-0.008, local_y, 0.0), spec["hinge_yaw"]),
                    rpy=(0.0, 0.0, spec["hinge_yaw"]),
                ),
                material=body_gray,
                name=f'{spec["arm_name"]}_hinge_cheek_{cheek_name}',
            )

    for spec in arm_specs:
        arm = model.part(spec["arm_name"])
        arm.inertial = Inertial.from_geometry(
            Box((0.310, 0.080, 0.070)),
            mass=0.65,
            origin=Origin(xyz=(0.155, 0.0, 0.018)),
        )
        _build_arm(arm, arm_material=carbon, motor_material=motor_silver)

        propeller = model.part(spec["prop_name"])
        propeller.inertial = Inertial.from_geometry(
            Cylinder(radius=0.150, length=0.016),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )
        _build_propeller(propeller, hub_material=body_dark, blade_material=prop_black)

        model.articulation(
            spec["arm_joint"],
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=spec["hinge_xyz"], rpy=(0.0, 0.0, spec["hinge_yaw"])),
            axis=spec["arm_axis"],
            motion_limits=spec["arm_limits"],
        )
        model.articulation(
            spec["prop_joint"],
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(0.268, 0.0, 0.036)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.5, velocity=80.0),
        )

    landing_gear = model.part("landing_gear")
    landing_gear.inertial = Inertial.from_geometry(
        Box((0.32, 0.24, 0.22)),
        mass=1.4,
        origin=Origin(xyz=(-0.090, 0.0, -0.100)),
    )
    landing_gear.visual(
        Box((0.120, 0.220, 0.024)),
        origin=Origin(xyz=(-0.060, 0.0, -0.012)),
        material=gear_gray,
        name="pivot_spine",
    )
    landing_gear.visual(
        Box((0.130, 0.220, 0.012)),
        origin=Origin(xyz=(-0.085, 0.0, -0.105)),
        material=gear_gray,
        name="payload_plate",
    )
    for side_name, y_pos in (("left", 0.100), ("right", -0.100)):
        landing_gear.visual(
            Box((0.022, 0.022, 0.170)),
            origin=Origin(xyz=(-0.080, y_pos, -0.096)),
            material=gear_gray,
            name=f"{side_name}_leg",
        )
        landing_gear.visual(
            Cylinder(radius=0.011, length=0.280),
            origin=Origin(xyz=(-0.090, y_pos, -0.180), rpy=(0.0, pi / 2.0, 0.0)),
            material=gear_gray,
            name=f"{side_name}_runner",
        )
    for end_name, x_pos in (("front", 0.015), ("rear", -0.195)):
        landing_gear.visual(
            Cylinder(radius=0.010, length=0.200),
            origin=Origin(xyz=(x_pos, 0.0, -0.180), rpy=(pi / 2.0, 0.0, 0.0)),
            material=gear_gray,
            name=f"{end_name}_crossbar",
        )

    model.articulation(
        "body_to_landing_gear",
        ArticulationType.REVOLUTE,
        parent=body,
        child=landing_gear,
        origin=Origin(xyz=(0.080, 0.0, -0.085)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=0.78),
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
    landing_gear = object_model.get_part("landing_gear")
    gear_joint = object_model.get_articulation("body_to_landing_gear")

    arm_expectations = (
        ("body_to_front_left_arm", "front_left_propeller", (0.0, 0.0, 1.0), 0.95, "x", "decrease"),
        ("body_to_front_right_arm", "front_right_propeller", (0.0, 0.0, 1.0), -0.95, "x", "decrease"),
        ("body_to_rear_left_arm", "rear_left_propeller", (0.0, 0.0, 1.0), -0.95, "x", "increase"),
        ("body_to_rear_right_arm", "rear_right_propeller", (0.0, 0.0, 1.0), 0.95, "x", "increase"),
    )

    for joint_name, prop_name, axis, fold_q, moving_axis, motion in arm_expectations:
        joint = object_model.get_articulation(joint_name)
        propeller = object_model.get_part(prop_name)
        ctx.check(
            f"{joint_name} is a revolute fold hinge",
            joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == axis,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        rest_pos = ctx.part_world_position(propeller)
        with ctx.pose({joint: fold_q}):
            folded_pos = ctx.part_world_position(propeller)
        rest_axis = None if rest_pos is None else rest_pos[0 if moving_axis == "x" else 1]
        folded_axis = None if folded_pos is None else folded_pos[0 if moving_axis == "x" else 1]
        if motion == "decrease":
            moved_ok = (
                rest_axis is not None
                and folded_axis is not None
                and folded_axis < rest_axis - 0.10
            )
        else:
            moved_ok = (
                rest_axis is not None
                and folded_axis is not None
                and folded_axis > rest_axis + 0.10
            )
        ctx.check(
            f"{joint_name} folds the arm inward",
            moved_ok,
            details=f"rest={rest_pos}, folded={folded_pos}",
        )

    for prop_joint_name in (
        "front_left_prop_spin",
        "front_right_prop_spin",
        "rear_left_prop_spin",
        "rear_right_prop_spin",
    ):
        prop_joint = object_model.get_articulation(prop_joint_name)
        limits = prop_joint.motion_limits
        ctx.check(
            f"{prop_joint_name} is continuous vertical spin",
            prop_joint.articulation_type == ArticulationType.CONTINUOUS
            and prop_joint.axis == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={prop_joint.articulation_type}, axis={prop_joint.axis}, "
                f"limits=({None if limits is None else limits.lower}, "
                f"{None if limits is None else limits.upper})"
            ),
        )

    ctx.check(
        "landing gear hinge pitches about body lateral axis",
        gear_joint.articulation_type == ArticulationType.REVOLUTE and gear_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={gear_joint.articulation_type}, axis={gear_joint.axis}",
    )

    with ctx.pose({gear_joint: 0.0}):
        ctx.expect_overlap(
            landing_gear,
            body,
            axes="xy",
            min_overlap=0.12,
            name="landing gear stays centered under the body",
        )
        ctx.expect_gap(
            body,
            landing_gear,
            axis="z",
            max_gap=0.004,
            max_penetration=0.001,
            name="landing gear pivots from the belly mount",
        )
        deployed_aabb = ctx.part_world_aabb(landing_gear)

    with ctx.pose({gear_joint: 0.72}):
        retracted_aabb = ctx.part_world_aabb(landing_gear)

    ctx.check(
        "landing gear retracts upward",
        deployed_aabb is not None
        and retracted_aabb is not None
        and retracted_aabb[1][2] > deployed_aabb[1][2] + 0.06,
        details=f"deployed={deployed_aabb}, retracted={retracted_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
