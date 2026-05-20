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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_HALF_X = 0.070
BODY_HALF_Y = 0.052
CORNER_CUT_RADIUS = 0.022
TOP_PLATE_Z0 = 0.003
TOP_PLATE_THICKNESS = 0.002
BODY_RAIL_Z = -0.001
BODY_RAIL_HEIGHT = 0.008
BATTERY_FLOOR_Z = -0.001
BATTERY_FLOOR_THICKNESS = 0.002
BATTERY_WALL_Z = 0.0005
BATTERY_WALL_HEIGHT = 0.005

ARM_HINGE_Z = 0.0084
BODY_KNUCKLE_RADIUS = 0.0042
BODY_KNUCKLE_HEIGHT = 0.0022
ARM_COLLAR_RADIUS = 0.0078
ARM_COLLAR_HEIGHT = 0.0044
ARM_TUBE_START = 0.018
ARM_TUBE_LENGTH = 0.096
ARM_TUBE_WIDTH = 0.016
ARM_TUBE_HEIGHT = 0.008
ARM_TUBE_WALL = 0.0022
ARM_TUBE_REL_Z = 0.0048
MOTOR_RADIUS = 0.011
MOTOR_HEIGHT = 0.010
MOTOR_X = ARM_TUBE_START + ARM_TUBE_LENGTH
MOTOR_REL_Z = 0.0095
PROP_HUB_RADIUS = 0.009
PROP_HUB_THICKNESS = 0.002
PROP_BLADE_THICKNESS = 0.0012

CAMERA_HINGE_Y = BODY_HALF_Y - 0.007
CAMERA_HINGE_Z = -0.001
CAMERA_KNUCKLE_RADIUS = 0.0045
CAMERA_KNUCKLE_LENGTH = 0.009
CAMERA_KNUCKLE_OFFSET_X = 0.0072
CAMERA_COLLAR_RADIUS = 0.0064
CAMERA_COLLAR_LENGTH = 0.0054
CAMERA_DEFAULT_TILT = -0.35

ARM_CONFIGS = (
    ("front_right", (0.050, 0.034), math.pi / 4.0),
    ("front_left", (-0.050, 0.034), 3.0 * math.pi / 4.0),
    ("rear_left", (-0.050, -0.034), -3.0 * math.pi / 4.0),
    ("rear_right", (0.050, -0.034), -math.pi / 4.0),
)


def _append_arc(
    points: list[tuple[float, float]],
    center: tuple[float, float],
    radius: float,
    start_angle: float,
    end_angle: float,
    steps: int,
) -> None:
    for index in range(steps + 1):
        t = index / steps
        angle = start_angle + (end_angle - start_angle) * t
        point = (
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle),
        )
        if not points or point != points[-1]:
            points.append(point)


def _corner_cut_profile(
    half_x: float,
    half_y: float,
    cut_radius: float,
    arc_steps: int = 6,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = [(half_x, half_y - cut_radius), (half_x, -half_y + cut_radius)]
    _append_arc(profile, (half_x, -half_y), cut_radius, math.pi / 2.0, math.pi, arc_steps)
    profile.append((-half_x + cut_radius, -half_y))
    _append_arc(profile, (-half_x, -half_y), cut_radius, 0.0, math.pi / 2.0, arc_steps)
    profile.append((-half_x, half_y - cut_radius))
    _append_arc(profile, (-half_x, half_y), cut_radius, -math.pi / 2.0, 0.0, arc_steps)
    profile.append((half_x - cut_radius, half_y))
    _append_arc(profile, (half_x, half_y), cut_radius, math.pi, 3.0 * math.pi / 2.0, arc_steps)
    return profile


def _build_body_plate_mesh():
    profile = _corner_cut_profile(BODY_HALF_X, BODY_HALF_Y, CORNER_CUT_RADIUS)
    geometry = ExtrudeGeometry.from_z0(profile, TOP_PLATE_THICKNESS)
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "quad_body_plate.obj")


def _build_arm_tube_mesh():
    outer_profile = [
        (-ARM_TUBE_HEIGHT / 2.0, -ARM_TUBE_WIDTH / 2.0),
        (ARM_TUBE_HEIGHT / 2.0, -ARM_TUBE_WIDTH / 2.0),
        (ARM_TUBE_HEIGHT / 2.0, ARM_TUBE_WIDTH / 2.0),
        (-ARM_TUBE_HEIGHT / 2.0, ARM_TUBE_WIDTH / 2.0),
    ]
    inner_profile = [
        (-(ARM_TUBE_HEIGHT / 2.0 - ARM_TUBE_WALL), -(ARM_TUBE_WIDTH / 2.0 - ARM_TUBE_WALL)),
        ((ARM_TUBE_HEIGHT / 2.0 - ARM_TUBE_WALL), -(ARM_TUBE_WIDTH / 2.0 - ARM_TUBE_WALL)),
        ((ARM_TUBE_HEIGHT / 2.0 - ARM_TUBE_WALL), (ARM_TUBE_WIDTH / 2.0 - ARM_TUBE_WALL)),
        (-(ARM_TUBE_HEIGHT / 2.0 - ARM_TUBE_WALL), (ARM_TUBE_WIDTH / 2.0 - ARM_TUBE_WALL)),
    ]
    geometry = ExtrudeWithHolesGeometry(
        outer_profile=outer_profile,
        hole_profiles=[inner_profile],
        height=ARM_TUBE_LENGTH,
        center=False,
    )
    geometry.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "quad_arm_tube.obj")


def _build_prop_blade_mesh():
    blade_profile = [
        (0.000, -0.0036),
        (0.016, -0.0030),
        (0.031, -0.0018),
        (0.043, -0.0007),
        (0.047, 0.0000),
        (0.043, 0.0007),
        (0.031, 0.0018),
        (0.016, 0.0030),
        (0.000, 0.0036),
    ]
    geometry = ExtrudeGeometry.from_z0(blade_profile, PROP_BLADE_THICKNESS)
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "quad_prop_blade.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_racing_quadrotor", assets=ASSETS)

    carbon = model.material("carbon_fiber_composite", rgba=(0.12, 0.12, 0.13, 1.0))
    carbon_shadow = model.material("carbon_shadow", rgba=(0.08, 0.08, 0.09, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    matte_black = model.material("matte_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.37, 0.40, 1.0))
    camera_gray = model.material("camera_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    lens_black = model.material("lens_black", rgba=(0.04, 0.04, 0.05, 1.0))

    body_plate_mesh = _build_body_plate_mesh()
    arm_tube_mesh = _build_arm_tube_mesh()
    blade_mesh = _build_prop_blade_mesh()

    body = model.part("body")
    body.visual(body_plate_mesh, origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z0)), material=carbon, name="hub_plate")
    body.visual(Box((0.080, 0.014, BODY_RAIL_HEIGHT)), origin=Origin(xyz=(0.0, 0.029, BODY_RAIL_Z)), material=carbon, name="front_rail")
    body.visual(Box((0.080, 0.014, BODY_RAIL_HEIGHT)), origin=Origin(xyz=(0.0, -0.029, BODY_RAIL_Z)), material=carbon, name="rear_rail")
    body.visual(Box((0.014, 0.054, BODY_RAIL_HEIGHT)), origin=Origin(xyz=(0.046, 0.0, BODY_RAIL_Z)), material=carbon, name="right_rail")
    body.visual(Box((0.014, 0.054, BODY_RAIL_HEIGHT)), origin=Origin(xyz=(-0.046, 0.0, BODY_RAIL_Z)), material=carbon, name="left_rail")
    body.visual(Box((0.052, 0.032, BATTERY_FLOOR_THICKNESS)), origin=Origin(xyz=(0.0, 0.0, BATTERY_FLOOR_Z)), material=carbon_shadow, name="battery_floor")
    body.visual(Box((0.052, 0.003, BATTERY_WALL_HEIGHT)), origin=Origin(xyz=(0.0, 0.0175, BATTERY_WALL_Z)), material=carbon, name="battery_front_wall")
    body.visual(Box((0.052, 0.003, BATTERY_WALL_HEIGHT)), origin=Origin(xyz=(0.0, -0.0175, BATTERY_WALL_Z)), material=carbon, name="battery_rear_wall")
    body.visual(Box((0.003, 0.032, BATTERY_WALL_HEIGHT)), origin=Origin(xyz=(0.0275, 0.0, BATTERY_WALL_Z)), material=carbon, name="battery_right_wall")
    body.visual(Box((0.003, 0.032, BATTERY_WALL_HEIGHT)), origin=Origin(xyz=(-0.0275, 0.0, BATTERY_WALL_Z)), material=carbon, name="battery_left_wall")
    body.visual(Box((0.016, 0.008, 0.004)), origin=Origin(xyz=(0.0, CAMERA_HINGE_Y - 0.011, 0.0010)), material=carbon, name="nose_bridge")
    body.visual(
        Cylinder(radius=CAMERA_KNUCKLE_RADIUS, length=CAMERA_KNUCKLE_LENGTH),
        origin=Origin(xyz=(-CAMERA_KNUCKLE_OFFSET_X, CAMERA_HINGE_Y, CAMERA_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="camera_knuckle_left",
    )
    body.visual(
        Cylinder(radius=CAMERA_KNUCKLE_RADIUS, length=CAMERA_KNUCKLE_LENGTH),
        origin=Origin(xyz=(CAMERA_KNUCKLE_OFFSET_X, CAMERA_HINGE_Y, CAMERA_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="camera_knuckle_right",
    )

    for arm_name, (root_x, root_y), _yaw in ARM_CONFIGS:
        body.visual(
            Box((0.018, 0.018, 0.004)),
            origin=Origin(xyz=(root_x, root_y, 0.0052)),
            material=carbon,
            name=f"{arm_name}_root_pad",
        )
        body.visual(
            Cylinder(radius=BODY_KNUCKLE_RADIUS, length=BODY_KNUCKLE_HEIGHT),
            origin=Origin(xyz=(root_x, root_y, ARM_HINGE_Z - (ARM_COLLAR_HEIGHT + BODY_KNUCKLE_HEIGHT) / 2.0)),
            material=black_plastic,
            name=f"{arm_name}_knuckle_lower",
        )
        arm = model.part(f"{arm_name}_arm")
        arm.visual(Cylinder(radius=ARM_COLLAR_RADIUS, length=ARM_COLLAR_HEIGHT), material=black_plastic, name="hinge_collar")
        arm.visual(Box((0.018, 0.008, 0.004)), origin=Origin(xyz=(0.009, 0.0, 0.0)), material=black_plastic, name="root_bridge")
        arm.visual(arm_tube_mesh, origin=Origin(xyz=(ARM_TUBE_START, 0.0, ARM_TUBE_REL_Z)), material=carbon, name="arm_tube")
        arm.visual(
            Cylinder(radius=MOTOR_RADIUS, length=MOTOR_HEIGHT),
            origin=Origin(xyz=(MOTOR_X, 0.0, MOTOR_REL_Z)),
            material=dark_metal,
            name="motor_pod",
        )

        model.articulation(
            f"{arm_name}_fold",
            ArticulationType.REVOLUTE,
            parent=body,
            child=arm,
            origin=Origin(xyz=(root_x, root_y, ARM_HINGE_Z), rpy=(0.0, 0.0, _yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=2.4, lower=0.0, upper=1.6),
        )

        propeller = model.part(f"{arm_name}_propeller")
        propeller.visual(
            Cylinder(radius=PROP_HUB_RADIUS, length=PROP_HUB_THICKNESS),
            origin=Origin(xyz=(0.0, 0.0, PROP_HUB_THICKNESS / 2.0)),
            material=black_plastic,
            name="prop_hub",
        )
        propeller.visual(blade_mesh, origin=Origin(xyz=(0.0, 0.0, PROP_HUB_THICKNESS)), material=matte_black, name="blade_a")
        propeller.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, PROP_HUB_THICKNESS), rpy=(0.0, 0.0, math.pi)),
            material=matte_black,
            name="blade_b",
        )

        model.articulation(
            f"{arm_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=propeller,
            origin=Origin(xyz=(MOTOR_X, 0.0, MOTOR_REL_Z + MOTOR_HEIGHT / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=120.0),
        )

    camera_mount = model.part("camera_mount")
    camera_mount.visual(
        Cylinder(radius=CAMERA_COLLAR_RADIUS, length=CAMERA_COLLAR_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="tilt_collar",
    )
    camera_mount.visual(Box((0.008, 0.010, 0.010)), origin=Origin(xyz=(0.0, 0.004, -0.006)), material=black_plastic, name="tilt_stem")
    camera_mount.visual(Box((0.012, 0.026, 0.004)), origin=Origin(xyz=(0.0, 0.017, -0.010)), material=black_plastic, name="bracket_spine")
    camera_mount.visual(Box((0.003, 0.018, 0.018)), origin=Origin(xyz=(-0.010, 0.025, -0.011)), material=black_plastic, name="bracket_left_cheek")
    camera_mount.visual(Box((0.003, 0.018, 0.018)), origin=Origin(xyz=(0.010, 0.025, -0.011)), material=black_plastic, name="bracket_right_cheek")
    camera_mount.visual(Box((0.018, 0.018, 0.016)), origin=Origin(xyz=(0.0, 0.036, -0.015)), material=camera_gray, name="camera_body")
    camera_mount.visual(Box((0.010, 0.004, 0.010)), origin=Origin(xyz=(0.0, 0.047, -0.015)), material=lens_black, name="camera_lens")

    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=camera_mount,
        origin=Origin(xyz=(0.0, CAMERA_HINGE_Y, CAMERA_HINGE_Z), rpy=(CAMERA_DEFAULT_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=2.0, lower=-0.25, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    hub_plate = body.get_visual("hub_plate")
    battery_floor = body.get_visual("battery_floor")
    camera_mount = object_model.get_part("camera_mount")
    camera_tilt = object_model.get_articulation("camera_tilt")
    camera_body = camera_mount.get_visual("camera_body")
    camera_collar = camera_mount.get_visual("tilt_collar")
    camera_knuckle_left = body.get_visual("camera_knuckle_left")
    camera_knuckle_right = body.get_visual("camera_knuckle_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=0.0025,
        positive_elem=hub_plate,
        negative_elem=battery_floor,
        name="battery_bay_recess_depth",
    )

    for arm_name, _root, _yaw in ARM_CONFIGS:
        arm = object_model.get_part(f"{arm_name}_arm")
        propeller = object_model.get_part(f"{arm_name}_propeller")
        fold_joint = object_model.get_articulation(f"{arm_name}_fold")
        arm_tube = arm.get_visual("arm_tube")
        arm_collar = arm.get_visual("hinge_collar")
        motor_pod = arm.get_visual("motor_pod")
        prop_hub = propeller.get_visual("prop_hub")
        body_lower = body.get_visual(f"{arm_name}_knuckle_lower")

        ctx.expect_contact(body, arm, elem_a=body_lower, elem_b=arm_collar, name=f"{arm_name}_lower_knuckle_contact")
        ctx.expect_overlap(body, arm, axes="xy", min_overlap=0.006, name=f"{arm_name}_root_overlap")
        ctx.expect_gap(
            arm,
            body,
            axis="z",
            min_gap=0.003,
            positive_elem=arm_tube,
            negative_elem=hub_plate,
            name=f"{arm_name}_tube_clear_of_hub",
        )
        ctx.expect_contact(arm, propeller, elem_a=motor_pod, elem_b=prop_hub, name=f"{arm_name}_prop_seated_on_motor")

        with ctx.pose({fold_joint: 1.5}):
            ctx.expect_overlap(arm, body, axes="xy", min_overlap=0.024, name=f"{arm_name}_folds_inward_over_hub")
            ctx.expect_gap(
                arm,
                body,
                axis="z",
                min_gap=0.002,
                positive_elem=arm_tube,
                negative_elem=hub_plate,
                name=f"{arm_name}_folded_clearance_above_hub",
            )

    ctx.expect_contact(body, camera_mount, elem_a=camera_knuckle_left, elem_b=camera_collar, name="camera_left_knuckle_contact")
    ctx.expect_contact(body, camera_mount, elem_a=camera_knuckle_right, elem_b=camera_collar, name="camera_right_knuckle_contact")
    ctx.expect_origin_distance(camera_mount, body, axes="x", max_dist=0.01)
    ctx.expect_gap(
        body,
        camera_mount,
        axis="z",
        min_gap=0.003,
        positive_elem=hub_plate,
        negative_elem=camera_body,
        name="camera_below_nose",
    )

    with ctx.pose({camera_tilt: 0.35}):
        ctx.expect_gap(
            body,
            camera_mount,
            axis="z",
            min_gap=0.0015,
            positive_elem=hub_plate,
            negative_elem=camera_body,
            name="camera_tilt_clearance",
        )
        ctx.expect_origin_distance(camera_mount, body, axes="x", max_dist=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
