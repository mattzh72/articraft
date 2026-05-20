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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

DECK_LENGTH = 0.90
DECK_WIDTH = 0.56
DECK_THICKNESS = 0.016
DECK_TOP = 0.145
DECK_CENTER_Z = DECK_TOP - (DECK_THICKNESS * 0.5)

CORNER_X = 0.370
CORNER_Y = 0.215
SWIVEL_Z = 0.121

WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.026
WHEEL_CENTER_DROP = (SWIVEL_Z - WHEEL_RADIUS) + 0.004
CASTER_TRAIL = 0.022

HANDLE_HALF_WIDTH = 0.170
HANDLE_MOUNT_X = -0.430
HANDLE_MOUNT_Z = 0.153


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_wheel_visuals(part, *, tire_material, core_material, mesh_prefix: str) -> None:
    wheel_profile = [
        (0.004, -0.010),
        (0.018, -0.010),
        (0.028, -0.013),
        (0.046, -0.013),
        (0.052, -0.011),
        (0.055, -0.006),
        (0.055, 0.006),
        (0.052, 0.011),
        (0.046, 0.013),
        (0.028, 0.013),
        (0.018, 0.010),
        (0.004, 0.010),
    ]
    wheel_mesh = _save_mesh(
        f"{mesh_prefix}_wheel.obj",
        LatheGeometry(wheel_profile, segments=48).rotate_x(math.pi / 2.0),
    )
    part.visual(
        wheel_mesh,
        material=tire_material,
        name="wheel_shell",
    )


def _add_fork_visuals(part, *, fork_material) -> None:
    part.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=fork_material,
        name="swivel_plate",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=fork_material,
        name="kingpin",
    )
    part.visual(
        Box((0.040, 0.042, 0.008)),
        origin=Origin(xyz=(-0.012, 0.0, -0.010)),
        material=fork_material,
        name="crown",
    )
    part.visual(
        Box((0.016, 0.006, 0.062)),
        origin=Origin(xyz=(-CASTER_TRAIL, -0.018, -0.043)),
        material=fork_material,
        name="left_arm",
    )
    part.visual(
        Box((0.016, 0.006, 0.062)),
        origin=Origin(xyz=(-CASTER_TRAIL, 0.018, -0.043)),
        material=fork_material,
        name="right_arm",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(
            xyz=(-CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=fork_material,
        name="axle_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_platform_cart", assets=ASSETS)

    deck_blue = model.material("deck_blue", rgba=(0.18, 0.36, 0.71, 1.0))
    deck_blue_dark = model.material("deck_blue_dark", rgba=(0.12, 0.24, 0.50, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.82, 0.84, 0.87, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.76, 0.78, 0.79, 1.0))
    tire_black = model.material("tire_black", rgba=(0.09, 0.09, 0.10, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.94, 0.60, 0.18)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    deck_mesh = _save_mesh(
        "platform_cart_deck.obj",
        ExtrudeGeometry(
            rounded_rect_profile(DECK_LENGTH, DECK_WIDTH, 0.035, corner_segments=10),
            DECK_THICKNESS,
            center=True,
        ),
    )
    chassis.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=deck_blue,
        name="deck_plate",
    )
    chassis.visual(
        Box((0.78, 0.44, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP + 0.002)),
        material=charcoal,
        name="deck_mat",
    )
    chassis.visual(
        Box((0.70, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, -0.165, 0.122)),
        material=deck_blue_dark,
        name="left_stiffener",
    )
    chassis.visual(
        Box((0.70, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, 0.165, 0.122)),
        material=deck_blue_dark,
        name="right_stiffener",
    )
    chassis.visual(
        Box((0.070, 0.360, 0.022)),
        origin=Origin(xyz=(0.395, 0.0, 0.124)),
        material=deck_blue_dark,
        name="front_crossmember",
    )
    chassis.visual(
        Box((0.070, 0.360, 0.022)),
        origin=Origin(xyz=(-0.395, 0.0, 0.124)),
        material=deck_blue_dark,
        name="rear_crossmember",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(CORNER_X, CORNER_Y, 0.125)),
        material=fork_gray,
        name="front_left_mount_pad",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(CORNER_X, -CORNER_Y, 0.125)),
        material=fork_gray,
        name="front_right_mount_pad",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(-CORNER_X, CORNER_Y, 0.125)),
        material=fork_gray,
        name="rear_left_mount_pad",
    )
    chassis.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(-CORNER_X, -CORNER_Y, 0.125)),
        material=fork_gray,
        name="rear_right_mount_pad",
    )
    chassis.visual(
        Box((0.040, 0.032, 0.008)),
        origin=Origin(xyz=(HANDLE_MOUNT_X, -HANDLE_HALF_WIDTH, 0.149)),
        material=deck_blue_dark,
        name="handle_left_pad",
    )
    chassis.visual(
        Box((0.040, 0.032, 0.008)),
        origin=Origin(xyz=(HANDLE_MOUNT_X, HANDLE_HALF_WIDTH, 0.149)),
        material=deck_blue_dark,
        name="handle_right_pad",
    )
    chassis.visual(
        Box((0.030, 0.320, 0.012)),
        origin=Origin(xyz=(0.435, 0.0, 0.149)),
        material=charcoal,
        name="front_bumper",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.08, 0.40, 0.48)),
        mass=4.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.24)),
    )
    handle.visual(
        Box((0.028, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -HANDLE_HALF_WIDTH, 0.015)),
        material=handle_metal,
        name="left_mount",
    )
    handle.visual(
        Box((0.028, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, HANDLE_HALF_WIDTH, 0.015)),
        material=handle_metal,
        name="right_mount",
    )
    handle.visual(
        _save_mesh(
            "platform_cart_handle_left_upright.obj",
            tube_from_spline_points(
                [
                    (0.0, -HANDLE_HALF_WIDTH, 0.030),
                    (-0.006, -HANDLE_HALF_WIDTH, 0.210),
                    (-0.028, -HANDLE_HALF_WIDTH, 0.430),
                ],
                radius=0.012,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=handle_metal,
        name="left_upright",
    )
    handle.visual(
        _save_mesh(
            "platform_cart_handle_right_upright.obj",
            tube_from_spline_points(
                [
                    (0.0, HANDLE_HALF_WIDTH, 0.030),
                    (-0.006, HANDLE_HALF_WIDTH, 0.210),
                    (-0.028, HANDLE_HALF_WIDTH, 0.430),
                ],
                radius=0.012,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=handle_metal,
        name="right_upright",
    )
    handle.visual(
        _save_mesh(
            "platform_cart_handle_top_bar.obj",
            tube_from_spline_points(
                [
                    (-0.028, -HANDLE_HALF_WIDTH, 0.430),
                    (-0.028, 0.0, 0.432),
                    (-0.028, HANDLE_HALF_WIDTH, 0.430),
                ],
                radius=0.012,
                samples_per_segment=12,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=handle_metal,
        name="top_bar",
    )

    front_left_fork = model.part("front_left_fork")
    front_left_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(-0.010, 0.0, -0.050)),
    )
    _add_fork_visuals(front_left_fork, fork_material=fork_gray)

    front_right_fork = model.part("front_right_fork")
    front_right_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(-0.010, 0.0, -0.050)),
    )
    _add_fork_visuals(front_right_fork, fork_material=fork_gray)

    rear_left_fork = model.part("rear_left_fork")
    rear_left_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(-0.010, 0.0, -0.050)),
    )
    _add_fork_visuals(rear_left_fork, fork_material=fork_gray)

    rear_right_fork = model.part("rear_right_fork")
    rear_right_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.10)),
        mass=1.2,
        origin=Origin(xyz=(-0.010, 0.0, -0.050)),
    )
    _add_fork_visuals(rear_right_fork, fork_material=fork_gray)

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        tire_material=tire_black,
        core_material=wheel_core,
        mesh_prefix="front_left_wheel",
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        tire_material=tire_black,
        core_material=wheel_core,
        mesh_prefix="front_right_wheel",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_left_wheel,
        tire_material=tire_black,
        core_material=wheel_core,
        mesh_prefix="rear_left_wheel",
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_right_wheel,
        tire_material=tire_black,
        core_material=wheel_core,
        mesh_prefix="rear_right_wheel",
    )

    model.articulation(
        "handle_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=handle,
        origin=Origin(xyz=(HANDLE_MOUNT_X, 0.0, HANDLE_MOUNT_Z)),
    )
    model.articulation(
        "front_left_swivel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_left_fork,
        origin=Origin(xyz=(CORNER_X, CORNER_Y, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "front_right_swivel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_right_fork,
        origin=Origin(xyz=(CORNER_X, -CORNER_Y, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "rear_left_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=rear_left_fork,
        origin=Origin(xyz=(-CORNER_X, CORNER_Y, SWIVEL_Z)),
    )
    model.articulation(
        "rear_right_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=rear_right_fork,
        origin=Origin(xyz=(-CORNER_X, -CORNER_Y, SWIVEL_Z)),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(-CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(-CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_left_fork,
        child=rear_left_wheel,
        origin=Origin(xyz=(-CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_right_fork,
        child=rear_right_wheel,
        origin=Origin(xyz=(-CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    handle = object_model.get_part("handle")
    front_left_fork = object_model.get_part("front_left_fork")
    front_right_fork = object_model.get_part("front_right_fork")
    rear_left_fork = object_model.get_part("rear_left_fork")
    rear_right_fork = object_model.get_part("rear_right_fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    handle_mount = object_model.get_articulation("handle_mount")
    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_right_swivel = object_model.get_articulation("front_right_swivel")
    rear_left_mount = object_model.get_articulation("rear_left_mount")
    rear_right_mount = object_model.get_articulation("rear_right_mount")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")

    deck_plate = chassis.get_visual("deck_plate")
    front_left_mount_pad = chassis.get_visual("front_left_mount_pad")
    front_right_mount_pad = chassis.get_visual("front_right_mount_pad")
    rear_left_mount_pad = chassis.get_visual("rear_left_mount_pad")
    rear_right_mount_pad = chassis.get_visual("rear_right_mount_pad")
    handle_left_pad = chassis.get_visual("handle_left_pad")
    handle_right_pad = chassis.get_visual("handle_right_pad")

    handle_left_mount = handle.get_visual("left_mount")
    handle_right_mount = handle.get_visual("right_mount")

    front_left_plate = front_left_fork.get_visual("swivel_plate")
    front_right_plate = front_right_fork.get_visual("swivel_plate")
    rear_left_plate = rear_left_fork.get_visual("swivel_plate")
    rear_right_plate = rear_right_fork.get_visual("swivel_plate")

    front_left_axle = front_left_fork.get_visual("axle_pin")
    front_right_axle = front_right_fork.get_visual("axle_pin")
    rear_left_axle = rear_left_fork.get_visual("axle_pin")
    rear_right_axle = rear_right_fork.get_visual("axle_pin")

    front_left_arm_l = front_left_fork.get_visual("left_arm")
    front_left_arm_r = front_left_fork.get_visual("right_arm")
    front_right_arm_l = front_right_fork.get_visual("left_arm")
    front_right_arm_r = front_right_fork.get_visual("right_arm")
    rear_left_arm_l = rear_left_fork.get_visual("left_arm")
    rear_left_arm_r = rear_left_fork.get_visual("right_arm")
    rear_right_arm_l = rear_right_fork.get_visual("left_arm")
    rear_right_arm_r = rear_right_fork.get_visual("right_arm")

    front_left_shell = front_left_wheel.get_visual("wheel_shell")
    front_right_shell = front_right_wheel.get_visual("wheel_shell")
    rear_left_shell = rear_left_wheel.get_visual("wheel_shell")
    rear_right_shell = rear_right_wheel.get_visual("wheel_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    for part_name, part in [
        ("chassis", chassis),
        ("handle", handle),
        ("front_left_fork", front_left_fork),
        ("front_right_fork", front_right_fork),
        ("rear_left_fork", rear_left_fork),
        ("rear_right_fork", rear_right_fork),
        ("front_left_wheel", front_left_wheel),
        ("front_right_wheel", front_right_wheel),
        ("rear_left_wheel", rear_left_wheel),
        ("rear_right_wheel", rear_right_wheel),
    ]:
        ctx.check(f"{part_name}_present", part is not None)

    for articulation_name, articulation, expected_type, expected_axis in [
        ("handle_mount", handle_mount, ArticulationType.FIXED, None),
        ("front_left_swivel", front_left_swivel, ArticulationType.REVOLUTE, (0.0, 0.0, 1.0)),
        ("front_right_swivel", front_right_swivel, ArticulationType.REVOLUTE, (0.0, 0.0, 1.0)),
        ("rear_left_mount", rear_left_mount, ArticulationType.FIXED, None),
        ("rear_right_mount", rear_right_mount, ArticulationType.FIXED, None),
        ("front_left_spin", front_left_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("front_right_spin", front_right_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("rear_left_spin", rear_left_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
        ("rear_right_spin", rear_right_spin, ArticulationType.CONTINUOUS, (0.0, 1.0, 0.0)),
    ]:
        ctx.check(
            f"{articulation_name}_type",
            articulation.articulation_type == expected_type,
            details=f"Expected {expected_type} but got {articulation.articulation_type}.",
        )
        if expected_axis is not None:
            ctx.check(
                f"{articulation_name}_axis",
                all(abs(a - b) <= 1e-9 for a, b in zip(articulation.axis, expected_axis)),
                details=f"Expected axis {expected_axis} but got {articulation.axis}.",
            )

    for fork, wheel, axle, shell, label in [
        (front_left_fork, front_left_wheel, front_left_axle, front_left_shell, "front_left"),
        (front_right_fork, front_right_wheel, front_right_axle, front_right_shell, "front_right"),
        (rear_left_fork, rear_left_wheel, rear_left_axle, rear_left_shell, "rear_left"),
        (rear_right_fork, rear_right_wheel, rear_right_axle, rear_right_shell, "rear_right"),
    ]:
        ctx.allow_overlap(
            fork,
            wheel,
            elem_a=axle,
            elem_b=shell,
            reason=f"{label} caster axle is intentionally nested through the wheel bore.",
        )

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(
        handle,
        chassis,
        elem_a=handle_left_mount,
        elem_b=handle_left_pad,
        name="handle_left_mount_contact",
    )
    ctx.expect_contact(
        handle,
        chassis,
        elem_a=handle_right_mount,
        elem_b=handle_right_pad,
        name="handle_right_mount_contact",
    )

    for fork, pad, plate, label in [
        (front_left_fork, front_left_mount_pad, front_left_plate, "front_left_fork_mount"),
        (front_right_fork, front_right_mount_pad, front_right_plate, "front_right_fork_mount"),
        (rear_left_fork, rear_left_mount_pad, rear_left_plate, "rear_left_fork_mount"),
        (rear_right_fork, rear_right_mount_pad, rear_right_plate, "rear_right_fork_mount"),
    ]:
        ctx.expect_contact(
            fork,
            chassis,
            elem_a=plate,
            elem_b=pad,
            name=f"{label}_contact",
        )

    for fork, wheel, arm_l, arm_r, shell, label in [
        (
            front_left_fork,
            front_left_wheel,
            front_left_arm_l,
            front_left_arm_r,
            front_left_shell,
            "front_left",
        ),
        (
            front_right_fork,
            front_right_wheel,
            front_right_arm_l,
            front_right_arm_r,
            front_right_shell,
            "front_right",
        ),
        (
            rear_left_fork,
            rear_left_wheel,
            rear_left_arm_l,
            rear_left_arm_r,
            rear_left_shell,
            "rear_left",
        ),
        (
            rear_right_fork,
            rear_right_wheel,
            rear_right_arm_l,
            rear_right_arm_r,
            rear_right_shell,
            "rear_right",
        ),
    ]:
        ctx.expect_gap(
            wheel,
            fork,
            axis="y",
            positive_elem=shell,
            negative_elem=arm_l,
            min_gap=0.001,
            max_gap=0.004,
            name=f"{label}_left_arm_clearance",
        )
        ctx.expect_gap(
            fork,
            wheel,
            axis="y",
            positive_elem=arm_r,
            negative_elem=shell,
            min_gap=0.001,
            max_gap=0.004,
            name=f"{label}_right_arm_clearance",
        )
        ctx.expect_origin_gap(
            fork,
            wheel,
            axis="x",
            min_gap=0.020,
            max_gap=0.024,
            name=f"{label}_caster_trail",
        )
        ctx.expect_origin_gap(
            fork,
            wheel,
            axis="z",
            min_gap=0.068,
            max_gap=0.072,
            name=f"{label}_wheel_drop",
        )

    for wheel, shell, label in [
        (front_left_wheel, front_left_shell, "front_left"),
        (front_right_wheel, front_right_shell, "front_right"),
        (rear_left_wheel, rear_left_shell, "rear_left"),
        (rear_right_wheel, rear_right_shell, "rear_right"),
    ]:
        ctx.expect_gap(
            chassis,
            wheel,
            axis="z",
            positive_elem=deck_plate,
            negative_elem=shell,
            min_gap=0.015,
            name=f"{label}_wheel_to_deck_clearance",
        )

    rest_left = ctx.part_world_position(front_left_wheel)
    rest_right = ctx.part_world_position(front_right_wheel)
    ctx.check("front_wheel_positions_available", rest_left is not None and rest_right is not None)

    with ctx.pose(
        {
            front_left_swivel: 1.0,
            front_right_swivel: -1.0,
            front_left_spin: 1.8,
            front_right_spin: -2.2,
            rear_left_spin: 1.2,
            rear_right_spin: -1.4,
        }
    ):
        posed_left = ctx.part_world_position(front_left_wheel)
        posed_right = ctx.part_world_position(front_right_wheel)
        ctx.check("posed_front_wheel_positions_available", posed_left is not None and posed_right is not None)
        if rest_left is not None and rest_right is not None and posed_left is not None and posed_right is not None:
            ctx.check(
                "front_left_swivel_moves_wheel_inboard",
                abs(posed_left[1]) < abs(rest_left[1]) - 0.010,
                details=f"Rest y={rest_left[1]:.4f}, posed y={posed_left[1]:.4f}.",
            )
            ctx.check(
                "front_right_swivel_moves_wheel_inboard",
                abs(posed_right[1]) < abs(rest_right[1]) - 0.010,
                details=f"Rest y={rest_right[1]:.4f}, posed y={posed_right[1]:.4f}.",
            )

        ctx.expect_contact(
            front_left_fork,
            chassis,
            elem_a=front_left_plate,
            elem_b=front_left_mount_pad,
            name="front_left_mount_contact_in_pose",
        )
        ctx.expect_contact(
            front_right_fork,
            chassis,
            elem_a=front_right_plate,
            elem_b=front_right_mount_pad,
            name="front_right_mount_contact_in_pose",
        )
        ctx.expect_contact(
            front_left_fork,
            front_left_wheel,
            elem_a=front_left_axle,
            elem_b=front_left_shell,
            name="front_left_axle_nested_in_pose",
        )
        ctx.expect_contact(
            front_right_fork,
            front_right_wheel,
            elem_a=front_right_axle,
            elem_b=front_right_shell,
            name="front_right_axle_nested_in_pose",
        )
        ctx.expect_gap(
            chassis,
            front_left_wheel,
            axis="z",
            positive_elem=deck_plate,
            negative_elem=front_left_shell,
            min_gap=0.012,
            name="front_left_clearance_in_pose",
        )
        ctx.expect_gap(
            chassis,
            front_right_wheel,
            axis="z",
            positive_elem=deck_plate,
            negative_elem=front_right_shell,
            min_gap=0.012,
            name="front_right_clearance_in_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
