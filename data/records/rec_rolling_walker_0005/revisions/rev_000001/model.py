from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_HALF_WIDTH = 0.275
FRAME_TUBE_RADIUS = 0.012
FRAME_BAR_RADIUS = 0.010
LOWER_BRACE_RADIUS = 0.009
GRIP_RADIUS = 0.017
WHEEL_RADIUS = 0.072
WHEEL_TIRE_WIDTH = 0.018
WHEEL_TOTAL_WIDTH = 0.022
CASTER_ORIGIN_Z = 0.190
BRAKE_PIVOT_Y = -0.020
BRAKE_PIVOT_Z = 0.842


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _side_rail_points(side_sign: float) -> list[tuple[float, float, float]]:
    x = side_sign * FRAME_HALF_WIDTH
    return [
        (x, -0.170, 0.030),
        (x, -0.150, 0.290),
        (x, -0.120, 0.460),
        (x, -0.055, 0.790),
        (x, 0.020, 0.845),
        (x, 0.110, 0.850),
        (x, 0.165, 0.610),
        (x, 0.175, 0.322),
        (x, 0.240, 0.235),
    ]


def _lower_brace_points(side_sign: float) -> list[tuple[float, float, float]]:
    x = side_sign * (FRAME_HALF_WIDTH - 0.018)
    return [
        (x, -0.136, 0.332),
        (x, -0.030, 0.335),
        (x, 0.082, 0.325),
        (x, 0.165, 0.298),
    ]


def _add_brake_lever(part, mesh_name: str, lever_finish, hardware_finish) -> None:
    part.visual(
        Box((0.022, 0.016, 0.020)),
        material=hardware_finish,
        name="pivot_collar",
    )
    part.visual(
        Box((0.014, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.000, -0.012)),
        material=lever_finish,
        name="lever_body",
    )
    part.visual(
        _save_mesh(
            mesh_name,
            tube_from_spline_points(
                [
                    (0.0, 0.002, -0.010),
                    (0.0, 0.016, -0.018),
                    (0.0, 0.030, -0.032),
                    (0.0, 0.043, -0.044),
                ],
                radius=0.0045,
                samples_per_segment=14,
                radial_segments=14,
            ),
        ),
        material=lever_finish,
        name="lever_blade",
    )
    part.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.045, -0.044)),
        material=lever_finish,
        name="lever_pad",
    )
    part.visual(
        Box((0.008, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, -0.004, -0.012)),
        material=hardware_finish,
        name="parking_tab",
    )


def _add_caster_fork(part, mesh_prefix: str, fork_finish, hardware_finish) -> None:
    part.visual(
        Cylinder(radius=0.0075, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=hardware_finish,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=hardware_finish,
        name="stem_cap",
    )
    part.visual(
        Box((0.036, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.010, -0.032)),
        material=fork_finish,
        name="crown",
    )
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_arm_left.obj",
            tube_from_spline_points(
                [
                    (-0.012, -0.010, -0.040),
                    (-0.013, -0.010, -0.066),
                    (-0.013, -0.010, -0.094),
                    (-0.013, -0.010, -0.116),
                ],
                radius=0.0045,
                samples_per_segment=12,
                radial_segments=12,
            ),
        ),
        material=fork_finish,
        name="arm_left",
    )
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_arm_right.obj",
            tube_from_spline_points(
                [
                    (0.012, -0.010, -0.040),
                    (0.013, -0.010, -0.066),
                    (0.013, -0.010, -0.094),
                    (0.013, -0.010, -0.116),
                ],
                radius=0.0045,
                samples_per_segment=12,
                radial_segments=12,
            ),
        ),
        material=fork_finish,
        name="arm_right",
    )
    part.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(-0.018, -0.010, -0.116), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_finish,
        name="axle_cap_left",
    )
    part.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.018, -0.010, -0.116), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_finish,
        name="axle_cap_right",
    )


def _add_front_wheel(part, tire_finish, rim_finish, hardware_finish) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_TIRE_WIDTH),
        origin=spin_origin,
        material=tire_finish,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=spin_origin,
        material=rim_finish,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.016),
        origin=spin_origin,
        material=hardware_finish,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.002),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_finish,
        name="hub_cap_left",
    )
    part.visual(
        Cylinder(radius=0.021, length=0.002),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_finish,
        name="hub_cap_right",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_rolling_walker", assets=ASSETS)

    frame_finish = model.material("frame_finish", rgba=(0.19, 0.21, 0.23, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.63, 0.65, 0.68, 1.0))
    fork_finish = model.material("fork_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.09, 0.10, 0.11, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    rim_finish = model.material("rim_finish", rgba=(0.56, 0.58, 0.61, 1.0))
    glide_rubber = model.material("glide_rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    lever_finish = model.material("lever_finish", rgba=(0.30, 0.31, 0.33, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.60, 0.50, 0.88)),
        mass=8.6,
        origin=Origin(xyz=(0.0, 0.030, 0.440)),
    )

    frame.visual(
        _save_mesh(
            "walker_left_side_rail.obj",
            tube_from_spline_points(
                _side_rail_points(-1.0),
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_finish,
        name="left_side_rail",
    )
    frame.visual(
        _save_mesh(
            "walker_right_side_rail.obj",
            tube_from_spline_points(
                _side_rail_points(1.0),
                radius=FRAME_TUBE_RADIUS,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_finish,
        name="right_side_rail",
    )
    frame.visual(
        _save_mesh(
            "walker_left_lower_brace.obj",
            tube_from_spline_points(
                _lower_brace_points(-1.0),
                radius=LOWER_BRACE_RADIUS,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=frame_finish,
        name="left_lower_brace",
    )
    frame.visual(
        _save_mesh(
            "walker_right_lower_brace.obj",
            tube_from_spline_points(
                _lower_brace_points(1.0),
                radius=LOWER_BRACE_RADIUS,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=frame_finish,
        name="right_lower_brace",
    )
    frame.visual(
        Cylinder(radius=FRAME_BAR_RADIUS, length=0.530),
        origin=Origin(xyz=(0.0, -0.055, 0.790), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_finish,
        name="upper_crossbar",
    )
    frame.visual(
        Cylinder(radius=FRAME_BAR_RADIUS, length=0.530),
        origin=Origin(xyz=(0.0, -0.120, 0.460), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_finish,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=FRAME_BAR_RADIUS, length=0.530),
        origin=Origin(xyz=(0.0, 0.175, 0.322), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_finish,
        name="lower_crossbar",
    )

    for side_name, x in (("left", -FRAME_HALF_WIDTH), ("right", FRAME_HALF_WIDTH)):
        mount_sign = -1.0 if side_name == "left" else 1.0
        frame.visual(
            Cylinder(radius=GRIP_RADIUS, length=0.118),
            origin=Origin(xyz=(x, 0.020, 0.846), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name=f"{side_name}_grip",
        )
        frame.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(x, -0.036, 0.846), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_hardware,
            name=f"{side_name}_grip_rear_ring",
        )
        frame.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(x, 0.076, 0.846), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_hardware,
            name=f"{side_name}_grip_front_ring",
        )
        frame.visual(
            Box((0.008, 0.028, 0.020)),
            origin=Origin(xyz=(x + mount_sign * 0.014, -0.040, BRAKE_PIVOT_Z)),
            material=satin_hardware,
            name=f"{side_name}_brake_mount",
        )
        frame.visual(
            Box((0.020, 0.018, 0.018)),
            origin=Origin(xyz=(x + mount_sign * 0.006, -0.040, BRAKE_PIVOT_Z)),
            material=frame_finish,
            name=f"{side_name}_brake_bridge",
        )
        frame.visual(
            Box((0.014, 0.010, 0.004)),
            origin=Origin(xyz=(x, 0.002, 0.824)),
            material=satin_hardware,
            name=f"{side_name}_lock_plate",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.022),
            origin=Origin(xyz=(x, 0.240, 0.201)),
            material=satin_hardware,
            name=f"{side_name}_caster_mount",
        )
        frame.visual(
            Box((0.018, 0.020, 0.024)),
            origin=Origin(xyz=(x, 0.240, 0.221)),
            material=frame_finish,
            name=f"{side_name}_caster_socket",
        )
        frame.visual(
            Box((0.030, 0.016, 0.020)),
            origin=Origin(xyz=(x, -0.170, 0.020)),
            material=glide_rubber,
            name=f"{side_name}_rear_glide",
        )

    left_brake = model.part("left_brake_lever")
    left_brake.inertial = Inertial.from_geometry(
        Box((0.03, 0.08, 0.07)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.014, -0.030)),
    )
    _add_brake_lever(left_brake, "walker_left_brake_lever.obj", lever_finish, satin_hardware)

    right_brake = model.part("right_brake_lever")
    right_brake.inertial = Inertial.from_geometry(
        Box((0.03, 0.08, 0.07)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.014, -0.030)),
    )
    _add_brake_lever(right_brake, "walker_right_brake_lever.obj", lever_finish, satin_hardware)

    left_fork = model.part("left_caster_fork")
    left_fork.inertial = Inertial.from_geometry(
        Box((0.04, 0.03, 0.14)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.008, -0.074)),
    )
    _add_caster_fork(left_fork, "walker_left_caster_fork", fork_finish, satin_hardware)

    right_fork = model.part("right_caster_fork")
    right_fork.inertial = Inertial.from_geometry(
        Box((0.04, 0.03, 0.14)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.008, -0.074)),
    )
    _add_caster_fork(right_fork, "walker_right_caster_fork", fork_finish, satin_hardware)

    left_wheel = model.part("left_front_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_TIRE_WIDTH),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_front_wheel(left_wheel, tire_rubber, rim_finish, satin_hardware)

    right_wheel = model.part("right_front_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_TIRE_WIDTH),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_front_wheel(right_wheel, tire_rubber, rim_finish, satin_hardware)

    model.articulation(
        "left_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_brake,
        origin=Origin(xyz=(-FRAME_HALF_WIDTH - 0.029, BRAKE_PIVOT_Y, BRAKE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.10, upper=0.50),
    )
    model.articulation(
        "right_brake_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_brake,
        origin=Origin(xyz=(FRAME_HALF_WIDTH + 0.029, BRAKE_PIVOT_Y, BRAKE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.10, upper=0.50),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_fork,
        origin=Origin(xyz=(-FRAME_HALF_WIDTH, 0.240, CASTER_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_fork,
        origin=Origin(xyz=(FRAME_HALF_WIDTH, 0.240, CASTER_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_fork,
        child=left_wheel,
        origin=Origin(xyz=(0.0, -0.010, -0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_fork,
        child=right_wheel,
        origin=Origin(xyz=(0.0, -0.010, -0.116)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_brake = object_model.get_part("left_brake_lever")
    right_brake = object_model.get_part("right_brake_lever")
    left_fork = object_model.get_part("left_caster_fork")
    right_fork = object_model.get_part("right_caster_fork")
    left_wheel = object_model.get_part("left_front_wheel")
    right_wheel = object_model.get_part("right_front_wheel")

    left_brake_hinge = object_model.get_articulation("left_brake_hinge")
    right_brake_hinge = object_model.get_articulation("right_brake_hinge")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    left_grip = frame.get_visual("left_grip")
    right_grip = frame.get_visual("right_grip")
    left_brake_mount = frame.get_visual("left_brake_mount")
    right_brake_mount = frame.get_visual("right_brake_mount")
    left_caster_mount = frame.get_visual("left_caster_mount")
    right_caster_mount = frame.get_visual("right_caster_mount")

    left_pivot = left_brake.get_visual("pivot_collar")
    right_pivot = right_brake.get_visual("pivot_collar")
    left_pad = left_brake.get_visual("lever_pad")
    right_pad = right_brake.get_visual("lever_pad")

    left_stem_cap = left_fork.get_visual("stem_cap")
    right_stem_cap = right_fork.get_visual("stem_cap")
    left_arm_left = left_fork.get_visual("arm_left")
    left_arm_right = left_fork.get_visual("arm_right")
    right_arm_left = right_fork.get_visual("arm_left")
    right_arm_right = right_fork.get_visual("arm_right")

    left_hub_left = left_wheel.get_visual("hub_cap_left")
    left_hub_right = left_wheel.get_visual("hub_cap_right")
    right_hub_left = right_wheel.get_visual("hub_cap_left")
    right_hub_right = right_wheel.get_visual("hub_cap_right")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "walker_part_count",
        len(object_model.parts) == 7 and len(object_model.articulations) == 6,
        f"expected 7 parts / 6 articulations, got {len(object_model.parts)} / {len(object_model.articulations)}",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        frame_min, frame_max = frame_aabb
        frame_width = frame_max[0] - frame_min[0]
        frame_depth = frame_max[1] - frame_min[1]
        frame_height = frame_max[2] - frame_min[2]
        ctx.check(
            "frame_proportions",
            0.54 <= frame_width <= 0.62 and 0.42 <= frame_depth <= 0.54 and 0.84 <= frame_height <= 0.92,
            f"frame dims were {frame_width:.3f} x {frame_depth:.3f} x {frame_height:.3f}",
        )

    ctx.expect_origin_distance(
        left_brake,
        right_brake,
        axes="x",
        min_dist=0.52,
        max_dist=0.62,
        name="handle_spacing",
    )
    ctx.expect_origin_distance(
        left_fork,
        right_fork,
        axes="x",
        min_dist=0.52,
        max_dist=0.58,
        name="caster_track",
    )

    ctx.expect_contact(
        left_brake,
        frame,
        elem_a=left_pivot,
        elem_b=left_brake_mount,
        name="left_brake_mount_contact",
    )
    ctx.expect_contact(
        right_brake,
        frame,
        elem_a=right_pivot,
        elem_b=right_brake_mount,
        name="right_brake_mount_contact",
    )
    ctx.expect_gap(
        frame,
        left_brake,
        axis="z",
        positive_elem=left_grip,
        negative_elem=left_pad,
        min_gap=0.020,
        max_gap=0.045,
        name="left_brake_rest_gap",
    )
    ctx.expect_gap(
        frame,
        right_brake,
        axis="z",
        positive_elem=right_grip,
        negative_elem=right_pad,
        min_gap=0.020,
        max_gap=0.045,
        name="right_brake_rest_gap",
    )

    ctx.expect_contact(
        left_fork,
        frame,
        elem_a=left_stem_cap,
        elem_b=left_caster_mount,
        name="left_caster_mount_contact",
    )
    ctx.expect_contact(
        right_fork,
        frame,
        elem_a=right_stem_cap,
        elem_b=right_caster_mount,
        name="right_caster_mount_contact",
    )
    ctx.expect_gap(
        frame,
        left_fork,
        axis="z",
        positive_elem=left_caster_mount,
        negative_elem=left_stem_cap,
        min_gap=0.0,
        max_gap=0.001,
        name="left_caster_seat_gap",
    )
    ctx.expect_gap(
        frame,
        right_fork,
        axis="z",
        positive_elem=right_caster_mount,
        negative_elem=right_stem_cap,
        min_gap=0.0,
        max_gap=0.001,
        name="right_caster_seat_gap",
    )
    ctx.expect_within(left_wheel, left_fork, axes="x", margin=0.0, name="left_wheel_within_fork_width")
    ctx.expect_within(right_wheel, right_fork, axes="x", margin=0.0, name="right_wheel_within_fork_width")
    ctx.expect_overlap(left_wheel, left_fork, axes="yz", min_overlap=0.010, name="left_wheel_within_fork_aperture")
    ctx.expect_overlap(right_wheel, right_fork, axes="yz", min_overlap=0.010, name="right_wheel_within_fork_aperture")
    ctx.expect_contact(
        left_wheel,
        left_fork,
        elem_a=left_hub_left,
        elem_b=left_arm_left,
        name="left_wheel_left_bearing_contact",
    )
    ctx.expect_contact(
        left_wheel,
        left_fork,
        elem_a=left_hub_right,
        elem_b=left_arm_right,
        name="left_wheel_right_bearing_contact",
    )
    ctx.expect_contact(
        right_wheel,
        right_fork,
        elem_a=right_hub_left,
        elem_b=right_arm_left,
        name="right_wheel_left_bearing_contact",
    )
    ctx.expect_contact(
        right_wheel,
        right_fork,
        elem_a=right_hub_right,
        elem_b=right_arm_right,
        name="right_wheel_right_bearing_contact",
    )

    with ctx.pose(
        {
            left_brake_hinge: 0.36,
            right_brake_hinge: 0.36,
        }
    ):
        ctx.expect_contact(
            left_brake,
            frame,
            elem_a=left_pivot,
            elem_b=left_brake_mount,
            name="left_brake_mount_contact_pulled",
        )
        ctx.expect_contact(
            right_brake,
            frame,
            elem_a=right_pivot,
            elem_b=right_brake_mount,
            name="right_brake_mount_contact_pulled",
        )
        ctx.expect_gap(
            frame,
            left_brake,
            axis="z",
            positive_elem=left_grip,
            negative_elem=left_pad,
            min_gap=0.003,
            max_gap=0.010,
            name="left_brake_pulled_gap",
        )
        ctx.expect_gap(
            frame,
            right_brake,
            axis="z",
            positive_elem=right_grip,
            negative_elem=right_pad,
            min_gap=0.003,
            max_gap=0.010,
            name="right_brake_pulled_gap",
        )

    with ctx.pose(
        {
            left_caster_swivel: 0.95,
            right_caster_swivel: -0.78,
            left_wheel_spin: 1.40,
            right_wheel_spin: -1.05,
        }
    ):
        ctx.expect_contact(
            left_fork,
            frame,
            elem_a=left_stem_cap,
            elem_b=left_caster_mount,
            name="left_caster_contact_swiveled",
        )
        ctx.expect_contact(
            right_fork,
            frame,
            elem_a=right_stem_cap,
            elem_b=right_caster_mount,
            name="right_caster_contact_swiveled",
        )
        ctx.expect_contact(
            left_wheel,
            left_fork,
            elem_a=left_hub_left,
            elem_b=left_arm_left,
            name="left_wheel_bearing_contact_swiveled",
        )
        ctx.expect_contact(
            right_wheel,
            right_fork,
            elem_a=right_hub_right,
            elem_b=right_arm_right,
            name="right_wheel_bearing_contact_swiveled",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
