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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    def mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(x, -y, z) for x, y, z in points]

    def add_wheel(part, prefix: str, *, radius: float, width: float, tire_mat, hub_mat, cap_mat) -> None:
        wheel_spin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
        part.visual(
            Cylinder(radius=radius, length=width),
            origin=wheel_spin,
            material=tire_mat,
            name=f"{prefix}_tire",
        )
        part.visual(
            Cylinder(radius=radius * 0.78, length=width * 0.78),
            origin=wheel_spin,
            material=hub_mat,
            name=f"{prefix}_hub",
        )
        for side in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=radius * 0.82, length=0.004),
                origin=Origin(xyz=(0.0, side * width * 0.26, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
                material=cap_mat,
            )
        part.visual(Box((radius * 1.15, 0.008, 0.016)), material=cap_mat)
        part.visual(Box((0.016, 0.008, radius * 1.15)), material=cap_mat)
        part.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=width),
            mass=0.75,
            origin=wheel_spin,
        )

    def build_basket():
        wire_radius = 0.004
        top_loop = wire_from_points(
            [
                (0.05, -0.11, 0.39),
                (0.23, -0.11, 0.39),
                (0.23, 0.11, 0.39),
                (0.05, 0.11, 0.39),
            ],
            radius=wire_radius,
            closed_path=True,
            corner_mode="miter",
        )
        bottom_loop = wire_from_points(
            [
                (0.07, -0.09, 0.28),
                (0.21, -0.09, 0.28),
                (0.21, 0.09, 0.28),
                (0.07, 0.09, 0.28),
            ],
            radius=wire_radius,
            closed_path=True,
            corner_mode="miter",
        )
        basket_geom = top_loop.merge(bottom_loop)
        for y_sign in (-1.0, 1.0):
            basket_geom.merge(
                wire_from_points(
                    [
                        (0.05, 0.11 * y_sign, 0.39),
                        (0.07, 0.09 * y_sign, 0.28),
                    ],
                    radius=wire_radius,
                    cap_ends=True,
                )
            )
            basket_geom.merge(
                wire_from_points(
                    [
                        (0.23, 0.11 * y_sign, 0.39),
                        (0.21, 0.09 * y_sign, 0.28),
                    ],
                    radius=wire_radius,
                    cap_ends=True,
                )
            )
            basket_geom.merge(
                wire_from_points(
                    [
                        (0.02, 0.04 * y_sign, 0.33),
                        (0.05, 0.08 * y_sign, 0.37),
                        (0.05, 0.11 * y_sign, 0.39),
                    ],
                    radius=wire_radius,
                    cap_ends=True,
                )
            )
        for y in (-0.055, 0.055):
            basket_geom.merge(
                wire_from_points(
                    [
                        (0.21, y, 0.28),
                        (0.23, y * 1.15, 0.34),
                        (0.23, y * 1.15, 0.39),
                    ],
                    radius=wire_radius * 0.9,
                    cap_ends=True,
                )
            )
        return basket_geom

    model.material("frame_paint", rgba=(0.20, 0.23, 0.26, 1.0))
    model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    model.material("wheel_hub", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("bright_metal", rgba=(0.65, 0.67, 0.70, 1.0))
    model.material("basket_wire", rgba=(0.15, 0.16, 0.18, 1.0))

    frame_paint = "frame_paint"
    satin_black = "satin_black"
    rubber = "rubber"
    wheel_hub = "wheel_hub"
    bright_metal = "bright_metal"
    basket_wire = "basket_wire"

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.80, 0.36, 0.95)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
    )
    frame.visual(
        mesh(
            "frame_main_tube",
            tube_from_spline_points(
                [
                    (-0.26, 0.0, 0.10),
                    (-0.12, 0.0, 0.16),
                    (0.05, 0.0, 0.18),
                    (0.15, 0.0, 0.19),
                    (0.17, 0.0, 0.185),
                ],
                radius=0.018,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="main_tube",
    )
    for y in (-0.026, 0.026):
        frame.visual(
            Box((0.050, 0.016, 0.030)),
            origin=Origin(xyz=(0.188, y, 0.188)),
            material=frame_paint,
            name="head_gusset" if y > 0.0 else None,
        )
    frame.visual(
        Cylinder(radius=0.026, length=0.12),
        origin=Origin(xyz=(0.22, 0.0, 0.22)),
        material=frame_paint,
        name="head_tube",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.26),
        origin=Origin(xyz=(-0.24, 0.0, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_crossmember",
    )
    for side, points in (
        ("left", [(-0.24, 0.118, 0.10), (-0.16, 0.08, 0.15), (-0.05, 0.03, 0.18)]),
        ("right", mirror_y([(-0.24, 0.118, 0.10), (-0.16, 0.08, 0.15), (-0.05, 0.03, 0.18)])),
    ):
        frame.visual(
            mesh(
                f"frame_{side}_rear_strut",
                tube_from_spline_points(
                    points,
                    radius=0.012,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
        )
    for y in (-0.130, 0.130):
        frame.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(-0.24, y, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bright_metal,
        )
    frame.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.02, 0.0, 0.27)),
        material=frame_paint,
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.10, 0.0, 0.27)),
        material=frame_paint,
    )
    frame.visual(
        Box((0.19, 0.16, 0.02)),
        origin=Origin(xyz=(0.04, 0.0, 0.355)),
        material=bright_metal,
    )
    frame.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(0.04, 0.0, 0.385)),
        material=satin_black,
        name="knee_pad",
    )
    frame.visual(
        Box((0.20, 0.14, 0.028)),
        origin=Origin(xyz=(0.04, 0.0, 0.414)),
        material=satin_black,
    )
    frame.visual(
        Box((0.06, 0.045, 0.014)),
        origin=Origin(xyz=(-0.02, 0.0, 0.155)),
        material=frame_paint,
        name="stand_bracket",
    )
    for y in (-0.0165, 0.0165):
        frame.visual(
            Box((0.022, 0.015, 0.026)),
            origin=Origin(xyz=(-0.02, y, 0.137)),
            material=frame_paint,
        )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.55, 0.32, 0.82)),
        mass=4.5,
        origin=Origin(xyz=(0.10, 0.0, 0.25)),
    )
    fork.visual(
        Cylinder(radius=0.015, length=0.16),
        material=bright_metal,
        name="steer_stem",
    )
    fork.visual(
        Cylinder(radius=0.018, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=frame_paint,
        name="handlebar_mast",
    )
    fork.visual(
        Box((0.12, 0.12, 0.03)),
        origin=Origin(xyz=(0.06, 0.0, -0.095)),
        material=frame_paint,
        name="fork_crown",
    )
    for side, points in (
        ("left", [(0.03, 0.045, -0.09), (0.08, 0.07, -0.11), (0.12, 0.09, -0.12)]),
        ("right", mirror_y([(0.03, 0.045, -0.09), (0.08, 0.07, -0.11), (0.12, 0.09, -0.12)])),
    ):
        fork.visual(
            mesh(
                f"fork_blade_{side}",
                tube_from_spline_points(
                    points,
                    radius=0.011,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
        )
    fork.visual(
        Cylinder(radius=0.008, length=0.19),
        origin=Origin(xyz=(0.12, 0.0, -0.12), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="front_axle",
    )
    for y in (-0.101, 0.101):
        fork.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.12, y, -0.12), rpy=(pi / 2.0, 0.0, 0.0)),
            material=bright_metal,
        )
    fork.visual(
        Box((0.028, 0.12, 0.02)),
        origin=Origin(xyz=(0.015, 0.0, 0.33)),
        material=frame_paint,
    )
    fork.visual(
        mesh("front_basket", build_basket()),
        material=basket_wire,
        name="basket",
    )
    fork.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.69), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="handlebar",
    )
    for y in (-0.18, 0.18):
        fork.visual(
            Cylinder(radius=0.016, length=0.08),
            origin=Origin(xyz=(0.0, y, 0.69), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
        )

    front_left_wheel = model.part("front_left_wheel")
    add_wheel(
        front_left_wheel,
        "front_left",
        radius=0.10,
        width=0.038,
        tire_mat=rubber,
        hub_mat=wheel_hub,
        cap_mat=bright_metal,
    )

    front_right_wheel = model.part("front_right_wheel")
    add_wheel(
        front_right_wheel,
        "front_right",
        radius=0.10,
        width=0.038,
        tire_mat=rubber,
        hub_mat=wheel_hub,
        cap_mat=bright_metal,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    add_wheel(
        rear_left_wheel,
        "rear_left",
        radius=0.10,
        width=0.038,
        tire_mat=rubber,
        hub_mat=wheel_hub,
        cap_mat=bright_metal,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    add_wheel(
        rear_right_wheel,
        "rear_right",
        radius=0.10,
        width=0.038,
        tire_mat=rubber,
        hub_mat=wheel_hub,
        cap_mat=bright_metal,
    )

    parking_stand = model.part("parking_stand")
    parking_stand.inertial = Inertial.from_geometry(
        Box((0.19, 0.09, 0.10)),
        mass=0.8,
        origin=Origin(xyz=(-0.08, 0.0, -0.012)),
    )
    parking_stand.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="stand_knuckle",
    )
    parking_stand.visual(
        mesh(
            "parking_stand_leg",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (-0.07, 0.0, -0.022),
                    (-0.16, 0.0, -0.046),
                ],
                radius=0.008,
                samples_per_segment=14,
                radial_segments=14,
            ),
        ),
        material=frame_paint,
        name="stand_leg",
    )
    parking_stand.visual(
        Cylinder(radius=0.007, length=0.09),
        origin=Origin(xyz=(-0.16, 0.0, -0.046), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="stand_foot",
    )

    model.articulation(
        "fork_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(0.22, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.12, 0.126, -0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.12, -0.126, -0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.24, 0.155, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.24, -0.155, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "parking_stand_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=parking_stand,
        origin=Origin(xyz=(-0.02, 0.0, 0.137)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.10, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    fork = object_model.get_part("fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    parking_stand = object_model.get_part("parking_stand")

    fork_steer = object_model.get_articulation("fork_steer")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    parking_stand_hinge = object_model.get_articulation("parking_stand_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        frame,
        fork,
        elem_a="head_tube",
        elem_b="steer_stem",
        reason="The steering stem is intentionally seated inside the frame head tube.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    for part_name in (
        "frame",
        "fork",
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
        "parking_stand",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.expect_contact(frame, fork, elem_a="head_tube", elem_b="steer_stem")
    ctx.expect_contact(front_left_wheel, fork)
    ctx.expect_contact(front_right_wheel, fork)
    ctx.expect_contact(rear_left_wheel, frame)
    ctx.expect_contact(rear_right_wheel, frame)
    ctx.expect_contact(parking_stand, frame)

    ctx.check(
        "fork_steer_is_vertical_yaw",
        fork_steer.axis == (0.0, 0.0, 1.0),
        details=f"axis={fork_steer.axis}",
    )
    for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin):
        ctx.check(
            f"{joint.name}_spins_on_axle",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
    ctx.check(
        "parking_stand_rotates_side_to_side_pivot",
        parking_stand_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={parking_stand_hinge.axis}",
    )

    basket_aabb = ctx.part_element_world_aabb(fork, elem="basket")
    knee_pad_aabb = ctx.part_element_world_aabb(frame, elem="knee_pad")
    if basket_aabb is not None and knee_pad_aabb is not None:
        ctx.check(
            "basket_is_forward_of_knee_pad",
            basket_aabb[0][0] > knee_pad_aabb[1][0] - 0.01,
            details=f"basket_min_x={basket_aabb[0][0]:.3f}, knee_pad_max_x={knee_pad_aabb[1][0]:.3f}",
        )
        ctx.check(
            "basket_is_above_front_axle",
            basket_aabb[0][2] > 0.20,
            details=f"basket_min_z={basket_aabb[0][2]:.3f}",
        )

    front_left_rest = ctx.part_world_position(front_left_wheel)
    rear_left_rest = ctx.part_world_position(rear_left_wheel)
    with ctx.pose({fork_steer: 0.45}):
        front_left_steered = ctx.part_world_position(front_left_wheel)
        rear_left_steered = ctx.part_world_position(rear_left_wheel)
        if front_left_rest is not None and front_left_steered is not None:
            ctx.check(
                "front_wheel_moves_with_steering",
                abs(front_left_steered[0] - front_left_rest[0]) > 0.04
                and abs(front_left_steered[1] - front_left_rest[1]) > 0.008,
                details=f"rest={front_left_rest}, steered={front_left_steered}",
            )
        if rear_left_rest is not None and rear_left_steered is not None:
            ctx.check(
                "rear_wheel_stays_put_when_steering",
                abs(rear_left_steered[0] - rear_left_rest[0]) < 1e-6
                and abs(rear_left_steered[1] - rear_left_rest[1]) < 1e-6,
                details=f"rest={rear_left_rest}, steered={rear_left_steered}",
            )
        ctx.expect_contact(front_left_wheel, fork)
        ctx.expect_contact(front_right_wheel, fork)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_steered_pose")

    stand_stowed = ctx.part_world_aabb(parking_stand)
    with ctx.pose({parking_stand_hinge: -1.02}):
        stand_parked = ctx.part_world_aabb(parking_stand)
        ctx.expect_contact(parking_stand, frame)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_parked_pose")
        if stand_stowed is not None and stand_parked is not None:
            ctx.check(
                "parking_stand_drops_when_deployed",
                stand_parked[0][2] < stand_stowed[0][2] - 0.06 and stand_parked[0][2] < 0.02,
                details=f"stowed_min_z={stand_stowed[0][2]:.3f}, parked_min_z={stand_parked[0][2]:.3f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
