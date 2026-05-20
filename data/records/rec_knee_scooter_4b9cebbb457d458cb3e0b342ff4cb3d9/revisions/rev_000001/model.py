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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points):
    return [(x, -y, z) for x, y, z in points]


def _add_wheel_visuals(part, *, tire_radius: float, tire_width: float, rubber, rim) -> None:
    wheel_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    side_offset = tire_width * 0.18
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=wheel_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.68, length=tire_width * 0.72),
        origin=wheel_origin,
        material=rim,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.74, length=0.005),
        origin=Origin(xyz=(0.0, side_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim,
        name="rim_face_outer",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.74, length=0.005),
        origin=Origin(xyz=(0.0, -side_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim,
        name="rim_face_inner",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.20, length=tire_width * 1.10),
        origin=wheel_origin,
        material=rim,
        name="hub",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=0.9,
        origin=wheel_origin,
    )


def _build_basket_wire_geometry():
    top_loop = wire_from_points(
        [
            (0.14, -0.11, 0.06),
            (0.30, -0.11, 0.06),
            (0.30, 0.11, 0.06),
            (0.14, 0.11, 0.06),
        ],
        radius=0.0042,
        radial_segments=14,
        closed_path=True,
        corner_mode="miter",
    )
    bottom_loop = wire_from_points(
        [
            (0.14, -0.09, -0.02),
            (0.28, -0.09, -0.02),
            (0.28, 0.09, -0.02),
            (0.14, 0.09, -0.02),
        ],
        radius=0.0036,
        radial_segments=12,
        closed_path=True,
        corner_mode="miter",
    )
    rear_frame = wire_from_points(
        [
            (0.14, -0.09, -0.02),
            (0.14, 0.09, -0.02),
            (0.14, 0.11, 0.06),
            (0.14, -0.11, 0.06),
        ],
        radius=0.0036,
        radial_segments=12,
        closed_path=True,
        corner_mode="miter",
    )
    basket = top_loop
    basket.merge(bottom_loop)
    basket.merge(rear_frame)

    vertical_pairs = [
        ((0.14, -0.11, 0.06), (0.14, -0.09, -0.02)),
        ((0.28, -0.09, -0.02), (0.30, -0.11, 0.06)),
        ((0.30, 0.11, 0.06), (0.28, 0.09, -0.02)),
        ((0.14, 0.09, -0.02), (0.14, 0.11, 0.06)),
    ]
    for top_point, bottom_point in vertical_pairs:
        basket.merge(
            tube_from_spline_points(
                [top_point, bottom_point],
                radius=0.0028,
                samples_per_segment=2,
                radial_segments=10,
                cap_ends=True,
            )
        )

    for y in (-0.04, 0.0, 0.04):
        basket.merge(
            tube_from_spline_points(
                [(0.14, y, -0.018), (0.28, y, -0.018)],
                radius=0.0025,
                samples_per_segment=2,
                radial_segments=10,
                cap_ends=True,
            )
        )

    return basket


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    frame_paint = model.material("frame_paint", rgba=(0.20, 0.23, 0.26, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    alloy = model.material("alloy", rgba=(0.73, 0.76, 0.79, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    knee_pad_vinyl = model.material("knee_pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.78, 0.30, 0.30)),
        mass=7.5,
        origin=Origin(xyz=(0.30, 0.0, 0.05)),
    )

    left_rail = tube_from_spline_points(
        [
            (0.00, 0.07, 0.00),
            (0.12, 0.075, -0.01),
            (0.30, 0.07, -0.015),
            (0.50, 0.055, -0.010),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        _mirror_y(
            [
                (0.00, 0.07, 0.00),
                (0.12, 0.075, -0.01),
                (0.30, 0.07, -0.015),
                (0.50, 0.055, -0.010),
            ]
        ),
        radius=0.018,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    frame.visual(_save_mesh("frame_left_rail", left_rail), material=frame_paint, name="left_rail")
    frame.visual(_save_mesh("frame_right_rail", right_rail), material=frame_paint, name="right_rail")
    frame.visual(
        Box((0.10, 0.24, 0.05)),
        origin=Origin(xyz=(0.00, 0.0, 0.00)),
        material=frame_paint,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.26, 0.12, 0.012)),
        origin=Origin(xyz=(0.18, 0.0, -0.014)),
        material=satin_black,
        name="deck",
    )
    frame.visual(
        Box((0.10, 0.14, 0.04)),
        origin=Origin(xyz=(0.52, 0.0, -0.005)),
        material=frame_paint,
        name="front_bridge",
    )
    frame.visual(
        Box((0.035, 0.028, 0.15)),
        origin=Origin(xyz=(0.56, 0.045, 0.055)),
        material=frame_paint,
        name="left_head_gusset",
    )
    frame.visual(
        Box((0.035, 0.028, 0.15)),
        origin=Origin(xyz=(0.56, -0.045, 0.055)),
        material=frame_paint,
        name="right_head_gusset",
    )
    frame.visual(
        Box((0.09, 0.10, 0.05)),
        origin=Origin(xyz=(0.30, 0.0, -0.005)),
        material=frame_paint,
        name="post_guide_base",
    )
    frame.visual(
        Box((0.030, 0.012, 0.22)),
        origin=Origin(xyz=(0.32, 0.032, 0.07)),
        material=frame_paint,
        name="left_post_guide",
    )
    frame.visual(
        Box((0.030, 0.012, 0.22)),
        origin=Origin(xyz=(0.32, -0.032, 0.07)),
        material=frame_paint,
        name="right_post_guide",
    )
    frame.visual(
        Box((0.016, 0.070, 0.22)),
        origin=Origin(xyz=(0.286, 0.0, 0.07)),
        material=frame_paint,
        name="rear_post_guide",
    )
    frame.visual(
        Box((0.18, 0.08, 0.03)),
        origin=Origin(xyz=(0.40, 0.0, 0.000)),
        material=frame_paint,
        name="center_brace",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.11525, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="rear_left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, -0.11525, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="rear_right_axle_stub",
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.28, 0.44, 0.86)),
        mass=3.2,
        origin=Origin(xyz=(0.07, 0.0, 0.31)),
    )
    front_fork.visual(
        Box((0.045, 0.16, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=alloy,
        name="steering_thrust_plate",
    )
    front_fork.visual(
        Cylinder(radius=0.020, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=frame_paint,
        name="steering_column",
    )
    front_fork.visual(
        Box((0.09, 0.14, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, -0.01)),
        material=frame_paint,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.06, 0.020, 0.12)),
        origin=Origin(xyz=(0.11, 0.055, -0.05)),
        material=frame_paint,
        name="left_fork_leg",
    )
    front_fork.visual(
        Box((0.06, 0.020, 0.12)),
        origin=Origin(xyz=(0.11, -0.055, -0.05)),
        material=frame_paint,
        name="right_fork_leg",
    )
    front_fork.visual(
        Box((0.05, 0.15, 0.026)),
        origin=Origin(xyz=(0.15, 0.0, -0.10)),
        material=alloy,
        name="front_axle_block",
    )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.15, 0.07525, -0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="front_left_axle_stub",
    )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.15, -0.07525, -0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="front_right_axle_stub",
    )
    front_fork.visual(
        Box((0.055, 0.085, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=frame_paint,
        name="stem_clamp",
    )
    front_fork.visual(
        Cylinder(radius=0.012, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="handlebar",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(0.0, 0.19, 0.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="left_grip",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(0.0, -0.19, 0.74), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="right_grip",
    )
    front_fork.visual(
        Box((0.022, 0.10, 0.05)),
        origin=Origin(xyz=(0.029, 0.0, 0.56)),
        material=frame_paint,
        name="basket_mount",
    )

    knee_post = model.part("knee_post")
    knee_post.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.58)),
        mass=2.1,
        origin=Origin(xyz=(0.05, 0.0, 0.17)),
    )
    knee_post.visual(
        Box((0.036, 0.036, 0.48)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=alloy,
        name="inner_post",
    )
    knee_post.visual(
        Box((0.08, 0.045, 0.05)),
        origin=Origin(xyz=(0.045, 0.0, 0.29)),
        material=alloy,
        name="pad_bracket",
    )
    knee_post.visual(
        Box((0.18, 0.11, 0.018)),
        origin=Origin(xyz=(0.06, 0.0, 0.33)),
        material=grip_gray,
        name="pad_base",
    )
    knee_post.visual(
        Box((0.22, 0.13, 0.05)),
        origin=Origin(xyz=(0.07, 0.0, 0.364)),
        material=knee_pad_vinyl,
        name="knee_pad_top",
    )

    basket_support = model.part("basket_support")
    basket_support.inertial = Inertial.from_geometry(
        Box((0.34, 0.24, 0.14)),
        mass=0.9,
        origin=Origin(xyz=(0.16, 0.0, 0.02)),
    )
    basket_support.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=alloy,
        name="hinge_barrel",
    )
    basket_support.visual(
        Box((0.14, 0.030, 0.018)),
        origin=Origin(xyz=(0.07, 0.0, -0.012)),
        material=alloy,
        name="basket_arm",
    )
    basket_support.visual(
        Box((0.020, 0.16, 0.08)),
        origin=Origin(xyz=(0.14, 0.0, 0.01)),
        material=alloy,
        name="rear_plate",
    )
    left_basket_brace = tube_from_spline_points(
        [(0.01, 0.03, 0.0), (0.08, 0.05, 0.01), (0.14, 0.06, 0.03)],
        radius=0.0048,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    right_basket_brace = tube_from_spline_points(
        [(0.01, -0.03, 0.0), (0.08, -0.05, 0.01), (0.14, -0.06, 0.03)],
        radius=0.0048,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    basket_support.visual(
        _save_mesh("basket_left_brace", left_basket_brace),
        material=alloy,
        name="left_basket_brace",
    )
    basket_support.visual(
        _save_mesh("basket_right_brace", right_basket_brace),
        material=alloy,
        name="right_basket_brace",
    )
    basket_support.visual(
        _save_mesh("basket_wire", _build_basket_wire_geometry()),
        material=basket_gray,
        name="basket_wire",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(rear_left_wheel, tire_radius=0.10, tire_width=0.045, rubber=rubber, rim=alloy)

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(rear_right_wheel, tire_radius=0.10, tire_width=0.045, rubber=rubber, rim=alloy)

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel, tire_radius=0.10, tire_width=0.045, rubber=rubber, rim=alloy)

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel, tire_radius=0.10, tire_width=0.045, rubber=rubber, rim=alloy)

    model.articulation(
        "frame_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.60, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "frame_to_knee_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=knee_post,
        origin=Origin(xyz=(0.32, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.08),
    )
    model.articulation(
        "front_fork_to_basket_support",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=basket_support,
        origin=Origin(xyz=(0.05, 0.0, 0.56)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.4, lower=0.0, upper=0.95),
    )
    model.articulation(
        "front_fork_to_front_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.15, 0.11, -0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "front_fork_to_front_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.15, -0.11, -0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.0, 0.155, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    front_fork = object_model.get_part("front_fork")
    knee_post = object_model.get_part("knee_post")
    basket_support = object_model.get_part("basket_support")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    steer_joint = object_model.get_articulation("frame_to_front_fork")
    knee_slide = object_model.get_articulation("frame_to_knee_post")
    basket_hinge = object_model.get_articulation("front_fork_to_basket_support")
    wheel_joints = [
        object_model.get_articulation("front_fork_to_front_left_wheel"),
        object_model.get_articulation("front_fork_to_front_right_wheel"),
        object_model.get_articulation("frame_to_rear_left_wheel"),
        object_model.get_articulation("frame_to_rear_right_wheel"),
    ]

    joints_ok = (
        steer_joint.axis == (0.0, 0.0, 1.0)
        and knee_slide.axis == (0.0, 0.0, 1.0)
        and basket_hinge.axis == (0.0, -1.0, 0.0)
        and all(j.articulation_type == ArticulationType.CONTINUOUS and j.axis == (0.0, 1.0, 0.0) for j in wheel_joints)
    )
    ctx.check(
        "articulations match knee scooter mechanisms",
        joints_ok,
        details=(
            f"steer_axis={steer_joint.axis}, knee_axis={knee_slide.axis}, "
            f"basket_axis={basket_hinge.axis}, wheel_axes={[j.axis for j in wheel_joints]}"
        ),
    )

    ctx.expect_gap(
        basket_support,
        front_fork,
        axis="x",
        positive_elem="rear_plate",
        negative_elem="steering_column",
        min_gap=0.08,
        name="basket sits ahead of the steering column",
    )

    rest_front_left = ctx.part_world_position(front_left_wheel)
    with ctx.pose({steer_joint: 0.45}):
        turned_front_left = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "positive steering turns the fork left",
        rest_front_left is not None
        and turned_front_left is not None
        and turned_front_left[1] > rest_front_left[1] + 0.04,
        details=f"rest={rest_front_left}, turned={turned_front_left}",
    )

    rest_post = ctx.part_world_position(knee_post)
    with ctx.pose({knee_slide: 0.08}):
        raised_post = ctx.part_world_position(knee_post)
        ctx.expect_overlap(
            knee_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="left_post_guide",
            min_overlap=0.08,
            name="knee post remains captured in the guide at full extension",
        )
    ctx.check(
        "knee pad post raises upward",
        rest_post is not None and raised_post is not None and raised_post[2] > rest_post[2] + 0.06,
        details=f"rest={rest_post}, raised={raised_post}",
    )

    rest_basket_center = _aabb_center(ctx.part_world_aabb(basket_support))
    with ctx.pose({basket_hinge: 0.85}):
        folded_basket_center = _aabb_center(ctx.part_world_aabb(basket_support))
        ctx.expect_gap(
            basket_support,
            front_fork,
            axis="x",
            positive_elem="rear_plate",
            negative_elem="steering_column",
            min_gap=0.015,
            name="folded basket support still clears the steering column",
        )
    ctx.check(
        "basket support folds upward",
        rest_basket_center is not None
        and folded_basket_center is not None
        and folded_basket_center[2] > rest_basket_center[2] + 0.08,
        details=f"rest={rest_basket_center}, folded={folded_basket_center}",
    )

    wheel_spacing_ok = all(
        ctx.part_world_position(part) is not None
        for part in (front_right_wheel, rear_left_wheel, rear_right_wheel, frame, front_fork)
    )
    ctx.check("wheel assemblies resolve in world space", wheel_spacing_ok, details="One or more wheel or support parts had no world pose.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
