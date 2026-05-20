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
    rounded_rect_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wheel_visuals(
    part,
    *,
    prefix: str,
    wheel_radius: float,
    tire_width: float,
    alloy,
    rubber,
    dark_steel,
) -> None:
    half_width = tire_width * 0.5
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    tire_profile = [
        (wheel_radius * 0.72, -half_width * 0.98),
        (wheel_radius * 0.86, -half_width),
        (wheel_radius * 0.96, -half_width * 0.78),
        (wheel_radius, -half_width * 0.28),
        (wheel_radius, half_width * 0.28),
        (wheel_radius * 0.96, half_width * 0.78),
        (wheel_radius * 0.86, half_width),
        (wheel_radius * 0.72, half_width * 0.98),
        (wheel_radius * 0.58, half_width * 0.34),
        (wheel_radius * 0.54, 0.0),
        (wheel_radius * 0.58, -half_width * 0.34),
        (wheel_radius * 0.72, -half_width * 0.98),
    ]
    tire_mesh = _save_mesh(
        f"{prefix}_tire",
        LatheGeometry(tire_profile, segments=56).rotate_y(math.pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")

    part.visual(
        Cylinder(radius=wheel_radius * 0.71, length=tire_width * 0.78),
        origin=spin_origin,
        material=alloy,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.56, length=tire_width * 0.22),
        origin=Origin(xyz=(tire_width * 0.19, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="rim_face_outer",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.56, length=tire_width * 0.22),
        origin=Origin(xyz=(-tire_width * 0.19, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="rim_face_inner",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.20, length=tire_width * 1.16),
        origin=spin_origin,
        material=dark_steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.08, length=tire_width + 0.039),
        origin=spin_origin,
        material=alloy,
        name="axle_sleeve",
    )


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_delivery_kick_scooter")

    frame_blue = model.material("frame_blue", rgba=(0.14, 0.21, 0.28, 1.0))
    deck_black = model.material("deck_black", rgba=(0.09, 0.09, 0.10, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.26, 0.28, 0.30, 1.0))
    alloy = model.material("alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_black = model.material("grip_black", rgba=(0.07, 0.07, 0.07, 1.0))
    accent_red = model.material("accent_red", rgba=(0.72, 0.16, 0.12, 1.0))

    steering_rake = 0.30

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.28, 1.14, 0.74)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.02, 0.31)),
    )

    deck_geom = ExtrudeGeometry(rounded_rect_profile(0.22, 0.62, 0.040), 0.038, center=True)
    chassis.visual(
        _save_mesh("deck_shell", deck_geom),
        origin=Origin(xyz=(0.0, -0.05, 0.090)),
        material=frame_blue,
        name="deck_shell",
    )
    chassis.visual(
        Box((0.17, 0.50, 0.007)),
        origin=Origin(xyz=(0.0, -0.05, 0.1135)),
        material=deck_black,
        name="deck_pad",
    )
    chassis.visual(
        _save_mesh(
            "head_tube",
            tube_from_spline_points(
                [
                    (0.0, 0.16, 0.102),
                    (0.0, 0.24, 0.205),
                    (0.0, 0.32, 0.355),
                    (0.0, 0.392, 0.500),
                ],
                radius=0.030,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=fork_gray,
        name="head_tube",
    )
    chassis.visual(
        _save_mesh(
            "left_down_rail",
            tube_from_spline_points(
                [
                    (0.082, 0.14, 0.098),
                    (0.076, 0.21, 0.125),
                    (0.050, 0.29, 0.205),
                    (0.020, 0.34, 0.290),
                ],
                radius=0.018,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="left_down_rail",
    )
    chassis.visual(
        _save_mesh(
            "right_down_rail",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (0.082, 0.14, 0.098),
                        (0.076, 0.21, 0.125),
                        (0.050, 0.29, 0.205),
                        (0.020, 0.34, 0.290),
                    ]
                ),
                radius=0.018,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="right_down_rail",
    )
    chassis.visual(
        Box((0.11, 0.07, 0.045)),
        origin=Origin(xyz=(0.0, 0.175, 0.110)),
        material=fork_gray,
        name="front_neck_block",
    )
    chassis.visual(
        _save_mesh(
            "left_rear_stay",
            tube_from_spline_points(
                [
                    (0.092, -0.22, 0.095),
                    (0.088, -0.33, 0.118),
                    (0.080, -0.44, 0.155),
                    (0.066, -0.540, 0.188),
                ],
                radius=0.016,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="left_rear_stay",
    )
    chassis.visual(
        _save_mesh(
            "right_rear_stay",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (0.092, -0.22, 0.095),
                        (0.088, -0.33, 0.118),
                        (0.080, -0.44, 0.155),
                        (0.066, -0.540, 0.188),
                    ]
                ),
                radius=0.016,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=frame_blue,
        name="right_rear_stay",
    )
    chassis.visual(
        Box((0.018, 0.040, 0.060)),
        origin=Origin(xyz=(0.056, -0.540, 0.188)),
        material=fork_gray,
        name="left_dropout",
    )
    chassis.visual(
        Box((0.018, 0.040, 0.060)),
        origin=Origin(xyz=(-0.056, -0.540, 0.188)),
        material=fork_gray,
        name="right_dropout",
    )
    chassis.visual(
        Box((0.12, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, -0.525, 0.390)),
        material=fork_gray,
        name="rear_fender",
    )
    chassis.visual(
        Box((0.018, 0.080, 0.187)),
        origin=Origin(xyz=(0.058, -0.525, 0.2965)),
        material=fork_gray,
        name="rear_fender_left_strut",
    )
    chassis.visual(
        Box((0.018, 0.080, 0.187)),
        origin=Origin(xyz=(-0.058, -0.525, 0.2965)),
        material=fork_gray,
        name="rear_fender_right_strut",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.055),
        mass=2.6,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        rear_wheel,
        prefix="rear_wheel",
        wheel_radius=0.18,
        tire_width=0.055,
        alloy=alloy,
        rubber=rubber,
        dark_steel=fork_gray,
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.18, 0.40, 0.72)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.05, -0.14)),
    )
    front_fork.visual(
        Cylinder(radius=0.024, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=fork_gray,
        name="steerer_tube",
    )
    front_fork.visual(
        Box((0.090, 0.044, 0.026)),
        origin=Origin(xyz=(0.0, 0.044, -0.024)),
        material=fork_gray,
        name="neck_clamp_body",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(-0.020, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fork_gray,
        name="left_hinge_ear",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.020, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fork_gray,
        name="right_hinge_ear",
    )
    front_fork.visual(
        Box((0.012, 0.040, 0.020)),
        origin=Origin(xyz=(-0.020, 0.020, -0.012)),
        material=fork_gray,
        name="left_hinge_gusset",
    )
    front_fork.visual(
        Box((0.012, 0.040, 0.020)),
        origin=Origin(xyz=(0.020, 0.020, -0.012)),
        material=fork_gray,
        name="right_hinge_gusset",
    )
    front_fork.visual(
        Box((0.050, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, 0.028, -0.020)),
        material=fork_gray,
        name="hinge_bridge",
    )
    front_fork.visual(
        Box((0.125, 0.120, 0.060)),
        origin=Origin(xyz=(0.0, 0.035, -0.242)),
        material=fork_gray,
        name="fork_crown",
    )
    front_fork.visual(
        _save_mesh(
            "left_fork_blade",
            tube_from_spline_points(
                [
                    (-0.058, 0.040, -0.226),
                    (-0.066, 0.048, -0.320),
                    (-0.071, 0.055, -0.410),
                    (-0.072, 0.060, -0.505),
                ],
                radius=0.018,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=fork_gray,
        name="left_fork_blade",
    )
    front_fork.visual(
        _save_mesh(
            "right_fork_blade",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (-0.058, 0.040, -0.226),
                        (-0.066, 0.048, -0.320),
                        (-0.071, 0.055, -0.410),
                        (-0.072, 0.060, -0.505),
                    ]
                ),
                radius=0.018,
                samples_per_segment=10,
                radial_segments=16,
            ),
        ),
        material=fork_gray,
        name="right_fork_blade",
    )
    front_fork.visual(
        Box((0.018, 0.030, 0.050)),
        origin=Origin(xyz=(0.056, 0.060, -0.505)),
        material=fork_gray,
        name="left_axle_block",
    )
    front_fork.visual(
        Box((0.018, 0.030, 0.050)),
        origin=Origin(xyz=(-0.056, 0.060, -0.505)),
        material=fork_gray,
        name="right_axle_block",
    )
    front_fork.visual(
        Box((0.100, 0.020, 0.100)),
        origin=Origin(xyz=(0.0, 0.030, -0.080)),
        material=fork_gray,
        name="fork_basket_plate",
    )
    front_fork.visual(
        _save_mesh(
            "left_basket_strut",
            tube_from_spline_points(
                [
                    (-0.032, 0.078, -0.223),
                    (-0.070, 0.128, -0.175),
                    (-0.100, 0.165, -0.128),
                ],
                radius=0.012,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=fork_gray,
        name="left_basket_strut",
    )
    front_fork.visual(
        _save_mesh(
            "right_basket_strut",
            tube_from_spline_points(
                _mirror_x(
                    [
                        (-0.032, 0.078, -0.223),
                        (-0.070, 0.128, -0.175),
                        (-0.100, 0.165, -0.128),
                    ]
                ),
                radius=0.012,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=fork_gray,
        name="right_basket_strut",
    )

    basket = model.part("basket")
    basket.inertial = Inertial.from_geometry(
        Box((0.42, 0.32, 0.26)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )
    basket.visual(
        Box((0.40, 0.30, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=basket_gray,
        name="basket_floor",
    )
    basket.visual(
        Box((0.40, 0.018, 0.24)),
        origin=Origin(xyz=(0.0, 0.141, 0.120)),
        material=basket_gray,
        name="basket_front_wall",
    )
    basket.visual(
        Box((0.40, 0.018, 0.24)),
        origin=Origin(xyz=(0.0, -0.141, 0.120)),
        material=basket_gray,
        name="basket_rear_wall",
    )
    basket.visual(
        Box((0.018, 0.30, 0.24)),
        origin=Origin(xyz=(0.191, 0.0, 0.120)),
        material=basket_gray,
        name="basket_left_wall",
    )
    basket.visual(
        Box((0.018, 0.30, 0.24)),
        origin=Origin(xyz=(-0.191, 0.0, 0.120)),
        material=basket_gray,
        name="basket_right_wall",
    )
    basket.visual(
        Box((0.12, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.150, 0.000)),
        material=fork_gray,
        name="basket_mount",
    )
    basket.visual(
        Box((0.40, 0.015, 0.015)),
        origin=Origin(xyz=(0.0, 0.142, 0.242)),
        material=alloy,
        name="basket_front_rim",
    )
    basket.visual(
        Box((0.40, 0.015, 0.015)),
        origin=Origin(xyz=(0.0, -0.142, 0.242)),
        material=alloy,
        name="basket_rear_rim",
    )
    basket.visual(
        Box((0.015, 0.30, 0.015)),
        origin=Origin(xyz=(0.192, 0.0, 0.242)),
        material=alloy,
        name="basket_left_rim",
    )
    basket.visual(
        Box((0.015, 0.30, 0.015)),
        origin=Origin(xyz=(-0.192, 0.0, 0.242)),
        material=alloy,
        name="basket_right_rim",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.055),
        mass=2.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        prefix="front_wheel",
        wheel_radius=0.20,
        tire_width=0.055,
        alloy=alloy,
        rubber=rubber,
        dark_steel=fork_gray,
    )

    stem_upper = model.part("stem_upper")
    stem_upper.inertial = Inertial.from_geometry(
        Box((0.62, 0.14, 0.66)),
        mass=3.2,
        origin=Origin(xyz=(0.0, -0.02, 0.32)),
    )
    stem_upper.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fork_gray,
        name="fold_hinge_barrel",
    )
    stem_upper.visual(
        Box((0.030, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, -0.022, 0.028)),
        material=fork_gray,
        name="fold_yoke",
    )
    stem_upper.visual(
        _save_mesh(
            "upper_stem_tube",
            tube_from_spline_points(
                [
                    (0.0, -0.020, 0.040),
                    (0.0, -0.026, 0.220),
                    (0.0, -0.032, 0.410),
                    (0.0, -0.036, 0.565),
                ],
                radius=0.021,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_blue,
        name="upper_stem_tube",
    )
    stem_upper.visual(
        Box((0.090, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, -0.032, 0.575)),
        material=fork_gray,
        name="handlebar_clamp",
    )
    stem_upper.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, -0.018, 0.600)),
        material=fork_gray,
        name="handlebar_riser",
    )
    stem_upper.visual(
        _save_mesh(
            "handlebar_bar",
            tube_from_spline_points(
                [
                    (-0.305, -0.016, 0.590),
                    (-0.210, -0.010, 0.608),
                    (-0.090, -0.004, 0.612),
                    (0.090, -0.004, 0.612),
                    (0.210, -0.010, 0.608),
                    (0.305, -0.016, 0.590),
                ],
                radius=0.014,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=fork_gray,
        name="handlebar_bar",
    )
    stem_upper.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.305, -0.018, 0.585), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    stem_upper.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(-0.305, -0.018, 0.585), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    stem_upper.visual(
        Box((0.048, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.035, 0.490)),
        material=accent_red,
        name="fold_latch",
    )

    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, -0.540, 0.188)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=28.0),
    )
    model.articulation(
        "front_steer",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_fork,
        origin=Origin(xyz=(0.0, 0.420, 0.560), rpy=(steering_rake, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "basket_mount",
        ArticulationType.FIXED,
        parent=front_fork,
        child=basket,
        origin=Origin(xyz=(0.0, 0.230, -0.080)),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.060, -0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=28.0),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=stem_upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    front_fork = object_model.get_part("front_fork")
    basket = object_model.get_part("basket")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    stem_upper = object_model.get_part("stem_upper")

    front_steer = object_model.get_articulation("front_steer")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    stem_fold = object_model.get_articulation("stem_fold")

    ctx.allow_overlap(
        chassis,
        front_fork,
        elem_a="head_tube",
        elem_b="steerer_tube",
        reason=(
            "The steering bearing is represented with a simplified solid head tube, "
            "so the steerer intentionally passes through that proxy sleeve."
        ),
    )

    ctx.check(
        "front steering uses continuous bearing",
        front_steer.articulation_type == ArticulationType.CONTINUOUS
        and front_steer.motion_limits is not None
        and front_steer.motion_limits.lower is None
        and front_steer.motion_limits.upper is None,
        details=f"type={front_steer.articulation_type}, limits={front_steer.motion_limits}",
    )
    ctx.check(
        "both wheels rotate continuously",
        front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"front={front_wheel_spin.articulation_type}, "
            f"rear={rear_wheel_spin.articulation_type}"
        ),
    )
    ctx.check(
        "stem fold hinge has realistic folding travel",
        stem_fold.articulation_type == ArticulationType.REVOLUTE
        and stem_fold.motion_limits is not None
        and stem_fold.motion_limits.lower == 0.0
        and stem_fold.motion_limits.upper is not None
        and stem_fold.motion_limits.upper >= 1.35,
        details=f"limits={stem_fold.motion_limits}",
    )

    with ctx.pose({front_steer: 0.0, stem_fold: 0.0}):
        ctx.expect_contact(
            basket,
            front_fork,
            elem_a="basket_mount",
            elem_b="fork_basket_plate",
            contact_tol=0.002,
            name="basket mount seats on fork plate",
        )
        ctx.expect_gap(
            basket,
            front_wheel,
            axis="z",
            positive_elem="basket_floor",
            negative_elem="tire",
            min_gap=0.020,
            name="basket clears the front tire",
        )
        ctx.expect_gap(
            front_wheel,
            chassis,
            axis="y",
            positive_elem="tire",
            negative_elem="deck_shell",
            min_gap=0.050,
            name="front wheel stays ahead of the deck",
        )
        ctx.expect_gap(
            chassis,
            rear_wheel,
            axis="y",
            positive_elem="deck_shell",
            negative_elem="tire",
            max_gap=0.060,
            max_penetration=0.0,
            name="rear wheel tucks close behind the deck tail",
        )

    rest_front_pos = ctx.part_world_position(front_wheel)
    rest_basket_pos = ctx.part_world_position(basket)
    with ctx.pose({front_steer: 0.60}):
        steered_front_pos = ctx.part_world_position(front_wheel)
        steered_basket_pos = ctx.part_world_position(basket)
    ctx.check(
        "steering turns the forked load and wheel together",
        rest_front_pos is not None
        and steered_front_pos is not None
        and rest_basket_pos is not None
        and steered_basket_pos is not None
        and abs(steered_front_pos[0] - rest_front_pos[0]) > 0.02
        and abs(steered_basket_pos[0] - rest_basket_pos[0]) > 0.06,
        details=(
            f"front_rest={rest_front_pos}, front_steered={steered_front_pos}, "
            f"basket_rest={rest_basket_pos}, basket_steered={steered_basket_pos}"
        ),
    )

    rest_bar_aabb = ctx.part_element_world_aabb(stem_upper, elem="handlebar_bar")
    with ctx.pose({stem_fold: 1.35}):
        folded_bar_aabb = ctx.part_element_world_aabb(stem_upper, elem="handlebar_bar")
    rest_bar_center = _aabb_center(rest_bar_aabb) if rest_bar_aabb is not None else None
    folded_bar_center = _aabb_center(folded_bar_aabb) if folded_bar_aabb is not None else None
    ctx.check(
        "stem folds backward and downward from the neck clamp",
        rest_bar_center is not None
        and folded_bar_center is not None
        and folded_bar_center[1] < rest_bar_center[1] - 0.20
        and folded_bar_center[2] < rest_bar_center[2] - 0.12,
        details=f"rest={rest_bar_center}, folded={folded_bar_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
