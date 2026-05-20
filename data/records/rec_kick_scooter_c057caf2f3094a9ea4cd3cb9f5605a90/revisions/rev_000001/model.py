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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_visuals(part, mesh_name: str, *, radius: float, width: float, rubber, metal, dark) -> None:
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.70, -half_width),
        (radius * 0.88, -half_width * 0.96),
        (radius * 0.97, -half_width * 0.52),
        (radius, 0.0),
        (radius * 0.97, half_width * 0.52),
        (radius * 0.88, half_width * 0.96),
        (radius * 0.70, half_width),
        (radius * 0.52, half_width * 0.62),
        (radius * 0.42, 0.0),
        (radius * 0.52, -half_width * 0.62),
        (radius * 0.70, -half_width),
    ]
    tire = LatheGeometry(tire_profile, segments=56).rotate_x(pi / 2.0)
    part.visual(_save_mesh(mesh_name, tire), material=rubber, name="tire")

    wheel_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(Cylinder(radius=radius * 0.78, length=width * 0.52), origin=wheel_axis, material=metal, name="rim")
    part.visual(Cylinder(radius=radius * 0.28, length=width * 0.78), origin=wheel_axis, material=dark, name="hub")
    part.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="left_axle_stub",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="right_axle_stub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_folding_scooter")

    deck_gray = model.material("deck_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    accent = model.material("accent", rgba=(0.77, 0.12, 0.10, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.62, 0.13, 0.24)),
        mass=4.8,
        origin=Origin(xyz=(0.02, 0.0, 0.12)),
    )
    deck_profile = [
        (-0.220, -0.056),
        (0.155, -0.056),
        (0.193, -0.047),
        (0.214, -0.028),
        (0.224, 0.0),
        (0.214, 0.028),
        (0.193, 0.047),
        (0.155, 0.056),
        (-0.208, 0.056),
        (-0.223, 0.046),
        (-0.232, 0.015),
        (-0.232, -0.015),
        (-0.223, -0.046),
    ]
    deck_shell = ExtrudeGeometry.from_z0(deck_profile, 0.018).translate(0.0, 0.0, 0.050)
    deck.visual(_save_mesh("deck_shell", deck_shell), material=deck_gray, name="deck_shell")
    deck.visual(Box((0.31, 0.080, 0.002)), origin=Origin(xyz=(-0.005, 0.0, 0.069)), material=rubber, name="grip_tape")
    deck.visual(
        _save_mesh(
            "front_left_strut",
            tube_from_spline_points(
                [(0.165, 0.028, 0.060), (0.194, 0.026, 0.108), (0.212, 0.024, 0.164), (0.220, 0.022, 0.215)],
                radius=0.010,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=dark_steel,
        name="front_left_strut",
    )
    deck.visual(
        _save_mesh(
            "front_right_strut",
            tube_from_spline_points(
                [(0.165, -0.028, 0.060), (0.194, -0.026, 0.108), (0.212, -0.024, 0.164), (0.220, -0.022, 0.215)],
                radius=0.010,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=dark_steel,
        name="front_right_strut",
    )
    deck.visual(Box((0.022, 0.010, 0.060)), origin=Origin(xyz=(0.220, 0.022, 0.245)), material=dark_steel, name="front_left_yoke")
    deck.visual(Box((0.022, 0.010, 0.060)), origin=Origin(xyz=(0.220, -0.022, 0.245)), material=dark_steel, name="front_right_yoke")
    deck.visual(Cylinder(radius=0.014, length=0.020), origin=Origin(xyz=(0.220, 0.0, 0.205)), material=steel, name="steering_boss")
    deck.visual(
        _save_mesh(
            "rear_left_arm",
            tube_from_spline_points(
                [(-0.188, 0.030, 0.060), (-0.238, 0.030, 0.078), (-0.290, 0.030, 0.102), (-0.337, 0.030, 0.122)],
                radius=0.010,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=dark_steel,
        name="rear_left_arm",
    )
    deck.visual(
        _save_mesh(
            "rear_right_arm",
            tube_from_spline_points(
                [(-0.188, -0.030, 0.060), (-0.238, -0.030, 0.078), (-0.290, -0.030, 0.102), (-0.337, -0.030, 0.122)],
                radius=0.010,
                samples_per_segment=12,
                radial_segments=16,
            ),
        ),
        material=dark_steel,
        name="rear_right_arm",
    )
    deck.visual(
        Box((0.038, 0.012, 0.022)),
        origin=Origin(xyz=(-0.337, 0.031, 0.100)),
        material=dark_steel,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.038, 0.012, 0.022)),
        origin=Origin(xyz=(-0.337, -0.031, 0.100)),
        material=dark_steel,
        name="rear_right_dropout",
    )
    deck.visual(Box((0.045, 0.060, 0.018)), origin=Origin(xyz=(-0.205, 0.0, 0.069)), material=accent, name="tail_plate")

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.034),
        mass=1.6,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(rear_wheel, "rear_wheel_tire", radius=0.10, width=0.034, rubber=rubber, metal=steel, dark=dark_steel)

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.22, 0.08, 0.34)),
        mass=2.2,
        origin=Origin(xyz=(0.07, 0.0, -0.08)),
    )
    front_fork.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="steering_post",
    )
    front_fork.visual(
        Box((0.028, 0.030, 0.028)),
        origin=Origin(xyz=(0.028, 0.0, 0.006)),
        material=dark_steel,
        name="upper_neck",
    )
    front_fork.visual(
        Box((0.070, 0.028, 0.094)),
        origin=Origin(xyz=(0.060, 0.0, -0.048)),
        material=dark_steel,
        name="lower_neck",
    )
    front_fork.visual(
        Box((0.016, 0.010, 0.026)),
        origin=Origin(xyz=(0.018, -0.010, 0.035)),
        material=dark_steel,
        name="hinge_left_lug",
    )
    front_fork.visual(
        Box((0.016, 0.010, 0.026)),
        origin=Origin(xyz=(0.018, 0.010, 0.035)),
        material=dark_steel,
        name="hinge_right_lug",
    )
    front_fork.visual(
        Box((0.012, 0.008, 0.020)),
        origin=Origin(xyz=(0.018, -0.016, 0.020)),
        material=dark_steel,
        name="left_lug_support",
    )
    front_fork.visual(
        Box((0.012, 0.008, 0.020)),
        origin=Origin(xyz=(0.018, 0.016, 0.020)),
        material=dark_steel,
        name="right_lug_support",
    )
    front_fork.visual(Box((0.080, 0.074, 0.022)), origin=Origin(xyz=(0.095, 0.0, -0.103)), material=dark_steel, name="crown")
    front_fork.visual(
        Box((0.046, 0.010, 0.120)),
        origin=Origin(xyz=(0.130, 0.031, -0.155)),
        material=dark_steel,
        name="front_left_blade",
    )
    front_fork.visual(
        Box((0.046, 0.010, 0.120)),
        origin=Origin(xyz=(0.130, -0.031, -0.155)),
        material=dark_steel,
        name="front_right_blade",
    )
    front_fork.visual(
        Box((0.020, 0.012, 0.018)),
        origin=Origin(xyz=(0.156, 0.031, -0.215)),
        material=dark_steel,
        name="front_left_dropout",
    )
    front_fork.visual(
        Box((0.020, 0.012, 0.018)),
        origin=Origin(xyz=(0.156, -0.031, -0.215)),
        material=dark_steel,
        name="front_right_dropout",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=0.034),
        mass=1.6,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(front_wheel, "front_wheel_tire", radius=0.10, width=0.034, rubber=rubber, metal=steel, dark=dark_steel)

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.24, 0.46, 0.92)),
        mass=2.7,
        origin=Origin(xyz=(-0.02, 0.0, 0.44)),
    )
    stem.visual(
        Box((0.018, 0.012, 0.026)),
        origin=Origin(),
        material=steel,
        name="hinge_knuckle",
    )
    stem.visual(Box((0.022, 0.022, 0.024)), origin=Origin(xyz=(-0.014, 0.0, 0.018)), material=steel, name="hinge_block")
    stem.visual(Box((0.024, 0.024, 0.420)), origin=Origin(xyz=(-0.020, 0.0, 0.230)), material=dark_steel, name="lower_stem")
    stem.visual(Box((0.024, 0.024, 0.430)), origin=Origin(xyz=(-0.032, 0.0, 0.640)), material=dark_steel, name="upper_stem")
    stem.visual(Cylinder(radius=0.014, length=0.150), origin=Origin(xyz=(-0.038, 0.0, 0.835), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="crossbar")
    stem.visual(Box((0.056, 0.040, 0.030)), origin=Origin(xyz=(-0.034, 0.0, 0.823)), material=accent, name="bar_clamp")
    stem.visual(
        Box((0.018, 0.022, 0.020)),
        origin=Origin(xyz=(-0.038, 0.077, 0.823)),
        material=steel,
        name="left_hinge_body",
    )
    stem.visual(
        Box((0.018, 0.022, 0.020)),
        origin=Origin(xyz=(-0.038, -0.077, 0.823)),
        material=steel,
        name="right_hinge_body",
    )
    stem.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.050, 0.089, 0.835)),
        material=steel,
        name="left_inner_lug",
    )
    stem.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.026, 0.089, 0.835)),
        material=steel,
        name="left_outer_lug",
    )
    stem.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.050, -0.089, 0.835)),
        material=steel,
        name="right_inner_lug",
    )
    stem.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(-0.026, -0.089, 0.835)),
        material=steel,
        name="right_outer_lug",
    )

    left_grip = model.part("left_grip")
    left_grip.inertial = Inertial.from_geometry(
        Box((0.08, 0.13, 0.05)),
        mass=0.24,
        origin=Origin(xyz=(0.018, 0.046, 0.0)),
    )
    left_grip.visual(
        Box((0.012, 0.018, 0.018)),
        origin=Origin(),
        material=steel,
        name="hinge_knuckle",
    )
    left_grip.visual(
        Cylinder(radius=0.016, length=0.068),
        origin=Origin(xyz=(0.018, 0.041, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="grip_body",
    )
    left_grip.visual(Box((0.016, 0.018, 0.020)), origin=Origin(xyz=(0.014, 0.010, 0.0)), material=steel, name="hinge_block")

    right_grip = model.part("right_grip")
    right_grip.inertial = Inertial.from_geometry(
        Box((0.08, 0.13, 0.05)),
        mass=0.24,
        origin=Origin(xyz=(0.018, -0.046, 0.0)),
    )
    right_grip.visual(
        Box((0.012, 0.018, 0.018)),
        origin=Origin(),
        material=steel,
        name="hinge_knuckle",
    )
    right_grip.visual(
        Cylinder(radius=0.016, length=0.068),
        origin=Origin(xyz=(0.018, -0.041, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="grip_body",
    )
    right_grip.visual(Box((0.016, 0.018, 0.020)), origin=Origin(xyz=(0.014, -0.010, 0.0)), material=steel, name="hinge_block")

    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.337, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.220, 0.0, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.156, 0.0, -0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=stem,
        origin=Origin(xyz=(0.020, 0.0, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.2, lower=0.0, upper=1.30),
    )
    model.articulation(
        "left_grip_fold",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=left_grip,
        origin=Origin(xyz=(-0.038, 0.089, 0.835)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=4.0, lower=-1.60, upper=0.0),
    )
    model.articulation(
        "right_grip_fold",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=right_grip,
        origin=Origin(xyz=(-0.038, -0.089, 0.835)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=4.0, lower=0.0, upper=1.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    rear_wheel = object_model.get_part("rear_wheel")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    stem = object_model.get_part("stem")
    left_grip = object_model.get_part("left_grip")
    right_grip = object_model.get_part("right_grip")

    steering = object_model.get_articulation("steering")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    stem_fold = object_model.get_articulation("stem_fold")
    left_grip_fold = object_model.get_articulation("left_grip_fold")
    right_grip_fold = object_model.get_articulation("right_grip_fold")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(front_fork, stem, reason="Simplified folding stem clevis uses nested hinge solids around the pivot.")
    ctx.allow_overlap(stem, left_grip, reason="Simplified left folding grip hinge uses nested lug solids around the pivot.")
    ctx.allow_overlap(stem, right_grip, reason="Simplified right folding grip hinge uses nested lug solids around the pivot.")

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

    ctx.check("all_primary_parts_present", all([deck, rear_wheel, front_fork, front_wheel, stem, left_grip, right_grip]))
    ctx.check("steering_axis_is_vertical", tuple(steering.axis) == (0.0, 0.0, 1.0))
    ctx.check("wheels_spin_on_axles", tuple(front_wheel_spin.axis) == (0.0, 1.0, 0.0) and tuple(rear_wheel_spin.axis) == (0.0, 1.0, 0.0))
    ctx.check("stem_folds_about_lateral_hinge", tuple(stem_fold.axis) == (0.0, 1.0, 0.0))
    ctx.check(
        "grips_fold_from_bar_ends",
        tuple(left_grip_fold.axis) == (1.0, 0.0, 0.0)
        and tuple(right_grip_fold.axis) == (1.0, 0.0, 0.0)
        and left_grip_fold.motion_limits is not None
        and right_grip_fold.motion_limits is not None
        and left_grip_fold.motion_limits.upper == 0.0
        and right_grip_fold.motion_limits.lower == 0.0,
    )

    ctx.expect_contact(front_fork, deck, elem_a="steering_post", elem_b="steering_boss", contact_tol=0.001, name="fork_seats_on_head_tube")
    ctx.expect_contact(stem, front_fork, elem_a="hinge_knuckle", elem_b="hinge_left_lug", contact_tol=0.001, name="stem_attached_at_fold_hinge")
    ctx.expect_contact(left_grip, stem, elem_a="hinge_knuckle", elem_b="left_inner_lug", contact_tol=0.001, name="left_grip_attached_to_bar_end")
    ctx.expect_contact(right_grip, stem, elem_a="hinge_knuckle", elem_b="right_inner_lug", contact_tol=0.001, name="right_grip_attached_to_bar_end")
    ctx.expect_contact(front_wheel, front_fork, elem_a="left_axle_stub", elem_b="front_left_dropout", contact_tol=0.001, name="front_wheel_left_dropout_contact")
    ctx.expect_contact(front_wheel, front_fork, elem_a="right_axle_stub", elem_b="front_right_dropout", contact_tol=0.001, name="front_wheel_right_dropout_contact")
    ctx.expect_contact(rear_wheel, deck, elem_a="left_axle_stub", elem_b="rear_left_dropout", contact_tol=0.001, name="rear_wheel_left_dropout_contact")
    ctx.expect_contact(rear_wheel, deck, elem_a="right_axle_stub", elem_b="rear_right_dropout", contact_tol=0.001, name="rear_wheel_right_dropout_contact")

    with ctx.pose({stem_fold: 1.18, left_grip_fold: -1.08, right_grip_fold: 1.08}):
        ctx.fail_if_parts_overlap_in_current_pose(name="storage_pose_no_part_overlap")
        ctx.expect_contact(stem, front_fork, elem_a="hinge_knuckle", elem_b="hinge_left_lug", contact_tol=0.001, name="stem_stays_clipped_when_folded")
        ctx.expect_contact(left_grip, stem, elem_a="hinge_knuckle", elem_b="left_inner_lug", contact_tol=0.001, name="left_grip_stays_clipped_when_folded")
        ctx.expect_contact(right_grip, stem, elem_a="hinge_knuckle", elem_b="right_inner_lug", contact_tol=0.001, name="right_grip_stays_clipped_when_folded")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
