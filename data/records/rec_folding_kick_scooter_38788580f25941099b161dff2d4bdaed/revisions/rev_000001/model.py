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
    rounded_rect_profile,
    tube_from_spline_points,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_tire_mesh(name: str, *, radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.58, -half_width),
        (radius * 0.82, -half_width * 0.92),
        (radius * 0.96, -half_width * 0.58),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.96, half_width * 0.58),
        (radius * 0.82, half_width * 0.92),
        (radius * 0.58, half_width),
        (radius * 0.48, half_width * 0.42),
        (radius * 0.44, 0.0),
        (radius * 0.48, -half_width * 0.42),
        (radius * 0.58, -half_width),
    ]
    return _mesh(name, LatheGeometry(profile, segments=64).rotate_x(pi / 2.0))


def _add_wheel_visuals(
    part,
    *,
    tire_mesh,
    tire_width: float,
    rim_radius: float,
    hub_radius: float,
    axle_radius: float,
    axle_length: float,
    rubber,
    rim_metal,
    hub_metal,
) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.74),
        origin=spin_origin,
        material=rim_metal,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.86, length=tire_width * 0.18),
        origin=Origin(xyz=(0.0, tire_width * 0.18, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="rim_left_face",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.86, length=tire_width * 0.18),
        origin=Origin(xyz=(0.0, -tire_width * 0.18, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
        name="rim_right_face",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=tire_width * 0.84),
        origin=spin_origin,
        material=hub_metal,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=axle_radius, length=axle_length),
        origin=spin_origin,
        material=hub_metal,
        name="axle_spindle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_deck_electric_scooter")

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))
    deck_grey = model.material("deck_grey", rgba=(0.22, 0.23, 0.24, 1.0))
    grip_black = model.material("grip_black", rgba=(0.05, 0.05, 0.05, 1.0))
    alloy = model.material("alloy", rgba=(0.66, 0.68, 0.71, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.36, 0.38, 0.40, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.30, 0.34, 0.55))
    accent_red = model.material("accent_red", rgba=(0.72, 0.14, 0.12, 1.0))

    wheel_radius = 0.125
    wheel_width = 0.055
    deck_bottom = 0.070
    deck_height = 0.055
    deck_top = deck_bottom + deck_height
    fold_origin = (0.255, 0.0, 0.150)
    front_axle_local = (0.190, 0.0, -0.025)
    swing_origin = (-0.330, 0.0, 0.115)
    rear_axle_local = (-0.180, 0.0, -0.025)

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.70, 0.22, 0.14)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )
    deck_shell_mesh = _mesh(
        "scooter_deck_shell",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.620, 0.200, 0.022),
            deck_height,
            cap=True,
            closed=True,
        ),
    )
    deck.visual(
        deck_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_bottom)),
        material=deck_grey,
        name="deck_shell",
    )
    deck.visual(
        Box((0.520, 0.150, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, deck_top + 0.0015)),
        material=grip_black,
        name="grip_pad",
    )
    deck.visual(
        Box((0.070, 0.090, 0.020)),
        origin=Origin(xyz=(0.195, 0.0, deck_top - 0.010)),
        material=matte_black,
        name="front_gusset",
    )
    deck.visual(
        Box((0.055, 0.020, 0.065)),
        origin=Origin(xyz=(fold_origin[0], 0.040, deck_top + 0.0325)),
        material=matte_black,
        name="left_fold_cheek",
    )
    deck.visual(
        Box((0.055, 0.020, 0.065)),
        origin=Origin(xyz=(fold_origin[0], -0.040, deck_top + 0.0325)),
        material=matte_black,
        name="right_fold_cheek",
    )
    deck.visual(
        Box((0.070, 0.100, 0.030)),
        origin=Origin(xyz=(-0.205, 0.0, deck_top + 0.015)),
        material=matte_black,
        name="rear_suspension_mount",
    )
    deck.visual(
        Box((0.050, 0.020, 0.050)),
        origin=Origin(xyz=(swing_origin[0], 0.050, deck_top + 0.025)),
        material=matte_black,
        name="left_swing_cheek",
    )
    deck.visual(
        Box((0.050, 0.020, 0.050)),
        origin=Origin(xyz=(swing_origin[0], -0.050, deck_top + 0.025)),
        material=matte_black,
        name="right_swing_cheek",
    )
    deck.visual(
        Box((0.120, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.101, deck_bottom + 0.012)),
        material=accent_red,
        name="left_side_reflector",
    )
    deck.visual(
        Box((0.120, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.101, deck_bottom + 0.012)),
        material=accent_red,
        name="right_side_reflector",
    )

    stem_fork = model.part("stem_fork")
    stem_fork.inertial = Inertial.from_geometry(
        Box((0.45, 0.62, 0.98)),
        mass=6.5,
        origin=Origin(xyz=(-0.065, 0.0, 0.450)),
    )
    stem_fork.visual(
        Box((0.040, 0.060, 0.050)),
        origin=Origin(),
        material=matte_black,
        name="hinge_lug",
    )
    stem_fork.visual(
        Cylinder(radius=0.026, length=0.820),
        origin=Origin(xyz=(-0.053, 0.0, 0.407), rpy=(0.0, -0.130, 0.0)),
        material=satin_black,
        name="stem_tube",
    )
    stem_fork.visual(
        _mesh(
            "scooter_stem_lower_neck",
            wire_from_points(
                [(0.0, 0.0, 0.0), (0.065, 0.0, 0.130)],
                radius=0.018,
                radial_segments=18,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=satin_black,
        name="lower_neck",
    )
    stem_fork.visual(
        Box((0.050, 0.100, 0.040)),
        origin=Origin(xyz=(0.065, 0.0, 0.132)),
        material=satin_black,
        name="fork_crown",
    )
    stem_fork.visual(
        _mesh(
            "scooter_left_fork_blade",
            wire_from_points(
                [(0.065, 0.045, 0.126), (0.120, 0.045, 0.055), (0.190, 0.045, -0.025)],
                radius=0.011,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=8,
            ),
        ),
        material=satin_black,
        name="left_fork_blade",
    )
    stem_fork.visual(
        _mesh(
            "scooter_right_fork_blade",
            wire_from_points(
                [(0.065, -0.045, 0.126), (0.120, -0.045, 0.055), (0.190, -0.045, -0.025)],
                radius=0.011,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=8,
            ),
        ),
        material=satin_black,
        name="right_fork_blade",
    )
    stem_fork.visual(
        Box((0.020, 0.025, 0.040)),
        origin=Origin(xyz=(front_axle_local[0], 0.0475, front_axle_local[2])),
        material=matte_black,
        name="left_front_dropout",
    )
    stem_fork.visual(
        Box((0.020, 0.025, 0.040)),
        origin=Origin(xyz=(front_axle_local[0], -0.0475, front_axle_local[2])),
        material=matte_black,
        name="right_front_dropout",
    )
    stem_fork.visual(
        Box((0.060, 0.060, 0.054)),
        origin=Origin(xyz=(-0.102, 0.0, 0.840)),
        material=satin_black,
        name="bar_clamp",
    )
    stem_fork.visual(
        Box((0.050, 0.032, 0.042)),
        origin=Origin(xyz=(-0.080, 0.0, 0.778), rpy=(0.0, -0.130, 0.0)),
        material=matte_black,
        name="display_housing",
    )
    stem_fork.visual(
        Box((0.034, 0.004, 0.024)),
        origin=Origin(xyz=(-0.058, 0.0, 0.781), rpy=(0.0, -0.130, 0.0)),
        material=display_glass,
        name="display_screen",
    )
    stem_fork.visual(
        _mesh(
            "scooter_handlebar_bar",
            tube_from_spline_points(
                [
                    (-0.120, -0.290, 0.840),
                    (-0.108, -0.205, 0.855),
                    (-0.095, 0.000, 0.865),
                    (-0.108, 0.205, 0.855),
                    (-0.120, 0.290, 0.840),
                ],
                radius=0.014,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=alloy,
        name="handlebar_bar",
    )
    stem_fork.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(-0.120, 0.3275, 0.840), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    stem_fork.visual(
        Cylinder(radius=0.019, length=0.095),
        origin=Origin(xyz=(-0.120, -0.3275, 0.840), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    swing_arm = model.part("rear_swing_arm")
    swing_arm.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.08)),
        mass=4.2,
        origin=Origin(xyz=(-0.080, 0.0, -0.010)),
    )
    swing_arm.visual(
        Box((0.040, 0.080, 0.045)),
        origin=Origin(),
        material=matte_black,
        name="pivot_lug",
    )
    swing_arm.visual(
        _mesh(
            "scooter_left_swing_rail",
            wire_from_points(
                [(-0.005, 0.046, -0.015), (-0.090, 0.048, -0.026), (-0.180, 0.046, -0.025)],
                radius=0.010,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=8,
            ),
        ),
        material=satin_black,
        name="left_swing_rail",
    )
    swing_arm.visual(
        _mesh(
            "scooter_right_swing_rail",
            wire_from_points(
                [(-0.005, -0.046, -0.015), (-0.090, -0.048, -0.026), (-0.180, -0.046, -0.025)],
                radius=0.010,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.020,
                corner_segments=8,
            ),
        ),
        material=satin_black,
        name="right_swing_rail",
    )
    swing_arm.visual(
        Box((0.022, 0.025, 0.040)),
        origin=Origin(xyz=(rear_axle_local[0], 0.0475, rear_axle_local[2])),
        material=matte_black,
        name="left_rear_dropout",
    )
    swing_arm.visual(
        Box((0.022, 0.025, 0.040)),
        origin=Origin(xyz=(rear_axle_local[0], -0.0475, rear_axle_local[2])),
        material=matte_black,
        name="right_rear_dropout",
    )

    tire_mesh = _wheel_tire_mesh(
        "scooter_tire_mesh",
        radius=wheel_radius,
        width=wheel_width,
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=2.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_wheel,
        tire_mesh=tire_mesh,
        tire_width=wheel_width,
        rim_radius=0.086,
        hub_radius=0.058,
        axle_radius=0.012,
        axle_length=0.070,
        rubber=grip_black,
        rim_metal=alloy,
        hub_metal=dark_alloy,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=3.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_wheel,
        tire_mesh=tire_mesh,
        tire_width=wheel_width,
        rim_radius=0.086,
        hub_radius=0.060,
        axle_radius=0.012,
        axle_length=0.070,
        rubber=grip_black,
        rim_metal=alloy,
        hub_metal=dark_alloy,
    )

    model.articulation(
        "deck_to_stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem_fork,
        origin=Origin(xyz=fold_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.10,
        ),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.REVOLUTE,
        parent=stem_fork,
        child=front_wheel,
        origin=Origin(xyz=front_axle_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=25.0,
            lower=-6.50,
            upper=6.50,
        ),
    )
    model.articulation(
        "deck_to_swing_arm",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=swing_arm,
        origin=Origin(xyz=swing_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-0.12,
            upper=0.22,
        ),
    )
    model.articulation(
        "swing_arm_to_rear_wheel",
        ArticulationType.REVOLUTE,
        parent=swing_arm,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=25.0,
            lower=-6.50,
            upper=6.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem_fork = object_model.get_part("stem_fork")
    front_wheel = object_model.get_part("front_wheel")
    swing_arm = object_model.get_part("rear_swing_arm")
    rear_wheel = object_model.get_part("rear_wheel")

    fold = object_model.get_articulation("deck_to_stem_fold")
    front_axle = object_model.get_articulation("stem_to_front_wheel")
    swing = object_model.get_articulation("deck_to_swing_arm")
    rear_axle = object_model.get_articulation("swing_arm_to_rear_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.check(
        "primary_joint_axes_and_limits",
        fold.axis == (0.0, 1.0, 0.0)
        and front_axle.axis == (0.0, 1.0, 0.0)
        and swing.axis == (0.0, 1.0, 0.0)
        and rear_axle.axis == (0.0, 1.0, 0.0)
        and fold.motion_limits is not None
        and fold.motion_limits.lower is not None
        and fold.motion_limits.upper is not None
        and fold.motion_limits.lower <= -0.95
        and fold.motion_limits.upper >= 0.0
        and swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and swing.motion_limits.upper >= 0.18
        and front_axle.motion_limits is not None
        and front_axle.motion_limits.upper is not None
        and front_axle.motion_limits.lower is not None
        and front_axle.motion_limits.upper >= 6.0
        and front_axle.motion_limits.lower <= -6.0
        and rear_axle.motion_limits is not None
        and rear_axle.motion_limits.upper is not None
        and rear_axle.motion_limits.lower is not None
        and rear_axle.motion_limits.upper >= 6.0
        and rear_axle.motion_limits.lower <= -6.0,
        "Fold hinge, wheel axles, and swing arm should all rotate about left-right axes with realistic travel.",
    )
    ctx.expect_contact(deck, stem_fork, name="deck_to_stem_hinge_contact")
    ctx.expect_contact(deck, swing_arm, name="deck_to_swing_pivot_contact")
    ctx.expect_contact(stem_fork, front_wheel, name="front_wheel_axle_contact")
    ctx.expect_contact(swing_arm, rear_wheel, name="rear_wheel_axle_contact")
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.92,
        max_gap=0.99,
        name="scooter_wheelbase",
    )

    with ctx.pose({fold: -1.0}):
        handlebar_aabb = ctx.part_element_world_aabb(stem_fork, elem="handlebar_bar")
        deck_aabb = ctx.part_world_aabb(deck)
        folded_ok = (
            handlebar_aabb is not None
            and deck_aabb is not None
            and handlebar_aabb[1][2] < deck_aabb[1][2] + 0.42
        )
        ctx.check(
            "folded_stem_lowers_handlebar",
            folded_ok,
            "The folded stem should carry the handlebar down toward the deck instead of staying upright.",
        )

    rear_rest = ctx.part_world_position(rear_wheel)
    with ctx.pose({swing: 0.18}):
        rear_raised = ctx.part_world_position(rear_wheel)
        swing_ok = (
            rear_rest is not None
            and rear_raised is not None
            and rear_raised[2] > rear_rest[2] + 0.015
        )
        ctx.check(
            "rear_swing_arm_moves_wheel",
            swing_ok,
            "Raising the swing arm should lift the rear wheel center.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
