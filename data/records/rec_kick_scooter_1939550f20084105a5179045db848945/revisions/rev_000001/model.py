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
    tube_from_spline_points,
)


def _wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    hub_length: float,
    rim_radius: float,
    rubber,
    metal,
    dark_metal,
) -> None:
    wheel_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=wheel_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.72),
        origin=wheel_origin,
        material=metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.42, length=hub_length),
        origin=wheel_origin,
        material=dark_metal,
        name="hub",
    )
    side_disc_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=rim_radius * 0.92, length=0.004),
        origin=Origin(
            xyz=(0.0, tire_width * 0.18, 0.0),
            rpy=side_disc_origin.rpy,
        ),
        material=metal,
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.92, length=0.004),
        origin=Origin(
            xyz=(0.0, -tire_width * 0.18, 0.0),
            rpy=side_disc_origin.rpy,
        ),
        material=metal,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_scooter")

    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.57, 0.60, 0.64, 1.0))

    wheel_radius = 0.072
    wheel_width = 0.026
    deck_top_z = 0.098
    deck_center_z = 0.084
    steering_x = 0.300
    steering_z = 0.155
    stem_height = 0.720

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.58, 0.14, 0.18)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )
    deck_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.44, 0.11, 0.018, corner_segments=8),
            0.028,
            center=True,
        ),
        "deck_shell",
    )
    deck.visual(
        deck_shell,
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=aluminum,
        name="deck_shell",
    )
    deck.visual(
        Box((0.38, 0.086, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, deck_top_z)),
        material=grip_black,
        name="grip_pad",
    )
    deck.visual(
        Box((0.24, 0.060, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.073)),
        material=dark_aluminum,
    )
    front_brace = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.165, 0.0, 0.095),
                (0.214, 0.0, 0.108),
                (0.266, 0.0, 0.142),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "front_brace",
    )
    deck.visual(front_brace, material=dark_aluminum, name="front_brace")
    left_neck_rail = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.182, 0.030, 0.091),
                (0.228, 0.027, 0.112),
                (0.288, 0.020, 0.149),
            ],
            radius=0.0085,
            samples_per_segment=14,
            radial_segments=16,
        ),
        "left_neck_rail",
    )
    right_neck_rail = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.182, -0.030, 0.091),
                (0.228, -0.027, 0.112),
                (0.288, -0.020, 0.149),
            ],
            radius=0.0085,
            samples_per_segment=14,
            radial_segments=16,
        ),
        "right_neck_rail",
    )
    deck.visual(left_neck_rail, material=dark_aluminum, name="left_neck_rail")
    deck.visual(right_neck_rail, material=dark_aluminum, name="right_neck_rail")
    deck.visual(
        Box((0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.294, 0.0175, 0.151)),
        material=dark_aluminum,
        name="left_headset_pad",
    )
    deck.visual(
        Box((0.012, 0.008, 0.008)),
        origin=Origin(xyz=(0.294, -0.0175, 0.151)),
        material=dark_aluminum,
        name="right_headset_pad",
    )
    deck.visual(
        Box((0.100, 0.014, 0.104)),
        origin=Origin(xyz=(-0.260, 0.023, 0.123)),
        material=dark_aluminum,
    )
    deck.visual(
        Box((0.100, 0.014, 0.104)),
        origin=Origin(xyz=(-0.260, -0.023, 0.123)),
        material=dark_aluminum,
    )
    deck.visual(
        Box((0.140, 0.060, 0.018)),
        origin=Origin(xyz=(-0.305, 0.0, 0.158)),
        material=dark_aluminum,
        name="rear_fender",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_wheel,
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        hub_length=0.032,
        rim_radius=0.050,
        rubber=rubber,
        metal=wheel_metal,
        dark_metal=dark_aluminum,
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.82)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=aluminum,
        name="stem_tube",
    )
    front_fork.visual(
        Cylinder(radius=0.021, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_aluminum,
        name="lower_headset_collar",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=dark_aluminum,
    )
    front_fork.visual(
        Box((0.045, 0.058, 0.022)),
        origin=Origin(xyz=(0.016, 0.0, 0.045)),
        material=dark_aluminum,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.016, 0.010, 0.140)),
        origin=Origin(xyz=(0.018, 0.024, -0.020)),
        material=dark_aluminum,
        name="left_blade",
    )
    front_fork.visual(
        Box((0.016, 0.010, 0.140)),
        origin=Origin(xyz=(0.018, -0.024, -0.020)),
        material=dark_aluminum,
        name="right_blade",
    )
    front_fork.visual(
        Box((0.110, 0.050, 0.010)),
        origin=Origin(xyz=(0.048, 0.0, 0.040)),
        material=dark_aluminum,
        name="front_fender",
    )
    front_fork.visual(
        Box((0.034, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.696)),
        material=dark_aluminum,
        name="hinge_base",
    )
    front_fork.visual(
        Box((0.024, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.012, 0.720)),
        material=dark_aluminum,
        name="hinge_ear_left",
    )
    front_fork.visual(
        Box((0.024, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.012, 0.720)),
        material=dark_aluminum,
        name="hinge_ear_right",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=0.9,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        tire_radius=wheel_radius,
        tire_width=wheel_width,
        hub_length=0.038,
        rim_radius=0.050,
        rubber=rubber,
        metal=wheel_metal,
        dark_metal=dark_aluminum,
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.18)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )
    handlebar.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="hinge_knuckle",
    )
    handlebar.visual(
        Cylinder(radius=0.014, length=0.136),
        origin=Origin(xyz=(0.022, 0.0, 0.068)),
        material=aluminum,
        name="bar_riser",
    )
    handlebar.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.022, 0.0, 0.141), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="crossbar",
    )
    handlebar.visual(
        Cylinder(radius=0.015, length=0.095),
        origin=Origin(xyz=(0.022, 0.173, 0.141), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.015, length=0.095),
        origin=Origin(xyz=(0.022, -0.173, 0.141), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    handlebar.visual(
        Box((0.028, 0.018, 0.024)),
        origin=Origin(xyz=(0.012, 0.0, 0.020)),
        material=dark_aluminum,
        name="fold_latch",
    )

    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.305, 0.0, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(steering_x, 0.0, steering_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.018, 0.0, wheel_radius - steering_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.0, stem_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    rear_wheel = object_model.get_part("rear_wheel")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    handlebar = object_model.get_part("handlebar")

    steer = object_model.get_articulation("deck_to_front_fork")
    rear_spin = object_model.get_articulation("rear_wheel_spin")
    front_spin = object_model.get_articulation("front_wheel_spin")
    fold = object_model.get_articulation("stem_to_handlebar")

    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.56,
        max_gap=0.64,
        name="compact scooter wheelbase stays realistic",
    )
    ctx.expect_origin_gap(
        front_fork,
        deck,
        axis="x",
        min_gap=0.28,
        max_gap=0.32,
        name="steering column sits at the front of the slim deck",
    )
    ctx.expect_origin_gap(
        handlebar,
        deck,
        axis="z",
        min_gap=0.84,
        max_gap=0.90,
        name="folding handlebar hinge sits high above the deck",
    )
    ctx.expect_contact(
        deck,
        front_fork,
        elem_b="lower_headset_collar",
        contact_tol=0.0001,
        name="front steering collar is seated on the deck headset pads",
    )

    ctx.check(
        "front fork steers on a vertical axis",
        tuple(round(v, 3) for v in steer.axis) == (0.0, 0.0, 1.0),
        details=f"axis={steer.axis}",
    )
    ctx.check(
        "wheel axles spin laterally",
        tuple(round(v, 3) for v in front_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(round(v, 3) for v in rear_spin.axis) == (0.0, 1.0, 0.0),
        details=f"front_axis={front_spin.axis}, rear_axis={rear_spin.axis}",
    )
    ctx.check(
        "wheel joints are continuous spins",
        front_spin.motion_limits is not None
        and rear_spin.motion_limits is not None
        and front_spin.motion_limits.lower is None
        and front_spin.motion_limits.upper is None
        and rear_spin.motion_limits.lower is None
        and rear_spin.motion_limits.upper is None,
        details=(
            f"front_limits={front_spin.motion_limits}, "
            f"rear_limits={rear_spin.motion_limits}"
        ),
    )
    ctx.check(
        "handlebar folds on a lateral hinge",
        tuple(round(v, 3) for v in fold.axis) == (0.0, 1.0, 0.0)
        and fold.motion_limits is not None
        and fold.motion_limits.lower == 0.0
        and fold.motion_limits.upper is not None
        and fold.motion_limits.upper >= 1.30,
        details=f"axis={fold.axis}, limits={fold.motion_limits}",
    )

    front_wheel_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steer: 0.45}):
        ctx.expect_contact(
            deck,
            front_fork,
            elem_b="lower_headset_collar",
            contact_tol=0.0001,
            name="steering collar stays supported while the fork yaws",
        )
        front_wheel_turned = ctx.part_world_position(front_wheel)
    ctx.check(
        "positive steering yaws the front fork to one side",
        front_wheel_rest is not None
        and front_wheel_turned is not None
        and front_wheel_turned[1] > front_wheel_rest[1] + 0.004
        and abs(front_wheel_turned[2] - front_wheel_rest[2]) < 0.001,
        details=f"rest={front_wheel_rest}, turned={front_wheel_turned}",
    )

    crossbar_rest = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="crossbar"))
    with ctx.pose({fold: 1.20}):
        crossbar_folded = _aabb_center(ctx.part_element_world_aabb(handlebar, elem="crossbar"))
    ctx.check(
        "crossbar folds forward and down from the stem top",
        crossbar_rest is not None
        and crossbar_folded is not None
        and crossbar_folded[0] > crossbar_rest[0] + 0.10
        and crossbar_folded[2] < crossbar_rest[2] - 0.06,
        details=f"rest={crossbar_rest}, folded={crossbar_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
