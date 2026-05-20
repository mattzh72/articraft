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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _wheel_part(
    model: ArticulatedObject,
    name: str,
    *,
    tire_radius: float,
    tire_width: float,
    tire_material,
    hub_material,
) :
    wheel = model.part(name)
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    wheel.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=tire_radius * 0.68, length=tire_width * 0.72),
        origin=spin_origin,
        material=hub_material,
        name="hub_core",
    )
    wheel.visual(
        Cylinder(radius=tire_radius * 0.24, length=tire_width * 0.30),
        origin=spin_origin,
        material=hub_material,
        name="bearing_hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=0.42 if tire_radius > 0.055 else 0.30,
        origin=spin_origin,
    )
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_kick_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.21, 0.63, 0.86, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stem_silver = model.material("stem_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.24, 0.27, 1.0))
    axle_metal = model.material("axle_metal", rgba=(0.60, 0.63, 0.67, 1.0))
    wheel_pu = model.material("wheel_pu", rgba=(0.74, 0.82, 0.26, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.84, 0.86, 0.88, 1.0))

    deck = model.part("deck")
    deck_shell = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.45, 0.13, radius=0.030, corner_segments=10),
            0.018,
            cap=True,
            closed=True,
        ),
        "scooter_deck_shell",
    )
    deck.visual(
        deck_shell,
        origin=Origin(xyz=(-0.015, 0.0, 0.051)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.28, 0.092, 0.003)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0615)),
        material=grip_black,
        name="deck_grip",
    )
    deck.visual(
        Box((0.090, 0.090, 0.022)),
        origin=Origin(xyz=(0.235, 0.0, 0.053)),
        material=deck_blue,
        name="nose_block",
    )
    deck.visual(
        Cylinder(radius=0.024, length=0.072),
        origin=Origin(xyz=(0.244, 0.0, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_cover",
    )
    deck.visual(
        Box((0.120, 0.012, 0.040)),
        origin=Origin(xyz=(-0.270, 0.024, 0.061)),
        material=dark_metal,
        name="rear_left_fork",
    )
    deck.visual(
        Box((0.120, 0.012, 0.040)),
        origin=Origin(xyz=(-0.270, -0.024, 0.061)),
        material=dark_metal,
        name="rear_right_fork",
    )
    deck.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(-0.300, 0.018, 0.052)),
        material=axle_metal,
        name="rear_left_axle_stub",
    )
    deck.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(-0.300, -0.018, 0.052)),
        material=axle_metal,
        name="rear_right_axle_stub",
    )
    rear_fender_shell = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.062, 0.052, radius=0.013, corner_segments=8),
            0.010,
            cap=True,
            closed=True,
        ),
        "rear_fender_shell",
    )
    deck.visual(
        rear_fender_shell,
        origin=Origin(xyz=(-0.345, 0.0, 0.114)),
        material=deck_blue,
        name="rear_fender",
    )
    deck.visual(
        Box((0.024, 0.012, 0.028)),
        origin=Origin(xyz=(-0.326, 0.022, 0.095)),
        material=dark_metal,
        name="rear_left_fender_stay",
    )
    deck.visual(
        Box((0.024, 0.012, 0.028)),
        origin=Origin(xyz=(-0.326, -0.022, 0.095)),
        material=dark_metal,
        name="rear_right_fender_stay",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.54, 0.14, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(-0.010, 0.0, 0.065)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Box((0.040, 0.055, 0.044)),
        origin=Origin(xyz=(0.020, 0.0, -0.022)),
        material=dark_metal,
        name="pivot_mount",
    )
    front_assembly.visual(
        Box((0.125, 0.218, 0.022)),
        origin=Origin(xyz=(0.100, 0.0, 0.004)),
        material=dark_metal,
        name="bogie_beam",
    )
    front_assembly.visual(
        Box((0.050, 0.016, 0.060)),
        origin=Origin(xyz=(0.085, 0.090, -0.004)),
        material=dark_metal,
        name="left_wheel_support",
    )
    front_assembly.visual(
        Box((0.050, 0.016, 0.060)),
        origin=Origin(xyz=(0.085, -0.090, -0.004)),
        material=dark_metal,
        name="right_wheel_support",
    )
    front_assembly.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(0.093, 0.101, 0.018)),
        material=axle_metal,
        name="left_axle_stub",
    )
    front_assembly.visual(
        Box((0.010, 0.006, 0.010)),
        origin=Origin(xyz=(0.093, -0.101, 0.018)),
        material=axle_metal,
        name="right_axle_stub",
    )
    front_assembly.visual(
        Box((0.040, 0.056, 0.042)),
        origin=Origin(xyz=(0.062, 0.0, 0.016)),
        material=dark_metal,
        name="stem_collar",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.57),
        origin=Origin(xyz=(0.002, 0.0, 0.302), rpy=(0.0, -0.22, 0.0)),
        material=stem_silver,
        name="steering_stem",
    )
    front_assembly.visual(
        Box((0.075, 0.046, 0.036)),
        origin=Origin(xyz=(-0.090, 0.0, 0.584)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    front_assembly.visual(
        Cylinder(radius=0.010, length=0.330),
        origin=Origin(xyz=(-0.100, 0.0, 0.584), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stem_silver,
        name="handlebar_bar",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(-0.100, 0.120, 0.584), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(-0.100, -0.120, 0.584), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.28, 0.34, 0.74)),
        mass=1.6,
        origin=Origin(xyz=(0.010, 0.0, 0.280)),
    )

    front_left_wheel = _wheel_part(
        model,
        "front_left_wheel",
        tire_radius=0.060,
        tire_width=0.022,
        tire_material=wheel_pu,
        hub_material=wheel_core,
    )
    front_right_wheel = _wheel_part(
        model,
        "front_right_wheel",
        tire_radius=0.060,
        tire_width=0.022,
        tire_material=wheel_pu,
        hub_material=wheel_core,
    )
    rear_wheel = _wheel_part(
        model,
        "rear_wheel",
        tire_radius=0.052,
        tire_width=0.026,
        tire_material=wheel_pu,
        hub_material=wheel_core,
    )

    model.articulation(
        "deck_to_front_assembly",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.245, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.34,
            upper=0.34,
        ),
    )
    model.articulation(
        "front_assembly_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_left_wheel,
        origin=Origin(xyz=(0.093, 0.115, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "front_assembly_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_right_wheel,
        origin=Origin(xyz=(0.093, -0.115, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.300, 0.0, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    lean_joint = object_model.get_articulation("deck_to_front_assembly")
    left_spin = object_model.get_articulation("front_assembly_to_left_wheel")
    right_spin = object_model.get_articulation("front_assembly_to_right_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    ctx.check(
        "lean joint uses longitudinal roll axis",
        lean_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={lean_joint.axis}",
    )
    ctx.check(
        "wheel spin axes align with wheel axles",
        left_spin.axis == (0.0, 1.0, 0.0)
        and right_spin.axis == (0.0, 1.0, 0.0)
        and rear_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"left={left_spin.axis}, right={right_spin.axis}, rear={rear_spin.axis}"
        ),
    )

    with ctx.pose({lean_joint: 0.0}):
        ctx.expect_contact(
            front_assembly,
            deck,
            elem_a="pivot_mount",
            elem_b="nose_block",
            name="front bogie mount seats against deck nose",
        )
        ctx.expect_origin_gap(
            front_left_wheel,
            front_right_wheel,
            axis="y",
            min_gap=0.20,
            name="front wheels sit on a wide twin-wheel track",
        )
        ctx.expect_origin_gap(
            deck,
            rear_wheel,
            axis="x",
            min_gap=0.20,
            name="rear wheel sits aft of the deck center",
        )

    with ctx.pose({lean_joint: 0.26}):
        ctx.expect_origin_gap(
            front_left_wheel,
            front_right_wheel,
            axis="z",
            min_gap=0.04,
            name="positive lean lifts the left front wheel relative to the right",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
