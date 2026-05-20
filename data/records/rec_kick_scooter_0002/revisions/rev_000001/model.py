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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_kick_scooter", assets=ASSETS)

    deck_metal = model.material("deck_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    stem_metal = model.material("stem_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.46, 0.49, 0.53, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))

    wheel_radius = 0.05
    tire_width = 0.024
    hub_width = 0.024
    deck_thickness = 0.018
    deck_bottom_z = 0.056
    deck_top_z = deck_bottom_z + deck_thickness
    hinge_xyz = (0.205, 0.0, 0.104)
    rear_axle_xyz = (-0.300, 0.0, wheel_radius)

    deck_profile = [
        (-0.245, -0.055),
        (0.145, -0.055),
        (0.190, -0.043),
        (0.225, -0.030),
        (0.240, 0.000),
        (0.225, 0.030),
        (0.190, 0.043),
        (0.145, 0.055),
        (-0.245, 0.055),
    ]
    deck_geom = ExtrudeGeometry.from_z0(deck_profile, deck_thickness)
    deck_mesh = mesh_from_geometry(deck_geom, ASSETS.mesh_path("scooter_deck.obj"))
    tire_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.039, tube=0.011, radial_segments=16, tubular_segments=36),
        ASSETS.mesh_path("scooter_tire.obj"),
    )

    deck = model.part("deck")
    deck.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_bottom_z)),
        material=deck_metal,
        name="deck_shell",
    )
    deck.visual(
        Box((0.300, 0.082, 0.003)),
        origin=Origin(xyz=(-0.035, 0.0, deck_top_z + 0.0015)),
        material=grip_black,
        name="grip_pad",
    )
    deck.visual(
        Box((0.068, 0.046, 0.026)),
        origin=Origin(xyz=(0.188, 0.0, deck_top_z + 0.013)),
        material=deck_metal,
        name="hinge_pedestal",
    )
    deck.visual(
        Box((0.038, 0.012, 0.040)),
        origin=Origin(xyz=(0.212, 0.026, 0.102)),
        material=deck_metal,
        name="hinge_left_cheek",
    )
    deck.visual(
        Box((0.038, 0.012, 0.040)),
        origin=Origin(xyz=(0.212, -0.026, 0.102)),
        material=deck_metal,
        name="hinge_right_cheek",
    )
    deck.visual(
        Box((0.060, 0.017, 0.058)),
        origin=Origin(xyz=(-0.275, 0.022, 0.027)),
        material=deck_metal,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.060, 0.017, 0.058)),
        origin=Origin(xyz=(-0.275, -0.022, 0.027)),
        material=deck_metal,
        name="rear_right_dropout",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=rear_axle_xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rear_axle",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.49, 0.11, 0.08)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    stem = model.part("stem")
    stem.visual(
        Box((0.032, 0.022, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, 0.008)),
        material=stem_metal,
        name="hinge_tongue",
    )
    stem.visual(
        Box((0.050, 0.044, 0.020)),
        origin=Origin(xyz=(0.035, 0.0, 0.032)),
        material=stem_metal,
        name="fork_crown",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=stem_metal,
        name="stem_tube",
    )
    stem.visual(
        Box((0.044, 0.017, 0.108)),
        origin=Origin(xyz=(0.082, 0.022, -0.011)),
        material=stem_metal,
        name="fork_left_leg",
    )
    stem.visual(
        Box((0.044, 0.017, 0.108)),
        origin=Origin(xyz=(0.082, -0.022, -0.011)),
        material=stem_metal,
        name="fork_right_leg",
    )
    stem.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.090, 0.0, -0.054), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="fork_axle",
    )
    stem.visual(
        Cylinder(radius=0.012, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stem_metal,
        name="handlebar",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, 0.125, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, -0.125, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.36, 0.34, 0.52)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_rubber,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.028, length=hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="front_hub",
    )
    front_wheel.visual(
        Box((0.012, hub_width, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=hub_gray,
        name="front_top_spoke",
    )
    front_wheel.visual(
        Box((0.012, hub_width, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=hub_gray,
        name="front_bottom_spoke",
    )
    front_wheel.visual(
        Box((0.018, hub_width, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=hub_gray,
        name="front_front_spoke",
    )
    front_wheel.visual(
        Box((0.018, hub_width, 0.012)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=hub_gray,
        name="front_rear_spoke",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_rubber,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.028, length=hub_width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rear_hub",
    )
    rear_wheel.visual(
        Box((0.012, hub_width, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=hub_gray,
        name="rear_top_spoke",
    )
    rear_wheel.visual(
        Box((0.012, hub_width, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=hub_gray,
        name="rear_bottom_spoke",
    )
    rear_wheel.visual(
        Box((0.018, hub_width, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=hub_gray,
        name="rear_front_spoke",
    )
    rear_wheel.visual(
        Box((0.018, hub_width, 0.012)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=hub_gray,
        name="rear_rear_spoke",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "deck_to_stem",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=hinge_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=3.0,
            lower=-1.48,
            upper=0.0,
        ),
    )
    model.articulation(
        "stem_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=front_wheel,
        origin=Origin(xyz=(0.090, 0.0, -0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_fold = object_model.get_articulation("deck_to_stem")
    front_spin = object_model.get_articulation("stem_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")

    deck_shell = deck.get_visual("deck_shell")
    grip_pad = deck.get_visual("grip_pad")
    hinge_pedestal = deck.get_visual("hinge_pedestal")
    rear_left_dropout = deck.get_visual("rear_left_dropout")
    rear_right_dropout = deck.get_visual("rear_right_dropout")
    rear_axle = deck.get_visual("rear_axle")

    hinge_tongue = stem.get_visual("hinge_tongue")
    stem_tube = stem.get_visual("stem_tube")
    fork_left_leg = stem.get_visual("fork_left_leg")
    fork_right_leg = stem.get_visual("fork_right_leg")
    fork_axle = stem.get_visual("fork_axle")
    handlebar = stem.get_visual("handlebar")

    front_tire = front_wheel.get_visual("front_tire")
    front_hub = front_wheel.get_visual("front_hub")
    rear_tire = rear_wheel.get_visual("rear_tire")
    rear_hub = rear_wheel.get_visual("rear_hub")

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
    ctx.allow_overlap(
        stem,
        front_wheel,
        elem_a=fork_axle,
        elem_b=front_hub,
        reason="Front wheel hub rotates around the fork axle; the solid hub visual intentionally encloses the axle.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a=rear_axle,
        elem_b=rear_hub,
        reason="Rear wheel hub rotates around the rear axle; the solid hub visual intentionally encloses the axle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "required scooter parts exist",
        all(part is not None for part in (deck, stem, front_wheel, rear_wheel)),
        "Deck, stem, and both wheel parts must all exist.",
    )
    ctx.check(
        "required scooter joints exist",
        all(joint is not None for joint in (stem_fold, front_spin, rear_spin)),
        "Stem fold hinge and both wheel spin joints must all exist.",
    )
    ctx.check(
        "wheel joints are continuous",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(front_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(rear_spin.axis) == (0.0, 1.0, 0.0),
        "Both wheels should spin continuously about lateral y-axis axles.",
    )
    ctx.check(
        "stem hinge folds about lateral axis",
        stem_fold.articulation_type == ArticulationType.REVOLUTE
        and tuple(stem_fold.axis) == (0.0, 1.0, 0.0)
        and stem_fold.motion_limits.lower <= -1.40
        and stem_fold.motion_limits.upper == 0.0,
        "Stem should use a y-axis revolute hinge with a broad folding range.",
    )

    ctx.expect_gap(
        stem,
        deck,
        axis="z",
        positive_elem=hinge_tongue,
        negative_elem=hinge_pedestal,
        min_gap=0.0,
        max_gap=0.003,
        name="stem tongue sits on hinge pedestal",
    )
    ctx.expect_overlap(
        stem,
        deck,
        axes="x",
        elem_a=hinge_tongue,
        elem_b=hinge_pedestal,
        min_overlap=0.020,
        name="hinge tongue overlaps pedestal footprint",
    )

    ctx.expect_gap(
        stem,
        front_wheel,
        axis="y",
        positive_elem=fork_left_leg,
        negative_elem=front_hub,
        min_gap=0.0,
        max_gap=0.002,
        name="front hub clears left fork leg with axle-like gap",
    )
    ctx.expect_gap(
        front_wheel,
        stem,
        axis="y",
        positive_elem=front_hub,
        negative_elem=fork_right_leg,
        min_gap=0.0,
        max_gap=0.002,
        name="front hub clears right fork leg with axle-like gap",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="y",
        positive_elem=rear_left_dropout,
        negative_elem=rear_hub,
        min_gap=0.0,
        max_gap=0.002,
        name="rear hub clears left dropout with axle-like gap",
    )
    ctx.expect_gap(
        rear_wheel,
        deck,
        axis="y",
        positive_elem=rear_hub,
        negative_elem=rear_right_dropout,
        min_gap=0.0,
        max_gap=0.002,
        name="rear hub clears right dropout with axle-like gap",
    )
    ctx.expect_overlap(
        stem,
        front_wheel,
        axes="xz",
        elem_a=fork_axle,
        elem_b=front_hub,
        min_overlap=0.010,
        name="front axle passes through the front hub core",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="xz",
        elem_a=rear_axle,
        elem_b=rear_hub,
        min_overlap=0.010,
        name="rear axle passes through the rear hub core",
    )

    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        positive_elem=deck_shell,
        negative_elem=rear_tire,
        min_gap=0.003,
        max_gap=0.015,
        name="rear tire sits just aft of the deck tail",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        positive_elem=front_tire,
        negative_elem=hinge_pedestal,
        min_gap=0.018,
        max_gap=0.080,
        name="front wheel sits ahead of the hinge block and deck nose",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.56,
        max_gap=0.63,
        name="small wheels are spaced by a scooter-like wheelbase",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a=grip_pad,
        elem_b=rear_tire,
        min_overlap=0.020,
        name="rear wheel stays centered under the deck",
    )
    ctx.expect_overlap(
        stem,
        front_wheel,
        axes="xz",
        elem_a=fork_left_leg,
        elem_b=front_tire,
        min_overlap=0.010,
        name="front wheel sits within fork blade projection",
    )

    with ctx.pose({stem_fold: -1.35}):
        ctx.expect_overlap(
            stem,
            deck,
            axes="x",
            elem_a=stem_tube,
            elem_b=deck_shell,
            min_overlap=0.18,
            name="folded stem lies back over the deck length",
        )
        ctx.expect_gap(
            stem,
            deck,
            axis="z",
            positive_elem=handlebar,
            negative_elem=grip_pad,
            min_gap=0.10,
            max_gap=0.18,
            name="folded handlebar stays above the deck surface",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
