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
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_cylinder_member(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_rpy_for_cylinder(start, end)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    hub_width: float,
    rim_radius: float,
    tire_material,
    hub_material,
    cap_material,
) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=tire_width * 0.92),
        origin=spin_origin,
        material=cap_material,
        name="rim_band",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=hub_material,
        name="hub_core",
    )
    cap_length = hub_width * 0.12
    cap_offset = hub_width * 0.5 - cap_length * 0.5
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=cap_length),
        origin=Origin(xyz=(0.0, cap_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cap_material,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.55, length=cap_length),
        origin=Origin(xyz=(0.0, -cap_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cap_material,
        name="inner_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_kick_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.18, 0.50, 0.84, 1.0))
    stem_blue = model.material("stem_blue", rgba=(0.13, 0.57, 0.89, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.70, 0.73, 0.76, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.56, 0.60, 0.64, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.58, 0.18, 0.10)),
        mass=3.2,
        origin=Origin(xyz=(0.01, 0.0, 0.07)),
    )

    deck_outline = sample_catmull_rom_spline_2d(
        [
            (-0.26, -0.070),
            (-0.24, -0.080),
            (0.00, -0.082),
            (0.18, -0.080),
            (0.25, -0.062),
            (0.30, -0.026),
            (0.305, 0.0),
            (0.30, 0.026),
            (0.25, 0.062),
            (0.18, 0.080),
            (0.00, 0.082),
            (-0.24, 0.080),
        ],
        samples_per_segment=8,
        closed=True,
    )
    deck_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(deck_outline, 0.022),
        "scooter_deck_shell",
    )
    deck.visual(
        deck_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.38, 0.112, 0.004)),
        origin=Origin(xyz=(-0.015, 0.0, 0.066)),
        material=dark_trim,
        name="grip_pad",
    )
    deck.visual(
        Box((0.040, 0.018, 0.038)),
        origin=Origin(xyz=(0.240, -0.030, 0.065)),
        material=dark_trim,
        name="left_hinge_support",
    )
    deck.visual(
        Box((0.040, 0.018, 0.038)),
        origin=Origin(xyz=(0.240, 0.030, 0.065)),
        material=dark_trim,
        name="right_hinge_support",
    )
    deck.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.255, -0.030, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="left_hinge_lug",
    )
    deck.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.255, 0.030, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="right_hinge_lug",
    )
    deck.visual(
        Box((0.034, 0.008, 0.064)),
        origin=Origin(xyz=(-0.323, -0.025, 0.074)),
        material=dark_trim,
        name="left_dropout",
    )
    deck.visual(
        Box((0.034, 0.008, 0.064)),
        origin=Origin(xyz=(-0.323, 0.025, 0.074)),
        material=dark_trim,
        name="right_dropout",
    )
    deck.visual(
        Box((0.124, 0.064, 0.018)),
        origin=Origin(xyz=(-0.318, 0.0, 0.115)),
        material=dark_trim,
        name="rear_fender",
    )
    deck.visual(
        Box((0.090, 0.016, 0.070)),
        origin=Origin(xyz=(-0.286, -0.028, 0.085)),
        material=dark_trim,
        name="left_rear_stay",
    )
    deck.visual(
        Box((0.090, 0.016, 0.070)),
        origin=Origin(xyz=(-0.286, 0.028, 0.085)),
        material=dark_trim,
        name="right_rear_stay",
    )

    stem = model.part("stem_assembly")
    stem.inertial = Inertial.from_geometry(
        Box((0.42, 0.36, 0.84)),
        mass=2.1,
        origin=Origin(xyz=(-0.03, 0.0, 0.36)),
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="hinge_barrel",
    )
    stem.visual(
        Box((0.040, 0.042, 0.036)),
        origin=Origin(xyz=(0.006, 0.0, 0.018)),
        material=dark_trim,
        name="base_block",
    )
    stem.visual(
        Cylinder(radius=0.023, length=0.058),
        origin=Origin(xyz=(0.012, 0.0, 0.032)),
        material=dark_trim,
        name="lower_collar",
    )
    _add_cylinder_member(
        stem,
        (0.008, 0.0, 0.030),
        (-0.118, 0.0, 0.695),
        radius=0.018,
        material=stem_blue,
        name="stem_tube",
    )
    stem.visual(
        Box((0.052, 0.042, 0.036)),
        origin=Origin(xyz=(-0.122, 0.0, 0.712)),
        material=dark_trim,
        name="bar_clamp",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.340),
        origin=Origin(xyz=(-0.125, 0.0, 0.725), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stem_blue,
        name="handlebar_bar",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.090),
        origin=Origin(xyz=(-0.125, -0.125, 0.725), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.090),
        origin=Origin(xyz=(-0.125, 0.125, 0.725), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="right_grip",
    )
    left_fork_arm = tube_from_spline_points(
        [
            (0.012, 0.0, 0.022),
            (0.034, -0.024, 0.018),
            (0.064, -0.050, 0.004),
            (0.094, -0.062, -0.018),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_fork_arm = tube_from_spline_points(
        [
            (0.012, 0.0, 0.022),
            (0.034, 0.024, 0.018),
            (0.064, 0.050, 0.004),
            (0.094, 0.062, -0.018),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
    )
    stem.visual(
        mesh_from_geometry(left_fork_arm, "scooter_left_fork_arm"),
        material=stem_blue,
        name="left_fork_arm",
    )
    stem.visual(
        mesh_from_geometry(right_fork_arm, "scooter_right_fork_arm"),
        material=stem_blue,
        name="right_fork_arm",
    )
    stem.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(xyz=(0.048, 0.0, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="fork_crossbrace",
    )
    stem.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.095, -0.072, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="left_axle_boss",
    )
    stem.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.095, 0.072, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="right_axle_boss",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.030),
        mass=0.50,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        tire_radius=0.060,
        tire_width=0.022,
        hub_radius=0.037,
        hub_width=0.030,
        rim_radius=0.049,
        tire_material=rubber_black,
        hub_material=wheel_gray,
        cap_material=axle_gray,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.030),
        mass=0.50,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        tire_radius=0.060,
        tire_width=0.022,
        hub_radius=0.037,
        hub_width=0.030,
        rim_radius=0.049,
        tire_material=rubber_black,
        hub_material=wheel_gray,
        cap_material=axle_gray,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.042),
        mass=0.42,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_wheel,
        tire_radius=0.052,
        tire_width=0.024,
        hub_radius=0.030,
        hub_width=0.042,
        rim_radius=0.042,
        tire_material=rubber_black,
        hub_material=wheel_gray,
        cap_material=axle_gray,
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(0.255, 0.0, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.25,
            upper=0.18,
        ),
    )
    model.articulation(
        "front_left_axle",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=front_left_wheel,
        origin=Origin(xyz=(0.095, -0.092, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "front_right_axle",
        ArticulationType.CONTINUOUS,
        parent=stem,
        child=front_right_wheel,
        origin=Origin(xyz=(0.095, 0.092, -0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.325, 0.0, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem_assembly")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    stem_fold = object_model.get_articulation("stem_fold")
    front_left_axle = object_model.get_articulation("front_left_axle")
    front_right_axle = object_model.get_articulation("front_right_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(stem, deck, elem_a="hinge_barrel", elem_b="left_hinge_lug")
    ctx.expect_contact(stem, deck, elem_a="hinge_barrel", elem_b="right_hinge_lug")
    ctx.expect_contact(front_left_wheel, stem, elem_b="left_axle_boss")
    ctx.expect_contact(front_right_wheel, stem, elem_b="right_axle_boss")
    ctx.expect_contact(rear_wheel, deck)

    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="y",
        min_dist=0.17,
        max_dist=0.20,
        name="front wheels are side by side",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.45,
        name="front axle line sits ahead of rear wheel",
    )

    ctx.check(
        "stem hinge axis and limits",
        stem_fold.axis == (0.0, 1.0, 0.0)
        and stem_fold.motion_limits is not None
        and stem_fold.motion_limits.lower is not None
        and stem_fold.motion_limits.upper is not None
        and stem_fold.motion_limits.lower <= -1.20
        and stem_fold.motion_limits.upper >= 0.0,
        details=f"unexpected stem hinge configuration: axis={stem_fold.axis}, limits={stem_fold.motion_limits}",
    )
    ctx.check(
        "wheel axles spin about scooter width axis",
        front_left_axle.axis == (0.0, 1.0, 0.0)
        and front_right_axle.axis == (0.0, 1.0, 0.0)
        and rear_axle.axis == (0.0, 1.0, 0.0),
        details=(
            f"front_left={front_left_axle.axis}, "
            f"front_right={front_right_axle.axis}, rear={rear_axle.axis}"
        ),
    )

    with ctx.pose({stem_fold: -1.25}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in folded pose")
        ctx.expect_overlap(
            stem,
            deck,
            axes="x",
            elem_a="stem_tube",
            min_overlap=0.10,
            name="stem tube folds back over deck length",
        )
        ctx.expect_gap(
            stem,
            deck,
            axis="z",
            positive_elem="stem_tube",
            negative_elem="deck_shell",
            max_gap=0.03,
            max_penetration=0.0,
            name="folded stem tube stays just above deck shell",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
