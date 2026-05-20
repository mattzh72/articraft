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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brutalist_clock_tower")

    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    weathered_concrete = model.material("weathered_concrete", rgba=(0.71, 0.71, 0.68, 1.0))
    dial_stone = model.material("dial_stone", rgba=(0.87, 0.87, 0.84, 1.0))
    oxidized_metal = model.material("oxidized_metal", rgba=(0.20, 0.21, 0.22, 1.0))
    hand_metal = model.material("hand_metal", rgba=(0.10, 0.10, 0.11, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((5.4, 3.2, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        Box((2.8, 1.9, 13.2)),
        origin=Origin(xyz=(0.0, 0.0, 7.5)),
        material=concrete,
        name="main_shaft",
    )
    tower.visual(
        Box((3.4, 2.4, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 14.375)),
        material=weathered_concrete,
        name="roof_cap",
    )
    tower.visual(
        Box((3.7, 0.5, 8.8)),
        origin=Origin(xyz=(0.0, 1.10, 10.25)),
        material=weathered_concrete,
        name="front_slab",
    )
    tower.visual(
        Box((0.42, 0.82, 7.2)),
        origin=Origin(xyz=(-1.35, 0.90, 9.45)),
        material=concrete,
        name="left_fin",
    )
    tower.visual(
        Box((0.42, 0.82, 7.2)),
        origin=Origin(xyz=(1.35, 0.90, 9.45)),
        material=concrete,
        name="right_fin",
    )
    tower.visual(
        Box((2.7, 0.65, 0.70)),
        origin=Origin(xyz=(0.0, 0.95, 6.20)),
        material=concrete,
        name="cross_beam",
    )
    tower.visual(
        Cylinder(radius=1.35, length=0.12),
        origin=Origin(xyz=(0.0, 1.29, 10.95), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="clock_bezel",
    )
    tower.visual(
        Cylinder(radius=1.25, length=0.08),
        origin=Origin(xyz=(0.0, 1.31, 10.95), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_stone,
        name="clock_face",
    )
    tower.inertial = Inertial.from_geometry(
        Box((5.4, 3.2, 14.65)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.325)),
    )

    clock_hub = model.part("clock_hub")
    clock_hub.visual(
        Cylinder(radius=0.11, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="hub_flange",
    )
    clock_hub.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="hour_arbor",
    )
    clock_hub.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxidized_metal,
        name="minute_arbor",
    )
    clock_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.038),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.09, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_metal,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.12, 0.004, 0.06)),
        origin=Origin(xyz=(0.0, 0.002, 0.11)),
        material=hand_metal,
        name="hour_neck",
    )
    hour_hand.visual(
        Box((0.14, 0.004, 0.62)),
        origin=Origin(xyz=(0.0, 0.004, 0.41)),
        material=hand_metal,
        name="hour_blade",
    )
    hour_hand.visual(
        Box((0.10, 0.004, 0.10)),
        origin=Origin(xyz=(0.0, 0.004, 0.75)),
        material=hand_metal,
        name="hour_tip",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.14, 0.014, 0.85)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.001, 0.37)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hand_metal,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.10, 0.006, 1.00)),
        origin=Origin(xyz=(0.0, 0.003, 0.50)),
        material=hand_metal,
        name="minute_blade",
    )
    minute_hand.visual(
        Box((0.06, 0.006, 0.22)),
        origin=Origin(xyz=(0.0, 0.003, -0.11)),
        material=hand_metal,
        name="minute_tail",
    )
    minute_hand.visual(
        Box((0.07, 0.006, 0.12)),
        origin=Origin(xyz=(0.0, 0.003, 1.03)),
        material=hand_metal,
        name="minute_tip",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.10, 0.016, 1.15)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0005, 0.435)),
    )

    model.articulation(
        "tower_to_hub",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_hub,
        origin=Origin(xyz=(0.0, 1.35, 10.95)),
    )
    model.articulation(
        "hub_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_hub,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0),
    )
    model.articulation(
        "hub_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_hub,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    tower = object_model.get_part("tower")
    clock_hub = object_model.get_part("clock_hub")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")

    tower_to_hub = object_model.get_articulation("tower_to_hub")
    hour_spin = object_model.get_articulation("hub_to_hour_hand")
    minute_spin = object_model.get_articulation("hub_to_minute_hand")

    ctx.allow_overlap(
        clock_hub,
        hour_hand,
        elem_a="hour_arbor",
        elem_b="hour_hub",
        reason="The hour hand is represented as a solid sleeve nested over the arbor instead of a hollow bored hub.",
    )
    ctx.allow_overlap(
        clock_hub,
        minute_hand,
        elem_a="minute_arbor",
        elem_b="minute_hub",
        reason="The minute hand is represented as a solid sleeve nested over the arbor instead of a hollow bored hub.",
    )

    ctx.expect_contact(
        clock_hub,
        tower,
        elem_a="hub_flange",
        elem_b="clock_face",
        name="hub flange seats against the clock face",
    )
    ctx.expect_gap(
        hour_hand,
        tower,
        axis="y",
        min_gap=0.024,
        max_gap=0.035,
        positive_elem="hour_blade",
        negative_elem="clock_face",
        name="hour blade stands off in front of the dial",
    )
    ctx.expect_gap(
        minute_hand,
        tower,
        axis="y",
        min_gap=0.038,
        max_gap=0.050,
        positive_elem="minute_blade",
        negative_elem="clock_face",
        name="minute blade stands off in front of the dial",
    )
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="y",
        min_gap=0.008,
        max_gap=0.020,
        positive_elem="minute_blade",
        negative_elem="hour_blade",
        name="minute blade clears the hour blade in depth",
    )
    ctx.expect_overlap(
        minute_hand,
        tower,
        axes="xz",
        elem_a="minute_blade",
        elem_b="clock_face",
        min_overlap=0.08,
        name="minute hand remains within the clock face footprint",
    )
    ctx.expect_overlap(
        hour_hand,
        tower,
        axes="xz",
        elem_a="hour_blade",
        elem_b="clock_face",
        min_overlap=0.08,
        name="hour hand remains within the clock face footprint",
    )

    ctx.check(
        "clock hands use continuous coaxial joints",
        tower_to_hub.articulation_type == ArticulationType.FIXED
        and hour_spin.articulation_type == ArticulationType.CONTINUOUS
        and minute_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(hour_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(minute_spin.axis) == (0.0, 1.0, 0.0)
        and abs(hour_spin.origin.xyz[0] - minute_spin.origin.xyz[0]) < 1e-9
        and abs(hour_spin.origin.xyz[2] - minute_spin.origin.xyz[2]) < 1e-9,
        details=(
            f"tower_to_hub_type={tower_to_hub.articulation_type}, "
            f"hour_type={hour_spin.articulation_type}, minute_type={minute_spin.articulation_type}, "
            f"hour_axis={hour_spin.axis}, minute_axis={minute_spin.axis}, "
            f"hour_origin={hour_spin.origin.xyz}, minute_origin={minute_spin.origin.xyz}"
        ),
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    minute_tip_rest = _aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
    hour_tip_rest = _aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))

    with ctx.pose({minute_spin: math.pi / 2.0, hour_spin: math.pi / 2.0}):
        minute_tip_turned = _aabb_center(ctx.part_element_world_aabb(minute_hand, elem="minute_tip"))
        hour_tip_turned = _aabb_center(ctx.part_element_world_aabb(hour_hand, elem="hour_tip"))

    ctx.check(
        "minute hand rotates around the dial center",
        minute_tip_rest is not None
        and minute_tip_turned is not None
        and minute_tip_turned[0] > minute_tip_rest[0] + 0.75
        and minute_tip_turned[2] < minute_tip_rest[2] - 0.75,
        details=f"rest={minute_tip_rest}, turned={minute_tip_turned}",
    )
    ctx.check(
        "hour hand rotates around the same axis",
        hour_tip_rest is not None
        and hour_tip_turned is not None
        and hour_tip_turned[0] > hour_tip_rest[0] + 0.45
        and hour_tip_turned[2] < hour_tip_rest[2] - 0.45,
        details=f"rest={hour_tip_rest}, turned={hour_tip_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
