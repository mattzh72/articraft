from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _square_profile(size: float, z: float) -> list[tuple[float, float, float]]:
    half = size / 2.0
    return [
        (-half, -half, z),
        (half, -half, z),
        (half, half, z),
        (-half, half, z),
    ]


def _add_clock_ticks(tower, material: str, *, radius: float, y: float, z: float) -> None:
    for i in range(12):
        angle = i * math.tau / 12.0
        major = i % 3 == 0
        tick_len = 0.135 if major else 0.095
        tick_w = 0.040 if major else 0.025
        tower.visual(
            Box((tick_w, 0.018, tick_len)),
            origin=Origin(
                xyz=(radius * math.sin(angle), y, z + radius * math.cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=material,
            name=f"hour_mark_{i}",
        )


def _add_front_brickwork(tower, mortar: str, *, width: float, front_y: float) -> None:
    course_z = 0.38
    z = 0.58
    course = 0
    while z < 4.15:
        tower.visual(
            Box((width + 0.018, 0.018, 0.020)),
            origin=Origin(xyz=(0.0, front_y, z)),
            material=mortar,
            name=f"front_mortar_course_{course}",
        )
        offset = -0.60 if course % 2 == 0 else -0.38
        joint = 0
        x = offset
        while x <= 0.62:
            tower.visual(
                Box((0.018, 0.019, 0.24)),
                origin=Origin(xyz=(x, front_y - 0.001, z + 0.16)),
                material=mortar,
                name=f"front_mortar_joint_{course}_{joint}",
            )
            joint += 1
            x += 0.44
        course += 1
        z += course_z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brick_railway_station_clock_tower")

    brick = model.material("deep_red_brick", color=(0.56, 0.18, 0.10, 1.0))
    dark_brick = model.material("shadowed_brick", color=(0.34, 0.10, 0.07, 1.0))
    mortar = model.material("pale_mortar", color=(0.76, 0.68, 0.56, 1.0))
    stone = model.material("limestone_trim", color=(0.70, 0.66, 0.55, 1.0))
    slate = model.material("weathered_slate", color=(0.12, 0.15, 0.17, 1.0))
    cream = model.material("aged_clock_face", color=(0.92, 0.87, 0.74, 1.0))
    black = model.material("painted_black", color=(0.01, 0.01, 0.01, 1.0))
    brass = model.material("aged_brass", color=(0.82, 0.60, 0.24, 1.0))
    glass = model.material("dark_station_glass", color=(0.05, 0.09, 0.12, 1.0))

    tower = model.part("tower")

    # Square brick shaft with a wider stone plinth and cap, scaled like a small
    # station clock tower rather than a tabletop object.
    tower.visual(
        Box((1.84, 1.84, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=stone,
        name="stone_plinth",
    )
    tower.visual(
        Box((1.58, 1.58, 4.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.28)),
        material=brick,
        name="brick_square_body",
    )
    tower.visual(
        Box((1.68, 1.68, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=dark_brick,
        name="lower_belt_course",
    )
    tower.visual(
        Box((1.78, 1.78, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 4.41)),
        material=stone,
        name="cornice",
    )

    # Hip roof cap: a square pyramidal slate roof with a small flat crown.
    roof_geom = LoftGeometry(
        [_square_profile(1.98, 0.0), _square_profile(0.22, 0.70)],
        cap=True,
        closed=True,
    )
    tower.visual(
        mesh_from_geometry(roof_geom, "slate_hip_roof"),
        origin=Origin(xyz=(0.0, 0.0, 4.50)),
        material=slate,
        name="slate_hip_roof",
    )
    tower.visual(
        Cylinder(radius=0.045, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 5.30)),
        material=brass,
        name="roof_finial",
    )

    # Mortar strips and staggered vertical joints make the shaft read as brick.
    _add_front_brickwork(tower, mortar.name, width=1.56, front_y=-0.798)
    for side, yaw, x, y in (
        ("side_0", math.pi / 2.0, 0.797, 0.0),
        ("side_1", math.pi / 2.0, -0.797, 0.0),
    ):
        for course in range(9):
            tower.visual(
                Box((1.54, 0.016, 0.018)),
                origin=Origin(
                    xyz=(x, y, 0.74 + course * 0.38),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=mortar,
                name=f"{side}_mortar_course_{course}",
            )

    # A dark arched station window below the clock, with limestone side trim.
    tower.visual(
        Box((0.42, 0.020, 0.56)),
        origin=Origin(xyz=(0.0, -0.800, 1.50)),
        material=glass,
        name="arched_window_lower_glass",
    )
    tower.visual(
        Cylinder(radius=0.21, length=0.020),
        origin=Origin(xyz=(0.0, -0.800, 1.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="arched_window_top_glass",
    )
    tower.visual(
        Box((0.055, 0.026, 0.64)),
        origin=Origin(xyz=(-0.25, -0.803, 1.49)),
        material=stone,
        name="window_left_jamb",
    )
    tower.visual(
        Box((0.055, 0.026, 0.64)),
        origin=Origin(xyz=(0.25, -0.803, 1.49)),
        material=stone,
        name="window_right_jamb",
    )
    tower.visual(
        Box((0.54, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, -0.803, 1.17)),
        material=stone,
        name="window_sill",
    )

    # Clock face on the front elevation.
    hub_z = 3.34
    tower.visual(
        Cylinder(radius=0.50, length=0.056),
        origin=Origin(xyz=(0.0, -0.803, hub_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="clock_face",
    )
    rim_geom = TorusGeometry(radius=0.505, tube=0.026, radial_segments=18, tubular_segments=48)
    rim_geom.rotate_x(math.pi / 2.0)
    tower.visual(
        mesh_from_geometry(rim_geom, "black_clock_rim"),
        origin=Origin(xyz=(0.0, -0.848, hub_z)),
        material=black,
        name="black_clock_rim",
    )
    _add_clock_ticks(tower, black.name, radius=0.395, y=-0.840, z=hub_z)
    tower.visual(
        Cylinder(radius=0.075, length=0.067),
        origin=Origin(xyz=(0.0, -0.8535, hub_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="fixed_hub_bushing",
    )

    # Separate hands, each on its own revolute joint.  The two pivots share the
    # same X/Z hub center and the same front-back axis, like nested clock shafts.
    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Box((0.070, 0.014, 0.295)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=black,
        name="short_pointer",
    )
    hour_hand.visual(
        Box((0.050, 0.014, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=black,
        name="short_counterweight",
    )
    hour_hand.visual(
        Cylinder(radius=0.070, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hour_hub_cap",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Box((0.038, 0.012, 0.415)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=black,
        name="long_pointer",
    )
    minute_hand.visual(
        Box((0.028, 0.012, 0.155)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=black,
        name="long_counterweight",
    )
    minute_hand.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="minute_hub_cap",
    )

    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=hour_hand,
        origin=Origin(xyz=(0.0, -0.895, hub_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.tau, effort=0.2, velocity=0.20),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=minute_hand,
        origin=Origin(xyz=(0.0, -0.910, hub_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.tau, effort=0.1, velocity=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    ctx.expect_origin_distance(
        hour_hand,
        minute_hand,
        axes="xz",
        max_dist=0.001,
        name="hand pivots share the same clock-face center",
    )
    ctx.expect_contact(
        tower,
        hour_hand,
        elem_a="fixed_hub_bushing",
        elem_b="hour_hub_cap",
        name="hour hand bears on the fixed clock bushing",
    )
    ctx.expect_gap(
        hour_hand,
        minute_hand,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="hour_hub_cap",
        negative_elem="minute_hub_cap",
        name="minute hand touches the front of the hour hand without overlap",
    )
    with ctx.pose({minute_joint: math.pi / 2.0, hour_joint: math.pi / 6.0}):
        ctx.expect_gap(
            hour_hand,
            minute_hand,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="hour_hub_cap",
            negative_elem="minute_hub_cap",
            name="rotated hands remain on touching concentric layers",
        )
        ctx.expect_overlap(
            minute_hand,
            tower,
            axes="xz",
            min_overlap=0.09,
            elem_a="minute_hub_cap",
            elem_b="clock_face",
            name="minute hand stays centered on the clock face when rotated",
        )

    return ctx.report()


object_model = build_object_model()
