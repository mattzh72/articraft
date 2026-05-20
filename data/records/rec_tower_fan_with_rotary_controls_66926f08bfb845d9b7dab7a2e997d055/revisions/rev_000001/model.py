from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_RADIUS = 0.165
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.062
PEDESTAL_HEIGHT = 0.040
BEARING_CAP_RADIUS = 0.048
BEARING_CAP_HEIGHT = 0.010
OSCILLATION_Z = 0.065

HOUSING_HEIGHT = 0.918
HOUSING_FRONT_Y = 0.049
SHELL_SECTION_Z = 0.470
SHELL_SECTION_HEIGHT = 0.760
TOP_SURFACE_Z = HOUSING_HEIGHT

GRILLE_WIDTH = 0.086
GRILLE_HEIGHT = 0.736
GRILLE_Z_CENTER = 0.472

DRUM_BOTTOM_Z = 0.108
DRUM_HEIGHT = 0.700
DRUM_RADIUS = 0.033
DRUM_BLADE_COUNT = 24


def _build_grille_mesh():
    grille = SlotPatternPanelGeometry(
        (GRILLE_WIDTH, GRILLE_HEIGHT),
        0.0025,
        slot_size=(0.024, 0.0038),
        pitch=(0.010, 0.031),
        frame=0.006,
        corner_radius=0.013,
        slot_angle_deg=88.0,
        center=True,
    )
    grille.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(grille, "tower_fan_grille")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="oscillating_tower_fan")

    base_black = model.material("base_black", rgba=(0.13, 0.14, 0.15, 1.0))
    shell_silver = model.material("shell_silver", rgba=(0.84, 0.86, 0.88, 1.0))
    grille_charcoal = model.material("grille_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    drum_black = model.material("drum_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.12, 0.13, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.58, 0.60, 0.63, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=base_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=base_black,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=BEARING_CAP_RADIUS, length=BEARING_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=base_black,
        name="bearing_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=OSCILLATION_Z),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_Z * 0.5)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.112, 0.108, 0.095)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material=shell_silver,
        name="motor_pod",
    )
    housing.visual(
        Box((0.072, 0.012, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.045, SHELL_SECTION_Z)),
        material=shell_silver,
        name="rear_spine",
    )
    housing.visual(
        Box((0.010, 0.090, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(-0.055, 0.0, SHELL_SECTION_Z)),
        material=shell_silver,
        name="left_side",
    )
    housing.visual(
        Box((0.010, 0.090, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(0.055, 0.0, SHELL_SECTION_Z)),
        material=shell_silver,
        name="right_side",
    )
    housing.visual(
        Box((0.008, 0.036, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(-0.049, 0.024, SHELL_SECTION_Z), rpy=(0.0, 0.0, 0.46)),
        material=shell_silver,
        name="left_front_facet",
    )
    housing.visual(
        Box((0.008, 0.036, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(0.049, 0.024, SHELL_SECTION_Z), rpy=(0.0, 0.0, -0.46)),
        material=shell_silver,
        name="right_front_facet",
    )
    housing.visual(
        Box((0.012, 0.018, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(-0.044, 0.040, SHELL_SECTION_Z)),
        material=shell_silver,
        name="left_front_post",
    )
    housing.visual(
        Box((0.012, 0.018, SHELL_SECTION_HEIGHT)),
        origin=Origin(xyz=(0.044, 0.040, SHELL_SECTION_Z)),
        material=shell_silver,
        name="right_front_post",
    )
    housing.visual(
        Box((0.108, 0.102, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=shell_silver,
        name="top_cap",
    )
    housing.visual(
        Box((0.094, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.909)),
        material=shell_silver,
        name="control_pod",
    )
    housing.visual(
        Box((0.018, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.100)),
        material=shell_silver,
        name="lower_bearing_block",
    )
    housing.visual(
        Box((0.018, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.015, 0.838)),
        material=shell_silver,
        name="upper_bearing_block",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.112, 0.108, HOUSING_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT * 0.5)),
    )

    grille = model.part("grille")
    grille.visual(
        _build_grille_mesh(),
        material=grille_charcoal,
        name="front_grille",
    )
    grille.inertial = Inertial.from_geometry(
        Box((GRILLE_WIDTH, 0.003, GRILLE_HEIGHT)),
        mass=0.22,
        origin=Origin(),
    )

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=0.005, length=DRUM_HEIGHT + 0.034),
        origin=Origin(xyz=(0.0, 0.0, (DRUM_HEIGHT + 0.034) * 0.5)),
        material=drum_black,
        name="shaft",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=drum_black,
        name="lower_end_ring",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, DRUM_HEIGHT - 0.004)),
        material=drum_black,
        name="upper_end_ring",
    )
    for blade_index in range(DRUM_BLADE_COUNT):
        angle = (2.0 * math.pi * blade_index) / DRUM_BLADE_COUNT
        drum.visual(
            Box((0.004, 0.010, DRUM_HEIGHT - 0.010)),
            origin=Origin(
                xyz=(0.029 * math.cos(angle), 0.029 * math.sin(angle), DRUM_HEIGHT * 0.5),
                rpy=(0.0, 0.0, angle),
            ),
            material=drum_black,
            name=f"blade_{blade_index:02d}",
        )
    drum.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, DRUM_HEIGHT + 0.010)),
        material=drum_black,
        name="top_collar",
    )
    drum.visual(
        Box((0.014, 0.006, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, DRUM_HEIGHT + 0.010)),
        material=drum_black,
        name="index_block",
    )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=DRUM_RADIUS, length=DRUM_HEIGHT + 0.034),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, (DRUM_HEIGHT + 0.034) * 0.5)),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=knob_cap,
        name="shaft",
    )
    speed_knob.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=knob_black,
        name="body",
    )
    speed_knob.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=knob_cap,
        name="cap",
    )
    speed_knob.visual(
        Box((0.004, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.013, 0.026)),
        material=knob_cap,
        name="indicator",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.030),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=knob_cap,
        name="shaft",
    )
    timer_knob.visual(
        Cylinder(radius=0.020, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=knob_black,
        name="body",
    )
    timer_knob.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=knob_cap,
        name="cap",
    )
    timer_knob.visual(
        Box((0.004, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.014, 0.027)),
        material=knob_cap,
        name="indicator",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.031),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, OSCILLATION_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "housing_to_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(0.0, 0.049, GRILLE_Z_CENTER)),
    )
    model.articulation(
        "housing_to_drum",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, DRUM_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )
    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(-0.027, 0.000, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "housing_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=timer_knob,
        origin=Origin(xyz=(0.027, 0.000, TOP_SURFACE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    grille = object_model.get_part("grille")
    drum = object_model.get_part("drum")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")

    housing_joint = object_model.get_articulation("base_to_housing")
    drum_joint = object_model.get_articulation("housing_to_drum")
    speed_joint = object_model.get_articulation("housing_to_speed_knob")
    timer_joint = object_model.get_articulation("housing_to_timer_knob")

    with ctx.pose({housing_joint: 0.0}):
        ctx.expect_gap(
            housing,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.004,
            name="housing seats on the base bearing",
        )
        ctx.expect_overlap(
            grille,
            housing,
            axes="xz",
            min_overlap=0.070,
            name="front grille stays registered to the housing opening",
        )
        ctx.expect_origin_distance(
            drum,
            housing,
            axes="xy",
            max_dist=0.001,
            name="blower drum remains centered in the housing",
        )

    timer_rest = ctx.part_world_position(timer_knob)
    with ctx.pose({housing_joint: 0.60}):
        timer_oscillated = ctx.part_world_position(timer_knob)
    ctx.check(
        "housing oscillation swings the top controls around the base axis",
        timer_rest is not None
        and timer_oscillated is not None
        and timer_oscillated[1] > timer_rest[1] + 0.012,
        details=f"rest={timer_rest}, oscillated={timer_oscillated}",
    )

    drum_rest = _aabb_center(ctx.part_element_world_aabb(drum, elem="index_block"))
    with ctx.pose({drum_joint: math.pi / 2.0}):
        drum_quarter_turn = _aabb_center(ctx.part_element_world_aabb(drum, elem="index_block"))
    ctx.check(
        "blower drum rotates about the vertical centerline",
        drum_rest is not None
        and drum_quarter_turn is not None
        and abs(drum_quarter_turn[1] - drum_rest[1]) > 0.008
        and abs(drum_quarter_turn[0] - drum_rest[0]) > 0.010,
        details=f"rest={drum_rest}, quarter_turn={drum_quarter_turn}",
    )

    speed_rest = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="indicator"))
    with ctx.pose({speed_joint: math.pi / 2.0}):
        speed_quarter_turn = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="indicator"))
    ctx.check(
        "speed knob rotates on its own shaft",
        speed_rest is not None
        and speed_quarter_turn is not None
        and abs(speed_quarter_turn[0] - speed_rest[0]) > 0.008
        and abs(speed_quarter_turn[1] - speed_rest[1]) > 0.008,
        details=f"rest={speed_rest}, quarter_turn={speed_quarter_turn}",
    )

    timer_rest_indicator = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="indicator"))
    with ctx.pose({timer_joint: math.pi / 2.0}):
        timer_quarter_turn = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="indicator"))
    ctx.check(
        "timer knob rotates on its own shaft",
        timer_rest_indicator is not None
        and timer_quarter_turn is not None
        and abs(timer_quarter_turn[0] - timer_rest_indicator[0]) > 0.008
        and abs(timer_quarter_turn[1] - timer_rest_indicator[1]) > 0.008,
        details=f"rest={timer_rest_indicator}, quarter_turn={timer_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
