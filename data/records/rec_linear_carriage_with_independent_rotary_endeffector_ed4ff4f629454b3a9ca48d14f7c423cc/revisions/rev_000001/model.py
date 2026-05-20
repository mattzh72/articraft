from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.18
BASE_THICKNESS = 0.018
GUIDE_LENGTH = 0.66
GUIDE_WIDTH = 0.082
GUIDE_HEIGHT = 0.032
GUIDE_TOP_Z = BASE_THICKNESS + GUIDE_HEIGHT

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.15
CARRIAGE_BODY_HEIGHT = 0.024
CARRIAGE_PAD_LENGTH = 0.156
CARRIAGE_PAD_WIDTH = 0.098
CARRIAGE_PAD_HEIGHT = 0.010
CARRIAGE_SIDE_WALL_HEIGHT = 0.032
CARRIAGE_TOP_BOSS_LENGTH = 0.116
CARRIAGE_TOP_BOSS_WIDTH = 0.108
CARRIAGE_TOP_BOSS_HEIGHT = 0.016
CARRIAGE_TRAVEL = 0.22

HOUSING_RADIUS = 0.055
HOUSING_HEIGHT = 0.056
HOUSING_BOTTOM_Z = CARRIAGE_BODY_HEIGHT + 0.014
HOUSING_TOP_Z = HOUSING_BOTTOM_Z + HOUSING_HEIGHT
HOUSING_MOUNT_Z = HOUSING_TOP_Z - 0.004

SPINDLE_FLANGE_RADIUS = 0.032
SPINDLE_FLANGE_HEIGHT = 0.008
SPINDLE_BODY_RADIUS = 0.028
SPINDLE_BODY_HEIGHT = 0.040
SPINDLE_NOSE_RADIUS = 0.018
SPINDLE_NOSE_HEIGHT = 0.020
SPINDLE_TOTAL_HEIGHT = SPINDLE_FLANGE_HEIGHT + SPINDLE_BODY_HEIGHT + SPINDLE_NOSE_HEIGHT


def _build_rail_shape() -> cq.Workplane:
    hole_x = (-0.24, -0.08, 0.08, 0.24)
    hole_y = (-0.055, 0.055)
    hole_points = [(x, y) for x in hole_x for y in hole_y]

    base = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    base = (
        base.faces(">Z")
        .workplane()
        .pushPoints(hole_points)
        .cskHole(0.009, 0.018, 82.0)
        .edges("|Z")
        .fillet(0.004)
    )

    guide = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
        centered=(True, True, False),
    )
    guide = guide.translate((0.0, 0.0, BASE_THICKNESS))

    rail = base.union(guide)
    return rail


def _build_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    )
    pad = cq.Workplane("XY").box(
        CARRIAGE_PAD_LENGTH,
        CARRIAGE_PAD_WIDTH,
        CARRIAGE_PAD_HEIGHT,
        centered=(True, True, False),
    )
    pad = pad.translate((0.0, 0.0, 0.002))

    left_wall = cq.Workplane("XY").box(
        CARRIAGE_LENGTH - 0.018,
        0.020,
        CARRIAGE_SIDE_WALL_HEIGHT,
        centered=(True, True, False),
    )
    left_wall = left_wall.translate((0.0, 0.052, CARRIAGE_BODY_HEIGHT - 0.006))
    right_wall = left_wall.translate((0.0, -0.104, 0.0))

    carriage = body.union(pad).union(left_wall).union(right_wall)

    left_window = cq.Workplane("XY").box(
        0.090,
        0.018,
        0.022,
        centered=(True, True, False),
    )
    left_window = left_window.translate((-0.006, 0.052, CARRIAGE_BODY_HEIGHT + 0.004))
    right_window = left_window.translate((0.0, -0.104, 0.0))
    carriage = carriage.cut(left_window).cut(right_window)

    top_boss = cq.Workplane("XY").box(
        CARRIAGE_TOP_BOSS_LENGTH,
        CARRIAGE_TOP_BOSS_WIDTH,
        CARRIAGE_TOP_BOSS_HEIGHT,
        centered=(True, True, False),
    )
    top_boss = top_boss.translate((0.0, 0.0, CARRIAGE_BODY_HEIGHT - 0.004))

    housing = cq.Workplane("XY").cylinder(
        HOUSING_HEIGHT,
        HOUSING_RADIUS,
        centered=(True, True, False),
    )
    housing = housing.translate((0.0, 0.0, HOUSING_BOTTOM_Z - 0.004))

    return carriage.union(top_boss).union(housing)


def _build_spindle_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(SPINDLE_FLANGE_RADIUS)
        .extrude(SPINDLE_FLANGE_HEIGHT)
    )

    body = (
        cq.Workplane("XY")
        .workplane(offset=SPINDLE_FLANGE_HEIGHT - 0.002)
        .circle(SPINDLE_BODY_RADIUS)
        .extrude(SPINDLE_BODY_HEIGHT)
    )

    nose = (
        cq.Workplane("XY")
        .workplane(offset=SPINDLE_FLANGE_HEIGHT + SPINDLE_BODY_HEIGHT - 0.004)
        .circle(SPINDLE_NOSE_RADIUS)
        .extrude(SPINDLE_NOSE_HEIGHT)
    )

    index_lug = cq.Workplane("XY").box(
        0.022,
        0.010,
        0.008,
        centered=(True, True, False),
    )
    index_lug = index_lug.translate((0.030, 0.0, 0.016))

    return flange.union(body).union(nose).union(index_lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_spindle_carriage")

    model.material("base_paint", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("carriage_gray", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("spindle_steel", rgba=(0.65, 0.67, 0.70, 1.0))

    rail = model.part("rail_base")
    rail.visual(
        mesh_from_cadquery(_build_rail_shape(), "rail_base_shell"),
        material="base_paint",
        name="rail_shell",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, GUIDE_TOP_Z)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage_shell"),
        material="carriage_gray",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, HOUSING_MOUNT_Z)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_MOUNT_Z / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(_build_spindle_shape(), "spindle_shell"),
        material="spindle_steel",
        name="spindle_shell",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_FLANGE_RADIUS, length=SPINDLE_TOTAL_HEIGHT),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=240.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=14.0,
            velocity=8.0,
        ),
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

    rail = object_model.get_part("rail_base")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    spindle_joint = object_model.get_articulation("carriage_to_spindle")

    ctx.check(
        "slide axis is aligned to the rail",
        tuple(round(v, 4) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "spindle axis is vertical in the carriage frame",
        tuple(round(v, 4) for v in spindle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spindle_joint.axis}",
    )

    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage sits directly on the rail guide at home",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="x",
        min_overlap=0.16,
        name="carriage has substantial supported overlap with the rail at home",
    )
    ctx.expect_origin_distance(
        spindle,
        carriage,
        axes="xy",
        max_dist=0.001,
        name="spindle remains centered on the carriage housing axis",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="spindle seats on top of the bearing housing",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="xy",
        min_overlap=0.048,
        name="spindle occupies the housing footprint on the carriage",
    )

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    if slide_upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: slide_upper}):
            extended_pos = ctx.part_world_position(carriage)
            ctx.expect_overlap(
                carriage,
                rail,
                axes="x",
                min_overlap=0.16,
                name="carriage retains rail engagement at maximum travel",
            )
        ctx.check(
            "carriage translates positively along the rail axis",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.18
            and abs(extended_pos[1] - rest_pos[1]) < 1e-6
            and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    spindle_rest_aabb = ctx.part_element_world_aabb(spindle, elem="spindle_shell")
    with ctx.pose({spindle_joint: pi / 2.0}):
        spindle_quarter_aabb = ctx.part_element_world_aabb(spindle, elem="spindle_shell")

    if spindle_rest_aabb is not None and spindle_quarter_aabb is not None:
        rest_dx = spindle_rest_aabb[1][0] - spindle_rest_aabb[0][0]
        rest_dy = spindle_rest_aabb[1][1] - spindle_rest_aabb[0][1]
        quarter_dx = spindle_quarter_aabb[1][0] - spindle_quarter_aabb[0][0]
        quarter_dy = spindle_quarter_aabb[1][1] - spindle_quarter_aabb[0][1]
        ctx.check(
            "spindle revolute joint turns the asymmetric drive lug around the tool axis",
            rest_dx > rest_dy + 0.004 and quarter_dy > quarter_dx + 0.004,
            details=(
                f"rest_xy=({rest_dx:.4f}, {rest_dy:.4f}), "
                f"quarter_xy=({quarter_dx:.4f}, {quarter_dy:.4f})"
            ),
        )
    else:
        ctx.fail(
            "spindle shell AABBs are available for pose testing",
            details=f"rest={spindle_rest_aabb}, quarter={spindle_quarter_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
