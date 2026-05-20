from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.62
BASE_WIDTH = 0.22
BASE_THICKNESS = 0.02

OUTER_GUIDE_X = 0.04
OUTER_LENGTH = 0.54
OUTER_WIDTH = 0.13
OUTER_HEIGHT = 0.082
OUTER_WALL = 0.008
OUTER_FLOOR = 0.008

SUPPORT_HEIGHT = 0.022
SUPPORT_LENGTH = 0.10
SUPPORT_WIDTH = 0.11
SUPPORT_X0 = 0.08
SUPPORT_X1 = 0.36
GUIDE_BASE_Z = BASE_THICKNESS + SUPPORT_HEIGHT

MIDDLE_LENGTH = 0.48
MIDDLE_WIDTH = 0.098
MIDDLE_HEIGHT = 0.060
MIDDLE_WALL = 0.007
MIDDLE_FLOOR = 0.006

INNER_LENGTH = 0.38
INNER_BAR_WIDTH = 0.068
INNER_BAR_HEIGHT = 0.024
INNER_TOP_WIDTH = 0.082
INNER_TOP_LENGTH = 0.18
INNER_TOP_THICKNESS = 0.012
INNER_FRONT_PLATE_THICKNESS = 0.012
INNER_FRONT_PLATE_HEIGHT = 0.048

OUTER_TO_MIDDLE_INSERT = 0.05
OUTER_TO_MIDDLE_Z = GUIDE_BASE_Z + OUTER_FLOOR
OUTER_TRAVEL = 0.18

MIDDLE_TO_INNER_INSERT = 0.07
MIDDLE_TO_INNER_Z = MIDDLE_FLOOR
INNER_TRAVEL = 0.16

def _outer_base_mount_shape():
    hole_points = [
        (-0.24, -0.07),
        (-0.24, 0.07),
        (0.24, -0.07),
        (0.24, 0.07),
    ]
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(False, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(0.012)
    )

    support_a = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            SUPPORT_WIDTH,
            SUPPORT_HEIGHT,
            centered=(False, True, False),
        )
        .translate((SUPPORT_X0, 0.0, BASE_THICKNESS))
    )
    support_b = (
        cq.Workplane("XY")
        .box(
            SUPPORT_LENGTH,
            SUPPORT_WIDTH,
            SUPPORT_HEIGHT,
            centered=(False, True, False),
        )
        .translate((SUPPORT_X1, 0.0, BASE_THICKNESS))
    )
    return base.union(support_a).union(support_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_linear_stack")

    model.material("powder_coat", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("dark_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("brushed_aluminum", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("light_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        mesh_from_cadquery(_outer_base_mount_shape(), "outer_base_mount"),
        material="powder_coat",
        name="outer_base_mount",
    )
    outer_guide.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_FLOOR)),
        origin=Origin(
            xyz=(
                OUTER_GUIDE_X + (OUTER_LENGTH / 2.0),
                0.0,
                GUIDE_BASE_Z + (OUTER_FLOOR / 2.0),
            )
        ),
        material="dark_steel",
        name="outer_track_floor",
    )
    outer_guide.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_GUIDE_X + (OUTER_LENGTH / 2.0),
                (OUTER_WIDTH / 2.0) - (OUTER_WALL / 2.0),
                GUIDE_BASE_Z + (OUTER_HEIGHT / 2.0),
            )
        ),
        material="dark_steel",
        name="outer_left_wall",
    )
    outer_guide.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_GUIDE_X + (OUTER_LENGTH / 2.0),
                -(OUTER_WIDTH / 2.0) + (OUTER_WALL / 2.0),
                GUIDE_BASE_Z + (OUTER_HEIGHT / 2.0),
            )
        ),
        material="dark_steel",
        name="outer_right_wall",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, GUIDE_BASE_Z + OUTER_HEIGHT)),
        mass=6.0,
        origin=Origin(
            xyz=(
                BASE_LENGTH / 2.0,
                0.0,
                (GUIDE_BASE_Z + OUTER_HEIGHT) / 2.0,
            )
        ),
    )

    middle_carriage = model.part("middle_carriage")
    middle_carriage.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_FLOOR)),
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_FLOOR / 2.0)),
        material="brushed_aluminum",
        name="middle_track_floor",
    )
    middle_carriage.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                (MIDDLE_WIDTH / 2.0) - (MIDDLE_WALL / 2.0),
                MIDDLE_HEIGHT / 2.0,
            )
        ),
        material="brushed_aluminum",
        name="middle_left_wall",
    )
    middle_carriage.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                -(MIDDLE_WIDTH / 2.0) + (MIDDLE_WALL / 2.0),
                MIDDLE_HEIGHT / 2.0,
            )
        ),
        material="brushed_aluminum",
        name="middle_right_wall",
    )
    middle_carriage.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    inner_carriage = model.part("inner_carriage")
    inner_carriage.visual(
        Box((INNER_LENGTH, INNER_BAR_WIDTH, INNER_BAR_HEIGHT)),
        origin=Origin(
            xyz=(INNER_LENGTH / 2.0, 0.0, INNER_BAR_HEIGHT / 2.0)
        ),
        material="light_aluminum",
        name="inner_lower_bar",
    )
    inner_carriage.visual(
        Box((INNER_TOP_LENGTH, INNER_TOP_WIDTH, INNER_TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.10 + (INNER_TOP_LENGTH / 2.0),
                0.0,
                INNER_BAR_HEIGHT + (INNER_TOP_THICKNESS / 2.0),
            )
        ),
        material="light_aluminum",
        name="inner_top_saddle",
    )
    inner_carriage.visual(
        Box((INNER_FRONT_PLATE_THICKNESS, INNER_TOP_WIDTH, INNER_FRONT_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_LENGTH - (INNER_FRONT_PLATE_THICKNESS / 2.0),
                0.0,
                INNER_FRONT_PLATE_HEIGHT / 2.0,
            )
        ),
        material="light_aluminum",
        name="inner_front_plate",
    )
    inner_carriage.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_TOP_WIDTH, INNER_FRONT_PLATE_HEIGHT)),
        mass=1.2,
        origin=Origin(
            xyz=(
                INNER_LENGTH / 2.0,
                0.0,
                INNER_FRONT_PLATE_HEIGHT / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=middle_carriage,
        origin=Origin(xyz=(OUTER_GUIDE_X + OUTER_TO_MIDDLE_INSERT, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_carriage,
        child=inner_carriage,
        origin=Origin(xyz=(MIDDLE_TO_INNER_INSERT, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=INNER_TRAVEL,
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
    outer_guide = object_model.get_part("outer_guide")
    middle_carriage = object_model.get_part("middle_carriage")
    inner_carriage = object_model.get_part("inner_carriage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    outer_floor = outer_guide.get_visual("outer_track_floor")
    middle_floor = middle_carriage.get_visual("middle_track_floor")
    inner_bar = inner_carriage.get_visual("inner_lower_bar")

    ctx.check(
        "outer stage is a +X prismatic joint",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and outer_slide.axis == (1.0, 0.0, 0.0)
        and outer_slide.motion_limits is not None
        and outer_slide.motion_limits.lower == 0.0
        and outer_slide.motion_limits.upper == OUTER_TRAVEL,
        details=f"axis={outer_slide.axis}, limits={outer_slide.motion_limits}",
    )
    ctx.check(
        "inner stage is a +X prismatic joint",
        inner_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.axis == (1.0, 0.0, 0.0)
        and inner_slide.motion_limits is not None
        and inner_slide.motion_limits.lower == 0.0
        and inner_slide.motion_limits.upper == INNER_TRAVEL,
        details=f"axis={inner_slide.axis}, limits={inner_slide.motion_limits}",
    )

    ctx.expect_within(
        middle_carriage,
        outer_guide,
        axes="yz",
        margin=0.0,
        name="middle carriage stays within the outer guide section",
    )
    ctx.expect_contact(
        middle_carriage,
        outer_guide,
        elem_a=middle_floor,
        elem_b=outer_floor,
        contact_tol=1e-6,
        name="middle carriage is physically supported by the outer guide",
    )
    ctx.expect_overlap(
        middle_carriage,
        outer_guide,
        axes="x",
        min_overlap=0.30,
        name="middle carriage remains inserted in the outer guide at rest",
    )
    ctx.expect_within(
        inner_carriage,
        middle_carriage,
        axes="yz",
        margin=0.0,
        name="inner carriage stays within the middle carriage section",
    )
    ctx.expect_contact(
        inner_carriage,
        middle_carriage,
        elem_a=inner_bar,
        elem_b=middle_floor,
        contact_tol=1e-6,
        name="inner carriage is physically supported by the middle carriage",
    )
    ctx.expect_overlap(
        inner_carriage,
        middle_carriage,
        axes="x",
        min_overlap=0.34,
        name="inner carriage remains inserted in the middle carriage at rest",
    )

    middle_rest = ctx.part_world_position(middle_carriage)
    with ctx.pose({outer_slide: OUTER_TRAVEL}):
        ctx.expect_within(
            middle_carriage,
            outer_guide,
            axes="yz",
            margin=0.0,
            name="extended middle carriage stays guided by the outer section",
        )
        ctx.expect_overlap(
            middle_carriage,
            outer_guide,
            axes="x",
            min_overlap=0.28,
            name="extended middle carriage still retains insertion in the outer guide",
        )
        middle_extended = ctx.part_world_position(middle_carriage)

    ctx.check(
        "middle carriage extends along +X",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.10,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    inner_rest = ctx.part_world_position(inner_carriage)
    with ctx.pose({inner_slide: INNER_TRAVEL}):
        middle_during_inner = ctx.part_world_position(middle_carriage)
        ctx.expect_within(
            inner_carriage,
            middle_carriage,
            axes="yz",
            margin=0.0,
            name="extended inner carriage stays guided by the middle section",
        )
        ctx.expect_overlap(
            inner_carriage,
            middle_carriage,
            axes="x",
            min_overlap=0.22,
            name="extended inner carriage still retains insertion in the middle carriage",
        )
        inner_extended = ctx.part_world_position(inner_carriage)

    ctx.check(
        "inner carriage extends relative to the middle carriage",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.10,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )
    ctx.check(
        "inner stage actuation does not move the middle stage",
        middle_rest is not None
        and middle_during_inner is not None
        and abs(middle_during_inner[0] - middle_rest[0]) < 1e-6
        and abs(middle_during_inner[1] - middle_rest[1]) < 1e-6
        and abs(middle_during_inner[2] - middle_rest[2]) < 1e-6,
        details=f"rest={middle_rest}, inner_only_pose={middle_during_inner}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
