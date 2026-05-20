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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.82
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.02
RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.06
RAIL_HEIGHT = 0.05
STOP_LENGTH = 0.03
STOP_WIDTH = 0.074
STOP_HEIGHT = 0.065

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.14
CHEEK_WIDTH = 0.03
CHEEK_HEIGHT = 0.074
CHEEK_CENTER_Y = 0.055
CHEEK_CENTER_Z = 0.015
SLIDER_PAD_WIDTH = 0.011
SLIDER_PAD_HEIGHT = 0.048
SLIDER_PAD_CENTER_Y = 0.0355
TOP_BRIDGE_HEIGHT = 0.025
TOP_BRIDGE_CENTER_Z = 0.059
EAR_LENGTH = 0.028
EAR_WIDTH = 0.022
EAR_HEIGHT = 0.045
EAR_CENTER_X = 0.102
EAR_CENTER_Y = 0.047
HINGE_Z = 0.045

BARREL_RADIUS = 0.012
BARREL_LENGTH = 0.072
OUTPUT_BEAM_LENGTH = 0.20
OUTPUT_BEAM_WIDTH = 0.04
OUTPUT_BEAM_HEIGHT = 0.03
OUTPUT_BEAM_CENTER_X = 0.112
OUTPUT_PAD_LENGTH = 0.032
OUTPUT_PAD_WIDTH = 0.05
OUTPUT_PAD_HEIGHT = 0.024
OUTPUT_PAD_CENTER_X = 0.206

SLIDE_LOWER = -0.19
SLIDE_UPPER = 0.19
HINGE_UPPER = 1.20


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_hinge_chain")

    model.material("base_paint", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_orange", rgba=(0.87, 0.47, 0.13, 1.0))
    model.material("link_blue", rgba=(0.17, 0.33, 0.68, 1.0))
    model.material("pin_steel", rgba=(0.58, 0.60, 0.64, 1.0))

    base_guide = model.part("base_guide")
    base_guide.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_paint",
        name="guide_plate",
    )
    base_guide.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + (RAIL_HEIGHT / 2.0)),
        ),
        material="rail_steel",
        name="guide_rail",
    )
    stop_center_x = (RAIL_LENGTH / 2.0) - (STOP_LENGTH / 2.0)
    stop_center_z = BASE_THICKNESS + (STOP_HEIGHT / 2.0)
    base_guide.visual(
        Box((STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT)),
        origin=Origin(xyz=(stop_center_x, 0.0, stop_center_z)),
        material="base_paint",
        name="front_stop",
    )
    base_guide.visual(
        Box((STOP_LENGTH, STOP_WIDTH, STOP_HEIGHT)),
        origin=Origin(xyz=(-stop_center_x, 0.0, stop_center_z)),
        material="base_paint",
        name="rear_stop",
    )
    base_guide.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + RAIL_HEIGHT)),
        mass=12.0,
        origin=Origin(
            xyz=(0.0, 0.0, (BASE_THICKNESS + RAIL_HEIGHT) / 2.0),
        ),
    )

    sliding_carriage = model.part("sliding_carriage")
    sliding_carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BRIDGE_CENTER_Z)),
        material="carriage_orange",
        name="top_bridge",
    )
    sliding_carriage.visual(
        Box((CARRIAGE_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, CHEEK_CENTER_Z)),
        material="carriage_orange",
        name="left_cheek",
    )
    sliding_carriage.visual(
        Box((CARRIAGE_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, CHEEK_CENTER_Z)),
        material="carriage_orange",
        name="right_cheek",
    )
    sliding_carriage.visual(
        Box((CARRIAGE_LENGTH, SLIDER_PAD_WIDTH, SLIDER_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, SLIDER_PAD_CENTER_Y, 0.0)),
        material="pin_steel",
        name="left_slider_pad",
    )
    sliding_carriage.visual(
        Box((CARRIAGE_LENGTH, SLIDER_PAD_WIDTH, SLIDER_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -SLIDER_PAD_CENTER_Y, 0.0)),
        material="pin_steel",
        name="right_slider_pad",
    )
    sliding_carriage.visual(
        Box((EAR_LENGTH, EAR_WIDTH, EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_CENTER_X, EAR_CENTER_Y, HINGE_Z)),
        material="carriage_orange",
        name="left_ear",
    )
    sliding_carriage.visual(
        Box((EAR_LENGTH, EAR_WIDTH, EAR_HEIGHT)),
        origin=Origin(xyz=(EAR_CENTER_X, -EAR_CENTER_Y, HINGE_Z)),
        material="carriage_orange",
        name="right_ear",
    )
    sliding_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.10)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    output_link = model.part("output_link")
    output_link.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="hinge_barrel",
    )
    output_link.visual(
        Box((OUTPUT_BEAM_LENGTH, OUTPUT_BEAM_WIDTH, OUTPUT_BEAM_HEIGHT)),
        origin=Origin(xyz=(OUTPUT_BEAM_CENTER_X, 0.0, 0.0)),
        material="link_blue",
        name="output_beam",
    )
    output_link.visual(
        Box((OUTPUT_PAD_LENGTH, OUTPUT_PAD_WIDTH, OUTPUT_PAD_HEIGHT)),
        origin=Origin(xyz=(OUTPUT_PAD_CENTER_X, 0.0, 0.0)),
        material="link_blue",
        name="output_pad",
    )
    output_link.inertial = Inertial.from_geometry(
        Box((0.24, 0.07, 0.04)),
        mass=1.1,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_guide,
        child=sliding_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (RAIL_HEIGHT / 2.0))),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.45,
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
        ),
    )
    model.articulation(
        "carriage_to_output",
        ArticulationType.REVOLUTE,
        parent=sliding_carriage,
        child=output_link,
        origin=Origin(xyz=(EAR_CENTER_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=HINGE_UPPER,
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

    base_guide = object_model.get_part("base_guide")
    sliding_carriage = object_model.get_part("sliding_carriage")
    output_link = object_model.get_part("output_link")
    slide = object_model.get_articulation("base_to_carriage")
    hinge = object_model.get_articulation("carriage_to_output")

    ctx.check("base guide present", base_guide is not None)
    ctx.check("sliding carriage present", sliding_carriage is not None)
    ctx.check("output link present", output_link is not None)
    ctx.check(
        "slide joint axis follows guide x-axis",
        tuple(round(v, 6) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "hinge pin axis runs across carriage width",
        tuple(round(v, 6) for v in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_contact(
            sliding_carriage,
            base_guide,
            elem_a="left_slider_pad",
            elem_b="guide_rail",
            name="left slider pad is supported by the guide rail",
        )
        ctx.expect_contact(
            output_link,
            sliding_carriage,
            elem_a="hinge_barrel",
            elem_b="left_ear",
            name="hinge barrel bears against the left clevis ear",
        )
        ctx.expect_contact(
            output_link,
            sliding_carriage,
            elem_a="hinge_barrel",
            elem_b="right_ear",
            name="hinge barrel bears against the right clevis ear",
        )
        ctx.expect_gap(
            sliding_carriage,
            base_guide,
            axis="z",
            positive_elem="left_cheek",
            negative_elem="guide_plate",
            min_gap=0.0025,
            max_gap=0.0035,
            name="carriage cheek clears the base plate",
        )
        ctx.expect_gap(
            output_link,
            sliding_carriage,
            axis="x",
            positive_elem="output_beam",
            negative_elem="top_bridge",
            min_gap=0.020,
            max_gap=0.028,
            name="output beam starts ahead of the carriage front",
        )
        ctx.expect_overlap(
            sliding_carriage,
            base_guide,
            axes="x",
            min_overlap=0.18,
            name="home pose keeps carriage engaged with the guide",
        )

    rest_carriage_pos = ctx.part_world_position(sliding_carriage)
    with ctx.pose({slide: SLIDE_UPPER, hinge: 0.0}):
        ctx.expect_overlap(
            sliding_carriage,
            base_guide,
            axes="x",
            min_overlap=0.18,
            name="max extension still retains guide overlap",
        )
        extended_carriage_pos = ctx.part_world_position(sliding_carriage)

    ctx.check(
        "prismatic joint drives carriage forward along +x",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.18,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        rest_beam_aabb = ctx.part_element_world_aabb(output_link, elem="output_beam")
    with ctx.pose({slide: 0.0, hinge: HINGE_UPPER}):
        opened_beam_aabb = ctx.part_element_world_aabb(output_link, elem="output_beam")

    ctx.check(
        "hinge raises the output link upward",
        rest_beam_aabb is not None
        and opened_beam_aabb is not None
        and opened_beam_aabb[1][2] > rest_beam_aabb[1][2] + 0.10,
        details=f"rest={rest_beam_aabb}, opened={opened_beam_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
