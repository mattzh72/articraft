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


BODY_LENGTH = 0.80
BODY_WIDTH = 0.34
BODY_FOOT_HEIGHT = 0.018
BODY_BASE_THICKNESS = 0.022
BODY_RAIL_LENGTH = 0.64
BODY_RAIL_WIDTH = 0.030
BODY_RAIL_HEIGHT = 0.026
BODY_RAIL_Y = 0.100
BODY_TOP = BODY_FOOT_HEIGHT + BODY_BASE_THICKNESS + BODY_RAIL_HEIGHT

SLIDER1_LENGTH = 0.54
SLIDER1_WIDTH = 0.26
SLIDER1_PAD_LENGTH = 0.44
SLIDER1_PAD_WIDTH = 0.026
SLIDER1_PAD_HEIGHT = 0.010
SLIDER1_PAD_Y = 0.090
SLIDER1_DECK_THICKNESS = 0.018
SLIDER1_UPPER_RAIL_LENGTH = 0.42
SLIDER1_UPPER_RAIL_WIDTH = 0.024
SLIDER1_UPPER_RAIL_HEIGHT = 0.012
SLIDER1_UPPER_RAIL_Y = 0.068
SLIDER1_HEIGHT = SLIDER1_PAD_HEIGHT + SLIDER1_DECK_THICKNESS + SLIDER1_UPPER_RAIL_HEIGHT
SLIDER1_TRAVEL = 0.18

SLIDER2_LENGTH = 0.40
SLIDER2_WIDTH = 0.20
SLIDER2_PAD_LENGTH = 0.30
SLIDER2_PAD_WIDTH = 0.022
SLIDER2_PAD_HEIGHT = 0.008
SLIDER2_PAD_Y = 0.068
SLIDER2_DECK_THICKNESS = 0.016
SLIDER2_UPPER_RAIL_LENGTH = 0.28
SLIDER2_UPPER_RAIL_WIDTH = 0.020
SLIDER2_UPPER_RAIL_HEIGHT = 0.010
SLIDER2_UPPER_RAIL_Y = 0.052
SLIDER2_HEIGHT = SLIDER2_PAD_HEIGHT + SLIDER2_DECK_THICKNESS + SLIDER2_UPPER_RAIL_HEIGHT
SLIDER2_TRAVEL = 0.15

SLIDER3_LENGTH = 0.28
SLIDER3_WIDTH = 0.15
SLIDER3_PAD_LENGTH = 0.20
SLIDER3_PAD_WIDTH = 0.018
SLIDER3_PAD_HEIGHT = 0.006
SLIDER3_PAD_Y = 0.052
SLIDER3_DECK_THICKNESS = 0.014
SLIDER3_HEIGHT = SLIDER3_PAD_HEIGHT + SLIDER3_DECK_THICKNESS
SLIDER3_TRAVEL = 0.12


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]) -> cq.Workplane:
    shape: cq.Workplane | None = None
    for size, center in boxes:
        piece = _box_shape(size, center)
        shape = piece if shape is None else shape.union(piece)
    assert shape is not None
    return shape


def _body_shape() -> cq.Workplane:
    return _union_boxes(
        [
            ((BODY_LENGTH, BODY_WIDTH, BODY_BASE_THICKNESS), (0.0, 0.0, BODY_FOOT_HEIGHT + (BODY_BASE_THICKNESS / 2.0))),
            ((0.11, 0.08, BODY_FOOT_HEIGHT), (-0.26, -0.10, BODY_FOOT_HEIGHT / 2.0)),
            ((0.11, 0.08, BODY_FOOT_HEIGHT), (-0.26, 0.10, BODY_FOOT_HEIGHT / 2.0)),
            ((0.11, 0.08, BODY_FOOT_HEIGHT), (0.26, -0.10, BODY_FOOT_HEIGHT / 2.0)),
            ((0.11, 0.08, BODY_FOOT_HEIGHT), (0.26, 0.10, BODY_FOOT_HEIGHT / 2.0)),
            ((BODY_RAIL_LENGTH, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT), (0.0, -BODY_RAIL_Y, BODY_FOOT_HEIGHT + BODY_BASE_THICKNESS + (BODY_RAIL_HEIGHT / 2.0))),
            ((BODY_RAIL_LENGTH, BODY_RAIL_WIDTH, BODY_RAIL_HEIGHT), (0.0, BODY_RAIL_Y, BODY_FOOT_HEIGHT + BODY_BASE_THICKNESS + (BODY_RAIL_HEIGHT / 2.0))),
            ((0.08, BODY_WIDTH - 0.06, BODY_RAIL_HEIGHT), (-0.36, 0.0, BODY_FOOT_HEIGHT + BODY_BASE_THICKNESS + (BODY_RAIL_HEIGHT / 2.0))),
        ]
    )


def _slider_shape(
    *,
    length: float,
    width: float,
    pad_length: float,
    pad_width: float,
    pad_height: float,
    pad_y: float,
    deck_thickness: float,
    upper_rail_length: float | None = None,
    upper_rail_width: float | None = None,
    upper_rail_height: float | None = None,
    upper_rail_y: float | None = None,
) -> cq.Workplane:
    boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = [
        ((length, width, deck_thickness), (0.0, 0.0, pad_height + (deck_thickness / 2.0))),
        ((pad_length, pad_width, pad_height), (0.0, -pad_y, pad_height / 2.0)),
        ((pad_length, pad_width, pad_height), (0.0, pad_y, pad_height / 2.0)),
        ((0.038, width, deck_thickness * 0.55), (-(length / 2.0) + 0.019, 0.0, pad_height + (deck_thickness * 0.275))),
    ]
    if (
        upper_rail_length is not None
        and upper_rail_width is not None
        and upper_rail_height is not None
        and upper_rail_y is not None
    ):
        boxes.extend(
            [
                (
                    (upper_rail_length, upper_rail_width, upper_rail_height),
                    (
                        0.0,
                        -upper_rail_y,
                        pad_height + deck_thickness + (upper_rail_height / 2.0),
                    ),
                ),
                (
                    (upper_rail_length, upper_rail_width, upper_rail_height),
                    (
                        0.0,
                        upper_rail_y,
                        pad_height + deck_thickness + (upper_rail_height / 2.0),
                    ),
                ),
            ]
        )
    return _union_boxes(boxes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_three_stage_axis")

    model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("stage_one", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("stage_two", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("stage_three", rgba=(0.74, 0.77, 0.80, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "service_axis_body"), material="body_dark")
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_TOP)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    slider1 = model.part("slider1")
    slider1.visual(
        mesh_from_cadquery(
            _slider_shape(
                length=SLIDER1_LENGTH,
                width=SLIDER1_WIDTH,
                pad_length=SLIDER1_PAD_LENGTH,
                pad_width=SLIDER1_PAD_WIDTH,
                pad_height=SLIDER1_PAD_HEIGHT,
                pad_y=SLIDER1_PAD_Y,
                deck_thickness=SLIDER1_DECK_THICKNESS,
                upper_rail_length=SLIDER1_UPPER_RAIL_LENGTH,
                upper_rail_width=SLIDER1_UPPER_RAIL_WIDTH,
                upper_rail_height=SLIDER1_UPPER_RAIL_HEIGHT,
                upper_rail_y=SLIDER1_UPPER_RAIL_Y,
            ),
            "service_axis_slider1",
        ),
        material="stage_one",
    )
    slider1.inertial = Inertial.from_geometry(
        Box((SLIDER1_LENGTH, SLIDER1_WIDTH, SLIDER1_HEIGHT)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, SLIDER1_HEIGHT / 2.0)),
    )

    slider2 = model.part("slider2")
    slider2.visual(
        mesh_from_cadquery(
            _slider_shape(
                length=SLIDER2_LENGTH,
                width=SLIDER2_WIDTH,
                pad_length=SLIDER2_PAD_LENGTH,
                pad_width=SLIDER2_PAD_WIDTH,
                pad_height=SLIDER2_PAD_HEIGHT,
                pad_y=SLIDER2_PAD_Y,
                deck_thickness=SLIDER2_DECK_THICKNESS,
                upper_rail_length=SLIDER2_UPPER_RAIL_LENGTH,
                upper_rail_width=SLIDER2_UPPER_RAIL_WIDTH,
                upper_rail_height=SLIDER2_UPPER_RAIL_HEIGHT,
                upper_rail_y=SLIDER2_UPPER_RAIL_Y,
            ),
            "service_axis_slider2",
        ),
        material="stage_two",
    )
    slider2.inertial = Inertial.from_geometry(
        Box((SLIDER2_LENGTH, SLIDER2_WIDTH, SLIDER2_HEIGHT)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, SLIDER2_HEIGHT / 2.0)),
    )

    slider3 = model.part("slider3")
    slider3.visual(
        mesh_from_cadquery(
            _slider_shape(
                length=SLIDER3_LENGTH,
                width=SLIDER3_WIDTH,
                pad_length=SLIDER3_PAD_LENGTH,
                pad_width=SLIDER3_PAD_WIDTH,
                pad_height=SLIDER3_PAD_HEIGHT,
                pad_y=SLIDER3_PAD_Y,
                deck_thickness=SLIDER3_DECK_THICKNESS,
            ),
            "service_axis_slider3",
        ),
        material="stage_three",
    )
    slider3.inertial = Inertial.from_geometry(
        Box((SLIDER3_LENGTH, SLIDER3_WIDTH, SLIDER3_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, SLIDER3_HEIGHT / 2.0)),
    )

    model.articulation(
        "body_to_slider1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider1,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDER1_TRAVEL, effort=800.0, velocity=0.30),
    )
    model.articulation(
        "slider1_to_slider2",
        ArticulationType.PRISMATIC,
        parent=slider1,
        child=slider2,
        origin=Origin(xyz=(0.0, 0.0, SLIDER1_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDER2_TRAVEL, effort=500.0, velocity=0.32),
    )
    model.articulation(
        "slider2_to_slider3",
        ArticulationType.PRISMATIC,
        parent=slider2,
        child=slider3,
        origin=Origin(xyz=(0.0, 0.0, SLIDER2_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDER3_TRAVEL, effort=300.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    slider1 = object_model.get_part("slider1")
    slider2 = object_model.get_part("slider2")
    slider3 = object_model.get_part("slider3")
    body_to_slider1 = object_model.get_articulation("body_to_slider1")
    slider1_to_slider2 = object_model.get_articulation("slider1_to_slider2")
    slider2_to_slider3 = object_model.get_articulation("slider2_to_slider3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "three serial +x prismatic joints",
        (
            body_to_slider1.articulation_type == ArticulationType.PRISMATIC
            and slider1_to_slider2.articulation_type == ArticulationType.PRISMATIC
            and slider2_to_slider3.articulation_type == ArticulationType.PRISMATIC
            and body_to_slider1.axis == (1.0, 0.0, 0.0)
            and slider1_to_slider2.axis == (1.0, 0.0, 0.0)
            and slider2_to_slider3.axis == (1.0, 0.0, 0.0)
            and body_to_slider1.parent == "body"
            and body_to_slider1.child == "slider1"
            and slider1_to_slider2.parent == "slider1"
            and slider1_to_slider2.child == "slider2"
            and slider2_to_slider3.parent == "slider2"
            and slider2_to_slider3.child == "slider3"
        ),
        details="Expected a body-rooted three-stage serial prismatic chain along +X.",
    )

    ctx.check(
        "travel limits stay positive and ordered",
        (
            body_to_slider1.motion_limits is not None
            and slider1_to_slider2.motion_limits is not None
            and slider2_to_slider3.motion_limits is not None
            and body_to_slider1.motion_limits.lower == 0.0
            and slider1_to_slider2.motion_limits.lower == 0.0
            and slider2_to_slider3.motion_limits.lower == 0.0
            and body_to_slider1.motion_limits.upper == SLIDER1_TRAVEL
            and slider1_to_slider2.motion_limits.upper == SLIDER2_TRAVEL
            and slider2_to_slider3.motion_limits.upper == SLIDER3_TRAVEL
        ),
        details="The three sliders should extend forward from the retracted pose.",
    )

    with ctx.pose(
        {
            body_to_slider1: 0.0,
            slider1_to_slider2: 0.0,
            slider2_to_slider3: 0.0,
        }
    ):
        ctx.expect_contact(body, slider1, name="slider1 is carried by the body")
        ctx.expect_contact(slider1, slider2, name="slider2 is carried by slider1")
        ctx.expect_contact(slider2, slider3, name="slider3 is carried by slider2")
        ctx.expect_overlap(body, slider1, axes="xy", min_overlap=0.22, name="body and slider1 visibly overlap")
        ctx.expect_overlap(slider1, slider2, axes="xy", min_overlap=0.16, name="slider1 and slider2 visibly overlap")
        ctx.expect_overlap(slider2, slider3, axes="xy", min_overlap=0.12, name="slider2 and slider3 visibly overlap")

    with ctx.pose(
        {
            body_to_slider1: SLIDER1_TRAVEL,
            slider1_to_slider2: SLIDER2_TRAVEL,
            slider2_to_slider3: SLIDER3_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="extended pose stays clear")
        ctx.expect_overlap(body, slider1, axes="xy", min_overlap=0.18, name="slider1 still overlaps the body when extended")
        ctx.expect_overlap(slider1, slider2, axes="xy", min_overlap=0.12, name="slider2 still overlaps slider1 when extended")
        ctx.expect_overlap(slider2, slider3, axes="xy", min_overlap=0.08, name="slider3 still overlaps slider2 when extended")
        ctx.expect_origin_gap(slider1, body, axis="x", min_gap=0.15, name="slider1 travels forward from the body")
        ctx.expect_origin_gap(slider2, slider1, axis="x", min_gap=0.10, name="slider2 travels forward from slider1")
        ctx.expect_origin_gap(slider3, slider2, axis="x", min_gap=0.07, name="slider3 travels forward from slider2")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
