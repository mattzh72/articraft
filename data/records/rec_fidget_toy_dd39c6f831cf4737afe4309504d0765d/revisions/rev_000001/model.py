from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_LENGTH = 0.086
HOUSING_WIDTH = 0.048
HOUSING_HEIGHT = 0.026
SLOT_WIDTH = 0.036
SLOT_HEIGHT = 0.008
SLOT_Z = -0.003
SLOT_FACE_X = -HOUSING_LENGTH / 2.0

TONGUE_LENGTH = 0.120
TONGUE_WIDTH = 0.027
TONGUE_THICKNESS = 0.003
TONGUE_INSERTED_LENGTH = 0.064
TONGUE_TRAVEL = 0.040

BUTTON_LENGTH = 0.032
BUTTON_WIDTH = 0.026
BUTTON_HEIGHT = 0.005
BUTTON_TRAVEL = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="seat_belt_buckle_clicker")

    black_plastic = Material("slightly_textured_black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    dark_recess = Material("shadowed_slot_black", rgba=(0.002, 0.002, 0.002, 1.0))
    button_red = Material("satin_red_release_button", rgba=(0.72, 0.035, 0.025, 1.0))
    brushed_steel = Material("brushed_stainless_steel", rgba=(0.72, 0.70, 0.66, 1.0))

    # The housing is a compact, rounded plastic shell with two functional
    # openings cut into it: the front tongue slot and the top release-button well.
    housing_shape = cq.Workplane("XY").box(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_HEIGHT)
    housing_shape = housing_shape.edges().fillet(0.004)

    slot_cutter = (
        cq.Workplane("XY")
        .box(0.078, SLOT_WIDTH, SLOT_HEIGHT)
        .translate((-0.018, 0.0, SLOT_Z))
    )
    button_well_cutter = (
        cq.Workplane("XY")
        .box(BUTTON_LENGTH, BUTTON_WIDTH, 0.018)
        .translate((-0.003, 0.0, HOUSING_HEIGHT / 2.0 + 0.0005))
    )
    housing_shape = housing_shape.cut(slot_cutter).cut(button_well_cutter)

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(housing_shape, "housing_shell", tolerance=0.0007, angular_tolerance=0.08),
        material=black_plastic,
        name="housing_shell",
    )

    # A dark back wall at the end of the tongue slot makes the slot read as a
    # recessed buckle throat without blocking the sliding tongue path.
    housing.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(SLOT_WIDTH, SLOT_HEIGHT, 0.0015), 0.001, center=True),
            "slot_back_shadow",
        ),
        origin=Origin(xyz=(0.021, 0.0, SLOT_Z), rpy=(1.57079632679, 0.0, 1.57079632679)),
        material=dark_recess,
        name="slot_back_shadow",
    )

    tongue_outer = rounded_rect_profile(TONGUE_LENGTH, TONGUE_WIDTH, 0.0045, corner_segments=8)
    latch_hole = [
        (x + 0.030, y)
        for x, y in rounded_rect_profile(0.020, 0.010, 0.0025, corner_segments=6)
    ]
    tongue_plate_mesh = ExtrudeWithHolesGeometry(
        tongue_outer,
        [latch_hole],
        TONGUE_THICKNESS,
        center=True,
    )

    tongue = model.part("tongue")
    tongue.visual(
        mesh_from_geometry(tongue_plate_mesh, "tongue_plate"),
        # The tongue part frame sits at the slot mouth.  More of the plate
        # extends into +X than -X so the clicker remains retained while it slides.
        origin=Origin(
            xyz=(
                TONGUE_INSERTED_LENGTH - TONGUE_LENGTH / 2.0,
                0.0,
                SLOT_Z,
            )
        ),
        material=brushed_steel,
        name="tongue_plate",
    )

    button_profile = rounded_rect_profile(BUTTON_LENGTH, BUTTON_WIDTH, 0.006, corner_segments=10)
    button = model.part("button")
    button.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(button_profile, BUTTON_HEIGHT + BUTTON_TRAVEL, cap=True),
            "release_button",
        ),
        origin=Origin(xyz=(0.0, 0.0, -BUTTON_TRAVEL)),
        material=button_red,
        name="release_button",
    )

    model.articulation(
        "housing_to_tongue",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tongue,
        origin=Origin(xyz=(SLOT_FACE_X, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=TONGUE_TRAVEL),
    )

    model.articulation(
        "housing_to_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button,
        origin=Origin(xyz=(-0.003, 0.0, HOUSING_HEIGHT / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    tongue = object_model.get_part("tongue")
    button = object_model.get_part("button")
    tongue_slide = object_model.get_articulation("housing_to_tongue")
    button_slide = object_model.get_articulation("housing_to_button")

    ctx.expect_within(
        tongue,
        housing,
        axes="yz",
        margin=0.001,
        elem_a="tongue_plate",
        elem_b="housing_shell",
        name="tongue plate is centered in the buckle slot envelope",
    )
    ctx.expect_overlap(
        tongue,
        housing,
        axes="x",
        min_overlap=0.055,
        elem_a="tongue_plate",
        elem_b="housing_shell",
        name="inserted tongue has substantial retained length",
    )
    ctx.expect_within(
        button,
        housing,
        axes="xy",
        margin=0.002,
        elem_a="release_button",
        elem_b="housing_shell",
        name="release button sits within the top opening footprint",
    )
    button_aabb = ctx.part_element_world_aabb(button, elem="release_button")
    housing_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    ctx.check(
        "release button starts proud above the housing top",
        button_aabb is not None
        and housing_aabb is not None
        and button_aabb[1][2] > housing_aabb[1][2] + 0.004,
        details=f"button_aabb={button_aabb}, housing_aabb={housing_aabb}",
    )

    tongue_rest = ctx.part_world_position(tongue)
    with ctx.pose({tongue_slide: TONGUE_TRAVEL}):
        ctx.expect_overlap(
            tongue,
            housing,
            axes="x",
            min_overlap=0.018,
            elem_a="tongue_plate",
            elem_b="housing_shell",
            name="withdrawn tongue remains captured in the slot",
        )
        tongue_extended = ctx.part_world_position(tongue)

    ctx.check(
        "tongue slider withdraws outward from the slot",
        tongue_rest is not None
        and tongue_extended is not None
        and tongue_extended[0] < tongue_rest[0] - 0.030,
        details=f"rest={tongue_rest}, extended={tongue_extended}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_slide: BUTTON_TRAVEL}):
        ctx.expect_within(
            button,
            housing,
            axes="xy",
            margin=0.002,
            elem_a="release_button",
            elem_b="housing_shell",
            name="depressed button stays guided by the top well",
        )
        button_pressed = ctx.part_world_position(button)

    ctx.check(
        "release button depresses downward into the housing",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
