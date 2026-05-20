from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SURFACE_Z = 0.052
FADER_TRAVEL = 0.045


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="all_in_one_dj_controller")

    body = model.material("satin_black_body", rgba=(0.055, 0.057, 0.062, 1.0))
    deck_panel = model.material("dark_deck_panels", rgba=(0.085, 0.088, 0.096, 1.0))
    mixer_panel = model.material("mixer_graphite", rgba=(0.12, 0.125, 0.135, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.018, 0.019, 0.021, 1.0))
    jog_metal = model.material("brushed_jog_aluminum", rgba=(0.58, 0.60, 0.61, 1.0))
    jog_mark = model.material("white_index_marks", rgba=(0.88, 0.90, 0.92, 1.0))
    slot_dark = model.material("slot_shadow_black", rgba=(0.0, 0.0, 0.0, 1.0))
    fader_silver = model.material("fader_cap_silver", rgba=(0.72, 0.73, 0.73, 1.0))
    trim = model.material("soft_grey_trim", rgba=(0.30, 0.31, 0.33, 1.0))
    led_blue = model.material("dim_blue_labels", rgba=(0.12, 0.36, 0.70, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.86, 0.34, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=body,
        name="main_housing",
    )
    chassis.visual(
        Box((0.84, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.159, 0.052)),
        material=trim,
        name="front_lip",
    )
    chassis.visual(
        Box((0.84, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.158, 0.051)),
        material=trim,
        name="rear_lip",
    )
    chassis.visual(
        Box((0.016, 0.32, 0.010)),
        origin=Origin(xyz=(-0.421, 0.0, 0.052)),
        material=trim,
        name="side_cheek_0",
    )
    chassis.visual(
        Box((0.016, 0.32, 0.010)),
        origin=Origin(xyz=(0.421, 0.0, 0.052)),
        material=trim,
        name="side_cheek_1",
    )

    for index, x in enumerate((-0.275, 0.275)):
        chassis.visual(
            Box((0.285, 0.292, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.050)),
            material=deck_panel,
            name=f"deck_panel_{index}",
        )
        chassis.visual(
            Cylinder(radius=0.107, length=0.004),
            origin=Origin(xyz=(x, 0.020, SURFACE_Z - 0.002)),
            material=slot_dark,
            name=f"jog_well_{index}",
        )
        chassis.visual(
            Box((0.105, 0.008, 0.001)),
            origin=Origin(xyz=(x, -0.124, SURFACE_Z - 0.0005)),
            material=led_blue,
            name=f"deck_label_{index}",
        )
        chassis.visual(
            Box((0.008, 0.105, 0.001)),
            origin=Origin(xyz=(x - 0.124, 0.0, SURFACE_Z - 0.0005)),
            material=trim,
            name=f"deck_side_rule_{index}",
        )

    chassis.visual(
        Box((0.205, 0.304, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=mixer_panel,
        name="mixer_panel",
    )
    chassis.visual(
        Box((0.006, 0.315, 0.008)),
        origin=Origin(xyz=(-0.115, 0.0, 0.052)),
        material=trim,
        name="mixer_divider_0",
    )
    chassis.visual(
        Box((0.006, 0.315, 0.008)),
        origin=Origin(xyz=(0.115, 0.0, 0.052)),
        material=trim,
        name="mixer_divider_1",
    )
    chassis.visual(
        Box((0.088, 0.026, 0.003)),
        origin=Origin(xyz=(0.0, 0.138, SURFACE_Z - 0.0015)),
        material=slot_dark,
        name="display_window",
    )

    for index, x in enumerate((-0.045, 0.045)):
        chassis.visual(
            Box((0.018, 0.150, 0.001)),
            origin=Origin(xyz=(x, -0.045, SURFACE_Z - 0.0005)),
            material=slot_dark,
            name=f"slot_{index}",
        )
        for tick_index, y in enumerate((-0.105, -0.075, -0.045, -0.015, 0.015)):
            chassis.visual(
                Box((0.020, 0.002, 0.001)),
                origin=Origin(xyz=(x + 0.022, y, SURFACE_Z - 0.0005)),
                material=trim,
                name=f"slot_{index}_tick_{tick_index}",
            )

    for index, x in enumerate((-0.275, 0.275)):
        jog = model.part(f"jog_{index}")
        jog.visual(
            Cylinder(radius=0.094, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=rubber,
            name="outer_platter",
        )
        jog.visual(
            Cylinder(radius=0.070, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=jog_metal,
            name="brushed_disk",
        )
        jog.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=rubber,
            name="center_hub",
        )
        jog.visual(
            Box((0.012, 0.032, 0.0012)),
            origin=Origin(xyz=(0.0, 0.048, 0.0194)),
            material=jog_mark,
            name="index_mark",
        )
        model.articulation(
            f"chassis_to_jog_{index}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=jog,
            origin=Origin(xyz=(x, 0.020, SURFACE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=18.0),
        )

    for index, x in enumerate((-0.045, 0.045)):
        fader = model.part(f"fader_{index}")
        fader.visual(
            Box((0.042, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=fader_silver,
            name="fader_cap",
        )
        fader.visual(
            Box((0.030, 0.004, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0144)),
            material=slot_dark,
            name="grip_line",
        )
        model.articulation(
            f"chassis_to_fader_{index}",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=fader,
            origin=Origin(xyz=(x, -0.045, SURFACE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.18,
                lower=-FADER_TRAVEL,
                upper=FADER_TRAVEL,
            ),
        )

    for row, y in enumerate((0.052, 0.092, 0.132)):
        for column, x in enumerate((-0.056, 0.056)):
            knob = model.part(f"knob_{row}_{column}")
            knob.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(0.0, 0.0, 0.006)),
                material=rubber,
                name="knob_cap",
            )
            knob.visual(
                Box((0.004, 0.017, 0.0012)),
                origin=Origin(xyz=(0.0, 0.002, 0.0124)),
                material=jog_mark,
                name="pointer_line",
            )
            model.articulation(
                f"chassis_to_knob_{row}_{column}",
                ArticulationType.REVOLUTE,
                parent=chassis,
                child=knob,
                origin=Origin(xyz=(x, y, SURFACE_Z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(
                    effort=0.18,
                    velocity=5.0,
                    lower=-2.35,
                    upper=2.35,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")

    for index in (0, 1):
        jog = object_model.get_part(f"jog_{index}")
        jog_joint = object_model.get_articulation(f"chassis_to_jog_{index}")
        ctx.check(
            f"jog_{index}_continuous",
            getattr(jog_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
            details=f"joint_type={getattr(jog_joint, 'articulation_type', None)!r}",
        )
        ctx.expect_gap(
            jog,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="outer_platter",
            negative_elem=f"jog_well_{index}",
            name=f"jog_{index}_sits_on_recess",
        )

    jog_0 = object_model.get_part("jog_0")
    jog_0_joint = object_model.get_articulation("chassis_to_jog_0")
    mark_rest = ctx.part_element_world_aabb(jog_0, elem="index_mark")
    with ctx.pose({jog_0_joint: math.pi / 2.0}):
        mark_rotated = ctx.part_element_world_aabb(jog_0, elem="index_mark")
    if mark_rest is not None and mark_rotated is not None:
        rest_center = tuple((mark_rest[0][i] + mark_rest[1][i]) * 0.5 for i in range(3))
        rotated_center = tuple((mark_rotated[0][i] + mark_rotated[1][i]) * 0.5 for i in range(3))
        moved = (
            abs(rotated_center[0] - rest_center[0]) > 0.035
            or abs(rotated_center[1] - rest_center[1]) > 0.035
        )
    else:
        rest_center = None
        rotated_center = None
        moved = False
    ctx.check(
        "jog_index_mark_rotates",
        moved,
        details=f"rest={rest_center}, rotated={rotated_center}",
    )

    for index in (0, 1):
        fader = object_model.get_part(f"fader_{index}")
        fader_joint = object_model.get_articulation(f"chassis_to_fader_{index}")
        ctx.check(
            f"fader_{index}_prismatic",
            getattr(fader_joint, "articulation_type", None) == ArticulationType.PRISMATIC,
            details=f"joint_type={getattr(fader_joint, 'articulation_type', None)!r}",
        )
        ctx.expect_gap(
            fader,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="fader_cap",
            negative_elem=f"slot_{index}",
            name=f"fader_{index}_rides_above_slot",
        )
        ctx.expect_within(
            fader,
            chassis,
            axes="y",
            inner_elem="fader_cap",
            outer_elem=f"slot_{index}",
            name=f"fader_{index}_centered_in_slot",
        )
        rest_position = ctx.part_world_position(fader)
        with ctx.pose({fader_joint: FADER_TRAVEL}):
            ctx.expect_within(
                fader,
                chassis,
                axes="y",
                inner_elem="fader_cap",
                outer_elem=f"slot_{index}",
                name=f"fader_{index}_upper_travel_stays_in_slot",
            )
            upper_position = ctx.part_world_position(fader)
        with ctx.pose({fader_joint: -FADER_TRAVEL}):
            ctx.expect_within(
                fader,
                chassis,
                axes="y",
                inner_elem="fader_cap",
                outer_elem=f"slot_{index}",
                name=f"fader_{index}_lower_travel_stays_in_slot",
            )
        ctx.check(
            f"fader_{index}_slides_along_slot",
            rest_position is not None
            and upper_position is not None
            and upper_position[1] > rest_position[1] + FADER_TRAVEL * 0.9,
            details=f"rest={rest_position}, upper={upper_position}",
        )

    return ctx.report()


object_model = build_object_model()
