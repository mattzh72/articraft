from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

HOOD_W = 0.76
HOOD_D = 0.30
HOOD_H = 0.08
SHELL_T = 0.006
BOTTOM_T = 0.004

INTAKE_W = 0.60
INTAKE_D = 0.20
WELL_T = 0.010
DIVIDER_W = 0.012

FILTER_T = 0.005
FILTER_DROP = math.radians(82.0)
SCREEN_W = ((INTAKE_W - DIVIDER_W) * 0.5) - 0.008
SCREEN_D = INTAKE_D - 0.014

KNOB_D = 0.032
KNOB_H = 0.018
KNOB_SHAFT_L = 0.004
KNOB_X_OFFSET = 0.12
KNOB_Y = HOOD_D * 0.5 - 0.038


def _joint_type_name(joint) -> str:
    return getattr(joint.articulation_type, "name", str(joint.articulation_type)).upper()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="insert_range_hood")

    housing_finish = model.material("housing_finish", rgba=(0.86, 0.87, 0.88, 1.0))
    intake_finish = model.material("intake_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    filter_finish = model.material("filter_finish", rgba=(0.58, 0.61, 0.64, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    hood = model.part("hood")

    hood.visual(
        Box((HOOD_W, HOOD_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, HOOD_H - (SHELL_T * 0.5))),
        material=housing_finish,
        name="top_panel",
    )

    hood.visual(
        Box((HOOD_W, SHELL_T, HOOD_H)),
        origin=Origin(xyz=(0.0, HOOD_D * 0.5 - (SHELL_T * 0.5), HOOD_H * 0.5)),
        material=housing_finish,
        name="front_fascia",
    )
    hood.visual(
        Box((HOOD_W, SHELL_T, HOOD_H)),
        origin=Origin(xyz=(0.0, -HOOD_D * 0.5 + (SHELL_T * 0.5), HOOD_H * 0.5)),
        material=housing_finish,
        name="rear_fascia",
    )
    hood.visual(
        Box((SHELL_T, HOOD_D - (2.0 * SHELL_T), HOOD_H)),
        origin=Origin(xyz=(-HOOD_W * 0.5 + (SHELL_T * 0.5), 0.0, HOOD_H * 0.5)),
        material=housing_finish,
        name="side_0",
    )
    hood.visual(
        Box((SHELL_T, HOOD_D - (2.0 * SHELL_T), HOOD_H)),
        origin=Origin(xyz=(HOOD_W * 0.5 - (SHELL_T * 0.5), 0.0, HOOD_H * 0.5)),
        material=housing_finish,
        name="side_1",
    )

    front_strip_depth = (HOOD_D * 0.5 - SHELL_T) - (INTAKE_D * 0.5 + WELL_T)
    side_strip_width = (HOOD_W * 0.5 - SHELL_T) - (INTAKE_W * 0.5 + WELL_T)
    strip_center_y = ((HOOD_D * 0.5 - SHELL_T) + (INTAKE_D * 0.5 + WELL_T)) * 0.5
    strip_center_x = ((HOOD_W * 0.5 - SHELL_T) + (INTAKE_W * 0.5 + WELL_T)) * 0.5

    hood.visual(
        Box((HOOD_W, front_strip_depth, BOTTOM_T)),
        origin=Origin(xyz=(0.0, strip_center_y, BOTTOM_T * 0.5)),
        material=housing_finish,
        name="underside_front",
    )
    hood.visual(
        Box((HOOD_W, front_strip_depth, BOTTOM_T)),
        origin=Origin(xyz=(0.0, -strip_center_y, BOTTOM_T * 0.5)),
        material=housing_finish,
        name="underside_rear",
    )
    hood.visual(
        Box((side_strip_width, INTAKE_D + (2.0 * WELL_T), BOTTOM_T)),
        origin=Origin(xyz=(-strip_center_x, 0.0, BOTTOM_T * 0.5)),
        material=housing_finish,
        name="underside_side_0",
    )
    hood.visual(
        Box((side_strip_width, INTAKE_D + (2.0 * WELL_T), BOTTOM_T)),
        origin=Origin(xyz=(strip_center_x, 0.0, BOTTOM_T * 0.5)),
        material=housing_finish,
        name="underside_side_1",
    )

    hood.visual(
        Box((INTAKE_W + (2.0 * WELL_T), WELL_T, HOOD_H)),
        origin=Origin(
            xyz=(
                0.0,
                (INTAKE_D * 0.5) + (WELL_T * 0.5),
                HOOD_H * 0.5,
            )
        ),
        material=intake_finish,
        name="well_front",
    )
    hood.visual(
        Box((INTAKE_W + (2.0 * WELL_T), WELL_T, HOOD_H)),
        origin=Origin(
            xyz=(
                0.0,
                -(INTAKE_D * 0.5) - (WELL_T * 0.5),
                HOOD_H * 0.5,
            )
        ),
        material=intake_finish,
        name="well_rear",
    )
    hood.visual(
        Box((WELL_T, INTAKE_D, HOOD_H)),
        origin=Origin(
            xyz=(
                -(INTAKE_W * 0.5) - (WELL_T * 0.5),
                0.0,
                HOOD_H * 0.5,
            )
        ),
        material=intake_finish,
        name="well_side_0",
    )
    hood.visual(
        Box((WELL_T, INTAKE_D, HOOD_H)),
        origin=Origin(
            xyz=(
                (INTAKE_W * 0.5) + (WELL_T * 0.5),
                0.0,
                HOOD_H * 0.5,
            )
        ),
        material=intake_finish,
        name="well_side_1",
    )
    hood.visual(
        Box((DIVIDER_W, INTAKE_D, HOOD_H)),
        origin=Origin(xyz=(0.0, 0.0, HOOD_H * 0.5)),
        material=intake_finish,
        name="divider",
    )

    filter_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (SCREEN_W, SCREEN_D),
            FILTER_T,
            slot_size=(0.024, 0.005),
            pitch=(0.034, 0.016),
            frame=0.010,
            corner_radius=0.004,
            slot_angle_deg=18.0,
            stagger=True,
        ),
        "range_hood_filter_screen",
    )

    bay_center_x = (INTAKE_W + DIVIDER_W) * 0.25
    for index, x_sign in enumerate((-1.0, 1.0)):
        hood.visual(
            Box((SCREEN_W, 0.008, 0.004)),
            origin=Origin(
                xyz=(
                    x_sign * bay_center_x,
                    -(INTAKE_D * 0.5) + 0.004,
                    -0.002,
                )
            ),
            material=intake_finish,
            name=f"hinge_rail_{index}",
        )

    for index, x_sign in enumerate((-1.0, 1.0)):
        filter_part = model.part(f"filter_{index}")
        filter_part.visual(
            filter_mesh,
            origin=Origin(xyz=(0.0, SCREEN_D * 0.5, -(FILTER_T * 0.5))),
            material=filter_finish,
            name="screen",
        )
        filter_part.visual(
            Cylinder(radius=0.004, length=SCREEN_W * 0.92),
            origin=Origin(
                xyz=(0.0, 0.004, -0.004),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=intake_finish,
            name="hinge_barrel",
        )
        filter_part.visual(
            Box((0.040, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, SCREEN_D - 0.004, -0.006)),
            material=intake_finish,
            name="pull_tab",
        )

        model.articulation(
            f"hood_to_filter_{index}",
            ArticulationType.REVOLUTE,
            parent=hood,
            child=filter_part,
            origin=Origin(
                xyz=(
                    x_sign * bay_center_x,
                    -(INTAKE_D * 0.5),
                    0.0,
                )
            ),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.8,
                lower=0.0,
                upper=FILTER_DROP,
            ),
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_D,
            KNOB_H,
            body_style="skirted",
            top_diameter=0.026,
            skirt=KnobSkirt(0.036, 0.004, flare=0.08),
            grip=KnobGrip(style="fluted", count=16, depth=0.0010),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0006,
                angle_deg=0.0,
            ),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "range_hood_knob",
    )

    for index, x_sign in enumerate((-1.0, 1.0)):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0045, length=KNOB_SHAFT_L),
            origin=Origin(xyz=(0.0, 0.0, -(KNOB_SHAFT_L * 0.5))),
            material=knob_finish,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(
                xyz=(0.0, 0.0, -KNOB_SHAFT_L),
                rpy=(math.pi, 0.0, 0.0),
            ),
            material=knob_finish,
            name="knob_cap",
        )

        model.articulation(
            f"hood_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(xyz=(x_sign * KNOB_X_OFFSET, KNOB_Y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=8.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    filter_0 = object_model.get_part("filter_0")
    filter_1 = object_model.get_part("filter_1")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")

    filter_joint_0 = object_model.get_articulation("hood_to_filter_0")
    filter_joint_1 = object_model.get_articulation("hood_to_filter_1")
    knob_joint_0 = object_model.get_articulation("hood_to_knob_0")
    knob_joint_1 = object_model.get_articulation("hood_to_knob_1")

    hood_aabb = ctx.part_world_aabb(hood)
    hood_size = None
    if hood_aabb is not None:
        mins, maxs = hood_aabb
        hood_size = (
            float(maxs[0] - mins[0]),
            float(maxs[1] - mins[1]),
            float(maxs[2] - mins[2]),
        )
    ctx.check(
        "hood_scale_reads_as_cabinet_insert",
        hood_size is not None
        and 0.72 <= hood_size[0] <= 0.80
        and 0.28 <= hood_size[1] <= 0.34
        and 0.075 <= hood_size[2] <= 0.085,
        details=f"size={hood_size!r}",
    )

    for joint in (knob_joint_0, knob_joint_1):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_continuous",
            _joint_type_name(joint) == "CONTINUOUS"
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type!r}, limits={limits!r}",
        )

    for joint in (filter_joint_0, filter_joint_1):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_opens_downward_like_filter_screen",
            _joint_type_name(joint) == "REVOLUTE"
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) < 1e-6
            and 1.25 <= limits.upper <= 1.50,
            details=f"type={joint.articulation_type!r}, limits={limits!r}",
        )

    ctx.expect_gap(
        hood,
        filter_0,
        axis="z",
        positive_elem="underside_rear",
        negative_elem="screen",
        max_gap=0.001,
        max_penetration=0.0,
        name="filter_0_sits_flush_in_closed_position",
    )
    ctx.expect_gap(
        hood,
        filter_1,
        axis="z",
        positive_elem="underside_rear",
        negative_elem="screen",
        max_gap=0.001,
        max_penetration=0.0,
        name="filter_1_sits_flush_in_closed_position",
    )
    ctx.expect_gap(
        hood,
        knob_0,
        axis="z",
        positive_elem="underside_front",
        negative_elem="shaft",
        max_gap=0.0005,
        max_penetration=0.0,
        name="knob_0_mounts_to_hood_underside",
    )
    ctx.expect_gap(
        hood,
        knob_1,
        axis="z",
        positive_elem="underside_front",
        negative_elem="shaft",
        max_gap=0.0005,
        max_penetration=0.0,
        name="knob_1_mounts_to_hood_underside",
    )

    rest_filter_0 = ctx.part_element_world_aabb(filter_0, elem="screen")
    rest_filter_1 = ctx.part_element_world_aabb(filter_1, elem="screen")
    with ctx.pose({filter_joint_0: 1.20, filter_joint_1: 1.20}):
        open_filter_0 = ctx.part_element_world_aabb(filter_0, elem="screen")
        open_filter_1 = ctx.part_element_world_aabb(filter_1, elem="screen")

    ctx.check(
        "filter_0_swings_down_below_intake",
        rest_filter_0 is not None
        and open_filter_0 is not None
        and float(open_filter_0[0][2]) < float(rest_filter_0[0][2]) - 0.08,
        details=f"rest={rest_filter_0!r}, open={open_filter_0!r}",
    )
    ctx.check(
        "filter_1_swings_down_below_intake",
        rest_filter_1 is not None
        and open_filter_1 is not None
        and float(open_filter_1[0][2]) < float(rest_filter_1[0][2]) - 0.08,
        details=f"rest={rest_filter_1!r}, open={open_filter_1!r}",
    )

    return ctx.report()


object_model = build_object_model()
