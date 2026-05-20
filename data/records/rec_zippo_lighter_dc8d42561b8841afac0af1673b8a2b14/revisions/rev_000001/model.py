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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CASE_W = 0.0375
CASE_D = 0.0128
BODY_H = 0.0415
LID_H = 0.0160
SHELL_T = 0.0007
LID_CLEAR_X = 0.00025
LID_CLEAR_Y = 0.00020
LID_W = CASE_W + 2.0 * (SHELL_T + LID_CLEAR_X)
LID_D = CASE_D + 2.0 * (SHELL_T + LID_CLEAR_Y)
OUTER_FILLET = 0.0012

HINGE_PIN_R = 0.00125
HINGE_SLEEVE_R = 0.00165
HINGE_Y_LEN = CASE_D * 0.80
HINGE_Z = 0.0418
AXIS_FROM_LID_BOTTOM = 0.0018
LID_CENTER_X = -(LID_W / 2.0 + 0.00040)
LID_CENTER_Z = (LID_H / 2.0) - AXIS_FROM_LID_BOTTOM

INSERT_W = 0.0348
INSERT_D = 0.0104
INSERT_H = 0.0310
INSERT_DECK_H = 0.0012
CHIMNEY_W = 0.0145
CHIMNEY_D = 0.0080
CHIMNEY_H = 0.0195
CHIMNEY_T = 0.00055

WHEEL_X = 0.0034
WHEEL_Z = INSERT_H + INSERT_DECK_H + 0.0065
WHEEL_AXLE_LEN = CHIMNEY_D - (2.0 * CHIMNEY_T) - 0.0004


def _guard_mesh(name: str):
    return mesh_from_geometry(
        PerforatedPanelGeometry(
            (CHIMNEY_W, CHIMNEY_H),
            CHIMNEY_T,
            hole_diameter=0.0019,
            pitch=(0.0060, 0.0045),
            frame=0.0015,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_lighter")

    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.34, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    wick_fiber = model.material("wick_fiber", rgba=(0.84, 0.80, 0.68, 1.0))
    brass = model.material("brass", rgba=(0.68, 0.58, 0.31, 1.0))

    case = model.part("case")
    case_wall_h = BODY_H - SHELL_T
    case_wall_z = SHELL_T + (case_wall_h / 2.0)
    case.visual(
        Box((CASE_W, CASE_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_T / 2.0)),
        material=chrome,
        name="case_floor",
    )
    case.visual(
        Box((CASE_W, SHELL_T, case_wall_h)),
        origin=Origin(xyz=(0.0, (CASE_D - SHELL_T) / 2.0, case_wall_z)),
        material=chrome,
        name="front_wall",
    )
    case.visual(
        Box((CASE_W, SHELL_T, case_wall_h)),
        origin=Origin(xyz=(0.0, -(CASE_D - SHELL_T) / 2.0, case_wall_z)),
        material=chrome,
        name="rear_wall",
    )
    case.visual(
        Box((SHELL_T, CASE_D - (2.0 * SHELL_T), case_wall_h)),
        origin=Origin(xyz=(-(CASE_W - SHELL_T) / 2.0, 0.0, case_wall_z)),
        material=chrome,
        name="left_wall",
    )
    case.visual(
        Box((SHELL_T, CASE_D - (2.0 * SHELL_T), case_wall_h)),
        origin=Origin(xyz=((CASE_W - SHELL_T) / 2.0, 0.0, case_wall_z)),
        material=chrome,
        name="right_wall",
    )
    case.visual(
        Box((0.0014, HINGE_Y_LEN, 0.0080)),
        origin=Origin(xyz=(CASE_W / 2.0 + 0.0007, 0.0, HINGE_Z - 0.0036)),
        material=chrome,
        name="hinge_leaf",
    )
    case.inertial = Inertial.from_geometry(
        Box((CASE_W + 0.004, CASE_D + 0.004, BODY_H)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    insert = model.part("insert")
    insert.visual(
        Box((INSERT_W, INSERT_D, INSERT_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_H / 2.0)),
        material=brushed_steel,
        name="insert_body",
    )
    insert.visual(
        Box((INSERT_W - 0.0015, INSERT_D - 0.0004, INSERT_DECK_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_H + INSERT_DECK_H / 2.0)),
        material=brushed_steel,
        name="insert_deck",
    )
    insert.visual(
        _guard_mesh("lighter_front_guard"),
        origin=Origin(
            xyz=(0.0, (CHIMNEY_D - CHIMNEY_T) / 2.0, INSERT_H + INSERT_DECK_H + CHIMNEY_H / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="front_guard",
    )
    insert.visual(
        _guard_mesh("lighter_rear_guard"),
        origin=Origin(
            xyz=(0.0, -(CHIMNEY_D - CHIMNEY_T) / 2.0, INSERT_H + INSERT_DECK_H + CHIMNEY_H / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="rear_guard",
    )
    insert.visual(
        Box((CHIMNEY_T, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(
            xyz=(-(CHIMNEY_W - CHIMNEY_T) / 2.0, 0.0, INSERT_H + INSERT_DECK_H + CHIMNEY_H / 2.0)
        ),
        material=dark_steel,
        name="left_guard_spine",
    )
    insert.visual(
        Box((CHIMNEY_T, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(
            xyz=((CHIMNEY_W - CHIMNEY_T) / 2.0, 0.0, INSERT_H + INSERT_DECK_H + CHIMNEY_H / 2.0)
        ),
        material=dark_steel,
        name="right_guard_spine",
    )
    insert.visual(
        Box((CHIMNEY_W, CHIMNEY_D, CHIMNEY_T)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_H + INSERT_DECK_H + CHIMNEY_T / 2.0)),
        material=dark_steel,
        name="chimney_base",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.0125),
        origin=Origin(xyz=(-0.0015, 0.0, INSERT_H + INSERT_DECK_H + 0.0063)),
        material=wick_fiber,
        name="wick",
    )
    insert.visual(
        Cylinder(radius=0.0010, length=0.0100),
        origin=Origin(xyz=(WHEEL_X, 0.0, INSERT_H + 0.0050)),
        material=brass,
        name="flint_tube",
    )
    insert.inertial = Inertial.from_geometry(
        Box((INSERT_W, INSERT_D, INSERT_H + CHIMNEY_H)),
        mass=0.028,
        origin=Origin(xyz=(0.0, 0.0, (INSERT_H + CHIMNEY_H) / 2.0)),
    )

    lid = model.part("lid")
    lid_wall_h = LID_H - SHELL_T
    lid_wall_z = LID_CENTER_Z - (SHELL_T / 2.0)
    lid.visual(
        Box((LID_W, LID_D, SHELL_T)),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, LID_CENTER_Z + LID_H / 2.0 - SHELL_T / 2.0)),
        material=chrome,
        name="lid_top",
    )
    lid.visual(
        Box((LID_W, SHELL_T, lid_wall_h)),
        origin=Origin(xyz=(LID_CENTER_X, (LID_D - SHELL_T) / 2.0, lid_wall_z)),
        material=chrome,
        name="lid_front",
    )
    lid.visual(
        Box((LID_W, SHELL_T, lid_wall_h)),
        origin=Origin(xyz=(LID_CENTER_X, -(LID_D - SHELL_T) / 2.0, lid_wall_z)),
        material=chrome,
        name="lid_rear",
    )
    lid.visual(
        Box((SHELL_T, LID_D - (2.0 * SHELL_T), lid_wall_h)),
        origin=Origin(xyz=(LID_CENTER_X - (LID_W - SHELL_T) / 2.0, 0.0, lid_wall_z)),
        material=chrome,
        name="lid_left",
    )
    lid.visual(
        Box((SHELL_T, LID_D - (2.0 * SHELL_T), lid_wall_h)),
        origin=Origin(xyz=(LID_CENTER_X + (LID_W - SHELL_T) / 2.0, 0.0, lid_wall_z)),
        material=chrome,
        name="lid_right",
    )
    lid.visual(
        Cylinder(radius=HINGE_SLEEVE_R, length=HINGE_Y_LEN - 0.0005),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.0017, HINGE_Y_LEN - 0.0009, 0.0064)),
        origin=Origin(xyz=(-0.0010, 0.0, 0.0018)),
        material=chrome,
        name="hinge_web",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_H)),
        mass=0.015,
        origin=Origin(xyz=(-(LID_W / 2.0), 0.0, LID_H / 2.0)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.00048, length=WHEEL_AXLE_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="wheel_axle",
    )
    wheel.visual(
        Cylinder(radius=0.0032, length=0.0016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_steel,
        name="striker_wheel",
    )
    wheel.visual(
        Cylinder(radius=0.0027, length=0.00045),
        origin=Origin(xyz=(0.0, -0.00115, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_flange_0",
    )
    wheel.visual(
        Cylinder(radius=0.0027, length=0.00045),
        origin=Origin(xyz=(0.0, 0.00115, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_flange_1",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0032, length=CHIMNEY_D),
        mass=0.002,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, SHELL_T)),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(CASE_W / 2.0 + HINGE_PIN_R, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("wheel")
    lid_hinge = object_model.get_articulation("case_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            case,
            axes="xy",
            min_overlap=0.010,
            name="closed lid covers the case footprint",
        )
        ctx.expect_within(
            wheel,
            insert,
            axes="x",
            inner_elem="striker_wheel",
            outer_elem="insert_body",
            margin=0.0008,
            name="striker wheel stays inside the insert width",
        )
        ctx.expect_gap(
            insert,
            wheel,
            axis="y",
            positive_elem="front_guard",
            negative_elem="striker_wheel",
            min_gap=0.0015,
            name="front chimney guard clears the striker wheel",
        )
        ctx.expect_gap(
            wheel,
            insert,
            axis="y",
            positive_elem="striker_wheel",
            negative_elem="rear_guard",
            min_gap=0.0015,
            name="rear chimney guard clears the striker wheel",
        )

        wheel_aabb = ctx.part_element_world_aabb(wheel, elem="striker_wheel")
        wick_aabb = ctx.part_element_world_aabb(insert, elem="wick")
        ctx.check(
            "striker wheel sits beside the wick",
            wheel_aabb is not None
            and wick_aabb is not None
            and wheel_aabb[0][0] > wick_aabb[1][0] + 0.0004,
            details=f"wheel_aabb={wheel_aabb}, wick_aabb={wick_aabb}",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    limits = lid_hinge.motion_limits
    open_lid_aabb = None
    if limits is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.upper}):
            ctx.expect_overlap(
                lid,
                case,
                axes="y",
                min_overlap=0.008,
                name="open lid stays aligned on the side hinge axis",
            )
            open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward above the chimney",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
