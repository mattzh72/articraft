from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.0372
CASE_D = 0.0124
CASE_H = 0.0410
CASE_WALL = 0.0010

LID_W = 0.0380
LID_D = 0.0130
LID_H = 0.0180
LID_WALL = 0.0009
LID_OVERLAP = 0.0045

HINGE_R = 0.00115
HINGE_LEN = 0.0047

INSERT_W = 0.0346
INSERT_D = 0.0094
INSERT_H = 0.0375
INSERT_WALL = 0.0007
INSERT_FLOOR_Z = CASE_WALL

CHIMNEY_W = 0.0160
CHIMNEY_D = 0.0082
CHIMNEY_H = 0.0190
CHIMNEY_BASE_Z = 0.0240
CHIMNEY_X = -0.0045

WHEEL_R = 0.0031
WHEEL_LEN = 0.0060
WHEEL_X = 0.0069
WHEEL_Z = 0.0382
EAR_T = 0.0011
EAR_H = 0.0070
EAR_CLEAR = 0.0
AXLE_R = 0.00042
WICK_R = 0.00085
WICK_LEN = 0.0195


def _striker_wheel_shape():
    rim = cq.Workplane("XZ").polygon(18, WHEEL_R * 2.04).extrude((WHEEL_LEN - 0.0008) / 2.0, both=True)
    hub = cq.Workplane("XZ").circle(WHEEL_R * 0.72).extrude(WHEEL_LEN / 2.0, both=True)
    bore = cq.Workplane("XZ").circle(AXLE_R + 0.00012).extrude(WHEEL_LEN / 2.0 + 0.0003, both=True)
    return rim.union(hub).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_top_lighter")

    chrome = model.material("chrome", rgba=(0.76, 0.78, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.35, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.60, 0.29, 1.0))
    wick_fiber = model.material("wick_fiber", rgba=(0.88, 0.82, 0.70, 1.0))

    outer_case = model.part("outer_case")
    outer_case.visual(Box((CASE_W, CASE_D, CASE_WALL)), origin=Origin(xyz=(0.0, 0.0, CASE_WALL / 2.0)), material=chrome, name="case_floor")
    outer_case.visual(
        Box((CASE_WALL, CASE_D, CASE_H - CASE_WALL)),
        origin=Origin(xyz=(-(CASE_W - CASE_WALL) / 2.0, 0.0, CASE_WALL + (CASE_H - CASE_WALL) / 2.0)),
        material=chrome,
        name="case_side_0",
    )
    outer_case.visual(
        Box((CASE_WALL, CASE_D, CASE_H - CASE_WALL)),
        origin=Origin(xyz=((CASE_W - CASE_WALL) / 2.0, 0.0, CASE_WALL + (CASE_H - CASE_WALL) / 2.0)),
        material=chrome,
        name="case_side_1",
    )
    outer_case.visual(
        Box((CASE_W - 2.0 * CASE_WALL, CASE_WALL, CASE_H - CASE_WALL)),
        origin=Origin(xyz=(0.0, -(CASE_D - CASE_WALL) / 2.0, CASE_WALL + (CASE_H - CASE_WALL) / 2.0)),
        material=chrome,
        name="case_face_0",
    )
    outer_case.visual(
        Box((CASE_W - 2.0 * CASE_WALL, CASE_WALL, CASE_H - CASE_WALL)),
        origin=Origin(xyz=(0.0, (CASE_D - CASE_WALL) / 2.0, CASE_WALL + (CASE_H - CASE_WALL) / 2.0)),
        material=chrome,
        name="case_face_1",
    )
    outer_case.visual(
        Cylinder(radius=HINGE_R, length=HINGE_LEN),
        origin=Origin(xyz=(-CASE_W / 2.0 - 0.00015, 0.0, CASE_H - 0.0135), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="case_knuckle_0",
    )
    outer_case.visual(
        Cylinder(radius=HINGE_R, length=HINGE_LEN),
        origin=Origin(xyz=(-CASE_W / 2.0 - 0.00015, 0.0, CASE_H - 0.0045), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="case_knuckle_1",
    )

    lid = model.part("lid")
    lid_x = 0.00085
    lid_wall_h = LID_H - LID_WALL
    lid.visual(
        Box((LID_W, LID_D, LID_WALL)),
        origin=Origin(xyz=(lid_x + LID_W / 2.0, 0.0, LID_H - LID_OVERLAP - LID_WALL / 2.0)),
        material=chrome,
        name="lid_roof",
    )
    lid.visual(
        Box((LID_WALL, LID_D, lid_wall_h)),
        origin=Origin(xyz=(lid_x + LID_WALL / 2.0, 0.0, -LID_OVERLAP + lid_wall_h / 2.0)),
        material=chrome,
        name="lid_side_0",
    )
    lid.visual(
        Box((LID_WALL, LID_D, lid_wall_h)),
        origin=Origin(xyz=(lid_x + LID_W - LID_WALL / 2.0, 0.0, -LID_OVERLAP + lid_wall_h / 2.0)),
        material=chrome,
        name="lid_side_1",
    )
    lid.visual(
        Box((LID_W - 2.0 * LID_WALL, LID_WALL, lid_wall_h)),
        origin=Origin(xyz=(lid_x + LID_W / 2.0, -(LID_D - LID_WALL) / 2.0, -LID_OVERLAP + lid_wall_h / 2.0)),
        material=chrome,
        name="lid_face_0",
    )
    lid.visual(
        Box((LID_W - 2.0 * LID_WALL, LID_WALL, lid_wall_h)),
        origin=Origin(xyz=(lid_x + LID_W / 2.0, (LID_D - LID_WALL) / 2.0, -LID_OVERLAP + lid_wall_h / 2.0)),
        material=chrome,
        name="lid_face_1",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=HINGE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0055), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lid_knuckle",
    )

    insert = model.part("insert")
    insert.visual(Box((INSERT_W, INSERT_D, INSERT_WALL)), origin=Origin(xyz=(0.0, 0.0, INSERT_WALL / 2.0)), material=steel, name="insert_floor")
    insert.visual(
        Box((INSERT_WALL, INSERT_D, INSERT_H - INSERT_WALL)),
        origin=Origin(xyz=(-(INSERT_W - INSERT_WALL) / 2.0, 0.0, INSERT_WALL + (INSERT_H - INSERT_WALL) / 2.0)),
        material=steel,
        name="insert_side_0",
    )
    insert.visual(
        Box((INSERT_WALL, INSERT_D, INSERT_H - INSERT_WALL)),
        origin=Origin(xyz=((INSERT_W - INSERT_WALL) / 2.0, 0.0, INSERT_WALL + (INSERT_H - INSERT_WALL) / 2.0)),
        material=steel,
        name="insert_side_1",
    )
    insert.visual(
        Box((INSERT_W - 2.0 * INSERT_WALL, INSERT_WALL, INSERT_H - INSERT_WALL)),
        origin=Origin(xyz=(0.0, -(INSERT_D - INSERT_WALL) / 2.0, INSERT_WALL + (INSERT_H - INSERT_WALL) / 2.0)),
        material=steel,
        name="insert_face_0",
    )
    insert.visual(
        Box((INSERT_W - 2.0 * INSERT_WALL, INSERT_WALL, INSERT_H - INSERT_WALL)),
        origin=Origin(xyz=(0.0, (INSERT_D - INSERT_WALL) / 2.0, INSERT_WALL + (INSERT_H - INSERT_WALL) / 2.0)),
        material=steel,
        name="insert_face_1",
    )
    insert.visual(
        Box((CHIMNEY_W, CHIMNEY_D, INSERT_WALL)),
        origin=Origin(xyz=(CHIMNEY_X, 0.0, CHIMNEY_BASE_Z + INSERT_WALL / 2.0)),
        material=steel,
        name="chimney_floor",
    )
    insert.visual(
        Box((INSERT_WALL, CHIMNEY_D, CHIMNEY_H - INSERT_WALL)),
        origin=Origin(
            xyz=(CHIMNEY_X - (CHIMNEY_W - INSERT_WALL) / 2.0, 0.0, CHIMNEY_BASE_Z + INSERT_WALL + (CHIMNEY_H - INSERT_WALL) / 2.0)
        ),
        material=steel,
        name="chimney_side_0",
    )
    insert.visual(
        Box((INSERT_WALL, CHIMNEY_D, CHIMNEY_H - INSERT_WALL)),
        origin=Origin(
            xyz=(CHIMNEY_X + (CHIMNEY_W - INSERT_WALL) / 2.0, 0.0, CHIMNEY_BASE_Z + INSERT_WALL + (CHIMNEY_H - INSERT_WALL) / 2.0)
        ),
        material=steel,
        name="chimney_side_1",
    )
    insert.visual(
        Box((CHIMNEY_W - 2.0 * INSERT_WALL, INSERT_WALL, CHIMNEY_H - INSERT_WALL)),
        origin=Origin(
            xyz=(CHIMNEY_X, -(CHIMNEY_D - INSERT_WALL) / 2.0, CHIMNEY_BASE_Z + INSERT_WALL + (CHIMNEY_H - INSERT_WALL) / 2.0)
        ),
        material=steel,
        name="chimney_face_0",
    )
    insert.visual(
        Box((CHIMNEY_W - 2.0 * INSERT_WALL, INSERT_WALL, CHIMNEY_H - INSERT_WALL)),
        origin=Origin(
            xyz=(CHIMNEY_X, (CHIMNEY_D - INSERT_WALL) / 2.0, CHIMNEY_BASE_Z + INSERT_WALL + (CHIMNEY_H - INSERT_WALL) / 2.0)
        ),
        material=steel,
        name="chimney_face_1",
    )
    insert.visual(
        Cylinder(radius=AXLE_R, length=WHEEL_LEN + 2.0 * EAR_T),
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )
    ear_span_x = (WHEEL_X + WHEEL_R + 0.0008) - (CHIMNEY_X + CHIMNEY_W / 2.0 - 0.0002)
    ear_center_x = (WHEEL_X + WHEEL_R + 0.0008 + CHIMNEY_X + CHIMNEY_W / 2.0 - 0.0002) / 2.0
    ear_center_z = WHEEL_Z - 0.0002
    ear_offset_y = WHEEL_LEN / 2.0 + EAR_CLEAR + EAR_T / 2.0
    insert.visual(
        Box((ear_span_x, EAR_T, EAR_H)),
        origin=Origin(xyz=(ear_center_x, -ear_offset_y, ear_center_z)),
        material=dark_steel,
        name="wheel_ear_0",
    )
    insert.visual(
        Box((ear_span_x, EAR_T, EAR_H)),
        origin=Origin(xyz=(ear_center_x, ear_offset_y, ear_center_z)),
        material=dark_steel,
        name="wheel_ear_1",
    )
    insert.visual(
        Cylinder(radius=WICK_R, length=WICK_LEN),
        origin=Origin(xyz=(CHIMNEY_X - 0.0023, 0.0, CHIMNEY_BASE_Z + INSERT_WALL + WICK_LEN / 2.0), rpy=(0.0, 0.0, 0.08)),
        material=wick_fiber,
        name="wick",
    )

    wheel = model.part("striker_wheel")
    wheel.visual(
        mesh_from_cadquery(_striker_wheel_shape(), "striker_wheel"),
        material=brass,
        name="wheel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_case,
        child=lid,
        origin=Origin(xyz=(-CASE_W / 2.0 - 0.00015, 0.0, CASE_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0, effort=0.3, velocity=6.0),
    )

    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=outer_case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.029, effort=2.0, velocity=0.2),
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
        meta={"qc_samples": [0.0, pi / 3.0, 1.2]},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_case = object_model.get_part("outer_case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("striker_wheel")

    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")

    lid_limits = lid_hinge.motion_limits
    slide_limits = insert_slide.motion_limits

    with ctx.pose({lid_hinge: 0.0, insert_slide: 0.0}):
        ctx.expect_overlap(
            lid,
            outer_case,
            axes="xy",
            min_overlap=0.010,
            name="closed lid covers the case footprint",
        )
        ctx.expect_within(
            insert,
            outer_case,
            axes="xy",
            margin=0.0012,
            name="insert stays laterally nested inside the lower shell",
        )
        ctx.expect_overlap(
            insert,
            outer_case,
            axes="z",
            min_overlap=0.020,
            name="insert remains deeply seated at rest",
        )
        ctx.expect_within(
            wheel,
            insert,
            axes="xy",
            margin=0.0030,
            name="striker wheel stays mounted within the insert footprint",
        )

    closed_lid_aabb = None
    open_lid_aabb = None
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.lower or 0.0}):
            closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid swings upward from the case",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.010
            and open_lid_aabb[0][0] < closed_lid_aabb[0][0] - 0.010,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    rest_insert_pos = None
    extended_insert_pos = None
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({insert_slide: 0.0}):
            rest_insert_pos = ctx.part_world_position(insert)
        with ctx.pose({insert_slide: slide_limits.upper}):
            ctx.expect_within(
                insert,
                outer_case,
                axes="xy",
                margin=0.0012,
                name="raised insert stays centered in the case opening",
            )
            ctx.expect_overlap(
                insert,
                outer_case,
                axes="z",
                min_overlap=0.007,
                name="raised insert remains partially retained in the shell",
            )
            extended_insert_pos = ctx.part_world_position(insert)
        ctx.check(
            "insert lifts upward along the case height",
            rest_insert_pos is not None
            and extended_insert_pos is not None
            and extended_insert_pos[2] > rest_insert_pos[2] + 0.020,
            details=f"rest={rest_insert_pos}, extended={extended_insert_pos}",
        )

    return ctx.report()


object_model = build_object_model()
