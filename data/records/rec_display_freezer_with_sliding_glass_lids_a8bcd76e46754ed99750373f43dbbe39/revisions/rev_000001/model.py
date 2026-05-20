from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_L = 2.10
BODY_W = 0.96
BODY_H = 0.84
PLINTH_H = 0.10

OUTER_WALL_T = 0.035
INNER_WALL_T = 0.030
OPENING_L = 1.80
OPENING_W = 0.74
TOP_FRAME_H = 0.050
TOP_FRAME_Z = BODY_H - TOP_FRAME_H / 2.0
INNER_FLOOR_Z = 0.165
INNER_FLOOR_T = 0.030
WELL_TOP_Z = BODY_H - TOP_FRAME_H
WELL_DEPTH = WELL_TOP_Z - (INNER_FLOOR_Z + INNER_FLOOR_T / 2.0)

DIVIDER_T = 0.045
LID_PANEL_L = 0.54
LID_PANEL_W = 0.72
LID_TRAVEL = 0.12
TRACK_RAIL_L = 0.68
TRACK_RAIL_T = 0.012
TRACK_RAIL_H = 0.014
TRACK_RAIL_Z = BODY_H - 0.013
LID_ORIGIN_Z = TRACK_RAIL_Z + TRACK_RAIL_H / 2.0 + 0.003


def _bay_centers() -> list[float]:
    bay_clear_l = (OPENING_L - 2.0 * DIVIDER_T) / 3.0
    start_x = -OPENING_L / 2.0
    centers: list[float] = []
    cursor = start_x
    for bay_index in range(3):
        centers.append(cursor + bay_clear_l / 2.0)
        cursor += bay_clear_l
        if bay_index < 2:
            cursor += DIVIDER_T
    return centers


def _add_lid_geometry(lid_part, frame_material, glass_material) -> None:
    lid_part.visual(
        Box((LID_PANEL_L - 0.050, LID_PANEL_W - 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=glass_material,
        name="glass",
    )
    lid_part.visual(
        Box((LID_PANEL_L, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, -(LID_PANEL_W / 2.0 - 0.014), 0.011)),
        material=frame_material,
        name="rail_0",
    )
    lid_part.visual(
        Box((LID_PANEL_L, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, LID_PANEL_W / 2.0 - 0.014, 0.011)),
        material=frame_material,
        name="rail_1",
    )
    lid_part.visual(
        Box((0.028, LID_PANEL_W, 0.028)),
        origin=Origin(xyz=(-(LID_PANEL_L / 2.0) + 0.014, 0.0, 0.011)),
        material=frame_material,
        name="end_0",
    )
    lid_part.visual(
        Box((0.028, LID_PANEL_W, 0.028)),
        origin=Origin(xyz=((LID_PANEL_L / 2.0) - 0.014, 0.0, 0.011)),
        material=frame_material,
        name="end_1",
    )
    lid_part.visual(
        Box((0.080, 0.028, 0.014)),
        origin=Origin(xyz=(-(LID_PANEL_L / 2.0) + 0.040, 0.0, 0.021)),
        material=frame_material,
        name="handle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_freezer")

    body_white = model.material("body_white", rgba=(0.90, 0.92, 0.93, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    liner_white = model.material("liner_white", rgba=(0.96, 0.97, 0.98, 1.0))
    plinth_gray = model.material("plinth_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    control_dark = model.material("control_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.84, 0.90, 0.34))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((2.02, 0.90, PLINTH_H)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_H / 2.0)),
        material=plinth_gray,
        name="plinth",
    )

    shell_height = BODY_H - PLINTH_H
    shell_z = PLINTH_H + shell_height / 2.0 - 0.003
    body.visual(
        Box((BODY_L, OUTER_WALL_T, shell_height)),
        origin=Origin(xyz=(0.0, -(BODY_W / 2.0) + OUTER_WALL_T / 2.0, shell_z)),
        material=body_white,
        name="side_0",
    )
    body.visual(
        Box((BODY_L, OUTER_WALL_T, shell_height)),
        origin=Origin(xyz=(0.0, (BODY_W / 2.0) - OUTER_WALL_T / 2.0, shell_z)),
        material=body_white,
        name="side_1",
    )
    body.visual(
        Box((OUTER_WALL_T, BODY_W - 2.0 * OUTER_WALL_T, shell_height)),
        origin=Origin(xyz=(-(BODY_L / 2.0) + OUTER_WALL_T / 2.0, 0.0, shell_z)),
        material=body_white,
        name="control_end",
    )
    body.visual(
        Box((OUTER_WALL_T, BODY_W - 2.0 * OUTER_WALL_T, shell_height)),
        origin=Origin(xyz=((BODY_L / 2.0) - OUTER_WALL_T / 2.0, 0.0, shell_z)),
        material=body_white,
        name="end_1",
    )

    inner_side_y = OPENING_W / 2.0 + INNER_WALL_T / 2.0
    inner_wall_h = WELL_DEPTH + 0.040
    inner_wall_z = 0.150 + inner_wall_h / 2.0
    body.visual(
        Box((OPENING_L + 0.030, INNER_WALL_T, inner_wall_h)),
        origin=Origin(xyz=(0.0, -inner_side_y, inner_wall_z)),
        material=liner_white,
        name="liner_side_0",
    )
    body.visual(
        Box((OPENING_L + 0.030, INNER_WALL_T, inner_wall_h)),
        origin=Origin(xyz=(0.0, inner_side_y, inner_wall_z)),
        material=liner_white,
        name="liner_side_1",
    )
    body.visual(
        Box((INNER_WALL_T, OPENING_W, inner_wall_h)),
        origin=Origin(
            xyz=(-(OPENING_L / 2.0) - INNER_WALL_T / 2.0 + 0.015, 0.0, inner_wall_z)
        ),
        material=liner_white,
        name="liner_end_0",
    )
    body.visual(
        Box((INNER_WALL_T, OPENING_W, inner_wall_h)),
        origin=Origin(
            xyz=((OPENING_L / 2.0) + INNER_WALL_T / 2.0 - 0.015, 0.0, inner_wall_z)
        ),
        material=liner_white,
        name="liner_end_1",
    )
    body.visual(
        Box((OPENING_L + 0.030, OPENING_W, INNER_FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, INNER_FLOOR_Z)),
        material=liner_white,
        name="liner_floor",
    )

    frame_side_w = (BODY_W - OPENING_W) / 2.0
    body.visual(
        Box((OPENING_L, frame_side_w, TOP_FRAME_H)),
        origin=Origin(xyz=(0.0, -(BODY_W / 2.0) + frame_side_w / 2.0, TOP_FRAME_Z)),
        material=trim_silver,
        name="frame_side_0",
    )
    body.visual(
        Box((0.12, frame_side_w, TOP_FRAME_H)),
        origin=Origin(xyz=(-0.84, (BODY_W / 2.0) - frame_side_w / 2.0, TOP_FRAME_Z)),
        material=trim_silver,
        name="frame_side_1_corner",
    )
    body.visual(
        Box((1.56, frame_side_w, TOP_FRAME_H)),
        origin=Origin(xyz=(0.18, (BODY_W / 2.0) - frame_side_w / 2.0, TOP_FRAME_Z)),
        material=trim_silver,
        name="frame_side_1_main",
    )
    end_frame_l = 0.150
    body.visual(
        Box((end_frame_l, OPENING_W + 0.040, TOP_FRAME_H)),
        origin=Origin(xyz=(-(BODY_L / 2.0) + end_frame_l / 2.0, 0.0, TOP_FRAME_Z)),
        material=trim_silver,
        name="frame_end_0",
    )
    body.visual(
        Box((end_frame_l, OPENING_W + 0.040, TOP_FRAME_H)),
        origin=Origin(xyz=((BODY_L / 2.0) - end_frame_l / 2.0, 0.0, TOP_FRAME_Z)),
        material=trim_silver,
        name="frame_end_1",
    )
    for divider_index, divider_x in enumerate((-0.3075, 0.3075)):
        body.visual(
            Box((DIVIDER_T, OPENING_W + 0.040, TOP_FRAME_H)),
            origin=Origin(xyz=(divider_x, 0.0, TOP_FRAME_Z)),
            material=trim_silver,
            name=f"divider_{divider_index}",
        )

    pocket_x = -0.74
    pocket_y = (BODY_W / 2.0) - frame_side_w / 2.0
    pocket_l = 0.18
    pocket_w = 0.060
    pocket_floor_z = BODY_H - 0.048
    body.visual(
        Box((pocket_l, pocket_w, 0.010)),
        origin=Origin(xyz=(pocket_x, pocket_y, pocket_floor_z)),
        material=control_dark,
        name="control_pocket_floor",
    )
    body.visual(
        Box((pocket_l, 0.012, 0.030)),
        origin=Origin(xyz=(pocket_x, pocket_y - (pocket_w / 2.0) + 0.006, BODY_H - 0.039)),
        material=trim_silver,
        name="control_pocket_wall_0",
    )
    body.visual(
        Box((pocket_l, 0.012, 0.030)),
        origin=Origin(xyz=(pocket_x, pocket_y + (pocket_w / 2.0) - 0.006, BODY_H - 0.039)),
        material=trim_silver,
        name="control_pocket_wall_1",
    )
    body.visual(
        Box((0.012, pocket_w, 0.030)),
        origin=Origin(xyz=(pocket_x - (pocket_l / 2.0) + 0.006, pocket_y, BODY_H - 0.039)),
        material=trim_silver,
        name="control_pocket_wall_2",
    )
    body.visual(
        Box((0.012, pocket_w, 0.030)),
        origin=Origin(xyz=(pocket_x + (pocket_l / 2.0) - 0.006, pocket_y, BODY_H - 0.039)),
        material=trim_silver,
        name="control_pocket_wall_3",
    )
    body.visual(
        Box((0.100, 0.028, 0.004)),
        origin=Origin(xyz=(pocket_x, pocket_y, pocket_floor_z + 0.007)),
        material=rail_gray,
        name="control_insert",
    )

    track_y = OPENING_W / 2.0 - 0.018
    for bay_index, bay_x in enumerate(_bay_centers()):
        body.visual(
            Box((TRACK_RAIL_L, TRACK_RAIL_T, TRACK_RAIL_H)),
            origin=Origin(xyz=(bay_x, -track_y, TRACK_RAIL_Z)),
            material=rail_gray,
            name=f"bay_{bay_index}_rail_0",
        )
        body.visual(
            Box((TRACK_RAIL_L, TRACK_RAIL_T, TRACK_RAIL_H)),
            origin=Origin(xyz=(bay_x, track_y, TRACK_RAIL_Z)),
            material=rail_gray,
            name=f"bay_{bay_index}_rail_1",
        )

    for lid_index, bay_x in enumerate(_bay_centers()):
        lid = model.part(f"lid_{lid_index}")
        _add_lid_geometry(lid, trim_silver, glass)
        model.articulation(
            f"body_to_lid_{lid_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=lid,
            origin=Origin(xyz=(bay_x, 0.0, LID_ORIGIN_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0,
                velocity=0.28,
                lower=0.0,
                upper=LID_TRAVEL,
            ),
        )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.022,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=14, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "freezer_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )
    knob.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=rail_gray,
        name="shaft",
    )
    knob.visual(
        Box((0.004, 0.002, 0.012)),
        origin=Origin(xyz=(-0.022, 0.0, 0.010)),
        material=trim_silver,
        name="pointer",
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(-(BODY_L / 2.0), 0.270, 0.530)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lids = [object_model.get_part(f"lid_{index}") for index in range(3)]
    lid_joints = [object_model.get_articulation(f"body_to_lid_{index}") for index in range(3)]
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("body_to_knob")

    ctx.expect_gap(
        lids[1],
        lids[0],
        axis="x",
        min_gap=0.030,
        max_gap=0.090,
        name="lid_0_and_lid_1_stay_distinct",
    )
    ctx.expect_gap(
        lids[2],
        lids[1],
        axis="x",
        min_gap=0.030,
        max_gap=0.090,
        name="lid_1_and_lid_2_stay_distinct",
    )

    for lid_index, (lid, joint) in enumerate(zip(lids, lid_joints)):
        upper = 0.0
        if joint.motion_limits is not None and joint.motion_limits.upper is not None:
            upper = joint.motion_limits.upper

        ctx.expect_contact(
            lid,
            body,
            elem_a="rail_0",
            elem_b=f"bay_{lid_index}_rail_0",
            name=f"lid_{lid_index}_rail_0_contacts_track",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="rail_1",
            elem_b=f"bay_{lid_index}_rail_1",
            name=f"lid_{lid_index}_rail_1_contacts_track",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="rail_0",
            elem_b=f"bay_{lid_index}_rail_0",
            min_overlap=0.010,
            name=f"lid_{lid_index}_rail_0_aligned",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="rail_1",
            elem_b=f"bay_{lid_index}_rail_1",
            min_overlap=0.010,
            name=f"lid_{lid_index}_rail_1_aligned",
        )

        rest_pos = ctx.part_world_position(lid)
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                elem_a="rail_0",
                elem_b=f"bay_{lid_index}_rail_0",
                min_overlap=0.010,
                name=f"lid_{lid_index}_rail_0_retained_when_open",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                elem_a="rail_1",
                elem_b=f"bay_{lid_index}_rail_1",
                min_overlap=0.010,
                name=f"lid_{lid_index}_rail_1_retained_when_open",
            )
            open_pos = ctx.part_world_position(lid)

        ctx.check(
            f"lid_{lid_index}_slides_along_x",
            rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.05,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    ctx.expect_contact(
        body,
        knob,
        elem_a="control_end",
        elem_b="shaft",
        name="knob_shaft_meets_end_wall",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_pointer = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_pointer = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    ctx.check(
        "knob_pointer_rotates_about_shaft",
        rest_pointer is not None
        and turned_pointer is not None
        and (
            abs(turned_pointer[1] - rest_pointer[1]) > 0.006
            or abs(turned_pointer[2] - rest_pointer[2]) > 0.006
        ),
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
