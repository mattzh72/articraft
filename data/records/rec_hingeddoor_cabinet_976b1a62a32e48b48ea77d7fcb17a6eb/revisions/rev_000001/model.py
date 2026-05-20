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


PI = math.pi


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="framed_pantry_cabinet")

    maple = model.material("warm_maple", rgba=(0.72, 0.47, 0.25, 1.0))
    panel_wood = model.material("recessed_panel_wood", rgba=(0.64, 0.39, 0.20, 1.0))
    end_grain = model.material("end_grain_shadow", rgba=(0.46, 0.27, 0.13, 1.0))
    interior = model.material("pale_interior", rgba=(0.82, 0.70, 0.55, 1.0))
    dark = model.material("dark_pin_holes", rgba=(0.025, 0.020, 0.015, 1.0))
    metal = model.material("brushed_nickel", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("shadow_black", rgba=(0.02, 0.018, 0.015, 1.0))

    width = 1.00
    depth = 0.46
    height = 2.05
    side_t = 0.035
    top_t = 0.040
    back_t = 0.022
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    cabinet = model.part("cabinet")

    # Hollow carcass: separate boards that overlap at hidden seams so the
    # cabinet reads as a manufactured pantry case rather than a solid block.
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, height / 2.0)),
        material=interior,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, height / 2.0)),
        material=interior,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, top_t / 2.0)),
        material=interior,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=interior,
        name="top_panel",
    )
    cabinet.visual(
        Box((width, back_t, height)),
        origin=Origin(xyz=(0.0, rear_y - back_t / 2.0, height / 2.0)),
        material=interior,
        name="back_panel",
    )

    # A shallow face frame and plinth give the front opening believable cabinet
    # construction while staying behind the closed door backs.
    frame_depth = 0.028
    frame_y = front_y + frame_depth / 2.0
    stile_w = 0.050
    rail_h = 0.060
    cabinet.visual(
        Box((stile_w, frame_depth, height)),
        origin=Origin(xyz=(-width / 2.0 + stile_w / 2.0, frame_y, height / 2.0)),
        material=maple,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((stile_w, frame_depth, height)),
        origin=Origin(xyz=(width / 2.0 - stile_w / 2.0, frame_y, height / 2.0)),
        material=maple,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((width, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, frame_y, height - rail_h / 2.0)),
        material=maple,
        name="top_face_rail",
    )
    cabinet.visual(
        Box((width, frame_depth, rail_h)),
        origin=Origin(xyz=(0.0, frame_y, rail_h / 2.0)),
        material=maple,
        name="bottom_face_rail",
    )
    cabinet.visual(
        Box((0.86, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, front_y + 0.040, 0.035)),
        material=black,
        name="recessed_plinth",
    )

    # Adjustable shelves are shown at several heights, supported by pins and
    # backed by rows of dark adjustment holes on both side panels.
    shelf_w = width - 2.0 * side_t + 0.006
    shelf_d = depth - back_t - 0.085
    shelf_t = 0.030
    shelf_y = front_y + 0.060 + shelf_d / 2.0
    shelf_zs = (0.47, 0.89, 1.31, 1.56)
    for i, z in enumerate(shelf_zs):
        cabinet.visual(
            Box((shelf_w, shelf_d, shelf_t)),
            origin=Origin(xyz=(0.0, shelf_y, z)),
            material=interior,
            name=f"shelf_{i}",
        )

    hole_zs = [0.32 + i * 0.145 for i in range(11)]
    hole_ys = (front_y + 0.115, rear_y - 0.120)
    left_inner_x = -width / 2.0 + side_t
    right_inner_x = width / 2.0 - side_t
    for side_index, x in enumerate((left_inner_x + 0.0015, right_inner_x - 0.0015)):
        for row_index, y in enumerate(hole_ys):
            for hole_index, z in enumerate(hole_zs):
                cabinet.visual(
                    Cylinder(radius=0.006, length=0.004),
                    origin=Origin(xyz=(x, y, z), rpy=(0.0, PI / 2.0, 0.0)),
                    material=dark,
                    name=f"pin_hole_{side_index}_{row_index}_{hole_index}",
                )

    for shelf_index, z in enumerate(shelf_zs):
        for y_index, y in enumerate((front_y + 0.105, rear_y - 0.135)):
            cabinet.visual(
                Cylinder(radius=0.0065, length=0.038),
                origin=Origin(
                    xyz=(left_inner_x + 0.014, y, z - shelf_t / 2.0 - 0.002),
                    rpy=(0.0, PI / 2.0, 0.0),
                ),
                material=metal,
                name=f"shelf_pin_0_{shelf_index}_{y_index}",
            )
            cabinet.visual(
                Cylinder(radius=0.0065, length=0.038),
                origin=Origin(
                    xyz=(right_inner_x - 0.014, y, z - shelf_t / 2.0 - 0.002),
                    rpy=(0.0, PI / 2.0, 0.0),
                ),
                material=metal,
                name=f"shelf_pin_1_{shelf_index}_{y_index}",
            )

    # Fixed leaves of the concealed hinges, mounted to the cabinet side panels.
    hinge_zs = (0.32, 1.025, 1.73)
    for i, z in enumerate(hinge_zs):
        cabinet.visual(
            Box((0.006, 0.045, 0.095)),
            origin=Origin(xyz=(left_inner_x + 0.003, front_y + 0.092, z)),
            material=metal,
            name=f"hinge_plate_0_{i}",
        )
        cabinet.visual(
            Box((0.006, 0.045, 0.095)),
            origin=Origin(xyz=(right_inner_x - 0.003, front_y + 0.092, z)),
            material=metal,
            name=f"hinge_plate_1_{i}",
        )

    # Door geometry is authored in hinge-line frames.  Door_0 extends along +X;
    # door_1 is mirrored and extends along -X.
    door_t = 0.035
    door_w = 0.484
    door_gap_from_axis = 0.008
    door_bottom = 0.075
    door_top = 1.995
    door_h = door_top - door_bottom
    door_center_z = (door_bottom + door_top) / 2.0
    door_back_y = front_y - 0.006
    hinge_y = door_back_y - door_t / 2.0
    door_stile = 0.075
    door_rail = 0.095
    mid_rail = 0.075
    panel_t = 0.018
    panel_y = 0.005

    def add_framed_door(part_name: str, sign: float) -> None:
        door = model.part(part_name)
        inner_edge = sign * door_gap_from_axis
        meeting_edge = sign * (door_gap_from_axis + door_w)
        center_x = sign * (door_gap_from_axis + door_w / 2.0)
        hinge_stile_x = sign * (door_gap_from_axis + door_stile / 2.0)
        meeting_stile_x = sign * (door_gap_from_axis + door_w - door_stile / 2.0)

        door.visual(
            Box((door_w, door_t, door_h)),
            origin=Origin(xyz=(center_x, 0.0, door_center_z)),
            material=end_grain,
            name="solid_backer",
        )
        door.visual(
            Box((door_stile, door_t + 0.006, door_h)),
            origin=Origin(xyz=(hinge_stile_x, -0.001, door_center_z)),
            material=maple,
            name="hinge_stile",
        )
        door.visual(
            Box((door_stile, door_t + 0.006, door_h)),
            origin=Origin(xyz=(meeting_stile_x, -0.001, door_center_z)),
            material=maple,
            name="meeting_stile",
        )
        door.visual(
            Box((door_w, door_t + 0.006, door_rail)),
            origin=Origin(
                xyz=(center_x, -0.001, door_top - door_rail / 2.0)
            ),
            material=maple,
            name="top_rail",
        )
        door.visual(
            Box((door_w, door_t + 0.006, door_rail)),
            origin=Origin(
                xyz=(center_x, -0.001, door_bottom + door_rail / 2.0)
            ),
            material=maple,
            name="bottom_rail",
        )
        door.visual(
            Box((door_w, door_t + 0.006, mid_rail)),
            origin=Origin(xyz=(center_x, -0.001, door_center_z)),
            material=maple,
            name="middle_rail",
        )

        panel_w = door_w - 2.0 * door_stile + 0.010
        lower_top = door_center_z - mid_rail / 2.0
        lower_bottom = door_bottom + door_rail
        upper_top = door_top - door_rail
        upper_bottom = door_center_z + mid_rail / 2.0
        panel_specs = (
            ("lower_panel", (lower_top + lower_bottom) / 2.0, lower_top - lower_bottom + 0.010),
            ("upper_panel", (upper_top + upper_bottom) / 2.0, upper_top - upper_bottom + 0.010),
        )
        for panel_name, z, panel_h in panel_specs:
            door.visual(
                Box((panel_w, panel_t, panel_h)),
                origin=Origin(xyz=(center_x, panel_y, z)),
                material=panel_wood,
                name=panel_name,
            )
            # Thin dark reveal at the inset panel edge.
            door.visual(
                Box((panel_w + 0.012, 0.004, 0.012)),
                origin=Origin(xyz=(center_x, -door_t / 2.0 - 0.001, z + panel_h / 2.0 - 0.006)),
                material=dark,
                name=f"{panel_name}_top_reveal",
            )
            door.visual(
                Box((panel_w + 0.012, 0.004, 0.012)),
                origin=Origin(xyz=(center_x, -door_t / 2.0 - 0.001, z - panel_h / 2.0 + 0.006)),
                material=dark,
                name=f"{panel_name}_bottom_reveal",
            )
            door.visual(
                Box((0.012, 0.004, panel_h + 0.012)),
                origin=Origin(xyz=(center_x - sign * (panel_w / 2.0 - 0.006), -door_t / 2.0 - 0.001, z)),
                material=dark,
                name=f"{panel_name}_side_reveal_0",
            )
            door.visual(
                Box((0.012, 0.004, panel_h + 0.012)),
                origin=Origin(xyz=(center_x + sign * (panel_w / 2.0 - 0.006), -door_t / 2.0 - 0.001, z)),
                material=dark,
                name=f"{panel_name}_side_reveal_1",
            )

        # Concealed hinge cups and short arms on the inside face of each door.
        cup_x = sign * (door_gap_from_axis + 0.080)
        arm_x = sign * (door_gap_from_axis + 0.052)
        for i, z in enumerate(hinge_zs):
            door.visual(
                Cylinder(radius=0.034, length=0.006),
                origin=Origin(xyz=(cup_x, door_t / 2.0 - 0.002, z), rpy=(PI / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"hinge_cup_{i}",
            )
            door.visual(
                Box((0.022, 0.086, 0.020)),
                origin=Origin(xyz=(arm_x, 0.0605, z)),
                material=metal,
                name=f"hinge_arm_{i}",
            )

        if sign < 0:
            # A strike plate on the companion meeting stile gives the rotary
            # latch a plausible keeper without adding a second latch mechanism.
            door.visual(
                Box((0.038, 0.006, 0.120)),
                origin=Origin(
                    xyz=(meeting_stile_x - sign * 0.010, -door_t / 2.0 - 0.002, 1.12)
                ),
                material=metal,
                name="strike_plate",
            )

    add_framed_door("door_0", 1.0)
    add_framed_door("door_1", -1.0)

    model.articulation(
        "cabinet_to_door_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="door_0",
        origin=Origin(xyz=(-width / 2.0, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_door_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child="door_1",
        origin=Origin(xyz=(width / 2.0, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.85),
    )

    latch = model.part("rotary_latch")
    latch.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(PI / 2.0, 0.0, 0.0)),
        material=metal,
        name="latch_disk",
    )
    latch.visual(
        Box((0.165, 0.012, 0.030)),
        origin=Origin(xyz=(0.045, -0.012, 0.0)),
        material=metal,
        name="latch_bar",
    )
    latch.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(PI / 2.0, 0.0, 0.0)),
        material=black,
        name="finger_knob",
    )

    # The latch pivot is on the meeting stile of door_0, normal to the door
    # face.  At q=0 the bar bridges the center seam; at q=90 degrees it turns
    # vertical and clears the strike plate.
    model.articulation(
        "door_0_to_latch",
        ArticulationType.REVOLUTE,
        parent="door_0",
        child=latch,
        origin=Origin(
            xyz=(door_gap_from_axis + door_w - 0.040, -door_t / 2.0 - 0.004, 1.12)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=PI / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    latch = object_model.get_part("rotary_latch")
    hinge_0 = object_model.get_articulation("cabinet_to_door_0")
    hinge_1 = object_model.get_articulation("cabinet_to_door_1")
    latch_pivot = object_model.get_articulation("door_0_to_latch")

    ctx.check(
        "three independent rotary mechanisms",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            for j in (hinge_0, hinge_1, latch_pivot)
        ),
        details="Both tall doors and the compact latch should be revolute.",
    )
    ctx.check(
        "door hinge axes are vertical and opposed",
        hinge_0.axis == (0.0, 0.0, -1.0) and hinge_1.axis == (0.0, 0.0, 1.0),
        details=f"axes were {hinge_0.axis} and {hinge_1.axis}",
    )
    ctx.check(
        "latch pivot is normal to the door face",
        latch_pivot.axis == (0.0, -1.0, 0.0),
        details=f"latch axis was {latch_pivot.axis}",
    )

    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            cabinet,
            door_0,
            axis="y",
            min_gap=0.002,
            max_gap=0.020,
            negative_elem="solid_backer",
            name="door_0 sits just proud of the cabinet face",
        )
        ctx.expect_gap(
            cabinet,
            door_1,
            axis="y",
            min_gap=0.002,
            max_gap=0.020,
            negative_elem="solid_backer",
            name="door_1 sits just proud of the cabinet face",
        )
        ctx.expect_overlap(
            door_0,
            cabinet,
            axes="z",
            min_overlap=1.80,
            elem_a="solid_backer",
            name="door_0 is a tall pantry door",
        )
        ctx.expect_overlap(
            door_1,
            cabinet,
            axes="z",
            min_overlap=1.80,
            elem_a="solid_backer",
            name="door_1 is a tall pantry door",
        )
        ctx.expect_gap(
            door_0,
            latch,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="meeting_stile",
            negative_elem="latch_disk",
            name="rotary latch disk is seated on the meeting stile",
        )
        ctx.expect_overlap(
            latch,
            door_1,
            axes="x",
            min_overlap=0.025,
            elem_a="latch_bar",
            elem_b="strike_plate",
            name="latch bar bridges toward the strike plate when locked",
        )

    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_0 = ctx.part_world_aabb(door_0)
        open_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "door_0 swings outward from its side hinge",
        closed_0 is not None and open_0 is not None and open_0[0][1] < closed_0[0][1] - 0.20,
        details=f"closed={closed_0}, open={open_0}",
    )
    ctx.check(
        "door_1 swings outward from its side hinge",
        closed_1 is not None and open_1 is not None and open_1[0][1] < closed_1[0][1] - 0.20,
        details=f"closed={closed_1}, open={open_1}",
    )

    locked_bar = ctx.part_element_world_aabb(latch, elem="latch_bar")
    with ctx.pose({latch_pivot: PI / 2.0}):
        turned_bar = ctx.part_element_world_aabb(latch, elem="latch_bar")
    ctx.check(
        "latch bar rotates a quarter turn",
        locked_bar is not None
        and turned_bar is not None
        and (locked_bar[1][0] - locked_bar[0][0]) > 0.14
        and (turned_bar[1][2] - turned_bar[0][2]) > 0.14,
        details=f"locked={locked_bar}, turned={turned_bar}",
    )

    return ctx.report()


object_model = build_object_model()
