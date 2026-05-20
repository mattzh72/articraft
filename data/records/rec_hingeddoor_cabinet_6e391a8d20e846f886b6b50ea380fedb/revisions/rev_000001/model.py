from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WOOD = Material("warm_walnut", rgba=(0.43, 0.22, 0.10, 1.0))
DARK_WOOD = Material("recessed_walnut_panel", rgba=(0.27, 0.12, 0.055, 1.0))
INTERIOR = Material("shadowed_interior", rgba=(0.09, 0.055, 0.035, 1.0))
GLASS = Material("slightly_blue_glass", rgba=(0.62, 0.82, 0.95, 0.38))
BRASS = Material("aged_brass", rgba=(0.78, 0.57, 0.23, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glazed_hutch_cabinet")

    width = 1.20
    depth = 0.46
    height = 1.80
    wall = 0.040
    front_y = -0.252
    hinge_x = 0.580
    door_w = 0.535

    model.materials.extend([WOOD, DARK_WOOD, INTERIOR, GLASS, BRASS])

    case = model.part("case")

    # A fixed wooden hutch case: side walls, back, shelves, center mullion, and
    # front face rails are authored as a single supported cabinet carcass.
    case.visual(Box((wall, depth, height - 0.10)), origin=Origin(xyz=(-0.600, 0.0, 0.875)), material=WOOD, name="side_wall_0")
    case.visual(Box((wall, depth, height - 0.10)), origin=Origin(xyz=(0.600, 0.0, 0.875)), material=WOOD, name="side_wall_1")
    case.visual(Box((width, 0.024, height - 0.10)), origin=Origin(xyz=(0.0, 0.218, 0.875)), material=WOOD, name="back_panel")
    case.visual(Box((width + 0.06, depth + 0.035, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=WOOD, name="bottom_deck")
    case.visual(Box((width + 0.08, depth + 0.050, 0.055)), origin=Origin(xyz=(0.0, 0.0, 1.752)), material=WOOD, name="crown_cap")
    case.visual(Box((width + 0.02, depth + 0.010, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.785)), material=WOOD, name="section_divider")
    case.visual(Box((width - 0.04, 0.045, 0.045)), origin=Origin(xyz=(0.0, 0.192, 1.300)), material=WOOD, name="upper_shelf")

    # Dark rear fields make the open glazed section read as a cabinet interior.
    case.visual(Box((width - 0.13, 0.030, 0.840)), origin=Origin(xyz=(0.0, 0.192, 1.265)), material=INTERIOR, name="upper_shadow")
    case.visual(Box((width - 0.13, 0.030, 0.570)), origin=Origin(xyz=(0.0, 0.192, 0.420)), material=INTERIOR, name="lower_shadow")

    # Front face frame, attached into the side walls and horizontal decks.
    case.visual(Box((0.045, 0.044, 1.620)), origin=Origin(xyz=(-0.600, -0.232, 0.885)), material=WOOD, name="front_stile_0")
    case.visual(Box((0.045, 0.044, 1.620)), origin=Origin(xyz=(0.600, -0.232, 0.885)), material=WOOD, name="front_stile_1")
    case.visual(Box((0.040, 0.042, 1.610)), origin=Origin(xyz=(0.0, -0.234, 0.885)), material=WOOD, name="center_mullion")
    case.visual(Box((width + 0.02, 0.045, 0.070)), origin=Origin(xyz=(0.0, -0.232, 0.090)), material=WOOD, name="lower_rail")
    case.visual(Box((width + 0.02, 0.045, 0.065)), origin=Origin(xyz=(0.0, -0.232, 0.785)), material=WOOD, name="middle_rail")
    case.visual(Box((width + 0.02, 0.045, 0.070)), origin=Origin(xyz=(0.0, -0.232, 1.690)), material=WOOD, name="upper_rail")

    def add_hinge_knuckles(door, sign: float, z_centers: tuple[float, ...], barrel_name: str) -> None:
        for idx, zc in enumerate(z_centers):
            door.visual(
                Cylinder(radius=0.013, length=0.115),
                origin=Origin(xyz=(0.0, -0.018, zc)),
                material=BRASS,
                name=f"{barrel_name}_{idx}",
            )
            door.visual(
                Box((0.050, 0.006, 0.085)),
                origin=Origin(xyz=(sign * 0.022, -0.028, zc)),
                material=BRASS,
                name=f"hinge_leaf_{idx}",
            )

    def make_upper_door(name: str, sign: float):
        door = model.part(name)
        door_h = 0.790
        stile = 0.052
        rail = 0.058
        y_thick = 0.032
        cx = sign * door_w * 0.5
        # Wood frame around one inset glass sheet, plus muntins over the glass.
        door.visual(Box((stile, y_thick, door_h)), origin=Origin(xyz=(sign * stile * 0.5, 0.0, door_h * 0.5)), material=WOOD, name="hinge_stile")
        door.visual(Box((stile, y_thick, door_h)), origin=Origin(xyz=(sign * (door_w - stile * 0.5), 0.0, door_h * 0.5)), material=WOOD, name="meeting_stile")
        door.visual(Box((door_w, y_thick, rail)), origin=Origin(xyz=(cx, 0.0, rail * 0.5)), material=WOOD, name="bottom_rail")
        door.visual(Box((door_w, y_thick, rail)), origin=Origin(xyz=(cx, 0.0, door_h - rail * 0.5)), material=WOOD, name="top_rail")
        door.visual(Box((door_w - 0.050, y_thick + 0.004, 0.034)), origin=Origin(xyz=(cx, -0.001, door_h * 0.50)), material=WOOD, name="cross_muntin")
        door.visual(Box((0.026, y_thick + 0.004, door_h - 0.100)), origin=Origin(xyz=(cx, -0.001, door_h * 0.50)), material=WOOD, name="vertical_muntin")
        door.visual(Box((door_w - 0.090, 0.007, door_h - 0.095)), origin=Origin(xyz=(cx, 0.007, door_h * 0.50)), material=GLASS, name="glass_pane")
        add_hinge_knuckles(door, sign, (0.145, 0.395, 0.645), "upper_hinge_barrel")
        return door

    def make_lower_door(name: str, sign: float):
        door = model.part(name)
        door_h = 0.610
        stile = 0.060
        rail = 0.070
        y_thick = 0.034
        cx = sign * door_w * 0.5
        door.visual(Box((door_w, y_thick, door_h)), origin=Origin(xyz=(cx, 0.0, door_h * 0.5)), material=WOOD, name="solid_slab")
        door.visual(Box((door_w - 0.140, 0.008, door_h - 0.180)), origin=Origin(xyz=(cx, -0.021, door_h * 0.5)), material=DARK_WOOD, name="recessed_panel")
        door.visual(Box((stile, 0.012, door_h - 0.040)), origin=Origin(xyz=(sign * (stile * 0.5 + 0.012), -0.026, door_h * 0.5)), material=WOOD, name="outer_raise")
        door.visual(Box((stile, 0.012, door_h - 0.040)), origin=Origin(xyz=(sign * (door_w - stile * 0.5 - 0.012), -0.026, door_h * 0.5)), material=WOOD, name="inner_raise")
        door.visual(Box((door_w - 0.040, 0.012, rail)), origin=Origin(xyz=(cx, -0.026, rail * 0.5 + 0.018)), material=WOOD, name="bottom_raise")
        door.visual(Box((door_w - 0.040, 0.012, rail)), origin=Origin(xyz=(cx, -0.026, door_h - rail * 0.5 - 0.018)), material=WOOD, name="top_raise")
        add_hinge_knuckles(door, sign, (0.140, 0.470), "lower_hinge_barrel")
        return door

    upper_door_0 = make_upper_door("upper_door_0", 1.0)
    upper_door_1 = make_upper_door("upper_door_1", -1.0)
    lower_door_0 = make_lower_door("lower_door_0", 1.0)
    lower_door_1 = make_lower_door("lower_door_1", -1.0)

    # Hinges are on the outside cabinet side walls. The numeric door suffixes
    # distinguish the two symmetric leaves without encoding state.
    upper_z = 0.865
    lower_z = 0.125
    model.articulation(
        "case_to_upper_door_0",
        ArticulationType.REVOLUTE,
        parent=case,
        child=upper_door_0,
        origin=Origin(xyz=(-hinge_x, front_y, upper_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.72),
    )
    model.articulation(
        "case_to_upper_door_1",
        ArticulationType.REVOLUTE,
        parent=case,
        child=upper_door_1,
        origin=Origin(xyz=(hinge_x, front_y, upper_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.72),
    )
    model.articulation(
        "case_to_lower_door_0",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lower_door_0,
        origin=Origin(xyz=(-hinge_x, front_y, lower_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "case_to_lower_door_1",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lower_door_1,
        origin=Origin(xyz=(hinge_x, front_y, lower_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    def make_lower_knob(name: str):
        knob = model.part(name)
        # The part frame lies on the small latch shaft at the door face.  The
        # brass stem and thumb bar then protrude forward from the panel.
        knob.visual(
            Cylinder(radius=0.008, length=0.020),
            origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BRASS,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=BRASS,
            name="round_rosette",
        )
        knob.visual(
            Box((0.082, 0.013, 0.020)),
            origin=Origin(xyz=(0.0, -0.034, 0.0)),
            material=BRASS,
            name="turn_bar",
        )
        return knob

    lower_knob_0 = make_lower_knob("lower_knob_0")
    lower_knob_1 = make_lower_knob("lower_knob_1")

    model.articulation(
        "lower_door_0_to_knob",
        ArticulationType.REVOLUTE,
        parent=lower_door_0,
        child=lower_knob_0,
        origin=Origin(xyz=(door_w - 0.085, -0.025, 0.330)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "lower_door_1_to_knob",
        ArticulationType.REVOLUTE,
        parent=lower_door_1,
        child=lower_knob_1,
        origin=Origin(xyz=(-(door_w - 0.085), -0.025, 0.330)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door_joints = [
        object_model.get_articulation("case_to_upper_door_0"),
        object_model.get_articulation("case_to_upper_door_1"),
        object_model.get_articulation("case_to_lower_door_0"),
        object_model.get_articulation("case_to_lower_door_1"),
    ]
    ctx.check(
        "four side-mounted vertical door hinges",
        all(j.axis == (0.0, 0.0, -1.0) or j.axis == (0.0, 0.0, 1.0) for j in door_joints),
        details=f"axes={[j.axis for j in door_joints]}",
    )

    knob_joints = [
        object_model.get_articulation("lower_door_0_to_knob"),
        object_model.get_articulation("lower_door_1_to_knob"),
    ]
    ctx.check(
        "lower latch knobs rotate on door-normal shafts",
        all(j.axis == (0.0, -1.0, 0.0) for j in knob_joints),
        details=f"axes={[j.axis for j in knob_joints]}",
    )

    # At the upper limit each leaf swings outward from the front plane, not into
    # the cabinet.  The hinge-line part origin stays fixed, so compare AABBs.
    for door_name, joint_name in [
        ("upper_door_0", "case_to_upper_door_0"),
        ("upper_door_1", "case_to_upper_door_1"),
        ("lower_door_0", "case_to_lower_door_0"),
        ("lower_door_1", "case_to_lower_door_1"),
    ]:
        door = object_model.get_part(door_name)
        joint = object_model.get_articulation(joint_name)
        closed = ctx.part_world_aabb(door)
        with ctx.pose({joint: joint.motion_limits.upper}):
            opened = ctx.part_world_aabb(door)
        ctx.check(
            f"{door_name} swings outward",
            closed is not None and opened is not None and opened[0][1] < closed[0][1] - 0.20,
            details=f"closed={closed}, opened={opened}",
        )

    # A quarter turn changes each lower latch bar from horizontal to vertical.
    for knob_name, joint_name in [
        ("lower_knob_0", "lower_door_0_to_knob"),
        ("lower_knob_1", "lower_door_1_to_knob"),
    ]:
        knob = object_model.get_part(knob_name)
        joint = object_model.get_articulation(joint_name)
        closed_bar = ctx.part_element_world_aabb(knob, elem="turn_bar")
        with ctx.pose({joint: math.pi / 2.0}):
            turned_bar = ctx.part_element_world_aabb(knob, elem="turn_bar")
        if closed_bar is None or turned_bar is None:
            ok = False
        else:
            closed_dx = closed_bar[1][0] - closed_bar[0][0]
            closed_dz = closed_bar[1][2] - closed_bar[0][2]
            turned_dx = turned_bar[1][0] - turned_bar[0][0]
            turned_dz = turned_bar[1][2] - turned_bar[0][2]
            ok = closed_dx > closed_dz + 0.035 and turned_dz > turned_dx + 0.035
        ctx.check(
            f"{knob_name} latch bar quarter-turns",
            ok,
            details=f"closed={closed_bar}, turned={turned_bar}",
        )

    return ctx.report()


object_model = build_object_model()
