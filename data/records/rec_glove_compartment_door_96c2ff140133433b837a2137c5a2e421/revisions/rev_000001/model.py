from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_box")

    fascia_dark = model.material("fascia_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.09, 0.09, 0.10, 1.0))
    liner_dark = model.material("liner_dark", rgba=(0.13, 0.13, 0.14, 1.0))
    knob_black = model.material("knob_black", rgba=(0.07, 0.07, 0.08, 1.0))
    hardware = model.material("hardware", rgba=(0.38, 0.39, 0.40, 1.0))

    outer_width = 0.640
    outer_height = 0.285
    fascia_depth = 0.012
    opening_height = 0.205
    door_width = 0.496
    door_height = 0.200

    fascia = model.part("fascia")
    fascia.visual(
        Box((outer_width, fascia_depth, 0.040)),
        origin=Origin(xyz=(0.0, -0.006, 0.1225)),
        material=fascia_dark,
        name="top_rail",
    )
    fascia.visual(
        Box((outer_width, fascia_depth, 0.040)),
        origin=Origin(xyz=(0.0, -0.006, -0.1225)),
        material=fascia_dark,
        name="bottom_rail",
    )
    fascia.visual(
        Box((0.070, fascia_depth, opening_height)),
        origin=Origin(xyz=(-0.285, -0.006, 0.0)),
        material=fascia_dark,
        name="left_side_rail",
    )
    fascia.visual(
        Box((0.070, fascia_depth, opening_height)),
        origin=Origin(xyz=(0.285, -0.006, 0.0)),
        material=fascia_dark,
        name="right_side_rail",
    )
    fascia.visual(
        Box((0.520, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.027, 0.108)),
        material=trim_black,
        name="hinge_header",
    )
    fascia.visual(
        Box((0.010, 0.018, 0.020)),
        origin=Origin(xyz=(-0.266, -0.051, 0.104)),
        material=trim_black,
        name="left_stay_bracket",
    )
    fascia.visual(
        Box((0.016, 0.048, 0.012)),
        origin=Origin(xyz=(-0.258, -0.030, 0.104)),
        material=trim_black,
        name="left_stay_web",
    )
    fascia.visual(
        Box((0.010, 0.018, 0.020)),
        origin=Origin(xyz=(0.266, -0.051, 0.104)),
        material=trim_black,
        name="right_stay_bracket",
    )
    fascia.visual(
        Box((0.016, 0.048, 0.012)),
        origin=Origin(xyz=(0.258, -0.030, 0.104)),
        material=trim_black,
        name="right_stay_web",
    )
    fascia.inertial = Inertial.from_geometry(
        Box((outer_width, 0.080, outer_height)),
        mass=2.4,
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
    )

    tub = model.part("storage_tub")
    tub_width = 0.484
    tub_height = 0.190
    tub_depth = 0.190
    tub_wall = 0.008
    tub.visual(
        Box((tub_width, tub_depth, tub_wall)),
        origin=Origin(xyz=(0.0, -0.095, 0.091)),
        material=liner_dark,
        name="tub_top",
    )
    tub.visual(
        Box((tub_width, tub_depth, tub_wall)),
        origin=Origin(xyz=(0.0, -0.095, -0.091)),
        material=liner_dark,
        name="tub_bottom",
    )
    tub.visual(
        Box((tub_wall, tub_depth, tub_height - tub_wall)),
        origin=Origin(xyz=(-0.238, -0.095, 0.0)),
        material=liner_dark,
        name="tub_left_wall",
    )
    tub.visual(
        Box((tub_wall, tub_depth, tub_height - tub_wall)),
        origin=Origin(xyz=(0.238, -0.095, 0.0)),
        material=liner_dark,
        name="tub_right_wall",
    )
    tub.visual(
        Box((tub_width - 2.0 * tub_wall, tub_wall, tub_height - tub_wall)),
        origin=Origin(xyz=(0.0, -0.186, 0.0)),
        material=liner_dark,
        name="tub_back",
    )
    tub.visual(
        Box((0.010, 0.060, 0.026)),
        origin=Origin(xyz=(-0.267, -0.018, 0.085)),
        material=liner_dark,
        name="left_mount_flange",
    )
    tub.visual(
        Box((0.028, 0.034, 0.008)),
        origin=Origin(xyz=(-0.250, -0.065, 0.091)),
        material=liner_dark,
        name="left_mount_gusset",
    )
    tub.visual(
        Box((0.010, 0.060, 0.026)),
        origin=Origin(xyz=(0.267, -0.018, 0.085)),
        material=liner_dark,
        name="right_mount_flange",
    )
    tub.visual(
        Box((0.028, 0.034, 0.008)),
        origin=Origin(xyz=(0.250, -0.065, 0.091)),
        material=liner_dark,
        name="right_mount_gusset",
    )
    tub.visual(
        Box((0.468, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.018, -0.082)),
        material=trim_black,
        name="tub_front_floor_return",
    )
    tub.inertial = Inertial.from_geometry(
        Box((tub_width, tub_depth, tub_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.095, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((door_width, 0.010, door_height)),
        origin=Origin(xyz=(0.0, 0.0, -door_height / 2.0)),
        material=fascia_dark,
        name="door_panel",
    )
    door.visual(
        Box((0.470, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, -0.016, -0.008)),
        material=trim_black,
        name="upper_beam",
    )
    door.visual(
        Box((0.018, 0.014, 0.126)),
        origin=Origin(xyz=(-0.218, -0.012, -0.096)),
        material=trim_black,
        name="left_rib",
    )
    door.visual(
        Box((0.018, 0.014, 0.126)),
        origin=Origin(xyz=(0.218, -0.012, -0.096)),
        material=trim_black,
        name="right_rib",
    )
    door.visual(
        Box((0.340, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.013, -0.171)),
        material=trim_black,
        name="lower_rib",
    )
    door.visual(
        Box((0.024, 0.012, 0.030)),
        origin=Origin(xyz=(-0.216, -0.011, -0.024)),
        material=trim_black,
        name="left_stay_pad",
    )
    door.visual(
        Box((0.024, 0.012, 0.030)),
        origin=Origin(xyz=(0.216, -0.011, -0.024)),
        material=trim_black,
        name="right_stay_pad",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.028, door_height)),
        mass=1.05,
        origin=Origin(xyz=(0.0, -0.010, -door_height / 2.0)),
    )

    knob = model.part("latch_knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="knob_bezel",
    )
    knob.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_grip",
    )
    knob.visual(
        Box((0.024, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=hardware,
        name="knob_wing",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.034, 0.024, 0.034)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
    )

    def _build_stay(part_name: str, lateral_offset: float):
        stay = model.part(part_name)
        stay_angle = -0.78
        stay.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="pivot_eye",
        )
        stay.visual(
            Box((0.008, 0.052, 0.010)),
            origin=Origin(
                xyz=(
                    lateral_offset,
                    0.026 * math.cos(stay_angle),
                    0.026 * math.sin(stay_angle),
                ),
                rpy=(stay_angle, 0.0, 0.0),
            ),
            material=hardware,
            name="arm",
        )
        stay.visual(
            Box((0.010, 0.014, 0.012)),
            origin=Origin(
                xyz=(
                    lateral_offset,
                    0.058 * math.cos(stay_angle),
                    0.058 * math.sin(stay_angle),
                ),
                rpy=(stay_angle, 0.0, 0.0),
            ),
            material=hardware,
            name="shoe",
        )
        stay.inertial = Inertial.from_geometry(
            Box((0.018, 0.070, 0.020)),
            mass=0.09,
            origin=Origin(xyz=(0.0, 0.022, -0.020)),
        )
        return stay

    left_stay = _build_stay("left_stay", 0.010)
    right_stay = _build_stay("right_stay", -0.010)

    model.articulation(
        "fascia_to_storage_tub",
        ArticulationType.FIXED,
        parent=fascia,
        child=tub,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
    )
    model.articulation(
        "fascia_to_door",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=door,
        origin=Origin(xyz=(0.0, -0.005, opening_height / 2.0 - 0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "door_to_latch_knob",
        ArticulationType.REVOLUTE,
        parent=door,
        child=knob,
        origin=Origin(xyz=(0.0, 0.005, -0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=4.0,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "fascia_to_left_stay",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=left_stay,
        origin=Origin(xyz=(-0.255, -0.070, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "fascia_to_right_stay",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=right_stay,
        origin=Origin(xyz=(0.255, -0.070, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fascia = object_model.get_part("fascia")
    tub = object_model.get_part("storage_tub")
    door = object_model.get_part("door")
    knob = object_model.get_part("latch_knob")
    left_stay = object_model.get_part("left_stay")
    right_stay = object_model.get_part("right_stay")

    door_hinge = object_model.get_articulation("fascia_to_door")
    knob_joint = object_model.get_articulation("door_to_latch_knob")
    left_stay_joint = object_model.get_articulation("fascia_to_left_stay")
    right_stay_joint = object_model.get_articulation("fascia_to_right_stay")

    def _elem_aabb(part, elem: str):
        return ctx.part_element_world_aabb(part, elem=elem)

    ctx.expect_gap(
        fascia,
        tub,
        axis="y",
        positive_elem="top_rail",
        negative_elem="tub_top",
        min_gap=0.024,
        max_gap=0.032,
        name="storage tub sits behind the fascia",
    )
    ctx.expect_gap(
        door,
        fascia,
        axis="x",
        positive_elem="door_panel",
        negative_elem="left_side_rail",
        min_gap=0.0015,
        max_gap=0.0030,
        name="left door reveal is narrow and even",
    )
    ctx.expect_gap(
        fascia,
        door,
        axis="x",
        positive_elem="right_side_rail",
        negative_elem="door_panel",
        min_gap=0.0015,
        max_gap=0.0030,
        name="right door reveal is narrow and even",
    )
    ctx.expect_gap(
        fascia,
        door,
        axis="z",
        positive_elem="top_rail",
        negative_elem="door_panel",
        min_gap=0.0015,
        max_gap=0.0030,
        name="top door reveal is narrow and even",
    )
    ctx.expect_gap(
        door,
        fascia,
        axis="z",
        positive_elem="door_panel",
        negative_elem="bottom_rail",
        min_gap=0.0032,
        max_gap=0.0055,
        name="bottom door reveal is narrow and even",
    )
    ctx.expect_gap(
        knob,
        door,
        axis="y",
        positive_elem="knob_grip",
        negative_elem="door_panel",
        min_gap=0.008,
        max_gap=0.014,
        name="latch knob projects proud of the door skin",
    )

    door_panel_aabb = _elem_aabb(door, "door_panel")
    top_rail_aabb = _elem_aabb(fascia, "top_rail")
    flush_ok = False
    flush_details = "missing AABB"
    if door_panel_aabb is not None and top_rail_aabb is not None:
        door_front = door_panel_aabb[1][1]
        fascia_front = top_rail_aabb[1][1]
        flush_ok = abs(door_front - fascia_front) <= 0.0005
        flush_details = f"door_front={door_front:.4f}, fascia_front={fascia_front:.4f}"
    ctx.check("door panel is flush with the dashboard fascia", flush_ok, details=flush_details)

    axis_ok = (
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(knob_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(left_stay_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(right_stay_joint.axis) == (1.0, 0.0, 0.0)
    )
    ctx.check(
        "door, latch, and stays use the intended hinge axes",
        axis_ok,
        details=(
            f"door_axis={door_hinge.axis}, knob_axis={knob_joint.axis}, "
            f"left_stay_axis={left_stay_joint.axis}, right_stay_axis={right_stay_joint.axis}"
        ),
    )

    rest_door = _elem_aabb(door, "door_panel")
    rest_left_shoe = _elem_aabb(left_stay, "shoe")
    rest_right_shoe = _elem_aabb(right_stay, "shoe")
    with ctx.pose({door_hinge: 1.05, left_stay_joint: 0.88, right_stay_joint: 0.88, knob_joint: 0.50}):
        open_door = _elem_aabb(door, "door_panel")
        open_left_shoe = _elem_aabb(left_stay, "shoe")
        open_right_shoe = _elem_aabb(right_stay, "shoe")

    door_opens_up = False
    if rest_door is not None and open_door is not None:
        door_opens_up = (
            open_door[0][2] > rest_door[0][2] + 0.075
            and open_door[1][1] > rest_door[1][1] + 0.060
        )
    ctx.check(
        "upper-hinged door swings upward and outward",
        door_opens_up,
        details=f"rest={rest_door}, open={open_door}",
    )

    left_stay_moves = False
    if rest_left_shoe is not None and open_left_shoe is not None:
        left_stay_moves = (
            open_left_shoe[1][2] > rest_left_shoe[1][2] + 0.030
            and open_left_shoe[1][1] > rest_left_shoe[1][1] + 0.010
        )
    ctx.check(
        "left friction stay rotates on its support pivot",
        left_stay_moves,
        details=f"rest={rest_left_shoe}, open={open_left_shoe}",
    )

    right_stay_moves = False
    if rest_right_shoe is not None and open_right_shoe is not None:
        right_stay_moves = (
            open_right_shoe[1][2] > rest_right_shoe[1][2] + 0.030
            and open_right_shoe[1][1] > rest_right_shoe[1][1] + 0.010
        )
    ctx.check(
        "right friction stay rotates on its support pivot",
        right_stay_moves,
        details=f"rest={rest_right_shoe}, open={open_right_shoe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
