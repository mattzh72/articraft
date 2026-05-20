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
    model = ArticulatedObject(name="center_console_glove_compartment")

    panel_dark = model.material("panel_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    door_dark = model.material("door_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    bin_liner = model.material("bin_liner", rgba=(0.08, 0.08, 0.09, 1.0))
    pull_trim = model.material("pull_trim", rgba=(0.33, 0.34, 0.36, 1.0))

    opening_w = 0.180
    opening_h = 0.072
    fascia_w = 0.218
    fascia_h = 0.108
    fascia_t = 0.004
    side_bezel = (fascia_w - opening_w) / 2.0
    top_bezel = (fascia_h - opening_h) / 2.0

    bin_depth = 0.128
    wall_t = 0.003
    bin_inner_w = 0.174
    bin_inner_h = 0.068
    wall_center_x = fascia_t + bin_depth / 2.0

    console_housing = model.part("console_housing")
    console_housing.visual(
        Box((fascia_t, fascia_w, top_bezel)),
        origin=Origin(xyz=(fascia_t / 2.0, 0.0, opening_h / 2.0 + top_bezel / 2.0)),
        material=panel_dark,
        name="fascia_top",
    )
    console_housing.visual(
        Box((fascia_t, fascia_w, top_bezel)),
        origin=Origin(xyz=(fascia_t / 2.0, 0.0, -(opening_h / 2.0 + top_bezel / 2.0))),
        material=panel_dark,
        name="fascia_bottom",
    )
    console_housing.visual(
        Box((fascia_t, side_bezel, opening_h)),
        origin=Origin(xyz=(fascia_t / 2.0, -(opening_w / 2.0 + side_bezel / 2.0), 0.0)),
        material=panel_dark,
        name="left_jamb",
    )
    console_housing.visual(
        Box((fascia_t, side_bezel, opening_h)),
        origin=Origin(xyz=(fascia_t / 2.0, opening_w / 2.0 + side_bezel / 2.0, 0.0)),
        material=panel_dark,
        name="right_jamb",
    )
    console_housing.visual(
        Box((bin_depth, bin_inner_w, wall_t)),
        origin=Origin(xyz=(wall_center_x, 0.0, -(opening_h / 2.0 - wall_t / 2.0))),
        material=bin_liner,
        name="bin_floor",
    )
    console_housing.visual(
        Box((bin_depth, bin_inner_w, wall_t)),
        origin=Origin(xyz=(wall_center_x, 0.0, opening_h / 2.0 - wall_t / 2.0)),
        material=bin_liner,
        name="bin_roof",
    )
    console_housing.visual(
        Box((bin_depth, wall_t, opening_h)),
        origin=Origin(xyz=(wall_center_x, -(bin_inner_w / 2.0 + wall_t / 2.0), 0.0)),
        material=bin_liner,
        name="bin_left_wall",
    )
    console_housing.visual(
        Box((bin_depth, wall_t, opening_h)),
        origin=Origin(xyz=(wall_center_x, bin_inner_w / 2.0 + wall_t / 2.0, 0.0)),
        material=bin_liner,
        name="bin_right_wall",
    )
    console_housing.visual(
        Box((wall_t, bin_inner_w, bin_inner_h)),
        origin=Origin(xyz=(fascia_t + bin_depth + wall_t / 2.0, 0.0, 0.0)),
        material=bin_liner,
        name="bin_back",
    )
    for side_name, side_y in (("left", -0.062), ("right", 0.062)):
        console_housing.visual(
            Box((0.012, 0.018, 0.010)),
            origin=Origin(xyz=(0.008, side_y, -0.037)),
            material=panel_dark,
            name=f"{side_name}_hinge_pedestal",
        )
    console_housing.inertial = Inertial.from_geometry(
        Box((fascia_t + bin_depth + wall_t, fascia_w, fascia_h)),
        mass=0.65,
        origin=Origin(xyz=((fascia_t + bin_depth + wall_t) / 2.0, 0.0, 0.0)),
    )

    door_w = 0.176
    door_h = 0.068
    door_t = 0.0035

    glove_door = model.part("glove_door")
    glove_door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(-door_t / 2.0, 0.0, door_h / 2.0)),
        material=door_dark,
        name="door_panel",
    )
    glove_door.visual(
        Box((0.010, 0.132, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.007)),
        material=door_dark,
        name="hinge_rail",
    )
    for side_name, side_y in (("left", -0.062), ("right", 0.062)):
        glove_door.visual(
            Box((0.014, 0.018, 0.014)),
            origin=Origin(xyz=(0.007, side_y, 0.009)),
            material=door_dark,
            name=f"{side_name}_hinge_bracket",
        )
    glove_door.inertial = Inertial.from_geometry(
        Box((0.016, door_w, door_h)),
        mass=0.24,
        origin=Origin(xyz=(0.004, 0.0, door_h / 2.0)),
    )

    finger_pull = model.part("finger_pull")
    finger_pull.visual(
        Cylinder(radius=0.0025, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pull_trim,
        name="pull_barrel",
    )
    finger_pull.visual(
        Box((0.006, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=pull_trim,
        name="pull_bridge",
    )
    finger_pull.visual(
        Box((0.0045, 0.038, 0.014)),
        origin=Origin(xyz=(-0.00475, 0.0, -0.0085)),
        material=pull_trim,
        name="pull_grip",
    )
    finger_pull.visual(
        Cylinder(radius=0.0035, length=0.034),
        origin=Origin(xyz=(-0.007, 0.0, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pull_trim,
        name="pull_lip",
    )
    finger_pull.inertial = Inertial.from_geometry(
        Box((0.012, 0.040, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(-0.004, 0.0, -0.008)),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=console_housing,
        child=glove_door,
        origin=Origin(xyz=(0.0, 0.0, -door_h / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(86.0),
        ),
    )
    model.articulation(
        "door_to_finger_pull",
        ArticulationType.REVOLUTE,
        parent=glove_door,
        child=finger_pull,
        origin=Origin(xyz=(-0.0065, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(24.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    console_housing = object_model.get_part("console_housing")
    glove_door = object_model.get_part("glove_door")
    finger_pull = object_model.get_part("finger_pull")
    door_hinge = object_model.get_articulation("housing_to_door")
    pull_hinge = object_model.get_articulation("door_to_finger_pull")

    with ctx.pose({door_hinge: 0.0, pull_hinge: 0.0}):
        ctx.expect_gap(
            console_housing,
            glove_door,
            axis="z",
            positive_elem="fascia_top",
            negative_elem="door_panel",
            min_gap=0.001,
            max_gap=0.0035,
            name="door clears upper frame",
        )
        ctx.expect_gap(
            glove_door,
            console_housing,
            axis="z",
            positive_elem="door_panel",
            negative_elem="fascia_bottom",
            min_gap=0.001,
            max_gap=0.0035,
            name="door clears lower frame",
        )
        ctx.expect_gap(
            glove_door,
            console_housing,
            axis="y",
            positive_elem="door_panel",
            negative_elem="left_jamb",
            min_gap=0.001,
            max_gap=0.0035,
            name="door clears left jamb",
        )
        ctx.expect_gap(
            console_housing,
            glove_door,
            axis="y",
            positive_elem="right_jamb",
            negative_elem="door_panel",
            min_gap=0.001,
            max_gap=0.0035,
            name="door clears right jamb",
        )
        ctx.expect_gap(
            console_housing,
            glove_door,
            axis="x",
            positive_elem="bin_back",
            negative_elem="door_panel",
            min_gap=0.12,
            name="fixed bin sits behind the front door",
        )
        ctx.expect_gap(
            glove_door,
            finger_pull,
            axis="x",
            positive_elem="door_panel",
            negative_elem="pull_grip",
            min_gap=0.004,
            name="finger pull sits proud of the door face",
        )

    closed_door_aabb = None
    open_door_aabb = None
    with ctx.pose({door_hinge: 0.0, pull_hinge: 0.0}):
        closed_door_aabb = ctx.part_element_world_aabb(glove_door, elem="door_panel")
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper, pull_hinge: 0.0}):
        open_door_aabb = ctx.part_element_world_aabb(glove_door, elem="door_panel")

    ctx.check(
        "door rotates downward on the lower hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.05
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.03,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_pull_aabb = None
    open_pull_aabb = None
    with ctx.pose({door_hinge: 0.0, pull_hinge: 0.0}):
        closed_pull_aabb = ctx.part_element_world_aabb(finger_pull, elem="pull_grip")
    with ctx.pose({door_hinge: 0.0, pull_hinge: pull_hinge.motion_limits.upper}):
        open_pull_aabb = ctx.part_element_world_aabb(finger_pull, elem="pull_grip")

    ctx.check(
        "finger pull pivots outward",
        closed_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[0][0] < closed_pull_aabb[0][0] - 0.0015,
        details=f"closed={closed_pull_aabb}, open={open_pull_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
