from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.34
BODY_D = 0.22
BODY_H = 0.58
BODY_WALL = 0.005
BODY_CORNER_R = 0.018

CONTROL_FASCIA_H = 0.11
FRONT_SILL_H = 0.03
DOOR_W = 0.296
DOOR_H = 0.40
DOOR_T = 0.018
DOOR_BOTTOM = 0.045
DOOR_OPEN_LIMIT = 1.35

FILTER_W = 0.290
FILTER_H = 0.38
FILTER_D = 0.10
FILTER_FRAME = 0.016
FILTER_BOTTOM = 0.055
FILTER_FRONT_Y = 0.060
FILTER_TRAVEL = 0.070

DIAL_CENTER_Y = 0.028
DIAL_BASE_Z = BODY_H + 0.004

BUTTON_DEPTH = 0.004
BUTTON_STROKE = 0.002
BUTTON_Z = 0.492
BUTTON_XS = (-0.030, 0.030)


def _housing_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).edges("|Z").fillet(BODY_CORNER_R)
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            BODY_D - BODY_WALL + 0.004,
            BODY_H - 2.0 * BODY_WALL + 0.002,
        )
        .translate((0.0, BODY_WALL * 0.5 + 0.002, 0.0))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxy_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.93, 1.0))
    door_white = model.material("door_white", rgba=(0.90, 0.91, 0.91, 1.0))
    control_dark = model.material("control_dark", rgba=(0.24, 0.26, 0.28, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.67, 0.69, 0.71, 1.0))
    filter_frame = model.material("filter_frame", rgba=(0.19, 0.21, 0.22, 1.0))
    filter_media = model.material("filter_media", rgba=(0.50, 0.56, 0.54, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.30, 0.32, 0.34, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "purifier_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=shell_white,
        name="shell",
    )
    housing.visual(
        Box((BODY_W - 2.0 * BODY_WALL + 0.004, 0.020, CONTROL_FASCIA_H + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D * 0.5 - 0.010,
                BODY_H - CONTROL_FASCIA_H * 0.5,
            )
        ),
        material=shell_white,
        name="front_fascia",
    )
    housing.visual(
        Box((BODY_W - 2.0 * BODY_WALL + 0.004, 0.018, FRONT_SILL_H + 0.002)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D * 0.5 - 0.009,
                FRONT_SILL_H * 0.5,
            )
        ),
        material=shell_white,
        name="front_sill",
    )
    housing.visual(
        Box((0.176, 0.092, 0.004)),
        origin=Origin(xyz=(0.0, DIAL_CENTER_Y, BODY_H + 0.002)),
        material=vent_dark,
        name="control_deck",
    )
    housing.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, DIAL_CENTER_Y, BODY_H + 0.002)),
        material=trim_grey,
        name="dial_bezel",
    )
    housing.visual(
        Box((0.020, 0.100, FILTER_H - 0.010)),
        origin=Origin(xyz=(-0.155, -0.008, FILTER_BOTTOM + FILTER_H * 0.5)),
        material=trim_grey,
        name="guide_rail_0",
    )
    housing.visual(
        Box((0.020, 0.100, FILTER_H - 0.010)),
        origin=Origin(xyz=(0.155, -0.008, FILTER_BOTTOM + FILTER_H * 0.5)),
        material=trim_grey,
        name="guide_rail_1",
    )
    housing.visual(
        Box((0.010, 0.010, DOOR_H)),
        origin=Origin(xyz=(-0.165, BODY_D * 0.5 - 0.005, DOOR_BOTTOM + DOOR_H * 0.5)),
        material=trim_grey,
        name="hinge_jamb",
    )
    housing.visual(
        Box((0.146, 0.072, 0.002)),
        origin=Origin(xyz=(0.0, -0.038, BODY_H + 0.001)),
        material=vent_dark,
        name="top_vent_panel",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.122, -0.070),
            (-0.122, 0.070),
            (0.122, -0.070),
            (0.122, 0.070),
        )
    ):
        housing.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(foot_x, foot_y, 0.004)),
            material=rubber_dark,
            name=f"foot_{foot_index}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W * 0.5, 0.0, DOOR_H * 0.5)),
        material=door_white,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_W - 0.040, 0.004, DOOR_H - 0.080)),
        origin=Origin(xyz=(DOOR_W * 0.5, DOOR_T * 0.5 + 0.002, DOOR_H * 0.5)),
        material=trim_grey,
        name="door_inset",
    )
    door.visual(
        Box((0.016, 0.014, 0.100)),
        origin=Origin(xyz=(DOOR_W - 0.012, DOOR_T * 0.5 + 0.007, DOOR_H * 0.5)),
        material=control_dark,
        name="door_pull",
    )
    door.visual(
        Cylinder(radius=0.006, length=DOOR_H),
        origin=Origin(xyz=(0.0, -0.002, DOOR_H * 0.5)),
        material=trim_grey,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.034, 0.010, DOOR_H)),
        origin=Origin(xyz=(-0.017, -0.004, DOOR_H * 0.5)),
        material=trim_grey,
        name="hinge_leaf",
    )
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-DOOR_W * 0.5, BODY_D * 0.5 + DOOR_T * 0.5, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )

    filter_pack = model.part("filter_pack")
    filter_pack.visual(
        Box((FILTER_FRAME, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(-FILTER_W * 0.5 + FILTER_FRAME * 0.5, -FILTER_D * 0.5, FILTER_H * 0.5)),
        material=filter_frame,
        name="filter_side_0",
    )
    filter_pack.visual(
        Box((FILTER_FRAME, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(FILTER_W * 0.5 - FILTER_FRAME * 0.5, -FILTER_D * 0.5, FILTER_H * 0.5)),
        material=filter_frame,
        name="filter_side_1",
    )
    filter_pack.visual(
        Box((FILTER_W, FILTER_D, FILTER_FRAME)),
        origin=Origin(xyz=(0.0, -FILTER_D * 0.5, FILTER_FRAME * 0.5)),
        material=filter_frame,
        name="filter_bottom",
    )
    filter_pack.visual(
        Box((FILTER_W, FILTER_D, FILTER_FRAME)),
        origin=Origin(xyz=(0.0, -FILTER_D * 0.5, FILTER_H - FILTER_FRAME * 0.5)),
        material=filter_frame,
        name="filter_top",
    )
    filter_pack.visual(
        Box(
            (
                FILTER_W - 2.0 * FILTER_FRAME + 0.004,
                FILTER_D - 0.014,
                FILTER_H - 2.0 * FILTER_FRAME + 0.004,
            )
        ),
        origin=Origin(xyz=(0.0, -FILTER_D * 0.5, FILTER_H * 0.5)),
        material=filter_media,
        name="filter_media",
    )
    filter_pack.visual(
        Box((0.070, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.002, FILTER_H - 0.040)),
        material=control_dark,
        name="filter_pull",
    )
    for slat_index, slat_x in enumerate((-0.084, -0.056, -0.028, 0.0, 0.028, 0.056, 0.084)):
        filter_pack.visual(
            Box((0.004, 0.004, FILTER_H - 0.084)),
            origin=Origin(xyz=(slat_x, -0.005, FILTER_H * 0.5)),
            material=filter_frame,
            name=f"filter_pleat_{slat_index}",
        )
    model.articulation(
        "housing_to_filter_pack",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=filter_pack,
        origin=Origin(xyz=(0.0, FILTER_FRONT_Y, FILTER_BOTTOM)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.20,
            lower=0.0,
            upper=FILTER_TRAVEL,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim_grey,
        name="dial_shaft",
    )
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.018,
                body_style="skirted",
                top_diameter=0.036,
                edge_radius=0.0012,
                center=False,
            ),
            "purifier_dial_cap",
        ),
        material=control_dark,
        name="dial_cap",
    )
    dial.visual(
        Box((0.003, 0.014, 0.002)),
        origin=Origin(xyz=(0.0, 0.014, 0.017)),
        material=shell_white,
        name="dial_indicator",
    )
    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_CENTER_Y, DIAL_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    for button_index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{button_index}")
        button.visual(
            Box((0.018, BUTTON_DEPTH, 0.011)),
            material=trim_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=trim_grey,
            name="button_stem",
        )
        model.articulation(
            f"housing_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(
                xyz=(
                    button_x,
                    BODY_D * 0.5 + BUTTON_DEPTH * 0.5 + BUTTON_STROKE,
                    BUTTON_Z,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_STROKE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    filter_pack = object_model.get_part("filter_pack")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    door_joint = object_model.get_articulation("housing_to_door")
    filter_joint = object_model.get_articulation("housing_to_filter_pack")
    dial_joint = object_model.get_articulation("housing_to_dial")
    button_joint_0 = object_model.get_articulation("housing_to_button_0")
    button_joint_1 = object_model.get_articulation("housing_to_button_1")

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        max_gap=0.003,
        max_penetration=0.0,
        name="door sits flush against the purifier front",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_panel",
        min_overlap=0.20,
        name="door covers the purifier front opening",
    )
    ctx.expect_gap(
        door,
        filter_pack,
        axis="y",
        positive_elem="door_panel",
        negative_elem="filter_media",
        min_gap=0.030,
        max_gap=0.060,
        name="filter pack rests behind the closed door",
    )
    ctx.expect_within(
        filter_pack,
        housing,
        axes="xz",
        inner_elem="filter_media",
        margin=0.020,
        name="filter pack stays within the front cavity",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    rest_filter_pos = ctx.part_world_position(filter_pack)
    rest_button_0_pos = ctx.part_world_position(button_0)
    rest_button_1_pos = ctx.part_world_position(button_1)

    with ctx.pose({door_joint: DOOR_OPEN_LIMIT}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward on its vertical hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_joint: DOOR_OPEN_LIMIT, filter_joint: FILTER_TRAVEL}):
        ctx.expect_within(
            filter_pack,
            housing,
            axes="xz",
            inner_elem="filter_media",
            margin=0.020,
            name="extended filter stays centered in the cavity",
        )
        ctx.expect_overlap(
            filter_pack,
            housing,
            axes="y",
            elem_a="filter_media",
            min_overlap=0.030,
            name="extended filter retains insertion depth",
        )
        extended_filter_pos = ctx.part_world_position(filter_pack)
    ctx.check(
        "filter pack slides forward when pulled",
        rest_filter_pos is not None
        and extended_filter_pos is not None
        and extended_filter_pos[1] > rest_filter_pos[1] + 0.050,
        details=f"rest={rest_filter_pos}, extended={extended_filter_pos}",
    )

    with ctx.pose({button_joint_0: BUTTON_STROKE}):
        pressed_button_0 = ctx.part_world_position(button_0)
        idle_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "front button 0 depresses independently",
        rest_button_0_pos is not None
        and pressed_button_0 is not None
        and rest_button_1_pos is not None
        and idle_button_1 is not None
        and pressed_button_0[1] < rest_button_0_pos[1] - 0.0015
        and abs(idle_button_1[1] - rest_button_1_pos[1]) < 1e-6,
        details=(
            f"button_0_rest={rest_button_0_pos}, button_0_pressed={pressed_button_0}, "
            f"button_1_rest={rest_button_1_pos}, button_1_idle={idle_button_1}"
        ),
    )

    with ctx.pose({button_joint_1: BUTTON_STROKE}):
        pressed_button_1 = ctx.part_world_position(button_1)
        idle_button_0 = ctx.part_world_position(button_0)
    ctx.check(
        "front button 1 depresses independently",
        rest_button_1_pos is not None
        and pressed_button_1 is not None
        and rest_button_0_pos is not None
        and idle_button_0 is not None
        and pressed_button_1[1] < rest_button_1_pos[1] - 0.0015
        and abs(idle_button_0[1] - rest_button_0_pos[1]) < 1e-6,
        details=(
            f"button_1_rest={rest_button_1_pos}, button_1_pressed={pressed_button_1}, "
            f"button_0_rest={rest_button_0_pos}, button_0_idle={idle_button_0}"
        ),
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "top dial uses a continuous rotary joint",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"dial_joint_type={dial_joint.articulation_type}, limits={dial_limits}",
    )
    ctx.expect_gap(
        dial,
        housing,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="control_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial cap seats on the top control deck",
    )

    return ctx.report()


object_model = build_object_model()
