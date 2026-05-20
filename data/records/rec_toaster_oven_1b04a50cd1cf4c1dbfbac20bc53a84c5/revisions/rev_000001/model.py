from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_DEPTH = 0.32
BODY_WIDTH = 0.41
BODY_HEIGHT = 0.255
WALL_THICKNESS = 0.012
FRONT_PANEL_THICKNESS = 0.010
REAR_PANEL_THICKNESS = 0.014
BODY_CORNER_RADIUS = 0.028
OPENING_WIDTH = 0.275
OPENING_HEIGHT = 0.155
OPENING_CENTER_Y = -0.055
OPENING_BOTTOM_Z = -0.075
DOOR_WIDTH = 0.293
DOOR_HEIGHT = 0.175
DOOR_THICKNESS = 0.018
WINDOW_WIDTH = 0.205
WINDOW_HEIGHT = 0.092
WINDOW_BOTTOM_Z = 0.048
KNOB_X = BODY_DEPTH / 2.0
KNOB_Y = 0.145
KNOB_ZS = (0.068, 0.000, -0.068)
ROCKER_X = 0.070
ROCKER_Z = -0.008


def build_housing_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_DEPTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        centered=(True, True, True),
    )
    outer = outer.edges("|X").fillet(BODY_CORNER_RADIUS)

    cavity = cq.Workplane("XY").box(
        BODY_DEPTH - FRONT_PANEL_THICKNESS - REAR_PANEL_THICKNESS,
        BODY_WIDTH - 2.0 * WALL_THICKNESS,
        BODY_HEIGHT - 2.0 * WALL_THICKNESS,
        centered=(True, True, True),
    ).translate(((REAR_PANEL_THICKNESS - FRONT_PANEL_THICKNESS) / 2.0, 0.0, 0.0))

    housing = outer.cut(cavity)

    door_cut = cq.Workplane("XY").box(
        FRONT_PANEL_THICKNESS + 0.010,
        OPENING_WIDTH,
        OPENING_HEIGHT,
        centered=(True, True, True),
    ).translate(
        (
            BODY_DEPTH / 2.0 - FRONT_PANEL_THICKNESS / 2.0,
            OPENING_CENTER_Y,
            OPENING_BOTTOM_Z + OPENING_HEIGHT / 2.0,
        )
    )
    housing = housing.cut(door_cut)

    for knob_z in KNOB_ZS:
        knob_hole = (
            cq.Workplane(
                "YZ",
                origin=(BODY_DEPTH / 2.0 - FRONT_PANEL_THICKNESS - 0.003, KNOB_Y, knob_z),
            )
            .circle(0.0115)
            .extrude(FRONT_PANEL_THICKNESS + 0.012)
        )
        housing = housing.cut(knob_hole)

    rocker_cut = cq.Workplane("XY").box(
        0.036,
        WALL_THICKNESS + 0.010,
        0.067,
        centered=(True, True, True),
    ).translate((ROCKER_X, BODY_WIDTH / 2.0 - WALL_THICKNESS / 2.0, ROCKER_Z))
    housing = housing.cut(rocker_cut)

    vent_slot_span = 0.16
    slot_width = 0.006
    slot_height = 0.045
    slot_x = -BODY_DEPTH / 2.0 + REAR_PANEL_THICKNESS / 2.0
    for slot_y in (-0.050, -0.026, -0.002, 0.022, 0.046):
        vent_cut = cq.Workplane("XY").box(
            REAR_PANEL_THICKNESS + 0.008,
            slot_width,
            slot_height,
            centered=(True, True, True),
        ).translate((slot_x, slot_y, 0.030))
        housing = housing.cut(vent_cut)

    foot_size = (0.050, 0.030, 0.008)
    foot_xs = (-0.100, 0.100)
    foot_ys = (-0.145, 0.145)
    for foot_x in foot_xs:
        for foot_y in foot_ys:
            foot = cq.Workplane("XY").box(
                foot_size[0],
                foot_size[1],
                foot_size[2],
                centered=(True, True, False),
            ).translate((foot_x, foot_y, -BODY_HEIGHT / 2.0 - foot_size[2]))
            housing = housing.union(foot)

    return housing


def build_door_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(
        DOOR_THICKNESS,
        DOOR_WIDTH,
        DOOR_HEIGHT,
        centered=(False, True, False),
    )
    frame = frame.edges("|X").fillet(0.015)

    window_cut = cq.Workplane("XY").box(
        DOOR_THICKNESS + 0.004,
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        centered=(False, True, False),
    ).translate((-0.002, 0.0, WINDOW_BOTTOM_Z))
    frame = frame.cut(window_cut)

    handle_span = 0.165
    handle_z = DOOR_HEIGHT - 0.032

    for handle_y in (-0.050, 0.050):
        standoff = cq.Workplane("XY").box(
            0.012,
            0.014,
            0.022,
            centered=(False, True, False),
        ).translate((DOOR_THICKNESS - 0.002, handle_y, handle_z - 0.011))
        frame = frame.union(standoff)

    handle_bar = cq.Workplane("XY").box(
        0.012,
        handle_span,
        0.014,
        centered=(False, True, True),
    ).translate((DOOR_THICKNESS + 0.007, 0.0, handle_z))
    handle_bar = handle_bar.edges("|Y").fillet(0.004)
    return frame.union(handle_bar)


def build_knob_mesh(name: str) -> object:
    knob_geometry = KnobGeometry(
        0.032,
        0.020,
        body_style="skirted",
        top_diameter=0.026,
        edge_radius=0.0012,
        skirt=KnobSkirt(0.038, 0.004, flare=0.05),
        grip=KnobGrip(style="fluted", count=14, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=-35.0),
        center=False,
    )
    return mesh_from_geometry(knob_geometry, name)


def build_rocker_shape() -> cq.Workplane:
    paddle = (
        cq.Workplane("XY")
        .box(0.030, 0.009, 0.058, centered=(True, True, True))
        .edges("|X").fillet(0.004)
        .edges("|Z").fillet(0.002)
    )
    axle = cq.Workplane("XY").box(0.036, 0.0048, 0.0048, centered=(True, True, True)).translate(
        (0.0, -0.0058, 0.0)
    )
    return paddle.union(axle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_toaster_oven")

    enamel = model.material("enamel", rgba=(0.84, 0.79, 0.69, 1.0))
    trim = model.material("trim", rgba=(0.22, 0.18, 0.15, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.92, 0.87, 0.77, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.22, 0.26, 0.45))
    rocker_finish = model.material("rocker_finish", rgba=(0.58, 0.10, 0.08, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(build_housing_shape(), "housing_shell"),
        material=enamel,
        name="housing_shell",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(build_door_shape(), "door_frame"),
        material=trim,
        name="door_frame",
    )
    door.visual(
        Box((0.004, WINDOW_WIDTH + 0.018, WINDOW_HEIGHT + 0.018)),
        origin=Origin(
            xyz=(DOOR_THICKNESS * 0.62, 0.0, WINDOW_BOTTOM_Z + WINDOW_HEIGHT / 2.0),
        ),
        material=glass,
        name="door_glass",
    )

    hinge_y = OPENING_CENTER_Y
    hinge_z = OPENING_BOTTOM_Z
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(BODY_DEPTH / 2.0, hinge_y, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    for index, knob_z in enumerate(KNOB_ZS):
        knob = model.part(f"knob_{index}")
        knob.visual(
            build_knob_mesh(f"knob_{index}_mesh"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=knob_finish,
            name="knob_shell",
        )
        model.articulation(
            f"housing_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(KNOB_X, KNOB_Y, knob_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    rocker = model.part("side_rocker")
    rocker.visual(
        mesh_from_cadquery(build_rocker_shape(), "side_rocker"),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(math.radians(10.0), 0.0, 0.0)),
        material=rocker_finish,
        name="rocker_paddle",
    )
    model.articulation(
        "housing_to_side_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rocker,
        origin=Origin(xyz=(ROCKER_X, BODY_WIDTH / 2.0 + 0.007, ROCKER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-math.radians(14.0),
            upper=math.radians(14.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    rocker = object_model.get_part("side_rocker")
    door_joint = object_model.get_articulation("housing_to_door")
    rocker_joint = object_model.get_articulation("housing_to_side_rocker")
    knob_joints = [
        object_model.get_articulation(f"housing_to_knob_{index}")
        for index in range(len(KNOB_ZS))
    ]

    ctx.allow_isolated_part(
        rocker,
        reason=(
            "The visible rocker is modeled with realistic operating clearance in the side-panel cutout; "
            "the hidden snap-in switch body that supports it inside the housing is intentionally simplified away."
        ),
    )

    ctx.expect_gap(
        door,
        housing,
        axis="x",
        min_gap=0.0,
        max_gap=0.006,
        name="door sits just proud of the front fascia",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="yz",
        min_overlap=0.12,
        name="door covers the oven opening footprint",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.radians(85.0)}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.10
        and open_aabb[1][2] < closed_aabb[1][2] - 0.03,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rocker_closed = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")
    with ctx.pose({rocker_joint: math.radians(12.0)}):
        rocker_on = ctx.part_element_world_aabb(rocker, elem="rocker_paddle")
    ctx.check(
        "side rocker pivots on the side panel",
        rocker_closed is not None
        and rocker_on is not None
        and rocker_on[1][1] > rocker_closed[1][1] + 0.0015,
        details=f"rocker_closed={rocker_closed}, rocker_on={rocker_on}",
    )

    all_knobs_continuous = all(
        joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(float(axis), 4) for axis in joint.axis) == (1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None
        for joint in knob_joints
    )
    ctx.check(
        "three front knobs use continuous shaft rotation",
        all_knobs_continuous and len(knob_joints) == 3,
        details=str(
            [
                {
                    "name": joint.name,
                    "type": str(joint.articulation_type),
                    "axis": joint.axis,
                    "lower": None if joint.motion_limits is None else joint.motion_limits.lower,
                    "upper": None if joint.motion_limits is None else joint.motion_limits.upper,
                }
                for joint in knob_joints
            ]
        ),
    )

    ctx.check(
        "door and rocker joints use appliance-like hinge ranges",
        door_joint.motion_limits is not None
        and rocker_joint.motion_limits is not None
        and door_joint.motion_limits.upper is not None
        and door_joint.motion_limits.upper >= math.radians(90.0)
        and rocker_joint.motion_limits.upper is not None
        and abs(rocker_joint.motion_limits.upper) <= math.radians(18.0),
        details=(
            f"door_limits={door_joint.motion_limits}, "
            f"rocker_limits={rocker_joint.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
