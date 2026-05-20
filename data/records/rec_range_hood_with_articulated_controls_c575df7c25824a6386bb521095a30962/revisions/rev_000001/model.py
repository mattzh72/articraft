from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)

HOOD_WIDTH = 0.90
BODY_BACK_Y = -0.29
BODY_FRONT_Y = 0.29
TOP_FRONT_Y = 0.14
CANOPY_HEIGHT = 0.38
SHELL_THICKNESS = 0.018

CHIMNEY_WIDTH = 0.34
CHIMNEY_DEPTH = 0.16
CHIMNEY_HEIGHT = 0.34

DOOR_WIDTH = 0.82
DOOR_DEPTH = 0.42
DOOR_THICKNESS = 0.006
DOOR_HINGE_Y = -0.232
DOOR_HINGE_Z = 0.002

KNOB_CENTER_Z = 0.102
KNOB_X_POSITIONS = (-0.115, 0.115)
CONTROL_RAIL_Y = 0.255
CONTROL_RAIL_DEPTH = 0.050
CONTROL_RAIL_HEIGHT = 0.062


def _build_canopy_shell() -> cq.Workplane:
    outer_profile = [
        (BODY_BACK_Y, 0.0),
        (BODY_FRONT_Y, 0.0),
        (TOP_FRONT_Y, CANOPY_HEIGHT),
        (BODY_BACK_Y, CANOPY_HEIGHT),
    ]
    inner_profile = [
        (BODY_BACK_Y + SHELL_THICKNESS, 0.0),
        (BODY_FRONT_Y - 0.050, 0.0),
        (TOP_FRONT_Y - 0.030, CANOPY_HEIGHT - SHELL_THICKNESS),
        (BODY_BACK_Y + SHELL_THICKNESS, CANOPY_HEIGHT - SHELL_THICKNESS),
    ]

    canopy = (
        cq.Workplane("YZ")
        .polyline(outer_profile)
        .close()
        .extrude(HOOD_WIDTH, both=True)
    )
    cavity = (
        cq.Workplane("YZ")
        .polyline(inner_profile)
        .close()
        .extrude(HOOD_WIDTH - 2.0 * SHELL_THICKNESS, both=True)
    )

    plenum = cq.Workplane("XY").box(HOOD_WIDTH * 0.78, 0.14, 0.06).translate(
        (0.0, BODY_BACK_Y + 0.07, CANOPY_HEIGHT - 0.03)
    )
    chimney = cq.Workplane("XY").box(CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT).translate(
        (0.0, BODY_BACK_Y + CHIMNEY_DEPTH / 2.0 + 0.03, CANOPY_HEIGHT + CHIMNEY_HEIGHT / 2.0)
    )

    return canopy.cut(cavity).union(plenum).union(chimney)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canopy_range_hood")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.82, 1.0))
    filter_finish = model.material("filter_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.28, 0.29, 0.31, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_build_canopy_shell(), "range_hood_shell"),
        material=stainless,
        name="shell",
    )
    hood.visual(
        Box((HOOD_WIDTH * 0.82, CONTROL_RAIL_DEPTH, CONTROL_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_RAIL_Y, KNOB_CENTER_Z)),
        material=stainless,
        name="control_rail",
    )
    hood.visual(
        Box((DOOR_WIDTH + 0.06, 0.044, 0.016)),
        origin=Origin(xyz=(0.0, -0.257, 0.010)),
        material=stainless,
        name="hinge_leaf",
    )

    knob_cap_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.026,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.058, 0.006, flare=0.08),
            grip=KnobGrip(style="fluted", count=22, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "range_hood_knob_cap",
    )

    for knob_index, knob_x in enumerate(KNOB_X_POSITIONS):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.021, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=stainless,
            name="collar",
        )
        knob.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=trim_dark,
            name="shaft",
        )
        knob.visual(
            knob_cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=knob_finish,
            name="cap",
        )
        model.articulation(
            f"hood_to_knob_{knob_index}",
            ArticulationType.CONTINUOUS,
            parent=hood,
            child=knob,
            origin=Origin(
                xyz=(knob_x, CONTROL_RAIL_Y + CONTROL_RAIL_DEPTH / 2.0, KNOB_CENTER_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=12.0),
        )

    filter_panel_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (DOOR_WIDTH, DOOR_DEPTH),
            DOOR_THICKNESS,
            slot_size=(0.030, 0.0055),
            pitch=(0.044, 0.016),
            frame=0.030,
            corner_radius=0.004,
            slot_angle_deg=18.0,
            stagger=True,
        ),
        "range_hood_filter_panel",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        filter_panel_mesh,
        origin=Origin(xyz=(0.0, DOOR_DEPTH / 2.0, -0.010)),
        material=filter_finish,
        name="filter_panel",
    )
    filter_door.visual(
        Box((DOOR_WIDTH * 0.90, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, -0.005)),
        material=stainless,
        name="hinge_bar",
    )
    filter_door.visual(
        Box((DOOR_WIDTH * 0.86, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, DOOR_DEPTH - 0.014, -0.014)),
        material=stainless,
        name="pull_lip",
    )
    for knuckle_index, knuckle_x in enumerate((-0.24, 0.0, 0.24)):
        filter_door.visual(
            Cylinder(radius=0.006, length=0.120),
            origin=Origin(xyz=(knuckle_x, 0.003, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"knuckle_{knuckle_index}",
        )

    model.articulation(
        "hood_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=filter_door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    filter_door = object_model.get_part("filter_door")
    door_joint = object_model.get_articulation("hood_to_filter_door")
    knob_joints = [
        object_model.get_articulation("hood_to_knob_0"),
        object_model.get_articulation("hood_to_knob_1"),
    ]

    ctx.check(
        "knobs use continuous rotation",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in knob_joints
        ),
        details=str(
            [
                (
                    joint.name,
                    str(joint.articulation_type),
                    None if joint.motion_limits is None else joint.motion_limits.lower,
                    None if joint.motion_limits is None else joint.motion_limits.upper,
                )
                for joint in knob_joints
            ]
        ),
    )

    with ctx.pose({door_joint: 0.0}):
        rest_panel_aabb = ctx.part_element_world_aabb(filter_door, elem="filter_panel")
        rest_pull_aabb = ctx.part_element_world_aabb(filter_door, elem="pull_lip")
        ctx.check(
            "filter door remains under the canopy front",
            rest_pull_aabb is not None and rest_pull_aabb[1][1] < BODY_FRONT_Y - 0.02,
            details=f"pull_lip_aabb={rest_pull_aabb}",
        )
        ctx.check(
            "filter door stays centered within hood width",
            rest_panel_aabb is not None
            and rest_panel_aabb[0][0] > -HOOD_WIDTH / 2.0
            and rest_panel_aabb[1][0] < HOOD_WIDTH / 2.0,
            details=f"filter_panel_aabb={rest_panel_aabb}",
        )

    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_pull_aabb = ctx.part_element_world_aabb(filter_door, elem="pull_lip")

    ctx.check(
        "filter door swings downward",
        rest_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[0][2] < rest_pull_aabb[0][2] - 0.14,
        details=f"rest={rest_pull_aabb}, open={open_pull_aabb}",
    )
    ctx.check(
        "filter door pivots from the rear edge",
        rest_pull_aabb is not None
        and open_pull_aabb is not None
        and open_pull_aabb[1][1] < rest_pull_aabb[1][1] - 0.05,
        details=f"rest={rest_pull_aabb}, open={open_pull_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
