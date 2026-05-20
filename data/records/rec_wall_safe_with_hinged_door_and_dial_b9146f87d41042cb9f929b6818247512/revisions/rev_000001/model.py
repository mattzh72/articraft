from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DOOR_WIDTH = 0.54
DOOR_HEIGHT = 0.54
DOOR_THICKNESS = 0.055
DOOR_CENTER_X = 0.300
DOOR_CENTER_Y = -0.015
DOOR_FRONT_Y = DOOR_CENTER_Y + DOOR_THICKNESS / 2.0

DIAL_CENTER = (DOOR_CENTER_X, 0.023, 0.105)
HANDLE_CENTER = (DOOR_CENTER_X, 0.024, -0.115)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_along_y_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_recessed_wall_safe")

    plaster = model.material("warm_plaster", color=(0.62, 0.61, 0.57, 1.0))
    frame_metal = model.material("blackened_frame_steel", color=(0.035, 0.038, 0.040, 1.0))
    door_steel = model.material("charcoal_door_steel", color=(0.12, 0.13, 0.14, 1.0))
    hinge_steel = model.material("oiled_hinge_steel", color=(0.07, 0.075, 0.078, 1.0))
    dial_black = model.material("satin_black_dial", color=(0.015, 0.015, 0.014, 1.0))
    polished = model.material("brushed_polished_steel", color=(0.68, 0.70, 0.69, 1.0))
    white_mark = model.material("white_enamel_marks", color=(0.92, 0.90, 0.82, 1.0))

    frame = model.part("frame")

    wall_ring = _cq_box((0.96, 0.040, 0.96), (0.0, 0.010, 0.0)).cut(
        _cq_box((0.72, 0.060, 0.72), (0.0, 0.010, 0.0))
    )
    frame.visual(
        mesh_from_cadquery(wall_ring, "wall_surround", tolerance=0.0015),
        material=plaster,
        name="wall_surround",
    )

    frame_shell = _cq_box((0.76, 0.140, 0.76), (0.0, -0.045, 0.0)).cut(
        _cq_box((0.62, 0.130, 0.62), (0.0, -0.030, 0.0))
    )
    frame.visual(
        mesh_from_cadquery(frame_shell, "frame_shell", tolerance=0.001),
        material=frame_metal,
        name="frame_shell",
    )

    # Fixed hinge knuckles and leaves on the frame side of the vertical pin line.
    for suffix, zc in (("lower", -0.175), ("upper", 0.175)):
        frame.visual(
            Cylinder(radius=0.014, length=0.150),
            origin=Origin(xyz=(-0.285, -0.006, zc)),
            material=hinge_steel,
            name=f"fixed_knuckle_{suffix}",
        )
        frame.visual(
            Box((0.040, 0.008, 0.150)),
            origin=Origin(xyz=(-0.309, -0.006, zc)),
            material=hinge_steel,
            name=f"fixed_leaf_{suffix}",
        )

    door = model.part("door")
    door_plate = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)
        .edges()
        .chamfer(0.006)
        .translate((DOOR_CENTER_X, DOOR_CENTER_Y, 0.0))
    )
    door.visual(
        mesh_from_cadquery(door_plate, "door_plate", tolerance=0.001),
        material=door_steel,
        name="door_plate",
    )
    door.visual(
        Box((0.460, 0.006, 0.460)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_FRONT_Y + 0.002, 0.0)),
        material=model.material("slightly_lighter_plate_face", color=(0.16, 0.17, 0.18, 1.0)),
        name="front_plate_face",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_steel,
        name="door_knuckle",
    )
    door.visual(
        Box((0.048, 0.008, 0.200)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=hinge_steel,
        name="door_leaf",
    )

    # Short fixed shafts visibly mount the independently rotating controls.
    dial_shaft_len = DIAL_CENTER[1] - (DOOR_FRONT_Y - 0.001)
    door.visual(
        Cylinder(radius=0.010, length=dial_shaft_len),
        origin=_cylinder_along_y_origin(
            DIAL_CENTER[0],
            (DIAL_CENTER[1] + DOOR_FRONT_Y - 0.001) / 2.0,
            DIAL_CENTER[2],
        ),
        material=polished,
        name="dial_shaft",
    )
    handle_shaft_len = HANDLE_CENTER[1] - (DOOR_FRONT_Y - 0.001)
    door.visual(
        Cylinder(radius=0.016, length=handle_shaft_len),
        origin=_cylinder_along_y_origin(
            HANDLE_CENTER[0],
            (HANDLE_CENTER[1] + DOOR_FRONT_Y - 0.001) / 2.0,
            HANDLE_CENTER[2],
        ),
        material=polished,
        name="handle_shaft",
    )

    # Combination-dial scale ticks are raised enamel marks on the door plate.
    tick_radius = 0.077
    for i in range(20):
        theta = 2.0 * math.pi * i / 20.0
        tick_len = 0.016 if i % 5 == 0 else 0.010
        door.visual(
            Box((tick_len, 0.003, 0.003)),
            origin=Origin(
                xyz=(
                    DIAL_CENTER[0] + tick_radius * math.cos(theta),
                    DOOR_FRONT_Y + 0.0025,
                    DIAL_CENTER[2] + tick_radius * math.sin(theta),
                ),
                rpy=(0.0, -theta, 0.0),
            ),
            material=white_mark,
            name=f"dial_tick_{i}",
        )

    dial = model.part("dial")
    dial_body = KnobGeometry(
        0.118,
        0.038,
        body_style="cylindrical",
        edge_radius=0.001,
        grip=KnobGrip(style="knurled", count=54, depth=0.0010, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=True,
    )
    dial.visual(
        mesh_from_geometry(dial_body, "dial_body"),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_body",
    )
    dial.visual(
        Box((0.006, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, 0.039, 0.020)),
        material=white_mark,
        name="dial_indicator",
    )

    handle = model.part("spoke_handle")
    handle.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=_cylinder_along_y_origin(0.0, 0.025, 0.0),
        material=polished,
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=_cylinder_along_y_origin(0.0, 0.057, 0.0),
        material=polished,
        name="front_cap",
    )
    spoke_radius = 0.115
    spoke_len = 0.125
    for i, theta in enumerate((math.pi / 2.0, 7.0 * math.pi / 6.0, 11.0 * math.pi / 6.0)):
        cx = 0.055 * math.cos(theta)
        cz = 0.055 * math.sin(theta)
        ex = spoke_radius * math.cos(theta)
        ez = spoke_radius * math.sin(theta)
        handle.visual(
            Box((spoke_len, 0.018, 0.018)),
            origin=Origin(xyz=(cx, 0.049, cz), rpy=(0.0, -theta, 0.0)),
            material=polished,
            name=f"spoke_{i}",
        )
        handle.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(ex, 0.049, ez)),
            material=polished,
            name=f"spoke_knob_{i}",
        )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.285, -0.006, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=DIAL_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )
    model.articulation(
        "door_to_spoke_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=HANDLE_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("spoke_handle")
    door_hinge = object_model.get_articulation("frame_to_door")
    dial_joint = object_model.get_articulation("door_to_dial")
    handle_joint = object_model.get_articulation("door_to_spoke_handle")

    ctx.check(
        "combination dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    handle_limits = handle_joint.motion_limits
    ctx.check(
        "spoke handle has finite rotary stops",
        handle_joint.articulation_type == ArticulationType.REVOLUTE
        and handle_limits is not None
        and handle_limits.lower == 0.0
        and handle_limits.upper is not None
        and handle_limits.upper >= 1.5,
        details=f"type={handle_joint.articulation_type}, limits={handle_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_within(
            door,
            frame,
            axes="xz",
            inner_elem="door_plate",
            outer_elem="frame_shell",
            margin=0.0,
            name="closed door plate sits inside square frame",
        )
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_body",
            elem_b="dial_shaft",
            contact_tol=0.002,
            name="dial is seated on its center shaft",
        )
        ctx.expect_contact(
            handle,
            door,
            elem_a="handle_hub",
            elem_b="handle_shaft",
            contact_tol=0.002,
            name="spoke handle is seated on its shaft",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_plate")
    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_plate")
    ctx.check(
        "door swings outward on vertical side hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.15,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
