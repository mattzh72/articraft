from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DOOR_DISPLAY_YAW = -1.12


def _rz_xy(x: float, y: float, yaw: float = DOOR_DISPLAY_YAW) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _door_pose_origin(
    x: float,
    y: float,
    z: float,
    *,
    yaw: float = DOOR_DISPLAY_YAW,
    roll: float = 0.0,
    pitch: float = 0.0,
) -> Origin:
    rx, ry = _rz_xy(x, y, yaw)
    return Origin(xyz=(rx, ry, z), rpy=(roll, pitch, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_wall_safe")

    gunmetal = model.material("gunmetal", rgba=(0.10, 0.12, 0.14, 1.0))
    dark_cavity = model.material("dark_cavity", rgba=(0.025, 0.028, 0.030, 1.0))
    door_paint = model.material("door_paint", rgba=(0.075, 0.095, 0.115, 1.0))
    black = model.material("black", rgba=(0.005, 0.005, 0.006, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    paper_label = model.material("paper_label", rgba=(0.82, 0.78, 0.64, 1.0))

    case = model.part("case")

    front_frame = BezelGeometry(
        opening_size=(0.270, 0.700),
        outer_size=(0.400, 0.900),
        depth=0.045,
        opening_shape="rect",
        outer_shape="rect",
        center=False,
    )
    case.visual(
        mesh_from_geometry(front_frame, "front_frame"),
        origin=Origin(xyz=(0.0, 0.0, 0.450), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="front_frame",
    )

    # Four real side walls and a rear plate make the safe read as a hollow wall
    # cavity rather than a solid block with a painted rectangle.
    case.visual(
        Box((0.050, 0.215, 0.740)),
        origin=Origin(xyz=(-0.160, 0.1075, 0.450)),
        material=dark_cavity,
        name="left_wall",
    )
    case.visual(
        Box((0.050, 0.215, 0.740)),
        origin=Origin(xyz=(0.160, 0.1075, 0.450)),
        material=dark_cavity,
        name="right_wall",
    )
    case.visual(
        Box((0.370, 0.215, 0.050)),
        origin=Origin(xyz=(0.0, 0.1075, 0.825)),
        material=dark_cavity,
        name="top_wall",
    )
    case.visual(
        Box((0.370, 0.215, 0.050)),
        origin=Origin(xyz=(0.0, 0.1075, 0.075)),
        material=dark_cavity,
        name="bottom_wall",
    )
    case.visual(
        Box((0.370, 0.016, 0.740)),
        origin=Origin(xyz=(0.0, 0.223, 0.450)),
        material=dark_cavity,
        name="back_panel",
    )
    case.visual(
        Box((0.014, 0.032, 0.830)),
        origin=Origin(xyz=(-0.205, -0.052, 0.450)),
        material=satin_steel,
        name="hinge_jamb",
    )

    door = model.part("door")
    door_width = 0.340
    door_thickness = 0.055
    door_height = 0.780
    door_center = _door_pose_origin(door_width / 2.0, 0.0, 0.0)
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=door_center,
        material=door_paint,
        name="door_slab",
    )
    door.visual(
        Cylinder(radius=0.014, length=door_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.300, 0.004, 0.020)),
        origin=_door_pose_origin(0.170, -0.0295, 0.255),
        material=satin_steel,
        name="top_trim",
    )
    door.visual(
        Box((0.300, 0.004, 0.020)),
        origin=_door_pose_origin(0.170, -0.0295, -0.255),
        material=satin_steel,
        name="bottom_trim",
    )
    door.visual(
        Cylinder(radius=0.064, length=0.003),
        origin=_door_pose_origin(0.200, -0.0290, 0.095, roll=math.pi / 2.0),
        material=satin_steel,
        name="dial_scale",
    )
    # Short live-bolt noses on the latch edge make the open slab read as a safe
    # door. They are fixed here because the prompt's moving user controls are
    # the hinge, dial, and document flap.
    for idx, z in enumerate((-0.205, 0.0, 0.205)):
        door.visual(
            Box((0.038, 0.030, 0.026)),
            origin=_door_pose_origin(door_width + 0.019, 0.0, z),
            material=satin_steel,
            name=f"bolt_{idx}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.085,
                0.030,
                body_style="cylindrical",
                edge_radius=0.002,
                grip=KnobGrip(style="knurled", count=48, depth=0.0012, helix_angle_deg=18.0),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "combination_dial",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=black,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=black,
        name="dial_hub",
    )

    flap = model.part("document_flap")
    flap.visual(
        Box((0.215, 0.010, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=paper_label,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.007, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="flap_hinge",
    )
    flap.visual(
        Box((0.095, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, -0.092)),
        material=satin_steel,
        name="finger_lip",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(-0.185, -0.072, 0.450)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=DOOR_DISPLAY_YAW, upper=0.45),
    )
    model.articulation(
        "dial_axis",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=_door_pose_origin(0.200, -0.0305, 0.095, roll=math.pi / 2.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=flap,
        origin=Origin(xyz=(0.0, 0.020, 0.793)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    flap = object_model.get_part("document_flap")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_axis = object_model.get_articulation("dial_axis")
    flap_hinge = object_model.get_articulation("flap_hinge")

    ctx.check(
        "door hinge is vertical and left mounted",
        door_hinge.axis == (0.0, 0.0, -1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower < -1.0
        and door_hinge.motion_limits.upper > 0.4,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "dial is continuous",
        str(dial_axis.articulation_type).lower().endswith("continuous"),
        details=f"type={dial_axis.articulation_type}",
    )
    ctx.check(
        "document flap has a horizontal top hinge",
        flap_hinge.axis == (1.0, 0.0, 0.0)
        and flap_hinge.motion_limits is not None
        and flap_hinge.motion_limits.upper >= 1.0,
        details=f"axis={flap_hinge.axis}, limits={flap_hinge.motion_limits}",
    )

    ctx.expect_within(
        "document_flap",
        "case",
        axes="x",
        inner_elem="flap_panel",
        outer_elem="front_frame",
        margin=0.0,
        name="document flap fits inside the narrow opening width",
    )
    ctx.expect_overlap(
        "dial",
        "door",
        axes="xz",
        elem_a="dial_knob",
        elem_b="dial_scale",
        min_overlap=0.025,
        name="combination dial is centered on its escutcheon",
    )

    with ctx.pose({door_hinge: DOOR_DISPLAY_YAW}):
        ctx.expect_overlap(
            door,
            case,
            axes="xz",
            elem_a="door_slab",
            elem_b="front_frame",
            min_overlap=0.250,
            name="closed door covers the tall safe opening",
        )
        ctx.expect_gap(
            case,
            door,
            axis="y",
            positive_elem="front_frame",
            negative_elem="door_slab",
            max_gap=0.010,
            max_penetration=0.001,
            name="closed door seats just in front of the frame",
        )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 1.0}):
        raised_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    if closed_flap_aabb is None or raised_flap_aabb is None:
        ctx.fail("flap pose can be measured", "missing flap panel AABB")
    else:
        closed_y = (closed_flap_aabb[0][1] + closed_flap_aabb[1][1]) / 2.0
        raised_y = (raised_flap_aabb[0][1] + raised_flap_aabb[1][1]) / 2.0
        ctx.check(
            "document flap swings inward",
            raised_y > closed_y + 0.025,
            details=f"closed_y={closed_y:.4f}, raised_y={raised_y:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
