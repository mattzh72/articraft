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
    model = ArticulatedObject(name="wall_safe")

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.17, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    black_bakelite = model.material("black_bakelite", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")

    # Compact recessed safe body with a square front frame and visible open cavity.
    body.visual(
        Box((0.300, 0.008, 0.300)),
        origin=Origin(xyz=(0.0, -0.176, 0.0)),
        material=cavity_dark,
        name="back_panel",
    )
    body.visual(
        Box((0.012, 0.186, 0.300)),
        origin=Origin(xyz=(-0.144, -0.085, 0.0)),
        material=painted_steel,
        name="left_wall",
    )
    body.visual(
        Box((0.012, 0.186, 0.300)),
        origin=Origin(xyz=(0.144, -0.085, 0.0)),
        material=painted_steel,
        name="right_wall",
    )
    body.visual(
        Box((0.276, 0.186, 0.012)),
        origin=Origin(xyz=(0.0, -0.085, 0.144)),
        material=painted_steel,
        name="top_wall",
    )
    body.visual(
        Box((0.276, 0.186, 0.012)),
        origin=Origin(xyz=(0.0, -0.085, -0.144)),
        material=painted_steel,
        name="bottom_wall",
    )
    body.visual(
        Box((0.340, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.007, 0.155)),
        material=painted_steel,
        name="frame_top",
    )
    body.visual(
        Box((0.340, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.007, -0.155)),
        material=painted_steel,
        name="frame_bottom",
    )
    body.visual(
        Box((0.030, 0.014, 0.280)),
        origin=Origin(xyz=(-0.155, 0.007, 0.0)),
        material=painted_steel,
        name="frame_left",
    )
    body.visual(
        Box((0.030, 0.014, 0.280)),
        origin=Origin(xyz=(0.155, 0.007, 0.0)),
        material=painted_steel,
        name="frame_right",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(0.146, 0.007, 0.090)),
        material=brushed_metal,
        name="hinge_barrel_upper",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(0.146, 0.007, 0.000)),
        material=brushed_metal,
        name="hinge_barrel_center",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.072),
        origin=Origin(xyz=(0.146, 0.007, -0.090)),
        material=brushed_metal,
        name="hinge_barrel_lower",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.190, 0.340)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -0.081, 0.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.276, 0.038, 0.276)),
        origin=Origin(xyz=(-0.138, 0.0, 0.0)),
        material=dark_steel,
        name="door_slab",
    )
    door.visual(
        Box((0.248, 0.006, 0.248)),
        origin=Origin(xyz=(-0.138, 0.022, 0.0)),
        material=painted_steel,
        name="door_face_plate",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.248),
        origin=Origin(xyz=(-0.010, -0.003, 0.0)),
        material=brushed_metal,
        name="door_hinge_spine",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.276, 0.038, 0.276)),
        mass=7.5,
        origin=Origin(xyz=(-0.138, 0.0, 0.0)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.045, length=0.003),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="dial_backplate",
    )
    dial.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_bakelite,
        name="dial_center_cap",
    )
    dial.visual(
        Box((0.008, 0.008, 0.014)),
        origin=Origin(xyz=(0.024, 0.022, 0.0)),
        material=black_bakelite,
        name="dial_grip_ridge",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.030),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Box((0.030, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        material=dark_steel,
        name="lever_bracket",
    )
    latch_lever.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="lever_pivot",
    )
    latch_lever.visual(
        Box((0.016, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, 0.022, -0.026)),
        material=dark_steel,
        name="lever_handle",
    )
    latch_lever.inertial = Inertial.from_geometry(
        Box((0.030, 0.026, 0.060)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.013, -0.018)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.138, -0.009, 0.0)),
        # Closed door geometry extends toward local -X from the right hinge line.
        # -Z makes positive motion swing the free edge outward toward +Y.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.138, 0.019, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    model.articulation(
        "door_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_lever,
        origin=Origin(xyz=(-0.138, 0.025, -0.098)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=math.radians(-15.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    latch_lever = object_model.get_part("latch_lever")

    body_to_door = object_model.get_articulation("body_to_door")
    door_to_dial = object_model.get_articulation("door_to_dial")
    door_to_latch_lever = object_model.get_articulation("door_to_latch_lever")

    ctx.check(
        "articulations use the intended types and axes",
        body_to_door.articulation_type == ArticulationType.REVOLUTE
        and door_to_dial.articulation_type == ArticulationType.CONTINUOUS
        and door_to_latch_lever.articulation_type == ArticulationType.REVOLUTE
        and body_to_door.axis == (0.0, 0.0, -1.0)
        and door_to_dial.axis == (0.0, 1.0, 0.0)
        and door_to_latch_lever.axis == (1.0, 0.0, 0.0),
        details=(
            f"door=({body_to_door.articulation_type}, {body_to_door.axis}), "
            f"dial=({door_to_dial.articulation_type}, {door_to_dial.axis}), "
            f"lever=({door_to_latch_lever.articulation_type}, {door_to_latch_lever.axis})"
        ),
    )

    with ctx.pose({body_to_door: 0.0, door_to_latch_lever: 0.0}):
        ctx.expect_contact(
            dial,
            door,
            elem_a="dial_backplate",
            elem_b="door_face_plate",
            name="dial backplate mounts flush to the door face",
        )
        ctx.expect_contact(
            latch_lever,
            door,
            elem_a="lever_bracket",
            elem_b="door_face_plate",
            name="latch lever bracket mounts to the lower door face",
        )
        ctx.expect_overlap(
            dial,
            door,
            axes="xz",
            elem_a="dial_backplate",
            elem_b="door_face_plate",
            min_overlap=0.050,
            name="dial sits centrally within the door face footprint",
        )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({body_to_door: body_to_door.motion_limits.upper}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")

    door_swings_out = (
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10
    )
    ctx.check(
        "right-hinged door opens outward",
        door_swings_out,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    lever_lower = door_to_latch_lever.motion_limits.lower
    lever_upper = door_to_latch_lever.motion_limits.upper
    lever_down_aabb = None
    lever_up_aabb = None
    with ctx.pose({door_to_latch_lever: lever_lower}):
        lever_down_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_handle")
    with ctx.pose({door_to_latch_lever: lever_upper}):
        lever_up_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_handle")

    lever_lifts = (
        lever_down_aabb is not None
        and lever_up_aabb is not None
        and ((lever_up_aabb[0][2] + lever_up_aabb[1][2]) * 0.5)
        > ((lever_down_aabb[0][2] + lever_down_aabb[1][2]) * 0.5) + 0.01
    )
    ctx.check(
        "latch lever rotates upward about its transverse pivot",
        lever_lifts,
        details=f"down={lever_down_aabb}, up={lever_up_aabb}",
    )

    dial_start_aabb = None
    dial_quarter_turn_aabb = None
    with ctx.pose({door_to_dial: 0.0}):
        dial_start_aabb = ctx.part_element_world_aabb(dial, elem="dial_grip_ridge")
    with ctx.pose({door_to_dial: math.pi / 2.0}):
        dial_quarter_turn_aabb = ctx.part_element_world_aabb(dial, elem="dial_grip_ridge")

    dial_turns = (
        dial_start_aabb is not None
        and dial_quarter_turn_aabb is not None
        and abs(
            ((dial_quarter_turn_aabb[0][0] + dial_quarter_turn_aabb[1][0]) * 0.5)
            - ((dial_start_aabb[0][0] + dial_start_aabb[1][0]) * 0.5)
        )
        > 0.01
    )
    ctx.check(
        "combination dial rotates about its face-normal axis",
        dial_turns,
        details=f"start={dial_start_aabb}, quarter_turn={dial_quarter_turn_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
