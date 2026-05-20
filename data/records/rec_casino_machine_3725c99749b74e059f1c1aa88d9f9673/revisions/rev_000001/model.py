import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slant_top_casino_machine")

    # 1. Base Cabinet Shell
    # Profile in YZ plane. X is right, Y is forward, Z is up.
    # Player is at -Y, looking at +Y.
    profile = (
        cq.Workplane("YZ")
        .moveTo(-0.3, 0.0)
        .lineTo(0.4, 0.0)
        .lineTo(0.4, 1.4)
        .lineTo(0.3, 1.4)
        .lineTo(0.0, 0.65)
        .lineTo(-0.3, 0.5)
        .close()
    )
    # Extrude along X (width 0.6). Center on X=0.
    cabinet_shape = profile.extrude(0.6).translate((-0.3, 0, 0)).shell(-0.02)

    # Cut belly opening
    belly_cut = cq.Workplane("XY").box(0.5, 0.2, 0.4).translate((0, -0.3, 0.25))
    cabinet_shape = cabinet_shape.cut(belly_cut)

    # Deck angle and normal
    deck_angle = math.atan2(0.15, 0.3) # ~26.56 deg
    deck_center = (0, -0.15, 0.575)

    # Cut button hole in deck
    button_hole = (
        cq.Workplane("XY")
        .circle(0.03)
        .extrude(0.1)
        .translate((0, 0, -0.05))
        .rotate((0,0,0), (1,0,0), math.degrees(deck_angle))
        .translate(deck_center)
    )
    cabinet_shape = cabinet_shape.cut(button_hole)

    # Screen angle and normal
    screen_angle = math.atan2(0.75, 0.3) # ~68.2 deg
    knob_center = (0.2, 0.05, 0.775)

    # Cut volume knob hole
    knob_hole = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(0.1)
        .translate((0, 0, -0.05))
        .rotate((0,0,0), (1,0,0), math.degrees(screen_angle))
        .translate(knob_center)
    )
    cabinet_shape = cabinet_shape.cut(knob_hole)

    base = model.part("base_cabinet")
    base.visual(mesh_from_cadquery(cabinet_shape, "cabinet_shell"), name="cabinet_shell")

    # Add Screen Visual
    screen_geom = cq.Workplane("XY").box(0.5, 0.6, 0.01).translate((0, 0, 0.005))
    base.visual(
        mesh_from_cadquery(screen_geom, "screen_panel"),
        origin=Origin(xyz=(0, 0.15, 1.025), rpy=(screen_angle, 0, 0)),
        name="screen_panel"
    )

    # Add Sleeve Visual to Base
    sleeve_geom = (
        cq.Workplane("XY")
        .circle(0.04).circle(0.03).extrude(0.02)
    )
    deck_origin = Origin(xyz=deck_center, rpy=(deck_angle, 0, 0))
    base.visual(mesh_from_cadquery(sleeve_geom, "spin_button_sleeve"), origin=deck_origin, name="spin_button_sleeve")

    # 2. Belly Door
    belly_door = model.part("belly_door")
    # Hinge at X=0.25, Y=-0.3, Z=0.25
    door_geom = cq.Workplane("XY").box(0.488, 0.018, 0.39).translate((-0.244, 0.01, 0))
    belly_door.visual(mesh_from_cadquery(door_geom, "door_panel"), name="door_panel")
    
    lock_geom = cq.Workplane("XY").circle(0.01).extrude(0.005)
    belly_door.visual(
        mesh_from_cadquery(lock_geom, "door_lock"),
        origin=Origin(xyz=(-0.45, 0.0, 0.1), rpy=(-math.pi/2, 0, 0)),
        name="door_lock"
    )

    model.articulation(
        "belly_door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=belly_door,
        origin=Origin(xyz=(0.25, -0.3, 0.25)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=math.pi/2)
    )

    # 3. Spin Button
    spin_button = model.part("spin_button")
    button_geom = (
        cq.Workplane("XY").circle(0.028).extrude(0.02)
        .add(cq.Workplane("XY").circle(0.03).extrude(-0.05))
    )
    spin_button.visual(
        mesh_from_cadquery(button_geom, "button_cap"),
        origin=Origin(xyz=(0, 0, 0.005)),
        name="button_cap"
    )

    model.articulation(
        "spin_button_press",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spin_button,
        origin=deck_origin,
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.005, upper=0.0)
    )

    # 4. Volume Knob
    volume_knob = model.part("volume_knob")
    knob_geom = (
        cq.Workplane("XY")
        .circle(0.015).extrude(0.015)
        .faces(">Z").workplane().circle(0.01).extrude(0.005)
    ).translate((0, 0, 0.01))
    shaft_geom = cq.Workplane("XY").circle(0.005).extrude(0.035).translate((0, 0, -0.025))
    
    volume_knob.visual(mesh_from_cadquery(knob_geom, "knob_cap"), name="knob_cap")
    volume_knob.visual(mesh_from_cadquery(shaft_geom, "knob_shaft"), name="knob_shaft")

    knob_origin = Origin(xyz=knob_center, rpy=(screen_angle, 0, 0))
    model.articulation(
        "volume_turn",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=volume_knob,
        origin=knob_origin,
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base_cabinet")
    door = object_model.get_part("belly_door")
    button = object_model.get_part("spin_button")
    knob = object_model.get_part("volume_knob")

    # Allow intentional overlaps for parts fitting in holes
    ctx.allow_overlap(
        base, door,
        elem_a="cabinet_shell", elem_b="door_panel",
        reason="The belly door sits inside the opening cut into the cabinet shell."
    )
    ctx.allow_overlap(
        base, button,
        elem_a="cabinet_shell", elem_b="button_cap",
        reason="The spin button stem fits into the hole in the cabinet deck."
    )
    ctx.allow_overlap(
        base, button,
        elem_a="spin_button_sleeve", elem_b="button_cap",
        reason="The spin button stem translates inside the sleeve."
    )
    ctx.allow_overlap(
        base, knob,
        elem_a="cabinet_shell", elem_b="knob_shaft",
        reason="The volume knob shaft penetrates the cabinet shell."
    )
    ctx.allow_overlap(
        base, knob,
        elem_a="screen_panel", elem_b="knob_shaft",
        reason="The volume knob shaft penetrates the screen panel."
    )

    # Check button movement
    press_joint = object_model.get_articulation("spin_button_press")
    button_pos_up = ctx.part_world_position(button)
    with ctx.pose({press_joint: -0.005}):
        button_pos_down = ctx.part_world_position(button)
    if button_pos_up and button_pos_down:
        ctx.check("button_presses_down", button_pos_down[2] < button_pos_up[2], "Button should move down when pressed.")

    # Check door hinge
    door_joint = object_model.get_articulation("belly_door_hinge")
    door_aabb_closed = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.0}):
        door_aabb_open = ctx.part_world_aabb(door)
    if door_aabb_closed and door_aabb_open:
        # The center of the door's AABB should move to a more negative Y
        closed_center_y = (door_aabb_closed[0][1] + door_aabb_closed[1][1]) / 2
        open_center_y = (door_aabb_open[0][1] + door_aabb_open[1][1]) / 2
        ctx.check("door_opens_outward", open_center_y < closed_center_y, "Door should swing outward (negative Y).")

    return ctx.report()

object_model = build_object_model()