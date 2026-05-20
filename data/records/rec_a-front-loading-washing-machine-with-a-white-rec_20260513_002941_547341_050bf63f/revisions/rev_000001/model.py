from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="washing_machine")
    
    # Materials
    white_plastic = Material("white_plastic", rgba=(0.9, 0.9, 0.9, 1.0))
    dark_gray = Material("dark_gray", rgba=(0.3, 0.3, 0.3, 1.0))
    glass = Material("glass", rgba=(0.8, 0.9, 0.9, 0.5))
    metal = Material("metal", rgba=(0.7, 0.7, 0.7, 1.0))
    black_plastic = Material("black_plastic", rgba=(0.1, 0.1, 0.1, 1.0))
    
    # 1. Cabinet
    cabinet_shape = (
        cq.Workplane("XY").box(0.6, 0.6, 0.85).translate((0, 0, 0.425))
        .cut(
            cq.Workplane("YZ").workplane(offset=0.3)
            .center(0, 0.4)
            .circle(0.22)
            .extrude(-0.5)
        )
        .cut(
            cq.Workplane("YZ").workplane(offset=0.3)
            .center(0.15, 0.775)
            .rect(0.205, 0.105)
            .extrude(-0.25)
        )
    )
    
    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(cabinet_shape, "cabinet_mesh"),
        material=white_plastic,
        name="cabinet_shell"
    )
    
    # 2. Drum
    drum_shape = (
        cq.Workplane("YZ").workplane(offset=0.24)
        .center(0, 0.4)
        .circle(0.21)
        .extrude(-0.39)
        .cut(
            cq.Workplane("YZ").workplane(offset=0.24)
            .center(0, 0.4)
            .circle(0.20)
            .extrude(-0.38)
        )
        .union(
            cq.Workplane("YZ").workplane(offset=-0.15)
            .center(0, 0.4)
            .circle(0.02)
            .extrude(-0.055)
        )
    ).translate((-0.05, 0.0, -0.4))
    
    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(drum_shape, "drum_mesh"),
        material=metal,
        name="drum_shell"
    )
    
    model.articulation(
        "drum_axle",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.05, 0.0, 0.4)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0)
    )
    
    # 3. Door
    door_frame_shape = (
        cq.Workplane("YZ").workplane(offset=0.34)
        .center(0, 0.4)
        .circle(0.24)
        .extrude(-0.045)
        .cut(
            cq.Workplane("YZ").workplane(offset=0.34)
            .center(0, 0.4)
            .circle(0.15)
            .extrude(-0.05)
        )
    ).translate((-0.3, -0.24, -0.4))
    
    door_glass_shape = (
        cq.Workplane("YZ").workplane(offset=0.31)
        .center(0, 0.4)
        .circle(0.15)
        .extrude(-0.05)
    ).translate((-0.3, -0.24, -0.4))
    
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(door_frame_shape, "door_frame_mesh"),
        material=dark_gray,
        name="door_frame"
    )
    door.visual(
        mesh_from_cadquery(door_glass_shape, "door_glass_mesh"),
        material=glass,
        name="door_glass"
    )
    
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.3, 0.24, 0.4)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.5)
    )
    
    # 4. Detergent Drawer
    drawer_shape = (
        cq.Workplane("YZ").workplane(offset=0.305)
        .center(0.15, 0.775)
        .rect(0.22, 0.12)
        .extrude(-0.015)
        .union(
            cq.Workplane("YZ").workplane(offset=0.295)
            .center(0.15, 0.775)
            .rect(0.18, 0.08)
            .extrude(-0.23)
        )
    ).translate((-0.3, -0.15, -0.775))
    
    drawer = model.part("detergent_drawer")
    drawer.visual(
        mesh_from_cadquery(drawer_shape, "drawer_mesh"),
        material=white_plastic,
        name="drawer_body"
    )
    
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.3, 0.15, 0.775)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.2)
    )
    
    # 5. Knob
    knob_shape = (
        cq.Workplane("YZ").workplane(offset=0.32)
        .center(-0.15, 0.775)
        .circle(0.03)
        .extrude(-0.025)
    ).translate((-0.3, 0.15, -0.775))
    
    knob = model.part("knob")
    knob.visual(
        mesh_from_cadquery(knob_shape, "knob_mesh"),
        material=black_plastic,
        name="knob_body"
    )
    
    model.articulation(
        "knob_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.3, -0.15, 0.775)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("cabinet", "detergent_drawer", reason="Drawer slides inside the cabinet recess and front panel seats against it.")
    ctx.allow_overlap("cabinet", "door", reason="Door frame seats closely against the cabinet front face.")
    ctx.allow_overlap("cabinet", "knob", reason="Knob seats into the cabinet face.")
    ctx.allow_overlap("cabinet", "drum", reason="Drum axle connects to cabinet back wall.")
    
    # Assertions
    cabinet = object_model.get_part("cabinet")
    drawer = object_model.get_part("detergent_drawer")
    door = object_model.get_part("door")
    
    ctx.expect_within(drawer, cabinet, axes="yz", name="Drawer remains within cabinet on YZ")
    ctx.expect_contact(door, cabinet, name="Door touches cabinet when closed")
    
    return ctx.report()

object_model = build_object_model()