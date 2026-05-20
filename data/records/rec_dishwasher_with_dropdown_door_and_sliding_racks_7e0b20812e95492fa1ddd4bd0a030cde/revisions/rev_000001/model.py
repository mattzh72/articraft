import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dishwasher")

    # Tub
    tub = model.part("tub")
    tub_cq = (
        cq.Workplane("XY")
        .box(0.60, 0.56, 0.85)
        .faces("-Y")
        .shell(-0.02)
    )
    tub.visual(
        mesh_from_cadquery(tub_cq, "tub_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        name="tub_shell",
    )
    
    # Rails for lower rack
    tub.visual(Box((0.02, 0.50, 0.02)), origin=Origin(xyz=(-0.27, 0.0, 0.15)), name="lower_rail_l")
    tub.visual(Box((0.02, 0.50, 0.02)), origin=Origin(xyz=(0.27, 0.0, 0.15)), name="lower_rail_r")
    
    # Rails for upper rack
    tub.visual(Box((0.02, 0.50, 0.02)), origin=Origin(xyz=(-0.27, 0.0, 0.55)), name="upper_rail_l")
    tub.visual(Box((0.02, 0.50, 0.02)), origin=Origin(xyz=(0.27, 0.0, 0.55)), name="upper_rail_r")
    
    # Lower hub
    tub.visual(
        Cylinder(radius=0.02, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        name="lower_hub",
    )
    
    # Upper hub pipe and hub
    tub.visual(
        Box((0.04, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, 0.13, 0.52)),
        name="upper_hub_pipe",
    )
    tub.visual(
        Cylinder(radius=0.02, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        name="upper_hub",
    )
    
    # Door Hinge Barrel on tub
    tub.visual(
        Cylinder(radius=0.01, length=0.6),
        origin=Origin(xyz=(0.0, -0.28, 0.10), rpy=(0.0, 1.5708, 0.0)),
        name="hinge_barrel",
    )

    # Door
    door = model.part("door")
    door_cq = (
        cq.Workplane("XY")
        .box(0.60, 0.05, 0.75)
        .faces("-Z")
        .shell(-0.005)
    )
    door.visual(
        mesh_from_cadquery(door_cq, "door_shell"),
        origin=Origin(xyz=(0.0, -0.025, 0.375)),
        name="door_shell",
    )
    model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.28, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )

    # Buttons
    button_names = ["start_button", "option_button_0", "option_button_1", "option_button_2"]
    button_xs = [-0.15, -0.05, 0.05, 0.15]
    for name, x_pos in zip(button_names, button_xs):
        btn = model.part(name)
        btn.visual(
            Box((0.04, 0.02, 0.01)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            name=f"{name}_visual",
        )
        model.articulation(
            f"door_to_{name}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=btn,
            origin=Origin(xyz=(x_pos, -0.025, 0.75)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.005),
        )

    # Rinse-aid cap
    cap = model.part("rinse_aid_cap")
    cap.visual(
        Cylinder(radius=0.03, length=0.01),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(1.5708, 0.0, 0.0)),
        name="cap_visual",
    )
    model.articulation(
        "door_to_rinse_aid_cap",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cap,
        origin=Origin(xyz=(0.0, -0.002, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # Racks
    rack_cq = (
        cq.Workplane("XY")
        .box(0.52, 0.50, 0.15)
        .faces("+Z")
        .shell(-0.005)
    )
    
    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        mesh_from_cadquery(rack_cq, "lower_rack_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="lower_rack_shell",
    )
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.40),
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(
        mesh_from_cadquery(rack_cq, "upper_rack_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="upper_rack_shell",
    )
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.40),
    )

    # Mug shelves
    shelf_0 = model.part("mug_shelf_0")
    shelf_0.visual(
        Box((0.10, 0.40, 0.01)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        name="shelf_0_visual",
    )
    shelf_0.visual(
        Cylinder(radius=0.005, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5708, 0.0, 0.0)),
        name="shelf_0_hinge",
    )
    model.articulation(
        "upper_rack_to_mug_shelf_0",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=shelf_0,
        origin=Origin(xyz=(-0.26, 0.0, 0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )

    shelf_1 = model.part("mug_shelf_1")
    shelf_1.visual(
        Box((0.10, 0.40, 0.01)),
        origin=Origin(xyz=(-0.05, 0.0, 0.0)),
        name="shelf_1_visual",
    )
    shelf_1.visual(
        Cylinder(radius=0.005, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5708, 0.0, 0.0)),
        name="shelf_1_hinge",
    )
    model.articulation(
        "upper_rack_to_mug_shelf_1",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=shelf_1,
        origin=Origin(xyz=(0.26, 0.0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708),
    )

    # Spray arms
    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(
        Box((0.40, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="lower_spray_arm_visual",
    )
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(
        Box((0.40, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="upper_spray_arm_visual",
    )
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("tub", "door", reason="Door hinge barrel embeds into the door to form the pivot joint.")
    ctx.allow_overlap("door", "rinse_aid_cap", reason="Rinse aid cap embeds into the door liner.")
    ctx.allow_overlap("upper_rack", "mug_shelf_0", reason="Mug shelf hinge embeds into the rack wireframe.")
    ctx.allow_overlap("upper_rack", "mug_shelf_1", reason="Mug shelf hinge embeds into the rack wireframe.")
    
    for btn in ["start_button", "option_button_0", "option_button_1", "option_button_2"]:
        ctx.allow_overlap("door", btn, reason="Buttons embed into the door when pressed.")

    # Some basic checks
    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    
    ctx.expect_contact(tub, door, name="Door hinge is in contact with tub")

    door_hinge = object_model.get_articulation("tub_to_door")
    with ctx.pose({door_hinge: 1.5}):
        door_aabb = ctx.part_world_aabb(door)
        ctx.check("door_drops_down", door_aabb is not None and door_aabb[0][1] < -0.8, "Door should open forward")

    upper_rack = object_model.get_part("upper_rack")
    upper_slide = object_model.get_articulation("tub_to_upper_rack")
    with ctx.pose({upper_slide: 0.3}):
        pos = ctx.part_world_position(upper_rack)
        ctx.check("upper_rack_slides_out", pos is not None and pos[1] < -0.2, "Rack should slide forward")

    return ctx.report()

object_model = build_object_model()