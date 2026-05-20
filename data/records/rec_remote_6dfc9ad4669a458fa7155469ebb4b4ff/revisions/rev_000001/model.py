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
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_remote")

    body = model.part("body")
    
    body_geom = (
        cq.Workplane("XY")
        .box(0.06, 0.20, 0.02)
        .faces("-Z")
        .workplane()
        .center(0.005, 0.04)
        .rect(0.05, 0.08)
        .cutBlind(-0.005)
    )
    body.visual(
        mesh_from_cadquery(body_geom, "body_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="body_shell",
    )

    # Body hinge supports (slightly narrowed to avoid exact touching with guard)
    body.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(xyz=(-0.018, 0.01, 0.016), rpy=(0.0, 1.5708, 0.0)),
        name="body_barrel_left",
    )
    body.visual(
        Box((0.014, 0.006, 0.006)),
        origin=Origin(xyz=(-0.018, 0.01, 0.013)),
        name="body_support_left",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(xyz=(0.018, 0.01, 0.016), rpy=(0.0, 1.5708, 0.0)),
        name="body_barrel_right",
    )
    body.visual(
        Box((0.014, 0.006, 0.006)),
        origin=Origin(xyz=(0.018, 0.01, 0.013)),
        name="body_support_right",
    )

    guard = model.part("guard")
    guard.visual(
        Box((0.05, 0.065, 0.004)),
        origin=Origin(xyz=(0.0, 0.0475, -0.002)),
        name="guard_panel",
    )
    guard.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        name="guard_barrel",
    )
    guard.visual(
        Box((0.02, 0.015, 0.004)),
        origin=Origin(xyz=(0.0, 0.0075, -0.002)),
        name="guard_barrel_connector",
    )

    model.articulation(
        "body_to_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=guard,
        origin=Origin(xyz=(0.0, 0.01, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=3.14),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.048, 0.078, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="door_panel",
    )
    battery_door.visual(
        Box((0.002, 0.06, 0.001)),
        origin=Origin(xyz=(0.015, 0.0, -0.0025)),
        name="door_grip",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.005, 0.04, -0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.03),
    )

    def add_button(name, y, x, radius=0.004, length=0.0015):
        btn = model.part(name)
        btn.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, length / 2)),
            name=f"{name}_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(x, y, 0.0100)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.001, upper=0.0),
        )

    # Dish controls (under guard)
    add_button("dish_up", 0.075, 0.0)
    add_button("dish_down", 0.045, 0.0)
    add_button("dish_select", 0.060, 0.0, radius=0.005)
    add_button("dish_left", 0.060, -0.015)
    add_button("dish_right", 0.060, 0.015)

    # Lower body buttons
    add_button("power", -0.08, 0.0, radius=0.004, length=0.002)
    add_button("vol_up", -0.04, -0.015, length=0.002)
    add_button("vol_down", -0.06, -0.015, length=0.002)
    add_button("ch_up", -0.04, 0.015, length=0.002)
    add_button("ch_down", -0.06, 0.015, length=0.002)

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    body = object_model.get_part("body")
    guard = object_model.get_part("guard")
    battery_door = object_model.get_part("battery_door")
    
    # Check battery door fits in pocket
    ctx.expect_within(
        battery_door,
        body,
        axes="y",
        inner_elem="door_panel",
        outer_elem="body_shell",
        margin=0.002,
        name="battery door fits in Y pocket",
    )
    
    # Check guard closed state
    ctx.expect_gap(
        guard,
        body,
        axis="z",
        positive_elem="guard_panel",
        negative_elem="body_shell",
        min_gap=0.001,
        name="guard panel clears body when closed",
    )
    
    # Check guard open state
    guard_joint = object_model.get_articulation("body_to_guard")
    with ctx.pose({guard_joint: 3.14}):
        ctx.expect_gap(
            guard,
            body,
            axis="z",
            positive_elem="guard_panel",
            negative_elem="body_shell",
            min_gap=0.001,
            name="guard panel clears body when fully open",
        )
        
    # Check battery door sliding
    door_joint = object_model.get_articulation("body_to_battery_door")
    with ctx.pose({door_joint: 0.03}):
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            elem_a="door_panel",
            elem_b="body_shell",
            min_overlap=0.01,
            name="battery door remains inserted when slid open",
        )

    return ctx.report()

object_model = build_object_model()