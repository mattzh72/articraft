import cadquery as cq
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


def build_base_body():
    plate = cq.Workplane("XY").box(0.3, 0.3, 0.02).translate((0, 0, -0.05))
    hub = cq.Workplane("XY").cylinder(0.04, 0.04).translate((0, 0, -0.02))
    
    # Support ribs
    rib1 = cq.Workplane("XY").box(0.15, 0.01, 0.04).translate((0.075, 0, -0.02))
    rib2 = cq.Workplane("XY").box(0.15, 0.01, 0.04).translate((-0.075, 0, -0.02))
    rib3 = cq.Workplane("XY").box(0.01, 0.15, 0.04).translate((0, 0.075, -0.02))
    rib4 = cq.Workplane("XY").box(0.01, 0.15, 0.04).translate((0, -0.075, -0.02))
    
    base = plate.union(hub).union(rib1).union(rib2).union(rib3).union(rib4)
    
    # Hole for the first module's pin
    hole = cq.Workplane("XY").cylinder(0.04 + 0.01, 0.017).translate((0, 0, -0.02))
    base = base.cut(hole)
    
    return base


def build_arm_body(length=0.2, hub_r=0.03, hub_h=0.04, plate_t=0.005, spacer_r=0.005):
    # Two hubs
    hub1 = cq.Workplane("XY").cylinder(hub_h, hub_r).translate((0, 0, hub_h / 2))
    hub2 = cq.Workplane("XY").cylinder(hub_h, hub_r).translate((length, 0, hub_h / 2))
    
    # Rigid side plates
    plate1 = cq.Workplane("XY").box(length, plate_t, hub_h).translate((length / 2, hub_r + plate_t / 2, hub_h / 2))
    plate2 = cq.Workplane("XY").box(length, plate_t, hub_h).translate((length / 2, -(hub_r + plate_t / 2), hub_h / 2))
    
    # Bracketed spacers
    spacer_len = (hub_r + plate_t) * 2
    sp1 = cq.Workplane("XZ").workplane(offset=-(hub_r + plate_t)).center(length * 0.33, hub_h / 2).circle(spacer_r).extrude(spacer_len)
    sp2 = cq.Workplane("XZ").workplane(offset=-(hub_r + plate_t)).center(length * 0.66, hub_h / 2).circle(spacer_r).extrude(spacer_len)
    
    # Brackets for the spacers
    br1_a = cq.Workplane("XZ").workplane(offset=-(hub_r + plate_t/2)).center(length * 0.33, hub_h / 2).rect(spacer_r*4, spacer_r*4).extrude(plate_t)
    br1_b = cq.Workplane("XZ").workplane(offset=(hub_r - plate_t/2)).center(length * 0.33, hub_h / 2).rect(spacer_r*4, spacer_r*4).extrude(plate_t)
    br2_a = cq.Workplane("XZ").workplane(offset=-(hub_r + plate_t/2)).center(length * 0.66, hub_h / 2).rect(spacer_r*4, spacer_r*4).extrude(plate_t)
    br2_b = cq.Workplane("XZ").workplane(offset=(hub_r - plate_t/2)).center(length * 0.66, hub_h / 2).rect(spacer_r*4, spacer_r*4).extrude(plate_t)
    
    arm = hub1.union(hub2).union(plate1).union(plate2).union(sp1).union(sp2).union(br1_a).union(br1_b).union(br2_a).union(br2_b)
    
    # Hole for the next module's pin
    hole = cq.Workplane("XY").cylinder(hub_h + 0.01, 0.017).translate((length, 0, hub_h / 2))
    arm = arm.cut(hole)
    
    return arm


def build_pin(pin_r=0.015, pin_h=0.04):
    return cq.Workplane("XY").cylinder(pin_h, pin_r).translate((0, 0, -pin_h / 2))


def build_end_effector_body():
    cap = cq.Workplane("XY").cylinder(0.02, 0.03).translate((0, 0, 0.01))
    pointer = cq.Workplane("XY").box(0.1, 0.02, 0.01).translate((0.05, 0, 0.025))
    return cap.union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack")

    metal = Material(name="metal", color=(0.7, 0.7, 0.75))
    dark_metal = Material(name="dark_metal", color=(0.3, 0.3, 0.35))
    accent = Material(name="accent", color=(0.8, 0.2, 0.1))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_body(), "base_body"),
        name="body",
        material=dark_metal,
    )

    arm_body_cq = build_arm_body()
    pin_cq = build_pin()

    mod1 = model.part("module_1")
    mod1.visual(mesh_from_cadquery(arm_body_cq, "arm_body"), name="body", material=metal)
    mod1.visual(mesh_from_cadquery(pin_cq, "pin"), name="pin", material=dark_metal)

    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mod1,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    mod2 = model.part("module_2")
    mod2.visual(mesh_from_cadquery(arm_body_cq, "arm_body"), name="body", material=metal)
    mod2.visual(mesh_from_cadquery(pin_cq, "pin"), name="pin", material=dark_metal)

    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=mod1,
        child=mod2,
        origin=Origin(xyz=(0.2, 0, 0.04)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    mod3 = model.part("module_3")
    mod3.visual(
        mesh_from_cadquery(build_end_effector_body(), "end_effector_body"),
        name="body",
        material=accent,
    )
    mod3.visual(mesh_from_cadquery(pin_cq, "pin"), name="pin", material=dark_metal)

    model.articulation(
        "joint_3",
        ArticulationType.REVOLUTE,
        parent=mod2,
        child=mod3,
        origin=Origin(xyz=(0.2, 0, 0.04)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mod1 = object_model.get_part("module_1")
    mod2 = object_model.get_part("module_2")
    mod3 = object_model.get_part("module_3")

    # Overlap allowances for the captured pins
    ctx.allow_overlap(mod1, base, elem_a="pin", elem_b="body", reason="Pin 1 is captured in base hole")
    ctx.allow_overlap(mod2, mod1, elem_a="pin", elem_b="body", reason="Pin 2 is captured in mod1 hole")
    ctx.allow_overlap(mod3, mod2, elem_a="pin", elem_b="body", reason="Pin 3 is captured in mod2 hole")

    # Exact assertions for pin containment
    ctx.expect_within(mod1, base, axes="xy", inner_elem="pin", outer_elem="body")
    ctx.expect_within(mod2, mod1, axes="xy", inner_elem="pin", outer_elem="body")
    ctx.expect_within(mod3, mod2, axes="xy", inner_elem="pin", outer_elem="body")

    # Exact assertions for pin insertion depth
    ctx.expect_overlap(mod1, base, axes="z", elem_a="pin", elem_b="body", min_overlap=0.03)
    ctx.expect_overlap(mod2, mod1, axes="z", elem_a="pin", elem_b="body", min_overlap=0.03)
    ctx.expect_overlap(mod3, mod2, axes="z", elem_a="pin", elem_b="body", min_overlap=0.03)

    # Exact assertions for body seating
    ctx.expect_contact(mod1, base, elem_a="body", elem_b="body")
    ctx.expect_contact(mod2, mod1, elem_a="body", elem_b="body")
    ctx.expect_contact(mod3, mod2, elem_a="body", elem_b="body")

    return ctx.report()


object_model = build_object_model()
