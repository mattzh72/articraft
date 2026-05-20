from sdk import (
    ArticulatedObject,
    ArticulationType,
    ClevisBracketGeometry,
    Cylinder,
    Box,
    MotionLimits,
    mesh_from_geometry,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_and_tab")
    
    fork = model.part("fork")
    clevis = ClevisBracketGeometry(
        (0.05, 0.05, 0.06),
        gap_width=0.022,
        bore_diameter=0.01,
        bore_center_z=0.04,
        base_thickness=0.01,
        center=False,
    )
    fork.visual(mesh_from_geometry(clevis, "clevis"), name="clevis")

    # The through-pin
    fork.visual(
        Cylinder(0.005, 0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.04), rpy=(0.0, 1.570796, 0.0)),
        name="pin",
    )

    tab = model.part("tab")
    # The tab has a short barrel segment that fits between the cheeks
    tab.visual(
        Cylinder(0.008, 0.02),
        origin=Origin(rpy=(0.0, 1.570796, 0.0)),
        name="barrel",
    )
    # The tab plate extending from the barrel
    tab.visual(
        Box((0.02, 0.05, 0.006)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        name="plate",
    )

    model.articulation(
        "fork_to_tab",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=tab,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    fork = object_model.get_part("fork")
    tab = object_model.get_part("tab")
    joint = object_model.get_articulation("fork_to_tab")
    
    # The pin is captured inside the tab barrel
    ctx.allow_overlap(fork, tab, elem_a="pin", elem_b="barrel", reason="The pin passes through the tab barrel.")
    
    ctx.expect_within(tab, fork, axes="x", inner_elem="barrel", outer_elem="clevis", margin=0.001)
    
    with ctx.pose({joint: 1.57}):
        ctx.expect_within(tab, fork, axes="x", inner_elem="barrel", outer_elem="clevis", margin=0.001)

    return ctx.report()

object_model = build_object_model()
