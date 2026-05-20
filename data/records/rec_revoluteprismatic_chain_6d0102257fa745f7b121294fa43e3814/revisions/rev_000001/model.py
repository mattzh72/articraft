from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_and_slide_fixture")

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.02, 0.20, 0.40)),
        origin=Origin(xyz=(-0.01, 0.0, 0.0)),
        name="plate",
    )
    backplate.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.06)),
        name="top_hinge_mount",
    )
    backplate.visual(
        Box((0.04, 0.04, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, -0.06)),
        name="bottom_hinge_mount",
    )

    main_link = model.part("main_link")
    main_link.visual(
        Box((0.31, 0.04, 0.04)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        name="arm",
    )
    main_link.visual(
        Box((0.04, 0.04, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        name="hinge_barrel",
    )
    main_link.visual(
        Box((0.08, 0.08, 0.06)),
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        name="sleeve",
    )

    model.articulation(
        "backplate_to_main_link",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=main_link,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-1.57, upper=1.57),
    )

    ram = model.part("ram")
    ram.visual(
        Box((0.04, 0.04, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="head",
    )

    model.articulation(
        "main_link_to_ram",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=ram,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    backplate = object_model.get_part("backplate")
    main_link = object_model.get_part("main_link")
    ram = object_model.get_part("ram")
    
    ctx.expect_gap(
        main_link, backplate, axis="z",
        positive_elem="hinge_barrel", negative_elem="bottom_hinge_mount",
        max_penetration=1e-5, max_gap=0.001
    )
    ctx.expect_gap(
        backplate, main_link, axis="z",
        positive_elem="top_hinge_mount", negative_elem="hinge_barrel",
        max_penetration=1e-5, max_gap=0.001
    )
    
    ctx.allow_overlap(
        main_link, ram,
        elem_a="sleeve", elem_b="head",
        reason="The ram is intentionally represented as sliding inside the solid sleeve proxy."
    )
    
    ctx.expect_within(
        ram, main_link, axes="xy",
        inner_elem="head", outer_elem="sleeve",
        margin=0.001,
        name="ram stays centered in the sleeve"
    )
    
    ctx.expect_overlap(
        ram, main_link, axes="z",
        elem_a="head", elem_b="sleeve",
        min_overlap=0.05,
        name="ram remains inserted in the sleeve at rest"
    )

    slide_joint = object_model.get_articulation("main_link_to_ram")
    with ctx.pose({slide_joint: 0.15}):
        ctx.expect_overlap(
            ram, main_link, axes="z",
            elem_a="head", elem_b="sleeve",
            min_overlap=0.01,
            name="ram retains insertion in the sleeve when extended"
        )
    
    return ctx.report()

object_model = build_object_model()
