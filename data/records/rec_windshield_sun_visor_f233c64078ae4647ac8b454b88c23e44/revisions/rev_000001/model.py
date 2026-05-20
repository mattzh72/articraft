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
    model = ArticulatedObject(name="sun_visor")

    roof_bracket = model.part("roof_bracket")
    roof_bracket.visual(
        Box((0.08, 0.06, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="bracket_base",
    )
    roof_bracket.visual(
        Box((0.04, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        name="bracket_mount",
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.006, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        name="vertical_pin",
    )
    pivot_arm.visual(
        Cylinder(radius=0.006, length=0.38),
        origin=Origin(xyz=(0.0, -0.19, -0.04), rpy=(1.5708, 0.0, 0.0)),
        name="horizontal_pin",
    )

    model.articulation(
        "roof_to_arm",
        ArticulationType.REVOLUTE,
        parent=roof_bracket,
        child=pivot_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.57),
    )

    visor_panel = model.part("visor_panel")

    panel_solid = (
        cq.Workplane("XY")
        .box(0.16, 0.36, 0.022)
        .edges("|Z")
        .fillet(0.04)
        .faces(">Z or <Z")
        .edges()
        .fillet(0.005)
        .translate((-0.07, -0.20, 0.0))
    )
    visor_panel.visual(
        mesh_from_cadquery(panel_solid, "visor_panel_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="panel_body",
    )

    model.articulation(
        "arm_to_visor",
        ArticulationType.REVOLUTE,
        parent=pivot_arm,
        child=visor_panel,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    roof_bracket = object_model.get_part("roof_bracket")
    pivot_arm = object_model.get_part("pivot_arm")
    visor_panel = object_model.get_part("visor_panel")

    roof_to_arm = object_model.get_articulation("roof_to_arm")
    arm_to_visor = object_model.get_articulation("arm_to_visor")

    ctx.allow_overlap(
        pivot_arm,
        roof_bracket,
        elem_a="vertical_pin",
        elem_b="bracket_mount",
        reason="The pivot arm pin is mounted inside the roof bracket.",
    )

    ctx.allow_overlap(
        visor_panel,
        pivot_arm,
        elem_a="panel_body",
        elem_b="horizontal_pin",
        reason="The pivot arm horizontal pin is embedded inside the visor panel.",
    )

    # Check stowed pose (q=0)
    ctx.expect_gap(
        roof_bracket,
        visor_panel,
        axis="z",
        positive_elem="bracket_base",
        negative_elem="panel_body",
        min_gap=0.0,
        name="visor panel is below roof bracket",
    )

    # Check flipped down pose
    with ctx.pose({arm_to_visor: 1.57}):
        aabb = ctx.part_world_aabb(visor_panel)
        ctx.check(
            "visor_flips_down",
            aabb is not None and aabb[0][2] < -0.15,
            "Visor panel should extend downwards when flipped.",
        )

    # Check swung side pose
    with ctx.pose({roof_to_arm: 1.57}):
        aabb = ctx.part_world_aabb(pivot_arm)
        ctx.check(
            "arm_swings_side",
            aabb is not None and aabb[0][0] < -0.15,
            "Pivot arm should swing towards -X.",
        )

    return ctx.report()


object_model = build_object_model()