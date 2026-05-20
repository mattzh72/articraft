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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="treasure_chest")

    wood_mat = Material(name="wood", color=(0.4, 0.25, 0.1))
    metal_mat = Material(name="metal", color=(0.7, 0.7, 0.75))

    # 1. Base Part
    base = model.part("base")

    base_shell_cq = (
        cq.Workplane("XY")
        .box(0.6, 0.4, 0.3, centered=(True, True, False))
        .faces(">Z")
        .shell(-0.02)
    )
    base.visual(
        mesh_from_cadquery(base_shell_cq, "base_shell"),
        name="base_shell",
        material=wood_mat,
    )

    base_clasp_cq = (
        cq.Workplane("XY")
        .box(0.06, 0.02, 0.059, centered=(True, True, False))
        .translate((0, 0.2, 0.24))
        .faces(">Y")
        .workplane()
        .hole(0.01)
    )
    base.visual(
        mesh_from_cadquery(base_clasp_cq, "base_clasp"),
        name="base_clasp",
        material=metal_mat,
    )

    # 2. Lid Part
    lid = model.part("lid")

    lid_shell_cq = (
        cq.Workplane("YZ")
        .moveTo(0.2, 0)
        .threePointArc((0.0, 0.2), (-0.2, 0))
        .lineTo(0.2, 0)
        .close()
        .extrude(0.3, both=True)
        .faces("<Z")
        .shell(-0.02)
        .translate((0, 0.2, 0))
    )
    lid.visual(
        mesh_from_cadquery(lid_shell_cq, "lid_shell"),
        name="lid_shell",
        material=wood_mat,
    )

    lid_clasp_cq = (
        cq.Workplane("XY")
        .box(0.06, 0.02, 0.059, centered=(True, True, False))
        .translate((0, 0.4, 0.001))
    )
    lid.visual(
        mesh_from_cadquery(lid_clasp_cq, "lid_clasp"),
        name="lid_clasp",
        material=metal_mat,
    )

    # 3. Articulation
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -0.2, 0.3)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=2.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    # Closed pose exact checks
    ctx.expect_gap(lid, base, axis="z", max_penetration=0.001, elem_a="lid_shell", elem_b="base_shell")
    ctx.expect_within(lid, base, axes="x", inner_elem="lid_clasp", outer_elem="base_clasp", margin=0.001)

    # Open pose check
    with ctx.pose({hinge: 1.5}):
        lid_pos = ctx.part_world_position(lid)
        base_pos = ctx.part_world_position(base)
        if lid_pos and base_pos:
            ctx.check("lid opens upward", lid_pos[2] > base_pos[2])

    return ctx.report()

object_model = build_object_model()