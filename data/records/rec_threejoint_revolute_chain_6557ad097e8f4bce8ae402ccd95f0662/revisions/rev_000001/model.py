from __future__ import annotations

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


def build_link_mesh(name: str, is_base: bool = False, is_end: bool = False):
    parts = []
    clearance = 0.0
    
    if not is_base:
        # Knuckle 1 (bottom): Z from -0.02 to -clearance with a hole for the pin
        h1 = 0.02 - clearance
        k1 = (
            cq.Workplane("XY")
            .workplane(offset=-0.02)
            .circle(0.015)
            .circle(0.005)  # Hole for pin
            .extrude(h1)
        )
        # arm1 must not fill the hole! Start at X=0.006
        arm1_len = 0.025 - 0.006
        arm1_cx = 0.006 + arm1_len / 2
        arm1 = cq.Workplane("XY").workplane(offset=-0.02 + h1/2).box(arm1_len, 0.03, h1).translate((arm1_cx, -0.015, 0))
        parts.extend([k1, arm1])
        
    start_x = 0.0 if is_base else 0.02
    end_x = 0.05 if is_end else 0.08
    length = end_x - start_x
    center_x = start_x + length / 2
    body = cq.Workplane("XY").box(length, 0.02, 0.04).translate((center_x, -0.03, 0))
    parts.append(body)
    
    if not is_end:
        # Knuckle 2 (top): Z from clearance to 0.02
        h2 = 0.02 - clearance
        k2 = cq.Workplane("XY").center(0.1, 0).workplane(offset=clearance).circle(0.015).extrude(h2)
        # Pin extending down through the next link's knuckle hole
        pin = cq.Workplane("XY").center(0.1, 0).workplane(offset=-0.02).circle(0.005).extrude(0.04)
        k2 = k2.union(pin)
        
        arm2 = cq.Workplane("XY").workplane(offset=clearance + h2/2).box(0.025, 0.03, h2).translate((0.0875, -0.015, 0))
        parts.extend([k2, arm2])
        
    result = parts[0]
    for p in parts[1:]:
        result = result.union(p)
        
    result = result.clean()
    return mesh_from_cadquery(result, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_hinge_chain")

    base = model.part("base")
    base.visual(build_link_mesh("base_link", is_base=True), name="base_visual")

    link_1 = model.part("link_1")
    link_1.visual(build_link_mesh("link_1"), name="link_1_visual")

    link_2 = model.part("link_2")
    link_2.visual(build_link_mesh("link_2"), name="link_2_visual")

    end_link = model.part("end_link")
    end_link.visual(build_link_mesh("end_link", is_end=True), name="end_link_visual")

    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(0.1, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-2.0, upper=2.0),
    )

    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.1, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-2.0, upper=2.0),
    )

    model.articulation(
        "joint_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_link,
        origin=Origin(xyz=(0.1, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=-2.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # The hinge pins and knuckles exactly touch to form captured hinges.
    ctx.allow_overlap("base", "link_1", reason="Hinge pin and knuckles exactly touch to form a captured hinge.")
    ctx.allow_overlap("link_1", "link_2", reason="Hinge pin and knuckles exactly touch to form a captured hinge.")
    ctx.allow_overlap("link_2", "end_link", reason="Hinge pin and knuckles exactly touch to form a captured hinge.")

    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_link = object_model.get_part("end_link")
    joint_1 = object_model.get_articulation("joint_1")

    pos_1 = ctx.part_world_position(link_1)
    pos_2 = ctx.part_world_position(link_2)
    pos_3 = ctx.part_world_position(end_link)

    ctx.check("link_1_at_0_1", pos_1 is not None and abs(pos_1[0] - 0.1) < 0.001)
    ctx.check("link_2_at_0_2", pos_2 is not None and abs(pos_2[0] - 0.2) < 0.001)
    ctx.check("end_link_at_0_3", pos_3 is not None and abs(pos_3[0] - 0.3) < 0.001)

    with ctx.pose({joint_1: 3.14159}):
        pos_2_folded = ctx.part_world_position(link_2)
        ctx.check("link_1_folds_back", pos_2_folded is not None and abs(pos_2_folded[0] - 0.0) < 0.001)

    return ctx.report()


object_model = build_object_model()
