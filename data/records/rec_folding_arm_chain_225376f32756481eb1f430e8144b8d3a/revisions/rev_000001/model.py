import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
    Cylinder,
)

L = 0.4
W = 0.06
T = 0.012
C = 0.002
pin_R = W * 0.24
pin_H = 2 * T + C

def make_link_mesh():
    shape = (
        cq.Workplane("XY")
        .slot2D(L, W)
        .extrude(T / 2, both=True)
        .faces(">Z").workplane()
        .pushPoints([(-L / 2, 0)])
        .hole(pin_R * 2.05)
    )
    return shape.translate((L / 2, 0, 0))

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm")

    yellow = Material(name="yellow", color=(0.9, 0.7, 0.1))
    dark_metal = Material(name="dark_metal", color=(0.2, 0.2, 0.2))

    # Base
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.08, height=0.05),
        origin=Origin(xyz=(0, 0, 0.025)),
        material=dark_metal,
        name="base_vis"
    )
    # Washer connecting base to link_1
    base.visual(
        Cylinder(radius=pin_R * 1.5, height=0.004),
        origin=Origin(xyz=(0, 0, 0.052)),
        material=dark_metal,
        name="base_washer"
    )
    # Pin connecting base to link_1, flush with link_1 top
    base.visual(
        Cylinder(radius=pin_R, height=0.041),
        origin=Origin(xyz=(0, 0, 0.0455)),
        material=dark_metal,
        name="base_pin"
    )

    link_mesh = mesh_from_cadquery(make_link_mesh(), "link_mesh")

    num_links = 5
    parent = base
    parent_name = "base"

    for i in range(num_links):
        link_name = f"link_{i+1}"
        link = model.part(link_name)
        link.visual(
            link_mesh,
            origin=Origin(),
            material=yellow,
            name=f"{link_name}_vis"
        )

        if i == 0:
            origin = Origin(xyz=(0, 0, 0.06))
        else:
            origin = Origin(xyz=(L, 0, T + C))
            # Add washer to parent to connect to the new child
            parent.visual(
                Cylinder(radius=pin_R * 1.5, height=C),
                origin=Origin(xyz=(L, 0, T / 2 + C / 2)),
                material=dark_metal,
                name=f"washer_{i}"
            )
            # Add pin to parent to connect to the new child
            parent.visual(
                Cylinder(radius=pin_R, height=pin_H),
                origin=Origin(xyz=(L, 0, (T + C) / 2)),
                material=dark_metal,
                name=f"pin_{i}"
            )

        model.articulation(
            f"{parent_name}_to_{link_name}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=link,
            origin=origin,
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-3.15, upper=3.15)
        )

        parent = link
        parent_name = link_name

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Basic exact gaps at rest
    ctx.expect_gap("link_2", "link_1", axis="z", positive_elem="link_2_vis", negative_elem="link_1_vis", min_gap=0.001, max_gap=0.003)
    ctx.expect_gap("link_3", "link_2", axis="z", positive_elem="link_3_vis", negative_elem="link_2_vis", min_gap=0.001, max_gap=0.003)
    ctx.expect_gap("link_4", "link_3", axis="z", positive_elem="link_4_vis", negative_elem="link_3_vis", min_gap=0.001, max_gap=0.003)
    ctx.expect_gap("link_5", "link_4", axis="z", positive_elem="link_5_vis", negative_elem="link_4_vis", min_gap=0.001, max_gap=0.003)

    # Check folded pose to ensure links can compactly fold over each other
    pose_dict = {
        "link_1_to_link_2": math.pi,
        "link_2_to_link_3": -math.pi,
        "link_3_to_link_4": math.pi,
        "link_4_to_link_5": -math.pi,
    }
    with ctx.pose(pose_dict):
        ctx.expect_overlap("link_1", "link_3", axes="x", min_overlap=0.3)
        ctx.expect_overlap("link_1", "link_5", axes="x", min_overlap=0.3)
        ctx.expect_overlap("link_1", "link_3", axes="y", min_overlap=0.05)
        ctx.expect_overlap("link_1", "link_5", axes="y", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()
