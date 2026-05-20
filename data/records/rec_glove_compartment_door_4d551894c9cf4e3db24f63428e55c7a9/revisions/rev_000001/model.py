import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glove_compartment")

    dashboard = model.part("dashboard")
    # Dashboard housing: 0.6 wide, 0.4 high, 0.4 deep.
    # Front face at Y=-0.2.
    # Opening: 0.4 wide, 0.2 high, 0.3 deep.
    dashboard_cq = (
        cq.Workplane("XY")
        .box(0.6, 0.4, 0.4)
        .faces("-Y")
        .workplane(offset=0.01)
        .rect(0.4, 0.2)
        .cutBlind(-0.31)
    )
    dashboard.visual(
        mesh_from_cadquery(dashboard_cq, "dashboard_shell"),
        origin=Origin(xyz=(0, 0, 0)),
        name="housing",
    )

    door = model.part("door")
    # Door panel: slightly smaller than the 0.4 x 0.2 opening.
    door.visual(
        Box((0.39, 0.02, 0.19)),
        # Local origin such that Z=0 is the bottom hinge edge.
        # Panel starts at Z=0.003 for bottom clearance but overlaps barrel.
        origin=Origin(xyz=(0, 0.01, 0.098)),
        name="panel",
    )
    # Hinge barrel on the door.
    # The barrel is a cylinder along the X axis.
    door.visual(
        mesh_from_cadquery(cq.Workplane("YZ").cylinder(0.39, 0.005), "hinge_barrel"),
        origin=Origin(xyz=(0, 0, 0)),
        name="hinge_barrel",
    )
    # Handle on the outer face (Y < 0).
    door.visual(
        Box((0.08, 0.02, 0.03)),
        origin=Origin(xyz=(0, -0.01, 0.16)),
        name="handle",
    )

    # Hinge at the bottom front edge of the opening.
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=dashboard,
        child=door,
        origin=Origin(xyz=(0, -0.20, -0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=math.pi / 2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    dashboard = object_model.get_part("dashboard")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")

    # Hinge barrel is allowed to overlap the dashboard floor to act as a hinge joint.
    ctx.allow_overlap(
        door,
        dashboard,
        elem_a="hinge_barrel",
        elem_b="housing",
        reason="Hinge barrel is embedded in the dashboard floor to act as a hinge joint.",
    )

    # At rest (closed), door panel fits inside the opening.
    ctx.expect_within(
        door,
        dashboard,
        axes="xz",
        inner_elem="panel",
        outer_elem="housing",
        margin=0.001,
        name="door_fits_in_opening",
    )

    # When open 90 degrees, the door extends forward.
    with ctx.pose({hinge: math.pi / 2}):
        ctx.expect_gap(
            dashboard,
            door,
            axis="y",
            positive_elem="housing",
            negative_elem="panel",
            max_penetration=0.001,
            name="door_extends_forward_when_open",
        )

    return ctx.report()


object_model = build_object_model()