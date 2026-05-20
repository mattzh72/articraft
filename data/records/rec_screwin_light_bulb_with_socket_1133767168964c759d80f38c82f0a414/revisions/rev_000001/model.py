from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_bulb_and_socket")

    socket = model.part("socket")
    socket_cq = (
        cq.Workplane("XY")
        .circle(0.025)
        .extrude(0.05)
        .faces(">Z")
        .workplane()
        .circle(0.016)
        .cutBlind(-0.03)  # Cavity from z=0.02 to z=0.05
    )
    socket.visual(
        mesh_from_cadquery(socket_cq, "socket_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="socket_body",
    )
    
    socket_collar_cq = (
        cq.Workplane("XY", origin=(0, 0, 0.02))
        .circle(0.016)
        .circle(0.015)
        .extrude(0.03)
    )
    socket.visual(
        mesh_from_cadquery(socket_collar_cq, "socket_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="socket_collar",
    )

    bulb = model.part("bulb")
    # Base of the bulb fitting into the socket cavity
    # Cavity is z=0.02 to z=0.05 in world
    # Joint is at z=0.02, so base is z=0.0 to z=0.03 in local
    bulb.visual(
        Cylinder(radius=0.0145, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        name="bulb_base",
    )
    # Narrow and long envelope
    # Starts at local z=0.03
    bulb_envelope_cq = (
        cq.Workplane("XY", origin=(0, 0, 0.03))
        .circle(0.015)
        .extrude(0.17)
        .faces(">Z")
        .sphere(0.015)
    )
    bulb.visual(
        mesh_from_cadquery(bulb_envelope_cq, "bulb_envelope"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="bulb_envelope",
    )

    model.articulation(
        "bulb_rotation",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")

    # The bulb base intentionally overlaps the socket collar to model the threaded fit
    ctx.allow_overlap(
        bulb,
        socket,
        elem_a="bulb_base",
        elem_b="socket_collar",
        reason="The threaded bulb base seats into the threaded socket collar.",
    )
    ctx.allow_overlap(
        bulb,
        socket,
        elem_a="bulb_base",
        elem_b="socket_body",
        reason="The socket body is modeled as a mesh, whose collision hull fills the cavity.",
    )

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="bulb_base",
        outer_elem="socket_collar",
        margin=0.001,
        name="bulb base is within socket collar in XY",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="z",
        inner_elem="bulb_base",
        outer_elem="socket_collar",
        margin=0.001,
        name="bulb base is within socket collar in Z",
    )

    return ctx.report()


object_model = build_object_model()