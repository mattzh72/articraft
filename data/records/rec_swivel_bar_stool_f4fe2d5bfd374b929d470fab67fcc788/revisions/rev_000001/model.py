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
    model = ArticulatedObject(name="pedestal_bar_stool")

    base = model.part("base")
    # Base is a chamfered cylinder
    base_shape = cq.Workplane("XY").cylinder(0.05, 0.22).edges(">Z").chamfer(0.015)
    base.visual(
        mesh_from_cadquery(base_shape, "base_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_shell",
    )
    # The post is rigidly attached to the base, so it belongs to the same part
    base.visual(
        Cylinder(radius=0.03, length=0.65),
        origin=Origin(xyz=(0.0, 0.0, 0.05 + 0.325)),
        name="post_shell",
    )

    seat = model.part("seat")
    seat_shape = cq.Workplane("XY").cylinder(0.06, 0.18).edges(">Z or <Z").chamfer(0.01)
    seat.visual(
        mesh_from_cadquery(seat_shape, "seat_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        name="seat_shell",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    seat_swivel = object_model.get_articulation("seat_swivel")

    ctx.expect_contact(seat, base, name="seat rests on base post")
    ctx.expect_within(seat, base, axes="xy", name="seat is centered over base")
    
    with ctx.pose({seat_swivel: 1.57}):
        ctx.expect_within(seat, base, axes="xy", name="seat remains centered when rotated")
    
    return ctx.report()


object_model = build_object_model()
