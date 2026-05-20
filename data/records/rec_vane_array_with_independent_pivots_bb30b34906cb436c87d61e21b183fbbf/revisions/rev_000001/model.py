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
    model = ArticulatedObject(name="louver_panel")

    frame = model.part("frame")

    frame_w = 0.6
    frame_h = 0.8
    frame_d = 0.05

    open_w = 0.5
    open_h = 0.7

    frame_cq = (
        cq.Workplane("XY")
        .box(frame_w, frame_d, frame_h)
        .faces(">Y")
        .workplane()
        .rect(open_w, open_h)
        .cutBlind(-frame_d)
    )

    frame.visual(
        mesh_from_cadquery(frame_cq, "frame_mesh"),
        origin=Origin(xyz=(0, 0, frame_h / 2)),
        material=Material(name="frame_material", color=(0.3, 0.3, 0.3)),
        name="frame_vis",
    )

    num_flaps = 4
    flap_w = 0.49
    flap_h = 0.165
    flap_d = 0.01
    pin_r = 0.003
    pin_l = 0.01

    flap_body = cq.Workplane("XY").box(flap_w, flap_d, flap_h).translate((0, 0, -flap_h / 2 + 0.005))
    right_pin = cq.Workplane("YZ").workplane(offset=flap_w / 2).circle(pin_r).extrude(pin_l)
    left_pin = cq.Workplane("YZ").workplane(offset=-flap_w / 2 - pin_l).circle(pin_r).extrude(pin_l)
    flap_cq = flap_body.union(right_pin).union(left_pin)

    for i in range(num_flaps):
        flap = model.part(f"flap_{i}")

        z_hinge = 0.740 - i * 0.175

        flap.visual(
            mesh_from_cadquery(flap_cq, f"flap_{i}_mesh"),
            origin=Origin(),
            material=Material(name=f"flap_{i}_material", color=(0.8, 0.8, 0.8)),
            name=f"flap_{i}_vis",
        )

        model.articulation(
            f"hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(xyz=(0, 0, z_hinge)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.57),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    for i in range(4):
        flap = object_model.get_part(f"flap_{i}")
        # The pins intentionally extend into the frame by 0.005 on each side
        ctx.allow_overlap(
            flap,
            frame,
            elem_a=f"flap_{i}_vis",
            elem_b="frame_vis",
            reason="Pins on the sides of the flaps intentionally embed into the frame to form the hinges.",
        )

        ctx.expect_within(
            flap,
            frame,
            axes="xy",
            margin=0.01,
            name=f"flap_{i} remains within the frame bounds",
        )

    return ctx.report()

object_model = build_object_model()
