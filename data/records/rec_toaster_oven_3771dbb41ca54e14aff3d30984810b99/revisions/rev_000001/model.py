from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_toaster_oven")

    def make_body():
        # Main body
        body = cq.Workplane("XY").box(0.48, 0.30, 0.25)
        # Cavity
        cavity = cq.Workplane("XY").center(-0.06, -0.02).box(0.32, 0.28, 0.21)
        body = body.cut(cavity)
        return body

    def make_door():
        # Door frame
        door = cq.Workplane("XY").box(0.16, 0.02, 0.23)
        # Window cutout
        window = cq.Workplane("XY").box(0.10, 0.04, 0.17)
        door = door.cut(window)
        return door

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_body(), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        name="body_shell",
        material=Material("red", color=(0.85, 0.2, 0.2)),
    )

    # Feet
    for x in [-0.20, 0.20]:
        for y in [-0.10, 0.10]:
            base.visual(
                Cylinder(radius=0.015, length=0.02),
                origin=Origin(xyz=(x, y, 0.01)),
                name=f"foot_{x}_{y}",
                material=Material("black", color=(0.1, 0.1, 0.1)),
            )

    left_door = model.part("left_door")
    left_door.visual(
        mesh_from_cadquery(make_door(), "left_door_frame"),
        origin=Origin(xyz=(0.08, -0.01, 0.0)),
        name="frame",
        material=Material("silver", color=(0.8, 0.8, 0.8)),
    )
    left_door.visual(
        Box((0.10, 0.005, 0.17)),
        origin=Origin(xyz=(0.08, -0.01, 0.0)),
        name="glass",
        material=Material("dark_glass", color=(0.2, 0.2, 0.2)),
    )
    left_door.visual(
        Box((0.02, 0.015, 0.10)),
        origin=Origin(xyz=(0.14, -0.0275, 0.0)),
        name="handle",
        material=Material("chrome", color=(0.9, 0.9, 0.9)),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_door,
        origin=Origin(xyz=(-0.22, -0.15, 0.145)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=2.0),
    )

    right_door = model.part("right_door")
    right_door.visual(
        mesh_from_cadquery(make_door(), "right_door_frame"),
        origin=Origin(xyz=(-0.08, -0.01, 0.0)),
        name="frame",
        material=Material("silver", color=(0.8, 0.8, 0.8)),
    )
    right_door.visual(
        Box((0.10, 0.005, 0.17)),
        origin=Origin(xyz=(-0.08, -0.01, 0.0)),
        name="glass",
        material=Material("dark_glass", color=(0.2, 0.2, 0.2)),
    )
    right_door.visual(
        Box((0.02, 0.015, 0.10)),
        origin=Origin(xyz=(-0.14, -0.0275, 0.0)),
        name="handle",
        material=Material("chrome", color=(0.9, 0.9, 0.9)),
    )

    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_door,
        origin=Origin(xyz=(0.10, -0.15, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=2.0),
    )

    # Knobs
    for i, z in enumerate([0.08, 0.145, 0.21]):
        knob = model.part(f"knob_{i}")
        knob.visual(
            Cylinder(radius=0.015, length=0.015),
            origin=Origin(xyz=(0.0, -0.0075, 0.0), rpy=(1.5708, 0.0, 0.0)),
            name="dial",
            material=Material("chrome", color=(0.9, 0.9, 0.9)),
        )
        knob.visual(
            Box((0.005, 0.02, 0.03)),
            origin=Origin(xyz=(0.0, -0.01, 0.0)),
            name="grip",
            material=Material("chrome", color=(0.9, 0.9, 0.9)),
        )

        model.articulation(
            f"knob_{i}_joint",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=knob,
            origin=Origin(xyz=(0.17, -0.15, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=5.0),
        )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")

    # At rest, doors should be closed and sit flush on the front face
    ctx.expect_contact(left_door, base, name="left door contacts base when closed")
    ctx.expect_contact(right_door, base, name="right door contacts base when closed")

    # Check that doors open outward (forward)
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    with ctx.pose({left_hinge: 1.5, right_hinge: 1.5}):
        # Base is on the positive side of the doors along Y (doors move into negative Y)
        ctx.expect_gap(base, left_door, axis="y", min_gap=0.0, name="left door opens outward")
        ctx.expect_gap(base, right_door, axis="y", min_gap=0.0, name="right door opens outward")

    return ctx.report()

object_model = build_object_model()
