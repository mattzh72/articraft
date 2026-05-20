from __future__ import annotations

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
)


STEEL = Material("satin_steel", color=(0.56, 0.58, 0.60, 1.0))
DARK_STEEL = Material("dark_bushings", color=(0.12, 0.13, 0.14, 1.0))
BLUE = Material("blue_branch_arm", color=(0.05, 0.22, 0.72, 1.0))
ORANGE = Material("orange_branch_arm", color=(0.88, 0.36, 0.06, 1.0))
PIN = Material("polished_pin_faces", color=(0.78, 0.80, 0.82, 1.0))
RUBBER = Material("black_rubber_grips", color=(0.02, 0.02, 0.018, 1.0))


def _add_arm_link(part, *, length: float, color: Material, elevated: bool = False) -> None:
    """Add one link whose local origin is the proximal hinge line."""
    # The forearm rides one bearing thickness above the proximal arm so the
    # elbow knuckles visibly stack instead of occupying the same solid volume.
    z_center = 0.0525 if elevated else 0.0175
    z_span = (z_center - 0.0175, z_center + 0.0175)

    part.visual(
        Cylinder(0.042, 0.035),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=DARK_STEEL,
        name="proximal_boss",
    )
    part.visual(
        Box((length - 0.060, 0.038, 0.035)),
        origin=Origin(xyz=(length * 0.5, 0.0, z_center)),
        material=color,
        name="web_beam",
    )
    part.visual(
        Cylinder(0.038, 0.035),
        origin=Origin(xyz=(length, 0.0, z_center)),
        material=DARK_STEEL,
        name="distal_boss",
    )
    part.visual(
        Box((length - 0.105, 0.010, 0.039)),
        origin=Origin(xyz=(length * 0.5 + 0.020, 0.023, (z_span[0] + z_span[1]) * 0.5)),
        material=PIN,
        name="side_rib_a",
    )
    part.visual(
        Box((length - 0.105, 0.010, 0.039)),
        origin=Origin(xyz=(length * 0.5 + 0.020, -0.023, (z_span[0] + z_span[1]) * 0.5)),
        material=PIN,
        name="side_rib_b",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="independent_branching_hinge_mechanism")

    base = model.part("base")
    base.visual(
        Box((0.42, 0.50, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=STEEL,
        name="floor_plate",
    )
    for i, (x, y) in enumerate(((-0.155, -0.205), (-0.155, 0.205), (0.155, -0.205), (0.155, 0.205))):
        base.visual(
            Box((0.070, 0.060, 0.012)),
            origin=Origin(xyz=(x, y, -0.006)),
            material=RUBBER,
            name=f"rubber_foot_{i}",
        )

    base.visual(
        Cylinder(0.045, 0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=STEEL,
        name="central_riser",
    )
    base.visual(
        Cylinder(0.035, 0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.105), rpy=(1.57079632679, 0.0, 0.0)),
        material=STEEL,
        name="branch_manifold",
    )
    base.visual(
        Cylinder(0.034, 0.090),
        origin=Origin(xyz=(0.0, 0.160, 0.095)),
        material=DARK_STEEL,
        name="branch_0_lower_pin",
    )
    base.visual(
        Cylinder(0.034, 0.090),
        origin=Origin(xyz=(0.0, -0.160, 0.095)),
        material=DARK_STEEL,
        name="branch_1_lower_pin",
    )
    base.visual(
        Box((0.090, 0.044, 0.025)),
        origin=Origin(xyz=(0.0, 0.160, 0.060)),
        material=STEEL,
        name="branch_0_socket_block",
    )
    base.visual(
        Box((0.090, 0.044, 0.025)),
        origin=Origin(xyz=(0.0, -0.160, 0.060)),
        material=STEEL,
        name="branch_1_socket_block",
    )

    branch_0_arm = model.part("branch_0_arm")
    _add_arm_link(branch_0_arm, length=0.280, color=BLUE, elevated=False)

    branch_0_forearm = model.part("branch_0_forearm")
    _add_arm_link(branch_0_forearm, length=0.235, color=BLUE, elevated=True)
    branch_0_forearm.visual(
        Box((0.040, 0.060, 0.032)),
        origin=Origin(xyz=(0.260, 0.0, 0.0525)),
        material=RUBBER,
        name="end_grip",
    )

    branch_1_arm = model.part("branch_1_arm")
    _add_arm_link(branch_1_arm, length=0.280, color=ORANGE, elevated=False)

    branch_1_forearm = model.part("branch_1_forearm")
    _add_arm_link(branch_1_forearm, length=0.235, color=ORANGE, elevated=True)
    branch_1_forearm.visual(
        Box((0.040, 0.060, 0.032)),
        origin=Origin(xyz=(0.260, 0.0, 0.0525)),
        material=RUBBER,
        name="end_grip",
    )

    model.articulation(
        "branch_0_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=branch_0_arm,
        origin=Origin(xyz=(0.0, 0.160, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "branch_0_elbow",
        ArticulationType.REVOLUTE,
        parent=branch_0_arm,
        child=branch_0_forearm,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.4, lower=0.0, upper=1.45),
    )
    model.articulation(
        "branch_1_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=branch_1_arm,
        origin=Origin(xyz=(0.0, -0.160, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=2.0, lower=-1.15, upper=0.0),
    )
    model.articulation(
        "branch_1_elbow",
        ArticulationType.REVOLUTE,
        parent=branch_1_arm,
        child=branch_1_forearm,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.4, lower=-1.45, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    branch_0_arm = object_model.get_part("branch_0_arm")
    branch_0_forearm = object_model.get_part("branch_0_forearm")
    branch_1_arm = object_model.get_part("branch_1_arm")
    branch_1_forearm = object_model.get_part("branch_1_forearm")
    base = object_model.get_part("base")
    branch_0_shoulder = object_model.get_articulation("branch_0_shoulder")
    branch_0_elbow = object_model.get_articulation("branch_0_elbow")
    branch_1_shoulder = object_model.get_articulation("branch_1_shoulder")
    branch_1_elbow = object_model.get_articulation("branch_1_elbow")

    ctx.expect_contact(
        base,
        branch_0_arm,
        elem_a="branch_0_lower_pin",
        elem_b="proximal_boss",
        name="branch 0 shoulder is seated on its base pin",
    )
    ctx.expect_contact(
        branch_0_arm,
        branch_0_forearm,
        elem_a="distal_boss",
        elem_b="proximal_boss",
        name="branch 0 elbow knuckles are stacked in contact",
    )
    ctx.expect_contact(
        base,
        branch_1_arm,
        elem_a="branch_1_lower_pin",
        elem_b="proximal_boss",
        name="branch 1 shoulder is seated on its base pin",
    )
    ctx.expect_contact(
        branch_1_arm,
        branch_1_forearm,
        elem_a="distal_boss",
        elem_b="proximal_boss",
        name="branch 1 elbow knuckles are stacked in contact",
    )

    branch_1_rest = ctx.part_world_position(branch_1_forearm)
    with ctx.pose({branch_0_shoulder: 0.65, branch_0_elbow: 0.55}):
        branch_0_moved = ctx.part_world_position(branch_0_forearm)
        branch_1_still = ctx.part_world_position(branch_1_forearm)
    ctx.check(
        "branch 0 moves independently of branch 1",
        branch_0_moved is not None
        and branch_1_rest is not None
        and branch_1_still is not None
        and branch_0_moved[1] > 0.30
        and abs(branch_1_still[1] - branch_1_rest[1]) < 1e-6,
        details=f"branch_0_moved={branch_0_moved}, branch_1_rest={branch_1_rest}, branch_1_still={branch_1_still}",
    )

    branch_0_rest = ctx.part_world_position(branch_0_forearm)
    with ctx.pose({branch_1_shoulder: -0.65, branch_1_elbow: -0.55}):
        branch_1_moved = ctx.part_world_position(branch_1_forearm)
        branch_0_still = ctx.part_world_position(branch_0_forearm)
    ctx.check(
        "branch 1 moves independently of branch 0",
        branch_1_moved is not None
        and branch_0_rest is not None
        and branch_0_still is not None
        and branch_1_moved[1] < -0.30
        and abs(branch_0_still[1] - branch_0_rest[1]) < 1e-6,
        details=f"branch_1_moved={branch_1_moved}, branch_0_rest={branch_0_rest}, branch_0_still={branch_0_still}",
    )

    return ctx.report()


object_model = build_object_model()
