from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


UPPER_ARM_LENGTH = 0.280
FOREARM_LENGTH = 0.240

ROOT_FORK_INNER_GAP = 0.052
ROOT_FORK_CHEEK_THICKNESS = 0.012
SHOULDER_HUB_RADIUS = 0.022
SHOULDER_HUB_LENGTH = 0.048

ELBOW_YOKE_INNER_GAP = 0.040
ELBOW_YOKE_CHEEK_THICKNESS = 0.010
ELBOW_HUB_RADIUS = 0.0175
ELBOW_HUB_LENGTH = 0.036


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_elbow_module")

    model.material("powder_coat", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("cast_aluminum", rgba=(0.69, 0.72, 0.75, 1.0))
    model.material("machined_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("fastener_black", rgba=(0.10, 0.11, 0.12, 1.0))

    root_fork = model.part("root_fork")
    root_fork.visual(
        Box((0.140, 0.100, 0.014)),
        origin=Origin(xyz=(-0.028, 0.0, -0.095)),
        material="powder_coat",
        name="base_plate",
    )
    root_fork.visual(
        Box((0.050, 0.060, 0.062)),
        origin=Origin(xyz=(-0.045, 0.0, -0.057)),
        material="powder_coat",
        name="pedestal",
    )
    root_fork.visual(
        Box((0.016, 0.076, 0.080)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material="powder_coat",
        name="rear_bridge",
    )
    root_fork.visual(
        Box((0.068, ROOT_FORK_CHEEK_THICKNESS, 0.080)),
        origin=Origin(
            xyz=(0.008, ROOT_FORK_INNER_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0, 0.0)
        ),
        material="powder_coat",
        name="left_cheek",
    )
    root_fork.visual(
        Box((0.068, ROOT_FORK_CHEEK_THICKNESS, 0.080)),
        origin=Origin(
            xyz=(0.008, -(ROOT_FORK_INNER_GAP / 2.0 + ROOT_FORK_CHEEK_THICKNESS / 2.0), 0.0)
        ),
        material="powder_coat",
        name="right_cheek",
    )
    root_fork.inertial = Inertial.from_geometry(
        Box((0.140, 0.100, 0.110)),
        mass=3.8,
        origin=Origin(xyz=(-0.028, 0.0, -0.040)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="left_shoulder_pad",
    )
    upper_arm.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="right_shoulder_pad",
    )
    upper_arm.visual(
        Box((0.028, 0.042, 0.036)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material="machined_steel",
        name="shoulder_block",
    )
    upper_arm.visual(
        Box((0.230, 0.040, 0.034)),
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        material="cast_aluminum",
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.150, 0.024, 0.014)),
        origin=Origin(xyz=(0.115, 0.0, -0.020)),
        material="cast_aluminum",
        name="lower_rib",
    )
    upper_arm.visual(
        Box((0.038, 0.050, 0.036)),
        origin=Origin(xyz=(0.236, 0.0, 0.0)),
        material="cast_aluminum",
        name="yoke_bridge",
    )
    upper_arm.visual(
        Box((0.058, ELBOW_YOKE_CHEEK_THICKNESS, 0.062)),
        origin=Origin(
            xyz=(0.271, ELBOW_YOKE_INNER_GAP / 2.0 + ELBOW_YOKE_CHEEK_THICKNESS / 2.0, 0.0)
        ),
        material="cast_aluminum",
        name="left_elbow_ear",
    )
    upper_arm.visual(
        Box((0.058, ELBOW_YOKE_CHEEK_THICKNESS, 0.062)),
        origin=Origin(
            xyz=(0.271, -(ELBOW_YOKE_INNER_GAP / 2.0 + ELBOW_YOKE_CHEEK_THICKNESS / 2.0), 0.0)
        ),
        material="cast_aluminum",
        name="right_elbow_ear",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.305, 0.060, 0.070)),
        mass=1.7,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="left_elbow_pad",
    )
    forearm.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="right_elbow_pad",
    )
    forearm.visual(
        Box((0.228, 0.032, 0.028)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material="cast_aluminum",
        name="forearm_body",
    )
    forearm.visual(
        Box((0.012, 0.090, 0.074)),
        origin=Origin(xyz=(FOREARM_LENGTH - 0.006, 0.0, 0.0)),
        material="fastener_black",
        name="end_plate",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.255, 0.090, 0.080)),
        mass=1.2,
        origin=Origin(xyz=(0.128, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_revolute",
        ArticulationType.REVOLUTE,
        parent=root_fork,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.45, effort=90.0, velocity=1.4),
    )
    model.articulation(
        "elbow_revolute",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.12, upper=2.30, effort=60.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_fork = object_model.get_part("root_fork")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder_revolute")
    elbow = object_model.get_articulation("elbow_revolute")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return 0.5 * (lower[2] + upper[2])

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "three-part fork elbow chain present",
        [part.name for part in object_model.parts] == ["root_fork", "upper_arm", "forearm"],
        details=str([part.name for part in object_model.parts]),
    )
    ctx.check(
        "shoulder articulation is rooted in the fork",
        shoulder.parent == "root_fork" and shoulder.child == "upper_arm",
        details=f"parent={shoulder.parent}, child={shoulder.child}",
    )
    ctx.check(
        "elbow articulation is rooted in the upper arm",
        elbow.parent == "upper_arm" and elbow.child == "forearm",
        details=f"parent={elbow.parent}, child={elbow.child}",
    )
    ctx.check(
        "both joints use the arm-raising pitch axis",
        shoulder.axis == (0.0, -1.0, 0.0) and elbow.axis == (0.0, -1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_origin_distance(
            root_fork,
            upper_arm,
            axes=("x", "y", "z"),
            max_dist=0.0005,
            name="upper arm root sits on the fork shoulder axis",
        )
        ctx.expect_origin_gap(
            forearm,
            upper_arm,
            axis="x",
            min_gap=UPPER_ARM_LENGTH - 0.001,
            max_gap=UPPER_ARM_LENGTH + 0.001,
            name="elbow joint sits at the upper arm tip",
        )
        ctx.expect_contact(
            root_fork,
            upper_arm,
            elem_a="left_cheek",
            elem_b="left_shoulder_pad",
            name="left shoulder thrust pad bears on the fork cheek",
        )
        ctx.expect_contact(
            root_fork,
            upper_arm,
            elem_a="right_cheek",
            elem_b="right_shoulder_pad",
            name="right shoulder thrust pad bears on the fork cheek",
        )
        ctx.expect_contact(
            upper_arm,
            forearm,
            elem_a="left_elbow_ear",
            elem_b="left_elbow_pad",
            name="left elbow thrust pad bears on the upper-arm ear",
        )
        ctx.expect_contact(
            upper_arm,
            forearm,
            elem_a="right_elbow_ear",
            elem_b="right_elbow_pad",
            name="right elbow thrust pad bears on the upper-arm ear",
        )
        ctx.expect_origin_gap(
            forearm,
            upper_arm,
            axis="x",
            min_gap=UPPER_ARM_LENGTH - 0.001,
            max_gap=UPPER_ARM_LENGTH + 0.001,
            name="forearm starts from the elbow axis instead of floating off-link",
        )

    rest_elbow_pos = None
    raised_elbow_pos = None
    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        rest_elbow_pos = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.85, elbow: 0.0}):
        raised_elbow_pos = ctx.part_world_position(forearm)
    ctx.check(
        "positive shoulder rotation raises the elbow module",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.14,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )

    rest_end_plate_z = None
    bent_end_plate_z = None
    with ctx.pose({shoulder: 0.45, elbow: 0.0}):
        rest_end_plate_z = _aabb_center_z(ctx.part_element_world_aabb(forearm, elem="end_plate"))
    with ctx.pose({shoulder: 0.45, elbow: 1.65}):
        bent_end_plate_z = _aabb_center_z(ctx.part_element_world_aabb(forearm, elem="end_plate"))
        ctx.expect_contact(
            upper_arm,
            forearm,
            elem_a="left_elbow_ear",
            elem_b="left_elbow_pad",
            name="left elbow thrust pad remains seated in a bent pose",
        )
        ctx.expect_contact(
            upper_arm,
            forearm,
            elem_a="right_elbow_ear",
            elem_b="right_elbow_pad",
            name="right elbow thrust pad remains seated in a bent pose",
        )
    ctx.check(
        "positive elbow rotation lifts the end plate",
        rest_end_plate_z is not None
        and bent_end_plate_z is not None
        and bent_end_plate_z > rest_end_plate_z + 0.10,
        details=f"rest_z={rest_end_plate_z}, bent_z={bent_end_plate_z}",
    )

    ctx.check(
        "plain end plate remains a simple centered terminus",
        isclose(forearm.get_visual("end_plate").origin.xyz[1], 0.0, abs_tol=1e-9)
        and isclose(forearm.get_visual("end_plate").origin.xyz[2], 0.0, abs_tol=1e-9),
        details=str(forearm.get_visual("end_plate").origin.xyz),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
