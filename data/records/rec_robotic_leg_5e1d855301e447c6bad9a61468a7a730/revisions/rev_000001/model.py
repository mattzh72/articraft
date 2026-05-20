from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


Y_CYLINDER = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=Y_CYLINDER.rpy)


def _add_side_bolt(part, *, x: float, y: float, z: float, radius: float, length: float, material, name: str) -> None:
    """Add a shallow socket-head bolt on a lateral side face."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_y_cylinder_origin(x, y, z),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="walking_machine_leg_module")

    dark_cast = model.material("dark_cast_aluminum", rgba=(0.10, 0.115, 0.125, 1.0))
    satin_metal = model.material("satin_machined_metal", rgba=(0.55, 0.57, 0.58, 1.0))
    black = model.material("black_hard_anodized", rgba=(0.015, 0.017, 0.018, 1.0))
    bolt = model.material("black_socket_bolts", rgba=(0.005, 0.005, 0.006, 1.0))
    warning = model.material("amber_service_covers", rgba=(0.95, 0.58, 0.12, 1.0))
    rubber = model.material("matte_rubber_pad", rgba=(0.025, 0.024, 0.023, 1.0))

    root = model.part("root_housing")
    # Fixed upper module: bolted flange, compact actuator spine, and hip yoke cheeks.
    root.visual(
        Box((0.36, 0.26, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=dark_cast,
        name="fixed_mount_flange",
    )
    root.visual(
        Box((0.23, 0.14, 0.110)),
        origin=Origin(xyz=(-0.015, 0.0, 0.140)),
        material=dark_cast,
        name="upper_actuator_spine",
    )
    root.visual(
        Box((0.205, 0.040, 0.190)),
        origin=Origin(xyz=(0.0, 0.105, 0.000)),
        material=dark_cast,
        name="hip_yoke_cheek_0",
    )
    root.visual(
        Box((0.205, 0.040, 0.190)),
        origin=Origin(xyz=(0.0, -0.105, 0.000)),
        material=dark_cast,
        name="hip_yoke_cheek_1",
    )
    root.visual(
        Box((0.205, 0.230, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_cast,
        name="hip_top_bridge",
    )
    root.visual(
        Box((0.040, 0.230, 0.110)),
        origin=Origin(xyz=(-0.098, 0.0, 0.020)),
        material=dark_cast,
        name="hip_rear_bridge",
    )
    for i, (x, y) in enumerate(((-0.135, -0.090), (-0.135, 0.090), (0.135, -0.090), (0.135, 0.090))):
        root.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, 0.198)),
            material=bolt,
            name=f"flange_bolt_{i}",
        )
    for i, y in enumerate((0.128, -0.128)):
        _add_side_bolt(root, x=0.052, y=y, z=0.052, radius=0.010, length=0.008, material=bolt, name=f"hip_bolt_top_{i}")
        _add_side_bolt(root, x=0.052, y=y, z=-0.052, radius=0.010, length=0.008, material=bolt, name=f"hip_bolt_lower_{i}")

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.064, length=0.170),
        origin=Y_CYLINDER,
        material=satin_metal,
        name="hip_rotor_barrel",
    )
    thigh.visual(
        Cylinder(radius=0.043, length=0.136),
        origin=Y_CYLINDER,
        material=black,
        name="hip_clamp_band",
    )
    thigh.visual(
        Box((0.078, 0.078, 0.365)),
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
        material=black,
        name="thigh_structural_box",
    )
    thigh.visual(
        Box((0.124, 0.016, 0.320)),
        origin=Origin(xyz=(0.0, 0.039, -0.240)),
        material=dark_cast,
        name="thigh_side_plate_0",
    )
    thigh.visual(
        Box((0.124, 0.016, 0.320)),
        origin=Origin(xyz=(0.0, -0.039, -0.240)),
        material=dark_cast,
        name="thigh_side_plate_1",
    )
    thigh.visual(
        Box((0.118, 0.102, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=warning,
        name="thigh_service_cover",
    )
    thigh.visual(
        Box((0.095, 0.170, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        material=dark_cast,
        name="knee_top_bridge",
    )
    thigh.visual(
        Box((0.032, 0.175, 0.118)),
        origin=Origin(xyz=(-0.090, 0.0, -0.480)),
        material=dark_cast,
        name="knee_rear_bridge",
    )
    thigh.visual(
        Box((0.168, 0.026, 0.140)),
        origin=Origin(xyz=(0.0, 0.075, -0.480)),
        material=dark_cast,
        name="knee_yoke_cheek_0",
    )
    thigh.visual(
        Box((0.168, 0.026, 0.140)),
        origin=Origin(xyz=(0.0, -0.075, -0.480)),
        material=dark_cast,
        name="knee_yoke_cheek_1",
    )
    for i, y in enumerate((0.070, -0.070)):
        _add_side_bolt(thigh, x=0.031, y=y, z=0.031, radius=0.0065, length=0.006, material=bolt, name=f"hip_cap_bolt_0_{i}")
        _add_side_bolt(thigh, x=-0.031, y=y, z=0.031, radius=0.0065, length=0.006, material=bolt, name=f"hip_cap_bolt_1_{i}")
        _add_side_bolt(thigh, x=0.031, y=y, z=-0.031, radius=0.0065, length=0.006, material=bolt, name=f"hip_cap_bolt_2_{i}")
        _add_side_bolt(thigh, x=-0.031, y=y, z=-0.031, radius=0.0065, length=0.006, material=bolt, name=f"hip_cap_bolt_3_{i}")
    for i, y in enumerate((0.091, -0.091)):
        _add_side_bolt(thigh, x=0.052, y=y, z=-0.438, radius=0.0075, length=0.007, material=bolt, name=f"knee_yoke_bolt_top_{i}")
        _add_side_bolt(thigh, x=0.052, y=y, z=-0.522, radius=0.0075, length=0.007, material=bolt, name=f"knee_yoke_bolt_lower_{i}")

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.052, length=0.124),
        origin=Y_CYLINDER,
        material=satin_metal,
        name="knee_rotor_barrel",
    )
    shank.visual(
        Cylinder(radius=0.036, length=0.106),
        origin=Y_CYLINDER,
        material=black,
        name="knee_clamp_band",
    )
    shank.visual(
        Box((0.064, 0.060, 0.310)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=black,
        name="shank_structural_box",
    )
    shank.visual(
        Box((0.104, 0.014, 0.270)),
        origin=Origin(xyz=(0.0, 0.032, -0.205)),
        material=dark_cast,
        name="shank_side_plate_0",
    )
    shank.visual(
        Box((0.104, 0.014, 0.270)),
        origin=Origin(xyz=(0.0, -0.032, -0.205)),
        material=dark_cast,
        name="shank_side_plate_1",
    )
    shank.visual(
        Box((0.084, 0.104, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.345)),
        material=dark_cast,
        name="ankle_top_bridge",
    )
    shank.visual(
        Box((0.026, 0.140, 0.094)),
        origin=Origin(xyz=(-0.070, 0.0, -0.410)),
        material=dark_cast,
        name="ankle_rear_bridge",
    )
    shank.visual(
        Box((0.132, 0.022, 0.110)),
        origin=Origin(xyz=(0.0, 0.060, -0.410)),
        material=dark_cast,
        name="ankle_yoke_cheek_0",
    )
    shank.visual(
        Box((0.132, 0.022, 0.110)),
        origin=Origin(xyz=(0.0, -0.060, -0.410)),
        material=dark_cast,
        name="ankle_yoke_cheek_1",
    )
    shank.visual(
        Box((0.088, 0.088, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=warning,
        name="shank_service_cover",
    )
    for i, y in enumerate((0.057, -0.057)):
        _add_side_bolt(shank, x=0.026, y=y, z=0.026, radius=0.0058, length=0.005, material=bolt, name=f"knee_cap_bolt_0_{i}")
        _add_side_bolt(shank, x=-0.026, y=y, z=0.026, radius=0.0058, length=0.005, material=bolt, name=f"knee_cap_bolt_1_{i}")
        _add_side_bolt(shank, x=0.026, y=y, z=-0.026, radius=0.0058, length=0.005, material=bolt, name=f"knee_cap_bolt_2_{i}")
        _add_side_bolt(shank, x=-0.026, y=y, z=-0.026, radius=0.0058, length=0.005, material=bolt, name=f"knee_cap_bolt_3_{i}")
    for i, y in enumerate((0.074, -0.074)):
        y_embedded = 0.073 if y > 0.0 else -0.073
        _add_side_bolt(shank, x=0.040, y=y_embedded, z=-0.382, radius=0.0065, length=0.008, material=bolt, name=f"ankle_yoke_bolt_top_{i}")
        _add_side_bolt(shank, x=0.040, y=y_embedded, z=-0.438, radius=0.0065, length=0.008, material=bolt, name=f"ankle_yoke_bolt_lower_{i}")

    foot = model.part("ankle_foot")
    foot.visual(
        Cylinder(radius=0.038, length=0.098),
        origin=Y_CYLINDER,
        material=satin_metal,
        name="ankle_rotor_barrel",
    )
    foot.visual(
        Cylinder(radius=0.027, length=0.082),
        origin=Y_CYLINDER,
        material=black,
        name="ankle_clamp_band",
    )
    foot.visual(
        Box((0.050, 0.052, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=dark_cast,
        name="ankle_neck_block",
    )
    foot.visual(
        Box((0.115, 0.090, 0.052)),
        origin=Origin(xyz=(0.030, 0.0, -0.090)),
        material=dark_cast,
        name="compact_ankle_carrier",
    )
    foot.visual(
        Box((0.260, 0.120, 0.034)),
        origin=Origin(xyz=(0.050, 0.0, -0.132)),
        material=rubber,
        name="rubber_foot_pad",
    )
    foot.visual(
        Box((0.090, 0.100, 0.016)),
        origin=Origin(xyz=(0.130, 0.0, -0.128)),
        material=satin_metal,
        name="toe_wear_plate",
    )
    foot.visual(
        Box((0.070, 0.100, 0.016)),
        origin=Origin(xyz=(-0.055, 0.0, -0.128)),
        material=satin_metal,
        name="heel_wear_plate",
    )
    for i, y in enumerate((0.045, -0.045)):
        _add_side_bolt(foot, x=0.020, y=y, z=0.020, radius=0.0048, length=0.005, material=bolt, name=f"ankle_cap_bolt_0_{i}")
        _add_side_bolt(foot, x=-0.020, y=y, z=0.020, radius=0.0048, length=0.005, material=bolt, name=f"ankle_cap_bolt_1_{i}")
        _add_side_bolt(foot, x=0.020, y=y, z=-0.020, radius=0.0048, length=0.005, material=bolt, name=f"ankle_cap_bolt_2_{i}")
        _add_side_bolt(foot, x=-0.020, y=y, z=-0.020, radius=0.0048, length=0.005, material=bolt, name=f"ankle_cap_bolt_3_{i}")

    model.articulation(
        "hip_joint",
        ArticulationType.REVOLUTE,
        parent=root,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=4.0, lower=-0.75, upper=0.95),
    )
    model.articulation(
        "knee_joint",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=230.0, velocity=5.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "ankle_joint",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.410)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=5.5, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip = object_model.get_articulation("hip_joint")
    knee = object_model.get_articulation("knee_joint")
    ankle = object_model.get_articulation("ankle_joint")
    root = object_model.get_part("root_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")

    ctx.check(
        "three lateral revolute axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[0]) < 1e-6
            and abs(abs(joint.axis[1]) - 1.0) < 1e-6
            and abs(joint.axis[2]) < 1e-6
            for joint in (hip, knee, ankle)
        ),
        details=f"axes={[hip.axis, knee.axis, ankle.axis]}",
    )
    ctx.expect_origin_gap(thigh, shank, axis="z", min_gap=0.46, max_gap=0.50, name="thigh spans hip to knee")
    ctx.expect_origin_gap(shank, foot, axis="z", min_gap=0.39, max_gap=0.43, name="shank spans knee to ankle")
    ctx.expect_contact(root, thigh, elem_a="hip_yoke_cheek_0", elem_b="hip_rotor_barrel", contact_tol=0.001, name="hip barrel seats against yoke cheek")
    ctx.expect_contact(root, thigh, elem_a="hip_yoke_cheek_1", elem_b="hip_rotor_barrel", contact_tol=0.001, name="hip barrel seats against opposite cheek")
    ctx.expect_contact(thigh, shank, elem_a="knee_yoke_cheek_0", elem_b="knee_rotor_barrel", contact_tol=0.001, name="knee barrel seats in thigh module")
    ctx.expect_contact(thigh, shank, elem_a="knee_yoke_cheek_1", elem_b="knee_rotor_barrel", contact_tol=0.001, name="knee barrel seats against opposite cheek")
    ctx.expect_contact(shank, foot, elem_a="ankle_yoke_cheek_0", elem_b="ankle_rotor_barrel", contact_tol=0.001, name="ankle barrel seats in shank module")
    ctx.expect_contact(shank, foot, elem_a="ankle_yoke_cheek_1", elem_b="ankle_rotor_barrel", contact_tol=0.001, name="ankle barrel seats against opposite cheek")

    rest_knee = ctx.part_world_position(shank)
    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.45, knee: 0.70, ankle: -0.25}):
        posed_knee = ctx.part_world_position(shank)
        posed_foot = ctx.part_world_position(foot)
    ctx.check(
        "hip flexion moves knee forward",
        rest_knee is not None and posed_knee is not None and posed_knee[0] > rest_knee[0] + 0.15,
        details=f"rest={rest_knee}, posed={posed_knee}",
    )
    ctx.check(
        "compound joints move foot",
        rest_foot is not None and posed_foot is not None and posed_foot[0] > rest_foot[0] + 0.30,
        details=f"rest={rest_foot}, posed={posed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
