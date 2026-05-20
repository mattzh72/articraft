from __future__ import annotations

from math import pi

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


def _box(part, name: str, size, xyz, material: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_y(part, name: str, radius: float, length: float, xyz, material: str) -> None:
    # SDK cylinders are local-Z; rotate into the lateral bearing axis.
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_robotic_leg")

    model.material("aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("hard_anodized", rgba=(0.19, 0.21, 0.23, 1.0))
    model.material("dark_recess", rgba=(0.045, 0.050, 0.055, 1.0))
    model.material("datum_white", rgba=(0.92, 0.93, 0.88, 1.0))
    model.material("laser_black", rgba=(0.00, 0.00, 0.00, 1.0))
    model.material("blue_index", rgba=(0.08, 0.22, 0.70, 1.0))
    model.material("rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    # Root metrology fixture: a stiff hip yoke with two cheeks, a top datum
    # plate, bearing caps, and visible zero marks.  The lateral gap is larger
    # than the moving hip lug, so the joint reads as controlled clearance
    # rather than accidental collision.
    hip_fixture = model.part("hip_fixture")
    _box(hip_fixture, "datum_plate", (0.220, 0.280, 0.040), (-0.018, 0.0, 0.100), "aluminum")
    _box(hip_fixture, "rear_datum_bar", (0.034, 0.190, 0.130), (-0.080, 0.0, 0.016), "aluminum")
    _box(hip_fixture, "hip_cheek_pos", (0.145, 0.024, 0.170), (0.0, 0.082, 0.000), "aluminum")
    _box(hip_fixture, "hip_cheek_neg", (0.145, 0.024, 0.170), (0.0, -0.082, 0.000), "aluminum")
    _cyl_y(hip_fixture, "hip_bearing_pos", 0.062, 0.020, (0.0, 0.100, 0.0), "hard_anodized")
    _cyl_y(hip_fixture, "hip_bearing_neg", 0.062, 0.020, (0.0, -0.100, 0.0), "hard_anodized")
    for i, z in enumerate((-0.044, -0.022, 0.000, 0.022, 0.044)):
        _box(hip_fixture, f"hip_zero_mark_{i}", (0.004, 0.014, 0.003), (0.073, 0.082, z), "datum_white")
    _box(hip_fixture, "datum_pad_front", (0.052, 0.040, 0.004), (0.055, 0.078, 0.121), "datum_white")
    _box(hip_fixture, "datum_pad_rear", (0.052, 0.040, 0.004), (-0.070, -0.078, 0.121), "datum_white")
    _box(hip_fixture, "serial_tag", (0.003, 0.076, 0.022), (-0.098, 0.0, 0.060), "laser_black")

    # Upper limb shell: compact hip lug, calibrated actuator bay on the front
    # face, lower clevis cheeks for the knee, and index lines that are slightly
    # proud of the load-bearing shell.
    thigh = model.part("thigh")
    _cyl_y(thigh, "hip_lug", 0.045, 0.082, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(thigh, "hip_axle", 0.018, 0.140, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(thigh, "hip_spacer_pos", 0.052, 0.006, (0.0, 0.043, 0.0), "dark_recess")
    _cyl_y(thigh, "hip_spacer_neg", 0.052, 0.006, (0.0, -0.043, 0.0), "dark_recess")
    _box(thigh, "hip_transition_block", (0.120, 0.078, 0.055), (0.0, 0.0, -0.055), "aluminum")
    _box(thigh, "thigh_shell", (0.115, 0.075, 0.340), (0.0, 0.0, -0.235), "aluminum")
    _box(thigh, "thigh_front_bay", (0.010, 0.065, 0.220), (0.061, 0.0, -0.235), "dark_recess")
    _box(thigh, "thigh_side_datum", (0.082, 0.008, 0.280), (0.0, -0.041, -0.235), "datum_white")
    _box(thigh, "thigh_adjust_slot", (0.004, 0.045, 0.120), (0.068, 0.0, -0.245), "laser_black")
    for i, z in enumerate((-0.150, -0.205, -0.260, -0.315)):
        _box(thigh, f"thigh_scale_{i}", (0.026, 0.004, 0.003), (0.026, 0.039, z), "blue_index")
    _box(thigh, "knee_cheek_pos", (0.130, 0.022, 0.135), (0.0, 0.065, -0.445), "aluminum")
    _box(thigh, "knee_cheek_neg", (0.130, 0.022, 0.135), (0.0, -0.065, -0.445), "aluminum")
    _box(thigh, "knee_rear_bridge", (0.028, 0.142, 0.052), (-0.066, 0.0, -0.405), "aluminum")
    _cyl_y(thigh, "knee_bearing_pos", 0.052, 0.010, (0.0, 0.080, -0.445), "hard_anodized")
    _cyl_y(thigh, "knee_bearing_neg", 0.052, 0.010, (0.0, -0.080, -0.445), "hard_anodized")
    for i, x in enumerate((-0.036, -0.012, 0.012, 0.036)):
        _box(thigh, f"knee_index_{i}", (0.003, 0.012, 0.004), (x, 0.080, -0.390), "datum_white")

    # Lower limb shell: similar but narrower, with an ankle clevis and a second
    # adjustment bay so the stack reads as a repeatable calibration fixture.
    shank = model.part("shank")
    _cyl_y(shank, "knee_lug", 0.041, 0.080, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(shank, "knee_axle", 0.016, 0.108, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(shank, "knee_spacer_pos", 0.047, 0.006, (0.0, 0.042, 0.0), "dark_recess")
    _cyl_y(shank, "knee_spacer_neg", 0.047, 0.006, (0.0, -0.042, 0.0), "dark_recess")
    _box(shank, "knee_transition_block", (0.105, 0.070, 0.052), (0.0, 0.0, -0.055), "aluminum")
    _box(shank, "shank_shell", (0.100, 0.068, 0.325), (0.0, 0.0, -0.225), "aluminum")
    _box(shank, "shank_front_bay", (0.010, 0.058, 0.200), (0.055, 0.0, -0.230), "dark_recess")
    _box(shank, "shank_side_datum", (0.072, 0.007, 0.260), (0.0, -0.037, -0.225), "datum_white")
    _box(shank, "shank_adjust_slot", (0.004, 0.040, 0.110), (0.062, 0.0, -0.238), "laser_black")
    for i, z in enumerate((-0.145, -0.200, -0.255, -0.310)):
        _box(shank, f"shank_scale_{i}", (0.022, 0.004, 0.003), (0.022, 0.036, z), "blue_index")
    _box(shank, "ankle_cheek_pos", (0.115, 0.020, 0.120), (0.0, 0.056, -0.425), "aluminum")
    _box(shank, "ankle_cheek_neg", (0.115, 0.020, 0.120), (0.0, -0.056, -0.425), "aluminum")
    _box(shank, "ankle_rear_bridge", (0.030, 0.122, 0.050), (-0.058, 0.0, -0.388), "aluminum")
    _cyl_y(shank, "ankle_bearing_pos", 0.045, 0.010, (0.0, 0.068, -0.425), "hard_anodized")
    _cyl_y(shank, "ankle_bearing_neg", 0.045, 0.010, (0.0, -0.068, -0.425), "hard_anodized")
    for i, x in enumerate((-0.030, -0.010, 0.010, 0.030)):
        _box(shank, f"ankle_index_{i}", (0.003, 0.010, 0.004), (x, 0.068, -0.377), "datum_white")

    # Foot/ankle link: a compact ankle lug, vertical load path, broad flat
    # contact plate, replaceable rubber sole, and toe datum marks.
    foot = model.part("foot")
    _cyl_y(foot, "ankle_lug", 0.034, 0.068, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(foot, "ankle_axle", 0.014, 0.092, (0.0, 0.0, 0.0), "hard_anodized")
    _cyl_y(foot, "ankle_spacer_pos", 0.039, 0.006, (0.0, 0.036, 0.0), "dark_recess")
    _cyl_y(foot, "ankle_spacer_neg", 0.039, 0.006, (0.0, -0.036, 0.0), "dark_recess")
    _box(foot, "ankle_strut", (0.070, 0.050, 0.100), (0.0, 0.0, -0.070), "aluminum")
    _box(foot, "foot_plate", (0.340, 0.110, 0.035), (0.065, 0.0, -0.132), "aluminum")
    _box(foot, "rubber_sole", (0.350, 0.115, 0.018), (0.065, 0.0, -0.154), "rubber")
    _box(foot, "toe_datum_pad", (0.105, 0.080, 0.004), (0.155, 0.0, -0.1125), "datum_white")
    _box(foot, "heel_stop", (0.025, 0.115, 0.055), (-0.108, 0.0, -0.128), "hard_anodized")
    for i, y in enumerate((-0.036, -0.018, 0.000, 0.018, 0.036)):
        _box(foot, f"toe_index_{i}", (0.045, 0.003, 0.003), (0.207, y, -0.109), "blue_index")

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_fixture,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.75, upper=1.15, effort=180.0, velocity=2.0),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.445)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=150.0, velocity=2.2),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.425)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.65, effort=90.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_fixture = object_model.get_part("hip_fixture")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    for joint_name, low, high in (
        ("hip_pitch", -0.75, 1.15),
        ("knee_pitch", 0.0, 1.85),
        ("ankle_pitch", -0.70, 0.65),
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} has calibrated limits",
            limits is not None and limits.lower == low and limits.upper == high,
            details=f"{joint_name} limits={limits}",
        )

    # Element-level gap checks document the deliberate clearance between every
    # moving lug and the adjacent clevis cheeks at the calibrated zero pose.
    ctx.expect_gap(
        hip_fixture,
        thigh,
        axis="y",
        min_gap=0.020,
        positive_elem="hip_cheek_pos",
        negative_elem="hip_lug",
        name="hip positive cheek clearance",
    )
    ctx.expect_gap(
        thigh,
        hip_fixture,
        axis="y",
        min_gap=0.020,
        positive_elem="hip_lug",
        negative_elem="hip_cheek_neg",
        name="hip negative cheek clearance",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="y",
        min_gap=0.010,
        positive_elem="knee_cheek_pos",
        negative_elem="knee_lug",
        name="knee positive cheek clearance",
    )
    ctx.expect_gap(
        shank,
        thigh,
        axis="y",
        min_gap=0.010,
        positive_elem="knee_lug",
        negative_elem="knee_cheek_neg",
        name="knee negative cheek clearance",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="y",
        min_gap=0.008,
        positive_elem="ankle_cheek_pos",
        negative_elem="ankle_lug",
        name="ankle positive cheek clearance",
    )
    ctx.expect_gap(
        foot,
        shank,
        axis="y",
        min_gap=0.008,
        positive_elem="ankle_lug",
        negative_elem="ankle_cheek_neg",
        name="ankle negative cheek clearance",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.60}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "hip pitch moves the serial chain",
        rest_foot is not None
        and flexed_foot is not None
        and flexed_foot[0] < rest_foot[0] - 0.25
        and flexed_foot[2] > rest_foot[2] + 0.10,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    with ctx.pose({knee: 1.00}):
        knee_flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee pitch folds the distal links",
        rest_foot is not None
        and knee_flexed_foot is not None
        and knee_flexed_foot[0] < rest_foot[0] - 0.25
        and knee_flexed_foot[2] > rest_foot[2] + 0.08,
        details=f"rest={rest_foot}, knee_flexed={knee_flexed_foot}",
    )

    rest_plate = ctx.part_element_world_aabb(foot, elem="foot_plate")
    with ctx.pose({ankle: -0.50}):
        dorsiflexed_plate = ctx.part_element_world_aabb(foot, elem="foot_plate")
    ctx.check(
        "ankle pitch rotates the datum foot plate",
        rest_plate is not None
        and dorsiflexed_plate is not None
        and dorsiflexed_plate[1][2] > rest_plate[1][2] + 0.04,
        details=f"rest_plate={rest_plate}, dorsiflexed_plate={dorsiflexed_plate}",
    )

    return ctx.report()


object_model = build_object_model()
