from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, radius, length, xyz, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_study")

    bead_aluminum = model.material("bead_blasted_aluminum", rgba=(0.60, 0.62, 0.60, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    rail_steel = model.material("ground_bearing_steel", rgba=(0.78, 0.80, 0.79, 1.0))
    hardcoat = model.material("dark_hardcoat", rgba=(0.18, 0.20, 0.22, 1.0))
    bronze = model.material("oil_bronze", rgba=(0.77, 0.55, 0.26, 1.0))
    rubber = model.material("matte_stop_rubber", rgba=(0.03, 0.03, 0.025, 1.0))
    safety_orange = model.material("orange_limit_tabs", rgba=(0.95, 0.38, 0.08, 1.0))

    base = model.part("base")
    _box(base, (1.05, 0.44, 0.035), (0.02, 0.0, 0.0175), bead_aluminum, "bed_plate")
    _box(base, (0.88, 0.035, 0.050), (0.02, -0.135, 0.060), rail_steel, "rail_0")
    _box(base, (0.88, 0.035, 0.050), (0.02, 0.135, 0.060), rail_steel, "rail_1")
    _box(base, (0.92, 0.028, 0.018), (0.02, -0.190, 0.044), dark_steel, "outer_guide_fence_0")
    _box(base, (0.92, 0.028, 0.018), (0.02, 0.190, 0.044), dark_steel, "outer_guide_fence_1")

    # Rigid end brackets and travel stops.  The orange faces are replaceable
    # bumper pads mounted to fabricated stop towers rather than decoration.
    _box(base, (0.040, 0.205, 0.090), (-0.460, 0.0, 0.080), dark_steel, "rear_stop")
    _box(base, (0.040, 0.205, 0.090), (0.390, 0.0, 0.080), dark_steel, "front_stop")
    _box(base, (0.010, 0.120, 0.055), (-0.435, 0.0, 0.090), rubber, "rear_bumper")
    _box(base, (0.010, 0.120, 0.055), (0.365, 0.0, 0.090), rubber, "front_bumper")
    _box(base, (0.070, 0.030, 0.060), (-0.455, -0.135, 0.072), dark_steel, "rear_stop_foot_0")
    _box(base, (0.070, 0.030, 0.060), (-0.455, 0.135, 0.072), dark_steel, "rear_stop_foot_1")
    _box(base, (0.070, 0.030, 0.060), (0.390, -0.135, 0.072), dark_steel, "front_stop_foot_0")
    _box(base, (0.070, 0.030, 0.060), (0.390, 0.135, 0.072), dark_steel, "front_stop_foot_1")

    for i, x in enumerate((-0.40, -0.20, 0.00, 0.20, 0.40)):
        _cylinder(base, 0.008, 0.004, (x, -0.135, 0.087), dark_steel, f"rail_screw_0_{i}")
        _cylinder(base, 0.008, 0.004, (x, 0.135, 0.087), dark_steel, f"rail_screw_1_{i}")
    for i, y in enumerate((-0.075, -0.025, 0.025, 0.075)):
        _cylinder(base, 0.009, 0.004, (-0.460, y, 0.127), rail_steel, f"rear_stop_cap_{i}")
        _cylinder(base, 0.009, 0.004, (0.390, y, 0.127), rail_steel, f"front_stop_cap_{i}")

    carriage = model.part("carriage")
    # Child frame origin sits at rail top at the rear travel position.
    for suffix, x in (("rear", -0.100), ("front", 0.100)):
        _box(carriage, (0.110, 0.070, 0.040), (x, -0.135, 0.020), hardcoat, f"truck_{suffix}_0")
        _box(carriage, (0.110, 0.070, 0.040), (x, 0.135, 0.020), hardcoat, f"truck_{suffix}_1")
        _box(carriage, (0.090, 0.012, 0.024), (x, -0.135, 0.012), rail_steel, f"truck_wiper_{suffix}_0")
        _box(carriage, (0.090, 0.012, 0.024), (x, 0.135, 0.012), rail_steel, f"truck_wiper_{suffix}_1")

    _box(carriage, (0.370, 0.315, 0.030), (0.020, 0.0, 0.055), bead_aluminum, "carriage_plate")
    _box(carriage, (0.300, 0.030, 0.045), (0.030, -0.155, 0.076), bead_aluminum, "side_rib_0")
    _box(carriage, (0.300, 0.030, 0.045), (0.030, 0.155, 0.076), bead_aluminum, "side_rib_1")
    _box(carriage, (0.030, 0.240, 0.050), (-0.155, 0.0, 0.079), bead_aluminum, "rear_cross_rib")
    _box(carriage, (0.030, 0.240, 0.050), (0.175, 0.0, 0.079), bead_aluminum, "front_cross_rib")

    # Stop strikers on the moving stage: rear one contacts at q=0, front one at
    # the prismatic upper limit.
    _box(carriage, (0.050, 0.105, 0.045), (-0.185, 0.0, 0.035), safety_orange, "rear_striker")
    _box(carriage, (0.060, 0.105, 0.045), (0.230, 0.0, 0.035), safety_orange, "front_striker")

    # Clevis head fabricated on the carriage.  Cheeks, bridge, bushings, and
    # gussets make the downstream revolute support explicit.
    _box(carriage, (0.170, 0.245, 0.026), (0.215, 0.0, 0.083), dark_steel, "clevis_foot")
    _box(carriage, (0.060, 0.145, 0.085), (0.225, 0.0, 0.123), dark_steel, "clevis_bridge")
    _box(carriage, (0.115, 0.025, 0.145), (0.300, -0.065, 0.140), dark_steel, "cheek_0")
    _box(carriage, (0.115, 0.025, 0.145), (0.300, 0.065, 0.140), dark_steel, "cheek_1")
    _box(carriage, (0.100, 0.018, 0.045), (0.260, -0.042, 0.085), dark_steel, "gusset_0")
    _box(carriage, (0.100, 0.018, 0.045), (0.260, 0.042, 0.085), dark_steel, "gusset_1")
    _cylinder(
        carriage,
        0.034,
        0.008,
        (0.300, -0.0485, 0.140),
        bronze,
        "bearing_0",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    _cylinder(
        carriage,
        0.034,
        0.008,
        (0.300, 0.0485, 0.140),
        bronze,
        "bearing_1",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    _cylinder(carriage, 0.007, 0.006, (0.255, -0.092, 0.096), rail_steel, "clevis_bolt_0")
    _cylinder(carriage, 0.007, 0.006, (0.255, 0.092, 0.096), rail_steel, "clevis_bolt_1")
    _cylinder(carriage, 0.007, 0.006, (0.293, -0.092, 0.096), rail_steel, "clevis_bolt_2")
    _cylinder(carriage, 0.007, 0.006, (0.293, 0.092, 0.096), rail_steel, "clevis_bolt_3")

    rotary_arm = model.part("rotary_arm")
    _cylinder(
        rotary_arm,
        0.026,
        0.084,
        (0.0, 0.0, 0.0),
        rail_steel,
        "trunnion",
        rpy=(-pi / 2.0, 0.0, 0.0),
    )
    _box(rotary_arm, (0.240, 0.046, 0.036), (0.120, 0.0, 0.000), bead_aluminum, "output_arm")
    _box(rotary_arm, (0.055, 0.075, 0.060), (0.250, 0.0, 0.000), dark_steel, "distal_flange")
    _cylinder(rotary_arm, 0.008, 0.080, (0.252, 0.0, 0.018), rail_steel, "flange_pin_upper", rpy=(-pi / 2.0, 0.0, 0.0))
    _cylinder(rotary_arm, 0.008, 0.080, (0.252, 0.0, -0.018), rail_steel, "flange_pin_lower", rpy=(-pi / 2.0, 0.0, 0.0))
    _box(rotary_arm, (0.055, 0.010, 0.050), (0.035, -0.0395, 0.000), bronze, "thrust_washer_0")
    _box(rotary_arm, (0.055, 0.010, 0.050), (0.035, 0.0395, 0.000), bronze, "thrust_washer_1")

    base_cover = model.part("base_cover")
    _box(base_cover, (0.520, 0.098, 0.006), (0.0, 0.0, 0.003), hardcoat, "cover_panel")
    for i, x in enumerate((-0.220, -0.075, 0.075, 0.220)):
        _cylinder(base_cover, 0.006, 0.003, (x, -0.035, 0.007), dark_steel, f"cover_screw_0_{i}")
        _cylinder(base_cover, 0.006, 0.003, (x, 0.035, 0.007), dark_steel, f"cover_screw_1_{i}")

    carriage_cover = model.part("carriage_cover")
    _box(carriage_cover, (0.160, 0.145, 0.006), (0.0, 0.0, 0.003), hardcoat, "cover_panel")
    _box(carriage_cover, (0.040, 0.135, 0.004), (-0.060, 0.0, 0.008), rail_steel, "lift_tab")
    for i, x in enumerate((-0.060, 0.060)):
        _cylinder(carriage_cover, 0.006, 0.003, (x, -0.050, 0.007), dark_steel, f"cover_screw_0_{i}")
        _cylinder(carriage_cover, 0.006, 0.003, (x, 0.050, 0.007), dark_steel, f"cover_screw_1_{i}")

    slide = model.articulation(
        "slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.220, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.18, lower=0.0, upper=0.320),
        motion_properties=MotionProperties(damping=18.0, friction=5.0),
    )
    slide.meta["purpose"] = "linear carriage feed along twin exposed guide rails"

    hinge = model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=rotary_arm,
        origin=Origin(xyz=(0.300, 0.0, 0.140)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.20, upper=1.10),
        motion_properties=MotionProperties(damping=6.0, friction=1.2),
    )
    hinge.meta["purpose"] = "distal revolute output carried by clevis bushings"

    model.articulation(
        "base_cover_mount",
        ArticulationType.FIXED,
        parent=base,
        child=base_cover,
        origin=Origin(xyz=(0.020, 0.0, 0.035)),
    )
    model.articulation(
        "carriage_cover_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child=carriage_cover,
        origin=Origin(xyz=(-0.035, 0.0, 0.070)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    rotary_arm = object_model.get_part("rotary_arm")
    base_cover = object_model.get_part("base_cover")
    carriage_cover = object_model.get_part("carriage_cover")
    slide = object_model.get_articulation("slide")
    hinge = object_model.get_articulation("hinge")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="truck_rear_0",
        elem_b="rail_0",
        contact_tol=0.001,
        name="rear bearing truck sits on rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="truck_front_1",
        elem_b="rail_1",
        contact_tol=0.001,
        name="front bearing truck sits on rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="rear_striker",
        elem_b="rear_bumper",
        contact_tol=0.001,
        name="lower travel stop is seated",
    )
    ctx.expect_gap(
        carriage,
        rotary_arm,
        axis="y",
        positive_elem="bearing_1",
        negative_elem="trunnion",
        min_gap=0.001,
        max_gap=0.006,
        name="upper clevis bushing clears trunnion",
    )
    ctx.expect_gap(
        rotary_arm,
        carriage,
        axis="y",
        positive_elem="trunnion",
        negative_elem="bearing_0",
        min_gap=0.001,
        max_gap=0.006,
        name="lower clevis bushing clears trunnion",
    )
    ctx.expect_contact(
        base_cover,
        base,
        elem_a="cover_panel",
        elem_b="bed_plate",
        contact_tol=0.001,
        name="base access cover is seated on bed",
    )
    ctx.expect_contact(
        carriage_cover,
        carriage,
        elem_a="cover_panel",
        elem_b="carriage_plate",
        contact_tol=0.001,
        name="carriage access cover is seated",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.320}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            base,
            elem_a="front_striker",
            elem_b="front_bumper",
            contact_tol=0.001,
            name="upper travel stop is seated",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a="truck_front_0",
            elem_b="rail_0",
            contact_tol=0.001,
            name="extended slide remains on rail",
        )
    ctx.check(
        "slide moves in positive x",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.30,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_tip = ctx.part_element_world_aabb(rotary_arm, elem="distal_flange")
    with ctx.pose({hinge: 1.0}):
        raised_tip = ctx.part_element_world_aabb(rotary_arm, elem="distal_flange")
    closed_z = None if closed_tip is None else (closed_tip[0][2] + closed_tip[1][2]) * 0.5
    raised_z = None if raised_tip is None else (raised_tip[0][2] + raised_tip[1][2]) * 0.5
    ctx.check(
        "hinge raises distal flange",
        closed_z is not None and raised_z is not None and raised_z > closed_z + 0.10,
        details=f"closed_z={closed_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
