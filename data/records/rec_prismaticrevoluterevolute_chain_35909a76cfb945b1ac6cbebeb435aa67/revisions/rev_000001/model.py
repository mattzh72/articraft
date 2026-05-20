from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PI = math.pi


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, axis="z"):
    if axis == "x":
        rpy = (0.0, PI / 2.0, 0.0)
    elif axis == "y":
        rpy = (-PI / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _slot_cover(name, panel_size, thickness, *, slot_size, pitch, frame=0.012):
    return mesh_from_geometry(
        SlotPatternPanelGeometry(
            panel_size,
            thickness,
            slot_size=slot_size,
            pitch=pitch,
            frame=frame,
            corner_radius=0.004,
            stagger=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_revolute_study")

    model.material("dark_oxide", rgba=(0.05, 0.055, 0.06, 1.0))
    model.material("blued_steel", rgba=(0.16, 0.19, 0.21, 1.0))
    model.material("ground_rail", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("machined_aluminum", rgba=(0.58, 0.61, 0.62, 1.0))
    model.material("brushed_cover", rgba=(0.40, 0.42, 0.42, 1.0))
    model.material("bearing_black", rgba=(0.015, 0.015, 0.016, 1.0))
    model.material("fastener", rgba=(0.09, 0.095, 0.10, 1.0))

    base = model.part("linear_base")
    _box(base, "base_plate", (1.10, 0.32, 0.035), (0.0, 0.0, 0.0175), "dark_oxide")
    _box(base, "rail_bed_0", (1.02, 0.040, 0.035), (0.0, -0.085, 0.0525), "blued_steel")
    _box(base, "rail_bed_1", (1.02, 0.040, 0.035), (0.0, 0.085, 0.0525), "blued_steel")
    _cyl(base, "linear_rail_0", 0.012, 1.00, (0.0, -0.085, 0.0815), "ground_rail", axis="x")
    _cyl(base, "linear_rail_1", 0.012, 1.00, (0.0, 0.085, 0.0815), "ground_rail", axis="x")
    _box(base, "end_stop_neg", (0.040, 0.265, 0.088), (-0.525, 0.0, 0.079), "blued_steel")
    _box(base, "end_stop_pos", (0.040, 0.265, 0.088), (0.525, 0.0, 0.079), "blued_steel")
    _box(base, "bearing_block_neg", (0.040, 0.060, 0.060), (-0.465, 0.0, 0.065), "machined_aluminum")
    _box(base, "bearing_block_pos", (0.040, 0.060, 0.060), (0.465, 0.0, 0.065), "machined_aluminum")
    _cyl(base, "drive_screw", 0.006, 0.93, (0.0, 0.0, 0.088), "ground_rail", axis="x")
    _cyl(base, "stop_bumper_neg", 0.014, 0.010, (-0.501, 0.0, 0.100), "bearing_black", axis="x")
    _cyl(base, "stop_bumper_pos", 0.014, 0.010, (0.501, 0.0, 0.100), "bearing_black", axis="x")
    _box(base, "left_access_cover", (0.30, 0.038, 0.006), (0.0, -0.142, 0.038), "brushed_cover")
    _box(base, "right_access_cover", (0.30, 0.038, 0.006), (0.0, 0.142, 0.038), "brushed_cover")
    for side, y in (("left", -0.142), ("right", 0.142)):
        for i, x in enumerate((-0.120, 0.120)):
            _cyl(base, f"{side}_cover_screw_{i}", 0.005, 0.004, (x, y, 0.043), "fastener", axis="z")

    turret = model.part("turret")
    _box(turret, "carriage_plate", (0.230, 0.235, 0.026), (0.0, 0.0, 0.132), "machined_aluminum")
    for idx, rail_y in enumerate((-0.085, 0.085)):
        _box(turret, f"guide_bridge_{idx}", (0.172, 0.062, 0.018), (0.0, rail_y, 0.1025), "machined_aluminum")
        _box(turret, f"guide_cheek_{idx}_inner", (0.172, 0.008, 0.049), (0.0, rail_y * 0.705, 0.105), "machined_aluminum")
        _box(turret, f"guide_cheek_{idx}_outer", (0.172, 0.008, 0.049), (0.0, rail_y * 1.295, 0.105), "machined_aluminum")
    _box(turret, "nut_bridge", (0.095, 0.056, 0.014), (0.0, 0.0, 0.114), "machined_aluminum")
    _box(turret, "nut_lug_neg", (0.085, 0.014, 0.030), (0.0, -0.022, 0.093), "machined_aluminum")
    _box(turret, "nut_lug_pos", (0.085, 0.014, 0.030), (0.0, 0.022, 0.093), "machined_aluminum")
    _cyl(turret, "lower_bearing_ring", 0.088, 0.018, (0.0, 0.0, 0.154), "bearing_black", axis="z")
    _cyl(turret, "turret_register", 0.062, 0.025, (0.0, 0.0, 0.176), "ground_rail", axis="z")
    _box(turret, "riser_block", (0.120, 0.120, 0.180), (0.0, 0.0, 0.235), "machined_aluminum")
    turret_cover = _slot_cover(
        "turret_access_cover_mesh",
        (0.076, 0.096),
        0.004,
        slot_size=(0.020, 0.006),
        pitch=(0.030, 0.020),
        frame=0.010,
    )
    turret.visual(
        turret_cover,
        origin=Origin(xyz=(0.0, 0.0618, 0.235), rpy=(-PI / 2.0, 0.0, 0.0)),
        material="brushed_cover",
        name="turret_access_cover",
    )
    for x in (-0.031, 0.031):
        for z in (0.198, 0.272):
            _cyl(turret, f"turret_cover_screw_{x}_{z}", 0.0045, 0.004, (x, 0.0645, z), "fastener", axis="y")
    _box(turret, "hinge1_saddle", (0.145, 0.170, 0.026), (0.0, 0.0, 0.338), "machined_aluminum")
    _box(turret, "hinge1_cheek_neg", (0.072, 0.018, 0.112), (0.0, -0.074, 0.386), "machined_aluminum")
    _box(turret, "hinge1_cheek_pos", (0.072, 0.018, 0.112), (0.0, 0.074, 0.386), "machined_aluminum")
    _box(turret, "hinge1_rear_bridge", (0.018, 0.166, 0.070), (-0.062, 0.0, 0.382), "machined_aluminum")
    _cyl(turret, "hinge1_boss_neg", 0.037, 0.012, (0.0, -0.089, 0.386), "blued_steel", axis="y")
    _cyl(turret, "hinge1_boss_pos", 0.037, 0.012, (0.0, 0.089, 0.386), "blued_steel", axis="y")
    for y in (-0.089, 0.089):
        _cyl(turret, f"hinge1_pin_cap_{y}", 0.018, 0.005, (0.0, y, 0.386), "fastener", axis="y")

    arm = model.part("first_arm")
    _cyl(arm, "first_hub", 0.036, 0.100, (0.0, 0.0, 0.0), "ground_rail", axis="y")
    _cyl(arm, "first_pin", 0.015, 0.126, (0.0, 0.0, 0.0), "fastener", axis="y")
    _box(arm, "arm_plate_neg", (0.260, 0.014, 0.045), (0.165, -0.032, 0.0), "blued_steel")
    _box(arm, "arm_plate_pos", (0.260, 0.014, 0.045), (0.165, 0.032, 0.0), "blued_steel")
    _box(arm, "arm_top_spacer", (0.170, 0.075, 0.014), (0.165, 0.0, 0.027), "machined_aluminum")
    _box(arm, "arm_lower_spacer", (0.115, 0.064, 0.012), (0.115, 0.0, -0.027), "machined_aluminum")
    arm_cover = _slot_cover(
        "arm_access_cover_mesh",
        (0.150, 0.052),
        0.004,
        slot_size=(0.026, 0.005),
        pitch=(0.040, 0.016),
        frame=0.009,
    )
    arm.visual(
        arm_cover,
        origin=Origin(xyz=(0.165, 0.0, 0.036)),
        material="brushed_cover",
        name="arm_access_cover",
    )
    for x in (0.100, 0.230):
        for y in (-0.018, 0.018):
            _cyl(arm, f"arm_cover_screw_{x}_{y}", 0.0038, 0.006, (x, y, 0.041), "fastener", axis="z")
    _box(arm, "hinge2_root_block", (0.028, 0.112, 0.050), (0.270, 0.0, 0.0), "machined_aluminum")
    _box(arm, "hinge2_cheek_neg", (0.058, 0.012, 0.086), (0.320, -0.045, 0.0), "machined_aluminum")
    _box(arm, "hinge2_cheek_pos", (0.058, 0.012, 0.086), (0.320, 0.045, 0.0), "machined_aluminum")
    _cyl(arm, "hinge2_boss_neg", 0.030, 0.012, (0.320, -0.057, 0.0), "blued_steel", axis="y")
    _cyl(arm, "hinge2_boss_pos", 0.030, 0.012, (0.320, 0.057, 0.0), "blued_steel", axis="y")

    forearm = model.part("forearm")
    _cyl(forearm, "second_hub", 0.027, 0.062, (0.0, 0.0, 0.0), "ground_rail", axis="y")
    _cyl(forearm, "second_pin", 0.011, 0.088, (0.0, 0.0, 0.0), "fastener", axis="y")
    _box(forearm, "forearm_spine", (0.220, 0.052, 0.035), (0.130, 0.0, 0.0), "blued_steel")
    _box(forearm, "forearm_rib_top", (0.165, 0.064, 0.012), (0.135, 0.0, 0.0235), "machined_aluminum")
    _box(forearm, "forearm_rib_bottom", (0.135, 0.042, 0.010), (0.126, 0.0, -0.0225), "machined_aluminum")
    _box(forearm, "terminal_pad", (0.066, 0.082, 0.060), (0.270, 0.0, 0.0), "machined_aluminum")
    _box(forearm, "terminal_face", (0.006, 0.090, 0.068), (0.306, 0.0, 0.0), "brushed_cover")
    for y in (-0.026, 0.026):
        for z in (-0.020, 0.020):
            _cyl(forearm, f"terminal_fastener_{y}_{z}", 0.0042, 0.004, (0.311, y, z), "fastener", axis="x")
    _box(forearm, "forearm_access_cover", (0.115, 0.044, 0.005), (0.143, 0.0, 0.032), "brushed_cover")
    for x in (0.100, 0.185):
        _cyl(forearm, f"forearm_cover_screw_{x}", 0.0035, 0.004, (x, 0.0, 0.0365), "fastener", axis="z")

    model.articulation(
        "base_to_turret",
        ArticulationType.PRISMATIC,
        parent=base,
        child=turret,
        origin=Origin(xyz=(-0.220, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.35, lower=0.0, upper=0.380),
    )
    model.articulation(
        "turret_to_first_arm",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.386)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.2, lower=-0.55, upper=1.05),
    )
    model.articulation(
        "first_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=forearm,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-1.15, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide = object_model.get_articulation("base_to_turret")
    shoulder = object_model.get_articulation("turret_to_first_arm")
    elbow = object_model.get_articulation("first_arm_to_forearm")
    base = object_model.get_part("linear_base")
    turret = object_model.get_part("turret")
    arm = object_model.get_part("first_arm")
    forearm = object_model.get_part("forearm")

    ctx.check(
        "explicit PRR chain",
        (
            slide.articulation_type == ArticulationType.PRISMATIC
            and shoulder.articulation_type == ArticulationType.REVOLUTE
            and elbow.articulation_type == ArticulationType.REVOLUTE
            and slide.parent == "linear_base"
            and slide.child == "turret"
            and shoulder.parent == "turret"
            and shoulder.child == "first_arm"
            and elbow.parent == "first_arm"
            and elbow.child == "forearm"
        ),
        details="Expected linear_base -> turret -> first_arm -> forearm as a prismatic-revolute-revolute serial chain.",
    )

    ctx.expect_gap(
        turret,
        base,
        axis="z",
        positive_elem="guide_bridge_0",
        negative_elem="linear_rail_0",
        max_gap=0.001,
        max_penetration=0.0002,
        name="lower guide block rides on rail",
    )
    ctx.expect_gap(
        turret,
        base,
        axis="z",
        positive_elem="guide_bridge_1",
        negative_elem="linear_rail_1",
        max_gap=0.001,
        max_penetration=0.0002,
        name="upper guide block rides on rail",
    )
    ctx.expect_overlap(
        turret,
        base,
        axes="x",
        elem_a="carriage_plate",
        elem_b="linear_rail_0",
        min_overlap=0.16,
        name="carriage spans the linear guide rail",
    )
    ctx.expect_gap(
        turret,
        arm,
        axis="y",
        positive_elem="hinge1_cheek_pos",
        negative_elem="first_hub",
        min_gap=0.010,
        max_gap=0.025,
        name="first hinge positive cheek clears hub",
    )
    ctx.expect_gap(
        arm,
        turret,
        axis="y",
        positive_elem="first_hub",
        negative_elem="hinge1_cheek_neg",
        min_gap=0.010,
        max_gap=0.025,
        name="first hinge negative cheek clears hub",
    )
    ctx.expect_gap(
        arm,
        forearm,
        axis="y",
        positive_elem="hinge2_cheek_pos",
        negative_elem="second_hub",
        min_gap=0.006,
        max_gap=0.018,
        name="second hinge positive cheek clears hub",
    )
    ctx.expect_gap(
        forearm,
        arm,
        axis="y",
        positive_elem="second_hub",
        negative_elem="hinge2_cheek_neg",
        min_gap=0.006,
        max_gap=0.018,
        name="second hinge negative cheek clears hub",
    )

    rest_turret = ctx.part_world_position(turret)
    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({slide: 0.30, shoulder: 0.70}):
        extended_turret = ctx.part_world_position(turret)
        raised_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "slide translates turret along base axis",
        (
            rest_turret is not None
            and extended_turret is not None
            and extended_turret[0] > rest_turret[0] + 0.25
            and abs(extended_turret[1] - rest_turret[1]) < 0.002
        ),
        details=f"rest={rest_turret}, extended={extended_turret}",
    )
    ctx.check(
        "first rotary stage raises second hinge",
        (
            rest_forearm is not None
            and raised_forearm is not None
            and raised_forearm[2] > rest_forearm[2] + 0.15
        ),
        details=f"rest={rest_forearm}, raised={raised_forearm}",
    )

    rest_terminal = ctx.part_element_world_aabb(forearm, elem="terminal_pad")
    with ctx.pose({elbow: 0.85}):
        curled_terminal = ctx.part_element_world_aabb(forearm, elem="terminal_pad")
    terminal_z0 = (rest_terminal[0][2] + rest_terminal[1][2]) / 2.0 if rest_terminal else None
    terminal_z1 = (curled_terminal[0][2] + curled_terminal[1][2]) / 2.0 if curled_terminal else None
    ctx.check(
        "second rotary stage curls terminal bracket",
        terminal_z0 is not None and terminal_z1 is not None and terminal_z1 > terminal_z0 + 0.10,
        details=f"rest_terminal={rest_terminal}, curled_terminal={curled_terminal}",
    )

    return ctx.report()


object_model = build_object_model()
