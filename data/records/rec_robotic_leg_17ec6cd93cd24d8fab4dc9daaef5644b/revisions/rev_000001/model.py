from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_robotic_leg")

    dark_steel = model.material("dark_blasted_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    worn_steel = model.material("worn_pin_steel", rgba=(0.56, 0.57, 0.55, 1.0))
    safety_yellow = model.material("safety_yellow_guard", rgba=(1.0, 0.72, 0.04, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.88, 0.03, 0.02, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def bolt_z(part, name, x, y, top_z, radius=0.014):
        cyl(part, name, radius, 0.014, (x, y, top_z + 0.005), worn_steel)

    def bolt_y(part, name, x, y, z, radius=0.013):
        cyl(part, name, radius, 0.014, (x, y, z), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))

    hip_mount = model.part("hip_mount")
    box(hip_mount, "back_mount_plate", (0.12, 0.54, 0.38), (-0.26, 0.0, 0.09), dark_steel)
    box(hip_mount, "top_anchor_flange", (0.24, 0.62, 0.055), (-0.20, 0.0, 0.305), dark_steel)
    box(hip_mount, "mount_spine", (0.12, 0.12, 0.26), (-0.225, 0.0, 0.075), dark_steel)
    box(hip_mount, "upper_bridge", (0.22, 0.46, 0.055), (-0.07, 0.0, 0.165), dark_steel)
    box(hip_mount, "lower_bridge", (0.12, 0.46, 0.045), (-0.12, 0.0, -0.125), dark_steel)
    box(hip_mount, "hip_fork_0", (0.18, 0.055, 0.28), (-0.02, -0.17, 0.0), dark_steel)
    box(hip_mount, "hip_fork_1", (0.18, 0.055, 0.28), (-0.02, 0.17, 0.0), dark_steel)
    cyl(hip_mount, "hip_pin", 0.045, 0.44, (0.0, 0.0, 0.0), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(hip_mount, "hip_guard_0", (0.035, 0.035, 0.22), (0.07, -0.215, 0.0), safety_yellow)
    box(hip_mount, "hip_guard_1", (0.035, 0.035, 0.22), (0.07, 0.215, 0.0), safety_yellow)
    box(hip_mount, "hip_stop_0", (0.055, 0.055, 0.04), (0.085, -0.17, 0.135), lockout_red)
    box(hip_mount, "hip_stop_1", (0.055, 0.055, 0.04), (0.085, 0.17, 0.135), lockout_red)
    box(hip_mount, "hip_lockout_tab", (0.045, 0.026, 0.14), (-0.08, 0.207, -0.035), lockout_red)
    for i, x in enumerate((-0.275, -0.125)):
        for j, y in enumerate((-0.22, 0.22)):
            bolt_z(hip_mount, f"anchor_bolt_{i}_{j}", x, y, 0.333)
    for side, y in (("0", -0.203), ("1", 0.203)):
        bolt_y(hip_mount, f"hip_fork_bolt_{side}_0", 0.035, y, 0.075)
        bolt_y(hip_mount, f"hip_fork_bolt_{side}_1", -0.075, y, -0.075)

    thigh_link = model.part("thigh_link")
    cyl(thigh_link, "hip_bearing", 0.100, 0.22, (0.0, 0.0, 0.0), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(thigh_link, "thigh_rail_0", (0.075, 0.042, 0.40), (0.010, -0.092, -0.300), dark_steel)
    box(thigh_link, "thigh_rail_1", (0.075, 0.042, 0.40), (0.010, 0.092, -0.300), dark_steel)
    box(thigh_link, "thigh_upper_tie", (0.11, 0.25, 0.045), (0.010, 0.0, -0.125), dark_steel)
    box(thigh_link, "thigh_lower_tie", (0.12, 0.36, 0.045), (-0.035, 0.0, -0.492), dark_steel)
    box(thigh_link, "thigh_bay_mount", (0.105, 0.220, 0.045), (0.070, 0.0, -0.160), dark_steel)
    box(thigh_link, "thigh_bay", (0.060, 0.150, 0.310), (0.095, 0.0, -0.330), dark_steel)
    box(thigh_link, "thigh_bay_guard", (0.020, 0.172, 0.270), (0.135, 0.0, -0.330), safety_yellow)
    box(thigh_link, "thigh_bay_flange_0", (0.030, 0.052, 0.260), (0.138, -0.105, -0.330), dark_steel)
    box(thigh_link, "thigh_bay_flange_1", (0.030, 0.052, 0.260), (0.138, 0.105, -0.330), dark_steel)
    box(thigh_link, "thigh_brace_0", (0.034, 0.018, 0.410), (0.030, -0.126, -0.330), worn_steel, rpy=(0.0, 0.28, 0.0))
    box(thigh_link, "thigh_brace_1", (0.034, 0.018, 0.410), (0.030, 0.126, -0.330), worn_steel, rpy=(0.0, -0.28, 0.0))
    box(thigh_link, "knee_fork_0", (0.19, 0.055, 0.225), (0.005, -0.165, -0.620), dark_steel)
    box(thigh_link, "knee_fork_1", (0.19, 0.055, 0.225), (0.005, 0.165, -0.620), dark_steel)
    cyl(thigh_link, "knee_pin", 0.040, 0.405, (0.0, 0.0, -0.620), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(thigh_link, "knee_stop_0", (0.050, 0.052, 0.035), (0.090, -0.165, -0.500), lockout_red)
    box(thigh_link, "knee_stop_1", (0.050, 0.052, 0.035), (0.090, 0.165, -0.500), lockout_red)
    box(thigh_link, "knee_guard_0", (0.035, 0.032, 0.180), (0.092, -0.207, -0.620), safety_yellow)
    box(thigh_link, "knee_guard_1", (0.035, 0.032, 0.180), (0.092, 0.207, -0.620), safety_yellow)
    box(thigh_link, "knee_lockout_tab", (0.045, 0.026, 0.115), (-0.085, 0.202, -0.620), lockout_red)
    for side, y in (("0", -0.126), ("1", 0.126)):
        bolt_y(thigh_link, f"thigh_bay_bolt_{side}_0", 0.138, y, -0.230, radius=0.010)
        bolt_y(thigh_link, f"thigh_bay_bolt_{side}_1", 0.138, y, -0.430, radius=0.010)
    for side, y in (("0", -0.199), ("1", 0.199)):
        bolt_y(thigh_link, f"knee_fork_bolt_{side}_0", 0.055, y, -0.555)
        bolt_y(thigh_link, f"knee_fork_bolt_{side}_1", -0.060, y, -0.685)

    shin_link = model.part("shin_link")
    cyl(shin_link, "knee_bearing", 0.095, 0.215, (0.0, 0.0, 0.0), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(shin_link, "shin_rail_0", (0.070, 0.040, 0.380), (-0.006, -0.086, -0.270), dark_steel)
    box(shin_link, "shin_rail_1", (0.070, 0.040, 0.380), (-0.006, 0.086, -0.270), dark_steel)
    box(shin_link, "shin_upper_tie", (0.110, 0.235, 0.042), (0.0, 0.0, -0.110), dark_steel)
    box(shin_link, "shin_lower_tie", (0.115, 0.330, 0.060), (-0.030, 0.0, -0.440), dark_steel)
    box(shin_link, "shin_bay_mount", (0.098, 0.205, 0.042), (0.062, 0.0, -0.155), dark_steel)
    box(shin_link, "shin_bay", (0.055, 0.135, 0.270), (0.087, 0.0, -0.290), dark_steel)
    box(shin_link, "shin_bay_guard", (0.020, 0.156, 0.240), (0.122, 0.0, -0.290), safety_yellow)
    box(shin_link, "shin_bay_flange_0", (0.028, 0.050, 0.230), (0.126, -0.096, -0.290), dark_steel)
    box(shin_link, "shin_bay_flange_1", (0.028, 0.050, 0.230), (0.126, 0.096, -0.290), dark_steel)
    box(shin_link, "shin_brace_0", (0.030, 0.016, 0.360), (0.025, -0.116, -0.300), worn_steel, rpy=(0.0, 0.25, 0.0))
    box(shin_link, "shin_brace_1", (0.030, 0.016, 0.360), (0.025, 0.116, -0.300), worn_steel, rpy=(0.0, -0.25, 0.0))
    box(shin_link, "ankle_fork_0", (0.165, 0.050, 0.190), (0.000, -0.150, -0.560), dark_steel)
    box(shin_link, "ankle_fork_1", (0.165, 0.050, 0.190), (0.000, 0.150, -0.560), dark_steel)
    cyl(shin_link, "ankle_pin", 0.036, 0.360, (0.0, 0.0, -0.560), worn_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(shin_link, "ankle_stop_0", (0.045, 0.048, 0.032), (0.078, -0.150, -0.455), lockout_red)
    box(shin_link, "ankle_stop_1", (0.045, 0.048, 0.032), (0.078, 0.150, -0.455), lockout_red)
    box(shin_link, "ankle_guard_0", (0.030, 0.030, 0.145), (0.080, -0.187, -0.560), safety_yellow)
    box(shin_link, "ankle_guard_1", (0.030, 0.030, 0.145), (0.080, 0.187, -0.560), safety_yellow)
    box(shin_link, "ankle_lockout_tab", (0.040, 0.024, 0.100), (-0.073, 0.184, -0.560), lockout_red)
    for side, y in (("0", -0.112), ("1", 0.112)):
        bolt_y(shin_link, f"shin_bay_bolt_{side}_0", 0.126, y, -0.210, radius=0.009)
        bolt_y(shin_link, f"shin_bay_bolt_{side}_1", 0.126, y, -0.370, radius=0.009)
    for side, y in (("0", -0.180), ("1", 0.180)):
        bolt_y(shin_link, f"ankle_fork_bolt_{side}_0", 0.045, y, -0.515, radius=0.011)
        bolt_y(shin_link, f"ankle_fork_bolt_{side}_1", -0.055, y, -0.610, radius=0.011)

    foot = model.part("foot")
    cyl(foot, "ankle_bearing", 0.080, 0.195, (0.0, 0.0, 0.0), dark_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(foot, "ankle_saddle", (0.120, 0.165, 0.165), (0.010, 0.0, -0.130), dark_steel)
    box(foot, "sole_plate", (0.500, 0.290, 0.065), (0.105, 0.0, -0.245), dark_steel)
    box(foot, "toe_guard", (0.140, 0.292, 0.040), (0.335, 0.0, -0.185), safety_yellow)
    box(foot, "heel_guard", (0.105, 0.292, 0.036), (-0.145, 0.0, -0.190), safety_yellow)
    box(foot, "sole_rubber", (0.520, 0.305, 0.030), (0.105, 0.0, -0.292), rubber)
    for i, x in enumerate((-0.085, 0.095, 0.275)):
        box(foot, f"tread_rib_{i}", (0.035, 0.308, 0.014), (x, 0.0, -0.309), rubber)
    box(foot, "ankle_lock_socket", (0.045, 0.030, 0.085), (-0.070, 0.093, -0.030), lockout_red)
    for i, x in enumerate((-0.095, 0.105, 0.300)):
        for j, y in enumerate((-0.095, 0.095)):
            bolt_z(foot, f"sole_bolt_{i}_{j}", x, y, -0.212, radius=0.010)

    hip = model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2800.0, velocity=1.6, lower=-0.65, upper=0.95),
    )
    knee = model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shin_link,
        origin=Origin(xyz=(0.0, 0.0, -0.620)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=1.8, lower=0.0, upper=1.85),
    )
    ankle = model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin_link,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.560)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=2.1, lower=-0.55, upper=0.65),
    )

    # Keep variables alive for readability; articulations are already registered on the model.
    _ = (hip, knee, ankle)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_mount = object_model.get_part("hip_mount")
    thigh_link = object_model.get_part("thigh_link")
    shin_link = object_model.get_part("shin_link")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.allow_overlap(
        hip_mount,
        thigh_link,
        elem_a="hip_pin",
        elem_b="hip_bearing",
        reason="The heavy hip pin is intentionally captured through the thigh bearing proxy.",
    )
    ctx.expect_within(
        hip_mount,
        thigh_link,
        axes="xz",
        inner_elem="hip_pin",
        outer_elem="hip_bearing",
        margin=0.002,
        name="hip pin centered inside bearing radius",
    )
    ctx.expect_overlap(
        hip_mount,
        thigh_link,
        axes="y",
        elem_a="hip_pin",
        elem_b="hip_bearing",
        min_overlap=0.18,
        name="hip pin spans the captured bearing",
    )

    ctx.allow_overlap(
        thigh_link,
        shin_link,
        elem_a="knee_pin",
        elem_b="knee_bearing",
        reason="The knee pin is intentionally modeled as a captured shaft through the shin bearing.",
    )
    ctx.expect_within(
        thigh_link,
        shin_link,
        axes="xz",
        inner_elem="knee_pin",
        outer_elem="knee_bearing",
        margin=0.002,
        name="knee pin centered inside bearing radius",
    )
    ctx.expect_overlap(
        thigh_link,
        shin_link,
        axes="y",
        elem_a="knee_pin",
        elem_b="knee_bearing",
        min_overlap=0.17,
        name="knee pin spans the captured bearing",
    )

    ctx.allow_overlap(
        shin_link,
        foot,
        elem_a="ankle_pin",
        elem_b="ankle_bearing",
        reason="The ankle pin is intentionally captured through the foot bearing proxy.",
    )
    ctx.expect_within(
        shin_link,
        foot,
        axes="xz",
        inner_elem="ankle_pin",
        outer_elem="ankle_bearing",
        margin=0.002,
        name="ankle pin centered inside bearing radius",
    )
    ctx.expect_overlap(
        shin_link,
        foot,
        axes="y",
        elem_a="ankle_pin",
        elem_b="ankle_bearing",
        min_overlap=0.15,
        name="ankle pin spans the captured bearing",
    )

    ctx.check(
        "serial pitch joints have bounded heavy-duty ranges",
        hip.motion_limits is not None
        and knee.motion_limits is not None
        and ankle.motion_limits is not None
        and hip.motion_limits.lower < 0.0 < hip.motion_limits.upper
        and 1.4 < knee.motion_limits.upper < 2.1
        and ankle.motion_limits.lower < 0.0 < ankle.motion_limits.upper,
        details=f"hip={hip.motion_limits}, knee={knee.motion_limits}, ankle={ankle.motion_limits}",
    )
    ctx.expect_origin_gap(hip_mount, foot, axis="z", min_gap=1.0, name="foot sits well below hip support at rest")

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({knee: 1.2}):
        flexed_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee pitch advances the downstream ankle-foot chain",
        rest_foot is not None and flexed_foot is not None and flexed_foot[0] > rest_foot[0] + 0.28,
        details=f"rest={rest_foot}, flexed={flexed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
