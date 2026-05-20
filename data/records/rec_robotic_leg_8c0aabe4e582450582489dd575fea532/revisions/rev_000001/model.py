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


ANODIZED = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
ALUMINUM = Material("brushed_aluminum", color=(0.68, 0.70, 0.69, 1.0))
BEARING = Material("black_bearing_rubber", color=(0.015, 0.015, 0.014, 1.0))
ACCENT = Material("safety_orange_joint_marks", color=(1.0, 0.38, 0.05, 1.0))
RUBBER = Material("matte_black_foot_rubber", color=(0.02, 0.022, 0.024, 1.0))


def _box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder_y(part, name, radius, length, xyz, material) -> None:
    # SDK cylinders run along local +Z; rotate the cylinder onto the lateral Y axis.
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _xz_strut(part, name, start, end, thickness, width_y, material) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-9:
        raise ValueError("This helper is for struts in an X-Z side plane.")
    theta = math.atan2(dx, dz)
    _box(
        part,
        name,
        (thickness, width_y, length),
        ((sx + ex) / 2.0, sy, (sz + ez) / 2.0),
        material,
        rpy=(0.0, theta, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_robotic_leg_module")

    upper_housing = model.part("upper_housing")
    _box(upper_housing, "electronics_case", (0.30, 0.23, 0.15), (0.0, 0.0, 0.145), ANODIZED)
    _box(upper_housing, "top_mount", (0.24, 0.18, 0.035), (0.0, 0.0, 0.237), ALUMINUM)
    _box(upper_housing, "hip_fork_plate_0", (0.17, 0.035, 0.24), (0.0, -0.145, 0.02), ALUMINUM)
    _box(upper_housing, "hip_fork_plate_1", (0.17, 0.035, 0.24), (0.0, 0.145, 0.02), ALUMINUM)
    _box(upper_housing, "fork_back_tie", (0.055, 0.30, 0.055), (-0.082, 0.0, 0.055), ALUMINUM)
    _cylinder_y(upper_housing, "hip_bearing_cap_0", 0.073, 0.034, (0.0, -0.174, 0.0), BEARING)
    _cylinder_y(upper_housing, "hip_bearing_cap_1", 0.073, 0.034, (0.0, 0.174, 0.0), BEARING)
    _box(upper_housing, "front_service_panel", (0.012, 0.17, 0.095), (0.156, 0.0, 0.150), ACCENT)

    thigh_link = model.part("thigh_link")
    _cylinder_y(thigh_link, "hip_hub", 0.055, 0.18, (0.0, 0.0, 0.0), BEARING)
    _cylinder_y(thigh_link, "hip_inner_axle", 0.026, 0.255, (0.0, 0.0, 0.0), ALUMINUM)
    _box(thigh_link, "hip_bridge", (0.13, 0.16, 0.060), (0.0, 0.0, -0.080), ALUMINUM)
    _box(thigh_link, "rail_0", (0.030, 0.026, 0.435), (-0.035, -0.065, -0.315), ALUMINUM)
    _box(thigh_link, "rail_1", (0.030, 0.026, 0.435), (0.035, 0.065, -0.315), ALUMINUM)
    _xz_strut(thigh_link, "diagonal_web_0", (-0.045, 0.0, -0.135), (0.045, 0.0, -0.500), 0.018, 0.050, ALUMINUM)
    _xz_strut(thigh_link, "diagonal_web_1", (0.045, 0.0, -0.135), (-0.045, 0.0, -0.500), 0.018, 0.050, ALUMINUM)
    _box(thigh_link, "knee_bridge", (0.14, 0.192, 0.050), (0.0, 0.0, -0.525), ALUMINUM)
    _box(thigh_link, "knee_yoke_plate_0", (0.145, 0.026, 0.135), (0.0, -0.096, -0.615), ALUMINUM)
    _box(thigh_link, "knee_yoke_plate_1", (0.145, 0.026, 0.135), (0.0, 0.096, -0.615), ALUMINUM)
    _cylinder_y(thigh_link, "knee_outer_bearing_0", 0.050, 0.026, (0.0, -0.116, -0.620), BEARING)
    _cylinder_y(thigh_link, "knee_outer_bearing_1", 0.050, 0.026, (0.0, 0.116, -0.620), BEARING)

    shank_link = model.part("shank_link")
    _cylinder_y(shank_link, "knee_hub", 0.045, 0.130, (0.0, 0.0, 0.0), BEARING)
    _cylinder_y(shank_link, "knee_axle", 0.021, 0.166, (0.0, 0.0, 0.0), ALUMINUM)
    _box(shank_link, "knee_socket", (0.110, 0.125, 0.055), (0.0, 0.0, -0.063), ALUMINUM)
    _box(shank_link, "rail_0", (0.025, 0.023, 0.340), (-0.030, -0.052, -0.260), ALUMINUM)
    _box(shank_link, "rail_1", (0.025, 0.023, 0.340), (0.030, 0.052, -0.260), ALUMINUM)
    _xz_strut(shank_link, "diagonal_web_0", (-0.040, 0.0, -0.089), (0.040, 0.0, -0.425), 0.016, 0.045, ALUMINUM)
    _xz_strut(shank_link, "diagonal_web_1", (0.040, 0.0, -0.089), (-0.040, 0.0, -0.425), 0.016, 0.045, ALUMINUM)
    _box(shank_link, "ankle_bridge", (0.120, 0.130, 0.040), (0.0, 0.0, -0.445), ALUMINUM)
    _box(shank_link, "ankle_yoke_plate_0", (0.115, 0.023, 0.120), (0.0, -0.075, -0.520), ALUMINUM)
    _box(shank_link, "ankle_yoke_plate_1", (0.115, 0.023, 0.120), (0.0, 0.075, -0.520), ALUMINUM)
    _cylinder_y(shank_link, "ankle_outer_bearing_0", 0.042, 0.022, (0.0, -0.094, -0.520), BEARING)
    _cylinder_y(shank_link, "ankle_outer_bearing_1", 0.042, 0.022, (0.0, 0.094, -0.520), BEARING)

    ankle_foot = model.part("ankle_foot")
    _cylinder_y(ankle_foot, "ankle_hub", 0.038, 0.108, (0.0, 0.0, 0.0), BEARING)
    _cylinder_y(ankle_foot, "ankle_axle", 0.018, 0.127, (0.0, 0.0, 0.0), ALUMINUM)
    _box(ankle_foot, "ankle_block", (0.080, 0.104, 0.102), (0.0, 0.0, -0.075), ALUMINUM)
    _box(ankle_foot, "sole_plate", (0.335, 0.160, 0.045), (0.070, 0.0, -0.142), ANODIZED)
    _cylinder_y(ankle_foot, "toe_roller", 0.026, 0.160, (0.245, 0.0, -0.142), RUBBER)
    _cylinder_y(ankle_foot, "heel_roller", 0.024, 0.150, (-0.112, 0.0, -0.142), RUBBER)
    _box(ankle_foot, "sole_pad_0", (0.105, 0.052, 0.016), (0.150, -0.047, -0.169), RUBBER)
    _box(ankle_foot, "sole_pad_1", (0.105, 0.052, 0.016), (0.150, 0.047, -0.169), RUBBER)
    _box(ankle_foot, "sole_pad_2", (0.090, 0.050, 0.016), (-0.040, -0.045, -0.169), RUBBER)
    _box(ankle_foot, "sole_pad_3", (0.090, 0.050, 0.016), (-0.040, 0.045, -0.169), RUBBER)

    model.articulation(
        "hip",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=4.5, lower=-0.70, upper=0.90),
    )
    model.articulation(
        "knee",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=5.0, lower=0.0, upper=2.10),
    )
    model.articulation(
        "ankle",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=ankle_foot,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=5.0, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_housing")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("ankle_foot")
    hip = object_model.get_articulation("hip")
    knee = object_model.get_articulation("knee")
    ankle = object_model.get_articulation("ankle")

    for joint in (hip, knee, ankle):
        ctx.check(
            f"{joint.name} uses a lateral hinge axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_within(
        thigh,
        upper,
        axes="y",
        inner_elem="hip_hub",
        name="hip hub sits between upper fork plates",
    )
    ctx.expect_within(
        shank,
        thigh,
        axes="y",
        inner_elem="knee_hub",
        name="knee hub sits inside thigh yoke",
    )
    ctx.expect_within(
        foot,
        shank,
        axes="y",
        inner_elem="ankle_hub",
        name="ankle hub sits inside shank yoke",
    )

    rest_knee = ctx.part_world_position(shank)
    with ctx.pose({hip: 0.45}):
        swung_knee = ctx.part_world_position(shank)
    ctx.check(
        "positive hip swing moves the leg forward",
        rest_knee is not None and swung_knee is not None and swung_knee[0] < rest_knee[0] - 0.15,
        details=f"rest={rest_knee}, swung={swung_knee}",
    )

    rest_ankle = ctx.part_world_position(foot)
    with ctx.pose({knee: 0.80}):
        flexed_ankle = ctx.part_world_position(foot)
    ctx.check(
        "positive knee flex folds the shank",
        rest_ankle is not None and flexed_ankle is not None and flexed_ankle[0] < rest_ankle[0] - 0.20,
        details=f"rest={rest_ankle}, flexed={flexed_ankle}",
    )

    rest_foot_aabb = ctx.part_element_world_aabb(foot, elem="toe_roller")
    with ctx.pose({ankle: -0.35}):
        dorsiflexed_aabb = ctx.part_element_world_aabb(foot, elem="toe_roller")
    if rest_foot_aabb is not None and dorsiflexed_aabb is not None:
        rest_toe_z = rest_foot_aabb[1][2]
        flexed_toe_z = dorsiflexed_aabb[1][2]
        ankle_moves = flexed_toe_z > rest_toe_z + 0.015
    else:
        ankle_moves = False
    ctx.check(
        "ankle rotation lifts the compact foot section",
        ankle_moves,
        details=f"rest_aabb={rest_foot_aabb}, dorsiflexed_aabb={dorsiflexed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
