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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_telescope_altaz_tripod")

    wood = model.material("warm_oiled_wood", rgba=(0.55, 0.31, 0.13, 1.0))
    dark_wood = model.material("dark_end_grain", rgba=(0.28, 0.16, 0.08, 1.0))
    white_enamel = model.material("white_enamel", rgba=(0.92, 0.92, 0.86, 1.0))
    satin_black = model.material("satin_black", rgba=(0.01, 0.01, 0.012, 1.0))
    brushed_metal = model.material("brushed_aluminum", rgba=(0.62, 0.62, 0.58, 1.0))
    glass_blue = model.material("blue_coated_glass", rgba=(0.25, 0.48, 0.78, 0.62))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.23, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 1.0125)),
        material=wood,
        name="tripod_top",
    )
    tripod.visual(
        Cylinder(radius=0.145, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.93)),
        material=dark_wood,
        name="central_hub",
    )
    tripod.visual(
        Cylinder(radius=0.24, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=wood,
        name="accessory_tray",
    )

    top_radius = 0.15
    foot_radius = 0.66
    top_z = 0.98
    foot_z = 0.035
    leg_len = math.sqrt((foot_radius - top_radius) ** 2 + (foot_z - top_z) ** 2)
    leg_pitch = math.acos((foot_z - top_z) / leg_len)
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        ct = math.cos(theta)
        st = math.sin(theta)
        top = (top_radius * ct, top_radius * st, top_z)
        foot = (foot_radius * ct, foot_radius * st, foot_z)
        mid = tuple((a + b) * 0.5 for a, b in zip(top, foot))
        tripod.visual(
            Box((0.065, 0.085, leg_len)),
            origin=Origin(xyz=mid, rpy=(0.0, leg_pitch, theta)),
            material=wood,
            name=f"leg_{i}",
        )
        tripod.visual(
            Box((0.16, 0.11, 0.035)),
            origin=Origin(xyz=(foot[0], foot[1], 0.0175), rpy=(0.0, 0.0, theta)),
            material=satin_black,
            name=f"rubber_foot_{i}",
        )

        brace_inner = (0.10 * ct, 0.10 * st, 0.46)
        brace_outer = (0.44 * ct, 0.44 * st, 0.36)
        brace_mid = tuple((a + b) * 0.5 for a, b in zip(brace_inner, brace_outer))
        brace_len = math.sqrt((brace_outer[0] - brace_inner[0]) ** 2 + (brace_outer[1] - brace_inner[1]) ** 2 + (brace_outer[2] - brace_inner[2]) ** 2)
        brace_pitch = math.acos((brace_outer[2] - brace_inner[2]) / brace_len)
        tripod.visual(
            Box((0.032, 0.038, brace_len)),
            origin=Origin(xyz=brace_mid, rpy=(0.0, brace_pitch, theta)),
            material=dark_wood,
            name=f"tray_brace_{i}",
        )

    azimuth_mount = model.part("azimuth_mount")
    azimuth_mount.visual(
        Cylinder(radius=0.215, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=brushed_metal,
        name="azimuth_turntable",
    )
    azimuth_mount.visual(
        Cylinder(radius=0.085, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=satin_black,
        name="center_post",
    )
    azimuth_mount.visual(
        Box((0.16, 0.32, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=brushed_metal,
        name="fork_bridge",
    )
    azimuth_mount.visual(
        Box((0.11, 0.05, 0.43)),
        origin=Origin(xyz=(0.0, -0.155, 0.255)),
        material=brushed_metal,
        name="fork_arm_0",
    )
    azimuth_mount.visual(
        Cylinder(radius=0.075, length=0.012),
        origin=Origin(xyz=(0.0, -0.154225, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="altitude_bushing_0",
    )
    azimuth_mount.visual(
        Box((0.11, 0.05, 0.43)),
        origin=Origin(xyz=(0.0, 0.155, 0.255)),
        material=brushed_metal,
        name="fork_arm_1",
    )
    azimuth_mount.visual(
        Cylinder(radius=0.075, length=0.012),
        origin=Origin(xyz=(0.0, 0.154225, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="altitude_bushing_1",
    )
    azimuth_mount.visual(
        Cylinder(radius=0.03, length=0.18),
        origin=Origin(xyz=(0.0, -0.24, 0.42), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="altitude_lock_knob",
    )

    telescope = model.part("telescope")
    telescope.visual(
        Cylinder(radius=0.074, length=0.86),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_enamel,
        name="main_tube",
    )
    telescope.visual(
        Cylinder(radius=0.091, length=0.22),
        origin=Origin(xyz=(0.64, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_enamel,
        name="dew_shield",
    )
    telescope.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.756, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="objective_lens",
    )
    telescope.visual(
        Cylinder(radius=0.079, length=0.035),
        origin=Origin(xyz=(-0.345, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_cell",
    )
    telescope.visual(
        Cylinder(radius=0.041, length=0.22),
        origin=Origin(xyz=(-0.47, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="focuser_tube",
    )
    telescope.visual(
        Box((0.09, 0.085, 0.075)),
        origin=Origin(xyz=(-0.60, 0.0, 0.055), rpy=(0.0, 0.25, 0.0)),
        material=satin_black,
        name="star_diagonal",
    )
    telescope.visual(
        Cylinder(radius=0.024, length=0.18),
        origin=Origin(xyz=(-0.645, 0.0, 0.135), rpy=(0.0, -0.45, 0.0)),
        material=satin_black,
        name="eyepiece",
    )
    telescope.visual(
        Cylinder(radius=0.059, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="altitude_axle",
    )
    telescope.visual(
        Cylinder(radius=0.025, length=0.48),
        origin=Origin(xyz=(0.12, 0.0, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_enamel,
        name="finder_scope",
    )
    for i, x in enumerate((-0.07, 0.30)):
        telescope.visual(
            Box((0.035, 0.025, 0.105)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=satin_black,
            name=f"finder_bracket_{i}",
        )
    telescope.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.53, -0.045, 0.0)),
        material=satin_black,
        name="focus_knob_0",
    )
    telescope.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.53, 0.045, 0.0)),
        material=satin_black,
        name="focus_knob_1",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=azimuth_mount,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_mount,
        child=telescope,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=0.9, lower=-0.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    azimuth_mount = object_model.get_part("azimuth_mount")
    telescope = object_model.get_part("telescope")
    azimuth_axis = object_model.get_articulation("azimuth_axis")
    altitude_axis = object_model.get_articulation("altitude_axis")

    ctx.check(
        "azimuth and altitude are revolute pointing axes",
        azimuth_axis.articulation_type == ArticulationType.REVOLUTE
        and altitude_axis.articulation_type == ArticulationType.REVOLUTE
        and azimuth_axis.axis == (0.0, 0.0, 1.0)
        and altitude_axis.axis == (0.0, -1.0, 0.0),
        details=f"azimuth={azimuth_axis.articulation_type} {azimuth_axis.axis}, altitude={altitude_axis.articulation_type} {altitude_axis.axis}",
    )
    ctx.expect_gap(
        azimuth_mount,
        tripod,
        axis="z",
        positive_elem="azimuth_turntable",
        negative_elem="tripod_top",
        max_gap=0.001,
        max_penetration=0.0,
        name="azimuth bearing sits on the wooden tripod top",
    )
    ctx.expect_overlap(
        azimuth_mount,
        tripod,
        axes="xy",
        elem_a="azimuth_turntable",
        elem_b="tripod_top",
        min_overlap=0.18,
        name="azimuth turntable footprint is carried by tripod top",
    )
    ctx.expect_gap(
        telescope,
        azimuth_mount,
        axis="y",
        positive_elem="altitude_axle",
        negative_elem="fork_arm_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="altitude axle bears against one fork cheek",
    )
    ctx.expect_gap(
        azimuth_mount,
        telescope,
        axis="y",
        positive_elem="fork_arm_1",
        negative_elem="altitude_axle",
        max_gap=0.001,
        max_penetration=0.0,
        name="altitude axle bears against opposite fork cheek",
    )

    def element_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({azimuth_axis: 0.0, altitude_axis: 0.0}):
        rest_objective = element_center(telescope, "objective_lens")
    with ctx.pose({azimuth_axis: 0.0, altitude_axis: 0.75}):
        raised_objective = element_center(telescope, "objective_lens")
    with ctx.pose({azimuth_axis: math.pi / 2.0, altitude_axis: 0.0}):
        slewed_objective = element_center(telescope, "objective_lens")

    ctx.check(
        "positive altitude raises the objective end",
        rest_objective is not None
        and raised_objective is not None
        and raised_objective[2] > rest_objective[2] + 0.25,
        details=f"rest={rest_objective}, raised={raised_objective}",
    )
    ctx.check(
        "positive azimuth slews the tube around the vertical axis",
        rest_objective is not None
        and slewed_objective is not None
        and slewed_objective[1] > rest_objective[1] + 0.55
        and abs(slewed_objective[0]) < 0.10,
        details=f"rest={rest_objective}, slewed={slewed_objective}",
    )

    return ctx.report()


object_model = build_object_model()
