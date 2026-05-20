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


AXIS_THICKNESS = 0.044
FORK_EAR_THICKNESS = 0.026
FORK_EAR_OFFSET = 0.052
FORK_GAP = FORK_EAR_OFFSET - 0.5 * FORK_EAR_THICKNESS - 0.5 * AXIS_THICKNESS


def _bar_origin(dx: float, dz: float, start: float, end: float) -> tuple[Origin, float]:
    length = math.hypot(dx, dz)
    ux = dx / length
    uz = dz / length
    mid = 0.5 * (start + end)
    angle = math.atan2(dz, dx)
    return Origin(xyz=(ux * mid, 0.0, uz * mid), rpy=(0.0, -angle, 0.0)), end - start


def _offset_bar(
    part,
    *,
    dx: float,
    dz: float,
    start: float,
    end: float,
    y: float,
    y_size: float,
    plane_width: float,
    material,
    name: str,
) -> None:
    origin, length = _bar_origin(dx, dz, start, end)
    part.visual(
        Box((length, y_size, plane_width)),
        origin=Origin(xyz=(origin.xyz[0], y, origin.xyz[2]), rpy=origin.rpy),
        material=material,
        name=name,
    )


def _joint_cylinder(
    part,
    *,
    center: tuple[float, float, float],
    radius: float,
    length: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_link(
    part,
    *,
    dx: float,
    dz: float,
    spine_width: float,
    fork_width: float,
    boss_radius: float,
    dist_radius: float,
    body_material,
    dark_material,
    accent_material,
    accent_side: float,
) -> None:
    length = math.hypot(dx, dz)
    ux = dx / length
    uz = dz / length
    distal = (dx, 0.0, dz)

    _joint_cylinder(
        part,
        center=(0.0, 0.0, 0.0),
        radius=boss_radius,
        length=AXIS_THICKNESS,
        material=dark_material,
        name="prox_boss",
    )

    # The central tongue stops short of the distal fork, leaving a real
    # y-clearanced bearing pocket for the next link rather than a repeated bar.
    _offset_bar(
        part,
        dx=dx,
        dz=dz,
        start=0.030,
        end=length - 0.115,
        y=0.0,
        y_size=AXIS_THICKNESS,
        plane_width=spine_width,
        material=body_material,
        name="spine",
    )
    step_start = min(0.120, length * 0.33)
    step_end = max(step_start + 0.030, length - 0.070)
    _offset_bar(
        part,
        dx=dx,
        dz=dz,
        start=step_start,
        end=step_end,
        y=accent_side * 0.026,
        y_size=0.010,
        plane_width=spine_width * 0.62,
        material=accent_material,
        name="raised_step",
    )

    # Split distal fork: two outer ears and two side rails, with an open middle
    # where the next link's tongue can rotate without occupying the same volume.
    fork_start = max(length - 0.180, min(length - 0.080, 0.135))
    fork_end = length - 0.020
    bridge_start = max(0.060, fork_start - 0.035)
    bridge_end = min(fork_end, fork_start + 0.025)
    for suffix, y in (("neg", -FORK_EAR_OFFSET), ("pos", FORK_EAR_OFFSET)):
        _offset_bar(
            part,
            dx=dx,
            dz=dz,
            start=fork_start,
            end=fork_end,
            y=y,
            y_size=FORK_EAR_THICKNESS,
            plane_width=fork_width * 0.78,
            material=body_material,
            name=f"fork_arm_{suffix}",
        )
        _joint_cylinder(
            part,
            center=(distal[0], y, distal[2]),
            radius=dist_radius,
            length=FORK_EAR_THICKNESS,
            material=dark_material,
            name=f"dist_boss_{suffix}",
        )
    _joint_cylinder(
        part,
        center=(distal[0], 0.0, distal[2]),
        radius=0.018,
        length=0.128,
        material=dark_material,
        name="dist_pin",
    )

    # A small crosshead just before the fork is part of the same machined link
    # and makes the visible change in width deliberate rather than accidental.
    cross_station = 0.5 * (bridge_start + bridge_end)
    cross_center = (cross_station * ux, 0.0, cross_station * uz)
    _offset_bar(
        part,
        dx=dx,
        dz=dz,
        start=bridge_start,
        end=bridge_end,
        y=0.0,
        y_size=0.130,
        plane_width=fork_width * 0.52,
        material=body_material,
        name="fork_bridge",
    )
    _joint_cylinder(
        part,
        center=(cross_center[0], accent_side * 0.064, cross_center[2]),
        radius=0.012,
        length=0.012,
        material=accent_material,
        name="bridge_plug",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rocker_four_joint_chain")

    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    gunmetal = model.material("gunmetal_bushings", rgba=(0.10, 0.11, 0.12, 1.0))
    blue = model.material("blue_anodized_link", rgba=(0.05, 0.22, 0.75, 1.0))
    orange = model.material("orange_anodized_link", rgba=(0.92, 0.39, 0.08, 1.0))
    green = model.material("green_anodized_link", rgba=(0.05, 0.48, 0.23, 1.0))
    violet = model.material("violet_anodized_link", rgba=(0.38, 0.20, 0.68, 1.0))
    dark = model.material("black_oxide_step_faces", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.25, 0.030)),
        origin=Origin(xyz=(0.04, 0.0, 0.015)),
        material=steel,
        name="foot",
    )
    base.visual(
        Box((0.15, 0.13, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=steel,
        name="pedestal",
    )
    for suffix, y in (("neg", -FORK_EAR_OFFSET), ("pos", FORK_EAR_OFFSET)):
        base.visual(
            Box((0.12, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.1775)),
            material=steel,
            name=f"shoulder_{suffix}",
        )
    for suffix, y in (("neg", -FORK_EAR_OFFSET), ("pos", FORK_EAR_OFFSET)):
        base.visual(
            Box((0.070, FORK_EAR_THICKNESS, 0.145)),
            origin=Origin(xyz=(0.0, y, 0.265)),
            material=steel,
            name=f"root_ear_{suffix}",
        )
        _joint_cylinder(
            base,
            center=(0.0, y, 0.240),
            radius=0.062,
            length=FORK_EAR_THICKNESS,
            material=gunmetal,
            name=f"root_boss_{suffix}",
        )
    _joint_cylinder(
        base,
        center=(0.0, 0.0, 0.240),
        radius=0.018,
        length=0.128,
        material=gunmetal,
        name="root_pin",
    )

    link_specs = [
        # dx, dz, spine_width, fork_width, boss_radius, dist_radius, material, accent, accent_side
        (0.42, 0.10, 0.074, 0.092, 0.054, 0.058, blue, dark, 1.0),
        (0.36, -0.06, 0.061, 0.079, 0.050, 0.052, orange, gunmetal, -1.0),
        (0.30, 0.13, 0.068, 0.087, 0.047, 0.055, green, dark, 1.0),
        (0.24, -0.03, 0.052, 0.071, 0.043, 0.047, violet, gunmetal, -1.0),
    ]

    links = []
    for index, (dx, dz, spine, fork, prox_r, dist_r, mat, accent, side) in enumerate(link_specs):
        link = model.part(f"link_{index}")
        _add_link(
            link,
            dx=dx,
            dz=dz,
            spine_width=spine,
            fork_width=fork,
            boss_radius=prox_r,
            dist_radius=dist_r,
            body_material=mat,
            dark_material=gunmetal,
            accent_material=accent,
            accent_side=side,
        )
        links.append(link)

    root_limits = MotionLimits(effort=18.0, velocity=2.0, lower=-0.80, upper=1.05)
    joint_limits = [
        MotionLimits(effort=12.0, velocity=2.4, lower=-1.20, upper=1.15),
        MotionLimits(effort=10.0, velocity=2.8, lower=-1.05, upper=1.30),
        MotionLimits(effort=8.0, velocity=3.0, lower=-1.35, upper=1.05),
    ]

    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=root_limits,
    )

    for index in range(3):
        dx, dz = link_specs[index][0], link_specs[index][1]
        model.articulation(
            f"link_{index}_to_link_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(dx, 0.0, dz)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=joint_limits[index],
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = object_model.articulations
    ctx.check(
        "four revolute joints",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint_count={len(joints)}, types={[j.articulation_type for j in joints]}",
    )
    ctx.check(
        "parallel y axes",
        all(tuple(round(v, 6) for v in (j.axis or ())) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.allow_overlap(
        "base",
        "link_0",
        elem_a="root_pin",
        elem_b="prox_boss",
        reason="The root hinge pin is intentionally captured inside the first link bushing.",
    )
    for index in range(3):
        ctx.allow_overlap(
            f"link_{index}",
            f"link_{index + 1}",
            elem_a="dist_pin",
            elem_b="prox_boss",
            reason="Each distal hinge pin is intentionally captured inside the next link bushing.",
        )

    # The root and every inter-link joint is a fork-and-tongue stack: the
    # projected bearing circles overlap in the motion plane, but the volumes are
    # separated along the pin axis so the revolute joints are visually captured.
    ctx.expect_within(
        "base",
        "link_0",
        axes="xz",
        inner_elem="root_pin",
        outer_elem="prox_boss",
        margin=0.001,
        name="root pin centered in bushing",
    )
    ctx.expect_overlap(
        "base",
        "link_0",
        axes="y",
        elem_a="root_pin",
        elem_b="prox_boss",
        min_overlap=AXIS_THICKNESS - 0.002,
        name="root pin passes through bushing",
    )
    ctx.expect_gap(
        "base",
        "link_0",
        axis="y",
        positive_elem="root_boss_pos",
        negative_elem="prox_boss",
        min_gap=FORK_GAP - 0.002,
        max_gap=FORK_GAP + 0.002,
        name="root positive fork clearance",
    )
    ctx.expect_gap(
        "link_0",
        "base",
        axis="y",
        positive_elem="prox_boss",
        negative_elem="root_boss_neg",
        min_gap=FORK_GAP - 0.002,
        max_gap=FORK_GAP + 0.002,
        name="root negative fork clearance",
    )
    ctx.expect_overlap(
        "base",
        "link_0",
        axes="xz",
        elem_a="root_boss_pos",
        elem_b="prox_boss",
        min_overlap=0.085,
        name="root bearing footprints align",
    )

    for index in range(3):
        parent = f"link_{index}"
        child = f"link_{index + 1}"
        ctx.expect_within(
            parent,
            child,
            axes="xz",
            inner_elem="dist_pin",
            outer_elem="prox_boss",
            margin=0.001,
            name=f"joint {index + 1} pin centered in bushing",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a="dist_pin",
            elem_b="prox_boss",
            min_overlap=AXIS_THICKNESS - 0.002,
            name=f"joint {index + 1} pin passes through bushing",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem="dist_boss_pos",
            negative_elem="prox_boss",
            min_gap=FORK_GAP - 0.002,
            max_gap=FORK_GAP + 0.002,
            name=f"joint {index + 1} positive fork clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem="prox_boss",
            negative_elem="dist_boss_neg",
            min_gap=FORK_GAP - 0.002,
            max_gap=FORK_GAP + 0.002,
            name=f"joint {index + 1} negative fork clearance",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="xz",
            elem_a="dist_boss_pos",
            elem_b="prox_boss",
            min_overlap=0.075,
            name=f"joint {index + 1} bearing footprints align",
        )

    rest_tip = ctx.part_world_position("link_3")
    with ctx.pose(
        {
            "base_to_link_0": 0.45,
            "link_0_to_link_1": -0.35,
            "link_1_to_link_2": 0.40,
            "link_2_to_link_3": -0.25,
        }
    ):
        moved_tip = ctx.part_world_position("link_3")

    ctx.check(
        "chain stays in one motion plane",
        moved_tip is not None and abs(moved_tip[1]) < 1.0e-6,
        details=f"moved_tip={moved_tip}",
    )
    ctx.check(
        "tip responds to four-joint pose",
        rest_tip is not None
        and moved_tip is not None
        and math.hypot(moved_tip[0] - rest_tip[0], moved_tip[2] - rest_tip[2]) > 0.05,
        details=f"rest_tip={rest_tip}, moved_tip={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
