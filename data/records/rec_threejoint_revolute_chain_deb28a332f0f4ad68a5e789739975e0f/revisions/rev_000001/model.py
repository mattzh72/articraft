from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIS_Y = (0.0, 1.0, 0.0)
PLATE_ROT = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 36, *, reverse: bool = False):
    angles = [2.0 * math.pi * i / segments for i in range(segments)]
    if reverse:
        angles = list(reversed(angles))
    return [(cx + radius * math.cos(a), cy + radius * math.sin(a)) for a in angles]


def _capsule_profile(start_y: float, end_y: float, radius: float, segments: int = 18):
    """A vertical capsule in local XY, where +Y becomes downward after PLATE_ROT."""
    pts = []
    pts.append((radius, start_y))
    pts.append((radius, end_y))
    for i in range(1, segments + 1):
        a = 0.0 + math.pi * i / segments
        pts.append((radius * math.cos(a), end_y + radius * math.sin(a)))
    pts.append((-radius, start_y))
    for i in range(1, segments + 1):
        a = math.pi + math.pi * i / segments
        pts.append((radius * math.cos(a), start_y + radius * math.sin(a)))
    return pts


def _plate_mesh(name: str, outer_profile, holes, thickness: float):
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        holes,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _add_pin(part, *, z: float, length: float, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )
    cap_radius = radius * 1.65
    cap_len = 0.004
    for side, y in (("negative", -(length + cap_len) / 2.0), ("positive", (length + cap_len) / 2.0)):
        part.visual(
            Cylinder(radius=cap_radius, length=cap_len),
            origin=Origin(xyz=(0.0, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=f"{name}_cap_{side}",
        )


def _add_hanging_link(
    part,
    *,
    length: float,
    thickness: float,
    material,
    pin_material,
    mesh_prefix: str,
) -> None:
    web_radius = 0.027
    bore_radius = 0.012
    fork_radius = 0.024
    fork_start = length - 0.078

    body_outer = _capsule_profile(0.0, length - 0.075, web_radius)
    body_holes = [_circle_profile(0.0, 0.0, bore_radius)]
    part.visual(
        _plate_mesh(f"{mesh_prefix}_body_mesh", body_outer, body_holes, thickness),
        material=material,
        name="body",
    )

    # A real fork bridge joins the central strap to the two outer cheek ears
    # above the next link's swinging eye, leaving the hinge itself open.
    part.visual(
        Box((0.070, 0.068, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -length + 0.075)),
        material=material,
        name="fork_bridge",
    )

    ear_outer = _capsule_profile(fork_start, length, fork_radius)
    ear_holes = [_circle_profile(0.0, length, bore_radius)]
    for suffix, y in (("0", -0.026), ("1", 0.026)):
        part.visual(
            _plate_mesh(f"{mesh_prefix}_fork_ear_{suffix}_mesh", ear_outer, ear_holes, 0.011),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=material,
            name=f"fork_ear_{suffix}",
        )

    _add_pin(
        part,
        z=-length,
        length=0.078,
        radius=0.008,
        material=pin_material,
        name="lower_pin",
    )


def _add_end_tab(part, *, length: float, thickness: float, material, mesh_prefix: str) -> None:
    outer = _capsule_profile(0.0, length, 0.028)
    holes = [
        _circle_profile(0.0, 0.0, 0.012),
        _circle_profile(0.0, length - 0.035, 0.007),
    ]
    part.visual(
        _plate_mesh(f"{mesh_prefix}_mesh", outer, holes, thickness),
        material=material,
        name="tab_body",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_three_joint_chain")

    bracket_mat = Material("powder_coated_bracket", color=(0.16, 0.17, 0.18, 1.0))
    link_mat = Material("blue_anodized_links", color=(0.05, 0.22, 0.65, 1.0))
    tab_mat = Material("burnished_end_tab", color=(0.55, 0.56, 0.52, 1.0))
    pin_mat = Material("brushed_steel_pins", color=(0.78, 0.76, 0.70, 1.0))
    dark_mat = Material("black_recesses", color=(0.015, 0.015, 0.012, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.18, 0.12, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=bracket_mat,
        name="top_plate",
    )
    for suffix, y in (("0", -0.035), ("1", 0.035)):
        support.visual(
            Box((0.078, 0.012, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.025)),
            material=bracket_mat,
            name=f"cheek_{suffix}",
        )
        support.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.19, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_mat,
            name=f"outer_boss_{suffix}",
        )
    _add_pin(support, z=0.0, length=0.088, radius=0.008, material=pin_mat, name="top_pin")

    for i, x in enumerate((-0.055, 0.055)):
        for j, y in enumerate((-0.033, 0.033)):
            support.visual(
                Cylinder(radius=0.007, length=0.003),
                origin=Origin(xyz=(x, y, 0.1065)),
                material=dark_mat,
                name=f"mount_bolt_{i}_{j}",
            )

    link_0_len = 0.285
    link_1_len = 0.245
    tab_len = 0.120

    link_0 = model.part("link_0")
    _add_hanging_link(
        link_0,
        length=link_0_len,
        thickness=0.024,
        material=link_mat,
        pin_material=pin_mat,
        mesh_prefix="link_0",
    )

    link_1 = model.part("link_1")
    _add_hanging_link(
        link_1,
        length=link_1_len,
        thickness=0.024,
        material=link_mat,
        pin_material=pin_mat,
        mesh_prefix="link_1",
    )

    end_tab = model.part("end_tab")
    _add_end_tab(end_tab, length=tab_len, thickness=0.022, material=tab_mat, mesh_prefix="end_tab")

    limits = MotionLimits(effort=8.0, velocity=2.5, lower=-1.35, upper=1.35)
    model.articulation(
        "support_to_link_0",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=AXIS_Y,
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.0, 0.0, -link_0_len)),
        axis=AXIS_Y,
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -link_1_len)),
        axis=AXIS_Y,
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_tab = object_model.get_part("end_tab")
    j0 = object_model.get_articulation("support_to_link_0")
    j1 = object_model.get_articulation("link_0_to_link_1")
    j2 = object_model.get_articulation("link_1_to_end_tab")

    revolute_joints = [j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE]
    ctx.allow_overlap(
        support,
        link_0,
        elem_a="top_pin",
        elem_b="body",
        reason="The top shaft is intentionally captured through the first link's bored eye.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="lower_pin",
        elem_b="body",
        reason="The middle shaft is intentionally captured through the second link's bored eye.",
    )
    ctx.allow_overlap(
        link_1,
        end_tab,
        elem_a="lower_pin",
        elem_b="tab_body",
        reason="The lower shaft is intentionally captured through the compact end tab's bored eye.",
    )

    ctx.check(
        "exactly three revolute hinge joints",
        len(revolute_joints) == 3,
        details=f"found {len(revolute_joints)} revolute joints",
    )
    ctx.check(
        "hinge axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == AXIS_Y for j in revolute_joints),
        details=f"axes={[j.axis for j in revolute_joints]}",
    )

    ctx.expect_origin_gap(link_0, link_1, axis="z", min_gap=0.20, name="second link hangs below first joint")
    ctx.expect_origin_gap(link_1, end_tab, axis="z", min_gap=0.18, name="end tab hangs below second link")
    ctx.expect_overlap(
        support,
        link_0,
        axes="xy",
        elem_a="top_pin",
        elem_b="body",
        min_overlap=0.010,
        name="top pin is centered through first link eye",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="xy",
        elem_a="lower_pin",
        elem_b="body",
        min_overlap=0.010,
        name="middle pin is centered through second link eye",
    )
    ctx.expect_overlap(
        link_1,
        end_tab,
        axes="xy",
        elem_a="lower_pin",
        elem_b="tab_body",
        min_overlap=0.010,
        name="lower pin is centered through end tab eye",
    )

    rest_link_1 = ctx.part_world_position(link_1)
    rest_tab = ctx.part_world_position(end_tab)
    with ctx.pose({j0: 0.55, j1: -0.30, j2: 0.45}):
        posed_link_1 = ctx.part_world_position(link_1)
        posed_tab = ctx.part_world_position(end_tab)
    ctx.check(
        "first hinge swings the suspended chain sideways",
        rest_link_1 is not None
        and posed_link_1 is not None
        and abs(posed_link_1[0] - rest_link_1[0]) > 0.08,
        details=f"rest={rest_link_1}, posed={posed_link_1}",
    )
    ctx.check(
        "downstream joints move compact end tab",
        rest_tab is not None and posed_tab is not None and abs(posed_tab[0] - rest_tab[0]) > 0.12,
        details=f"rest={rest_tab}, posed={posed_tab}",
    )

    return ctx.report()


object_model = build_object_model()
