from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _arched_profile(width: float, side_height: float, *, segments: int = 24) -> list[tuple[float, float]]:
    """Closed x/z profile for a flat-bottomed round arch."""

    radius = width / 2.0
    points: list[tuple[float, float]] = [
        (-radius, 0.0),
        (radius, 0.0),
        (radius, side_height),
    ]
    for i in range(1, segments + 1):
        theta = math.pi * i / segments
        points.append((radius * math.cos(theta), side_height + radius * math.sin(theta)))
    return points


def _extruded_arch(width: float, side_height: float, thickness: float, *, segments: int = 24) -> MeshGeometry:
    """Extrude an arched profile through local Y, with the bottom on z=0."""

    profile = _arched_profile(width, side_height, segments=segments)
    geom = MeshGeometry()
    front: list[int] = []
    back: list[int] = []
    for x, z in profile:
        front.append(geom.add_vertex(x, -thickness / 2.0, z))
    for x, z in profile:
        back.append(geom.add_vertex(x, thickness / 2.0, z))

    center_front = geom.add_vertex(0.0, -thickness / 2.0, side_height * 0.45)
    center_back = geom.add_vertex(0.0, thickness / 2.0, side_height * 0.45)
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(center_front, front[i], front[j])
        geom.add_face(center_back, back[j], back[i])
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _arch_ring(
    outer_width: float,
    outer_side_height: float,
    inner_width: float,
    inner_side_height: float,
    depth: float,
    *,
    segments: int = 32,
) -> MeshGeometry:
    """Semi-annular top arch band, extruded through local Y."""

    ro = outer_width / 2.0
    ri = inner_width / 2.0
    geom = MeshGeometry()
    outer_front: list[int] = []
    inner_front: list[int] = []
    outer_back: list[int] = []
    inner_back: list[int] = []

    for i in range(segments + 1):
        theta = math.pi * i / segments
        xo = ro * math.cos(theta)
        zo = outer_side_height + ro * math.sin(theta)
        xi = ri * math.cos(theta)
        zi = inner_side_height + ri * math.sin(theta)
        outer_front.append(geom.add_vertex(xo, -depth / 2.0, zo))
        inner_front.append(geom.add_vertex(xi, -depth / 2.0, zi))
        outer_back.append(geom.add_vertex(xo, depth / 2.0, zo))
        inner_back.append(geom.add_vertex(xi, depth / 2.0, zi))

    for i in range(segments):
        j = i + 1
        # Front and rear faces of the masonry arch band.
        geom.add_face(outer_front[i], outer_front[j], inner_front[j])
        geom.add_face(outer_front[i], inner_front[j], inner_front[i])
        geom.add_face(outer_back[i], inner_back[j], outer_back[j])
        geom.add_face(outer_back[i], inner_back[i], inner_back[j])
        # Curved outside and inside returns through the depth.
        geom.add_face(outer_front[i], outer_back[i], outer_back[j])
        geom.add_face(outer_front[i], outer_back[j], outer_front[j])
        geom.add_face(inner_front[i], inner_front[j], inner_back[j])
        geom.add_face(inner_front[i], inner_back[j], inner_back[i])

    # End faces at the two spring points.
    for i in (0, segments):
        geom.add_face(outer_front[i], inner_front[i], inner_back[i])
        geom.add_face(outer_front[i], inner_back[i], outer_back[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wood_fired_bread_oven")

    firebrick = Material("warm_firebrick", rgba=(0.72, 0.54, 0.34, 1.0))
    pale_clay = Material("limewashed_clay", rgba=(0.78, 0.70, 0.58, 1.0))
    hearth_stone = Material("hearth_stone", rgba=(0.42, 0.39, 0.34, 1.0))
    soot = Material("soot_black", rgba=(0.015, 0.013, 0.011, 1.0))
    iron = Material("seasoned_cast_iron", rgba=(0.035, 0.037, 0.040, 1.0))
    ash = Material("ash_dust", rgba=(0.35, 0.34, 0.32, 1.0))

    oven = model.part("oven")

    # Raised masonry hearth with a real front cavity for the ash drawer.
    oven.visual(
        Box((1.68, 1.36, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=hearth_stone,
        name="bottom_plinth",
    )
    oven.visual(
        Box((0.34, 1.18, 0.35)),
        origin=Origin(xyz=(-0.60, -0.02, 0.245)),
        material=firebrick,
        name="support_pier_0",
    )
    oven.visual(
        Box((0.34, 1.18, 0.35)),
        origin=Origin(xyz=(0.60, -0.02, 0.245)),
        material=firebrick,
        name="support_pier_1",
    )
    oven.visual(
        Box((0.90, 0.30, 0.35)),
        origin=Origin(xyz=(0.0, 0.45, 0.245)),
        material=firebrick,
        name="rear_support",
    )
    oven.visual(
        Box((1.68, 1.36, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=hearth_stone,
        name="hearth_lintel",
    )
    oven.visual(
        Box((1.48, 1.18, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=hearth_stone,
        name="baking_hearth",
    )

    dome = DomeGeometry(0.68, radial_segments=56, height_segments=18, closed=True)
    dome.scale(1.0, 0.82, 0.96).translate(0.0, 0.0, 0.66)
    oven.visual(
        mesh_from_geometry(dome, "domed_body"),
        material=pale_clay,
        name="domed_body",
    )

    # Iron guide rails inside the base make the ash drawer a supported slider rather than a floating pan.
    oven.visual(
        Box((0.135, 0.72, 0.055)),
        origin=Origin(xyz=(-0.3675, -0.28, 0.22)),
        material=iron,
        name="drawer_guide_0",
    )
    oven.visual(
        Box((0.135, 0.72, 0.055)),
        origin=Origin(xyz=(0.3675, -0.28, 0.22)),
        material=iron,
        name="drawer_guide_1",
    )

    # Projecting arched face: side jambs plus a curved voussoir band around the mouth.
    arch_depth = 0.16
    arch_y = -0.64
    arch_base_z = 0.66
    outer_w = 0.82
    inner_w = 0.58
    outer_side = 0.27
    inner_side = 0.22
    jamb_w = (outer_w - inner_w) / 2.0
    for idx, x in enumerate((-(inner_w / 2.0 + jamb_w / 2.0), inner_w / 2.0 + jamb_w / 2.0)):
        oven.visual(
            Box((jamb_w, arch_depth, outer_side)),
            origin=Origin(xyz=(x, arch_y, arch_base_z + outer_side / 2.0)),
            material=firebrick,
            name=f"arch_jamb_{idx}",
        )
    arch_band = _arch_ring(outer_w, outer_side, inner_w, inner_side, arch_depth, segments=36)
    arch_band.translate(0.0, arch_y, arch_base_z)
    oven.visual(
        mesh_from_geometry(arch_band, "arch_ring"),
        material=firebrick,
        name="arch_ring",
    )
    shadow = _extruded_arch(inner_w + 0.035, inner_side + 0.012, 0.012, segments=30)
    shadow.translate(0.0, -0.718, arch_base_z)
    oven.visual(
        mesh_from_geometry(shadow, "firebox_shadow"),
        material=soot,
        name="firebox_shadow",
    )

    # Two visible fixed hinge pin brackets at the lower arch face.
    for idx, x in enumerate((-0.265, 0.265)):
        bracket_x = x + (-0.054 if x < 0.0 else 0.054)
        oven.visual(
            Box((0.026, 0.07, 0.055)),
            origin=Origin(xyz=(bracket_x, -0.735, 0.635)),
            material=iron,
            name=f"hinge_bracket_{idx}",
        )
        oven.visual(
            Cylinder(radius=0.018, length=0.12),
            origin=Origin(xyz=(x, -0.775, 0.66), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"hinge_pin_{idx}",
        )

    door = model.part("door")
    door_plate = _extruded_arch(0.62, 0.24, 0.052, segments=36)
    door_plate.translate(0.0, 0.0, 0.035)
    door.visual(
        mesh_from_geometry(door_plate, "door_plate"),
        material=iron,
        name="door_plate",
    )
    for idx, x in enumerate((-0.18, 0.0, 0.18)):
        door.visual(
            Box((0.026, 0.014, 0.34)),
            origin=Origin(xyz=(x, -0.032, 0.24)),
            material=iron,
            name=f"cast_rib_{idx}",
        )
    for idx, (x, z) in enumerate(((-0.23, 0.16), (0.23, 0.16), (-0.16, 0.43), (0.16, 0.43))):
        door.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, -0.031, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"rivet_{idx}",
        )
    for idx, x in enumerate((-0.12, 0.12)):
        door.visual(
            Cylinder(radius=0.013, length=0.082),
            origin=Origin(xyz=(x, -0.066, 0.33), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=f"handle_post_{idx}",
        )
    door.visual(
        Cylinder(radius=0.019, length=0.31),
        origin=Origin(xyz=(0.0, -0.108, 0.33), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_handle",
    )
    for idx, x in enumerate((-0.265, 0.265)):
        door.visual(
            Box((0.070, 0.030, 0.075)),
            origin=Origin(xyz=(x, 0.000, 0.061)),
            material=iron,
            name=f"hinge_strap_{idx}",
        )
        door.visual(
            Cylinder(radius=0.024, length=0.072),
            origin=Origin(xyz=(x, 0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=iron,
            name=f"hinge_eye_{idx}",
        )

    model.articulation(
        "oven_to_door",
        ArticulationType.REVOLUTE,
        parent=oven,
        child=door,
        origin=Origin(xyz=(0.0, -0.780, 0.66)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=1.55),
    )

    drawer = model.part("ash_drawer")
    drawer.visual(
        Box((0.56, 0.50, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
        material=iron,
        name="tray_floor",
    )
    drawer.visual(
        Box((0.025, 0.50, 0.12)),
        origin=Origin(xyz=(-0.292, 0.0, -0.010)),
        material=iron,
        name="tray_side_0",
    )
    drawer.visual(
        Box((0.025, 0.50, 0.12)),
        origin=Origin(xyz=(0.292, 0.0, -0.010)),
        material=iron,
        name="tray_side_1",
    )
    drawer.visual(
        Box((0.56, 0.026, 0.12)),
        origin=Origin(xyz=(0.0, 0.237, -0.010)),
        material=iron,
        name="tray_back",
    )
    drawer.visual(
        Box((0.72, 0.052, 0.24)),
        origin=Origin(xyz=(0.0, -0.260, 0.0)),
        material=iron,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.46, 0.34, 0.012)),
        origin=Origin(xyz=(0.0, 0.025, -0.062)),
        material=ash,
        name="ash_bed",
    )
    drawer.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, -0.306, 0.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="drawer_pull",
    )

    model.articulation(
        "oven_to_ash_drawer",
        ArticulationType.PRISMATIC,
        parent=oven,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.405, 0.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    oven = object_model.get_part("oven")
    door = object_model.get_part("door")
    drawer = object_model.get_part("ash_drawer")
    door_joint = object_model.get_articulation("oven_to_door")
    drawer_joint = object_model.get_articulation("oven_to_ash_drawer")

    for idx in (0, 1):
        ctx.allow_overlap(
            oven,
            door,
            elem_a=f"hinge_pin_{idx}",
            elem_b=f"hinge_eye_{idx}",
            reason="The cast-iron hinge eye is intentionally modeled as captured around the fixed hinge pin.",
        )
        ctx.expect_overlap(
            oven,
            door,
            axes="xyz",
            elem_a=f"hinge_pin_{idx}",
            elem_b=f"hinge_eye_{idx}",
            min_overlap=0.025,
            name=f"hinge pin {idx} is captured in the door eye",
        )
    ctx.allow_overlap(
        oven,
        drawer,
        elem_a="drawer_guide_0",
        elem_b="tray_side_0",
        reason="The ash drawer side wall is represented as sliding in a close iron guide rail.",
    )
    ctx.allow_overlap(
        oven,
        drawer,
        elem_a="drawer_guide_1",
        elem_b="tray_side_1",
        reason="The ash drawer side wall is represented as sliding in a close iron guide rail.",
    )
    ctx.expect_overlap(
        oven,
        drawer,
        axes="y",
        elem_a="drawer_guide_0",
        elem_b="tray_side_0",
        min_overlap=0.40,
        name="left ash drawer rail supports the tray through its travel",
    )
    ctx.expect_overlap(
        oven,
        drawer,
        axes="y",
        elem_a="drawer_guide_1",
        elem_b="tray_side_1",
        min_overlap=0.40,
        name="right ash drawer rail supports the tray through its travel",
    )

    ctx.expect_gap(
        oven,
        door,
        axis="y",
        positive_elem="arch_ring",
        negative_elem="door_plate",
        min_gap=0.025,
        max_gap=0.050,
        name="closed iron door stands just proud of the arch face",
    )
    ctx.expect_overlap(
        door,
        oven,
        axes="xz",
        elem_a="door_plate",
        elem_b="firebox_shadow",
        min_overlap=0.45,
        name="arched door covers the firebox mouth",
    )
    ctx.expect_within(
        drawer,
        oven,
        axes="xy",
        inner_elem="tray_floor",
        outer_elem="hearth_lintel",
        margin=0.02,
        name="ash drawer tray is centered below the hearth",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.55}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "bottom-hinged door folds downward and outward",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < rest_door_aabb[1][2] - 0.35
        and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.45,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_drawer_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_joint: 0.42}):
        extended_drawer_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(
            drawer,
            oven,
            axes="y",
            elem_a="tray_floor",
            elem_b="hearth_lintel",
            min_overlap=0.05,
            name="extended ash drawer retains insertion under the hearth",
        )
    ctx.check(
        "ash drawer slides out through the front",
        rest_drawer_aabb is not None
        and extended_drawer_aabb is not None
        and extended_drawer_aabb[0][1] < rest_drawer_aabb[0][1] - 0.35,
        details=f"rest={rest_drawer_aabb}, extended={extended_drawer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
