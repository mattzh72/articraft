from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_cylinder_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 64,
) -> MeshGeometry:
    """Closed annular tube mesh along +Z, with a real central bore."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for radius in (outer_radius, inner_radius):
        for z in (z_min, z_max):
            ring: list[int] = []
            for i in range(segments):
                angle = 2.0 * math.pi * i / segments
                ring.append(geom.add_vertex(radius * math.cos(angle), radius * math.sin(angle), z))
            rings.append(ring)

    outer_bottom, outer_top, inner_bottom, inner_top = rings
    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner bore wall (opposite winding).
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        # Top and bottom annular faces.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    rubber = model.material("rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    clip_yellow = model.material("clip_yellow", rgba=(1.0, 0.72, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.20, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="base_plate",
    )
    body.visual(
        Box((0.11, 0.16, 0.010)),
        origin=Origin(xyz=(-0.105, 0.0, 0.040)),
        material=rubber,
        name="foot_pad_0",
    )
    body.visual(
        Box((0.11, 0.16, 0.010)),
        origin=Origin(xyz=(0.105, 0.0, 0.040)),
        material=rubber,
        name="foot_pad_1",
    )
    body.visual(
        mesh_from_geometry(
            _hollow_cylinder_mesh(outer_radius=0.039, inner_radius=0.022, z_min=0.032, z_max=0.760),
            "barrel_shell",
        ),
        material=polished_steel,
        name="barrel_shell",
    )
    body.visual(
        mesh_from_geometry(
            _hollow_cylinder_mesh(outer_radius=0.055, inner_radius=0.024, z_min=0.026, z_max=0.105),
            "lower_socket",
        ),
        material=dark_steel,
        name="lower_socket",
    )
    body.visual(
        mesh_from_geometry(
            _hollow_cylinder_mesh(outer_radius=0.052, inner_radius=0.016, z_min=0.735, z_max=0.805),
            "top_guide_collar",
        ),
        material=dark_steel,
        name="top_guide_collar",
    )
    body.visual(
        Box((0.028, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, 0.043, 0.230)),
        material=dark_steel,
        name="clip_pivot_boss",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.052),
        origin=Origin(xyz=(0.0, 0.061, 0.230), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clip_pivot_pin",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.016, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, -0.250)),
        material=polished_steel,
        name="piston_rod",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_steel,
        name="handle_stem",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="t_handle_bar",
    )
    plunger.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(-0.160, 0.0, 0.078)),
        material=rubber,
        name="handle_end_0",
    )
    plunger.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.160, 0.0, 0.078)),
        material=rubber,
        name="handle_end_1",
    )

    clip = model.part("hose_clip")
    clip.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clip_yellow,
        name="clip_hub",
    )
    clip.visual(
        Box((0.020, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=clip_yellow,
        name="clip_back",
    )
    clip.visual(
        Box((0.060, 0.014, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.088)),
        material=clip_yellow,
        name="hook_shelf",
    )
    clip.visual(
        Box((0.018, 0.014, 0.046)),
        origin=Origin(xyz=(0.054, 0.0, -0.070)),
        material=clip_yellow,
        name="hook_tip",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=0.0, upper=0.280),
    )
    model.articulation(
        "body_to_hose_clip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clip,
        origin=Origin(xyz=(0.0, 0.079, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    hose_clip = object_model.get_part("hose_clip")
    slide = object_model.get_articulation("body_to_plunger")
    clip_pivot = object_model.get_articulation("body_to_hose_clip")

    ctx.allow_overlap(
        body,
        hose_clip,
        elem_a="clip_pivot_pin",
        elem_b="clip_hub",
        reason="The clip hub is intentionally captured around the small side pivot pin.",
    )
    ctx.allow_overlap(
        body,
        plunger,
        elem_a="top_guide_collar",
        elem_b="piston_rod",
        reason="The rod is intentionally shown as a close sliding fit through the top guide bushing.",
    )

    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="piston_rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="piston rod stays centered inside the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="barrel_shell",
        min_overlap=0.40,
        name="collapsed rod remains deeply inserted in the barrel",
    )
    ctx.expect_overlap(
        body,
        hose_clip,
        axes="xyz",
        elem_a="clip_pivot_pin",
        elem_b="clip_hub",
        min_overlap=0.010,
        name="hose clip hub surrounds the side pivot pin",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="piston_rod",
        elem_b="top_guide_collar",
        min_overlap=0.060,
        name="rod is retained through the top guide bushing",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(plunger, elem="t_handle_bar")
    rest_hook_aabb = ctx.part_element_world_aabb(hose_clip, elem="hook_tip")
    with ctx.pose({slide: 0.280, clip_pivot: 0.65}):
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="piston_rod",
            elem_b="barrel_shell",
            min_overlap=0.20,
            name="extended rod still has retained insertion",
        )
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="piston_rod",
            outer_elem="barrel_shell",
            margin=0.0,
            name="extended rod remains coaxial with the barrel",
        )
        raised_handle_aabb = ctx.part_element_world_aabb(plunger, elem="t_handle_bar")
        rotated_hook_aabb = ctx.part_element_world_aabb(hose_clip, elem="hook_tip")

    ctx.check(
        "T-handle slides upward along the barrel axis",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[0][2] > rest_handle_aabb[0][2] + 0.25,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )
    ctx.check(
        "hose clip rotates about its side pivot",
        rest_hook_aabb is not None
        and rotated_hook_aabb is not None
        and rotated_hook_aabb[0][0] < rest_hook_aabb[0][0] - 0.025,
        details=f"rest={rest_hook_aabb}, rotated={rotated_hook_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
