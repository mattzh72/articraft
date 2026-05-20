from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _oval_solid(
    *,
    depth_x: float,
    width_y: float,
    height_z: float,
    center=(0.0, 0.0, 0.0),
    segments: int = 64,
    exponent: float = 2.7,
) -> MeshGeometry:
    """Closed superellipse solid extruded along local X."""
    profile = superellipse_profile(width_y, height_z, exponent=exponent, segments=segments)
    geom = MeshGeometry()
    back = []
    front = []
    for y, z in profile:
        back.append(geom.add_vertex(-depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    for y, z in profile:
        front.append(geom.add_vertex(depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    n = len(profile)
    cb = geom.add_vertex(-depth_x / 2.0 + center[0], center[1], center[2])
    cf = geom.add_vertex(depth_x / 2.0 + center[0], center[1], center[2])
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, back[i], back[j], front[j], front[i])
        geom.add_face(cb, back[j], back[i])
        geom.add_face(cf, front[i], front[j])
    return geom


def _oval_ring(
    *,
    depth_x: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    center=(0.0, 0.0, 0.0),
    segments: int = 64,
    exponent: float = 2.7,
) -> MeshGeometry:
    """Thin oval cushion ring extruded along local X with a real center opening."""
    outer = superellipse_profile(outer_y, outer_z, exponent=exponent, segments=segments)
    inner = superellipse_profile(inner_y, inner_z, exponent=exponent, segments=segments)
    geom = MeshGeometry()
    ob, of, ib, inf = [], [], [], []
    for y, z in outer:
        ob.append(geom.add_vertex(-depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    for y, z in outer:
        of.append(geom.add_vertex(depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    for y, z in inner:
        ib.append(geom.add_vertex(-depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    for y, z in inner:
        inf.append(geom.add_vertex(depth_x / 2.0 + center[0], y + center[1], z + center[2]))
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, ob[i], ob[j], of[j], of[i])
        _add_quad(geom, ib[j], ib[i], inf[i], inf[j])
        _add_quad(geom, of[i], of[j], inf[j], inf[i])
        _add_quad(geom, ob[j], ob[i], ib[i], ib[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_wireless_headphone")

    matte_black = model.material("matte_black", color=(0.015, 0.015, 0.018, 1.0))
    charcoal = model.material("charcoal_plastic", color=(0.08, 0.085, 0.09, 1.0))
    soft_foam = model.material("soft_foam", color=(0.0, 0.0, 0.0, 1.0))
    dark_metal = model.material("dark_metal", color=(0.35, 0.36, 0.38, 1.0))
    accent = model.material("cool_grey_accent", color=(0.18, 0.24, 0.30, 1.0))

    headband = model.part("headband")
    band_mesh = tube_from_spline_points(
        [
            (-0.101, 0.0, 0.086),
            (-0.092, 0.0, 0.118),
            (-0.055, 0.0, 0.164),
            (0.000, 0.0, 0.184),
            (0.055, 0.0, 0.164),
            (0.092, 0.0, 0.118),
            (0.101, 0.0, 0.086),
        ],
        radius=0.0085,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    headband.visual(
        mesh_from_geometry(band_mesh, "arched_headband"),
        material=matte_black,
        name="arched_band",
    )
    pad_mesh = tube_from_spline_points(
        [
            (-0.078, 0.0, 0.122),
            (-0.040, 0.0, 0.154),
            (0.000, 0.0, 0.171),
            (0.040, 0.0, 0.154),
            (0.078, 0.0, 0.122),
        ],
        radius=0.0055,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    headband.visual(
        mesh_from_geometry(pad_mesh, "underside_head_pad"),
        material=soft_foam,
        name="underside_pad",
    )
    for side_name, x in (("left", -0.108), ("right", 0.108)):
        headband.visual(
            Cylinder(radius=0.0125, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"{side_name}_fold_socket",
        )
        headband.visual(
            Box((0.026, 0.044, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.083)),
            material=charcoal,
            name=f"{side_name}_side_cap",
        )
    for i, x in enumerate((-0.040, 0.040)):
        headband.visual(
            Cylinder(radius=0.0075, length=0.015),
            origin=Origin(xyz=(x, 0.0, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"carry_socket_{i}",
        )
    headband.visual(
        Box((0.094, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.006, 0.193)),
        material=accent,
        name="carry_recess",
    )

    carry_loop = model.part("carry_loop")
    loop_mesh = tube_from_spline_points(
        [
            (-0.040, 0.000, 0.003),
            (-0.030, 0.036, 0.004),
            (0.000, 0.057, 0.004),
            (0.030, 0.036, 0.004),
            (0.040, 0.000, 0.003),
        ],
        radius=0.0037,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    carry_loop.visual(
        mesh_from_geometry(loop_mesh, "folding_carry_loop"),
        material=dark_metal,
        name="loop_tube",
    )
    for i, x in enumerate((-0.040, 0.040)):
        carry_loop.visual(
            Sphere(radius=0.0062),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_metal,
            name=f"pivot_{i}",
        )
    carry_loop.visual(
        Box((0.026, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.055, 0.004)),
        material=soft_foam,
        name="finger_pad",
    )

    def make_yoke(name: str) -> object:
        yoke = model.part(name)
        yoke.visual(
            Cylinder(radius=0.0072, length=0.084),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="fold_pin",
        )
        for y in (-0.037, 0.037):
            yoke.visual(
                Box((0.014, 0.014, 0.012)),
                origin=Origin(xyz=(0.0, y, -0.010)),
                material=charcoal,
                name=f"hinge_cheek_{'front' if y > 0 else 'rear'}",
            )
        for y in (-0.044, 0.044):
            yoke.visual(
                Box((0.010, 0.006, 0.070)),
                origin=Origin(xyz=(0.0, y, -0.040)),
                material=charcoal,
                name=f"side_arm_{'front' if y > 0 else 'rear'}",
            )
        for y in (-0.026, 0.026):
            yoke.visual(
                Box((0.010, 0.035, 0.006)),
                origin=Origin(xyz=(0.0, y, -0.075)),
                material=charcoal,
                name=f"pivot_bridge_{'front' if y > 0 else 'rear'}",
            )
        yoke.visual(
            Cylinder(radius=0.0125, length=0.013),
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
            material=dark_metal,
            name="swivel_socket",
        )
        return yoke

    left_yoke = make_yoke("left_yoke")
    right_yoke = make_yoke("right_yoke")

    def make_cup(name: str, inner_sign: float) -> object:
        cup = model.part(name)
        shell = _oval_solid(
            depth_x=0.038,
            width_y=0.073,
            height_z=0.090,
            center=(0.0, 0.0, -0.052),
            segments=72,
            exponent=2.45,
        )
        cushion = _oval_ring(
            depth_x=0.012,
            outer_y=0.078,
            outer_z=0.094,
            inner_y=0.043,
            inner_z=0.057,
            center=(inner_sign * 0.023, 0.0, -0.052),
            segments=72,
            exponent=2.55,
        )
        grille = _oval_solid(
            depth_x=0.004,
            width_y=0.050,
            height_z=0.064,
            center=(inner_sign * 0.028, 0.0, -0.052),
            segments=56,
            exponent=2.2,
        )
        outer_cap = _oval_solid(
            depth_x=0.004,
            width_y=0.052,
            height_z=0.064,
            center=(-inner_sign * 0.021, 0.0, -0.052),
            segments=56,
            exponent=2.7,
        )
        cup.visual(mesh_from_geometry(shell, f"{name}_shell_mesh"), material=charcoal, name="outer_shell")
        cup.visual(mesh_from_geometry(cushion, f"{name}_cushion_mesh"), material=soft_foam, name="ear_cushion")
        cup.visual(mesh_from_geometry(grille, f"{name}_grille_mesh"), material=matte_black, name="driver_grille")
        cup.visual(mesh_from_geometry(outer_cap, f"{name}_outer_cap_mesh"), material=accent, name="outer_cap")
        cup.visual(
            Cylinder(radius=0.0075, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_metal,
            name="swivel_stem",
        )
        return cup

    left_cup = make_cup("left_cup", inner_sign=1.0)
    right_cup = make_cup("right_cup", inner_sign=-1.0)

    model.articulation(
        "left_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.108, 0.0, 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.108, 0.0, 0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.15),
    )
    model.articulation(
        "left_swivel_pivot",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "right_swivel_pivot",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_cup,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "carry_loop_pivot",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=carry_loop,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_cup")
    right_cup = object_model.get_part("right_cup")
    carry_loop = object_model.get_part("carry_loop")

    for side, yoke in (("left", left_yoke), ("right", right_yoke)):
        ctx.allow_overlap(
            headband,
            yoke,
            elem_a=f"{side}_fold_socket",
            elem_b="fold_pin",
            reason="The yoke fold pin is intentionally captured inside the headband hinge barrel.",
        )
        ctx.expect_within(
            yoke,
            headband,
            axes="xz",
            inner_elem="fold_pin",
            outer_elem=f"{side}_fold_socket",
            margin=0.001,
            name=f"{side} fold pin centered in socket",
        )
        ctx.expect_overlap(
            yoke,
            headband,
            axes="y",
            elem_a="fold_pin",
            elem_b=f"{side}_fold_socket",
            min_overlap=0.040,
            name=f"{side} fold pin retained through socket",
        )

    for side, yoke, cup in (("left", left_yoke, left_cup), ("right", right_yoke, right_cup)):
        ctx.allow_overlap(
            yoke,
            cup,
            elem_a="swivel_socket",
            elem_b="swivel_stem",
            reason="The earcup swivel stem is intentionally nested in the yoke pivot socket.",
        )
        ctx.expect_within(
            cup,
            yoke,
            axes="xy",
            inner_elem="swivel_stem",
            outer_elem="swivel_socket",
            margin=0.001,
            name=f"{side} swivel stem centered in yoke socket",
        )
        ctx.expect_overlap(
            cup,
            yoke,
            axes="z",
            elem_a="swivel_stem",
            elem_b="swivel_socket",
            min_overlap=0.009,
            name=f"{side} swivel stem retained vertically",
        )

    for i in (0, 1):
        ctx.allow_overlap(
            headband,
            carry_loop,
            elem_a=f"carry_socket_{i}",
            elem_b=f"pivot_{i}",
            reason="The carry-loop pivot button is seated in its headband socket.",
        )
        ctx.allow_overlap(
            headband,
            carry_loop,
            elem_a=f"carry_socket_{i}",
            elem_b="loop_tube",
            reason="The metal carry loop tube intentionally enters the same small pivot boss at each end.",
        )
        ctx.expect_within(
            carry_loop,
            headband,
            axes="yz",
            inner_elem=f"pivot_{i}",
            outer_elem=f"carry_socket_{i}",
            margin=0.001,
            name=f"carry pivot {i} centered in socket",
        )
        ctx.expect_overlap(
            carry_loop,
            headband,
            axes="x",
            elem_a=f"pivot_{i}",
            elem_b=f"carry_socket_{i}",
            min_overlap=0.008,
            name=f"carry pivot {i} retained in socket",
        )
        ctx.expect_overlap(
            carry_loop,
            headband,
            axes="xyz",
            elem_a="loop_tube",
            elem_b=f"carry_socket_{i}",
            min_overlap=0.002,
            name=f"carry loop tube reaches pivot boss {i}",
        )

    ctx.check(
        "five user facing revolute mechanisms",
        len(object_model.articulations) == 5
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    left_fold = object_model.get_articulation("left_fold_hinge")
    right_fold = object_model.get_articulation("right_fold_hinge")
    left_swivel = object_model.get_articulation("left_swivel_pivot")
    right_swivel = object_model.get_articulation("right_swivel_pivot")
    carry_pivot = object_model.get_articulation("carry_loop_pivot")

    rest_left = ctx.part_world_position(left_cup)
    rest_right = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: 0.9, right_fold: 0.9}):
        folded_left = ctx.part_world_position(left_cup)
        folded_right = ctx.part_world_position(right_cup)
    ctx.check(
        "fold hinges swing earcups inward",
        rest_left is not None
        and rest_right is not None
        and folded_left is not None
        and folded_right is not None
        and folded_left[0] > rest_left[0] + 0.035
        and folded_right[0] < rest_right[0] - 0.035,
        details=f"left {rest_left}->{folded_left}, right {rest_right}->{folded_right}",
    )

    rest_loop_aabb = ctx.part_world_aabb(carry_loop)
    with ctx.pose({carry_pivot: 1.25}):
        raised_loop_aabb = ctx.part_world_aabb(carry_loop)
    ctx.check(
        "carry loop folds up from headband",
        rest_loop_aabb is not None
        and raised_loop_aabb is not None
        and raised_loop_aabb[1][2] > rest_loop_aabb[1][2] + 0.035,
        details=f"rest={rest_loop_aabb}, raised={raised_loop_aabb}",
    )

    rest_cup_aabb = ctx.part_world_aabb(right_cup)
    with ctx.pose({right_swivel: 0.50, left_swivel: -0.50}):
        swivel_cup_aabb = ctx.part_world_aabb(right_cup)
    if rest_cup_aabb is not None and swivel_cup_aabb is not None:
        rest_span_x = rest_cup_aabb[1][0] - rest_cup_aabb[0][0]
        swivel_span_x = swivel_cup_aabb[1][0] - swivel_cup_aabb[0][0]
    else:
        rest_span_x = swivel_span_x = 0.0
    ctx.check(
        "swivel pivots yaw the earcups",
        swivel_span_x > rest_span_x + 0.010,
        details=f"right cup x span {rest_span_x:.3f}->{swivel_span_x:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
