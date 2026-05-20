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
    superellipse_profile,
    sweep_profile_along_spline,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _oval_solid(width_y: float, height_z: float, depth_x: float, name: str):
    """Extrude a rounded oval in the local YZ plane through local X."""
    profile = superellipse_profile(width_y, height_z, exponent=2.65, segments=64)
    geom = MeshGeometry()
    back = []
    front = []
    x0 = -depth_x / 2.0
    x1 = depth_x / 2.0
    for y, z in profile:
        back.append(geom.add_vertex(x0, y, z))
        front.append(geom.add_vertex(x1, y, z))
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back[i], back[j], front[j])
        geom.add_face(back[i], front[j], front[i])
    center_back = geom.add_vertex(x0, 0.0, 0.0)
    center_front = geom.add_vertex(x1, 0.0, 0.0)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(center_back, back[j], back[i])
        geom.add_face(center_front, front[i], front[j])
    return mesh_from_geometry(geom, name)


def _oval_ring(
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    depth_x: float,
    name: str,
):
    """A soft oval earpad ring with a real center opening."""
    outer = superellipse_profile(outer_y, outer_z, exponent=2.45, segments=64)
    inner = superellipse_profile(inner_y, inner_z, exponent=2.35, segments=64)
    geom = MeshGeometry()
    x0 = -depth_x / 2.0
    x1 = depth_x / 2.0
    ob, of, ib, inf = [], [], [], []
    for (oy, oz), (iy, iz) in zip(outer, inner):
        ob.append(geom.add_vertex(x0, oy, oz))
        of.append(geom.add_vertex(x1, oy, oz))
        ib.append(geom.add_vertex(x0, iy, iz))
        inf.append(geom.add_vertex(x1, iy, iz))
    n = len(outer)
    for i in range(n):
        j = (i + 1) % n
        # Outside and inside walls.
        geom.add_face(ob[i], ob[j], of[j])
        geom.add_face(ob[i], of[j], of[i])
        geom.add_face(ib[j], ib[i], inf[i])
        geom.add_face(ib[j], inf[i], inf[j])
        # Front and back annular faces.
        geom.add_face(of[i], of[j], inf[j])
        geom.add_face(of[i], inf[j], inf[i])
        geom.add_face(ob[j], ob[i], ib[i])
        geom.add_face(ob[j], ib[i], ib[j])
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_gaming_headset")

    matte_black = Material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    soft_black = Material("soft_black", rgba=(0.005, 0.005, 0.006, 1.0))
    cushion = Material("cushion", rgba=(0.075, 0.075, 0.080, 1.0))
    gunmetal = Material("gunmetal", rgba=(0.23, 0.23, 0.25, 1.0))
    red_accent = Material("red_accent", rgba=(0.85, 0.04, 0.035, 1.0))
    mesh_cloth = Material("mesh_cloth", rgba=(0.025, 0.026, 0.030, 1.0))
    metal = Material("dark_pin_metal", rgba=(0.48, 0.48, 0.50, 1.0))
    for mat in (matte_black, soft_black, cushion, gunmetal, red_accent, mesh_cloth, metal):
        model.materials.append(mat)

    cyl_y = (-math.pi / 2.0, 0.0, 0.0)
    cyl_x = (0.0, math.pi / 2.0, 0.0)

    headband = model.part("headband")
    outer_band = sweep_profile_along_spline(
        [
            (-0.125, 0.0, 0.160),
            (-0.100, 0.0, 0.245),
            (0.000, 0.0, 0.305),
            (0.100, 0.0, 0.245),
            (0.125, 0.0, 0.160),
        ],
        profile=rounded_rect_profile(0.040, 0.012, radius=0.004, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(
        mesh_from_geometry(outer_band, "outer_band"),
        material=matte_black,
        name="outer_band",
    )
    pad_band = sweep_profile_along_spline(
        [
            (-0.102, 0.0, 0.178),
            (-0.070, 0.0, 0.235),
            (0.000, 0.0, 0.262),
            (0.070, 0.0, 0.235),
            (0.102, 0.0, 0.178),
        ],
        profile=rounded_rect_profile(0.052, 0.018, radius=0.007, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(
        mesh_from_geometry(pad_band, "head_pad"),
        material=cushion,
        name="head_pad",
    )

    hinge_x = (-0.124, 0.124)
    for i, x in enumerate(hinge_x):
        # A visible side hinge above each yoke: top bridge, side cheeks, and a steel pin.
        headband.visual(
            Box((0.028, 0.105, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.184)),
            material=matte_black,
            name=f"hinge_bridge_{i}",
        )
        headband.visual(
            Box((0.026, 0.022, 0.034)),
            origin=Origin(xyz=(x, 0.0, 0.170)),
            material=matte_black,
            name=f"hinge_stem_{i}",
        )
        for y in (-0.043, 0.043):
            headband.visual(
                Box((0.026, 0.010, 0.030)),
                origin=Origin(xyz=(x, y, 0.166)),
                material=matte_black,
                name=f"hinge_cheek_{i}_{'front' if y < 0 else 'rear'}",
            )
        headband.visual(
            Cylinder(radius=0.0042, length=0.096),
            origin=Origin(xyz=(x, 0.0, 0.160), rpy=cyl_y),
            material=metal,
            name=f"fold_pin_{i}",
        )

    def make_yoke(index: int, parent_x: float, fold_axis):
        yoke = model.part(f"yoke_{index}")
        # Interleaved hinge knuckles wrapped around the headband pin.
        for y, tag in ((-0.024, "front"), (0.024, "rear")):
            yoke.visual(
                Cylinder(radius=0.0105, length=0.019),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_y),
                material=gunmetal,
                name=f"fold_knuckle_{tag}",
            )
        yoke.visual(
            Box((0.020, 0.114, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.019)),
            material=matte_black,
            name="top_bridge",
        )
        for y, tag in ((-0.056, "front"), (0.056, "rear")):
            yoke.visual(
                Box((0.014, 0.011, 0.122)),
                origin=Origin(xyz=(0.0, y, -0.079)),
                material=matte_black,
                name=f"side_arm_{tag}",
            )
            yoke.visual(
                Cylinder(radius=0.0135, length=0.014),
                origin=Origin(xyz=(0.0, y, -0.102), rpy=cyl_y),
                material=gunmetal,
                name=f"swivel_bearing_{tag}",
            )
        model.articulation(
            f"fold_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=headband,
            child=yoke,
            origin=Origin(xyz=(parent_x, 0.0, 0.160)),
            axis=fold_axis,
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.55),
        )
        return yoke

    yoke_0 = make_yoke(0, -0.124, (0.0, -1.0, 0.0))
    yoke_1 = make_yoke(1, 0.124, (0.0, 1.0, 0.0))

    shell_mesh = _oval_solid(0.096, 0.124, 0.044, "cup_shell_mesh")
    pad_mesh = _oval_ring(0.092, 0.112, 0.052, 0.066, 0.018, "earpad_ring_mesh")
    grille_mesh = _oval_solid(0.047, 0.060, 0.004, "speaker_grille_mesh")
    accent_mesh = _oval_ring(0.074, 0.095, 0.052, 0.068, 0.006, "outer_accent_ring_mesh")

    def make_cup(index: int, yoke, pad_x: float, accent_x: float):
        cup = model.part(f"cup_{index}")
        cup.visual(
            shell_mesh,
            material=matte_black,
            name="cup_shell",
        )
        cup.visual(
            pad_mesh,
            origin=Origin(xyz=(pad_x, 0.0, 0.0)),
            material=soft_black,
            name="ear_pad",
        )
        cup.visual(
            grille_mesh,
            origin=Origin(xyz=(pad_x * 0.76, 0.0, 0.0)),
            material=mesh_cloth,
            name="speaker_grille",
        )
        cup.visual(
            accent_mesh,
            origin=Origin(xyz=(accent_x, 0.0, 0.0)),
            material=red_accent,
            name="accent_ring",
        )
        for y, tag in ((-0.057, "front"), (0.057, "rear")):
            cup.visual(
                Cylinder(radius=0.0095, length=0.024),
                origin=Origin(xyz=(0.0, y, 0.000), rpy=cyl_y),
                material=metal,
                name=f"swivel_pin_{tag}",
            )
        model.articulation(
            f"cup_swivel_{index}",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -0.102)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.45, upper=0.45),
        )
        return cup

    cup_0 = make_cup(0, yoke_0, pad_x=0.029, accent_x=-0.023)
    cup_1 = make_cup(1, yoke_1, pad_x=-0.029, accent_x=0.023)

    # A fixed post on the microphone-side cup shell for the boom's own pivot.
    cup_0.visual(
        Cylinder(radius=0.0105, length=0.017),
        origin=Origin(xyz=(-0.032, -0.050, -0.004), rpy=cyl_x),
        material=gunmetal,
        name="boom_socket",
    )
    cup_0.visual(
        Box((0.008, 0.020, 0.016)),
        origin=Origin(xyz=(-0.024, -0.044, -0.004)),
        material=gunmetal,
        name="socket_tab",
    )

    boom = model.part("boom_microphone")
    boom.visual(
        Cylinder(radius=0.0155, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=cyl_x),
        material=gunmetal,
        name="pivot_collar",
    )
    boom_path = tube_from_spline_points(
        [
            (-0.012, -0.003, -0.001),
            (-0.015, -0.030, -0.010),
            (-0.014, -0.083, -0.039),
            (-0.011, -0.126, -0.035),
        ],
        radius=0.0032,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    boom.visual(
        mesh_from_geometry(boom_path, "boom_arm"),
        material=matte_black,
        name="boom_arm",
    )
    boom.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(-0.011, -0.139, -0.034), rpy=cyl_y),
        material=soft_black,
        name="mic_capsule",
    )
    boom.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.011, -0.154, -0.034)),
        material=soft_black,
        name="mic_end_cap",
    )
    model.articulation(
        "boom_pivot",
        ArticulationType.REVOLUTE,
        parent=cup_0,
        child=boom,
        origin=Origin(xyz=(-0.032, -0.050, -0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=1.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    yoke_0 = object_model.get_part("yoke_0")
    yoke_1 = object_model.get_part("yoke_1")
    cup_0 = object_model.get_part("cup_0")
    cup_1 = object_model.get_part("cup_1")
    boom = object_model.get_part("boom_microphone")
    fold_0 = object_model.get_articulation("fold_hinge_0")
    fold_1 = object_model.get_articulation("fold_hinge_1")
    swivel_0 = object_model.get_articulation("cup_swivel_0")
    swivel_1 = object_model.get_articulation("cup_swivel_1")
    boom_pivot = object_model.get_articulation("boom_pivot")

    # Captured pins and collars are intentionally nested through barrels rather
    # than left as visually floating hinge decoration.
    for idx, yoke in ((0, yoke_0), (1, yoke_1)):
        for tag in ("front", "rear"):
            ctx.allow_overlap(
                headband,
                yoke,
                elem_a=f"fold_pin_{idx}",
                elem_b=f"fold_knuckle_{tag}",
                reason="The folding hinge pin is intentionally captured inside the yoke knuckle.",
            )
            ctx.expect_overlap(
                headband,
                yoke,
                axes="y",
                elem_a=f"fold_pin_{idx}",
                elem_b=f"fold_knuckle_{tag}",
                min_overlap=0.012,
                name=f"fold pin {idx} retained in {tag} knuckle",
            )

    for cup, yoke, idx in ((cup_0, yoke_0, 0), (cup_1, yoke_1, 1)):
        for tag in ("front", "rear"):
            ctx.allow_overlap(
                yoke,
                cup,
                elem_a=f"swivel_bearing_{tag}",
                elem_b=f"swivel_pin_{tag}",
                reason="The cup swivel pin is intentionally captured in the yoke bearing.",
            )
            ctx.expect_overlap(
                yoke,
                cup,
                axes="y",
                elem_a=f"swivel_bearing_{tag}",
                elem_b=f"swivel_pin_{tag}",
                min_overlap=0.008,
                name=f"cup {idx} {tag} trunnion retained",
            )
            ctx.allow_overlap(
                yoke,
                cup,
                elem_a=f"side_arm_{tag}",
                elem_b=f"swivel_pin_{tag}",
                reason="The cup swivel pin passes through the yoke arm bore represented by the solid arm.",
            )
            ctx.expect_overlap(
                yoke,
                cup,
                axes="y",
                elem_a=f"side_arm_{tag}",
                elem_b=f"swivel_pin_{tag}",
                min_overlap=0.004,
                name=f"cup {idx} {tag} pin passes through yoke arm",
            )

    ctx.allow_overlap(
        cup_0,
        boom,
        elem_a="boom_socket",
        elem_b="pivot_collar",
        reason="The boom collar clips around the cup-mounted pivot socket.",
    )
    ctx.expect_overlap(
        cup_0,
        boom,
        axes="x",
        elem_a="boom_socket",
        elem_b="pivot_collar",
        min_overlap=0.006,
        name="boom collar clipped on cup socket",
    )

    left_rest = ctx.part_world_position(cup_0)
    right_rest = ctx.part_world_position(cup_1)
    with ctx.pose({fold_0: 1.20, fold_1: 1.20}):
        left_folded = ctx.part_world_position(cup_0)
        right_folded = ctx.part_world_position(cup_1)
    ctx.check(
        "folding hinges swing cups inward and upward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.050
        and right_folded[0] < right_rest[0] - 0.050
        and left_folded[2] > left_rest[2] + 0.035
        and right_folded[2] > right_rest[2] + 0.035,
        details=f"left rest/folded={left_rest}/{left_folded}, right rest/folded={right_rest}/{right_folded}",
    )

    cup0_shell_rest = ctx.part_element_world_aabb(cup_0, elem="cup_shell")
    cup1_shell_rest = ctx.part_element_world_aabb(cup_1, elem="cup_shell")
    with ctx.pose({swivel_0: 0.35, swivel_1: -0.35}):
        cup0_shell_swivel = ctx.part_element_world_aabb(cup_0, elem="cup_shell")
        cup1_shell_swivel = ctx.part_element_world_aabb(cup_1, elem="cup_shell")
    ctx.check(
        "earcups visibly swivel inside yokes",
        cup0_shell_rest is not None
        and cup1_shell_rest is not None
        and cup0_shell_swivel is not None
        and cup1_shell_swivel is not None
        and (cup0_shell_swivel[1][0] - cup0_shell_swivel[0][0])
        > (cup0_shell_rest[1][0] - cup0_shell_rest[0][0]) + 0.025
        and (cup1_shell_swivel[1][0] - cup1_shell_swivel[0][0])
        > (cup1_shell_rest[1][0] - cup1_shell_rest[0][0]) + 0.025,
        details=f"shell rest/swivel aabbs={cup0_shell_rest}/{cup0_shell_swivel}, {cup1_shell_rest}/{cup1_shell_swivel}",
    )

    boom_rest = ctx.part_world_aabb(boom)
    with ctx.pose({boom_pivot: 1.55}):
        boom_up = ctx.part_world_aabb(boom)
    ctx.check(
        "boom microphone flips upward",
        boom_rest is not None
        and boom_up is not None
        and boom_up[1][2] > boom_rest[1][2] + 0.055,
        details=f"boom rest/up aabb={boom_rest}/{boom_up}",
    )

    return ctx.report()


object_model = build_object_model()
