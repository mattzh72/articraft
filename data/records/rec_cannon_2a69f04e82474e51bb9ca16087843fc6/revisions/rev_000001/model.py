from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder(radius: float, length: float, center_x: float = 0.0) -> cq.Workplane:
    """CadQuery cylinder whose axis is the model X axis."""

    return cq.Workplane("YZ").cylinder(length, radius).translate((center_x, 0.0, 0.0))


def _bombard_barrel_mesh() -> cq.Workplane:
    """Short, wide, hollow cast-iron bombard barrel with stepped reinforces."""

    barrel = _x_cylinder(0.235, 1.05, 0.02)
    barrel = barrel.union(_x_cylinder(0.300, 0.18, 0.52))  # flared muzzle swell
    barrel = barrel.union(_x_cylinder(0.280, 0.24, -0.46))  # heavy closed breech
    barrel = barrel.union(_x_cylinder(0.258, 0.16, -0.06))  # trunnion reinforce band
    barrel = barrel.union(_x_cylinder(0.252, 0.055, 0.29))
    barrel = barrel.union(_x_cylinder(0.252, 0.050, -0.25))

    # Blind bore: open at the muzzle, terminating in a dark recessed chamber wall.
    bore = _x_cylinder(0.150, 1.16, 0.14)
    return barrel.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_bombard")

    wood = model.material("aged_oak", rgba=(0.42, 0.25, 0.12, 1.0))
    wood_dark = model.material("dark_endgrain", rgba=(0.25, 0.13, 0.055, 1.0))
    iron = model.material("blackened_cast_iron", rgba=(0.025, 0.028, 0.030, 1.0))
    iron_edge = model.material("worn_iron_edges", rgba=(0.12, 0.115, 0.105, 1.0))
    bore_black = model.material("sooty_bore", rgba=(0.002, 0.002, 0.002, 1.0))

    cradle = model.part("cradle")
    cradle.visual(
        Box((1.70, 0.74, 0.10)),
        origin=Origin(xyz=(-0.06, 0.0, 0.05)),
        material=wood,
        name="bed_plank",
    )
    cradle.visual(
        Box((1.82, 0.13, 0.16)),
        origin=Origin(xyz=(-0.04, 0.255, 0.095)),
        material=wood_dark,
        name="side_runner_0",
    )
    cradle.visual(
        Box((1.82, 0.13, 0.16)),
        origin=Origin(xyz=(-0.04, -0.255, 0.095)),
        material=wood_dark,
        name="side_runner_1",
    )
    for i, x in enumerate((-0.62, 0.48)):
        cradle.visual(
            Box((0.18, 0.82, 0.11)),
            origin=Origin(xyz=(x, 0.0, 0.110)),
            material=wood,
            name=f"cross_tie_{i}",
        )
    cradle.visual(
        Box((0.62, 0.24, 0.070)),
        origin=Origin(xyz=(0.11, 0.0, 0.130)),
        material=wood,
        name="barrel_saddle",
    )

    for idx, y in enumerate((0.345, -0.345)):
        cradle.visual(
            Box((0.66, 0.105, 0.24)),
            origin=Origin(xyz=(-0.05, y, 0.235)),
            material=wood,
            name=f"cheek_bed_{idx}",
        )
        cradle.visual(
            Box((0.13, 0.105, 0.52)),
            origin=Origin(xyz=(-0.325, y, 0.37)),
            material=wood,
            name=f"rear_cheek_{idx}",
        )
        cradle.visual(
            Box((0.13, 0.105, 0.52)),
            origin=Origin(xyz=(0.255, y, 0.37)),
            material=wood,
            name=f"front_cheek_{idx}",
        )
        cradle.visual(
            Box((0.68, 0.105, 0.09)),
            origin=Origin(xyz=(-0.035, y, 0.61)),
            material=wood_dark,
            name=f"cheek_cap_{idx}",
        )

        collar_y = 0.286 if y > 0 else -0.286
        cradle.visual(
            Cylinder(radius=0.128, length=0.046),
            origin=Origin(xyz=(0.0, collar_y, 0.48), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=iron_edge,
            name=f"pivot_collar_{idx}",
        )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_bombard_barrel_mesh(), "bombard_barrel"),
        material=iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.075, length=0.82),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron_edge,
        name="trunnion_bar",
    )
    barrel.visual(
        Cylinder(radius=0.154, length=0.016),
        origin=Origin(xyz=(-0.432, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bore_black,
        name="bore_shadow",
    )
    barrel.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.340, 0.0, 0.282)),
        material=bore_black,
        name="touch_hole",
    )
    for idx, y in enumerate((0.106, -0.106)):
        barrel.visual(
            Box((0.036, 0.028, 0.068)),
            origin=Origin(xyz=(-0.430, y, 0.256)),
            material=iron_edge,
            name=f"cover_hinge_foot_{idx}",
        )
        barrel.visual(
            Box((0.050, 0.036, 0.052)),
            origin=Origin(xyz=(-0.430, y, 0.302)),
            material=iron_edge,
            name=f"cover_hinge_ear_{idx}",
        )

    model.articulation(
        "cradle_to_barrel",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.45),
    )

    cover = model.part("touch_cover")
    cover.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron_edge,
        name="hinge_knuckle",
    )
    cover.visual(
        Box((0.215, 0.158, 0.014)),
        origin=Origin(xyz=(0.108, 0.0, -0.010)),
        material=iron_edge,
        name="cover_plate",
    )
    cover.visual(
        Box((0.040, 0.082, 0.024)),
        origin=Origin(xyz=(0.188, 0.0, 0.007)),
        material=iron_edge,
        name="lift_tab",
    )

    model.articulation(
        "barrel_to_touch_cover",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=cover,
        origin=Origin(xyz=(-0.430, 0.0, 0.302)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    cradle = object_model.get_part("cradle")
    barrel = object_model.get_part("barrel")
    cover = object_model.get_part("touch_cover")
    elev = object_model.get_articulation("cradle_to_barrel")
    cover_hinge = object_model.get_articulation("barrel_to_touch_cover")

    for collar in ("pivot_collar_0", "pivot_collar_1"):
        ctx.allow_overlap(
            cradle,
            barrel,
            elem_a=collar,
            elem_b="trunnion_bar",
            reason="The trunnion is intentionally captured inside the iron pivot collar at the cheek bracket.",
        )
        ctx.expect_within(
            barrel,
            cradle,
            axes="xz",
            inner_elem="trunnion_bar",
            outer_elem=collar,
            margin=0.0,
            name=f"{collar} surrounds trunnion section",
        )
        ctx.expect_overlap(
            barrel,
            cradle,
            axes="y",
            elem_a="trunnion_bar",
            elem_b=collar,
            min_overlap=0.030,
            name=f"{collar} retains trunnion across cheek",
        )

    ctx.expect_gap(
        cover,
        barrel,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="touch_hole",
        min_gap=0.0,
        max_gap=0.010,
        name="cover rests just above touch hole",
    )
    ctx.expect_overlap(
        cover,
        barrel,
        axes="xy",
        elem_a="cover_plate",
        elem_b="touch_hole",
        min_overlap=0.010,
        name="cover footprint shields touch hole",
    )

    rest_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    with ctx.pose({elev: 0.35}):
        raised_barrel_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    ctx.check(
        "elevation joint raises muzzle end",
        rest_barrel_aabb is not None
        and raised_barrel_aabb is not None
        and raised_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.06,
        details=f"rest={rest_barrel_aabb}, raised={raised_barrel_aabb}",
    )

    rest_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    with ctx.pose({cover_hinge: 0.95}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    ctx.check(
        "touch-hole cover hinges upward",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.07,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
