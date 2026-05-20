from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bi_fold_garden_gate")

    wood = model.material("weathered_wood", color=(0.34, 0.22, 0.12, 1.0))
    gate_green = model.material("painted_green", color=(0.08, 0.34, 0.15, 1.0))
    dark_metal = model.material("blackened_steel", color=(0.015, 0.017, 0.016, 1.0))
    cap_grey = model.material("galvanized_cap", color=(0.55, 0.58, 0.56, 1.0))

    post_size = 0.12
    post_depth = 0.14
    post_height = 1.45
    right_post_x = 1.43
    hinge_x = 0.10
    hinge_y = -0.09
    gate_bottom = 0.16
    panel_width = 0.60
    panel_height = 1.08
    panel_depth = 0.045
    panel_y = 0.07
    stile = 0.055
    rail = 0.070

    posts = model.part("posts")
    for name, x in (("hinge_post", 0.0), ("latch_post", right_post_x)):
        posts.visual(
            Box((post_size, post_depth, post_height)),
            origin=Origin(xyz=(x, 0.0, post_height / 2.0)),
            material=wood,
            name=name,
        )
        posts.visual(
            Box((post_size + 0.035, post_depth + 0.035, 0.030)),
            origin=Origin(xyz=(x, 0.0, post_height + 0.015)),
            material=cap_grey,
            name=f"{name}_cap",
        )

    posts.visual(
        Box((right_post_x + 0.22, 0.10, 0.07)),
        origin=Origin(xyz=(right_post_x / 2.0, 0.0, 0.035)),
        material=wood,
        name="ground_sill",
    )

    # Stationary latch keeper on the far post, placed on the gate-facing edge.
    latch_z = gate_bottom + 0.62
    posts.visual(
        Box((0.020, 0.080, 0.150)),
        origin=Origin(xyz=(right_post_x - post_size / 2.0 - 0.010, -0.070, latch_z)),
        material=dark_metal,
        name="keeper_plate",
    )
    for suffix, z in (("lower", latch_z - 0.055), ("upper", latch_z + 0.055)):
        posts.visual(
            Box((0.055, 0.018, 0.018)),
            origin=Origin(xyz=(right_post_x - post_size / 2.0 - 0.045, -0.085, z)),
            material=dark_metal,
            name=f"keeper_{suffix}_lug",
        )

    # Post-side hinge knuckles and straps for the first vertical hinge.
    hinge_knuckles = (
        ("lower", 0.24, 0.3432),
        ("upper", 0.86, 0.2968),
    )
    for suffix, z_offset, knuckle_length in hinge_knuckles:
        posts.visual(
            Cylinder(radius=0.016, length=knuckle_length),
            origin=Origin(xyz=(hinge_x, hinge_y, gate_bottom + z_offset)),
            material=dark_metal,
            name=f"post_hinge_{suffix}_barrel",
        )
        posts.visual(
            Box((0.070, 0.030, 0.105)),
            origin=Origin(xyz=(0.065, -0.080, gate_bottom + z_offset)),
            material=dark_metal,
            name=f"post_hinge_{suffix}_strap",
        )

    def add_panel(part, *, prefix: str, diagonal_sign: int = 1) -> None:
        part.visual(
            Box((stile, panel_depth, panel_height)),
            origin=Origin(xyz=(0.035, panel_y, panel_height / 2.0)),
            material=gate_green,
            name=f"{prefix}_hinge_stile",
        )
        part.visual(
            Box((stile, panel_depth, panel_height)),
            origin=Origin(xyz=(panel_width - 0.035, panel_y, panel_height / 2.0)),
            material=gate_green,
            name=f"{prefix}_free_stile",
        )
        part.visual(
            Box((panel_width, panel_depth, rail)),
            origin=Origin(xyz=(panel_width / 2.0, panel_y, rail / 2.0)),
            material=gate_green,
            name=f"{prefix}_bottom_rail",
        )
        part.visual(
            Box((panel_width, panel_depth, rail)),
            origin=Origin(xyz=(panel_width / 2.0, panel_y, panel_height - rail / 2.0)),
            material=gate_green,
            name=f"{prefix}_top_rail",
        )
        part.visual(
            Box((panel_width - 0.04, panel_depth * 0.82, 0.045)),
            origin=Origin(xyz=(panel_width / 2.0, panel_y, panel_height * 0.52)),
            material=gate_green,
            name=f"{prefix}_middle_rail",
        )
        for index, x in enumerate((0.18, 0.30, 0.42)):
            part.visual(
                Box((0.026, panel_depth * 0.70, panel_height - 2.0 * rail)),
                origin=Origin(xyz=(x, panel_y, panel_height / 2.0)),
                material=gate_green,
                name=f"{prefix}_picket_{index}",
            )

        brace_dx = 0.38
        brace_dz = 0.74
        brace_len = math.hypot(brace_dx, brace_dz)
        brace_angle = math.atan2(brace_dz, brace_dx)
        pitch = -brace_angle if diagonal_sign > 0 else brace_angle
        center_z = panel_height / 2.0
        part.visual(
            Box((brace_len, panel_depth * 0.72, 0.040)),
            origin=Origin(
                xyz=(panel_width / 2.0, panel_y - 0.002, center_z),
                rpy=(0.0, pitch, 0.0),
            ),
            material=gate_green,
            name=f"{prefix}_diagonal_brace",
        )

    outer_panel = model.part("outer_panel")
    add_panel(outer_panel, prefix="outer", diagonal_sign=1)

    # Moving side of the post hinge: a centered knuckle and a strap tied into
    # the hinge stile, sharing the child frame located on the hinge axis.
    outer_panel.visual(
        Cylinder(radius=0.016, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.52)),
        material=dark_metal,
        name="post_hinge_middle_barrel",
    )
    outer_panel.visual(
        Box((0.115, 0.050, 0.105)),
        origin=Origin(xyz=(0.055, 0.030, panel_height * 0.52)),
        material=dark_metal,
        name="post_hinge_middle_strap",
    )

    # Parent-side knuckles for the second hinge between the two gate leaves.
    for suffix, z, knuckle_length in hinge_knuckles:
        outer_panel.visual(
            Cylinder(radius=0.015, length=knuckle_length),
            origin=Origin(xyz=(panel_width, 0.0, z)),
            material=dark_metal,
            name=f"fold_hinge_{suffix}_barrel",
        )
        outer_panel.visual(
            Box((0.105, 0.050, 0.095)),
            origin=Origin(xyz=(panel_width - 0.052, 0.030, z)),
            material=dark_metal,
            name=f"fold_hinge_{suffix}_strap",
        )

    inner_panel = model.part("inner_panel")
    add_panel(inner_panel, prefix="inner", diagonal_sign=-1)
    inner_panel.visual(
        Cylinder(radius=0.015, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.52)),
        material=dark_metal,
        name="fold_hinge_middle_barrel",
    )
    inner_panel.visual(
        Box((0.115, 0.050, 0.095)),
        origin=Origin(xyz=(0.055, 0.030, panel_height * 0.52)),
        material=dark_metal,
        name="fold_hinge_middle_strap",
    )
    inner_panel.visual(
        Box((0.070, 0.022, 0.095)),
        origin=Origin(xyz=(panel_width - 0.060, 0.040, latch_z - gate_bottom)),
        material=dark_metal,
        name="latch_backplate",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    latch.visual(
        Box((0.205, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=dark_metal,
        name="latch_lever",
    )
    latch.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.112, -0.018, 0.0)),
        material=dark_metal,
        name="finger_knob",
    )

    model.articulation(
        "post_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=outer_panel,
        origin=Origin(xyz=(hinge_x, hinge_y, gate_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_panel,
        child=inner_panel,
        origin=Origin(xyz=(panel_width, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=2.55),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=inner_panel,
        child=latch,
        origin=Origin(xyz=(panel_width - 0.060, 0.030, latch_z - gate_bottom)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    outer_panel = object_model.get_part("outer_panel")
    inner_panel = object_model.get_part("inner_panel")
    latch = object_model.get_part("latch")
    post_hinge = object_model.get_articulation("post_hinge")
    fold_hinge = object_model.get_articulation("fold_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.check(
        "three revolute user mechanisms",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.expect_gap(
        outer_panel,
        posts,
        axis="x",
        positive_elem="outer_hinge_stile",
        negative_elem="hinge_post",
        min_gap=0.025,
        max_gap=0.070,
        name="outer panel clears hinge post",
    )
    ctx.expect_gap(
        inner_panel,
        outer_panel,
        axis="x",
        positive_elem="inner_hinge_stile",
        negative_elem="outer_free_stile",
        min_gap=0.005,
        max_gap=0.050,
        name="two leaves meet at narrow fold gap",
    )
    ctx.expect_overlap(
        latch,
        inner_panel,
        axes="z",
        elem_a="latch_lever",
        elem_b="latch_backplate",
        min_overlap=0.025,
        name="latch lever is mounted at backplate height",
    )

    rest_latch_pos = ctx.part_world_position(latch)
    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({fold_hinge: 1.20}):
        folded_latch_pos = ctx.part_world_position(latch)
    ctx.check(
        "inner leaf folds on outer leaf",
        rest_latch_pos is not None
        and folded_latch_pos is not None
        and folded_latch_pos[1] < rest_latch_pos[1] - 0.25,
        details=f"rest={rest_latch_pos}, folded={folded_latch_pos}",
    )

    rest_inner_pos = ctx.part_world_position(inner_panel)
    with ctx.pose({post_hinge: 1.00}):
        opened_inner_pos = ctx.part_world_position(inner_panel)
    ctx.check(
        "post hinge swings the pair off the post line",
        rest_inner_pos is not None
        and opened_inner_pos is not None
        and opened_inner_pos[1] > rest_inner_pos[1] + 0.45,
        details=f"rest={rest_inner_pos}, opened={opened_inner_pos}",
    )

    with ctx.pose({latch_pivot: 0.65}):
        lifted_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "latch lever rotates about its own pivot",
        rest_latch_aabb is not None
        and lifted_latch_aabb is not None
        and lifted_latch_aabb[1][2] > rest_latch_aabb[1][2] + 0.045,
        details=f"rest={rest_latch_aabb}, lifted={lifted_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
