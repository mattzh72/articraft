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


ALUMINUM = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
WHITE_ENAMEL = Material("white_enamel", rgba=(0.92, 0.94, 0.93, 1.0))
WARM_WHITE = Material("warm_white_insulation", rgba=(0.82, 0.84, 0.80, 1.0))
LINER = Material("pale_freezer_liner", rgba=(0.74, 0.88, 0.94, 1.0))
GLASS = Material("slightly_blue_glass", rgba=(0.62, 0.86, 0.96, 0.38))
DARK_RUBBER = Material("dark_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
DARK_GRILLE = Material("black_grille_shadow", rgba=(0.01, 0.012, 0.014, 1.0))


def _box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_x(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_lid(model: ArticulatedObject, tub, index: int, center_x: float, handle_x: float):
    lid = model.part(f"lid_{index}")

    # The frame, inset glass, under-side slide shoes, and handle brackets are all
    # one manufactured lid assembly in the child frame.
    _box(lid, "glass_panel", (1.02, 0.96, 0.024), (0.0, 0.0, 0.006), GLASS)
    _box(lid, "front_frame", (1.10, 0.060, 0.045), (0.0, -0.505, 0.0), ALUMINUM)
    _box(lid, "rear_frame", (1.10, 0.060, 0.045), (0.0, 0.505, 0.0), ALUMINUM)
    _box(lid, "outer_frame", (0.050, 1.03, 0.045), (math.copysign(0.525, handle_x), 0.0, 0.0), ALUMINUM)
    _box(lid, "inner_frame", (0.050, 1.03, 0.045), (-math.copysign(0.525, handle_x), 0.0, 0.0), ALUMINUM)
    _box(lid, "outer_runner", (0.055, 1.02, 0.025), (math.copysign(0.525, handle_x), 0.0, -0.035), ALUMINUM)
    _box(lid, "inner_runner", (0.055, 1.02, 0.025), (-math.copysign(0.525, handle_x), 0.0, -0.035), ALUMINUM)

    _box(lid, "handle_mount_0", (0.030, 0.037, 0.040), (handle_x - 0.055, -0.552, -0.001), ALUMINUM)
    _box(lid, "handle_mount_1", (0.030, 0.037, 0.040), (handle_x + 0.055, -0.552, -0.001), ALUMINUM)

    model.articulation(
        f"tub_to_lid_{index}",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lid,
        origin=Origin(xyz=(center_x, 0.0, 0.8925)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.30),
    )

    handle = model.part(f"handle_{index}")
    _cyl_x(handle, "pivot_0", 0.014, 0.030, (-0.055, 0.0, 0.0), ALUMINUM)
    _cyl_x(handle, "pivot_1", 0.014, 0.030, (0.055, 0.0, 0.0), ALUMINUM)
    _box(handle, "arm_0", (0.018, 0.170, 0.018), (-0.055, -0.085, -0.005), DARK_RUBBER)
    _box(handle, "arm_1", (0.018, 0.170, 0.018), (0.055, -0.085, -0.005), DARK_RUBBER)
    _box(handle, "grip", (0.175, 0.045, 0.032), (0.0, -0.180, -0.006), DARK_RUBBER)

    model.articulation(
        f"lid_{index}_to_handle_{index}",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(handle_x, -0.583, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return lid, handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_freezer_island")

    tub = model.part("tub")

    # Insulated open-topped freezer tub with a visible hollow liner.
    _box(tub, "base_plinth", (2.60, 1.40, 0.16), (0.0, 0.0, 0.08), DARK_RUBBER)
    _box(tub, "tub_floor", (2.34, 1.14, 0.10), (0.0, 0.0, 0.19), WARM_WHITE)
    _box(tub, "liner_floor", (2.24, 1.04, 0.035), (0.0, 0.0, 0.255), LINER)
    _box(tub, "front_wall", (2.60, 0.12, 0.64), (0.0, -0.66, 0.46), WHITE_ENAMEL)
    _box(tub, "rear_wall", (2.60, 0.12, 0.64), (0.0, 0.66, 0.46), WHITE_ENAMEL)
    _box(tub, "end_wall_0", (0.12, 1.40, 0.64), (-1.30, 0.0, 0.46), WHITE_ENAMEL)
    _box(tub, "end_wall_1", (0.12, 1.40, 0.64), (1.30, 0.0, 0.46), WHITE_ENAMEL)

    # Rounded-looking top cap represented with separate connected rim members
    # and four straight slide rails. The centre rail separates the two glass
    # halves and gives each lid its own track pair.
    _box(tub, "front_rim", (2.60, 0.12, 0.040), (0.0, -0.66, 0.795), ALUMINUM)
    _box(tub, "rear_rim", (2.60, 0.12, 0.040), (0.0, 0.66, 0.795), ALUMINUM)
    _box(tub, "end_rim_0", (0.12, 1.40, 0.040), (-1.30, 0.0, 0.795), ALUMINUM)
    _box(tub, "end_rim_1", (0.12, 1.40, 0.040), (1.30, 0.0, 0.795), ALUMINUM)
    _box(tub, "center_divider", (0.090, 1.30, 0.055), (0.0, 0.0, 0.805), ALUMINUM)

    for name, x in (
        ("track_0_outer", -1.095),
        ("track_0_inner", -0.045),
        ("track_1_inner", 0.045),
        ("track_1_outer", 1.095),
    ):
        _box(tub, name, (0.070, 1.20, 0.040), (x, 0.0, 0.825), ALUMINUM)

    # A shallow front service grille helps the island read as a supermarket
    # refrigerated fixture rather than a plain box.
    _box(tub, "front_grille_panel", (0.72, 0.018, 0.18), (0.0, -0.724, 0.31), DARK_GRILLE)
    for i, x in enumerate((-0.27, -0.18, -0.09, 0.0, 0.09, 0.18, 0.27)):
        _box(tub, f"grille_louver_{i}", (0.045, 0.026, 0.14), (x, -0.738, 0.31), WHITE_ENAMEL)

    _add_lid(model, tub, 0, center_x=-0.570, handle_x=-0.455)
    _add_lid(model, tub, 1, center_x=0.570, handle_x=0.455)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tub = object_model.get_part("tub")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    handle_0 = object_model.get_part("handle_0")
    handle_1 = object_model.get_part("handle_1")
    slide_0 = object_model.get_articulation("tub_to_lid_0")
    slide_1 = object_model.get_articulation("tub_to_lid_1")
    fold_0 = object_model.get_articulation("lid_0_to_handle_0")
    fold_1 = object_model.get_articulation("lid_1_to_handle_1")

    for handle, lid, pivot, mount in (
        (handle_0, lid_0, "pivot_0", "handle_mount_0"),
        (handle_0, lid_0, "pivot_1", "handle_mount_1"),
        (handle_1, lid_1, "pivot_0", "handle_mount_0"),
        (handle_1, lid_1, "pivot_1", "handle_mount_1"),
    ):
        ctx.allow_overlap(
            handle,
            lid,
            elem_a=pivot,
            elem_b=mount,
            reason="The round folding-handle pivot is intentionally captured inside the small lid bracket.",
        )

    ctx.expect_gap(
        lid_0,
        tub,
        axis="z",
        positive_elem="outer_runner",
        negative_elem="track_0_outer",
        min_gap=0.0,
        max_gap=0.004,
        name="lid_0 shoe rides just above outer rail",
    )
    ctx.expect_gap(
        lid_1,
        tub,
        axis="z",
        positive_elem="outer_runner",
        negative_elem="track_1_outer",
        min_gap=0.0,
        max_gap=0.004,
        name="lid_1 shoe rides just above outer rail",
    )
    ctx.expect_gap(
        lid_1,
        lid_0,
        axis="x",
        min_gap=0.015,
        max_gap=0.070,
        name="closed glass lids have a centre clearance",
    )
    ctx.expect_overlap(
        lid_0,
        tub,
        axes="xy",
        elem_a="glass_panel",
        elem_b="liner_floor",
        min_overlap=0.50,
        name="lid_0 covers its freezer half",
    )
    ctx.expect_overlap(
        lid_1,
        tub,
        axes="xy",
        elem_a="glass_panel",
        elem_b="liner_floor",
        min_overlap=0.50,
        name="lid_1 covers its freezer half",
    )
    ctx.expect_contact(
        handle_0,
        lid_0,
        elem_a="pivot_0",
        elem_b="handle_mount_0",
        contact_tol=0.003,
        name="handle_0 pivot is seated in bracket",
    )
    ctx.expect_contact(
        handle_1,
        lid_1,
        elem_a="pivot_1",
        elem_b="handle_mount_1",
        contact_tol=0.003,
        name="handle_1 pivot is seated in bracket",
    )

    rest_lid_0 = ctx.part_world_position(lid_0)
    rest_lid_1 = ctx.part_world_position(lid_1)
    rest_grip_0 = ctx.part_element_world_aabb(handle_0, elem="grip")
    rest_grip_1 = ctx.part_element_world_aabb(handle_1, elem="grip")

    with ctx.pose({slide_0: 0.30, slide_1: 0.30, fold_0: 1.15, fold_1: 1.15}):
        ctx.expect_overlap(
            lid_0,
            tub,
            axes="y",
            elem_a="inner_runner",
            elem_b="track_0_inner",
            min_overlap=0.60,
            name="lid_0 remains captured on centre rail while slid",
        )
        ctx.expect_overlap(
            lid_1,
            tub,
            axes="y",
            elem_a="inner_runner",
            elem_b="track_1_inner",
            min_overlap=0.60,
            name="lid_1 remains captured on centre rail while slid",
        )
        moved_lid_0 = ctx.part_world_position(lid_0)
        moved_lid_1 = ctx.part_world_position(lid_1)
        raised_grip_0 = ctx.part_element_world_aabb(handle_0, elem="grip")
        raised_grip_1 = ctx.part_element_world_aabb(handle_1, elem="grip")

    ctx.check(
        "lid_0 slides rearward on rails",
        rest_lid_0 is not None
        and moved_lid_0 is not None
        and moved_lid_0[1] > rest_lid_0[1] + 0.25,
        details=f"rest={rest_lid_0}, moved={moved_lid_0}",
    )
    ctx.check(
        "lid_1 slides rearward on rails",
        rest_lid_1 is not None
        and moved_lid_1 is not None
        and moved_lid_1[1] > rest_lid_1[1] + 0.25,
        details=f"rest={rest_lid_1}, moved={moved_lid_1}",
    )
    ctx.check(
        "handle_0 folds upward",
        rest_grip_0 is not None
        and raised_grip_0 is not None
        and raised_grip_0[1][2] > rest_grip_0[1][2] + 0.12,
        details=f"rest={rest_grip_0}, raised={raised_grip_0}",
    )
    ctx.check(
        "handle_1 folds upward",
        rest_grip_1 is not None
        and raised_grip_1 is not None
        and raised_grip_1[1][2] > rest_grip_1[1][2] + 0.12,
        details=f"rest={rest_grip_1}, raised={raised_grip_1}",
    )

    return ctx.report()


object_model = build_object_model()
