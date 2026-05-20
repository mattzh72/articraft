from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PIVOT_X = 0.004
PIVOT_Y = 0.0050
PIVOT_Z = 0.0120
SWIVEL_UPPER = math.pi


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 32,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _rounded_box_mesh(
    *,
    length: float,
    width: float,
    height: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=8),
            height,
            center=True,
        ),
        name,
    )


def _swivel_cover_mesh():
    # A thin stamped stainless plate with a real through-hole around the rivet.
    # Local +X is the covered-connector direction at q=0; after a 180 degree
    # turn it lies flat above the drive body for stowage.
    outer = _shift_profile(
        rounded_rect_profile(0.031, 0.021, 0.0038, corner_segments=10),
        0.0115,
        -0.0040,
    )
    pivot_hole = _circle_profile(0.0026, segments=36)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [pivot_hole], 0.0015, center=True),
        "swivel_cover_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_usb_swivel_drive")

    plastic = model.material("matte_graphite_plastic", rgba=(0.05, 0.055, 0.065, 1.0))
    rubber = model.material("soft_black_insert", rgba=(0.01, 0.012, 0.014, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    darker_steel = model.material("shadowed_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    gold = model.material("usb_gold_contacts", rgba=(1.0, 0.72, 0.18, 1.0))
    blue = model.material("tiny_blue_led", rgba=(0.05, 0.32, 0.85, 1.0))
    label = model.material("subtle_label_grey", rgba=(0.18, 0.19, 0.21, 1.0))

    body = model.part("drive_body")
    body.visual(
        _rounded_box_mesh(
            length=0.038,
            width=0.018,
            height=0.008,
            radius=0.004,
            name="usb_body_shell",
        ),
        origin=Origin(xyz=(-0.010, 0.0, 0.004)),
        material=plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.018, 0.010, 0.00035)),
        origin=Origin(xyz=(-0.011, -0.0005, 0.00812)),
        material=label,
        name="recessed_label",
    )
    body.visual(
        Box((0.0030, 0.0016, 0.00045)),
        origin=Origin(xyz=(0.001, -0.0064, 0.00818)),
        material=blue,
        name="status_led",
    )
    body.visual(
        Box((0.0020, 0.0128, 0.0062)),
        origin=Origin(xyz=(0.0087, 0.0, 0.0040)),
        material=darker_steel,
        name="connector_collar",
    )

    # USB-A shell: separate top/bottom/side walls make the open front legible.
    body.visual(
        Box((0.0148, 0.0120, 0.00065)),
        origin=Origin(xyz=(0.0160, 0.0, 0.00618)),
        material=stainless,
        name="connector_top",
    )
    body.visual(
        Box((0.0148, 0.0120, 0.00065)),
        origin=Origin(xyz=(0.0160, 0.0, 0.00182)),
        material=stainless,
        name="connector_bottom",
    )
    body.visual(
        Box((0.0148, 0.00070, 0.0049)),
        origin=Origin(xyz=(0.0160, 0.00565, 0.0040)),
        material=stainless,
        name="connector_side_0",
    )
    body.visual(
        Box((0.0148, 0.00070, 0.0049)),
        origin=Origin(xyz=(0.0160, -0.00565, 0.0040)),
        material=stainless,
        name="connector_side_1",
    )
    body.visual(
        Box((0.0020, 0.0074, 0.0025)),
        origin=Origin(xyz=(0.0093, 0.0, 0.00355)),
        material=rubber,
        name="tongue_root",
    )
    body.visual(
        Box((0.0120, 0.0074, 0.00095)),
        origin=Origin(xyz=(0.0163, 0.0, 0.00375)),
        material=rubber,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        body.visual(
            Box((0.0054, 0.00105, 0.00014)),
            origin=Origin(xyz=(0.0178, y, 0.00426)),
            material=gold,
            name=f"contact_{index}",
        )

    # The visible rivet/post is part of the fixed drive body.  The cover has a
    # clearance hole around it, and the head sits just above the cover plate.
    body.visual(
        Cylinder(radius=0.0016, length=0.0098),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0089)),
        material=darker_steel,
        name="pivot_shaft",
    )
    body.visual(
        Cylinder(radius=0.0037, length=0.0012),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0136)),
        material=stainless,
        name="pivot_head",
    )

    cover = model.part("swivel_cover")
    cover.visual(
        _swivel_cover_mesh(),
        origin=Origin(),
        material=stainless,
        name="cover_plate",
    )
    cover.visual(
        Box((0.024, 0.0012, 0.0025)),
        origin=Origin(xyz=(0.0130, 0.0058, -0.0016)),
        material=darker_steel,
        name="side_flange_0",
    )
    cover.visual(
        Box((0.024, 0.0012, 0.0025)),
        origin=Origin(xyz=(0.0130, -0.0138, -0.0016)),
        material=darker_steel,
        name="side_flange_1",
    )
    cover.visual(
        Box((0.0013, 0.0165, 0.0024)),
        origin=Origin(xyz=(0.0260, -0.0040, -0.0016)),
        material=darker_steel,
        name="nose_lip",
    )

    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=4.0, lower=0.0, upper=SWIVEL_UPPER),
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

    body = object_model.get_part("drive_body")
    cover = object_model.get_part("swivel_cover")
    swivel = object_model.get_articulation("body_to_swivel_cover")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="side_flange_0",
        negative_elem="connector_top",
        min_gap=0.0015,
        name="cover side flange clears connector top",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="connector_top",
        min_gap=0.0035,
        name="closed cover plate clears USB shell",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_plate",
        elem_b="connector_top",
        min_overlap=0.008,
        name="closed cover footprint protects connector",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="pivot_head",
        negative_elem="cover_plate",
        min_gap=0.0,
        max_gap=0.0008,
        name="rivet head captures cover with slight running clearance",
    )

    closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    with ctx.pose({swivel: SWIVEL_UPPER}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="side_flange_0",
            negative_elem="body_shell",
            min_gap=0.0008,
            name="stowed cover flange clears body top",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="cover_plate",
            elem_b="body_shell",
            min_overlap=0.010,
            name="stowed cover folds flat over drive body",
        )
        stowed_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")

    if closed_aabb is not None and stowed_aabb is not None:
        closed_front = float(closed_aabb[1][0])
        stowed_rear = float(stowed_aabb[0][0])
        ctx.check(
            "swivel cover reverses from connector to stowage side",
            closed_front > 0.025 and stowed_rear < -0.018,
            details=f"closed_front={closed_front:.4f}, stowed_rear={stowed_rear:.4f}",
        )
    else:
        ctx.fail("swivel cover pose AABBs available", "Expected cover plate AABBs in closed and stowed poses.")

    return ctx.report()


object_model = build_object_model()
