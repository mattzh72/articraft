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
)


FRAME_DEPTH = 0.009
HINGE_X = 0.075
HINGE_Y = 0.007
HINGE_RADIUS = 0.0035
HINGE_CENTER_KNUCKLE = 0.012
HINGE_OUTER_KNUCKLE = 0.016
FOLD_LIMIT = 1.42


def _front_frame_geometry() -> ExtrudeWithHolesGeometry:
    # 2-D profile coordinates are (horizontal X, vertical Z); the resulting
    # extrusion axis is rotated onto world Y when attached as a visual.
    outer = [
        (-0.070, 0.029),
        (0.070, 0.029),
        (0.068, 0.012),
        (0.062, -0.017),
        (0.045, -0.027),
        (0.014, -0.026),
        (0.006, -0.015),
        (0.000, -0.012),
        (-0.006, -0.015),
        (-0.014, -0.026),
        (-0.045, -0.027),
        (-0.062, -0.017),
        (-0.068, 0.012),
    ]
    left_opening = [
        (-0.062, 0.018),
        (-0.007, 0.018),
        (-0.018, -0.018),
        (-0.054, -0.020),
    ]
    right_opening = [(-x, z) for x, z in reversed(left_opening)]
    return ExtrudeWithHolesGeometry(
        outer,
        [left_opening, right_opening],
        FRAME_DEPTH,
        center=True,
    )


def _lens_geometry(side: int) -> ExtrudeGeometry:
    # Slightly oversize the lens blanks so their edges disappear into the rim.
    lens = [
        (side * 0.063, 0.019),
        (side * 0.006, 0.019),
        (side * 0.017, -0.019),
        (side * 0.055, -0.021),
    ]
    if side > 0:
        lens = list(reversed(lens))
    return ExtrudeGeometry(lens, 0.0012, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="thick_frame_wayfarer_glasses")

    acetate = model.material("dark_acetate", rgba=(0.015, 0.012, 0.010, 1.0))
    lens_smoke = model.material("smoke_lens", rgba=(0.46, 0.56, 0.62, 0.34))
    hinge_metal = model.material("warm_steel_pin", rgba=(0.62, 0.52, 0.38, 1.0))

    frame = model.part("front_frame")
    frame.visual(
        mesh_from_geometry(_front_frame_geometry(), "wayfarer_front_frame"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=acetate,
        name="front_shell",
    )
    frame.visual(
        mesh_from_geometry(_lens_geometry(-1), "left_smoke_lens"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_smoke,
        name="left_lens",
    )
    frame.visual(
        mesh_from_geometry(_lens_geometry(1), "right_smoke_lens"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_smoke,
        name="right_lens",
    )

    for side, label in [(-1, "left"), (1, "right")]:
        x = side * HINGE_X
        frame.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_OUTER_KNUCKLE),
            origin=Origin(xyz=(x, HINGE_Y, 0.014)),
            material=acetate,
            name=f"{label}_hinge_top",
        )
        frame.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_OUTER_KNUCKLE),
            origin=Origin(xyz=(x, HINGE_Y, -0.014)),
            material=acetate,
            name=f"{label}_hinge_bottom",
        )
        frame.visual(
            Box((0.006, 0.008, 0.012)),
            origin=Origin(xyz=(side * 0.069, HINGE_Y, 0.014)),
            material=acetate,
            name=f"{label}_top_bridge",
        )
        frame.visual(
            Box((0.006, 0.008, 0.012)),
            origin=Origin(xyz=(side * 0.069, HINGE_Y, -0.014)),
            material=acetate,
            name=f"{label}_bottom_bridge",
        )
        frame.visual(
            Cylinder(radius=0.00115, length=0.049),
            origin=Origin(xyz=(x, HINGE_Y, 0.0)),
            material=hinge_metal,
            name=f"{label}_hinge_pin",
        )

    def add_temple(name: str, side: int) -> None:
        temple = model.part(name)
        inward = -side
        temple.visual(
            Cylinder(radius=HINGE_RADIUS * 0.96, length=HINGE_CENTER_KNUCKLE),
            origin=Origin(),
            material=acetate,
            name="barrel",
        )
        temple.visual(
            Box((0.0085, 0.031, 0.010)),
            origin=Origin(xyz=(inward * 0.0038, 0.0155, 0.0)),
            material=acetate,
            name="hinge_leaf",
        )
        temple.visual(
            Box((0.010, 0.108, 0.017)),
            origin=Origin(xyz=(inward * 0.0042, 0.078, -0.001)),
            material=acetate,
            name="arm",
        )
        temple.visual(
            Box((0.008, 0.026, 0.012)),
            origin=Origin(xyz=(inward * 0.0035, 0.142, -0.004)),
            material=acetate,
            name="ear_tip",
        )

    add_temple("left_temple", -1)
    add_temple("right_temple", 1)

    model.articulation(
        "front_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="left_temple",
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=FOLD_LIMIT),
    )
    model.articulation(
        "front_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="right_temple",
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=0.0, upper=FOLD_LIMIT),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("front_frame")
    left = object_model.get_part("left_temple")
    right = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("front_to_left_temple")
    right_hinge = object_model.get_articulation("front_to_right_temple")

    # The arms start behind the brow frame, while the coaxial barrels touch the
    # fixed hinge knuckles. These are the important fit relationships for the
    # wayfarer-style fold.
    ctx.expect_gap(
        left,
        frame,
        axis="y",
        min_gap=0.010,
        positive_elem="arm",
        negative_elem="front_shell",
        name="left broad temple starts behind frame",
    )
    ctx.expect_gap(
        right,
        frame,
        axis="y",
        min_gap=0.010,
        positive_elem="arm",
        negative_elem="front_shell",
        name="right broad temple starts behind frame",
    )
    ctx.expect_contact(
        left,
        frame,
        elem_a="barrel",
        elem_b="left_hinge_top",
        contact_tol=0.0008,
        name="left temple barrel clips under top knuckle",
    )
    ctx.expect_contact(
        right,
        frame,
        elem_a="barrel",
        elem_b="right_hinge_top",
        contact_tol=0.0008,
        name="right temple barrel clips under top knuckle",
    )

    left_rest_aabb = ctx.part_element_world_aabb(left, elem="arm")
    right_rest_aabb = ctx.part_element_world_aabb(right, elem="arm")
    with ctx.pose({left_hinge: FOLD_LIMIT}):
        ctx.expect_contact(
            left,
            frame,
            elem_a="barrel",
            elem_b="left_hinge_top",
            contact_tol=0.0008,
            name="left hinge remains clipped while folded",
        )
        left_fold_aabb = ctx.part_element_world_aabb(left, elem="arm")
    with ctx.pose({right_hinge: FOLD_LIMIT}):
        ctx.expect_contact(
            right,
            frame,
            elem_a="barrel",
            elem_b="right_hinge_top",
            contact_tol=0.0008,
            name="right hinge remains clipped while folded",
        )
        right_fold_aabb = ctx.part_element_world_aabb(right, elem="arm")

    def aabb_center_x(aabb):
        return None if aabb is None else 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "left temple folds inward",
        left_rest_aabb is not None
        and left_fold_aabb is not None
        and aabb_center_x(left_fold_aabb) > aabb_center_x(left_rest_aabb) + 0.045,
        details=f"rest={left_rest_aabb}, folded={left_fold_aabb}",
    )
    ctx.check(
        "right temple folds inward",
        right_rest_aabb is not None
        and right_fold_aabb is not None
        and aabb_center_x(right_fold_aabb) < aabb_center_x(right_rest_aabb) - 0.045,
        details=f"rest={right_rest_aabb}, folded={right_fold_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
