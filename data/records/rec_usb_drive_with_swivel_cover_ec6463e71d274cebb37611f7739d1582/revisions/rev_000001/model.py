from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    matte_black = model.material("matte_black", rgba=(0.15, 0.16, 0.18, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    usb_blue = model.material("usb_blue", rgba=(0.18, 0.43, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.040, 0.0174, 0.0082)),
        material=matte_black,
        name="housing_shell",
    )
    body.visual(
        Box((0.0108, 0.0122, 0.0044)),
        origin=Origin(xyz=(0.0254, 0.0, 0.0)),
        material=brushed_steel,
        name="usb_shell",
    )
    body.visual(
        Box((0.0078, 0.0102, 0.0014)),
        origin=Origin(xyz=(0.0260, 0.0, -0.0002)),
        material=usb_blue,
        name="usb_tongue",
    )
    body.visual(
        Cylinder(radius=0.0029, length=0.0089),
        origin=Origin(xyz=(-0.0150, 0.0098, 0.0)),
        material=dark_gray,
        name="pivot_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.052, 0.018, 0.010)),
        mass=0.030,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.054, 0.0196, 0.00085)),
        origin=Origin(xyz=(0.0230, -0.0098, 0.004875)),
        material=brushed_steel,
        name="top_leaf",
    )
    cover.visual(
        Box((0.054, 0.0196, 0.00085)),
        origin=Origin(xyz=(0.0230, -0.0098, -0.004875)),
        material=brushed_steel,
        name="bottom_leaf",
    )
    cover.visual(
        Box((0.045, 0.0009, 0.0106)),
        origin=Origin(xyz=(0.0280, -0.01915, 0.0)),
        material=brushed_steel,
        name="side_spine",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.056, 0.021, 0.011)),
        mass=0.014,
        origin=Origin(xyz=(0.020, -0.0090, 0.0)),
    )

    model.articulation(
        "cover_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0150, 0.0098, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("cover_swivel")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_leaf",
            negative_elem="housing_shell",
            min_gap=0.0002,
            max_gap=0.0010,
            name="top leaf sits just above the body",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="housing_shell",
            negative_elem="bottom_leaf",
            min_gap=0.0002,
            max_gap=0.0010,
            name="bottom leaf sits just below the body",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_leaf",
            elem_b="housing_shell",
            min_overlap=0.016,
            name="closed cover wraps the body footprint",
        )
        ctx.expect_gap(
            body,
            body,
            axis="x",
            positive_elem="usb_shell",
            negative_elem="housing_shell",
            min_gap=-0.0002,
            max_gap=0.0002,
            name="usb shell starts flush with the housing nose",
        )

    closed_top = ctx.part_element_world_aabb(cover, elem="top_leaf")
    with ctx.pose({swivel: pi}):
        opened_top = ctx.part_element_world_aabb(cover, elem="top_leaf")

    closed_center = _aabb_center(closed_top) if closed_top is not None else None
    opened_center = _aabb_center(opened_top) if opened_top is not None else None
    ctx.check(
        "cover flips behind the drive at half turn",
        closed_center is not None
        and opened_center is not None
        and opened_center[0] < closed_center[0] - 0.030,
        details=f"closed_center={closed_center}, opened_center={opened_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
