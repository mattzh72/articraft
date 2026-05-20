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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    body_plastic = model.material("body_plastic", rgba=(0.13, 0.15, 0.18, 1.0))
    body_accent = model.material("body_accent", rgba=(0.20, 0.24, 0.30, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.75, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.055, 0.018, 0.008)),
        mass=0.020,
        origin=Origin(),
    )

    body_shell = ExtrudeGeometry.centered(
        rounded_rect_profile(0.008, 0.018, 0.0022, corner_segments=8),
        0.055,
    ).rotate_y(pi / 2.0)
    body.visual(_mesh("usb_body_shell", body_shell), material=body_plastic, name="body_shell")
    body.visual(
        Box((0.026, 0.010, 0.0012)),
        origin=Origin(xyz=(-0.001, 0.0, 0.0043)),
        material=body_accent,
        name="thumb_pad",
    )
    body.visual(
        Box((0.012, 0.0122, 0.0046)),
        origin=Origin(xyz=(0.0335, 0.0, 0.0)),
        material=brushed_steel,
        name="connector",
    )
    body.visual(
        Box((0.003, 0.0145, 0.0062)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=dark_steel,
        name="connector_collar",
    )
    body.visual(
        Cylinder(radius=0.0022, length=0.020),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0012),
        origin=Origin(xyz=(-0.020, 0.0096, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_pin_head",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0012),
        origin=Origin(xyz=(-0.020, -0.0096, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_pin_head",
    )

    flange_profile = rounded_rect_profile(0.018, 0.010, 0.0020, corner_segments=8)
    flange_hole = superellipse_profile(0.0046, 0.0046, exponent=2.0, segments=20)
    mounting_flange = ExtrudeWithHolesGeometry(
        flange_profile,
        [flange_hole],
        0.002,
        center=True,
    ).rotate_x(pi / 2.0)
    body.visual(
        Box((0.006, 0.0036, 0.005)),
        origin=Origin(xyz=(-0.026, 0.0098, 0.0)),
        material=body_accent,
        name="mounting_neck",
    )
    body.visual(
        _mesh("usb_mounting_flange", mounting_flange),
        origin=Origin(xyz=(-0.031, 0.0116, 0.0)),
        material=body_accent,
        name="mounting_flange",
    )

    cover = model.part("cover")
    cover.inertial = Inertial.from_geometry(
        Box((0.061, 0.020, 0.012)),
        mass=0.006,
        origin=Origin(xyz=(0.0305, 0.0, 0.0)),
    )
    cover.visual(
        Box((0.061, 0.020, 0.0012)),
        origin=Origin(xyz=(0.0305, 0.0, 0.0052)),
        material=brushed_steel,
        name="top_plate",
    )
    cover.visual(
        Box((0.061, 0.020, 0.0012)),
        origin=Origin(xyz=(0.0305, 0.0, -0.0052)),
        material=brushed_steel,
        name="bottom_plate",
    )
    cover.visual(
        Box((0.0014, 0.020, 0.0116)),
        origin=Origin(xyz=(0.0617, 0.0, 0.0)),
        material=brushed_steel,
        name="front_bridge",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_joint = object_model.get_articulation("body_to_cover")

    ctx.expect_gap(
        cover,
        body,
        axis="x",
        positive_elem="front_bridge",
        negative_elem="connector",
        min_gap=0.0010,
        max_gap=0.0030,
        name="closed cover bridge clears connector nose",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="top_plate",
        negative_elem="body_shell",
        min_gap=0.0002,
        max_gap=0.0015,
        name="top plate sits just above the body shell",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="body_shell",
        negative_elem="bottom_plate",
        min_gap=0.0002,
        max_gap=0.0015,
        name="bottom plate sits just below the body shell",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="top_plate",
        elem_b="body_shell",
        min_overlap=0.016,
        name="closed cover spans the body footprint",
    )

    closed_bridge = ctx.part_element_world_aabb(cover, elem="front_bridge")
    with ctx.pose({cover_joint: pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector",
            negative_elem="front_bridge",
            min_gap=0.040,
            name="opened cover swings fully behind the connector",
        )
        open_bridge = ctx.part_element_world_aabb(cover, elem="front_bridge")

    ctx.check(
        "cover bridge moves rearward when opened",
        closed_bridge is not None
        and open_bridge is not None
        and open_bridge[1][0] < closed_bridge[0][0] - 0.020,
        details=f"closed={closed_bridge}, open={open_bridge}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
