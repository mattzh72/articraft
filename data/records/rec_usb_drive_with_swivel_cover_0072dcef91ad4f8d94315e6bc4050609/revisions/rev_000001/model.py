from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.11, 0.12, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))

    body_length = 0.044
    body_width = 0.017
    body_thickness = 0.0076
    body_corner = 0.0033
    connector_length = 0.012
    connector_width = 0.0122
    connector_thickness = 0.0046

    body_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_length, body_width, body_corner, corner_segments=8),
            body_thickness,
            center=True,
        ),
        "usb_body_shell",
    )

    body = model.part("body")
    body.visual(body_shell, material=plastic_black, name="plastic_shell")
    body.visual(
        Box((connector_length, connector_width, connector_thickness)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=metal_silver,
        name="connector_shell",
    )
    body.visual(
        Box((0.0054, 0.0007, 0.0054)),
        origin=Origin(xyz=(-0.0155, -0.00885, 0.0)),
        material=metal_silver,
        name="pivot_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.056, 0.018, 0.008)),
        mass=0.018,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover_length = 0.055
    rear_overhang = 0.006
    cover_wall = 0.0009
    side_clearance = 0.0007
    top_clearance = 0.0008
    cover_inner_width = body_width + 2.0 * side_clearance
    cover_inner_height = body_thickness + 2.0 * top_clearance
    cover_outer_height = cover_inner_height + 2.0 * cover_wall
    cover_center_x = cover_length * 0.5 - rear_overhang
    cover_plate_center_y = cover_inner_width * 0.5 + cover_wall * 0.5
    cover_plate_center_z = cover_inner_height * 0.5 + cover_wall * 0.5

    cover.visual(
        Box((cover_length, cover_inner_width + cover_wall, cover_wall)),
        origin=Origin(xyz=(cover_center_x, cover_plate_center_y, cover_plate_center_z)),
        material=brushed_steel,
        name="top_shell",
    )
    cover.visual(
        Box((cover_length, cover_inner_width + cover_wall, cover_wall)),
        origin=Origin(xyz=(cover_center_x, cover_plate_center_y, -cover_plate_center_z)),
        material=brushed_steel,
        name="bottom_shell",
    )
    cover.visual(
        Box((cover_length, cover_wall, cover_outer_height)),
        origin=Origin(xyz=(cover_center_x, 0.0, 0.0)),
        material=brushed_steel,
        name="side_spine",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_length, cover_inner_width + cover_wall, cover_outer_height)),
        mass=0.010,
        origin=Origin(xyz=(cover_center_x, cover_plate_center_y * 0.95, 0.0)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(
            xyz=(
                -0.0155,
                -(body_width * 0.5 + side_clearance + cover_wall * 0.5),
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    ctx.expect_within(
        body,
        cover,
        axes="yz",
        inner_elem="plastic_shell",
        margin=0.0010,
        name="cover wraps the body in the closed pose",
    )
    ctx.expect_within(
        body,
        cover,
        axes="yz",
        inner_elem="connector_shell",
        margin=0.0010,
        name="cover encloses the connector in the closed pose",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        elem_b="connector_shell",
        min_overlap=0.010,
        name="closed cover spans the connector lengthwise",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({swivel: pi / 2.0}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector_shell",
            min_gap=0.010,
            name="open cover clears the connector nose",
        )
        open_cover_aabb = ctx.part_world_aabb(cover)

    cover_swings_sideways = (
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and (open_cover_aabb[1][1] - closed_cover_aabb[1][1]) > 0.020
    )
    ctx.check(
        "cover swings out to the side",
        cover_swings_sideways,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
