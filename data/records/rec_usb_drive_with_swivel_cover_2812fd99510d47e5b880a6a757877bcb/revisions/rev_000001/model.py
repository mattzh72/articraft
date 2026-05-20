from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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

    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    connector_steel = model.material("connector_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    tongue_ivory = model.material("tongue_ivory", rgba=(0.93, 0.92, 0.86, 1.0))

    body_length = 0.038
    body_width = 0.017
    body_thickness = 0.0074
    connector_length = 0.012
    connector_width = 0.0122
    connector_thickness = 0.0046

    sheet_thickness = 0.0008
    cover_clearance = 0.0006
    plate_length = 0.050
    plate_width = 0.0188
    bridge_thickness = 0.0030
    plate_center_x = 0.021
    plate_center_y = -0.0088
    plate_center_z = (body_thickness / 2.0) + cover_clearance + (sheet_thickness / 2.0)
    bridge_height = body_thickness + (2.0 * (cover_clearance + sheet_thickness))

    pivot_xyz = (-0.014, (body_width / 2.0) - 0.0002, 0.0)

    body_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_length, body_width, radius=0.0030, corner_segments=8),
            body_thickness,
            cap=True,
            center=True,
        ),
        "usb_body_shell",
    )

    body = model.part("body")
    body.visual(
        body_shell_mesh,
        material=plastic_black,
        name="plastic_body",
    )
    body.visual(
        Box((connector_length, connector_width, connector_thickness)),
        origin=Origin(
            xyz=(
                (body_length / 2.0) + (connector_length / 2.0) - 0.0002,
                0.0,
                0.0,
            )
        ),
        material=connector_steel,
        name="connector_shell",
    )
    body.visual(
        Box((0.0085, 0.0082, 0.0014)),
        origin=Origin(
            xyz=(
                (body_length / 2.0) + 0.0055,
                0.0,
                -0.0002,
            )
        ),
        material=tongue_ivory,
        name="connector_tongue",
    )
    body.visual(
        Cylinder(radius=0.0023, length=0.0006),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.0040)),
        material=connector_steel,
        name="top_pivot_boss",
    )
    body.visual(
        Cylinder(radius=0.0023, length=0.0006),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], -0.0040)),
        material=connector_steel,
        name="bottom_pivot_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length + connector_length, body_width, body_thickness)),
        mass=0.018,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((plate_length, plate_width, sheet_thickness)),
        origin=Origin(xyz=(plate_center_x, plate_center_y, plate_center_z)),
        material=satin_aluminum,
        name="top_plate",
    )
    cover.visual(
        Box((plate_length, plate_width, sheet_thickness)),
        origin=Origin(xyz=(plate_center_x, plate_center_y, -plate_center_z)),
        material=satin_aluminum,
        name="bottom_plate",
    )
    cover.visual(
        Box((bridge_thickness, plate_width, bridge_height)),
        origin=Origin(xyz=(0.0472, plate_center_y, 0.0)),
        material=satin_aluminum,
        name="front_bridge",
    )
    cover.visual(
        Cylinder(radius=0.0032, length=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.00525)),
        material=satin_aluminum,
        name="top_rivet",
    )
    cover.visual(
        Cylinder(radius=0.0032, length=0.0007),
        origin=Origin(xyz=(0.0, 0.0, -0.00525)),
        material=satin_aluminum,
        name="bottom_rivet",
    )
    cover.inertial = Inertial.from_geometry(
        Box((plate_length, plate_width, bridge_height)),
        mass=0.010,
        origin=Origin(xyz=(0.020, plate_center_y, 0.0)),
    )

    model.articulation(
        "swivel_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=pivot_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("swivel_cover")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="plastic_body",
            min_gap=0.00045,
            max_gap=0.00085,
            name="top plate sits closely above the body",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="plastic_body",
            negative_elem="bottom_plate",
            min_gap=0.00045,
            max_gap=0.00085,
            name="bottom plate sits closely below the body",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_plate",
            elem_b="plastic_body",
            min_overlap=0.015,
            name="cover plate wraps over the body footprint",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="front_bridge",
            negative_elem="connector_shell",
            min_gap=0.0006,
            max_gap=0.0014,
            name="front bridge clears the connector tip when closed",
        )

    body_aabb = ctx.part_world_aabb(body)

    with ctx.pose({swivel: 0.0}):
        closed_bridge = ctx.part_element_world_aabb(cover, elem="front_bridge")
    with ctx.pose({swivel: math.pi / 2.0}):
        side_bridge = ctx.part_element_world_aabb(cover, elem="front_bridge")
    with ctx.pose({swivel: math.pi}):
        rear_bridge = ctx.part_element_world_aabb(cover, elem="front_bridge")

    ctx.check(
        "cover swings outboard at quarter turn",
        body_aabb is not None
        and side_bridge is not None
        and side_bridge[0][1] > body_aabb[1][1] + 0.02,
        details=f"body_aabb={body_aabb}, side_bridge={side_bridge}",
    )
    ctx.check(
        "cover bridge rotates behind the body at half turn",
        body_aabb is not None
        and rear_bridge is not None
        and rear_bridge[1][0] < body_aabb[0][0] - 0.02,
        details=f"body_aabb={body_aabb}, rear_bridge={rear_bridge}",
    )
    ctx.check(
        "closed and open bridge positions differ substantially",
        closed_bridge is not None
        and rear_bridge is not None
        and ((closed_bridge[0][0] + closed_bridge[1][0]) / 2.0)
        > ((rear_bridge[0][0] + rear_bridge[1][0]) / 2.0) + 0.08,
        details=f"closed_bridge={closed_bridge}, rear_bridge={rear_bridge}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
