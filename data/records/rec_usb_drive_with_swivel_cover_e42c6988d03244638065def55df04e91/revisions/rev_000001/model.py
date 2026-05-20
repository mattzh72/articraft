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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    connector_steel = model.material("connector_steel", rgba=(0.83, 0.85, 0.87, 1.0))
    pcb_blue = model.material("pcb_blue", rgba=(0.20, 0.49, 0.78, 1.0))

    body_length = 0.042
    body_width = 0.0172
    body_thickness = 0.0078

    connector_length = 0.0125
    connector_width = 0.0122
    connector_thickness = 0.0046

    pivot_x = -0.0185
    pivot_y = body_width / 2.0

    cover_length = 0.054
    cover_width = 0.0194
    cover_plate_thickness = 0.0009
    flange_thickness = 0.0008
    flange_height = 0.0088
    flange_length = 0.047
    rear_overhang = 0.003

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, body_thickness)),
        material=charcoal,
        name="body_shell",
    )
    body.visual(
        Box((connector_length, connector_width, connector_thickness)),
        origin=Origin(xyz=(body_length / 2.0 + connector_length / 2.0, 0.0, 0.0)),
        material=connector_steel,
        name="usb_shell",
    )
    body.visual(
        Box((0.0095, 0.0102, 0.0014)),
        origin=Origin(
            xyz=(body_length / 2.0 + 0.0048, 0.0, -(connector_thickness - 0.0014) / 2.0),
        ),
        material=pcb_blue,
        name="usb_insert",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.0005),
        origin=Origin(xyz=(pivot_x, pivot_y - 0.0002, body_thickness / 2.0 - 0.0002)),
        material=connector_steel,
        name="pivot_cap_top",
    )
    body.visual(
        Cylinder(radius=0.0025, length=0.0005),
        origin=Origin(xyz=(pivot_x, pivot_y - 0.0002, -body_thickness / 2.0 + 0.0002)),
        material=connector_steel,
        name="pivot_cap_bottom",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length + connector_length, body_width, body_thickness)),
        mass=0.016,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((cover_length, cover_width, cover_plate_thickness)),
        origin=Origin(
            xyz=(
                cover_length / 2.0 - rear_overhang,
                -pivot_y,
                0.0045,
            ),
        ),
        material=satin_steel,
        name="outer_panel",
    )
    cover.visual(
        Box((flange_length, flange_thickness, flange_height)),
        origin=Origin(
            xyz=(
                flange_length / 2.0 - rear_overhang,
                cover_width / 2.0 - flange_thickness / 2.0 - pivot_y,
                0.0,
            ),
        ),
        material=satin_steel,
        name="pivot_flange",
    )
    cover.visual(
        Box((flange_length, flange_thickness, flange_height)),
        origin=Origin(
            xyz=(
                flange_length / 2.0 - rear_overhang,
                -cover_width / 2.0 + flange_thickness / 2.0 - pivot_y,
                0.0,
            ),
        ),
        material=satin_steel,
        name="outer_flange",
    )
    cover.visual(
        Box((0.0065, cover_width, 0.0006)),
        origin=Origin(xyz=(0.0015, -pivot_y, 0.00415)),
        material=satin_steel,
        name="reinforcement_pad",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_length, cover_width, flange_height)),
        mass=0.006,
        origin=Origin(xyz=(cover_length / 2.0 - rear_overhang, -pivot_y, 0.0)),
    )

    model.articulation(
        "cover_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("cover_swivel")

    ctx.expect_overlap(
        body,
        cover,
        axes="x",
        elem_a="body_shell",
        elem_b="outer_panel",
        min_overlap=0.040,
        name="closed cover spans most of the body length",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="y",
        elem_a="body_shell",
        elem_b="outer_panel",
        min_overlap=0.016,
        name="closed cover is broad enough to cap the body",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="outer_panel",
        negative_elem="body_shell",
        min_gap=0.00005,
        max_gap=0.0004,
        name="cover plate sits just above the plastic shell",
    )

    with ctx.pose({swivel: math.pi / 2.0}):
        panel_aabb = ctx.part_element_world_aabb(cover, elem="outer_panel")
        body_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
        swung_out = (
            panel_aabb is not None
            and body_aabb is not None
            and ((panel_aabb[0][1] + panel_aabb[1][1]) / 2.0) > body_aabb[1][1] + 0.02
        )
        ctx.check(
            "quarter-turn swings the cover out to the side",
            swung_out,
            details=f"panel={panel_aabb}, body={body_aabb}",
        )

    with ctx.pose({swivel: math.pi}):
        cover_aabb = ctx.part_element_world_aabb(cover, elem="outer_panel")
        connector_aabb = ctx.part_element_world_aabb(body, elem="usb_shell")
        clear_of_connector = (
            cover_aabb is not None
            and connector_aabb is not None
            and cover_aabb[1][0] < connector_aabb[0][0] - 0.02
        )
        ctx.check(
            "half-turn parks the cover behind the connector",
            clear_of_connector,
            details=f"cover={cover_aabb}, connector={connector_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
