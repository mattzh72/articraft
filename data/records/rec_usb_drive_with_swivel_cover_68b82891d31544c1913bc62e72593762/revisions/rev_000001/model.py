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

    plastic_black = model.material("plastic_black", rgba=(0.11, 0.12, 0.13, 1.0))
    plastic_blue = model.material("plastic_blue", rgba=(0.11, 0.35, 0.72, 1.0))
    steel = model.material("steel", rgba=(0.74, 0.76, 0.78, 1.0))
    body_length = 0.040
    body_width = 0.018
    body_thickness = 0.008
    pivot_x = -0.0175
    pivot_y = 0.0082

    main_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_length, body_width, radius=0.0025, corner_segments=8),
            body_thickness,
            center=True,
        ),
        "usb_body_shell",
    )

    body = model.part("body")
    body.visual(
        main_shell_mesh,
        origin=Origin(xyz=(-0.0008, -0.0015, 0.0)),
        material=plastic_black,
        name="main_shell",
    )
    body.visual(
        Box((0.013, 0.0074, 0.0086)),
        origin=Origin(xyz=(-0.0138, 0.0046, 0.0)),
        material=plastic_blue,
        name="pivot_pad",
    )
    body.visual(
        Box((0.0124, 0.0118, 0.0046)),
        origin=Origin(xyz=(0.0248, -0.0016, 0.0)),
        material=steel,
        name="connector_shell",
    )
    body.visual(
        Box((0.0086, 0.0102, 0.0012)),
        origin=Origin(xyz=(0.0268, -0.0016, -0.0011)),
        material=plastic_blue,
        name="connector_tongue",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.055, 0.020, 0.010)),
        mass=0.018,
        origin=Origin(xyz=(0.003, -0.001, 0.0)),
    )

    cover = model.part("cover")
    cover_plate_thickness = 0.0009
    cover_plate_z = 0.00475
    cover_width = 0.0198
    cover_y_center = -0.0095
    cover.visual(
        Box((0.0480, cover_width, cover_plate_thickness)),
        origin=Origin(xyz=(0.0249, cover_y_center, cover_plate_z)),
        material=steel,
        name="top_plate",
    )
    cover.visual(
        Box((0.0480, cover_width, cover_plate_thickness)),
        origin=Origin(xyz=(0.0249, cover_y_center, -cover_plate_z)),
        material=steel,
        name="bottom_plate",
    )
    cover.visual(
        Box((0.0018, cover_width, 0.0104)),
        origin=Origin(xyz=(0.0497, cover_y_center, 0.0)),
        material=steel,
        name="front_bridge",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=cover_plate_thickness),
        origin=Origin(xyz=(0.0021, -0.0002, cover_plate_z)),
        material=steel,
        name="pivot_disc_top",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=cover_plate_thickness),
        origin=Origin(xyz=(0.0021, -0.0002, -cover_plate_z)),
        material=steel,
        name="pivot_disc_bottom",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.050, 0.020, 0.011)),
        mass=0.005,
        origin=Origin(xyz=(0.024, -0.009, 0.0)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="top_plate",
            elem_b="main_shell",
            min_overlap=0.012,
            name="closed cover plate spans the body footprint",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="main_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed top plate clears the shell",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="main_shell",
            negative_elem="bottom_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed bottom plate clears the shell",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            elem_a="top_plate",
            elem_b="connector_shell",
            min_overlap=0.010,
            name="closed cover reaches over the connector",
        )

    with ctx.pose({swivel: math.pi / 2.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="y",
            positive_elem="front_bridge",
            negative_elem="main_shell",
            min_gap=0.018,
            name="open cover swings clear to the side",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
