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

    body_length = 0.041
    body_width = 0.018
    body_thickness = 0.008
    usb_length = 0.012
    usb_width = 0.0124
    usb_thickness = 0.0046

    cover_length = 0.058
    cover_width = 0.0192
    cover_inner_height = 0.0092
    cover_plate_thickness = 0.0008

    pivot_x = -0.010
    pivot_y = -(body_width * 0.5 - 0.0016)
    cover_center_x = 0.015
    cover_center_y = -pivot_y
    cover_panel_z = cover_inner_height * 0.5 + cover_plate_thickness * 0.5
    collar_length = 0.0014
    collar_z = body_thickness * 0.5 + collar_length * 0.5
    rear_bridge_x = -0.0145
    rear_bridge_thickness = 0.0066

    body_plastic = model.material("body_plastic", rgba=(0.09, 0.10, 0.12, 1.0))
    body_accent = model.material("body_accent", rgba=(0.18, 0.20, 0.24, 1.0))
    cover_metal = model.material("cover_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    connector_metal = model.material("connector_metal", rgba=(0.82, 0.83, 0.85, 1.0))
    connector_core = model.material("connector_core", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, body_thickness)),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.013, body_width * 0.92, body_thickness * 0.78)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=body_accent,
        name="rear_grip",
    )
    body.visual(
        Box((usb_length, usb_width, usb_thickness)),
        origin=Origin(xyz=(body_length * 0.5 + usb_length * 0.5, 0.0, 0.0)),
        material=connector_metal,
        name="usb_shell",
    )
    body.visual(
        Box((0.0082, 0.0100, 0.0012)),
        origin=Origin(xyz=(body_length * 0.5 + 0.0065, 0.0, -0.0009)),
        material=connector_core,
        name="usb_tongue",
    )

    body.inertial = Inertial.from_geometry(
        Box((body_length + usb_length, body_width, 0.011)),
        mass=0.022,
    )

    cover = model.part("cover")
    cover.visual(
        Box((cover_length, cover_width, cover_plate_thickness)),
        origin=Origin(xyz=(cover_center_x, cover_center_y, cover_panel_z)),
        material=cover_metal,
        name="top_panel",
    )
    cover.visual(
        Box((cover_length, cover_width, cover_plate_thickness)),
        origin=Origin(xyz=(cover_center_x, cover_center_y, -cover_panel_z)),
        material=cover_metal,
        name="bottom_panel",
    )
    cover.visual(
        Box(
            (
                rear_bridge_thickness,
                cover_width,
                cover_inner_height + 2.0 * cover_plate_thickness + 0.0004,
            )
        ),
        origin=Origin(xyz=(rear_bridge_x, cover_center_y, 0.0)),
        material=cover_metal,
        name="rear_bridge",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=collar_length),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=cover_metal,
        name="top_collar",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=collar_length),
        origin=Origin(xyz=(0.0, 0.0, -collar_z)),
        material=cover_metal,
        name="bottom_collar",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_length, cover_width, cover_inner_height + 2.0 * cover_plate_thickness)),
        mass=0.010,
        origin=Origin(xyz=(0.010, cover_center_y, 0.0)),
    )

    model.articulation(
        "cover_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(pivot_x, pivot_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("cover_swivel")

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="top_panel",
        negative_elem="body_shell",
        min_gap=0.0003,
        max_gap=0.0010,
        name="top panel clears the body shell",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="body_shell",
        negative_elem="bottom_panel",
        min_gap=0.0003,
        max_gap=0.0010,
        name="bottom panel clears the body shell",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="top_panel",
        elem_b="body_shell",
        min_overlap=0.015,
        name="cover stays centered over the body in the closed pose",
    )

    with ctx.pose({swivel: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="usb_shell",
            min_gap=0.010,
            name="swung cover clears the connector opening direction",
        )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
