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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_usb_drive")

    black_plastic = model.material("black_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.19, 0.20, 0.22, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    shell_steel = model.material("shell_steel", rgba=(0.68, 0.70, 0.74, 1.0))
    tongue_blue = model.material("tongue_blue", rgba=(0.18, 0.43, 0.74, 1.0))

    body_width = 0.0186
    body_depth = 0.0100
    body_length = 0.0560
    connector_width = 0.0122
    connector_depth = 0.0046
    connector_length = 0.0130
    connector_insertion = 0.0020
    pivot_z = -0.0240

    body = model.part("body")
    body_shell = _mesh(
        "usb_body_shell",
        ExtrudeGeometry(
            rounded_rect_profile(body_width, body_depth, radius=0.0026, corner_segments=8),
            body_length,
            center=True,
        ),
    )
    body.visual(body_shell, material=black_plastic, name="plastic_shell")
    body.visual(
        Box((body_width * 0.70, body_depth * 0.74, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, pivot_z - 0.0015)),
        material=dark_insert,
        name="tail_insert",
    )

    connector_center_z = (body_length * 0.5) + (connector_length * 0.5) - connector_insertion
    shell_wall = 0.00045
    side_wall_depth = connector_depth - (2.0 * shell_wall)
    body.visual(
        Box((connector_width, shell_wall, connector_length)),
        origin=Origin(xyz=(0.0, (connector_depth * 0.5) - (shell_wall * 0.5), connector_center_z)),
        material=brushed_steel,
        name="connector_top",
    )
    body.visual(
        Box((connector_width, shell_wall, connector_length)),
        origin=Origin(xyz=(0.0, -(connector_depth * 0.5) + (shell_wall * 0.5), connector_center_z)),
        material=brushed_steel,
        name="connector_bottom",
    )
    body.visual(
        Box((shell_wall, side_wall_depth, connector_length)),
        origin=Origin(xyz=((connector_width * 0.5) - (shell_wall * 0.5), 0.0, connector_center_z)),
        material=brushed_steel,
        name="connector_right_wall",
    )
    body.visual(
        Box((shell_wall, side_wall_depth, connector_length)),
        origin=Origin(xyz=(-(connector_width * 0.5) + (shell_wall * 0.5), 0.0, connector_center_z)),
        material=brushed_steel,
        name="connector_left_wall",
    )
    body.visual(
        Box((0.0094, 0.0016, 0.0100)),
        origin=Origin(xyz=(0.0, -0.0003, connector_center_z - 0.0006)),
        material=tongue_blue,
        name="connector_tongue",
    )
    body.visual(
        Box((0.0010, 0.0048, 0.0070)),
        origin=Origin(xyz=(-(body_width * 0.5) - 0.0005, 0.0, pivot_z + 0.0035)),
        material=brushed_steel,
        name="pivot_lug",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_length + connector_length * 0.6)),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    cover = model.part("cover")
    cover_width = 0.0218
    cover_depth = 0.0136
    cover_length = 0.0640
    sheet_thickness = 0.0012
    hub_radius = 0.0019
    cover.visual(
        Box((cover_width, sheet_thickness, cover_length)),
        origin=Origin(
            xyz=(cover_width * 0.5, (cover_depth * 0.5) - (sheet_thickness * 0.5), cover_length * 0.5),
        ),
        material=shell_steel,
        name="front_panel",
    )
    cover.visual(
        Box((sheet_thickness, cover_depth, cover_length)),
        origin=Origin(xyz=(sheet_thickness * 0.5, 0.0, cover_length * 0.5)),
        material=shell_steel,
        name="left_arm",
    )
    cover.visual(
        Box((sheet_thickness, cover_depth, cover_length)),
        origin=Origin(xyz=(cover_width - (sheet_thickness * 0.5), 0.0, cover_length * 0.5)),
        material=shell_steel,
        name="right_arm",
    )
    cover.visual(
        Box((cover_width, sheet_thickness, 0.0080)),
        origin=Origin(
            xyz=(cover_width * 0.5, -((cover_depth * 0.5) - (sheet_thickness * 0.5)), 0.0040),
        ),
        material=shell_steel,
        name="rear_bridge",
    )
    cover.visual(
        Cylinder(radius=hub_radius, length=0.0040),
        origin=Origin(xyz=(sheet_thickness * 0.5, 0.0, 0.0020)),
        material=shell_steel,
        name="pivot_ear",
    )
    cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_depth, cover_length)),
        mass=0.010,
        origin=Origin(xyz=(cover_width * 0.5, 0.0, cover_length * 0.5)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-(body_width * 0.5) - 0.0012, 0.0, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
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
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="y",
            positive_elem="front_panel",
            negative_elem="plastic_shell",
            min_gap=0.0003,
            max_gap=0.0012,
            name="closed cover sits just proud of the body front",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="xz",
            elem_a="front_panel",
            elem_b="plastic_shell",
            min_overlap=0.016,
            name="closed cover spans the body face",
        )

    with ctx.pose({swivel: pi / 2.0}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector_top",
            negative_elem="front_panel",
            min_gap=0.008,
            name="opened cover swings clear of the connector mouth",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
