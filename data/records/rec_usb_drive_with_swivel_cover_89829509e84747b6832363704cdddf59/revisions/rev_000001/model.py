from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="swivel_usb_drive")

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    accent_black = model.material("accent_black", rgba=(0.07, 0.07, 0.08, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))
    connector_white = model.material("connector_white", rgba=(0.90, 0.92, 0.94, 1.0))

    body_core_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.034, 0.018, 0.0034, corner_segments=8), 0.008),
        "usb_body_core",
    )
    front_bezel_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.006, 0.018, 0.0024, corner_segments=8), 0.006),
        "usb_front_bezel",
    )

    body = model.part("body")

    body.visual(
        body_core_mesh,
        origin=Origin(xyz=(0.013, 0.010, 0.0)),
        material=body_black,
        name="body_core",
    )
    body.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.033, 0.010, 0.0)),
        material=accent_black,
        name="front_bezel",
    )
    body.visual(
        Box((0.008, 0.0045, 0.006)),
        origin=Origin(xyz=(0.002, 0.00325, 0.0)),
        material=accent_black,
        name="pivot_cheek",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0092),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material=accent_black,
        name="pivot_hub",
    )

    connector_x = 0.043
    connector_y = 0.010
    shell_len = 0.014
    shell_w = 0.0122
    shell_h = 0.0046
    shell_wall = 0.00045

    body.visual(
        Box((shell_len, shell_w, shell_wall)),
        origin=Origin(xyz=(connector_x, connector_y, shell_h * 0.5 - shell_wall * 0.5)),
        material=metal,
        name="connector_top",
    )
    body.visual(
        Box((shell_len, shell_w, shell_wall)),
        origin=Origin(xyz=(connector_x, connector_y, -shell_h * 0.5 + shell_wall * 0.5)),
        material=metal,
        name="connector_bottom",
    )
    body.visual(
        Box((shell_len, shell_wall, shell_h - 2.0 * shell_wall)),
        origin=Origin(
            xyz=(
                connector_x,
                connector_y - shell_w * 0.5 + shell_wall * 0.5,
                0.0,
            )
        ),
        material=metal,
        name="connector_left_wall",
    )
    body.visual(
        Box((shell_len, shell_wall, shell_h - 2.0 * shell_wall)),
        origin=Origin(
            xyz=(
                connector_x,
                connector_y + shell_w * 0.5 - shell_wall * 0.5,
                0.0,
            )
        ),
        material=metal,
        name="connector_right_wall",
    )
    body.visual(
        Box((0.012, 0.0088, 0.0013)),
        origin=Origin(xyz=(0.039, connector_y, -0.00015)),
        material=connector_white,
        name="connector_tongue",
    )
    body.visual(
        Box((0.012, 0.0132, 0.0042)),
        origin=Origin(xyz=(0.0345, connector_y, 0.0)),
        material=accent_black,
        name="connector_carrier",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.052, 0.020, 0.010)),
        mass=0.03,
        origin=Origin(xyz=(0.022, 0.010, 0.0)),
    )

    cover = model.part("cover")
    cover_plate_t = 0.0008
    cover_plate_z = 0.00495
    cover_len = 0.053
    cover_plate_w = 0.0208
    cover_center_x = 0.0225
    cover_center_y = 0.0100
    side_wall_y = 0.0208
    side_wall_t = 0.0008
    cover_outer_h = 2.0 * (cover_plate_z + cover_plate_t * 0.5)
    cover_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(cover_len, cover_plate_w, 0.0043, corner_segments=10),
            cover_plate_t,
        ),
        "usb_cover_plate",
    )

    cover.visual(
        cover_plate_mesh,
        origin=Origin(xyz=(cover_center_x, cover_center_y, cover_plate_z)),
        material=metal,
        name="cover_top_plate",
    )
    cover.visual(
        cover_plate_mesh,
        origin=Origin(xyz=(cover_center_x, cover_center_y, -cover_plate_z)),
        material=metal,
        name="cover_bottom_plate",
    )
    cover.visual(
        Box((cover_len, side_wall_t, cover_outer_h)),
        origin=Origin(xyz=(cover_center_x, side_wall_y, 0.0)),
        material=metal,
        name="cover_side_wall",
    )
    cover.visual(
        Cylinder(radius=0.0039, length=0.0012),
        origin=Origin(xyz=(0.0, 0.001, 0.00595)),
        material=metal,
        name="cover_top_rivet",
    )
    cover.visual(
        Cylinder(radius=0.0039, length=0.0012),
        origin=Origin(xyz=(0.0, 0.001, -0.00595)),
        material=metal,
        name="cover_bottom_rivet",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.053, 0.022, 0.012)),
        mass=0.012,
        origin=Origin(xyz=(0.0225, 0.0105, 0.0)),
    )

    model.articulation(
        "cover_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=7.0),
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
    swivel = object_model.get_articulation("cover_swivel")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_top_plate",
        elem_b="body_core",
        min_overlap=0.016,
        name="closed cover dominates the broad front face",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_top_plate",
        negative_elem="body_core",
        min_gap=0.0003,
        max_gap=0.0012,
        name="closed top plate sits just above the body",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="body_core",
        negative_elem="cover_bottom_plate",
        min_gap=0.0003,
        max_gap=0.0012,
        name="closed bottom plate sits just below the body",
    )

    with ctx.pose({swivel: 1.5708}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="connector_top",
            negative_elem="cover_top_plate",
            min_gap=0.03,
            name="quarter-turn swivel clears the connector",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
