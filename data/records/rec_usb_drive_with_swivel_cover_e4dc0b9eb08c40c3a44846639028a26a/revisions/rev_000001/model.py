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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    plastic_black = model.material("plastic_black", rgba=(0.08, 0.09, 0.10, 1.0))
    plastic_gray = model.material("plastic_gray", rgba=(0.23, 0.24, 0.27, 1.0))
    metal_shell = model.material("metal_shell", rgba=(0.76, 0.78, 0.80, 1.0))
    metal_cover = model.material("metal_cover", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.42, 0.44, 0.47, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.064, 0.020, 0.010)),
        mass=0.020,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
    )

    body.visual(
        Box((0.012, 0.018, 0.007)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=plastic_gray,
        name="rear_cap",
    )
    body.visual(
        Box((0.022, 0.018, 0.007)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=plastic_black,
        name="center_body",
    )
    body.visual(
        Box((0.018, 0.016, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=plastic_black,
        name="front_shoulder",
    )
    body.visual(
        Box((0.006, 0.014, 0.0052)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=dark_metal,
        name="connector_collar",
    )
    body.visual(
        Box((0.013, 0.0122, 0.0046)),
        origin=Origin(xyz=(0.0315, 0.0, 0.0)),
        material=metal_shell,
        name="usb_shell",
    )
    body.visual(
        Cylinder(radius=0.0031, length=0.0087),
        origin=Origin(xyz=(-0.020, 0.0092, 0.0)),
        material=dark_metal,
        name="pivot_pin",
    )

    cover = model.part("cover")
    cover.inertial = Inertial.from_geometry(
        Box((0.069, 0.023, 0.011)),
        mass=0.009,
        origin=Origin(xyz=(0.034, -0.010, 0.0)),
    )

    cover.visual(
        Box((0.069, 0.020, 0.0012)),
        origin=Origin(xyz=(0.034, -0.010, 0.00495)),
        material=metal_cover,
        name="top_plate",
    )
    cover.visual(
        Box((0.069, 0.020, 0.0012)),
        origin=Origin(xyz=(0.034, -0.010, -0.00495)),
        material=metal_cover,
        name="bottom_plate",
    )
    cover.visual(
        Box((0.069, 0.003, 0.0100)),
        origin=Origin(xyz=(0.034, -0.0215, 0.0)),
        material=metal_cover,
        name="side_bridge",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.00595)),
        material=metal_cover,
        name="top_rivet_head",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=0.0008),
        origin=Origin(xyz=(0.0, 0.0, -0.00595)),
        material=metal_cover,
        name="bottom_rivet_head",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.020, 0.010, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
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
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            min_overlap=0.015,
            name="closed cover tracks the body footprint",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="top_plate",
            negative_elem="center_body",
            min_gap=0.0005,
            max_gap=0.0012,
            name="top plate clears the body core",
        )
        ctx.expect_gap(
            body,
            cover,
            axis="z",
            positive_elem="center_body",
            negative_elem="bottom_plate",
            min_gap=0.0005,
            max_gap=0.0012,
            name="bottom plate clears the body core",
        )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({swivel: pi}):
        open_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "cover swings behind the drive at half turn",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] < 0.0
        and open_aabb[0][0] < closed_aabb[0][0] - 0.050,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
