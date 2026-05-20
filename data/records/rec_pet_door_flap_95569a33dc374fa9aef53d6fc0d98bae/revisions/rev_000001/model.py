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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_OUTER_RADIUS = 0.101
FRAME_INNER_RADIUS = 0.076
FRAME_DEPTH = 0.042
HINGE_Z = 0.060
FLAP_RADIUS = 0.071


def _build_frame_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.092, -0.021),
            (0.101, -0.017),
            (0.109, -0.010),
            (0.109, -0.004),
            (0.097, 0.000),
            (0.109, 0.004),
            (0.109, 0.010),
            (0.101, 0.017),
            (0.092, 0.021),
        ],
        [
            (0.073, -0.021),
            (FRAME_INNER_RADIUS, -0.015),
            (FRAME_INNER_RADIUS, 0.015),
            (0.073, 0.021),
        ],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    shell.rotate_y(pi / 2.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_cat_flap")

    frame_plastic = model.material("frame_plastic", rgba=(0.92, 0.92, 0.90, 1.0))
    hinge_plastic = model.material("hinge_plastic", rgba=(0.78, 0.78, 0.76, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.28, 0.31, 0.35, 0.38))
    seal_dark = model.material("seal_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_build_frame_shell_mesh(), "cat_flap_frame_shell"),
        material=frame_plastic,
        name="frame_shell",
    )
    frame.visual(
        Box((0.010, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.050, 0.053)),
        material=hinge_plastic,
        name="hinge_bridge_left",
    )
    frame.visual(
        Box((0.010, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.050, 0.053)),
        material=hinge_plastic,
        name="hinge_bridge_right",
    )
    frame.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, -0.050, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_plastic,
        name="hinge_knuckle_left",
    )
    frame.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.050, HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_plastic,
        name="hinge_knuckle_right",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=FLAP_RADIUS, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, -HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=flap_tint,
        name="flap_panel",
    )
    flap.visual(
        Box((0.004, 0.092, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=seal_dark,
        name="flap_top_rail",
    )
    flap.visual(
        Cylinder(radius=0.004, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=seal_dark,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.008, 0.028, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, -0.124)),
        material=seal_dark,
        name="pull_lip",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=0.0,
            upper=1.20,
        ),
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

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")
    flap_panel = flap.get_visual("flap_panel")

    ctx.check(
        "cat flap assembly is authored",
        frame is not None and flap is not None and hinge is not None and flap_panel is not None,
        details="Expected a fixed round frame part, a flap part, and a revolute hinge articulation.",
    )
    ctx.expect_origin_gap(
        flap,
        frame,
        axis="z",
        min_gap=0.050,
        max_gap=0.070,
        name="hinge axis sits near the top of the circular frame",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            frame,
            axes="yz",
            elem_a=flap_panel,
            min_overlap=0.130,
            name="closed flap spans the round opening footprint",
        )

        closed_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    with ctx.pose({hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(flap, elem=flap_panel)

    closed_center_x = None
    open_center_x = None
    closed_bottom_z = None
    open_bottom_z = None
    if closed_aabb is not None:
        closed_center_x = 0.5 * (closed_aabb[0][0] + closed_aabb[1][0])
        closed_bottom_z = closed_aabb[0][2]
    if open_aabb is not None:
        open_center_x = 0.5 * (open_aabb[0][0] + open_aabb[1][0])
        open_bottom_z = open_aabb[0][2]

    ctx.check(
        "flap swings inward through the frame",
        open_center_x is not None
        and closed_center_x is not None
        and open_center_x > closed_center_x + 0.040,
        details=f"closed_center_x={closed_center_x}, open_center_x={open_center_x}",
    )
    ctx.check(
        "opening the flap lifts its lower edge",
        open_bottom_z is not None
        and closed_bottom_z is not None
        and open_bottom_z > closed_bottom_z + 0.030,
        details=f"closed_bottom_z={closed_bottom_z}, open_bottom_z={open_bottom_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
