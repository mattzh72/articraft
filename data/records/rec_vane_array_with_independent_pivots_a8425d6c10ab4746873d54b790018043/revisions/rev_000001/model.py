from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.37
FRAME_HEIGHT = 0.27
FRAME_DEPTH = 0.05
SIDE_RAIL = 0.025
TOP_BOTTOM_RAIL = 0.025

OPENING_WIDTH = FRAME_WIDTH - 2.0 * SIDE_RAIL
OPENING_HEIGHT = FRAME_HEIGHT - 2.0 * TOP_BOTTOM_RAIL

VANE_COUNT = 5
LUG_DEPTH = 0.012
LUG_WIDTH_Y = 0.022
LUG_HEIGHT = 0.018

VANE_HUB_RADIUS = 0.007
VANE_BLADE_DEPTH = 0.03
VANE_BLADE_THICKNESS = 0.007
VANE_TOTAL_LENGTH = OPENING_WIDTH - 2.0 * LUG_DEPTH
VANE_HUB_LENGTH = 0.006
VANE_BLADE_LENGTH = VANE_TOTAL_LENGTH - 2.0 * VANE_HUB_LENGTH


def _vane_centers() -> list[float]:
    pitch = OPENING_HEIGHT / (VANE_COUNT + 1)
    bottom_inner = -OPENING_HEIGHT / 2.0
    return [bottom_inner + pitch * (index + 1) for index in range(VANE_COUNT)]


def _build_vane_shape() -> cq.Workplane:
    left_hub = cq.Workplane("YZ").circle(VANE_HUB_RADIUS).extrude(VANE_HUB_LENGTH)
    blade = (
        cq.Workplane("YZ")
        .ellipse(VANE_BLADE_DEPTH / 2.0, VANE_BLADE_THICKNESS / 2.0)
        .extrude(VANE_BLADE_LENGTH)
        .translate((VANE_HUB_LENGTH, 0.0, 0.0))
    )
    right_hub = (
        cq.Workplane("YZ")
        .circle(VANE_HUB_RADIUS)
        .extrude(VANE_HUB_LENGTH)
        .translate((VANE_HUB_LENGTH + VANE_BLADE_LENGTH, 0.0, 0.0))
    )
    return left_hub.union(blade).union(right_hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_shutter_vane_module")

    frame_finish = model.material("frame_finish", rgba=(0.23, 0.25, 0.28, 1.0))
    vane_finish = model.material("vane_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-FRAME_WIDTH / 2.0 + SIDE_RAIL / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="left_rail",
    )
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(FRAME_WIDTH / 2.0 - SIDE_RAIL / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="right_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, TOP_BOTTOM_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0 - TOP_BOTTOM_RAIL / 2.0)),
        material=frame_finish,
        name="top_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, FRAME_DEPTH, TOP_BOTTOM_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT / 2.0 + TOP_BOTTOM_RAIL / 2.0)),
        material=frame_finish,
        name="bottom_rail",
    )

    left_inner_face = -OPENING_WIDTH / 2.0
    right_inner_face = OPENING_WIDTH / 2.0
    for index, z_center in enumerate(_vane_centers(), start=1):
        frame.visual(
            Box((LUG_DEPTH, LUG_WIDTH_Y, LUG_HEIGHT)),
            origin=Origin(xyz=(left_inner_face + LUG_DEPTH / 2.0, 0.0, z_center)),
            material=frame_finish,
            name=f"left_lug_{index}",
        )
        frame.visual(
            Box((LUG_DEPTH, LUG_WIDTH_Y, LUG_HEIGHT)),
            origin=Origin(xyz=(right_inner_face - LUG_DEPTH / 2.0, 0.0, z_center)),
            material=frame_finish,
            name=f"right_lug_{index}",
        )

    left_support_face = -OPENING_WIDTH / 2.0 + LUG_DEPTH

    for index, z_center in enumerate(_vane_centers(), start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(
            mesh_from_cadquery(_build_vane_shape(), f"vane_{index}_body"),
            origin=Origin(),
            material=vane_finish,
            name="vane_body",
        )
        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(left_support_face, 0.0, z_center)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=-0.85,
                upper=0.85,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    vanes = [object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)]
    hinges = [
        object_model.get_articulation(f"frame_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "independent_vane_joint_count",
        len(hinges) == VANE_COUNT,
        details=f"expected {VANE_COUNT} vane joints, found {len(hinges)}",
    )

    for index, (vane, hinge) in enumerate(zip(vanes, hinges), start=1):
        ctx.expect_contact(vane, frame, name=f"vane_{index}_mounted_to_frame")
        ctx.check(
            f"vane_{index}_hinge_axis_is_longitudinal",
            tuple(float(component) for component in hinge.axis) == (1.0, 0.0, 0.0),
            details=f"expected hinge axis (1, 0, 0), got {hinge.axis}",
        )

    center_index = VANE_COUNT // 2
    center_vane = vanes[center_index]
    center_hinge = hinges[center_index]

    with ctx.pose({center_hinge: 0.75}):
        ctx.expect_contact(center_vane, frame, name="center_vane_supported_in_open_pose")
        if center_index > 0:
            ctx.expect_gap(
                center_vane,
                vanes[center_index - 1],
                axis="z",
                min_gap=0.008,
                name="center_vane_clears_lower_neighbor_when_open",
            )
        if center_index + 1 < len(vanes):
            ctx.expect_gap(
                vanes[center_index + 1],
                center_vane,
                axis="z",
                min_gap=0.008,
                name="center_vane_clears_upper_neighbor_when_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
