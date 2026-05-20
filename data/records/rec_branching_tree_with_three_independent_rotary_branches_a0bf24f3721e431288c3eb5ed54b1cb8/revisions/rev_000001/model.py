from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SPINE_CENTER_Y = -0.020
SPINE_DEPTH = 0.040
SPINE_FRONT_Y = SPINE_CENTER_Y + SPINE_DEPTH / 2.0

ROOT_BARREL_RADIUS = 0.012
ROOT_BARREL_LENGTH = 0.020
HINGE_Y = 0.012

LOWER_Z = 0.170
MIDDLE_Z = 0.300
UPPER_Z = 0.430


def _beam_origin(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    z_center: float = 0.0,
) -> tuple[Origin, float]:
    x0, y0 = start_xy
    x1, y1 = end_xy
    dx = x1 - x0
    dy = y1 - y0
    length = (dx * dx + dy * dy) ** 0.5
    return (
        Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, z_center),
            rpy=(0.0, 0.0, atan2(dx, dy)),
        ),
        length,
    )


def _add_support_visuals(model: ArticulatedObject, support) -> None:
    frame_material = "graphite_frame"

    support.visual(
        Box((0.300, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, -0.118, 0.010)),
        material=frame_material,
        name="base_rail",
    )
    support.visual(
        Box((0.030, 0.040, 0.440)),
        origin=Origin(xyz=(-0.100, -0.090, 0.239)),
        material=frame_material,
        name="left_upright",
    )
    support.visual(
        Box((0.030, 0.040, 0.440)),
        origin=Origin(xyz=(0.100, -0.090, 0.239)),
        material=frame_material,
        name="right_upright",
    )
    support.visual(
        Box((0.250, 0.040, 0.022)),
        origin=Origin(xyz=(0.0, -0.090, 0.469)),
        material=frame_material,
        name="top_rail",
    )
    support.visual(
        Box((0.052, SPINE_DEPTH, 0.490)),
        origin=Origin(xyz=(0.0, SPINE_CENTER_Y, 0.245)),
        material=frame_material,
        name="forward_spine",
    )
    support.visual(
        Box((0.240, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, -0.055, LOWER_Z)),
        material=frame_material,
        name="lower_bridge_plate",
    )
    support.visual(
        Box((0.240, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, -0.055, UPPER_Z)),
        material=frame_material,
        name="upper_bridge_plate",
    )

    for hinge_x, hinge_z, prefix in (
        (-0.070, LOWER_Z, "lower"),
        (0.000, MIDDLE_Z, "middle"),
        (0.070, UPPER_Z, "upper"),
    ):
        support.visual(
            Box((0.052, 0.012, 0.020)),
            origin=Origin(xyz=(hinge_x, -0.025, hinge_z)),
            material=frame_material,
            name=f"{prefix}_hinge_bridge",
        )
        support.visual(
            Box((0.010, 0.032, 0.028)),
            origin=Origin(xyz=(hinge_x - 0.015, -0.003, hinge_z)),
            material=frame_material,
            name=f"{prefix}_ear_left",
        )
        support.visual(
            Box((0.010, 0.032, 0.028)),
            origin=Origin(xyz=(hinge_x + 0.015, -0.003, hinge_z)),
            material=frame_material,
            name=f"{prefix}_ear_right",
        )


def _add_branch(
    model: ArticulatedObject,
    *,
    name: str,
    color: str,
    hinge_name: str,
    hinge_x: float,
    hinge_z: float,
    offset_x: float,
    reach_y: float,
    tray_width: float,
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0),
):
    branch = model.part(name)
    branch.visual(
        Cylinder(radius=ROOT_BARREL_RADIUS, length=ROOT_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=color,
        name=f"{name}_barrel",
    )
    branch.visual(
        Box((0.018, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=color,
        name=f"{name}_root_block",
    )
    branch.visual(
        Box((0.026, 0.060, 0.016)),
        origin=Origin(xyz=(offset_x * 0.20, 0.062, 0.0)),
        material=color,
        name=f"{name}_connector",
    )
    beam_origin, beam_length = _beam_origin((offset_x * 0.20, 0.070), (offset_x, reach_y + 0.004))
    branch.visual(
        Box((0.020, beam_length, 0.016)),
        origin=beam_origin,
        material=color,
        name=f"{name}_arm",
    )
    branch.visual(
        Box((tray_width, 0.036, 0.012)),
        origin=Origin(xyz=(offset_x, reach_y + 0.016, -0.003)),
        material=color,
        name=f"{name}_tray",
    )
    branch.visual(
        Box((tray_width, 0.006, 0.022)),
        origin=Origin(xyz=(offset_x, reach_y + 0.031, 0.002)),
        material=color,
        name=f"{name}_front_lip",
    )

    model.articulation(
        hinge_name,
        ArticulationType.REVOLUTE,
        parent="support",
        child=branch,
        origin=Origin(xyz=(hinge_x, HINGE_Y, hinge_z)),
        axis=axis,
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=16.0, velocity=1.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_tooling_tree")

    model.material("graphite_frame", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("branch_silver", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("branch_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("branch_dark", rgba=(0.41, 0.45, 0.49, 1.0))

    support = model.part("support")
    _add_support_visuals(model, support)

    _add_branch(
        model,
        name="lower_branch",
        color="branch_dark",
        hinge_name="lower_branch_hinge",
        hinge_x=-0.070,
        hinge_z=LOWER_Z,
        offset_x=-0.035,
        reach_y=0.170,
        tray_width=0.080,
        axis=(1.0, 0.0, 0.0),
    )
    _add_branch(
        model,
        name="middle_branch",
        color="branch_silver",
        hinge_name="middle_branch_hinge",
        hinge_x=0.000,
        hinge_z=MIDDLE_Z,
        offset_x=0.000,
        reach_y=0.192,
        tray_width=0.092,
        axis=(1.0, 0.0, 0.0),
    )
    _add_branch(
        model,
        name="upper_branch",
        color="branch_steel",
        hinge_name="upper_branch_hinge",
        hinge_x=0.070,
        hinge_z=UPPER_Z,
        offset_x=0.035,
        reach_y=0.170,
        tray_width=0.080,
        axis=(1.0, 0.0, 0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    lower_branch = object_model.get_part("lower_branch")
    middle_branch = object_model.get_part("middle_branch")
    upper_branch = object_model.get_part("upper_branch")
    lower_hinge = object_model.get_articulation("lower_branch_hinge")
    middle_hinge = object_model.get_articulation("middle_branch_hinge")
    upper_hinge = object_model.get_articulation("upper_branch_hinge")

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

    ctx.expect_contact(support, lower_branch, name="lower branch is mounted to support")
    ctx.expect_contact(support, middle_branch, name="middle branch is mounted to support")
    ctx.expect_contact(support, upper_branch, name="upper branch is mounted to support")

    ctx.expect_gap(middle_branch, lower_branch, axis="z", min_gap=0.070, name="middle branch clears lower branch")
    ctx.expect_gap(upper_branch, middle_branch, axis="z", min_gap=0.070, name="upper branch clears middle branch")

    def _branch_lifts(branch, hinge, label: str) -> bool:
        rest_aabb = ctx.part_world_aabb(branch)
        with ctx.pose({hinge: 0.75}):
            raised_aabb = ctx.part_world_aabb(branch)
        if rest_aabb is None or raised_aabb is None:
            return ctx.fail(f"{label} pose query", "missing AABB for articulation direction check")
        rest_top = rest_aabb[1][2]
        raised_top = raised_aabb[1][2]
        return ctx.check(
            f"{label} positive rotation raises branch",
            raised_top > rest_top + 0.060,
            f"expected raised top z > {rest_top + 0.060:.3f}, got {raised_top:.3f}",
        )

    _branch_lifts(lower_branch, lower_hinge, "lower branch")
    _branch_lifts(middle_branch, middle_hinge, "middle branch")
    _branch_lifts(upper_branch, upper_hinge, "upper branch")

    with ctx.pose({lower_hinge: 0.70, middle_hinge: 0.25, upper_hinge: 0.55}):
        ctx.fail_if_parts_overlap_in_current_pose(name="staggered branch pose remains clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
