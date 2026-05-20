from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


BARREL_OUTER_RADIUS = 0.024
BARREL_INNER_RADIUS = 0.0205
GUIDE_BORE_RADIUS = 0.0085
GUIDE_OUTER_RADIUS = 0.030

BASE_HUB_RADIUS = 0.050
BASE_HUB_HEIGHT = 0.018
BASE_COLLAR_RADIUS = 0.040
BASE_COLLAR_HEIGHT = 0.038

BARREL_TUBE_BOTTOM_Z = 0.030
BARREL_TUBE_TOP_Z = 0.482
GUIDE_BOTTOM_Z = BARREL_TUBE_TOP_Z
GUIDE_TOP_Z = 0.514
LOWER_COLLAR_BOTTOM_Z = 0.018
LOWER_COLLAR_HEIGHT = 0.046
LOWER_COLLAR_RADIUS = 0.031
CAVITY_BOTTOM_Z = 0.060

ROD_RADIUS = 0.0055
ROD_LENGTH = 0.420
ROD_CENTER_Z = -0.070
SLIDE_TRAVEL = 0.220

HANDLE_WIDTH = 0.285
HANDLE_DEPTH = 0.032
HANDLE_THICKNESS = 0.024
HANDLE_CENTER_Z = 0.148
PISTON_LENGTH = 0.012
PISTON_CENTER_Z = -0.258
PISTON_RADIUS = 0.0175


def _tripod_base_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(BASE_HUB_RADIUS).extrude(BASE_HUB_HEIGHT)
    collar = cq.Workplane("XY").circle(BASE_COLLAR_RADIUS).extrude(BASE_COLLAR_HEIGHT)

    foot = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.028, 0.004),
                (0.028, 0.020),
                (0.072, 0.023),
                (0.145, 0.014),
                (0.188, 0.008),
                (0.195, 0.002),
                (0.188, 0.000),
                (0.145, 0.000),
                (0.062, 0.003),
            ]
        )
        .close()
        .extrude(0.024, both=True)
    )

    base = hub.union(collar).union(foot)
    for angle_deg in (120.0, 240.0):
        base = base.union(foot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    return base


def _barrel_shell_shape() -> cq.Workplane:
    outer_tube = (
        cq.Workplane("XY")
        .workplane(offset=BARREL_TUBE_BOTTOM_Z)
        .circle(BARREL_OUTER_RADIUS)
        .extrude(BARREL_TUBE_TOP_Z - BARREL_TUBE_BOTTOM_Z)
    )
    lower_collar = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_COLLAR_BOTTOM_Z)
        .circle(LOWER_COLLAR_RADIUS)
        .extrude(LOWER_COLLAR_HEIGHT)
    )
    top_guide = (
        cq.Workplane("XY")
        .workplane(offset=GUIDE_BOTTOM_Z)
        .circle(GUIDE_OUTER_RADIUS)
        .extrude(GUIDE_TOP_Z - GUIDE_BOTTOM_Z)
    )

    barrel = outer_tube.union(lower_collar).union(top_guide)

    barrel_cavity = (
        cq.Workplane("XY")
        .workplane(offset=CAVITY_BOTTOM_Z)
        .circle(BARREL_INNER_RADIUS)
        .extrude((BARREL_TUBE_TOP_Z - CAVITY_BOTTOM_Z) + 0.004)
    )
    guide_bore = (
        cq.Workplane("XY")
        .workplane(offset=GUIDE_BOTTOM_Z - 0.002)
        .circle(GUIDE_BORE_RADIUS)
        .extrude((GUIDE_TOP_Z - GUIDE_BOTTOM_Z) + 0.004)
    )

    return barrel.cut(barrel_cavity).cut(guide_bore)


def _handle_shape() -> cq.Workplane:
    grip = (
        cq.Workplane("YZ")
        .ellipse(HANDLE_DEPTH / 2.0, HANDLE_THICKNESS / 2.0)
        .extrude(HANDLE_WIDTH / 2.0, both=True)
        .translate((0.0, 0.0, HANDLE_CENTER_Z))
    )
    stem = (
        cq.Workplane("XY")
        .workplane(offset=0.026)
        .ellipse(0.014, 0.010)
        .extrude(0.096)
    )
    upper_spine = (
        cq.Workplane("XY")
        .workplane(offset=0.112)
        .rect(0.060, 0.028)
        .extrude(0.022)
    )

    return grip.union(stem).union(upper_spine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_track_pump")

    model.material("powder_gray", rgba=(0.28, 0.30, 0.32, 1.0))
    model.material("satin_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("chrome", rgba=(0.84, 0.86, 0.88, 1.0))
    model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("gauge_black", rgba=(0.05, 0.06, 0.07, 1.0))

    pump_body = model.part("pump_body")
    pump_body.visual(
        mesh_from_cadquery(_tripod_base_shape(), "tripod_base"),
        material="powder_gray",
        name="tripod_base",
    )
    pump_body.visual(
        mesh_from_cadquery(_barrel_shell_shape(), "barrel_shell"),
        material="satin_aluminum",
        name="barrel_shell",
    )
    pump_body.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.033, 0.0, 0.064), rpy=(0.0, pi / 2.0, 0.0)),
        material="powder_gray",
        name="gauge_housing",
    )
    pump_body.visual(
        Cylinder(radius=0.025, length=0.0025),
        origin=Origin(xyz=(0.04175, 0.0, 0.064), rpy=(0.0, pi / 2.0, 0.0)),
        material="gauge_black",
        name="gauge_face",
    )
    pump_body.inertial = Inertial.from_geometry(
        Box((0.39, 0.35, GUIDE_TOP_Z)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z / 2.0)),
    )

    rod_handle = model.part("rod_handle")
    rod_handle.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, ROD_CENTER_Z)),
        material="chrome",
        name="pump_rod",
    )
    rod_handle.visual(
        Cylinder(radius=PISTON_RADIUS, length=PISTON_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PISTON_CENTER_Z)),
        material="rubber_black",
        name="piston_head",
    )
    rod_handle.visual(
        mesh_from_cadquery(_handle_shape(), "t_handle"),
        material="rubber_black",
        name="t_handle",
    )
    rod_handle.inertial = Inertial.from_geometry(
        Box((HANDLE_WIDTH, HANDLE_DEPTH, 0.19)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "body_to_rod_handle",
        ArticulationType.PRISMATIC,
        parent=pump_body,
        child=rod_handle,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=180.0,
            velocity=1.0,
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
    pump_body = object_model.get_part("pump_body")
    rod_handle = object_model.get_part("rod_handle")
    slider = object_model.get_articulation("body_to_rod_handle")
    limits = slider.motion_limits

    ctx.allow_isolated_part(
        rod_handle,
        reason=(
            "The pump rod and piston ride inside the barrel with running clearance; "
            "the prismatic stage is intentionally support-disconnected in the "
            "authored rest pose while remaining centered and retained inside the barrel."
        ),
    )

    ctx.check(
        "pump uses a vertical prismatic slide",
        slider.articulation_type == ArticulationType.PRISMATIC
        and slider.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 0.20,
        details=(
            f"type={slider.articulation_type}, axis={slider.axis}, "
            f"limits={limits}"
        ),
    )

    ctx.expect_within(
        rod_handle,
        pump_body,
        axes="xy",
        inner_elem="pump_rod",
        outer_elem="barrel_shell",
        margin=0.002,
        name="rod stays centered within the barrel at rest",
    )
    ctx.expect_overlap(
        rod_handle,
        pump_body,
        axes="z",
        elem_a="pump_rod",
        elem_b="barrel_shell",
        min_overlap=0.24,
        name="rod is deeply inserted in the barrel at rest",
    )

    rest_pos = ctx.part_world_position(rod_handle)
    with ctx.pose({slider: SLIDE_TRAVEL}):
        ctx.expect_within(
            rod_handle,
            pump_body,
            axes="xy",
            inner_elem="pump_rod",
            outer_elem="barrel_shell",
            margin=0.002,
            name="rod stays centered within the barrel when extended",
        )
        ctx.expect_overlap(
            rod_handle,
            pump_body,
            axes="z",
            elem_a="pump_rod",
            elem_b="barrel_shell",
            min_overlap=0.055,
            name="rod retains insertion at full extension",
        )
        extended_pos = ctx.part_world_position(rod_handle)

    ctx.check(
        "handle rises upward when the pump is pulled",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.18,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    handle_aabb = ctx.part_element_world_aabb(rod_handle, elem="t_handle")
    barrel_aabb = ctx.part_element_world_aabb(pump_body, elem="barrel_shell")
    handle_width = None
    barrel_diameter = None
    if handle_aabb is not None and barrel_aabb is not None:
        handle_width = handle_aabb[1][0] - handle_aabb[0][0]
        barrel_diameter = barrel_aabb[1][0] - barrel_aabb[0][0]
    ctx.check(
        "T-handle reads broad relative to the barrel",
        handle_width is not None
        and barrel_diameter is not None
        and handle_width > 0.22
        and handle_width > (4.5 * barrel_diameter),
        details=f"handle_width={handle_width}, barrel_diameter={barrel_diameter}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
