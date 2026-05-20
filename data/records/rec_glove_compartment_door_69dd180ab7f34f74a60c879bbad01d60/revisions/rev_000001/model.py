from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OPENING_WIDTH = 0.54
OPENING_HEIGHT = 0.25
HINGE_Y = -0.047
HINGE_Z = -0.018
DOOR_WIDTH = 0.50
DOOR_HEIGHT = 0.225
DOOR_THICKNESS = 0.026
DOOR_PANEL_CENTER_Z = 0.1275


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dashboard_glove_compartment")

    dashboard_mat = Material("soft_charcoal_dashboard", rgba=(0.08, 0.085, 0.085, 1.0))
    bin_mat = Material("matte_black_bin", rgba=(0.015, 0.016, 0.017, 1.0))
    door_mat = Material("slightly_lighter_door", rgba=(0.11, 0.115, 0.115, 1.0))
    handle_mat = Material("black_handle", rgba=(0.01, 0.011, 0.012, 1.0))
    hinge_mat = Material("black_hinge_hardware", rgba=(0.02, 0.02, 0.02, 1.0))

    housing = model.part("housing")

    # A simple dashboard face with a real rectangular opening cut through it.
    fascia = (
        cq.Workplane("XY")
        .box(0.72, 0.060, 0.38)
        .translate((0.0, 0.0, 0.125))
        .cut(
            cq.Workplane("XY")
            .box(OPENING_WIDTH, 0.090, OPENING_HEIGHT)
            .translate((0.0, 0.0, 0.125))
        )
    )
    try:
        fascia = fascia.edges("|Y").fillet(0.008)
    except Exception:
        # Sharp-edged fallback is still a valid cut opening if a CAD kernel
        # refuses one of the small trim fillets.
        pass
    housing.visual(
        mesh_from_cadquery(fascia, "dashboard_opening"),
        material=dashboard_mat,
        name="dashboard_opening",
    )

    # Hollow shallow bin behind the opening: open at the front, closed at rear,
    # with side/top/bottom walls instead of a solid block.
    bin_shell = (
        cq.Workplane("XY")
        .box(0.512, 0.230, 0.236)
        .translate((0.0, 0.140, 0.115))
        .cut(
            cq.Workplane("XY")
            .box(0.472, 0.235, 0.196)
            .translate((0.0, 0.118, 0.117))
        )
    )
    try:
        bin_shell = bin_shell.edges("|Y").fillet(0.004)
    except Exception:
        pass
    housing.visual(
        mesh_from_cadquery(bin_shell, "shallow_bin"),
        material=bin_mat,
        name="shallow_bin",
    )

    for name, x in (("hinge_leaf_0", -0.185), ("hinge_leaf_1", 0.185)):
        housing.visual(
            Box((0.130, 0.018, 0.026)),
            origin=Origin(xyz=(x, -0.039, -0.020)),
            material=hinge_mat,
            name=name,
        )
    for name, x in (("hinge_barrel_0", -0.185), ("hinge_barrel_1", 0.185)):
        housing.visual(
            Cylinder(radius=0.012, length=0.120),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_mat,
            name=name,
        )
    housing.visual(
        Cylinder(radius=0.0045, length=0.500),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=Material("dark_steel_pin", rgba=(0.045, 0.045, 0.042, 1.0)),
        name="hinge_pin",
    )

    door = model.part("door")

    door_panel = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)
        .translate((0.0, 0.0, DOOR_PANEL_CENTER_Z))
    )
    try:
        door_panel = door_panel.edges("|Y").fillet(0.009)
    except Exception:
        pass
    door.visual(
        mesh_from_cadquery(door_panel, "door_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_mat,
        name="door_panel",
    )

    # Subtle molded styling on the face and a separate dark pull handle seated
    # into the panel surface.
    door.visual(
        Box((0.430, 0.004, 0.155)),
        origin=Origin(xyz=(0.0, -0.015, 0.132)),
        material=Material("door_recess_shadow", rgba=(0.075, 0.078, 0.078, 1.0)),
        name="recess_panel",
    )
    door.visual(
        Box((0.185, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, -0.022, 0.191)),
        material=handle_mat,
        name="pull_handle",
    )
    door.visual(
        Box((0.115, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.034, 0.191)),
        material=Material("handle_highlight", rgba=(0.18, 0.185, 0.18, 1.0)),
        name="handle_lip",
    )

    door.visual(
        Cylinder(radius=0.011, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.190, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, 0.019)),
        material=hinge_mat,
        name="door_hinge_leaf",
    )

    model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("lower_hinge")

    ctx.allow_overlap(
        housing,
        door,
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel",
        reason="The steel pin is intentionally captured inside the rotating hinge barrel.",
    )

    ctx.check(
        "single bottom hinge joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(hinge.axis[0] - 1.0) < 1e-9,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "hinge opens about ninety degrees",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and abs(hinge.motion_limits.upper - pi / 2.0) < 1e-6,
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_ok = panel_aabb is not None and (
            panel_aabb[0][0] > -OPENING_WIDTH / 2.0 + 0.010
            and panel_aabb[1][0] < OPENING_WIDTH / 2.0 - 0.010
            and panel_aabb[0][2] > -0.006
            and panel_aabb[1][2] < OPENING_HEIGHT - 0.010
            and panel_aabb[1][1] < -0.030
        )
        ctx.check(
            "closed door fits inside dashboard opening",
            closed_ok,
            details=f"door_panel_aabb={panel_aabb}",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="x",
            elem_a="door_hinge_barrel",
            elem_b="dashboard_opening",
            min_overlap=0.15,
            name="hinge is centered below opening",
        )
        ctx.expect_within(
            housing,
            door,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem="door_hinge_barrel",
            margin=0.001,
            name="hinge pin is captured inside door barrel",
        )
        ctx.expect_overlap(
            housing,
            door,
            axes="x",
            elem_a="hinge_pin",
            elem_b="door_hinge_barrel",
            min_overlap=0.18,
            name="hinge pin spans the rotating barrel",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: pi / 2.0}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        open_ok = (
            closed_panel_aabb is not None
            and open_panel_aabb is not None
            and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.16
            and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.17
            and (open_panel_aabb[1][2] - open_panel_aabb[0][2]) < 0.040
        )
        ctx.check(
            "door swings downward and outward",
            open_ok,
            details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
