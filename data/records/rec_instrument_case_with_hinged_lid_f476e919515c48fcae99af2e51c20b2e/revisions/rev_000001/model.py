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
    MeshGeometry,
    mesh_from_geometry,
)


CASE_WIDTH = 0.64
CASE_DEPTH = 0.44
LOWER_HEIGHT = 0.18
LID_HEIGHT = 0.070
WALL = 0.018
REAR_X = -CASE_DEPTH / 2.0
FRONT_X = CASE_DEPTH / 2.0
HINGE_STANDOFF = 0.020
HINGE_X = REAR_X - HINGE_STANDOFF


def _open_shell_mesh(
    name: str,
    depth: float,
    width: float,
    height: float,
    *,
    open_face: str,
    x_offset: float = 0.0,
    fillet: float = 0.026,
):
    """Single connected thin-wall rectangular shell, open at the top or bottom."""

    del fillet  # The mesh is intentionally simple, with molded ribs adding edge detail.
    geom = MeshGeometry()

    def v(p):
        return geom.add_vertex(p[0] + x_offset, p[1], p[2])

    def quad(a, b, c, d):
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    xo0, xo1 = -depth / 2.0, depth / 2.0
    yo0, yo1 = -width / 2.0, width / 2.0
    xi0, xi1 = xo0 + WALL, xo1 - WALL
    yi0, yi1 = yo0 + WALL, yo1 - WALL
    z0, z1 = 0.0, height

    if open_face == ">Z":
        z_inner = z0 + WALL
        outer_low = [v(p) for p in ((xo0, yo0, z0), (xo1, yo0, z0), (xo1, yo1, z0), (xo0, yo1, z0))]
        outer_high = [v(p) for p in ((xo0, yo0, z1), (xo1, yo0, z1), (xo1, yo1, z1), (xo0, yo1, z1))]
        inner_floor = [v(p) for p in ((xi0, yi0, z_inner), (xi1, yi0, z_inner), (xi1, yi1, z_inner), (xi0, yi1, z_inner))]
        inner_high = [v(p) for p in ((xi0, yi0, z1), (xi1, yi0, z1), (xi1, yi1, z1), (xi0, yi1, z1))]

        quad(outer_low[0], outer_low[1], outer_low[2], outer_low[3])  # underside
        for i in range(4):
            quad(outer_low[i], outer_low[(i + 1) % 4], outer_high[(i + 1) % 4], outer_high[i])
            quad(inner_floor[(i + 1) % 4], inner_floor[i], inner_high[i], inner_high[(i + 1) % 4])
            quad(outer_high[i], outer_high[(i + 1) % 4], inner_high[(i + 1) % 4], inner_high[i])
        quad(inner_floor[3], inner_floor[2], inner_floor[1], inner_floor[0])  # visible interior floor
    else:
        z_inner = z1 - WALL
        outer_low = [v(p) for p in ((xo0, yo0, z0), (xo1, yo0, z0), (xo1, yo1, z0), (xo0, yo1, z0))]
        outer_high = [v(p) for p in ((xo0, yo0, z1), (xo1, yo0, z1), (xo1, yo1, z1), (xo0, yo1, z1))]
        inner_low = [v(p) for p in ((xi0, yi0, z0), (xi1, yi0, z0), (xi1, yi1, z0), (xi0, yi1, z0))]
        inner_top = [v(p) for p in ((xi0, yi0, z_inner), (xi1, yi0, z_inner), (xi1, yi1, z_inner), (xi0, yi1, z_inner))]

        quad(outer_high[3], outer_high[2], outer_high[1], outer_high[0])  # lid top
        for i in range(4):
            quad(outer_low[i], outer_low[(i + 1) % 4], outer_high[(i + 1) % 4], outer_high[i])
            quad(inner_low[(i + 1) % 4], inner_low[i], inner_top[i], inner_top[(i + 1) % 4])
            quad(outer_low[(i + 1) % 4], outer_low[i], inner_low[i], inner_low[(i + 1) % 4])
        quad(inner_top[0], inner_top[1], inner_top[2], inner_top[3])  # underside of shallow lid top

    return mesh_from_geometry(geom, name)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_equipment_case")

    plastic = model.material("dark_textured_polypropylene", rgba=(0.05, 0.055, 0.052, 1.0))
    molded_edge = model.material("slightly_lighter_molded_ribs", rgba=(0.085, 0.09, 0.085, 1.0))
    rubber = model.material("black_rubber_grip_gasket", rgba=(0.015, 0.015, 0.014, 1.0))
    steel = model.material("brushed_stainless_hardware", rgba=(0.60, 0.58, 0.54, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        _open_shell_mesh(
            "lower_deep_shell",
            CASE_DEPTH,
            CASE_WIDTH,
            LOWER_HEIGHT,
            open_face=">Z",
            fillet=0.032,
        ),
        material=plastic,
        name="lower_shell",
    )

    # Proud molded rim and compressed gasket around the upper opening.
    lower.visual(Box((CASE_DEPTH + 0.018, WALL, 0.014)), origin=Origin(xyz=(0.0, CASE_WIDTH / 2.0 + 0.002, LOWER_HEIGHT - 0.008)), material=molded_edge, name="side_rim_0")
    lower.visual(Box((CASE_DEPTH + 0.018, WALL, 0.014)), origin=Origin(xyz=(0.0, -CASE_WIDTH / 2.0 - 0.002, LOWER_HEIGHT - 0.008)), material=molded_edge, name="side_rim_1")
    lower.visual(Box((WALL, CASE_WIDTH + 0.018, 0.014)), origin=Origin(xyz=(FRONT_X + 0.002, 0.0, LOWER_HEIGHT - 0.008)), material=molded_edge, name="front_rim")

    # Molded external ribs and foot pads make the lower half read as a rugged case.
    for i, y in enumerate((-0.285, -0.078, 0.078, 0.285)):
        lower.visual(Box((0.022, 0.048, LOWER_HEIGHT - 0.026)), origin=Origin(xyz=(FRONT_X + 0.008, y, 0.084)), material=molded_edge, name=f"front_rib_{i}")

    # Fixed rear hinge leaf and alternating parent-side knuckles.
    for i, y in enumerate((-0.225, -0.045, 0.135)):
        lower.visual(Box((0.026, 0.080, 0.045)), origin=Origin(xyz=(REAR_X - 0.011, y, LOWER_HEIGHT - 0.020)), material=steel, name=f"rear_hinge_leaf_{i}")
        lower.visual(
            Cylinder(radius=0.013, length=0.075),
            origin=Origin(xyz=(HINGE_X, y, LOWER_HEIGHT), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"rear_knuckle_{i}",
        )

    # Front hardware mounting plates and bosses for latches and handle.
    latch_ys = (-0.205, 0.205)
    for i, y in enumerate(latch_ys):
        lower.visual(Box((0.020, 0.075, 0.070)), origin=Origin(xyz=(FRONT_X + 0.006, y, LOWER_HEIGHT - 0.047)), material=steel, name=f"latch_base_{i}")
        lower.visual(
            Cylinder(radius=0.013, length=0.066),
            origin=Origin(xyz=(FRONT_X + 0.014, y, LOWER_HEIGHT - 0.055), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"latch_boss_{i}",
        )

    lower.visual(Box((0.012, 0.165, 0.034)), origin=Origin(xyz=(FRONT_X + 0.008, 0.0, 0.130)), material=molded_edge, name="handle_recess")
    for i, y in enumerate((-0.075, 0.075)):
        lower.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(xyz=(FRONT_X + 0.016, y, 0.130), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"handle_boss_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        _open_shell_mesh(
            "shallow_lid_shell",
            CASE_DEPTH,
            CASE_WIDTH,
            LID_HEIGHT,
            open_face="<Z",
            x_offset=CASE_DEPTH / 2.0 + HINGE_STANDOFF,
            fillet=0.030,
        ),
        material=plastic,
        name="lid_shell",
    )
    lid.visual(Box((CASE_DEPTH - 0.070, 0.034, 0.016)), origin=Origin(xyz=(CASE_DEPTH / 2.0 + HINGE_STANDOFF, 0.0, LID_HEIGHT + 0.006)), material=molded_edge, name="top_center_rib")
    for i, y in enumerate((-0.245, 0.245)):
        lid.visual(Box((CASE_DEPTH - 0.095, 0.030, 0.013)), origin=Origin(xyz=(CASE_DEPTH / 2.0 + HINGE_STANDOFF, y, LID_HEIGHT + 0.005)), material=molded_edge, name=f"top_side_rib_{i}")
    lid.visual(Box((0.010, CASE_WIDTH - 0.060, 0.040)), origin=Origin(xyz=(0.016, 0.0, 0.025)), material=steel, name="lid_hinge_leaf")
    for i, y in enumerate((-0.135, 0.045, 0.225)):
        lid.visual(
            Cylinder(radius=0.0125, length=0.075),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"lid_knuckle_{i}",
        )
    for i, y in enumerate(latch_ys):
        lid.visual(Box((0.009, 0.082, 0.045)), origin=Origin(xyz=(CASE_DEPTH + HINGE_STANDOFF + 0.0025, y, 0.032)), material=steel, name=f"latch_keeper_{i}")
        lid.visual(Box((0.020, 0.060, 0.010)), origin=Origin(xyz=(CASE_DEPTH + HINGE_STANDOFF + 0.0050, y, 0.008)), material=steel, name=f"keeper_lip_{i}")

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, LOWER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.95),
    )

    for i, y in enumerate(latch_ys):
        latch = model.part(f"latch_{i}")
        latch.visual(Box((0.010, 0.042, 0.098)), origin=Origin(xyz=(0.013, 0.0, 0.049)), material=steel, name="draw_lever")
        latch.visual(Box((0.020, 0.052, 0.018)), origin=Origin(xyz=(0.002, 0.0, 0.093)), material=steel, name="overcenter_hook")
        latch.visual(
            Cylinder(radius=0.010, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="pivot_barrel",
        )
        latch.visual(Box((0.014, 0.048, 0.020)), origin=Origin(xyz=(0.013, 0.0, 0.020)), material=rubber, name="finger_pad")
        model.articulation(
            f"latch_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(FRONT_X + 0.024, y, LOWER_HEIGHT - 0.055)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.20),
        )

    handle = model.part("handle")
    handle.visual(Box((0.018, 0.018, 0.088)), origin=Origin(xyz=(0.0, -0.052, -0.044)), material=rubber, name="handle_arm_0")
    handle.visual(Box((0.018, 0.018, 0.088)), origin=Origin(xyz=(0.0, 0.052, -0.044)), material=rubber, name="handle_arm_1")
    handle.visual(Box((0.022, 0.122, 0.022)), origin=Origin(xyz=(0.0, 0.0, -0.088)), material=rubber, name="grip_bar")
    handle.visual(
        Cylinder(radius=0.010, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_pivot",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=handle,
        origin=Origin(xyz=(FRONT_X + 0.027, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")
    handle = object_model.get_part("handle")

    for i in (0, 1):
        latch = object_model.get_part(f"latch_{i}")
        ctx.allow_overlap(
            lower,
            latch,
            elem_a=f"latch_boss_{i}",
            elem_b="pivot_barrel",
            reason="The draw-latch pivot barrel is intentionally captured inside the fixed front boss.",
        )
        ctx.expect_overlap(
            lower,
            latch,
            axes="yz",
            elem_a=f"latch_boss_{i}",
            elem_b="pivot_barrel",
            min_overlap=0.015,
            name=f"latch {i} pivot is retained in its boss",
        )

    for i in (0, 1):
        ctx.allow_overlap(
            lower,
            handle,
            elem_a=f"handle_boss_{i}",
            elem_b="handle_pivot",
            reason="The carry-handle axle is intentionally nested through the fixed molded pivot boss.",
        )
        ctx.expect_overlap(
            lower,
            handle,
            axes="yz",
            elem_a=f"handle_boss_{i}",
            elem_b="handle_pivot",
            min_overlap=0.014,
            name=f"handle axle is retained in boss {i}",
        )

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        name="closed lid seats on lower shell rim",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.30,
        name="lid footprint matches the lower shell",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({rear_hinge: 1.55}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "rear hinge swings lid upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    for i in (0, 1):
        latch = object_model.get_part(f"latch_{i}")
        joint = object_model.get_articulation(f"latch_pivot_{i}")
        ctx.expect_gap(
            latch,
            lid,
            axis="x",
            min_gap=0.0005,
            max_gap=0.012,
            positive_elem="overcenter_hook",
            negative_elem=f"latch_keeper_{i}",
            name=f"latch {i} hook sits just outside the keeper",
        )
        closed = ctx.part_world_aabb(latch)
        with ctx.pose({joint: 0.95}):
            released = ctx.part_world_aabb(latch)
        ctx.check(
            f"latch {i} rotates outward to release",
            closed is not None and released is not None and released[1][0] > closed[1][0] + 0.035,
            details=f"closed={closed}, released={released}",
        )

    handle_joint = object_model.get_articulation("handle_pivot")
    stowed = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.10}):
        lifted = ctx.part_world_aabb(handle)
    ctx.check(
        "center carry handle pivots out from the front edge",
        stowed is not None and lifted is not None and lifted[1][0] > stowed[1][0] + 0.045,
        details=f"stowed={stowed}, lifted={lifted}",
    )

    return ctx.report()


object_model = build_object_model()
