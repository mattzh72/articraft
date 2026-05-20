from __future__ import annotations

import math

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
import cadquery as cq


def _hinge_barrel_mesh(length: float, outer_radius: float, inner_radius: float, name: str):
    """A centered hollow hinge eye with a vertical pin bore."""
    barrel = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(barrel, name, tolerance=0.0007, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_garden_gate")

    painted_wood = Material("painted_wood", rgba=(0.36, 0.50, 0.36, 1.0))
    end_grain = Material("slightly_worn_edges", rgba=(0.46, 0.62, 0.43, 1.0))
    dark_metal = Material("dark_galvanized_hardware", rgba=(0.05, 0.055, 0.05, 1.0))
    shadow_gap = Material("panel_shadow", rgba=(0.20, 0.28, 0.20, 1.0))

    posts = model.part("posts")
    posts.visual(Box((0.12, 0.12, 1.05)), origin=Origin(xyz=(0.0, 0.0, 0.525)), material=painted_wood, name="hinge_post")
    posts.visual(Box((0.12, 0.12, 1.05)), origin=Origin(xyz=(1.25, 0.0, 0.525)), material=painted_wood, name="latch_post")
    posts.visual(Box((1.37, 0.08, 0.06)), origin=Origin(xyz=(0.625, 0.0, 0.03)), material=painted_wood, name="low_sill")
    posts.visual(Box((0.16, 0.16, 0.035)), origin=Origin(xyz=(0.0, 0.0, 1.0675)), material=end_grain, name="hinge_post_cap")
    posts.visual(Box((0.16, 0.16, 0.035)), origin=Origin(xyz=(1.25, 0.0, 1.0675)), material=end_grain, name="latch_post_cap")

    for name, z in (("lower", 0.285), ("upper", 0.735)):
        posts.visual(Box((0.075, 0.010, 0.145)), origin=Origin(xyz=(0.020, -0.064, z)), material=dark_metal, name=f"{name}_post_leaf")
        posts.visual(Box((0.040, 0.020, 0.026)), origin=Origin(xyz=(0.073, -0.075, z - 0.070)), material=dark_metal, name=f"{name}_post_tab_0")
        posts.visual(Box((0.040, 0.020, 0.026)), origin=Origin(xyz=(0.073, -0.075, z + 0.070)), material=dark_metal, name=f"{name}_post_tab_1")
        posts.visual(Cylinder(radius=0.009, length=0.155), origin=Origin(xyz=(0.085, -0.075, z)), material=dark_metal, name=f"{name}_pin")

    posts.visual(Box((0.035, 0.020, 0.155)), origin=Origin(xyz=(1.183, -0.033, 0.58)), material=dark_metal, name="keeper_plate")
    posts.visual(Box((0.018, 0.030, 0.070)), origin=Origin(xyz=(1.178, -0.045, 0.58)), material=dark_metal, name="keeper_lip")

    leaf = model.part("leaf")
    # The leaf frame is the hinge pin line at ground level.  The gate body extends along +X.
    leaf.visual(Box((0.070, 0.048, 0.860)), origin=Origin(xyz=(0.065, 0.075, 0.510)), material=painted_wood, name="hinge_stile")
    leaf.visual(Box((0.070, 0.048, 0.860)), origin=Origin(xyz=(1.035, 0.075, 0.510)), material=painted_wood, name="free_stile")
    leaf.visual(Box((1.040, 0.055, 0.080)), origin=Origin(xyz=(0.550, 0.075, 0.120)), material=painted_wood, name="bottom_rail")
    leaf.visual(Box((1.040, 0.055, 0.070)), origin=Origin(xyz=(0.550, 0.075, 0.480)), material=painted_wood, name="middle_rail")
    leaf.visual(Box((1.040, 0.055, 0.075)), origin=Origin(xyz=(0.550, 0.075, 0.905)), material=painted_wood, name="top_rail")
    leaf.visual(Box((0.850, 0.030, 0.330)), origin=Origin(xyz=(0.550, 0.078, 0.300)), material=painted_wood, name="solid_panel")
    leaf.visual(Box((0.800, 0.006, 0.020)), origin=Origin(xyz=(0.550, 0.050, 0.475)), material=shadow_gap, name="panel_reveal")

    for i, x in enumerate((0.255, 0.405, 0.555, 0.705, 0.855)):
        leaf.visual(Box((0.026, 0.036, 0.385)), origin=Origin(xyz=(x, 0.075, 0.690)), material=painted_wood, name=f"lattice_picket_{i}")

    diag_angle = -math.atan2(0.34, 0.82)
    leaf.visual(Box((0.900, 0.034, 0.026)), origin=Origin(xyz=(0.550, 0.075, 0.690), rpy=(0.0, diag_angle, 0.0)), material=painted_wood, name="lattice_diagonal_0")
    leaf.visual(Box((0.900, 0.034, 0.026)), origin=Origin(xyz=(0.550, 0.075, 0.690), rpy=(0.0, -diag_angle, 0.0)), material=painted_wood, name="lattice_diagonal_1")

    barrel_mesh = _hinge_barrel_mesh(0.105, 0.021, 0.0085, "gate_hinge_barrel")
    for name, z in (("lower", 0.285), ("upper", 0.735)):
        leaf.visual(barrel_mesh, origin=Origin(xyz=(0.0, 0.0, z)), material=dark_metal, name=f"{name}_barrel")
        leaf.visual(Box((0.238, 0.055, 0.036)), origin=Origin(xyz=(0.137, 0.025, z)), material=dark_metal, name=f"{name}_strap")
        leaf.visual(Box((0.040, 0.048, 0.020)), origin=Origin(xyz=(0.245, 0.052, z + 0.020)), material=dark_metal, name=f"{name}_strap_bolt_0")
        leaf.visual(Box((0.040, 0.048, 0.020)), origin=Origin(xyz=(0.245, 0.052, z - 0.020)), material=dark_metal, name=f"{name}_strap_bolt_1")

    leaf.visual(Cylinder(radius=0.045, length=0.012), origin=Origin(xyz=(0.975, 0.046, 0.580), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="latch_rosette")

    latch_handle = model.part("latch_handle")
    latch_handle.visual(Cylinder(radius=0.034, length=0.018), origin=Origin(xyz=(0.0, -0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=dark_metal, name="pivot_cap")
    latch_handle.visual(Box((0.120, 0.020, 0.034)), origin=Origin(xyz=(0.040, -0.012, 0.0)), material=dark_metal, name="turn_bar")
    latch_handle.visual(Box((0.022, 0.022, 0.050)), origin=Origin(xyz=(0.090, -0.012, 0.0)), material=dark_metal, name="hook_nose")

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(0.085, -0.075, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=latch_handle,
        origin=Origin(xyz=(0.975, 0.034, 0.580)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    latch_handle = object_model.get_part("latch_handle")
    gate_hinge = object_model.get_articulation("gate_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.check(
        "gate leaf uses a vertical revolute hinge",
        gate_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(gate_hinge.axis) == (0.0, 0.0, 1.0)
        and gate_hinge.motion_limits.lower == 0.0
        and gate_hinge.motion_limits.upper >= 1.2,
        details=f"type={gate_hinge.articulation_type}, axis={gate_hinge.axis}, limits={gate_hinge.motion_limits}",
    )
    ctx.check(
        "latch handle has its own compact rotary pivot",
        latch_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(latch_pivot.axis) == (0.0, -1.0, 0.0)
        and latch_pivot.motion_limits.upper <= 1.2,
        details=f"type={latch_pivot.articulation_type}, axis={latch_pivot.axis}, limits={latch_pivot.motion_limits}",
    )

    for hinge_name in ("lower", "upper"):
        ctx.allow_overlap(
            posts,
            leaf,
            elem_a=f"{hinge_name}_pin",
            elem_b=f"{hinge_name}_barrel",
            reason="The hinge pin is intentionally captured in the gate-side barrel eye so the leaf stays clipped to the post while swinging.",
        )
        ctx.expect_within(
            posts,
            leaf,
            axes="xy",
            inner_elem=f"{hinge_name}_pin",
            outer_elem=f"{hinge_name}_barrel",
            margin=0.0,
            name=f"{hinge_name} hinge pin stays captured in barrel",
        )
        ctx.expect_overlap(
            posts,
            leaf,
            axes="z",
            elem_a=f"{hinge_name}_pin",
            elem_b=f"{hinge_name}_barrel",
            min_overlap=0.095,
            name=f"{hinge_name} hinge has retained pin length",
        )

    closed_free = ctx.part_element_world_aabb(leaf, elem="free_stile")
    closed_latch = ctx.part_element_world_aabb(latch_handle, elem="turn_bar")

    with ctx.pose({gate_hinge: 1.20}):
        open_free = ctx.part_element_world_aabb(leaf, elem="free_stile")
        for hinge_name in ("lower", "upper"):
            ctx.expect_within(
                posts,
                leaf,
                axes="xy",
                inner_elem=f"{hinge_name}_pin",
                outer_elem=f"{hinge_name}_barrel",
                margin=0.0,
                name=f"{hinge_name} hinge remains clipped while open",
            )

    with ctx.pose({latch_pivot: 1.0}):
        turned_latch = ctx.part_element_world_aabb(latch_handle, elem="turn_bar")

    def _center_y(aabb):
        return (aabb[0][1] + aabb[1][1]) / 2.0 if aabb is not None else None

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0 if aabb is not None else None

    ctx.check(
        "free edge swings outward rather than separating from hinge",
        closed_free is not None and open_free is not None and _center_y(open_free) > _center_y(closed_free) + 0.75,
        details=f"closed={closed_free}, open={open_free}",
    )
    ctx.check(
        "rotary latch handle visibly turns upward",
        closed_latch is not None and turned_latch is not None and _center_z(turned_latch) > _center_z(closed_latch) + 0.030,
        details=f"closed={closed_latch}, turned={turned_latch}",
    )

    return ctx.report()


object_model = build_object_model()
