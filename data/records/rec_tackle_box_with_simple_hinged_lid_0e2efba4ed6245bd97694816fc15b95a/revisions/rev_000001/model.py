from __future__ import annotations

import math

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


SHELL_W = 0.460
SHELL_D = 0.240
SHELL_H = 0.170
WALL = 0.018
RIM_H = 0.010
HINGE_Y = SHELL_D / 2.0 + 0.030
HINGE_Z = SHELL_H + RIM_H + 0.015
LID_D = 0.300
LID_W = 0.500
LID_FRONT_Y = -0.295
LATCH_XS = (-0.130, 0.130)


def _ring_box(width: float, depth: float, inner_width: float, inner_depth: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    cutter = (
        cq.Workplane("XY")
        .box(inner_width, inner_depth, height + 0.004, centered=(True, True, False))
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(cutter)


def _make_shell_body() -> cq.Workplane:
    outer = cq.Workplane("XY").box(SHELL_W, SHELL_D, SHELL_H, centered=(True, True, False))
    try:
        outer = outer.edges("|Z").fillet(0.018)
    except Exception:
        # Keep the primary hollow form even if an edge fillet cannot be solved.
        pass

    inner = (
        cq.Workplane("XY")
        .box(SHELL_W - 2.0 * WALL, SHELL_D - 2.0 * WALL, SHELL_H + 0.020, centered=(True, True, False))
        .translate((0.0, 0.0, WALL))
    )
    body = outer.cut(inner)

    rim = _ring_box(SHELL_W + 0.012, SHELL_D + 0.012, SHELL_W - 0.062, SHELL_D - 0.062, RIM_H).translate(
        (0.0, 0.0, SHELL_H)
    )
    body = body.union(rim)

    # Molded ribs and rubber feet are fused into the plastic shell so every
    # visible detail has a hard path back into the tub wall.
    front_rib = (
        cq.Workplane("XY")
        .box(SHELL_W - 0.055, 0.010, 0.022, centered=True)
        .translate((0.0, -SHELL_D / 2.0 - 0.004, 0.090))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(SHELL_W - 0.055, 0.010, 0.018, centered=True)
        .translate((0.0, SHELL_D / 2.0 + 0.004, 0.082))
    )
    body = body.union(front_rib).union(rear_rib)

    for x in (-0.150, 0.150):
        foot = cq.Workplane("XY").box(0.115, 0.048, 0.010, centered=True).translate((x, -0.052, 0.005))
        body = body.union(foot)

    return body


def _make_lid_body() -> cq.Workplane:
    lid_center_y = -0.153
    deck = (
        cq.Workplane("XY")
        .box(LID_W, 0.286, 0.026, centered=(True, True, False))
        .translate((0.0, lid_center_y, 0.0))
    )

    crown = (
        cq.Workplane("XY")
        .box(0.360, 0.145, 0.016, centered=(True, True, False))
        .translate((0.0, lid_center_y, 0.026))
    )
    body = deck.union(crown)

    # Downturned skirt and drip beads: the skirt overhangs the base rim and the
    # lower beads make the closed seam shed rain before it can wick inward.
    skirts = [
        cq.Workplane("XY").box(0.012, 0.286, 0.037, centered=True).translate((-LID_W / 2.0 + 0.006, lid_center_y, -0.017)),
        cq.Workplane("XY").box(0.012, 0.286, 0.037, centered=True).translate((LID_W / 2.0 - 0.006, lid_center_y, -0.017)),
        cq.Workplane("XY").box(LID_W, 0.012, 0.037, centered=True).translate((0.0, LID_FRONT_Y, -0.017)),
        cq.Workplane("XY").box(LID_W + 0.018, 0.010, 0.008, centered=True).translate((0.0, LID_FRONT_Y - 0.007, -0.034)),
        cq.Workplane("XY").box(0.010, 0.304, 0.008, centered=True).translate((-LID_W / 2.0 - 0.001, lid_center_y, -0.034)),
        cq.Workplane("XY").box(0.010, 0.304, 0.008, centered=True).translate((LID_W / 2.0 + 0.001, lid_center_y, -0.034)),
    ]
    for skirt in skirts:
        body = body.union(skirt)

    # Integrated low carry grip and front latch bosses.
    grip = cq.Workplane("XY").box(0.210, 0.042, 0.018, centered=True).translate((0.0, lid_center_y, 0.049))
    body = body.union(grip)
    for x in LATCH_XS:
        boss = cq.Workplane("XY").box(0.082, 0.016, 0.024, centered=True).translate((x, LID_FRONT_Y - 0.002, -0.010))
        body = body.union(boss)

    return body


def _make_gasket() -> cq.Workplane:
    gasket = _ring_box(0.438, 0.222, 0.396, 0.180, 0.014)
    return gasket.translate((0.0, -HINGE_Y, -0.014))


def _make_latch_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.060, 0.006, 0.080, centered=True).translate((0.0, -0.006, -0.040))
    slot = cq.Workplane("XY").box(0.032, 0.020, 0.026, centered=True).translate((0.0, -0.006, -0.055))
    plate = plate.cut(slot)
    hook_lip = cq.Workplane("XY").box(0.046, 0.012, 0.008, centered=True).translate((0.0, -0.010, -0.078))
    barrel = cq.Workplane("YZ").circle(0.006).extrude(0.062).translate((-0.031, 0.0, 0.0))
    return plate.union(hook_lip).union(barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_tackle_box")

    model.material("olive_uv_stabilized_polymer", rgba=(0.12, 0.22, 0.15, 1.0))
    model.material("dark_lid_polymer", rgba=(0.07, 0.14, 0.10, 1.0))
    model.material("black_epdm_gasket", rgba=(0.005, 0.006, 0.005, 1.0))
    model.material("marine_stainless", rgba=(0.74, 0.76, 0.72, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_make_shell_body(), "shell_body", tolerance=0.0008, angular_tolerance=0.08),
        material="olive_uv_stabilized_polymer",
        name="shell_body",
    )

    # Fixed corrosion-resistant keeper hardware is backed by broad plates and
    # protruding posts that pass through the latch slots.
    for idx, x in enumerate(LATCH_XS):
        keeper = model.part(f"keeper_{idx}")
        keeper.visual(
            Box((0.074, 0.010, 0.034)),
            origin=Origin(xyz=(x, -SHELL_D / 2.0 - 0.005, 0.136)),
            material="marine_stainless",
            name="keeper_plate",
        )
        keeper.visual(
            Cylinder(radius=0.007, length=0.038),
            origin=Origin(xyz=(x, -SHELL_D / 2.0 - 0.020, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="marine_stainless",
            name="keeper_post",
        )
        model.articulation(
            f"shell_to_keeper_{idx}",
            ArticulationType.FIXED,
            parent=shell,
            child=keeper,
            origin=Origin(),
        )

    # The hinge base has a continuous leaf tied into the rear wall and two
    # outer knuckles, leaving a protected center gap for the moving lid knuckle.
    hinge_base = model.part("hinge_base")
    hinge_base.visual(
        Box((0.430, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, SHELL_D / 2.0 + 0.004, SHELL_H - 0.030)),
        material="marine_stainless",
        name="hinge_base_leaf",
    )
    for idx, x in enumerate((-0.145, 0.145)):
        hinge_base.visual(
            Box((0.125, 0.024, 0.006)),
            origin=Origin(xyz=(x, SHELL_D / 2.0 + 0.018, SHELL_H - 0.019)),
            material="marine_stainless",
            name=f"hinge_base_bridge_{idx}",
        )
        hinge_base.visual(
            Box((0.125, 0.008, 0.048)),
            origin=Origin(xyz=(x, SHELL_D / 2.0 + 0.034, SHELL_H + 0.002)),
            material="marine_stainless",
            name=f"hinge_base_web_{idx}",
        )
        hinge_base.visual(
            Cylinder(radius=0.0065, length=0.145),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="marine_stainless",
            name=f"hinge_base_knuckle_{idx}",
        )
    model.articulation(
        "shell_to_hinge_base",
        ArticulationType.FIXED,
        parent=shell,
        child=hinge_base,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_body(), "lid_body", tolerance=0.0008, angular_tolerance=0.08),
        material="dark_lid_polymer",
        name="lid_body",
    )
    lid.visual(
        mesh_from_cadquery(_make_gasket(), "gasket_ring", tolerance=0.0006, angular_tolerance=0.08),
        material="black_epdm_gasket",
        name="gasket_ring",
    )
    lid.visual(
        Box((0.145, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.003, -0.012)),
        material="marine_stainless",
        name="hinge_lid_leaf",
    )
    lid.visual(
        Cylinder(radius=0.0063, length=0.145),
        origin=Origin(xyz=(0.0, HINGE_Y - HINGE_Y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="marine_stainless",
        name="hinge_lid_knuckle",
    )
    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    latch_mesh = mesh_from_cadquery(_make_latch_plate(), "latch_plate", tolerance=0.0006, angular_tolerance=0.08)
    for idx, x in enumerate(LATCH_XS):
        latch = model.part(f"latch_{idx}")
        latch.visual(
            latch_mesh,
            material="marine_stainless",
            name="latch_plate",
        )
        latch.visual(
            Box((0.052, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, -0.010, -0.076)),
            material="black_rubber",
            name="thumb_pad",
        )
        model.articulation(
            f"latch_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, LID_FRONT_Y - 0.016, -0.005)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=1.25),
        )

    # Store key joints for inspection without changing the public link names.
    model.meta["primary_lid_joint"] = lid_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    hinge_base = object_model.get_part("hinge_base")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.allow_overlap(
        hinge_base,
        shell,
        elem_a="hinge_base_leaf",
        elem_b="shell_body",
        reason="The stainless rear hinge leaf is screw-seated into the molded rear shell boss.",
    )
    ctx.expect_gap(
        hinge_base,
        shell,
        axis="y",
        positive_elem="hinge_base_leaf",
        negative_elem="shell_body",
        max_penetration=0.012,
        name="hinge leaf is locally seated in rear shell boss",
    )
    for idx in range(2):
        keeper = object_model.get_part(f"keeper_{idx}")
        ctx.allow_overlap(
            shell,
            keeper,
            elem_a="shell_body",
            elem_b="keeper_plate",
            reason="The keeper strike plate is screw-seated into the molded front shell boss.",
        )
        ctx.expect_gap(
            shell,
            keeper,
            axis="y",
            positive_elem="shell_body",
            negative_elem="keeper_plate",
            max_penetration=0.012,
            name=f"keeper_{idx} plate is locally seated in front shell boss",
        )

    # Closed weather seal: the EPDM ring is seated immediately over the raised
    # shell rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="gasket_ring",
            negative_elem="shell_body",
            min_gap=0.0,
            max_gap=0.003,
            name="gasket seats above raised rim",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="gasket_ring",
            elem_b="shell_body",
            min_overlap=0.16,
            name="gasket ring covers the shell opening",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_body")
    with ctx.pose({lid_hinge: 1.2}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_body")
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_body",
            negative_elem="shell_body",
            min_gap=0.001,
            name="lid clears the shell when opened",
        )
    ctx.check(
        "lid hinge has realistic bounded travel",
        lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and 1.4 <= lid_hinge.motion_limits.upper <= 1.9,
        details=f"limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "lid free edge rises when opened",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    for idx in range(2):
        latch = object_model.get_part(f"latch_{idx}")
        keeper = object_model.get_part(f"keeper_{idx}")
        joint = object_model.get_articulation(f"latch_hinge_{idx}")
        ctx.expect_overlap(
            latch,
            keeper,
            axes="xz",
            elem_a="latch_plate",
            elem_b="keeper_post",
            min_overlap=0.006,
            name=f"latch_{idx} slot aligns with keeper post",
        )
        closed_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
        with ctx.pose({joint: 0.95}):
            open_aabb = ctx.part_element_world_aabb(latch, elem="latch_plate")
        ctx.check(
            f"latch_{idx} swings outward",
            closed_aabb is not None and open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.015,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        ctx.check(
            f"latch_{idx} is bounded by its hinge hardware",
            joint.motion_limits is not None and joint.motion_limits.lower == 0.0 and 0.9 <= (joint.motion_limits.upper or 0.0) <= 1.4,
            details=f"limits={joint.motion_limits}",
        )

    return ctx.report()


object_model = build_object_model()
