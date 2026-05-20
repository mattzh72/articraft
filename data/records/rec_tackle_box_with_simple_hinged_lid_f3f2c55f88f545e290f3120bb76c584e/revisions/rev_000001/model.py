from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.46
BODY_D = 0.24
BODY_H = 0.17
LID_H = 0.070
WALL = 0.008

HINGE_Y = BODY_D / 2.0 + 0.012
HINGE_Z = BODY_H + 0.012


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _body_shell() -> cq.Workplane:
    """Open rugged lower tub with radiused exterior and a real visible cavity."""

    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.018)
    )
    inner = _box(
        (BODY_W - 2.0 * WALL, BODY_D - 2.0 * WALL, BODY_H + 0.030),
        (0.0, 0.0, WALL + (BODY_H + 0.030) / 2.0),
    )
    shell = outer.cut(inner)

    # Molded external rub rails and a thick upper rim are unioned into the tub.
    shell = shell.union(_box((BODY_W - 0.045, 0.010, 0.014), (0.0, -BODY_D / 2.0 - 0.003, 0.058)))
    shell = shell.union(_box((BODY_W - 0.045, 0.010, 0.014), (0.0, -BODY_D / 2.0 - 0.003, 0.122)))
    shell = shell.union(_box((BODY_W - 0.055, 0.010, 0.012), (0.0, BODY_D / 2.0 + 0.003, 0.082)))
    for x in (-BODY_W / 2.0 - 0.003, BODY_W / 2.0 + 0.003):
        shell = shell.union(_box((0.010, BODY_D - 0.060, 0.012), (x, 0.0, 0.074)))
        shell = shell.union(_box((0.010, BODY_D - 0.060, 0.012), (x, 0.0, 0.132)))
    return shell


def _lid_shell() -> cq.Workplane:
    """Shallow hollow lid modeled in its hinge-line frame."""

    lid_w = BODY_W + 0.018
    lid_d = BODY_D + 0.035
    bottom = -0.010
    outer = (
        cq.Workplane("XY")
        .box(lid_w, lid_d, LID_H)
        .translate((0.0, -BODY_D / 2.0, bottom + LID_H / 2.0))
        .edges("|Z")
        .fillet(0.016)
    )
    cutter_h = LID_H - WALL + 0.010
    cutter = _box(
        (lid_w - 2.0 * WALL, lid_d - 2.0 * WALL, cutter_h),
        (0.0, -BODY_D / 2.0, bottom - 0.010 + cutter_h / 2.0),
    )
    shell = outer.cut(cutter)
    # Raised top ribs make the lid read like molded outdoor gear.
    shell = shell.union(_box((lid_w - 0.080, 0.012, 0.010), (0.0, -BODY_D / 2.0, bottom + LID_H + 0.002)))
    shell = shell.union(_box((0.012, lid_d - 0.090, 0.010), (-0.115, -BODY_D / 2.0, bottom + LID_H + 0.002)))
    shell = shell.union(_box((0.012, lid_d - 0.090, 0.010), (0.115, -BODY_D / 2.0, bottom + LID_H + 0.002)))
    return shell


def _organizer_tray() -> cq.Workplane:
    """Shallow pivoting divided tray; origin is at its rear hinge pin."""

    tray_w = 0.360
    tray_d = 0.170
    tray_h = 0.035
    t = 0.004
    tray = _box((tray_w, tray_d, t), (0.0, -tray_d / 2.0, -tray_h + t / 2.0))
    tray = tray.union(_box((tray_w, t, tray_h), (0.0, -tray_d + t / 2.0, -tray_h / 2.0)))
    tray = tray.union(_box((tray_w, t, tray_h), (0.0, -t / 2.0, -tray_h / 2.0)))
    tray = tray.union(_box((t, tray_d, tray_h), (-tray_w / 2.0 + t / 2.0, -tray_d / 2.0, -tray_h / 2.0)))
    tray = tray.union(_box((t, tray_d, tray_h), (tray_w / 2.0 - t / 2.0, -tray_d / 2.0, -tray_h / 2.0)))
    # Compartment dividers of varied sizes for lures, weights, and hooks.
    tray = tray.union(_box((t, tray_d - 2.0 * t, tray_h - 0.004), (-0.065, -tray_d / 2.0, -tray_h / 2.0 - 0.002)))
    tray = tray.union(_box((t, tray_d - 2.0 * t, tray_h - 0.004), (0.075, -tray_d / 2.0, -tray_h / 2.0 - 0.002)))
    tray = tray.union(_box((tray_w - 2.0 * t, t, tray_h - 0.007), (0.0, -0.070, -tray_h / 2.0 - 0.0035)))
    tray = tray.union(_box((0.130, t, tray_h - 0.007), (0.145, -0.122, -tray_h / 2.0 - 0.0035)))
    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_tackle_box")

    body_mat = model.material("olive_impact_plastic", rgba=(0.18, 0.30, 0.20, 1.0))
    lid_mat = model.material("weathered_lid_plastic", rgba=(0.23, 0.38, 0.24, 1.0))
    dark_mat = model.material("black_rubber_trim", rgba=(0.025, 0.028, 0.025, 1.0))
    latch_mat = model.material("safety_orange_latches", rgba=(0.95, 0.42, 0.07, 1.0))
    tray_mat = model.material("smoky_translucent_tray", rgba=(0.62, 0.72, 0.64, 0.58))
    metal_mat = model.material("dull_stainless_pin", rgba=(0.62, 0.63, 0.60, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell(), "body_shell"), material=body_mat, name="body_shell")
    body.inertial = Inertial.from_geometry(Box((BODY_W, BODY_D, BODY_H)), mass=2.4, origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)))

    # Rubber feet and molded corner guards.
    for i, x in enumerate((-0.155, 0.155)):
        for j, y in enumerate((-0.078, 0.078)):
            body.visual(
                Box((0.090, 0.052, 0.014)),
                origin=Origin(xyz=(x, y, -0.005)),
                material=dark_mat,
                name=f"foot_{i}_{j}",
            )
    for x in (-BODY_W / 2.0 - 0.004, BODY_W / 2.0 + 0.004):
        for y in (-BODY_D / 2.0 + 0.030, BODY_D / 2.0 - 0.030):
            body.visual(
                Box((0.014, 0.044, 0.120)),
                origin=Origin(xyz=(x, y, 0.073)),
                material=dark_mat,
                name=f"corner_guard_{'p' if x > 0 else 'n'}_{'r' if y > 0 else 'f'}",
            )

    # Rear fixed hinge knuckles and leaves attached to the tub.
    body.visual(
        Cylinder(radius=0.0038, length=0.435),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="main_hinge_pin",
    )
    for idx, x in enumerate((-0.170, 0.0, 0.170)):
        body.visual(
            Cylinder(radius=0.0075, length=0.070),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"body_hinge_barrel_{idx}",
        )
        body.visual(
            Box((0.072, 0.008, 0.028)),
            origin=Origin(xyz=(x, BODY_D / 2.0 + 0.003, BODY_H - 0.004)),
            material=dark_mat,
            name=f"body_hinge_leaf_{idx}",
        )

    # Latch hinge bases on the front of the lower box.
    for idx, x in enumerate((-0.115, 0.115)):
        body.visual(
            Box((0.074, 0.010, 0.025)),
            origin=Origin(xyz=(x, -BODY_D / 2.0 - 0.004, BODY_H - 0.018)),
            material=dark_mat,
            name=f"latch_base_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0030, length=0.084),
            origin=Origin(xyz=(x, -BODY_D / 2.0 - 0.015, BODY_H + 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"latch_axle_{idx}",
        )
        for side, dx in enumerate((-0.038, 0.038)):
            body.visual(
                Box((0.010, 0.014, 0.040)),
                origin=Origin(xyz=(x + dx, -BODY_D / 2.0 - 0.010, BODY_H + 0.009)),
                material=dark_mat,
                name=f"latch_ear_{idx}_{side}",
            )

    # Fixed axle and side brackets for the pivoting internal organizer tray.
    body.visual(
        Cylinder(radius=0.0032, length=0.430),
        origin=Origin(xyz=(0.0, 0.075, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="tray_axle",
    )
    for idx, x in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.016, 0.064, 0.034)),
            origin=Origin(xyz=(x, 0.088, 0.132)),
            material=dark_mat,
            name=f"tray_bracket_{idx}",
        )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shell(), "lid_shell"), material=lid_mat, name="lid_shell")
    lid.inertial = Inertial.from_geometry(Box((BODY_W, BODY_D, LID_H)), mass=1.1, origin=Origin(xyz=(0.0, -BODY_D / 2.0, LID_H / 2.0)))

    # Moving half of the rear hinge, in alternating knuckles.
    for idx, x in enumerate((-0.085, 0.085)):
        lid.visual(
            Cylinder(radius=0.0070, length=0.070),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"lid_hinge_barrel_{idx}",
        )
        lid.visual(
            Box((0.070, 0.008, 0.026)),
            origin=Origin(xyz=(x, 0.009, 0.002)),
            material=dark_mat,
            name=f"lid_hinge_leaf_{idx}",
        )

    # Latch keeper hooks on the lid front and low pads for the folding handle.
    for idx, x in enumerate((-0.115, 0.115)):
        lid.visual(
            Box((0.065, 0.012, 0.018)),
            origin=Origin(xyz=(x, -BODY_D - 0.014, 0.005)),
            material=dark_mat,
            name=f"latch_keeper_{idx}",
        )
    for idx, x in enumerate((-0.130, 0.130)):
        lid.visual(
            Box((0.058, 0.032, 0.006)),
            origin=Origin(xyz=(x, -0.088, 0.061)),
            material=dark_mat,
            name=f"handle_pad_{idx}",
        )
        lid.visual(
            Cylinder(radius=0.0032, length=0.046),
            origin=Origin(xyz=(x, -0.088, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"handle_axle_{idx}",
        )
        lid.visual(
            Box((0.014, 0.014, 0.048)),
            origin=Origin(xyz=(x, -0.088, 0.070)),
            material=dark_mat,
            name=f"handle_stanchion_{idx}",
        )

    tray = model.part("organizer_tray")
    tray.visual(mesh_from_cadquery(_organizer_tray(), "organizer_tray"), material=tray_mat, name="tray_pan")
    tray.visual(
        Cylinder(radius=0.0050, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="tray_hinge_pin",
    )
    tray.inertial = Inertial.from_geometry(Box((0.360, 0.170, 0.035)), mass=0.35, origin=Origin(xyz=(0.0, -0.085, -0.018)))

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0080, length=0.110),
        origin=Origin(xyz=(-0.130, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="handle_arm_0",
    )
    handle.visual(
        Cylinder(radius=0.0080, length=0.110),
        origin=Origin(xyz=(0.130, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="handle_arm_1",
    )
    handle.visual(
        Cylinder(radius=0.0120, length=0.285),
        origin=Origin(xyz=(0.0, -0.110, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="handle_grip",
    )
    for idx, x in enumerate((-0.130, 0.130)):
        handle.visual(Sphere(radius=0.010), origin=Origin(xyz=(x, -0.110, 0.0)), material=dark_mat, name=f"handle_elbow_{idx}")
        handle.visual(
            Cylinder(radius=0.0060, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"handle_pivot_{idx}",
        )

    for idx, x in enumerate((-0.115, 0.115)):
        latch = model.part(f"latch_{idx}")
        latch.visual(
            Cylinder(radius=0.0055, length=0.064),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name="latch_pin",
        )
        latch.visual(
            Box((0.058, 0.008, 0.058)),
            origin=Origin(xyz=(0.0, -0.001, -0.034)),
            material=latch_mat,
            name="latch_plate",
        )
        latch.visual(
            Box((0.040, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.004, -0.063)),
            material=dark_mat,
            name="latch_pull_lip",
        )
        latch.inertial = Inertial.from_geometry(Box((0.060, 0.010, 0.070)), mass=0.08, origin=Origin(xyz=(0.0, 0.0, -0.035)))
        model.articulation(
            f"body_to_latch_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(x, -BODY_D / 2.0 - 0.015, BODY_H + 0.025)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.78, effort=2.0, velocity=3.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=8.0, velocity=1.4),
    )
    model.articulation(
        "body_to_organizer_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.075, 0.145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=3.0, velocity=1.0),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -0.088, 0.088)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("organizer_tray")
    handle = object_model.get_part("handle")
    latch_0 = object_model.get_part("latch_0")
    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_organizer_tray")
    handle_joint = object_model.get_articulation("lid_to_handle")
    latch_joint = object_model.get_articulation("body_to_latch_0")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.018,
        positive_elem="lid_shell",
        negative_elem="body_shell",
        name="closed lid has a narrow weather seal gap",
    )
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18, elem_a="lid_shell", elem_b="body_shell", name="lid covers the compact body footprint")
    ctx.expect_within(tray, body, axes="xy", margin=0.012, inner_elem="tray_pan", outer_elem="body_shell", name="organizer tray nests inside the tub")
    ctx.expect_overlap(tray, body, axes="z", min_overlap=0.020, elem_a="tray_pan", elem_b="body_shell", name="organizer tray sits down inside the storage cavity")

    for idx in (0, 1):
        ctx.allow_overlap(
            body,
            lid,
            elem_a="main_hinge_pin",
            elem_b=f"lid_hinge_barrel_{idx}",
            reason="The continuous metal hinge pin is intentionally captured inside the lid hinge barrel.",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a=f"lid_hinge_barrel_{idx}",
            elem_b="main_hinge_pin",
            min_overlap=0.055,
            name=f"lid hinge barrel {idx} remains captured on the main pin",
        )

        ctx.allow_overlap(
            body,
            f"latch_{idx}",
            elem_a=f"latch_axle_{idx}",
            elem_b="latch_pin",
            reason="The latch rotates around a small body-mounted axle represented as a captured pin.",
        )
        ctx.expect_overlap(
            f"latch_{idx}",
            body,
            axes="x",
            elem_a="latch_pin",
            elem_b=f"latch_axle_{idx}",
            min_overlap=0.055,
            name=f"latch {idx} pin is retained on its axle",
        )

        ctx.allow_overlap(
            lid,
            handle,
            elem_a=f"handle_axle_{idx}",
            elem_b=f"handle_pivot_{idx}",
            reason="The folding handle pivot is intentionally captured on the lid axle.",
        )
        ctx.allow_overlap(
            lid,
            handle,
            elem_a=f"handle_stanchion_{idx}",
            elem_b=f"handle_pivot_{idx}",
            reason="The handle pivot sits between molded stanchion cheeks at the axle.",
        )
        ctx.allow_overlap(
            lid,
            handle,
            elem_a=f"handle_stanchion_{idx}",
            elem_b=f"handle_arm_{idx}",
            reason="The handle arm begins at the pivot inside the molded stanchion pocket.",
        )
        ctx.expect_overlap(
            handle,
            lid,
            axes="x",
            elem_a=f"handle_pivot_{idx}",
            elem_b=f"handle_axle_{idx}",
            min_overlap=0.030,
            name=f"handle pivot {idx} is retained on its lid axle",
        )
        ctx.expect_overlap(
            handle,
            lid,
            axes="z",
            elem_a=f"handle_pivot_{idx}",
            elem_b=f"handle_stanchion_{idx}",
            min_overlap=0.006,
            name=f"handle pivot {idx} sits in the stanchion pocket",
        )
        ctx.expect_overlap(
            handle,
            lid,
            axes="z",
            elem_a=f"handle_arm_{idx}",
            elem_b=f"handle_stanchion_{idx}",
            min_overlap=0.006,
            name=f"handle arm {idx} begins inside the stanchion pocket",
        )

    ctx.allow_overlap(
        body,
        tray,
        elem_a="tray_axle",
        elem_b="tray_hinge_pin",
        reason="The organizer tray rotates on a body-mounted axle captured through its hinge sleeve.",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_hinge_pin",
        elem_b="tray_axle",
        min_overlap=0.350,
        name="organizer tray hinge sleeve is retained on the body axle",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid hinge lifts the front of the cover",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_tray_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tray_joint: 0.90}):
        open_tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "organizer tray pivots upward for access",
        closed_tray_aabb is not None
        and open_tray_aabb is not None
        and open_tray_aabb[1][2] > closed_tray_aabb[1][2] + 0.07,
        details=f"closed={closed_tray_aabb}, open={open_tray_aabb}",
    )

    flat_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: 1.05}):
        raised_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "folding handle rises above the lid",
        flat_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > flat_handle_aabb[1][2] + 0.07,
        details=f"flat={flat_handle_aabb}, raised={raised_handle_aabb}",
    )

    locked_latch_aabb = ctx.part_world_aabb(latch_0)
    with ctx.pose({latch_joint: 0.65}):
        released_latch_aabb = ctx.part_world_aabb(latch_0)
    ctx.check(
        "front latch flips outward from the body",
        locked_latch_aabb is not None
        and released_latch_aabb is not None
        and released_latch_aabb[0][1] < locked_latch_aabb[0][1] - 0.015,
        details=f"locked={locked_latch_aabb}, released={released_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
