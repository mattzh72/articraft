from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _box(dx: float, dy: float, dz: float, cx: float, cy: float, cz: float):
    return cq.Workplane("XY").box(dx, dy, dz).translate((cx, cy, cz))


def _x_cylinder(radius: float, length: float, x0: float, y: float, z: float):
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, y, z))


def _x_rect_cut(width_y: float, height_z: float, length: float, x0: float, y: float, z: float):
    return cq.Workplane("YZ").rect(width_y, height_z).extrude(length).translate((x0, y, z))


def _z_cylinder(radius: float, length: float, x: float, y: float, z0: float):
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z0))


def _make_base_body():
    """One-piece cast base with the fixed rear jaw and through mounting slots."""
    body = _box(0.400, 0.180, 0.025, 0.000, 0.000, 0.0125)
    body = body.union(_box(0.078, 0.156, 0.112, 0.111, 0.000, 0.081))
    body = body.union(_box(0.040, 0.126, 0.026, 0.100, 0.000, 0.038))
    body = body.union(_box(0.044, 0.052, 0.060, 0.151, 0.000, 0.070))

    # A round central bore clears the fine-adjust screw where it runs through
    # the rear jaw/nut casting.
    body = body.cut(_x_cylinder(0.017, 0.120, 0.048, 0.000, 0.065))

    # Four drill-table mounting slots in the flat base plate.
    for sx in (-0.125, 0.125):
        for sy in (-0.060, 0.060):
            body = body.cut(_box(0.036, 0.014, 0.040, sx, sy, 0.014))
            body = body.cut(_z_cylinder(0.007, 0.040, sx - 0.018, sy, -0.004))
            body = body.cut(_z_cylinder(0.007, 0.040, sx + 0.018, sy, -0.004))
    return body


def _make_moving_jaw_body():
    """Sliding front jaw casting with square guide-bar bores and screw clearance."""
    body = _box(0.058, 0.152, 0.088, -0.029, 0.000, 0.096)
    body = body.union(_box(0.074, 0.132, 0.030, -0.039, 0.000, 0.044))

    for y in (-0.045, 0.045):
        body = body.union(_box(0.102, 0.040, 0.040, -0.052, y, 0.055))

    # Side boss for the quick-release cam lever pivot.
    body = body.union(_box(0.038, 0.020, 0.038, -0.042, -0.085, 0.081))

    # Two square through-holes ride over the fixed guide bars with visible
    # clearance rather than interpenetrating them.
    for y in (-0.045, 0.045):
        body = body.cut(_x_rect_cut(0.024, 0.024, 0.140, -0.120, y, 0.040))

    # A central clearance bore lets the screw spin through the moving jaw.
    body = body.cut(_x_cylinder(0.015, 0.145, -0.118, 0.000, 0.065))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_action_drill_press_vise")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.105, 0.11, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    blackened = model.material("blackened_steel", rgba=(0.025, 0.026, 0.028, 1.0))
    worn_edge = model.material("polished_wear", rgba=(0.78, 0.76, 0.70, 1.0))
    red_grip = model.material("red_release_grip", rgba=(0.75, 0.04, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "base_casting", tolerance=0.0008),
        material=cast_iron,
        name="base_casting",
    )

    # Two fixed square-section guide bars, visibly captured in the rear casting.
    base.visual(
        Box((0.300, 0.018, 0.018)),
        origin=Origin(xyz=(-0.035, -0.045, 0.040)),
        material=machined,
        name="guide_bar_0",
    )
    base.visual(
        Box((0.300, 0.018, 0.018)),
        origin=Origin(xyz=(-0.035, 0.045, 0.040)),
        material=machined,
        name="guide_bar_1",
    )

    base.visual(
        Box((0.010, 0.140, 0.052)),
        origin=Origin(xyz=(0.075, 0.000, 0.105)),
        material=worn_edge,
        name="fixed_jaw_plate",
    )
    for i, z in enumerate((0.086, 0.096, 0.106, 0.116, 0.126)):
        base.visual(
            Box((0.003, 0.142, 0.003)),
            origin=Origin(xyz=(0.0685, 0.000, z)),
            material=machined,
            name=f"fixed_jaw_tooth_{i}",
        )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        mesh_from_cadquery(_make_moving_jaw_body(), "moving_jaw_casting", tolerance=0.0008),
        material=cast_iron,
        name="moving_jaw_casting",
    )
    moving_jaw.visual(
        Box((0.010, 0.140, 0.052)),
        origin=Origin(xyz=(0.005, 0.000, 0.105)),
        material=worn_edge,
        name="moving_jaw_plate",
    )
    for i, z in enumerate((0.086, 0.096, 0.106, 0.116, 0.126)):
        moving_jaw.visual(
            Box((0.003, 0.142, 0.003)),
            origin=Origin(xyz=(0.0115, 0.000, z)),
            material=machined,
            name=f"moving_jaw_tooth_{i}",
        )

    slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=moving_jaw,
        origin=Origin(xyz=(0.045, 0.000, 0.000)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=0.120),
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.009, length=0.345),
        origin=Origin(xyz=(0.1325, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="threaded_screw",
    )
    for i, x in enumerate((0.050, 0.075, 0.100, 0.125, 0.150, 0.175, 0.200, 0.225)):
        screw_handle.visual(
            Cylinder(radius=0.0098, length=0.003),
            origin=Origin(xyz=(x, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
            material=blackened,
            name=f"screw_thread_band_{i}",
        )
    screw_handle.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(-0.034, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=blackened,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.006, length=0.145),
        origin=Origin(xyz=(-0.040, 0.000, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="tommy_bar",
    )
    screw_handle.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.040, -0.0725, 0.000)),
        material=blackened,
        name="handle_knob_0",
    )
    screw_handle.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.040, 0.0725, 0.000)),
        material=blackened,
        name="handle_knob_1",
    )

    screw = model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(-0.075, 0.000, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=6.0),
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.000, -0.006, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blackened,
        name="cam_pivot",
    )
    cam_lever.visual(
        Box((0.018, 0.012, 0.070)),
        origin=Origin(xyz=(-0.020, -0.006, -0.034), rpy=(0.0, -0.22, 0.0)),
        material=blackened,
        name="lever_arm",
    )
    cam_lever.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(-0.032, -0.006, -0.064), rpy=(pi / 2.0, 0.0, 0.0)),
        material=red_grip,
        name="lever_grip",
    )

    cam = model.articulation(
        "cam_release",
        ArticulationType.REVOLUTE,
        parent=moving_jaw,
        child=cam_lever,
        origin=Origin(xyz=(-0.042, -0.096, 0.081)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.80, upper=0.80),
    )

    # Keep names referenced so static analysis does not confuse these authored
    # mechanisms with unused construction locals.
    assert slide is not None and screw is not None and cam is not None
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

    base = object_model.get_part("base")
    moving_jaw = object_model.get_part("moving_jaw")
    screw_handle = object_model.get_part("screw_handle")
    cam_lever = object_model.get_part("cam_lever")
    slide = object_model.get_articulation("jaw_slide")
    screw = object_model.get_articulation("screw_spin")
    cam = object_model.get_articulation("cam_release")

    ctx.allow_overlap(
        base,
        screw_handle,
        elem_a="base_casting",
        elem_b="threaded_screw",
        reason="The fine-adjust screw is intentionally shown engaged inside the fixed rear jaw's threaded nut bore.",
    )
    ctx.expect_overlap(
        screw_handle,
        base,
        axes="x",
        elem_a="threaded_screw",
        elem_b="base_casting",
        min_overlap=0.070,
        name="fine screw remains engaged through rear jaw nut",
    )
    ctx.expect_within(
        screw_handle,
        base,
        axes="yz",
        inner_elem="threaded_screw",
        outer_elem="base_casting",
        margin=0.0,
        name="fine screw axis is carried by rear jaw bore",
    )

    ctx.expect_overlap(
        moving_jaw,
        base,
        axes="yz",
        elem_a="moving_jaw_casting",
        elem_b="guide_bar_0",
        min_overlap=0.015,
        name="front jaw rides on first square guide bar",
    )
    ctx.expect_overlap(
        moving_jaw,
        base,
        axes="yz",
        elem_a="moving_jaw_casting",
        elem_b="guide_bar_1",
        min_overlap=0.015,
        name="front jaw rides on second square guide bar",
    )
    ctx.expect_gap(
        base,
        moving_jaw,
        axis="x",
        positive_elem="fixed_jaw_plate",
        negative_elem="moving_jaw_plate",
        min_gap=0.010,
        max_gap=0.025,
        name="closed vise has a small jaw opening",
    )

    closed_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({slide: 0.120}):
        ctx.expect_gap(
            base,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw_plate",
            negative_elem="moving_jaw_plate",
            min_gap=0.125,
            max_gap=0.150,
            name="quick slide increases jaw capacity",
        )
        ctx.expect_overlap(
            moving_jaw,
            base,
            axes="x",
            elem_a="moving_jaw_casting",
            elem_b="guide_bar_0",
            min_overlap=0.030,
            name="first guide bar remains inserted when open",
        )
        ctx.expect_overlap(
            moving_jaw,
            base,
            axes="x",
            elem_a="moving_jaw_casting",
            elem_b="guide_bar_1",
            min_overlap=0.030,
            name="second guide bar remains inserted when open",
        )
        open_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "front jaw moves toward the operator/front",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.10,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    handle_aabb = ctx.part_world_aabb(screw_handle)
    with ctx.pose({screw: pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(screw_handle)
    ctx.check(
        "central screw handle rotates about its screw axis",
        handle_aabb is not None
        and spun_aabb is not None
        and (spun_aabb[1][2] - spun_aabb[0][2]) > (handle_aabb[1][2] - handle_aabb[0][2]) + 0.080,
        details=f"rest={handle_aabb}, spun={spun_aabb}",
    )

    lever_aabb = ctx.part_world_aabb(cam_lever)
    with ctx.pose({cam: 0.80}):
        raised_aabb = ctx.part_world_aabb(cam_lever)
    ctx.check(
        "side cam lever has a useful revolute swing",
        lever_aabb is not None
        and raised_aabb is not None
        and (raised_aabb[0][2] - lever_aabb[0][2]) > 0.025,
        details=f"rest={lever_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
