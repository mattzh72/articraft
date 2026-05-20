from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _slab_mesh():
    slab = cq.Workplane("XY").box(0.95, 0.95, 0.12).translate((0.0, 0.0, -0.06))
    opening_cutter = cq.Workplane("XY").circle(0.175).extrude(0.24).translate((0.0, 0.0, -0.12))
    return slab.cut(opening_cutter)


def _sleeve_mesh():
    recessed_tube = cq.Workplane("XY").circle(0.205).circle(0.165).extrude(0.585).translate((0.0, 0.0, -0.580))
    raised_rim = cq.Workplane("XY").circle(0.265).circle(0.165).extrude(0.035)
    return recessed_tube.union(raised_rim)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    concrete = model.material("broom_finished_concrete", color=(0.50, 0.49, 0.46, 1.0))
    galvanized = model.material("galvanized_steel", color=(0.58, 0.61, 0.60, 1.0))
    dark_steel = model.material("dark_recessed_steel", color=(0.08, 0.09, 0.09, 1.0))
    yellow = model.material("safety_yellow_powdercoat", color=(1.0, 0.78, 0.08, 1.0))
    black = model.material("black_rubber_shadow", color=(0.01, 0.01, 0.012, 1.0))
    amber = model.material("amber_warning_lens", color=(1.0, 0.45, 0.05, 0.72))

    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_slab_mesh(), "concrete_slab", tolerance=0.002),
        material=concrete,
        name="concrete_slab",
    )
    sleeve.visual(
        mesh_from_cadquery(_sleeve_mesh(), "sleeve_shell", tolerance=0.0015),
        material=galvanized,
        name="sleeve_shell",
    )
    sleeve.visual(
        Cylinder(radius=0.026, length=0.004),
        origin=Origin(xyz=(0.235, 0.0, 0.037)),
        material=black,
        name="lock_recess",
    )
    sleeve.visual(
        Box((0.038, 0.125, 0.009)),
        origin=Origin(xyz=(0.210, 0.0, 0.0395)),
        material=galvanized,
        name="lock_hinge_leaf",
    )
    sleeve.visual(
        Cylinder(radius=0.007, length=0.025),
        origin=Origin(xyz=(0.210, -0.035, 0.051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lock_hinge_lug_0",
    )
    sleeve.visual(
        Cylinder(radius=0.007, length=0.025),
        origin=Origin(xyz=(0.210, 0.035, 0.051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lock_hinge_lug_1",
    )
    sleeve.visual(
        Box((0.025, 0.055, 0.110)),
        origin=Origin(xyz=(0.1525, 0.0, -0.300)),
        material=dark_steel,
        name="guide_pad_0",
    )
    sleeve.visual(
        Box((0.025, 0.055, 0.110)),
        origin=Origin(xyz=(-0.1525, 0.0, -0.300)),
        material=dark_steel,
        name="guide_pad_1",
    )
    sleeve.visual(
        Box((0.055, 0.025, 0.110)),
        origin=Origin(xyz=(0.0, 0.1525, -0.300)),
        material=dark_steel,
        name="guide_pad_2",
    )
    sleeve.visual(
        Box((0.055, 0.025, 0.110)),
        origin=Origin(xyz=(0.0, -0.1525, -0.300)),
        material=dark_steel,
        name="guide_pad_3",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.140, length=1.470),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=yellow,
        name="bollard_post",
    )
    post.visual(
        Cylinder(radius=0.143, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.505)),
        material=dark_steel,
        name="lower_wear_band",
    )
    post.visual(
        Cylinder(radius=0.152, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=yellow,
        name="top_cap",
    )
    post.visual(
        Cylinder(radius=0.142, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=black,
        name="black_reflector_band",
    )
    post.visual(
        Cylinder(radius=0.055, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.9675)),
        material=amber,
        name="warning_light",
    )
    post.visual(
        Box((0.020, 0.016, 0.045)),
        origin=Origin(xyz=(-0.085, -0.023, 0.9725)),
        material=galvanized,
        name="crown_hinge_pedestal_0",
    )
    post.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(-0.085, -0.023, 1.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="crown_hinge_lug_0",
    )
    post.visual(
        Box((0.020, 0.016, 0.045)),
        origin=Origin(xyz=(-0.085, 0.023, 0.9725)),
        material=galvanized,
        name="crown_hinge_pedestal_1",
    )
    post.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(-0.085, 0.023, 1.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="crown_hinge_lug_1",
    )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Cylinder(radius=0.0065, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lock_barrel",
    )
    lock_cover.visual(
        Box((0.034, 0.035, 0.007)),
        origin=Origin(xyz=(0.017, 0.0, 0.008)),
        material=galvanized,
        name="lock_cover_strap",
    )
    lock_cover.visual(
        Box((0.160, 0.100, 0.010)),
        origin=Origin(xyz=(0.090, 0.0, 0.008)),
        material=galvanized,
        name="lock_panel",
    )
    lock_cover.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.115, 0.0, 0.015)),
        material=dark_steel,
        name="lock_panel_plug",
    )

    warning_cover = model.part("warning_cover")
    warning_cover.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="warning_barrel",
    )
    warning_cover.visual(
        Box((0.052, 0.018, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, 0.006)),
        material=galvanized,
        name="warning_cover_strap",
    )
    warning_cover.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.083, 0.0, 0.007)),
        material=amber,
        name="warning_lid",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=0.35),
    )
    model.articulation(
        "sleeve_to_lock_cover",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lock_cover,
        origin=Origin(xyz=(0.210, 0.0, 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "post_to_warning_cover",
        ArticulationType.REVOLUTE,
        parent=post,
        child=warning_cover,
        origin=Origin(xyz=(-0.085, 0.0, 0.995)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    lock_cover = object_model.get_part("lock_cover")
    warning_cover = object_model.get_part("warning_cover")
    post_slide = object_model.get_articulation("sleeve_to_post")
    lock_hinge = object_model.get_articulation("sleeve_to_lock_cover")
    warning_hinge = object_model.get_articulation("post_to_warning_cover")

    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="bollard_post",
        elem_b="sleeve_shell",
        min_overlap=0.45,
        name="post remains deeply seated in recessed sleeve",
    )
    ctx.expect_contact(
        lock_cover,
        sleeve,
        elem_a="lock_barrel",
        elem_b="lock_hinge_lug_0",
        contact_tol=0.001,
        name="lock cover hinge is mounted to sleeve rim",
    )
    ctx.expect_contact(
        warning_cover,
        post,
        elem_a="warning_barrel",
        elem_b="crown_hinge_lug_0",
        contact_tol=0.001,
        name="warning light cover hinge is mounted at crown",
    )

    rest_post = ctx.part_world_position(post)
    with ctx.pose({post_slide: 0.35}):
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="bollard_post",
            elem_b="sleeve_shell",
            min_overlap=0.18,
            name="raised post retains insertion in sleeve",
        )
        raised_post = ctx.part_world_position(post)
    ctx.check(
        "post slides upward along vertical sleeve axis",
        rest_post is not None and raised_post is not None and raised_post[2] > rest_post[2] + 0.30,
        details=f"rest={rest_post}, raised={raised_post}",
    )

    rest_lock_aabb = ctx.part_element_world_aabb(lock_cover, elem="lock_panel")
    with ctx.pose({lock_hinge: 1.20}):
        open_lock_aabb = ctx.part_element_world_aabb(lock_cover, elem="lock_panel")
    ctx.check(
        "lock cover rotates upward from the rim",
        rest_lock_aabb is not None
        and open_lock_aabb is not None
        and open_lock_aabb[1][2] > rest_lock_aabb[1][2] + 0.07,
        details=f"rest={rest_lock_aabb}, open={open_lock_aabb}",
    )

    rest_warning_aabb = ctx.part_element_world_aabb(warning_cover, elem="warning_lid")
    with ctx.pose({warning_hinge: 1.10}):
        open_warning_aabb = ctx.part_element_world_aabb(warning_cover, elem="warning_lid")
    ctx.check(
        "warning light cover rotates upward at the crown",
        rest_warning_aabb is not None
        and open_warning_aabb is not None
        and open_warning_aabb[1][2] > rest_warning_aabb[1][2] + 0.045,
        details=f"rest={rest_warning_aabb}, open={open_warning_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
