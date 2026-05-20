from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _square_profile(width: float, depth: float) -> list[tuple[float, float]]:
    hx = width * 0.5
    hy = depth * 0.5
    return [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_bollard")

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.23, 1.0))
    galvanized = model.material("galvanized", rgba=(0.54, 0.56, 0.56, 1.0))
    concrete = model.material("concrete", rgba=(0.45, 0.46, 0.45, 1.0))
    rubber = model.material("rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    red_reflector = model.material("red_reflector", rgba=(0.80, 0.06, 0.04, 1.0))
    white_reflector = model.material("white_reflector", rgba=(0.92, 0.92, 0.86, 1.0))
    amber = model.material("amber_lens", rgba=(1.00, 0.58, 0.08, 0.72))
    smoked_amber = model.material("smoked_amber_cover", rgba=(0.96, 0.56, 0.12, 0.38))
    black_slot = model.material("black_keyway", rgba=(0.02, 0.02, 0.018, 1.0))

    sleeve = model.part("sleeve")
    slab_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .box(0.62, 0.62, 0.070)
        .cut(cq.Workplane("XY").cylinder(0.16, 0.205))
        .translate((0.0, 0.0, -0.035)),
        "pavement_cutaway",
    )
    sleeve.visual(slab_mesh, material=concrete, name="pavement_cutaway")

    flange_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .cylinder(0.024, 0.212)
        .cut(cq.Workplane("XY").cylinder(0.050, 0.102))
        .translate((0.0, 0.0, -0.012)),
        "flush_flange",
    )
    sleeve.visual(flange_mesh, material=galvanized, name="flush_flange")

    sleeve_tube_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.125, -1.250), (0.125, 0.030)],
            inner_profile=[(0.098, -1.235), (0.098, 0.038)],
            segments=96,
        ),
        "sleeve_tube",
    )
    sleeve.visual(sleeve_tube_mesh, material=dark_steel, name="sleeve_tube")

    wiper_mesh = mesh_from_cadquery(
        cq.Workplane("XY")
        .cylinder(0.012, 0.113)
        .cut(cq.Workplane("XY").cylinder(0.030, 0.081))
        .translate((0.0, 0.0, 0.006)),
        "rubber_wiper",
    )
    sleeve.visual(wiper_mesh, material=rubber, name="rubber_wiper")

    for i in range(6):
        angle = i * math.tau / 6.0
        sleeve.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(
                xyz=(0.162 * math.cos(angle), 0.162 * math.sin(angle), 0.0035)
            ),
            material=dark_steel,
            name=f"flange_bolt_{i}",
        )
    sleeve.inertial = Inertial.from_geometry(
        Box((0.62, 0.62, 1.30)), mass=80.0, origin=Origin(xyz=(0.0, 0.0, -0.62))
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.082, length=1.200),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=brushed_steel,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.094, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=dark_steel,
        name="lower_stop",
    )
    post.visual(
        Cylinder(radius=0.100, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.8425)),
        material=brushed_steel,
        name="crown_plate",
    )
    post.visual(
        Cylinder(radius=0.0845, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=red_reflector,
        name="lower_reflector_band",
    )
    post.visual(
        Cylinder(radius=0.0845, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=white_reflector,
        name="upper_reflector_band",
    )
    post.visual(
        Box((0.072, 0.042, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, 0.865)),
        material=dark_steel,
        name="light_base",
    )
    post.visual(
        Box((0.050, 0.030, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, 0.876)),
        material=amber,
        name="warning_lens",
    )
    post.visual(
        Box((0.020, 0.052, 0.040)),
        origin=Origin(xyz=(0.008, 0.0, 0.876)),
        material=dark_steel,
        name="hinge_mount",
    )
    post.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.012, -0.021, 0.897), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="fixed_hinge_knuckle_0",
    )
    post.visual(
        Cylinder(radius=0.0055, length=0.012),
        origin=Origin(xyz=(0.012, 0.021, 0.897), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="fixed_hinge_knuckle_1",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.10, length=1.24), mass=42.0, origin=Origin(xyz=(0.0, 0.0, 0.27))
    )

    cap = model.part("keyed_cap")
    cap.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=brushed_steel,
        name="cap_body",
    )
    cap.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=dark_steel,
        name="cap_top_recess",
    )
    cap.visual(
        Box((0.050, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=black_slot,
        name="key_slot",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.030), mass=0.35, origin=Origin(xyz=(0.0, 0.0, 0.014))
    )

    light_cover = model.part("light_cover")
    light_cover.visual(
        Cylinder(radius=0.0048, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cover_knuckle",
    )
    light_cover.visual(
        Box((0.010, 0.018, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, 0.003)),
        material=dark_steel,
        name="cover_spine",
    )
    light_cover.visual(
        Box((0.064, 0.046, 0.006)),
        origin=Origin(xyz=(0.039, 0.0, 0.003)),
        material=smoked_amber,
        name="cover_panel",
    )
    light_cover.visual(
        Box((0.006, 0.046, 0.013)),
        origin=Origin(xyz=(0.071, 0.0, -0.0005)),
        material=smoked_amber,
        name="front_lip",
    )
    light_cover.inertial = Inertial.from_geometry(
        Box((0.078, 0.050, 0.018)), mass=0.08, origin=Origin(xyz=(0.038, 0.0, 0.003))
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.35, lower=-0.820, upper=0.0),
    )
    model.articulation(
        "cap_rotation",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=cap,
        origin=Origin(xyz=(-0.038, 0.0, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    model.articulation(
        "light_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=light_cover,
        origin=Origin(xyz=(0.012, 0.0, 0.897)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    post = object_model.get_part("post")
    cap = object_model.get_part("keyed_cap")
    cover = object_model.get_part("light_cover")
    slide = object_model.get_articulation("post_slide")
    cap_joint = object_model.get_articulation("cap_rotation")
    cover_hinge = object_model.get_articulation("light_cover_hinge")

    ctx.allow_overlap(
        post,
        sleeve,
        elem_a="post_shaft",
        elem_b="rubber_wiper",
        reason="The sleeve-mouth rubber wiper is intentionally compressed around the sliding steel post.",
    )

    ctx.check(
        "post uses prismatic travel in sleeve",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower < -0.75
        and slide.motion_limits.upper == 0.0,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "keyed cap has continuous local rotation",
        cap_joint.articulation_type == ArticulationType.CONTINUOUS
        and cap_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={cap_joint.articulation_type}, axis={cap_joint.axis}",
    )
    ctx.check(
        "warning cover has crown hinge limits",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE
        and cover_hinge.motion_limits is not None
        and cover_hinge.motion_limits.lower == 0.0
        and cover_hinge.motion_limits.upper > 1.2,
        details=f"type={cover_hinge.articulation_type}, limits={cover_hinge.motion_limits}",
    )

    ctx.expect_within(
        post,
        sleeve,
        axes="xy",
        inner_elem="post_shaft",
        outer_elem="sleeve_tube",
        margin=0.0,
        name="post shaft stays within the underground sleeve diameter",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_shaft",
        elem_b="sleeve_tube",
        min_overlap=0.25,
        name="raised post retains insertion in sleeve",
    )
    ctx.expect_overlap(
        post,
        sleeve,
        axes="z",
        elem_a="post_shaft",
        elem_b="rubber_wiper",
        min_overlap=0.010,
        name="rubber wiper wraps the post at the sleeve mouth",
    )
    ctx.expect_gap(
        cap,
        post,
        axis="z",
        positive_elem="cap_body",
        negative_elem="crown_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="separate keyed cap is seated on the crown",
    )
    ctx.expect_overlap(
        cap,
        post,
        axes="xy",
        elem_a="cap_body",
        elem_b="crown_plate",
        min_overlap=0.050,
        name="keyed cap is seated on the crown footprint",
    )
    ctx.expect_overlap(
        cover,
        post,
        axes="xz",
        elem_a="cover_knuckle",
        elem_b="fixed_hinge_knuckle_0",
        min_overlap=0.006,
        name="warning cover knuckle aligns with fixed crown hinge",
    )
    ctx.expect_overlap(
        cover,
        post,
        axes="xy",
        elem_a="cover_panel",
        elem_b="warning_lens",
        min_overlap=0.020,
        name="closed cover spans the warning lens",
    )

    raised_pos = ctx.part_world_position(post)
    with ctx.pose({slide: -0.820}):
        ctx.expect_within(
            post,
            sleeve,
            axes="xy",
            inner_elem="post_shaft",
            outer_elem="sleeve_tube",
            margin=0.0,
            name="retracted post remains centered in sleeve",
        )
        ctx.expect_overlap(
            post,
            sleeve,
            axes="z",
            elem_a="post_shaft",
            elem_b="sleeve_tube",
            min_overlap=1.05,
            name="retracted post stores inside underground sleeve",
        )
        retracted_pos = ctx.part_world_position(post)
    ctx.check(
        "post retracts downward from the raised pose",
        raised_pos is not None and retracted_pos is not None and retracted_pos[2] < raised_pos[2] - 0.75,
        details=f"raised={raised_pos}, retracted={retracted_pos}",
    )

    cover_closed = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.10}):
        cover_open = ctx.part_world_aabb(cover)
    ctx.check(
        "warning light cover opens upward on hinge",
        cover_closed is not None
        and cover_open is not None
        and cover_open[1][2] > cover_closed[1][2] + 0.025,
        details=f"closed={cover_closed}, open={cover_open}",
    )

    return ctx.report()


object_model = build_object_model()
