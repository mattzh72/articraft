from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bakery_prep_stand_mixer")

    enamel = model.material("warm_cream_enamel", rgba=(0.86, 0.78, 0.62, 1.0))
    shadow = model.material("shadowed_cream", rgba=(0.64, 0.56, 0.43, 1.0))
    steel = model.material("brushed_stainless_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    dark = model.material("black_rubber", rgba=(0.025, 0.024, 0.022, 1.0))
    red = model.material("red_release", rgba=(0.62, 0.08, 0.045, 1.0))

    # Root assembly: a heavy, broad base with a rear pedestal and visible hinge
    # barrel.  Dimensions are sized like a large bakery-prep counter mixer.
    base = model.part("base")
    base_shell = superellipse_side_loft(
        [
            (-0.36, 0.000, 0.105, 0.34),
            (-0.23, 0.000, 0.130, 0.43),
            (0.18, 0.000, 0.135, 0.47),
            (0.38, 0.000, 0.105, 0.35),
        ],
        exponents=3.0,
        segments=64,
        cap=True,
    )
    base.visual(mesh_from_geometry(base_shell, "base_shell"), material=enamel, name="base_shell")

    pedestal_shell = ExtrudeGeometry(
        rounded_rect_profile(0.25, 0.16, 0.035, corner_segments=10),
        0.49,
        cap=True,
        center=True,
    )
    base.visual(
        mesh_from_geometry(pedestal_shell, "pedestal_shell"),
        origin=Origin(xyz=(0.0, -0.24, 0.355)),
        material=enamel,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.24, 0.18, 0.16)),
        origin=Origin(xyz=(0.0, -0.16, 0.185), rpy=(math.radians(-10.0), 0.0, 0.0)),
        material=shadow,
        name="reinforced_throat",
    )
    base.visual(
        Box((0.040, 0.44, 0.030)),
        origin=Origin(xyz=(-0.12, 0.14, 0.145)),
        material=steel,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.040, 0.44, 0.030)),
        origin=Origin(xyz=(0.12, 0.14, 0.145)),
        material=steel,
        name="slide_rail_1",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.30),
        origin=Origin(xyz=(0.0, -0.26, 0.62), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    # Moving bowl carriage: a short front slide carries a reinforced tray and a
    # large hollow metal bowl.  The carriage is authored as one moving part.
    carriage = model.part("bowl_carriage")
    carriage.visual(
        Box((0.34, 0.32, 0.024)),
        origin=Origin(xyz=(0.0, 0.02, 0.012)),
        material=steel,
        name="slide_plate",
    )
    carriage.visual(
        Box((0.36, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.17, 0.044)),
        material=steel,
        name="front_crossbar",
    )
    for x in (-0.155, 0.155):
        carriage.visual(
            Box((0.036, 0.075, 0.112)),
            origin=Origin(xyz=(x, 0.035, 0.080)),
            material=steel,
            name=f"side_yoke_{0 if x < 0 else 1}",
        )
    carriage.visual(
        mesh_from_geometry(TorusGeometry(0.105, 0.012, radial_segments=18, tubular_segments=64), "bowl_saddle_ring"),
        origin=Origin(xyz=(0.0, 0.035, 0.034)),
        material=steel,
        name="saddle_ring",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.050, 0.000),
            (0.086, 0.018),
            (0.128, 0.085),
            (0.165, 0.180),
            (0.188, 0.250),
        ],
        inner_profile=[
            (0.030, 0.018),
            (0.064, 0.034),
            (0.108, 0.095),
            (0.145, 0.178),
            (0.168, 0.229),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    carriage.visual(
        mesh_from_geometry(bowl_shell, "large_bowl_shell"),
        origin=Origin(xyz=(0.0, 0.035, 0.024)),
        material=steel,
        name="large_bowl",
    )
    model.articulation(
        "bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.10, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.08),
    )

    # The tilt head is a stout rounded casting.  Its frame sits on the rear
    # hinge line so positive rotation about +X raises the nose.
    head = model.part("tilt_head")
    for x in (-0.18, 0.18):
        head.visual(
            Cylinder(radius=0.035, length=0.06),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_lug_{0 if x < 0 else 1}",
        )
        head.visual(
            Box((0.075, 0.095, 0.060)),
            origin=Origin(xyz=(-0.19 if x < 0 else 0.19, 0.055, -0.015)),
            material=enamel,
            name=f"side_cheek_{0 if x < 0 else 1}",
        )
    head.visual(
        Box((0.34, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, 0.120, 0.015)),
        material=enamel,
        name="rear_bridge",
    )
    head_shell = superellipse_side_loft(
        [
            (0.105, -0.105, 0.035, 0.25),
            (0.23, -0.130, 0.060, 0.36),
            (0.41, -0.130, 0.052, 0.35),
            (0.55, -0.105, 0.030, 0.25),
        ],
        exponents=2.8,
        segments=64,
        cap=True,
    )
    head.visual(mesh_from_geometry(head_shell, "tilt_head_shell"), material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.390, -0.150)),
        material=shadow,
        name="planetary_socket",
    )
    model.articulation(
        "head_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.26, 0.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.1, lower=0.0, upper=1.05),
    )

    # Continuous rotating dough hook under the head.  The mesh is one swept tube
    # from the drive coupling into a C-shaped hook so it reads as one tool.
    hook = model.part("dough_hook")
    hook.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=steel,
        name="drive_coupling",
    )
    hook_curve = tube_from_spline_points(
        [
            (0.000, 0.000, -0.055),
            (0.004, 0.010, -0.100),
            (0.032, 0.030, -0.140),
            (0.055, 0.018, -0.185),
            (0.040, -0.030, -0.205),
            (0.000, -0.042, -0.190),
            (-0.040, -0.024, -0.155),
            (-0.030, 0.018, -0.120),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    hook.visual(mesh_from_geometry(hook_curve, "c_hook_tube"), material=steel, name="c_hook")
    model.articulation(
        "hook_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.0, 0.390, -0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    # User controls on the base/pedestal: an articulated speed selector lever
    # and a translating head-lock release.
    speed = model.part("speed_selector")
    selector_knob = KnobGeometry(
        0.058,
        0.018,
        body_style="faceted",
        grip=KnobGrip(style="ribbed", count=12, depth=0.001),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
    )
    speed.visual(
        mesh_from_geometry(selector_knob, "speed_selector_knob"),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="selector_knob",
    )
    speed.visual(
        Box((0.012, 0.012, 0.072)),
        origin=Origin(xyz=(0.017, 0.0, 0.046)),
        material=dark,
        name="selector_pointer",
    )
    model.articulation(
        "speed_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed,
        origin=Origin(xyz=(0.1247, -0.20, 0.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.55, upper=0.55),
    )

    release = model.part("lock_release")
    release.visual(
        Box((0.105, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=red,
        name="release_button",
    )
    model.articulation(
        "lock_release_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=release,
        origin=Origin(xyz=(0.0, 0.380, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("tilt_head")
    hook = object_model.get_part("dough_hook")
    speed = object_model.get_part("speed_selector")
    release = object_model.get_part("lock_release")

    bowl_slide = object_model.get_articulation("bowl_slide")
    head_hinge = object_model.get_articulation("head_hinge")
    hook_spin = object_model.get_articulation("hook_spin")
    speed_pivot = object_model.get_articulation("speed_pivot")
    release_slide = object_model.get_articulation("lock_release_slide")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="slide_plate",
        elem_b="slide_rail_0",
        contact_tol=0.002,
        name="carriage rides on base slide rail",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="xy",
        inner_elem="large_bowl",
        outer_elem="base_shell",
        margin=0.25,
        name="large bowl remains over the pedestal footprint",
    )
    ctx.expect_within(
        hook,
        carriage,
        axes="xy",
        inner_elem="c_hook",
        outer_elem="large_bowl",
        margin=0.002,
        name="dough hook sits inside the large bowl opening",
    )
    ctx.expect_gap(
        head,
        carriage,
        axis="z",
        positive_elem="head_shell",
        negative_elem="large_bowl",
        min_gap=0.025,
        name="tilt head clears the bowl rim at rest",
    )
    ctx.expect_contact(
        speed,
        base,
        elem_a="selector_knob",
        elem_b="pedestal_shell",
        contact_tol=0.003,
        name="speed selector is mounted on pedestal side",
    )
    ctx.expect_contact(
        release,
        base,
        elem_a="release_button",
        elem_b="base_shell",
        contact_tol=0.002,
        name="head lock release sits in the front base face",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({bowl_slide: 0.08}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="y",
            elem_a="slide_plate",
            elem_b="slide_rail_0",
            min_overlap=0.20,
            name="extended bowl carriage stays captured on rail",
        )
    ctx.check(
        "bowl carriage slides toward the front",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[1] > rest_carriage_pos[1] + 0.075,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_head_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
    with ctx.pose({head_hinge: 0.85}):
        raised_head_aabb = ctx.part_element_world_aabb(head, elem="head_shell")
    ctx.check(
        "head hinge raises the mixer nose",
        rest_head_aabb is not None
        and raised_head_aabb is not None
        and raised_head_aabb[1][2] > rest_head_aabb[1][2] + 0.08,
        details=f"rest={rest_head_aabb}, raised={raised_head_aabb}",
    )

    with ctx.pose({speed_pivot: 0.45, release_slide: 0.020, hook_spin: 1.4}):
        ctx.expect_contact(
            hook,
            head,
            elem_a="drive_coupling",
            elem_b="planetary_socket",
            contact_tol=0.004,
            name="spinning hook remains seated in the head socket",
        )
        ctx.expect_origin_gap(
            release,
            base,
            axis="y",
            min_gap=0.39,
            name="release button translates outward from the base",
        )

    return ctx.report()


object_model = build_object_model()
