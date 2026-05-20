from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _offset_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_laptop_stand")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.09, 0.10, 0.11, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    stand = model.part("stand")

    base_profile = superellipse_profile(0.64, 0.38, exponent=2.0, segments=80)
    base_mesh = mesh_from_geometry(
        ExtrudeGeometry(base_profile, 0.030, center=False),
        "oval_base",
    )
    stand.visual(base_mesh, material=satin_aluminum, name="oval_base")

    spine = tube_from_spline_points(
        [
            (0.0, -0.155, 0.030),
            (0.0, -0.150, 0.130),
            (0.0, -0.120, 0.255),
            (0.0, -0.090, 0.337),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=28,
        cap_ends=True,
    )
    stand.visual(mesh_from_geometry(spine, "curved_spine"), material=dark_graphite, name="curved_spine")
    stand.visual(
        Cylinder(radius=0.018, length=0.425),
        origin=Origin(xyz=(0.0, -0.090, 0.5475)),
        material=dark_graphite,
        name="vertical_guide",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, -0.155, 0.032)),
        material=dark_graphite,
        name="spine_foot",
    )
    stand.visual(
        Box((0.30, 0.025, 0.008)),
        origin=Origin(xyz=(0.0, 0.070, 0.034)),
        material=black_rubber,
        name="front_base_pad",
    )
    stand.visual(
        Box((0.24, 0.025, 0.008)),
        origin=Origin(xyz=(0.0, -0.090, 0.034)),
        material=black_rubber,
        name="rear_base_pad",
    )

    sleeve = model.part("sleeve")
    sleeve_outer = rounded_rect_profile(0.116, 0.084, radius=0.018, corner_segments=10)
    sleeve_inner = rounded_rect_profile(0.068, 0.054, radius=0.012, corner_segments=8)
    sleeve_shell = ExtrudeWithHolesGeometry(sleeve_outer, [sleeve_inner], 0.170, center=False)
    sleeve.visual(mesh_from_geometry(sleeve_shell, "sliding_sleeve"), material=dark_graphite, name="sleeve_shell")
    sleeve.visual(
        Box((0.112, 0.054, 0.050)),
        origin=Origin(xyz=(0.0, 0.044, 0.150)),
        material=dark_graphite,
        name="hinge_boss",
    )
    sleeve.visual(
        Cylinder(radius=0.014, length=0.122),
        origin=Origin(xyz=(0.0, 0.075, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_graphite,
        name="center_knuckle",
    )
    sleeve.visual(
        Cylinder(radius=0.0045, length=0.340),
        origin=Origin(xyz=(0.0, 0.075, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin",
    )

    tray = model.part("tray")
    tray_depth = 0.320
    tray_rear_y = 0.070
    tray_center_y = tray_rear_y + tray_depth * 0.5
    tray_outer = _offset_profile(
        rounded_rect_profile(0.460, tray_depth, radius=0.035, corner_segments=12),
        dy=tray_center_y,
    )
    vent_hole = _offset_profile(
        rounded_rect_profile(0.260, 0.165, radius=0.028, corner_segments=12),
        dy=tray_center_y,
    )
    tray_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(tray_outer, [vent_hole], 0.012, center=True),
        "vented_tray_plate",
    )
    tray.visual(tray_plate, material=satin_aluminum, name="vented_plate")
    tray.visual(
        Box((0.438, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, tray_rear_y + tray_depth + 0.004, 0.022)),
        material=satin_aluminum,
        name="front_lip",
    )
    tray.visual(
        Box((0.430, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, tray_rear_y + tray_depth + 0.014, 0.050)),
        material=black_rubber,
        name="lip_rubber",
    )
    hinge_details = (
        (-0.170, "rubber_strip_0", "hinge_leaf_0", "tray_knuckle_0"),
        (0.170, "rubber_strip_1", "hinge_leaf_1", "tray_knuckle_1"),
    )
    for x_pos, strip_name, leaf_name, knuckle_name in hinge_details:
        tray.visual(
            Box((0.030, 0.245, 0.006)),
            origin=Origin(xyz=(x_pos, tray_center_y, 0.009)),
            material=black_rubber,
            name=strip_name,
        )
        tray.visual(
            Box((0.062, 0.072, 0.010)),
            origin=Origin(xyz=(x_pos * 0.66, 0.038, -0.006)),
            material=satin_aluminum,
            name=leaf_name,
        )
        tray.visual(
            Cylinder(radius=0.013, length=0.070),
            origin=Origin(xyz=(x_pos * 0.66, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=satin_aluminum,
            name=knuckle_name,
        )

    slide_joint = model.articulation(
        "spine_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=sleeve,
        origin=Origin(xyz=(0.0, -0.090, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.180),
    )
    slide_joint.meta["qc_samples"] = [0.0, 0.09, 0.18]

    tilt_joint = model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=tray,
        origin=Origin(xyz=(0.0, 0.075, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.75),
    )
    tilt_joint.meta["qc_samples"] = [0.0, 0.35, 0.75]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    sleeve = object_model.get_part("sleeve")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("spine_slide")
    hinge = object_model.get_articulation("tray_hinge")

    ctx.allow_overlap(
        sleeve,
        tray,
        elem_a="hinge_pin",
        elem_b="tray_knuckle_0",
        reason="The steel hinge pin is intentionally captured inside the tray hinge knuckle.",
    )
    ctx.allow_overlap(
        sleeve,
        tray,
        elem_a="hinge_pin",
        elem_b="tray_knuckle_1",
        reason="The steel hinge pin is intentionally captured inside the tray hinge knuckle.",
    )
    ctx.allow_overlap(
        sleeve,
        stand,
        elem_a="sleeve_shell",
        elem_b="vertical_guide",
        reason="The prismatic sleeve is modeled as riding around the spine guide with the guide nested inside the sleeve bore.",
    )

    ctx.expect_overlap(
        sleeve,
        tray,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="tray_knuckle_0",
        min_overlap=0.003,
        name="hinge pin passes through first tray knuckle",
    )
    ctx.expect_overlap(
        sleeve,
        tray,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="tray_knuckle_1",
        min_overlap=0.003,
        name="hinge pin passes through second tray knuckle",
    )

    ctx.expect_within(
        stand,
        sleeve,
        axes="xy",
        inner_elem="vertical_guide",
        outer_elem="sleeve_shell",
        margin=0.0,
        name="sleeve wraps around the spine guide",
    )
    ctx.expect_overlap(
        sleeve,
        stand,
        axes="z",
        elem_a="sleeve_shell",
        elem_b="vertical_guide",
        min_overlap=0.120,
        name="sleeve has retained overlap on the spine at rest",
    )
    ctx.expect_overlap(
        tray,
        sleeve,
        axes="x",
        elem_a="vented_plate",
        elem_b="center_knuckle",
        min_overlap=0.100,
        name="tray spans the sleeve hinge width",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    rest_sleeve_pos = ctx.part_world_position(sleeve)
    with ctx.pose({slide: 0.180}):
        raised_sleeve_pos = ctx.part_world_position(sleeve)
        ctx.expect_overlap(
            sleeve,
            stand,
            axes="z",
            elem_a="sleeve_shell",
            elem_b="vertical_guide",
            min_overlap=0.120,
            name="raised sleeve remains engaged on the spine",
        )
        ctx.expect_within(
            stand,
            sleeve,
            axes="xy",
            inner_elem="vertical_guide",
            outer_elem="sleeve_shell",
            margin=0.0,
            name="raised sleeve stays centered around the spine",
        )

    ctx.check(
        "sleeve moves upward along the spine",
        rest_sleeve_pos is not None
        and raised_sleeve_pos is not None
        and raised_sleeve_pos[2] > rest_sleeve_pos[2] + 0.160,
        details=f"rest={rest_sleeve_pos}, raised={raised_sleeve_pos}",
    )

    with ctx.pose({hinge: 0.70}):
        tilted_tray_pos = ctx.part_world_position(tray)
        rest_aabb = None
        tilted_aabb = ctx.part_world_aabb(tray)

    with ctx.pose({hinge: 0.0}):
        rest_aabb = ctx.part_world_aabb(tray)

    front_lift_ok = False
    if rest_aabb is not None and tilted_aabb is not None:
        front_lift_ok = tilted_aabb[1][2] > rest_aabb[1][2] + 0.10
    ctx.check(
        "tray hinge tilts the front upward",
        front_lift_ok and rest_tray_pos is not None and tilted_tray_pos is not None,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
