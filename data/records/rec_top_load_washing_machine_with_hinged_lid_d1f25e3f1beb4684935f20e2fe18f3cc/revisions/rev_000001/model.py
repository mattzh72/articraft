from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _washer_cabinet_body() -> cq.Workplane:
    """Rounded compact cabinet shell with a real cylindrical top opening."""
    width = 0.56
    depth = 0.60
    height = 0.66
    base_z = 0.09
    cut_radius = 0.235

    outer = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, base_z + height / 2.0))
        .edges("|Z")
        .fillet(0.025)
        .edges(">Z")
        .fillet(0.010)
    )
    opening = (
        cq.Workplane("XY")
        .cylinder(height + 0.04, cut_radius)
        .translate((0.0, 0.0, base_z + height / 2.0))
    )
    return outer.cut(opening)


def _tub_bucket() -> cq.Workplane:
    """Open-top wash basket with wall thickness and a load-sized hollow."""
    outer_radius = 0.205
    inner_radius = 0.178
    height = 0.50
    bottom_thickness = 0.045

    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner_void = (
        cq.Workplane("XY")
        .workplane(offset=bottom_thickness)
        .circle(inner_radius)
        .extrude(height)
    )
    tub = outer.cut(inner_void)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=height - 0.018)
        .circle(outer_radius + 0.010)
        .circle(inner_radius - 0.004)
        .extrude(0.018)
    )
    return tub.union(rim).edges(">Z").fillet(0.004)


def _lid_frame() -> cq.Workplane:
    """Opaque perimeter frame around the smoked transparent lid insert."""
    outer_w = 0.52
    outer_d = 0.50
    inner_w = 0.40
    inner_d = 0.32
    thickness = 0.032

    frame = cq.Workplane("XY").box(outer_w, outer_d, thickness)
    window = cq.Workplane("XY").box(inner_w, inner_d, thickness + 0.010)
    return (
        frame.cut(window)
        .translate((0.0, -outer_d / 2.0, thickness / 2.0))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_top_load_washer")

    cabinet_mat = model.material("warm_white_enamel", rgba=(0.93, 0.91, 0.84, 1.0))
    trim_mat = model.material("soft_gray_trim", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_mat = model.material("charcoal_panel", rgba=(0.04, 0.045, 0.05, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    steel_mat = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.73, 1.0))
    glass_mat = model.material("smoked_clear_lid", rgba=(0.33, 0.55, 0.72, 0.45))
    button_mat = model.material("pale_blue_buttons", rgba=(0.46, 0.62, 0.78, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_washer_cabinet_body(), "cabinet_shell", tolerance=0.0015),
        material=cabinet_mat,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.38, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, -0.303, 0.42)),
        material=Material("subtle_front_panel", rgba=(0.86, 0.85, 0.80, 1.0)),
        name="front_service_panel",
    )
    cabinet.visual(
        Box((0.54, 0.13, 0.16)),
        origin=Origin(xyz=(0.0, 0.345, 0.83)),
        material=cabinet_mat,
        name="control_pod_shell",
    )
    cabinet.visual(
        Box((0.46, 0.008, 0.105)),
        origin=Origin(xyz=(0.0, 0.276, 0.835)),
        material=dark_mat,
        name="control_face",
    )
    # Rubber top gasket: four connected strips around the laundry opening.
    cabinet.visual(
        Box((0.50, 0.045, 0.008)),
        origin=Origin(xyz=(0.0, 0.220, 0.754)),
        material=rubber_mat,
        name="rear_gasket",
    )
    cabinet.visual(
        Box((0.50, 0.045, 0.008)),
        origin=Origin(xyz=(0.0, -0.220, 0.754)),
        material=rubber_mat,
        name="front_gasket",
    )
    cabinet.visual(
        Box((0.045, 0.395, 0.008)),
        origin=Origin(xyz=(-0.2275, 0.0, 0.754)),
        material=rubber_mat,
        name="side_gasket_0",
    )
    cabinet.visual(
        Box((0.045, 0.395, 0.008)),
        origin=Origin(xyz=(0.2275, 0.0, 0.754)),
        material=rubber_mat,
        name="side_gasket_1",
    )
    # Alternating hinge knuckles and a fixed leaf, mounted to the rear top deck.
    cabinet.visual(
        Box((0.13, 0.034, 0.008)),
        origin=Origin(xyz=(-0.16, 0.263, 0.762)),
        material=trim_mat,
        name="fixed_hinge_leaf_0",
    )
    cabinet.visual(
        Box((0.13, 0.034, 0.008)),
        origin=Origin(xyz=(0.16, 0.263, 0.762)),
        material=trim_mat,
        name="fixed_hinge_leaf_1",
    )
    for i, x in enumerate((-0.16, 0.16)):
        cabinet.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(x, 0.263, 0.770), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_mat,
            name=f"fixed_hinge_knuckle_{i}",
        )

    # Internal drive support spanning the lower opening, with a vertical spindle for the rotating tub.
    cabinet.visual(
        Box((0.52, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=trim_mat,
        name="drive_crossbar_x",
    )
    cabinet.visual(
        Box((0.040, 0.52, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=trim_mat,
        name="drive_crossbar_y",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.1725)),
        material=trim_mat,
        name="drive_spindle",
    )

    # Caster fork plates are fixed to the cabinet underside; the wheels are separate links.
    caster_positions = [
        (-0.215, -0.235, 0.043),
        (0.215, -0.235, 0.043),
        (-0.215, 0.235, 0.043),
        (0.215, 0.235, 0.043),
    ]
    for i, (x, y, z) in enumerate(caster_positions):
        cabinet.visual(
            Box((0.082, 0.065, 0.010)),
            origin=Origin(xyz=(x, y, 0.085)),
            material=trim_mat,
            name=f"caster_plate_{i}",
        )
        cabinet.visual(
            Box((0.006, 0.052, 0.060)),
            origin=Origin(xyz=(x - 0.021, y, 0.065)),
            material=trim_mat,
            name=f"caster_fork_{i}_0",
        )
        cabinet.visual(
            Box((0.006, 0.052, 0.060)),
            origin=Origin(xyz=(x + 0.021, y, 0.065)),
            material=trim_mat,
            name=f"caster_fork_{i}_1",
        )

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_tub_bucket(), "wash_tub", tolerance=0.001),
        material=steel_mat,
        name="wash_tub",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radius = 0.171
        tub.visual(
            Box((0.050, 0.018, 0.280)),
            origin=Origin(
                xyz=(-math.sin(angle) * radius, math.cos(angle) * radius, 0.205),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel_mat,
            name=f"agitator_baffle_{i}",
        )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame(), "lid_frame", tolerance=0.001),
        origin=Origin(xyz=(0.0, -0.018, -0.012)),
        material=cabinet_mat,
        name="lid_frame",
    )
    lid.visual(
        Box((0.415, 0.335, 0.010)),
        origin=Origin(xyz=(0.0, -0.268, 0.012)),
        material=glass_mat,
        name="lid_window",
    )
    lid.visual(
        Box((0.18, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.493, 0.029)),
        material=trim_mat,
        name="front_lid_handle",
    )
    lid.visual(
        Box((0.18, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, -0.014, -0.008)),
        material=trim_mat,
        name="moving_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_mat,
        name="lid_hinge_knuckle",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.263, 0.770)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.85),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.068,
                0.028,
                body_style="skirted",
                top_diameter=0.052,
                edge_radius=0.0012,
                skirt=KnobSkirt(0.076, 0.006, flare=0.04, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=24, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="timer_dial_knob",
    )
    model.articulation(
        "pod_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=timer_dial,
        origin=Origin(xyz=(-0.125, 0.272, 0.842)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0),
    )

    for i, x in enumerate((0.065, 0.145)):
        button = model.part(f"mode_button_{i}")
        button.visual(
            Box((0.055, 0.020, 0.028)),
            origin=Origin(xyz=(0.0, -0.010, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.045, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, -0.023, 0.0)),
            material=Material("button_highlight", rgba=(0.68, 0.82, 0.92, 1.0)),
            name="button_highlight",
        )
        model.articulation(
            f"pod_to_mode_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.272, 0.832)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.012),
        )

    for i, (x, y, z) in enumerate(caster_positions):
        wheel = model.part(f"caster_wheel_{i}")
        wheel.visual(
            Cylinder(radius=0.035, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.036),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_mat,
            name="hub",
        )
        model.articulation(
            f"cabinet_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    timer = object_model.get_part("timer_dial")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    button_joint = object_model.get_articulation("pod_to_mode_button_0")

    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        inner_elem="wash_tub",
        outer_elem="cabinet_shell",
        margin=0.0,
        name="wash tub fits inside compact cabinet footprint",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="rear_gasket",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid sits on rear gasket",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="lid_window",
        elem_b="cabinet_shell",
        min_overlap=0.20,
        name="transparent lid covers the tub opening",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.35}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_button = ctx.part_world_position("mode_button_0")
    with ctx.pose({button_joint: 0.010}):
        pushed_button = ctx.part_world_position("mode_button_0")
    ctx.check(
        "mode button pushes inward toward pod",
        rest_button is not None
        and pushed_button is not None
        and pushed_button[1] > rest_button[1] + 0.008,
        details=f"rest={rest_button}, pushed={pushed_button}",
    )
    ctx.expect_contact(
        timer,
        cabinet,
        elem_a="timer_dial_knob",
        elem_b="control_face",
        contact_tol=0.002,
        name="timer dial is mounted on control pod face",
    )

    return ctx.report()


object_model = build_object_model()
