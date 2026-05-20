from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _loft_geometry(sections) -> MeshGeometry:
    """Fast mesh loft for corresponding closed profile loops."""
    geom = MeshGeometry()
    rings = []
    for section in sections:
        ring = [geom.add_vertex(x, y, z) for x, y, z in section]
        rings.append(ring)
    n = len(rings[0])
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])
    for ring in (rings[0], rings[-1]):
        pts = [geom.vertices[i] for i in ring]
        cx = sum(p[0] for p in pts) / n
        cy = sum(p[1] for p in pts) / n
        cz = sum(p[2] for p in pts) / n
        center = geom.add_vertex(cx, cy, cz)
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(center, ring[i], ring[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flagship_tilt_head_stand_mixer")

    enamel = model.material("champagne_enamel", rgba=(0.83, 0.73, 0.58, 1.0))
    dark_trim = model.material("black_phenolic", rgba=(0.025, 0.023, 0.022, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.88, 0.88, 0.84, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.96, 0.94, 0.88, 1.0))
    shadow = model.material("dark_shadow", rgba=(0.06, 0.055, 0.05, 1.0))

    def rounded_section(x_center: float, z: float, width_x: float, width_y: float, radius: float):
        return [(x_center + x, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius, corner_segments=8)]

    def yz_section(x: float, z_center: float, width_y: float, height_z: float, radius: float):
        return [(x, y, z_center + z) for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=10)]

    base = model.part("base")
    base_plate = _loft_geometry(
        [
            rounded_section(0.08, 0.0, 0.68, 0.34, 0.075),
            rounded_section(0.08, 0.028, 0.65, 0.325, 0.070),
            rounded_section(0.08, 0.055, 0.60, 0.300, 0.060),
        ]
    )
    base.visual(
        mesh_from_geometry(base_plate, "base_plate"),
        material=enamel,
        name="base_plate",
    )

    pedestal = _loft_geometry(
        [
            rounded_section(-0.13, 0.052, 0.235, 0.255, 0.060),
            rounded_section(-0.115, 0.145, 0.190, 0.215, 0.055),
            rounded_section(-0.135, 0.275, 0.145, 0.185, 0.047),
            rounded_section(-0.175, 0.392, 0.118, 0.160, 0.040),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal, "pedestal"),
        material=enamel,
        name="sculpted_pedestal",
    )
    for y in (-0.107, 0.107):
        base.visual(
            Box((0.090, 0.036, 0.086)),
            origin=Origin(xyz=(-0.180, y, 0.470)),
            material=enamel,
            name=f"hinge_cheek_{'neg' if y < 0 else 'pos'}",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.060),
            origin=Origin(xyz=(-0.180, y, 0.510), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=enamel,
            name=f"hinge_barrel_{'neg' if y < 0 else 'pos'}",
        )
    base.visual(
        Box((0.105, 0.240, 0.030)),
        origin=Origin(xyz=(-0.180, 0.0, 0.432)),
        material=enamel,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.088, 0.170, 0.060)),
        origin=Origin(xyz=(-0.180, 0.0, 0.405)),
        material=enamel,
        name="hinge_plinth",
    )

    base.visual(
        Box((0.270, 0.026, 0.010)),
        origin=Origin(xyz=(0.260, -0.092, 0.060)),
        material=chrome,
        name="bowl_slide_rail_neg",
    )
    base.visual(
        Box((0.270, 0.026, 0.010)),
        origin=Origin(xyz=(0.260, 0.092, 0.060)),
        material=chrome,
        name="bowl_slide_rail_pos",
    )
    base.visual(
        Box((0.190, 0.120, 0.006)),
        origin=Origin(xyz=(0.270, 0.0, 0.058)),
        material=shadow,
        name="cradle_shadow_slot",
    )
    base.visual(
        Box((0.085, 0.052, 0.095)),
        origin=Origin(xyz=(-0.065, -0.104, 0.255)),
        material=dark_trim,
        name="speed_control_plinth",
    )
    base.visual(
        Box((0.070, 0.052, 0.040)),
        origin=Origin(xyz=(-0.180, 0.104, 0.335)),
        material=dark_trim,
        name="head_lock_slot",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.68, 0.34, 0.45)),
        mass=8.0,
        origin=Origin(xyz=(0.02, 0.0, 0.20)),
    )

    bowl_carriage = model.part("bowl_carriage")
    cradle_plate = _loft_geometry(
        [
            rounded_section(0.018, 0.000, 0.255, 0.235, 0.045),
            rounded_section(0.018, 0.012, 0.242, 0.222, 0.040),
            rounded_section(0.018, 0.024, 0.215, 0.198, 0.034),
        ]
    )
    bowl_carriage.visual(
        mesh_from_geometry(cradle_plate, "cradle_plate"),
        material=dark_trim,
        name="cradle_plate",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(0.050, 0.0, 0.031)),
        material=chrome,
        name="bowl_twist_ring",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.038, 0.000),
            (0.070, 0.012),
            (0.118, 0.070),
            (0.142, 0.148),
            (0.148, 0.188),
        ],
        inner_profile=[
            (0.020, 0.008),
            (0.061, 0.020),
            (0.106, 0.074),
            (0.132, 0.150),
            (0.139, 0.180),
        ],
        segments=32,
        lip_samples=8,
    )
    bowl_carriage.visual(
        mesh_from_geometry(bowl_shell, "bowl_shell"),
        origin=Origin(xyz=(0.050, 0.0, 0.038)),
        material=stainless,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        mesh_from_geometry(TorusGeometry(0.145, 0.004, radial_segments=32, tubular_segments=10), "rolled_bowl_lip"),
        origin=Origin(xyz=(0.050, 0.0, 0.225)),
        material=chrome,
        name="rolled_bowl_lip",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=0.23),
        mass=1.4,
        origin=Origin(xyz=(0.05, 0.0, 0.125)),
    )

    head = model.part("head")
    head_shell = _loft_geometry(
        [
            yz_section(0.000, 0.060, 0.135, 0.120, 0.042),
            yz_section(0.115, 0.030, 0.210, 0.168, 0.060),
            yz_section(0.315, -0.018, 0.245, 0.188, 0.072),
            yz_section(0.495, -0.070, 0.180, 0.150, 0.058),
            yz_section(0.555, -0.095, 0.118, 0.104, 0.040),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        material=enamel,
        name="rounded_head_shell",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="center_hinge_barrel",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.548, 0.0, -0.104)),
        material=chrome,
        name="nose_bezel",
    )
    head.visual(
        Cylinder(radius=0.031, length=0.110),
        origin=Origin(xyz=(0.548, 0.0, -0.153)),
        material=enamel,
        name="drive_boss",
    )
    head.visual(
        Box((0.150, 0.012, 0.040)),
        origin=Origin(xyz=(0.245, 0.126, -0.025)),
        material=chrome,
        name="side_name_band",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.58, 0.25, 0.19)),
        mass=4.2,
        origin=Origin(xyz=(0.28, 0.0, -0.03)),
    )

    dough_hook = model.part("dough_hook")
    dough_hook.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=chrome,
        name="drive_collar",
    )
    dough_hook.visual(
        Cylinder(radius=0.010, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=stainless,
        name="tool_shank",
    )
    hook_path = [
        (0.000, 0.000, -0.102),
        (0.016, 0.000, -0.112),
        (0.032, 0.013, -0.135),
        (0.019, 0.039, -0.158),
        (-0.021, 0.036, -0.176),
        (-0.044, 0.002, -0.185),
        (-0.024, -0.034, -0.174),
        (0.024, -0.033, -0.151),
        (0.046, -0.004, -0.126),
        (0.034, 0.020, -0.110),
    ]
    hook_geom = tube_from_spline_points(
        hook_path,
        radius=0.0065,
        samples_per_segment=16,
        radial_segments=20,
    )
    dough_hook.visual(
        mesh_from_geometry(hook_geom, "spiral_hook"),
        material=stainless,
        name="spiral_hook",
    )
    dough_hook.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.195),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.033, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_dial",
    )
    speed_control.visual(
        Box((0.004, 0.004, 0.042)),
        origin=Origin(xyz=(0.010, -0.029, 0.010), rpy=(0.0, 0.0, math.radians(25.0))),
        material=chrome,
        name="speed_indicator_line",
    )
    speed_control.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="speed_pointer_cap",
    )
    speed_control.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.035),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.055, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=dark_trim,
        name="lock_thumb_slide",
    )
    head_lock.visual(
        Box((0.018, 0.026, 0.012)),
        origin=Origin(xyz=(-0.018, 0.037, 0.0)),
        material=chrome,
        name="lock_indicator_tab",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.318, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=-0.025, upper=0.055),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.180, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=0.0, upper=math.radians(62.0)),
    )
    model.articulation(
        "head_to_dough_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=dough_hook,
        origin=Origin(xyz=(0.548, 0.0, -0.208)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.065, -0.130, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.2, lower=math.radians(-38.0), upper=math.radians(38.0)),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.180, 0.130, 0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    dough_hook = object_model.get_part("dough_hook")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    tilt = object_model.get_articulation("base_to_head")
    tool_spin = object_model.get_articulation("head_to_dough_hook")
    speed_dial = object_model.get_articulation("base_to_speed_control")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.allow_overlap(
        bowl_carriage,
        dough_hook,
        elem_a="bowl_shell",
        elem_b="spiral_hook",
        reason=(
            "The spiral hook works inside the open stainless bowl; the thin concave "
            "bowl mesh is treated conservatively by overlap QC as a proxy."
        ),
    )

    ctx.check("bowl carriage uses short prismatic travel", bowl_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tilt head uses rear revolute hinge", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("tool drive is continuous rotary", tool_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("speed control is base revolute", speed_dial.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("head lock is base prismatic", lock_slide.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_contact(
        bowl_carriage,
        base,
        elem_a="cradle_plate",
        elem_b="bowl_slide_rail_pos",
        contact_tol=0.001,
        name="sliding cradle rests on raised rails",
    )
    ctx.expect_within(
        dough_hook,
        bowl_carriage,
        axes="xy",
        inner_elem="spiral_hook",
        outer_elem="bowl_shell",
        margin=0.012,
        name="spiral hook is centered within the bowl footprint",
    )
    ctx.expect_overlap(
        dough_hook,
        bowl_carriage,
        axes="z",
        elem_a="spiral_hook",
        elem_b="bowl_shell",
        min_overlap=0.070,
        name="spiral hook hangs down into the open bowl",
    )
    ctx.expect_gap(
        base,
        speed_control,
        axis="y",
        positive_elem="speed_control_plinth",
        negative_elem="speed_dial",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed dial seats on base plinth",
    )
    ctx.expect_gap(
        head_lock,
        base,
        axis="y",
        positive_elem="lock_thumb_slide",
        negative_elem="head_lock_slot",
        max_gap=0.001,
        max_penetration=0.0,
        name="head lock slide sits in base slot",
    )

    rest_carriage = ctx.part_world_position(bowl_carriage)
    with ctx.pose({bowl_slide: bowl_slide.motion_limits.upper}):
        extended_carriage = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage slides forward",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.04,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    closed_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "tilt head raises the tool end",
        closed_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > closed_head_aabb[1][2] + 0.15,
        details=f"closed={closed_head_aabb}, tilted={tilted_head_aabb}",
    )

    rest_lock = ctx.part_world_position(head_lock)
    with ctx.pose({lock_slide: lock_slide.motion_limits.upper}):
        out_lock = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock travels outward",
        rest_lock is not None and out_lock is not None and out_lock[1] > rest_lock[1] + 0.012,
        details=f"rest={rest_lock}, out={out_lock}",
    )

    return ctx.report()


object_model = build_object_model()
