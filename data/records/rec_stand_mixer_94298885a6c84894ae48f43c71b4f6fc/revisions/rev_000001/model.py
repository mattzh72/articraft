from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _add_cylinder_between(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _section_yz(
    *,
    x: float,
    y_width: float,
    z_height: float,
    z_center: float,
    radius: float,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(y_width, z_height, radius, corner_segments=samples)
    ]


def _section_xy(
    *,
    z: float,
    x_width: float,
    y_width: float,
    x_center: float,
    radius: float,
    samples: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x_center + x, y, z)
        for x, y in rounded_rect_profile(x_width, y_width, radius, corner_segments=samples)
    ]


def _build_head_mesh() -> MeshGeometry:
    return section_loft(
        [
            _section_yz(x=0.018, y_width=0.150, z_height=0.115, z_center=-0.018, radius=0.034),
            _section_yz(x=0.145, y_width=0.220, z_height=0.158, z_center=-0.024, radius=0.052),
            _section_yz(x=0.310, y_width=0.215, z_height=0.150, z_center=-0.035, radius=0.050),
            _section_yz(x=0.455, y_width=0.155, z_height=0.115, z_center=-0.046, radius=0.038),
        ],
        repair="mesh",
    )


def _build_neck_mesh() -> MeshGeometry:
    return section_loft(
        [
            _section_xy(z=0.052, x_width=0.170, y_width=0.205, x_center=-0.205, radius=0.045),
            _section_xy(z=0.165, x_width=0.140, y_width=0.185, x_center=-0.232, radius=0.044),
            _section_xy(z=0.305, x_width=0.100, y_width=0.165, x_center=-0.258, radius=0.040),
            _section_xy(z=0.405, x_width=0.065, y_width=0.150, x_center=-0.255, radius=0.032),
        ],
        repair="mesh",
    )


def _build_dough_hook_mesh() -> MeshGeometry:
    hook = tube_from_spline_points(
        [
            (0.000, 0.000, -0.060),
            (0.000, 0.000, -0.095),
            (0.016, 0.010, -0.126),
            (0.045, 0.022, -0.150),
            (0.066, 0.000, -0.172),
            (0.052, -0.035, -0.180),
            (0.016, -0.052, -0.164),
            (-0.015, -0.036, -0.133),
            (0.002, -0.008, -0.118),
        ],
        radius=0.0065,
        samples_per_segment=14,
        radial_segments=18,
        up_hint=(0.0, 0.0, 1.0),
    )
    hook.merge(CylinderGeometry(radius=0.007, height=0.074).translate(0.0, 0.0, -0.037))
    hook.merge(CylinderGeometry(radius=0.016, height=0.026).translate(0.0, 0.0, -0.072))
    return hook


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_countertop_stand_mixer")

    pearl = model.material("warm_pearl_enamel", rgba=(0.86, 0.83, 0.77, 1.0))
    pearl_shadow = model.material("shadowed_enamel", rgba=(0.70, 0.68, 0.62, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.77, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.055, 0.057, 0.060, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    slot_dark = model.material("recess_black", rgba=(0.015, 0.016, 0.018, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.90, 0.91, 0.90, 1.0))

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.565, 0.345, 0.070, corner_segments=12), 0.055),
        "low_rounded_base",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.0275)),
        material=pearl,
        name="base_plate",
    )
    base.visual(
        Box((0.480, 0.020, 0.014)),
        origin=Origin(xyz=(0.070, -0.176, 0.028)),
        material=pearl_shadow,
        name="front_shadow_seam",
    )
    base.visual(
        Box((0.090, 0.026, 0.002)),
        origin=Origin(xyz=(-0.210, 0.000, 0.054)),
        material=slot_dark,
        name="lock_slot",
    )
    base.visual(
        Box((0.082, 0.012, 0.002)),
        origin=Origin(xyz=(0.270, -0.128, 0.054)),
        material=slot_dark,
        name="speed_index_arc",
    )

    neck_mesh = mesh_from_geometry(_build_neck_mesh(), "rear_streamlined_neck")
    base.visual(neck_mesh, material=pearl, name="rear_neck")

    for index, y in enumerate((-0.105, 0.105)):
        _add_cylinder_between(
            base,
            (-0.065, y, 0.063),
            (0.215, y, 0.063),
            0.008,
            chrome,
            name=f"slide_rail_{index}",
        )
        base.visual(
            Box((0.030, 0.030, 0.014)),
            origin=Origin(xyz=(-0.060, y, 0.057)),
            material=pearl_shadow,
            name=f"rear_rail_pedestal_{index}",
        )
        base.visual(
            Box((0.030, 0.030, 0.014)),
            origin=Origin(xyz=(0.210, y, 0.057)),
            material=pearl_shadow,
            name=f"front_rail_pedestal_{index}",
        )

    for index, y in enumerate((-0.116, 0.116)):
        base.visual(
            Cylinder(radius=0.033, length=0.040),
            origin=Origin(xyz=(-0.205, y, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pearl_shadow,
            name=f"hinge_lug_{index}",
        )
        base.visual(
            Box((0.036, 0.028, 0.072)),
            origin=Origin(xyz=(-0.224, y * 0.86, 0.396)),
            material=pearl_shadow,
            name=f"hinge_cheek_{index}",
        )
    base.visual(
        Box((0.040, 0.198, 0.035)),
        origin=Origin(xyz=(-0.246, 0.0, 0.388)),
        material=pearl_shadow,
        name="hinge_bridge",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.245),
        origin=Origin(xyz=(-0.205, 0.0, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    for index, x in enumerate((-0.130, 0.245)):
        for y in (-0.125, 0.125):
            base.visual(
                Cylinder(radius=0.025, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=dark_rubber,
                name=f"rubber_foot_{index}_{0 if y < 0 else 1}",
            )

    bowl_carriage = model.part("bowl_carriage")
    carriage_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.250, 0.220, 0.030, corner_segments=8), 0.012),
        "bowl_carriage_plate",
    )
    bowl_carriage.visual(
        carriage_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=pearl_shadow,
        name="carriage_plate",
    )
    for index, y in enumerate((-0.105, 0.105)):
        bowl_carriage.visual(
            Box((0.072, 0.034, 0.012)),
            origin=Origin(xyz=(0.000, y, 0.077)),
            material=pearl_shadow,
            name=f"rail_saddle_{index}",
        )
    bowl_carriage.visual(
        Cylinder(radius=0.076, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=chrome,
        name="bowl_twist_ring",
    )
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.050, 0.000),
            (0.073, 0.014),
            (0.112, 0.060),
            (0.142, 0.145),
            (0.151, 0.190),
            (0.148, 0.203),
        ],
        [
            (0.024, 0.009),
            (0.058, 0.022),
            (0.101, 0.062),
            (0.131, 0.145),
            (0.138, 0.192),
        ],
        segments=72,
        lip_samples=8,
    )
    bowl_carriage.visual(
        mesh_from_geometry(bowl_geom, "deep_open_brushed_bowl"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=brushed_steel,
        name="bowl_shell",
    )
    bowl_carriage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.148, tube=0.0045, radial_segments=18, tubular_segments=96), "bowl_rolled_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.296)),
        material=brushed_steel,
        name="rolled_lip",
    )
    for index, z in enumerate((0.168, 0.214, 0.258)):
        bowl_carriage.visual(
            mesh_from_geometry(TorusGeometry(radius=(0.118, 0.134, 0.146)[index], tube=0.0018, radial_segments=8, tubular_segments=80), f"brushed_bowl_band_{index}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"brushed_band_{index}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_mesh(), "tilt_head_shell"),
        material=pearl,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.134),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pearl_shadow,
        name="head_hinge_knuckle",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.335, 0.0, -0.112)),
        material=chrome,
        name="planetary_trim",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.335, 0.0, -0.122)),
        material=slot_dark,
        name="tool_socket_shadow",
    )
    head.visual(
        Box((0.105, 0.010, 0.014)),
        origin=Origin(xyz=(0.200, -0.113, -0.020)),
        material=chrome,
        name="side_speed_scale",
    )

    tool_shaft = model.part("tool_shaft")
    tool_shaft.visual(
        mesh_from_geometry(_build_dough_hook_mesh(), "spiral_dough_hook"),
        material=brushed_steel,
        name="dough_hook",
    )
    tool_shaft.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=chrome,
        name="upper_coupler",
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black_plastic,
        name="selector_dial",
    )
    speed_selector.visual(
        Box((0.006, 0.032, 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.0195)),
        material=chrome,
        name="selector_pointer",
    )
    speed_selector.visual(
        Cylinder(radius=0.010, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=chrome,
        name="selector_cap",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.040, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black_plastic,
        name="lock_slider",
    )
    head_lock.visual(
        Box((0.018, 0.030, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=chrome,
        name="lock_thumb_grip",
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.10, lower=-0.025, upper=0.045),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.205, 0.0, 0.430)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.8, lower=0.0, upper=math.radians(58.0)),
    )
    model.articulation(
        "head_to_tool_shaft",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool_shaft,
        origin=Origin(xyz=(0.335, 0.0, -0.134)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.270, -0.128, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.210, 0.000, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=-0.018, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    tool_shaft = object_model.get_part("tool_shaft")
    speed_selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    bowl_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_tilt = object_model.get_articulation("base_to_head")
    shaft_spin = object_model.get_articulation("head_to_tool_shaft")
    speed_joint = object_model.get_articulation("base_to_speed_selector")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    ctx.allow_overlap(
        bowl_carriage,
        tool_shaft,
        elem_a="bowl_shell",
        elem_b="dough_hook",
        reason=(
            "The dough hook is intentionally positioned in the open bowl working volume; "
            "the thin lathed bowl visual is treated as a closed proxy by exact overlap QC."
        ),
    )
    ctx.allow_overlap(
        head,
        tool_shaft,
        elem_a="planetary_trim",
        elem_b="upper_coupler",
        reason="The rotating tool coupler is intentionally captured through the planetary trim socket.",
    )
    ctx.allow_overlap(
        head,
        tool_shaft,
        elem_a="tool_socket_shadow",
        elem_b="upper_coupler",
        reason="The coupler intentionally passes through the dark socket bushing below the mixer head.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_pin",
        elem_b="head_hinge_knuckle",
        reason="The head hinge knuckle is intentionally captured around the rear horizontal hinge pin.",
    )
    ctx.check(
        "required articulation types",
        bowl_slide.articulation_type == ArticulationType.PRISMATIC
        and head_tilt.articulation_type == ArticulationType.REVOLUTE
        and shaft_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.expect_gap(
        bowl_carriage,
        base,
        axis="z",
        positive_elem="rail_saddle_0",
        negative_elem="slide_rail_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="carriage saddle rides on rail",
    )
    ctx.expect_overlap(
        bowl_carriage,
        base,
        axes="x",
        elem_a="rail_saddle_0",
        elem_b="slide_rail_0",
        min_overlap=0.060,
        name="bowl slide has retained rail engagement",
    )
    ctx.expect_within(
        tool_shaft,
        bowl_carriage,
        axes="xy",
        inner_elem="dough_hook",
        outer_elem="bowl_shell",
        margin=0.002,
        name="dough hook stays centered inside bowl footprint",
    )
    ctx.expect_overlap(
        tool_shaft,
        bowl_carriage,
        axes="z",
        elem_a="dough_hook",
        elem_b="bowl_shell",
        min_overlap=0.090,
        name="dough hook reaches into the bowl",
    )
    ctx.expect_overlap(
        tool_shaft,
        head,
        axes="xy",
        elem_a="upper_coupler",
        elem_b="planetary_trim",
        min_overlap=0.018,
        name="tool coupler is centered in planetary trim",
    )
    ctx.expect_overlap(
        tool_shaft,
        head,
        axes="xy",
        elem_a="upper_coupler",
        elem_b="tool_socket_shadow",
        min_overlap=0.018,
        name="tool coupler passes through socket bushing",
    )
    ctx.expect_overlap(
        base,
        head,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="head_hinge_knuckle",
        min_overlap=0.020,
        name="head knuckle surrounds rear hinge pin",
    )
    ctx.expect_gap(
        speed_selector,
        base,
        axis="z",
        positive_elem="selector_dial",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.00001,
        name="speed selector sits on base",
    )
    ctx.expect_gap(
        head_lock,
        base,
        axis="z",
        positive_elem="lock_slider",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.00001,
        name="head lock slider rides in base slot",
    )

    bowl_rest = ctx.part_world_position(bowl_carriage)
    lock_rest = ctx.part_world_position(head_lock)
    tool_rest = ctx.part_world_position(tool_shaft)
    with ctx.pose({bowl_slide: 0.045, lock_slide: 0.018, head_tilt: math.radians(58.0)}):
        ctx.expect_gap(
            bowl_carriage,
            base,
            axis="z",
            positive_elem="rail_saddle_0",
            negative_elem="slide_rail_0",
            max_gap=0.001,
            max_penetration=0.00001,
            name="extended carriage remains supported",
        )
        ctx.expect_overlap(
            bowl_carriage,
            base,
            axes="x",
            elem_a="rail_saddle_0",
            elem_b="slide_rail_0",
            min_overlap=0.045,
            name="extended bowl carriage remains captured on rail",
        )
        bowl_forward = ctx.part_world_position(bowl_carriage)
        lock_forward = ctx.part_world_position(head_lock)
        tool_raised = ctx.part_world_position(tool_shaft)

    ctx.check(
        "bowl carriage slides fore-aft",
        bowl_rest is not None and bowl_forward is not None and bowl_forward[0] > bowl_rest[0] + 0.035,
        details=f"rest={bowl_rest}, forward={bowl_forward}",
    )
    ctx.check(
        "head lock translates on base",
        lock_rest is not None and lock_forward is not None and lock_forward[0] > lock_rest[0] + 0.012,
        details=f"rest={lock_rest}, translated={lock_forward}",
    )
    ctx.check(
        "tilt head raises tool shaft",
        tool_rest is not None and tool_raised is not None and tool_raised[2] > tool_rest[2] + 0.18,
        details=f"rest={tool_rest}, raised={tool_raised}",
    )

    return ctx.report()


object_model = build_object_model()
