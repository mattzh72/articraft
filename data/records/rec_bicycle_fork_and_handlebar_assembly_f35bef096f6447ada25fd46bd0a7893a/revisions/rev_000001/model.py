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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _ring_between(z0: float, z1: float, outer_radius: float, inner_radius: float):
    height = z1 - z0
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate((0.0, 0.0, z0))
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.006)
        .translate((0.0, 0.0, z0 - 0.003))
    )
    return outer.cut(cutter)


def _head_tube_shape():
    main = _ring_between(0.0, 0.225, 0.035, 0.0185)
    lower_cup = _ring_between(-0.016, 0.020, 0.045, 0.0185)
    upper_cup = _ring_between(0.205, 0.242, 0.043, 0.0185)
    return main.union(lower_cup).union(upper_cup)


def _stem_clamp_shape():
    collar = _ring_between(0.285, 0.395, 0.030, 0.0110)
    rear_ear = cq.Workplane("XY").box(0.020, 0.070, 0.088).translate((-0.032, 0.0, 0.340))
    slot = cq.Workplane("XY").box(0.018, 0.013, 0.098).translate((-0.032, 0.0, 0.340))
    return collar.union(rear_ear).cut(slot)


def _face_collar_shape():
    body = cq.Workplane("XY").box(0.080, 0.145, 0.070).translate((0.215, 0.0, 0.515))
    bar_hole = cq.Workplane("XY").box(0.028, 0.165, 0.020).translate((0.215, 0.0, 0.515))
    face_slot = cq.Workplane("XY").box(0.018, 0.170, 0.080).translate((0.247, 0.0, 0.515))
    return body.cut(bar_hole).cut(face_slot)


def _rack_platform_shape():
    deck = cq.Workplane("XY").box(0.440, 0.580, 0.014).translate((0.415, 0.0, -0.300))
    front_lip = cq.Workplane("XY").box(0.026, 0.610, 0.042).translate((0.645, 0.0, -0.276))
    rear_lip = cq.Workplane("XY").box(0.026, 0.610, 0.030).translate((0.185, 0.0, -0.282))
    side_lip_0 = cq.Workplane("XY").box(0.452, 0.026, 0.036).translate((0.415, 0.305, -0.279))
    side_lip_1 = cq.Workplane("XY").box(0.452, 0.026, 0.036).translate((0.415, -0.305, -0.279))
    return deck.union(front_lip).union(rear_lip).union(side_lip_0).union(side_lip_1)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_bike_front_end")

    powder_blue = model.material("powder_blue", rgba=(0.05, 0.29, 0.47, 1.0))
    black = model.material("satin_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.65, 0.67, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    lowrider_mount_names = (
        ("lowrider_mount_0_0", "lowrider_mount_0_1"),
        ("lowrider_mount_1_0", "lowrider_mount_1_1"),
    )
    rack_strut_names = (
        ("rack_strut_0_0", "rack_strut_0_1"),
        ("rack_strut_1_0", "rack_strut_1_1"),
    )
    rack_bolt_names = (
        ("rack_bolt_0_0", "rack_bolt_0_1"),
        ("rack_bolt_1_0", "rack_bolt_1_1"),
    )

    head_tube = model.part("head_tube")
    head_tube.visual(
        mesh_from_cadquery(_head_tube_shape(), "hollow_head_tube", tolerance=0.0008),
        material=dark_steel,
        name="hollow_shell",
    )

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.013, length=0.595),
        origin=Origin(xyz=(0.0, 0.0, 0.2425)),
        material=brushed_steel,
        name="straight_steerer",
    )
    fork.visual(
        Cylinder(radius=0.0196, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="lower_bearing_cone",
    )
    fork.visual(
        Cylinder(radius=0.0196, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.219)),
        material=brushed_steel,
        name="upper_bearing_cone",
    )
    fork.visual(
        Box((0.105, 0.215, 0.054)),
        origin=Origin(xyz=(0.022, 0.0, -0.068)),
        material=powder_blue,
        name="unicrown_block",
    )
    crown_arch = tube_from_spline_points(
        [
            (0.005, -0.108, -0.080),
            (0.012, -0.052, -0.054),
            (0.000, 0.0, -0.044),
            (0.012, 0.052, -0.054),
            (0.005, 0.108, -0.080),
        ],
        radius=0.020,
        samples_per_segment=14,
        radial_segments=18,
    )
    fork.visual(mesh_from_geometry(crown_arch, "unicrown_arch"), material=powder_blue, name="unicrown_arch")

    blade_profile = rounded_rect_profile(0.044, 0.020, 0.007, corner_segments=8)
    for index, side in enumerate((-1.0, 1.0)):
        blade = sweep_profile_along_spline(
            [
                (0.010, side * 0.070, -0.078),
                (0.035, side * 0.105, -0.180),
                (0.078, side * 0.145, -0.440),
                (0.095, side * 0.162, -0.660),
            ],
            profile=blade_profile,
            samples_per_segment=18,
            up_hint=(0.0, side, 0.25),
        )
        fork.visual(mesh_from_geometry(blade, f"curved_blade_{index}"), material=powder_blue, name=f"blade_{index}")
        fork.visual(
            Box((0.055, 0.014, 0.072)),
            origin=Origin(xyz=(0.098, side * 0.168, -0.682)),
            material=dark_steel,
            name=f"dropout_{index}",
        )
        fork.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(xyz=(0.100, side * 0.168, -0.660), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"axle_eye_{index}",
        )
        for mount_index, (mx, mz) in enumerate(((0.070, -0.390), (0.098, -0.520))):
            fork.visual(
                Cylinder(radius=0.015, length=0.070),
                origin=Origin(xyz=(mx, side * 0.175, mz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=powder_blue,
                name=lowrider_mount_names[index][mount_index],
            )

    stem = model.part("stem")
    stem.visual(
        mesh_from_cadquery(_stem_clamp_shape(), "steerer_clamp", tolerance=0.0008),
        material=black,
        name="steerer_clamp",
    )
    riser_geom = tube_from_spline_points(
        [
            (0.045, 0.0, 0.360),
            (0.083, 0.0, 0.430),
            (0.130, 0.0, 0.492),
            (0.170, 0.0, 0.515),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
    )
    stem.visual(mesh_from_geometry(riser_geom, "riser_stem_neck"), material=black, name="riser_neck")
    stem.visual(
        mesh_from_cadquery(_face_collar_shape(), "face_collar", tolerance=0.0008),
        material=black,
        name="face_collar",
    )
    for bolt_index, z in enumerate((0.540, 0.490)):
        stem.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.226, -0.047, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"face_bolt_{bolt_index}",
        )
        stem.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.226, 0.047, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"face_bolt_{bolt_index + 2}",
        )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Box((0.030, 0.930, 0.022)),
        origin=Origin(xyz=(0.215, 0.0, 0.515)),
        material=dark_steel,
        name="flat_bar",
    )
    handlebar.visual(
        Box((0.036, 0.135, 0.030)),
        origin=Origin(xyz=(0.215, -0.497, 0.515)),
        material=rubber,
        name="grip_0",
    )
    handlebar.visual(
        Box((0.036, 0.135, 0.030)),
        origin=Origin(xyz=(0.215, 0.497, 0.515)),
        material=rubber,
        name="grip_1",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.215, -0.568, 0.515), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="bar_end_0",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.215, 0.568, 0.515), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="bar_end_1",
    )

    rack = model.part("front_rack")
    rack.visual(
        mesh_from_cadquery(_rack_platform_shape(), "front_rack_platform", tolerance=0.0008),
        material=black,
        name="platform",
    )
    for index, side in enumerate((-1.0, 1.0)):
        outer_y = side * 0.222
        rail_y = side * 0.258
        for mount_index, (mx, mz, top_x) in enumerate(((0.070, -0.390, 0.270), (0.098, -0.520, 0.550))):
            _add_member(
                rack,
                (mx, outer_y, mz),
                (top_x, rail_y, -0.307),
                0.0075,
                black,
                name=rack_strut_names[index][mount_index],
            )
            rack.visual(
                Cylinder(radius=0.009, length=0.030),
                origin=Origin(xyz=(mx, side * 0.207, mz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brushed_steel,
                name=rack_bolt_names[index][mount_index],
            )
        _add_member(
            rack,
            (0.270, rail_y, -0.307),
            (0.550, rail_y, -0.307),
            0.007,
            black,
            name=f"side_rail_{index}",
        )

    steering = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-1.20, upper=1.20),
    )
    steering.meta["description"] = "Head tube steering axis for the complete fork, rack, stem, and handlebar assembly."

    model.articulation("stem_mount", ArticulationType.FIXED, parent=fork, child=stem, origin=Origin())
    model.articulation("handlebar_clamp", ArticulationType.FIXED, parent=stem, child=handlebar, origin=Origin())
    model.articulation("rack_mount", ArticulationType.FIXED, parent=fork, child=rack, origin=Origin())
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    head_tube = object_model.get_part("head_tube")
    rack = object_model.get_part("front_rack")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    steering = object_model.get_articulation("steering")

    limits = steering.motion_limits
    ctx.check(
        "realistic steering limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < -1.0
        and limits.upper > 1.0,
        details=f"limits={limits}",
    )

    ctx.allow_overlap(
        fork,
        head_tube,
        elem_a="upper_bearing_cone",
        elem_b="hollow_shell",
        reason="The rotating headset bearing cone is intentionally seated inside the upper head-tube cup.",
    )
    ctx.allow_overlap(
        fork,
        head_tube,
        elem_a="lower_bearing_cone",
        elem_b="hollow_shell",
        reason="The rotating lower bearing cone is intentionally seated inside the lower head-tube cup.",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="z",
        elem_a="upper_bearing_cone",
        elem_b="hollow_shell",
        min_overlap=0.010,
        name="upper headset cone is seated in the head-tube cup",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="z",
        elem_a="lower_bearing_cone",
        elem_b="hollow_shell",
        min_overlap=0.010,
        name="lower headset cone is seated in the head-tube cup",
    )

    ctx.allow_overlap(
        fork,
        stem,
        elem_a="straight_steerer",
        elem_b="steerer_clamp",
        reason="The split threadless stem clamp intentionally grips the straight steerer tube.",
    )

    ctx.allow_overlap(
        handlebar,
        stem,
        elem_a="flat_bar",
        elem_b="face_collar",
        reason="The stem face collar intentionally clamps the wide flat handlebar.",
    )

    for index in (0, 1):
        for mount_index in (0, 1):
            ctx.allow_overlap(
                rack,
                fork,
                elem_a=f"rack_bolt_{index}_{mount_index}",
                elem_b=f"lowrider_mount_{index}_{mount_index}",
                reason="Rack bolt heads intentionally seat into the fork's threaded low-rider bosses.",
            )
            ctx.expect_contact(
                rack,
                fork,
                elem_a=f"rack_bolt_{index}_{mount_index}",
                elem_b=f"lowrider_mount_{index}_{mount_index}",
                contact_tol=0.006,
                name=f"rack bolt {index}-{mount_index} seats on low-rider boss",
            )

    ctx.expect_within(
        fork,
        stem,
        axes="xy",
        inner_elem="straight_steerer",
        outer_elem="steerer_clamp",
        margin=0.002,
        name="threadless steerer is centered in riser stem clamp",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="xz",
        inner_elem="flat_bar",
        outer_elem="face_collar",
        margin=0.002,
        name="flat handlebar passes through face collar",
    )
    ctx.expect_overlap(
        rack,
        fork,
        axes="z",
        elem_a="rack_strut_0_0",
        elem_b="lowrider_mount_0_0",
        min_overlap=0.010,
        name="rack strut reaches lower low-rider mount height",
    )

    def _center_y_from_aabb(aabb) -> float | None:
        if aabb is None:
            return None
        lo, hi = aabb
        try:
            return (lo[1] + hi[1]) * 0.5
        except TypeError:
            return (lo.y + hi.y) * 0.5

    rest_bar = ctx.part_element_world_aabb(handlebar, elem="flat_bar")
    with ctx.pose({steering: math.radians(35.0)}):
        turned_bar = ctx.part_element_world_aabb(handlebar, elem="flat_bar")
    rest_y = _center_y_from_aabb(rest_bar)
    turned_y = _center_y_from_aabb(turned_bar)
    ctx.check(
        "handlebar rotates with steering",
        rest_y is not None and turned_y is not None and abs(turned_y - rest_y) > 0.10,
        details=f"rest_y={rest_y}, turned_y={turned_y}",
    )

    return ctx.report()


object_model = build_object_model()
