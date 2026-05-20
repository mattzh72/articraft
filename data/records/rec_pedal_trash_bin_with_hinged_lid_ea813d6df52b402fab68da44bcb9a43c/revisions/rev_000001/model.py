from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rounded_rect_loop(size_x: float, size_y: float, radius: float, *, segments: int = 5):
    """Clockwise rounded rectangle loop in the XY plane, centered on the origin."""

    hx = size_x / 2.0
    hy = size_y / 2.0
    r = min(radius, hx - 1e-4, hy - 1e-4)
    centers = (
        (hx - r, hy - r, 0.0, math.pi / 2.0),
        (-hx + r, hy - r, math.pi / 2.0, math.pi),
        (-hx + r, -hy + r, math.pi, 3.0 * math.pi / 2.0),
        (hx - r, -hy + r, 3.0 * math.pi / 2.0, 2.0 * math.pi),
    )
    pts = []
    for cx, cy, a0, a1 in centers:
        for i in range(segments + 1):
            if pts and i == 0:
                continue
            a = a0 + (a1 - a0) * (i / segments)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _add_loop(geom: MeshGeometry, loop, z: float):
    return [geom.add_vertex(x, y, z) for x, y in loop]


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int):
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _shell_geometry(
    *,
    bottom_x: float,
    bottom_y: float,
    top_x: float,
    top_y: float,
    height: float,
    wall: float,
    bottom_thickness: float,
    radius: float,
    segments: int = 6,
) -> MeshGeometry:
    """Thin-walled tapered rectangular shell with an open top and rounded corners."""

    geom = MeshGeometry()
    outer_bottom_loop = _rounded_rect_loop(bottom_x, bottom_y, radius * 0.75, segments=segments)
    outer_top_loop = _rounded_rect_loop(top_x, top_y, radius, segments=segments)
    inner_bottom_loop = _rounded_rect_loop(
        max(bottom_x - 2.0 * wall, 0.01),
        max(bottom_y - 2.0 * wall, 0.01),
        max(radius * 0.75 - wall, 0.002),
        segments=segments,
    )
    inner_top_loop = _rounded_rect_loop(
        max(top_x - 2.0 * wall, 0.01),
        max(top_y - 2.0 * wall, 0.01),
        max(radius - wall, 0.002),
        segments=segments,
    )

    outer_bottom = _add_loop(geom, outer_bottom_loop, 0.0)
    outer_top = _add_loop(geom, outer_top_loop, height)
    inner_bottom = _add_loop(geom, inner_bottom_loop, bottom_thickness)
    inner_top = _add_loop(geom, inner_top_loop, height - 0.004)
    n = len(outer_top)
    center_floor = geom.add_vertex(0.0, 0.0, bottom_thickness)

    for i in range(n):
        j = (i + 1) % n
        # Exterior drafted side.
        _quad(geom, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        # Interior wall.
        _quad(geom, inner_top[i], inner_top[j], inner_bottom[j], inner_bottom[i])
        # Rolled top rim.
        _quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        # Sloped bottom floor/radius area.
        _quad(geom, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
        # Bucket/base floor.
        geom.add_face(center_floor, inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clinical_waste_bin")

    white_plastic = model.material("warm_white_plastic", rgba=(0.90, 0.90, 0.84, 1.0))
    yellow_plastic = model.material("clinical_yellow_lid", rgba=(1.0, 0.74, 0.05, 1.0))
    grey_plastic = model.material("inner_bucket_grey", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_rubber = model.material("dark_textured_rubber", rgba=(0.06, 0.065, 0.06, 1.0))
    metal = model.material("brushed_pin_metal", rgba=(0.62, 0.64, 0.64, 1.0))
    label_red = model.material("clinical_waste_label", rgba=(0.83, 0.12, 0.07, 1.0))
    label_black = model.material("label_black_marks", rgba=(0.01, 0.01, 0.01, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.025, 0.430, 0.780)),
        origin=Origin(xyz=(-0.1775, 0.0, 0.390)),
        material=white_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.025, 0.430, 0.780)),
        origin=Origin(xyz=(0.1775, 0.0, 0.390)),
        material=white_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.355, 0.025, 0.780)),
        origin=Origin(xyz=(0.0, -0.2025, 0.390)),
        material=white_plastic,
        name="side_wall_0",
    )
    body.visual(
        Box((0.355, 0.025, 0.780)),
        origin=Origin(xyz=(0.0, 0.2025, 0.390)),
        material=white_plastic,
        name="side_wall_1",
    )
    body.visual(
        Box((0.355, 0.430, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=white_plastic,
        name="base_floor",
    )
    body.visual(
        Box((0.060, 0.300, 0.018)),
        origin=Origin(xyz=(-0.182, 0.0, 0.028)),
        material=white_plastic,
        name="front_toe_recess_lip",
    )
    for idx, y in enumerate((-0.165, 0.165)):
        body.visual(
            Box((0.040, 0.035, 0.052)),
            origin=Origin(xyz=(-0.205, y, 0.073)),
            material=white_plastic,
            name=f"front_pivot_bracket_{idx}",
        )
    for idx, y in enumerate((-0.158, 0.158)):
        body.visual(
            Box((0.030, 0.090, 0.020)),
            origin=Origin(xyz=(0.181, y, 0.790)),
            material=white_plastic,
            name=f"rear_hinge_mount_{idx}",
        )
        body.visual(
            Cylinder(radius=0.017, length=0.082),
            origin=Origin(xyz=(0.181, y, 0.804), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=white_plastic,
            name=f"rear_hinge_knuckle_{idx}",
        )
    body.visual(
        Box((0.004, 0.225, 0.090)),
        origin=Origin(xyz=(-0.1785, 0.0, 0.485)),
        material=label_red,
        name="front_waste_label",
    )
    body.visual(
        Box((0.005, 0.045, 0.010)),
        origin=Origin(xyz=(-0.181, 0.0, 0.500)),
        material=label_black,
        name="label_bar_0",
    )
    body.visual(
        Box((0.005, 0.025, 0.010)),
        origin=Origin(xyz=(-0.181, -0.045, 0.468), rpy=(0.0, 0.0, 0.75)),
        material=label_black,
        name="label_bar_1",
    )
    body.visual(
        Box((0.005, 0.025, 0.010)),
        origin=Origin(xyz=(-0.181, 0.045, 0.468), rpy=(0.0, 0.0, -0.75)),
        material=label_black,
        name="label_bar_2",
    )

    inner_bucket = model.part("inner_bucket")
    inner_bucket.visual(
        mesh_from_geometry(
            _shell_geometry(
                bottom_x=0.205,
                bottom_y=0.260,
                top_x=0.270,
                top_y=0.324,
                height=0.615,
                wall=0.014,
                bottom_thickness=0.035,
                radius=0.025,
            ),
            "removable_inner_bucket_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=grey_plastic,
        name="bucket_shell",
    )
    inner_bucket.visual(
        Box((0.018, 0.344, 0.018)),
        origin=Origin(xyz=(-0.145, 0.0, 0.728)),
        material=grey_plastic,
        name="bucket_upper_rim",
    )
    inner_bucket.visual(
        Box((0.018, 0.344, 0.018)),
        origin=Origin(xyz=(0.145, 0.0, 0.728)),
        material=grey_plastic,
        name="bucket_rear_rim",
    )
    inner_bucket.visual(
        Box((0.290, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.163, 0.728)),
        material=grey_plastic,
        name="bucket_side_rim_0",
    )
    inner_bucket.visual(
        Box((0.290, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.163, 0.728)),
        material=grey_plastic,
        name="bucket_side_rim_1",
    )
    inner_bucket.visual(
        Box((0.120, 0.160, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=grey_plastic,
        name="bucket_bottom_plinth",
    )
    for boss_name, y in (("handle_pivot_boss_0", -0.174), ("handle_pivot_boss_1", 0.174)):
        inner_bucket.visual(
            Cylinder(radius=0.014, length=0.030),
            origin=Origin(xyz=(-0.120, y, 0.704), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grey_plastic,
            name=boss_name,
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.365, 0.470, 0.040)),
        origin=Origin(xyz=(-0.195, 0.0, 0.000)),
        material=yellow_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.024, 0.446, 0.050)),
        origin=Origin(xyz=(-0.386, 0.0, -0.010)),
        material=yellow_plastic,
        name="front_lid_lip",
    )
    lid.visual(
        Box((0.320, 0.024, 0.048)),
        origin=Origin(xyz=(-0.192, -0.246, -0.010)),
        material=yellow_plastic,
        name="side_lid_lip_0",
    )
    lid.visual(
        Box((0.320, 0.024, 0.048)),
        origin=Origin(xyz=(-0.192, 0.246, -0.010)),
        material=yellow_plastic,
        name="side_lid_lip_1",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.170),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow_plastic,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.230, 0.320, 0.004)),
        origin=Origin(xyz=(-0.198, 0.0, 0.021)),
        material=Material("subtle_lid_top_highlight", rgba=(1.0, 0.82, 0.18, 1.0)),
        name="raised_lid_center",
    )

    pedal = model.part("foot_pedal")
    pedal.visual(
        Cylinder(radius=0.011, length=0.335),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pedal_pivot",
    )
    pedal.visual(
        Box((0.172, 0.285, 0.024)),
        origin=Origin(xyz=(-0.098, 0.0, -0.032)),
        material=dark_rubber,
        name="pedal_plate",
    )
    for idx, x in enumerate((-0.155, -0.132, -0.109, -0.086, -0.063, -0.040)):
        pedal.visual(
            Box((0.007, 0.250, 0.006)),
            origin=Origin(xyz=(x, 0.0, -0.017)),
            material=Material(f"pedal_rib_black_{idx}", rgba=(0.015, 0.015, 0.014, 1.0)),
            name=f"pedal_rib_{idx}",
        )
    pedal.visual(
        Box((0.070, 0.270, 0.024)),
        origin=Origin(xyz=(-0.035, 0.0, -0.014)),
        material=dark_rubber,
        name="pedal_rear_neck",
    )

    bucket_handle = model.part("bucket_handle")
    handle_wire = tube_from_spline_points(
        [
            (0.000, -0.178, 0.000),
            (-0.014, -0.170, 0.004),
            (-0.028, -0.110, 0.010),
            (-0.035, 0.000, 0.014),
            (-0.028, 0.110, 0.010),
            (-0.014, 0.170, 0.004),
            (0.000, 0.178, 0.000),
        ],
        radius=0.0052,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    bucket_handle.visual(
        mesh_from_geometry(handle_wire, "inner_bucket_handle_wire"),
        material=metal,
        name="handle_wire",
    )
    for pin_name, y in (("handle_pivot_pin_0", -0.178), ("handle_pivot_pin_1", 0.178)):
        bucket_handle.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=pin_name,
        )

    model.articulation(
        "body_to_inner_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=inner_bucket,
        origin=Origin(),
    )
    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.181, 0.0, 0.804)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "front_pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(-0.205, 0.0, 0.073)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=0.36),
    )
    model.articulation(
        "bucket_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=inner_bucket,
        child=bucket_handle,
        origin=Origin(xyz=(-0.120, 0.0, 0.704)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("foot_pedal")
    inner_bucket = object_model.get_part("inner_bucket")
    handle = object_model.get_part("bucket_handle")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    pedal_pivot = object_model.get_articulation("front_pedal_pivot")
    handle_pivot = object_model.get_articulation("bucket_handle_pivot")

    for bracket in ("front_pivot_bracket_0", "front_pivot_bracket_1"):
        ctx.allow_overlap(
            body,
            pedal,
            elem_a=bracket,
            elem_b="pedal_pivot",
            reason="The foot pedal pivot shaft is intentionally captured by the lower front body bracket.",
        )
        ctx.expect_overlap(
            body,
            pedal,
            axes="yz",
            elem_a=bracket,
            elem_b="pedal_pivot",
            min_overlap=0.010,
            name=f"{bracket} captures the pedal pivot shaft",
        )

    for boss in ("handle_pivot_boss_0", "handle_pivot_boss_1"):
        pin = boss.replace("boss", "pin")
        ctx.allow_overlap(
            inner_bucket,
            handle,
            elem_a=boss,
            elem_b="handle_wire",
            reason="The small bucket handle wire end is seated in the molded side pivot boss.",
        )
        ctx.expect_overlap(
            inner_bucket,
            handle,
            axes="xz",
            elem_a=boss,
            elem_b="handle_wire",
            min_overlap=0.004,
            name=f"{boss} seats the bucket handle wire",
        )
        ctx.allow_overlap(
            inner_bucket,
            handle,
            elem_a=boss,
            elem_b=pin,
            reason="The bucket handle pivot pin is intentionally nested inside the molded bucket boss.",
        )
        ctx.expect_overlap(
            inner_bucket,
            handle,
            axes="xz",
            elem_a=boss,
            elem_b=pin,
            min_overlap=0.006,
            name=f"{pin} is captured in {boss}",
        )

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.040,
        positive_elem="lid_panel",
        negative_elem="front_wall",
        name="closed flap lid sits just above body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.250,
        elem_a="lid_panel",
        name="lid covers the rectangular bin opening",
    )
    ctx.expect_gap(
        lid,
        inner_bucket,
        axis="z",
        min_gap=0.020,
        max_gap=0.090,
        positive_elem="lid_panel",
        negative_elem="bucket_upper_rim",
        name="inner bucket rim remains below the lid line",
    )
    ctx.expect_gap(
        body,
        pedal,
        axis="x",
        min_gap=0.000,
        max_gap=0.040,
        positive_elem="front_wall",
        negative_elem="pedal_plate",
        name="pedal stays distinct in front of the lower wall",
    )

    closed_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_hinge: 1.10}):
        raised_lid_pos = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.025,
            positive_elem="front_lid_lip",
            negative_elem="front_wall",
            name="lid front edge rises when opened",
        )
    ctx.check(
        "rear hinge lifts the lid upward",
        closed_lid_pos is not None
        and raised_lid_pos is not None
        and raised_lid_pos[2] > closed_lid_pos[2] - 0.005,
        details=f"closed={closed_lid_pos}, raised={raised_lid_pos}",
    )

    rest_pedal_pos = ctx.part_world_position(pedal)
    with ctx.pose({pedal_pivot: 0.30}):
        pressed_pedal_pos = ctx.part_world_position(pedal)
        ctx.expect_gap(
            body,
            pedal,
            axis="x",
            min_gap=0.000,
            max_gap=0.055,
            positive_elem="front_wall",
            negative_elem="pedal_plate",
            name="pressed pedal remains a separate front lever",
        )
    ctx.check(
        "positive pedal motion presses the wide foot plate downward",
        rest_pedal_pos is not None
        and pressed_pedal_pos is not None
        and pressed_pedal_pos[2] < rest_pedal_pos[2] + 0.002,
        details=f"rest={rest_pedal_pos}, pressed={pressed_pedal_pos}",
    )

    folded_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_pivot: 1.25}):
        raised_handle_pos = ctx.part_world_position(handle)
        ctx.expect_gap(
            handle,
            inner_bucket,
            axis="z",
            min_gap=-0.040,
            max_gap=0.090,
            positive_elem="handle_wire",
            negative_elem="bucket_upper_rim",
            name="raised handle remains mounted at the bucket pivots",
        )
        ctx.expect_overlap(
            inner_bucket,
            handle,
            axes="xz",
            elem_a="handle_pivot_boss_0",
            elem_b="handle_pivot_pin_0",
            min_overlap=0.006,
            name="raised handle pin stays in its side boss",
        )
    ctx.check(
        "inner bucket handle rotates upward on its side pivots",
        folded_handle_pos is not None
        and raised_handle_pos is not None
        and raised_handle_pos[2] >= folded_handle_pos[2] - 0.005,
        details=f"folded={folded_handle_pos}, raised={raised_handle_pos}",
    )

    return ctx.report()


object_model = build_object_model()
