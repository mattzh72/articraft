from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _fuse(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length, both=True)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        .translate(center)
    )


def _panel_xy(points: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(points).close().extrude(thickness, both=True)


def _panel_xz(points: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().extrude(thickness, both=True)


def _hull_shape() -> cq.Workplane:
    envelope = (
        cq.Workplane("YZ")
        .workplane(offset=-21.0)
        .ellipse(0.35, 0.45)
        .workplane(offset=4.0)
        .ellipse(1.60, 1.90)
        .workplane(offset=6.0)
        .ellipse(4.10, 4.50)
        .workplane(offset=10.0)
        .ellipse(5.10, 5.30)
        .workplane(offset=11.0)
        .ellipse(4.80, 5.00)
        .workplane(offset=7.0)
        .ellipse(2.20, 2.50)
        .workplane(offset=4.0)
        .ellipse(0.55, 0.70)
        .loft(combine=True)
    )
    keel = (
        cq.Workplane("XY")
        .box(10.5, 0.95, 0.46)
        .translate((1.0, 0.0, -4.96))
    )
    return _fuse(envelope, keel)


def _gondola_shape() -> cq.Workplane:
    cabin = (
        cq.Workplane("XY")
        .box(6.8, 2.8, 1.9)
        .translate((-0.35, 0.0, -1.95))
        .edges("|Z")
        .fillet(0.28)
    )
    roof_pylon = (
        cq.Workplane("XY")
        .box(1.20, 0.80, 1.02)
        .translate((0.10, 0.0, -0.49))
        .edges("|Z")
        .fillet(0.10)
    )
    nose = (
        cq.Workplane("YZ")
        .workplane(offset=2.9)
        .center(0.0, -1.15)
        .ellipse(0.95, 0.82)
        .workplane(offset=1.5)
        .center(0.0, -1.15)
        .ellipse(0.48, 0.54)
        .loft(combine=True)
    )
    stern = (
        cq.Workplane("YZ")
        .workplane(offset=-3.7)
        .center(0.0, -1.15)
        .ellipse(0.92, 0.80)
        .workplane(offset=-1.7)
        .center(0.0, -1.15)
        .ellipse(0.42, 0.46)
        .loft(combine=True)
    )
    return _fuse(cabin, roof_pylon, nose, stern)


def _pylon_frame_shape(side_sign: float) -> cq.Workplane:
    mount_block = cq.Workplane("XY").box(0.90, 0.42, 0.82).translate((0.10, side_sign * 0.21, -0.41))
    upper_beam = cq.Workplane("XY").box(0.34, 1.58, 0.20).translate((0.30, side_sign * 0.95, -0.08))
    lower_beam = cq.Workplane("XY").box(0.28, 1.44, 0.18).translate((0.26, side_sign * 0.90, -0.58))
    connector = cq.Workplane("XY").box(0.12, 0.30, 0.54).translate((0.26, side_sign * 1.66, -0.30))
    brace = (
        cq.Workplane("XY")
        .box(0.20, 0.76, 0.18)
        .translate((0.42, side_sign * 1.36, -0.34))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -14.0 * side_sign)
    )
    return _fuse(mount_block, upper_beam, lower_beam, connector, brace)


def _pylon_boss_shape(side_sign: float) -> cq.Workplane:
    return _y_cylinder(0.20, 0.14, (0.12, side_sign * 1.95, -0.30))


def _pod_body_shape(side_sign: float) -> cq.Workplane:
    nacelle = (
        cq.Workplane("YZ")
        .workplane(offset=-1.30)
        .center(side_sign * 0.52, -0.36)
        .ellipse(0.16, 0.18)
        .workplane(offset=0.70)
        .center(side_sign * 0.52, -0.36)
        .ellipse(0.28, 0.36)
        .workplane(offset=1.55)
        .center(side_sign * 0.52, -0.36)
        .ellipse(0.24, 0.32)
        .workplane(offset=0.95)
        .center(side_sign * 0.52, -0.36)
        .ellipse(0.18, 0.20)
        .loft(combine=True)
    )
    return _fuse(nacelle, _pod_stalk_shape(side_sign), _drive_shaft_shape(side_sign))


def _pod_stalk_shape(side_sign: float) -> cq.Workplane:
    return cq.Workplane("XY").box(0.28, 0.70, 0.40).translate((0.02, side_sign * 0.42, -0.28))


def _drive_shaft_shape(side_sign: float) -> cq.Workplane:
    return _x_cylinder(0.08, 0.70, (2.25, side_sign * 0.52, -0.36))


def _mount_sleeve_shape(side_sign: float) -> cq.Workplane:
    return _y_cylinder(0.18, 0.14, (0.0, -side_sign * 0.07, 0.0))


def _propeller_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(0.13).extrude(0.24, both=True)
    spinner = (
        cq.Workplane("YZ")
        .workplane(offset=0.02)
        .circle(0.12)
        .workplane(offset=0.18)
        .circle(0.08)
        .workplane(offset=0.18)
        .circle(0.02)
        .loft(combine=True)
    )
    blade = (
        cq.Workplane("XY")
        .box(0.24, 0.07, 1.72)
        .translate((0.02, 0.0, 0.86))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 10.0)
    )
    blades = blade
    for angle in (120.0, 240.0):
        blades = blades.union(blade.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle))
    return _fuse(hub, spinner, blades)


def _tail_frame_shape() -> cq.Workplane:
    tail_saddle = cq.Workplane("XY").box(0.55, 1.20, 1.10).translate((-0.28, 0.0, 0.0))
    dorsal_root = _panel_xz([(0.0, 0.0), (-2.20, 0.0), (-2.20, 4.10), (-0.40, 4.10)], 0.34)
    ventral_fin = _panel_xz([(0.0, 0.0), (-1.85, 0.0), (-1.20, -2.90), (-0.22, -2.90)], 0.28)
    right_root = _panel_xy([(0.0, 0.0), (-1.80, 0.0), (-1.80, 1.80), (-0.48, 1.80)], 0.28)
    left_root = _panel_xy([(0.0, 0.0), (-1.80, 0.0), (-1.80, -1.80), (-0.48, -1.80)], 0.28)
    return _fuse(tail_saddle, dorsal_root, ventral_fin, right_root, left_root)


def _rudder_shape() -> cq.Workplane:
    return _panel_xz([(0.0, 0.0), (-1.85, 0.0), (-1.15, 4.35), (-0.05, 4.35)], 0.24)


def _elevator_shape(side_sign: float) -> cq.Workplane:
    return _panel_xy(
        [(0.0, 0.0), (-1.70, 0.0), (-1.22, side_sign * 2.10), (-0.05, side_sign * 2.10)],
        0.24,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_service_blimp")

    hull_paint = model.material("hull_paint", rgba=(0.93, 0.52, 0.18, 1.0))
    gondola_paint = model.material("gondola_paint", rgba=(0.92, 0.94, 0.96, 1.0))
    structure_paint = model.material("structure_paint", rgba=(0.24, 0.26, 0.30, 1.0))
    pod_paint = model.material("pod_paint", rgba=(0.84, 0.36, 0.17, 1.0))
    prop_paint = model.material("prop_paint", rgba=(0.10, 0.10, 0.12, 1.0))

    hull = model.part("hull")
    hull.visual(
        mesh_from_cadquery(_hull_shape(), "hull"),
        material=hull_paint,
        name="envelope",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_cadquery(_gondola_shape(), "gondola"),
        material=gondola_paint,
        name="cabin",
    )

    left_pylon = model.part("left_pylon")
    left_pylon.visual(
        mesh_from_cadquery(_pylon_frame_shape(1.0), "left_pylon_frame"),
        material=structure_paint,
        name="pylon_frame",
    )
    left_pylon.visual(
        mesh_from_cadquery(_pylon_boss_shape(1.0), "left_pylon_boss"),
        material=structure_paint,
        name="pivot_boss",
    )

    right_pylon = model.part("right_pylon")
    right_pylon.visual(
        mesh_from_cadquery(_pylon_frame_shape(-1.0), "right_pylon_frame"),
        material=structure_paint,
        name="pylon_frame",
    )
    right_pylon.visual(
        mesh_from_cadquery(_pylon_boss_shape(-1.0), "right_pylon_boss"),
        material=structure_paint,
        name="pivot_boss",
    )

    left_pod = model.part("left_pod")
    left_pod.visual(
        mesh_from_cadquery(_pod_body_shape(1.0), "left_pod_body"),
        material=pod_paint,
        name="pod_body",
    )
    left_pod.visual(
        mesh_from_cadquery(_mount_sleeve_shape(1.0), "left_pod_sleeve"),
        material=structure_paint,
        name="mount_sleeve",
    )

    right_pod = model.part("right_pod")
    right_pod.visual(
        mesh_from_cadquery(_pod_body_shape(-1.0), "right_pod_body"),
        material=pod_paint,
        name="pod_body",
    )
    right_pod.visual(
        mesh_from_cadquery(_mount_sleeve_shape(-1.0), "right_pod_sleeve"),
        material=structure_paint,
        name="mount_sleeve",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        mesh_from_cadquery(_propeller_shape(), "left_propeller"),
        material=prop_paint,
        name="propeller",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        mesh_from_cadquery(_propeller_shape(), "right_propeller"),
        material=prop_paint,
        name="propeller",
    )

    tail_frame = model.part("tail_frame")
    tail_frame.visual(
        mesh_from_cadquery(_tail_frame_shape(), "tail_frame"),
        material=hull_paint,
        name="tail_surface",
    )

    rudder = model.part("rudder")
    rudder.visual(
        mesh_from_cadquery(_rudder_shape(), "rudder"),
        material=hull_paint,
        name="rudder_panel",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        mesh_from_cadquery(_elevator_shape(-1.0), "left_elevator"),
        material=hull_paint,
        name="elevator_panel",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        mesh_from_cadquery(_elevator_shape(1.0), "right_elevator"),
        material=hull_paint,
        name="elevator_panel",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(1.0, 0.0, -5.349)),
    )
    model.articulation(
        "gondola_to_left_pylon",
        ArticulationType.FIXED,
        parent=gondola,
        child=left_pylon,
        origin=Origin(xyz=(0.55, 1.40, -0.72)),
    )
    model.articulation(
        "gondola_to_right_pylon",
        ArticulationType.FIXED,
        parent=gondola,
        child=right_pylon,
        origin=Origin(xyz=(0.55, -1.40, -0.72)),
    )
    model.articulation(
        "left_pylon_to_left_pod",
        ArticulationType.REVOLUTE,
        parent=left_pylon,
        child=left_pod,
        origin=Origin(xyz=(0.12, 2.02, -0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "right_pylon_to_right_pod",
        ArticulationType.REVOLUTE,
        parent=right_pylon,
        child=right_pod,
        origin=Origin(xyz=(0.12, -2.02, -0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "left_pod_to_left_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_pod,
        child=left_propeller,
        origin=Origin(xyz=(2.84, 0.52, -0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=30.0),
    )
    model.articulation(
        "right_pod_to_right_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_pod,
        child=right_propeller,
        origin=Origin(xyz=(2.84, -0.52, -0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=30.0),
    )
    model.articulation(
        "hull_to_tail_frame",
        ArticulationType.FIXED,
        parent=hull,
        child=tail_frame,
        origin=Origin(xyz=(-21.0, 0.0, 0.0)),
    )
    model.articulation(
        "tail_frame_to_rudder",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=rudder,
        origin=Origin(xyz=(-2.20, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.1, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "tail_frame_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=left_elevator,
        origin=Origin(xyz=(-1.80, -1.80, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.35, upper=0.45),
    )
    model.articulation(
        "tail_frame_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=tail_frame,
        child=right_elevator,
        origin=Origin(xyz=(-1.80, 1.80, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.35, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    left_pylon = object_model.get_part("left_pylon")
    right_pylon = object_model.get_part("right_pylon")
    left_pod = object_model.get_part("left_pod")
    right_pod = object_model.get_part("right_pod")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_pod_joint = object_model.get_articulation("left_pylon_to_left_pod")
    right_pod_joint = object_model.get_articulation("right_pylon_to_right_pod")
    rudder_joint = object_model.get_articulation("tail_frame_to_rudder")
    left_elevator_joint = object_model.get_articulation("tail_frame_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("tail_frame_to_right_elevator")

    hull_aabb = ctx.part_world_aabb(hull)
    ctx.check("hull_aabb_present", hull_aabb is not None, "Expected a world AABB for the hull.")
    if hull_aabb is not None:
        mins, maxs = hull_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("emergency_airship_scale", 40.0 <= size[0] <= 43.5 and 9.0 <= size[2] <= 11.5, f"size={size!r}")

    hull_pos = ctx.part_world_position(hull)
    gondola_pos = ctx.part_world_position(gondola)
    ctx.check(
        "gondola_hangs_below_hull",
        hull_pos is not None and gondola_pos is not None and gondola_pos[2] < hull_pos[2] - 4.5,
        details=f"hull={hull_pos}, gondola={gondola_pos}",
    )

    ctx.allow_overlap(
        left_pylon,
        left_pod,
        elem_a="pivot_boss",
        elem_b="mount_sleeve",
        reason="The pod pivots on a short boss-and-sleeve mount at the pylon tip.",
    )
    ctx.allow_overlap(
        hull,
        gondola,
        elem_a="envelope",
        elem_b="cabin",
        reason="The gondola roof saddle is faired into the hull envelope at the suspension mount.",
    )
    ctx.allow_overlap(
        right_pylon,
        right_pod,
        elem_a="pivot_boss",
        elem_b="mount_sleeve",
        reason="The pod pivots on a short boss-and-sleeve mount at the pylon tip.",
    )
    ctx.allow_overlap(
        left_pod,
        left_propeller,
        reason="The propeller hub is intentionally seated onto the pod drive shaft.",
    )
    ctx.allow_overlap(
        right_pod,
        right_propeller,
        reason="The propeller hub is intentionally seated onto the pod drive shaft.",
    )

    ctx.expect_within(
        left_pod,
        left_pylon,
        axes="xz",
        inner_elem="mount_sleeve",
        outer_elem="pivot_boss",
        margin=0.03,
        name="left pod sleeve stays captured on the pivot boss",
    )
    ctx.expect_overlap(
        left_pod,
        left_pylon,
        axes="y",
        elem_a="mount_sleeve",
        elem_b="pivot_boss",
        min_overlap=0.10,
        name="left pod sleeve remains engaged through the pivot depth",
    )
    ctx.expect_within(
        right_pod,
        right_pylon,
        axes="xz",
        inner_elem="mount_sleeve",
        outer_elem="pivot_boss",
        margin=0.03,
        name="right pod sleeve stays captured on the pivot boss",
    )
    ctx.expect_overlap(
        right_pod,
        right_pylon,
        axes="y",
        elem_a="mount_sleeve",
        elem_b="pivot_boss",
        min_overlap=0.10,
        name="right pod sleeve remains engaged through the pivot depth",
    )

    left_pod_upper = left_pod_joint.motion_limits.upper if left_pod_joint.motion_limits is not None else None
    right_pod_upper = right_pod_joint.motion_limits.upper if right_pod_joint.motion_limits is not None else None
    rest_prop_pos = ctx.part_world_position(left_propeller)
    if left_pod_upper is not None:
        with ctx.pose({left_pod_joint: left_pod_upper}):
            raised_prop_pos = ctx.part_world_position(left_propeller)
        ctx.check(
            "left pod vectors thrust upward",
            rest_prop_pos is not None and raised_prop_pos is not None and raised_prop_pos[2] > rest_prop_pos[2] + 0.20,
            details=f"rest={rest_prop_pos}, raised={raised_prop_pos}",
        )
    if right_pod_upper is not None:
        right_rest = ctx.part_world_position(right_propeller)
        with ctx.pose({right_pod_joint: right_pod_upper}):
            right_raised = ctx.part_world_position(right_propeller)
        ctx.check(
            "right pod rotates on its pylon pivot",
            right_rest is not None and right_raised is not None and right_raised[2] > right_rest[2] + 0.12,
            details=f"rest={right_rest}, raised={right_raised}",
        )

    rudder_upper = rudder_joint.motion_limits.upper if rudder_joint.motion_limits is not None else None
    rudder_rest = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
    if rudder_upper is not None:
        with ctx.pose({rudder_joint: rudder_upper}):
            rudder_turned = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
        ctx.check(
            "rudder_deflects_sideways",
            rudder_rest is not None and rudder_turned is not None and rudder_turned[1][1] > rudder_rest[1][1] + 0.12,
            details=f"rest={rudder_rest}, turned={rudder_turned}",
        )

    left_elevator_upper = left_elevator_joint.motion_limits.upper if left_elevator_joint.motion_limits is not None else None
    left_elevator_rest = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
    if left_elevator_upper is not None:
        with ctx.pose({left_elevator_joint: left_elevator_upper}):
            left_elevator_up = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
        ctx.check(
            "left_elevator_trailing_edge_rises",
            left_elevator_rest is not None
            and left_elevator_up is not None
            and left_elevator_up[1][2] > left_elevator_rest[1][2] + 0.10,
            details=f"rest={left_elevator_rest}, raised={left_elevator_up}",
        )

    right_elevator_upper = right_elevator_joint.motion_limits.upper if right_elevator_joint.motion_limits is not None else None
    right_elevator_rest = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
    if right_elevator_upper is not None:
        with ctx.pose({right_elevator_joint: right_elevator_upper}):
            right_elevator_up = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
        ctx.check(
            "right_elevator_trailing_edge_rises",
            right_elevator_rest is not None
            and right_elevator_up is not None
            and right_elevator_up[1][2] > right_elevator_rest[1][2] + 0.10,
            details=f"rest={right_elevator_rest}, raised={right_elevator_up}",
        )

    return ctx.report()


object_model = build_object_model()
