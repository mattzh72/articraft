from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.102
BODY_WIDTH = 0.048
BODY_HEIGHT = 0.050
BODY_TOP = BODY_HEIGHT * 0.5
BODY_SIDE = BODY_WIDTH * 0.5


def _tube_x(*, outer_radius: float, inner_radius: float, length: float, start_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((start_x, 0.0, 0.0))
    )


def _centered_tube_x(*, outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length * 0.5, 0.0, 0.0))
    )


def _body_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
    shell = shell.edges("|Z").fillet(0.009)

    top_hump = (
        cq.Workplane("XY")
        .box(0.070, 0.040, 0.016)
        .translate((-0.008, 0.0, 0.017))
        .edges("|Z")
        .fillet(0.004)
    )

    grip_bulge = (
        cq.Workplane("XY")
        .center(0.018, -0.027)
        .circle(0.017)
        .extrude(0.048)
        .translate((0.0, 0.0, -0.024))
    )

    return shell.union(top_hump).union(grip_bulge)


def _lens_barrel_shape() -> cq.Workplane:
    rear_collar = _tube_x(outer_radius=0.0195, inner_radius=0.0140, length=0.012, start_x=0.040)
    front_barrel = _tube_x(outer_radius=0.0165, inner_radius=0.0115, length=0.028, start_x=0.047)
    return rear_collar.union(front_barrel)


def _lens_ring_shape() -> cq.Workplane:
    ring = _centered_tube_x(outer_radius=0.0198, inner_radius=0.0140, length=0.004)
    grip_tab = cq.Workplane("XY").box(0.004, 0.003, 0.002).translate((0.0, 0.0, 0.0208))
    return ring.union(grip_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_camcorder")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    monitor_glass = model.material("monitor_glass", rgba=(0.19, 0.27, 0.31, 0.55))
    lens_finish = model.material("lens_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "camcorder_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_lens_barrel_shape(), "camcorder_lens_barrel"),
        material=lens_finish,
        name="lens_barrel",
    )
    body.visual(
        Box((0.050, 0.004, 0.030)),
        origin=Origin(xyz=(0.012, -0.039, -0.002)),
        material=rubber_finish,
        name="grip_pad",
    )
    body.visual(
        Box((0.030, 0.020, 0.004)),
        origin=Origin(xyz=(-0.022, 0.000, BODY_TOP + 0.002)),
        material=trim_finish,
        name="top_deck",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_cadquery(_lens_ring_shape(), "camcorder_lens_ring"),
        material=lens_finish,
        name="lens_ring_shell",
    )
    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=12.0),
    )

    monitor_hinge = model.part("monitor_hinge")
    monitor_hinge.visual(
        Box((0.010, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=trim_finish,
        name="hinge_block",
    )
    monitor_hinge.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=trim_finish,
        name="upper_barrel",
    )
    monitor_hinge.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=trim_finish,
        name="lower_barrel",
    )
    model.articulation(
        "body_to_monitor_hinge",
        ArticulationType.FIXED,
        parent=body,
        child=monitor_hinge,
        origin=Origin(xyz=(-0.034, 0.032, 0.002)),
    )

    monitor = model.part("monitor")
    monitor.visual(
        Cylinder(radius=0.0030, length=0.012),
        material=trim_finish,
        name="hinge_sleeve",
    )
    monitor.visual(
        Box((0.012, 0.006, 0.020)),
        origin=Origin(xyz=(0.006, 0.003, 0.0)),
        material=trim_finish,
        name="hinge_arm",
    )
    monitor.visual(
        Box((0.058, 0.008, 0.040)),
        origin=Origin(xyz=(0.033, 0.008, 0.0)),
        material=body_finish,
        name="monitor_shell",
    )
    monitor.visual(
        Box((0.050, 0.003, 0.032)),
        origin=Origin(xyz=(0.033, 0.0065, 0.0)),
        material=trim_finish,
        name="monitor_bezel",
    )
    monitor.visual(
        Box((0.046, 0.0012, 0.028)),
        origin=Origin(xyz=(0.033, 0.0046, 0.0)),
        material=monitor_glass,
        name="screen",
    )
    model.articulation(
        "monitor_hinge_to_monitor",
        ArticulationType.REVOLUTE,
        parent=monitor_hinge,
        child=monitor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=2.5),
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.020,
                0.008,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=14, depth=0.0006),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0004, angle_deg=0.0),
                center=False,
            ),
            "camcorder_mode_dial",
        ),
        material=dial_finish,
        name="dial_cap",
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.016, -0.010, BODY_TOP + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    monitor = object_model.get_part("monitor")
    monitor_hinge = object_model.get_part("monitor_hinge")
    mode_dial = object_model.get_part("mode_dial")

    monitor_joint = object_model.get_articulation("monitor_hinge_to_monitor")
    lens_joint = object_model.get_articulation("body_to_lens_ring")
    dial_joint = object_model.get_articulation("body_to_mode_dial")

    ctx.check(
        "lens_ring_joint_axis",
        lens_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(lens_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={lens_joint.articulation_type}, axis={lens_joint.axis}",
    )
    ctx.check(
        "mode_dial_joint_axis",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "monitor_hinge_range",
        monitor_joint.motion_limits is not None
        and monitor_joint.motion_limits.lower == 0.0
        and monitor_joint.motion_limits.upper is not None
        and monitor_joint.motion_limits.upper >= 2.2,
        details=f"limits={monitor_joint.motion_limits!r}",
    )

    ctx.expect_gap(
        monitor,
        body,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem="monitor_shell",
        name="closed_monitor_sits_alongside_body",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="xz",
        min_overlap=0.020,
        name="closed_monitor_covers_rear_side_area",
    )
    ctx.expect_contact(
        monitor,
        monitor_hinge,
        elem_a="hinge_sleeve",
        elem_b="upper_barrel",
        contact_tol=1e-6,
        name="monitor_sleeve_is_supported_by_upper_barrel",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        max_gap=0.010,
        max_penetration=0.0001,
        name="mode_dial_sits_on_top_shell",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="yz",
        min_overlap=0.030,
        elem_a="lens_ring_shell",
        elem_b="lens_barrel",
        name="lens_ring_stays_coaxial_with_barrel",
    )

    def _elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[index]) + float(maxs[index])) * 0.5 for index in range(3))

    closed_monitor_pos = _elem_center("monitor", "monitor_shell")
    with ctx.pose({monitor_joint: 1.5}):
        open_monitor_pos = _elem_center("monitor", "monitor_shell")
        ctx.expect_gap(
            monitor,
            body,
            axis="y",
            min_gap=0.010,
            positive_elem="monitor_shell",
            name="open_monitor_swings_outboard",
        )

    ctx.check(
        "monitor_moves_outward",
        closed_monitor_pos is not None
        and open_monitor_pos is not None
        and open_monitor_pos[1] > closed_monitor_pos[1] + 0.020,
        details=f"closed={closed_monitor_pos}, open={open_monitor_pos}",
    )

    return ctx.report()


object_model = build_object_model()
