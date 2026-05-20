from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 48,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_swing_pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.24, 0.28, 0.33, 1.0))
    inner_paint = model.material("inner_paint", rgba=(0.20, 0.22, 0.25, 1.0))
    trim_black = model.material("trim_black", rgba=(0.09, 0.09, 0.10, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    latch_black = model.material("latch_black", rgba=(0.08, 0.08, 0.08, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.inertial = Inertial.from_geometry(
        Box((1.66, 0.56, 0.68)),
        mass=95.0,
        origin=Origin(xyz=(0.0, -0.16, 0.34)),
    )
    bed_frame.visual(
        Box((1.66, 0.48, 0.06)),
        origin=Origin(xyz=(0.0, -0.24, 0.03)),
        material=inner_paint,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((0.08, 0.56, 0.56)),
        origin=Origin(xyz=(-0.79, -0.28, 0.34)),
        material=body_paint,
        name="left_bed_side",
    )
    bed_frame.visual(
        Box((0.08, 0.56, 0.56)),
        origin=Origin(xyz=(0.79, -0.28, 0.34)),
        material=body_paint,
        name="right_bed_side",
    )
    bed_frame.visual(
        Box((0.12, 0.56, 0.06)),
        origin=Origin(xyz=(-0.77, -0.28, 0.65)),
        material=body_paint,
        name="left_top_rail",
    )
    bed_frame.visual(
        Box((0.12, 0.56, 0.06)),
        origin=Origin(xyz=(0.77, -0.28, 0.65)),
        material=body_paint,
        name="right_top_rail",
    )
    bed_frame.visual(
        Box((0.05, 0.04, 0.52)),
        origin=Origin(xyz=(-0.751, -0.02, 0.34)),
        material=inner_paint,
        name="left_hinge_reinforcement",
    )
    bed_frame.visual(
        Box((0.08, 0.05, 0.52)),
        origin=Origin(xyz=(0.735, 0.005, 0.34)),
        material=inner_paint,
        name="receiver_post",
    )
    for name, z_pos in (
        ("hinge_body_lower", 0.12),
        ("hinge_body_middle", 0.34),
        ("hinge_body_upper", 0.56),
    ):
        bed_frame.visual(
            Box((0.044, 0.036, 0.06)),
            origin=Origin(xyz=(-0.724, 0.012, z_pos)),
            material=hinge_steel,
            name=f"{name}_ear",
        )
        bed_frame.visual(
            Cylinder(radius=0.018, length=0.08),
            origin=Origin(xyz=(-0.705, 0.03, z_pos)),
            material=hinge_steel,
            name=name,
        )

    tailgate = model.part("tailgate")
    tailgate.inertial = Inertial.from_geometry(
        Box((1.40, 0.06, 0.54)),
        mass=28.0,
        origin=Origin(xyz=(0.70, 0.0, 0.0)),
    )

    gate_shell = (
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(1.34, 0.54, 0.035, corner_segments=8),
            [_circle_profile(0.28, cx=0.04, cy=0.02, segments=56)],
            height=0.028,
            center=True,
        )
        .rotate_x(pi / 2.0)
    )
    tailgate.visual(
        _save_mesh("tailgate_shell", gate_shell),
        origin=Origin(xyz=(0.72, -0.025, 0.0)),
        material=body_paint,
        name="gate_shell",
    )
    tailgate.visual(
        Box((0.04, 0.045, 0.54)),
        origin=Origin(xyz=(0.04, -0.003, 0.0)),
        material=inner_paint,
        name="hinge_edge_beam",
    )
    tailgate.visual(
        Box((0.05, 0.045, 0.54)),
        origin=Origin(xyz=(1.375, -0.003, 0.0)),
        material=inner_paint,
        name="meeting_edge_beam",
    )
    tailgate.visual(
        Box((1.34, 0.04, 0.05)),
        origin=Origin(xyz=(0.73, -0.004, 0.245)),
        material=inner_paint,
        name="top_rail_beam",
    )
    tailgate.visual(
        Box((1.34, 0.05, 0.07)),
        origin=Origin(xyz=(0.73, -0.005, -0.225)),
        material=inner_paint,
        name="bottom_rail_beam",
    )
    tailgate.visual(
        Box((0.72, 0.04, 0.05)),
        origin=Origin(xyz=(0.73, -0.004, -0.175)),
        material=inner_paint,
        name="lower_brace",
    )
    tailgate.visual(
        Box((0.12, 0.022, 0.18)),
        origin=Origin(xyz=(1.29, 0.012, 0.03)),
        material=trim_black,
        name="latch_mount",
    )
    tailgate.visual(
        Cylinder(radius=0.0145, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=hinge_steel,
        name="hinge_leaf_lower",
    )
    tailgate.visual(
        Cylinder(radius=0.0145, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=hinge_steel,
        name="hinge_leaf_upper",
    )
    tailgate.visual(
        Box((0.058, 0.016, 0.10)),
        origin=Origin(xyz=(0.032, -0.022, -0.12)),
        material=hinge_steel,
        name="hinge_strap_lower",
    )
    tailgate.visual(
        Box((0.058, 0.016, 0.10)),
        origin=Origin(xyz=(0.032, -0.022, 0.12)),
        material=hinge_steel,
        name="hinge_strap_upper",
    )

    latch_lever = model.part("latch_lever")
    latch_lever.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.18)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.01, -0.07)),
    )
    latch_lever.visual(
        Cylinder(radius=0.011, length=0.06),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="lever_pivot",
    )
    latch_lever.visual(
        Box((0.06, 0.016, 0.16)),
        origin=Origin(xyz=(0.0, 0.010, -0.08)),
        material=latch_black,
        name="handle_grip",
    )
    latch_lever.visual(
        Box((0.03, 0.014, 0.03)),
        origin=Origin(xyz=(0.018, 0.010, 0.015)),
        material=latch_black,
        name="latch_cam",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(-0.705, 0.03, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "tailgate_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_lever,
        origin=Origin(xyz=(1.29, 0.034, 0.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    latch_lever = object_model.get_part("latch_lever")
    gate_swing = object_model.get_articulation("bed_to_tailgate")
    latch_pivot = object_model.get_articulation("tailgate_to_latch_lever")

    receiver_post = bed_frame.get_visual("receiver_post")
    meeting_edge = tailgate.get_visual("meeting_edge_beam")
    latch_mount = tailgate.get_visual("latch_mount")
    handle_grip = latch_lever.get_visual("handle_grip")

    ctx.expect_gap(
        bed_frame,
        tailgate,
        axis="x",
        positive_elem=receiver_post,
        negative_elem=meeting_edge,
        max_gap=0.008,
        max_penetration=0.0001,
        name="tailgate meeting edge closes against the receiver post",
    )
    ctx.expect_overlap(
        tailgate,
        bed_frame,
        axes="z",
        elem_a=meeting_edge,
        elem_b=receiver_post,
        min_overlap=0.40,
        name="tailgate meeting edge lines up with the receiver post height",
    )
    ctx.expect_gap(
        latch_lever,
        tailgate,
        axis="y",
        positive_elem=handle_grip,
        negative_elem=latch_mount,
        min_gap=0.004,
        max_gap=0.04,
        name="latch handle sits proud of the tailgate mount plate",
    )
    ctx.expect_overlap(
        latch_lever,
        tailgate,
        axes="xz",
        elem_a=handle_grip,
        elem_b=latch_mount,
        min_overlap=0.05,
        name="latch handle stays centered over the latch mount",
    )

    closed_edge_center = _aabb_center(ctx.part_element_world_aabb(tailgate, elem="meeting_edge_beam"))
    with ctx.pose({gate_swing: gate_swing.motion_limits.upper}):
        opened_edge_center = _aabb_center(ctx.part_element_world_aabb(tailgate, elem="meeting_edge_beam"))
    ctx.check(
        "tailgate swings outward from the left rear corner",
        closed_edge_center is not None
        and opened_edge_center is not None
        and opened_edge_center[1] > closed_edge_center[1] + 1.0
        and opened_edge_center[0] < closed_edge_center[0] - 0.6,
        details=f"closed_edge_center={closed_edge_center}, opened_edge_center={opened_edge_center}",
    )

    closed_handle_center = _aabb_center(ctx.part_element_world_aabb(latch_lever, elem="handle_grip"))
    with ctx.pose({latch_pivot: latch_pivot.motion_limits.upper}):
        opened_handle_center = _aabb_center(ctx.part_element_world_aabb(latch_lever, elem="handle_grip"))
    ctx.check(
        "latch lever rotates outward off the tailgate skin",
        closed_handle_center is not None
        and opened_handle_center is not None
        and opened_handle_center[1] > closed_handle_center[1] + 0.035,
        details=f"closed_handle_center={closed_handle_center}, opened_handle_center={opened_handle_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
