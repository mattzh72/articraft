from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SLATE_WIDTH = 0.282
SLATE_HEIGHT = 0.198
SLATE_THICKNESS = 0.008
HINGE_RADIUS = 0.0045


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((a + b) / 2.0 for a, b in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="script_supervisor_slate")

    frame_black = model.material("frame_black", rgba=(0.09, 0.09, 0.10, 1.0))
    board_white = model.material("board_white", rgba=(0.95, 0.96, 0.94, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.98, 0.98, 0.97, 1.0))
    loop_black = model.material("loop_black", rgba=(0.12, 0.12, 0.13, 1.0))

    slate = model.part("slate")
    slate.visual(
        Box((SLATE_WIDTH, SLATE_THICKNESS, SLATE_HEIGHT)),
        origin=Origin(),
        material=frame_black,
        name="slate_body",
    )
    slate.visual(
        Box((0.248, 0.0016, 0.158)),
        origin=Origin(xyz=(0.0, 0.0039, -0.010)),
        material=board_white,
        name="writing_surface",
    )
    slate.visual(
        Box((SLATE_WIDTH, 0.010, 0.015)),
        origin=Origin(xyz=(0.0, 0.0010, 0.0915)),
        material=frame_black,
        name="top_cap",
    )
    for suffix, x_pos in enumerate((-0.103, 0.103)):
        slate.visual(
            Box((0.076, 0.0012, 0.012)),
            origin=Origin(xyz=(x_pos, 0.0046, 0.0925)),
            material=hinge_metal,
            name=f"hinge_leaf_{suffix}",
        )
        slate.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.076),
            origin=Origin(xyz=(x_pos, 0.0062, 0.101), rpy=(0.0, 1.5707963267948966, 0.0)),
            material=hinge_metal,
            name=f"hinge_knuckle_{suffix}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((0.302, 0.009, 0.028)),
        origin=Origin(xyz=(0.0, 0.010, -0.014)),
        material=frame_black,
        name="clapstick_bar",
    )
    clapstick.visual(
        Box((0.130, 0.0012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0046, -0.006)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    clapstick.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.130),
        origin=Origin(xyz=(0.0, 0.0062, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=hinge_metal,
        name="hinge_knuckle",
    )
    for idx, x_pos in enumerate((-0.102, -0.036, 0.030, 0.096)):
        clapstick.visual(
            Box((0.082, 0.0010, 0.0055)),
            origin=Origin(xyz=(x_pos, 0.0140, -0.014), rpy=(0.0, -0.68, 0.0)),
            material=stripe_white,
            name=f"stripe_{idx}",
        )

    marker_loop = model.part("marker_loop")
    marker_loop.visual(
        Box((0.006, 0.012, 0.048)),
        origin=Origin(),
        material=loop_black,
        name="mount_base",
    )
    marker_loop.visual(
        Cylinder(radius=0.0025, length=0.022),
        origin=Origin(xyz=(0.010, 0.0, 0.0175), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=loop_black,
        name="loop_top",
    )
    marker_loop.visual(
        Cylinder(radius=0.0025, length=0.022),
        origin=Origin(xyz=(0.010, 0.0, -0.0175), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=loop_black,
        name="loop_bottom",
    )
    marker_loop.visual(
        Cylinder(radius=0.0025, length=0.036),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=loop_black,
        name="loop_outer",
    )

    model.articulation(
        "slate_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.0062, 0.101)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=1.1),
    )
    model.articulation(
        "slate_to_marker_loop",
        ArticulationType.FIXED,
        parent=slate,
        child=marker_loop,
        origin=Origin(xyz=(SLATE_WIDTH / 2.0 + 0.003, 0.0, 0.055)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    marker_loop = object_model.get_part("marker_loop")
    hinge = object_model.get_articulation("slate_to_clapstick")

    ctx.expect_contact(
        marker_loop,
        slate,
        elem_a="mount_base",
        elem_b="slate_body",
        name="marker loop fitting mounts to the slate edge",
    )
    ctx.expect_overlap(
        clapstick,
        slate,
        axes="x",
        elem_a="clapstick_bar",
        elem_b="writing_surface",
        min_overlap=0.24,
        name="clapstick spans the writing slate width",
    )
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            clapstick,
            slate,
            axis="y",
            positive_elem="clapstick_bar",
            negative_elem="writing_surface",
            min_gap=0.0004,
            max_gap=0.010,
            name="closed clapstick sits just proud of the dry erase face",
        )

    loop_pos = ctx.part_world_position(marker_loop)
    ctx.check(
        "marker loop sits near the upper side corner",
        loop_pos is not None and loop_pos[0] > 0.14 and loop_pos[2] > 0.04,
        details=f"marker loop origin={loop_pos}",
    )

    limits = hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            closed_center = _center_from_aabb(ctx.part_element_world_aabb(clapstick, elem="clapstick_bar"))
        with ctx.pose({hinge: limits.upper}):
            open_center = _center_from_aabb(ctx.part_element_world_aabb(clapstick, elem="clapstick_bar"))
        ctx.check(
            "clapstick opens upward and forward",
            closed_center is not None
            and open_center is not None
            and open_center[1] > closed_center[1] + 0.006
            and open_center[2] > closed_center[2] + 0.010,
            details=f"closed_center={closed_center}, open_center={open_center}",
        )

    return ctx.report()


object_model = build_object_model()
