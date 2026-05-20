from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


CASE_LENGTH = 0.86
CASE_WIDTH = 0.32
LOWER_HEIGHT = 0.060
LID_HEIGHT = 0.052
WALL = 0.020
BOTTOM_THICKNESS = 0.012
HINGE_Y = CASE_WIDTH / 2.0
FRONT_Y = -CASE_WIDTH / 2.0
HINGE_Z = LOWER_HEIGHT + 0.001
LID_BOTTOM_GAP = 0.0015
LATCH_XS = (-0.250, 0.250)


def _ring_mesh(length: float, width: float, inner_length: float, inner_width: float, height: float) -> MeshGeometry:
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(length, width, 0.050, corner_segments=8),
        [rounded_rect_profile(inner_length, inner_width, 0.032, corner_segments=8)],
        height,
        center=True,
    )


def _rounded_tray(*, length: float, width: float, height: float, wall: float, bottom: float) -> MeshGeometry:
    """Open-top rounded rectangular hard-shell tray."""
    outer = rounded_rect_profile(length, width, 0.050, corner_segments=10)
    inner = rounded_rect_profile(length - 2.0 * wall, width - 2.0 * wall, 0.032, corner_segments=10)
    bottom_slab = ExtrudeGeometry.from_z0(outer, bottom)
    side_walls = ExtrudeWithHolesGeometry(outer, [inner], height - bottom, center=True).translate(
        0.0,
        0.0,
        bottom + (height - bottom) / 2.0,
    )
    return bottom_slab.merge(side_walls)


def _open_bottom_lid(*, length: float, width: float, height: float, wall: float, top: float) -> MeshGeometry:
    """Open-bottom shell so the lid reads as a hollow clamshell half."""
    outer = rounded_rect_profile(length, width, 0.050, corner_segments=10)
    inner = rounded_rect_profile(length - 2.0 * wall, width - 2.0 * wall, 0.032, corner_segments=10)
    side_walls = ExtrudeWithHolesGeometry(outer, [inner], height, center=True).translate(0.0, 0.0, height / 2.0)
    top_slab = ExtrudeGeometry.from_z0(outer, top).translate(0.0, 0.0, height - top)
    return side_walls.merge(top_slab)


def _rounded_ring(length: float, width: float, inner_length: float, inner_width: float, height: float) -> MeshGeometry:
    return _ring_mesh(length, width, inner_length, inner_width, height)


def _violin_cavity_profile(samples: int = 72) -> list[tuple[float, float]]:
    """A plush violin-shaped cavity footprint: neck, waist, upper/lower bouts."""

    def half_width(x: float) -> float:
        # Narrow scroll/neck at the left, swelling into upper/lower bouts to the right.
        if x < -0.13:
            return 0.016 + 0.008 * math.exp(-((x + 0.27) / 0.050) ** 2)
        upper = 0.061 * math.exp(-((x + 0.055) / 0.085) ** 2)
        waist = -0.020 * math.exp(-((x - 0.095) / 0.045) ** 2)
        lower = 0.086 * math.exp(-((x - 0.220) / 0.115) ** 2)
        end_taper = 1.0 - 0.36 * max(0.0, (x - 0.270) / 0.070)
        return max(0.030, (0.036 + upper + waist + lower) * end_taper)

    xs = [-0.345 + i * (0.690 / (samples - 1)) for i in range(samples)]
    top = [(x, half_width(x)) for x in xs]
    bottom = [(x, -half_width(x)) for x in reversed(xs)]
    return top + bottom


def _latch_part(latch, metal: Material) -> None:
    latch.visual(
        Cylinder(radius=0.0052, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_barrel",
    )
    latch.visual(
        Box((0.040, 0.006, 0.060)),
        origin=Origin(xyz=(0.0, 0.000, 0.030)),
        material=metal,
        name="toggle_plate",
    )
    latch.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, 0.060)),
        material=metal,
        name="hook_lip",
    )
    latch.visual(
        Box((0.018, 0.004, 0.038)),
        origin=Origin(xyz=(0.0, -0.0035, 0.034)),
        material=metal,
        name="raised_center_pull",
    )


def _add_lower_latch_mounts(lower, x: float, index: int, metal: Material) -> None:
    pivot_y = FRONT_Y - 0.017
    pivot_z = LOWER_HEIGHT - 0.010
    lower.visual(
        Box((0.052, 0.012, 0.030)),
        origin=Origin(xyz=(x, FRONT_Y - 0.006, pivot_z)),
        material=metal,
        name=f"latch_base_{index}",
    )
    for side, sx in (("a", -1.0), ("b", 1.0)):
        lower.visual(
            Box((0.006, 0.014, 0.024)),
            origin=Origin(xyz=(x + sx * 0.017, pivot_y, pivot_z)),
            material=metal,
            name=f"latch_ear_{index}_{side}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_shell_violin_case")

    shell_mat = model.material("charcoal_hard_shell", rgba=(0.035, 0.038, 0.042, 1.0))
    edge_mat = model.material("black_rubber_edge", rgba=(0.005, 0.005, 0.006, 1.0))
    velvet = model.material("burgundy_velvet", rgba=(0.38, 0.020, 0.055, 1.0))
    dark_velvet = model.material("shadowed_cavity_velvet", rgba=(0.14, 0.005, 0.018, 1.0))
    nickel = model.material("brushed_nickel", rgba=(0.70, 0.67, 0.60, 1.0))
    handle_mat = model.material("pebbled_black_handle", rgba=(0.012, 0.010, 0.009, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_geometry(_rounded_tray(length=CASE_LENGTH, width=CASE_WIDTH, height=LOWER_HEIGHT, wall=WALL, bottom=BOTTOM_THICKNESS), "lower_hard_shell"),
        material=shell_mat,
        name="lower_hard_shell",
    )
    lower.visual(
        mesh_from_geometry(
            _rounded_ring(
                CASE_LENGTH + 0.006,
                CASE_WIDTH + 0.006,
                CASE_LENGTH - 2.0 * WALL,
                CASE_WIDTH - 2.0 * WALL,
                0.006,
            ),
            "lower_rubber_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT - 0.002)),
        material=edge_mat,
        name="lower_rubber_rim",
    )
    lower.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(CASE_LENGTH - 2.0 * WALL - 0.018, CASE_WIDTH - 2.0 * WALL - 0.018, 0.026),
                0.003,
            ),
            "velvet_floor",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS - 0.0010)),
        material=velvet,
        name="velvet_floor",
    )
    lower.visual(
        mesh_from_geometry(ExtrudeGeometry.from_z0(_violin_cavity_profile(), 0.004), "violin_cavity"),
        origin=Origin(xyz=(0.018, 0.0, BOTTOM_THICKNESS + 0.0015)),
        material=dark_velvet,
        name="violin_cavity",
    )
    # Plush support blocks and bow troughs inside the shallow shell.
    for i, (x, y, sx, sy) in enumerate(
        [
            (-0.205, 0.000, 0.070, 0.034),
            (0.055, 0.000, 0.050, 0.028),
            (0.220, 0.000, 0.080, 0.034),
            (0.040, 0.116, 0.650, 0.014),
            (0.040, -0.116, 0.650, 0.014),
        ]
    ):
        lower.visual(
            Box((sx, sy, 0.007)),
            origin=Origin(xyz=(x, y, BOTTOM_THICKNESS + 0.0045)),
            material=velvet,
            name=f"plush_support_{i}",
        )

    # Rear hinge hardware: lower leaf and alternating knuckles on the long hinge axis.
    lower.visual(
        Box((0.760, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.002, LOWER_HEIGHT + 0.0005)),
        material=nickel,
        name="lower_hinge_leaf",
    )
    for i, (cx, length) in enumerate([(-0.300, 0.120), (0.000, 0.130), (0.300, 0.120)]):
        lower.visual(
            Box((length, 0.012, 0.006)),
            origin=Origin(xyz=(cx, HINGE_Y + 0.004, HINGE_Z + 0.003)),
            material=nickel,
            name=f"lower_hinge_web_{i}",
        )
        lower.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(cx, HINGE_Y + 0.010, HINGE_Z + 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nickel,
            name=f"lower_hinge_knuckle_{i}",
        )

    # Handle brackets and a continuous curved black grip between the latch stations.
    for i, sx in enumerate((-1.0, 1.0)):
        lower.visual(
            Box((0.026, 0.014, 0.028)),
            origin=Origin(xyz=(sx * 0.080, FRONT_Y - 0.006, LOWER_HEIGHT - 0.012)),
            material=nickel,
            name=f"handle_bracket_{i}",
        )
    lower.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.080, FRONT_Y - 0.010, LOWER_HEIGHT - 0.010),
                    (-0.055, FRONT_Y - 0.050, LOWER_HEIGHT - 0.021),
                    (0.000, FRONT_Y - 0.064, LOWER_HEIGHT - 0.024),
                    (0.055, FRONT_Y - 0.050, LOWER_HEIGHT - 0.021),
                    (0.080, FRONT_Y - 0.010, LOWER_HEIGHT - 0.010),
                ],
                radius=0.007,
                samples_per_segment=12,
                radial_segments=18,
            ),
            "front_handle",
        ),
        material=handle_mat,
        name="front_handle",
    )

    for index, x in enumerate(LATCH_XS):
        _add_lower_latch_mounts(lower, x, index, nickel)

    lower.inertial = Inertial.from_geometry(
        Box((CASE_LENGTH, CASE_WIDTH, LOWER_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEIGHT / 2.0)),
    )

    lid = model.part("upper_shell")
    lid.visual(
        mesh_from_geometry(_open_bottom_lid(length=CASE_LENGTH, width=CASE_WIDTH, height=LID_HEIGHT, wall=WALL, top=0.010), "upper_hard_shell"),
        origin=Origin(xyz=(0.0, -CASE_WIDTH / 2.0, LID_BOTTOM_GAP)),
        material=shell_mat,
        name="upper_hard_shell",
    )
    lid.visual(
        mesh_from_geometry(
            _rounded_ring(
                CASE_LENGTH + 0.006,
                CASE_WIDTH + 0.006,
                CASE_LENGTH - 2.0 * WALL,
                CASE_WIDTH - 2.0 * WALL,
                0.005,
            ),
            "upper_rubber_rim",
        ),
        origin=Origin(xyz=(0.0, -CASE_WIDTH / 2.0, LID_BOTTOM_GAP)),
        material=edge_mat,
        name="upper_rubber_rim",
    )
    for i, yoff in enumerate((-0.108, -0.212)):
        lid.visual(
            Box((0.690, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, yoff, LID_BOTTOM_GAP + LID_HEIGHT + 0.001)),
            material=edge_mat,
            name=f"raised_top_rib_{i}",
        )
    for index, x in enumerate(LATCH_XS):
        lid.visual(
            Box((0.054, 0.005, 0.038)),
            origin=Origin(xyz=(x, -CASE_WIDTH - 0.001, 0.034)),
            material=nickel,
            name=f"latch_strike_{index}",
        )
    lid.visual(
        Box((0.760, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, LID_BOTTOM_GAP + 0.002)),
        material=nickel,
        name="upper_hinge_leaf",
    )
    for i, (cx, length) in enumerate([(-0.150, 0.120), (0.150, 0.120)]):
        lid.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(cx, 0.005, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nickel,
            name=f"upper_hinge_knuckle_{i}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_LENGTH, CASE_WIDTH, LID_HEIGHT)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -CASE_WIDTH / 2.0, LID_HEIGHT / 2.0)),
    )

    lid_hinge = model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    lid_hinge.meta["qc_samples"] = [0.0, 0.8, 1.45]

    for index, x in enumerate(LATCH_XS):
        latch = model.part(f"front_latch_{index}")
        _latch_part(latch, nickel)
        latch.inertial = Inertial.from_geometry(
            Box((0.045, 0.010, 0.070)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.032)),
        )
        joint = model.articulation(
            f"latch_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=latch,
            origin=Origin(xyz=(x, FRONT_Y - 0.017, LOWER_HEIGHT - 0.010)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.15),
        )
        joint.meta["qc_samples"] = [0.0, 0.75, 1.15]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("upper_shell")
    lid_hinge = object_model.get_articulation("shell_to_lid")

    ctx.expect_gap(
        lid,
        lower,
        axis="z",
        min_gap=0.0005,
        max_gap=0.005,
        positive_elem="upper_hard_shell",
        negative_elem="lower_hard_shell",
        name="closed clamshell seam has small clearance",
    )
    ctx.expect_overlap(
        lid,
        lower,
        axes="xy",
        min_overlap=0.250,
        elem_a="upper_hard_shell",
        elem_b="lower_hard_shell",
        name="lid footprint matches lower shell",
    )

    lower_aabb = ctx.part_world_aabb(lower)
    ctx.check("case full size is violin-case scale", lower_aabb is not None, "Expected lower shell AABB.")
    if lower_aabb is not None:
        mins, maxs = lower_aabb
        ctx.check("case length realistic", 0.82 <= float(maxs[0] - mins[0]) <= 0.90, f"aabb={lower_aabb}")
        ctx.check("case width realistic including handle", 0.30 <= float(maxs[1] - mins[1]) <= 0.44, f"aabb={lower_aabb}")

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20, "latch_pivot_0": 1.0, "latch_pivot_1": 1.0}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid rotates upward about rear hinge",
            closed_aabb is not None and open_aabb is not None and float(open_aabb[1][2]) > float(closed_aabb[1][2]) + 0.16,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for index in range(len(LATCH_XS)):
        latch = object_model.get_part(f"front_latch_{index}")
        joint = object_model.get_articulation(f"latch_pivot_{index}")
        ctx.expect_contact(
            latch,
            lower,
            elem_a="pivot_barrel",
            elem_b=f"latch_ear_{index}_a",
            contact_tol=0.002,
            name=f"latch {index} pivot barrel is captured by clevis",
        )
        closed = ctx.part_world_aabb(latch)
        with ctx.pose({joint: 1.0}):
            released = ctx.part_world_aabb(latch)
        ctx.check(
            f"latch {index} swings outward to release",
            closed is not None and released is not None and float(released[0][1]) < float(closed[0][1]) - 0.010,
            details=f"closed={closed}, released={released}",
        )

    return ctx.report()


object_model = build_object_model()
