from __future__ import annotations

import math

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
    mesh_from_geometry,
)

OUTER_L = 1.55
OUTER_D = 0.76
BASE_T = 0.07
WALL_T = 0.055
WALL_H = 0.84
TOP_CAP_T = 0.04
TOP_CAP_Z = BASE_T + WALL_H - (TOP_CAP_T / 2.0)

OPEN_L = OUTER_L - (2.0 * WALL_T)
OPEN_D = OUTER_D - (2.0 * WALL_T)

LID_DEPTH = 0.615
LID_CENTER_GAP = 0.010
LID_LENGTH = (OPEN_L - LID_CENTER_GAP) / 2.0
LID_TRAVEL = OPEN_L - LID_LENGTH
LID_FRAME_W = 0.040
LID_FRAME_H = 0.016
GLASS_T = 0.004
HANDLE_W = 0.140
HANDLE_D = 0.018
HANDLE_H = 0.006

TRACK_L = OPEN_L - 0.020
TRACK_D = 0.030
LOWER_TRACK_T = 0.020
UPPER_TRACK_T = 0.008
LOWER_TRACK_Z = 0.870
UPPER_TRACK_Z = 0.904
LOWER_TRACK_TOP = LOWER_TRACK_Z + (LOWER_TRACK_T / 2.0)
UPPER_TRACK_TOP = UPPER_TRACK_Z + (UPPER_TRACK_T / 2.0)
TRACK_Y = (OPEN_D / 2.0) - 0.015

LOCK_X = 0.58
LOCK_Z = 0.74
FLAP_W = 0.085
FLAP_H = 0.118
FLAP_T = 0.006
HINGE_X = LOCK_X + (FLAP_W / 2.0)
FLAP_Y = -(OUTER_D / 2.0) - 0.014

KNOB_Y = 0.18
KNOB_Z = 0.48
KNOB_BEZEL_T = 0.002


def _add_lid_visuals(part, *, frame_material, glass_material, handle_material) -> None:
    half_depth = LID_DEPTH / 2.0

    part.visual(
        Box((LID_LENGTH, LID_FRAME_W, LID_FRAME_H)),
        origin=Origin(
            xyz=(LID_LENGTH / 2.0, -half_depth + (LID_FRAME_W / 2.0), LID_FRAME_H / 2.0)
        ),
        material=frame_material,
        name="front_rail",
    )
    part.visual(
        Box((LID_LENGTH, LID_FRAME_W, LID_FRAME_H)),
        origin=Origin(
            xyz=(LID_LENGTH / 2.0, half_depth - (LID_FRAME_W / 2.0), LID_FRAME_H / 2.0)
        ),
        material=frame_material,
        name="rear_rail",
    )
    part.visual(
        Box((LID_FRAME_W, LID_DEPTH, LID_FRAME_H)),
        origin=Origin(xyz=(LID_FRAME_W / 2.0, 0.0, LID_FRAME_H / 2.0)),
        material=frame_material,
        name="left_stile",
    )
    part.visual(
        Box((LID_FRAME_W, LID_DEPTH, LID_FRAME_H)),
        origin=Origin(xyz=(LID_LENGTH - (LID_FRAME_W / 2.0), 0.0, LID_FRAME_H / 2.0)),
        material=frame_material,
        name="right_stile",
    )
    part.visual(
        Box((LID_LENGTH - (2.0 * LID_FRAME_W), LID_DEPTH - (2.0 * LID_FRAME_W), GLASS_T)),
        origin=Origin(xyz=(LID_LENGTH / 2.0, 0.0, 0.005)),
        material=glass_material,
        name="glass",
    )
    part.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(
            xyz=(
                LID_LENGTH / 2.0,
                -half_depth + (LID_FRAME_W * 0.72),
                LID_FRAME_H + (HANDLE_H / 2.0),
            )
        ),
        material=handle_material,
        name="pull",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convenience_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner_white = model.material("liner_white", rgba=(0.98, 0.98, 0.99, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.86, 0.92, 0.35))
    seal_dark = model.material("seal_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.36, 0.38, 0.40, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((OUTER_L, OUTER_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=cabinet_white,
        name="outer_floor",
    )
    cabinet.visual(
        Box((WALL_T, OUTER_D, WALL_H)),
        origin=Origin(xyz=(-(OUTER_L / 2.0) + (WALL_T / 2.0), 0.0, BASE_T + (WALL_H / 2.0))),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL_T, OUTER_D, WALL_H)),
        origin=Origin(xyz=((OUTER_L / 2.0) - (WALL_T / 2.0), 0.0, BASE_T + (WALL_H / 2.0))),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((OUTER_L - (2.0 * WALL_T), WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, -(OUTER_D / 2.0) + (WALL_T / 2.0), BASE_T + (WALL_H / 2.0))),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((OUTER_L - (2.0 * WALL_T), WALL_T, WALL_H)),
        origin=Origin(xyz=(0.0, (OUTER_D / 2.0) - (WALL_T / 2.0), BASE_T + (WALL_H / 2.0))),
        material=cabinet_white,
        name="rear_wall",
    )

    liner_inset_x = 0.075
    liner_inset_y = 0.090
    liner_t = 0.010
    liner_h = 0.74
    liner_floor_z = BASE_T + (liner_t / 2.0)
    liner_side_x = (OUTER_L / 2.0) - liner_inset_x - (liner_t / 2.0)
    liner_front_y = -(OUTER_D / 2.0) + liner_inset_y + (liner_t / 2.0)
    liner_rear_y = -liner_front_y
    liner_inside_l = (2.0 * liner_side_x) - liner_t
    liner_inside_d = (liner_rear_y - liner_front_y) - liner_t

    cabinet.visual(
        Box((liner_inside_l, liner_inside_d, liner_t)),
        origin=Origin(xyz=(0.0, 0.0, liner_floor_z)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((liner_t, liner_inside_d, liner_h)),
        origin=Origin(xyz=(-liner_side_x, 0.0, BASE_T + (liner_h / 2.0))),
        material=liner_white,
        name="liner_left",
    )
    cabinet.visual(
        Box((liner_t, liner_inside_d, liner_h)),
        origin=Origin(xyz=(liner_side_x, 0.0, BASE_T + (liner_h / 2.0))),
        material=liner_white,
        name="liner_right",
    )
    cabinet.visual(
        Box((liner_inside_l, liner_t, liner_h)),
        origin=Origin(xyz=(0.0, liner_front_y, BASE_T + (liner_h / 2.0))),
        material=liner_white,
        name="liner_front",
    )
    cabinet.visual(
        Box((liner_inside_l, liner_t, liner_h)),
        origin=Origin(xyz=(0.0, liner_rear_y, BASE_T + (liner_h / 2.0))),
        material=liner_white,
        name="liner_rear",
    )

    cabinet.visual(
        Box((OUTER_L, WALL_T, TOP_CAP_T)),
        origin=Origin(xyz=(0.0, -(OUTER_D / 2.0) + (WALL_T / 2.0), TOP_CAP_Z)),
        material=trim_silver,
        name="front_cap",
    )
    cabinet.visual(
        Box((OUTER_L, WALL_T, TOP_CAP_T)),
        origin=Origin(xyz=(0.0, (OUTER_D / 2.0) - (WALL_T / 2.0), TOP_CAP_Z)),
        material=trim_silver,
        name="rear_cap",
    )
    cabinet.visual(
        Box((WALL_T, OUTER_D - (2.0 * WALL_T), TOP_CAP_T)),
        origin=Origin(xyz=(-(OUTER_L / 2.0) + (WALL_T / 2.0), 0.0, TOP_CAP_Z)),
        material=trim_silver,
        name="left_cap",
    )
    cabinet.visual(
        Box((WALL_T, OUTER_D - (2.0 * WALL_T), TOP_CAP_T)),
        origin=Origin(xyz=((OUTER_L / 2.0) - (WALL_T / 2.0), 0.0, TOP_CAP_Z)),
        material=trim_silver,
        name="right_cap",
    )

    cabinet.visual(
        Box((TRACK_L, TRACK_D, LOWER_TRACK_T)),
        origin=Origin(xyz=(0.0, -TRACK_Y, LOWER_TRACK_Z)),
        material=trim_silver,
        name="lower_track_front",
    )
    cabinet.visual(
        Box((TRACK_L, TRACK_D, LOWER_TRACK_T)),
        origin=Origin(xyz=(0.0, TRACK_Y, LOWER_TRACK_Z)),
        material=trim_silver,
        name="lower_track_rear",
    )
    cabinet.visual(
        Box((TRACK_L, TRACK_D, UPPER_TRACK_T)),
        origin=Origin(xyz=(0.0, -TRACK_Y, UPPER_TRACK_Z)),
        material=trim_silver,
        name="upper_track_front",
    )
    cabinet.visual(
        Box((TRACK_L, TRACK_D, UPPER_TRACK_T)),
        origin=Origin(xyz=(0.0, TRACK_Y, UPPER_TRACK_Z)),
        material=trim_silver,
        name="upper_track_rear",
    )

    cabinet.visual(
        Box((0.070, 0.004, 0.090)),
        origin=Origin(xyz=(LOCK_X, -(OUTER_D / 2.0) - 0.002, LOCK_Z)),
        material=metal_dark,
        name="lock_bezel",
    )
    cabinet.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(
            xyz=(LOCK_X, -(OUTER_D / 2.0) - 0.004, LOCK_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_dark,
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.012, 0.010, FLAP_H)),
        origin=Origin(xyz=(HINGE_X + 0.006, -(OUTER_D / 2.0) - 0.005, LOCK_Z)),
        material=seal_dark,
        name="flap_keeper",
    )

    cabinet.visual(
        Box((KNOB_BEZEL_T, 0.092, 0.092)),
        origin=Origin(xyz=((OUTER_L / 2.0) + (KNOB_BEZEL_T / 2.0), KNOB_Y, KNOB_Z)),
        material=metal_dark,
        name="control_bezel",
    )

    lid_0 = model.part("lid_0")
    _add_lid_visuals(
        lid_0,
        frame_material=trim_silver,
        glass_material=glass,
        handle_material=black_plastic,
    )

    lid_1 = model.part("lid_1")
    _add_lid_visuals(
        lid_1,
        frame_material=trim_silver,
        glass_material=glass,
        handle_material=black_plastic,
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-(OPEN_L / 2.0), 0.0, LOWER_TRACK_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.40, lower=0.0, upper=LID_TRAVEL),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=((OPEN_L / 2.0) - LID_LENGTH, 0.0, UPPER_TRACK_TOP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.40, lower=0.0, upper=LID_TRAVEL),
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(-(FLAP_W / 2.0), 0.0, 0.0)),
        material=seal_dark,
        name="flap_panel",
    )
    lock_flap.visual(
        Cylinder(radius=0.004, length=FLAP_H),
        origin=Origin(),
        material=black_plastic,
        name="hinge_barrel",
    )
    lock_flap.visual(
        Box((0.012, 0.010, 0.040)),
        origin=Origin(xyz=(-(FLAP_W - 0.006), -0.002, 0.0)),
        material=black_plastic,
        name="pull_lip",
    )

    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(HINGE_X, FLAP_Y, LOCK_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.022,
                body_style="skirted",
                top_diameter=0.040,
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "freezer_temperature_knob",
        ),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_shell",
    )

    model.articulation(
        "cabinet_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=((OUTER_L / 2.0) + KNOB_BEZEL_T, KNOB_Y, KNOB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )

    return model


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[index] - mins[index]) for index in range(3))


def _lid_stays_over_opening(ctx: TestContext, lid_name: str) -> bool:
    lid = object_model.get_part(lid_name)
    aabb = ctx.part_world_aabb(lid)
    if aabb is None:
        return False
    mins, maxs = aabb
    return (
        mins[0] >= (-(OPEN_L / 2.0) - 0.002)
        and maxs[0] <= ((OPEN_L / 2.0) + 0.002)
        and mins[1] >= (-(OPEN_D / 2.0) - 0.002)
        and maxs[1] <= ((OPEN_D / 2.0) + 0.002)
        and mins[2] >= 0.878
        and maxs[2] <= 0.934
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    lock_flap = object_model.get_part("lock_flap")
    knob = object_model.get_part("knob")

    lid_0_joint = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_joint = object_model.get_articulation("cabinet_to_lid_1")
    flap_joint = object_model.get_articulation("cabinet_to_lock_flap")
    knob_joint = object_model.get_articulation("cabinet_to_knob")

    cabinet_size = _aabb_size(ctx.part_world_aabb(cabinet))
    ctx.check("cabinet_aabb_present", cabinet_size is not None, "Expected a cabinet AABB.")
    if cabinet_size is not None:
        ctx.check(
            "cabinet_store_scale",
            1.45 <= cabinet_size[0] <= 1.65
            and 0.72 <= cabinet_size[1] <= 0.80
            and 0.90 <= cabinet_size[2] <= 0.93,
            f"size={cabinet_size!r}",
        )
        ctx.check(
            "cabinet_reads_as_deep_chest",
            cabinet_size[1] >= 0.74 and cabinet_size[2] >= 0.90,
            f"size={cabinet_size!r}",
        )

    with ctx.pose({lid_0_joint: 0.0, lid_1_joint: 0.0}):
        ctx.expect_contact(
            lid_0,
            cabinet,
            elem_a="front_rail",
            elem_b="lower_track_front",
            name="lid_0 front rail sits on the lower front track",
        )
        ctx.expect_contact(
            lid_0,
            cabinet,
            elem_a="rear_rail",
            elem_b="lower_track_rear",
            name="lid_0 rear rail sits on the lower rear track",
        )
        ctx.expect_contact(
            lid_1,
            cabinet,
            elem_a="front_rail",
            elem_b="upper_track_front",
            name="lid_1 front rail sits on the upper front track",
        )
        ctx.expect_contact(
            lid_1,
            cabinet,
            elem_a="rear_rail",
            elem_b="upper_track_rear",
            name="lid_1 rear rail sits on the upper rear track",
        )
        ctx.check(
            "closed lids stay within the opening",
            _lid_stays_over_opening(ctx, "lid_0") and _lid_stays_over_opening(ctx, "lid_1"),
            details=f"lid_0={ctx.part_world_aabb(lid_0)!r}, lid_1={ctx.part_world_aabb(lid_1)!r}",
        )
        ctx.expect_overlap(
            lock_flap,
            cabinet,
            axes="xz",
            elem_a="flap_panel",
            elem_b="lock_bezel",
            min_overlap=0.060,
            name="lock flap covers the lock bezel at rest",
        )
        ctx.expect_gap(
            cabinet,
            lock_flap,
            axis="y",
            positive_elem="lock_bezel",
            negative_elem="flap_panel",
            min_gap=0.0,
            max_gap=0.015,
            name="lock flap sits just ahead of the lock bezel",
        )
        ctx.expect_overlap(
            knob,
            cabinet,
            axes="yz",
            elem_a="knob_shell",
            elem_b="control_bezel",
            min_overlap=0.050,
            name="knob stays centered on the end control bezel",
        )
        ctx.expect_gap(
            knob,
            cabinet,
            axis="x",
            positive_elem="knob_shell",
            negative_elem="right_wall",
            min_gap=0.015,
            max_gap=0.050,
            name="knob projects clearly from the end wall",
        )

    lid_0_rest = ctx.part_world_position(lid_0)
    lid_1_rest = ctx.part_world_position(lid_1)

    lid_0_upper = lid_0_joint.motion_limits.upper if lid_0_joint.motion_limits is not None else None
    lid_1_upper = lid_1_joint.motion_limits.upper if lid_1_joint.motion_limits is not None else None
    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else None

    if lid_0_upper is not None:
        with ctx.pose({lid_0_joint: lid_0_upper}):
            lid_0_open = ctx.part_world_position(lid_0)
            ctx.check(
                "lid_0 slides toward the opposite side",
                lid_0_rest is not None
                and lid_0_open is not None
                and lid_0_open[0] > lid_0_rest[0] + 0.60,
                details=f"rest={lid_0_rest!r}, open={lid_0_open!r}",
            )
            ctx.check(
                "lid_0 stays on the tracks when open",
                _lid_stays_over_opening(ctx, "lid_0"),
                details=f"aabb={ctx.part_world_aabb(lid_0)!r}",
            )

    if lid_1_upper is not None:
        with ctx.pose({lid_1_joint: lid_1_upper}):
            lid_1_open = ctx.part_world_position(lid_1)
            ctx.check(
                "lid_1 slides toward the opposite side",
                lid_1_rest is not None
                and lid_1_open is not None
                and lid_1_open[0] < lid_1_rest[0] - 0.60,
                details=f"rest={lid_1_rest!r}, open={lid_1_open!r}",
            )
            ctx.check(
                "lid_1 stays on the tracks when open",
                _lid_stays_over_opening(ctx, "lid_1"),
                details=f"aabb={ctx.part_world_aabb(lid_1)!r}",
            )

    if lid_0_upper is not None:
        with ctx.pose({lid_0_joint: lid_0_upper, lid_1_joint: 0.0}):
            ctx.expect_gap(
                lid_1,
                lid_0,
                axis="z",
                elem_a="glass",
                elem_b="glass",
                min_gap=0.008,
                name="upper lid remains above the lower glass when the panels overlap",
            )

    closed_flap = ctx.part_element_world_aabb(lock_flap, elem="flap_panel")
    if flap_upper is not None:
        with ctx.pose({flap_joint: flap_upper}):
            open_flap = ctx.part_element_world_aabb(lock_flap, elem="flap_panel")
            ctx.check(
                "lock flap opens outward",
                closed_flap is not None
                and open_flap is not None
                and open_flap[0][1] < closed_flap[0][1] - 0.020,
                details=f"closed={closed_flap!r}, open={open_flap!r}",
            )

    knob_limits = getattr(knob_joint, "motion_limits", None)
    ctx.check(
        "knob joint is unbounded like a real control knob",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        details=f"motion_limits={knob_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
