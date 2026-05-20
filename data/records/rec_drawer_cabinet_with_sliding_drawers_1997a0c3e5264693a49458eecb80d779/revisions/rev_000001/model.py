from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_W = 0.40
CABINET_D = 0.68
CABINET_H = 1.10
STEEL_T = 0.012
FACE_T = 0.018
DRAWER_WALL_T = 0.010
BOTTOM_T = 0.010
RUNNER_T = 0.006
FIXED_RAIL_T = 0.016
RAIL_H = 0.018
FRONT_GAP = 0.003
SIDE_GAP = 0.003
RAIL_FRONT_SETBACK = 0.080

TOP_CAP_H = 0.018
TOP_RAIL_H = 0.060
ACCESSORY_H = 0.150
MID_RAIL_H = 0.024
FILE_H = 0.375
BASE_H = CABINET_H - TOP_RAIL_H - ACCESSORY_H - (2.0 * MID_RAIL_H) - (2.0 * FILE_H)

ACCESSORY_BOX_D = 0.46
FILE_BOX_D = 0.60
ACCESSORY_BODY_H = 0.108
FILE_BODY_H = 0.310
DRAWER_BODY_W = 0.338
DRAWER_FRONT_W = CABINET_W - (2.0 * STEEL_T) - (2.0 * SIDE_GAP)

ACCESSORY_TRAVEL = 0.24
FILE_TRAVEL = 0.34

LOCK_COVER_W = 0.074
LOCK_COVER_H = 0.034
LOCK_COVER_T = 0.006


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _add_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    front_h: float,
    body_h: float,
    box_d: float,
    handle_span: float,
    handle_z: float,
    rail_prefix: str,
    material: str,
    pull_material: str,
    hanging_rails: bool,
) -> None:
    drawer = model.part(name)

    drawer.visual(
        Box((FACE_T, DRAWER_FRONT_W, front_h)),
        origin=Origin(xyz=(-FACE_T / 2.0, 0.0, 0.0)),
        material=material,
        name="front_panel",
    )

    inner_w = DRAWER_BODY_W - (2.0 * DRAWER_WALL_T)
    body_x = -FACE_T - (box_d / 2.0)
    side_y = (DRAWER_BODY_W / 2.0) - (DRAWER_WALL_T / 2.0)
    bottom_z = -(body_h / 2.0) + (BOTTOM_T / 2.0)

    drawer.visual(
        Box((box_d, DRAWER_WALL_T, body_h)),
        origin=Origin(xyz=(body_x, side_y, 0.0)),
        material=material,
        name="side_wall_0",
    )
    drawer.visual(
        Box((box_d, DRAWER_WALL_T, body_h)),
        origin=Origin(xyz=(body_x, -side_y, 0.0)),
        material=material,
        name="side_wall_1",
    )
    drawer.visual(
        Box((DRAWER_WALL_T, inner_w, body_h)),
        origin=Origin(
            xyz=(-FACE_T - box_d + (DRAWER_WALL_T / 2.0), 0.0, 0.0)
        ),
        material=material,
        name="back_wall",
    )
    drawer.visual(
        Box((box_d - DRAWER_WALL_T, inner_w, BOTTOM_T)),
        origin=Origin(
            xyz=(-FACE_T - ((box_d - DRAWER_WALL_T) / 2.0), 0.0, bottom_z)
        ),
        material=material,
        name="bottom_pan",
    )

    handle_proj = 0.016
    handle_bar_t = 0.010
    stem_w = 0.018
    stem_h = 0.022
    stem_y = (handle_span * 0.32) / 2.0

    drawer.visual(
        Box((handle_proj, stem_w, stem_h)),
        origin=Origin(xyz=(handle_proj / 2.0, stem_y, handle_z)),
        material=pull_material,
        name="pull_stem_0",
    )
    drawer.visual(
        Box((handle_proj, stem_w, stem_h)),
        origin=Origin(xyz=(handle_proj / 2.0, -stem_y, handle_z)),
        material=pull_material,
        name="pull_stem_1",
    )
    drawer.visual(
        Box((handle_bar_t, handle_span, 0.018)),
        origin=Origin(
            xyz=(handle_proj + (handle_bar_t / 2.0), 0.0, handle_z),
        ),
        material=pull_material,
        name="pull_bar",
    )

    runner_len = box_d - RAIL_FRONT_SETBACK
    runner_x = -RAIL_FRONT_SETBACK - (runner_len / 2.0)
    runner_y = (DRAWER_BODY_W / 2.0) + (RUNNER_T / 2.0)
    runner_z = -(body_h * 0.08)

    drawer.visual(
        Box((runner_len, RUNNER_T, RAIL_H)),
        origin=Origin(xyz=(runner_x, runner_y, runner_z)),
        material=pull_material,
        name=f"{rail_prefix}_runner_0",
    )
    drawer.visual(
        Box((runner_len, RUNNER_T, RAIL_H)),
        origin=Origin(xyz=(runner_x, -runner_y, runner_z)),
        material=pull_material,
        name=f"{rail_prefix}_runner_1",
    )

    if hanging_rails:
        hanger_len = box_d - 0.06
        hanger_y = (DRAWER_BODY_W / 2.0) - DRAWER_WALL_T - 0.005
        hanger_z = (body_h / 2.0) - 0.008
        drawer.visual(
            Box((hanger_len, 0.012, 0.016)),
            origin=Origin(
                xyz=(-0.06 - (hanger_len / 2.0), hanger_y, hanger_z)
            ),
            material=pull_material,
            name="hanger_rail_0",
        )
        drawer.visual(
            Box((hanger_len, 0.012, 0.016)),
            origin=Origin(
                xyz=(-0.06 - (hanger_len / 2.0), -hanger_y, hanger_z)
            ),
            material=pull_material,
            name="hanger_rail_1",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="filing_cabinet")

    model.material("cabinet_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("drawer_paint", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("runner_metal", rgba=(0.52, 0.55, 0.58, 1.0))
    model.material("shadow_trim", rgba=(0.22, 0.23, 0.25, 1.0))

    cabinet = model.part("cabinet")
    side_y = (CABINET_W / 2.0) - (STEEL_T / 2.0)

    cabinet.visual(
        Box((CABINET_D, STEEL_T, CABINET_H)),
        origin=Origin(xyz=(0.0, side_y, CABINET_H / 2.0)),
        material="cabinet_paint",
        name="side_shell_0",
    )
    cabinet.visual(
        Box((CABINET_D, STEEL_T, CABINET_H)),
        origin=Origin(xyz=(0.0, -side_y, CABINET_H / 2.0)),
        material="cabinet_paint",
        name="side_shell_1",
    )
    cabinet.visual(
        Box((STEEL_T, CABINET_W - (2.0 * STEEL_T), CABINET_H - STEEL_T)),
        origin=Origin(
            xyz=(
                -CABINET_D / 2.0 + (STEEL_T / 2.0),
                0.0,
                (CABINET_H - STEEL_T) / 2.0,
            )
        ),
        material="cabinet_paint",
        name="rear_panel",
    )
    cabinet.visual(
        Box((CABINET_D, CABINET_W, STEEL_T)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_H - (STEEL_T / 2.0))),
        material="cabinet_paint",
        name="top_shell",
    )
    cabinet.visual(
        Box((CABINET_D - 0.03, CABINET_W - (2.0 * STEEL_T), STEEL_T)),
        origin=Origin(xyz=(-0.015, 0.0, STEEL_T / 2.0)),
        material="cabinet_paint",
        name="bottom_pan",
    )
    cabinet.visual(
        Box((FACE_T, CABINET_W - (2.0 * STEEL_T), BASE_H)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - (FACE_T / 2.0) - 0.025,
                0.0,
                BASE_H / 2.0,
            )
        ),
        material="shadow_trim",
        name="kick_face",
    )

    rail_y = (CABINET_W / 2.0) - STEEL_T - (FIXED_RAIL_T / 2.0)
    accessory_center_z = BASE_H + MID_RAIL_H + FILE_H + MID_RAIL_H + FILE_H + (ACCESSORY_H / 2.0)
    upper_file_center_z = BASE_H + MID_RAIL_H + FILE_H + (FILE_H / 2.0)
    lower_file_center_z = BASE_H + (FILE_H / 2.0)

    cover_gap = LOCK_COVER_W + 0.010
    side_rail_w = ((CABINET_W - (2.0 * STEEL_T)) - cover_gap) / 2.0
    top_lower_h = TOP_RAIL_H - TOP_CAP_H
    lower_top_center_z = CABINET_H - TOP_CAP_H - (top_lower_h / 2.0)

    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        cabinet.visual(
            Box((FACE_T, side_rail_w, top_lower_h)),
            origin=Origin(
                xyz=(
                    CABINET_D / 2.0 - (FACE_T / 2.0),
                    sign * ((cover_gap / 2.0) + (side_rail_w / 2.0)),
                    lower_top_center_z,
                )
            ),
            material="cabinet_paint",
            name=f"top_rail_{suffix}",
        )

    cabinet.visual(
        Box((FACE_T, CABINET_W - (2.0 * STEEL_T), TOP_CAP_H)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - (FACE_T / 2.0),
                0.0,
                CABINET_H - (TOP_CAP_H / 2.0),
            )
        ),
        material="cabinet_paint",
        name="top_cap",
    )
    cabinet.visual(
        Box((0.032, 0.088, 0.030)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - FACE_T - 0.017,
                0.0,
                CABINET_H - TOP_CAP_H - 0.015,
            )
        ),
        material="shadow_trim",
        name="lock_box",
    )
    cabinet.visual(
        Box((0.030, 0.020, 0.018)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - FACE_T + 0.001,
                0.050,
                CABINET_H - TOP_CAP_H - 0.015,
            )
        ),
        material="shadow_trim",
        name="lock_brace_0",
    )
    cabinet.visual(
        Box((0.030, 0.020, 0.018)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - FACE_T + 0.001,
                -0.050,
                CABINET_H - TOP_CAP_H - 0.015,
            )
        ),
        material="shadow_trim",
        name="lock_brace_1",
    )

    rail_center_x_accessory = CABINET_D / 2.0 - RAIL_FRONT_SETBACK - (
        (ACCESSORY_BOX_D - RAIL_FRONT_SETBACK) / 2.0
    )
    rail_center_x_file = CABINET_D / 2.0 - RAIL_FRONT_SETBACK - (
        (FILE_BOX_D - RAIL_FRONT_SETBACK) / 2.0
    )

    for prefix, center_z, center_x, rail_len in (
        (
            "accessory",
            accessory_center_z,
            rail_center_x_accessory,
            ACCESSORY_BOX_D - RAIL_FRONT_SETBACK,
        ),
        (
            "upper_file",
            upper_file_center_z,
            rail_center_x_file,
            FILE_BOX_D - RAIL_FRONT_SETBACK,
        ),
        (
            "lower_file",
            lower_file_center_z,
            rail_center_x_file,
            FILE_BOX_D - RAIL_FRONT_SETBACK,
        ),
    ):
        cabinet.visual(
            Box((rail_len, FIXED_RAIL_T, RAIL_H)),
            origin=Origin(xyz=(center_x, rail_y, center_z - 0.01)),
            material="runner_metal",
            name=f"{prefix}_rail_0",
        )
        cabinet.visual(
            Box((rail_len, FIXED_RAIL_T, RAIL_H)),
            origin=Origin(xyz=(center_x, -rail_y, center_z - 0.01)),
            material="runner_metal",
            name=f"{prefix}_rail_1",
        )

    cabinet.visual(
        Box((FACE_T, CABINET_W - (2.0 * STEEL_T), MID_RAIL_H)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - (FACE_T / 2.0),
                0.0,
                BASE_H + FILE_H + (MID_RAIL_H / 2.0),
            )
        ),
        material="cabinet_paint",
        name="mid_rail_0",
    )
    cabinet.visual(
        Box((FACE_T, CABINET_W - (2.0 * STEEL_T), MID_RAIL_H)),
        origin=Origin(
            xyz=(
                CABINET_D / 2.0 - (FACE_T / 2.0),
                0.0,
                BASE_H + FILE_H + MID_RAIL_H + FILE_H + (MID_RAIL_H / 2.0),
            )
        ),
        material="cabinet_paint",
        name="mid_rail_1",
    )

    _add_drawer(
        model,
        name="accessory_drawer",
        front_h=ACCESSORY_H - (2.0 * FRONT_GAP),
        body_h=ACCESSORY_BODY_H,
        box_d=ACCESSORY_BOX_D,
        handle_span=0.140,
        handle_z=0.0,
        rail_prefix="accessory",
        material="drawer_paint",
        pull_material="runner_metal",
        hanging_rails=False,
    )
    _add_drawer(
        model,
        name="file_drawer_0",
        front_h=FILE_H - (2.0 * FRONT_GAP),
        body_h=FILE_BODY_H,
        box_d=FILE_BOX_D,
        handle_span=0.180,
        handle_z=0.0,
        rail_prefix="upper_file",
        material="drawer_paint",
        pull_material="runner_metal",
        hanging_rails=True,
    )
    _add_drawer(
        model,
        name="file_drawer_1",
        front_h=FILE_H - (2.0 * FRONT_GAP),
        body_h=FILE_BODY_H,
        box_d=FILE_BOX_D,
        handle_span=0.180,
        handle_z=0.0,
        rail_prefix="lower_file",
        material="drawer_paint",
        pull_material="runner_metal",
        hanging_rails=True,
    )

    lock_cover = model.part("lock_cover")
    lock_cover.visual(
        Box((LOCK_COVER_T, LOCK_COVER_W, LOCK_COVER_H)),
        origin=Origin(xyz=(-LOCK_COVER_T / 2.0, 0.0, -(LOCK_COVER_H / 2.0))),
        material="drawer_paint",
        name="cover_panel",
    )
    lock_cover.visual(
        Box((0.010, 0.032, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, -(LOCK_COVER_H * 0.62))),
        material="shadow_trim",
        name="cover_pull",
    )

    model.articulation(
        "accessory_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child="accessory_drawer",
        origin=Origin(xyz=(CABINET_D / 2.0, 0.0, accessory_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=ACCESSORY_TRAVEL,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "upper_file_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child="file_drawer_0",
        origin=Origin(xyz=(CABINET_D / 2.0, 0.0, upper_file_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FILE_TRAVEL,
            effort=180.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "lower_file_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child="file_drawer_1",
        origin=Origin(xyz=(CABINET_D / 2.0, 0.0, lower_file_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=FILE_TRAVEL,
            effort=180.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "lock_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_cover,
        origin=Origin(
            xyz=(
                CABINET_D / 2.0,
                0.0,
                CABINET_H - TOP_CAP_H,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.20,
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    accessory_drawer = object_model.get_part("accessory_drawer")
    upper_file = object_model.get_part("file_drawer_0")
    lower_file = object_model.get_part("file_drawer_1")
    lock_cover = object_model.get_part("lock_cover")

    accessory_slide = object_model.get_articulation("accessory_slide")
    upper_file_slide = object_model.get_articulation("upper_file_slide")
    lower_file_slide = object_model.get_articulation("lower_file_slide")
    lock_cover_hinge = object_model.get_articulation("lock_cover_hinge")

    ctx.expect_gap(
        accessory_drawer,
        cabinet,
        axis="x",
        positive_elem="back_wall",
        negative_elem="rear_panel",
        min_gap=0.010,
        name="accessory drawer clears the rear panel when closed",
    )
    ctx.expect_gap(
        upper_file,
        cabinet,
        axis="x",
        positive_elem="back_wall",
        negative_elem="rear_panel",
        min_gap=0.010,
        name="upper file drawer clears the rear panel when closed",
    )
    ctx.expect_gap(
        lower_file,
        cabinet,
        axis="x",
        positive_elem="back_wall",
        negative_elem="rear_panel",
        min_gap=0.010,
        name="lower file drawer clears the rear panel when closed",
    )
    ctx.expect_gap(
        lock_cover,
        cabinet,
        axis="x",
        positive_elem="cover_panel",
        negative_elem="lock_box",
        min_gap=0.001,
        name="lock cover sits ahead of the lock housing",
    )

    accessory_rest = ctx.part_world_position(accessory_drawer)
    with ctx.pose({accessory_slide: ACCESSORY_TRAVEL}):
        accessory_open = ctx.part_world_position(accessory_drawer)
        ctx.expect_overlap(
            accessory_drawer,
            cabinet,
            axes="x",
            elem_a="accessory_runner_0",
            elem_b="accessory_rail_0",
            min_overlap=0.120,
            name="accessory drawer retains runner engagement at full travel",
        )
    ctx.check(
        "accessory drawer opens outward",
        accessory_rest is not None
        and accessory_open is not None
        and accessory_open[0] > accessory_rest[0] + 0.20,
        details=f"rest={accessory_rest}, open={accessory_open}",
    )

    upper_file_rest = ctx.part_world_position(upper_file)
    with ctx.pose({upper_file_slide: FILE_TRAVEL}):
        upper_file_open = ctx.part_world_position(upper_file)
        ctx.expect_overlap(
            upper_file,
            cabinet,
            axes="x",
            elem_a="upper_file_runner_0",
            elem_b="upper_file_rail_0",
            min_overlap=0.140,
            name="upper file drawer retains runner engagement at full travel",
        )
    ctx.check(
        "upper file drawer opens outward",
        upper_file_rest is not None
        and upper_file_open is not None
        and upper_file_open[0] > upper_file_rest[0] + 0.30,
        details=f"rest={upper_file_rest}, open={upper_file_open}",
    )

    lower_file_rest = ctx.part_world_position(lower_file)
    with ctx.pose({lower_file_slide: FILE_TRAVEL}):
        lower_file_open = ctx.part_world_position(lower_file)
        ctx.expect_overlap(
            lower_file,
            cabinet,
            axes="x",
            elem_a="lower_file_runner_0",
            elem_b="lower_file_rail_0",
            min_overlap=0.140,
            name="lower file drawer retains runner engagement at full travel",
        )
    ctx.check(
        "lower file drawer opens outward",
        lower_file_rest is not None
        and lower_file_open is not None
        and lower_file_open[0] > lower_file_rest[0] + 0.30,
        details=f"rest={lower_file_rest}, open={lower_file_open}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_panel")
    with ctx.pose({lock_cover_hinge: 1.0}):
        open_cover_aabb = ctx.part_element_world_aabb(lock_cover, elem="cover_panel")
    closed_center = _aabb_center(closed_cover_aabb)
    open_center = _aabb_center(open_cover_aabb)

    ctx.check(
        "lock cover flips upward and outward",
        closed_center is not None
        and open_center is not None
        and open_center[0] > closed_center[0] + 0.010
        and open_center[2] > closed_center[2] + 0.004,
        details=f"closed={closed_center}, open={open_center}",
    )

    return ctx.report()


object_model = build_object_model()
