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


def _add_lid(
    part,
    *,
    length: float,
    width: float,
    frame_w: float,
    frame_t: float,
    glass_t: float,
    runner_sign: float,
    runner_drop: float,
) -> None:
    side_y = width / 2.0 - frame_w / 2.0
    end_x = length / 2.0 - frame_w / 2.0
    zc = frame_t / 2.0

    part.visual(
        Box((length, frame_w, frame_t)),
        origin=Origin(xyz=(0.0, side_y, zc)),
        material="frame",
        name="front_edge",
    )
    part.visual(
        Box((length, frame_w, frame_t)),
        origin=Origin(xyz=(0.0, -side_y, zc)),
        material="frame",
        name="rear_edge",
    )
    part.visual(
        Box((frame_w, width - 2.0 * frame_w, frame_t)),
        origin=Origin(xyz=(end_x, 0.0, zc)),
        material="frame",
        name="end_edge_0",
    )
    part.visual(
        Box((frame_w, width - 2.0 * frame_w, frame_t)),
        origin=Origin(xyz=(-end_x, 0.0, zc)),
        material="frame",
        name="end_edge_1",
    )
    part.visual(
        Box((length - 2.0 * frame_w, width - 2.0 * frame_w, glass_t)),
        origin=Origin(xyz=(0.0, 0.0, zc)),
        material="glass",
        name="glass",
    )
    runner_t = runner_drop + 0.002
    runner_z = (0.002 - runner_drop) / 2.0
    runner_x = runner_sign * (length / 2.0 - 0.070)
    runner_y = width / 2.0 - 0.014
    for index, y_sign in enumerate((1.0, -1.0)):
        part.visual(
            Box((0.100, 0.028, runner_t)),
            origin=Origin(xyz=(runner_x, y_sign * runner_y, runner_z)),
            material="gasket",
            name=f"runner_{index}",
        )
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_display_freezer")

    model.material("cabinet_white", rgba=(0.94, 0.95, 0.97, 1.0))
    model.material("liner", rgba=(0.82, 0.85, 0.89, 1.0))
    model.material("trim", rgba=(0.44, 0.48, 0.52, 1.0))
    model.material("frame", rgba=(0.70, 0.74, 0.78, 1.0))
    model.material("handle", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("glass", rgba=(0.75, 0.88, 0.96, 0.35))
    model.material("gasket", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("lock", rgba=(0.82, 0.79, 0.67, 1.0))

    cabinet_len = 1.86
    cabinet_w = 0.92
    cabinet_h = 0.92
    plinth_h = 0.09
    plinth_len = 1.58
    plinth_w = 0.68
    floor_t = 0.06
    wall_t = 0.055
    rim_h = 0.035
    rim_band = 0.090

    opening_len = cabinet_len - 2.0 * rim_band
    opening_w = cabinet_w - 2.0 * rim_band

    rail_w = 0.040
    rail_h = 0.014
    rail_z = cabinet_h + rail_h / 2.0
    rail_y = opening_w / 2.0 - rail_w / 2.0

    lid_len = 0.96
    lid_w = opening_w - 0.018
    lid_frame_w = 0.038
    lid_frame_t = 0.018
    glass_t = 0.006
    lid_closed_x = 0.36
    lid_0_z = cabinet_h + 0.016
    lid_1_z = cabinet_h + 0.036
    lid_travel = 0.42

    service_end_x = cabinet_len / 2.0
    lock_y = 0.0
    lock_z = 0.66
    hinge_z = 0.71

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((plinth_len, plinth_w, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h / 2.0)),
        material="trim",
        name="plinth",
    )
    cabinet.visual(
        Box((cabinet_len, cabinet_w, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + floor_t / 2.0)),
        material="cabinet_white",
        name="floor",
    )

    wall_h = cabinet_h - rim_h - (plinth_h + floor_t)
    wall_z = plinth_h + floor_t + wall_h / 2.0
    cabinet.visual(
        Box((cabinet_len, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, cabinet_w / 2.0 - wall_t / 2.0, wall_z)),
        material="cabinet_white",
        name="front_wall",
    )
    cabinet.visual(
        Box((cabinet_len, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -cabinet_w / 2.0 + wall_t / 2.0, wall_z)),
        material="cabinet_white",
        name="rear_wall",
    )
    end_wall_w = cabinet_w - 2.0 * wall_t
    cabinet.visual(
        Box((wall_t, end_wall_w, wall_h)),
        origin=Origin(xyz=(-cabinet_len / 2.0 + wall_t / 2.0, 0.0, wall_z)),
        material="cabinet_white",
        name="end_wall_0",
    )
    cabinet.visual(
        Box((wall_t, end_wall_w, wall_h)),
        origin=Origin(xyz=(service_end_x - wall_t / 2.0, 0.0, wall_z)),
        material="cabinet_white",
        name="service_end_wall",
    )

    rim_z = cabinet_h - rim_h / 2.0
    cabinet.visual(
        Box((cabinet_len, rim_band, rim_h)),
        origin=Origin(xyz=(0.0, cabinet_w / 2.0 - rim_band / 2.0, rim_z)),
        material="liner",
        name="rim_front",
    )
    cabinet.visual(
        Box((cabinet_len, rim_band, rim_h)),
        origin=Origin(xyz=(0.0, -cabinet_w / 2.0 + rim_band / 2.0, rim_z)),
        material="liner",
        name="rim_rear",
    )
    cabinet.visual(
        Box((rim_band, opening_w, rim_h)),
        origin=Origin(xyz=(-cabinet_len / 2.0 + rim_band / 2.0, 0.0, rim_z)),
        material="liner",
        name="rim_end_0",
    )
    cabinet.visual(
        Box((rim_band, opening_w, rim_h)),
        origin=Origin(xyz=(cabinet_len / 2.0 - rim_band / 2.0, 0.0, rim_z)),
        material="liner",
        name="rim_end_1",
    )

    cabinet.visual(
        Box((opening_len, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, rail_y, rail_z)),
        material="frame",
        name="rail_front",
    )
    cabinet.visual(
        Box((opening_len, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, -rail_y, rail_z)),
        material="frame",
        name="rail_rear",
    )

    cabinet.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(service_end_x + 0.004, lock_y, lock_z),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material="lock",
        name="key_cylinder",
    )
    cabinet.visual(
        Box((0.004, 0.096, 0.016)),
        origin=Origin(xyz=(service_end_x + 0.002, 0.0, hinge_z)),
        material="trim",
        name="flap_mount",
    )

    lid_0 = model.part("lid_0")
    _add_lid(
        lid_0,
        length=lid_len,
        width=lid_w,
        frame_w=lid_frame_w,
        frame_t=lid_frame_t,
        glass_t=glass_t,
        runner_sign=-1.0,
        runner_drop=0.002,
    )

    lid_1 = model.part("lid_1")
    _add_lid(
        lid_1,
        length=lid_len,
        width=lid_w,
        frame_w=lid_frame_w,
        frame_t=lid_frame_t,
        glass_t=glass_t,
        runner_sign=1.0,
        runner_drop=0.022,
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material="trim",
        name="hinge_barrel",
    )
    lock_flap.visual(
        Box((0.008, 0.088, 0.058)),
        origin=Origin(xyz=(0.004, 0.0, -0.029)),
        material="trim",
        name="plate",
    )
    lock_flap.visual(
        Box((0.003, 0.050, 0.028)),
        origin=Origin(xyz=(0.0085, 0.0, -0.032)),
        material="gasket",
        name="skirt",
    )

    model.articulation(
        "cabinet_to_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_0,
        origin=Origin(xyz=(-lid_closed_x, 0.0, lid_0_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=0.0,
            upper=lid_travel,
        ),
    )
    model.articulation(
        "cabinet_to_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_1,
        origin=Origin(xyz=(lid_closed_x, 0.0, lid_1_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=0.0,
            upper=lid_travel,
        ),
    )
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(service_end_x + 0.010, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    lock_flap = object_model.get_part("lock_flap")

    lid_0_slide = object_model.get_articulation("cabinet_to_lid_0")
    lid_1_slide = object_model.get_articulation("cabinet_to_lid_1")
    flap_hinge = object_model.get_articulation("cabinet_to_lock_flap")

    with ctx.pose({lid_0_slide: 0.0, lid_1_slide: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            lid_0,
            cabinet,
            axis="z",
            positive_elem="front_edge",
            negative_elem="rail_front",
            min_gap=0.0,
            max_gap=0.010,
            name="lid_0 sits on the front rail plane",
        )
        ctx.expect_gap(
            lid_0,
            cabinet,
            axis="z",
            positive_elem="rear_edge",
            negative_elem="rail_rear",
            min_gap=0.0,
            max_gap=0.010,
            name="lid_0 sits on the rear rail plane",
        )
        ctx.expect_gap(
            lid_1,
            cabinet,
            axis="z",
            positive_elem="front_edge",
            negative_elem="rail_front",
            min_gap=0.012,
            max_gap=0.028,
            name="lid_1 rides on the raised track above lid_0",
        )
        ctx.expect_gap(
            lock_flap,
            cabinet,
            axis="x",
            positive_elem="plate",
            negative_elem="service_end_wall",
            min_gap=0.0,
            max_gap=0.012,
            name="lock flap rests close over the service end wall",
        )

    lid_0_rest = ctx.part_world_position(lid_0)
    lid_1_rest = ctx.part_world_position(lid_1)
    flap_closed_aabb = ctx.part_element_world_aabb(lock_flap, elem="plate")

    lid_0_upper = lid_0_slide.motion_limits.upper if lid_0_slide.motion_limits else None
    lid_1_upper = lid_1_slide.motion_limits.upper if lid_1_slide.motion_limits else None
    flap_upper = flap_hinge.motion_limits.upper if flap_hinge.motion_limits else None

    if lid_0_upper is not None:
        with ctx.pose({lid_0_slide: lid_0_upper}):
            ctx.expect_overlap(
                lid_0,
                cabinet,
                axes="x",
                elem_a="front_edge",
                elem_b="rail_front",
                min_overlap=0.38,
                name="lid_0 remains captured by the front rail at full travel",
            )
            lid_0_open = ctx.part_world_position(lid_0)
        ctx.check(
            "lid_0 slides toward the cabinet center",
            lid_0_rest is not None
            and lid_0_open is not None
            and lid_0_open[0] > lid_0_rest[0] + 0.20,
            details=f"rest={lid_0_rest}, open={lid_0_open}",
        )

    if lid_1_upper is not None:
        with ctx.pose({lid_1_slide: lid_1_upper}):
            ctx.expect_overlap(
                lid_1,
                cabinet,
                axes="x",
                elem_a="rear_edge",
                elem_b="rail_rear",
                min_overlap=0.38,
                name="lid_1 remains captured by the rear rail at full travel",
            )
            lid_1_open = ctx.part_world_position(lid_1)
        ctx.check(
            "lid_1 slides toward the cabinet center",
            lid_1_rest is not None
            and lid_1_open is not None
            and lid_1_open[0] < lid_1_rest[0] - 0.20,
            details=f"rest={lid_1_rest}, open={lid_1_open}",
        )

    if flap_upper is not None:
        with ctx.pose({flap_hinge: flap_upper}):
            flap_open_aabb = ctx.part_element_world_aabb(lock_flap, elem="plate")
        ctx.check(
            "lock flap swings outward from the side wall",
            flap_closed_aabb is not None
            and flap_open_aabb is not None
            and flap_open_aabb[1][0] > flap_closed_aabb[1][0] + 0.030,
            details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
        )
        ctx.check(
            "lock flap clears upward when opened",
            flap_closed_aabb is not None
            and flap_open_aabb is not None
            and flap_open_aabb[0][2] > flap_closed_aabb[0][2] + 0.030,
            details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
