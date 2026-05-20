from __future__ import annotations

import math

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


HEIGHT = 1.55
WIDTH = 0.60
FRONT_FOOT_X = -0.35
REAR_FOOT_X = 0.65
SIDE_Y = WIDTH / 2.0
HINGE_Z = HEIGHT


def _cyl_y(length: float, radius: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_aframe_step_ladder")

    model.material("powdercoat", rgba=(0.18, 0.24, 0.23, 1.0))
    model.material("tread_gray", rgba=(0.58, 0.64, 0.63, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.025, 0.022, 1.0))
    model.material("stainless", rgba=(0.78, 0.78, 0.72, 1.0))
    model.material("seal_dark", rgba=(0.015, 0.018, 0.016, 1.0))
    model.material("warning_orange", rgba=(0.95, 0.42, 0.08, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")
    braces = [model.part("brace_0"), model.part("brace_1")]

    # Front climbing frame: paired sealed rectangular-tube legs tied together by
    # overhanging treads and a weather-shedding top platform.
    front_len = math.hypot(-FRONT_FOOT_X, HEIGHT)
    front_theta = math.atan2(-FRONT_FOOT_X, HEIGHT)
    for idx, y in enumerate((-SIDE_Y, SIDE_Y)):
        front.visual(
            Box((0.055, 0.055, front_len)),
            origin=Origin(
                xyz=(FRONT_FOOT_X / 2.0, y, HEIGHT / 2.0),
                rpy=(0.0, front_theta, 0.0),
            ),
            material="powdercoat",
            name=f"front_leg_{idx}",
        )
        front.visual(
            Box((0.19, 0.12, 0.045)),
            origin=Origin(xyz=(FRONT_FOOT_X - 0.015, y, 0.023)),
            material="rubber_black",
            name=f"front_foot_{idx}",
        )
        front.visual(
            Box((0.090, 0.020, 0.105)),
            origin=Origin(xyz=(-0.015, y, HEIGHT - 0.065)),
            material="stainless",
            name=f"hinge_cheek_{idx}",
        )

    tread_specs = [
        (0.32, 0.31, "step_0"),
        (0.66, 0.32, "step_1"),
        (1.00, 0.34, "step_2"),
    ]
    for z, depth, label in tread_specs:
        x_on_leg = FRONT_FOOT_X + (-FRONT_FOOT_X) * (z / HEIGHT)
        x = x_on_leg - 0.055
        front.visual(
            Box((depth, 0.70, 0.046)),
            origin=Origin(xyz=(x, 0.0, z)),
            material="tread_gray",
            name=label,
        )
        front.visual(
            Box((0.026, 0.72, 0.034)),
            origin=Origin(xyz=(x - depth / 2.0 + 0.010, 0.0, z - 0.036)),
            material="powdercoat",
            name=f"{label}_drip_lip",
        )
        for rib_i, dx in enumerate((-0.09, 0.0, 0.09)):
            front.visual(
                Box((0.014, 0.62, 0.007)),
                origin=Origin(xyz=(x + dx, 0.0, z + 0.025)),
                material="seal_dark",
                name=f"{label}_grip_{rib_i}",
            )

    front.visual(
        Box((0.44, 0.58, 0.060)),
        origin=Origin(xyz=(-0.020, 0.0, 1.345)),
        material="tread_gray",
        name="top_platform",
    )
    front.visual(
        Box((0.50, 0.62, 0.024)),
        origin=Origin(xyz=(-0.035, 0.0, 1.395)),
        material="powdercoat",
        name="top_drip_cap",
    )
    front.visual(
        Box((0.53, 0.025, 0.018)),
        origin=Origin(xyz=(-0.035, -0.300, 1.375)),
        material="seal_dark",
        name="side_gasket_0",
    )
    front.visual(
        Box((0.53, 0.025, 0.018)),
        origin=Origin(xyz=(-0.035, 0.300, 1.375)),
        material="seal_dark",
        name="side_gasket_1",
    )
    front.visual(
        _cyl_y(0.84, 0.020),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="stainless",
        name="top_pin",
    )

    # Front-side spread-brace pivot tabs and stainless shoulder pins.
    brace_front = (-0.225, 0.0, 0.700)
    for idx, y in enumerate((-0.385, 0.385)):
        tab_y = y - math.copysign(0.050, y)
        front.visual(
            Box((0.105, 0.038, 0.084)),
            origin=Origin(xyz=(brace_front[0] + 0.015, tab_y, brace_front[2])),
            material="powdercoat",
            name=f"brace_tab_{idx}",
        )
        front.visual(
            _cyl_y(0.070, 0.018),
            origin=Origin(
                xyz=(brace_front[0], y, brace_front[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="stainless",
            name=f"brace_pin_{idx}",
        )
        front.visual(
            Box((0.030, 0.074, 0.030)),
            origin=Origin(xyz=(brace_front[0] - 0.050, (tab_y + y) / 2.0, brace_front[2] - 0.044)),
            material="warning_orange",
            name=f"open_stop_{idx}",
        )

    # Rear support frame in the child frame whose origin is the top pivot.
    rear_vec = (REAR_FOOT_X, 0.0, -HEIGHT)
    rear_len = math.hypot(REAR_FOOT_X, HEIGHT)
    rear_theta = math.atan2(REAR_FOOT_X, -HEIGHT)
    rear_top_t = 0.17
    rear_center_t = (1.0 + rear_top_t) / 2.0
    rear_tube_len = rear_len * (1.0 - rear_top_t)
    for idx, y in enumerate((-SIDE_Y, SIDE_Y)):
        hinge_y = y + math.copysign(0.050, y)
        rear.visual(
            Box((0.052, 0.052, rear_tube_len)),
            origin=Origin(
                xyz=(rear_vec[0] * rear_center_t, y, rear_vec[2] * rear_center_t),
                rpy=(0.0, rear_theta, 0.0),
            ),
            material="powdercoat",
            name=f"rear_leg_{idx}",
        )
        rear.visual(
            Box((0.20, 0.12, 0.045)),
            origin=Origin(xyz=(REAR_FOOT_X + 0.010, y, -HEIGHT + 0.023)),
            material="rubber_black",
            name=f"rear_foot_{idx}",
        )
        rear.visual(
            Box((0.090, 0.050, 0.270)),
            origin=Origin(xyz=(0.065, hinge_y, -0.135)),
            material="powdercoat",
            name=f"hinge_strap_{idx}",
        )
        rear.visual(
            _cyl_y(0.100, 0.032),
            origin=Origin(xyz=(0.0, hinge_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="stainless",
            name=f"hinge_knuckle_{idx}",
        )
    for i, t in enumerate((0.38, 0.72)):
        rear.visual(
            Box((0.074, 0.68, 0.052)),
            origin=Origin(xyz=(rear_vec[0] * t, 0.0, rear_vec[2] * t)),
            material="powdercoat",
            name=f"rear_crossbar_{i}",
        )
        rear.visual(
            Box((0.045, 0.62, 0.012)),
            origin=Origin(xyz=(rear_vec[0] * t - 0.020, 0.0, rear_vec[2] * t + 0.034)),
            material="seal_dark",
            name=f"crossbar_seal_{i}",
        )

    # Rear catches that the spread-limit braces physically bear against at the
    # open stop.
    brace_rear = (0.425, 0.0, -1.000)
    for idx, (y, peg_name) in enumerate(((-0.385, "brace_peg_0"), (0.385, "brace_peg_1"))):
        lug_y = y - math.copysign(0.042, y)
        rear.visual(
            Box((0.085, 0.036, 0.075)),
            origin=Origin(xyz=(brace_rear[0] - 0.015, lug_y, brace_rear[2])),
            material="powdercoat",
            name=f"rear_lug_{idx}",
        )
        rear.visual(
            _cyl_y(0.072, 0.018),
            origin=Origin(
                xyz=(brace_rear[0], y, brace_rear[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="stainless",
            name=peg_name,
        )

    # Paired folding spread-limit braces.  Each brace is hinged to the front
    # frame and has a sealed U-catch that lands on the corresponding rear peg.
    brace_dx = brace_rear[0] - brace_front[0]
    brace_dz = brace_rear[2] + HINGE_Z - brace_front[2]
    # brace_rear is in rear-frame coordinates, so convert its rest world z.
    brace_dz = 0.550 - brace_front[2]
    brace_len = math.hypot(brace_dx, brace_dz)
    brace_theta = math.asin(-brace_dz / brace_len)
    brace_bar_dx = brace_dx - 0.070
    brace_bar_dz = brace_dz * (brace_bar_dx / brace_dx)
    brace_bar_len = math.hypot(brace_bar_dx, brace_bar_dz)
    for idx, (brace, y) in enumerate(zip(braces, (-0.385, 0.385))):
        brace.visual(
            Box((brace_bar_len, 0.028, 0.028)),
            origin=Origin(
                xyz=(brace_bar_dx / 2.0, 0.0, brace_bar_dz / 2.0),
                rpy=(0.0, brace_theta, 0.0),
            ),
            material="stainless",
            name="brace_bar",
        )
        brace.visual(
            _cyl_y(0.060, 0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="stainless",
            name="pivot_bushing",
        )
        brace.visual(
            Box((0.090, 0.034, 0.012)),
            origin=Origin(xyz=(brace_dx + 0.015, 0.0, brace_dz + 0.032)),
            material="stainless",
            name="catch_leaf_upper",
        )
        brace.visual(
            Box((0.090, 0.034, 0.012)),
            origin=Origin(xyz=(brace_dx + 0.015, 0.0, brace_dz - 0.032)),
            material="stainless",
            name="catch_leaf_lower",
        )
        brace.visual(
            Box((0.014, 0.034, 0.076)),
            origin=Origin(xyz=(brace_dx + 0.055, 0.0, brace_dz)),
            material="stainless",
            name="catch_bridge",
        )
        brace.visual(
            Box((0.020, 0.032, 0.080)),
            origin=Origin(xyz=(brace_dx - 0.025, 0.0, brace_dz)),
            material="stainless",
            name="catch_web",
        )
        brace.visual(
            Box((0.070, 0.042, 0.010)),
            origin=Origin(xyz=(brace_dx - 0.053, 0.0, brace_dz)),
            material="seal_dark",
            name="peg_gasket",
        )

    rear_pivot = model.articulation(
        "rear_pivot",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=0.35),
    )
    rear_pivot.meta["stable_states"] = {"open": 0.0, "closed": 0.35}

    for idx, (brace, y) in enumerate(zip(braces, (-0.385, 0.385))):
        joint = model.articulation(
            f"brace_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(brace_front[0], y, brace_front[2])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.05),
        )
        joint.meta["stable_states"] = {"open_locked": 0.0, "folded": 1.05}

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    brace_0 = object_model.get_part("brace_0")
    brace_1 = object_model.get_part("brace_1")
    rear_pivot = object_model.get_articulation("rear_pivot")
    brace_pivot_0 = object_model.get_articulation("brace_pivot_0")
    brace_pivot_1 = object_model.get_articulation("brace_pivot_1")

    for idx in (0, 1):
        ctx.allow_overlap(
            front,
            rear,
            elem_a="top_pin",
            elem_b=f"hinge_knuckle_{idx}",
            reason="The stainless top pin is intentionally captured inside the protected rear hinge knuckle.",
        )
        ctx.expect_overlap(
            rear,
            front,
            axes="y",
            elem_a=f"hinge_knuckle_{idx}",
            elem_b="top_pin",
            min_overlap=0.080,
            name=f"top hinge knuckle {idx} is retained on the pin",
        )
        ctx.allow_overlap(
            front,
            rear,
            elem_a=f"front_leg_{idx}",
            elem_b=f"hinge_knuckle_{idx}",
            reason="The weather-shielded hinge barrel is locally seated against the capped side rail at the top pivot.",
        )
        ctx.expect_overlap(
            rear,
            front,
            axes="z",
            elem_a=f"hinge_knuckle_{idx}",
            elem_b=f"front_leg_{idx}",
            min_overlap=0.020,
            name=f"top hinge barrel {idx} is seated by the side rail",
        )
        ctx.allow_overlap(
            front,
            rear,
            elem_a=f"hinge_cheek_{idx}",
            elem_b=f"hinge_knuckle_{idx}",
            reason="The stainless knuckle is locally nested between protective hinge cheeks under the drip cap.",
        )
        ctx.expect_overlap(
            rear,
            front,
            axes="z",
            elem_a=f"hinge_knuckle_{idx}",
            elem_b=f"hinge_cheek_{idx}",
            min_overlap=0.015,
            name=f"hinge cheek {idx} protects the knuckle",
        )

    for idx, brace in enumerate((brace_0, brace_1)):
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_pin_{idx}",
            elem_b="pivot_bushing",
            reason="The spread-brace bushing is intentionally captured on a stainless shoulder pin.",
        )
        ctx.expect_overlap(
            brace,
            front,
            axes="y",
            elem_a="pivot_bushing",
            elem_b=f"brace_pin_{idx}",
            min_overlap=0.050,
            name=f"brace {idx} bushing is retained on its shoulder pin",
        )
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_pin_{idx}",
            elem_b="brace_bar",
            reason="The folding spread-brace eye is represented as wrapping locally around the same sealed shoulder pin.",
        )
        ctx.expect_overlap(
            brace,
            front,
            axes="y",
            elem_a="brace_bar",
            elem_b=f"brace_pin_{idx}",
            min_overlap=0.020,
            name=f"brace {idx} pivot eye is retained laterally",
        )

    with ctx.pose({rear_pivot: 0.0, brace_pivot_0: 0.0, brace_pivot_1: 0.0}):
        open_rear_aabb = ctx.part_world_aabb(rear)
        open_brace_aabb = ctx.part_world_aabb(brace_0)
        ctx.expect_contact(
            brace_0,
            rear,
            elem_a="peg_gasket",
            elem_b="brace_peg_0",
            contact_tol=0.006,
            name="open brace bears on rear spread peg",
        )
        ctx.expect_contact(
            brace_1,
            rear,
            elem_a="peg_gasket",
            elem_b="brace_peg_1",
            contact_tol=0.006,
            name="opposite brace bears on rear spread peg",
        )

    with ctx.pose({rear_pivot: 0.35, brace_pivot_0: 1.05, brace_pivot_1: 1.05}):
        closed_rear_aabb = ctx.part_world_aabb(rear)
        folded_brace_aabb = ctx.part_world_aabb(brace_0)

    ctx.check(
        "rear frame folds toward front frame",
        open_rear_aabb is not None
        and closed_rear_aabb is not None
        and closed_rear_aabb[1][0] < open_rear_aabb[1][0] - 0.18,
        details=f"open={open_rear_aabb}, closed={closed_rear_aabb}",
    )
    ctx.check(
        "spread brace folds upward for storage",
        open_brace_aabb is not None
        and folded_brace_aabb is not None
        and folded_brace_aabb[1][2] > open_brace_aabb[1][2] + 0.40,
        details=f"open={open_brace_aabb}, folded={folded_brace_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
