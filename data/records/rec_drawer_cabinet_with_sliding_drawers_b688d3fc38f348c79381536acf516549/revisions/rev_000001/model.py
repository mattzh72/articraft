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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twelve_tray_watch_cabinet")

    wood = model.material("warm_walnut", rgba=(0.34, 0.17, 0.07, 1.0))
    dark_wood = model.material("dark_endgrain", rgba=(0.20, 0.095, 0.035, 1.0))
    felt = model.material("green_felt", rgba=(0.02, 0.22, 0.12, 1.0))
    velvet = model.material("soft_watch_cushion", rgba=(0.07, 0.30, 0.17, 1.0))
    brass = model.material("brass_pull", rgba=(0.95, 0.65, 0.20, 1.0))
    label = model.material("aged_label_card", rgba=(0.90, 0.82, 0.62, 1.0))

    width = 0.260
    depth = 0.340
    height = 0.920
    side_t = 0.018
    top_t = 0.024
    back_t = 0.018
    pitch = 0.068
    first_z = 0.086
    travel = 0.220

    cabinet = model.part("cabinet")

    # Narrow wooden carcass, open at the front with a rear back panel.
    cabinet.visual(
        Box((depth, side_t, height)),
        origin=Origin(xyz=(-depth / 2.0, width / 2.0 - side_t / 2.0, height / 2.0)),
        material=wood,
        name="side_wall_0",
    )
    cabinet.visual(
        Box((depth, side_t, height)),
        origin=Origin(xyz=(-depth / 2.0, -width / 2.0 + side_t / 2.0, height / 2.0)),
        material=wood,
        name="side_wall_1",
    )
    cabinet.visual(
        Box((depth, width, top_t)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, height - top_t / 2.0)),
        material=wood,
        name="top_slab",
    )
    cabinet.visual(
        Box((depth, width, top_t)),
        origin=Origin(xyz=(-depth / 2.0, 0.0, top_t / 2.0)),
        material=wood,
        name="bottom_slab",
    )
    cabinet.visual(
        Box((back_t, width, height)),
        origin=Origin(xyz=(-depth + back_t / 2.0, 0.0, height / 2.0)),
        material=dark_wood,
        name="rear_panel",
    )

    # Front rails and stiles frame the column of tray fronts.
    front_t = 0.016
    cabinet.visual(
        Box((front_t, side_t, height)),
        origin=Origin(xyz=(front_t / 2.0 - 0.004, width / 2.0 - side_t / 2.0, height / 2.0)),
        material=wood,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((front_t, side_t, height)),
        origin=Origin(xyz=(front_t / 2.0 - 0.004, -width / 2.0 + side_t / 2.0, height / 2.0)),
        material=wood,
        name="front_stile_1",
    )
    cabinet.visual(
        Box((front_t, width, top_t)),
        origin=Origin(xyz=(front_t / 2.0 - 0.004, 0.0, height - top_t / 2.0)),
        material=wood,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((front_t, width, top_t)),
        origin=Origin(xyz=(front_t / 2.0 - 0.004, 0.0, top_t / 2.0)),
        material=wood,
        name="front_bottom_rail",
    )

    # Felt-lined upper and lower guide channels for every tray.
    guide_len = 0.285
    guide_x = -0.168
    guide_y = 0.106
    guide_w = 0.012
    guide_z = 0.006
    for i in range(12):
        zc = first_z + i * pitch
        for side, y in enumerate((guide_y, -guide_y)):
            cabinet.visual(
                Box((guide_len, guide_w, guide_z)),
                origin=Origin(xyz=(guide_x, y, zc - 0.030)),
                material=felt,
                name=f"guide_lower_{i}_{side}",
            )
            cabinet.visual(
                Box((guide_len, guide_w, guide_z)),
                origin=Origin(xyz=(guide_x, y, zc + 0.023)),
                material=felt,
                name=f"guide_upper_{i}_{side}",
            )

    for i in range(12):
        zc = first_z + i * pitch
        tray = model.part(f"tray_{i}")

        # The drawer part frame is the front seating plane at the center of the
        # tray face. The tray body reaches rearward into the cabinet at q=0.
        tray.visual(
            Box((0.016, 0.216, 0.058)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=wood,
            name="front_panel",
        )
        tray.visual(
            Box((0.290, 0.186, 0.006)),
            origin=Origin(xyz=(-0.155, 0.0, -0.019)),
            material=dark_wood,
            name="tray_bottom",
        )
        tray.visual(
            Box((0.290, 0.006, 0.034)),
            origin=Origin(xyz=(-0.155, 0.091, -0.002)),
            material=wood,
            name="side_lip_0",
        )
        tray.visual(
            Box((0.290, 0.006, 0.034)),
            origin=Origin(xyz=(-0.155, -0.091, -0.002)),
            material=wood,
            name="side_lip_1",
        )
        tray.visual(
            Box((0.007, 0.186, 0.034)),
            origin=Origin(xyz=(-0.3025, 0.0, -0.002)),
            material=wood,
            name="rear_lip",
        )
        tray.visual(
            Box((0.008, 0.186, 0.034)),
            origin=Origin(xyz=(-0.012, 0.0, -0.002)),
            material=wood,
            name="front_lip",
        )
        tray.visual(
            Box((0.255, 0.150, 0.011)),
            origin=Origin(xyz=(-0.155, 0.0, -0.011)),
            material=velvet,
            name="cushion_pad",
        )

        tray.visual(
            Box((0.270, 0.010, 0.008)),
            origin=Origin(xyz=(-0.155, 0.099, -0.023)),
            material=dark_wood,
            name="runner_0",
        )
        tray.visual(
            Box((0.270, 0.010, 0.008)),
            origin=Origin(xyz=(-0.155, -0.099, -0.023)),
            material=dark_wood,
            name="runner_1",
        )

        # A small brass pull and label frame make the shallow watchmaker trays
        # readable without adding extra moving mechanisms.
        tray.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(xyz=(0.015, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name="round_pull",
        )
        tray.visual(
            Box((0.003, 0.064, 0.018)),
            origin=Origin(xyz=(0.0095, 0.0, 0.014)),
            material=brass,
            name="label_frame",
        )
        tray.visual(
            Box((0.003, 0.049, 0.011)),
            origin=Origin(xyz=(0.0125, 0.0, 0.014)),
            material=label,
            name="label_card",
        )

        model.articulation(
            f"cabinet_to_tray_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=tray,
            origin=Origin(xyz=(0.0, 0.0, zc)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=travel),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    ctx.check(
        "twelve independent trays",
        all(object_model.get_part(f"tray_{i}") is not None for i in range(12)),
        details="Expected tray_0 through tray_11.",
    )
    ctx.check(
        "twelve prismatic slides",
        all(
            object_model.get_articulation(f"cabinet_to_tray_{i}").articulation_type
            == ArticulationType.PRISMATIC
            for i in range(12)
        ),
        details="Every cushioned tray should slide prismatically.",
    )

    for i in (0, 5, 11):
        tray = object_model.get_part(f"tray_{i}")
        slide = object_model.get_articulation(f"cabinet_to_tray_{i}")
        ctx.expect_overlap(
            tray,
            cabinet,
            axes="xy",
            min_overlap=0.003,
            elem_a="runner_0",
            elem_b=f"guide_lower_{i}_0",
            name=f"tray_{i} runner sits over felt guide",
        )
        ctx.expect_gap(
            tray,
            cabinet,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="runner_0",
            negative_elem=f"guide_lower_{i}_0",
            name=f"tray_{i} runner rests on lower felt",
        )
        rest_pos = ctx.part_world_position(tray)
        with ctx.pose({slide: 0.220}):
            ctx.expect_overlap(
                tray,
                cabinet,
                axes="x",
                min_overlap=0.040,
                elem_a="runner_0",
                elem_b=f"guide_lower_{i}_0",
                name=f"tray_{i} stays retained when extended",
            )
            extended_pos = ctx.part_world_position(tray)
        ctx.check(
            f"tray_{i} extends forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
