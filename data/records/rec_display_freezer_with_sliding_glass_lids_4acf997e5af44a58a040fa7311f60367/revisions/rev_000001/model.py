from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_W = 3.10
CABINET_D = 1.25
CABINET_H = 0.95
WALL_T = 0.10
BOTTOM_T = 0.12

LID_W = 1.12
LID_D = 1.10
LID_RAIL = 0.055
LID_X = (-0.72, 0.0, 0.72)
LID_TRACK_Z = (1.005, 1.075, 1.145)


def _cabinet_shell():
    """One-piece insulated tub with a real open top cavity."""
    outer = cq.Workplane("XY").box(CABINET_W, CABINET_D, CABINET_H).translate(
        (0.0, 0.0, CABINET_H / 2.0)
    )
    inner = cq.Workplane("XY").box(
        CABINET_W - 2.0 * WALL_T,
        CABINET_D - 2.0 * WALL_T,
        CABINET_H,
    ).translate((0.0, 0.0, BOTTOM_T + CABINET_H / 2.0))
    return outer.cut(inner)


def _add_lid_visuals(lid, *, glass, aluminum, gasket):
    """Layered glass sliding lid; local z=0 is the shoe/track contact plane."""
    glass_w = LID_W - 2.0 * LID_RAIL + 0.018
    glass_d = LID_D - 2.0 * LID_RAIL + 0.018
    half_x = LID_W / 2.0 - LID_RAIL / 2.0
    half_y = LID_D / 2.0 - LID_RAIL / 2.0

    lid.visual(
        Box((glass_w, glass_d, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        Box((LID_W, LID_RAIL, 0.036)),
        origin=Origin(xyz=(0.0, -half_y, 0.026)),
        material=aluminum,
        name="front_frame",
    )
    lid.visual(
        Box((LID_W, LID_RAIL, 0.036)),
        origin=Origin(xyz=(0.0, half_y, 0.026)),
        material=aluminum,
        name="rear_frame",
    )
    lid.visual(
        Box((LID_RAIL, LID_D - 2.0 * LID_RAIL, 0.036)),
        origin=Origin(xyz=(-half_x, 0.0, 0.026)),
        material=aluminum,
        name="side_frame_0",
    )
    lid.visual(
        Box((LID_RAIL, LID_D - 2.0 * LID_RAIL, 0.036)),
        origin=Origin(xyz=(half_x, 0.0, 0.026)),
        material=aluminum,
        name="side_frame_1",
    )
    # Low-profile pull rail bonded to the glass/frame so the lid reads as a
    # merchandiser slider rather than a plain sheet.
    lid.visual(
        Box((0.56, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.31, 0.040)),
        material=gasket,
        name="pull_rail",
    )
    # Shoes ride on the supported front and rear tracks.  They also tie the
    # child part to the actual support geometry at the rest pose.
    lid.visual(
        Box((LID_W - 0.12, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.515, 0.006)),
        material=gasket,
        name="front_shoe",
    )
    lid.visual(
        Box((LID_W - 0.12, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.515, 0.006)),
        material=gasket,
        name="rear_shoe",
    )


def _add_basket_visuals(basket, *, wire):
    """Open hanging wire basket; local z=0 is the hanger support plane."""
    w = 0.82
    d = 0.86
    h = 0.48
    rod = 0.018
    bottom_z = -h
    rim_z = -0.045
    rail_y = 0.47
    vertical_h = h + 0.030
    vertical_z = (bottom_z + 0.030) / 2.0

    # Hangers sit on cabinet rails and connect into the top rim.
    basket.visual(
        Box((w + 0.08, rod, 0.020)),
        origin=Origin(xyz=(0.0, -rail_y, 0.010)),
        material=wire,
        name="front_hanger",
    )
    basket.visual(
        Box((w + 0.08, rod, 0.020)),
        origin=Origin(xyz=(0.0, rail_y, 0.010)),
        material=wire,
        name="rear_hanger",
    )
    basket.visual(
        Box((rod, 0.080, 0.020)),
        origin=Origin(xyz=(-w / 2.0, -0.435, 0.022)),
        material=wire,
        name="front_hook_0",
    )
    basket.visual(
        Box((rod, 0.080, 0.020)),
        origin=Origin(xyz=(w / 2.0, -0.435, 0.022)),
        material=wire,
        name="front_hook_1",
    )
    basket.visual(
        Box((rod, 0.080, 0.020)),
        origin=Origin(xyz=(-w / 2.0, 0.435, 0.022)),
        material=wire,
        name="rear_hook_0",
    )
    basket.visual(
        Box((rod, 0.080, 0.020)),
        origin=Origin(xyz=(w / 2.0, 0.435, 0.022)),
        material=wire,
        name="rear_hook_1",
    )

    # Top rim, vertical sides, and bottom grid are all physically connected.
    basket.visual(
        Box((w, rod, rod)),
        origin=Origin(xyz=(0.0, -d / 2.0, rim_z)),
        material=wire,
        name="front_rim",
    )
    basket.visual(
        Box((w, rod, rod)),
        origin=Origin(xyz=(0.0, d / 2.0, rim_z)),
        material=wire,
        name="rear_rim",
    )
    basket.visual(
        Box((rod, d, rod)),
        origin=Origin(xyz=(-w / 2.0, 0.0, rim_z)),
        material=wire,
        name="side_rim_0",
    )
    basket.visual(
        Box((rod, d, rod)),
        origin=Origin(xyz=(w / 2.0, 0.0, rim_z)),
        material=wire,
        name="side_rim_1",
    )

    for idx, x in enumerate((-w / 2.0, -w / 6.0, w / 6.0, w / 2.0)):
        basket.visual(
            Box((rod, rod, vertical_h)),
            origin=Origin(xyz=(x, -d / 2.0, vertical_z)),
            material=wire,
            name=f"front_wire_{idx}",
        )
        basket.visual(
            Box((rod, rod, vertical_h)),
            origin=Origin(xyz=(x, d / 2.0, vertical_z)),
            material=wire,
            name=f"rear_wire_{idx}",
        )
    for idx, y in enumerate((-d / 2.0, -d / 6.0, d / 6.0, d / 2.0)):
        basket.visual(
            Box((rod, rod, vertical_h)),
            origin=Origin(xyz=(-w / 2.0, y, vertical_z)),
            material=wire,
            name=f"side_wire_0_{idx}",
        )
        basket.visual(
            Box((rod, rod, vertical_h)),
            origin=Origin(xyz=(w / 2.0, y, vertical_z)),
            material=wire,
            name=f"side_wire_1_{idx}",
        )
    basket.visual(
        Box((w, rod, rod)),
        origin=Origin(xyz=(0.0, -d / 2.0, bottom_z)),
        material=wire,
        name="bottom_front",
    )
    basket.visual(
        Box((w, rod, rod)),
        origin=Origin(xyz=(0.0, d / 2.0, bottom_z)),
        material=wire,
        name="bottom_rear",
    )
    basket.visual(
        Box((rod, d, rod)),
        origin=Origin(xyz=(-w / 2.0, 0.0, bottom_z)),
        material=wire,
        name="bottom_side_0",
    )
    basket.visual(
        Box((rod, d, rod)),
        origin=Origin(xyz=(w / 2.0, 0.0, bottom_z)),
        material=wire,
        name="bottom_side_1",
    )
    for idx, x in enumerate((-0.30, -0.10, 0.10, 0.30)):
        basket.visual(
            Box((rod, d, rod)),
            origin=Origin(xyz=(x, 0.0, bottom_z)),
            material=wire,
            name=f"bottom_wire_x_{idx}",
        )
    for idx, y in enumerate((-0.31, -0.10, 0.10, 0.31)):
        basket.visual(
            Box((w, rod, rod)),
            origin=Origin(xyz=(0.0, y, bottom_z)),
            material=wire,
            name=f"bottom_wire_y_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_frozen_food_merchandiser")

    cabinet_mat = model.material("white_insulated_steel", rgba=(0.90, 0.96, 1.0, 1.0))
    blue_mat = model.material("blue_lower_panel", rgba=(0.08, 0.22, 0.52, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    gasket = model.material("black_gasket", rgba=(0.015, 0.018, 0.020, 1.0))
    glass = model.material("cold_glass", rgba=(0.58, 0.86, 1.0, 0.38))
    wire = model.material("zinc_wire", rgba=(0.78, 0.78, 0.72, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "hollow_cabinet", tolerance=0.004),
        material=cabinet_mat,
        name="hollow_tub",
    )
    cabinet.visual(
        Box((CABINET_W + 0.10, CABINET_D + 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=gasket,
        name="kick_plinth",
    )
    cabinet.visual(
        Box((CABINET_W - 0.12, 0.030, 0.50)),
        origin=Origin(xyz=(0.0, -CABINET_D / 2.0 - 0.006, 0.43)),
        material=blue_mat,
        name="front_brand_panel",
    )

    # Stacked, supported top tracks: the three glass lids overlap in plan but
    # each rides on its own vertical channel.
    cabinet.visual(
        Box((CABINET_W + 0.02, 0.040, 0.210)),
        origin=Origin(xyz=(0.0, -0.585, 1.055)),
        material=aluminum,
        name="front_track_wall",
    )
    cabinet.visual(
        Box((CABINET_W + 0.02, 0.040, 0.210)),
        origin=Origin(xyz=(0.0, 0.585, 1.055)),
        material=aluminum,
        name="rear_track_wall",
    )
    for idx, track_z in enumerate(LID_TRACK_Z):
        cabinet.visual(
            Box((CABINET_W - 0.08, 0.080, 0.012)),
            origin=Origin(xyz=(0.0, -0.540, track_z - 0.006)),
            material=aluminum,
            name=f"front_track_{idx}",
        )
        cabinet.visual(
            Box((CABINET_W - 0.08, 0.080, 0.012)),
            origin=Origin(xyz=(0.0, 0.540, track_z - 0.006)),
            material=aluminum,
            name=f"rear_track_{idx}",
        )
        cabinet.visual(
            Box((0.038, 0.140, 0.050)),
            origin=Origin(xyz=(-1.48, -0.540, track_z + 0.019)),
            material=gasket,
            name=f"front_stop_0_{idx}",
        )
        cabinet.visual(
            Box((0.038, 0.140, 0.050)),
            origin=Origin(xyz=(1.48, -0.540, track_z + 0.019)),
            material=gasket,
            name=f"front_stop_1_{idx}",
        )
        cabinet.visual(
            Box((0.038, 0.140, 0.050)),
            origin=Origin(xyz=(-1.48, 0.540, track_z + 0.019)),
            material=gasket,
            name=f"rear_stop_0_{idx}",
        )
        cabinet.visual(
            Box((0.038, 0.140, 0.050)),
            origin=Origin(xyz=(1.48, 0.540, track_z + 0.019)),
            material=gasket,
            name=f"rear_stop_1_{idx}",
        )

    # Internal suspension rails for removable wire baskets.
    cabinet.visual(
        Box((2.95, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.470, 0.7825)),
        material=aluminum,
        name="front_basket_rail",
    )
    cabinet.visual(
        Box((2.95, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.470, 0.7825)),
        material=aluminum,
        name="rear_basket_rail",
    )

    lid_axes = ((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    lid_limits = (
        MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
        MotionLimits(effort=80.0, velocity=0.35, lower=-0.32, upper=0.32),
        MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    for idx, (x, z) in enumerate(zip(LID_X, LID_TRACK_Z)):
        lid = model.part(f"lid_{idx}")
        _add_lid_visuals(lid, glass=glass, aluminum=aluminum, gasket=gasket)
        model.articulation(
            f"cabinet_to_lid_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=lid,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=lid_axes[idx],
            motion_limits=lid_limits[idx],
        )

    for idx, x in enumerate((-0.90, 0.0, 0.90)):
        basket = model.part(f"basket_{idx}")
        _add_basket_visuals(basket, wire=wire)
        model.articulation(
            f"cabinet_to_basket_{idx}",
            ArticulationType.FIXED,
            parent=cabinet,
            child=basket,
            origin=Origin(xyz=(x, 0.0, 0.800)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    for idx in range(3):
        lid = object_model.get_part(f"lid_{idx}")
        joint = object_model.get_articulation(f"cabinet_to_lid_{idx}")
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="front_shoe",
            negative_elem=f"front_track_{idx}",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"lid {idx} front shoe rides on track",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="rear_shoe",
            negative_elem=f"rear_track_{idx}",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"lid {idx} rear shoe rides on track",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="x",
            elem_a="front_shoe",
            elem_b=f"front_track_{idx}",
            min_overlap=0.75,
            name=f"lid {idx} retained on long front rail",
        )
        rest = ctx.part_world_position(lid)
        with ctx.pose({joint: joint.motion_limits.upper}):
            moved = ctx.part_world_position(lid)
            ctx.expect_overlap(
                lid,
                cabinet,
                axes="x",
                elem_a="rear_shoe",
                elem_b=f"rear_track_{idx}",
                min_overlap=0.75,
                name=f"lid {idx} stays captured when slid",
            )
        if idx == 2:
            ctx.check(
                "right lid slides inward",
                rest is not None and moved is not None and moved[0] < rest[0] - 0.30,
                details=f"rest={rest}, moved={moved}",
            )
        else:
            ctx.check(
                f"lid {idx} slides right",
                rest is not None and moved is not None and moved[0] > rest[0] + 0.25,
                details=f"rest={rest}, moved={moved}",
            )

    for idx in range(3):
        basket = object_model.get_part(f"basket_{idx}")
        ctx.expect_gap(
            basket,
            cabinet,
            axis="z",
            positive_elem="front_hanger",
            negative_elem="front_basket_rail",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"basket {idx} front hanger rests on rail",
        )
        ctx.expect_gap(
            basket,
            cabinet,
            axis="z",
            positive_elem="rear_hanger",
            negative_elem="rear_basket_rail",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"basket {idx} rear hanger rests on rail",
        )

    return ctx.report()


object_model = build_object_model()
