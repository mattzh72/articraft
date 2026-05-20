from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_wireless_keyboard")

    body_mat = model.material("anodized_graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    deck_mat = model.material("matte_black_recess", rgba=(0.015, 0.016, 0.018, 1.0))
    key_mat = model.material("charcoal_keycaps", rgba=(0.035, 0.037, 0.042, 1.0))
    key_alt_mat = model.material("warm_gray_keycaps", rgba=(0.060, 0.063, 0.070, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.003, 0.003, 0.004, 1.0))
    metal_mat = model.material("brushed_hinge_pin", rgba=(0.55, 0.56, 0.55, 1.0))

    key_mesh_cache: dict[tuple[float, float, float], object] = {}

    def rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
        return mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(width, depth, radius, corner_segments=8),
                height,
                cap=True,
                center=True,
            ),
            name,
        )

    def keycap_mesh(width: float, depth: float, height: float):
        key = (round(width, 4), round(depth, 4), round(height, 4))
        if key not in key_mesh_cache:
            radius = min(width, depth) * 0.19
            key_mesh_cache[key] = rounded_box_mesh(
                width,
                depth,
                height,
                radius,
                f"keycap_{int(width * 1000):03d}_{int(depth * 1000):03d}_{int(height * 1000):03d}",
            )
        return key_mesh_cache[key]

    def circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
        return [
            (math.cos(2.0 * math.pi * i / segments) * radius, math.sin(2.0 * math.pi * i / segments) * radius)
            for i in range(segments)
        ]

    tray = model.part("tray")

    tray.visual(
        rounded_box_mesh(0.320, 0.195, 0.008, 0.015, "low_tray_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_mat,
        name="tray_shell",
    )
    tray.visual(
        rounded_box_mesh(0.300, 0.132, 0.002, 0.010, "key_deck_inset"),
        origin=Origin(xyz=(0.0, -0.021, 0.009)),
        material=deck_mat,
        name="key_deck",
    )
    tray.visual(
        rounded_box_mesh(0.286, 0.033, 0.0014, 0.004, "stand_recess_floor"),
        origin=Origin(xyz=(0.0, 0.066, 0.0087)),
        material=deck_mat,
        name="stand_recess",
    )
    tray.visual(
        Box((0.320, 0.006, 0.009)),
        origin=Origin(xyz=(0.0, 0.094, 0.0095)),
        material=body_mat,
        name="rear_lip",
    )
    tray.visual(
        Box((0.315, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.093, 0.010)),
        material=body_mat,
        name="front_bevel",
    )
    tray.visual(
        Cylinder(radius=0.00155, length=0.296),
        origin=Origin(xyz=(0.0, 0.085, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )
    tray.visual(
        Cylinder(radius=0.0030, length=0.006),
        origin=Origin(xyz=(-0.151, 0.085, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_head_0",
    )
    tray.visual(
        Box((0.006, 0.006, 0.005)),
        origin=Origin(xyz=(-0.145, 0.085, 0.0105)),
        material=body_mat,
        name="hinge_block_0",
    )
    tray.visual(
        Cylinder(radius=0.0030, length=0.008),
        origin=Origin(xyz=(0.151, 0.085, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_head_1",
    )
    tray.visual(
        Box((0.006, 0.006, 0.005)),
        origin=Origin(xyz=(0.145, 0.085, 0.0105)),
        material=body_mat,
        name="hinge_block_1",
    )

    # Fold-out tablet stand: closed, it lies flat in the shallow rear recess with
    # its part frame sitting directly on the long rear hinge axis.
    stand_flap = model.part("stand_flap")
    stand_flap.visual(
        rounded_box_mesh(0.282, 0.034, 0.004, 0.004, "stand_flap_panel"),
        origin=Origin(xyz=(0.0, -0.017, -0.002)),
        material=body_mat,
        name="flap_panel",
    )
    hinge_sleeve_profile = ExtrudeWithHolesGeometry(
        circle_profile(0.0032, 48),
        [circle_profile(0.00205, 48)],
        0.278,
        cap=True,
        center=True,
        closed=True,
    )
    stand_flap.visual(
        mesh_from_geometry(hinge_sleeve_profile, "stand_hinge_sleeve"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_mat,
        name="hinge_sleeve",
    )
    stand_flap.visual(
        Box((0.230, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, -0.030, 0.0006)),
        material=rubber_mat,
        name="tablet_grip",
    )

    model.articulation(
        "tray_to_stand_flap",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=stand_flap,
        origin=Origin(xyz=(0.0, 0.085, 0.014)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    def add_key(
        part_name: str,
        x: float,
        y: float,
        width: float,
        depth: float,
        *,
        row_index: int,
        material: Material,
        travel: float = 0.003,
    ) -> None:
        key = model.part(part_name)
        cap_height = 0.004 if depth > 0.011 else 0.0035
        key.visual(
            keycap_mesh(width, depth, cap_height),
            origin=Origin(),
            material=material,
            name="cap",
        )
        # A short hidden plunger stem makes each cap read as mounted on a low
        # switch while leaving a hairline gap above the recessed key deck.
        if width > 0.045:
            stem_offsets = (-width * 0.32, width * 0.32)
        else:
            stem_offsets = (0.0,)
        for stem_i, sx in enumerate(stem_offsets):
            plunger_name = "plunger_0" if stem_i == 0 else f"plunger_{stem_i}"
            key.visual(
                Box((min(width * 0.38, 0.007), min(depth * 0.45, 0.006), 0.0020)),
                origin=Origin(xyz=(sx, 0.0, -0.0025)),
                material=rubber_mat,
                name=plunger_name,
            )
        model.articulation(
            f"{part_name}_slide",
            ArticulationType.PRISMATIC,
            parent=tray,
            child=key,
            origin=Origin(xyz=(x, y, 0.0135)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=travel),
        )

    def add_row(
        row_index: int,
        y: float,
        widths: list[float],
        depth: float,
        *,
        gap: float,
        material: Material,
    ) -> None:
        total = sum(widths) + gap * (len(widths) - 1)
        cursor = -total / 2.0
        for col, width in enumerate(widths):
            center_x = cursor + width / 2.0
            add_key(
                f"key_{row_index}_{col}",
                center_x,
                y,
                width,
                depth,
                row_index=row_index,
                material=material,
                travel=0.0025 if row_index == 0 else 0.0030,
            )
            cursor += width + gap

    add_row(0, 0.034, [0.014] * 15, 0.010, gap=0.0040, material=key_alt_mat)
    add_row(1, 0.012, [0.020] + [0.016] * 12 + [0.022], 0.014, gap=0.0025, material=key_mat)
    add_row(2, -0.009, [0.022] + [0.016] * 12 + [0.020], 0.014, gap=0.0025, material=key_mat)
    add_row(3, -0.030, [0.027] + [0.016] * 11 + [0.027], 0.014, gap=0.0025, material=key_mat)
    add_row(4, -0.051, [0.034] + [0.016] * 10 + [0.034], 0.014, gap=0.0025, material=key_mat)
    add_row(5, -0.073, [0.020, 0.020, 0.020, 0.020, 0.090, 0.020, 0.020, 0.020, 0.020], 0.014, gap=0.0025, material=key_alt_mat)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("tray")
    stand_flap = object_model.get_part("stand_flap")
    stand_joint = object_model.get_articulation("tray_to_stand_flap")
    sample_key = object_model.get_part("key_2_5")
    sample_key_joint = object_model.get_articulation("key_2_5_slide")

    key_parts = [p for p in object_model.parts if p.name.startswith("key_")]
    ctx.check(
        "full field of articulated keycaps",
        len(key_parts) >= 70
        and all(
            object_model.get_articulation(f"{key.name}_slide").articulation_type == ArticulationType.PRISMATIC
            for key in key_parts
        ),
        details=f"key_count={len(key_parts)}",
    )

    ctx.expect_contact(
        sample_key,
        tray,
        elem_a="plunger_0",
        elem_b="key_deck",
        contact_tol=0.0002,
        name="key plungers sit just above the recessed deck",
    )
    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({sample_key_joint: 0.003}):
        key_pressed = ctx.part_world_position(sample_key)
    ctx.check(
        "key press travels downward",
        key_rest is not None and key_pressed is not None and key_pressed[2] < key_rest[2] - 0.0025,
        details=f"rest={key_rest}, pressed={key_pressed}",
    )

    ctx.expect_gap(
        stand_flap,
        tray,
        axis="z",
        positive_elem="flap_panel",
        negative_elem="stand_recess",
        min_gap=0.0001,
        max_gap=0.0020,
        name="stand flap lies recessed above rear trough",
    )
    ctx.expect_overlap(
        stand_flap,
        tray,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="hinge_pin",
        min_overlap=0.25,
        name="stand sleeve spans the rear hinge pin",
    )
    ctx.expect_within(
        tray,
        stand_flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.0008,
        name="hinge pin remains captured inside sleeve",
    )

    hinge_closed = ctx.part_world_position(stand_flap)
    closed_panel_aabb = ctx.part_element_world_aabb(stand_flap, elem="flap_panel")
    with ctx.pose({stand_joint: 1.1}):
        hinge_open = ctx.part_world_position(stand_flap)
        open_panel_aabb = ctx.part_element_world_aabb(stand_flap, elem="flap_panel")
        ctx.expect_overlap(
            stand_flap,
            tray,
            axes="x",
            elem_a="hinge_sleeve",
            elem_b="hinge_pin",
            min_overlap=0.25,
            name="open stand remains clipped to hinge line",
        )

    ctx.check(
        "stand hinge line stays fixed while opening",
        hinge_closed is not None
        and hinge_open is not None
        and abs(hinge_closed[0] - hinge_open[0]) < 1e-6
        and abs(hinge_closed[1] - hinge_open[1]) < 1e-6
        and abs(hinge_closed[2] - hinge_open[2]) < 1e-6,
        details=f"closed={hinge_closed}, open={hinge_open}",
    )
    ctx.check(
        "stand flap rotates upward to prop a tablet slot",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.020,
        details=f"closed_aabb={closed_panel_aabb}, open_aabb={open_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
