from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_ironing_board")

    deck_paint = model.material("deck_paint", rgba=(0.79, 0.76, 0.70, 1.0))
    cover_fabric = model.material("cover_fabric", rgba=(0.83, 0.82, 0.78, 1.0))
    steel_tube = model.material("steel_tube", rgba=(0.32, 0.34, 0.37, 1.0))
    hardware = model.material("hardware", rgba=(0.58, 0.60, 0.62, 1.0))
    hatch_finish = model.material("hatch_finish", rgba=(0.42, 0.45, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    def yz_section(
        x_pos: float,
        width: float,
        thickness: float,
        radius: float,
        *,
        corner_segments: int = 8,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val)
            for z_val, y_val in rounded_rect_profile(
                thickness,
                width,
                radius,
                corner_segments=corner_segments,
            )
        ]

    def rod_mesh(
        name: str,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        *,
        radius: float,
    ):
        mid = (
            (start[0] + end[0]) * 0.5,
            (start[1] + end[1]) * 0.5,
            (start[2] + end[2]) * 0.5,
        )
        return mesh_from_geometry(
            tube_from_spline_points(
                [start, mid, end],
                radius=radius,
                samples_per_segment=6,
                radial_segments=16,
                cap_ends=True,
            ),
            name,
        )

    def add_beam(
        part_obj,
        name: str,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        *,
        thickness_y: float,
        thickness_z: float,
        material,
    ) -> None:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        horiz = math.sqrt(dx * dx + dy * dy)
        yaw = math.atan2(dy, dx)
        pitch = -math.atan2(dz, horiz) if horiz > 1e-9 else 0.0
        part_obj.visual(
            Box((length, thickness_y, thickness_z)),
            origin=Origin(
                xyz=(
                    (start[0] + end[0]) * 0.5,
                    (start[1] + end[1]) * 0.5,
                    (start[2] + end[2]) * 0.5,
                ),
                rpy=(0.0, pitch, yaw),
            ),
            material=material,
            name=name,
        )

    def add_hatch(part_name: str, x_pos: float) -> None:
        hatch = model.part(part_name)
        hatch.visual(
            Box((0.18, 0.08, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=hatch_finish,
            name="panel",
        )
        for idx, (bx, by) in enumerate(
            (
                (-0.065, -0.023),
                (-0.065, 0.023),
                (0.065, -0.023),
                (0.065, 0.023),
            ),
            start=1,
        ):
            hatch.visual(
                Cylinder(radius=0.004, length=0.004),
                origin=Origin(xyz=(bx, by, -0.008)),
                material=hardware,
                name=f"bolt_{idx}",
            )
        hatch.inertial = Inertial.from_geometry(
            Box((0.18, 0.08, 0.012)),
            mass=0.15,
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
        )
        model.articulation(
            f"deck_to_{part_name}",
            ArticulationType.FIXED,
            parent="deck",
            child=hatch,
            origin=Origin(xyz=(x_pos, 0.0, -0.048)),
        )

    def add_leg_frame(
        part_name: str,
        *,
        foot_x: float,
        boss_x: float,
        y_offset: float,
        add_lock_strike: bool,
    ) -> None:
        leg = model.part(part_name)
        top_z = -0.016
        bottom_z = -0.775
        rail_y = 0.105

        leg.visual(
            Box((0.070, 0.290, 0.026)),
            origin=Origin(xyz=(0.0, y_offset, top_z)),
            material=steel_tube,
            name="top_crossmember",
        )
        leg.visual(
            Cylinder(radius=0.006, length=0.24),
            origin=Origin(xyz=(0.0, y_offset, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name="hinge_barrel",
        )
        for side_y in (-0.095, 0.095):
            leg.visual(
                Box((0.048, 0.052, 0.032)),
                origin=Origin(xyz=(0.004, y_offset + side_y, -0.011)),
                material=hardware,
                name=f"hinge_adapter_{'left' if side_y < 0.0 else 'right'}",
            )

        add_beam(
            leg,
            "left_rail",
            (0.010, y_offset - rail_y, top_z),
            (foot_x, y_offset - rail_y, bottom_z),
            thickness_y=0.022,
            thickness_z=0.022,
            material=steel_tube,
        )
        add_beam(
            leg,
            "right_rail",
            (0.010, y_offset + rail_y, top_z),
            (foot_x, y_offset + rail_y, bottom_z),
            thickness_y=0.022,
            thickness_z=0.022,
            material=steel_tube,
        )

        leg.visual(
            Box((0.070, 0.320, 0.026)),
            origin=Origin(xyz=(foot_x, y_offset, bottom_z)),
            material=steel_tube,
            name="foot_crossbar",
        )
        for side_y in (-0.145, 0.145):
            leg.visual(
                Box((0.060, 0.030, 0.012)),
                origin=Origin(xyz=(foot_x, y_offset + side_y, bottom_z - 0.017)),
                material=rubber,
                name=f"{'left' if side_y < 0.0 else 'right'}_foot_pad",
            )

        add_beam(
            leg,
            "center_brace",
            (0.000, y_offset, -0.030),
            (boss_x, y_offset, -0.390),
            thickness_y=0.022,
            thickness_z=0.020,
            material=steel_tube,
        )
        leg.visual(
            Box((0.060, 0.050, 0.032)),
            origin=Origin(xyz=(boss_x, y_offset, -0.390)),
            material=hardware,
            name="cross_pivot_boss",
        )

        if add_lock_strike:
            leg.visual(
                Box((0.148, 0.018, 0.018)),
                origin=Origin(xyz=(0.095, 0.113, -0.030)),
                material=steel_tube,
                name="lock_top_bracket",
            )
            leg.visual(
                Box((0.028, 0.020, 0.310)),
                origin=Origin(xyz=(0.153, 0.130, -0.184)),
                material=steel_tube,
                name="lock_post",
            )
            leg.visual(
                Box((0.024, 0.050, 0.022)),
                origin=Origin(xyz=(0.153, 0.130, -0.339)),
                material=hardware,
                name="lock_strike",
            )
        leg.inertial = Inertial.from_geometry(
            Box((0.56, 0.42, 0.82)),
            mass=2.0,
            origin=Origin(xyz=(foot_x * 0.5, y_offset, -0.39)),
        )

    deck = model.part("deck")

    board_sections = [
        yz_section(-0.62, 0.32, 0.018, 0.008),
        yz_section(-0.28, 0.38, 0.018, 0.008),
        yz_section(0.18, 0.36, 0.018, 0.008),
        yz_section(0.50, 0.22, 0.015, 0.007),
        yz_section(0.61, 0.12, 0.010, 0.004),
    ]
    deck.visual(
        mesh_from_geometry(section_loft(board_sections), "board_shell"),
        material=deck_paint,
        name="board_shell",
    )

    pad_sections = [
        yz_section(-0.60, 0.33, 0.008, 0.003),
        yz_section(-0.28, 0.39, 0.008, 0.003),
        yz_section(0.18, 0.37, 0.008, 0.003),
        yz_section(0.48, 0.23, 0.008, 0.003),
        yz_section(0.58, 0.14, 0.008, 0.003),
    ]
    deck.visual(
        mesh_from_geometry(section_loft(pad_sections), "board_pad"),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=cover_fabric,
        name="pad_cover",
    )

    deck.visual(
        Box((0.22, 0.10, 0.040)),
        origin=Origin(xyz=(-0.40, 0.0, -0.028)),
        material=deck_paint,
        name="rear_spine_rail",
    )
    deck.visual(
        Box((0.14, 0.10, 0.040)),
        origin=Origin(xyz=(-0.05, 0.0, -0.028)),
        material=deck_paint,
        name="front_spine_rail",
    )

    for name, x_pos in (("nose_adapter_plate", 0.24), ("tail_adapter_plate", -0.24)):
        deck.visual(
            Box((0.18, 0.24, 0.016)),
            origin=Origin(xyz=(x_pos, 0.0, -0.020)),
            material=hardware,
            name=name,
        )
        for idx, bolt_y in enumerate((-0.085, 0.085), start=1):
            deck.visual(
                Cylinder(radius=0.005, length=0.020),
                origin=Origin(xyz=(x_pos + 0.055, bolt_y, -0.026)),
                material=hardware,
                name=f"{name}_bolt_{idx}",
            )
            deck.visual(
                Cylinder(radius=0.005, length=0.020),
                origin=Origin(xyz=(x_pos - 0.055, bolt_y, -0.026)),
                material=hardware,
                name=f"{name}_bolt_{idx + 2}",
            )

    deck.visual(
        Box((0.090, 0.090, 0.024)),
        origin=Origin(xyz=(0.11, 0.0, -0.020)),
        material=hardware,
        name="brace_anchor_block",
    )
    for side, y_pos in (("left", -0.040), ("right", 0.040)):
        deck.visual(
            Box((0.030, 0.018, 0.030)),
            origin=Origin(xyz=(0.11, y_pos, -0.033)),
            material=hardware,
            name=f"brace_hanger_{side}",
        )

    deck.inertial = Inertial.from_geometry(
        Box((1.24, 0.40, 0.10)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    add_hatch("front_service_hatch", -0.05)
    add_hatch("rear_service_hatch", -0.40)

    add_leg_frame(
        "nose_leg",
        foot_x=-0.48,
        boss_x=-0.24,
        y_offset=0.024,
        add_lock_strike=False,
    )
    add_leg_frame(
        "tail_leg",
        foot_x=0.48,
        boss_x=0.24,
        y_offset=-0.024,
        add_lock_strike=True,
    )

    lock_brace = model.part("lock_brace")
    lock_brace.visual(
        Cylinder(radius=0.008, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    lock_brace.visual(
        Box((0.028, 0.240, 0.024)),
        origin=Origin(xyz=(0.002, 0.120, -0.018)),
        material=hardware,
        name="hinge_collar",
    )
    add_beam(
        lock_brace,
        "brace_rod",
        (0.0, 0.180, -0.018),
        (-0.221, 0.180, -0.317),
        thickness_y=0.018,
        thickness_z=0.018,
        material=steel_tube,
    )
    lock_brace.visual(
        Box((0.024, 0.100, 0.022)),
        origin=Origin(xyz=(-0.221, 0.180, -0.356)),
        material=hardware,
        name="lock_shoe",
    )
    lock_brace.visual(
        Box((0.024, 0.018, 0.080)),
        origin=Origin(xyz=(-0.221, 0.180, -0.362)),
        material=steel_tube,
        name="shoe_stem",
    )
    lock_brace.inertial = Inertial.from_geometry(
        Box((0.18, 0.09, 0.28)),
        mass=0.4,
        origin=Origin(xyz=(-0.07, 0.0, -0.12)),
    )

    model.articulation(
        "deck_to_nose_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="nose_leg",
        origin=Origin(xyz=(0.24, 0.0, -0.039)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "deck_to_tail_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="tail_leg",
        origin=Origin(xyz=(-0.24, 0.0, -0.039)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.8,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "deck_to_lock_brace",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lock_brace,
        origin=Origin(xyz=(0.11, 0.0, -0.039)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    nose_leg = object_model.get_part("nose_leg")
    tail_leg = object_model.get_part("tail_leg")
    lock_brace = object_model.get_part("lock_brace")
    front_hatch = object_model.get_part("front_service_hatch")
    rear_hatch = object_model.get_part("rear_service_hatch")

    nose_hinge = object_model.get_articulation("deck_to_nose_leg")
    tail_hinge = object_model.get_articulation("deck_to_tail_leg")
    brace_hinge = object_model.get_articulation("deck_to_lock_brace")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        nose_leg,
        deck,
        elem_a="hinge_barrel",
        elem_b="nose_adapter_plate",
        name="nose_leg_hinge_supported",
    )
    ctx.expect_contact(
        tail_leg,
        deck,
        elem_a="hinge_barrel",
        elem_b="tail_adapter_plate",
        name="tail_leg_hinge_supported",
    )
    ctx.expect_contact(
        nose_leg,
        tail_leg,
        elem_a="cross_pivot_boss",
        elem_b="cross_pivot_boss",
        name="scissor_pivot_bosses_meet",
    )
    ctx.expect_contact(
        lock_brace,
        deck,
        elem_a="hinge_barrel",
        elem_b="brace_anchor_block",
        name="lock_brace_hinge_supported",
    )
    ctx.expect_contact(
        lock_brace,
        tail_leg,
        elem_a="lock_shoe",
        elem_b="lock_strike",
        name="open_stance_lock_engaged",
    )
    ctx.expect_contact(
        front_hatch,
        deck,
        elem_a="panel",
        elem_b="front_spine_rail",
        name="front_hatch_mounted",
    )
    ctx.expect_contact(
        rear_hatch,
        deck,
        elem_a="panel",
        elem_b="rear_spine_rail",
        name="rear_hatch_mounted",
    )

    open_left_foot = ctx.part_element_world_aabb(nose_leg, elem="left_foot_pad")
    open_right_foot = ctx.part_element_world_aabb(tail_leg, elem="right_foot_pad")
    with ctx.pose(
        {
            nose_hinge: 1.10,
            tail_hinge: 1.05,
            brace_hinge: 1.18,
        }
    ):
        folded_left_foot = ctx.part_element_world_aabb(nose_leg, elem="left_foot_pad")
        folded_right_foot = ctx.part_element_world_aabb(tail_leg, elem="right_foot_pad")
        if open_left_foot is None or open_right_foot is None or folded_left_foot is None or folded_right_foot is None:
            ctx.fail("fold_pose_measurements_available", "Could not resolve foot-pad AABBs for fold check.")
        else:
            open_left_top = open_left_foot[1][2]
            open_right_top = open_right_foot[1][2]
            folded_left_top = folded_left_foot[1][2]
            folded_right_top = folded_right_foot[1][2]
            ctx.check(
                "nose_leg_folds_up_under_deck",
                folded_left_top > open_left_top + 0.45,
                details=(
                    f"Expected left foot to rise at least 0.45 m when folded, "
                    f"got open_top={open_left_top:.3f}, folded_top={folded_left_top:.3f}."
                ),
            )
            ctx.check(
                "tail_leg_folds_up_under_deck",
                folded_right_top > open_right_top + 0.45,
                details=(
                    f"Expected right foot to rise at least 0.45 m when folded, "
                    f"got open_top={open_right_top:.3f}, folded_top={folded_right_top:.3f}."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
