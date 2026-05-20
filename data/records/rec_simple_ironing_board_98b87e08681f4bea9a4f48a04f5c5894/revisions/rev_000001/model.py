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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_ironing_board")

    shell_white = model.material("shell_white", rgba=(0.88, 0.89, 0.87, 1.0))
    work_pad = model.material("work_pad", rgba=(0.71, 0.76, 0.78, 1.0))
    powder_coat = model.material("powder_coat", rgba=(0.24, 0.28, 0.31, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    stainless = model.material("stainless", rgba=(0.76, 0.79, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    def _board_outline(length: float, tail_width: float, nose_width: float) -> list[tuple[float, float]]:
        tail_x = -length * 0.5
        shoulder_x = length * 0.14
        taper_x = length * 0.36
        nose_center_x = length * 0.5 - nose_width * 0.5
        tail_half = tail_width * 0.5
        nose_half = nose_width * 0.5

        right_side = [
            (tail_x, tail_half * 0.92),
            (tail_x + 0.08, tail_half),
            (-0.10, tail_half),
            (shoulder_x, tail_half * 0.92),
            (taper_x, tail_half * 0.60),
            (nose_center_x, nose_half),
        ]
        tip = []
        for idx in range(1, 8):
            angle = math.pi / 2.0 - idx * (math.pi / 7.0)
            tip.append((nose_center_x + nose_half * math.cos(angle), nose_half * math.sin(angle)))
        left_side = [(x, -y) for x, y in reversed(right_side[:-1])]
        return right_side + tip + left_side

    def _leg_tube(
        part,
        *,
        name: str,
        y: float,
        start_xz: tuple[float, float],
        end_xz: tuple[float, float],
        radius: float,
        material,
    ) -> None:
        x1, z1 = start_xz
        x2, z2 = end_xz
        dx = x2 - x1
        dz = z2 - z1
        length = math.hypot(dx, dz)
        pitch = math.atan2(dx, dz)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=((x1 + x2) * 0.5, y, (z1 + z2) * 0.5), rpy=(0.0, pitch, 0.0)),
            material=material,
            name=name,
        )

    def _y_bar(
        part,
        *,
        name: str,
        center: tuple[float, float, float],
        length: float,
        radius: float,
        material,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    deck = model.part("deck")
    deck_outline = _board_outline(length=1.42, tail_width=0.38, nose_width=0.13)
    inner_outline = _board_outline(length=1.34, tail_width=0.32, nose_width=0.10)
    work_outline = _board_outline(length=1.36, tail_width=0.34, nose_width=0.11)

    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(deck_outline, 0.016), "board_outer_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.898)),
        material=shell_white,
        name="outer_shell",
    )
    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(inner_outline, 0.028), "board_inner_tray"),
        origin=Origin(xyz=(-0.015, 0.0, 0.878)),
        material=shell_white,
        name="inner_tray",
    )
    deck.visual(
        mesh_from_geometry(ExtrudeGeometry(work_outline, 0.004), "board_work_pad"),
        origin=Origin(xyz=(0.01, 0.0, 0.908)),
        material=work_pad,
        name="work_surface",
    )
    deck.visual(
        Box((0.88, 0.06, 0.028)),
        origin=Origin(xyz=(-0.08, -0.12, 0.850)),
        material=powder_coat,
        name="left_stiffener",
    )
    deck.visual(
        Box((0.88, 0.06, 0.028)),
        origin=Origin(xyz=(-0.08, 0.12, 0.850)),
        material=powder_coat,
        name="right_stiffener",
    )
    deck.visual(
        Box((0.24, 0.10, 0.054)),
        origin=Origin(xyz=(0.20, 0.0, 0.837)),
        material=powder_coat,
        name="guide_housing",
    )
    deck.visual(
        Box((0.16, 0.10, 0.020)),
        origin=Origin(xyz=(0.20, 0.0, 0.820)),
        material=stainless,
        name="guide_plate",
    )
    deck.visual(
        Box((0.10, 0.012, 0.056)),
        origin=Origin(xyz=(0.20, -0.056, 0.837)),
        material=powder_coat,
        name="guide_cheek_left",
    )
    deck.visual(
        Box((0.10, 0.012, 0.056)),
        origin=Origin(xyz=(0.20, 0.056, 0.837)),
        material=powder_coat,
        name="guide_cheek_right",
    )
    deck.visual(
        Box((0.18, 0.08, 0.012)),
        origin=Origin(xyz=(-0.04, 0.0, 0.804)),
        material=stainless,
        name="lock_plate",
    )
    deck.visual(
        Box((0.12, 0.012, 0.060)),
        origin=Origin(xyz=(-0.04, -0.042, 0.834)),
        material=powder_coat,
        name="lock_cheek_left",
    )
    deck.visual(
        Box((0.12, 0.012, 0.060)),
        origin=Origin(xyz=(-0.04, 0.042, 0.834)),
        material=powder_coat,
        name="lock_cheek_right",
    )
    deck.visual(
        Box((0.032, 0.030, 0.050)),
        origin=Origin(xyz=(-0.22, -0.115, 0.839)),
        material=powder_coat,
        name="hinge_block_left",
    )
    deck.visual(
        Box((0.032, 0.030, 0.050)),
        origin=Origin(xyz=(-0.22, 0.115, 0.839)),
        material=powder_coat,
        name="hinge_block_right",
    )
    _y_bar(
        deck,
        name="hinge_ear_left",
        center=(-0.22, -0.115, 0.840),
        length=0.070,
        radius=0.012,
        material=stainless,
    )
    _y_bar(
        deck,
        name="hinge_ear_right",
        center=(-0.22, 0.115, 0.840),
        length=0.070,
        radius=0.012,
        material=stainless,
    )
    deck.inertial = Inertial.from_geometry(
        Box((1.42, 0.38, 0.11)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
    )

    primary_leg = model.part("primary_leg")
    _y_bar(
        primary_leg,
        name="left_top_hinge",
        center=(0.0, -0.045, 0.0),
        length=0.070,
        radius=0.011,
        material=stainless,
    )
    _y_bar(
        primary_leg,
        name="right_top_hinge",
        center=(0.0, 0.045, 0.0),
        length=0.070,
        radius=0.011,
        material=stainless,
    )
    primary_leg.visual(
        Box((0.080, 0.040, 0.028)),
        origin=Origin(xyz=(0.050, -0.078, -0.018)),
        material=powder_coat,
        name="left_top_bracket",
    )
    primary_leg.visual(
        Box((0.080, 0.040, 0.028)),
        origin=Origin(xyz=(0.050, 0.078, -0.018)),
        material=powder_coat,
        name="right_top_bracket",
    )
    _leg_tube(
        primary_leg,
        name="left_main_rail",
        y=-0.095,
        start_xz=(0.060, -0.025),
        end_xz=(0.470, -0.820),
        radius=0.011,
        material=powder_coat,
    )
    _leg_tube(
        primary_leg,
        name="right_main_rail",
        y=0.095,
        start_xz=(0.060, -0.025),
        end_xz=(0.470, -0.820),
        radius=0.011,
        material=powder_coat,
    )
    primary_leg.visual(
        Box((0.100, 0.180, 0.080)),
        origin=Origin(xyz=(0.120, 0.0, -0.110)),
        material=powder_coat,
        name="lock_tower",
    )
    primary_leg.visual(
        Box((0.050, 0.026, 0.044)),
        origin=Origin(xyz=(0.250, -0.087, -0.400)),
        material=powder_coat,
        name="left_pivot_lug",
    )
    primary_leg.visual(
        Box((0.050, 0.026, 0.044)),
        origin=Origin(xyz=(0.250, 0.087, -0.400)),
        material=powder_coat,
        name="right_pivot_lug",
    )
    _y_bar(
        primary_leg,
        name="pivot_pin",
        center=(0.250, 0.0, -0.400),
        length=0.180,
        radius=0.009,
        material=stainless,
    )
    primary_leg.visual(
        Box((0.070, 0.060, 0.028)),
        origin=Origin(xyz=(0.170, 0.0, -0.056)),
        material=powder_coat,
        name="lock_head",
    )
    primary_leg.visual(
        Box((0.070, 0.036, 0.010)),
        origin=Origin(xyz=(0.180, 0.0, -0.047)),
        material=stainless,
        name="lock_pad",
    )
    _y_bar(
        primary_leg,
        name="lock_pin",
        center=(0.182, 0.0, -0.043),
        length=0.046,
        radius=0.005,
        material=stainless,
    )
    _y_bar(
        primary_leg,
        name="foot_bar",
        center=(0.470, 0.0, -0.820),
        length=0.330,
        radius=0.012,
        material=powder_coat,
    )
    primary_leg.visual(
        Box((0.072, 0.046, 0.018)),
        origin=Origin(xyz=(0.470, -0.145, -0.829)),
        material=rubber,
        name="left_foot",
    )
    primary_leg.visual(
        Box((0.072, 0.046, 0.018)),
        origin=Origin(xyz=(0.470, 0.145, -0.829)),
        material=rubber,
        name="right_foot",
    )
    primary_leg.inertial = Inertial.from_geometry(
        Box((0.58, 0.34, 0.86)),
        mass=1.6,
        origin=Origin(xyz=(0.240, 0.0, -0.410)),
    )

    secondary_leg = model.part("secondary_leg")
    secondary_leg.visual(
        Box((0.020, 0.028, 0.044)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=stainless,
        name="pivot_block",
    )
    secondary_leg.visual(
        Box((0.050, 0.080, 0.020)),
        origin=Origin(xyz=(0.044, 0.0, -0.028)),
        material=powder_coat,
        name="pivot_mount",
    )
    _y_bar(
        secondary_leg,
        name="pivot_bridge",
        center=(0.040, 0.0, -0.035),
        length=0.100,
        radius=0.008,
        material=powder_coat,
    )
    _leg_tube(
        secondary_leg,
        name="left_lower_leg",
        y=-0.050,
        start_xz=(0.040, -0.020),
        end_xz=(-0.280, -0.420),
        radius=0.011,
        material=powder_coat,
    )
    _leg_tube(
        secondary_leg,
        name="right_lower_leg",
        y=0.050,
        start_xz=(0.040, -0.020),
        end_xz=(-0.280, -0.420),
        radius=0.011,
        material=powder_coat,
    )
    _leg_tube(
        secondary_leg,
        name="left_upper_leg",
        y=-0.050,
        start_xz=(0.040, -0.018),
        end_xz=(0.180, 0.352),
        radius=0.011,
        material=powder_coat,
    )
    _leg_tube(
        secondary_leg,
        name="right_upper_leg",
        y=0.050,
        start_xz=(0.040, -0.018),
        end_xz=(0.180, 0.352),
        radius=0.011,
        material=powder_coat,
    )
    _y_bar(
        secondary_leg,
        name="foot_bar",
        center=(-0.280, 0.0, -0.420),
        length=0.230,
        radius=0.012,
        material=powder_coat,
    )
    secondary_leg.visual(
        Box((0.068, 0.040, 0.018)),
        origin=Origin(xyz=(-0.280, -0.100, -0.429)),
        material=rubber,
        name="left_foot",
    )
    secondary_leg.visual(
        Box((0.068, 0.040, 0.018)),
        origin=Origin(xyz=(-0.280, 0.100, -0.429)),
        material=rubber,
        name="right_foot",
    )
    _y_bar(
        secondary_leg,
        name="top_crossbar",
        center=(0.180, 0.0, 0.340),
        length=0.230,
        radius=0.010,
        material=powder_coat,
    )
    secondary_leg.visual(
        Box((0.125, 0.080, 0.024)),
        origin=Origin(xyz=(0.175, 0.0, 0.358)),
        material=stainless,
        name="top_shoe",
    )
    secondary_leg.visual(
        Box((0.090, 0.100, 0.040)),
        origin=Origin(xyz=(0.180, 0.0, 0.334)),
        material=dark_polymer,
        name="shoe_shroud",
    )
    secondary_leg.inertial = Inertial.from_geometry(
        Box((0.52, 0.24, 0.80)),
        mass=1.35,
        origin=Origin(xyz=(-0.010, 0.0, -0.020)),
    )

    main_hinge = model.articulation(
        "deck_to_primary_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=primary_leg,
        origin=Origin(xyz=(-0.220, 0.0, 0.840)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=0.72,
        ),
    )
    scissor_pivot = model.articulation(
        "primary_to_secondary_leg",
        ArticulationType.REVOLUTE,
        parent=primary_leg,
        child=secondary_leg,
        origin=Origin(xyz=(0.250, 0.0, -0.400)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-0.82,
            upper=0.10,
        ),
    )

    model.meta["default_pose"] = {
        main_hinge.name: 0.0,
        scissor_pivot.name: 0.0,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    primary_leg = object_model.get_part("primary_leg")
    secondary_leg = object_model.get_part("secondary_leg")
    main_hinge = object_model.get_articulation("deck_to_primary_leg")
    scissor_pivot = object_model.get_articulation("primary_to_secondary_leg")

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

    with ctx.pose({main_hinge: 0.0, scissor_pivot: 0.0}):
        ctx.expect_contact(deck, primary_leg, elem_a="hinge_ear_left", elem_b="left_top_hinge")
        ctx.expect_contact(deck, primary_leg, elem_a="lock_plate", elem_b="lock_pad")
        ctx.expect_contact(deck, secondary_leg, elem_a="guide_plate", elem_b="top_shoe")
        ctx.expect_contact(primary_leg, secondary_leg, elem_a="pivot_pin", elem_b="pivot_block")
        ctx.expect_gap(
            deck,
            primary_leg,
            axis="z",
            positive_elem="outer_shell",
            negative_elem="foot_bar",
            min_gap=0.78,
            name="working_height_from_primary_feet",
        )
        ctx.expect_gap(
            deck,
            secondary_leg,
            axis="z",
            positive_elem="outer_shell",
            negative_elem="foot_bar",
            min_gap=0.78,
            name="working_height_from_secondary_feet",
        )
    main_limits = main_hinge.motion_limits
    scissor_limits = scissor_pivot.motion_limits
    ctx.check(
        "main_hinge_has_meaningful_range",
        main_limits is not None
        and main_limits.lower is not None
        and main_limits.upper is not None
        and (main_limits.upper - main_limits.lower) >= 0.60,
        details=f"main_limits={main_limits}",
    )
    ctx.check(
        "scissor_pivot_has_meaningful_range",
        scissor_limits is not None
        and scissor_limits.lower is not None
        and scissor_limits.upper is not None
        and (scissor_limits.upper - scissor_limits.lower) >= 0.70,
        details=f"scissor_limits={scissor_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
