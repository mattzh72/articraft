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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart")

    deck_len = 0.90
    deck_w = 0.55
    frame_th = 0.032
    mat_th = 0.006
    mount_plate = 0.09
    mount_plate_th = 0.008

    extension_len = 0.34
    extension_w = 0.34
    extension_th = 0.022
    runner_len = 0.30
    runner_w = 0.028
    runner_th = 0.010
    runner_y = 0.125
    guide_len = 0.34
    guide_w = 0.028
    guide_th = 0.020
    guide_x = 0.24
    slide_retracted_x = 0.21
    slide_origin_z = -0.041
    slide_travel = 0.24

    frame_color = model.material("frame_color", rgba=(0.26, 0.28, 0.30, 1.0))
    mat_color = model.material("mat_color", rgba=(0.12, 0.13, 0.14, 1.0))
    extension_color = model.material("extension_color", rgba=(0.30, 0.32, 0.34, 1.0))
    caster_color = model.material("caster_color", rgba=(0.70, 0.72, 0.75, 1.0))
    wheel_color = model.material("wheel_color", rgba=(0.08, 0.08, 0.08, 1.0))

    main_deck = model.part("main_deck")
    main_deck.visual(
        Box((deck_len, deck_w, frame_th)),
        origin=Origin(xyz=(0.0, 0.0, frame_th / 2.0)),
        material=frame_color,
        name="deck_frame",
    )
    main_deck.visual(
        Box((0.86, 0.51, mat_th)),
        origin=Origin(xyz=(0.0, 0.0, frame_th + (mat_th / 2.0))),
        material=mat_color,
        name="deck_mat",
    )
    for side, y in (("left", runner_y), ("right", -runner_y)):
        main_deck.visual(
            Box((guide_len, guide_w, guide_th)),
            origin=Origin(xyz=(guide_x, y, -(guide_th / 2.0))),
            material=frame_color,
            name=f"guide_{side}",
        )

    caster_locations = {
        "front_left": (0.31, runner_y + 0.08),
        "front_right": (0.31, -(runner_y + 0.08)),
        "rear_left": (-0.31, runner_y + 0.08),
        "rear_right": (-0.31, -(runner_y + 0.08)),
    }
    for name, (x, y) in caster_locations.items():
        main_deck.visual(
            Box((mount_plate, mount_plate, mount_plate_th)),
            origin=Origin(xyz=(x, y, -(mount_plate_th / 2.0))),
            material=frame_color,
            name=f"{name}_mount",
        )

    extension = model.part("extension_deck")
    extension.visual(
        Box((extension_len, extension_w, extension_th)),
        origin=Origin(),
        material=extension_color,
        name="extension_slab",
    )
    for side, y in (("left", runner_y), ("right", -runner_y)):
        extension.visual(
            Box((runner_len, runner_w, runner_th)),
            origin=Origin(xyz=(0.03, y, (extension_th / 2.0) + (runner_th / 2.0))),
            material=frame_color,
            name=f"{side}_runner",
        )
    extension.visual(
        Box((0.018, extension_w, 0.028)),
        origin=Origin(
            xyz=((extension_len / 2.0) - 0.009, 0.0, 0.003),
        ),
        material=extension_color,
        name="pull_lip",
    )

    model.articulation(
        "deck_extension",
        ArticulationType.PRISMATIC,
        parent=main_deck,
        child=extension,
        origin=Origin(xyz=(slide_retracted_x, 0.0, slide_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.40,
            lower=0.0,
            upper=slide_travel,
        ),
    )

    def add_caster(label: str, x: float, y: float) -> None:
        fork = model.part(f"{label}_fork")
        fork.visual(
            Cylinder(radius=0.011, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=caster_color,
            name="swivel_stem",
        )
        fork.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=caster_color,
            name="swivel_housing",
        )
        fork.visual(
            Box((0.070, 0.046, 0.010)),
            origin=Origin(xyz=(-0.010, 0.0, -0.029)),
            material=caster_color,
            name="fork_crown",
        )
        for side, y_sign in (("left", 0.020), ("right", -0.020)):
            fork.visual(
                Box((0.060, 0.006, 0.080)),
                origin=Origin(xyz=(-0.020, y_sign, -0.074)),
                material=caster_color,
                name=f"{side}_plate",
            )

        model.articulation(
            f"{label}_swivel",
            ArticulationType.CONTINUOUS,
            parent=main_deck,
            child=fork,
            origin=Origin(xyz=(x, y, -mount_plate_th)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=6.0),
        )

        wheel = model.part(f"{label}_wheel")
        wheel.visual(
            Cylinder(radius=0.050, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_color,
            name="tread",
        )
        for side, y_sign in (("left", 0.015), ("right", -0.015)):
            wheel.visual(
                Cylinder(radius=0.020, length=0.004),
                origin=Origin(xyz=(0.0, y_sign, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=caster_color,
                name=f"{side}_hub_cap",
            )

        model.articulation(
            f"{label}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(-0.020, 0.0, -0.100)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=20.0),
        )

    for label, (x, y) in caster_locations.items():
        add_caster(label, x, y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    main_deck = object_model.get_part("main_deck")
    extension = object_model.get_part("extension_deck")
    slide = object_model.get_articulation("deck_extension")

    def axis_is(joint, expected: tuple[float, float, float]) -> bool:
        return tuple(float(v) for v in joint.axis) == expected

    ctx.expect_contact(
        extension,
        main_deck,
        name="extension_contacts_guide_rails_retracted",
    )
    ctx.expect_gap(
        main_deck,
        extension,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="extension_runs_directly_below_deck",
    )
    ctx.expect_within(
        extension,
        main_deck,
        axes="y",
        margin=0.0,
        name="extension_stays_within_cart_width",
    )
    ctx.check(
        "extension_joint_axis_is_longitudinal",
        axis_is(slide, (1.0, 0.0, 0.0)),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "extension_joint_limits_match_slide_travel",
        (
            slide.motion_limits is not None
            and math.isclose(slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
            and math.isclose(slide.motion_limits.upper or -1.0, 0.24, abs_tol=1e-9)
        ),
        details=f"limits={slide.motion_limits}",
    )

    caster_labels = ("front_left", "front_right", "rear_left", "rear_right")
    for label in caster_labels:
        fork = object_model.get_part(f"{label}_fork")
        wheel = object_model.get_part(f"{label}_wheel")
        swivel = object_model.get_articulation(f"{label}_swivel")
        spin = object_model.get_articulation(f"{label}_wheel_spin")

        ctx.expect_contact(fork, main_deck, name=f"{label}_fork_is_mounted")
        ctx.expect_contact(wheel, fork, name=f"{label}_wheel_is_captured")
        ctx.check(
            f"{label}_swivel_axis_is_vertical",
            axis_is(swivel, (0.0, 0.0, 1.0)),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"{label}_wheel_axis_is_axial",
            axis_is(spin, (0.0, 1.0, 0.0)),
            details=f"axis={spin.axis}",
        )

    with ctx.pose({slide: 0.24}):
        ctx.expect_contact(
            extension,
            main_deck,
            name="extension_contacts_guide_rails_extended",
        )
        ctx.expect_overlap(
            extension,
            main_deck,
            axes="x",
            min_overlap=0.12,
            name="extension_remains_engaged_on_rails",
        )
        ctx.expect_within(
            extension,
            main_deck,
            axes="y",
            margin=0.0,
            name="extension_stays_centered_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
