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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.18, 0.24, 0.34, 1.0))
    liner = model.material("bed_liner", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("hinge_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    opening_width = 1.56
    wall_thickness = 0.06
    bedside_length = 0.22
    sill_height = 0.08
    bedside_height = 0.56

    main_gate_width = opening_width
    main_gate_height = 0.40
    main_gate_thickness = 0.055

    upper_flap_width = 1.46
    upper_flap_height = 0.12
    upper_flap_thickness = 0.045

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((opening_width + 2.0 * wall_thickness, 0.10, sill_height)),
        origin=Origin(xyz=(0.0, 0.05, sill_height * 0.5)),
        material=body_paint,
        name="rear_sill",
    )
    bed_frame.visual(
        Box((wall_thickness, bedside_length, bedside_height)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5 + wall_thickness * 0.5),
                bedside_length * 0.5,
                sill_height + bedside_height * 0.5,
            )
        ),
        material=body_paint,
        name="left_bedside",
    )
    bed_frame.visual(
        Box((wall_thickness, bedside_length, bedside_height)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 + wall_thickness * 0.5,
                bedside_length * 0.5,
                sill_height + bedside_height * 0.5,
            )
        ),
        material=body_paint,
        name="right_bedside",
    )
    bed_frame.visual(
        Box((opening_width, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, 0.09, sill_height + 0.009)),
        material=liner,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((wall_thickness + 0.02, bedside_length, 0.03)),
        origin=Origin(
            xyz=(
                -(opening_width * 0.5 + wall_thickness * 0.5),
                bedside_length * 0.5,
                sill_height + bedside_height - 0.015,
            )
        ),
        material=dark_trim,
        name="left_bedrail_cap",
    )
    bed_frame.visual(
        Box((wall_thickness + 0.02, bedside_length, 0.03)),
        origin=Origin(
            xyz=(
                opening_width * 0.5 + wall_thickness * 0.5,
                bedside_length * 0.5,
                sill_height + bedside_height - 0.015,
            )
        ),
        material=dark_trim,
        name="right_bedrail_cap",
    )
    bed_frame.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(
            xyz=(-opening_width * 0.5 + 0.035, 0.040, sill_height - 0.002),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_lower_hinge",
    )
    bed_frame.visual(
        Cylinder(radius=0.016, length=0.08),
        origin=Origin(
            xyz=(opening_width * 0.5 - 0.035, 0.040, sill_height - 0.002),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_lower_hinge",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((opening_width + 2.0 * wall_thickness, bedside_length, sill_height + bedside_height)),
        mass=75.0,
        origin=Origin(
            xyz=(
                0.0,
                bedside_length * 0.5,
                (sill_height + bedside_height) * 0.5,
            )
        ),
    )

    main_gate = model.part("main_gate")
    main_gate.visual(
        Box((main_gate_width, 0.030, main_gate_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(main_gate_thickness * 0.5 - 0.015),
                main_gate_height * 0.5,
            )
        ),
        material=body_paint,
        name="outer_skin",
    )
    main_gate.visual(
        Box((main_gate_width, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.0125, 0.0275)),
        material=dark_trim,
        name="inner_bottom_rail",
    )
    main_gate.visual(
        Box((main_gate_width, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, -0.0125, main_gate_height - 0.0275)),
        material=dark_trim,
        name="inner_top_rail",
    )
    main_gate.visual(
        Box((0.055, 0.025, main_gate_height - 0.11)),
        origin=Origin(
            xyz=(
                -(main_gate_width * 0.5 - 0.0275),
                -0.0125,
                main_gate_height * 0.5,
            )
        ),
        material=dark_trim,
        name="left_inner_rail",
    )
    main_gate.visual(
        Box((0.055, 0.025, main_gate_height - 0.11)),
        origin=Origin(
            xyz=(
                main_gate_width * 0.5 - 0.0275,
                -0.0125,
                main_gate_height * 0.5,
            )
        ),
        material=dark_trim,
        name="right_inner_rail",
    )
    main_gate.visual(
        Box((main_gate_width - 0.20, 0.020, main_gate_height - 0.14)),
        origin=Origin(
            xyz=(
                0.0,
                -0.015,
                main_gate_height * 0.5,
            )
        ),
        material=liner,
        name="inner_recess",
    )
    main_gate.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(
            xyz=(-main_gate_width * 0.5 + 0.03, -0.012, 0.012),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="left_hinge_knuckle",
    )
    main_gate.visual(
        Cylinder(radius=0.015, length=0.06),
        origin=Origin(
            xyz=(main_gate_width * 0.5 - 0.03, -0.012, 0.012),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="right_hinge_knuckle",
    )
    main_gate.inertial = Inertial.from_geometry(
        Box((main_gate_width, main_gate_thickness, main_gate_height)),
        mass=28.0,
        origin=Origin(
            xyz=(
                0.0,
                -main_gate_thickness * 0.5,
                main_gate_height * 0.5,
            )
        ),
    )

    upper_flap = model.part("upper_flap")
    upper_flap.visual(
        Box((upper_flap_width, 0.025, upper_flap_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(upper_flap_thickness * 0.5 - 0.0125),
                upper_flap_height * 0.5,
            )
        ),
        material=body_paint,
        name="skin_outer",
    )
    upper_flap.visual(
        Box((upper_flap_width, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, -0.010, upper_flap_height - 0.014)),
        material=dark_trim,
        name="top_cap",
    )
    upper_flap.visual(
        Box((0.050, 0.020, upper_flap_height - 0.03)),
        origin=Origin(
            xyz=(
                -(upper_flap_width * 0.5 - 0.025),
                -0.010,
                upper_flap_height * 0.5,
            )
        ),
        material=dark_trim,
        name="left_side_cap",
    )
    upper_flap.visual(
        Box((0.050, 0.020, upper_flap_height - 0.03)),
        origin=Origin(
            xyz=(
                upper_flap_width * 0.5 - 0.025,
                -0.010,
                upper_flap_height * 0.5,
            )
        ),
        material=dark_trim,
        name="right_side_cap",
    )
    upper_flap.visual(
        Box((upper_flap_width - 0.12, 0.020, upper_flap_height - 0.05)),
        origin=Origin(xyz=(0.0, -0.010, upper_flap_height * 0.5)),
        material=liner,
        name="inner_panel",
    )
    upper_flap.visual(
        Cylinder(radius=0.010, length=upper_flap_width - 0.18),
        origin=Origin(
            xyz=(0.0, -0.010, 0.010),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="hinge_barrel",
    )
    upper_flap.inertial = Inertial.from_geometry(
        Box((upper_flap_width, upper_flap_thickness, upper_flap_height)),
        mass=9.0,
        origin=Origin(
            xyz=(
                0.0,
                -upper_flap_thickness * 0.5,
                upper_flap_height * 0.5,
            )
        ),
    )

    model.articulation(
        "bed_to_main_gate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=main_gate,
        origin=Origin(xyz=(0.0, 0.0, sill_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=1.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "main_gate_to_upper_flap",
        ArticulationType.REVOLUTE,
        parent=main_gate,
        child=upper_flap,
        origin=Origin(xyz=(0.0, 0.0, main_gate_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    main_gate = object_model.get_part("main_gate")
    upper_flap = object_model.get_part("upper_flap")
    bed_to_main_gate = object_model.get_articulation("bed_to_main_gate")
    main_gate_to_upper_flap = object_model.get_articulation("main_gate_to_upper_flap")

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

    ctx.expect_contact(main_gate, bed_frame)
    ctx.expect_contact(upper_flap, main_gate)
    ctx.expect_gap(
        main_gate,
        bed_frame,
        axis="z",
        positive_elem="outer_skin",
        negative_elem="rear_sill",
        max_gap=0.001,
        max_penetration=0.0,
        name="main gate seats on rear sill",
    )
    ctx.expect_gap(
        upper_flap,
        main_gate,
        axis="z",
        positive_elem="skin_outer",
        negative_elem="inner_top_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper flap closes against main gate top edge",
    )
    ctx.expect_overlap(
        upper_flap,
        main_gate,
        axes="x",
        min_overlap=1.30,
        name="upper flap spans most of main gate width",
    )
    ctx.expect_overlap(
        main_gate,
        bed_frame,
        axes="x",
        min_overlap=1.50,
        name="main gate aligns across bed opening",
    )

    ctx.check(
        "lower gate hinge axis is transverse",
        tuple(bed_to_main_gate.axis) == (1.0, 0.0, 0.0),
        details=f"axis={bed_to_main_gate.axis}",
    )
    ctx.check(
        "upper flap hinge axis is transverse",
        tuple(main_gate_to_upper_flap.axis) == (1.0, 0.0, 0.0),
        details=f"axis={main_gate_to_upper_flap.axis}",
    )

    main_gate_closed = ctx.part_world_aabb(main_gate)
    upper_flap_closed = ctx.part_world_aabb(upper_flap)
    assert main_gate_closed is not None
    assert upper_flap_closed is not None

    with ctx.pose({bed_to_main_gate: 1.35}):
        main_gate_open = ctx.part_world_aabb(main_gate)
        upper_flap_carried = ctx.part_world_aabb(upper_flap)
        assert main_gate_open is not None
        assert upper_flap_carried is not None
        ctx.check(
            "main gate rotates downward from bottom hinge",
            main_gate_open[0][1] < main_gate_closed[0][1] - 0.28
            and main_gate_open[1][2] < main_gate_closed[1][2] - 0.15,
            details=f"closed={main_gate_closed}, open={main_gate_open}",
        )
        ctx.check(
            "upper flap rides with opened main gate",
            upper_flap_carried[1][1] < upper_flap_closed[1][1] - 0.28
            and upper_flap_carried[1][2] < upper_flap_closed[1][2] - 0.30,
            details=f"closed={upper_flap_closed}, carried={upper_flap_carried}",
        )

    with ctx.pose({main_gate_to_upper_flap: -1.20}):
        upper_flap_open = ctx.part_world_aabb(upper_flap)
        assert upper_flap_open is not None
        ctx.check(
            "upper flap folds inward for reach-in access",
            upper_flap_open[1][1] > upper_flap_closed[1][1] + 0.09
            and upper_flap_open[1][2] < upper_flap_closed[1][2] - 0.04,
            details=f"closed={upper_flap_closed}, open={upper_flap_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
