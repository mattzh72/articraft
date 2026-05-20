from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_paper_cutter", assets=ASSETS)

    def _yz_section(
        x_pos: float,
        width_y: float,
        height_z: float,
        radius: float,
        center_z: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_val + center_z)
            for z_val, y_val in rounded_rect_profile(height_z, width_y, radius)
        ]

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    bed_cream = model.material("bed_cream", rgba=(0.92, 0.91, 0.86, 1.0))
    guide_green = model.material("guide_green", rgba=(0.35, 0.55, 0.42, 1.0))
    fence_aluminum = model.material("fence_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.79, 0.81, 0.84, 1.0))
    grip_red = model.material("grip_red", rgba=(0.62, 0.11, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.48, 0.32, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=painted_steel,
        name="base_slab",
    )
    base.visual(
        Box((0.43, 0.28, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, 0.020)),
        material=bed_cream,
        name="bed_panel",
    )
    base.visual(
        Box((0.40, 0.010, 0.001)),
        origin=Origin(xyz=(0.012, 0.091, 0.0215)),
        material=guide_green,
        name="cutting_strip",
    )
    for idx, x_pos in enumerate((-0.12, -0.04, 0.04, 0.12, 0.20), start=1):
        base.visual(
            Box((0.0012, 0.255, 0.0006)),
            origin=Origin(xyz=(x_pos, -0.008, 0.0217)),
            material=guide_green,
            name=f"grid_vertical_{idx}",
        )
    for idx, y_pos in enumerate((-0.10, -0.05, 0.00, 0.05), start=1):
        base.visual(
            Box((0.36, 0.0012, 0.0006)),
            origin=Origin(xyz=(0.045, y_pos, 0.0217)),
            material=guide_green,
            name=f"grid_horizontal_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.48, 0.32, 0.018)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    fence = model.part("side_fence")
    fence.visual(
        Box((0.028, 0.20, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=fence_aluminum,
        name="fence_foot",
    )
    fence.visual(
        Box((0.006, 0.20, 0.044)),
        origin=Origin(xyz=(-0.010, 0.0, 0.032)),
        material=fence_aluminum,
        name="fence_wall",
    )
    fence.inertial = Inertial.from_geometry(
        Box((0.028, 0.20, 0.010)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    bracket = model.part("pivot_bracket")
    bracket.visual(
        Box((0.060, 0.064, 0.010)),
        origin=Origin(xyz=(-0.020, 0.0, -0.013)),
        material=painted_steel,
        name="bracket_seat",
    )
    bracket.visual(
        Box((0.024, 0.064, 0.046)),
        origin=Origin(xyz=(-0.030, 0.0, -0.001)),
        material=painted_steel,
        name="bracket_backbone",
    )
    bracket.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(-0.001, -0.015, 0.0)),
        material=painted_steel,
        name="cheek_inner",
    )
    bracket.visual(
        Box((0.018, 0.008, 0.034)),
        origin=Origin(xyz=(-0.001, 0.015, 0.0)),
        material=painted_steel,
        name="cheek_outer",
    )
    bracket.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_steel,
        name="bracket_axle",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.060, 0.064, 0.046)),
        mass=1.1,
        origin=Origin(xyz=(-0.020, 0.0, -0.001)),
    )

    arm = model.part("blade_arm")
    arm.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="pivot_sleeve",
    )
    arm.visual(
        Box((0.030, 0.020, 0.006)),
        origin=Origin(xyz=(0.022, 0.0, 0.011)),
        material=painted_steel,
        name="hinge_web",
    )
    arm.visual(
        Box((0.080, 0.028, 0.016)),
        origin=Origin(xyz=(0.070, 0.0, 0.002)),
        material=painted_steel,
        name="arm_gusset",
    )
    arm.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_section(0.030, 0.048, 0.020, 0.005, -0.001),
                    _yz_section(0.135, 0.052, 0.022, 0.006, -0.002),
                    _yz_section(0.275, 0.046, 0.019, 0.005, -0.004),
                    _yz_section(0.410, 0.036, 0.015, 0.004, -0.006),
                ]
            ),
            ASSETS.mesh_path("paper_cutter_arm_beam.obj"),
        ),
        material=painted_steel,
        name="arm_beam",
    )
    arm.visual(
        Box((0.370, 0.004, 0.008)),
        origin=Origin(xyz=(0.220, -0.021, -0.014)),
        material=blade_steel,
        name="blade_edge",
    )
    arm.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(xyz=(0.340, 0.0, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_red,
        name="handle_grip",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.39, 0.050, 0.018)),
        mass=2.0,
        origin=Origin(xyz=(0.225, 0.0, -0.005)),
    )

    model.articulation(
        "base_to_fence",
        ArticulationType.FIXED,
        parent=base,
        child=fence,
        origin=Origin(xyz=(-0.205, -0.030, 0.022)),
    )
    model.articulation(
        "base_to_bracket",
        ArticulationType.FIXED,
        parent=base,
        child=bracket,
        origin=Origin(xyz=(-0.190, 0.112, 0.040)),
    )
    model.articulation(
        "bracket_to_arm",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    fence = object_model.get_part("side_fence")
    bracket = object_model.get_part("pivot_bracket")
    arm = object_model.get_part("blade_arm")
    arm_hinge = object_model.get_articulation("bracket_to_arm")

    base_slab = base.get_visual("base_slab")
    bed_panel = base.get_visual("bed_panel")
    cutting_strip = base.get_visual("cutting_strip")
    fence_foot = fence.get_visual("fence_foot")
    bracket_seat = bracket.get_visual("bracket_seat")
    bracket_axle = bracket.get_visual("bracket_axle")
    blade_edge = arm.get_visual("blade_edge")
    handle_grip = arm.get_visual("handle_grip")
    pivot_sleeve = arm.get_visual("pivot_sleeve")

    ctx.allow_overlap(
        arm,
        bracket,
        elem_a=pivot_sleeve,
        elem_b=bracket_axle,
        reason="The blade arm sleeve intentionally wraps around the bracket axle at the hinge.",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check("base_present", base is not None, "Base part was not created.")
    ctx.check("fence_present", fence is not None, "Side fence part was not created.")
    ctx.check("bracket_present", bracket is not None, "Pivot bracket part was not created.")
    ctx.check("arm_present", arm is not None, "Blade arm part was not created.")

    ctx.expect_gap(
        fence,
        base,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0002,
        positive_elem=fence_foot,
        negative_elem=bed_panel,
        name="fence_sits_on_bed",
    )
    ctx.expect_within(
        fence,
        base,
        axes="xy",
        margin=0.0,
        inner_elem=fence_foot,
        outer_elem=base_slab,
        name="fence_footprint_within_base",
    )
    ctx.expect_gap(
        bracket,
        base,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0002,
        positive_elem=bracket_seat,
        negative_elem=bed_panel,
        name="bracket_sits_on_bed",
    )
    ctx.expect_overlap(
        bracket,
        base,
        axes="xy",
        min_overlap=0.04,
        elem_a=bracket_seat,
        elem_b=base_slab,
        name="bracket_has_wide_mounting_footprint",
    )
    ctx.expect_contact(
        arm,
        bracket,
        elem_a=pivot_sleeve,
        elem_b=bracket_axle,
        contact_tol=0.006,
        name="arm_hinge_reads_connected",
    )

    ctx.check(
        "arm_hinge_limits",
        arm_hinge.motion_limits is not None
        and abs(arm_hinge.motion_limits.lower) <= 1e-9
        and 1.45 <= arm_hinge.motion_limits.upper <= 1.70,
        "The blade arm should open from closed to roughly a right angle.",
    )

    with ctx.pose({arm_hinge: 0.0}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0005,
            positive_elem=blade_edge,
            negative_elem=cutting_strip,
            name="closed_blade_seats_on_cutting_strip",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="x",
            min_overlap=0.30,
            elem_a=blade_edge,
            elem_b=cutting_strip,
            name="closed_blade_spans_cutting_strip_length",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="y",
            min_overlap=0.003,
            elem_a=blade_edge,
            elem_b=cutting_strip,
            name="closed_blade_tracks_cutting_strip_width",
        )

    with ctx.pose({arm_hinge: math.radians(90.0)}):
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            min_gap=0.22,
            positive_elem=handle_grip,
            negative_elem=base_slab,
            name="open_handle_lifts_clear_of_base",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            min_gap=0.045,
            positive_elem=blade_edge,
            negative_elem=base_slab,
            name="open_blade_clears_the_bed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
