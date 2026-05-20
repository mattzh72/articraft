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
)


def _rounded_section_at_y(
    y_pos: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_val, y_pos, z_val)
        for x_val, z_val in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=6,
        )
    ]


def _make_flood_head_shell(name: str):
    return mesh_from_geometry(
        section_loft(
            [
                _rounded_section_at_y(-0.060, 0.240, 0.160, 0.016),
                _rounded_section_at_y(-0.005, 0.300, 0.200, 0.020),
                _rounded_section_at_y(0.055, 0.320, 0.220, 0.024),
            ]
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="highway_median_flood_mast")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.70, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.69, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.58, 0.66, 0.72, 0.88))
    dark_hardware = model.material("dark_hardware", rgba=(0.28, 0.28, 0.30, 1.0))

    head_shell_mesh = _make_flood_head_shell("flood_head_shell")

    foundation = model.part("foundation")
    foundation.visual(
        Box((1.00, 0.65, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=concrete,
        name="pedestal",
    )
    foundation.visual(
        Box((0.46, 0.46, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=concrete,
        name="grout_pad",
    )
    foundation.visual(
        Box((0.42, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=galvanized,
        name="base_flange",
    )
    for bolt_x in (-0.15, 0.15):
        for bolt_y in (-0.15, 0.15):
            foundation.visual(
                Cylinder(radius=0.013, length=0.07),
                origin=Origin(xyz=(bolt_x, bolt_y, 0.355)),
                material=dark_hardware,
                name=f"anchor_bolt_{'p' if bolt_x > 0 else 'n'}x_{'p' if bolt_y > 0 else 'n'}y",
            )
    foundation.inertial = Inertial.from_geometry(
        Box((1.00, 0.65, 0.39)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    pole = model.part("pole")
    pole.visual(
        Cylinder(radius=0.110, length=9.20),
        origin=Origin(xyz=(0.0, 0.0, 4.60)),
        material=galvanized,
        name="pole_shaft",
    )
    pole.visual(
        Cylinder(radius=0.135, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=galvanized,
        name="base_sleeve",
    )
    pole.visual(
        Cylinder(radius=0.102, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 9.22)),
        material=galvanized,
        name="top_cap",
    )
    pole.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=9.24),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 4.62)),
    )

    model.articulation(
        "foundation_to_pole",
        ArticulationType.FIXED,
        parent=foundation,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )

    crossarm = model.part("crossarm")
    crossarm.visual(
        Cylinder(radius=0.065, length=1.45),
        origin=Origin(xyz=(0.835, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="arm_tube",
    )
    crossarm.visual(
        Box((0.14, 0.22, 0.28)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=galvanized,
        name="mount_block",
    )
    crossarm.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(1.569, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="end_cap",
    )
    crossarm.inertial = Inertial.from_geometry(
        Box((1.60, 0.24, 0.28)),
        mass=46.0,
        origin=Origin(xyz=(0.82, 0.0, 0.0)),
    )

    model.articulation(
        "pole_to_crossarm",
        ArticulationType.FIXED,
        parent=pole,
        child=crossarm,
        origin=Origin(xyz=(0.0, 0.0, 9.05)),
    )

    def add_bracket(part_name: str):
        bracket = model.part(part_name)
        bracket.visual(
            Box((0.12, 0.08, 0.044)),
            origin=Origin(xyz=(0.0, 0.0, 0.103)),
            material=coated_steel,
            name="clamp_block",
        )
        bracket.visual(
            Box((0.36, 0.020, 0.008)),
            origin=Origin(xyz=(0.0, -0.005, 0.077)),
            material=coated_steel,
            name="upper_bridge",
        )
        bracket.visual(
            Box((0.014, 0.018, 0.156)),
            origin=Origin(xyz=(-0.179, -0.005, 0.001)),
            material=coated_steel,
            name="left_side_plate",
        )
        bracket.visual(
            Box((0.014, 0.018, 0.156)),
            origin=Origin(xyz=(0.179, -0.005, 0.001)),
            material=coated_steel,
            name="right_side_plate",
        )
        bracket.inertial = Inertial.from_geometry(
            Box((0.36, 0.08, 0.164)),
            mass=5.5,
            origin=Origin(xyz=(0.0, -0.005, 0.041)),
        )
        return bracket

    inner_bracket = add_bracket("inner_bracket")
    outer_bracket = add_bracket("outer_bracket")

    model.articulation(
        "crossarm_to_inner_bracket",
        ArticulationType.FIXED,
        parent=crossarm,
        child=inner_bracket,
        origin=Origin(xyz=(0.58, 0.0, -0.19)),
    )
    model.articulation(
        "crossarm_to_outer_bracket",
        ArticulationType.FIXED,
        parent=crossarm,
        child=outer_bracket,
        origin=Origin(xyz=(1.08, 0.0, -0.19)),
    )

    def add_flood_head(part_name: str):
        head = model.part(part_name)
        head.visual(
            head_shell_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.04)),
            material=coated_steel,
            name="housing_shell",
        )
        head.visual(
            Box((0.300, 0.018, 0.190)),
            origin=Origin(xyz=(0.0, 0.063, -0.04)),
            material=coated_steel,
            name="front_bezel",
        )
        head.visual(
            Box((0.278, 0.006, 0.165)),
            origin=Origin(xyz=(0.0, 0.060, -0.048)),
            material=lens_glass,
            name="lens",
        )
        head.visual(
            Box((0.250, 0.038, 0.130)),
            origin=Origin(xyz=(0.0, -0.075, -0.048)),
            material=coated_steel,
            name="rear_heat_sink",
        )
        for idx, fin_y in enumerate((-0.091, -0.083, -0.075)):
            head.visual(
                Box((0.270, 0.005, 0.142)),
                origin=Origin(xyz=(0.0, fin_y, -0.048)),
                material=coated_steel,
                name=f"heat_sink_fin_{idx}",
            )
        head.visual(
            Cylinder(radius=0.012, length=0.022),
            origin=Origin(xyz=(-0.161, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_hardware,
            name="left_trunnion",
        )
        head.visual(
            Cylinder(radius=0.012, length=0.022),
            origin=Origin(xyz=(0.161, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_hardware,
            name="right_trunnion",
        )
        head.inertial = Inertial.from_geometry(
            Box((0.32, 0.16, 0.22)),
            mass=12.0,
            origin=Origin(xyz=(0.0, -0.005, -0.04)),
        )
        return head

    inner_head = add_flood_head("inner_head")
    outer_head = add_flood_head("outer_head")

    model.articulation(
        "inner_head_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_bracket,
        child=inner_head,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-1.05,
            upper=0.55,
        ),
    )
    model.articulation(
        "outer_head_tilt",
        ArticulationType.REVOLUTE,
        parent=outer_bracket,
        child=outer_head,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-1.05,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    foundation = object_model.get_part("foundation")
    pole = object_model.get_part("pole")
    crossarm = object_model.get_part("crossarm")
    inner_bracket = object_model.get_part("inner_bracket")
    outer_bracket = object_model.get_part("outer_bracket")
    inner_head = object_model.get_part("inner_head")
    outer_head = object_model.get_part("outer_head")
    inner_head_tilt = object_model.get_articulation("inner_head_tilt")
    outer_head_tilt = object_model.get_articulation("outer_head_tilt")

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

    for part_name in (
        "foundation",
        "pole",
        "crossarm",
        "inner_bracket",
        "outer_bracket",
        "inner_head",
        "outer_head",
    ):
        ctx.check(
            f"part_present_{part_name}",
            object_model.get_part(part_name).name == part_name,
            details=f"Missing or misnamed part: {part_name}",
        )

    ctx.expect_contact(pole, foundation, elem_a="base_sleeve", elem_b="base_flange")
    ctx.expect_contact(crossarm, pole, elem_a="mount_block", elem_b="pole_shaft")
    ctx.expect_contact(inner_bracket, crossarm, elem_a="clamp_block", elem_b="arm_tube")
    ctx.expect_contact(outer_bracket, crossarm, elem_a="clamp_block", elem_b="arm_tube")
    ctx.expect_contact(
        inner_head,
        inner_bracket,
        elem_a="left_trunnion",
        elem_b="left_side_plate",
    )
    ctx.expect_contact(
        inner_head,
        inner_bracket,
        elem_a="right_trunnion",
        elem_b="right_side_plate",
    )
    ctx.expect_contact(
        outer_head,
        outer_bracket,
        elem_a="left_trunnion",
        elem_b="left_side_plate",
    )
    ctx.expect_contact(
        outer_head,
        outer_bracket,
        elem_a="right_trunnion",
        elem_b="right_side_plate",
    )

    ctx.expect_gap(
        crossarm,
        inner_head,
        axis="z",
        min_gap=0.04,
        positive_elem="arm_tube",
        name="inner_head_hangs_below_crossarm",
    )
    ctx.expect_gap(
        crossarm,
        outer_head,
        axis="z",
        min_gap=0.04,
        positive_elem="arm_tube",
        name="outer_head_hangs_below_crossarm",
    )
    ctx.expect_origin_distance(
        outer_head,
        inner_head,
        axes="x",
        min_dist=0.45,
        name="flood_heads_spaced_along_crossarm",
    )

    def tilt_joint_ok(joint_name: str) -> bool:
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        return (
            joint.axis == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and limits.upper - limits.lower >= 1.4
        )

    ctx.check(
        "inner_head_tilt_axis_and_limits",
        tilt_joint_ok("inner_head_tilt"),
        details="Inner LED head should pitch on a horizontal crossarm-parallel axis.",
    )
    ctx.check(
        "outer_head_tilt_axis_and_limits",
        tilt_joint_ok("outer_head_tilt"),
        details="Outer LED head should pitch on a horizontal crossarm-parallel axis.",
    )

    def aabb_center_z(part_obj, elem_name: str) -> float:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        assert aabb is not None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    inner_lens_rest_z = aabb_center_z(inner_head, "lens")
    outer_lens_rest_z = aabb_center_z(outer_head, "lens")
    with ctx.pose({inner_head_tilt: -0.65, outer_head_tilt: -0.45}):
        inner_lens_tilted_z = aabb_center_z(inner_head, "lens")
        outer_lens_tilted_z = aabb_center_z(outer_head, "lens")
        ctx.check(
            "inner_head_tilts_downward",
            inner_lens_tilted_z < inner_lens_rest_z - 0.02,
            details="Inner head lens did not swing downward under negative pitch.",
        )
        ctx.check(
            "outer_head_tilts_downward",
            outer_lens_tilted_z < outer_lens_rest_z - 0.015,
            details="Outer head lens did not swing downward under negative pitch.",
        )
        ctx.expect_contact(
            inner_head,
            inner_bracket,
            elem_a="left_trunnion",
            elem_b="left_side_plate",
        )
        ctx.expect_contact(
            outer_head,
            outer_bracket,
            elem_a="right_trunnion",
            elem_b="right_side_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
