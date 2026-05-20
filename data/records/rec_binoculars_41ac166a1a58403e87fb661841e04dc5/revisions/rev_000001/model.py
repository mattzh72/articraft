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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilized_binocular_10x30")

    armor = model.material("armor", rgba=(0.16, 0.17, 0.18, 1.0))
    armor_dark = model.material("armor_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.35, 0.37, 0.40, 1.0))
    glass = model.material("glass", rgba=(0.32, 0.40, 0.46, 0.50))
    accent = model.material("accent", rgba=(0.36, 0.43, 0.23, 1.0))

    def xz_section(
        width: float,
        height: float,
        radius: float,
        y_pos: float,
        *,
        z_offset: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y_pos, z + z_offset)
            for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)
        ]

    def barrel_shell_mesh():
        return section_loft(
            [
                xz_section(0.040, 0.044, 0.010, -0.068, z_offset=0.010),
                xz_section(0.052, 0.058, 0.014, -0.036, z_offset=0.006),
                xz_section(0.056, 0.062, 0.015, -0.004, z_offset=0.003),
                xz_section(0.050, 0.054, 0.013, 0.034, z_offset=0.000),
                xz_section(0.040, 0.040, 0.010, 0.072, z_offset=-0.003),
            ]
        )

    def bridge_arm_mesh(
        widths: tuple[float, ...],
        heights: tuple[float, ...],
        ys: tuple[float, ...],
        z_biases: tuple[float, ...],
    ):
        sections = []
        for width, height, y_pos, z_bias in zip(widths, heights, ys, z_biases):
            radius = min(width, height) * 0.35
            sections.append(xz_section(width, height, radius, y_pos, z_offset=z_bias))
        return section_loft(sections)

    barrel_mesh = mesh_from_geometry(barrel_shell_mesh(), "binocular_barrel_shell")
    left_bridge_mesh = mesh_from_geometry(
        bridge_arm_mesh(
            (0.024, 0.020, 0.012),
            (0.014, 0.016, 0.012),
            (-0.028, -0.012, 0.010),
            (0.014, 0.015, 0.013),
        ),
        "binocular_left_bridge_arm",
    )
    right_bridge_mesh = mesh_from_geometry(
        bridge_arm_mesh(
            (0.018, 0.016, 0.010),
            (0.012, 0.014, 0.010),
            (-0.022, -0.008, 0.006),
            (0.001, 0.001, 0.000),
        ),
        "binocular_right_bridge_arm",
    )

    left_body = model.part("left_body")
    left_body.visual(
        barrel_mesh,
        origin=Origin(xyz=(-0.040, 0.000, 0.000)),
        material=armor,
        name="barrel_shell",
    )
    left_body.visual(
        Cylinder(radius=0.0215, length=0.016),
        origin=Origin(xyz=(-0.040, 0.076, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="objective_shroud",
    )
    left_body.visual(
        Cylinder(radius=0.0185, length=0.004),
        origin=Origin(xyz=(-0.040, 0.082, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="objective_bezel",
    )
    left_body.visual(
        Cylinder(radius=0.0168, length=0.0012),
        origin=Origin(xyz=(-0.040, 0.074, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="objective_glass",
    )
    left_body.visual(
        Cylinder(radius=0.0165, length=0.014),
        origin=Origin(xyz=(-0.040, -0.072, 0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="eyecup_body",
    )
    left_body.visual(
        Cylinder(radius=0.0185, length=0.008),
        origin=Origin(xyz=(-0.040, -0.077, 0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="eyecup_lip",
    )
    left_body.visual(
        Cylinder(radius=0.0125, length=0.0012),
        origin=Origin(xyz=(-0.040, -0.067, 0.002), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="ocular_glass",
    )
    left_body.visual(
        Cylinder(radius=0.0065, length=0.034),
        origin=Origin(xyz=(-0.013, -0.010, 0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="bridge_arm",
    )
    left_body.visual(
        Box((0.022, 0.022, 0.014)),
        origin=Origin(xyz=(-0.003, -0.022, 0.019)),
        material=armor_dark,
        name="focus_bridge_pad",
    )
    left_body.visual(
        Box((0.003, 0.020, 0.016)),
        origin=Origin(xyz=(-0.0065, -0.015, 0.034)),
        material=metal,
        name="focus_left_cheek",
    )
    left_body.visual(
        Box((0.003, 0.020, 0.016)),
        origin=Origin(xyz=(0.0065, -0.015, 0.034)),
        material=metal,
        name="focus_right_cheek",
    )
    left_body.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(-0.004, 0.000, 0.014)),
        material=metal,
        name="hinge_knuckle",
    )
    left_body.inertial = Inertial.from_geometry(
        Box((0.116, 0.162, 0.078)),
        mass=0.42,
        origin=Origin(xyz=(-0.020, 0.000, 0.018)),
    )

    right_body = model.part("right_body")
    right_body.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.040, 0.000, -0.018)),
        material=armor,
        name="barrel_shell",
    )
    right_body.visual(
        Cylinder(radius=0.0215, length=0.016),
        origin=Origin(xyz=(0.040, 0.076, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="objective_shroud",
    )
    right_body.visual(
        Cylinder(radius=0.0185, length=0.004),
        origin=Origin(xyz=(0.040, 0.082, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="objective_bezel",
    )
    right_body.visual(
        Cylinder(radius=0.0168, length=0.0012),
        origin=Origin(xyz=(0.040, 0.074, -0.018), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="objective_glass",
    )
    right_body.visual(
        Cylinder(radius=0.0165, length=0.014),
        origin=Origin(xyz=(0.040, -0.072, -0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="eyecup_body",
    )
    right_body.visual(
        Cylinder(radius=0.0185, length=0.008),
        origin=Origin(xyz=(0.040, -0.077, -0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=armor_dark,
        name="eyecup_lip",
    )
    right_body.visual(
        Cylinder(radius=0.0125, length=0.0012),
        origin=Origin(xyz=(0.040, -0.067, -0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="ocular_glass",
    )
    right_body.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(xyz=(0.010, -0.010, 0.011), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="bridge_arm",
    )
    right_body.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.004, 0.000, -0.004)),
        material=metal,
        name="hinge_knuckle",
    )
    right_body.visual(
        Box((0.006, 0.010, 0.006)),
        origin=Origin(xyz=(0.007, -0.003, 0.004)),
        material=metal,
        name="hinge_web",
    )
    right_body.visual(
        Box((0.008, 0.018, 0.006)),
        origin=Origin(xyz=(0.072, -0.006, 0.002)),
        material=armor_dark,
        name="lever_plate",
    )
    right_body.visual(
        Box((0.003, 0.014, 0.014)),
        origin=Origin(xyz=(0.0665, -0.006, 0.011)),
        material=metal,
        name="lever_left_cheek",
    )
    right_body.visual(
        Box((0.003, 0.014, 0.014)),
        origin=Origin(xyz=(0.0755, -0.006, 0.011)),
        material=metal,
        name="lever_right_cheek",
    )
    right_body.inertial = Inertial.from_geometry(
        Box((0.092, 0.162, 0.078)),
        mass=0.40,
        origin=Origin(xyz=(0.040, 0.000, 0.000)),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor_dark,
        name="wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="wheel_band",
    )
    focus_knob.visual(
        Box((0.004, 0.003, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0115)),
        material=metal,
        name="focus_indicator",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Box((0.014, 0.028, 0.028)),
        mass=0.03,
    )

    stabilizer_lever = model.part("stabilizer_lever")
    stabilizer_lever.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="lever_pivot",
    )
    stabilizer_lever.visual(
        Box((0.006, 0.010, 0.004)),
        origin=Origin(xyz=(0.000, 0.005, 0.002)),
        material=metal,
        name="lever_arm",
    )
    stabilizer_lever.visual(
        Box((0.012, 0.014, 0.004)),
        origin=Origin(xyz=(0.000, 0.010, 0.004)),
        material=accent,
        name="switch_paddle",
    )
    stabilizer_lever.visual(
        Box((0.010, 0.006, 0.004)),
        origin=Origin(xyz=(0.000, 0.015, 0.007)),
        material=armor_dark,
        name="switch_tab",
    )
    stabilizer_lever.inertial = Inertial.from_geometry(
        Box((0.014, 0.022, 0.016)),
        mass=0.01,
        origin=Origin(xyz=(0.000, 0.006, 0.008)),
    )

    model.articulation(
        "center_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.0,
            lower=-0.28,
            upper=0.08,
        ),
    )
    model.articulation(
        "focus_knob_joint",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_knob,
        origin=Origin(xyz=(0.000, -0.015, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "stabilizer_lever_joint",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=stabilizer_lever,
        origin=Origin(xyz=(0.071, -0.006, 0.011)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=0.0,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_knob = object_model.get_part("focus_knob")
    stabilizer_lever = object_model.get_part("stabilizer_lever")

    center_hinge = object_model.get_articulation("center_hinge")
    focus_joint = object_model.get_articulation("focus_knob_joint")
    lever_joint = object_model.get_articulation("stabilizer_lever_joint")

    left_hinge = left_body.get_visual("hinge_knuckle")
    right_hinge = right_body.get_visual("hinge_knuckle")
    knob_wheel = focus_knob.get_visual("wheel")
    left_focus_cheek = left_body.get_visual("focus_left_cheek")
    lever_pivot = stabilizer_lever.get_visual("lever_pivot")
    lever_cheek = right_body.get_visual("lever_left_cheek")

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

    ctx.check(
        "center_hinge_axis_is_vertical",
        tuple(center_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {center_hinge.axis}",
    )
    ctx.check(
        "focus_axis_is_transverse",
        tuple(focus_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {focus_joint.axis}",
    )
    ctx.check(
        "stabilizer_axis_is_transverse",
        tuple(lever_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {lever_joint.axis}",
    )

    ctx.expect_contact(left_body, right_body, elem_a=left_hinge, elem_b=right_hinge)
    ctx.expect_contact(focus_knob, left_body, elem_a=knob_wheel, elem_b=left_focus_cheek)
    ctx.expect_contact(stabilizer_lever, right_body, elem_a=lever_pivot, elem_b=lever_cheek)

    right_objective_rest = ctx.part_element_world_aabb(right_body, elem="objective_bezel")
    knob_indicator_rest = ctx.part_element_world_aabb(focus_knob, elem="focus_indicator")
    lever_paddle_rest = ctx.part_element_world_aabb(stabilizer_lever, elem="switch_paddle")

    assert right_objective_rest is not None
    assert knob_indicator_rest is not None
    assert lever_paddle_rest is not None

    with ctx.pose({center_hinge: -0.22}):
        right_objective_open = ctx.part_element_world_aabb(right_body, elem="objective_bezel")
        assert right_objective_open is not None
        ctx.check(
            "center_hinge_opens_right_barrel_outward",
            right_objective_open[1][0] > right_objective_rest[1][0] + 0.010,
            details=(
                f"rest max x={right_objective_rest[1][0]:.4f}, "
                f"open max x={right_objective_open[1][0]:.4f}"
            ),
        )
        ctx.expect_contact(left_body, right_body, elem_a=left_hinge, elem_b=right_hinge)

    with ctx.pose({focus_joint: 1.1}):
        knob_indicator_turned = ctx.part_element_world_aabb(focus_knob, elem="focus_indicator")
        assert knob_indicator_turned is not None
        ctx.check(
            "focus_knob_rotates_indicator",
            knob_indicator_turned[0][1] < knob_indicator_rest[0][1] - 0.006,
            details=(
                f"rest min y={knob_indicator_rest[0][1]:.4f}, "
                f"turned min y={knob_indicator_turned[0][1]:.4f}"
            ),
        )
        ctx.expect_contact(focus_knob, left_body, elem_a=knob_wheel, elem_b=left_focus_cheek)

    with ctx.pose({lever_joint: 0.34}):
        lever_paddle_on = ctx.part_element_world_aabb(stabilizer_lever, elem="switch_paddle")
        assert lever_paddle_on is not None
        ctx.check(
            "stabilizer_lever_lifts_when_engaged",
            lever_paddle_on[1][2] > lever_paddle_rest[1][2] + 0.003,
            details=(
                f"rest max z={lever_paddle_rest[1][2]:.4f}, "
                f"engaged max z={lever_paddle_on[1][2]:.4f}"
            ),
        )
        ctx.expect_contact(stabilizer_lever, right_body, elem_a=lever_pivot, elem_b=lever_cheek)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
