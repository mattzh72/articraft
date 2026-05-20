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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_prism_binocular")

    armor = model.material("armor_green", rgba=(0.24, 0.29, 0.23, 1.0))
    bridge_black = model.material("bridge_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal = model.material("hinge_metal", rgba=(0.34, 0.36, 0.38, 1.0))

    barrel_offset_x = 0.038
    body_y_center = 0.012
    eyepiece_y = -0.046
    eyecup_front_y = -0.030
    objective_y = 0.078

    def barrel_section(y: float, width: float, height: float, exponent: float) -> list[tuple[float, float, float]]:
        return [(px, body_y_center + y, pz) for px, pz in superellipse_profile(width, height, exponent, segments=44)]

    barrel_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                barrel_section(-0.044, 0.036, 0.044, 2.5),
                barrel_section(-0.016, 0.039, 0.049, 2.8),
                barrel_section(0.014, 0.045, 0.058, 3.3),
                barrel_section(0.046, 0.043, 0.054, 3.1),
                barrel_section(0.060, 0.040, 0.046, 2.6),
            ]
        ),
        "binocular_barrel_shell",
    )

    center_bridge = model.part("center_bridge")
    center_bridge.visual(
        Cylinder(radius=0.004, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="hinge_spindle",
    )
    center_bridge.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bridge_black,
        name="hinge_hub",
    )
    center_bridge.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=metal,
        name="left_hinge_plate",
    )
    center_bridge.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=metal,
        name="right_hinge_plate",
    )
    center_bridge.visual(
        Box((0.020, 0.022, 0.011)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=bridge_black,
        name="focus_saddle",
    )
    center_bridge.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.055)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    def add_barrel(part_name: str, x_sign: float, collar_z: float, arm_z: float):
        barrel = model.part(part_name)
        barrel.visual(
            barrel_shell_mesh,
            origin=Origin(xyz=(x_sign * barrel_offset_x, 0.0, 0.0)),
            material=armor,
            name="barrel_shell",
        )
        barrel.visual(
            Cylinder(radius=0.0245, length=0.014),
            origin=Origin(
                xyz=(x_sign * barrel_offset_x, objective_y, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name="objective_ring",
        )
        barrel.visual(
            Cylinder(radius=0.0152, length=0.034),
            origin=Origin(
                xyz=(x_sign * barrel_offset_x, eyepiece_y, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=bridge_black,
            name="eyepiece_housing",
        )
        barrel.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(
                xyz=(x_sign * barrel_offset_x, -0.031, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=bridge_black,
            name="eyepiece_shoulder",
        )
        barrel.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, collar_z)),
            material=metal,
            name="hinge_washer",
        )
        barrel.visual(
            Box((0.030, 0.016, 0.008)),
            origin=Origin(xyz=(x_sign * 0.019, 0.0, arm_z)),
            material=bridge_black,
            name="bridge_arm",
        )
        barrel.inertial = Inertial.from_geometry(
            Box((0.056, 0.145, 0.062)),
            mass=0.34,
            origin=Origin(xyz=(x_sign * barrel_offset_x, 0.012, 0.0)),
        )
        return barrel

    left_barrel = add_barrel("left_barrel", -1.0, -0.010, -0.010)
    right_barrel = add_barrel("right_barrel", 1.0, 0.010, 0.010)

    model.articulation(
        "center_to_left_barrel",
        ArticulationType.FIXED,
        parent=center_bridge,
        child=left_barrel,
        origin=Origin(),
    )
    model.articulation(
        "center_to_right_barrel",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=right_barrel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=0.0,
            upper=0.18,
        ),
    )

    left_eyecup = model.part("left_eyecup")
    left_eyecup.visual(
        Cylinder(radius=0.0184, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup_shell",
    )
    left_eyecup.visual(
        Cylinder(radius=0.0215, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.018, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup_rim",
    )
    left_eyecup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0215, length=0.024),
        mass=0.015,
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
    )

    right_eyecup = model.part("right_eyecup")
    right_eyecup.visual(
        Cylinder(radius=0.0184, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup_shell",
    )
    right_eyecup.visual(
        Cylinder(radius=0.0215, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.018, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup_rim",
    )
    right_eyecup.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0215, length=0.024),
        mass=0.015,
        origin=Origin(
            xyz=(0.0, -0.012, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "left_eyecup_slide",
        ArticulationType.PRISMATIC,
        parent=left_barrel,
        child=left_eyecup,
        origin=Origin(xyz=(-barrel_offset_x, eyecup_front_y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.012,
        ),
    )
    model.articulation(
        "right_eyecup_slide",
        ArticulationType.PRISMATIC,
        parent=right_barrel,
        child=right_eyecup,
        origin=Origin(xyz=(barrel_offset_x, eyecup_front_y, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=0.012,
        ),
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="focus_wheel",
    )
    focus_knob.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_black,
        name="focus_core",
    )
    focus_knob.visual(
        Box((0.018, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=bridge_black,
        name="focus_tab",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.020),
        mass=0.025,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "center_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=center_bridge,
        child=focus_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=8.0,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    center_bridge = object_model.get_part("center_bridge")
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    left_eyecup = object_model.get_part("left_eyecup")
    right_eyecup = object_model.get_part("right_eyecup")
    focus_knob = object_model.get_part("focus_knob")

    barrel_hinge = object_model.get_articulation("center_to_right_barrel")
    left_eyecup_slide = object_model.get_articulation("left_eyecup_slide")
    right_eyecup_slide = object_model.get_articulation("right_eyecup_slide")
    focus_joint = object_model.get_articulation("center_to_focus_knob")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0008)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        left_barrel,
        left_eyecup,
        reason="Rubber eyecup is modeled as a telescoping sleeve nested over the eyepiece housing.",
    )
    ctx.allow_overlap(
        right_barrel,
        right_eyecup,
        reason="Rubber eyecup is modeled as a telescoping sleeve nested over the eyepiece housing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        center_bridge,
        left_barrel,
        elem_a="hinge_spindle",
        elem_b="hinge_washer",
        contact_tol=0.0008,
    )
    ctx.expect_contact(
        center_bridge,
        right_barrel,
        elem_a="hinge_spindle",
        elem_b="hinge_washer",
        contact_tol=0.0008,
    )
    ctx.expect_contact(
        left_eyecup,
        left_barrel,
        elem_a="eyecup_shell",
        elem_b="eyepiece_housing",
        contact_tol=0.0008,
    )
    ctx.expect_contact(
        right_eyecup,
        right_barrel,
        elem_a="eyecup_shell",
        elem_b="eyepiece_housing",
        contact_tol=0.0008,
    )
    ctx.expect_gap(
        focus_knob,
        center_bridge,
        axis="z",
        positive_elem="focus_wheel",
        negative_elem="focus_saddle",
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        focus_knob,
        center_bridge,
        axes="xy",
        elem_a="focus_wheel",
        elem_b="focus_saddle",
        min_overlap=0.010,
    )

    left_objective_aabb = ctx.part_element_world_aabb(left_barrel, elem="objective_ring")
    left_eyepiece_aabb = ctx.part_element_world_aabb(left_barrel, elem="eyepiece_housing")
    right_objective_aabb = ctx.part_element_world_aabb(right_barrel, elem="objective_ring")

    if left_objective_aabb is None or left_eyepiece_aabb is None:
        ctx.fail("left_barrel_measurement_ready", "Missing left barrel visual bounds.")
    else:
        barrel_length = left_objective_aabb[1][1] - left_eyepiece_aabb[0][1]
        ctx.check(
            "barrel_length_realistic",
            0.14 <= barrel_length <= 0.17,
            f"Measured barrel span {barrel_length:.4f} m.",
        )

    if left_objective_aabb is None or right_objective_aabb is None:
        ctx.fail("rest_width_measurement_ready", "Missing objective-ring bounds.")
    else:
        body_width = right_objective_aabb[1][0] - left_objective_aabb[0][0]
        ctx.check(
            "rest_width_realistic",
            0.11 <= body_width <= 0.15,
            f"Measured binocular width {body_width:.4f} m.",
        )

    right_rest = ctx.part_element_world_aabb(right_barrel, elem="objective_ring")
    if right_rest is None:
        ctx.fail("barrel_hinge_changes_spacing", "Missing right objective-ring bounds at rest.")
    else:
        rest_center_x = (right_rest[0][0] + right_rest[1][0]) * 0.5
        with ctx.pose({barrel_hinge: 0.16}):
            right_closed = ctx.part_element_world_aabb(right_barrel, elem="objective_ring")
            if right_closed is None:
                ctx.fail("barrel_hinge_changes_spacing", "Missing right objective-ring bounds at hinge pose.")
            else:
                closed_center_x = (right_closed[0][0] + right_closed[1][0]) * 0.5
                ctx.check(
                    "barrel_hinge_changes_spacing",
                    closed_center_x < rest_center_x - 0.006,
                    f"rest_x={rest_center_x:.4f}, closed_x={closed_center_x:.4f}",
                )
            ctx.expect_contact(
                center_bridge,
                right_barrel,
                elem_a="hinge_spindle",
                elem_b="hinge_washer",
                contact_tol=0.0008,
                name="barrel_hinge_closed_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_hinge_closed_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name="barrel_hinge_closed_no_floating")
        with ctx.pose({barrel_hinge: 0.0}):
            ctx.expect_contact(
                center_bridge,
                right_barrel,
                elem_a="hinge_spindle",
                elem_b="hinge_washer",
                contact_tol=0.0008,
                name="barrel_hinge_open_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_hinge_open_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name="barrel_hinge_open_no_floating")

    left_cup_rest = ctx.part_element_world_aabb(left_eyecup, elem="eyecup_shell")
    if left_cup_rest is None:
        ctx.fail("left_eyecup_extends", "Missing left eyecup bounds at rest.")
    else:
        with ctx.pose({left_eyecup_slide: 0.012, right_eyecup_slide: 0.012}):
            left_cup_extended = ctx.part_element_world_aabb(left_eyecup, elem="eyecup_shell")
            if left_cup_extended is None:
                ctx.fail("left_eyecup_extends", "Missing left eyecup bounds when extended.")
            else:
                ctx.check(
                    "left_eyecup_extends",
                    left_cup_extended[0][1] < left_cup_rest[0][1] - 0.008,
                    f"rest_min_y={left_cup_rest[0][1]:.4f}, extended_min_y={left_cup_extended[0][1]:.4f}",
                )
            ctx.expect_contact(
                left_eyecup,
                left_barrel,
                elem_a="eyecup_shell",
                elem_b="eyepiece_housing",
                contact_tol=0.0008,
                name="left_eyecup_extended_contact",
            )
            ctx.expect_contact(
                right_eyecup,
                right_barrel,
                elem_a="eyecup_shell",
                elem_b="eyepiece_housing",
                contact_tol=0.0008,
                name="right_eyecup_extended_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="eyecups_extended_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name="eyecups_extended_no_floating")

    focus_tab_rest = ctx.part_element_world_aabb(focus_knob, elem="focus_tab")
    if focus_tab_rest is None:
        ctx.fail("focus_knob_rotates", "Missing focus-tab bounds at rest.")
    else:
        rest_center_y = (focus_tab_rest[0][1] + focus_tab_rest[1][1]) * 0.5
        with ctx.pose({focus_joint: 1.6}):
            focus_tab_rotated = ctx.part_element_world_aabb(focus_knob, elem="focus_tab")
            if focus_tab_rotated is None:
                ctx.fail("focus_knob_rotates", "Missing focus-tab bounds when rotated.")
            else:
                rotated_center_y = (focus_tab_rotated[0][1] + focus_tab_rotated[1][1]) * 0.5
                ctx.check(
                    "focus_knob_rotates",
                    abs(rotated_center_y - rest_center_y) > 0.008,
                    f"rest_y={rest_center_y:.4f}, rotated_y={rotated_center_y:.4f}",
                )
            ctx.expect_gap(
                focus_knob,
                center_bridge,
                axis="z",
                positive_elem="focus_wheel",
                negative_elem="focus_saddle",
                max_gap=0.001,
                max_penetration=0.0,
                name="focus_knob_rotated_seating",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_knob_rotated_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name="focus_knob_rotated_no_floating")

    for joint_name in (
        "center_to_right_barrel",
        "left_eyecup_slide",
        "right_eyecup_slide",
        "center_to_focus_knob",
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name=f"{joint_name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=0.0008, name=f"{joint_name}_upper_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
