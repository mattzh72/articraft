from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_watch_winder_box")

    walnut = model.material("walnut", rgba=(0.34, 0.22, 0.12, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.24, 0.15, 0.08, 1.0))
    brass = model.material("aged_brass", rgba=(0.64, 0.55, 0.30, 1.0))
    steel = model.material("machined_steel", rgba=(0.57, 0.60, 0.64, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.17, 1.0))
    felt = model.material("green_felt", rgba=(0.08, 0.14, 0.10, 1.0))
    leather = model.material("tan_leather", rgba=(0.49, 0.33, 0.20, 1.0))

    case_body = model.part("case_body")
    case_body.visual(
        Box((0.280, 0.200, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=walnut_dark,
        name="floor_panel",
    )
    case_body.visual(
        Box((0.280, 0.012, 0.118)),
        origin=Origin(xyz=(0.0, 0.094, 0.071)),
        material=walnut,
        name="front_wall",
    )
    case_body.visual(
        Box((0.280, 0.012, 0.118)),
        origin=Origin(xyz=(0.0, -0.094, 0.071)),
        material=walnut,
        name="rear_wall",
    )
    case_body.visual(
        Box((0.012, 0.176, 0.118)),
        origin=Origin(xyz=(-0.134, 0.0, 0.071)),
        material=walnut,
        name="left_side_wall",
    )
    case_body.visual(
        Box((0.012, 0.176, 0.118)),
        origin=Origin(xyz=(0.134, 0.0, 0.071)),
        material=walnut,
        name="right_side_wall",
    )
    case_body.visual(
        Box((0.236, 0.156, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=felt,
        name="floor_liner",
    )
    case_body.visual(
        Box((0.016, 0.048, 0.090)),
        origin=Origin(xyz=(-0.083, 0.0, 0.057)),
        material=steel,
        name="left_bearing_block",
    )
    case_body.visual(
        Box((0.016, 0.048, 0.090)),
        origin=Origin(xyz=(0.083, 0.0, 0.057)),
        material=steel,
        name="right_bearing_block",
    )
    case_body.visual(
        Box((0.028, 0.014, 0.046)),
        origin=Origin(xyz=(-0.071, 0.032, 0.035)),
        material=black_oxide,
        name="left_front_gusset",
    )
    case_body.visual(
        Box((0.028, 0.014, 0.046)),
        origin=Origin(xyz=(-0.071, -0.032, 0.035)),
        material=black_oxide,
        name="left_rear_gusset",
    )
    case_body.visual(
        Box((0.028, 0.014, 0.046)),
        origin=Origin(xyz=(0.071, 0.032, 0.035)),
        material=black_oxide,
        name="right_front_gusset",
    )
    case_body.visual(
        Box((0.028, 0.014, 0.046)),
        origin=Origin(xyz=(0.071, -0.032, 0.035)),
        material=black_oxide,
        name="right_rear_gusset",
    )
    case_body.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(-0.085, -0.109, 0.125)),
        material=steel,
        name="left_hinge_pedestal",
    )
    case_body.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(0.085, -0.109, 0.125)),
        material=steel,
        name="right_hinge_pedestal",
    )
    case_body.visual(
        Box((0.214, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.105, 0.124)),
        material=black_oxide,
        name="hinge_backer_bar",
    )
    case_body.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(-0.118, -0.084, 0.124)),
        material=brass,
        name="left_rear_corner_cap",
    )
    case_body.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(0.118, -0.084, 0.124)),
        material=brass,
        name="right_rear_corner_cap",
    )
    case_body.visual(
        Box((0.018, 0.004, 0.098)),
        origin=Origin(xyz=(-0.129, 0.089, 0.061)),
        material=brass,
        name="front_left_vertical_strap",
    )
    case_body.visual(
        Box((0.018, 0.004, 0.098)),
        origin=Origin(xyz=(0.129, 0.089, 0.061)),
        material=brass,
        name="front_right_vertical_strap",
    )
    case_body.visual(
        Box((0.018, 0.004, 0.098)),
        origin=Origin(xyz=(-0.129, -0.089, 0.061)),
        material=brass,
        name="rear_left_vertical_strap",
    )
    case_body.visual(
        Box((0.018, 0.004, 0.098)),
        origin=Origin(xyz=(0.129, -0.089, 0.061)),
        material=brass,
        name="rear_right_vertical_strap",
    )
    case_body.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(-0.095, -0.065, -0.004)),
        material=black_oxide,
        name="left_rear_foot",
    )
    case_body.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.095, -0.065, -0.004)),
        material=black_oxide,
        name="right_rear_foot",
    )
    case_body.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(-0.095, 0.065, -0.004)),
        material=black_oxide,
        name="left_front_foot",
    )
    case_body.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(0.095, 0.065, -0.004)),
        material=black_oxide,
        name="right_front_foot",
    )
    case_body.inertial = Inertial.from_geometry(
        Box((0.280, 0.200, 0.138)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    left_service_hatch = model.part("left_service_hatch")
    left_service_hatch.visual(
        Box((0.003, 0.090, 0.060)),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=steel,
        name="hatch_plate",
    )
    left_service_hatch.visual(
        Box((0.002, 0.070, 0.040)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=black_oxide,
        name="adapter_frame",
    )
    for index, (y_pos, z_pos) in enumerate(
        ((-0.030, -0.018), (0.030, -0.018), (-0.030, 0.018), (0.030, 0.018))
    ):
        left_service_hatch.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(-0.005, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"bolt_{index:02d}",
        )
    left_service_hatch.inertial = Inertial.from_geometry(
        Box((0.010, 0.094, 0.064)),
        mass=0.15,
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
    )

    rear_service_hatch = model.part("rear_service_hatch")
    rear_service_hatch.visual(
        Box((0.120, 0.003, 0.050)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=steel,
        name="hatch_plate",
    )
    rear_service_hatch.visual(
        Box((0.092, 0.002, 0.030)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=black_oxide,
        name="service_frame",
    )
    for index, (x_pos, z_pos) in enumerate(
        ((-0.044, -0.015), (0.044, -0.015), (-0.044, 0.015), (0.044, 0.015))
    ):
        rear_service_hatch.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(x_pos, -0.005, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"bolt_{index:02d}",
        )
    rear_service_hatch.inertial = Inertial.from_geometry(
        Box((0.124, 0.010, 0.054)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    cradle_assembly = model.part("cradle_assembly")
    pillow_geom = CapsuleGeometry(radius=0.040, length=0.038)
    pillow_geom.rotate_y(math.pi / 2.0)
    pillow_mesh = mesh_from_geometry(pillow_geom, "watch_winder_pillow")
    cradle_assembly.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    cradle_assembly.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="left_drive_flange",
    )
    cradle_assembly.visual(
        Box((0.010, 0.052, 0.052)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=black_oxide,
        name="left_adapter_web",
    )
    cradle_assembly.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="center_shaft",
    )
    cradle_assembly.visual(
        pillow_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=leather,
        name="pillow_body",
    )
    cradle_assembly.visual(
        Box((0.094, 0.018, 0.008)),
        origin=Origin(xyz=(0.075, 0.028, 0.028)),
        material=leather,
        name="watch_retention_pad",
    )
    cradle_assembly.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="right_support_flange",
    )
    cradle_assembly.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.144, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        cradle_assembly.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(
                xyz=(
                    0.012,
                    0.014 * math.cos(angle),
                    0.014 * math.sin(angle),
                ),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"drive_bolt_{index:02d}",
        )
    cradle_assembly.inertial = Inertial.from_geometry(
        Box((0.150, 0.090, 0.090)),
        mass=0.65,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.304, 0.212, 0.012)),
        origin=Origin(xyz=(0.0, 0.106, 0.044)),
        material=walnut,
        name="top_panel",
    )
    lid.visual(
        Box((0.304, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, 0.206, 0.019)),
        material=walnut,
        name="front_rail",
    )
    lid.visual(
        Box((0.012, 0.200, 0.038)),
        origin=Origin(xyz=(-0.146, 0.100, 0.019)),
        material=walnut,
        name="left_rail",
    )
    lid.visual(
        Box((0.012, 0.200, 0.038)),
        origin=Origin(xyz=(0.146, 0.100, 0.019)),
        material=walnut,
        name="right_rail",
    )
    lid.visual(
        Box((0.252, 0.164, 0.004)),
        origin=Origin(xyz=(0.0, 0.106, 0.036)),
        material=felt,
        name="inner_liner",
    )
    lid.visual(
        Box((0.220, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.005, 0.018)),
        material=black_oxide,
        name="hinge_spine",
    )
    lid.visual(
        Box((0.238, 0.014, 0.021)),
        origin=Origin(xyz=(0.0, 0.008, 0.0285)),
        material=steel,
        name="rear_reinforcement_rail",
    )
    lid.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(-0.085, -0.006, 0.007)),
        material=steel,
        name="left_hinge_shoe",
    )
    lid.visual(
        Box((0.028, 0.012, 0.014)),
        origin=Origin(xyz=(0.085, -0.006, 0.007)),
        material=steel,
        name="right_hinge_shoe",
    )
    lid.visual(
        Box((0.024, 0.004, 0.082)),
        origin=Origin(xyz=(-0.140, 0.194, 0.049)),
        material=brass,
        name="front_left_lid_strap",
    )
    lid.visual(
        Box((0.024, 0.004, 0.082)),
        origin=Origin(xyz=(0.140, 0.194, 0.049)),
        material=brass,
        name="front_right_lid_strap",
    )
    lid.visual(
        Box((0.080, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.214, 0.012)),
        material=brass,
        name="lift_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.304, 0.220, 0.056)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.108, 0.028)),
    )

    model.articulation(
        "case_to_left_service_hatch",
        ArticulationType.FIXED,
        parent=case_body,
        child=left_service_hatch,
        origin=Origin(xyz=(-0.140, 0.020, 0.060)),
    )
    model.articulation(
        "case_to_rear_service_hatch",
        ArticulationType.FIXED,
        parent=case_body,
        child=rear_service_hatch,
        origin=Origin(xyz=(0.0, -0.100, 0.052)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case_body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.100, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=case_body,
        child=cradle_assembly,
        origin=Origin(xyz=(-0.075, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case_body = object_model.get_part("case_body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle_assembly")
    left_service_hatch = object_model.get_part("left_service_hatch")
    rear_service_hatch = object_model.get_part("rear_service_hatch")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

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

    ctx.check(
        "primary_articulations_are_horizontal_x_axes",
        tuple(round(v, 6) for v in lid_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 6) for v in cradle_spin.axis) == (1.0, 0.0, 0.0),
        "Both the hinged lid and cradle rotation should pivot about the horizontal X axis.",
    )

    with ctx.pose({lid_hinge: 0.0, cradle_spin: 0.0}):
        ctx.expect_contact(
            lid,
            case_body,
            elem_a="left_hinge_shoe",
            elem_b="left_hinge_pedestal",
            name="left_hinge_support_is_seated",
        )
        ctx.expect_contact(
            lid,
            case_body,
            elem_a="right_hinge_shoe",
            elem_b="right_hinge_pedestal",
            name="right_hinge_support_is_seated",
        )
        ctx.expect_contact(
            cradle,
            case_body,
            elem_a="left_trunnion",
            elem_b="left_bearing_block",
            name="left_cradle_support_contact",
        )
        ctx.expect_contact(
            cradle,
            case_body,
            elem_a="right_trunnion",
            elem_b="right_bearing_block",
            name="right_cradle_support_contact",
        )
        ctx.expect_contact(
            left_service_hatch,
            case_body,
            elem_a="hatch_plate",
            elem_b="left_side_wall",
            name="left_service_hatch_is_mounted",
        )
        ctx.expect_contact(
            rear_service_hatch,
            case_body,
            elem_a="hatch_plate",
            elem_b="rear_wall",
            name="rear_service_hatch_is_mounted",
        )
        ctx.expect_overlap(
            lid,
            case_body,
            axes="x",
            min_overlap=0.24,
            name="lid_spans_case_width",
        )

    with ctx.pose({lid_hinge: math.radians(78.0)}):
        front_rail_aabb = ctx.part_element_world_aabb(lid, elem="front_rail")
        case_aabb = ctx.part_world_aabb(case_body)
        lid_open_ok = (
            front_rail_aabb is not None
            and case_aabb is not None
            and front_rail_aabb[0][2] > case_aabb[1][2] + 0.055
        )
        ctx.check(
            "lid_opens_upward_clear_of_box",
            lid_open_ok,
            "At an open pose the lid front rail should lift well above the case.",
        )

    with ctx.pose({cradle_spin: math.pi / 2.0}):
        pillow_aabb = ctx.part_element_world_aabb(cradle, elem="pillow_body")
        left_block_aabb = ctx.part_element_world_aabb(case_body, elem="left_bearing_block")
        right_block_aabb = ctx.part_element_world_aabb(case_body, elem="right_bearing_block")
        cradle_clear = (
            pillow_aabb is not None
            and left_block_aabb is not None
            and right_block_aabb is not None
            and pillow_aabb[0][0] > left_block_aabb[1][0]
            and pillow_aabb[1][0] < right_block_aabb[0][0]
        )
        ctx.check(
            "rotating_pillow_stays_between_supports",
            cradle_clear,
            "The rotating cradle should remain bracketed between its two bearing blocks.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
