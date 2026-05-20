from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.090
BASE_WIDTH = 0.080
BASE_HEIGHT = 0.070
BASE_FRONT_X = BASE_LENGTH / 2.0

GUIDE_BORE_RADIUS = 0.017

REAR_HEAD_LENGTH = 0.018
REAR_SHAFT_LENGTH = 0.165
REAR_SECTION_LENGTH = REAR_HEAD_LENGTH + REAR_SHAFT_LENGTH

MIDDLE_HEAD_LENGTH = 0.012
MIDDLE_SHAFT_LENGTH = 0.146
MIDDLE_SECTION_LENGTH = MIDDLE_HEAD_LENGTH + MIDDLE_SHAFT_LENGTH

FRONT_HEAD_LENGTH = 0.010
FRONT_SHAFT_LENGTH = 0.135
FRONT_SECTION_LENGTH = FRONT_HEAD_LENGTH + FRONT_SHAFT_LENGTH


def cylinder_x(radius: float, x0: float, x1: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(x1 - x0)
        .translate((x0, 0.0, 0.0))
    )


def annulus_x(outer_radius: float, inner_radius: float, x0: float, x1: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .sketch()
        .circle(outer_radius)
        .circle(inner_radius, mode="s")
        .finalize()
        .extrude(x1 - x0)
        .translate((x0, 0.0, 0.0))
    )


def tube_x(outer_radius: float, inner_radius: float, x0: float, x1: float) -> cq.Workplane:
    return annulus_x(outer_radius, inner_radius, x0, x1)


def box_x(length: float, width: float, height: float, *, center_x: float, center_y: float = 0.0, center_z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((center_x, center_y, center_z))
    )


def lower_half_tube_x(
    outer_radius: float,
    inner_radius: float,
    x0: float,
    x1: float,
    *,
    top_z: float = 0.0,
) -> cq.Workplane:
    keep_height = outer_radius + 0.004
    keep_center_z = top_z - 0.5 * keep_height
    keep = box_x(
        x1 - x0 + 0.002,
        2.0 * outer_radius + 0.004,
        keep_height,
        center_x=0.5 * (x0 + x1),
        center_z=keep_center_z,
    )
    return tube_x(outer_radius, inner_radius, x0, x1).intersect(keep)


def stepped_rod_x(head_radius: float, head_length: float, shaft_radius: float, shaft_length: float) -> cq.Workplane:
    return cylinder_x(head_radius, 0.0, head_length).union(
        cylinder_x(shaft_radius, head_length, head_length + shaft_length)
    )


def x_extent(aabb) -> float:
    return aabb[1][0] - aabb[0][0]


def radial_extent(aabb) -> float:
    return max(aabb[1][1] - aabb[0][1], aabb[1][2] - aabb[0][2])


def build_base_block() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .edges("|X")
        .fillet(0.004)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .circle(GUIDE_BORE_RADIUS)
        .cutThruAll()
    )


def build_outer_guide() -> cq.Workplane:
    collar = lower_half_tube_x(0.029, 0.013, 0.000, 0.025, top_z=-0.013)
    body = lower_half_tube_x(0.023, 0.013, 0.025, 0.185, top_z=-0.013)
    return collar.union(body)


def build_middle_guide() -> cq.Workplane:
    collar = lower_half_tube_x(0.018, 0.0104, 0.085, 0.105)
    body = lower_half_tube_x(0.016, 0.0104, 0.105, 0.330)
    return collar.union(body)


def build_front_guide() -> cq.Workplane:
    collar = lower_half_tube_x(0.013, 0.0088, 0.310, 0.325)
    body = lower_half_tube_x(0.011, 0.0088, 0.325, 0.600)
    return collar.union(body)


def build_rear_section() -> cq.Workplane:
    return stepped_rod_x(0.013, REAR_HEAD_LENGTH, 0.010, REAR_SHAFT_LENGTH)


def build_middle_section() -> cq.Workplane:
    return stepped_rod_x(0.0085, MIDDLE_HEAD_LENGTH, 0.0065, MIDDLE_SHAFT_LENGTH)


def build_front_section() -> cq.Workplane:
    return stepped_rod_x(0.0052, FRONT_HEAD_LENGTH, 0.0035, FRONT_SHAFT_LENGTH)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ejector_plunger_stack")

    model.material("base_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("guide_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("rod_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("tip_steel", rgba=(0.56, 0.59, 0.63, 1.0))

    base = model.part("base_block")
    base.visual(
        mesh_from_cadquery(build_base_block(), "base_block"),
        material="base_steel",
        name="base_body",
    )

    guides = model.part("guide_stack")
    guides.visual(
        mesh_from_cadquery(build_outer_guide(), "outer_guide"),
        material="guide_steel",
        name="outer_guide",
    )
    guides.visual(
        mesh_from_cadquery(build_middle_guide(), "middle_guide"),
        material="guide_steel",
        name="middle_guide",
    )
    guides.visual(
        mesh_from_cadquery(build_front_guide(), "front_guide"),
        material="guide_steel",
        name="front_guide",
    )

    rear = model.part("rear_rod_section")
    rear.visual(
        mesh_from_cadquery(build_rear_section(), "rear_rod_section"),
        material="rod_steel",
        name="rear_section",
    )

    middle = model.part("middle_rod_section")
    middle.visual(
        mesh_from_cadquery(build_middle_section(), "middle_rod_section"),
        material="rod_steel",
        name="middle_section",
    )

    front = model.part("front_rod_section")
    front.visual(
        mesh_from_cadquery(build_front_section(), "front_rod_section"),
        material="tip_steel",
        name="front_section",
    )

    model.articulation(
        "guide_mount",
        ArticulationType.FIXED,
        parent=base,
        child=guides,
        origin=Origin(xyz=(BASE_FRONT_X, 0.0, 0.0)),
    )
    model.articulation(
        "rear_slide",
        ArticulationType.PRISMATIC,
        parent=guides,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.25,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=rear,
        child=middle,
        origin=Origin(xyz=(0.183, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "front_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=front,
        origin=Origin(xyz=(0.158, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_block")
    guides = object_model.get_part("guide_stack")
    rear = object_model.get_part("rear_rod_section")
    middle = object_model.get_part("middle_rod_section")
    front = object_model.get_part("front_rod_section")

    guide_mount = object_model.get_articulation("guide_mount")
    rear_slide = object_model.get_articulation("rear_slide")
    middle_slide = object_model.get_articulation("middle_slide")
    front_slide = object_model.get_articulation("front_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=1e-5)
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
        base,
        guides,
        elem_a="base_body",
        elem_b="outer_guide",
        name="guide_stack_seated_on_base",
    )
    ctx.expect_contact(
        guides,
        rear,
        contact_tol=1e-5,
        elem_a="outer_guide",
        elem_b="rear_section",
        name="rear_section_seats_on_outer_guide",
    )
    ctx.expect_contact(
        rear,
        middle,
        elem_a="rear_section",
        elem_b="middle_section",
        name="middle_section_seats_inside_rear_section",
    )
    ctx.expect_contact(
        middle,
        front,
        elem_a="middle_section",
        elem_b="front_section",
        name="front_section_seats_inside_middle_section",
    )

    all_parts_present = all(
        part is not None for part in (base, guides, rear, middle, front)
    )
    ctx.check("all_parts_present", all_parts_present, "One or more prompt-critical parts are missing.")

    slides = (rear_slide, middle_slide, front_slide)
    joints_ok = all(
        joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and joint.motion_limits.upper > 0.0
        for joint in slides
    )
    ctx.check(
        "rod_sections_use_coaxial_prismatic_joints",
        joints_ok,
        "Rear, middle, and front rod sections must each translate on the common +X plunger axis.",
    )

    outer_guide_aabb = ctx.part_element_world_aabb(guides, elem="outer_guide")
    middle_guide_aabb = ctx.part_element_world_aabb(guides, elem="middle_guide")
    front_guide_aabb = ctx.part_element_world_aabb(guides, elem="front_guide")
    rear_aabb = ctx.part_element_world_aabb(rear, elem="rear_section")
    middle_aabb = ctx.part_element_world_aabb(middle, elem="middle_section")
    front_aabb = ctx.part_element_world_aabb(front, elem="front_section")

    guide_lengths_ok = (
        outer_guide_aabb is not None
        and middle_guide_aabb is not None
        and front_guide_aabb is not None
        and x_extent(outer_guide_aabb) < x_extent(middle_guide_aabb) < x_extent(front_guide_aabb)
    )
    ctx.check(
        "guide_bodies_read_as_serially_nested",
        guide_lengths_ok,
        "Guide sleeves should visibly step forward in length from outer to front guide.",
    )

    guide_diameters_ok = (
        outer_guide_aabb is not None
        and middle_guide_aabb is not None
        and front_guide_aabb is not None
        and radial_extent(outer_guide_aabb) > radial_extent(middle_guide_aabb) > radial_extent(front_guide_aabb)
    )
    ctx.check(
        "guide_diameters_step_down",
        guide_diameters_ok,
        "Guide diameters should decrease toward the tip.",
    )

    rod_diameters_ok = (
        rear_aabb is not None
        and middle_aabb is not None
        and front_aabb is not None
        and radial_extent(rear_aabb) > radial_extent(middle_aabb) > radial_extent(front_aabb)
    )
    ctx.check(
        "rod_diameters_step_down_toward_tip",
        rod_diameters_ok,
        "Rear, middle, and front rod sections should step down in diameter toward the tip.",
    )

    rear_rest_x = ctx.part_world_position(rear)[0]
    middle_rest_x = ctx.part_world_position(middle)[0]
    front_rest_x = ctx.part_world_position(front)[0]

    with ctx.pose({rear_slide: 0.040}):
        rear_moved_x = ctx.part_world_position(rear)[0]
    with ctx.pose({middle_slide: 0.030}):
        middle_moved_x = ctx.part_world_position(middle)[0]
    with ctx.pose({front_slide: 0.025}):
        front_moved_x = ctx.part_world_position(front)[0]

    ctx.check(
        "rear_section_translates_forward",
        rear_moved_x > rear_rest_x + 0.020,
        "Rear rod section did not advance along the plunger axis.",
    )
    ctx.check(
        "middle_section_translates_forward",
        middle_moved_x > middle_rest_x + 0.015,
        "Middle rod section did not advance along the plunger axis.",
    )
    ctx.check(
        "front_section_translates_forward",
        front_moved_x > front_rest_x + 0.012,
        "Front rod section did not advance along the plunger axis.",
    )

    with ctx.pose({rear_slide: 0.040, middle_slide: 0.030, front_slide: 0.025}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_extended_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
