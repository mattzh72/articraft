from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_cartesian_stage")

    base_paint = model.material("base_paint", rgba=(0.18, 0.20, 0.23, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    frame_aluminum = model.material("frame_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    carriage_aluminum = model.material("carriage_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    tool_blue = model.material("tool_blue", rgba=(0.18, 0.35, 0.75, 1.0))

    def add_box(part, name, size, center, material):
        return part.visual(
            Box(size),
            origin=Origin(xyz=center),
            material=material,
            name=name,
        )

    base_length = 0.70
    base_width = 0.44
    base_height = 0.04
    x_rail_length = 0.58
    x_rail_width = 0.04
    x_rail_height = 0.02
    x_rail_y = 0.16

    x_travel = 0.16
    y_travel = 0.06
    z_travel = 0.05

    base = model.part("base")
    add_box(base, "bed", (base_length, base_width, base_height), (0.0, 0.0, base_height / 2.0), base_paint)
    add_box(
        base,
        "x_left_rail",
        (x_rail_length, x_rail_width, x_rail_height),
        (0.0, -x_rail_y, base_height + x_rail_height / 2.0),
        rail_steel,
    )
    add_box(
        base,
        "x_right_rail",
        (x_rail_length, x_rail_width, x_rail_height),
        (0.0, x_rail_y, base_height + x_rail_height / 2.0),
        rail_steel,
    )
    base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, base_height + x_rail_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (base_height + x_rail_height) / 2.0)),
    )

    x_bridge = model.part("x_bridge")
    add_box(x_bridge, "left_shoe", (0.12, 0.06, 0.02), (0.0, -x_rail_y, 0.01), frame_aluminum)
    add_box(x_bridge, "right_shoe", (0.12, 0.06, 0.02), (0.0, x_rail_y, 0.01), frame_aluminum)
    add_box(x_bridge, "lower_tie", (0.06, 0.26, 0.04), (0.0, 0.0, 0.02), frame_aluminum)
    add_box(x_bridge, "left_leg", (0.08, 0.08, 0.32), (0.0, -x_rail_y, 0.18), frame_aluminum)
    add_box(x_bridge, "right_leg", (0.08, 0.08, 0.32), (0.0, x_rail_y, 0.18), frame_aluminum)
    add_box(x_bridge, "top_beam", (0.10, 0.38, 0.06), (0.0, 0.0, 0.37), frame_aluminum)
    add_box(x_bridge, "y_lower_rail", (0.018, 0.30, 0.018), (0.059, 0.0, 0.354), rail_steel)
    add_box(x_bridge, "y_upper_rail", (0.018, 0.30, 0.018), (0.059, 0.0, 0.386), rail_steel)
    x_bridge.inertial = Inertial.from_geometry(
        Box((0.12, 0.38, 0.40)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    y_carriage = model.part("y_carriage")
    add_box(y_carriage, "carriage_plate", (0.024, 0.18, 0.18), (0.012, 0.0, 0.0), carriage_aluminum)
    add_box(y_carriage, "z_mount", (0.04, 0.09, 0.26), (0.044, 0.0, -0.12), carriage_aluminum)
    add_box(y_carriage, "z_left_rail", (0.016, 0.022, 0.28), (0.072, -0.022, -0.12), rail_steel)
    add_box(y_carriage, "z_right_rail", (0.016, 0.022, 0.28), (0.072, 0.022, -0.12), rail_steel)
    y_carriage.inertial = Inertial.from_geometry(
        Box((0.08, 0.18, 0.35)),
        mass=2.5,
        origin=Origin(xyz=(0.04, 0.0, -0.085)),
    )

    z_slide = model.part("z_slide")
    add_box(z_slide, "slide_plate", (0.02, 0.09, 0.18), (0.01, 0.0, 0.0), carriage_aluminum)
    add_box(z_slide, "tool_plate", (0.012, 0.08, 0.08), (0.026, 0.0, -0.13), tool_blue)
    z_slide.inertial = Inertial.from_geometry(
        Box((0.032, 0.09, 0.26)),
        mass=1.2,
        origin=Origin(xyz=(0.016, 0.0, -0.05)),
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_bridge,
        origin=Origin(xyz=(0.0, 0.0, base_height + x_rail_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=-x_travel,
            upper=x_travel,
        ),
    )
    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_bridge,
        child=y_carriage,
        origin=Origin(xyz=(0.068, 0.0, 0.37)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-y_travel,
            upper=y_travel,
        ),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.08, 0.0, -0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=-z_travel,
            upper=z_travel,
        ),
    )

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_sweep_no_overlap")
    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")

    def require_part(name: str):
        try:
            return object_model.get_part(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"part_{name}_present", str(exc))
            return None

    def require_joint(name: str):
        try:
            return object_model.get_articulation(name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"joint_{name}_present", str(exc))
            return None

    def require_visual(part, visual_name: str):
        if part is None:
            ctx.fail(f"visual_{visual_name}_present", "owning part missing")
            return None
        try:
            return part.get_visual(visual_name)
        except Exception as exc:  # pragma: no cover - defensive authored test helper
            ctx.fail(f"visual_{part.name}_{visual_name}_present", str(exc))
            return None

    base = require_part("base")
    x_bridge = require_part("x_bridge")
    y_carriage = require_part("y_carriage")
    z_slide = require_part("z_slide")

    x_axis = require_joint("x_axis")
    y_axis = require_joint("y_axis")
    z_axis = require_joint("z_axis")

    x_left_rail = require_visual(base, "x_left_rail")
    x_right_rail = require_visual(base, "x_right_rail")
    left_shoe = require_visual(x_bridge, "left_shoe")
    right_shoe = require_visual(x_bridge, "right_shoe")
    y_lower_rail = require_visual(x_bridge, "y_lower_rail")
    y_upper_rail = require_visual(x_bridge, "y_upper_rail")
    carriage_plate = require_visual(y_carriage, "carriage_plate")
    z_left_rail = require_visual(y_carriage, "z_left_rail")
    z_right_rail = require_visual(y_carriage, "z_right_rail")
    slide_plate = require_visual(z_slide, "slide_plate")
    tool_plate = require_visual(z_slide, "tool_plate")

    if any(
        item is None
        for item in (
            base,
            x_bridge,
            y_carriage,
            z_slide,
            x_axis,
            y_axis,
            z_axis,
            x_left_rail,
            x_right_rail,
            left_shoe,
            right_shoe,
            y_lower_rail,
            y_upper_rail,
            carriage_plate,
            z_left_rail,
            z_right_rail,
            slide_plate,
            tool_plate,
        )
    ):
        return ctx.report()

    def check_prismatic_joint(name: str, joint, axis, lower: float, upper: float):
        limits = joint.motion_limits
        ok = (
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == axis
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - lower) <= 1e-9
            and abs(limits.upper - upper) <= 1e-9
        )
        ctx.check(
            f"{name}_joint_definition",
            ok,
            details=(
                f"type={joint.articulation_type}, axis={joint.axis}, "
                f"limits={None if limits is None else (limits.lower, limits.upper)}"
            ),
        )

    check_prismatic_joint("x_axis", x_axis, (1.0, 0.0, 0.0), -0.16, 0.16)
    check_prismatic_joint("y_axis", y_axis, (0.0, 1.0, 0.0), -0.06, 0.06)
    check_prismatic_joint("z_axis", z_axis, (0.0, 0.0, 1.0), -0.05, 0.05)

    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.03, name="joint_origins_near_guides")

    ctx.expect_contact(
        x_bridge,
        base,
        elem_a=left_shoe,
        elem_b=x_left_rail,
        contact_tol=1e-6,
        name="left_x_shoe_contacts_left_rail_at_rest",
    )
    ctx.expect_contact(
        x_bridge,
        base,
        elem_a=right_shoe,
        elem_b=x_right_rail,
        contact_tol=1e-6,
        name="right_x_shoe_contacts_right_rail_at_rest",
    )
    ctx.expect_overlap(
        x_bridge,
        base,
        axes="xy",
        min_overlap=0.03,
        elem_a=left_shoe,
        elem_b=x_left_rail,
        name="left_x_shoe_overlaps_left_rail_plan",
    )
    ctx.expect_overlap(
        x_bridge,
        base,
        axes="xy",
        min_overlap=0.03,
        elem_a=right_shoe,
        elem_b=x_right_rail,
        name="right_x_shoe_overlaps_right_rail_plan",
    )

    ctx.expect_contact(
        y_carriage,
        x_bridge,
        elem_a=carriage_plate,
        elem_b=y_lower_rail,
        contact_tol=1e-6,
        name="carriage_plate_contacts_lower_y_rail_at_rest",
    )
    ctx.expect_contact(
        y_carriage,
        x_bridge,
        elem_a=carriage_plate,
        elem_b=y_upper_rail,
        contact_tol=1e-6,
        name="carriage_plate_contacts_upper_y_rail_at_rest",
    )
    ctx.expect_overlap(
        y_carriage,
        x_bridge,
        axes="y",
        min_overlap=0.17,
        elem_a=carriage_plate,
        elem_b=y_lower_rail,
        name="carriage_plate_has_y_bearing_on_lower_rail_at_rest",
    )
    ctx.expect_overlap(
        y_carriage,
        x_bridge,
        axes="y",
        min_overlap=0.17,
        elem_a=carriage_plate,
        elem_b=y_upper_rail,
        name="carriage_plate_has_y_bearing_on_upper_rail_at_rest",
    )

    ctx.expect_contact(
        z_slide,
        y_carriage,
        elem_a=slide_plate,
        elem_b=z_left_rail,
        contact_tol=1e-6,
        name="slide_plate_contacts_left_z_rail_at_rest",
    )
    ctx.expect_contact(
        z_slide,
        y_carriage,
        elem_a=slide_plate,
        elem_b=z_right_rail,
        contact_tol=1e-6,
        name="slide_plate_contacts_right_z_rail_at_rest",
    )
    ctx.expect_overlap(
        z_slide,
        y_carriage,
        axes="z",
        min_overlap=0.18,
        elem_a=slide_plate,
        elem_b=z_left_rail,
        name="slide_plate_has_z_bearing_on_left_rail_at_rest",
    )
    ctx.expect_overlap(
        z_slide,
        y_carriage,
        axes="z",
        min_overlap=0.18,
        elem_a=slide_plate,
        elem_b=z_right_rail,
        name="slide_plate_has_z_bearing_on_right_rail_at_rest",
    )

    slide_aabb = ctx.part_element_world_aabb(z_slide, elem="slide_plate")
    tool_aabb = ctx.part_element_world_aabb(z_slide, elem="tool_plate")
    if slide_aabb is None or tool_aabb is None:
        ctx.fail("tool_plate_aabb_available", "could not resolve slide or tool plate bounds")
    else:
        slide_min, slide_max = slide_aabb
        tool_min, tool_max = tool_aabb
        slide_y_center = (slide_min[1] + slide_max[1]) / 2.0
        tool_y_center = (tool_min[1] + tool_max[1]) / 2.0
        ctx.check(
            "tool_plate_below_slide",
            tool_max[2] <= slide_min[2] + 1e-6,
            details=f"tool_top={tool_max[2]:.6f}, slide_bottom={slide_min[2]:.6f}",
        )
        ctx.check(
            "tool_plate_centered_on_y",
            abs(tool_y_center - slide_y_center) <= 1e-6,
            details=f"tool_y={tool_y_center:.6f}, slide_y={slide_y_center:.6f}",
        )

    def check_limit_pose(joint, value: float, label: str):
        with ctx.pose({joint: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")

    for label, value in (("x_lower", -0.16), ("x_upper", 0.16)):
        with ctx.pose({x_axis: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
            ctx.expect_contact(
                x_bridge,
                base,
                elem_a=left_shoe,
                elem_b=x_left_rail,
                contact_tol=1e-6,
                name=f"{label}_left_shoe_contact",
            )
            ctx.expect_contact(
                x_bridge,
                base,
                elem_a=right_shoe,
                elem_b=x_right_rail,
                contact_tol=1e-6,
                name=f"{label}_right_shoe_contact",
            )
            ctx.expect_overlap(
                x_bridge,
                base,
                axes="x",
                min_overlap=0.12,
                elem_a=left_shoe,
                elem_b=x_left_rail,
                name=f"{label}_left_shoe_stays_on_rail_length",
            )
            ctx.expect_overlap(
                x_bridge,
                base,
                axes="x",
                min_overlap=0.12,
                elem_a=right_shoe,
                elem_b=x_right_rail,
                name=f"{label}_right_shoe_stays_on_rail_length",
            )

    for label, value in (("y_lower", -0.06), ("y_upper", 0.06)):
        with ctx.pose({y_axis: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
            ctx.expect_contact(
                y_carriage,
                x_bridge,
                elem_a=carriage_plate,
                elem_b=y_lower_rail,
                contact_tol=1e-6,
                name=f"{label}_carriage_contacts_lower_rail",
            )
            ctx.expect_contact(
                y_carriage,
                x_bridge,
                elem_a=carriage_plate,
                elem_b=y_upper_rail,
                contact_tol=1e-6,
                name=f"{label}_carriage_contacts_upper_rail",
            )
            ctx.expect_overlap(
                y_carriage,
                x_bridge,
                axes="y",
                min_overlap=0.17,
                elem_a=carriage_plate,
                elem_b=y_lower_rail,
                name=f"{label}_carriage_keeps_lower_rail_engagement",
            )
            ctx.expect_overlap(
                y_carriage,
                x_bridge,
                axes="y",
                min_overlap=0.17,
                elem_a=carriage_plate,
                elem_b=y_upper_rail,
                name=f"{label}_carriage_keeps_upper_rail_engagement",
            )

    for label, value in (("z_lower", -0.05), ("z_upper", 0.05)):
        with ctx.pose({z_axis: value}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
            ctx.expect_contact(
                z_slide,
                y_carriage,
                elem_a=slide_plate,
                elem_b=z_left_rail,
                contact_tol=1e-6,
                name=f"{label}_slide_contacts_left_rail",
            )
            ctx.expect_contact(
                z_slide,
                y_carriage,
                elem_a=slide_plate,
                elem_b=z_right_rail,
                contact_tol=1e-6,
                name=f"{label}_slide_contacts_right_rail",
            )
            ctx.expect_overlap(
                z_slide,
                y_carriage,
                axes="z",
                min_overlap=0.17,
                elem_a=slide_plate,
                elem_b=z_left_rail,
                name=f"{label}_slide_keeps_left_rail_engagement",
            )
            ctx.expect_overlap(
                z_slide,
                y_carriage,
                axes="z",
                min_overlap=0.17,
                elem_a=slide_plate,
                elem_b=z_right_rail,
                name=f"{label}_slide_keeps_right_rail_engagement",
            )

    with ctx.pose({x_axis: 0.16, y_axis: -0.06, z_axis: 0.05}):
        ctx.fail_if_parts_overlap_in_current_pose(name="corner_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="corner_pose_no_floating")
        ctx.expect_contact(
            x_bridge,
            base,
            elem_a=left_shoe,
            elem_b=x_left_rail,
            contact_tol=1e-6,
            name="corner_pose_x_contact",
        )
        ctx.expect_contact(
            y_carriage,
            x_bridge,
            elem_a=carriage_plate,
            elem_b=y_upper_rail,
            contact_tol=1e-6,
            name="corner_pose_y_contact",
        )
        ctx.expect_contact(
            z_slide,
            y_carriage,
            elem_a=slide_plate,
            elem_b=z_left_rail,
            contact_tol=1e-6,
            name="corner_pose_z_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
