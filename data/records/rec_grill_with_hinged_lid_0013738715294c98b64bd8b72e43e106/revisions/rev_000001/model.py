from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


BODY_WIDTH = 0.420
BODY_DEPTH = 0.280
BODY_HEIGHT = 0.082
FEET_HEIGHT = 0.018
BODY_TOP_Z = FEET_HEIGHT + BODY_HEIGHT
HINGE_Y = -(BODY_DEPTH * 0.5) - 0.009
HINGE_Z = BODY_TOP_Z + 0.004


def _rounded_rect_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_y: float = 0.0,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, center_y + y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_firebox_shell_mesh():
    outer = BoxGeometry((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)).translate(
        0.0,
        0.0,
        FEET_HEIGHT + BODY_HEIGHT * 0.5,
    )
    inner = BoxGeometry((BODY_WIDTH - 0.012, BODY_DEPTH - 0.012, 0.078)).translate(
        0.0,
        0.0,
        0.065,
    )
    return boolean_difference(outer, inner)


def _build_lid_shell_mesh():
    outer_sections = [
        _rounded_rect_section(0.432, 0.286, 0.034, -0.004, center_y=0.155),
        _rounded_rect_section(0.426, 0.276, 0.042, 0.028, center_y=0.150),
        _rounded_rect_section(0.408, 0.238, 0.060, 0.055, center_y=0.140),
        _rounded_rect_section(0.378, 0.192, 0.072, 0.080, center_y=0.128),
    ]
    return repair_loft(section_loft(outer_sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_hibachi_grill")

    enamel_black = model.material("enamel_black", rgba=(0.11, 0.11, 0.12, 1.0))
    graphite_lid = model.material("graphite_lid", rgba=(0.26, 0.27, 0.29, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    firebox = model.part("firebox")
    firebox.visual(
        mesh_from_geometry(_build_firebox_shell_mesh(), "firebox_shell"),
        material=enamel_black,
        name="firebox_shell",
    )

    foot_centers = (
        (-0.160, -0.105),
        (-0.160, 0.105),
        (0.160, -0.105),
        (0.160, 0.105),
    )
    for index, (x_pos, y_pos) in enumerate(foot_centers, start=1):
        firebox.visual(
            Box((0.042, 0.030, FEET_HEIGHT)),
            origin=Origin(xyz=(x_pos, y_pos, FEET_HEIGHT * 0.5)),
            material=dark_steel,
            name=f"foot_{index}",
        )

    for side, x_pos in (("left", -0.1275), ("right", 0.1275)):
        firebox.visual(
            Box((0.028, 0.010, 0.016)),
            origin=Origin(xyz=(x_pos, HINGE_Y + 0.0045, HINGE_Z - 0.016)),
            material=dark_steel,
            name=f"{side}_hinge_mount",
        )
        firebox.visual(
            Cylinder(radius=0.008, length=0.105),
            origin=Origin(
                xyz=(x_pos, HINGE_Y, HINGE_Z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"{side}_hinge_barrel",
        )

    firebox.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP_Z)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell_mesh(), "lid_shell"),
        material=graphite_lid,
        name="lid_shell",
    )
    lid.visual(
        Box((0.144, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.008, 0.009)),
        material=dark_steel,
        name="lid_hinge_strap",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.150),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(-0.055, 0.142, 0.055)),
        material=steel,
        name="handle_post_left",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.055, 0.142, 0.055)),
        material=steel,
        name="handle_post_right",
    )
    lid.visual(
        Cylinder(radius=0.0055, length=0.120),
        origin=Origin(
            xyz=(0.0, 0.142, 0.072),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )
    for side, x_pos in (("left", -0.024), ("right", 0.024)):
        lid.visual(
            Box((0.014, 0.018, 0.016)),
            origin=Origin(xyz=(x_pos, 0.307, 0.008)),
            material=dark_steel,
            name=f"{side}_latch_ear",
        )
        lid.visual(
            Cylinder(radius=0.0046, length=0.016),
            origin=Origin(
                xyz=(x_pos, 0.309, 0.008),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"{side}_latch_knuckle",
        )

    lid.inertial = Inertial.from_geometry(
        Box((0.432, 0.286, 0.086)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.143, 0.036)),
    )

    latch_tab = model.part("latch_tab")
    latch_tab.visual(
        Cylinder(radius=0.0042, length=0.040),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="latch_pivot",
    )
    latch_tab.visual(
        Box((0.026, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, 0.004, -0.024)),
        material=steel,
        name="latch_blade",
    )
    latch_tab.visual(
        Box((0.030, 0.014, 0.007)),
        origin=Origin(xyz=(0.0, 0.012, -0.050)),
        material=steel,
        name="latch_pull",
    )
    latch_tab.inertial = Inertial.from_geometry(
        Box((0.036, 0.018, 0.058)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.008, -0.029)),
    )

    model.articulation(
        "firebox_to_lid",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "lid_to_latch_tab",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch_tab,
        origin=Origin(xyz=(0.0, 0.309, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    firebox = object_model.get_part("firebox")
    lid = object_model.get_part("lid")
    latch_tab = object_model.get_part("latch_tab")
    lid_hinge = object_model.get_articulation("firebox_to_lid")
    latch_hinge = object_model.get_articulation("lid_to_latch_tab")

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
    ctx.allow_overlap(
        latch_tab,
        lid,
        elem_a="latch_pivot",
        elem_b="left_latch_knuckle",
        reason="The latch pivot pin is captured through the left lid knuckle.",
    )
    ctx.allow_overlap(
        latch_tab,
        lid,
        elem_a="latch_pivot",
        elem_b="right_latch_knuckle",
        reason="The latch pivot pin is captured through the right lid knuckle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(lid, firebox, name="lid_contacts_firebox_when_closed")
    ctx.expect_contact(latch_tab, lid, name="latch_contacts_lid_at_pivot")
    ctx.expect_gap(
        lid,
        firebox,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="firebox_shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="lid_shell_seats_on_firebox_rim",
    )
    ctx.expect_overlap(
        lid,
        firebox,
        axes="xy",
        elem_a="lid_shell",
        elem_b="firebox_shell",
        min_overlap=0.240,
        name="lid_spans_firebox_opening",
    )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_rest_ok = ctx.check(
        "lid_shell_rest_aabb_available",
        lid_rest_aabb is not None,
        "Could not resolve lid shell AABB in the rest pose.",
    )
    if lid_rest_ok and lid_rest_aabb is not None:
        with ctx.pose({lid_hinge: math.radians(80.0)}):
            lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
            lid_open_ok = ctx.check(
                "lid_shell_open_aabb_available",
                lid_open_aabb is not None,
                "Could not resolve lid shell AABB in the open pose.",
            )
            if lid_open_ok and lid_open_aabb is not None:
                ctx.check(
                    "lid_raises_when_opened",
                    lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.10,
                    "Opened lid did not rise decisively above its closed height.",
                )
            ctx.expect_contact(lid, firebox, name="lid_remains_captured_on_hinge")
            ctx.expect_gap(
                lid,
                firebox,
                axis="z",
                positive_elem="lid_shell",
                negative_elem="firebox_shell",
                min_gap=0.003,
                name="open_lid_shell_clears_firebox",
            )

    latch_rest_aabb = ctx.part_element_world_aabb(latch_tab, elem="latch_blade")
    latch_rest_ok = ctx.check(
        "latch_rest_aabb_available",
        latch_rest_aabb is not None,
        "Could not resolve latch blade AABB in the rest pose.",
    )
    if latch_rest_ok and latch_rest_aabb is not None:
        with ctx.pose({latch_hinge: math.radians(35.0)}):
            latch_lifted_aabb = ctx.part_element_world_aabb(latch_tab, elem="latch_blade")
            latch_open_ok = ctx.check(
                "latch_open_aabb_available",
                latch_lifted_aabb is not None,
                "Could not resolve latch blade AABB in the lifted pose.",
            )
            if latch_open_ok and latch_lifted_aabb is not None:
                rest_center_y = 0.5 * (latch_rest_aabb[0][1] + latch_rest_aabb[1][1])
                rest_center_z = 0.5 * (latch_rest_aabb[0][2] + latch_rest_aabb[1][2])
                lifted_center_y = 0.5 * (latch_lifted_aabb[0][1] + latch_lifted_aabb[1][1])
                lifted_center_z = 0.5 * (latch_lifted_aabb[0][2] + latch_lifted_aabb[1][2])
                ctx.check(
                    "latch_rotates_up_and_forward",
                    lifted_center_z > rest_center_z + 0.005
                    and lifted_center_y > rest_center_y + 0.010,
                    "Latch tab did not show the expected upward and forward swing.",
                )
            ctx.expect_contact(latch_tab, lid, name="latch_stays_captured_in_motion")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
