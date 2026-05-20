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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (-(z + z_center), y, x)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _bridge_leaf_shell_mesh():
    profiles = [
        _rounded_section(x=0.18, width=1.34, height=0.10, radius=0.036, z_center=0.002),
        _rounded_section(x=0.72, width=1.32, height=0.094, radius=0.032, z_center=0.000),
        _rounded_section(x=1.55, width=1.22, height=0.070, radius=0.024, z_center=-0.006),
        _rounded_section(x=2.46, width=1.08, height=0.045, radius=0.016, z_center=-0.012),
    ]
    return LoftGeometry(profiles, cap=True, closed=True).rotate_y(math.pi / 2.0)


def _bearing_sleeve_mesh(*, outer_radius: float, inner_radius: float, length: float):
    half_length = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_singleleaf_drawbridge")

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    leaf_paint = model.material("leaf_paint", rgba=(0.78, 0.80, 0.82, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.38, 0.40, 0.43, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.06, 0.06, 0.07, 1.0))
    wear_metal = model.material("wear_metal", rgba=(0.55, 0.57, 0.60, 1.0))

    bearing_sleeve = _mesh(
        "drawbridge_bearing_sleeve",
        _bearing_sleeve_mesh(outer_radius=0.075, inner_radius=0.055, length=0.18),
    )
    leaf_shell = _mesh("drawbridge_leaf_shell", _bridge_leaf_shell_mesh())

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.68, 2.02, 0.12)),
        origin=Origin(xyz=(-0.26, 0.0, -0.34)),
        material=frame_paint,
        name="base_plinth",
    )
    support_frame.visual(
        Box((0.26, 1.62, 0.48)),
        origin=Origin(xyz=(-0.33, 0.0, -0.10)),
        material=frame_paint,
        name="machinery_pedestal",
    )
    support_frame.visual(
        Box((0.12, 1.50, 0.14)),
        origin=Origin(xyz=(-0.22, 0.0, 0.10)),
        material=frame_paint,
        name="rear_tie_beam",
    )
    support_frame.visual(
        Box((0.18, 0.26, 0.42)),
        origin=Origin(xyz=(-0.17, 0.81, -0.01)),
        material=frame_paint,
        name="left_bearing_back_web",
    )
    support_frame.visual(
        Box((0.22, 0.30, 0.06)),
        origin=Origin(xyz=(-0.03, 0.84, 0.125)),
        material=frame_paint,
        name="left_bearing_upper_cap",
    )
    support_frame.visual(
        Box((0.22, 0.30, 0.06)),
        origin=Origin(xyz=(-0.03, 0.84, -0.125)),
        material=frame_paint,
        name="left_bearing_lower_cap",
    )
    support_frame.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(-0.02, 1.00, 0.0)),
        material=frame_paint,
        name="left_bearing_outer_jaw",
    )
    support_frame.visual(
        Box((0.06, 0.18, 0.16)),
        origin=Origin(xyz=(-0.085, 0.81, 0.0)),
        material=bearing_metal,
        name="left_bearing_mount_bridge",
    )
    support_frame.visual(
        Box((0.18, 0.26, 0.42)),
        origin=Origin(xyz=(-0.17, -0.81, -0.01)),
        material=frame_paint,
        name="right_bearing_back_web",
    )
    support_frame.visual(
        Box((0.22, 0.30, 0.06)),
        origin=Origin(xyz=(-0.03, -0.84, 0.125)),
        material=frame_paint,
        name="right_bearing_upper_cap",
    )
    support_frame.visual(
        Box((0.22, 0.30, 0.06)),
        origin=Origin(xyz=(-0.03, -0.84, -0.125)),
        material=frame_paint,
        name="right_bearing_lower_cap",
    )
    support_frame.visual(
        Box((0.18, 0.08, 0.22)),
        origin=Origin(xyz=(-0.02, -1.00, 0.0)),
        material=frame_paint,
        name="right_bearing_outer_jaw",
    )
    support_frame.visual(
        Box((0.06, 0.18, 0.16)),
        origin=Origin(xyz=(-0.085, -0.81, 0.0)),
        material=bearing_metal,
        name="right_bearing_mount_bridge",
    )
    support_frame.visual(
        bearing_sleeve,
        origin=Origin(xyz=(0.0, 0.81, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="left_bearing_sleeve",
    )
    support_frame.visual(
        bearing_sleeve,
        origin=Origin(xyz=(0.0, -0.81, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="right_bearing_sleeve",
    )
    support_frame.visual(
        Box((0.18, 1.12, 0.08)),
        origin=Origin(xyz=(-0.12, 0.0, 0.19)),
        material=frame_paint,
        name="top_service_bridge",
    )
    support_frame.visual(
        Box((0.18, 0.14, 0.016)),
        origin=Origin(xyz=(0.34, 0.58, -0.062)),
        material=elastomer_black,
        name="left_stop_pad",
    )
    support_frame.visual(
        Box((0.18, 0.14, 0.016)),
        origin=Origin(xyz=(0.34, -0.58, -0.062)),
        material=elastomer_black,
        name="right_stop_pad",
    )
    support_frame.visual(
        Box((0.40, 0.12, 0.068)),
        origin=Origin(xyz=(0.05, 0.74, -0.104)),
        material=frame_paint,
        name="left_stop_arm",
    )
    support_frame.visual(
        Box((0.18, 0.04, 0.068)),
        origin=Origin(xyz=(0.34, 0.665, -0.104)),
        material=frame_paint,
        name="left_stop_riser",
    )
    support_frame.visual(
        Box((0.40, 0.12, 0.068)),
        origin=Origin(xyz=(0.05, -0.74, -0.104)),
        material=frame_paint,
        name="right_stop_arm",
    )
    support_frame.visual(
        Box((0.18, 0.04, 0.068)),
        origin=Origin(xyz=(0.34, -0.665, -0.104)),
        material=frame_paint,
        name="right_stop_riser",
    )
    support_frame.visual(
        Box((0.16, 0.86, 0.045)),
        origin=Origin(xyz=(-0.28, 0.0, 0.145)),
        material=polymer_dark,
        name="service_cover",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.72, 2.04, 0.70)),
        mass=280.0,
        origin=Origin(xyz=(-0.20, 0.0, -0.02)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Cylinder(radius=0.045, length=0.216),
        origin=Origin(xyz=(0.0, 0.778, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="left_trunnion_stub",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.045, length=0.216),
        origin=Origin(xyz=(0.0, -0.778, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="right_trunnion_stub",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.125, length=1.34),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="pivot_drum",
    )
    bridge_leaf.visual(
        Box((0.20, 1.34, 0.12)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=leaf_paint,
        name="pivot_web",
    )
    bridge_leaf.visual(
        leaf_shell,
        material=leaf_paint,
        name="leaf_shell",
    )
    bridge_leaf.visual(
        Box((1.46, 0.24, 0.12)),
        origin=Origin(xyz=(0.95, 0.0, -0.02)),
        material=frame_paint,
        name="center_spine",
    )
    bridge_leaf.visual(
        Box((1.12, 0.12, 0.10)),
        origin=Origin(xyz=(0.84, 0.42, -0.016)),
        material=frame_paint,
        name="left_side_girder",
    )
    bridge_leaf.visual(
        Box((1.12, 0.12, 0.10)),
        origin=Origin(xyz=(0.84, -0.42, -0.016)),
        material=frame_paint,
        name="right_side_girder",
    )
    bridge_leaf.visual(
        Box((1.40, 0.88, 0.010)),
        origin=Origin(xyz=(1.25, 0.0, 0.028)),
        material=polymer_dark,
        name="deck_tread",
    )
    bridge_leaf.visual(
        Box((0.18, 0.12, 0.014)),
        origin=Origin(xyz=(0.34, 0.58, -0.047)),
        material=wear_metal,
        name="left_landing_shoe",
    )
    bridge_leaf.visual(
        Box((0.18, 0.12, 0.014)),
        origin=Origin(xyz=(0.34, -0.58, -0.047)),
        material=wear_metal,
        name="right_landing_shoe",
    )
    bridge_leaf.visual(
        Box((0.06, 1.02, 0.020)),
        origin=Origin(xyz=(2.49, 0.0, -0.010)),
        material=elastomer_black,
        name="nose_strip",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((2.56, 1.36, 0.22)),
        mass=180.0,
        origin=Origin(xyz=(1.22, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=bridge_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.45,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("frame_to_leaf")
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

    with ctx.pose({leaf_hinge: 0.0}):
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="left_landing_shoe",
            elem_b="left_stop_pad",
            contact_tol=0.002,
            name="left_shoe_seats_on_stop_pad",
        )
        ctx.expect_contact(
            bridge_leaf,
            support_frame,
            elem_a="right_landing_shoe",
            elem_b="right_stop_pad",
            contact_tol=0.002,
            name="right_shoe_seats_on_stop_pad",
        )
        ctx.expect_within(
            bridge_leaf,
            support_frame,
            axes="xz",
            inner_elem="left_trunnion_stub",
            outer_elem="left_bearing_sleeve",
            margin=0.0,
            name="left_trunnion_stays_within_bearing_footprint",
        )
        ctx.expect_within(
            bridge_leaf,
            support_frame,
            axes="xz",
            inner_elem="right_trunnion_stub",
            outer_elem="right_bearing_sleeve",
            margin=0.0,
            name="right_trunnion_stays_within_bearing_footprint",
        )
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="y",
            elem_a="left_trunnion_stub",
            elem_b="left_bearing_sleeve",
            min_overlap=0.15,
            name="left_trunnion_has_bearing_engagement",
        )
        ctx.expect_overlap(
            bridge_leaf,
            support_frame,
            axes="y",
            elem_a="right_trunnion_stub",
            elem_b="right_bearing_sleeve",
            min_overlap=0.15,
            name="right_trunnion_has_bearing_engagement",
        )

    with ctx.pose({leaf_hinge: 1.30}):
        ctx.expect_gap(
            bridge_leaf,
            support_frame,
            axis="z",
            positive_elem="nose_strip",
            min_gap=1.8,
            name="leaf_nose_rises_clear_when_open",
        )

        nose_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="nose_strip")
        frame_aabb = ctx.part_world_aabb(support_frame)
        if nose_aabb is None or frame_aabb is None:
            ctx.fail("open_pose_aabb_available", "Expected leaf nose and support frame AABBs in open pose.")
        else:
            nose_min, nose_max = nose_aabb
            frame_min, frame_max = frame_aabb
            ctx.check(
                "open_leaf_remains_forward_of_pivot_frame",
                nose_max[0] > frame_min[0] + 0.35,
                (
                    "Expected the raised leaf nose to remain visibly forward of the fixed frame; "
                    f"nose_max_x={nose_max[0]:.3f}, frame_min_x={frame_min[0]:.3f}, frame_max_x={frame_max[0]:.3f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
