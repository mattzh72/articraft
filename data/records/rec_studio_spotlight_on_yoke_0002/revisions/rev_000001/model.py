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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_studio_spotlight", assets=ASSETS)

    utility_yellow = model.material("utility_yellow", rgba=(0.77, 0.66, 0.17, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    polymer_black = model.material("polymer_black", rgba=(0.11, 0.12, 0.13, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.76, 0.86, 0.93, 0.42))

    bearing_collar_mesh = _save_mesh(
        "spotlight_bearing_collar.obj",
        LatheGeometry.from_shell_profiles(
            [(0.044, -0.007), (0.044, 0.007)],
            [(0.031, -0.007), (0.031, 0.007)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    can_shell_mesh = _save_mesh(
        "spotlight_can_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.034, -0.070),
                (0.038, -0.058),
                (0.039, -0.010),
                (0.040, 0.040),
                (0.040, 0.074),
                (0.047, 0.092),
            ],
            [
                (0.0, -0.062),
                (0.028, -0.062),
                (0.031, -0.006),
                (0.032, 0.046),
                (0.033, 0.082),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )
    handle_mesh = _save_mesh(
        "spotlight_top_handle.obj",
        tube_from_spline_points(
            [
                (-0.004, 0.0, 0.050),
                (0.012, 0.0, 0.067),
                (0.035, 0.0, 0.075),
                (0.058, 0.0, 0.070),
                (0.070, 0.0, 0.052),
            ],
            radius=0.0045,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    base = model.part("base")
    base.visual(
        Box((0.205, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.068, 0.006)),
        material=rubber,
        name="left_runner",
    )
    base.visual(
        Box((0.205, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.068, 0.006)),
        material=rubber,
        name="right_runner",
    )
    base.visual(
        Box((0.250, 0.180, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=charcoal,
        name="base_plate",
    )
    base.visual(
        Box((0.082, 0.100, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=charcoal,
        name="pedestal",
    )
    base.visual(
        Box((0.060, 0.070, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, 0.043), rpy=(0.0, -0.82, 0.0)),
        material=coated_steel,
        name="front_gusset",
    )
    base.visual(
        Box((0.060, 0.070, 0.010)),
        origin=Origin(xyz=(-0.038, 0.0, 0.043), rpy=(0.0, 0.82, 0.0)),
        material=coated_steel,
        name="rear_gusset",
    )
    base.visual(
        bearing_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=coated_steel,
        name="bearing_collar",
    )
    base.visual(
        Box((0.062, 0.028, 0.040)),
        origin=Origin(xyz=(-0.074, 0.094, 0.046)),
        material=polymer_black,
        name="control_box",
    )
    base.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(-0.044, 0.094, 0.061)),
        material=charcoal,
        name="switch_guard",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((0.043, 0.043), (0.043, -0.043), (-0.043, 0.043), (-0.043, -0.043))
    ):
        base.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.085)),
            material=fastener_steel,
            name=f"anchor_bolt_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.250, 0.180, 0.096)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    yoke_frame = model.part("yoke_frame")
    yoke_frame.visual(
        Cylinder(radius=0.051, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=coated_steel,
        name="pan_flange",
    )
    yoke_frame.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=coated_steel,
        name="pan_pilot",
    )
    yoke_frame.visual(
        Cylinder(radius=0.026, length=0.038),
        origin=Origin(xyz=(-0.018, 0.0, 0.015)),
        material=charcoal,
        name="slew_hub",
    )
    yoke_frame.visual(
        Box((0.032, 0.060, 0.050)),
        origin=Origin(xyz=(-0.040, 0.0, 0.025)),
        material=charcoal,
        name="rear_mast",
    )
    yoke_frame.visual(
        Box((0.030, 0.100, 0.012)),
        origin=Origin(xyz=(-0.040, 0.0, 0.037)),
        material=coated_steel,
        name="rear_tie",
    )
    yoke_frame.visual(
        Box((0.046, 0.022, 0.040)),
        origin=Origin(xyz=(-0.040, 0.061, 0.052)),
        material=utility_yellow,
        name="left_clevis_root",
    )
    yoke_frame.visual(
        Box((0.046, 0.022, 0.040)),
        origin=Origin(xyz=(-0.040, -0.061, 0.052)),
        material=utility_yellow,
        name="right_clevis_root",
    )
    yoke_frame.visual(
        Box((0.080, 0.012, 0.138)),
        origin=Origin(xyz=(-0.008, 0.071, 0.109)),
        material=utility_yellow,
        name="left_bracket",
    )
    yoke_frame.visual(
        Box((0.080, 0.012, 0.138)),
        origin=Origin(xyz=(-0.008, -0.071, 0.109)),
        material=utility_yellow,
        name="right_bracket",
    )
    yoke_frame.visual(
        Box((0.028, 0.012, 0.094)),
        origin=Origin(xyz=(-0.064, 0.060, 0.106)),
        material=coated_steel,
        name="left_side_web",
    )
    yoke_frame.visual(
        Box((0.028, 0.012, 0.094)),
        origin=Origin(xyz=(-0.064, -0.060, 0.106)),
        material=coated_steel,
        name="right_side_web",
    )
    yoke_frame.visual(
        Box((0.022, 0.130, 0.018)),
        origin=Origin(xyz=(-0.070, 0.0, 0.150)),
        material=coated_steel,
        name="top_bridge",
    )
    yoke_frame.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.067, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="left_pivot_collar",
    )
    yoke_frame.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, -0.067, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="right_pivot_collar",
    )
    yoke_frame.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.081, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_bearing_cap",
    )
    yoke_frame.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, -0.081, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_bearing_cap",
    )
    yoke_frame.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, 0.087, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="left_clamp_bolt",
    )
    yoke_frame.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(xyz=(0.0, -0.087, 0.104), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_steel,
        name="right_clamp_bolt",
    )
    for index, y_pos in enumerate((0.042, -0.042)):
        yoke_frame.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(-0.070, y_pos, 0.162)),
            material=fastener_steel,
            name=f"top_bridge_bolt_{index}",
        )
    yoke_frame.inertial = Inertial.from_geometry(
        Box((0.110, 0.162, 0.178)),
        mass=4.2,
        origin=Origin(xyz=(-0.002, 0.0, 0.089)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        can_shell_mesh,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=utility_yellow,
        name="can_shell",
    )
    spotlight_head.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer_black,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.033, length=0.004),
        origin=Origin(xyz=(0.122, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    spotlight_head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="left_trunnion",
    )
    spotlight_head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="right_trunnion",
    )
    spotlight_head.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(-0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_cap",
    )
    for index, x_pos in enumerate((-0.030, -0.036, -0.042)):
        spotlight_head.visual(
            Cylinder(radius=0.036, length=0.003),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=coated_steel,
            name=f"cooling_fin_{index}",
        )
    spotlight_head.visual(
        Box((0.050, 0.060, 0.030)),
        origin=Origin(xyz=(0.046, 0.0, -0.046)),
        material=charcoal,
        name="ballast_box",
    )
    spotlight_head.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.004, 0.0, 0.041)),
        material=coated_steel,
        name="rear_handle_post",
    )
    spotlight_head.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.070, 0.0, 0.041)),
        material=coated_steel,
        name="front_handle_post",
    )
    spotlight_head.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, 0.038)),
        material=coated_steel,
        name="rear_handle_boss",
    )
    spotlight_head.visual(
        Box((0.016, 0.018, 0.010)),
        origin=Origin(xyz=(0.070, 0.0, 0.038)),
        material=coated_steel,
        name="front_handle_boss",
    )
    spotlight_head.visual(handle_mesh, material=coated_steel, name="top_handle")
    for index, (y_pos, z_pos) in enumerate(
        ((0.018, 0.018), (-0.018, 0.018), (0.018, -0.018), (-0.018, -0.018))
    ):
        spotlight_head.visual(
            Cylinder(radius=0.0035, length=0.004),
            origin=Origin(xyz=(-0.051, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener_steel,
            name=f"rear_cap_screw_{index}",
        )
    spotlight_head.inertial = Inertial.from_geometry(
        Box((0.186, 0.120, 0.110)),
        mass=5.5,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke_frame,
        child=spotlight_head,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.5,
            lower=math.radians(-35.0),
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    yoke_frame = object_model.get_part("yoke_frame")
    spotlight_head = object_model.get_part("spotlight_head")
    base_to_yoke = object_model.get_articulation("base_to_yoke")
    yoke_to_head = object_model.get_articulation("yoke_to_head")

    bearing_collar = base.get_visual("bearing_collar")
    pan_flange = yoke_frame.get_visual("pan_flange")
    left_pivot_collar = yoke_frame.get_visual("left_pivot_collar")
    right_pivot_collar = yoke_frame.get_visual("right_pivot_collar")
    front_bezel = spotlight_head.get_visual("front_bezel")
    left_trunnion = spotlight_head.get_visual("left_trunnion")
    right_trunnion = spotlight_head.get_visual("right_trunnion")

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

    ctx.expect_gap(
        yoke_frame,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=pan_flange,
        negative_elem=bearing_collar,
        name="pan_stage_seats_on_bearing_collar",
    )
    ctx.expect_overlap(
        yoke_frame,
        base,
        axes="xy",
        min_overlap=0.080,
        elem_a=pan_flange,
        elem_b=bearing_collar,
        name="pan_stage_has_broad_support_footprint",
    )
    ctx.expect_contact(
        yoke_frame,
        spotlight_head,
        contact_tol=0.0005,
        elem_a=left_pivot_collar,
        elem_b=left_trunnion,
        name="left_trunnion_supported_by_yoke",
    )
    ctx.expect_contact(
        yoke_frame,
        spotlight_head,
        contact_tol=0.0005,
        elem_a=right_pivot_collar,
        elem_b=right_trunnion,
        name="right_trunnion_supported_by_yoke",
    )
    ctx.expect_gap(
        spotlight_head,
        base,
        axis="z",
        min_gap=0.040,
        name="head_clears_base_in_rest_pose",
    )

    def aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    rest_front_aabb = ctx.part_element_world_aabb(spotlight_head, elem=front_bezel)
    if rest_front_aabb is None:
        ctx.fail("front_bezel_measurable_at_rest", "Front bezel AABB was unavailable at rest.")
    else:
        rest_front_center = aabb_center(rest_front_aabb)

        with ctx.pose({yoke_to_head: math.radians(45.0)}):
            raised_front_aabb = ctx.part_element_world_aabb(spotlight_head, elem=front_bezel)
            if raised_front_aabb is None:
                ctx.fail("front_bezel_measurable_when_tilted", "Front bezel AABB was unavailable when tilted.")
            else:
                raised_front_center = aabb_center(raised_front_aabb)
                ctx.check(
                    "tilt_axis_raises_front_bezel",
                    raised_front_center[2] > rest_front_center[2] + 0.045,
                    details=(
                        f"Expected tilted bezel to rise by > 0.045 m; "
                        f"got rest z={rest_front_center[2]:.4f}, tilted z={raised_front_center[2]:.4f}."
                    ),
                )
            ctx.expect_contact(
                yoke_frame,
                spotlight_head,
                contact_tol=0.0005,
                elem_a=left_pivot_collar,
                elem_b=left_trunnion,
                name="left_trunnion_stays_supported_when_tilted",
            )
            ctx.expect_contact(
                yoke_frame,
                spotlight_head,
                contact_tol=0.0005,
                elem_a=right_pivot_collar,
                elem_b=right_trunnion,
                name="right_trunnion_stays_supported_when_tilted",
            )
            ctx.expect_gap(
                spotlight_head,
                base,
                axis="z",
                min_gap=0.030,
                name="head_clears_base_when_tilted",
            )

        with ctx.pose({base_to_yoke: math.pi / 2.0}):
            panned_front_aabb = ctx.part_element_world_aabb(spotlight_head, elem=front_bezel)
            if panned_front_aabb is None:
                ctx.fail("front_bezel_measurable_when_panned", "Front bezel AABB was unavailable when panned.")
            else:
                panned_front_center = aabb_center(panned_front_aabb)
                ctx.check(
                    "pan_axis_swings_bezel_around_vertical_stage",
                    abs(panned_front_center[0]) < 0.035 and panned_front_center[1] > 0.050,
                    details=(
                        f"Expected pan to swing the front around +Y; "
                        f"got center=({panned_front_center[0]:.4f}, {panned_front_center[1]:.4f}, {panned_front_center[2]:.4f})."
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
