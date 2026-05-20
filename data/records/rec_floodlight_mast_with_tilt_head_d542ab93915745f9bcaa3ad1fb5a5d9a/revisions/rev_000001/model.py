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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    return [
        (-half_width, -half_depth),
        (half_width, -half_depth),
        (half_width, half_depth),
        (-half_width, half_depth),
    ]


def _rect_tube_mesh(
    name: str,
    *,
    outer_x: float,
    outer_y: float,
    wall: float,
    length: float,
) -> object:
    inner_x = max(outer_x - 2.0 * wall, 0.002)
    inner_y = max(outer_y - 2.0 * wall, 0.002)
    geometry = ExtrudeWithHolesGeometry(
        _rect_profile(outer_x, outer_y),
        [_rect_profile(inner_x, inner_y)],
        length,
        center=False,
    )
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple(upper[index] - lower[index] for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="construction_floodlight_tower")

    trailer_yellow = model.material("trailer_yellow", rgba=(0.88, 0.73, 0.16, 1.0))
    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.16, 0.17, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.83, 0.90, 0.94, 0.35))
    led_face = model.material("led_face", rgba=(0.94, 0.95, 0.89, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.56, 0.58, 0.60, 1.0))

    trailer_base = model.part("trailer_base")
    trailer_base.inertial = Inertial.from_geometry(
        Box((2.35, 1.05, 0.82)),
        mass=420.0,
        origin=Origin(xyz=(0.18, 0.0, 0.41)),
    )

    frame_z = 0.345
    rail_y = 0.31
    rail_size = (1.15, 0.08, 0.11)
    trailer_base.visual(
        Box(rail_size),
        origin=Origin(xyz=(-0.05, rail_y, frame_z)),
        material=trailer_yellow,
        name="right_frame_rail",
    )
    trailer_base.visual(
        Box(rail_size),
        origin=Origin(xyz=(-0.05, -rail_y, frame_z)),
        material=trailer_yellow,
        name="left_frame_rail",
    )
    trailer_base.visual(
        Box((0.10, 0.70, 0.11)),
        origin=Origin(xyz=(0.45, 0.0, frame_z)),
        material=trailer_yellow,
        name="front_crossmember",
    )
    trailer_base.visual(
        Box((0.10, 0.70, 0.11)),
        origin=Origin(xyz=(-0.42, 0.0, frame_z)),
        material=trailer_yellow,
        name="rear_crossmember",
    )
    trailer_base.visual(
        Box((0.88, 0.70, 0.03)),
        origin=Origin(xyz=(-0.05, 0.0, 0.415)),
        material=dark_steel,
        name="deck_plate",
    )
    trailer_base.visual(
        Cylinder(radius=0.04, length=0.78),
        origin=Origin(xyz=(-0.20, 0.0, 0.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_beam",
    )

    drawbar_z = 0.33
    drawbar_front = (1.22, 0.0)
    for name, start, end in (
        ("right_drawbar", (0.48, 0.29), drawbar_front),
        ("left_drawbar", (0.48, -0.29), drawbar_front),
    ):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        beam_length = math.hypot(dx, dy)
        trailer_base.visual(
            Box((beam_length, 0.06, 0.06)),
            origin=Origin(
                xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, drawbar_z),
                rpy=(0.0, 0.0, math.atan2(dy, dx)),
            ),
            material=trailer_yellow,
            name=name,
        )

    trailer_base.visual(
        Box((0.52, 0.08, 0.06)),
        origin=Origin(xyz=(0.98, 0.0, drawbar_z)),
        material=trailer_yellow,
        name="drawbar_spine",
    )
    trailer_base.visual(
        Box((0.14, 0.08, 0.08)),
        origin=Origin(xyz=(1.31, 0.0, drawbar_z)),
        material=dark_steel,
        name="hitch_coupler",
    )
    trailer_base.visual(
        Cylinder(radius=0.032, length=0.38),
        origin=Origin(xyz=(0.98, 0.0, 0.19)),
        material=dark_steel,
        name="jack_tube",
    )
    trailer_base.visual(
        Box((0.14, 0.14, 0.02)),
        origin=Origin(xyz=(0.98, 0.0, 0.01)),
        material=dark_steel,
        name="jack_foot",
    )

    trailer_base.visual(
        Box((0.16, 0.20, 0.08)),
        origin=Origin(xyz=(0.12, 0.0, 0.43)),
        material=trailer_yellow,
        name="pivot_pedestal",
    )
    trailer_base.visual(
        Box((0.18, 0.08, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.46)),
        material=trailer_yellow,
        name="pedestal_front_brace",
    )
    trailer_base.visual(
        Box((0.12, 0.02, 0.16)),
        origin=Origin(xyz=(0.17, 0.10, 0.55)),
        material=dark_steel,
        name="pivot_clevis_right",
    )
    trailer_base.visual(
        Box((0.12, 0.02, 0.16)),
        origin=Origin(xyz=(0.17, -0.10, 0.55)),
        material=dark_steel,
        name="pivot_clevis_left",
    )
    trailer_base.visual(
        Box((0.24, 0.03, 0.03)),
        origin=Origin(xyz=(0.29, 0.10, 0.49)),
        material=dark_steel,
        name="chain_support",
    )
    trailer_base.visual(
        Box((0.04, 0.04, 0.10)),
        origin=Origin(xyz=(0.39, 0.10, 0.525)),
        material=chain_steel,
        name="chain_post",
    )
    trailer_base.visual(
        Box((0.04, 0.02, 0.04)),
        origin=Origin(xyz=(0.40, 0.10, 0.58)),
        material=chain_steel,
        name="chain_anchor",
    )
    trailer_base.visual(
        Box((0.18, 0.14, 0.03)),
        origin=Origin(xyz=(-0.38, 0.0, 0.43)),
        material=dark_steel,
        name="mast_transport_rest",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.24, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.16, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.07, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hub",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.14),
        mass=22.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.24, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.16, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.07, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hub",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.24, length=0.14),
        mass=22.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_drum",
    )
    mast.visual(
        Box((0.16, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_steel,
        name="hinge_shoe",
    )
    mast.visual(
        _rect_tube_mesh(
            "mast_lower_tube",
            outer_x=0.16,
            outer_y=0.12,
            wall=0.012,
            length=1.10,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=aluminum,
        name="lower_tube",
    )
    mast.visual(
        Box((0.18, 0.13, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
        material=aluminum,
        name="lower_transition",
    )
    mast.visual(
        _rect_tube_mesh(
            "mast_mid_tube",
            outer_x=0.12,
            outer_y=0.09,
            wall=0.010,
            length=0.52,
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
        material=aluminum,
        name="mid_tube",
    )
    mast.visual(
        Box((0.13, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.67)),
        material=aluminum,
        name="upper_transition",
    )
    mast.visual(
        _rect_tube_mesh(
            "mast_top_tube_v2",
            outer_x=0.085,
            outer_y=0.065,
            wall=0.008,
            length=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.67)),
        material=aluminum,
        name="top_tube",
    )
    mast.visual(
        Box((0.018, 0.010, 0.26)),
        origin=Origin(xyz=(0.0, 0.055, 1.81)),
        material=dark_steel,
        name="top_yoke_right",
    )
    mast.visual(
        Box((0.018, 0.010, 0.26)),
        origin=Origin(xyz=(0.0, -0.055, 1.81)),
        material=dark_steel,
        name="top_yoke_left",
    )
    mast.visual(
        Box((0.04, 0.02, 0.08)),
        origin=Origin(xyz=(0.07, 0.07, 0.42)),
        material=chain_steel,
        name="chain_lug",
    )
    mast.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.07, 0.07, 0.42),
                    (0.12, 0.08, 0.28),
                    (0.16, 0.09, 0.15),
                    (0.18, 0.10, 0.04),
                ],
                radius=0.006,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.03,
                corner_segments=10,
            ),
            "mast_chain_brace_v4",
        ),
        material=chain_steel,
        name="chain_brace",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 1.98)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
    )

    flood_head = model.part("flood_head")
    flood_head.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knuckle_drum",
    )
    flood_head.visual(
        Box((0.18, 0.012, 0.08)),
        origin=Origin(xyz=(0.13, 0.038, 0.0)),
        material=dark_steel,
        name="right_arm",
    )
    flood_head.visual(
        Box((0.18, 0.012, 0.08)),
        origin=Origin(xyz=(0.13, -0.038, 0.0)),
        material=dark_steel,
        name="left_arm",
    )
    flood_head.visual(
        Box((0.18, 0.46, 0.28)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=lamp_black,
        name="housing",
    )
    flood_head.visual(
        Box((0.02, 0.42, 0.24)),
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        material=dark_steel,
        name="bezel",
    )
    flood_head.visual(
        Box((0.015, 0.36, 0.18)),
        origin=Origin(xyz=(0.308, 0.0, 0.0)),
        material=led_face,
        name="led_array",
    )
    flood_head.visual(
        Box((0.006, 0.40, 0.22)),
        origin=Origin(xyz=(0.343, 0.0, 0.0)),
        material=glass,
        name="front_glass",
    )
    flood_head.visual(
        Box((0.05, 0.44, 0.02)),
        origin=Origin(xyz=(0.31, 0.0, 0.15)),
        material=lamp_black,
        name="top_viser",
    )
    for index, fin_z in enumerate((-0.09, -0.03, 0.03, 0.09)):
        flood_head.visual(
            Box((0.05, 0.34, 0.012)),
            origin=Origin(xyz=(0.15, 0.0, fin_z)),
            material=dark_steel,
            name=f"cooling_fin_{index}",
        )
    flood_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.50, 0.32)),
        mass=18.0,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=trailer_base,
        child=left_wheel,
        origin=Origin(xyz=(-0.20, -0.47, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=trailer_base,
        child=right_wheel,
        origin=Origin(xyz=(-0.20, 0.47, 0.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=20.0),
    )
    model.articulation(
        "mast_hinge",
        ArticulationType.REVOLUTE,
        parent=trailer_base,
        child=mast,
        origin=Origin(xyz=(0.22, 0.0, 0.56)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.6,
            lower=-1.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=flood_head,
        origin=Origin(xyz=(0.0, 0.0, 1.86)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-0.85,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trailer_base = object_model.get_part("trailer_base")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    mast = object_model.get_part("mast")
    flood_head = object_model.get_part("flood_head")

    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    mast_hinge = object_model.get_articulation("mast_hinge")
    head_tilt = object_model.get_articulation("head_tilt")

    trailer_base.get_visual("chain_anchor")
    mast.get_visual("chain_brace")
    flood_head.get_visual("front_glass")
    flood_head.get_visual("top_viser")

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

    def axis_tuple(joint) -> tuple[float, float, float]:
        return tuple(round(component, 6) for component in joint.axis)

    ctx.check(
        "left_wheel_spin_axis",
        axis_tuple(left_wheel_spin) == (0.0, 1.0, 0.0),
        details=f"axis={left_wheel_spin.axis}",
    )
    ctx.check(
        "right_wheel_spin_axis",
        axis_tuple(right_wheel_spin) == (0.0, 1.0, 0.0),
        details=f"axis={right_wheel_spin.axis}",
    )
    ctx.check(
        "mast_hinge_axis",
        axis_tuple(mast_hinge) == (0.0, 1.0, 0.0),
        details=f"axis={mast_hinge.axis}",
    )
    ctx.check(
        "head_tilt_axis",
        axis_tuple(head_tilt) == (0.0, 1.0, 0.0),
        details=f"axis={head_tilt.axis}",
    )

    mast_limits = mast_hinge.motion_limits
    head_limits = head_tilt.motion_limits
    ctx.check(
        "mast_hinge_range",
        mast_limits is not None
        and mast_limits.lower is not None
        and mast_limits.upper is not None
        and mast_limits.lower <= -1.40
        and abs(mast_limits.upper) <= 1e-6,
        details=f"limits={mast_limits}",
    )
    ctx.check(
        "head_tilt_range",
        head_limits is not None
        and head_limits.lower is not None
        and head_limits.upper is not None
        and head_limits.lower <= -0.80
        and head_limits.upper >= 0.30,
        details=f"limits={head_limits}",
    )

    ctx.expect_contact(
        left_wheel,
        trailer_base,
        contact_tol=0.003,
        name="left_wheel_contacts_axle",
    )
    ctx.expect_contact(
        right_wheel,
        trailer_base,
        contact_tol=0.003,
        name="right_wheel_contacts_axle",
    )
    ctx.expect_contact(
        mast,
        trailer_base,
        contact_tol=0.003,
        name="mast_contacts_base_at_pivot",
    )
    ctx.expect_contact(
        flood_head,
        mast,
        contact_tol=0.003,
        name="flood_head_contacts_mast_yoke",
    )
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="y",
        min_dist=0.90,
        max_dist=0.98,
        name="wheel_track_width",
    )
    ctx.expect_origin_gap(
        mast,
        trailer_base,
        axis="z",
        min_gap=0.54,
        max_gap=0.58,
        name="mast_pivot_height",
    )
    ctx.expect_origin_gap(
        flood_head,
        mast,
        axis="z",
        min_gap=1.82,
        max_gap=1.90,
        name="flood_head_mount_height",
    )

    mast_aabb = ctx.part_world_aabb(mast)
    glass_aabb = ctx.part_element_world_aabb(flood_head, elem="front_glass")
    chain_anchor_aabb = ctx.part_element_world_aabb(trailer_base, elem="chain_anchor")
    chain_brace_aabb = ctx.part_element_world_aabb(mast, elem="chain_brace")

    mast_size = _aabb_size(mast_aabb)
    glass_center = _aabb_center(glass_aabb)
    chain_anchor_center = _aabb_center(chain_anchor_aabb)
    chain_brace_center = _aabb_center(chain_brace_aabb)

    ctx.check(
        "mast_reads_tall_enough",
        mast_size is not None and mast_size[2] > 1.95,
        details=f"mast_size={mast_size}",
    )
    ctx.check(
        "light_head_above_trailer",
        glass_center is not None and glass_center[2] > 2.35,
        details=f"glass_center={glass_center}",
    )
    ctx.check(
        "chain_brace_reaches_lock_side",
        chain_anchor_center is not None
        and chain_brace_center is not None
        and abs(chain_anchor_center[1] - chain_brace_center[1]) < 0.03,
        details=f"chain_anchor_center={chain_anchor_center}, chain_brace_center={chain_brace_center}",
    )

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

    if mast_limits is not None and mast_limits.lower is not None and mast_limits.upper is not None:
        with ctx.pose({mast_hinge: mast_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_hinge_upper_no_floating")

        with ctx.pose({mast_hinge: mast_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_hinge_lower_no_floating")
            folded_glass_aabb = ctx.part_element_world_aabb(flood_head, elem="front_glass")
            folded_glass_center = _aabb_center(folded_glass_aabb)
            ctx.check(
                "mast_folds_rearward_for_transport",
                folded_glass_center is not None and folded_glass_center[0] < -1.20,
                details=f"folded_glass_center={folded_glass_center}",
            )
            ctx.check(
                "folded_head_stays_low",
                folded_glass_center is not None and folded_glass_center[2] < 1.15,
                details=f"folded_glass_center={folded_glass_center}",
            )

    if head_limits is not None and head_limits.lower is not None and head_limits.upper is not None:
        with ctx.pose({mast_hinge: 0.0, head_tilt: head_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_lower_no_floating")
            ctx.expect_contact(
                flood_head,
                mast,
                contact_tol=0.003,
                name="head_tilt_lower_knuckle_contact",
            )
            visor_down_aabb = ctx.part_element_world_aabb(flood_head, elem="top_viser")
            visor_down_center = _aabb_center(visor_down_aabb)

        with ctx.pose({mast_hinge: 0.0, head_tilt: head_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_upper_no_floating")
            ctx.expect_contact(
                flood_head,
                mast,
                contact_tol=0.003,
                name="head_tilt_upper_knuckle_contact",
            )
            visor_up_aabb = ctx.part_element_world_aabb(flood_head, elem="top_viser")
            visor_up_center = _aabb_center(visor_up_aabb)

        ctx.check(
            "head_tilt_changes_aim_height",
            visor_down_center is not None
            and visor_up_center is not None
            and abs(visor_up_center[2] - visor_down_center[2]) > 0.20,
            details=f"down={visor_down_center}, up={visor_up_center}",
        )
        ctx.check(
            "head_tilt_changes_forward_reach",
            visor_down_center is not None
            and visor_up_center is not None
            and abs(visor_up_center[0] - visor_down_center[0]) > 0.15,
            details=f"down={visor_down_center}, up={visor_up_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
