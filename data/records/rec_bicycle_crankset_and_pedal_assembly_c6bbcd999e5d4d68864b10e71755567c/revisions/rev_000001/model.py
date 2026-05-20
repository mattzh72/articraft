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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_chainring_road_crankset")

    alloy = model.material("alloy", rgba=(0.78, 0.80, 0.82, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.88, 0.89, 0.90, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    black_composite = model.material("black_composite", rgba=(0.10, 0.10, 0.11, 1.0))

    def circle_profile(radius: float, segments: int = 48, *, phase: float = 0.0) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos(phase + (2.0 * math.pi * i / segments)),
                radius * math.sin(phase + (2.0 * math.pi * i / segments)),
            )
            for i in range(segments)
        ]

    def toothed_profile(
        tip_radius: float,
        root_radius: float,
        teeth: int,
        *,
        phase: float = 0.0,
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for tooth in range(teeth):
            tip_angle = phase + (2.0 * math.pi * tooth / teeth)
            valley_angle = phase + (2.0 * math.pi * (tooth + 0.5) / teeth)
            points.append((tip_radius * math.cos(tip_angle), tip_radius * math.sin(tip_angle)))
            points.append((root_radius * math.cos(valley_angle), root_radius * math.sin(valley_angle)))
        return points

    def hollow_arm_mesh(name: str, *, length: float, width: float) -> object:
        outer = [
            (0.000, 0.000),
            (0.012, 0.008),
            (0.015, 0.026),
            (0.013, 0.070),
            (0.011, 0.132),
            (0.009, length),
            (-0.009, length),
            (-0.011, 0.132),
            (-0.013, 0.070),
            (-0.015, 0.026),
            (-0.012, 0.008),
        ]
        inner = [
            (0.000, 0.020),
            (0.008, 0.030),
            (0.0095, 0.066),
            (0.0085, 0.122),
            (0.0065, length - 0.016),
            (-0.0065, length - 0.016),
            (-0.0085, 0.122),
            (-0.0095, 0.066),
            (-0.008, 0.030),
        ]
        geom = ExtrudeWithHolesGeometry(outer, [inner], height=width, center=True).rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    def chainring_mesh(
        name: str,
        *,
        tip_radius: float,
        root_radius: float,
        teeth: int,
        thickness: float,
        center_hole_radius: float,
        bolt_circle_radius: float,
        bolt_hole_radius: float,
        phase: float = 0.0,
    ) -> object:
        holes = [circle_profile(center_hole_radius, 48)]
        for index in range(5):
            angle = phase + index * (2.0 * math.pi / 5.0)
            holes.append(
                [
                    (
                        bolt_circle_radius * math.cos(angle) + bolt_hole_radius * math.cos(t),
                        bolt_circle_radius * math.sin(angle) + bolt_hole_radius * math.sin(t),
                    )
                    for t in [2.0 * math.pi * i / 20 for i in range(20)]
                ]
            )
        geom = ExtrudeWithHolesGeometry(
            toothed_profile(tip_radius, root_radius, teeth, phase=phase),
            holes,
            height=thickness,
            center=True,
        ).rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    def shell_segment_mesh(
        name: str,
        *,
        outer_radius: float,
        inner_radius: float,
        x0: float,
        x1: float,
    ) -> object:
        geom = ExtrudeWithHolesGeometry(
            circle_profile(outer_radius, 64),
            [circle_profile(inner_radius, 64)],
            height=(x1 - x0),
            center=True,
        ).rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    def bottom_bracket_shell_mesh(name: str) -> object:
        geom = LatheGeometry.from_shell_profiles(
            [
                (0.025, -0.050),
                (0.025, -0.046),
                (0.021, -0.046),
                (0.021, 0.046),
                (0.025, 0.046),
                (0.025, 0.050),
            ],
            [
                (0.016, -0.050),
                (0.016, -0.046),
                (0.017, -0.046),
                (0.017, 0.046),
                (0.016, 0.046),
                (0.016, 0.050),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    def pedal_mesh(name: str, *, side_sign: float) -> object:
        geom = CylinderGeometry(radius=0.0055, height=0.060, radial_segments=18).rotate_y(math.pi / 2.0)
        geom.translate(side_sign * 0.030, 0.0, 0.0)
        geom.merge(BoxGeometry((0.012, 0.090, 0.012)).translate(side_sign * 0.028, 0.0, 0.0))
        geom.merge(BoxGeometry((0.012, 0.090, 0.012)).translate(side_sign * 0.042, 0.0, 0.0))
        geom.merge(BoxGeometry((0.048, 0.014, 0.007)).translate(side_sign * 0.030, 0.033, 0.0))
        geom.merge(BoxGeometry((0.048, 0.014, 0.007)).translate(side_sign * 0.030, -0.033, 0.0))
        geom.merge(
            CylinderGeometry(radius=0.0035, height=0.048, radial_segments=14)
            .rotate_y(math.pi / 2.0)
            .translate(side_sign * 0.030, 0.036, 0.006)
        )
        geom.merge(
            CylinderGeometry(radius=0.0035, height=0.048, radial_segments=14)
            .rotate_y(math.pi / 2.0)
            .translate(side_sign * 0.030, -0.036, 0.006)
        )
        return mesh_from_geometry(geom, name)

    bb_shell = model.part("bottom_bracket_shell")
    bb_shell.visual(
        bottom_bracket_shell_mesh("bb_shell_body_v4"),
        material=dark_steel,
        name="shell_tube",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.060)),
        mass=0.45,
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.009, length=0.092),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_core",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_dust_cap",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_dust_cap",
    )
    spindle.visual(
        Box((0.020, 0.014, 0.014)),
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        material=dark_steel,
        name="left_taper_base",
    )
    spindle.visual(
        Box((0.022, 0.011, 0.011)),
        origin=Origin(xyz=(-0.077, 0.0, 0.0)),
        material=dark_steel,
        name="left_taper_tip",
    )
    spindle.visual(
        Box((0.020, 0.014, 0.014)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=dark_steel,
        name="right_taper_base",
    )
    spindle.visual(
        Box((0.022, 0.011, 0.011)),
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        material=dark_steel,
        name="right_taper_tip",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.170),
        mass=0.32,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_arm = model.part("left_crank_arm")
    left_arm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="left_boss",
    )
    left_arm.visual(
        hollow_arm_mesh("left_crank_beam", length=0.170, width=0.018),
        material=alloy,
        name="left_beam",
    )
    left_arm.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="left_pedal_eye",
    )
    left_arm.inertial = Inertial.from_geometry(
        Box((0.020, 0.190, 0.040)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
    )

    right_crank = model.part("right_crank_assembly")
    right_crank.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="right_boss",
    )
    right_crank.visual(
        hollow_arm_mesh("right_crank_beam", length=0.170, width=0.018),
        material=alloy,
        name="right_beam",
    )
    right_crank.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="right_pedal_eye",
    )
    right_crank.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="spider_hub",
    )
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        right_crank.visual(
            Box((0.014, 0.056, 0.010)),
            origin=Origin(
                xyz=(0.014, 0.029 * math.cos(angle), 0.029 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=alloy,
            name=f"spider_arm_{index + 1}",
        )
        right_crank.visual(
            Cylinder(radius=0.0042, length=0.018),
            origin=Origin(
                xyz=(0.016, 0.065 * math.cos(angle), 0.065 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"outer_bolt_{index + 1}",
        )
        right_crank.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(
                xyz=(0.010, 0.039 * math.cos(angle), 0.039 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"inner_bolt_{index + 1}",
        )
    right_crank.visual(
        chainring_mesh(
            "outer_chainring_mesh",
            tip_radius=0.105,
            root_radius=0.101,
            teeth=52,
            thickness=0.0030,
            center_hole_radius=0.041,
            bolt_circle_radius=0.065,
            bolt_hole_radius=0.0034,
            phase=math.pi / 52.0,
        ),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=polished_alloy,
        name="outer_chainring",
    )
    right_crank.visual(
        chainring_mesh(
            "middle_chainring_mesh",
            tip_radius=0.084,
            root_radius=0.080,
            teeth=42,
            thickness=0.0028,
            center_hole_radius=0.041,
            bolt_circle_radius=0.065,
            bolt_hole_radius=0.0034,
            phase=math.pi / 42.0,
        ),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=polished_alloy,
        name="middle_chainring",
    )
    right_crank.visual(
        chainring_mesh(
            "inner_chainring_mesh",
            tip_radius=0.062,
            root_radius=0.058,
            teeth=30,
            thickness=0.0026,
            center_hole_radius=0.027,
            bolt_circle_radius=0.039,
            bolt_hole_radius=0.0030,
            phase=math.pi / 30.0,
        ),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=polished_alloy,
        name="inner_chainring",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.035, 0.220, 0.220)),
        mass=1.45,
        origin=Origin(xyz=(0.010, 0.055, 0.0)),
    )

    def add_pedal(part_name: str, *, side_sign: float) -> None:
        pedal = model.part(part_name)
        pedal.visual(
            pedal_mesh(f"{part_name}_mesh_v4", side_sign=side_sign),
            material=black_composite,
            name="pedal_body",
        )
        pedal.inertial = Inertial.from_geometry(
            Box((0.050, 0.095, 0.020)),
            mass=0.28,
            origin=Origin(xyz=(side_sign * 0.030, 0.0, 0.0)),
        )

    add_pedal("left_pedal", side_sign=-1.0)
    add_pedal("right_pedal", side_sign=1.0)

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=20.0),
    )
    model.articulation(
        "spindle_to_left_arm",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_arm,
        origin=Origin(xyz=(-0.097, 0.0, 0.0), rpy=(math.pi - math.pi / 3.0, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.097, 0.0, 0.0), rpy=(-math.pi / 3.0, 0.0, 0.0)),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child="left_pedal",
        origin=Origin(xyz=(-0.010, 0.170, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child="right_pedal",
        origin=Origin(xyz=(0.010, 0.170, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bb_shell = object_model.get_part("bottom_bracket_shell")
    spindle = object_model.get_part("spindle")
    left_arm = object_model.get_part("left_crank_arm")
    right_crank = object_model.get_part("right_crank_assembly")
    left_pedal = object_model.get_part("left_pedal")
    right_pedal = object_model.get_part("right_pedal")

    spindle_spin = object_model.get_articulation("spindle_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")

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
        "spindle_axis_is_lateral",
        spindle_spin.axis == (1.0, 0.0, 0.0),
        f"Expected spindle axis along +x, got {spindle_spin.axis}",
    )
    ctx.check(
        "left_pedal_axis_is_lateral",
        left_pedal_spin.axis == (1.0, 0.0, 0.0),
        f"Expected left pedal axis along +x, got {left_pedal_spin.axis}",
    )
    ctx.check(
        "right_pedal_axis_is_lateral",
        right_pedal_spin.axis == (1.0, 0.0, 0.0),
        f"Expected right pedal axis along +x, got {right_pedal_spin.axis}",
    )

    ctx.expect_contact(spindle, bb_shell, name="spindle_seated_in_bottom_bracket")
    ctx.expect_contact(left_arm, spindle, name="left_arm_contacts_square_taper")
    ctx.expect_contact(right_crank, spindle, name="right_crank_contacts_square_taper")
    ctx.expect_contact(left_pedal, left_arm, name="left_pedal_contacts_left_arm")
    ctx.expect_contact(right_pedal, right_crank, name="right_pedal_contacts_right_arm")

    ctx.expect_origin_gap(
        right_crank,
        left_arm,
        axis="x",
        min_gap=0.12,
        name="arms_on_opposite_sides_of_spindle",
    )
    ctx.expect_overlap(
        right_crank,
        bb_shell,
        axes="yz",
        min_overlap=0.03,
        name="right_crank_overlaps_shell_in_crank_plane",
    )

    shell_aabb = ctx.part_element_world_aabb(bb_shell, elem="shell_tube")
    outer_aabb = ctx.part_element_world_aabb(right_crank, elem="outer_chainring")
    middle_aabb = ctx.part_element_world_aabb(right_crank, elem="middle_chainring")
    inner_aabb = ctx.part_element_world_aabb(right_crank, elem="inner_chainring")
    assert shell_aabb is not None
    assert outer_aabb is not None
    assert middle_aabb is not None
    assert inner_aabb is not None

    outer_diam = outer_aabb[1][1] - outer_aabb[0][1]
    middle_diam = middle_aabb[1][1] - middle_aabb[0][1]
    inner_diam = inner_aabb[1][1] - inner_aabb[0][1]
    ctx.check(
        "triple_chainrings_descend_in_size",
        outer_diam > middle_diam > inner_diam,
        f"Expected outer > middle > inner diameters, got {outer_diam:.4f}, {middle_diam:.4f}, {inner_diam:.4f}",
    )
    ctx.check(
        "chainrings_are_outboard_of_shell",
        inner_aabb[0][0] > shell_aabb[1][0] - 0.002,
        f"Inner chainring starts at x={inner_aabb[0][0]:.4f} while shell ends at x={shell_aabb[1][0]:.4f}",
    )

    right_pedal_rest = ctx.part_world_position(right_pedal)
    left_pedal_rest = ctx.part_world_position(left_pedal)
    assert right_pedal_rest is not None
    assert left_pedal_rest is not None
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spindle_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="spindle_quarter_turn_no_floating")
        right_pedal_rotated = ctx.part_world_position(right_pedal)
        left_pedal_rotated = ctx.part_world_position(left_pedal)
        assert right_pedal_rotated is not None
        assert left_pedal_rotated is not None
        ctx.check(
            "right_pedal_rises_with_spindle_turn",
            right_pedal_rotated[2] > right_pedal_rest[2] + 0.10,
            f"Expected right pedal to rise, got rest z={right_pedal_rest[2]:.4f}, rotated z={right_pedal_rotated[2]:.4f}",
        )
        ctx.check(
            "left_pedal_drops_with_spindle_turn",
            left_pedal_rotated[2] < left_pedal_rest[2] - 0.10,
            f"Expected left pedal to drop, got rest z={left_pedal_rest[2]:.4f}, rotated z={left_pedal_rotated[2]:.4f}",
        )

    right_pedal_rest_aabb = ctx.part_world_aabb(right_pedal)
    assert right_pedal_rest_aabb is not None
    rest_y_span = right_pedal_rest_aabb[1][1] - right_pedal_rest_aabb[0][1]
    rest_z_span = right_pedal_rest_aabb[1][2] - right_pedal_rest_aabb[0][2]
    with ctx.pose({right_pedal_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_pedal_half_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="right_pedal_half_turn_no_floating")
        spun_aabb = ctx.part_world_aabb(right_pedal)
        assert spun_aabb is not None
        spun_y_span = spun_aabb[1][1] - spun_aabb[0][1]
        spun_z_span = spun_aabb[1][2] - spun_aabb[0][2]
        ctx.check(
            "pedal_body_rotates_about_stub_axle",
            abs(spun_y_span - rest_z_span) < 0.006 and abs(spun_z_span - rest_y_span) < 0.006,
            (
                f"Expected pedal footprint to rotate. "
                f"Rest spans y/z={rest_y_span:.4f}/{rest_z_span:.4f}, "
                f"spun spans y/z={spun_y_span:.4f}/{spun_z_span:.4f}"
            ),
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
