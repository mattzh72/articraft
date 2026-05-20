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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _yz_section(
    x: float,
    center_y: float,
    center_z: float,
    depth: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, center_y + py, center_z + pz)
        for py, pz in rounded_rect_profile(depth, height, radius, corner_segments=8)
    ]


def _xz_section(
    center_x: float,
    y: float,
    center_z: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, y, center_z + pz)
        for px, pz in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="browline_glasses")

    acetate_black = model.material("acetate_black", rgba=(0.10, 0.08, 0.08, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.55, 0.58, 0.62, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    pad_clear = model.material("pad_clear", rgba=(0.83, 0.86, 0.88, 0.92))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.146, 0.030, 0.050)),
        mass=0.040,
        origin=Origin(xyz=(0.0, -0.004, 0.002)),
    )

    brow_sections = [
        _yz_section(-0.070, -0.010, 0.018, 0.0052, 0.0080, 0.0018),
        _yz_section(-0.054, -0.007, 0.020, 0.0054, 0.0105, 0.0022),
        _yz_section(-0.034, -0.003, 0.021, 0.0050, 0.0110, 0.0024),
        _yz_section(-0.010, 0.000, 0.019, 0.0044, 0.0070, 0.0018),
        _yz_section(0.010, 0.000, 0.019, 0.0044, 0.0070, 0.0018),
        _yz_section(0.034, -0.003, 0.021, 0.0050, 0.0110, 0.0024),
        _yz_section(0.054, -0.007, 0.020, 0.0054, 0.0105, 0.0022),
        _yz_section(0.070, -0.010, 0.018, 0.0052, 0.0080, 0.0018),
    ]
    brow_mesh = mesh_from_geometry(section_loft(brow_sections), "browline_bar")
    front_frame.visual(brow_mesh, material=acetate_black, name="brow_bar")

    left_rim_points = [
        (-0.056, -0.008, 0.017),
        (-0.060, -0.010, 0.007),
        (-0.059, -0.009, -0.006),
        (-0.052, -0.006, -0.017),
        (-0.041, -0.003, -0.021),
        (-0.029, -0.001, -0.022),
        (-0.018, 0.000, -0.019),
        (-0.010, 0.000, -0.010),
        (-0.007, 0.000, 0.015),
    ]
    left_rim_mesh = mesh_from_geometry(
        tube_from_spline_points(
            left_rim_points,
            radius=0.0011,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "left_lower_rim",
    )
    right_rim_mesh = mesh_from_geometry(
        tube_from_spline_points(
            _mirror_x(left_rim_points),
            radius=0.0011,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "right_lower_rim",
    )
    front_frame.visual(left_rim_mesh, material=gunmetal, name="left_lower_rim")
    front_frame.visual(right_rim_mesh, material=gunmetal, name="right_lower_rim")

    front_frame.visual(
        Cylinder(radius=0.0011, length=0.018),
        origin=Origin(xyz=(0.0, -0.0008, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="bridge_bar",
    )
    front_frame.visual(
        Box((0.0068, 0.0054, 0.0080)),
        origin=Origin(xyz=(-0.0695, -0.0025, 0.0175)),
        material=acetate_black,
        name="left_hinge_block",
    )
    front_frame.visual(
        Box((0.0068, 0.0054, 0.0080)),
        origin=Origin(xyz=(0.0695, -0.0025, 0.0175)),
        material=acetate_black,
        name="right_hinge_block",
    )
    front_frame.visual(
        Box((0.0080, 0.0050, 0.0064)),
        origin=Origin(xyz=(-0.0655, -0.0074, 0.0180)),
        material=acetate_black,
        name="left_hinge_web",
    )
    front_frame.visual(
        Box((0.0080, 0.0050, 0.0064)),
        origin=Origin(xyz=(0.0655, -0.0074, 0.0180)),
        material=acetate_black,
        name="right_hinge_web",
    )

    def make_spring_segment(part_name: str, visual_name: str):
        segment = model.part(part_name)
        segment.inertial = Inertial.from_geometry(
            Box((0.006, 0.013, 0.006)),
            mass=0.003,
            origin=Origin(xyz=(0.0, -0.0065, 0.0)),
        )
        segment.visual(
            Box((0.0048, 0.0130, 0.0056)),
            origin=Origin(xyz=(0.0, -0.0065, 0.0)),
            material=dark_metal,
            name=visual_name,
        )
        segment.visual(
            Box((0.0026, 0.0070, 0.0014)),
            origin=Origin(xyz=(0.0, -0.0090, 0.0032)),
            material=gunmetal,
            name=f"{visual_name}_leaf",
        )
        return segment

    def make_temple(part_name: str, mesh_name: str, visual_name: str):
        temple = model.part(part_name)
        temple.inertial = Inertial.from_geometry(
            Box((0.008, 0.148, 0.026)),
            mass=0.009,
            origin=Origin(xyz=(0.0, -0.074, -0.007)),
        )
        temple_sections = [
            _xz_section(0.0, 0.000, 0.000, 0.0052, 0.0034, 0.0010),
            _xz_section(0.0, -0.030, -0.0005, 0.0050, 0.0032, 0.0010),
            _xz_section(0.0, -0.075, -0.0030, 0.0044, 0.0028, 0.0009),
            _xz_section(0.0005, -0.120, -0.0110, 0.0036, 0.0024, 0.0008),
            _xz_section(0.0010, -0.150, -0.0210, 0.0028, 0.0021, 0.0007),
        ]
        temple_mesh = mesh_from_geometry(section_loft(temple_sections), mesh_name)
        temple.visual(temple_mesh, material=acetate_black, name=visual_name)
        return temple

    left_spring = make_spring_segment("left_spring_hinge", "left_spring_body")
    right_spring = make_spring_segment("right_spring_hinge", "right_spring_body")
    left_temple = make_temple("left_temple", "left_temple_arm", "left_temple_body")
    right_temple = make_temple("right_temple", "right_temple_arm", "right_temple_body")

    left_bracket = model.part("left_nose_bracket")
    left_bracket.inertial = Inertial.from_geometry(
        Box((0.004, 0.013, 0.010)),
        mass=0.001,
        origin=Origin(xyz=(0.0, -0.006, -0.003)),
    )
    left_bracket.visual(
        Box((0.0030, 0.0016, 0.0026)),
        origin=Origin(xyz=(0.0, -0.0008, 0.0)),
        material=gunmetal,
        name="left_bracket_base",
    )
    left_bracket.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, -0.0016, 0.0), (0.0, -0.0050, -0.0020), (0.0, -0.0100, -0.0060)],
                radius=0.0007,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
            "left_nose_wire",
        ),
        material=gunmetal,
        name="left_bracket_wire",
    )
    left_bracket.visual(
        Cylinder(radius=0.0015, length=0.0022),
        origin=Origin(xyz=(0.0, -0.0110, -0.0068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_pad_pivot",
    )

    right_bracket = model.part("right_nose_bracket")
    right_bracket.inertial = Inertial.from_geometry(
        Box((0.004, 0.013, 0.010)),
        mass=0.001,
        origin=Origin(xyz=(0.0, -0.006, -0.003)),
    )
    right_bracket.visual(
        Box((0.0030, 0.0016, 0.0026)),
        origin=Origin(xyz=(0.0, -0.0008, 0.0)),
        material=gunmetal,
        name="right_bracket_base",
    )
    right_bracket.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, -0.0016, 0.0), (0.0, -0.0050, -0.0020), (0.0, -0.0100, -0.0060)],
                radius=0.0007,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
            "right_nose_wire",
        ),
        material=gunmetal,
        name="right_bracket_wire",
    )
    right_bracket.visual(
        Cylinder(radius=0.0015, length=0.0022),
        origin=Origin(xyz=(0.0, -0.0110, -0.0068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_pad_pivot",
    )

    nose_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(superellipse_profile(0.0075, 0.0105, exponent=2.0, segments=28), 0.0018)
        .rotate_x(math.pi / 2.0),
        "nose_pad_surface",
    )

    def make_pad(part_name: str, visual_name: str):
        pad = model.part(part_name)
        pad.inertial = Inertial.from_geometry(
            Box((0.008, 0.008, 0.012)),
            mass=0.0008,
            origin=Origin(xyz=(0.0, -0.0045, -0.0020)),
        )
        pad.visual(
            Box((0.0016, 0.0046, 0.0018)),
            origin=Origin(xyz=(0.0, -0.0023, -0.0002)),
            material=dark_metal,
            name=f"{visual_name}_stem",
        )
        pad.visual(
            nose_pad_mesh,
            origin=Origin(xyz=(0.0, -0.0044, -0.0019), rpy=(math.radians(18), 0.0, 0.0)),
            material=pad_clear,
            name=visual_name,
        )
        return pad

    left_pad = make_pad("left_nose_pad", "left_pad_surface")
    right_pad = make_pad("right_nose_pad", "right_pad_surface")

    left_fold = model.articulation(
        "front_to_left_spring",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spring,
        origin=Origin(xyz=(-0.0695, 0.0, 0.0175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    right_fold = model.articulation(
        "front_to_right_spring",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spring,
        origin=Origin(xyz=(0.0695, 0.0, 0.0175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=math.radians(-100.0),
            upper=0.0,
        ),
    )
    left_spring_joint = model.articulation(
        "left_spring_to_temple",
        ArticulationType.REVOLUTE,
        parent=left_spring,
        child=left_temple,
        origin=Origin(xyz=(0.0, -0.0130, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=5.0,
            lower=math.radians(-18.0),
            upper=math.radians(4.0),
        ),
    )
    right_spring_joint = model.articulation(
        "right_spring_to_temple",
        ArticulationType.REVOLUTE,
        parent=right_spring,
        child=right_temple,
        origin=Origin(xyz=(0.0, -0.0130, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=5.0,
            lower=math.radians(-4.0),
            upper=math.radians(18.0),
        ),
    )

    model.articulation(
        "front_to_left_nose_bracket",
        ArticulationType.FIXED,
        parent=front_frame,
        child=left_bracket,
        origin=Origin(xyz=(-0.0085, 0.0, -0.0012)),
    )
    model.articulation(
        "front_to_right_nose_bracket",
        ArticulationType.FIXED,
        parent=front_frame,
        child=right_bracket,
        origin=Origin(xyz=(0.0085, 0.0, -0.0012)),
    )
    left_pad_joint = model.articulation(
        "left_bracket_to_pad",
        ArticulationType.REVOLUTE,
        parent=left_bracket,
        child=left_pad,
        origin=Origin(xyz=(0.0, -0.0110, -0.0068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=3.0,
            lower=math.radians(-25.0),
            upper=math.radians(25.0),
        ),
    )
    right_pad_joint = model.articulation(
        "right_bracket_to_pad",
        ArticulationType.REVOLUTE,
        parent=right_bracket,
        child=right_pad,
        origin=Origin(xyz=(0.0, -0.0110, -0.0068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=3.0,
            lower=math.radians(-25.0),
            upper=math.radians(25.0),
        ),
    )

    # Keep local variables referenced so the articulated layout remains explicit.
    _ = (
        left_fold,
        right_fold,
        left_spring_joint,
        right_spring_joint,
        left_pad_joint,
        right_pad_joint,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    front_frame = object_model.get_part("front_frame")
    left_spring = object_model.get_part("left_spring_hinge")
    right_spring = object_model.get_part("right_spring_hinge")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_bracket = object_model.get_part("left_nose_bracket")
    right_bracket = object_model.get_part("right_nose_bracket")
    left_pad = object_model.get_part("left_nose_pad")
    right_pad = object_model.get_part("right_nose_pad")

    left_fold = object_model.get_articulation("front_to_left_spring")
    right_fold = object_model.get_articulation("front_to_right_spring")
    left_spring_joint = object_model.get_articulation("left_spring_to_temple")
    right_spring_joint = object_model.get_articulation("right_spring_to_temple")
    left_pad_joint = object_model.get_articulation("left_bracket_to_pad")
    right_pad_joint = object_model.get_articulation("right_bracket_to_pad")

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

    ctx.expect_contact(left_spring, front_frame, elem_a="left_spring_body", elem_b="left_hinge_block")
    ctx.expect_contact(right_spring, front_frame, elem_a="right_spring_body", elem_b="right_hinge_block")
    ctx.expect_contact(left_temple, left_spring, elem_a="left_temple_body", elem_b="left_spring_body")
    ctx.expect_contact(right_temple, right_spring, elem_a="right_temple_body", elem_b="right_spring_body")
    ctx.expect_contact(left_bracket, front_frame, elem_a="left_bracket_base", elem_b="bridge_bar")
    ctx.expect_contact(right_bracket, front_frame, elem_a="right_bracket_base", elem_b="bridge_bar")
    ctx.expect_contact(left_pad, left_bracket, elem_a="left_pad_surface_stem", elem_b="left_pad_pivot")
    ctx.expect_contact(right_pad, right_bracket, elem_a="right_pad_surface_stem", elem_b="right_pad_pivot")

    left_open_aabb = ctx.part_world_aabb(left_temple)
    right_open_aabb = ctx.part_world_aabb(right_temple)
    assert left_open_aabb is not None
    assert right_open_aabb is not None

    with ctx.pose({left_fold: math.radians(95.0), right_fold: math.radians(-95.0)}):
        ctx.expect_overlap(
            left_spring,
            front_frame,
            axes="xz",
            min_overlap=0.0035,
            elem_a="left_spring_body",
            elem_b="left_hinge_block",
            name="left_spring_stays_clipped_when_folded",
        )
        ctx.expect_overlap(
            right_spring,
            front_frame,
            axes="xz",
            min_overlap=0.0035,
            elem_a="right_spring_body",
            elem_b="right_hinge_block",
            name="right_spring_stays_clipped_when_folded",
        )
        left_fold_aabb = ctx.part_world_aabb(left_temple)
        right_fold_aabb = ctx.part_world_aabb(right_temple)
        assert left_fold_aabb is not None
        assert right_fold_aabb is not None
        ctx.check(
            "left_temple_folds_inward",
            left_fold_aabb[1][0] > left_open_aabb[1][0] + 0.050,
            details=f"left temple max x did not move inward enough: open={left_open_aabb}, folded={left_fold_aabb}",
        )
        ctx.check(
            "right_temple_folds_inward",
            right_fold_aabb[0][0] < right_open_aabb[0][0] - 0.050,
            details=f"right temple min x did not move inward enough: open={right_open_aabb}, folded={right_fold_aabb}",
        )

    with ctx.pose({left_spring_joint: math.radians(-12.0), right_spring_joint: math.radians(12.0)}):
        ctx.expect_contact(left_temple, left_spring, elem_a="left_temple_body", elem_b="left_spring_body")
        ctx.expect_contact(right_temple, right_spring, elem_a="right_temple_body", elem_b="right_spring_body")
        left_flex_aabb = ctx.part_world_aabb(left_temple)
        right_flex_aabb = ctx.part_world_aabb(right_temple)
        assert left_flex_aabb is not None
        assert right_flex_aabb is not None
        ctx.check(
            "left_spring_hinge_flexes_outward",
            left_flex_aabb[0][0] < left_open_aabb[0][0] - 0.004,
            details=f"left spring flex did not widen outward enough: open={left_open_aabb}, flex={left_flex_aabb}",
        )
        ctx.check(
            "right_spring_hinge_flexes_outward",
            right_flex_aabb[1][0] > right_open_aabb[1][0] + 0.004,
            details=f"right spring flex did not widen outward enough: open={right_open_aabb}, flex={right_flex_aabb}",
        )

    left_pad_rest_aabb = ctx.part_world_aabb(left_pad)
    right_pad_rest_aabb = ctx.part_world_aabb(right_pad)
    assert left_pad_rest_aabb is not None
    assert right_pad_rest_aabb is not None
    with ctx.pose({left_pad_joint: math.radians(18.0), right_pad_joint: math.radians(-18.0)}):
        ctx.expect_contact(left_pad, left_bracket, elem_a="left_pad_surface_stem", elem_b="left_pad_pivot")
        ctx.expect_contact(right_pad, right_bracket, elem_a="right_pad_surface_stem", elem_b="right_pad_pivot")
        left_pad_pose_aabb = ctx.part_world_aabb(left_pad)
        right_pad_pose_aabb = ctx.part_world_aabb(right_pad)
        assert left_pad_pose_aabb is not None
        assert right_pad_pose_aabb is not None
        ctx.check(
            "left_nose_pad_rotates",
            abs(left_pad_pose_aabb[0][2] - left_pad_rest_aabb[0][2]) > 0.0010,
            details=f"left pad z extent barely changed: rest={left_pad_rest_aabb}, posed={left_pad_pose_aabb}",
        )
        ctx.check(
            "right_nose_pad_rotates",
            abs(right_pad_pose_aabb[1][2] - right_pad_rest_aabb[1][2]) > 0.0010,
            details=f"right pad z extent barely changed: rest={right_pad_rest_aabb}, posed={right_pad_pose_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
