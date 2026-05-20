from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    hw = width * 0.5
    hh = height * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _lower_v_jaw_mesh(name: str, *, depth: float, width: float):
    profile = [
        (-depth * 0.50, 0.0),
        (depth * 0.50, 0.0),
        (depth * 0.50, 0.070),
        (depth * 0.18, 0.070),
        (0.0, 0.043),
        (-depth * 0.18, 0.070),
        (-depth * 0.50, 0.070),
    ]
    return _mesh(name, ExtrudeGeometry.from_z0(profile, width).rotate_x(pi / 2.0).translate(0.0, width * 0.5, 0.0))


def _upper_v_jaw_mesh(name: str, *, depth: float, width: float):
    profile = [
        (-depth * 0.50, 0.016),
        (-depth * 0.16, 0.016),
        (0.0, 0.0),
        (depth * 0.16, 0.016),
        (depth * 0.50, 0.016),
        (depth * 0.50, 0.046),
        (-depth * 0.50, 0.046),
    ]
    return _mesh(name, ExtrudeGeometry.from_z0(profile, width).rotate_x(pi / 2.0).translate(0.0, width * 0.5, 0.0))


def _crosshead_mesh(name: str, *, length: float, width: float, thickness: float, hole_radius: float):
    outer = _rect_profile(length, width)
    hole = superellipse_profile(hole_radius * 2.0, hole_radius * 2.0, exponent=2.0, segments=28)
    return _mesh(name, ExtrudeWithHolesGeometry(outer, [hole], thickness, center=True))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_vise")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.23, 0.24, 1.0))
    jaw_iron = model.material("jaw_iron", rgba=(0.30, 0.31, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    lower_jaw_mesh = _lower_v_jaw_mesh("pipe_vise_lower_v_jaw", depth=0.120, width=0.095)
    upper_jaw_mesh = _upper_v_jaw_mesh("pipe_vise_upper_v_jaw", depth=0.112, width=0.090)
    crosshead_mesh = _crosshead_mesh(
        "pipe_vise_crosshead",
        length=0.060,
        width=0.112,
        thickness=0.018,
        hole_radius=0.017,
    )

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.38, 0.25, 0.18)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )
    base.visual(
        Box((0.36, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="mounting_foot",
    )
    base.visual(
        Box((0.30, 0.18, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=cast_iron,
        name="base_block",
    )
    base.visual(
        Box((0.100, 0.180, 0.090)),
        origin=Origin(xyz=(-0.110, 0.0, 0.084)),
        material=cast_iron,
        name="rear_boss",
    )
    base.visual(
        Box((0.090, 0.180, 0.082)),
        origin=Origin(xyz=(0.130, 0.0, 0.084)),
        material=cast_iron,
        name="front_pedestal",
    )
    base.visual(
        Box((0.150, 0.140, 0.048)),
        origin=Origin(xyz=(0.045, 0.0, 0.060)),
        material=cast_iron,
        name="jaw_bridge",
    )
    base.visual(
        lower_jaw_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.055)),
        material=jaw_iron,
        name="lower_v_jaw",
    )
    base.visual(
        Box((0.055, 0.030, 0.040)),
        origin=Origin(xyz=(0.020, 0.110, 0.095)),
        material=cast_iron,
        name="chain_anchor",
    )
    base.visual(
        Box((0.060, 0.050, 0.030)),
        origin=Origin(xyz=(0.020, 0.085, 0.078)),
        material=cast_iron,
        name="chain_anchor_rib",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.020, 0.124, 0.098), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="chain_hook_pin",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(-0.112, -0.072, 0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="left_hinge_ear",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(-0.112, 0.072, 0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="right_hinge_ear",
    )

    upper_frame = model.part("upper_frame")
    upper_frame.inertial = Inertial.from_geometry(
        Box((0.30, 0.16, 0.18)),
        mass=8.0,
        origin=Origin(xyz=(0.12, 0.0, 0.08)),
    )
    upper_frame.visual(
        Cylinder(radius=0.016, length=0.094),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    upper_frame.visual(
        Box((0.032, 0.018, 0.028)),
        origin=Origin(xyz=(0.012, -0.055, 0.014)),
        material=jaw_iron,
        name="rear_seat",
    )
    upper_frame.visual(
        Box((0.032, 0.018, 0.028)),
        origin=Origin(xyz=(0.012, 0.055, 0.014)),
        material=jaw_iron,
        name="rear_seat_right",
    )
    upper_frame.visual(
        Box((0.260, 0.016, 0.086)),
        origin=Origin(xyz=(0.135, -0.055, 0.043)),
        material=jaw_iron,
        name="left_side_arm",
    )
    upper_frame.visual(
        Box((0.260, 0.016, 0.086)),
        origin=Origin(xyz=(0.135, 0.055, 0.043)),
        material=jaw_iron,
        name="right_side_arm",
    )
    upper_frame.visual(
        Box((0.170, 0.016, 0.020)),
        origin=Origin(xyz=(0.180, -0.055, 0.100)),
        material=jaw_iron,
        name="top_bridge",
    )
    upper_frame.visual(
        Box((0.170, 0.016, 0.020)),
        origin=Origin(xyz=(0.180, 0.055, 0.100)),
        material=jaw_iron,
        name="top_bridge_right",
    )
    upper_frame.visual(
        Box((0.064, 0.016, 0.090)),
        origin=Origin(xyz=(0.255, -0.055, 0.045)),
        material=jaw_iron,
        name="left_front_cheek",
    )
    upper_frame.visual(
        Box((0.064, 0.016, 0.090)),
        origin=Origin(xyz=(0.255, 0.055, 0.045)),
        material=jaw_iron,
        name="right_front_cheek",
    )
    upper_frame.visual(
        crosshead_mesh,
        origin=Origin(xyz=(0.255, 0.0, 0.079)),
        material=jaw_iron,
        name="crosshead",
    )
    upper_frame.visual(
        Box((0.056, 0.022, 0.032)),
        origin=Origin(xyz=(0.225, 0.071, 0.048)),
        material=jaw_iron,
        name="chain_lug",
    )
    upper_frame.visual(
        _mesh(
            "pipe_vise_chain_tail",
            tube_from_spline_points(
                [
                    (0.232, 0.082, 0.050),
                    (0.270, 0.122, 0.048),
                    (0.260, 0.145, 0.022),
                    (0.220, 0.130, -0.006),
                ],
                radius=0.0042,
                samples_per_segment=14,
                radial_segments=12,
                cap_ends=True,
            ),
        ),
        material=dark_steel,
        name="chain_tail",
    )

    clamp = model.part("clamp_screw")
    clamp.inertial = Inertial.from_geometry(
        Box((0.18, 0.14, 0.16)),
        mass=5.0,
        origin=Origin(xyz=(-0.04, 0.0, -0.02)),
    )
    clamp.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=steel,
        name="thrust_collar",
    )
    clamp.visual(
        Cylinder(radius=0.0115, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="lead_screw",
    )
    clamp.visual(
        Cylinder(radius=0.015, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=steel,
        name="screw_head",
    )
    clamp.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=steel,
        name="handle_hub",
    )
    clamp.visual(
        Cylinder(radius=0.0055, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.058), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    clamp.visual(
        Box((0.120, 0.088, 0.035)),
        origin=Origin(xyz=(-0.060, 0.0, -0.028)),
        material=jaw_iron,
        name="jaw_carrier",
    )
    clamp.visual(
        Box((0.042, 0.070, 0.024)),
        origin=Origin(xyz=(-0.115, 0.0, -0.012)),
        material=jaw_iron,
        name="rear_guide_pad",
    )
    clamp.visual(
        upper_jaw_mesh,
        origin=Origin(xyz=(-0.075, 0.0, -0.051)),
        material=jaw_iron,
        name="upper_v_jaw",
    )

    base_to_upper = model.articulation(
        "base_to_upper_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_frame,
        origin=Origin(xyz=(-0.115, 0.0, 0.145)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.25),
    )
    upper_to_clamp = model.articulation(
        "upper_frame_to_clamp_screw",
        ArticulationType.PRISMATIC,
        parent=upper_frame,
        child=clamp,
        origin=Origin(xyz=(0.255, 0.0, 0.070)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.05, lower=0.0, upper=0.037),
    )

    model.meta["primary_articulations"] = (base_to_upper.name, upper_to_clamp.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_frame = object_model.get_part("upper_frame")
    clamp = object_model.get_part("clamp_screw")
    hinge = object_model.get_articulation("base_to_upper_frame")
    slide = object_model.get_articulation("upper_frame_to_clamp_screw")

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

    ctx.expect_contact(
        upper_frame,
        base,
        elem_a="hinge_barrel",
        elem_b="rear_boss",
        name="upper yoke hinge barrel bears on the rear hinge boss",
    )
    ctx.expect_contact(
        clamp,
        upper_frame,
        elem_a="thrust_collar",
        elem_b="crosshead",
        name="lead screw collar bears on the yoke crosshead",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="z",
        positive_elem="upper_v_jaw",
        negative_elem="lower_v_jaw",
        min_gap=0.032,
        max_gap=0.050,
        name="upper jaw starts visibly open above the fixed V jaw",
    )
    ctx.expect_overlap(
        clamp,
        base,
        axes="y",
        elem_a="upper_v_jaw",
        elem_b="lower_v_jaw",
        min_overlap=0.082,
        name="upper and lower jaws align across the pipe width",
    )

    rest_clamp_pos = ctx.part_world_position(clamp)
    with ctx.pose({slide: slide.motion_limits.upper}):
        clamped_pos = ctx.part_world_position(clamp)
        ctx.expect_gap(
            clamp,
            base,
            axis="z",
            positive_elem="upper_v_jaw",
            negative_elem="lower_v_jaw",
            max_gap=0.006,
            max_penetration=0.0,
            name="screw drive closes the upper jaw down onto the fixed jaw line",
        )
    ctx.check(
        "clamp screw advances downward on its prismatic axis",
        rest_clamp_pos is not None and clamped_pos is not None and clamped_pos[2] < rest_clamp_pos[2] - 0.025,
        details=f"rest={rest_clamp_pos}, clamped={clamped_pos}",
    )

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_clamp_pos = ctx.part_world_position(clamp)
        ctx.expect_gap(
            clamp,
            base,
            axis="z",
            positive_elem="upper_v_jaw",
            negative_elem="lower_v_jaw",
            min_gap=0.140,
            name="hinged yoke swings the upper jaw clear for pipe loading",
        )
    ctx.check(
        "rear hinge swings the yoke upward",
        rest_clamp_pos is not None and open_clamp_pos is not None and open_clamp_pos[2] > rest_clamp_pos[2] + 0.120,
        details=f"rest={rest_clamp_pos}, open={open_clamp_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
