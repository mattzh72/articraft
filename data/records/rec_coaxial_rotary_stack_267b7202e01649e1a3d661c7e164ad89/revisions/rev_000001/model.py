from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, hypot, radians, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_TOP_Z = 0.060
LOWER_STAGE_Z = 0.068
MIDDLE_STAGE_Z = 0.100
TOP_STAGE_Z = 0.132


def _solid_cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _solid_ring(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _solid_box(
    size_x: float,
    size_y: float,
    size_z: float,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
    z0: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, size_z, centered=(True, True, False))
        .translate((center_xy[0], center_xy[1], z0))
    )


def _radial_box(
    inner_radius: float,
    outer_radius: float,
    width: float,
    height: float,
    angle_deg: float,
    *,
    z0: float = 0.0,
) -> cq.Workplane:
    length = outer_radius - inner_radius
    radial_center = 0.5 * (inner_radius + outer_radius)
    spoke = _solid_box(length, width, height, center_xy=(radial_center, 0.0), z0=z0)
    return spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def _offset_cylinder(
    radius: float,
    height: float,
    xy: tuple[float, float],
    *,
    z0: float = 0.0,
) -> cq.Workplane:
    return _solid_cylinder(radius, height, z0=z0).translate((xy[0], xy[1], 0.0))


def _build_center_shaft_base() -> cq.Workplane:
    shape = _solid_cylinder(0.160, 0.030, 0.000)
    shape = shape.union(_solid_cylinder(0.138, 0.030, 0.030))
    shape = shape.union(_solid_ring(0.135, 0.092, 0.008, z0=0.000))

    shape = shape.union(_solid_cylinder(0.040, 0.008, 0.060))
    shape = shape.union(_solid_cylinder(0.009, 0.032, 0.068))

    shape = shape.union(_solid_cylinder(0.034, 0.008, 0.092))
    shape = shape.union(_solid_cylinder(0.008, 0.032, 0.100))

    shape = shape.union(_solid_cylinder(0.024, 0.008, 0.124))
    shape = shape.union(_solid_cylinder(0.007, 0.028, 0.132))
    shape = shape.union(_solid_cylinder(0.018, 0.006, 0.160))
    return shape


def _build_base_turntable_body() -> cq.Workplane:
    shape = _solid_ring(0.138, 0.112, 0.012, z0=0.000)
    shape = shape.union(_solid_ring(0.042, 0.011, 0.022, z0=0.000))

    for angle_deg in (0.0, 120.0, 240.0):
        shape = shape.union(_radial_box(0.042, 0.112, 0.024, 0.022, angle_deg, z0=0.000))

    return shape


def _build_base_turntable_pointer() -> cq.Workplane:
    pointer = _radial_box(0.126, 0.152, 0.014, 0.008, 0.0, z0=0.008)
    pointer = pointer.union(_offset_cylinder(0.009, 0.008, (0.150, 0.000), z0=0.008))
    return pointer


def _build_middle_ring_body() -> cq.Workplane:
    shape = _solid_ring(0.110, 0.088, 0.018, z0=0.000)
    shape = shape.union(_solid_ring(0.036, 0.010, 0.018, z0=0.000))

    for angle_deg in (30.0, 150.0, 270.0):
        shape = shape.union(_radial_box(0.036, 0.088, 0.018, 0.018, angle_deg, z0=0.000))

    return shape


def _build_middle_ring_pointer() -> cq.Workplane:
    return _radial_box(0.098, 0.124, 0.014, 0.010, 66.0, z0=0.004)


def _build_top_flange_body() -> cq.Workplane:
    shape = _solid_ring(0.024, 0.009, 0.022, z0=0.000)
    shape = shape.union(_solid_ring(0.050, 0.022, 0.010, z0=0.000))

    for angle_deg in (0.0, 120.0, 240.0):
        shape = shape.union(_radial_box(0.022, 0.060, 0.014, 0.010, angle_deg, z0=0.000))
        angle_rad = radians(angle_deg)
        xy = (0.060 * cos(angle_rad), 0.060 * sin(angle_rad))
        shape = shape.union(_offset_cylinder(0.016, 0.010, xy, z0=0.000))
    return shape


def _build_top_flange_pointer() -> cq.Workplane:
    fin = _radial_box(0.050, 0.082, 0.018, 0.010, 90.0, z0=0.000)
    fin = fin.union(_offset_cylinder(0.008, 0.010, (0.000, 0.082), z0=0.000))
    return fin


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _xy_distance(a, b) -> float:
    if a is None or b is None:
        return -1.0
    return hypot(a[0] - b[0], a[1] - b[1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_stack")

    model.material("powder_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("graphite_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("bearing_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("anodized_blue", rgba=(0.29, 0.41, 0.63, 1.0))
    model.material("signal_orange", rgba=(0.84, 0.40, 0.12, 1.0))

    center_shaft_base = model.part("center_shaft_base")
    center_shaft_base.visual(
        mesh_from_cadquery(_build_center_shaft_base(), "center_shaft_base"),
        material="powder_black",
        name="center_shaft_base_body",
    )
    center_shaft_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.164),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )

    base_turntable = model.part("base_turntable")
    base_turntable.visual(
        mesh_from_cadquery(_build_base_turntable_body(), "base_turntable_body"),
        material="graphite_gray",
        name="base_turntable_body",
    )
    base_turntable.visual(
        mesh_from_cadquery(_build_base_turntable_pointer(), "base_turntable_pointer"),
        material="signal_orange",
        name="base_turntable_pointer",
    )
    base_turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.140, length=0.022),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        mesh_from_cadquery(_build_middle_ring_body(), "middle_ring_body"),
        material="bearing_silver",
        name="middle_ring_body",
    )
    middle_ring.visual(
        mesh_from_cadquery(_build_middle_ring_pointer(), "middle_ring_pointer"),
        material="signal_orange",
        name="middle_ring_pointer",
    )
    middle_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.018),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    top_flange = model.part("top_flange")
    top_flange.visual(
        mesh_from_cadquery(_build_top_flange_body(), "top_flange_body"),
        material="anodized_blue",
        name="top_flange_body",
    )
    top_flange.visual(
        mesh_from_cadquery(_build_top_flange_pointer(), "top_flange_pointer"),
        material="signal_orange",
        name="top_flange_pointer",
    )
    top_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.022),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "shaft_to_base_turntable",
        ArticulationType.REVOLUTE,
        parent=center_shaft_base,
        child=base_turntable,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "shaft_to_middle_ring",
        ArticulationType.REVOLUTE,
        parent=center_shaft_base,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.8,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "shaft_to_top_flange",
        ArticulationType.REVOLUTE,
        parent=center_shaft_base,
        child=top_flange,
        origin=Origin(xyz=(0.0, 0.0, TOP_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-2.6,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_shaft_base = object_model.get_part("center_shaft_base")
    base_turntable = object_model.get_part("base_turntable")
    middle_ring = object_model.get_part("middle_ring")
    top_flange = object_model.get_part("top_flange")

    shaft_to_base_turntable = object_model.get_articulation("shaft_to_base_turntable")
    shaft_to_middle_ring = object_model.get_articulation("shaft_to_middle_ring")
    shaft_to_top_flange = object_model.get_articulation("shaft_to_top_flange")

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
        center_shaft_base,
        base_turntable,
        elem_a="center_shaft_base_body",
        elem_b="base_turntable_body",
        reason="Grounded center shaft telescopes through the base stage bearing sleeve in this coaxial stack.",
    )
    ctx.allow_overlap(
        center_shaft_base,
        middle_ring,
        elem_a="center_shaft_base_body",
        elem_b="middle_ring_body",
        reason="Grounded center shaft telescopes through the middle stage bearing sleeve in this coaxial stack.",
    )
    ctx.allow_overlap(
        center_shaft_base,
        top_flange,
        elem_a="center_shaft_base_body",
        elem_b="top_flange_body",
        reason="Grounded center shaft telescopes through the top flange bearing sleeve in this coaxial stack.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for joint_name, joint in (
        ("shaft_to_base_turntable", shaft_to_base_turntable),
        ("shaft_to_middle_ring", shaft_to_middle_ring),
        ("shaft_to_top_flange", shaft_to_top_flange),
    ):
        ctx.check(
            f"{joint_name}_axis_is_vertical",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_contact(
        center_shaft_base,
        base_turntable,
        elem_b="base_turntable_body",
        name="base_turntable_supported_on_shaft",
    )
    ctx.expect_contact(
        center_shaft_base,
        middle_ring,
        elem_b="middle_ring_body",
        name="middle_ring_supported_on_shaft",
    )
    ctx.expect_contact(
        center_shaft_base,
        top_flange,
        elem_b="top_flange_body",
        name="top_flange_supported_on_shaft",
    )

    ctx.expect_gap(
        middle_ring,
        base_turntable,
        axis="z",
        min_gap=0.008,
        max_gap=0.015,
        positive_elem="middle_ring_body",
        negative_elem="base_turntable_body",
        name="middle_ring_clear_of_base_turntable",
    )
    ctx.expect_gap(
        top_flange,
        middle_ring,
        axis="z",
        min_gap=0.012,
        max_gap=0.018,
        positive_elem="top_flange_body",
        negative_elem="middle_ring_body",
        name="top_flange_clear_of_middle_ring",
    )
    ctx.expect_within(
        middle_ring,
        base_turntable,
        axes="xy",
        inner_elem="middle_ring_body",
        outer_elem="base_turntable_body",
        margin=0.0,
        name="middle_ring_nested_inside_turntable_footprint",
    )
    ctx.expect_within(
        top_flange,
        middle_ring,
        axes="xy",
        inner_elem="top_flange_body",
        outer_elem="middle_ring_body",
        margin=0.01,
        name="top_flange_nested_inside_middle_ring_footprint",
    )

    with ctx.pose(
        shaft_to_base_turntable=0.0,
        shaft_to_middle_ring=0.0,
        shaft_to_top_flange=0.0,
    ):
        rest_base_pointer = _aabb_center(
            ctx.part_element_world_aabb(base_turntable, elem="base_turntable_pointer")
        )
        rest_middle_pointer = _aabb_center(
            ctx.part_element_world_aabb(middle_ring, elem="middle_ring_pointer")
        )
        rest_top_pointer = _aabb_center(
            ctx.part_element_world_aabb(top_flange, elem="top_flange_pointer")
        )

    ctx.check(
        "pointer_visuals_present",
        all(
            pointer is not None
            for pointer in (rest_base_pointer, rest_middle_pointer, rest_top_pointer)
        ),
        details=(
            f"base={rest_base_pointer}, "
            f"middle={rest_middle_pointer}, "
            f"top={rest_top_pointer}"
        ),
    )

    if all(pointer is not None for pointer in (rest_base_pointer, rest_middle_pointer, rest_top_pointer)):
        with ctx.pose(
            shaft_to_base_turntable=1.10,
            shaft_to_middle_ring=0.0,
            shaft_to_top_flange=0.0,
        ):
            base_pointer_after_base_motion = _aabb_center(
                ctx.part_element_world_aabb(base_turntable, elem="base_turntable_pointer")
            )
            middle_pointer_after_base_motion = _aabb_center(
                ctx.part_element_world_aabb(middle_ring, elem="middle_ring_pointer")
            )
            top_pointer_after_base_motion = _aabb_center(
                ctx.part_element_world_aabb(top_flange, elem="top_flange_pointer")
            )

        ctx.check(
            "base_turntable_joint_is_independent",
            _xy_distance(rest_base_pointer, base_pointer_after_base_motion) > 0.050
            and _xy_distance(rest_middle_pointer, middle_pointer_after_base_motion) < 0.002
            and _xy_distance(rest_top_pointer, top_pointer_after_base_motion) < 0.002,
            details=(
                f"base_move={_xy_distance(rest_base_pointer, base_pointer_after_base_motion):.4f}, "
                f"middle_move={_xy_distance(rest_middle_pointer, middle_pointer_after_base_motion):.4f}, "
                f"top_move={_xy_distance(rest_top_pointer, top_pointer_after_base_motion):.4f}"
            ),
        )

        with ctx.pose(
            shaft_to_base_turntable=0.0,
            shaft_to_middle_ring=-0.95,
            shaft_to_top_flange=0.0,
        ):
            base_pointer_after_middle_motion = _aabb_center(
                ctx.part_element_world_aabb(base_turntable, elem="base_turntable_pointer")
            )
            middle_pointer_after_middle_motion = _aabb_center(
                ctx.part_element_world_aabb(middle_ring, elem="middle_ring_pointer")
            )
            top_pointer_after_middle_motion = _aabb_center(
                ctx.part_element_world_aabb(top_flange, elem="top_flange_pointer")
            )

        ctx.check(
            "middle_ring_joint_is_independent",
            _xy_distance(rest_middle_pointer, middle_pointer_after_middle_motion) > 0.035
            and _xy_distance(rest_base_pointer, base_pointer_after_middle_motion) < 0.002
            and _xy_distance(rest_top_pointer, top_pointer_after_middle_motion) < 0.002,
            details=(
                f"base_move={_xy_distance(rest_base_pointer, base_pointer_after_middle_motion):.4f}, "
                f"middle_move={_xy_distance(rest_middle_pointer, middle_pointer_after_middle_motion):.4f}, "
                f"top_move={_xy_distance(rest_top_pointer, top_pointer_after_middle_motion):.4f}"
            ),
        )

        with ctx.pose(
            shaft_to_base_turntable=0.0,
            shaft_to_middle_ring=0.0,
            shaft_to_top_flange=1.20,
        ):
            base_pointer_after_top_motion = _aabb_center(
                ctx.part_element_world_aabb(base_turntable, elem="base_turntable_pointer")
            )
            middle_pointer_after_top_motion = _aabb_center(
                ctx.part_element_world_aabb(middle_ring, elem="middle_ring_pointer")
            )
            top_pointer_after_top_motion = _aabb_center(
                ctx.part_element_world_aabb(top_flange, elem="top_flange_pointer")
            )

        ctx.check(
            "top_flange_joint_is_independent",
            _xy_distance(rest_top_pointer, top_pointer_after_top_motion) > 0.030
            and _xy_distance(rest_base_pointer, base_pointer_after_top_motion) < 0.002
            and _xy_distance(rest_middle_pointer, middle_pointer_after_top_motion) < 0.002,
            details=(
                f"base_move={_xy_distance(rest_base_pointer, base_pointer_after_top_motion):.4f}, "
                f"middle_move={_xy_distance(rest_middle_pointer, middle_pointer_after_top_motion):.4f}, "
                f"top_move={_xy_distance(rest_top_pointer, top_pointer_after_top_motion):.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
