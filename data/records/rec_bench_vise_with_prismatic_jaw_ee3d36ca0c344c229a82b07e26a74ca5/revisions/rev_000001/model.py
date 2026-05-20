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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _signed_area(points: list[tuple[float, float]]) -> float:
    area = 0.0
    for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1]):
        area += x1 * y2 - x2 * y1
    return 0.5 * area


def _orient(points: list[tuple[float, float]], *, ccw: bool) -> list[tuple[float, float]]:
    area = _signed_area(points)
    if ccw and area < 0.0:
        return list(reversed(points))
    if not ccw and area > 0.0:
        return list(reversed(points))
    return points


def _extrude_xz_profile_along_y(
    outer_xz: list[tuple[float, float]],
    length_y: float,
    *,
    name: str,
    holes_xz: list[list[tuple[float, float]]] | None = None,
):
    outer_local = _orient([(x, -z) for x, z in outer_xz], ccw=True)
    hole_loops = []
    for hole in holes_xz or []:
        hole_loops.append(_orient([(x, -z) for x, z in hole], ccw=False))

    if hole_loops:
        geom = ExtrudeWithHolesGeometry(
            outer_local,
            hole_loops,
            length_y,
            cap=True,
            center=True,
            closed=True,
        )
    else:
        geom = ExtrudeGeometry(
            outer_local,
            length_y,
            cap=True,
            center=True,
            closed=True,
        )
    geom.rotate_x(-math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _extrude_yz_profile_along_x(
    outer_yz: list[tuple[float, float]],
    length_x: float,
    *,
    name: str,
    holes_yz: list[list[tuple[float, float]]] | None = None,
):
    outer_local = _orient([(-z, y) for y, z in outer_yz], ccw=True)
    hole_loops = []
    for hole in holes_yz or []:
        hole_loops.append(_orient([(-z, y) for y, z in hole], ccw=False))

    if hole_loops:
        geom = ExtrudeWithHolesGeometry(
            outer_local,
            hole_loops,
            length_x,
            cap=True,
            center=True,
            closed=True,
        )
    else:
        geom = ExtrudeGeometry(
            outer_local,
            length_x,
            cap=True,
            center=True,
            closed=True,
        )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_milling_vise")

    cast_iron = model.material("cast_iron", rgba=(0.34, 0.36, 0.39, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    oxide = model.material("oxide", rgba=(0.10, 0.10, 0.11, 1.0))

    # World axes:
    # - X: vise jaw opening direction
    # - Y: lower cross-slide travel direction
    # - Z: vertical
    base = model.part("base")

    base_profile = [
        (-0.170, 0.000),
        (0.170, 0.000),
        (0.170, 0.016),
        (0.117, 0.016),
        (0.108, 0.030),
        (0.082, 0.030),
        (0.073, 0.016),
        (-0.073, 0.016),
        (-0.082, 0.030),
        (-0.108, 0.030),
        (-0.117, 0.016),
        (-0.170, 0.016),
    ]
    base.visual(
        _extrude_xz_profile_along_y(
            base_profile,
            0.220,
            name="vise_base_body",
        ),
        material=cast_iron,
        name="base_body",
    )
    base.visual(
        Box((0.290, 0.180, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_way_landing",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.220, 0.038)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    cross_slide_plate = model.part("cross_slide_plate")

    lower_carriage_outer = [
        (-0.150, 0.000),
        (0.150, 0.000),
        (0.150, 0.032),
        (-0.150, 0.032),
    ]
    lower_left_hole = [
        (-0.120, 0.000),
        (-0.070, 0.000),
        (-0.080, 0.014),
        (-0.110, 0.014),
    ]
    lower_right_hole = [
        (0.070, 0.000),
        (0.120, 0.000),
        (0.110, 0.014),
        (0.080, 0.014),
    ]
    cross_slide_plate.visual(
        _extrude_xz_profile_along_y(
            lower_carriage_outer,
            0.160,
            holes_xz=[lower_left_hole, lower_right_hole],
            name="cross_slide_lower_carriage",
        ),
        material=cast_iron,
        name="cross_slide_lower_carriage",
    )

    upper_way_profile = [
        (-0.060, 0.000),
        (0.060, 0.000),
        (0.060, 0.004),
        (0.053, 0.004),
        (0.047, 0.016),
        (0.029, 0.016),
        (0.023, 0.004),
        (-0.023, 0.004),
        (-0.029, 0.016),
        (-0.047, 0.016),
        (-0.053, 0.004),
        (-0.060, 0.004),
    ]
    cross_slide_plate.visual(
        _extrude_yz_profile_along_x(
            upper_way_profile,
            0.240,
            name="upper_jaw_bed",
        ),
        origin=Origin(xyz=(-0.010, 0.0, 0.032)),
        material=cast_iron,
        name="jaw_bed",
    )

    cross_slide_plate.visual(
        Box((0.040, 0.112, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.038)),
        material=cast_iron,
        name="fixed_jaw_pedestal",
    )
    cross_slide_plate.visual(
        Box((0.032, 0.100, 0.056)),
        origin=Origin(xyz=(0.094, 0.0, 0.072)),
        material=cast_iron,
        name="fixed_jaw_block",
    )
    cross_slide_plate.visual(
        Box((0.004, 0.086, 0.042)),
        origin=Origin(xyz=(0.081, 0.0, 0.065)),
        material=jaw_steel,
        name="fixed_jaw_face",
    )

    cross_slide_plate.visual(
        Box((0.020, 0.060, 0.024)),
        origin=Origin(xyz=(-0.120, 0.0, 0.044)),
        material=cast_iron,
        name="leadscrew_bearing_block",
    )
    cross_slide_plate.visual(
        Cylinder(radius=0.0045, length=0.180),
        origin=Origin(xyz=(-0.030, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="leadscrew",
    )
    cross_slide_plate.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.133, 0.0, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handwheel_collar",
    )
    cross_slide_plate.visual(
        Cylinder(radius=0.004, length=0.094),
        origin=Origin(xyz=(-0.139, 0.0, 0.046), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oxide,
        name="handwheel_bar",
    )
    cross_slide_plate.inertial = Inertial.from_geometry(
        Box((0.300, 0.160, 0.100)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    moving_jaw = model.part("moving_jaw")

    moving_carriage_outer = [
        (-0.056, 0.000),
        (0.056, 0.000),
        (0.056, 0.022),
        (-0.056, 0.022),
    ]
    upper_left_hole = [
        (-0.054, 0.000),
        (-0.022, 0.000),
        (-0.028, 0.012),
        (-0.048, 0.012),
    ]
    upper_right_hole = [
        (0.022, 0.000),
        (0.054, 0.000),
        (0.048, 0.012),
        (0.028, 0.012),
    ]
    screw_slot = [
        (-0.008, 0.004),
        (0.008, 0.004),
        (0.008, 0.016),
        (-0.008, 0.016),
    ]
    moving_jaw.visual(
        _extrude_yz_profile_along_x(
            moving_carriage_outer,
            0.050,
            holes_yz=[upper_left_hole, upper_right_hole, screw_slot],
            name="moving_jaw_carriage",
        ),
        material=cast_iron,
        name="moving_jaw_carriage",
    )
    moving_jaw.visual(
        Box((0.036, 0.098, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=cast_iron,
        name="moving_jaw_block",
    )
    moving_jaw.visual(
        Box((0.004, 0.084, 0.040)),
        origin=Origin(xyz=(0.016, 0.0, 0.040)),
        material=jaw_steel,
        name="moving_jaw_face",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.050, 0.098, 0.072)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    model.articulation(
        "base_to_cross_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cross_slide_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.10,
            lower=-0.030,
            upper=0.030,
        ),
    )
    model.articulation(
        "cross_slide_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=cross_slide_plate,
        child=moving_jaw,
        origin=Origin(xyz=(0.043, 0.0, 0.048)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.08,
            lower=0.0,
            upper=0.095,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cross_slide_plate = object_model.get_part("cross_slide_plate")
    moving_jaw = object_model.get_part("moving_jaw")
    base_to_cross_slide = object_model.get_articulation("base_to_cross_slide")
    cross_slide_to_moving_jaw = object_model.get_articulation("cross_slide_to_moving_jaw")

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
        cross_slide_plate,
        base,
        name="lower slide carriage is supported on the base ways",
    )
    ctx.expect_contact(
        moving_jaw,
        cross_slide_plate,
        name="moving jaw carriage rides on the upper ways",
    )
    ctx.expect_gap(
        cross_slide_plate,
        moving_jaw,
        axis="x",
        positive_elem="fixed_jaw_face",
        negative_elem="moving_jaw_face",
        min_gap=0.017,
        max_gap=0.0195,
        name="jaw faces start with a narrow opening",
    )
    ctx.expect_overlap(
        moving_jaw,
        cross_slide_plate,
        axes="y",
        elem_a="moving_jaw_carriage",
        elem_b="jaw_bed",
        min_overlap=0.100,
        name="moving jaw stays centered across the upper bed width",
    )

    rest_jaw_pos = ctx.part_world_position(moving_jaw)
    jaw_open_pos = None
    with ctx.pose({cross_slide_to_moving_jaw: 0.095}):
        ctx.expect_gap(
            cross_slide_plate,
            moving_jaw,
            axis="x",
            positive_elem="fixed_jaw_face",
            negative_elem="moving_jaw_face",
            min_gap=0.110,
            name="jaw opening increases at full travel",
        )
        ctx.expect_overlap(
            moving_jaw,
            cross_slide_plate,
            axes="x",
            elem_a="moving_jaw_carriage",
            elem_b="jaw_bed",
            min_overlap=0.045,
            name="moving jaw retains engagement on the upper dovetails",
        )
        jaw_open_pos = ctx.part_world_position(moving_jaw)
    ctx.check(
        "moving jaw opens toward negative X",
        rest_jaw_pos is not None
        and jaw_open_pos is not None
        and jaw_open_pos[0] < rest_jaw_pos[0] - 0.080,
        details=f"rest={rest_jaw_pos}, open={jaw_open_pos}",
    )

    rest_slide_pos = ctx.part_world_position(cross_slide_plate)
    cross_shifted_pos = None
    with ctx.pose({base_to_cross_slide: 0.030}):
        ctx.expect_contact(
            cross_slide_plate,
            base,
            name="lower cross slide remains seated at full lateral travel",
        )
        ctx.expect_overlap(
            cross_slide_plate,
            base,
            axes="y",
            elem_a="cross_slide_lower_carriage",
            elem_b="base_body",
            min_overlap=0.150,
            name="cross slide retains engagement on the lower dovetails",
        )
        cross_shifted_pos = ctx.part_world_position(cross_slide_plate)
    ctx.check(
        "cross slide moves toward positive Y",
        rest_slide_pos is not None
        and cross_shifted_pos is not None
        and cross_shifted_pos[1] > rest_slide_pos[1] + 0.020,
        details=f"rest={rest_slide_pos}, shifted={cross_shifted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
