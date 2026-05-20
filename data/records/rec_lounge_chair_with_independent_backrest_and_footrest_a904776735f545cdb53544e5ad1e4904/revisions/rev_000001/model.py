from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube_path(points, radius: float, name: str):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def _y_cylinder_origin(x: float, z: float) -> Origin:
    return Origin(xyz=(x, 0.0, z), rpy=(-pi / 2.0, 0.0, 0.0))


def _aabb_center(aabb):
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zero_gravity_patio_lounge_chair")

    frame_finish = model.material("frame_finish", rgba=(0.23, 0.24, 0.22, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.53, 0.55, 0.57, 1.0))
    sling_fabric = model.material("sling_fabric", rgba=(0.34, 0.37, 0.35, 1.0))
    armrest_trim = model.material("armrest_trim", rgba=(0.61, 0.50, 0.38, 1.0))
    foot_cap = model.material("foot_cap", rgba=(0.10, 0.10, 0.10, 1.0))

    support_frame = model.part("support_frame")
    support_radius = 0.018
    support_outer_y = 0.31
    side_support_profile = [
        (-0.58, 0.07),
        (-0.50, 0.20),
        (-0.33, 0.41),
        (-0.08, 0.56),
        (0.22, 0.56),
        (0.48, 0.44),
        (0.64, 0.18),
        (0.72, 0.05),
    ]
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        rail_points = [(x, y_sign * support_outer_y, z) for x, z in side_support_profile]
        support_frame.visual(
            _tube_path(rail_points, support_radius, f"{side_name}_support_rail"),
            material=frame_finish,
            name=f"{side_name}_support_rail",
        )
        support_frame.visual(
            Cylinder(radius=0.022, length=0.038),
            origin=Origin(
                xyz=(-0.06, y_sign * 0.301, 0.55),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_finish,
            name=f"{side_name}_recline_hub",
        )
        support_frame.visual(
            Cylinder(radius=0.020, length=0.052),
            origin=Origin(
                xyz=(0.50, y_sign * 0.301, 0.33),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_finish,
            name=f"{side_name}_footrest_hub",
        )
        support_frame.visual(
            Box((0.050, 0.030, 0.110)),
            origin=Origin(xyz=(0.50, y_sign * 0.316, 0.385)),
            material=frame_finish,
            name=f"{side_name}_footrest_bracket",
        )
        support_frame.visual(
            Box((0.23, 0.045, 0.020)),
            origin=Origin(xyz=(0.12, y_sign * 0.285, 0.586)),
            material=armrest_trim,
            name=f"{side_name}_armrest",
        )
        support_frame.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(-0.58, y_sign * 0.31, 0.05), rpy=(0.0, pi / 2.0, 0.0)),
            material=foot_cap,
            name=f"{side_name}_rear_foot",
        )
        support_frame.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(0.72, y_sign * 0.31, 0.03), rpy=(0.0, pi / 2.0, 0.0)),
            material=foot_cap,
            name=f"{side_name}_front_foot",
        )

    for bar_name, x, z, radius, length in (
        ("rear_spreader", -0.50, 0.20, 0.016, 0.66),
        ("front_spreader", 0.64, 0.18, 0.016, 0.66),
        ("upper_back_brace", -0.33, 0.41, 0.012, 0.62),
    ):
        support_frame.visual(
            Cylinder(radius=radius, length=length),
            origin=_y_cylinder_origin(x, z),
            material=frame_finish,
            name=bar_name,
        )

    seat_back_frame = model.part("seat_back_frame")
    seat_radius = 0.014
    seat_outer_y = 0.27
    seat_side_profile = [
        (-0.34, 0.38),
        (-0.28, 0.29),
        (-0.18, 0.16),
        (0.00, 0.00),
        (0.18, -0.10),
        (0.38, -0.18),
        (0.54, -0.21),
    ]
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        seat_back_frame.visual(
            _tube_path(
                [(x, y_sign * seat_outer_y, z) for x, z in seat_side_profile],
                seat_radius,
                f"{side_name}_seat_back_rail",
            ),
            material=frame_finish,
            name=f"{side_name}_seat_back_rail",
        )
        seat_back_frame.visual(
            Cylinder(radius=0.018, length=0.024),
            origin=Origin(
                xyz=(0.00, y_sign * 0.274, 0.00),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_finish,
            name=f"{side_name}_seat_trunnion",
        )

    for bar_name, x, z, radius, length in (
        ("head_rail", -0.33, 0.38, 0.014, 0.56),
        ("lumbar_rail", -0.12, 0.11, 0.011, 0.56),
        ("front_rail", 0.52, -0.21, 0.014, 0.56),
    ):
        seat_back_frame.visual(
            Cylinder(radius=radius, length=length),
            origin=_y_cylinder_origin(x, z),
            material=frame_finish,
            name=bar_name,
        )

    seat_back_frame.visual(
        Box((0.010, 0.556, 0.50)),
        origin=Origin(xyz=(-0.17, 0.0, 0.20), rpy=(0.0, -0.60, 0.0)),
        material=sling_fabric,
        name="back_sling",
    )
    seat_back_frame.visual(
        Box((0.40, 0.556, 0.010)),
        origin=Origin(xyz=(0.24, 0.0, -0.12), rpy=(0.0, 0.24, 0.0)),
        material=sling_fabric,
        name="seat_sling",
    )

    footrest_panel = model.part("footrest_panel")
    footrest_radius = 0.012
    footrest_outer_y = 0.26
    footrest_side_profile = [
        (0.00, 0.00),
        (0.12, -0.01),
        (0.25, -0.04),
        (0.38, -0.08),
        (0.46, -0.10),
    ]
    for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
        footrest_panel.visual(
            _tube_path(
                [(x, y_sign * footrest_outer_y, z) for x, z in footrest_side_profile],
                footrest_radius,
                f"{side_name}_footrest_rail",
            ),
            material=frame_finish,
            name=f"{side_name}_footrest_rail",
        )
        footrest_panel.visual(
            Cylinder(radius=0.016, length=0.022),
            origin=Origin(
                xyz=(0.00, y_sign * 0.264, 0.00),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=hinge_finish,
            name=f"{side_name}_footrest_trunnion",
        )

    for bar_name, x, z, radius, length in (
        ("hinge_rail", 0.00, 0.00, 0.012, 0.53),
        ("mid_rail", 0.22, -0.04, 0.010, 0.53),
        ("foot_front_rail", 0.45, -0.10, 0.012, 0.53),
    ):
        footrest_panel.visual(
            Cylinder(radius=radius, length=length),
            origin=_y_cylinder_origin(x, z),
            material=frame_finish,
            name=bar_name,
        )

    footrest_panel.visual(
        Box((0.38, 0.526, 0.008)),
        origin=Origin(xyz=(0.24, 0.0, -0.05), rpy=(0.0, 0.18, 0.0)),
        material=sling_fabric,
        name="footrest_sling",
    )

    model.articulation(
        "support_to_seat_back",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=seat_back_frame,
        origin=Origin(xyz=(-0.06, 0.0, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=radians(52.0),
        ),
    )
    model.articulation(
        "support_to_footrest",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=footrest_panel,
        origin=Origin(xyz=(0.50, 0.0, 0.33)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    support_frame = object_model.get_part("support_frame")
    seat_back_frame = object_model.get_part("seat_back_frame")
    footrest_panel = object_model.get_part("footrest_panel")
    recline = object_model.get_articulation("support_to_seat_back")
    footrest_hinge = object_model.get_articulation("support_to_footrest")

    head_rail = seat_back_frame.get_visual("head_rail")
    seat_front_rail = seat_back_frame.get_visual("front_rail")
    foot_front_rail = footrest_panel.get_visual("foot_front_rail")
    left_footrest_hub = support_frame.get_visual("left_footrest_hub")
    left_footrest_trunnion = footrest_panel.get_visual("left_footrest_trunnion")

    ctx.check(
        "recline axis is transverse",
        recline.axis == (0.0, -1.0, 0.0),
        details=f"axis={recline.axis}",
    )
    ctx.check(
        "footrest axis is transverse",
        footrest_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={footrest_hinge.axis}",
    )
    ctx.check(
        "footrest hinge is ahead of recline hinge",
        footrest_hinge.origin.xyz[0] > recline.origin.xyz[0] + 0.40,
        details=f"recline_origin={recline.origin.xyz}, footrest_origin={footrest_hinge.origin.xyz}",
    )

    with ctx.pose({recline: 0.0, footrest_hinge: 0.0}):
        ctx.expect_within(
            seat_back_frame,
            support_frame,
            axes="y",
            margin=0.04,
            name="seat-back frame stays between the side supports",
        )
        ctx.expect_within(
            footrest_panel,
            support_frame,
            axes="y",
            margin=0.04,
            name="footrest stays between the side supports",
        )
        ctx.expect_contact(
            support_frame,
            footrest_panel,
            elem_a=left_footrest_hub,
            elem_b=left_footrest_trunnion,
            name="left footrest trunnion seats in the support hinge barrel",
        )
        rest_head = _aabb_center(ctx.part_element_world_aabb(seat_back_frame, elem=head_rail))
        rest_seat_front = _aabb_center(
            ctx.part_element_world_aabb(seat_back_frame, elem=seat_front_rail)
        )
        rest_foot_front = _aabb_center(
            ctx.part_element_world_aabb(footrest_panel, elem=foot_front_rail)
        )

    with ctx.pose(
        {
            recline: recline.motion_limits.upper,
            footrest_hinge: footrest_hinge.motion_limits.upper,
        }
    ):
        reclined_head = _aabb_center(
            ctx.part_element_world_aabb(seat_back_frame, elem=head_rail)
        )
        reclined_seat_front = _aabb_center(
            ctx.part_element_world_aabb(seat_back_frame, elem=seat_front_rail)
        )
        raised_foot_front = _aabb_center(
            ctx.part_element_world_aabb(footrest_panel, elem=foot_front_rail)
        )

    ctx.check(
        "main panel reclines backward",
        rest_head is not None
        and reclined_head is not None
        and reclined_head[0] < rest_head[0] - 0.10
        and reclined_head[2] < rest_head[2] - 0.08,
        details=f"rest_head={rest_head}, reclined_head={reclined_head}",
    )
    ctx.check(
        "seat front rises as the lounge reclines",
        rest_seat_front is not None
        and reclined_seat_front is not None
        and reclined_seat_front[2] > rest_seat_front[2] + 0.08,
        details=f"rest_front={rest_seat_front}, reclined_front={reclined_seat_front}",
    )
    ctx.check(
        "footrest lifts independently",
        rest_foot_front is not None
        and raised_foot_front is not None
        and raised_foot_front[2] > rest_foot_front[2] + 0.10,
        details=f"rest_foot={rest_foot_front}, raised_foot={raised_foot_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
