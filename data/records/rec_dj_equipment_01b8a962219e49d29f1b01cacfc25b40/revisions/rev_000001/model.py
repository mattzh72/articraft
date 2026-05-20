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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _yz_prism(points: list[tuple[float, float]], thickness: float) -> MeshGeometry:
    geom = MeshGeometry()
    half = thickness * 0.5
    left = [geom.add_vertex(-half, y, z) for y, z in points]
    right = [geom.add_vertex(half, y, z) for y, z in points]
    count = len(points)

    for index in range(count):
        nxt = (index + 1) % count
        _add_quad(geom, left[index], left[nxt], right[nxt], right[index])

    for index in range(1, count - 1):
        geom.add_face(left[0], left[index + 1], left[index])
        geom.add_face(right[0], right[index], right[index + 1])

    return geom


def _segment_box(point_a: tuple[float, float], point_b: tuple[float, float]) -> tuple[float, float, float]:
    dy = point_b[0] - point_a[0]
    dz = point_b[1] - point_a[1]
    length = math.hypot(dy, dz)
    angle = math.atan2(-dy, dz)
    center_y = (point_a[0] + point_b[0]) * 0.5
    center_z = (point_a[1] + point_b[1]) * 0.5
    return length, center_y, center_z, angle


def _segment_point(point_a: tuple[float, float], point_b: tuple[float, float], t: float) -> tuple[float, float]:
    return (
        point_a[0] + (point_b[0] - point_a[0]) * t,
        point_a[1] + (point_b[1] - point_a[1]) * t,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_monitor_wedge_speaker")

    bracket_finish = model.material("bracket_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    shell_finish = model.material("shell_finish", rgba=(0.11, 0.12, 0.12, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.05, 0.05, 0.05, 0.72))
    driver_finish = model.material("driver_finish", rgba=(0.16, 0.16, 0.16, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.60, 0.62, 0.66, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.08, 0.08, 0.08, 1.0))

    cabinet_width = 0.56
    cabinet_side_thickness = 0.015
    panel_thickness = 0.015
    cabinet_panel_width = cabinet_width - 2.0 * cabinet_side_thickness + 0.008
    pivot_radius = 0.018
    pivot_boss_length = 0.014
    pivot_bushing_length = 0.012
    pivot_z = 0.195

    # YZ side profile in the cabinet's hinge-centered local frame.
    rear_bottom = (-0.04, -0.165)
    rear_top = (-0.01, 0.025)
    top_front = (0.18, 0.145)
    front_bottom = (0.34, -0.165)
    cabinet_profile = [rear_bottom, rear_top, top_front, front_bottom]

    cabinet_side_mesh = mesh_from_geometry(
        _yz_prism(cabinet_profile, cabinet_side_thickness),
        "monitor_cabinet_side_panel",
    )

    bracket_cheek_profile = [
        (-0.14, 0.015),
        (-0.125, 0.080),
        (-0.070, 0.165),
        (-0.005, 0.232),
        (0.085, 0.232),
        (0.150, 0.185),
        (0.180, 0.015),
    ]
    bracket_cheek_thickness = 0.018
    bracket_inner_face = cabinet_width * 0.5 + 0.010
    bracket_cheek_mesh = mesh_from_geometry(
        _yz_prism(bracket_cheek_profile, bracket_cheek_thickness),
        "tilt_bracket_side_cheek",
    )

    bracket = model.part("tilt_bracket")
    bracket.visual(
        Box((0.64, 0.46, 0.015)),
        origin=Origin(xyz=(0.0, 0.08, 0.0075)),
        material=bracket_finish,
        name="base_plate",
    )
    for foot_x in (-0.22, 0.22):
        for foot_y in (-0.06, 0.22):
            bracket.visual(
                Box((0.055, 0.035, 0.010)),
                origin=Origin(xyz=(foot_x, foot_y, -0.005)),
                material=rubber_finish,
                name=f"foot_{'l' if foot_x < 0.0 else 'r'}_{'r' if foot_y < 0.0 else 'f'}",
            )
    bracket.visual(
        bracket_cheek_mesh,
        origin=Origin(xyz=(-(bracket_inner_face + bracket_cheek_thickness * 0.5), 0.0, 0.0)),
        material=bracket_finish,
        name="left_cheek",
    )
    bracket.visual(
        bracket_cheek_mesh,
        origin=Origin(xyz=(bracket_inner_face + bracket_cheek_thickness * 0.5, 0.0, 0.0)),
        material=bracket_finish,
        name="right_cheek",
    )
    bracket.visual(
        Cylinder(radius=pivot_radius, length=pivot_bushing_length),
        origin=Origin(
            xyz=(-(bracket_inner_face - pivot_bushing_length * 0.5), 0.0, pivot_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="left_pivot_bushing",
    )
    bracket.visual(
        Cylinder(radius=pivot_radius, length=pivot_bushing_length),
        origin=Origin(
            xyz=(bracket_inner_face - pivot_bushing_length * 0.5, 0.0, pivot_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="right_pivot_bushing",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.64, 0.46, 0.24)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.08, 0.12)),
    )

    cabinet = model.part("speaker_cabinet")
    cabinet.visual(
        cabinet_side_mesh,
        origin=Origin(xyz=(-(cabinet_width * 0.5 - cabinet_side_thickness * 0.5), 0.0, 0.0)),
        material=shell_finish,
        name="left_side_panel",
    )
    cabinet.visual(
        cabinet_side_mesh,
        origin=Origin(xyz=(cabinet_width * 0.5 - cabinet_side_thickness * 0.5, 0.0, 0.0)),
        material=shell_finish,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_panel_width, 0.38, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (rear_bottom[0] + front_bottom[0]) * 0.5,
                rear_bottom[1] + panel_thickness * 0.5,
            )
        ),
        material=shell_finish,
        name="cabinet_bottom",
    )

    rear_length, rear_center_y, rear_center_z, rear_angle = _segment_box(rear_bottom, rear_top)
    cabinet.visual(
        Box((cabinet_panel_width, panel_thickness, rear_length)),
        origin=Origin(xyz=(0.0, rear_center_y, rear_center_z), rpy=(rear_angle, 0.0, 0.0)),
        material=shell_finish,
        name="rear_panel",
    )

    top_length, top_center_y, top_center_z, top_angle = _segment_box(rear_top, top_front)
    cabinet.visual(
        Box((cabinet_panel_width, panel_thickness, top_length)),
        origin=Origin(xyz=(0.0, top_center_y, top_center_z), rpy=(top_angle, 0.0, 0.0)),
        material=shell_finish,
        name="top_panel",
    )

    front_length, front_center_y, front_center_z, front_angle = _segment_box(front_bottom, top_front)
    front_normal_y = math.cos(front_angle)
    front_normal_z = math.sin(front_angle)
    cabinet.visual(
        Box((cabinet_panel_width + 0.010, 0.014, front_length + 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                front_center_y,
                front_center_z,
            ),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=grille_finish,
        name="grille_panel",
    )

    driver_angle = front_angle - math.pi / 2.0
    woofer_y, woofer_z = _segment_point(front_bottom, top_front, 0.38)
    cabinet.visual(
        Cylinder(radius=0.120, length=0.060),
        origin=Origin(
            xyz=(
                0.0,
                woofer_y - front_normal_y * 0.004,
                woofer_z - front_normal_z * 0.004,
            ),
            rpy=(driver_angle, 0.0, 0.0),
        ),
        material=driver_finish,
        name="woofer_driver",
    )
    horn_y, horn_z = _segment_point(front_bottom, top_front, 0.73)
    cabinet.visual(
        Box((0.18, 0.050, 0.080)),
        origin=Origin(
            xyz=(
                0.0,
                horn_y - front_normal_y * 0.004,
                horn_z - front_normal_z * 0.004,
            ),
            rpy=(front_angle, 0.0, 0.0),
        ),
        material=driver_finish,
        name="hf_horn",
    )
    cabinet.visual(
        Cylinder(radius=pivot_radius, length=pivot_boss_length),
        origin=Origin(
            xyz=(-(cabinet_width * 0.5 - pivot_boss_length * 0.5), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="left_trunnion",
    )
    cabinet.visual(
        Cylinder(radius=pivot_radius, length=pivot_boss_length),
        origin=Origin(
            xyz=(cabinet_width * 0.5 - pivot_boss_length * 0.5, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal_finish,
        name="right_trunnion",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, 0.38, 0.31)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.020, -0.010)),
    )

    model.articulation(
        "bracket_to_cabinet_tilt",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cabinet,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=0.65,
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

    bracket = object_model.get_part("tilt_bracket")
    cabinet = object_model.get_part("speaker_cabinet")
    tilt = object_model.get_articulation("bracket_to_cabinet_tilt")

    ctx.check(
        "tilt articulation uses a horizontal x-axis",
        tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    with ctx.pose({tilt: 0.0}):
        ctx.expect_contact(
            cabinet,
            bracket,
            elem_a="left_trunnion",
            elem_b="left_pivot_bushing",
            contact_tol=0.001,
            name="left trunnion seats on left bracket bushing",
        )
        ctx.expect_contact(
            cabinet,
            bracket,
            elem_a="right_trunnion",
            elem_b="right_pivot_bushing",
            contact_tol=0.001,
            name="right trunnion seats on right bracket bushing",
        )
        ctx.expect_gap(
            cabinet,
            bracket,
            axis="z",
            positive_elem="cabinet_bottom",
            negative_elem="base_plate",
            min_gap=0.010,
            max_gap=0.030,
            name="cabinet clears the base plate at rest",
        )

    upper = 0.65
    if tilt.motion_limits is not None and tilt.motion_limits.upper is not None:
        upper = tilt.motion_limits.upper

    grille_rest = None
    grille_raised = None
    with ctx.pose({tilt: 0.0}):
        grille_rest = ctx.part_element_world_aabb(cabinet, elem="grille_panel")
    with ctx.pose({tilt: upper}):
        ctx.expect_contact(
            cabinet,
            bracket,
            elem_a="left_trunnion",
            elem_b="left_pivot_bushing",
            contact_tol=0.001,
            name="left trunnion stays carried at maximum tilt",
        )
        ctx.expect_contact(
            cabinet,
            bracket,
            elem_a="right_trunnion",
            elem_b="right_pivot_bushing",
            contact_tol=0.001,
            name="right trunnion stays carried at maximum tilt",
        )
        ctx.expect_gap(
            cabinet,
            bracket,
            axis="z",
            positive_elem="cabinet_bottom",
            negative_elem="base_plate",
            min_gap=0.010,
            name="cabinet still clears the base plate at maximum tilt",
        )
        grille_raised = ctx.part_element_world_aabb(cabinet, elem="grille_panel")

    rest_center_z = None
    raised_center_z = None
    if grille_rest is not None:
        rest_center_z = (grille_rest[0][2] + grille_rest[1][2]) * 0.5
    if grille_raised is not None:
        raised_center_z = (grille_raised[0][2] + grille_raised[1][2]) * 0.5
    ctx.check(
        "positive tilt raises the front grille",
        rest_center_z is not None
        and raised_center_z is not None
        and raised_center_z > rest_center_z + 0.060,
        details=f"rest_center_z={rest_center_z}, raised_center_z={raised_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
