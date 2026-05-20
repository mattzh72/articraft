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
    rounded_rect_profile,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_cap(geom: MeshGeometry, loop: list[int], *, upward: bool) -> None:
    points = [geom.vertices[index] for index in loop]
    center = (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
        sum(point[2] for point in points) / len(points),
    )
    center_id = geom.add_vertex(*center)
    for index, start in enumerate(loop):
        end = loop[(index + 1) % len(loop)]
        if upward:
            geom.add_face(center_id, start, end)
        else:
            geom.add_face(center_id, end, start)


def _bridge_loops(
    geom: MeshGeometry,
    first_loop: list[int],
    second_loop: list[int],
    *,
    outward: bool,
) -> None:
    for index, first_a in enumerate(first_loop):
        first_b = first_loop[(index + 1) % len(first_loop)]
        second_a = second_loop[index]
        second_b = second_loop[(index + 1) % len(second_loop)]
        if outward:
            _add_quad(geom, first_a, first_b, second_b, second_a)
        else:
            _add_quad(geom, first_a, second_a, second_b, first_b)


def _rounded_rect_loop(
    depth: float,
    width: float,
    corner_radius: float,
    z: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            depth,
            width,
            min(corner_radius, depth * 0.25, width * 0.25),
            corner_segments=corner_segments,
        )
    ]


def _append_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(*point) for point in points]


def _build_bin_shell_mesh() -> MeshGeometry:
    body_height = 0.92
    wall = 0.014
    floor = 0.020

    outer_bottom = _rounded_rect_loop(0.50, 0.42, 0.030, 0.000)
    outer_top = _rounded_rect_loop(0.72, 0.58, 0.050, body_height)
    inner_bottom = _rounded_rect_loop(0.46, 0.38, 0.024, floor)
    inner_top = _rounded_rect_loop(0.68, 0.54, 0.040, body_height - wall)

    geom = MeshGeometry()
    outer_bottom_ids = _append_loop(geom, outer_bottom)
    outer_top_ids = _append_loop(geom, outer_top)
    inner_bottom_ids = _append_loop(geom, inner_bottom)
    inner_top_ids = _append_loop(geom, inner_top)

    _bridge_loops(geom, outer_bottom_ids, outer_top_ids, outward=True)
    _bridge_loops(geom, inner_top_ids, inner_bottom_ids, outward=True)
    _bridge_loops(geom, outer_top_ids, inner_top_ids, outward=True)
    _bridge_loops(geom, inner_bottom_ids, outer_bottom_ids, outward=True)
    _add_cap(geom, outer_bottom_ids, upward=False)
    _add_cap(geom, inner_bottom_ids, upward=True)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    body_plastic = model.material("body_plastic", rgba=(0.12, 0.20, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.22, 0.23, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_bin_shell_mesh(), "wheelie_bin_body_shell"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.08, 0.18, 0.18)),
        origin=Origin(xyz=(-0.23, 0.16, 0.18)),
        material=body_plastic,
        name="left_axle_support",
    )
    body.visual(
        Box((0.08, 0.18, 0.18)),
        origin=Origin(xyz=(-0.23, -0.16, 0.18)),
        material=body_plastic,
        name="right_axle_support",
    )
    body.visual(
        Box((0.08, 0.14, 0.10)),
        origin=Origin(xyz=(0.25, 0.12, 0.05)),
        material=body_plastic,
        name="left_front_foot",
    )
    body.visual(
        Box((0.08, 0.14, 0.10)),
        origin=Origin(xyz=(0.25, -0.12, 0.05)),
        material=body_plastic,
        name="right_front_foot",
    )
    body.visual(
        Box((0.10, 0.48, 0.06)),
        origin=Origin(xyz=(-0.18, 0.00, 0.06)),
        material=body_plastic,
        name="rear_base_beam",
    )
    body.visual(
        Box((0.05, 0.58, 0.04)),
        origin=Origin(xyz=(0.32, 0.00, 0.88)),
        material=body_plastic,
        name="front_rim_handle",
    )
    body.visual(
        Box((0.05, 0.035, 0.13)),
        origin=Origin(xyz=(-0.392, 0.305, 0.905)),
        material=body_plastic,
        name="left_hinge_cheek",
    )
    body.visual(
        Box((0.05, 0.035, 0.13)),
        origin=Origin(xyz=(-0.392, -0.305, 0.905)),
        material=body_plastic,
        name="right_hinge_cheek",
    )
    body.visual(
        Box((0.08, 0.58, 0.08)),
        origin=Origin(xyz=(-0.33, 0.00, 0.86)),
        material=body_plastic,
        name="rear_hinge_bridge",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.50),
        origin=Origin(xyz=(-0.365, 0.00, 0.918), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="upper_hinge_bar",
    )
    body.visual(
        Box((0.10, 0.04, 0.28)),
        origin=Origin(xyz=(-0.24, 0.20, 0.24)),
        material=body_plastic,
        name="left_rear_leg",
    )
    body.visual(
        Box((0.10, 0.04, 0.28)),
        origin=Origin(xyz=(-0.24, -0.20, 0.24)),
        material=body_plastic,
        name="right_rear_leg",
    )
    body.visual(
        Box((0.06, 0.44, 0.04)),
        origin=Origin(xyz=(-0.28, 0.00, 0.12)),
        material=axle_steel,
        name="axle_tube",
    )
    body.visual(
        Box((0.06, 0.07, 0.04)),
        origin=Origin(xyz=(-0.28, 0.2525, 0.12)),
        material=axle_steel,
        name="left_axle_stub",
    )
    body.visual(
        Box((0.06, 0.07, 0.04)),
        origin=Origin(xyz=(-0.28, -0.2525, 0.12)),
        material=axle_steel,
        name="right_axle_stub",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.72, 0.58, 0.96)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.70, 0.54, 0.024)),
        origin=Origin(xyz=(0.35, 0.0, 0.012)),
        material=body_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.05, 0.18, 0.025)),
        origin=Origin(xyz=(0.665, 0.0, 0.024)),
        material=body_plastic,
        name="front_lip",
    )
    lid.visual(
        Box((0.68, 0.018, 0.016)),
        origin=Origin(xyz=(0.34, 0.261, 0.016)),
        material=body_plastic,
        name="left_side_lip",
    )
    lid.visual(
        Box((0.68, 0.018, 0.016)),
        origin=Origin(xyz=(0.34, -0.261, 0.016)),
        material=body_plastic,
        name="right_side_lip",
    )
    lid.visual(
        Box((0.020, 0.012, 0.042)),
        origin=Origin(xyz=(-0.043, 0.2815, -0.010)),
        material=body_plastic,
        name="left_hinge_ear",
    )
    lid.visual(
        Box((0.020, 0.012, 0.042)),
        origin=Origin(xyz=(-0.043, -0.2815, -0.010)),
        material=body_plastic,
        name="right_hinge_ear",
    )
    lid.visual(
        Box((0.060, 0.024, 0.016)),
        origin=Origin(xyz=(-0.026, 0.2755, 0.004)),
        material=body_plastic,
        name="left_rear_corner_brace",
    )
    lid.visual(
        Box((0.060, 0.024, 0.016)),
        origin=Origin(xyz=(-0.026, -0.2755, 0.004)),
        material=body_plastic,
        name="right_rear_corner_brace",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.70, 0.54, 0.07)),
        mass=2.0,
        origin=Origin(xyz=(0.35, 0.0, 0.005)),
    )

    for wheel_name in ("left_wheel", "right_wheel"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.11, length=0.055),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.072, length=0.040),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.038, length=0.040),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=body_plastic,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.078, length=0.006),
            origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="outer_face",
        )
        wheel.visual(
            Cylinder(radius=0.078, length=0.006),
            origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="inner_face",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.11, length=0.055),
            mass=1.4,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.35, 0.0, 0.93)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="left_wheel",
        origin=Origin(xyz=(-0.30, 0.315, 0.13)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="right_wheel",
        origin=Origin(xyz=(-0.30, -0.315, 0.13)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
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
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_joint = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="body_shell",
        min_gap=0.003,
        max_gap=0.02,
        name="closed lid sits just above the bin rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="body_shell",
        min_overlap=0.53,
        name="lid covers the bin opening footprint",
    )
    ctx.expect_gap(
        left_wheel,
        body,
        axis="y",
        negative_elem="left_axle_stub",
        max_gap=0.003,
        max_penetration=0.0,
        name="left wheel seats against the left axle stub",
    )
    ctx.expect_gap(
        body,
        right_wheel,
        axis="y",
        positive_elem="right_axle_stub",
        max_gap=0.003,
        max_penetration=0.0,
        name="right wheel seats against the right axle stub",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_joint: 1.2}):
        open_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.20,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
