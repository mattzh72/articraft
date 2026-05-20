from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / segments),
            radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_porthole_hatch")

    frame_paint = model.material("frame_paint", rgba=(0.62, 0.66, 0.70, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.55, 0.59, 0.63, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    polished_metal = model.material("polished_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame_outer_radius = 0.33
    frame_inner_radius = 0.21
    frame_depth = 0.08
    flange_outer_radius = 0.36
    flange_inner_radius = 0.225
    flange_depth = 0.028
    lid_radius = 0.288
    lid_depth = 0.045
    hinge_axis_x = 0.092
    hinge_axis_y = -0.305
    latch_axis_x = 0.148
    latch_axis_y = 0.305

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.75, 0.78, 0.20)),
        mass=28.0,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    frame_ring = ExtrudeWithHolesGeometry(
        _circle_profile(frame_outer_radius, segments=72),
        [_circle_profile(frame_inner_radius, segments=72)],
        height=frame_depth,
        center=True,
    ).rotate_y(pi / 2.0)
    frame.visual(_mesh("frame_ring", frame_ring), material=frame_paint, name="frame_ring")

    frame_flange = ExtrudeWithHolesGeometry(
        _circle_profile(flange_outer_radius, segments=72),
        [_circle_profile(flange_inner_radius, segments=72)],
        height=flange_depth,
        center=True,
    ).rotate_y(pi / 2.0)
    frame.visual(
        _mesh("frame_front_flange", frame_flange),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=frame_paint,
        name="frame_flange",
    )

    frame.visual(
        Cylinder(radius=frame_inner_radius + 0.020, length=0.020),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=gasket_black,
        name="frame_gasket",
    )

    frame.visual(
        Box((0.072, 0.044, 0.300)),
        origin=Origin(xyz=(0.042, -0.336, 0.0)),
        material=dark_metal,
        name="hinge_bracket",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.110)),
        material=dark_metal,
        name="hinge_barrel_upper",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.110)),
        material=dark_metal,
        name="hinge_barrel_lower",
    )

    frame.visual(
        Box((0.098, 0.050, 0.180)),
        origin=Origin(xyz=(0.085, 0.320, 0.0)),
        material=dark_metal,
        name="latch_bracket",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.042),
        origin=Origin(xyz=(latch_axis_x, latch_axis_y, 0.055)),
        material=dark_metal,
        name="latch_ear_upper",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.042),
        origin=Origin(xyz=(latch_axis_x, latch_axis_y, -0.055)),
        material=dark_metal,
        name="latch_ear_lower",
    )
    frame.visual(
        Box((0.024, 0.040, 0.070)),
        origin=Origin(xyz=(0.114, 0.333, 0.0)),
        material=dark_metal,
        name="latch_stop",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.12, 0.64, 0.64)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.305, 0.0)),
    )

    lid_disc = ExtrudeGeometry(
        _circle_profile(lid_radius, segments=72),
        lid_depth,
        center=True,
    ).rotate_y(pi / 2.0)
    lid.visual(
        _mesh("lid_disc", lid_disc),
        origin=Origin(xyz=(0.0, 0.305, 0.0)),
        material=lid_paint,
        name="lid_disc",
    )

    lid_stiffener = ExtrudeWithHolesGeometry(
        _circle_profile(0.268, segments=72),
        [_circle_profile(0.220, segments=72)],
        height=0.020,
        center=True,
    ).rotate_y(pi / 2.0)
    lid.visual(
        _mesh("lid_stiffener_ring", lid_stiffener),
        origin=Origin(xyz=(0.014, 0.305, 0.0)),
        material=lid_paint,
        name="lid_stiffener",
    )

    lid_seal = ExtrudeWithHolesGeometry(
        _circle_profile(0.235, segments=72),
        [_circle_profile(0.205, segments=72)],
        height=0.016,
        center=True,
    ).rotate_y(pi / 2.0)
    lid.visual(
        _mesh("lid_rear_seal_ring", lid_seal),
        origin=Origin(xyz=(-0.016, 0.305, 0.0)),
        material=gasket_black,
        name="lid_seal",
    )

    lid.visual(
        Box((0.050, 0.034, 0.100)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=dark_metal,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.030, 0.040, 0.110)),
        origin=Origin(xyz=(0.024, 0.555, 0.0)),
        material=dark_metal,
        name="lid_strike",
    )

    clamp_handle = model.part("clamp_handle")
    clamp_handle.inertial = Inertial.from_geometry(
        Box((0.08, 0.12, 0.10)),
        mass=2.0,
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
    )
    clamp_handle.visual(
        Cylinder(radius=0.0135, length=0.068),
        origin=Origin(),
        material=polished_metal,
        name="handle_hub",
    )
    clamp_handle.visual(
        Box((0.016, 0.070, 0.026)),
        origin=Origin(xyz=(0.004, -0.035, 0.0)),
        material=polished_metal,
        name="handle_arm",
    )
    clamp_handle.visual(
        Box((0.018, 0.026, 0.050)),
        origin=Origin(xyz=(0.006, -0.074, 0.0)),
        material=dark_metal,
        name="handle_dog",
    )
    clamp_handle.visual(
        Box((0.032, 0.050, 0.052)),
        origin=Origin(xyz=(0.018, -0.030, 0.0)),
        material=polished_metal,
        name="handle_grip",
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "clamp_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=clamp_handle,
        origin=Origin(xyz=(latch_axis_x, latch_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.35),
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
    frame = object_model.get_part("frame")
    lid = object_model.get_part("lid")
    clamp_handle = object_model.get_part("clamp_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    clamp_pivot = object_model.get_articulation("clamp_pivot")

    ctx.check("frame exists", frame is not None)
    ctx.check("lid exists", lid is not None)
    ctx.check("clamp handle exists", clamp_handle is not None)

    with ctx.pose({lid_hinge: 0.0, clamp_pivot: 0.0}):
        ctx.expect_gap(
            lid,
            frame,
            axis="x",
            min_gap=0.001,
            max_gap=0.006,
            positive_elem="lid_seal",
            negative_elem="frame_gasket",
            name="closed lid stands just proud of frame",
        )
        ctx.expect_overlap(
            lid,
            frame,
            axes="yz",
            min_overlap=0.52,
            name="closed lid covers the frame opening footprint",
        )
        ctx.expect_gap(
            clamp_handle,
            lid,
            axis="x",
            min_gap=0.008,
            max_gap=0.025,
            positive_elem="handle_dog",
            negative_elem="lid_strike",
            name="dog latch sits just ahead of strike block",
        )

        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")
        closed_grip_aabb = ctx.part_element_world_aabb(clamp_handle, elem="handle_dog")

    with ctx.pose({lid_hinge: 1.05}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_disc")

    with ctx.pose({clamp_pivot: 1.10}):
        opened_grip_aabb = ctx.part_element_world_aabb(clamp_handle, elem="handle_dog")

    closed_lid_center_x = None if closed_lid_aabb is None else 0.5 * (closed_lid_aabb[0][0] + closed_lid_aabb[1][0])
    opened_lid_center_x = None if opened_lid_aabb is None else 0.5 * (opened_lid_aabb[0][0] + opened_lid_aabb[1][0])
    closed_grip_center_x = None if closed_grip_aabb is None else 0.5 * (closed_grip_aabb[0][0] + closed_grip_aabb[1][0])
    opened_grip_center_x = None if opened_grip_aabb is None else 0.5 * (opened_grip_aabb[0][0] + opened_grip_aabb[1][0])

    ctx.check(
        "lid opens outward on side hinge",
        closed_lid_center_x is not None
        and opened_lid_center_x is not None
        and opened_lid_center_x > closed_lid_center_x + 0.10,
        details=f"closed_x={closed_lid_center_x}, opened_x={opened_lid_center_x}",
    )
    ctx.check(
        "clamp handle lifts away from lid",
        closed_grip_center_x is not None
        and opened_grip_center_x is not None
        and opened_grip_center_x > closed_grip_center_x + 0.025,
        details=f"closed_x={closed_grip_center_x}, opened_x={opened_grip_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
