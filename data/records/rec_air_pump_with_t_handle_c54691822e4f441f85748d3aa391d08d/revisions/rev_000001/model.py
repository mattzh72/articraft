from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _ring_shape(outer_radius: float, inner_radius: float, length: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length)
    return outer.cut(inner)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_floor_pump")

    painted_steel = model.material("painted_steel", rgba=(0.78, 0.12, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    gauge_face = model.material("gauge_face", rgba=(0.95, 0.96, 0.97, 1.0))
    orange = model.material("orange", rgba=(0.93, 0.34, 0.10, 1.0))
    button_color = model.material("button_color", rgba=(0.85, 0.85, 0.87, 1.0))

    barrel_shell = mesh_from_cadquery(_ring_shape(0.034, 0.0275, 0.45), "barrel_shell")
    gauge_bezel = mesh_from_cadquery(_ring_shape(0.055, 0.047, 0.012), "gauge_bezel")
    button_sleeve = mesh_from_cadquery(_ring_shape(0.0072, 0.0043, 0.016), "button_sleeve")
    foot_base_mesh = mesh_from_cadquery(
        cq.Workplane("XY").box(0.290, 0.075, 0.018).edges("|Z").fillet(0.026),
        "foot_base",
    )
    foot_pad_mesh = mesh_from_cadquery(
        cq.Workplane("XY").box(0.238, 0.052, 0.008).edges("|Z").fillet(0.020),
        "foot_pad",
    )
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.052, 0.022, 0.125),
                (0.098, 0.014, 0.185),
                (0.092, 0.016, 0.310),
                (0.074, 0.018, 0.392),
                (0.094, 0.014, 0.290),
                (0.112, 0.008, 0.155),
                (0.123, 0.000, 0.077),
            ],
            radius=0.0055,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "hose",
    )

    body = model.part("body")
    body.visual(
        foot_base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_metal,
        name="foot_base",
    )
    body.visual(
        foot_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=rubber,
        name="foot_pad",
    )
    body.visual(
        Box((0.085, 0.060, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=painted_steel,
        name="pedestal",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=painted_steel,
        name="barrel_socket",
    )
    body.visual(
        barrel_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=painted_steel,
        name="barrel_shell",
    )
    body.visual(
        Box((0.032, 0.028, 0.110)),
        origin=Origin(xyz=(0.0, 0.020, 0.087)),
        material=painted_steel,
        name="gauge_column",
    )
    body.visual(
        Box((0.064, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, 0.035, 0.154)),
        material=painted_steel,
        name="gauge_mount",
    )
    body.visual(
        Cylinder(radius=0.049, length=0.028),
        origin=Origin(xyz=(0.0, 0.038, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="gauge_body",
    )
    body.visual(
        gauge_bezel,
        origin=Origin(xyz=(0.0, 0.064, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.046, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0526, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gauge_face,
        name="gauge_face",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.0, 0.055, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_post",
    )
    body.visual(
        button_sleeve,
        origin=Origin(xyz=(0.048, 0.038, 0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="button_sleeve",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.042, 0.022, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hose_fitting",
    )
    body.visual(
        hose_mesh,
        material=rubber,
        name="hose",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.055),
        origin=Origin(xyz=(0.126, 0.0, 0.077), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="nozzle_body",
    )
    body.visual(
        Box((0.016, 0.018, 0.080)),
        origin=Origin(xyz=(0.041, 0.016, 0.390)),
        material=dark_metal,
        name="side_clip_back",
    )
    body.visual(
        Box((0.026, 0.012, 0.010)),
        origin=Origin(xyz=(0.057, 0.016, 0.426)),
        material=dark_metal,
        name="side_clip_upper",
    )
    body.visual(
        Box((0.026, 0.012, 0.010)),
        origin=Origin(xyz=(0.057, 0.016, 0.354)),
        material=dark_metal,
        name="side_clip_lower",
    )

    rod_handle = model.part("rod_handle")
    rod_handle.visual(
        Cylinder(radius=0.007, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        material=satin_metal,
        name="rod",
    )
    rod_handle.visual(
        Box((0.032, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=dark_metal,
        name="handle_hub",
    )
    rod_handle.visual(
        Cylinder(radius=0.011, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="crossbar",
    )
    for index, x_pos in enumerate((-0.112, 0.112)):
        rod_handle.visual(
            Cylinder(radius=0.015, length=0.060),
            origin=Origin(xyz=(x_pos, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"grip_{index}",
        )

    needle = model.part("needle")
    needle.visual(
        Box((0.040, 0.0012, 0.0040)),
        origin=Origin(xyz=(0.020, 0.0015, 0.0)),
        material=orange,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.0042, length=0.0040),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.0036, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_color,
        name="button_stem",
    )
    release_button.visual(
        Cylinder(radius=0.0064, length=0.001),
        origin=Origin(xyz=(0.0005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_color,
        name="button_shoulder",
    )
    release_button.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_color,
        name="button_cap",
    )

    model.articulation(
        "body_to_rod_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rod_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.570)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "body_to_needle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=needle,
        origin=Origin(xyz=(0.0, 0.057, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
            lower=-1.25,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.064, 0.038, 0.198)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rod_handle = object_model.get_part("rod_handle")
    needle = object_model.get_part("needle")
    release_button = object_model.get_part("release_button")

    rod_joint = object_model.get_articulation("body_to_rod_handle")
    needle_joint = object_model.get_articulation("body_to_needle")
    button_joint = object_model.get_articulation("body_to_release_button")

    rod_limits = rod_joint.motion_limits
    needle_limits = needle_joint.motion_limits
    button_limits = button_joint.motion_limits

    ctx.expect_within(
        rod_handle,
        body,
        axes="xy",
        inner_elem="rod",
        outer_elem="barrel_shell",
        margin=0.007,
        name="rod stays centered in the barrel",
    )
    ctx.expect_overlap(
        rod_handle,
        body,
        axes="z",
        elem_a="rod",
        elem_b="barrel_shell",
        min_overlap=0.35,
        name="collapsed pump rod remains deeply inserted",
    )

    rod_aabb = ctx.part_element_world_aabb(rod_handle, elem="rod")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="barrel_shell")
    rod_visible = (
        rod_aabb is not None
        and barrel_aabb is not None
        and rod_aabb[1][2] > barrel_aabb[1][2] + 0.08
    )
    ctx.check(
        "rod remains visibly above the barrel",
        rod_visible,
        details=f"rod={rod_aabb}, barrel={barrel_aabb}",
    )
    ctx.expect_gap(
        rod_handle,
        body,
        axis="z",
        positive_elem="crossbar",
        negative_elem="barrel_shell",
        min_gap=0.045,
        name="handle crossbar sits above the barrel top",
    )

    if rod_limits is not None and rod_limits.upper is not None:
        rest_pos = ctx.part_world_position(rod_handle)
        with ctx.pose({rod_joint: rod_limits.upper}):
            ctx.expect_within(
                rod_handle,
                body,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.007,
                name="extended rod stays centered in the barrel",
            )
            ctx.expect_overlap(
                rod_handle,
                body,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.20,
                name="extended rod retains insertion in the barrel",
            )
            extended_pos = ctx.part_world_position(rod_handle)
        ctx.check(
            "handle slides upward when the pump extends",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.18,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    ctx.expect_within(
        release_button,
        body,
        axes="yz",
        inner_elem="button_stem",
        outer_elem="button_sleeve",
        margin=0.0015,
        name="release button stem stays guided by the sleeve",
    )
    ctx.expect_overlap(
        release_button,
        body,
        axes="x",
        elem_a="button_stem",
        elem_b="button_sleeve",
        min_overlap=0.010,
        name="release button stays retained in the sleeve",
    )

    if button_limits is not None and button_limits.upper is not None:
        rest_button_aabb = ctx.part_element_world_aabb(release_button, elem="button_cap")
        with ctx.pose({button_joint: button_limits.upper}):
            ctx.expect_within(
                release_button,
                body,
                axes="yz",
                inner_elem="button_stem",
                outer_elem="button_sleeve",
                margin=0.0015,
                name="pressed button stem stays guided by the sleeve",
            )
            ctx.expect_overlap(
                release_button,
                body,
                axes="x",
                elem_a="button_stem",
                elem_b="button_sleeve",
                min_overlap=0.010,
                name="pressed button remains captured in the sleeve",
            )
            pressed_button_aabb = ctx.part_element_world_aabb(release_button, elem="button_cap")
        rest_button_center = _aabb_center(rest_button_aabb)
        pressed_button_center = _aabb_center(pressed_button_aabb)
        ctx.check(
            "release button presses inward",
            rest_button_center is not None
            and pressed_button_center is not None
            and pressed_button_center[0] < rest_button_center[0] - 0.003,
            details=f"rest={rest_button_center}, pressed={pressed_button_center}",
        )

    if needle_limits is not None and needle_limits.lower is not None and needle_limits.upper is not None:
        with ctx.pose({needle_joint: needle_limits.lower}):
            low_origin = ctx.part_world_position(needle)
            low_pointer = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))
        with ctx.pose({needle_joint: needle_limits.upper}):
            high_origin = ctx.part_world_position(needle)
            high_pointer = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))
        ctx.check(
            "needle pivots around a fixed center",
            low_origin is not None
            and high_origin is not None
            and max(abs(low_origin[i] - high_origin[i]) for i in range(3)) < 1e-6,
            details=f"low_origin={low_origin}, high_origin={high_origin}",
        )
        ctx.check(
            "needle sweeps across the gauge face",
            low_pointer is not None
            and high_pointer is not None
            and (
                abs(high_pointer[0] - low_pointer[0]) > 0.020
                or abs(high_pointer[2] - low_pointer[2]) > 0.020
            ),
            details=f"low_pointer={low_pointer}, high_pointer={high_pointer}",
        )

    return ctx.report()


object_model = build_object_model()
