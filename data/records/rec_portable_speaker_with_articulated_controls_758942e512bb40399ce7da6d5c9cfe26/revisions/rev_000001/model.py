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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speaker_with_cover")

    body_width = 0.220
    body_depth = 0.085
    body_height = 0.128
    body_corner_radius = 0.016

    control_width = 0.156
    control_depth = 0.032
    control_center_y = 0.008
    control_center_z = body_height - 0.0015

    cover_width = 0.164
    cover_depth = 0.035
    cover_thickness = 0.003
    cover_hinge_y = control_center_y - control_depth / 2.0 - 0.001
    cover_hinge_z = body_height + 0.005

    handle_axis_x = body_width / 2.0 + 0.0045
    handle_axis_z = body_height - 0.010
    handle_span = 0.044

    body_mat = model.material("body_polymer", rgba=(0.18, 0.20, 0.22, 1.0))
    grille_mat = model.material("grille_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_mat = model.material("control_rubber", rgba=(0.22, 0.24, 0.27, 1.0))
    metal_mat = model.material("hinge_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    handle_mat = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(body_width, body_depth, body_corner_radius),
            body_height,
            cap=True,
            closed=True,
        ),
        "speaker_body_shell",
    )
    body.visual(body_shell, material=body_mat, name="body_shell")
    body.visual(
        Box((0.174, 0.004, 0.074)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 - 0.002, 0.067)),
        material=grille_mat,
        name="front_grille",
    )
    body.visual(
        Box((control_width, control_depth, 0.004)),
        origin=Origin(xyz=(0.0, control_center_y, control_center_z)),
        material=accent_mat,
        name="control_strip",
    )
    for index, x_pos in enumerate((-0.034, 0.0, 0.034), start=1):
        body.visual(
            Cylinder(radius=0.0043, length=0.0016),
            origin=Origin(xyz=(x_pos, control_center_y, body_height + 0.0009)),
            material=accent_mat,
            name=f"button_{index}",
        )
    body.visual(
        Box((cover_width - 0.010, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, cover_hinge_y - 0.001, body_height + 0.001)),
        material=metal_mat,
        name="cover_hinge_support",
    )
    for y_pos in (-handle_span / 2.0, handle_span / 2.0):
        body.visual(
            Box((0.006, 0.012, 0.012)),
            origin=Origin(xyz=(body_width / 2.0 - 0.001, y_pos, handle_axis_z)),
            material=metal_mat,
            name=f"handle_pivot_{'front' if y_pos < 0 else 'rear'}",
        )

    cover = model.part("control_cover")
    cover.visual(
        Cylinder(radius=0.0026, length=cover_width - 0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="cover_hinge_barrel",
    )
    cover.visual(
        Box((cover_width, cover_depth, cover_thickness)),
        origin=Origin(xyz=(0.0, cover_depth / 2.0, -0.0015)),
        material=body_mat,
        name="cover_panel",
    )
    cover.visual(
        Box((cover_width - 0.006, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, cover_depth - 0.002, -0.002)),
        material=accent_mat,
        name="cover_front_bar",
    )

    handle = model.part("carry_handle")
    handle_loop = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -handle_span / 2.0, 0.0),
                (0.009, -handle_span / 2.0, -0.001),
                (0.016, -0.017, -0.029),
                (0.019, 0.0, -0.052),
                (0.016, 0.017, -0.029),
                (0.009, handle_span / 2.0, -0.001),
                (0.0, handle_span / 2.0, 0.0),
            ],
            radius=0.0032,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
        "speaker_side_handle_loop",
    )
    handle.visual(handle_loop, material=handle_mat, name="handle_loop")
    handle.visual(
        Cylinder(radius=0.0048, length=0.032),
        origin=Origin(xyz=(0.019, 0.0, -0.052), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_mat,
        name="handle_grip",
    )
    for y_pos in (-handle_span / 2.0, handle_span / 2.0):
        handle.visual(
            Cylinder(radius=0.0025, length=0.008),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"handle_stub_{'front' if y_pos < 0 else 'rear'}",
        )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, cover_hinge_y, cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(handle_axis_x, 0.0, handle_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=2.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("control_cover")
    handle = object_model.get_part("carry_handle")
    cover_hinge = object_model.get_articulation("body_to_cover")
    handle_hinge = object_model.get_articulation("body_to_handle")

    body_shell = body.get_visual("body_shell")
    control_strip = body.get_visual("control_strip")
    cover_panel = cover.get_visual("cover_panel")
    cover_front_bar = cover.get_visual("cover_front_bar")
    handle_loop = handle.get_visual("handle_loop")
    handle_grip = handle.get_visual("handle_grip")

    ctx.check(
        "speaker parts and joints exist",
        all(
            item is not None
            for item in (
                body,
                cover,
                handle,
                cover_hinge,
                handle_hinge,
                body_shell,
                control_strip,
                cover_panel,
                handle_loop,
                handle_grip,
            )
        ),
        details="Expected body, control cover, carry handle, and both articulations.",
    )
    ctx.check(
        "cover hinge runs along the speaker width",
        tuple(round(v, 3) for v in cover_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={cover_hinge.axis}",
    )
    ctx.check(
        "handle hinge runs fore-aft at the side corner",
        tuple(round(v, 3) for v in handle_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={handle_hinge.axis}",
    )

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a=cover_panel,
        elem_b=control_strip,
        min_overlap=0.030,
        name="closed cover spans the top control strip",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem=cover_panel,
        negative_elem=control_strip,
        min_gap=0.001,
        max_gap=0.004,
        name="closed cover sits just above the controls",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="x",
        positive_elem=handle_loop,
        negative_elem=body_shell,
        min_gap=0.001,
        max_gap=0.008,
        name="stowed handle rests close to the speaker side",
    )

    with ctx.pose({cover_hinge: 1.15}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem=cover_front_bar,
            negative_elem=body_shell,
            min_gap=0.020,
            name="cover swings upward above the body when opened",
        )

    with ctx.pose({handle_hinge: 2.55}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            positive_elem=handle_grip,
            negative_elem=body_shell,
            min_gap=0.020,
            name="carry handle lifts above the body when rotated out",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
