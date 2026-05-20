from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


FRAME_DEPTH = 0.007
LENS_CENTER_X = 0.0335
LENS_CENTER_Z = 0.002
LENS_WIDTH = 0.051
LENS_HEIGHT = 0.037
RIM_OUTER_WIDTH = 0.062
RIM_OUTER_HEIGHT = 0.047
RIM_INNER_WIDTH = 0.0525
RIM_INNER_HEIGHT = 0.0385
HINGE_X = 0.066
HINGE_Y = 0.0045
HINGE_Z = 0.004


def _shift_profile(profile, cx: float, cz: float):
    return [(x + cx, z + cz) for x, z in profile]


def _rim_geometry(cx: float) -> MeshGeometry:
    outer = _shift_profile(
        superellipse_profile(
            RIM_OUTER_WIDTH,
            RIM_OUTER_HEIGHT,
            exponent=2.65,
            segments=96,
        ),
        cx,
        LENS_CENTER_Z,
    )
    inner = _shift_profile(
        superellipse_profile(
            RIM_INNER_WIDTH,
            RIM_INNER_HEIGHT,
            exponent=2.35,
            segments=96,
        ),
        cx,
        LENS_CENTER_Z,
    )
    # Extrude in local Z, then rotate so the lens profile lies in world XZ and
    # the frame thickness runs front-to-back along world Y.
    return ExtrudeWithHolesGeometry(
        outer,
        [inner],
        FRAME_DEPTH,
        cap=True,
        center=True,
    ).rotate_x(math.pi / 2)


def _superellipse_point(width: float, height: float, exponent: float, theta: float):
    c = math.cos(theta)
    s = math.sin(theta)
    x = 0.5 * width * math.copysign(abs(c) ** (2.0 / exponent), c)
    z = 0.5 * height * math.copysign(abs(s) ** (2.0 / exponent), s)
    return x, z


def _lens_geometry(
    width: float = LENS_WIDTH,
    height: float = LENS_HEIGHT,
    *,
    thickness: float = 0.0022,
    bulge: float = 0.00065,
    radial_rings: int = 7,
    segments: int = 96,
) -> MeshGeometry:
    """Thin biconvex lens with a superellipse outline in local XZ."""

    geom = MeshGeometry()
    half = thickness / 2.0
    side_boundary_loops = []

    for side in (-1.0, 1.0):
        center_index = geom.add_vertex(0.0, side * (half + bulge), 0.0)
        previous_loop = None
        for ring in range(1, radial_rings + 1):
            r = ring / radial_rings
            loop = []
            y = side * (half + bulge * (1.0 - r * r))
            for i in range(segments):
                theta = 2.0 * math.pi * i / segments
                bx, bz = _superellipse_point(width, height, 2.35, theta)
                loop.append(geom.add_vertex(r * bx, y, r * bz))

            if previous_loop is None:
                for i in range(segments):
                    j = (i + 1) % segments
                    if side < 0:
                        geom.add_face(center_index, loop[j], loop[i])
                    else:
                        geom.add_face(center_index, loop[i], loop[j])
            else:
                for i in range(segments):
                    j = (i + 1) % segments
                    if side < 0:
                        geom.add_face(previous_loop[i], previous_loop[j], loop[j])
                        geom.add_face(previous_loop[i], loop[j], loop[i])
                    else:
                        geom.add_face(previous_loop[i], loop[j], previous_loop[j])
                        geom.add_face(previous_loop[i], loop[i], loop[j])
            previous_loop = loop
        side_boundary_loops.append(previous_loop)

    front, back = side_boundary_loops
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])
    return geom


def _ellipsoid_geometry(rx: float, ry: float, rz: float, *, tilt_y: float = 0.0):
    geom = SphereGeometry(1.0, width_segments=32, height_segments=16).scale(rx, ry, rz)
    if tilt_y:
        geom.rotate_y(tilt_y)
    return geom


def _temple_arm_geometry(side: int) -> MeshGeometry:
    inward = -float(side)
    path = [
        (0.000, 0.004, 0.000),
        (inward * 0.001, 0.032, -0.001),
        (inward * 0.002, 0.082, -0.004),
        (inward * 0.006, 0.124, -0.016),
        (inward * 0.010, 0.147, -0.032),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.0058, 0.0032, radius=0.0011, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _ear_tip_geometry(side: int) -> MeshGeometry:
    inward = -float(side)
    path = [
        (inward * 0.002, 0.082, -0.004),
        (inward * 0.006, 0.124, -0.016),
        (inward * 0.010, 0.147, -0.032),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.0074, 0.0048, radius=0.0020, corner_segments=10),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_glasses")

    acetate = model.material("polished_dark_havana_acetate", rgba=(0.19, 0.095, 0.045, 1.0))
    amber = model.material("warm_amber_edge_glow", rgba=(0.62, 0.34, 0.13, 1.0))
    lens_mat = model.material("subtle_blue_clear_lens", rgba=(0.70, 0.88, 1.0, 0.34))
    metal = model.material("brushed_silver_hinge_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    silicone = model.material("soft_clear_silicone", rgba=(0.88, 0.96, 1.0, 0.55))
    rubber = model.material("matte_black_ear_sleeve", rgba=(0.01, 0.01, 0.012, 1.0))

    front = model.part("front_frame")
    front.visual(
        mesh_from_geometry(_rim_geometry(-LENS_CENTER_X), "left_acetate_rim"),
        material=acetate,
        name="left_rim",
    )
    front.visual(
        mesh_from_geometry(_rim_geometry(LENS_CENTER_X), "right_acetate_rim"),
        material=acetate,
        name="right_rim",
    )

    bridge = tube_from_spline_points(
        [
            (-0.0105, 0.000, 0.006),
            (-0.0040, -0.0004, 0.013),
            (0.0040, -0.0004, 0.013),
            (0.0105, 0.000, 0.006),
        ],
        radius=0.0035,
        samples_per_segment=18,
        radial_segments=24,
        cap_ends=True,
    )
    front.visual(
        mesh_from_geometry(bridge, "arched_keyhole_bridge"),
        material=acetate,
        name="bridge",
    )

    lower_bridge = tube_from_spline_points(
        [
            (-0.0075, 0.0010, -0.004),
            (0.0000, 0.0020, -0.009),
            (0.0075, 0.0010, -0.004),
        ],
        radius=0.0017,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    front.visual(
        mesh_from_geometry(lower_bridge, "lower_nose_bridge"),
        material=metal,
        name="lower_bridge",
    )

    for side, label in ((-1, "left"), (1, "right")):
        # Fine metal pad wire emerging from the bridge and carrying a soft nose pad.
        pad_wire = tube_from_spline_points(
            [
                (side * 0.0045, 0.0020, -0.006),
                (side * 0.0075, 0.0050, -0.009),
                (side * 0.0105, 0.0062, -0.013),
            ],
            radius=0.00065,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        )
        front.visual(
            mesh_from_geometry(pad_wire, f"{label}_nose_pad_wire"),
            material=metal,
            name=f"{label}_pad_wire",
        )
        front.visual(
            mesh_from_geometry(
                _ellipsoid_geometry(0.0037, 0.0017, 0.0068, tilt_y=-side * 0.28),
                f"{label}_silicone_nose_pad",
            ),
            origin=Origin(xyz=(side * 0.0112, 0.0064, -0.0138)),
            material=silicone,
            name=f"{label}_nose_pad",
        )

        # Alternating hinge barrels mounted just behind the outer rim edge.
        for z_off, suffix in ((0.010, "upper"), (-0.010, "lower")):
            front.visual(
                Cylinder(radius=0.00245, length=0.010),
                origin=Origin(xyz=(side * HINGE_X, HINGE_Y, HINGE_Z + z_off)),
                material=metal,
                name=f"{label}_hinge_{suffix}",
            )
        front.visual(
            Cylinder(radius=0.00155, length=0.0010),
            origin=Origin(xyz=(side * HINGE_X, HINGE_Y, HINGE_Z + 0.0155)),
            material=amber,
            name=f"{label}_hinge_screw_head",
        )

    for side, label in ((-1, "left"), (1, "right")):
        lens = model.part(f"{label}_lens")
        lens.visual(
            mesh_from_geometry(_lens_geometry(), f"{label}_thin_lens"),
            material=lens_mat,
            name="lens",
        )
        model.articulation(
            f"front_to_{label}_lens",
            ArticulationType.FIXED,
            parent=front,
            child=lens,
            origin=Origin(xyz=(side * LENS_CENTER_X, 0.0, LENS_CENTER_Z)),
        )

        temple = model.part(f"{label}_temple")
        temple.visual(
            mesh_from_geometry(_temple_arm_geometry(side), f"{label}_temple_core"),
            material=acetate,
            name="temple_bar",
        )
        temple.visual(
            mesh_from_geometry(_ear_tip_geometry(side), f"{label}_rubber_ear_tip"),
            material=rubber,
            name="ear_tip",
        )
        temple.visual(
            Box((0.0048, 0.0120, 0.0100)),
            origin=Origin(xyz=(0.0, 0.0060, 0.0)),
            material=metal,
            name="hinge_leaf",
        )
        temple.visual(
            Cylinder(radius=0.00215, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal,
            name="hinge_knuckle",
        )

        model.articulation(
            f"{label}_temple_hinge",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(side * HINGE_X, HINGE_Y, HINGE_Z)),
            axis=(0.0, 0.0, float(side)),
            motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    left_lens = object_model.get_part("left_lens")
    right_lens = object_model.get_part("right_lens")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_hinge = object_model.get_articulation("left_temple_hinge")
    right_hinge = object_model.get_articulation("right_temple_hinge")

    ctx.expect_within(
        left_lens,
        front,
        axes="xz",
        margin=0.002,
        name="left lens is retained within the left rim outline",
    )
    ctx.expect_within(
        right_lens,
        front,
        axes="xz",
        margin=0.002,
        name="right lens is retained within the right rim outline",
    )

    ctx.check(
        "temple hinges have wearable fold limits",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and right_hinge.motion_limits.lower == 0.0
        and left_hinge.motion_limits.upper is not None
        and right_hinge.motion_limits.upper is not None
        and 1.55 <= left_hinge.motion_limits.upper <= 1.9
        and 1.55 <= right_hinge.motion_limits.upper <= 1.9,
        details=f"left={left_hinge.motion_limits}, right={right_hinge.motion_limits}",
    )

    def _center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) * 0.5 if aabb is not None else None

    def _max_y(aabb):
        return aabb[1][1] if aabb is not None else None

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        left_tip_open = ctx.part_element_world_aabb(left_temple, elem="ear_tip")
        right_tip_open = ctx.part_element_world_aabb(right_temple, elem="ear_tip")
        ctx.check(
            "open temple arms extend rearward to wearable length",
            left_tip_open is not None
            and right_tip_open is not None
            and _max_y(left_tip_open) is not None
            and _max_y(right_tip_open) is not None
            and _max_y(left_tip_open) > 0.135
            and _max_y(right_tip_open) > 0.135,
            details=f"left_tip={left_tip_open}, right_tip={right_tip_open}",
        )

    with ctx.pose({left_hinge: 1.75, right_hinge: 1.75}):
        left_tip_folded = ctx.part_element_world_aabb(left_temple, elem="ear_tip")
        right_tip_folded = ctx.part_element_world_aabb(right_temple, elem="ear_tip")
        left_x = _center_x(left_tip_folded)
        right_x = _center_x(right_tip_folded)
        ctx.check(
            "folded temples swing inward across the frame",
            left_x is not None and right_x is not None and left_x > 0.010 and right_x < -0.010,
            details=f"left_tip={left_tip_folded}, right_tip={right_tip_folded}",
        )

    return ctx.report()


object_model = build_object_model()
